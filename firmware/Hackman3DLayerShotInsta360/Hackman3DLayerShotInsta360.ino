#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <NimBLEDevice.h>
#include "dashboard.h"

static const char *FIRMWARE_VERSION = "2.3.0-insta360-dual";
static const char *HOSTNAME = "hackman-layershot";
static const char *DEVICE_NAME = "Hackman3D LayerShot";
static const char *BLE_NAME = "Insta360 GPS Remote";
static const char *SETUP_AP = "Hackman3D-LayerShot-Setup";
static const uint8_t PAIR_BUTTON_PIN = 9;
// MakerGo C3 Super Mini: single blue LED on GPIO8, active-low.
// We map the original RGB states to on/off/blink:
//   Red    (not connected)  → slow blink
//   Blue   (pairing)        → fast blink
//   Green  (connected)      → steady on
//   Purple (shutter flash)  → brief off-on pulse
static const uint8_t LED_PIN = 8;
static const uint32_t LED_PAIR_BLINK_MS   = 450;
static const uint32_t LED_IDLE_BLINK_MS   = 1200;
static const uint32_t LED_SHUTTER_FLASH_MS = 350;
bool     ledOn = false;
uint32_t ledLastToggleAt = 0;

// --- PWM trigger input (CyberBrick Servo signal) ---
static const uint8_t  PWM_TRIGGER_PIN         = 4;
static const uint32_t PWM_TRIGGER_THRESHOLD_US = 1500;
static const uint32_t PWM_TRIGGER_MIN_US      = 500;
static const uint32_t PWM_TRIGGER_MAX_US      = 2500;
static const uint32_t PWM_TRIGGER_COOLDOWN_MS = 5000;
static const uint32_t PWM_STARTUP_GRACE_MS    = 3000;  // ignore PWM for 3 s after boot
NimBLEServer *instaServer = nullptr;
NimBLECharacteristic *instaNotify = nullptr;

// --- BE80 Direct Control client (Architecture B for Ace Pro) ---
// Ace Pro does not support GPS Remote (CE80). It exposes a BE80 GATT server
// service that the ESP32 must connect to as a BLE central/client. The shutter
// command (TAKE_PICTURE, code 0x03) is a 16-byte Header16 frame written to
// BE81. No sync handshake or authorization is required for Header16 cameras.
static const char *BE80_SERVICE_UUID   = "be80";
static const char *BE81_WRITE_UUID     = "be81";
static const char *BE82_NOTIFY_UUID    = "be82";
static const char *ACE_PRO_NAME_PREFIX = "Ace Pro";
static const uint32_t BE80_SCAN_DURATION_MS = 5000;   // scan burst length
static const uint32_t BE80_RETRY_BASE_MS     = 1000;  // initial backoff
static const uint32_t BE80_RETRY_MAX_MS      = 10000; // capped backoff

// Transport mode: which BLE protocol path is active.
enum CameraMode {
  CAM_NONE,            // not connected to any camera
  CAM_CE80_SERVER,     // X-series connected to our CE80 GATT server
  CAM_BE80_CONNECTING, // BE80 client connection in progress
  CAM_BE80_SETTING_UP, // connected, discovering services / subscribing
  CAM_BE80_CLIENT      // BE80 client ready, BE81 writable
};
CameraMode cameraMode = CAM_NONE;

NimBLEClient *cameraClient = nullptr;
NimBLERemoteCharacteristic *be81Write = nullptr;
NimBLERemoteCharacteristic *be82Notify = nullptr;
uint8_t  be81SeqCounter = 0;   // Header16 sequence: 1-254, wraps to 1
uint32_t be80RetryAt = 0;      // millis() when next scan attempt is allowed
uint32_t be80RetryDelay = BE80_RETRY_BASE_MS;
bool be80ScanActive = false;
// Pending BE80 connection from scan result — processed in loop() so we can
// disconnect CE80 server first (NimBLE rejects two connections to same peer).
NimBLEAddress be80PendingAddress;
bool be80ConnectPending = false;
// True once an Ace camera has been seen. While set, CE80 advertising stays
// off after BE80 disconnects: the Ace would otherwise endlessly retry the
// GPS-remote link (which it cannot use) instead of advertising for our
// BE80 client connection. Cleared by a power cycle.
bool be80PeerKnown = false;
// False until the first scan burst has finished. CE80 advertising is held
// off until then so an Ace (which remembers us as its remote) cannot win
// the boot race and occupy the CE80 server with a protocol it ignores.
bool be80InitialScanDone = false;
// Current CE80 server peer (for identifying/rejecting the Ace on CE80).
NimBLEAddress ce80PeerAddress;
bool ce80HasPeer = false;
uint16_t ce80ConnHandle = 0xFFFF;

WebServer web(80);
Preferences preferences;
bool bleConnected = false;   // CE80 server has a client connected
bool pairingMode = true;
bool wifiConnecting = false;
bool wifiError = false;
bool otaReady = false;
uint32_t triggerCount = 0;
uint32_t commandCount = 0;
String lastCommand = "startup";
uint32_t pairingStartedAt = 0;
uint32_t buttonPressedAt = 0;
uint32_t shutterFlashUntil = 0;

// --- Insta360 protocol: sequence counter + GPS liveness heartbeat ---
// byte[4] of every ce82 button frame is a running sequence counter that
// increments by +2 per frame (decoded from a live remote capture). Sending
// the same 0x00 every time causes the camera to treat repeats as duplicates.
uint8_t  ce82SeqCounter = 0;
// The real GPS remote streams a GPS RMC sentence on ce82 at 10 Hz as a
// liveness heartbeat. If the peripheral goes silent, the camera drops the
// link and ignores button frames.
uint32_t lastGpsHeartbeatAt = 0;
static const uint32_t GPS_HEARTBEAT_INTERVAL_MS = 100;  // 10 Hz

String serialLine;
String cameraType = "insta360";
String deviceHostname = HOSTNAME;
String wifiSsid;
String wifiPassword;
String preferredIp;
String preferredGateway;
String preferredNetmask;
String preferredDns;
uint32_t lastWiFiReconnectAttempt = 0;
String printerHost;
uint16_t printerPort = 4408;
uint16_t captureEvery = 1;
uint16_t skipLayers = 0;
uint16_t stopAfterLayer = 0;
uint16_t stabilizationMs = 3000;
bool shutterPending = false;
uint32_t shutterDueAt = 0;
bool autonomousEnabled = false;
int lastPrinterLayer = -1;
int printerTotalLayers = -1;
bool printerConnected = false;
String printerState = "unknown";
int printerHttpCode = 0;
uint32_t lastPrinterPoll = 0;

// --- PWM trigger state ---
// ISR-shared variables must be volatile; pulse width is read atomically (32-bit
// read is a single instruction on ESP32-C3, so no critical section needed).
volatile uint32_t pwmRiseMicros = 0;      // micros() at rising edge
volatile uint32_t pwmLastWidthUs = 0;     // last measured pulse width (0 = none)
volatile bool      pwmNewPulse   = false;  // flag: a new pulse was captured
uint32_t pwmTriggerCooldownUntil = 0;    // millis() when cooldown ends
bool     pwmCooldownActive = false;       // true while ignoring PWM
bool     pwmArmed = false;                 // true once startup grace expires

// Single-color LED helper.  GPIO8 is active-low on the Super Mini:
//   digitalWrite(LOW)  = LED on
//   digitalWrite(HIGH) = LED off
void setLedOn(bool on) {
  digitalWrite(LED_PIN, on ? LOW : HIGH);
  ledOn = on;
}

String jsonEscape(const String &value) {
  String result;
  result.reserve(value.length() + 8);
  for (char c : value) {
    if (c == '"' || c == '\\') result += '\\';
    if (c == '\n') result += "\\n";
    else result += c;
  }
  return result;
}

void sendJSON(int status, const String &body) {
  web.sendHeader("Access-Control-Allow-Origin", "*");
  web.send(status, "application/json; charset=utf-8", body);
}

void advertise() {
  pairingMode = true;
  pairingStartedAt = millis();
  NimBLEDevice::getAdvertising()->start();
}

bool triggerShutter() {
  Serial.printf("triggerShutter() called: cameraMode=%d bleConnected=%s instaNotify=%s be81Write=%s\n",
                (int)cameraMode, bleConnected ? "true" : "false",
                instaNotify ? "set" : "null",
                be81Write ? "set" : "null");

  // --- Architecture B: BE80 client (Ace Pro) ---
  if (cameraMode == CAM_BE80_CLIENT && be81Write != nullptr) {
    uint8_t seq = nextBe81Sequence();
    // Header16 TAKE_PICTURE: 16 bytes, no payload, no CRC, no BLE envelope.
    // bytes 0-3 = total_inner_size = 16 (header only, no protobuf payload)
    // This matches the Go2BlePacket inner-header interpretation where
    // bytes 0-3 = uint32 LE = 16 + payload_length. For payload_length=0
    // this is 16 (0x10), NOT 0. Writing 0x00000000 causes the camera to
    // see a zero-length inner packet and silently drop the command.
    uint8_t cmd[] = {
      0x10, 0x00, 0x00, 0x00,  // total_inner_size = 16 (uint32 LE)
      0x04,                     // mode = PacketType::Message
      0x00, 0x00,               // reserved
      0x03, 0x00,               // MessageCode = TAKE_PICTURE (uint16 LE)
      0x02,                     // ContentType = protobuf
      seq,                      // sequence number (1-254)
      0x00, 0x00,               // reserved
      0x80,                     // is_last_fragment=1, direction=0 (phone→camera)
      0x00, 0x00                // reserved
    };
    Serial.printf("BE80: writing TAKE_PICTURE (seq=%u) to BE81: hex:", seq);
    for (size_t i = 0; i < sizeof(cmd); i++) {
      Serial.printf(" %02X", cmd[i]);
    }
    Serial.println();
    if (!be81Write->writeValue(cmd, sizeof(cmd), true)) {
      Serial.println("BE80: BE81 write failed");
      return false;
    }
    triggerCount++;
    shutterFlashUntil = millis() + 350;
    return true;
  }

  // --- Architecture A: CE80 server (X-series) ---
  if (cameraMode == CAM_CE80_SERVER && bleConnected && instaNotify != nullptr) {
    // FC EF FE 86 <seq> 03 01 02 00 — byte[4] is a running counter (+2/frame).
    uint8_t shutter[] = {
      0xfc, 0xef, 0xfe, 0x86, ce82SeqCounter, 0x03, 0x01, 0x02, 0x00
    };
    ce82SeqCounter += 2;
    instaNotify->setValue(shutter, sizeof(shutter));
    if (!instaNotify->notify()) return false;
    triggerCount++;
    shutterFlashUntil = millis() + 350;
    return true;
  }

  // Fallback: if cameraMode is None but CE80 server has a connection, try it.
  if (bleConnected && instaNotify != nullptr) {
    uint8_t shutter[] = {
      0xfc, 0xef, 0xfe, 0x86, ce82SeqCounter, 0x03, 0x01, 0x02, 0x00
    };
    ce82SeqCounter += 2;
    instaNotify->setValue(shutter, sizeof(shutter));
    if (!instaNotify->notify()) return false;
    triggerCount++;
    shutterFlashUntil = millis() + 350;
    return true;
  }

  return false;
}

// Send a GPS RMC liveness heartbeat on ce82 so the camera does not drop
// the link.  The real remote streams this at 10 Hz; the camera treats
// silence as a dead remote and ignores button frames.
// Frame format: FC EF FE 83 00 <len> <payload>
// Payload is a minimal NMEA RMC sentence with status V (no fix).
void sendGpsHeartbeat() {
  if (!bleConnected || instaNotify == nullptr) return;
  // Minimal RMC: $GNRMC,,V,,,,,,,,,,N*53
  // The camera only cares that *something* arrives, not the GPS data itself.
  const char rmc[] = "$GNRMC,,V,,,,,,,,,,N*53";
  uint8_t frame[32];
  frame[0] = 0xfc;
  frame[1] = 0xef;
  frame[2] = 0xfe;
  frame[3] = 0x83;          // GPS frame type (not 0x86 button)
  frame[4] = 0x00;          // GPS frames always use seq 0x00
  frame[5] = (uint8_t)strlen(rmc);
  memcpy(&frame[6], rmc, strlen(rmc));
  instaNotify->setValue(frame, 6 + strlen(rmc));
  instaNotify->notify();
}

String cameraName() {
  return "Insta360 (experimental)";
}

void clearBluetoothBonds() {
  NimBLEDevice::deleteAllBonds();
  if (instaServer != nullptr) {
    instaServer->disconnect(0);
  }
  if (cameraClient && cameraClient->isConnected()) {
    cameraClient->disconnect();
  }
  cameraMode = CAM_NONE;
  be81Write = nullptr;
  be82Notify = nullptr;
  bleConnected = false;
  advertise();
}

class InstaServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &info) override {
    // Remember who is connected so we can identify/reject it later.
    ce80PeerAddress = info.getAddress();
    ce80HasPeer = true;
    ce80ConnHandle = info.getConnHandle();

    // Reject the CE80 (GPS-remote) link while a BE80 connection is pending
    // or active, or when this peer is a known Ace camera. The Ace connects
    // to any "Insta360 GPS Remote" it remembers, but it cannot use the
    // CE80 protocol — accepting it would wedge the firmware in a mode
    // where the shutter never fires.
    if (cameraMode == CAM_BE80_CLIENT || cameraMode == CAM_BE80_SETTING_UP ||
        cameraMode == CAM_BE80_CONNECTING || be80ConnectPending ||
        (be80PeerKnown && info.getAddress() == be80PendingAddress)) {
      Serial.println("CE80 server: rejecting client (BE80 mode / known Ace)");
      server->disconnect(info.getConnHandle());
      ce80HasPeer = false;
      ce80ConnHandle = 0xFFFF;
      return;
    }
    // Otherwise, accept the CE80 connection (X-series camera).
    bleConnected = true;
    pairingMode = false;
    cameraMode = CAM_CE80_SERVER;
    if (be80ScanActive) {
      NimBLEDevice::getScan()->stop();
      be80ScanActive = false;
    }
    Serial.printf("CE80 server: client connected, cameraMode=%d\n", (int)cameraMode);
  }
  void onDisconnect(
    NimBLEServer *server, NimBLEConnInfo &info, int reason) override {
    bleConnected = false;
    ce80HasPeer = false;
    ce80ConnHandle = 0xFFFF;
    if (cameraMode == CAM_CE80_SERVER) {
      cameraMode = CAM_NONE;
    }
    // Resume CE80 advertising only when no Ace is in play (be80PeerKnown)
    // and no BE80 attempt is running. Otherwise the Ace would immediately
    // reconnect to CE80 and block its own BE80 advertisement.
    if (cameraMode == CAM_NONE && !be80ConnectPending &&
        be80InitialScanDone && !be80PeerKnown) {
      advertise();
    }
    Serial.printf("CE80 server: client disconnected (reason=%d), cameraMode=%d\n",
                  reason, (int)cameraMode);
  }
} instaServerCallbacks;

// --- BE80 client callbacks ---
// These fire from the NimBLE client thread. Keep them short — only update
// state flags. Service discovery and BE82 subscription happen in loop().
class CameraClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *pClient) override {
    Serial.println("BE80 client: connected, setting up...");
    cameraMode = CAM_BE80_SETTING_UP;
    pairingMode = false;  // BE80 link established — stop pairing blink
  }
  void onConnectFail(NimBLEClient *pClient, int reason) override {
    Serial.printf("BE80 client: connect failed (reason=%d)\n", reason);
    cameraMode = CAM_NONE;
    be81Write = nullptr;
    be82Notify = nullptr;
    be80RetryAt = millis() + be80RetryDelay;
    be80RetryDelay = min(be80RetryDelay * 2, BE80_RETRY_MAX_MS);
  }
  void onDisconnect(NimBLEClient *pClient, int reason) override {
    Serial.printf("BE80 client: disconnected (reason=%d)\n", reason);
    cameraMode = CAM_NONE;
    be81Write = nullptr;
    be82Notify = nullptr;
    be80RetryAt = millis() + be80RetryDelay;
    be80RetryDelay = min(be80RetryDelay * 2, BE80_RETRY_MAX_MS);
  }
} cameraClientCallbacks;

// BE82 notification handler — logs full hex dump for debugging.
void onBe82Notify(NimBLERemoteCharacteristic *pChar, uint8_t *pData, size_t length, bool isNotify) {
  Serial.printf("BE82 notify: %u bytes hex:", (unsigned)length);
  for (size_t i = 0; i < length && i < 32; i++) {
    Serial.printf(" %02X", pData[i]);
  }
  if (length > 32) Serial.printf(" ...(%u more)", (unsigned)(length - 32));
  Serial.println();
  if (length >= 9) {
    uint16_t status = pData[7] | (pData[8] << 8);
    Serial.printf("  -> status=%u (0x%04X)\n", status, status);
  }
}

// Next BE81 sequence number (1-254, wrap to 1 — never 0).
uint8_t nextBe81Sequence() {
  if (++be81SeqCounter > 254) be81SeqCounter = 1;
  return be81SeqCounter;
}

// --- BE80 scan callbacks ---
// Looks for devices whose advertised name starts with "Ace Pro".
// When found, stores the address for loop() to initiate the connection
// (after disconnecting any CE80 server client to avoid dual-connection rejection).
class Be80ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    std::string name = dev->getName();
    if (name.size() >= strlen(ACE_PRO_NAME_PREFIX) &&
        strncmp(name.c_str(), ACE_PRO_NAME_PREFIX, strlen(ACE_PRO_NAME_PREFIX)) == 0) {
      Serial.printf("BE80 scan: found \"%s\" at %s\n", name.c_str(), dev->getAddress().toString().c_str());
      NimBLEDevice::getScan()->stop();
      be80ScanActive = false;
      be80PendingAddress = dev->getAddress();
      be80ConnectPending = true;
      be80PeerKnown = true;       // hold CE80 advertising off from now on
      be80InitialScanDone = true; // release the boot gate
    }
  }
  void onScanEnd(const NimBLEScanResults &results, int reason) override {
    // Scan burst ended without finding an Ace. Reset the active flag so
    // the next burst can start, and schedule a retry with backoff.
    be80ScanActive = false;
    be80InitialScanDone = true;
    be80RetryAt = millis() + be80RetryDelay;
    be80RetryDelay = min(be80RetryDelay * 2, BE80_RETRY_MAX_MS);
    Serial.println("BE80 scan: ended without Ace Pro");
  }
} be80ScanCallbacks;

void setupInsta360Bluetooth() {
  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setSecurityAuth(
    BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_SC);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  instaServer = NimBLEDevice::createServer();
  instaServer->setCallbacks(&instaServerCallbacks);

  NimBLEService *remote = instaServer->createService("ce80");
  remote->createCharacteristic("ce81", NIMBLE_PROPERTY::WRITE);
  instaNotify = remote->createCharacteristic(
    "ce82", NIMBLE_PROPERTY::NOTIFY);
  const uint8_t initial[] = {0};
  instaNotify->setValue(initial, sizeof(initial));
  NimBLECharacteristic *version = remote->createCharacteristic(
    "ce83", NIMBLE_PROPERTY::READ);
  const uint8_t remoteVersion[] = {0x01, 0x02};
  version->setValue(remoteVersion, sizeof(remoteVersion));

  NimBLEService *details = instaServer->createService(
    "0000d0ff-3c17-d293-8e48-14fe2e4da212");
  details->createCharacteristic("ffd1", NIMBLE_PROPERTY::WRITE);
  details->createCharacteristic("ffd2", NIMBLE_PROPERTY::READ);
  NimBLECharacteristic *detail3 = details->createCharacteristic(
    "ffd3", NIMBLE_PROPERTY::READ);
  const uint8_t detail3Value[] = {0x01, 0x90, 0x1e, 0x30};
  detail3->setValue(detail3Value, sizeof(detail3Value));
  NimBLECharacteristic *detail4 = details->createCharacteristic(
    "ffd4", NIMBLE_PROPERTY::READ);
  const uint8_t detail4Value[] = {0x01, 0x20, 0x00, 0x18};
  detail4->setValue(detail4Value, sizeof(detail4Value));
  details->createCharacteristic("ffd5", NIMBLE_PROPERTY::READ);
  details->createCharacteristic("ffd8", NIMBLE_PROPERTY::WRITE);
  details->createCharacteristic("fff1", NIMBLE_PROPERTY::READ);
  details->createCharacteristic("fff2", NIMBLE_PROPERTY::WRITE);
  details->createCharacteristic("ffe0", NIMBLE_PROPERTY::READ);
  remote->start();
  details->start();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->setName(BLE_NAME);
  advertising->addServiceUUID(remote->getUUID());
  advertising->addServiceUUID(details->getUUID());
  advertising->enableScanResponse(true);
  // Do NOT start advertising here. The first BE80 scan burst runs at boot;
  // if an Ace Pro is nearby it remembers this device as its GPS remote and
  // would immediately occupy the CE80 server with a protocol it cannot
  // use. Advertising starts from loop() once the first scan completes
  // without finding an Ace (or after the BE80 link ends for good).

  // Register BE80 scan callbacks for Ace Pro discovery.
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&be80ScanCallbacks);
  scan->setActiveScan(true);
  scan->setInterval(80);
  scan->setWindow(40);
}

// --- BE80 client management ---

// Start a BE80 scan burst if conditions allow.
void startBe80Scan() {
  if (be80ScanActive) return;
  if (cameraMode == CAM_BE80_CONNECTING || cameraMode == CAM_BE80_SETTING_UP ||
      cameraMode == CAM_BE80_CLIENT) return;
  be80ScanActive = true;
  Serial.println("BE80: starting scan for Ace Pro...");
  NimBLEDevice::getScan()->start(BE80_SCAN_DURATION_MS, false);
}

// Discover BE80 service, find BE81/BE82, and subscribe to BE82 notifications.
// Called from loop() when cameraMode == CAM_BE80_SETTING_UP.
void setupBe80Client() {
  if (!cameraClient || !cameraClient->isConnected()) {
    Serial.println("BE80 setup: client not connected, aborting");
    cameraMode = CAM_NONE;
    return;
  }
  Serial.println("BE80 setup: discovering services...");
  // Dump all services and characteristics for diagnostics.
  const std::vector<NimBLERemoteService *> &services = cameraClient->getServices(true);
  for (const auto &svc : services) {
    Serial.printf("  Service: %s\n", svc->getUUID().toString().c_str());
    const auto &chars = svc->getCharacteristics(true);
    for (const auto &chr : chars) {
      Serial.printf("    Char: %s  props:", chr->getUUID().toString().c_str());
      if (chr->canRead()) Serial.print(" R");
      if (chr->canWrite()) Serial.print(" W");
      if (chr->canWriteNoResponse()) Serial.print(" WNR");
      if (chr->canNotify()) Serial.print(" N");
      if (chr->canIndicate()) Serial.print(" I");
      Serial.println();
    }
  }

  NimBLERemoteService *be80 = cameraClient->getService(BE80_SERVICE_UUID);
  if (!be80) {
    Serial.println("BE80 setup: BE80 service not found — not an Ace Pro?");
    cameraClient->disconnect();
    cameraMode = CAM_NONE;
    be80RetryAt = millis() + be80RetryDelay;
    return;
  }
  be81Write = be80->getCharacteristic(BE81_WRITE_UUID);
  be82Notify = be80->getCharacteristic(BE82_NOTIFY_UUID);
  if (!be81Write || !be82Notify) {
    Serial.println("BE80 setup: BE81 or BE82 missing");
    cameraClient->disconnect();
    cameraMode = CAM_NONE;
    be80RetryAt = millis() + be80RetryDelay;
    return;
  }
  Serial.println("BE80 setup: subscribing to BE82 notifications...");
  if (!be82Notify->subscribe(true, onBe82Notify, true)) {
    Serial.println("BE80 setup: BE82 subscribe failed");
    cameraClient->disconnect();
    cameraMode = CAM_NONE;
    be80RetryAt = millis() + be80RetryDelay;
    return;
  }

  // Also subscribe to any other notify characteristics on the camera
  // (B002/B003/B004 on B000 service, AE02 on AE00 service).
  for (const auto &svc : services) {
    const auto &chars = svc->getCharacteristics();
    for (const auto &chr : chars) {
      if (chr->canNotify() && chr != be82Notify) {
        Serial.printf("BE80 setup: also subscribing to %s on %s\n",
                      chr->getUUID().toString().c_str(),
                      svc->getUUID().toString().c_str());
        chr->subscribe(true, onBe82Notify, true);
      }
    }
  }

  Serial.println("BE80 setup: ready! BE81 write + BE82 notify active");
  cameraMode = CAM_BE80_CLIENT;
  be80RetryDelay = BE80_RETRY_BASE_MS;  // reset backoff on success
}

// Non-blocking state machine for BE80 client. Called from loop().
void maintainCameraClient() {
  // Handle pending BE80 connection from scan result.
  if (be80ConnectPending) {
    be80ConnectPending = false;
    // An X-series camera already owns the CE80 link — it wins, drop the
    // BE80 attempt until the camera disconnects.
    if (bleConnected && cameraMode == CAM_CE80_SERVER) {
      Serial.println("BE80: CE80 camera connected, skipping BE80 attempt");
      return;
    }
    // The stuck-Ace case: the Ace occupies CE80 (accepted before we knew
    // better). Kick it off, wait for the link to drop, then connect.
    if (bleConnected && instaServer != nullptr) {
      Serial.println("BE80: disconnecting CE80 client (stuck Ace) before BE80 connect");
      instaServer->disconnect(ce80ConnHandle != 0xFFFF ? ce80ConnHandle : 0);
      bleConnected = false;
      be80RetryAt = millis() + 500;
      be80ConnectPending = true;  // re-try after the link settles
      return;
    }
    if (millis() < be80RetryAt) return;  // waiting for disconnect to settle

    // Stop CE80 advertising while connecting directly to the camera.
    // Otherwise the Ace keeps connecting to our CE80 server (which we then
    // reject) instead of staying connectable for our BE80 client link.
    if (NimBLEDevice::getAdvertising()->isAdvertising()) {
      Serial.println("BE80: pausing CE80 advertising during BE80 connect");
      NimBLEDevice::getAdvertising()->stop();
    }

    if (!cameraClient) {
      cameraClient = NimBLEDevice::createClient();
      cameraClient->setClientCallbacks(&cameraClientCallbacks, false);
    }
    cameraClient->setPeerAddress(be80PendingAddress);
    cameraMode = CAM_BE80_CONNECTING;
    Serial.println("BE80 client: initiating async connect...");
    cameraClient->connect(true, true, true);  // deleteAttrs=true, async=true, exchangeMTU=true
    return;
  }

  switch (cameraMode) {
    case CAM_NONE:
      // Keep CE80 advertising available for X-series cameras once the
      // initial scan burst has proven no Ace is around.
      if (be80InitialScanDone && !be80PeerKnown && !be80ScanActive &&
          !bleConnected && !NimBLEDevice::getAdvertising()->isAdvertising()) {
        advertise();
      }
      if (!bleConnected && millis() >= be80RetryAt) {
        startBe80Scan();
      }
      break;
    case CAM_BE80_CONNECTING:
      // Waiting for async connect — onConnect callback transitions to SETTING_UP.
      break;
    case CAM_BE80_SETTING_UP:
      setupBe80Client();
      break;
    case CAM_BE80_CLIENT:
      // Connection is live. triggerShutter() handles commands.
      break;
    case CAM_CE80_SERVER:
      // X-series mode — BE80 client not needed.
      break;
  }
}

void pollPrinter();

void setupWeb() {
  web.on("/", HTTP_GET, [] {
    web.send_P(200, "text/html; charset=utf-8", LAYERSHOT_DASHBOARD);
  });
  web.on("/status", HTTP_GET, [] {
    String ip = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    String body = "{\"ok\":true,\"name\":\"" + String(DEVICE_NAME) + "\",\"firmware\":\"" +
      FIRMWARE_VERSION + "\",\"hostname\":\"" + deviceHostname + ".local\",\"ip\":\"" + ip +
      "\",\"ssid\":\"" + jsonEscape(WiFi.SSID()) + "\",\"rssi\":" + String(WiFi.RSSI()) +
      ",\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") +
      ",\"camera_type\":\"" + jsonEscape(cameraType) + "\"" +
      ",\"camera_name\":\"" + jsonEscape(cameraName()) + "\"" +
      ",\"bluetooth\":" + String(bleConnected ? "true" : "false") +
      ",\"pairing\":" + String(pairingMode ? "true" : "false") +
      ",\"autonomous\":" + String(autonomousEnabled ? "true" : "false") +
      ",\"printer\":\"" + jsonEscape(printerHost) + "\"" +
      ",\"printer_port\":" + String(printerPort) +
      ",\"printer_connected\":" + String(printerConnected ? "true" : "false") +
      ",\"printer_state\":\"" + jsonEscape(printerState) + "\"" +
      ",\"printer_http_code\":" + String(printerHttpCode) +
      ",\"shutter_delay_ms\":" + String(stabilizationMs) +
      ",\"current_layer\":" + String(lastPrinterLayer) +
      ",\"total_layers\":" + String(printerTotalLayers) +
      ",\"commands\":" + String(commandCount) +
      ",\"last_command\":\"" + jsonEscape(lastCommand) + "\"" +
      ",\"triggers\":" + String(triggerCount) + "}";
    sendJSON(200, body);
  });
  web.on("/trigger", HTTP_POST, [] {
    commandCount++;
    if (triggerShutter()) {
      lastCommand = "shutter_sent";
      sendJSON(200, "{\"ok\":true,\"triggered\":true}");
    } else {
      lastCommand = "shutter_failed";
      sendJSON(409, "{\"ok\":false,\"error\":\"camera_not_connected\"}");
    }
  });
  web.on("/led-test", HTTP_POST, [] {
    commandCount++; lastCommand = "led_test";
    setLedOn(false); delay(225); setLedOn(true); delay(225);
    setLedOn(false); delay(225); setLedOn(true); delay(225);
    setLedOn(false); delay(225); setLedOn(true); delay(225);
    setLedOn(false); delay(225); setLedOn(true); delay(225);
    sendJSON(200, "{\"ok\":true,\"led\":true}");
  });
  web.on("/pair", HTTP_POST, [] {
    commandCount++; lastCommand = "pairing_enabled";
    advertise();
    sendJSON(200, "{\"ok\":true,\"pairing\":true}");
  });
  web.on("/reset-bonds", HTTP_POST, [] {
    commandCount++; lastCommand = "pairing_erased";
    clearBluetoothBonds();
    sendJSON(200, "{\"ok\":true,\"bondsCleared\":true}");
  });
  web.on("/configure", HTTP_POST, [] {
    String ssid = web.arg("ssid");
    if (ssid.isEmpty()) {
      sendJSON(400, "{\"ok\":false,\"error\":\"missing_ssid\"}");
      return;
    }
    preferences.begin("layershot", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", web.arg("password"));
    preferences.remove("static_ip");
    preferences.remove("gateway");
    preferences.remove("netmask");
    preferences.remove("dns");
    preferences.end();
    sendJSON(200, "{\"ok\":true,\"restarting\":true}");
    delay(500);
    ESP.restart();
  });
  web.on("/camera-config", HTTP_POST, [] {
    String target = web.arg("camera");
    if (target != "insta360") {
      sendJSON(400, "{\"ok\":false,\"error\":\"unsupported_camera\"}");
      return;
    }
    cameraType = target;
    preferences.begin("layershot", false);
    preferences.putString("camera", cameraType);
    preferences.end();
    sendJSON(200, "{\"ok\":true,\"camera\":\"" + jsonEscape(cameraType) + "\"}");
  });
  web.on("/printer-config", HTTP_POST, [] {
    String host = web.arg("host");
    if (host.isEmpty()) { sendJSON(400, "{\"ok\":false,\"error\":\"missing_host\"}"); return; }
    uint16_t newPort = (uint16_t)max(1L, web.arg("port").toInt());
    uint16_t newEvery = (uint16_t)max(1L, web.arg("every").toInt());
    uint16_t newSkip = (uint16_t)max(0L, web.arg("skip").toInt());
    uint16_t newStop = (uint16_t)max(0L, web.arg("stop").toInt());
    uint16_t newDelay = (uint16_t)max(0L, web.arg("delay").toInt());
    preferences.begin("layershot", false);
    preferences.putString("printer", host);
    preferences.putUShort("port", newPort);
    preferences.putUShort("every", newEvery);
    preferences.putUShort("skip", newSkip);
    preferences.putUShort("stop", newStop);
    preferences.putUShort("delay", newDelay);
    preferences.putBool("autonomous", true);
    preferences.end();
    printerHost = host;
    printerPort = newPort;
    captureEvery = newEvery;
    skipLayers = newSkip;
    stopAfterLayer = newStop;
    stabilizationMs = newDelay;
    autonomousEnabled = true;
    lastPrinterLayer = -1;
    sendJSON(200, "{\"ok\":true,\"autonomous\":true}");
  });
  web.on("/printer-test", HTTP_POST, [] {
    lastPrinterPoll = 0;
    pollPrinter();
    if (printerConnected) sendJSON(200, "{\"ok\":true,\"printer\":true}");
    else sendJSON(503, "{\"ok\":false,\"error\":\"printer_not_found\"}");
  });
  web.on("/autonomous-stop", HTTP_POST, [] {
    autonomousEnabled = false;
    preferences.begin("layershot", false); preferences.putBool("autonomous", false); preferences.end();
    sendJSON(200, "{\"ok\":true,\"autonomous\":false}");
  });
  web.onNotFound([] { sendJSON(404, "{\"ok\":false,\"error\":\"not_found\"}"); });
  web.begin();
}

String decodeHex(const String &value) {
  String decoded;
  decoded.reserve(value.length() / 2);
  for (size_t i = 0; i + 1 < value.length(); i += 2) {
    char pair[3] = {value[i], value[i + 1], 0};
    decoded += (char)strtoul(pair, nullptr, 16);
  }
  return decoded;
}

String serialField(const String &line, int field) {
  int start = 0;
  for (int current = 0; current < field; current++) {
    start = line.indexOf('\t', start);
    if (start < 0) return "";
    start++;
  }
  int end = line.indexOf('\t', start);
  return end < 0 ? line.substring(start) : line.substring(start, end);
}

void handleSerialProvisioning() {
  while (Serial.available()) {
    char incoming = (char)Serial.read();
    if (incoming == '\r') continue;
    if (incoming != '\n') {
      if (serialLine.length() < 1400) serialLine += incoming;
      continue;
    }
    if (serialLine.startsWith("LAYERSHOT_CONFIG\t")) {
      String newSsid = decodeHex(serialField(serialLine, 1));
      String newPassword = decodeHex(serialField(serialLine, 2));
      String newPrinter = serialField(serialLine, 3);
      if (!newSsid.isEmpty() && !newPrinter.isEmpty()) {
        preferences.begin("layershot", false);
        preferences.putString("ssid", newSsid);
        preferences.putString("password", newPassword);
        preferences.putString("printer", newPrinter);
        preferences.putUShort("port", (uint16_t)max(1L, serialField(serialLine, 4).toInt()));
        preferences.putUShort("every", (uint16_t)max(1L, serialField(serialLine, 5).toInt()));
        preferences.putUShort("skip", (uint16_t)max(0L, serialField(serialLine, 6).toInt()));
        preferences.putUShort("stop", (uint16_t)max(0L, serialField(serialLine, 7).toInt()));
        preferences.putUShort("delay", (uint16_t)max(0L, serialField(serialLine, 8).toInt()));
        String newCamera = serialField(serialLine, 9);
        newCamera = "insta360";
        preferences.putString("camera", newCamera);
        String newHostname = serialField(serialLine, 10);
        if (!newHostname.isEmpty()) preferences.putString("hostname", newHostname);
        preferences.putString("static_ip", serialField(serialLine, 11));
        preferences.putString("gateway", serialField(serialLine, 12));
        preferences.putString("netmask", serialField(serialLine, 13));
        preferences.putString("dns", serialField(serialLine, 14));
        preferences.putBool("autonomous", true);
        preferences.end();
        Serial.println("LAYERSHOT_CONFIG_OK");
        Serial.flush();
        delay(300);
        ESP.restart();
      } else {
        Serial.println("LAYERSHOT_CONFIG_ERROR");
      }
    }
    serialLine = "";
  }
}

int jsonIntegerAfter(const String &body, const String &key) {
  int position = body.indexOf("\"" + key + "\"");
  if (position < 0) return -1;
  position = body.indexOf(':', position);
  if (position < 0) return -1;
  position++;
  while (position < (int)body.length() && (body[position] == ' ' || body[position] == '"')) position++;
  if (body.substring(position, position + 4) == "null") return -1;
  return body.substring(position).toInt();
}

void pollPrinter() {
  if (!autonomousEnabled || printerHost.isEmpty() || WiFi.status() != WL_CONNECTED || millis() - lastPrinterPoll < 1000) return;
  lastPrinterPoll = millis();
  HTTPClient http;
  String url = "http://" + printerHost + ":" + String(printerPort) + "/printer/objects/query?print_stats&virtual_sdcard&display_status";
  http.setConnectTimeout(2500);
  http.setTimeout(3500);
  if (!http.begin(url)) return;
  int code = http.GET();
  printerHttpCode = code;
  if (code == 200) {
    String body = http.getString();
    printerConnected = true;
    bool printing = body.indexOf("\"state\":\"printing\"") >= 0 || body.indexOf("\"state\": \"printing\"") >= 0;
    bool virtualSdActive = body.indexOf("\"is_active\":true") >= 0 || body.indexOf("\"is_active\": true") >= 0;
    bool layerMonitoringActive = printing || virtualSdActive;
    if (layerMonitoringActive) printerState = "printing";
    else if (body.indexOf("\"state\":\"paused\"") >= 0 || body.indexOf("\"state\": \"paused\"") >= 0) printerState = "paused";
    else if (body.indexOf("\"state\":\"complete\"") >= 0 || body.indexOf("\"state\": \"complete\"") >= 0) printerState = "complete";
    else if (body.indexOf("\"state\":\"cancelled\"") >= 0 || body.indexOf("\"state\": \"cancelled\"") >= 0) printerState = "cancelled";
    else if (body.indexOf("\"state\":\"standby\"") >= 0 || body.indexOf("\"state\": \"standby\"") >= 0) printerState = "standby";
    else printerState = "ready";
    int currentLayer = jsonIntegerAfter(body, "current_layer");
    if (currentLayer < 0) currentLayer = jsonIntegerAfter(body, "layer");
    int totalLayers = jsonIntegerAfter(body, "total_layer");
    if (totalLayers < 0) totalLayers = jsonIntegerAfter(body, "layer_count");
    if (totalLayers >= 0) printerTotalLayers = totalLayers;
    if (layerMonitoringActive && currentLayer >= 0 && currentLayer != lastPrinterLayer) {
      if (lastPrinterLayer >= 0 && currentLayer > lastPrinterLayer && currentLayer > skipLayers &&
          (currentLayer - skipLayers) % max(1, (int)captureEvery) == 0 &&
          (stopAfterLayer == 0 || currentLayer <= stopAfterLayer)) {
        shutterPending = true;
        shutterDueAt = millis() + stabilizationMs;
      }
      lastPrinterLayer = currentLayer;
    } else if (!layerMonitoringActive) {
      lastPrinterLayer = -1;
    }
  } else {
    printerConnected = false;
    printerState = "offline";
  }
  http.end();
}

void updateScheduledShutter() {
  if (shutterPending && (int32_t)(millis() - shutterDueAt) >= 0) {
    shutterPending = false;
    triggerShutter();
  }
}

void applyPreferredNetwork() {
  if (preferredIp.isEmpty()) return;
  IPAddress address, gateway, netmask, dns;
  if (address.fromString(preferredIp) &&
      gateway.fromString(preferredGateway) &&
      netmask.fromString(preferredNetmask) &&
      dns.fromString(preferredDns)) {
    WiFi.config(address, gateway, netmask, dns);
  }
}

void startWiFiServices() {
  if (!otaReady) {
    MDNS.begin(deviceHostname.c_str());
    MDNS.addService("http", "tcp", 80);
    ArduinoOTA.setHostname(deviceHostname.c_str());
    ArduinoOTA.setPassword("layershot");
    ArduinoOTA.begin();
    otaReady = true;
  }
  WiFi.softAPdisconnect(true);
  wifiError = false;
}

void connectWiFi() {
  preferences.begin("layershot", true);
  wifiSsid = preferences.getString("ssid", "");
  wifiPassword = preferences.getString("password", "");
  preferredIp = preferences.getString("static_ip", "");
  preferredGateway = preferences.getString("gateway", "");
  preferredNetmask = preferences.getString("netmask", "");
  preferredDns = preferences.getString("dns", "");
  preferences.end();

  WiFi.setHostname(deviceHostname.c_str());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  if (!wifiSsid.isEmpty()) {
    wifiConnecting = true;
    WiFi.mode(WIFI_STA);
    applyPreferredNetwork();
    WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
    uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < 18000) {
      // The desktop app provisions the freshly flashed board over USB.
      // Continue consuming serial data while an older Wi-Fi profile is timing
      // out, otherwise opening the port repeatedly can reset the C3 forever.
      handleSerialProvisioning();
      setLedOn(true);
      delay(150);
      setLedOn(false);
      delay(150);
    }
    wifiConnecting = false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    wifiError = !wifiSsid.isEmpty();
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(SETUP_AP);
  } else {
    startWiFiServices();
  }
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!otaReady) startWiFiServices();
    return;
  }
  if (wifiSsid.isEmpty() || millis() - lastWiFiReconnectAttempt < 10000) return;
  lastWiFiReconnectAttempt = millis();
  wifiConnecting = true;
  WiFi.mode(WIFI_AP_STA);
  applyPreferredNetwork();
  WiFi.disconnect(false, false);
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  wifiConnecting = false;
  wifiError = true;
}

void updateButton() {
  bool pressed = digitalRead(PAIR_BUTTON_PIN) == LOW;
  if (pressed && buttonPressedAt == 0) {
    buttonPressedAt = millis();
    Serial.println("BOOT pressed");
  }
  if (!pressed && buttonPressedAt != 0) {
    uint32_t duration = millis() - buttonPressedAt;
    buttonPressedAt = 0;
    Serial.printf("BOOT released (held %lu ms)\n", (unsigned long)duration);
    if (duration >= 10000) { Serial.println("BOOT: clear bonds"); clearBluetoothBonds(); }
    else if (duration >= 3000) { Serial.println("BOOT: advertise/pairing"); advertise(); }
    else if (duration >= 50) {
      Serial.println("BOOT: short press -> triggerShutter()");
      bool ok = triggerShutter();
      Serial.printf("triggerShutter() returned %s (bleConnected=%s)\n",
                     ok ? "true" : "false", bleConnected ? "true" : "false");
    }
  }
}

void updateLED() {
  uint32_t now = millis();

  // Shutter flash: brief off-on pulse (purple equivalent).
  if (shutterFlashUntil > now) {
    setLedOn(true);
    return;
  }

  // Pairing mode expires after 60 s.
  if (pairingMode && !bleConnected && now - pairingStartedAt > 60000) {
    pairingMode = false;
  }

  // Pairing mode: fast blink (blue equivalent).
  if (pairingMode) {
    if (now - ledLastToggleAt > LED_PAIR_BLINK_MS) {
      ledLastToggleAt = now;
      setLedOn(!ledOn);
    }
    return;
  }

  // Connected: steady on (green equivalent). Covers both transports:
  // CE80 server (bleConnected) and BE80 client (CAM_BE80_CLIENT).
  if (bleConnected || cameraMode == CAM_BE80_CLIENT) {
    if (!ledOn) setLedOn(true);
    return;
  }

  // Not connected: slow blink (red equivalent).
  if (now - ledLastToggleAt > LED_IDLE_BLINK_MS) {
    ledLastToggleAt = now;
    setLedOn(!ledOn);
  }
}

// --- PWM trigger: ISR, setup, and non-blocking detection ---
// The CyberBrick servo port outputs a standard 50 Hz PWM signal (20 ms period,
// 500–2500 µs pulse width).  We measure the pulse width with a GPIO CHANGE
// interrupt and micros(), then trigger the shutter when the width exceeds the
// threshold.  A 5-second non-blocking cooldown prevents repeated triggers from
// the continuous 50 Hz signal.

void IRAM_ATTR pwmIsrHandler() {
  bool level = digitalRead(PWM_TRIGGER_PIN) == HIGH;
  if (level) {
    // Rising edge – start of pulse.
    pwmRiseMicros = micros();
  } else {
    // Falling edge – end of pulse.  Compute width only if we saw a rising edge.
    if (pwmRiseMicros != 0) {
      uint32_t width = micros() - pwmRiseMicros;
      pwmRiseMicros = 0;
      // Only accept pulses within the valid servo range to reject noise and
      // spurious edges at boot or during signal glitches.
      if (width >= PWM_TRIGGER_MIN_US && width <= PWM_TRIGGER_MAX_US) {
        pwmLastWidthUs = width;
        pwmNewPulse = true;
      }
    }
  }
}

void setupPWM() {
  pinMode(PWM_TRIGGER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_TRIGGER_PIN), pwmIsrHandler, CHANGE);
  // Start the startup grace timer so boot-time glitches and initial signal
  // settling do not cause a false trigger.
  pwmArmed = false;
  pwmTriggerCooldownUntil = millis() + PWM_STARTUP_GRACE_MS;
  pwmCooldownActive = true;
  Serial.println("PWM trigger: armed after startup grace");
}

void updatePWM() {
  uint32_t now = millis();

  // Handle startup grace / cooldown expiry.
  if (pwmCooldownActive && (int32_t)(now - pwmTriggerCooldownUntil) >= 0) {
    if (!pwmArmed) {
      pwmArmed = true;
      Serial.println("PWM trigger re-armed");
    }
    pwmCooldownActive = false;
  }

  // If a new pulse was captured by the ISR, examine it.
  if (pwmNewPulse) {
    // Atomically read and clear the shared values.
    noInterrupts();
    uint32_t width = pwmLastWidthUs;
    pwmNewPulse = false;
    interrupts();

    // Ignore all pulses while in cooldown or startup grace.
    if (pwmCooldownActive) {
      return;
    }

    // Valid pulse but below trigger threshold – no action, no log (avoid spam).
    if (width < PWM_TRIGGER_THRESHOLD_US) {
      return;
    }

    // Valid trigger: fire shutter once and enter cooldown.
    Serial.printf("PWM shutter triggered (pulse: %lu us)\n", (unsigned long)width);
    if (triggerShutter()) {
      lastCommand = "pwm_shutter_sent";
    } else {
      Serial.println("PWM trigger: shutter failed (camera not connected)");
      lastCommand = "pwm_shutter_failed";
    }
    pwmTriggerCooldownUntil = now + PWM_TRIGGER_COOLDOWN_MS;
    pwmCooldownActive = true;
    Serial.println("PWM trigger ignored: cooldown (5 s)");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PAIR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  setLedOn(true);  // LED on at boot (red equivalent — not connected yet)
  setupInsta360Bluetooth();
  setupPWM();
  preferences.begin("layershot", true);
  printerHost = preferences.getString("printer", "");
  printerPort = preferences.getUShort("port", 4408);
  captureEvery = preferences.getUShort("every", 1);
  skipLayers = preferences.getUShort("skip", 0);
  stopAfterLayer = preferences.getUShort("stop", 0);
  stabilizationMs = preferences.getUShort("delay", 3000);
  cameraType = "insta360";
  deviceHostname = preferences.getString("hostname", HOSTNAME);
  autonomousEnabled = preferences.getBool("autonomous", false);
  preferences.end();
  connectWiFi();
  setupWeb();
  Serial.printf("%s %s\n", BLE_NAME, FIRMWARE_VERSION);
}

void loop() {
  handleSerialProvisioning();
  maintainWiFi();
  web.handleClient();
  if (otaReady) ArduinoOTA.handle();
  updateButton();
  updateLED();
  maintainCameraClient();
  // GPS liveness heartbeat: send at 10 Hz so X-series cameras keep the link
  // alive. Only needed for CE80 server mode — not for BE80 client (Ace Pro).
  if (cameraMode == CAM_CE80_SERVER && bleConnected &&
      millis() - lastGpsHeartbeatAt >= GPS_HEARTBEAT_INTERVAL_MS) {
    lastGpsHeartbeatAt = millis();
    sendGpsHeartbeat();
  }
  updatePWM();
  pollPrinter();
  updateScheduledShutter();
  delay(5);
}
