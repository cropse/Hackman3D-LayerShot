#include <Arduino.h>
#include <WiFi.h>  // kept for ESP32-C3 RF coexistence (shared radio)
// #include <HTTPClient.h>    // WiFi disabled
// #include <WebServer.h>     // WiFi disabled
#include <Preferences.h>
// #include <ESPmDNS.h>       // WiFi disabled
// #include <ArduinoOTA.h>    // WiFi disabled
#include <NimBLEDevice.h>
#include "dashboard.h"

static const char *FIRMWARE_VERSION = "2.3.0-insta360-ace";
static const char *HOSTNAME = "hackman-layershot";
static const char *DEVICE_NAME = "Hackman3D LayerShot";
static const char *BLE_NAME = "Hackman3D LayerShot Insta360";
static const char *SETUP_AP = "Hackman3D-LayerShot-Setup";
// GPIO9 (BOOT) is a strapping pin — linking the NimBLE library locks it HIGH.
// Use GPIO3 with an external button to GND instead.
static const uint8_t PAIR_BUTTON_PIN = 3;
// MakerGo C3 Super Mini: single blue LED on GPIO8, active-low.
// Connection state is shown with slow/fast blink, steady on, and shutter flash.
static const uint8_t LED_PIN = 8;
static const uint32_t LED_PAIR_BLINK_MS   = 450;
static const uint32_t LED_IDLE_BLINK_MS   = 1200;
static const uint32_t LED_SHUTTER_FLASH_MS = 350;
bool     ledOn = false;
uint32_t ledLastToggleAt = 0;

// Ace cameras expose a BE80 GATT service that LayerShot controls directly as
// a BLE central/client. Shutter commands are Header16 frames written to BE81.
static const char *BE80_SERVICE_UUID   = "be80";
static const char *BE81_WRITE_UUID     = "be81";
static const char *BE82_NOTIFY_UUID    = "be82";
static const char *ACE_PRO_NAME_PREFIX = "Ace Pro";
static const uint32_t BE80_SCAN_DURATION_MS = 5000;   // scan burst length
static const uint32_t BE80_RETRY_BASE_MS     = 1000;  // initial backoff
static const uint32_t BE80_RETRY_MAX_MS      = 10000; // capped backoff

enum CameraMode {
  CAM_NONE,
  CAM_BE80_CONNECTING,
  CAM_BE80_SETTING_UP,
  CAM_BE80_CLIENT
};
CameraMode cameraMode = CAM_NONE;

NimBLEClient *cameraClient = nullptr;
NimBLERemoteCharacteristic *be81Write = nullptr;
NimBLERemoteCharacteristic *be82Notify = nullptr;
uint8_t  be81SeqCounter = 0;   // Header16 sequence: 1-254, wraps to 1
uint32_t be80RetryAt = 0;      // millis() when next scan attempt is allowed
uint32_t be80RetryDelay = BE80_RETRY_BASE_MS;
uint32_t lastBe82NotifyAt = 0;  // millis() of last BE82 notification (debug only)
bool be80ScanActive = false;
NimBLEAddress be80PendingAddress;
bool be80ConnectPending = false;

// WebServer web(80);  // WiFi disabled
Preferences preferences;
bool wifiConnecting = false;
bool wifiError = false;
bool otaReady = false;
uint32_t triggerCount = 0;
uint32_t commandCount = 0;
String lastCommand = "startup";
uint32_t buttonPressedAt = 0;
uint32_t shutterFlashUntil = 0;

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

// WiFi disabled — sendJSON removed
// void sendJSON(int status, const String &body) {
//   web.sendHeader("Access-Control-Allow-Origin", "*");
//   web.send(status, "application/json; charset=utf-8", body);
// }

void restartScan() {
  Serial.println("Scan restart requested");
  if (cameraClient && cameraClient->isConnected()) {
    cameraClient->disconnect();
  }
  cameraMode = CAM_NONE;
  be81Write = nullptr;
  be82Notify = nullptr;
  lastBe82NotifyAt = 0;
  be80ScanActive = false;
  be80RetryAt = 0;
  be80RetryDelay = BE80_RETRY_BASE_MS;
}

bool triggerShutter() {
  Serial.printf("triggerShutter() called: cameraMode=%d be81Write=%s\n",
                (int)cameraMode, be81Write ? "set" : "null");

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
      Serial.println("BE80: BE81 write failed — link may be dead, forcing reconnect");
      if (cameraClient && cameraClient->isConnected()) {
        cameraClient->disconnect();
      }
      cameraMode = CAM_NONE;
      be81Write = nullptr;
      be82Notify = nullptr;
      lastBe82NotifyAt = 0;
      // onDisconnect callback will set be80RetryAt + be80RetryDelay.
      return false;
    }
    triggerCount++;
    shutterFlashUntil = millis() + LED_SHUTTER_FLASH_MS;
    return true;
  }

  return false;
}

String cameraName() {
  return "Insta360 (experimental)";
}

void clearBluetoothBonds() {
  Serial.println("Clearing all Bluetooth bonds");
  NimBLEDevice::deleteAllBonds();
  Serial.flush();
  delay(100);
  ESP.restart();
}

// --- BE80 client callbacks ---
// These fire from the NimBLE client thread. Keep them short — only update
// state flags. Service discovery and BE82 subscription happen in loop().
class CameraClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *pClient) override {
    Serial.printf("BE80 client: connected (connHandle=%d, peer=%s), setting up...\n",
                  pClient->getConnHandle(), pClient->getPeerAddress().toString().c_str());
    cameraMode = CAM_BE80_SETTING_UP;
  }
  void onConnectFail(NimBLEClient *pClient, int reason) override {
    Serial.printf("BE80 client: connect failed (reason=%d)\n", reason);
    cameraMode = CAM_NONE;
    be81Write = nullptr;
    be82Notify = nullptr;
    lastBe82NotifyAt = 0;
    be80RetryAt = millis() + be80RetryDelay;
    be80RetryDelay = min(be80RetryDelay * 2, BE80_RETRY_MAX_MS);
  }
  void onDisconnect(NimBLEClient *pClient, int reason) override {
    Serial.printf("BE80 client: disconnected (reason=%d, connHandle=%d)\n",
                  reason, pClient->getConnHandle());
    cameraMode = CAM_NONE;
    be81Write = nullptr;
    be82Notify = nullptr;
    lastBe82NotifyAt = 0;
    be80RetryAt = millis() + be80RetryDelay;
    be80RetryDelay = min(be80RetryDelay * 2, BE80_RETRY_MAX_MS);
  }
} cameraClientCallbacks;

// BE82 notification handler — logs full hex dump for debugging.
void onBe82Notify(NimBLERemoteCharacteristic *pChar, uint8_t *pData, size_t length, bool isNotify) {
  lastBe82NotifyAt = millis();  // feed the watchdog
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
// When found, stores the address for loop() to initiate the connection.
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
    }
  }
  void onScanEnd(const NimBLEScanResults &results, int reason) override {
    be80ScanActive = false;
    be80RetryAt = millis() + be80RetryDelay;
    uint32_t prevDelay = be80RetryDelay;
    be80RetryDelay = min(be80RetryDelay * 2, BE80_RETRY_MAX_MS);
    Serial.printf("BE80 scan: ended without Ace Pro (retry in %lu ms)\n", prevDelay);
    // Don't reboot. Just keep retrying. Ace Pro may have a long advertising
    // interval — 5s scan might miss it. Cap backoff at 10s and keep trying.
  }
} be80ScanCallbacks;

void setupInsta360Bluetooth() {
  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setSecurityAuth(
    BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_SC);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
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
  uint32_t t0 = millis();
  const std::vector<NimBLERemoteService *> &services = cameraClient->getServices(true);
  uint32_t discMs = millis() - t0;
  Serial.printf("BE80 setup: getServices(true) took %lu ms, found %u service(s)\n",
                (unsigned long)discMs, (unsigned)services.size());

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
  bool subOk = be82Notify->subscribe(true, onBe82Notify, true);
  Serial.printf("BE80 setup: BE82 subscribe result = %s\n", subOk ? "true" : "false");
  if (!subOk) {
    Serial.println("BE80 setup: BE82 subscribe failed");
    cameraClient->disconnect();
    cameraMode = CAM_NONE;
    be80RetryAt = millis() + be80RetryDelay;
    return;
  }

  // Also subscribe to any other notify characteristics on the camera
  // (B002/B003/B004 on B000 service, AE02 on AE00 service).
  int extraSubCount = 0;
  for (const auto &svc : services) {
    const auto &chars = svc->getCharacteristics();
    for (const auto &chr : chars) {
      if (chr->canNotify() && chr != be82Notify) {
        Serial.printf("BE80 setup: also subscribing to %s on %s\n",
                      chr->getUUID().toString().c_str(),
                      svc->getUUID().toString().c_str());
        bool ok = chr->subscribe(true, onBe82Notify, true);
        Serial.printf("BE80 setup:   -> subscribe %s = %s\n",
                      chr->getUUID().toString().c_str(), ok ? "true" : "false");
        if (ok) extraSubCount++;
      }
    }
  }
  Serial.printf("BE80 setup: subscribed to BE82 + %d extra notify char(s)\n", extraSubCount);

  Serial.println("BE80 setup: ready! BE81 write + BE82 notify active");
  cameraMode = CAM_BE80_CLIENT;
  lastBe82NotifyAt = millis();  // start the watchdog clock
  be80RetryDelay = BE80_RETRY_BASE_MS;  // reset backoff on success
}

// Non-blocking state machine for BE80 client. Called from loop().
void maintainCameraClient() {
  // Watchdog: detect dead BLE link
  if (cameraMode == CAM_BE80_CLIENT && cameraClient && !cameraClient->isConnected()) {
    Serial.println("BE80 watchdog: cameraClient reports disconnected, forcing reconnect");
    cameraMode = CAM_NONE;
    be81Write = nullptr;
    be82Notify = nullptr;
    lastBe82NotifyAt = 0;
    be80RetryDelay = BE80_RETRY_BASE_MS;
    be80RetryAt = millis() + BE80_RETRY_BASE_MS;
  }

  // Handle pending BE80 connection from scan result
  if (be80ConnectPending) {
    be80ConnectPending = false;
    if (millis() < be80RetryAt) return;
    if (!cameraClient) {
      cameraClient = NimBLEDevice::createClient();
      cameraClient->setClientCallbacks(&cameraClientCallbacks, false);
    }
    cameraClient->setPeerAddress(be80PendingAddress);
    cameraMode = CAM_BE80_CONNECTING;
    Serial.println("BE80 client: initiating sync connect...");
    bool connResult = cameraClient->connect(true, false, true);
    Serial.printf("BE80 client: connect() returned %s (sync)\n", connResult ? "true" : "false");
    return;
  }

  switch (cameraMode) {
    case CAM_NONE:
      if (!be80ScanActive && millis() >= be80RetryAt)
        startBe80Scan();
      break;
    case CAM_BE80_CONNECTING:
      break;
    case CAM_BE80_SETTING_UP:
      setupBe80Client();
      break;
    case CAM_BE80_CLIENT:
      break;
  }
}

// void pollPrinter();  // WiFi disabled

// void setupWeb() {                    // WiFi disabled — entire function below
//   web.on("/", HTTP_GET, [] {
//     web.send_P(200, "text/html; charset=utf-8", LAYERSHOT_DASHBOARD);
//   });
//   web.on("/status", HTTP_GET, [] {
//     String ip = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
//     String body = "{\"ok\":true,\"name\":\"" + String(DEVICE_NAME) + "\",\"firmware\":\"" +
//       FIRMWARE_VERSION + "\",\"hostname\":\"" + deviceHostname + ".local\",\"ip\":\"" + ip +
//       "\",\"ssid\":\"" + jsonEscape(WiFi.SSID()) + "\",\"rssi\":" + String(WiFi.RSSI()) +
//       ",\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") +
//       ",\"camera_type\":\"" + jsonEscape(cameraType) + "\"" +
//       ",\"camera_name\":\"" + jsonEscape(cameraName()) + "\"" +
//       ",\"bluetooth\":" + String(cameraMode == CAM_BE80_CLIENT ? "true" : "false") +
//       ",\"scanning\":" + String(be80ScanActive ? "true" : "false") +
//       ",\"autonomous\":" + String(autonomousEnabled ? "true" : "false") +
//       ",\"printer\":\"" + jsonEscape(printerHost) + "\"" +
//       ",\"printer_port\":" + String(printerPort) +
//       ",\"printer_connected\":" + String(printerConnected ? "true" : "false") +
//       ",\"printer_state\":\"" + jsonEscape(printerState) + "\"" +
//       ",\"printer_http_code\":" + String(printerHttpCode) +
//       ",\"shutter_delay_ms\":" + String(stabilizationMs) +
//       ",\"current_layer\":" + String(lastPrinterLayer) +
//       ",\"total_layers\":" + String(printerTotalLayers) +
//       ",\"commands\":" + String(commandCount) +
//       ",\"last_command\":\"" + jsonEscape(lastCommand) + "\"" +
//       ",\"triggers\":" + String(triggerCount) + "}";
//     sendJSON(200, body);
//   });
//   web.on("/trigger", HTTP_POST, [] {
//     commandCount++;
//     if (triggerShutter()) {
//       lastCommand = "shutter_sent";
//       sendJSON(200, "{\"ok\":true,\"triggered\":true}");
//     } else {
//       lastCommand = "shutter_failed";
//       sendJSON(409, "{\"ok\":false,\"error\":\"camera_not_connected\"}");
//     }
//   });
//   web.on("/led-test", HTTP_POST, [] {
//     commandCount++; lastCommand = "led_test";
//     setLedOn(false); delay(225); setLedOn(true); delay(225);
//     setLedOn(false); delay(225); setLedOn(true); delay(225);
//     setLedOn(false); delay(225); setLedOn(true); delay(225);
//     setLedOn(false); delay(225); setLedOn(true); delay(225);
//     sendJSON(200, "{\"ok\":true,\"led\":true}");
//   });
//   web.on("/pair", HTTP_POST, [] {
//     commandCount++; lastCommand = "scan_restart";
//     restartScan();
//     sendJSON(200, "{\"ok\":true,\"scanning\":true}");
//   });
//   web.on("/reset-bonds", HTTP_POST, [] {
//     commandCount++; lastCommand = "bonds_erased";
//     clearBluetoothBonds();
//     sendJSON(200, "{\"ok\":true,\"bondsCleared\":true}");
//   });
//   web.on("/configure", HTTP_POST, [] {
//     String ssid = web.arg("ssid");
//     if (ssid.isEmpty()) {
//       sendJSON(400, "{\"ok\":false,\"error\":\"missing_ssid\"}");
//       return;
//     }
//     preferences.begin("layershot", false);
//     preferences.putString("ssid", ssid);
//     preferences.putString("password", web.arg("password"));
//     preferences.remove("static_ip");
//     preferences.remove("gateway");
//     preferences.remove("netmask");
//     preferences.remove("dns");
//     preferences.end();
//     sendJSON(200, "{\"ok\":true,\"restarting\":true}");
//     delay(500);
//     ESP.restart();
//   });
//   web.on("/camera-config", HTTP_POST, [] {
//     String target = web.arg("camera");
//     if (target != "insta360") {
//       sendJSON(400, "{\"ok\":false,\"error\":\"unsupported_camera\"}");
//       return;
//     }
//     cameraType = target;
//     preferences.begin("layershot", false);
//     preferences.putString("camera", cameraType);
//     preferences.end();
//     sendJSON(200, "{\"ok\":true,\"camera\":\"" + jsonEscape(cameraType) + "\"}");
//   });
//   web.on("/printer-config", HTTP_POST, [] {
//     String host = web.arg("host");
//     if (host.isEmpty()) { sendJSON(400, "{\"ok\":false,\"error\":\"missing_host\"}"); return; }
//     uint16_t newPort = (uint16_t)max(1L, web.arg("port").toInt());
//     uint16_t newEvery = (uint16_t)max(1L, web.arg("every").toInt());
//     uint16_t newSkip = (uint16_t)max(0L, web.arg("skip").toInt());
//     uint16_t newStop = (uint16_t)max(0L, web.arg("stop").toInt());
//     uint16_t newDelay = (uint16_t)max(0L, web.arg("delay").toInt());
//     preferences.begin("layershot", false);
//     preferences.putString("printer", host);
//     preferences.putUShort("port", newPort);
//     preferences.putUShort("every", newEvery);
//     preferences.putUShort("skip", newSkip);
//     preferences.putUShort("stop", newStop);
//     preferences.putUShort("delay", newDelay);
//     preferences.putBool("autonomous", true);
//     preferences.end();
//     printerHost = host;
//     printerPort = newPort;
//     captureEvery = newEvery;
//     skipLayers = newSkip;
//     stopAfterLayer = newStop;
//     stabilizationMs = newDelay;
//     autonomousEnabled = true;
//     lastPrinterLayer = -1;
//     sendJSON(200, "{\"ok\":true,\"autonomous\":true}");
//   });
//   web.on("/printer-test", HTTP_POST, [] {
//     lastPrinterPoll = 0;
//     pollPrinter();
//     if (printerConnected) sendJSON(200, "{\"ok\":true,\"printer\":true}");
//     else sendJSON(503, "{\"ok\":false,\"error\":\"printer_not_found\"}");
//   });
//   web.on("/autonomous-stop", HTTP_POST, [] {
//     autonomousEnabled = false;
//     preferences.begin("layershot", false); preferences.putBool("autonomous", false); preferences.end();
//     sendJSON(200, "{\"ok\":true,\"autonomous\":false}");
//   });
//   web.onNotFound([] { sendJSON(404, "{\"ok\":false,\"error\":\"not_found\"}"); });
//   web.begin();
// }

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

// WiFi disabled — pollPrinter removed
// void pollPrinter() {
//   if (!autonomousEnabled || printerHost.isEmpty() || WiFi.status() != WL_CONNECTED || millis() - lastPrinterPoll < 1000) return;
//   lastPrinterPoll = millis();
//   HTTPClient http;
//   String url = "http://" + printerHost + ":" + String(printerPort) + "/printer/objects/query?print_stats&virtual_sdcard&display_status";
//   http.setConnectTimeout(2500);
//   http.setTimeout(3500);
//   if (!http.begin(url)) return;
//   int code = http.GET();
//   printerHttpCode = code;
//   if (code == 200) {
//     String body = http.getString();
//     printerConnected = true;
//     bool printing = body.indexOf("\"state\":\"printing\"") >= 0 || body.indexOf("\"state\": \"printing\"") >= 0;
//     bool virtualSdActive = body.indexOf("\"is_active\":true") >= 0 || body.indexOf("\"is_active\": true") >= 0;
//     bool layerMonitoringActive = printing || virtualSdActive;
//     if (layerMonitoringActive) printerState = "printing";
//     else if (body.indexOf("\"state\":\"paused\"") >= 0 || body.indexOf("\"state\": \"paused\"") >= 0) printerState = "paused";
//     else if (body.indexOf("\"state\":\"complete\"") >= 0 || body.indexOf("\"state\": \"complete\"") >= 0) printerState = "complete";
//     else if (body.indexOf("\"state\":\"cancelled\"") >= 0 || body.indexOf("\"state\": \"cancelled\"") >= 0) printerState = "cancelled";
//     else if (body.indexOf("\"state\":\"standby\"") >= 0 || body.indexOf("\"state\": \"standby\"") >= 0) printerState = "standby";
//     else printerState = "ready";
//     int currentLayer = jsonIntegerAfter(body, "current_layer");
//     if (currentLayer < 0) currentLayer = jsonIntegerAfter(body, "layer");
//     int totalLayers = jsonIntegerAfter(body, "total_layer");
//     if (totalLayers < 0) totalLayers = jsonIntegerAfter(body, "layer_count");
//     if (totalLayers >= 0) printerTotalLayers = totalLayers;
//     if (layerMonitoringActive && currentLayer >= 0 && currentLayer != lastPrinterLayer) {
//       if (lastPrinterLayer >= 0 && currentLayer > lastPrinterLayer && currentLayer > skipLayers &&
//           (currentLayer - skipLayers) % max(1, (int)captureEvery) == 0 &&
//           (stopAfterLayer == 0 || currentLayer <= stopAfterLayer)) {
//         shutterPending = true;
//         shutterDueAt = millis() + stabilizationMs;
//       }
//       lastPrinterLayer = currentLayer;
//     } else if (!layerMonitoringActive) {
//       lastPrinterLayer = -1;
//     }
//   } else {
//     printerConnected = false;
//     printerState = "offline";
//   }
//   http.end();
// }

void updateScheduledShutter() {
  if (shutterPending && (int32_t)(millis() - shutterDueAt) >= 0) {
    shutterPending = false;
    triggerShutter();
  }
}

// WiFi disabled — applyPreferredNetwork removed
// void applyPreferredNetwork() {
//   if (preferredIp.isEmpty()) return;
//   IPAddress address, gateway, netmask, dns;
//   if (address.fromString(preferredIp) &&
//       gateway.fromString(preferredGateway) &&
//       netmask.fromString(preferredNetmask) &&
//       dns.fromString(preferredDns)) {
//     WiFi.config(address, gateway, netmask, dns);
//   }
// }

// WiFi disabled — startWiFiServices removed
// void startWiFiServices() {
//   if (!otaReady) {
//     MDNS.begin(deviceHostname.c_str());
//     MDNS.addService("http", "tcp", 80);
//     ArduinoOTA.setHostname(deviceHostname.c_str());
//     ArduinoOTA.setPassword("layershot");
//     ArduinoOTA.begin();
//     otaReady = true;
//   }
//   WiFi.softAPdisconnect(true);
//   wifiError = false;
// }

// WiFi disabled — connectWiFi removed
// void connectWiFi() {
//   preferences.begin("layershot", true);
//   wifiSsid = preferences.getString("ssid", "");
//   wifiPassword = preferences.getString("password", "");
//   preferredIp = preferences.getString("static_ip", "");
//   preferredGateway = preferences.getString("gateway", "");
//   preferredNetmask = preferences.getString("netmask", "");
//   preferredDns = preferences.getString("dns", "");
//   preferences.end();
//
//   WiFi.setHostname(deviceHostname.c_str());
//   WiFi.setAutoReconnect(true);
//   WiFi.persistent(false);
//   if (!wifiSsid.isEmpty()) {
//     wifiConnecting = true;
//     WiFi.mode(WIFI_STA);
//     applyPreferredNetwork();
//     WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
//     uint32_t started = millis();
//     while (WiFi.status() != WL_CONNECTED && millis() - started < 18000) {
//       handleSerialProvisioning();
//       setLedOn(true);
//       delay(150);
//       setLedOn(false);
//       delay(150);
//     }
//     wifiConnecting = false;
//   }
//   if (WiFi.status() != WL_CONNECTED) {
//     wifiError = !wifiSsid.isEmpty();
//     WiFi.mode(WIFI_AP_STA);
//     WiFi.softAP(SETUP_AP);
//   } else {
//     startWiFiServices();
//   }
// }

// WiFi disabled — maintainWiFi removed
// void maintainWiFi() {
//   if (WiFi.status() == WL_CONNECTED) {
//     if (!otaReady) startWiFiServices();
//     return;
//   }
//   if (wifiSsid.isEmpty() || millis() - lastWiFiReconnectAttempt < 10000) return;
//   lastWiFiReconnectAttempt = millis();
//   wifiConnecting = true;
//   WiFi.mode(WIFI_AP_STA);
//   applyPreferredNetwork();
//   WiFi.disconnect(false, false);
//   WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
//   wifiConnecting = false;
//   wifiError = true;
// }

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
    else if (duration >= 3000) { Serial.println("BOOT: restart scan"); restartScan(); }
    else if (duration >= 50) {
      Serial.println("BOOT: short press -> triggerShutter()");
      bool ok = triggerShutter();
      Serial.printf("triggerShutter() returned %s\n", ok ? "true" : "false");
    }
  }
}

void updateLED() {
  uint32_t now = millis();

  // Shutter flash: brief on pulse
  if (shutterFlashUntil > now) {
    setLedOn(true);
    return;
  }

  // Connected: steady on
  if (cameraMode == CAM_BE80_CLIENT) {
    if (!ledOn) setLedOn(true);
    return;
  }

  // Connecting: fast blink
  if (cameraMode == CAM_BE80_CONNECTING || cameraMode == CAM_BE80_SETTING_UP) {
    if (now - ledLastToggleAt > LED_PAIR_BLINK_MS) {
      ledLastToggleAt = now;
      setLedOn(!ledOn);
    }
    return;
  }

  // Not connected: slow blink
  if (now - ledLastToggleAt > LED_IDLE_BLINK_MS) {
    ledLastToggleAt = now;
    setLedOn(!ledOn);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PAIR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  setLedOn(true);

  setupInsta360Bluetooth();

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
  // WiFi disabled — Bluetooth shutter only mode
  // connectWiFi();
  // setupWeb();

  Serial.printf("%s %s\n", BLE_NAME, FIRMWARE_VERSION);
}

void loop() {
  handleSerialProvisioning();
  // WiFi disabled — no web/WiFi/OTA/printer calls
  // maintainWiFi();
  // web.handleClient();
  // if (otaReady) ArduinoOTA.handle();
  updateButton();
  updateLED();
  maintainCameraClient();
  // pollPrinter();
  updateScheduledShutter();
  delay(5);
}
