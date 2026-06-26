#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"
#include <Adafruit_XCA9554.h>
#include <Adafruit_MAX31865.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <Update.h>
#include <XPowersLib.h>

#ifndef AXP2101_SLAVE_ADDRESS
#define AXP2101_SLAVE_ADDRESS 0x34
#endif

HWCDC USBSerial;
Preferences prefs;

// =====================================================
// WATER WATCH PRO - V10.8 HOST OTA UPDATE MODE + POWER BUTTON SCREEN OFF
// Based on V10.1 partial shape calibration / no flicker build.
// Keeps existing calibration storage keys untouched so tank calibration is preserved.
// Keeps V10.3 service bar / set hours build and adds ESP-NOW remote display sender broadcast.
// Fixes AUTO mode: IO18 AUTO is now controlled ONLY by temperature probe/setpoint.
// LOW ALERT is visual warning only and no longer blocks IO18.
// =====================================================

// =====================================================
// PINS
// =====================================================

#define PUMP_GPIO 18
#define PUMP_ACTIVE_HIGH true

#define AUX_GPIO 42
#define AUX_ACTIVE_HIGH true

#define PRESSURE_GPIO 17

#define MAX31865_CLK  39
#define MAX31865_MOSI 40
#define MAX31865_MISO 41
#define MAX31865_CS   38

#define RREF      430.0
#define RNOMINAL 100.0

// Pressure smoothing
#define PRESSURE_ALPHA 0.35          // faster response; lower = smoother
#define DASHBOARD_PERCENT_REDRAW 0.5 // update dashboard faster without full-screen blink
#define DASHBOARD_VOLT_REDRAW 0.02   // voltage redraw threshold

// =====================================================
// HARDWARE OBJECTS
// =====================================================

Adafruit_MAX31865 max31865 = Adafruit_MAX31865(
  MAX31865_CS,
  MAX31865_MOSI,
  MAX31865_MISO,
  MAX31865_CLK
);

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS,
  LCD_SCLK,
  LCD_SDIO0,
  LCD_SDIO1,
  LCD_SDIO2,
  LCD_SDIO3
);

Arduino_SH8601 *gfx = new Arduino_SH8601(
  bus,
  GFX_NOT_DEFINED,
  0,
  LCD_WIDTH,
  LCD_HEIGHT
);

Adafruit_XCA9554 expander;

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);

std::unique_ptr<Arduino_IIC> FT3168(
  new Arduino_FT3x68(
    IIC_Bus,
    FT3168_DEVICE_ADDRESS,
    DRIVEBUS_DEFAULT_VALUE,
    TP_INT,
    Arduino_IIC_Touch_Interrupt
  )
);

void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
}

// =====================================================
// COLORS
// =====================================================

#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_DIM     0x4208
#define C_GRAY    0x8410
#define C_CARD    0x18E3
#define C_CARD2   0x2124
#define C_CYAN    0x07FF
#define C_BLUE    0x045F
#define C_GREEN   0x07E0
#define C_ORANGE  0xFD20
#define C_RED     0xF800
#define C_YELLOW  0xFFE0
#define C_PURPLE  0xA01F

#define SCREEN_W LCD_WIDTH
#define SCREEN_H LCD_HEIGHT

// =====================================================
// APP STATE
// =====================================================

enum ScreenMode {
  SCREEN_DASHBOARD,
  SCREEN_SETUP,
  SCREEN_CALIBRATE,
  SCREEN_TANK_SIZE,
  SCREEN_LOW_ALERT,
  SCREEN_TEMP_RANGE,
  SCREEN_TEMP_LIVE,
  SCREEN_PRESSURE_LIVE,
  SCREEN_TOTAL_HOURS,
  SCREEN_MAINTENANCE,
  SCREEN_UPDATE
};

enum RelayMode {
  RELAY_AUTO,
  RELAY_ON,
  RELAY_OFF
};

ScreenMode screen = SCREEN_DASHBOARD;
RelayMode relayMode = RELAY_AUTO;
bool auxOn = false;

// Host display screen control.
// Sensors, relay/AUX outputs, hour meter, and ESP-NOW keep running while screen is off.
// V10.7: bottom AXP2101 power button toggles screen off/on. No auto sleep timer.
XPowersPMU PMU;
bool hostPmuReady = false;
bool hostScreenAwake = true;
bool hostPowerKeyDown = false;
unsigned long hostPowerKeyDownMs = 0;
const unsigned long HOST_POWER_KEY_TAP_MAX_MS = 1200UL;
const uint8_t HOST_AWAKE_BRIGHTNESS = 255;
const uint8_t HOST_SCREEN_OFF_BRIGHTNESS = 0;

// Host OTA update mode.
// The host creates its own WiFi hotspot only when the UPDATE screen START button is pressed.
// Normal Water Watch operation uses ESP-NOW and does not keep OTA WiFi open.
WebServer otaServer(80);
bool otaModeActive = false;
bool otaUploadInProgress = false;
bool otaUpdateEnded = false;
bool otaUpdateOk = false;
unsigned long otaModeStartMs = 0;
unsigned long otaLastScreenDrawMs = 0;
size_t otaBytesWritten = 0;
const unsigned long OTA_MODE_TIMEOUT_MS = 600000UL;  // 10 minutes
const char *OTA_AP_SSID = "WaterWatchPro-Host";
const char *OTA_AP_PASS = "12345678";

float tankPercent = 0.0;
float tankGallons = 1.0;
float gallonsUsed = 0.0;
float gallonsLeft = 0.0;

float pressureVoltage = 0.0;
float pressureRawVoltage = 0.0;
float pressureFilteredVoltage = 0.0;
int pressureRaw = 0;
bool pressureFilterReady = false;
bool pressureSensorOK = false;

float emptyVoltage = 0.36;
float fullVoltage = 3.00;
bool emptySaved = false;
bool fullSaved = false;

// Tank shape calibration points.
// 0/25/50/75/100 are volume percent points, not height percent points.
// This makes gallons more accurate on curved/odd-shaped tanks.
float shapeCalV[5] = {0.36, 1.00, 1.65, 2.30, 3.00};
bool shapeCalSaved[5] = {false, false, false, false, false};
const float shapeCalPct[5] = {0, 25, 50, 75, 100};

float waterTempF = 0.0;
float tempLowF = 125.0;
float tempHighF = 135.0;
bool tempSensorOK = false;

float lowAlertPercent = 25.0;

bool lastPumpState = false;

// V10.5B: AUTO transition guard.
// When switching into AUTO, hold the current physical IO18 state briefly.
// This prevents a one-time on/off/on or off/on/off pulse that can fault pump controllers.
unsigned long autoModeSettleUntilMs = 0;
const unsigned long AUTO_MODE_SETTLE_MS = 1500UL;

unsigned long lastTouchMs = 0;
unsigned long lastReadMs = 0;

float lastDrawnTempF = -9999;
float lastDrawnTankPercent = -9999;
float lastDrawnPressureVoltage = -9999;
bool lastDrawnTempOK = false;
bool lastDrawnPressureOK = false;
bool lastDrawnPumpState = false;
RelayMode lastDrawnRelayMode = RELAY_AUTO;
ScreenMode lastDrawnScreen = SCREEN_DASHBOARD;

float lastLiveScreenVoltage = -9999;
float lastLiveScreenTemp = -9999;
float lastLiveScreenPercent = -9999;
unsigned long lastLiveScreenDrawMs = 0;

// Hour meter + maintenance tracking.
// Unit hours count whenever this display/controller is powered.
float unitHours = 0.0;
float lastDrawnUnitHours = -9999;
bool lastDrawnServiceDue = false;
unsigned long lastHourTickMs = 0;
unsigned long lastHourSaveMs = 0;

const int MAINT_COUNT = 4;
const char* maintNames[MAINT_COUNT] = {
  "ENGINE OIL",
  "PUMP OIL",
  "VAC OIL",
  "GENERAL"
};

float maintIntervals[MAINT_COUNT] = {100.0, 500.0, 1000.0, 250.0};
float maintLastResetHours[MAINT_COUNT] = {0.0, 0.0, 0.0, 0.0};
int selectedMaintItem = 0;
int lastDrawnDueBannerIndex = -1;
bool lastDrawnServiceFlashPhase = false;

const int MAINT_INTERVAL_COUNT = 7;
const float maintIntervalChoices[MAINT_INTERVAL_COUNT] = {0.0, 25.0, 50.0, 100.0, 250.0, 500.0, 1000.0};

// Swipe tracking. Tap is delayed until finger release/timeout so swipes do not trigger buttons.
bool touchDown = false;
bool swipeHandled = false;
int touchStartX = 0;
int touchStartY = 0;
int touchLastX = 0;
int touchLastY = 0;
unsigned long touchStartMs = 0;
unsigned long touchLastSeenMs = 0;

// ESP-NOW remote display sender.
// This main controller broadcasts live read-only dashboard data once per second.
// The remote display will have its own separate receiver firmware.
#define REMOTE_SEND_ENABLED true
#define REMOTE_SEND_INTERVAL_MS 1000UL

uint8_t remoteBroadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool remoteSenderReady = false;
unsigned long lastRemoteSendMs = 0;
uint32_t remotePacketSeq = 0;

struct WaterWatchRemotePacket {
  uint16_t magic;        // 0x57A7 = Water Watch packet marker
  uint8_t version;       // packet version
  uint32_t seq;
  uint32_t uptimeMs;

  float tankPercent;
  float gallonsLeft;
  float gallonsUsed;
  float tankGallons;

  float tempF;
  float pressureV;
  float pressureRawV;
  int pressureRaw;

  float totalHours;
  uint8_t relayMode;
  uint8_t pumpOn;
  uint8_t auxOn;
  uint8_t serviceDue;
  int8_t dueItem;

  uint8_t tempOK;
  uint8_t pressureOK;
  uint8_t calibrated;
  uint8_t reserved;
};
// =====================================================
// TWO-WAY ESP-NOW COMMAND PACKETS - V10.5 TEST
// Main unit remains boss. Remote/watch only requests commands.
// Main applies command, then broadcasts an ACK.
// Existing 1.8" read-only remote ignores these ACK packets because size/magic differ.
// =====================================================

#define WWP_COMMAND_MAGIC 0x57C0
#define WWP_ACK_MAGIC     0x57C1
#define WWP_COMMAND_VERSION 1
#define WWP_ACK_VERSION     1

enum WaterWatchCommandType : uint8_t {
  CMD_NONE = 0,
  CMD_AUX_TOGGLE = 1,
  CMD_RELAY_MODE_CYCLE = 2
};

struct WaterWatchCommandPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t command;
  uint32_t commandId;
  uint32_t senderMs;
};

struct WaterWatchAckPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t command;
  uint8_t accepted;
  uint32_t commandId;
  uint32_t ackMs;
  uint8_t auxOn;
  uint8_t relayMode;
  uint8_t pumpOn;
  uint8_t code;
};

uint32_t lastRemoteCommandId = 0;
uint32_t lastAckCommandId = 0;

// Forward declarations used by command handler.
bool isPumpActive();
void updateAuxOutput();
void updatePumpOutput();
void saveSettings();
void cycleRelayMode();

void sendCommandAck(uint8_t command, uint32_t commandId, bool accepted, uint8_t code) {
#if REMOTE_SEND_ENABLED
  if (!remoteSenderReady) return;

  WaterWatchAckPacket ack = {};
  ack.magic = WWP_ACK_MAGIC;
  ack.version = WWP_ACK_VERSION;
  ack.command = command;
  ack.accepted = accepted ? 1 : 0;
  ack.commandId = commandId;
  ack.ackMs = millis();
  ack.auxOn = auxOn ? 1 : 0;
  ack.relayMode = (uint8_t)relayMode;
  ack.pumpOn = lastPumpState ? 1 : 0;
  ack.code = code;

  esp_err_t result = esp_now_send(remoteBroadcastAddress, (uint8_t *)&ack, sizeof(ack));
  if (result != ESP_OK) {
    USBSerial.print("ESP-NOW ACK send failed: ");
    USBSerial.println((int)result);
  } else {
    lastAckCommandId = commandId;
  }
#endif
}

void handleRemoteCommandSimple(uint8_t command, uint32_t commandId) {
  // Duplicate protection: if the same command is received again, ACK but do not re-apply.
  if (commandId == lastRemoteCommandId) {
    sendCommandAck(command, commandId, true, 2);
    return;
  }

  bool accepted = false;
  uint8_t code = 0;

  if (command == CMD_AUX_TOGGLE) {
    auxOn = !auxOn;
    saveSettings();
    updateAuxOutput();
    accepted = true;
    code = 1;
  } else if (command == CMD_RELAY_MODE_CYCLE) {
    cycleRelayMode();
    updatePumpOutput();
    accepted = true;
    code = 1;
  } else {
    accepted = false;
    code = 9;
  }

  if (accepted) {
    lastRemoteCommandId = commandId;

    if (screen == SCREEN_DASHBOARD) {
      drawDashboard();
    }
  }

  sendCommandAck(command, commandId, accepted, code);

  USBSerial.print("Remote command ");
  USBSerial.print(command);
  USBSerial.print(" id ");
  USBSerial.print(commandId);
  USBSerial.print(" accepted=");
  USBSerial.println(accepted ? "YES" : "NO");
}

void onEspNowCommandReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(WaterWatchCommandPacket)) {
    return;
  }

  WaterWatchCommandPacket cmd;
  memcpy(&cmd, incomingData, sizeof(cmd));

  if (cmd.magic != WWP_COMMAND_MAGIC || cmd.version != WWP_COMMAND_VERSION) {
    return;
  }

  handleRemoteCommandSimple(cmd.command, cmd.commandId);
}


// =====================================================
// ESP-NOW REMOTE DISPLAY SENDER
// =====================================================

void initRemoteSender() {
#if REMOTE_SEND_ENABLED
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    USBSerial.println("ESP-NOW init failed - remote sender disabled");
    remoteSenderReady = false;
    return;
  }

  // V10.5: listen for remote/watch command requests.
  esp_now_register_recv_cb(onEspNowCommandReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, remoteBroadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(remoteBroadcastAddress)) {
    esp_err_t addStatus = esp_now_add_peer(&peerInfo);
    if (addStatus != ESP_OK) {
      USBSerial.print("ESP-NOW broadcast peer add failed: ");
      USBSerial.println((int)addStatus);
      remoteSenderReady = false;
      return;
    }
  }

  remoteSenderReady = true;
  USBSerial.print("ESP-NOW remote sender ready. Main MAC: ");
  USBSerial.println(WiFi.macAddress());
#else
  remoteSenderReady = false;
#endif
}

void sendRemotePacket() {
#if REMOTE_SEND_ENABLED
  if (!remoteSenderReady) return;

  WaterWatchRemotePacket pkt = {};
  pkt.magic = 0x57A7;
  pkt.version = 1;
  pkt.seq = ++remotePacketSeq;
  pkt.uptimeMs = millis();

  pkt.tankPercent = tankPercent;
  pkt.gallonsLeft = gallonsLeft;
  pkt.gallonsUsed = gallonsUsed;
  pkt.tankGallons = tankGallons;

  pkt.tempF = waterTempF;
  pkt.pressureV = pressureVoltage;
  pkt.pressureRawV = pressureRawVoltage;
  pkt.pressureRaw = pressureRaw;

  pkt.totalHours = unitHours;
  pkt.relayMode = (uint8_t)relayMode;
  pkt.pumpOn = lastPumpState ? 1 : 0;
  pkt.auxOn = auxOn ? 1 : 0;
  pkt.serviceDue = serviceDue() ? 1 : 0;
  pkt.dueItem = (int8_t)rotatingDueIndex();

  pkt.tempOK = tempSensorOK ? 1 : 0;
  pkt.pressureOK = pressureSensorOK ? 1 : 0;
  pkt.calibrated = pressureCalibrated() ? 1 : 0;
  pkt.reserved = 0;

  esp_err_t result = esp_now_send(remoteBroadcastAddress, (uint8_t *)&pkt, sizeof(pkt));
  if (result != ESP_OK) {
    USBSerial.print("ESP-NOW send failed: ");
    USBSerial.println((int)result);
  }
#endif
}

// =====================================================
// SETTINGS SAVE / LOAD
// =====================================================

void loadSettings() {
  prefs.begin("wwp", false);

  tankGallons = prefs.getFloat("tankGal", 1.0);
  lowAlertPercent = prefs.getFloat("lowAlert", 25.0);
  tempLowF = prefs.getFloat("tempLow", 125.0);
  tempHighF = prefs.getFloat("tempHigh", 135.0);

  emptyVoltage = prefs.getFloat("emptyV", 0.36);
  fullVoltage = prefs.getFloat("fullV", 3.00);
  emptySaved = prefs.getBool("emptySaved", false);
  fullSaved = prefs.getBool("fullSaved", false);

  // New V9.8 shape calibration.
  // If old EMPTY/FULL calibration exists, it will still be used.
  shapeCalV[0] = prefs.getFloat("cal0V", emptyVoltage);
  shapeCalV[1] = prefs.getFloat("cal25V", 1.00);
  shapeCalV[2] = prefs.getFloat("cal50V", 1.65);
  shapeCalV[3] = prefs.getFloat("cal75V", 2.30);
  shapeCalV[4] = prefs.getFloat("cal100V", fullVoltage);

  shapeCalSaved[0] = prefs.getBool("cal0Saved", emptySaved);
  shapeCalSaved[1] = prefs.getBool("cal25Saved", false);
  shapeCalSaved[2] = prefs.getBool("cal50Saved", false);
  shapeCalSaved[3] = prefs.getBool("cal75Saved", false);
  shapeCalSaved[4] = prefs.getBool("cal100Saved", fullSaved);

  relayMode = (RelayMode)prefs.getUChar("relayMode", RELAY_AUTO);
  auxOn = prefs.getBool("auxOn", false);

  // V10.2 hour meter / maintenance keys. These are new keys and do not touch calibration.
  unitHours = prefs.getFloat("unitH", 0.0);
  maintIntervals[0] = prefs.getFloat("mInt0", 100.0);
  maintIntervals[1] = prefs.getFloat("mInt1", 500.0);
  maintIntervals[2] = prefs.getFloat("mInt2", 1000.0);
  maintIntervals[3] = prefs.getFloat("mInt3", 250.0);
  maintLastResetHours[0] = prefs.getFloat("mLast0", 0.0);
  maintLastResetHours[1] = prefs.getFloat("mLast1", 0.0);
  maintLastResetHours[2] = prefs.getFloat("mLast2", 0.0);
  maintLastResetHours[3] = prefs.getFloat("mLast3", 0.0);

  if (tankGallons < 1 || tankGallons > 500) tankGallons = 1.0;
  if (lowAlertPercent < 5 || lowAlertPercent > 80) lowAlertPercent = 25.0;
  if (tempLowF < 40 || tempLowF > 250) tempLowF = 125.0;
  if (tempHighF < 45 || tempHighF > 300) tempHighF = 135.0;
  if (tempHighF <= tempLowF) tempHighF = tempLowF + 10;
  if (unitHours < 0 || unitHours > 99999) unitHours = 0.0;
  for (int i = 0; i < MAINT_COUNT; i++) {
    if (maintIntervals[i] < 0 || maintIntervals[i] > 10000) maintIntervals[i] = 0.0;
    if (maintLastResetHours[i] < 0 || maintLastResetHours[i] > 99999) maintLastResetHours[i] = 0.0;
  }

  if (emptyVoltage < 0 || emptyVoltage > 3.3) emptyVoltage = 0.36;
  if (fullVoltage < 0 || fullVoltage > 3.3) fullVoltage = 3.00;

  for (int i = 0; i < 5; i++) {
    if (shapeCalV[i] < 0 || shapeCalV[i] > 3.3) {
      shapeCalV[i] = 0.36 + (i * 0.66);
      shapeCalSaved[i] = false;
    }
  }
}

void saveSettings() {
  prefs.putFloat("tankGal", tankGallons);
  prefs.putFloat("lowAlert", lowAlertPercent);
  prefs.putFloat("tempLow", tempLowF);
  prefs.putFloat("tempHigh", tempHighF);

  prefs.putFloat("emptyV", emptyVoltage);
  prefs.putFloat("fullV", fullVoltage);
  prefs.putBool("emptySaved", emptySaved);
  prefs.putBool("fullSaved", fullSaved);

  prefs.putFloat("cal0V", shapeCalV[0]);
  prefs.putFloat("cal25V", shapeCalV[1]);
  prefs.putFloat("cal50V", shapeCalV[2]);
  prefs.putFloat("cal75V", shapeCalV[3]);
  prefs.putFloat("cal100V", shapeCalV[4]);

  prefs.putBool("cal0Saved", shapeCalSaved[0]);
  prefs.putBool("cal25Saved", shapeCalSaved[1]);
  prefs.putBool("cal50Saved", shapeCalSaved[2]);
  prefs.putBool("cal75Saved", shapeCalSaved[3]);
  prefs.putBool("cal100Saved", shapeCalSaved[4]);

  prefs.putUChar("relayMode", (uint8_t)relayMode);
  prefs.putBool("auxOn", auxOn);

  prefs.putFloat("unitH", unitHours);
  prefs.putFloat("mInt0", maintIntervals[0]);
  prefs.putFloat("mInt1", maintIntervals[1]);
  prefs.putFloat("mInt2", maintIntervals[2]);
  prefs.putFloat("mInt3", maintIntervals[3]);
  prefs.putFloat("mLast0", maintLastResetHours[0]);
  prefs.putFloat("mLast1", maintLastResetHours[1]);
  prefs.putFloat("mLast2", maintLastResetHours[2]);
  prefs.putFloat("mLast3", maintLastResetHours[3]);
}

void saveHourMeterOnly() {
  prefs.putFloat("unitH", unitHours);
}

void saveMaintenanceOnly() {
  prefs.putFloat("unitH", unitHours);
  prefs.putFloat("mInt0", maintIntervals[0]);
  prefs.putFloat("mInt1", maintIntervals[1]);
  prefs.putFloat("mInt2", maintIntervals[2]);
  prefs.putFloat("mInt3", maintIntervals[3]);
  prefs.putFloat("mLast0", maintLastResetHours[0]);
  prefs.putFloat("mLast1", maintLastResetHours[1]);
  prefs.putFloat("mLast2", maintLastResetHours[2]);
  prefs.putFloat("mLast3", maintLastResetHours[3]);
}

// =====================================================
// HELPERS
// =====================================================

bool hitBox(int x, int y, int l, int t, int r, int b) {
  return x >= l && x <= r && y >= t && y <= b;
}

float clampFloat(float v, float low, float high) {
  if (v < low) return low;
  if (v > high) return high;
  return v;
}

const char* relayModeLabel() {
  if (relayMode == RELAY_AUTO) return "AUTO";
  if (relayMode == RELAY_ON) return "ON";
  return "OFF";
}

void cycleRelayMode() {
  RelayMode oldMode = relayMode;

  if (relayMode == RELAY_AUTO) relayMode = RELAY_ON;
  else if (relayMode == RELAY_ON) relayMode = RELAY_OFF;
  else relayMode = RELAY_AUTO;

  // V10.5B: If entering AUTO, do not let IO18 chatter during the transition.
  // Keep the current physical output state for a short settle window, then apply final AUTO state once.
  if (oldMode != RELAY_AUTO && relayMode == RELAY_AUTO) {
    autoModeSettleUntilMs = millis() + AUTO_MODE_SETTLE_MS;
    USBSerial.println("AUTO mode settle guard active - holding IO18 state briefly");
  } else {
    autoModeSettleUntilMs = 0;
  }

  saveSettings();
}

void updateHourMeter() {
  unsigned long now = millis();

  if (lastHourTickMs == 0) {
    lastHourTickMs = now;
    lastHourSaveMs = now;
    return;
  }

  unsigned long elapsedMs = now - lastHourTickMs;
  lastHourTickMs = now;

  unitHours += ((float)elapsedMs / 3600000.0);

  // Save only once per minute to reduce flash wear.
  if (now - lastHourSaveMs >= 60000UL) {
    saveHourMeterOnly();
    lastHourSaveMs = now;
  }
}

float maintenanceUsedHours(int idx) {
  if (idx < 0 || idx >= MAINT_COUNT) return 0.0;
  float used = unitHours - maintLastResetHours[idx];
  if (used < 0) used = 0.0;
  return used;
}

bool maintenanceItemDue(int idx) {
  if (idx < 0 || idx >= MAINT_COUNT) return false;
  if (maintIntervals[idx] <= 0.0) return false;
  return maintenanceUsedHours(idx) >= maintIntervals[idx];
}

bool serviceDue() {
  for (int i = 0; i < MAINT_COUNT; i++) {
    if (maintenanceItemDue(i)) return true;
  }
  return false;
}

int serviceDueCount() {
  int count = 0;
  for (int i = 0; i < MAINT_COUNT; i++) {
    if (maintenanceItemDue(i)) count++;
  }
  return count;
}

int rotatingDueIndex() {
  int count = serviceDueCount();
  if (count <= 0) return -1;

  int slot = (millis() / 2000UL) % count;
  int seen = 0;

  for (int i = 0; i < MAINT_COUNT; i++) {
    if (!maintenanceItemDue(i)) continue;
    if (seen == slot) return i;
    seen++;
  }

  return -1;
}

bool serviceFlashPhase() {
  if (!serviceDue()) return false;
  return ((millis() / 600UL) % 2) == 0;
}

const char* maintShortName(int idx) {
  if (idx == 0) return "ENGINE";
  if (idx == 1) return "PUMP";
  if (idx == 2) return "VAC";
  if (idx == 3) return "GENERAL";
  return "SERVICE";
}

void serviceStatusText(char *out, size_t n) {
  if (!serviceDue()) {
    snprintf(out, n, "SERVICE OK");
    return;
  }

  int idx = rotatingDueIndex();
  snprintf(out, n, "DUE: %s", maintShortName(idx));
}

int currentIntervalChoiceIndex(float value) {
  for (int i = 0; i < MAINT_INTERVAL_COUNT; i++) {
    if (fabs(value - maintIntervalChoices[i]) < 0.1) return i;
  }
  return 0;
}

void cycleSelectedMaintenanceInterval() {
  int idx = currentIntervalChoiceIndex(maintIntervals[selectedMaintItem]);
  idx++;
  if (idx >= MAINT_INTERVAL_COUNT) idx = 0;
  maintIntervals[selectedMaintItem] = maintIntervalChoices[idx];
  saveMaintenanceOnly();
}

void resetSelectedMaintenanceItem() {
  maintLastResetHours[selectedMaintItem] = unitHours;
  saveMaintenanceOnly();
}


bool pressureCalibrated() {
  // Minimum required calibration: 0% and 100%.
  return shapeCalSaved[0] && shapeCalSaved[4] && fabs(shapeCalV[4] - shapeCalV[0]) >= 0.05;
}

int savedShapePointCount() {
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (shapeCalSaved[i]) count++;
  }
  return count;
}

bool shapeCalibrated() {
  // Shape calibration is active when endpoints exist and at least one midpoint exists.
  // It no longer requires all 0/25/50/75/100 points.
  if (!pressureCalibrated()) return false;
  return shapeCalSaved[1] || shapeCalSaved[2] || shapeCalSaved[3];
}

void clearMiddleShapeCalibration() {
  shapeCalSaved[1] = false;
  shapeCalSaved[2] = false;
  shapeCalSaved[3] = false;

  prefs.putBool("cal25Saved", false);
  prefs.putBool("cal50Saved", false);
  prefs.putBool("cal75Saved", false);
}

float pressurePercentFromCalibration(float v) {
  // Minimum setup is 0% and 100%.
  if (!pressureCalibrated()) {
    return ((v - 0.36) / (3.00 - 0.36)) * 100.0;
  }

  bool rising = shapeCalV[4] > shapeCalV[0];

  // Clamp outside endpoint range. Works for normal and reverse sensors.
  if (rising) {
    if (v <= shapeCalV[0]) return 0.0;
    if (v >= shapeCalV[4]) return 100.0;
  } else {
    if (v >= shapeCalV[0]) return 0.0;
    if (v <= shapeCalV[4]) return 100.0;
  }

  // Use whatever points are saved. This is the V10.1 fix.
  // Example: if 0/25/50/100 are saved and 75 is not saved, it uses:
  // 0->25, 25->50, and 50->100.
  int lastIdx = 0;
  for (int i = 1; i < 5; i++) {
    if (!shapeCalSaved[i]) continue;

    float v0 = shapeCalV[lastIdx];
    float v1 = shapeCalV[i];
    float p0 = shapeCalPct[lastIdx];
    float p1 = shapeCalPct[i];

    if (fabs(v1 - v0) < 0.001) {
      lastIdx = i;
      continue;
    }

    float lowV = min(v0, v1);
    float highV = max(v0, v1);

    if (v >= lowV && v <= highV) {
      float ratio = (v - v0) / (v1 - v0);
      return p0 + ratio * (p1 - p0);
    }

    lastIdx = i;
  }

  // Safety fallback: endpoint-only math.
  if (rising) {
    return ((v - shapeCalV[0]) / (shapeCalV[4] - shapeCalV[0])) * 100.0;
  } else {
    return ((shapeCalV[0] - v) / (shapeCalV[0] - shapeCalV[4])) * 100.0;
  }
}

bool isWaterSafe() {
  if (!pressureSensorOK) return false;
  if (!pressureCalibrated()) return false;
  return tankPercent > lowAlertPercent;
}

bool isTempInRange() {
  if (!tempSensorOK) return false;
  return waterTempF >= tempLowF && waterTempF <= tempHighF;
}

bool isPumpActive() {
  // OFF always forces IO18 off.
  if (relayMode == RELAY_OFF) return false;

  // Manual ON is a true override for testing/wiring.
  // It turns IO18 on even if water is low, pressure is unplugged, or temp is unplugged.
  if (relayMode == RELAY_ON) return true;

  // V10.2 FIX:
  // AUTO is controlled ONLY by the temperature probe/range.
  // LOW ALERT / water level / pressure calibration are visual warning only and do NOT block IO18.
  if (!tempSensorOK) return false;

  return isTempInRange();
}

void updateAuxOutput() {
  if (AUX_ACTIVE_HIGH) {
    digitalWrite(AUX_GPIO, auxOn ? HIGH : LOW);
  } else {
    digitalWrite(AUX_GPIO, auxOn ? LOW : HIGH);
  }

  USBSerial.print("AUX IO42: ");
  USBSerial.println(auxOn ? "ON" : "OFF");
}

void updatePumpOutput() {
  bool desiredActive = isPumpActive();
  bool activeToWrite = desiredActive;

  // V10.5B: AUTO settle guard.
  // If we just switched into AUTO, hold the current physical output state briefly.
  // This avoids the one-time pulse/drop that can make a connected pump controller fault.
  if (relayMode == RELAY_AUTO && autoModeSettleUntilMs != 0 && millis() < autoModeSettleUntilMs) {
    activeToWrite = lastPumpState;
  } else if (autoModeSettleUntilMs != 0 && millis() >= autoModeSettleUntilMs) {
    autoModeSettleUntilMs = 0;
    activeToWrite = desiredActive;
  }

  if (PUMP_ACTIVE_HIGH) {
    digitalWrite(PUMP_GPIO, activeToWrite ? HIGH : LOW);
  } else {
    digitalWrite(PUMP_GPIO, activeToWrite ? LOW : HIGH);
  }

  lastPumpState = activeToWrite;

  USBSerial.print("Relay Mode: ");
  USBSerial.print(relayModeLabel());
  USBSerial.print(" | Desired IO18: ");
  USBSerial.print(desiredActive ? "ON" : "OFF");
  USBSerial.print(" | Actual IO18: ");
  USBSerial.println(activeToWrite ? "ON" : "OFF");
}

void centerText(const char *text, int y, int size, uint16_t color, uint16_t bg = C_BLACK) {
  gfx->setTextSize(size);
  gfx->setTextColor(color, bg);

  int16_t x1, y1;
  uint16_t w, h;
  gfx->getTextBounds(text, 0, y, &x1, &y1, &w, &h);

  int x = (SCREEN_W - w) / 2;
  if (x < 0) x = 0;

  gfx->setCursor(x, y);
  gfx->print(text);
}

void drawButton(int x, int y, int w, int h, const char *label, uint16_t accent, bool active = false) {
  uint16_t fill = active ? accent : C_CARD2;
  uint16_t text = active ? C_BLACK : accent;

  gfx->fillRoundRect(x, y, w, h, 18, fill);
  gfx->drawRoundRect(x, y, w, h, 18, accent);

  gfx->setTextSize(2);
  gfx->setTextColor(text, fill);

  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);

  gfx->setCursor(x + ((w - tw) / 2), y + ((h - th) / 2));
  gfx->print(label);
}

void drawSmallButton(int x, int y, int w, int h, const char *label, uint16_t accent) {
  gfx->fillRoundRect(x, y, w, h, 14, C_CARD2);
  gfx->drawRoundRect(x, y, w, h, 14, accent);

  gfx->setTextSize(2);
  gfx->setTextColor(accent, C_CARD2);

  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);

  gfx->setCursor(x + ((w - tw) / 2), y + ((h - th) / 2));
  gfx->print(label);
}

void drawSmallCard(int x, int y, int w, int h, const char *label, const char *value, uint16_t accent) {
  gfx->fillRoundRect(x, y, w, h, 15, C_CARD);
  gfx->drawRoundRect(x, y, w, h, 15, accent);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_CARD);
  gfx->setCursor(x + 10, y + 10);
  gfx->print(label);

  gfx->setTextSize(2);
  gfx->setTextColor(C_WHITE, C_CARD);
  gfx->setCursor(x + 10, y + 30);
  gfx->print(value);
}

void drawHeader(const char *rightText, uint16_t dotColor) {
  gfx->fillRoundRect(14, 10, SCREEN_W - 28, 48, 18, C_CARD);

  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN, C_CARD);
  gfx->setCursor(28, 26);
  gfx->print("WATER WATCH PRO");

  gfx->setTextSize(1);
  gfx->setTextColor(C_WHITE, C_CARD);
  gfx->setCursor(SCREEN_W - 85, 30);
  gfx->print(rightText);

  gfx->fillCircle(SCREEN_W - 34, 34, 10, dotColor);
}

// =====================================================
// SENSOR READS
// =====================================================

void readTemperature() {
  uint8_t fault = max31865.readFault();

  if (fault) {
    tempSensorOK = false;
    max31865.clearFault();

    USBSerial.print("MAX31865 fault: 0x");
    USBSerial.println(fault, HEX);

    return;
  }

  float tempC = max31865.temperature(RNOMINAL, RREF);
  float tempF = (tempC * 9.0 / 5.0) + 32.0;

  if (isnan(tempF) || tempF < -100 || tempF > 600) {
    tempSensorOK = false;
  } else {
    tempSensorOK = true;
    waterTempF = tempF;
  }
}

void readPressure() {
  const int samples = 20;
  long totalRaw = 0;

  for (int i = 0; i < samples; i++) {
    totalRaw += analogRead(PRESSURE_GPIO);
    delayMicroseconds(250);
  }

  pressureRaw = totalRaw / samples;
  pressureRawVoltage = (pressureRaw / 4095.0) * 3.3;

  if (!pressureFilterReady) {
    pressureFilteredVoltage = pressureRawVoltage;
    pressureFilterReady = true;
  } else {
    pressureFilteredVoltage = (PRESSURE_ALPHA * pressureRawVoltage) + ((1.0 - PRESSURE_ALPHA) * pressureFilteredVoltage);
  }

  pressureVoltage = pressureFilteredVoltage;

  if (pressureVoltage < 0.01 || pressureVoltage > 3.30) {
    pressureSensorOK = false;
    tankPercent = 0.0;
    gallonsLeft = 0.0;
    gallonsUsed = tankGallons;
    return;
  }

  pressureSensorOK = true;

  float percent = pressurePercentFromCalibration(pressureVoltage);

  tankPercent = clampFloat(percent, 0.0, 100.0);
  gallonsLeft = tankGallons * (tankPercent / 100.0);
  gallonsUsed = tankGallons - gallonsLeft;
}

void readAllSensors() {
  readTemperature();
  readPressure();
  updatePumpOutput();

  USBSerial.print("Temp F: ");
  USBSerial.print(waterTempF);
  USBSerial.print(" | TempOK: ");
  USBSerial.print(tempSensorOK ? "YES" : "NO");
  USBSerial.print(" | Pressure Raw V: ");
  USBSerial.print(pressureRawVoltage, 3);
  USBSerial.print(" | Filtered V: ");
  USBSerial.print(pressureVoltage, 3);
  USBSerial.print(" | Raw: ");
  USBSerial.print(pressureRaw);
  USBSerial.print(" | Tank %: ");
  USBSerial.print(tankPercent);
  USBSerial.print(" | Cal: ");
  USBSerial.println(pressureCalibrated() ? "YES" : "NO");
}

// =====================================================
// DASHBOARD
// =====================================================

void drawGauge(float percent) {
  int cx = SCREEN_W / 2;
  int cy = 226;

  gfx->fillCircle(cx, cy, 62, C_BLACK);
  gfx->drawCircle(cx, cy, 64, C_CARD);
  gfx->drawCircle(cx, cy, 65, C_CARD);

  const int segments = 36;
  int active = (int)((segments * percent) / 100.0);

  for (int i = 0; i < segments; i++) {
    float deg = -215.0 + (i * (250.0 / (segments - 1)));
    float rad = deg * DEG_TO_RAD;

    int x1 = cx + cos(rad) * 72;
    int y1 = cy + sin(rad) * 72;
    int x2 = cx + cos(rad) * 88;
    int y2 = cy + sin(rad) * 88;

    uint16_t c = C_DIM;

    if (i < active) {
      if (!pressureCalibrated()) c = C_ORANGE;
      else if (percent <= lowAlertPercent) c = C_RED;
      else if (i < 10) c = C_BLUE;
      else if (i < 29) c = C_GREEN;
      else c = C_ORANGE;
    }

    gfx->drawLine(x1, y1, x2, y2, c);
    gfx->drawLine(x1 + 1, y1, x2 + 1, y2, c);
    gfx->drawLine(x1 - 1, y1, x2 - 1, y2, c);
  }

  char pct[12];

  if (!pressureSensorOK) {
    snprintf(pct, sizeof(pct), "ERR");
  } else {
    snprintf(pct, sizeof(pct), "%.0f%%", percent);
  }

  gfx->setTextSize(5);
  gfx->setTextColor(C_WHITE, C_BLACK);

  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(pct, 0, 0, &x1, &y1, &tw, &th);

  gfx->setCursor(cx - (tw / 2), cy - 25);
  gfx->print(pct);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(cx - 36, cy + 30);
  gfx->print("TANK LEVEL");
}

void drawLevelBar(float percent) {
  int x = 32;
  int y = 318;
  int w = SCREEN_W - 64;
  int h = 22;

  gfx->fillRoundRect(x, y, w, h, 11, C_CARD);
  gfx->drawRoundRect(x, y, w, h, 11, C_DIM);

  int fillW = (int)((w - 8) * percent / 100.0);
  fillW = constrain(fillW, 0, w - 8);

  uint16_t color = C_GREEN;
  if (!pressureSensorOK) color = C_RED;
  else if (!pressureCalibrated()) color = C_ORANGE;
  else if (percent <= lowAlertPercent) color = C_RED;
  else if (percent < 45) color = C_ORANGE;

  gfx->fillRoundRect(x + 4, y + 4, fillW, h - 8, 8, color);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(x, y + 31);
  gfx->print("EMPTY");

  gfx->setCursor(x + w - 28, y + 31);
  gfx->print("FULL");
}

void markDashboardDrawn() {
  lastDrawnScreen = SCREEN_DASHBOARD;
  lastDrawnTempF = waterTempF;
  lastDrawnTankPercent = tankPercent;
  lastDrawnPressureVoltage = pressureVoltage;
  lastDrawnTempOK = tempSensorOK;
  lastDrawnPressureOK = pressureSensorOK;
  lastDrawnPumpState = isPumpActive();
  lastDrawnRelayMode = relayMode;
  lastDrawnUnitHours = unitHours;
  lastDrawnServiceDue = serviceDue();
  lastDrawnDueBannerIndex = rotatingDueIndex();
  lastDrawnServiceFlashPhase = serviceFlashPhase();
}

bool shouldRedrawDashboard() {
  if (screen != SCREEN_DASHBOARD) return false;

  bool pumpState = isPumpActive();

  if (lastDrawnScreen != SCREEN_DASHBOARD) return true;
  if (tempSensorOK != lastDrawnTempOK) return true;
  if (pressureSensorOK != lastDrawnPressureOK) return true;
  if (pumpState != lastDrawnPumpState) return true;
  if (relayMode != lastDrawnRelayMode) return true;
  if (serviceDue() != lastDrawnServiceDue) return true;
  if (serviceDue() && rotatingDueIndex() != lastDrawnDueBannerIndex) return true;
  if (serviceDue() && serviceFlashPhase() != lastDrawnServiceFlashPhase) return true;
  if (abs(unitHours - lastDrawnUnitHours) >= 0.1) return true;

  if (tempSensorOK && abs(waterTempF - lastDrawnTempF) >= 1.0) return true;
  if (pressureSensorOK && abs(tankPercent - lastDrawnTankPercent) >= DASHBOARD_PERCENT_REDRAW) return true;
  if (abs(pressureVoltage - lastDrawnPressureVoltage) >= DASHBOARD_VOLT_REDRAW) return true;

  return false;
}

void drawDashboardTopControls() {
  // Upper-left WAIT/RUN dot removed. This spot is now the larger total hour meter.
  gfx->fillRoundRect(22, 14, 148, 42, 18, C_CARD2);
  gfx->drawRoundRect(22, 14, 148, 42, 18, C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_CARD2);
  gfx->setCursor(34, 21);
  gfx->print("HOURS");

  char hrValue[18];
  snprintf(hrValue, sizeof(hrValue), "%.1f", unitHours);
  gfx->setTextSize(2);
  gfx->setTextColor(C_CYAN, C_CARD2);

  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(hrValue, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(22 + ((148 - tw) / 2), 34);
  gfx->print(hrValue);

  drawButton(196, 14, 148, 42, auxOn ? "AUX ON" : "AUX OFF", auxOn ? C_GREEN : C_ORANGE, auxOn);
}

void drawDashboardServiceBar() {
  char svcText[28];
  serviceStatusText(svcText, sizeof(svcText));

  bool due = serviceDue();
  bool flash = serviceFlashPhase();

  uint16_t accent = due ? C_RED : C_GREEN;
  uint16_t fill = C_CARD2;
  uint16_t textColor = accent;

  if (due && flash) {
    fill = C_RED;
    textColor = C_BLACK;
  }

  gfx->fillRoundRect(196, 398, 148, 38, 18, fill);
  gfx->drawRoundRect(196, 398, 148, 38, 18, accent);
  gfx->setTextSize(2);
  gfx->setTextColor(textColor, fill);

  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(svcText, 0, 0, &x1, &y1, &tw, &th);

  // If a due label is too wide, drop down to size 1 so it still fits.
  if (tw > 136) {
    gfx->setTextSize(1);
    gfx->getTextBounds(svcText, 0, 0, &x1, &y1, &tw, &th);
  }

  gfx->setCursor(196 + ((148 - tw) / 2), 398 + ((38 - th) / 2));
  gfx->print(svcText);
}

void drawDashboard() {
  gfx->fillScreen(C_BLACK);

  drawDashboardTopControls();


  bool relayOn = isPumpActive();

  char leftText[20];
  char tempText[20];
  char voltText[20];

  if (pressureSensorOK) snprintf(leftText, sizeof(leftText), "%.2f G", gallonsLeft);
  else snprintf(leftText, sizeof(leftText), "ERR");

  if (tempSensorOK) snprintf(tempText, sizeof(tempText), "%.0f F", waterTempF);
  else snprintf(tempText, sizeof(tempText), "ERR");

  snprintf(voltText, sizeof(voltText), "%.2f V", pressureVoltage);

  drawSmallCard(22, 72, 100, 58, "LEFT", leftText, pressureSensorOK ? C_CYAN : C_RED);
  drawSmallCard(134, 72, 100, 58, "TEMP", tempText, tempSensorOK ? C_ORANGE : C_RED);
  drawSmallCard(246, 72, 100, 58, "PRES", voltText, pressureSensorOK ? C_BLUE : C_RED);

  drawGauge(tankPercent);
  drawLevelBar(tankPercent);

  char modeText[24];
  snprintf(modeText, sizeof(modeText), "MODE: %s", relayModeLabel());

  if (relayMode == RELAY_ON) {
    centerText("MANUAL OVERRIDE ON", 344, 1, C_GREEN);
    centerText(modeText, 362, 1, C_GREEN);
    centerText("12V ACCESSORY: ON", 380, 1, C_GREEN);
  } else if (relayMode == RELAY_OFF) {
    centerText("RELAY FORCED OFF", 344, 1, C_ORANGE);
    centerText(modeText, 362, 1, C_ORANGE);
    centerText("12V ACCESSORY: OFF", 380, 1, C_ORANGE);
  } else if (!tempSensorOK) {
    centerText("AUTO: TEMP SENSOR FAULT", 344, 1, C_RED);
    centerText(modeText, 362, 1, C_RED);
    centerText("12V ACCESSORY: OFF", 380, 1, C_RED);
  } else if (relayOn) {
    centerText("TEMP IN RANGE", 344, 1, C_YELLOW);
    centerText(modeText, 362, 1, C_YELLOW);
    centerText("AUTO ACCESSORY: ON", 380, 1, C_YELLOW);
  } else {
    centerText("WAITING FOR TEMP RANGE", 344, 1, C_ORANGE);
    centerText(modeText, 362, 1, C_ORANGE);
    centerText("AUTO ACCESSORY: OFF", 380, 1, C_ORANGE);
  }

  uint16_t modeColor = C_CYAN;
  bool modeButtonActive = (relayMode != RELAY_AUTO);
  const char *modeButtonText = relayModeLabel();

  if (relayMode == RELAY_ON) {
    modeColor = C_GREEN;
    modeButtonActive = true;
  } else if (relayMode == RELAY_OFF) {
    modeColor = C_RED;
    modeButtonActive = true;
  } else if (relayOn) {
    modeColor = C_YELLOW;
    modeButtonActive = true;
    modeButtonText = "AUTO ON";
  } else {
    modeColor = C_ORANGE;
    modeButtonActive = false;
    modeButtonText = "AUTO OFF";
  }

  drawButton(24, 398, 148, 38, modeButtonText, modeColor, modeButtonActive);
  drawDashboardServiceBar();

  markDashboardDrawn();
}

void updateDashboardNoFlicker() {
  // No fillScreen() here. Only repaint the parts that actually change.
  // This prevents the full-screen blink every time a sensor reading updates.

  drawDashboardTopControls();

  bool relayOn = isPumpActive();

  char leftText[20];
  char tempText[20];
  char voltText[20];

  if (pressureSensorOK) snprintf(leftText, sizeof(leftText), "%.2f G", gallonsLeft);
  else snprintf(leftText, sizeof(leftText), "ERR");

  if (tempSensorOK) snprintf(tempText, sizeof(tempText), "%.0f F", waterTempF);
  else snprintf(tempText, sizeof(tempText), "ERR");

  snprintf(voltText, sizeof(voltText), "%.2f V", pressureVoltage);

  drawSmallCard(22, 72, 100, 58, "LEFT", leftText, pressureSensorOK ? C_CYAN : C_RED);
  drawSmallCard(134, 72, 100, 58, "TEMP", tempText, tempSensorOK ? C_ORANGE : C_RED);
  drawSmallCard(246, 72, 100, 58, "PRES", voltText, pressureSensorOK ? C_BLUE : C_RED);

  drawGauge(tankPercent);
  drawLevelBar(tankPercent);

  // Clear only the status text strip before redrawing it.
  gfx->fillRect(0, 344, SCREEN_W, 48, C_BLACK);

  char modeText[24];
  snprintf(modeText, sizeof(modeText), "MODE: %s", relayModeLabel());

  if (relayMode == RELAY_ON) {
    centerText("MANUAL OVERRIDE ON", 344, 1, C_GREEN);
    centerText(modeText, 362, 1, C_GREEN);
    centerText("12V ACCESSORY: ON", 380, 1, C_GREEN);
  } else if (relayMode == RELAY_OFF) {
    centerText("RELAY FORCED OFF", 344, 1, C_ORANGE);
    centerText(modeText, 362, 1, C_ORANGE);
    centerText("12V ACCESSORY: OFF", 380, 1, C_ORANGE);
  } else if (!tempSensorOK) {
    centerText("AUTO: TEMP SENSOR FAULT", 344, 1, C_RED);
    centerText(modeText, 362, 1, C_RED);
    centerText("12V ACCESSORY: OFF", 380, 1, C_RED);
  } else if (relayOn) {
    centerText("TEMP IN RANGE", 344, 1, C_YELLOW);
    centerText(modeText, 362, 1, C_YELLOW);
    centerText("AUTO ACCESSORY: ON", 380, 1, C_YELLOW);
  } else {
    centerText("WAITING FOR TEMP RANGE", 344, 1, C_ORANGE);
    centerText(modeText, 362, 1, C_ORANGE);
    centerText("AUTO ACCESSORY: OFF", 380, 1, C_ORANGE);
  }

  uint16_t modeColor = C_CYAN;
  bool modeButtonActive = (relayMode != RELAY_AUTO);
  const char *modeButtonText = relayModeLabel();

  if (relayMode == RELAY_ON) {
    modeColor = C_GREEN;
    modeButtonActive = true;
  } else if (relayMode == RELAY_OFF) {
    modeColor = C_RED;
    modeButtonActive = true;
  } else if (relayOn) {
    modeColor = C_YELLOW;
    modeButtonActive = true;
    modeButtonText = "AUTO ON";
  } else {
    modeColor = C_ORANGE;
    modeButtonActive = false;
    modeButtonText = "AUTO OFF";
  }

  drawButton(24, 398, 148, 38, modeButtonText, modeColor, modeButtonActive);
  drawDashboardServiceBar();

  markDashboardDrawn();
}

// =====================================================
// SETUP SCREENS
// =====================================================

void drawSetupScreen() {
  gfx->fillScreen(C_BLACK);

  drawHeader("SET", C_CYAN);

  centerText("SETUP", 58, 3, C_WHITE);

  drawButton(44, 86, 280, 32, "CALIBRATE", C_CYAN, false);
  drawButton(44, 124, 280, 32, "TANK SIZE", C_GREEN, false);
  drawButton(44, 162, 280, 32, "LOW ALERT", C_ORANGE, false);
  drawButton(44, 200, 280, 32, "TEMP RANGE", C_PURPLE, false);
  drawButton(44, 238, 280, 32, "PRESSURE LIVE", C_BLUE, false);
  drawButton(44, 276, 280, 32, "TEMP LIVE", C_PURPLE, false);
  drawButton(44, 314, 280, 32, "MAINTENANCE", serviceDue() ? C_RED : C_GREEN, false);
  drawButton(44, 352, 280, 32, "SET TOTAL HOURS", C_CYAN, false);
  drawButton(44, 392, 280, 42, "BACK", C_WHITE, false);

  lastDrawnScreen = SCREEN_SETUP;
}

void drawCalibrateScreen() {
  gfx->fillScreen(C_BLACK);

  drawHeader("CAL", pressureCalibrated() ? C_GREEN : C_ORANGE);

  centerText("TANK SHAPE CAL", 58, 2, C_WHITE);

  char liveText[32];
  snprintf(liveText, sizeof(liveText), "LIVE %.3f V", pressureVoltage);
  centerText(liveText, 88, 2, C_BLUE);

  char b0[28];
  char b25[28];
  char b50[28];
  char b75[28];
  char b100[28];

  if (shapeCalSaved[0]) snprintf(b0, sizeof(b0), "0%% %.3f", shapeCalV[0]);
  else snprintf(b0, sizeof(b0), "SET 0%%");

  if (shapeCalSaved[1]) snprintf(b25, sizeof(b25), "25%% %.3f", shapeCalV[1]);
  else snprintf(b25, sizeof(b25), "SET 25%%");

  if (shapeCalSaved[2]) snprintf(b50, sizeof(b50), "50%% %.3f", shapeCalV[2]);
  else snprintf(b50, sizeof(b50), "SET 50%%");

  if (shapeCalSaved[3]) snprintf(b75, sizeof(b75), "75%% %.3f", shapeCalV[3]);
  else snprintf(b75, sizeof(b75), "SET 75%%");

  if (shapeCalSaved[4]) snprintf(b100, sizeof(b100), "100%% %.3f", shapeCalV[4]);
  else snprintf(b100, sizeof(b100), "SET 100%%");

  drawButton(44, 120, 132, 34, b0, shapeCalSaved[0] ? C_GREEN : C_CYAN, shapeCalSaved[0]);
  drawButton(192, 120, 132, 34, b25, shapeCalSaved[1] ? C_GREEN : C_CYAN, shapeCalSaved[1]);
  drawButton(44, 166, 132, 34, b50, shapeCalSaved[2] ? C_GREEN : C_CYAN, shapeCalSaved[2]);
  drawButton(192, 166, 132, 34, b75, shapeCalSaved[3] ? C_GREEN : C_CYAN, shapeCalSaved[3]);
  drawButton(44, 212, 280, 40, b100, shapeCalSaved[4] ? C_GREEN : C_ORANGE, shapeCalSaved[4]);

  if (shapeCalibrated()) {
    char calMsg[32];
    snprintf(calMsg, sizeof(calMsg), "SHAPE CAL %d/5", savedShapePointCount());
    centerText(calMsg, 262, 2, C_GREEN);
  }
  else if (pressureCalibrated()) centerText("ENDPOINT CAL READY", 262, 2, C_ORANGE);
  else centerText("SET 0% AND 100% MIN", 262, 1, C_ORANGE);

  centerText("MIDPOINTS OPTIONAL", 292, 1, C_GRAY);

  drawButton(44, 318, 280, 32, "CLEAR 25/50/75", C_RED, false);
  drawButton(44, 365, 132, 50, "SAVE", C_CYAN, false);
  drawButton(192, 365, 132, 50, "BACK", C_WHITE, false);

  lastDrawnScreen = SCREEN_CALIBRATE;
  lastLiveScreenVoltage = pressureVoltage;
  lastLiveScreenPercent = tankPercent;
  lastLiveScreenDrawMs = millis();
}

void drawTankSizeScreen() {
  gfx->fillScreen(C_BLACK);

  drawHeader("TANK", C_GREEN);

  centerText("TANK SIZE", 78, 3, C_WHITE);

  char tankText[24];
  snprintf(tankText, sizeof(tankText), "%.0f GAL", tankGallons);
  centerText(tankText, 148, 5, C_GREEN);

  drawSmallButton(52, 250, 96, 56, "-1", C_ORANGE);
  drawSmallButton(220, 250, 96, 56, "+1", C_GREEN);

  drawButton(44, 330, 132, 50, "SAVE", C_CYAN, false);
  drawButton(192, 330, 132, 50, "BACK", C_WHITE, false);

  lastDrawnScreen = SCREEN_TANK_SIZE;
}

void drawLowAlertScreen() {
  gfx->fillScreen(C_BLACK);

  drawHeader("ALRT", C_ORANGE);

  centerText("LOW ALERT", 78, 3, C_WHITE);

  char alertText[24];
  snprintf(alertText, sizeof(alertText), "%.0f%%", lowAlertPercent);
  centerText(alertText, 148, 5, C_ORANGE);

  drawSmallButton(52, 250, 96, 56, "-5", C_RED);
  drawSmallButton(220, 250, 96, 56, "+5", C_GREEN);

  drawButton(44, 330, 132, 50, "SAVE", C_CYAN, false);
  drawButton(192, 330, 132, 50, "BACK", C_WHITE, false);

  lastDrawnScreen = SCREEN_LOW_ALERT;
}

void drawTempRangeScreen() {
  gfx->fillScreen(C_BLACK);

  drawHeader("TEMP", C_PURPLE);

  centerText("TEMP RANGE", 72, 3, C_WHITE);

  char rangeText[32];
  snprintf(rangeText, sizeof(rangeText), "%.0fF - %.0fF", tempLowF, tempHighF);

  centerText(rangeText, 122, 3, C_PURPLE);

  char lowText[24];
  char highText[24];
  snprintf(lowText, sizeof(lowText), "LOW %.0fF", tempLowF);
  snprintf(highText, sizeof(highText), "HIGH %.0fF", tempHighF);

  centerText(lowText, 176, 2, C_CYAN);
  drawSmallButton(44, 210, 96, 48, "-5", C_RED);
  drawSmallButton(228, 210, 96, 48, "+5", C_GREEN);

  centerText(highText, 274, 2, C_ORANGE);
  drawSmallButton(44, 308, 96, 48, "-5", C_RED);
  drawSmallButton(228, 308, 96, 48, "+5", C_GREEN);

  drawButton(44, 380, 132, 46, "SAVE", C_CYAN, false);
  drawButton(192, 380, 132, 46, "BACK", C_WHITE, false);

  lastDrawnScreen = SCREEN_TEMP_RANGE;
}

void drawPressureLiveScreen() {
  gfx->fillScreen(C_BLACK);

  drawHeader("PRES", pressureSensorOK ? C_BLUE : C_RED);

  centerText("PRESSURE LIVE", 62, 3, C_WHITE);

  char voltText[32];
  char rawVoltText[32];
  char pctText[32];
  char galText[32];

  snprintf(voltText, sizeof(voltText), "%.3f V", pressureVoltage);
  snprintf(rawVoltText, sizeof(rawVoltText), "RAW %.3f V", pressureRawVoltage);
  snprintf(pctText, sizeof(pctText), "TANK %.0f%%", tankPercent);
  snprintf(galText, sizeof(galText), "%.2f GAL LEFT", gallonsLeft);

  centerText(voltText, 120, 5, pressureSensorOK ? C_BLUE : C_RED);
  centerText(rawVoltText, 200, 2, C_GRAY);
  centerText(pctText, 240, 3, pressureSensorOK ? C_GREEN : C_RED);
  centerText(galText, 288, 2, C_CYAN);

  if (pressureCalibrated()) centerText("TANK CAL SAVED", 320, 1, C_GREEN);
  else centerText("NEEDS CALIBRATION", 320, 1, C_ORANGE);

  drawButton(44, 365, 132, 50, "SAVE", C_CYAN, false);
  drawButton(192, 365, 132, 50, "BACK", C_WHITE, false);

  lastDrawnScreen = SCREEN_PRESSURE_LIVE;
  lastLiveScreenVoltage = pressureVoltage;
  lastLiveScreenPercent = tankPercent;
  lastLiveScreenDrawMs = millis();
}

void drawTempLiveScreen() {
  gfx->fillScreen(C_BLACK);

  bool relayOn = isPumpActive();

  drawHeader("LIVE", relayOn ? C_GREEN : C_ORANGE);

  centerText("TEMP LIVE", 72, 3, C_WHITE);

  char tempText[24];

  if (tempSensorOK) {
    snprintf(tempText, sizeof(tempText), "%.1fF", waterTempF);
    centerText(tempText, 135, 5, relayOn ? C_GREEN : C_ORANGE);
  } else {
    centerText("SENSOR ERR", 145, 3, C_RED);
  }

  char rangeText[40];
  snprintf(rangeText, sizeof(rangeText), "RANGE %.0fF - %.0fF", tempLowF, tempHighF);
  centerText(rangeText, 220, 1, C_GRAY);

  char modeText[24];
  snprintf(modeText, sizeof(modeText), "MODE: %s", relayModeLabel());
  centerText(modeText, 246, 1, relayOn ? C_GREEN : C_ORANGE);

  if (relayOn) centerText("12V ACCESSORY: ON", 266, 1, C_GREEN);
  else centerText("12V ACCESSORY: OFF", 266, 1, C_ORANGE);

  drawButton(44, 365, 132, 50, "SAVE", C_CYAN, false);
  drawButton(192, 365, 132, 50, "BACK", C_WHITE, false);

  lastDrawnScreen = SCREEN_TEMP_LIVE;
  lastLiveScreenTemp = waterTempF;
  lastLiveScreenDrawMs = millis();
}


void drawMaintenanceScreen() {
  gfx->fillScreen(C_BLACK);

  bool due = maintenanceItemDue(selectedMaintItem);
  drawHeader("HRS", due ? C_RED : C_GREEN);

  centerText("MAINTENANCE", 58, 2, C_WHITE);

  char totalText[32];
  snprintf(totalText, sizeof(totalText), "TOTAL %.1f HRS", unitHours);
  centerText(totalText, 88, 2, C_CYAN);

  centerText(maintNames[selectedMaintItem], 126, 2, due ? C_RED : C_GREEN);

  char everyText[32];
  if (maintIntervals[selectedMaintItem] <= 0.0) snprintf(everyText, sizeof(everyText), "EVERY: OFF");
  else snprintf(everyText, sizeof(everyText), "EVERY: %.0f HRS", maintIntervals[selectedMaintItem]);
  centerText(everyText, 160, 2, C_ORANGE);

  float used = maintenanceUsedHours(selectedMaintItem);
  char usedText[32];
  snprintf(usedText, sizeof(usedText), "USED: %.1f HRS", used);
  centerText(usedText, 195, 2, C_WHITE);

  char leftText[32];
  if (maintIntervals[selectedMaintItem] <= 0.0) {
    snprintf(leftText, sizeof(leftText), "STATUS: OFF");
    centerText(leftText, 226, 2, C_GRAY);
  } else {
    float left = maintIntervals[selectedMaintItem] - used;
    if (left < 0) left = 0;
    snprintf(leftText, sizeof(leftText), "LEFT: %.1f HRS", left);
    centerText(leftText, 226, 2, due ? C_RED : C_GREEN);
  }

  if (due) centerText("SERVICE DUE", 256, 2, C_RED);
  else centerText("SERVICE OK", 256, 2, C_GREEN);

  drawSmallButton(24, 292, 96, 40, "PREV", C_WHITE);
  drawSmallButton(136, 292, 96, 40, "NEXT", C_WHITE);
  drawSmallButton(248, 292, 96, 40, "INT", C_ORANGE);

  drawButton(44, 344, 280, 40, "RESET AFTER SERVICE", C_RED, false);
  drawButton(44, 396, 132, 38, "SAVE", C_CYAN, false);
  drawButton(192, 396, 132, 38, "BACK", C_WHITE, false);

  lastDrawnScreen = SCREEN_MAINTENANCE;
  lastLiveScreenDrawMs = millis();
}

void drawTotalHoursScreen() {
  gfx->fillScreen(C_BLACK);

  drawHeader("HRS", C_CYAN);

  centerText("SET TOTAL HOURS", 64, 2, C_WHITE);
  centerText("MATCH MACHINE HOUR METER", 92, 1, C_GRAY);

  char hrText[24];
  snprintf(hrText, sizeof(hrText), "%.1f HRS", unitHours);
  centerText(hrText, 145, 4, C_CYAN);

  drawSmallButton(52, 245, 96, 60, "-1", C_RED);
  drawSmallButton(220, 245, 96, 60, "+1", C_GREEN);

  centerText("ADJUSTS BY 1 HOUR", 324, 1, C_GRAY);

  drawButton(44, 370, 132, 50, "SAVE", C_CYAN, false);
  drawButton(192, 370, 132, 50, "BACK", C_WHITE, false);

  lastDrawnScreen = SCREEN_TOTAL_HOURS;
}


// =====================================================
// OTA UPDATE MODE
// =====================================================

String otaHtmlPage() {
  String html;
  html += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Water Watch Pro Host Update</title>";
  html += "<style>body{font-family:Arial;background:#05070a;color:#eaf7ff;margin:0;padding:22px;}";
  html += ".card{max-width:520px;margin:auto;background:#121820;border:1px solid #24d8ff;border-radius:18px;padding:20px;}";
  html += "h1{color:#24d8ff;font-size:24px;} .warn{color:#ffb000;} input,button{font-size:18px;margin-top:12px;width:100%;padding:12px;border-radius:10px;}";
  html += "button{background:#24d8ff;color:#000;border:0;font-weight:bold;}</style></head><body><div class='card'>";
  html += "<h1>Water Watch Pro Host Update</h1>";
  html += "<p>Upload the compiled HOST firmware .bin file only.</p>";
  html += "<p class='warn'>Do not power off during update.</p>";
  html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
  html += "<input type='file' name='update' accept='.bin'><button type='submit'>Upload Firmware</button></form>";
  html += "</div></body></html>";
  return html;
}

void drawUpdateScreen() {
  gfx->fillScreen(C_BLACK);
  drawHeader("UPD", otaModeActive ? C_GREEN : C_CYAN);

  centerText("UPDATE MODE", 68, 3, C_CYAN);
  centerText("ANDROID / WIFI OTA", 108, 1, C_GRAY);

  if (!otaModeActive) {
    centerText("START HOTSPOT", 150, 2, C_WHITE);
    drawButton(44, 182, 280, 52, "START UPDATE", C_GREEN, false);
    centerText("Phone connects to host WiFi", 262, 1, C_GRAY);
    centerText("Then upload HOST .bin", 280, 1, C_GRAY);
    drawButton(44, 380, 280, 46, "BACK", C_WHITE, false);
  } else {
    centerText("HOTSPOT ON", 134, 2, C_GREEN);

    gfx->setTextSize(2);
    gfx->setTextColor(C_WHITE, C_BLACK);
    gfx->setCursor(34, 174);
    gfx->print("WiFi:");
    gfx->setCursor(34, 198);
    gfx->print(OTA_AP_SSID);

    gfx->setTextColor(C_ORANGE, C_BLACK);
    gfx->setCursor(34, 236);
    gfx->print("Pass: 12345678");

    gfx->setTextColor(C_CYAN, C_BLACK);
    gfx->setCursor(34, 278);
    gfx->print("Go to:");
    gfx->setCursor(34, 304);
    gfx->print("192.168.4.1");

    if (otaUploadInProgress) {
      char buf[36];
      snprintf(buf, sizeof(buf), "UPLOADING %u KB", (unsigned)(otaBytesWritten / 1024));
      centerText(buf, 350, 2, C_ORANGE);
    } else if (otaUpdateEnded) {
      centerText(otaUpdateOk ? "UPDATE OK - REBOOTING" : "UPDATE FAILED", 350, 1, otaUpdateOk ? C_GREEN : C_RED);
    } else {
      centerText("10 MIN TIMEOUT", 350, 1, C_GRAY);
    }

    drawButton(44, 380, 280, 46, "CANCEL / RESTART", C_ORANGE, false);
  }

  lastDrawnScreen = SCREEN_UPDATE;
  otaLastScreenDrawMs = millis();
}

void handleOtaRoot() {
  otaServer.send(200, "text/html", otaHtmlPage());
}

void handleOtaFinished() {
  bool ok = !Update.hasError();
  otaServer.send(200, "text/html", ok ? "<h2>Update OK. Rebooting...</h2>" : "<h2>Update failed.</h2>");
  otaUpdateEnded = true;
  otaUpdateOk = ok;
  otaUploadInProgress = false;
  drawUpdateScreen();
  delay(1000);
  if (ok) ESP.restart();
}

void handleOtaUpload() {
  HTTPUpload &upload = otaServer.upload();

  if (upload.status == UPLOAD_FILE_START) {
    otaUploadInProgress = true;
    otaUpdateEnded = false;
    otaUpdateOk = false;
    otaBytesWritten = 0;
    USBSerial.print("OTA upload start: ");
    USBSerial.println(upload.filename);

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      USBSerial.println("OTA Update.begin failed");
      Update.printError(USBSerial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      USBSerial.println("OTA Update.write failed");
      Update.printError(USBSerial);
    }
    otaBytesWritten += upload.currentSize;

    if (hostScreenAwake && screen == SCREEN_UPDATE && millis() - otaLastScreenDrawMs > 700UL) {
      drawUpdateScreen();
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      USBSerial.print("OTA update complete bytes=");
      USBSerial.println((unsigned)otaBytesWritten);
    } else {
      USBSerial.println("OTA Update.end failed");
      Update.printError(USBSerial);
    }
    otaUploadInProgress = false;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    otaUploadInProgress = false;
    otaUpdateEnded = true;
    otaUpdateOk = false;
    USBSerial.println("OTA upload aborted");
    drawUpdateScreen();
  }
}

void startHostOtaMode() {
  if (otaModeActive) return;

  otaModeActive = true;
  otaUploadInProgress = false;
  otaUpdateEnded = false;
  otaUpdateOk = false;
  otaBytesWritten = 0;
  otaModeStartMs = millis();

  USBSerial.println("Starting host OTA AP update mode");

#if REMOTE_SEND_ENABLED
  if (remoteSenderReady) {
    esp_now_deinit();
    remoteSenderReady = false;
  }
#endif

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(OTA_AP_SSID, OTA_AP_PASS);

  otaServer.on("/", HTTP_GET, handleOtaRoot);
  otaServer.on("/update", HTTP_POST, handleOtaFinished, handleOtaUpload);
  otaServer.begin();

  if (hostScreenAwake) drawUpdateScreen();
}

void handleHostOtaMode() {
  if (!otaModeActive) return;

  otaServer.handleClient();

  if (!otaUploadInProgress && !otaUpdateEnded && millis() - otaModeStartMs > OTA_MODE_TIMEOUT_MS) {
    USBSerial.println("OTA update mode timeout - restarting normal host mode");
    ESP.restart();
  }
}

void cancelOtaModeRestart() {
  if (!otaModeActive) return;
  USBSerial.println("OTA update mode cancelled - restarting");
  ESP.restart();
}

// =====================================================
// NAVIGATION
// =====================================================

void goScreen(int next) {
  screen = (ScreenMode)next;

  if (screen == SCREEN_DASHBOARD) drawDashboard();
  else if (screen == SCREEN_SETUP) drawSetupScreen();
  else if (screen == SCREEN_CALIBRATE) drawCalibrateScreen();
  else if (screen == SCREEN_TANK_SIZE) drawTankSizeScreen();
  else if (screen == SCREEN_LOW_ALERT) drawLowAlertScreen();
  else if (screen == SCREEN_TEMP_RANGE) drawTempRangeScreen();
  else if (screen == SCREEN_TEMP_LIVE) drawTempLiveScreen();
  else if (screen == SCREEN_PRESSURE_LIVE) drawPressureLiveScreen();
  else if (screen == SCREEN_TOTAL_HOURS) drawTotalHoursScreen();
  else if (screen == SCREEN_MAINTENANCE) drawMaintenanceScreen();
  else if (screen == SCREEN_UPDATE) drawUpdateScreen();
}

// =====================================================
// TOUCH
// =====================================================


void goNextSwipeScreen() {
  if (screen == SCREEN_DASHBOARD) goScreen(SCREEN_PRESSURE_LIVE);
  else if (screen == SCREEN_PRESSURE_LIVE) goScreen(SCREEN_TEMP_LIVE);
  else if (screen == SCREEN_TEMP_LIVE) goScreen(SCREEN_MAINTENANCE);
  else if (screen == SCREEN_MAINTENANCE) goScreen(SCREEN_SETUP);
  else if (screen == SCREEN_SETUP) goScreen(SCREEN_UPDATE);
  else if (screen == SCREEN_UPDATE) goScreen(SCREEN_DASHBOARD);
}

void goPrevSwipeScreen() {
  if (screen == SCREEN_DASHBOARD) goScreen(SCREEN_UPDATE);
  else if (screen == SCREEN_UPDATE) goScreen(SCREEN_SETUP);
  else if (screen == SCREEN_SETUP) goScreen(SCREEN_MAINTENANCE);
  else if (screen == SCREEN_MAINTENANCE) goScreen(SCREEN_TEMP_LIVE);
  else if (screen == SCREEN_TEMP_LIVE) goScreen(SCREEN_PRESSURE_LIVE);
  else if (screen == SCREEN_PRESSURE_LIVE) goScreen(SCREEN_DASHBOARD);
}

void handleSwipe(int dx) {
  if (dx < 0) goNextSwipeScreen();
  else goPrevSwipeScreen();
}

void finishTouchAsTapOrSwipe() {
  if (!touchDown) return;

  int dx = touchLastX - touchStartX;
  int dy = touchLastY - touchStartY;
  unsigned long held = millis() - touchStartMs;

  if (!swipeHandled) {
    if (abs(dx) >= 60 && abs(dy) <= 90 && held >= 60 && held <= 1200) {
      handleSwipe(dx);
    } else if (abs(dx) <= 35 && abs(dy) <= 35 && held <= 1200) {
      handleTap(touchStartX, touchStartY);
    }
  }

  touchDown = false;
  swipeHandled = false;
}


void redrawCurrentHostScreen() {
  if (!hostScreenAwake) return;

  if (screen == SCREEN_DASHBOARD) drawDashboard();
  else if (screen == SCREEN_SETUP) drawSetupScreen();
  else if (screen == SCREEN_CALIBRATE) drawCalibrateScreen();
  else if (screen == SCREEN_TANK_SIZE) drawTankSizeScreen();
  else if (screen == SCREEN_LOW_ALERT) drawLowAlertScreen();
  else if (screen == SCREEN_TEMP_RANGE) drawTempRangeScreen();
  else if (screen == SCREEN_TEMP_LIVE) drawTempLiveScreen();
  else if (screen == SCREEN_PRESSURE_LIVE) drawPressureLiveScreen();
  else if (screen == SCREEN_TOTAL_HOURS) drawTotalHoursScreen();
  else if (screen == SCREEN_MAINTENANCE) drawMaintenanceScreen();
  else if (screen == SCREEN_UPDATE) drawUpdateScreen();
}

void enterHostScreenOff() {
  hostScreenAwake = false;
  touchDown = false;
  swipeHandled = false;
  gfx->fillScreen(C_BLACK);
  gfx->setBrightness(HOST_SCREEN_OFF_BRIGHTNESS);
  USBSerial.println("Host screen off - controller still running");
}

void wakeHostScreenFromPowerKey() {
  if (hostScreenAwake) return;

  hostScreenAwake = true;
  gfx->setBrightness(HOST_AWAKE_BRIGHTNESS);
  lastTouchMs = millis();
  touchDown = false;
  swipeHandled = false;
  USBSerial.println("Host screen wake from power button");
  redrawCurrentHostScreen();
}

void setupHostPowerKeyWake() {
  hostPmuReady = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);

  if (!hostPmuReady) {
    USBSerial.println("AXP2101 PMU not found - host power button screen toggle disabled");
    return;
  }

  PMU.clearIrqStatus();
  PMU.enableIRQ(XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ);
  USBSerial.println("AXP2101 PMU OK - host power button toggles screen only");
}

void toggleHostScreenFromPowerKey() {
  if (hostScreenAwake) {
    enterHostScreenOff();
  } else {
    wakeHostScreenFromPowerKey();
  }
}

void checkHostPowerKey() {
  if (!hostPmuReady) return;

  uint32_t status = PMU.getIrqStatus();
  if (status == 0) return;

  if (PMU.isPekeyNegativeIrq()) {
    hostPowerKeyDown = true;
    hostPowerKeyDownMs = millis();
  }

  if (PMU.isPekeyPositiveIrq()) {
    if (hostPowerKeyDown) {
      unsigned long held = millis() - hostPowerKeyDownMs;
      if (held <= HOST_POWER_KEY_TAP_MAX_MS) {
        toggleHostScreenFromPowerKey();
      }
    }

    hostPowerKeyDown = false;
  }

  PMU.clearIrqStatus();
}

void handleTap(int x, int y) {
  if (millis() - lastTouchMs < 250) return;
  lastTouchMs = millis();

  USBSerial.print("TOUCH X=");
  USBSerial.print(x);
  USBSerial.print(" Y=");
  USBSerial.println(y);

  if (screen == SCREEN_DASHBOARD) {
    if (hitBox(x, y, 196, 14, 344, 56)) {
      auxOn = !auxOn;
      saveSettings();
      updateAuxOutput();
      drawDashboard();
      return;
    }

    if (hitBox(x, y, 24, 398, 172, 436)) {
      cycleRelayMode();
      updatePumpOutput();
      drawDashboard();
      return;
    }

    if (hitBox(x, y, 196, 398, 344, 436)) {
      goScreen(SCREEN_MAINTENANCE);
      return;
    }

    // V10.7: touchscreen no longer turns the host screen off.
    // Bottom AXP2101 power button handles screen off/on, just like the remote.
  }

  else if (screen == SCREEN_SETUP) {
    if (hitBox(x, y, 44, 86, 324, 118)) {
      goScreen(SCREEN_CALIBRATE);
      return;
    }

    if (hitBox(x, y, 44, 124, 324, 156)) {
      goScreen(SCREEN_TANK_SIZE);
      return;
    }

    if (hitBox(x, y, 44, 162, 324, 194)) {
      goScreen(SCREEN_LOW_ALERT);
      return;
    }

    if (hitBox(x, y, 44, 200, 324, 232)) {
      goScreen(SCREEN_TEMP_RANGE);
      return;
    }

    if (hitBox(x, y, 44, 238, 324, 270)) {
      goScreen(SCREEN_PRESSURE_LIVE);
      return;
    }

    if (hitBox(x, y, 44, 276, 324, 308)) {
      goScreen(SCREEN_TEMP_LIVE);
      return;
    }

    if (hitBox(x, y, 44, 314, 324, 346)) {
      goScreen(SCREEN_MAINTENANCE);
      return;
    }

    if (hitBox(x, y, 44, 352, 324, 384)) {
      goScreen(SCREEN_TOTAL_HOURS);
      return;
    }

    if (hitBox(x, y, 44, 392, 324, 434)) {
      goScreen(SCREEN_DASHBOARD);
      return;
    }
  }

  else if (hostScreenAwake && screen == SCREEN_CALIBRATE) {
    if (hitBox(x, y, 44, 120, 176, 154)) {
      shapeCalV[0] = pressureVoltage;
      shapeCalSaved[0] = true;
      emptyVoltage = pressureVoltage;
      emptySaved = true;
      saveSettings();
      readPressure();
      drawCalibrateScreen();
      return;
    }

    if (hitBox(x, y, 192, 120, 324, 154)) {
      shapeCalV[1] = pressureVoltage;
      shapeCalSaved[1] = true;
      saveSettings();
      readPressure();
      drawCalibrateScreen();
      return;
    }

    if (hitBox(x, y, 44, 166, 176, 200)) {
      shapeCalV[2] = pressureVoltage;
      shapeCalSaved[2] = true;
      saveSettings();
      readPressure();
      drawCalibrateScreen();
      return;
    }

    if (hitBox(x, y, 192, 166, 324, 200)) {
      shapeCalV[3] = pressureVoltage;
      shapeCalSaved[3] = true;
      saveSettings();
      readPressure();
      drawCalibrateScreen();
      return;
    }

    if (hitBox(x, y, 44, 212, 324, 252)) {
      shapeCalV[4] = pressureVoltage;
      shapeCalSaved[4] = true;
      fullVoltage = pressureVoltage;
      fullSaved = true;
      saveSettings();
      readPressure();
      drawCalibrateScreen();
      return;
    }

    if (hitBox(x, y, 44, 318, 324, 350)) {
      clearMiddleShapeCalibration();
      readPressure();
      drawCalibrateScreen();
      return;
    }

    if (hitBox(x, y, 44, 355, 176, 405) || hitBox(x, y, 192, 355, 324, 405)) {
      saveSettings();
      goScreen(SCREEN_SETUP);
      return;
    }
  }

  else if (screen == SCREEN_TANK_SIZE) {
    if (hitBox(x, y, 52, 250, 148, 306)) {
      tankGallons -= 1;
      tankGallons = clampFloat(tankGallons, 1, 500);
      saveSettings();
      drawTankSizeScreen();
      return;
    }

    if (hitBox(x, y, 220, 250, 316, 306)) {
      tankGallons += 1;
      tankGallons = clampFloat(tankGallons, 1, 500);
      saveSettings();
      drawTankSizeScreen();
      return;
    }

    if (hitBox(x, y, 44, 330, 176, 380) || hitBox(x, y, 192, 330, 324, 380)) {
      saveSettings();
      goScreen(SCREEN_SETUP);
      return;
    }
  }

  else if (screen == SCREEN_LOW_ALERT) {
    if (hitBox(x, y, 52, 250, 148, 306)) {
      lowAlertPercent -= 5;
      lowAlertPercent = clampFloat(lowAlertPercent, 5, 80);
      saveSettings();
      drawLowAlertScreen();
      return;
    }

    if (hitBox(x, y, 220, 250, 316, 306)) {
      lowAlertPercent += 5;
      lowAlertPercent = clampFloat(lowAlertPercent, 5, 80);
      saveSettings();
      drawLowAlertScreen();
      return;
    }

    if (hitBox(x, y, 44, 330, 176, 380) || hitBox(x, y, 192, 330, 324, 380)) {
      saveSettings();
      goScreen(SCREEN_SETUP);
      return;
    }
  }

  else if (screen == SCREEN_TEMP_RANGE) {
    if (hitBox(x, y, 44, 210, 140, 258)) {
      tempLowF -= 5;
      tempLowF = clampFloat(tempLowF, 40, tempHighF - 5);
      saveSettings();
      updatePumpOutput();
      drawTempRangeScreen();
      return;
    }

    if (hitBox(x, y, 228, 210, 324, 258)) {
      tempLowF += 5;
      tempLowF = clampFloat(tempLowF, 40, tempHighF - 5);
      saveSettings();
      updatePumpOutput();
      drawTempRangeScreen();
      return;
    }

    if (hitBox(x, y, 44, 308, 140, 356)) {
      tempHighF -= 5;
      tempHighF = clampFloat(tempHighF, tempLowF + 5, 250);
      saveSettings();
      updatePumpOutput();
      drawTempRangeScreen();
      return;
    }

    if (hitBox(x, y, 228, 308, 324, 356)) {
      tempHighF += 5;
      tempHighF = clampFloat(tempHighF, tempLowF + 5, 250);
      saveSettings();
      updatePumpOutput();
      drawTempRangeScreen();
      return;
    }

    if (hitBox(x, y, 44, 380, 176, 426) || hitBox(x, y, 192, 380, 324, 426)) {
      saveSettings();
      goScreen(SCREEN_SETUP);
      return;
    }
  }

  else if (screen == SCREEN_TEMP_LIVE || screen == SCREEN_PRESSURE_LIVE) {
    if (hitBox(x, y, 44, 365, 176, 415) || hitBox(x, y, 192, 365, 324, 415)) {
      saveSettings();
      goScreen(SCREEN_SETUP);
      return;
    }
  }

  else if (hostScreenAwake && screen == SCREEN_MAINTENANCE) {
    if (hitBox(x, y, 24, 292, 120, 332)) {
      selectedMaintItem--;
      if (selectedMaintItem < 0) selectedMaintItem = MAINT_COUNT - 1;
      drawMaintenanceScreen();
      return;
    }

    if (hitBox(x, y, 136, 292, 232, 332)) {
      selectedMaintItem++;
      if (selectedMaintItem >= MAINT_COUNT) selectedMaintItem = 0;
      drawMaintenanceScreen();
      return;
    }

    if (hitBox(x, y, 248, 292, 344, 332)) {
      cycleSelectedMaintenanceInterval();
      drawMaintenanceScreen();
      return;
    }

    if (hitBox(x, y, 44, 344, 324, 384)) {
      resetSelectedMaintenanceItem();
      drawMaintenanceScreen();
      return;
    }

    if (hitBox(x, y, 44, 396, 176, 434) || hitBox(x, y, 192, 396, 324, 434)) {
      saveMaintenanceOnly();
      goScreen(SCREEN_SETUP);
      return;
    }
  }

  else if (screen == SCREEN_UPDATE) {
    if (!otaModeActive && hitBox(x, y, 44, 182, 324, 234)) {
      startHostOtaMode();
      return;
    }

    if (otaModeActive && hitBox(x, y, 44, 380, 324, 426)) {
      cancelOtaModeRestart();
      return;
    }

    if (!otaModeActive && hitBox(x, y, 44, 380, 324, 426)) {
      goScreen(SCREEN_SETUP);
      return;
    }
  }

  else if (screen == SCREEN_TOTAL_HOURS) {
    if (hitBox(x, y, 52, 245, 148, 305)) {
      unitHours -= 1.0;
      if (unitHours < 0) unitHours = 0;
      saveHourMeterOnly();
      drawTotalHoursScreen();
      return;
    }

    if (hitBox(x, y, 220, 245, 316, 305)) {
      unitHours += 1.0;
      if (unitHours > 99999) unitHours = 99999;
      saveHourMeterOnly();
      drawTotalHoursScreen();
      return;
    }

    if (hitBox(x, y, 44, 370, 176, 420) || hitBox(x, y, 192, 370, 324, 420)) {
      saveHourMeterOnly();
      goScreen(SCREEN_SETUP);
      return;
    }
  }
}

void checkTouch() {
  // Screen-off mode acts like the remote: touch is ignored while off.
  // Wake comes from the bottom AXP2101 power button only.
  if (!hostScreenAwake) {
    if (FT3168->IIC_Interrupt_Flag == true) {
      FT3168->IIC_Interrupt_Flag = false;
    }
    return;
  }

  if (FT3168->IIC_Interrupt_Flag == true) {
    FT3168->IIC_Interrupt_Flag = false;

    int32_t touch_x = FT3168->IIC_Read_Device_Value(
      FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X
    );

    int32_t touch_y = FT3168->IIC_Read_Device_Value(
      FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y
    );

    uint8_t fingers_number = FT3168->IIC_Read_Device_Value(
      FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER
    );

    unsigned long now = millis();

    if (fingers_number > 0 && touch_x >= 0 && touch_x < SCREEN_W && touch_y >= 0 && touch_y < SCREEN_H) {
      if (!touchDown) {
        touchDown = true;
        swipeHandled = false;
        touchStartX = (int)touch_x;
        touchStartY = (int)touch_y;
        touchLastX = (int)touch_x;
        touchLastY = (int)touch_y;
        touchStartMs = now;
        touchLastSeenMs = now;
      } else {
        touchLastX = (int)touch_x;
        touchLastY = (int)touch_y;
        touchLastSeenMs = now;
      }

      int dx = touchLastX - touchStartX;
      int dy = touchLastY - touchStartY;
      unsigned long held = now - touchStartMs;

      if (!swipeHandled && abs(dx) >= 70 && abs(dy) <= 90 && held >= 60 && held <= 1200) {
        swipeHandled = true;
        lastTouchMs = now;
        handleSwipe(dx);
      }
    } else {
      finishTouchAsTapOrSwipe();
    }
  }

  // Some touch controllers do not always generate a clean finger-up event.
  // If the finger disappeared after the last touch report, finish it as a tap/swipe.
  if (touchDown && (millis() - touchLastSeenMs > 180)) {
    finishTouchAsTapOrSwipe();
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  USBSerial.begin(115200);
  USBSerial.println("Water Watch Pro V10.8 - Host OTA Update Mode + Power Button Screen Toggle");

  loadSettings();
  initRemoteSender();
  lastHourTickMs = millis();
  lastHourSaveMs = millis();

  pinMode(PUMP_GPIO, OUTPUT);
  digitalWrite(PUMP_GPIO, PUMP_ACTIVE_HIGH ? LOW : HIGH);
  lastPumpState = false;

  pinMode(AUX_GPIO, OUTPUT);
  updateAuxOutput();

  pinMode(PRESSURE_GPIO, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PRESSURE_GPIO, ADC_11db);

  Wire.begin(IIC_SDA, IIC_SCL);
  setupHostPowerKeyWake();

  if (!expander.begin(0x20)) {
    USBSerial.println("Failed to find XCA9554 chip");
  } else {
    expander.pinMode(0, OUTPUT);
    expander.pinMode(1, OUTPUT);
    expander.pinMode(2, OUTPUT);

    expander.digitalWrite(0, LOW);
    expander.digitalWrite(1, LOW);
    expander.digitalWrite(2, LOW);

    delay(20);

    expander.digitalWrite(0, HIGH);
    expander.digitalWrite(1, HIGH);
    expander.digitalWrite(2, HIGH);
  }

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  if (!gfx->begin()) {
    USBSerial.println("gfx->begin() failed!");
  }

  gfx->setBrightness(HOST_AWAKE_BRIGHTNESS);
  gfx->fillScreen(C_BLACK);

  while (FT3168->begin() == false) {
    USBSerial.println("FT3168 initialization fail");
    delay(1000);
  }

  USBSerial.println("FT3168 initialization successfully");
  USBSerial.printf("Touch ID: %#X\n", (int32_t)FT3168->IIC_Read_Device_ID());

  max31865.begin(MAX31865_2WIRE);

  delay(300);
  max31865.clearFault();
  delay(200);

  readAllSensors();
  drawDashboard();
  sendRemotePacket();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  updateHourMeter();
  checkHostPowerKey();
  handleHostOtaMode();
  checkTouch();

  if (millis() - lastReadMs > 500) {
    lastReadMs = millis();

    readAllSensors();

    if (!otaModeActive && millis() - lastRemoteSendMs >= REMOTE_SEND_INTERVAL_MS) {
      lastRemoteSendMs = millis();
      sendRemotePacket();
    }

    if (hostScreenAwake && screen == SCREEN_DASHBOARD && shouldRedrawDashboard()) {
      updateDashboardNoFlicker();
    }

    if (hostScreenAwake && screen == SCREEN_CALIBRATE) {
      if ((millis() - lastLiveScreenDrawMs > 2500) &&
          (abs(pressureVoltage - lastLiveScreenVoltage) >= 0.08 ||
           abs(tankPercent - lastLiveScreenPercent) >= 3.0)) {
        drawCalibrateScreen();
      }
    }

    if (hostScreenAwake && screen == SCREEN_PRESSURE_LIVE) {
      if ((millis() - lastLiveScreenDrawMs > 750) &&
          (abs(pressureVoltage - lastLiveScreenVoltage) >= 0.02 ||
           abs(tankPercent - lastLiveScreenPercent) >= 2.0)) {
        drawPressureLiveScreen();
      }
    }

    if (hostScreenAwake && screen == SCREEN_TEMP_LIVE) {
      if ((millis() - lastLiveScreenDrawMs > 2000) &&
          (abs(waterTempF - lastLiveScreenTemp) >= 0.5)) {
        drawTempLiveScreen();
      }
    }

    if (hostScreenAwake && screen == SCREEN_MAINTENANCE) {
      if (millis() - lastLiveScreenDrawMs > 5000) {
        drawMaintenanceScreen();
        lastLiveScreenDrawMs = millis();
      }
    }
  }
}