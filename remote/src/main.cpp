#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"
#include <Adafruit_XCA9554.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_sleep.h>

#include <XPowersLib.h>
#define REMOTE_HAS_XPOWERS 1

#ifndef AXP2101_SLAVE_ADDRESS
#define AXP2101_SLAVE_ADDRESS 0x34
#endif

HWCDC USBSerial;

Preferences remotePrefs;

#if REMOTE_HAS_XPOWERS
XPowersPMU PMU;
#endif

int remoteBatteryPercent = -1;
int remoteBatteryMv = -1;
bool remoteBatteryCharging = false;
bool remoteUsbPresent = false;
int lastDrawnBatteryPercent = -9999;
int lastDrawnBatteryMv = -9999;
bool lastDrawnBatteryCharging = false;
bool lastDrawnUsbPresent = false;
bool remoteBatteryReady = false;
unsigned long lastBatteryReadMs = 0;
const unsigned long BATTERY_READ_INTERVAL_MS = 10000UL;

// =====================================================
// REMOTE POWER / MEMORY BEHAVIOR
// =====================================================

const unsigned long SCREEN_SLEEP_MS = 30000UL;          // 30 seconds no touch/activity = screen off
const unsigned long SAVE_PACKET_INTERVAL_MS = 5000UL;  // avoid flash wear

unsigned long lastUserActivityMs = 0;
unsigned long lastPacketSaveMs = 0;
bool screenAwake = true;
bool pluggedScreenOff = false;
bool liveStatusPage = false;
bool updateModePage = false;
bool liveStatusDirty = true;
unsigned long lastLiveStatusDrawMs = 0;
const unsigned long LIVE_STATUS_REFRESH_MS = 1000UL;
bool loadedSavedPacket = false;

// =====================================================
// REMOTE OTA UPDATE MODE
// =====================================================

WebServer remoteUpdateServer(80);
const char *REMOTE_OTA_AP_SSID = "WaterWatchPro-Remote";
const char *REMOTE_OTA_AP_PASS = "12345678";
const char *REMOTE_FIRMWARE_VERSION = "REMOTE V41 TEST";
const unsigned long REMOTE_OTA_TIMEOUT_MS = 600000UL;  // 10 minutes
bool remoteOtaActive = false;
unsigned long remoteOtaStartedMs = 0;
String remoteOtaStatus = "READY";
uint16_t remoteOtaStatusColor = 0x8410;

// =====================================================
// WATER WATCH PRO - 1.8 REMOTE RECEIVER - SAME DASH + TWO WAY V41_UPDATE_SCREEN_VERSION_TEST
// For the same Waveshare ESP32-S3 1.8" AMOLED display family.
// This is the read-only remote display firmware.
// It receives ESP-NOW packets from the main Water Watch Pro V10.4 sender.
// No pressure sensor, no temp board, no relay, no AUX wiring required.
// Power this display by USB-C / 5V only.
// =====================================================

// =====================================================
// DISPLAY HARDWARE OBJECTS
// =====================================================

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
// ESP-NOW PACKET - MUST MATCH MAIN V10.4 SENDER
// =====================================================

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
// TWO-WAY COMMAND / ACK PACKETS - MATCH MAIN V10.5A HOST
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

WaterWatchRemotePacket lastPacket = {};
bool havePacket = false;
unsigned long lastPacketMs = 0;
uint32_t lastSeq = 0;
int packetsReceived = 0;

const unsigned long LOST_SIGNAL_MS = 5000UL;
const unsigned long DRAW_INTERVAL_MS = 500UL;
unsigned long lastDrawMs = 0;
bool dashboardBaseDrawn = false;
bool dashboardDirty = false;
bool waitingScreenDrawn = false;
bool lastLostState = false;
uint32_t lastDrawnSeq = 0;

bool lowWaterAlarmLatched = false;
bool lowWaterAlarmAcknowledged = false;
bool lastLowWaterCondition = false;
unsigned long lastLowWaterWakeMs = 0;

const unsigned long DASHBOARD_MIN_REFRESH_MS = 1000UL;  // prevents constant AMOLED redraw flicker

bool remotePowerKeyDown = false;
unsigned long remotePowerKeyDownMs = 0;
const unsigned long POWER_KEY_TAP_MAX_MS = 1200UL;          // short press/tap toggles sleep/wake
const unsigned long POWER_KEY_SHUTDOWN_HOLD_MS = 3000UL;   // hold bottom power button about 3 seconds for full shutdown
const uint8_t REMOTE_AWAKE_BRIGHTNESS = 135;   // battery saver: still readable, much less AMOLED drain
const uint8_t REMOTE_SLEEP_BRIGHTNESS = 0;     // true black screen during sleep

uint8_t mainBroadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool commandPeerReady = false;
uint32_t nextCommandId = 1;
uint32_t pendingCommandId = 0;
uint8_t pendingCommandType = CMD_NONE;
unsigned long pendingCommandMs = 0;
uint8_t pendingSpinnerFrame = 0;
unsigned long lastPendingSpinnerMs = 0;
const unsigned long PENDING_SPINNER_INTERVAL_MS = 180UL;
String commandStatus = "";
uint16_t commandStatusColor = C_GRAY;
unsigned long commandStatusUntilMs = 0;

bool touchDown = false;
int touchStartX = 0;
int touchStartY = 0;
int touchLastX = 0;
int touchLastY = 0;
unsigned long touchStartMs = 0;
unsigned long touchLastSeenMs = 0;
unsigned long lastTapMs = 0;

void drawRemoteDashboard();
void drawWaitingScreen();
void markRemoteActivity();
void wakeRemoteScreen();
void checkRemotePowerTimers();
void saveLastRemotePacket();
void loadLastRemotePacket();
void enterRemoteDeepSleep();
void initRemoteBatteryGauge();
void readRemoteBattery();
void drawRemoteBatteryBadge();
void drawLiveStatusPage();
void toggleLiveStatusPage();
void refreshRemoteBatteryBadgeIfChanged();
bool isLowWaterConditionActive();
bool isLowWaterAlarmActive();
void acknowledgeLowWaterAlarm();
void updateLowWaterAlarmState();
void drawLowWaterAlarmOverlay(bool active);
void handleLowWaterAlarmWake();
void setupRemotePowerKeyWake();
void checkRemotePowerKeyWake();
void enterRemoteSleepScreen();
void wakeRemoteScreenFromPowerKey();
void shutdownRemoteDisplay();
void pluggedScreenOffMode();
void wakeFromPluggedScreenOff();
void drawPendingCommandButtons();
void updatePendingSpinner();
void drawRemoteUpdatePage();
void cycleRemotePage();
void handleRemoteUpdateTap(int x, int y);
String remoteOtaUploadPageHtml();
void setupRemoteOtaServer();
void startRemoteOtaMode();
void stopRemoteOtaModeAndReturn();
void handleRemoteOtaServer();
void initEspNowReceiver();
bool hitBoxRemote(int x, int y, int l, int t, int r, int b);

// =====================================================
// HELPER FUNCTIONS
// =====================================================

void centerText(const String &txt, int y, uint8_t size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color, C_BLACK);
  int16_t x1, y1;
  uint16_t w, h;
  gfx->getTextBounds(txt, 0, y, &x1, &y1, &w, &h);
  int x = (SCREEN_W - (int)w) / 2;
  gfx->setCursor(x, y);
  gfx->print(txt);
}

void centerTextInBox(const String &txt, int y, int boxX, int boxW, int boxH, uint8_t size, uint16_t color, uint16_t bg = C_BLACK) {
  gfx->fillRect(boxX, y - 2, boxW, boxH, bg);
  gfx->setTextSize(size);
  gfx->setTextColor(color, bg);
  int16_t x1, y1;
  uint16_t w, h;
  gfx->getTextBounds(txt, 0, y, &x1, &y1, &w, &h);
  int x = boxX + ((boxW - (int)w) / 2);
  if (x < boxX) x = boxX;
  gfx->setCursor(x, y);
  gfx->print(txt);
}

void drawLabelValue(int x, int y, const String &label, const String &value, uint16_t valueColor = C_WHITE) {
  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(x, y);
  gfx->print(label);
  gfx->setTextColor(valueColor, C_BLACK);
  gfx->setCursor(x, y + 16);
  gfx->print(value);
}

String pumpModeText(uint8_t mode, bool pumpOn) {
  // Sender enum: RELAY_AUTO=0, RELAY_ON=1, RELAY_OFF=2
  if (mode == 1) return pumpOn ? "ON" : "ON?";
  if (mode == 2) return "OFF";
  return pumpOn ? "AUTO ON" : "AUTO OFF";
}

String serviceName(int8_t idx) {
  if (idx == 0) return "ENGINE OIL";
  if (idx == 1) return "PUMP OIL";
  if (idx == 2) return "VAC OIL";
  if (idx == 3) return "GENERAL";
  return "SERVICE";
}

uint16_t percentColor(float pct) {
  if (!havePacket || !lastPacket.calibrated || !lastPacket.pressureOK) return C_PURPLE;
  if (pct <= 10.0) return C_RED;
  if (pct <= 25.0) return C_ORANGE;
  if (pct <= 50.0) return C_YELLOW;
  return C_GREEN;
}

String ageText(unsigned long ageMs) {
  if (ageMs < 1000) return "NOW";
  unsigned long sec = ageMs / 1000UL;
  if (sec < 60) return String(sec) + " SEC AGO";
  unsigned long min = sec / 60UL;
  return String(min) + " MIN AGO";
}



// =====================================================
// SENDER-MATCH DASHBOARD HELPERS
// These mirror the main Water Watch Pro dashboard visual layout.
// Remote buttons are display-only/read-only.
// =====================================================

const float REMOTE_LOW_ALERT_PERCENT = 25.0f;  // sender does not transmit user setting yet; visual fallback

// =====================================================
// LOW WATER ALARM
// =====================================================

bool isLowWaterConditionActive() {
  // V27 FINAL: water warning disabled by request.
  return false;
}

bool isLowWaterAlarmActive() {
  return isLowWaterConditionActive() && !lowWaterAlarmAcknowledged;
}

void acknowledgeLowWaterAlarm() {
  // One-and-done acknowledgement.
  // Once acknowledged, it stays quiet until water rises above low alert and later drops again.
  if (isLowWaterConditionActive()) {
    lowWaterAlarmAcknowledged = true;
    lowWaterAlarmLatched = false;
    lastUserActivityMs = millis();  // allow normal 30-second sleep after acknowledgement
    dashboardDirty = true;
  }
}

void updateLowWaterAlarmState() {
  bool lowNow = isLowWaterConditionActive();

  // Re-arm only after water recovers above low alert.
  if (!lowNow && lastLowWaterCondition) {
    lowWaterAlarmAcknowledged = false;
    lowWaterAlarmLatched = false;
  }

  lastLowWaterCondition = lowNow;
}

void handleLowWaterAlarmWake() {
  updateLowWaterAlarmState();

  if (!isLowWaterAlarmActive()) {
    return;
  }

  unsigned long now = millis();

  if (!lowWaterAlarmLatched || !screenAwake) {
    lowWaterAlarmLatched = true;
    lastLowWaterWakeMs = now;

    // Low water alarm wakes the screen and keeps it awake until acknowledged.
    screenAwake = true;
    gfx->setBrightness(REMOTE_AWAKE_BRIGHTNESS);
    lastUserActivityMs = now;
    dashboardBaseDrawn = false;
    dashboardDirty = true;
  }
}

void drawLowWaterAlarmOverlay(bool active) {
  // Solid warning only. No blinking. No animation redraw loop.
  const int x = 42;
  const int y = 300;
  const int w = SCREEN_W - 84;
  const int h = 30;

  if (!active) {
    gfx->fillRect(x - 2, y - 2, w + 4, h + 4, C_BLACK);
    return;
  }

  gfx->fillRoundRect(x, y, w, h, 12, C_RED);
  gfx->drawRoundRect(x, y, w, h, 12, C_WHITE);

  gfx->setTextSize(2);
  gfx->setTextColor(C_WHITE, C_RED);

  const char *msg = "LOW WATER!";
  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(msg, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - (int)tw) / 2, 308);
  gfx->print(msg);
}


const char *relayModeLabelRemote() {
  // Sender enum: RELAY_AUTO=0, RELAY_ON=1, RELAY_OFF=2
  if (lastPacket.relayMode == 1) return "ON";
  if (lastPacket.relayMode == 2) return "OFF";
  return "AUTO";
}

const char *remoteModeButtonLabel() {
  // Bottom-left button should show the real 12V output state in AUTO mode.
  if (lastPacket.relayMode == 1) return "ON";
  if (lastPacket.relayMode == 2) return "OFF";
  return lastPacket.pumpOn ? "AUTO ON" : "AUTO OFF";
}

uint16_t remoteModeButtonColor() {
  // Green when 12V/relay output is actually ON.
  // Orange/red when it is OFF or forced off.
  if (lastPacket.relayMode == 1) return C_GREEN;
  if (lastPacket.relayMode == 2) return C_RED;
  return lastPacket.pumpOn ? C_GREEN : C_ORANGE;
}

bool remoteModeButtonActive() {
  // Active fill means actual 12V output is ON.
  if (lastPacket.relayMode == 1) return true;
  if (lastPacket.relayMode == 2) return false;
  return lastPacket.pumpOn ? true : false;
}

const char *remoteActual12vText() {
  return lastPacket.pumpOn ? "12V RELAY SIGNAL: ON" : "12V RELAY SIGNAL: OFF";
}

uint16_t remoteActual12vColor() {
  return lastPacket.pumpOn ? C_GREEN : C_ORANGE;
}

bool pressureCalibratedRemote() {
  return havePacket && lastPacket.calibrated;
}

bool serviceDueRemote() {
  return havePacket && lastPacket.serviceDue;
}

bool serviceFlashPhaseRemote() {
  return ((millis() / 650UL) % 2) == 0;
}

String serviceNameRemote(int8_t idx) {
  if (idx == 0) return "ENGINE OIL";
  if (idx == 1) return "PUMP OIL";
  if (idx == 2) return "VAC OIL";
  if (idx == 3) return "GENERAL";
  return "SERVICE";
}

void serviceStatusTextRemote(char *out, size_t len) {
  if (!serviceDueRemote()) {
    snprintf(out, len, "SERVICE OK");
    return;
  }

  String s = serviceNameRemote(lastPacket.dueItem);
  snprintf(out, len, "%s", s.c_str());
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

void drawDashboardTopControlsRemote(bool lost) {
  // No full-width black wipe here. Each badge/card clears its own area only.
  drawRemoteBatteryBadge();

  const int topY = 30;

  gfx->fillRoundRect(22, topY, 148, 38, 16, C_CARD2);
  gfx->drawRoundRect(22, topY, 148, 38, 16, lost ? C_ORANGE : C_CYAN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_CARD2);
  gfx->setCursor(34, topY + 6);
  gfx->print("HOURS");

  char hrValue[18];
  snprintf(hrValue, sizeof(hrValue), "%.1f", havePacket ? lastPacket.totalHours : 0.0f);
  gfx->setTextSize(2);
  gfx->setTextColor(lost ? C_ORANGE : C_CYAN, C_CARD2);

  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(hrValue, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(22 + ((148 - tw) / 2), topY + 22);
  gfx->print(hrValue);

  drawButton(196, topY, 148, 38, lastPacket.auxOn ? "AUX ON" : "AUX OFF", lastPacket.auxOn ? C_GREEN : C_ORANGE, lastPacket.auxOn);
}

void drawDashboardServiceBarRemote() {
  char svcText[28];
  serviceStatusTextRemote(svcText, sizeof(svcText));

  bool due = serviceDueRemote();
  // Remote receiver does not flash the whole service bar; full-screen flashing looked like flicker.
  uint16_t accent = due ? C_RED : C_GREEN;
  uint16_t fill = C_CARD2;
  uint16_t textColor = accent;

  gfx->fillRoundRect(196, 398, 148, 38, 18, fill);
  gfx->drawRoundRect(196, 398, 148, 38, 18, accent);
  gfx->setTextSize(2);
  gfx->setTextColor(textColor, fill);

  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(svcText, 0, 0, &x1, &y1, &tw, &th);

  if (tw > 136) {
    gfx->setTextSize(1);
    gfx->getTextBounds(svcText, 0, 0, &x1, &y1, &tw, &th);
  }

  gfx->setCursor(196 + ((148 - tw) / 2), 398 + ((38 - th) / 2));
  gfx->print(svcText);
}

void drawGaugeRemote(float percent) {
  int cx = SCREEN_W / 2;
  int cy = 226;

  gfx->fillCircle(cx, cy, 62, C_BLACK);
  gfx->drawCircle(cx, cy, 64, C_CARD);
  gfx->drawCircle(cx, cy, 65, C_CARD);

  const int segments = 36;
  int active = (int)((segments * constrain(percent, 0.0f, 100.0f)) / 100.0f);

  for (int i = 0; i < segments; i++) {
    float deg = -215.0 + (i * (250.0 / (segments - 1)));
    float rad = deg * DEG_TO_RAD;

    int x1 = cx + cos(rad) * 72;
    int y1 = cy + sin(rad) * 72;
    int x2 = cx + cos(rad) * 88;
    int y2 = cy + sin(rad) * 88;

    uint16_t c = C_DIM;

    if (i < active) {
      if (!pressureCalibratedRemote()) c = C_ORANGE;
      else if (percent <= REMOTE_LOW_ALERT_PERCENT) c = C_RED;
      else if (i < 10) c = C_BLUE;
      else if (i < 29) c = C_GREEN;
      else c = C_ORANGE;
    }

    gfx->drawLine(x1, y1, x2, y2, c);
    gfx->drawLine(x1 + 1, y1, x2 + 1, y2, c);
    gfx->drawLine(x1 - 1, y1, x2 - 1, y2, c);
  }

  char pct[12];

  if (!lastPacket.pressureOK) {
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

void drawLevelBarRemote(float percent) {
  int x = 32;
  int y = 318;
  int w = SCREEN_W - 64;
  int h = 22;

  gfx->fillRoundRect(x, y, w, h, 11, C_CARD);
  gfx->drawRoundRect(x, y, w, h, 11, C_DIM);

  int fillW = (int)((w - 8) * constrain(percent, 0.0f, 100.0f) / 100.0f);
  fillW = constrain(fillW, 0, w - 8);

  uint16_t color = C_GREEN;
  if (!lastPacket.pressureOK) color = C_RED;
  else if (!pressureCalibratedRemote()) color = C_ORANGE;
  else if (percent <= REMOTE_LOW_ALERT_PERCENT) color = C_RED;
  else if (percent < 45) color = C_ORANGE;

  gfx->fillRoundRect(x + 4, y + 4, fillW, h - 8, 8, color);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(x, y + 31);
  gfx->print("EMPTY");

  gfx->setCursor(x + w - 28, y + 31);
  gfx->print("FULL");
}

void drawSenderMatchDashboard() {
  unsigned long now = millis();
  unsigned long ageMs = havePacket ? now - lastPacketMs : 999999UL;
  bool lost = havePacket && ageMs > LOST_SIGNAL_MS;
  updateLowWaterAlarmState();
  bool lowAlarm = isLowWaterAlarmActive() && !lost;

  // Critical no-flicker change:
  // Do NOT fill the whole screen every refresh. That causes the visible black blink.
  // Clear the screen only once when the dashboard first appears. Individual cards/areas
  // redraw themselves over their own backgrounds.
  if (!dashboardBaseDrawn) {
    gfx->fillScreen(C_BLACK);
    dashboardBaseDrawn = true;
  }

  gfx->startWrite();

  drawDashboardTopControlsRemote(lost);

  bool relayOn = lastPacket.pumpOn;

  char leftText[20];
  char tempText[20];
  char voltText[20];

  if (lastPacket.pressureOK) snprintf(leftText, sizeof(leftText), "%.2f G", lastPacket.gallonsLeft);
  else snprintf(leftText, sizeof(leftText), "ERR");

  if (lastPacket.tempOK) snprintf(tempText, sizeof(tempText), "%.0f F", lastPacket.tempF);
  else snprintf(tempText, sizeof(tempText), "ERR");

  snprintf(voltText, sizeof(voltText), "%.2f V", lastPacket.pressureV);

  drawSmallCard(22, 72, 100, 58, "LEFT", leftText, lastPacket.pressureOK ? C_CYAN : C_RED);
  drawSmallCard(134, 72, 100, 58, "TEMP", tempText, lastPacket.tempOK ? C_ORANGE : C_RED);
  drawSmallCard(246, 72, 100, 58, "PRES", voltText, lastPacket.pressureOK ? C_BLUE : C_RED);

  drawGaugeRemote(lastPacket.tankPercent);
  drawLevelBarRemote(lastPacket.tankPercent);
  drawLowWaterAlarmOverlay(lowAlarm);

  char modeText[28];
  if (lastPacket.relayMode == 0) {
    snprintf(modeText, sizeof(modeText), lastPacket.pumpOn ? "AUTO 12V: ON" : "AUTO 12V: OFF");
  } else {
    snprintf(modeText, sizeof(modeText), "MODE: %s", relayModeLabelRemote());
  }

  // Clear only the text status strip, not the whole screen.
  if (lowAlarm) {
    centerTextInBox("LOW WATER ALARM", 344, 0, SCREEN_W, 14, 1, C_RED);
    centerTextInBox("TAP SCREEN TO ACKNOWLEDGE", 362, 0, SCREEN_W, 14, 1, C_RED);
    centerTextInBox("WATER BELOW LOW ALERT", 380, 0, SCREEN_W, 14, 1, C_RED);
  } else if (lost) {
    centerTextInBox("OUT OF RANGE", 344, 0, SCREEN_W, 14, 1, C_ORANGE);
    centerTextInBox(modeText, 362, 0, SCREEN_W, 14, 1, C_ORANGE);
    centerTextInBox("SHOWING LAST DATA", 380, 0, SCREEN_W, 14, 1, C_ORANGE);
  } else if (lastPacket.relayMode == 1) {
    centerTextInBox("MANUAL OVERRIDE ON", 344, 0, SCREEN_W, 14, 1, C_GREEN);
    centerTextInBox(modeText, 362, 0, SCREEN_W, 14, 1, C_GREEN);
    centerTextInBox(remoteActual12vText(), 380, 0, SCREEN_W, 14, 1, remoteActual12vColor());
  } else if (lastPacket.relayMode == 2) {
    centerTextInBox("RELAY FORCED OFF", 344, 0, SCREEN_W, 14, 1, C_ORANGE);
    centerTextInBox(modeText, 362, 0, SCREEN_W, 14, 1, C_ORANGE);
    centerTextInBox(remoteActual12vText(), 380, 0, SCREEN_W, 14, 1, remoteActual12vColor());
  } else if (!lastPacket.tempOK) {
    centerTextInBox("AUTO: TEMP SENSOR FAULT", 344, 0, SCREEN_W, 14, 1, C_RED);
    centerTextInBox(modeText, 362, 0, SCREEN_W, 14, 1, C_RED);
    centerTextInBox(remoteActual12vText(), 380, 0, SCREEN_W, 14, 1, remoteActual12vColor());
  } else if (relayOn) {
    centerTextInBox("TEMP IN RANGE", 344, 0, SCREEN_W, 14, 1, C_GREEN);
    centerTextInBox(modeText, 362, 0, SCREEN_W, 14, 1, C_GREEN);
    centerTextInBox(remoteActual12vText(), 380, 0, SCREEN_W, 14, 1, remoteActual12vColor());
  } else {
    centerTextInBox("WAITING FOR TEMP RANGE", 344, 0, SCREEN_W, 14, 1, C_ORANGE);
    centerTextInBox(modeText, 362, 0, SCREEN_W, 14, 1, C_ORANGE);
    centerTextInBox(remoteActual12vText(), 380, 0, SCREEN_W, 14, 1, remoteActual12vColor());
  }

  if (commandStatus.length() > 0 && millis() < commandStatusUntilMs) {
    centerTextInBox(commandStatus, 380, 0, SCREEN_W, 14, 1, commandStatusColor);
  }

  uint16_t modeColor = remoteModeButtonColor();

  drawButton(24, 398, 148, 38, remoteModeButtonLabel(), modeColor, remoteModeButtonActive());
  drawDashboardServiceBarRemote();
  drawPendingCommandButtons();

  gfx->endWrite();

  lastLostState = lost;
  lastDrawnSeq = lastPacket.seq;
}






// =====================================================
// SIMPLE NON-BLOCKING POWER LOGOS - SAFE RECOVERY
// =====================================================

void drawSimplePowerLogo(const char *title, uint16_t accent) {
  gfx->fillScreen(C_BLACK);

  int cx = SCREEN_W / 2;
  int cy = 132;

  gfx->drawCircle(cx, cy, 48, accent);
  gfx->drawCircle(cx, cy, 47, accent);
  gfx->fillRect(cx - 4, cy - 58, 8, 40, C_BLACK);
  gfx->fillRect(cx - 3, cy - 54, 6, 34, accent);

  centerText(String("WATER WATCH"), 220, 3, C_CYAN);
  centerText(String(title), 266, 3, accent);
  centerText(String("REMOTE"), 316, 2, C_GRAY);
}

void showPowerUpLogoBrief() {
  drawSimplePowerLogo("POWER UP", C_GREEN);
  delay(700);
}


// =====================================================
// REMOTE BATTERY GAUGE
// =====================================================

int batteryPercentFromVoltageMv(int mv) {
  // Voltage-based Li-ion estimate for this remote.
  // AXP2101 percentage was proven unreliable on this battery.
  // Below ~3.45V, treat it as basically dead under ESP32 load.
  if (mv >= 4200) return 100;
  if (mv >= 4150) return 95;
  if (mv >= 4100) return 90;
  if (mv >= 4050) return 85;
  if (mv >= 4000) return 80;
  if (mv >= 3950) return 72;
  if (mv >= 3900) return 65;
  if (mv >= 3850) return 58;
  if (mv >= 3800) return 50;
  if (mv >= 3750) return 42;
  if (mv >= 3700) return 34;
  if (mv >= 3650) return 26;
  if (mv >= 3600) return 18;
  if (mv >= 3550) return 12;
  if (mv >= 3500) return 7;
  if (mv >= 3450) return 3;
  return 0;
}

void initRemoteBatteryGauge() {
  if (PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    remoteBatteryReady = true;
    USBSerial.println("AXP2101 battery gauge OK");

    // V39 CHARGE FIX:
    // Waveshare AXP2101 boards need charger/battery ADC setup after PMU init.
    // The TS pin temperature-measure function must be disabled on boards without
    // battery thermistor wiring, otherwise charging can behave abnormally.
    PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
    PMU.setChargeTargetVoltage(XPOWERS_AXP192_CHG_VOL_4V2);
    PMU.setSysPowerDownVoltage(2600);
    PMU.disableTSPinMeasure();
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();

    USBSerial.println("AXP2101 charger configured: 500mA / 4.2V target / TS disabled");
    setupRemotePowerKeyWake();
  } else {
    remoteBatteryReady = false;
    USBSerial.println("AXP2101 battery gauge failed");
  }

  readRemoteBattery();
}

void readRemoteBattery() {
  if (!remoteBatteryReady) {
    remoteBatteryPercent = -1;
    remoteBatteryMv = -1;
    remoteBatteryCharging = false;
    remoteUsbPresent = false;
    return;
  }

  remoteBatteryCharging = PMU.isCharging();
  remoteUsbPresent = PMU.isVbusIn();

  int mv = -1;
  int pct = -1;

  // V29 FIX:
  // Do NOT require PMU.isBatteryConnect() to be true.
  // Some AXP2101 boards report battery-connect weirdly, which caused BAT --%.
  // Voltage is the source of truth.
  mv = PMU.getBattVoltage();

  if (mv > 3000 && mv < 5000) {
    pct = batteryPercentFromVoltageMv(mv);
  } else {
    // Emergency fallback only so the badge never disappears.
    // This is NOT trusted for final battery truth.
    int axpPct = PMU.getBatteryPercent();
    if (axpPct >= 0 && axpPct <= 100) pct = axpPct;
  }

  remoteBatteryMv = mv;

  if (pct >= 0 && pct <= 100) remoteBatteryPercent = pct;
  else remoteBatteryPercent = -1;
}

void drawRemoteBatteryBadge() {
  // V35: remove live voltage row to stop overlap/blinking.
  // One clean big battery status line only.
  const int w = 190;
  const int h = 30;
  const int x = (SCREEN_W - w) / 2;
  const int y = 2;

  uint16_t c = C_GRAY;
  char line1[28];

  if (remoteBatteryMv > 3000 && remoteBatteryMv < 5000) {
    if (remoteBatteryMv >= 4100) {
      snprintf(line1, sizeof(line1), "BAT FULL");
      c = C_GREEN;
    } else if (remoteBatteryMv >= 3800) {
      snprintf(line1, sizeof(line1), "BAT GOOD");
      c = C_GREEN;
    } else if (remoteBatteryMv >= 3650) {
      snprintf(line1, sizeof(line1), "BAT LOW");
      c = C_ORANGE;
    } else if (remoteBatteryMv >= 3500) {
      snprintf(line1, sizeof(line1), "BAT CRIT");
      c = C_RED;
    } else {
      snprintf(line1, sizeof(line1), "BAT DEAD");
      c = C_RED;
    }

    if (remoteBatteryCharging || remoteUsbPresent) {
      strncat(line1, " +", sizeof(line1) - strlen(line1) - 1);
    }
  } else {
    snprintf(line1, sizeof(line1), "BAT UNKNOWN");
  }

  gfx->fillRoundRect(x, y, w, h, 12, C_BLACK);
  gfx->drawRoundRect(x, y, w, h, 12, c);

  int16_t x1, y1;
  uint16_t tw, th;

  gfx->setTextSize(2);
  gfx->setTextColor(c, C_BLACK);
  gfx->getTextBounds(line1, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(x + ((w - tw) / 2), y + 8);
  gfx->print(line1);

  lastDrawnBatteryPercent = remoteBatteryPercent;
  lastDrawnBatteryMv = remoteBatteryMv;
  lastDrawnBatteryCharging = remoteBatteryCharging;
  lastDrawnUsbPresent = remoteUsbPresent;
}



void drawLiveStatusPage() {
  if (pluggedScreenOff) return;

  readRemoteBattery();
  liveStatusDirty = false;
  lastLiveStatusDrawMs = millis();

  gfx->fillScreen(C_BLACK);

  centerText("LIVE STATUS", 18, 3, C_CYAN);

  if (!havePacket) {
    centerText("WAITING FOR DATA", 100, 3, C_ORANGE);

    // Still show battery voltage even if no ESP-NOW packet yet.
    gfx->setTextSize(2);
    gfx->setTextColor(C_GRAY, C_BLACK);
    gfx->setCursor(34, 300);
    gfx->print("BATTERY VOLTAGE");

    char bbuf[40];
    if (remoteBatteryMv > 3000 && remoteBatteryMv < 5000) {
      snprintf(bbuf, sizeof(bbuf), "%dmV %s", remoteBatteryMv,
               remoteBatteryCharging ? "CHG" : (remoteUsbPresent ? "USB" : "BAT"));
    } else {
      snprintf(bbuf, sizeof(bbuf), "----mV %s",
               remoteBatteryCharging ? "CHG" : (remoteUsbPresent ? "USB" : "BAT"));
    }

    gfx->setTextSize(3);
    gfx->setTextColor(C_WHITE, C_BLACK);
    gfx->setCursor(34, 330);
    gfx->print(bbuf);

    centerText("Swipe to return", 410, 1, C_GRAY);
    return;
  }

  char buf[48];

  gfx->setTextSize(2);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(34, 78);
  gfx->print("WATER LEVEL");

  gfx->setTextSize(5);
  gfx->setTextColor(C_CYAN, C_BLACK);
  snprintf(buf, sizeof(buf), "%.0f%%", lastPacket.tankPercent);
  gfx->setCursor(34, 108);
  gfx->print(buf);

  gfx->setTextSize(2);
  gfx->setTextColor(C_WHITE, C_BLACK);
  snprintf(buf, sizeof(buf), "%.1f GAL LEFT", lastPacket.gallonsLeft);
  gfx->setCursor(34, 178);
  gfx->print(buf);

  gfx->setTextSize(2);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(34, 238);
  gfx->print("TEMP");

  gfx->setTextSize(5);
  gfx->setTextColor(lastPacket.tempOK ? C_GREEN : C_RED, C_BLACK);
  if (lastPacket.tempOK) {
    snprintf(buf, sizeof(buf), "%.1fF", lastPacket.tempF);
  } else {
    snprintf(buf, sizeof(buf), "FAULT");
  }
  gfx->setCursor(34, 268);
  gfx->print(buf);

  gfx->setTextSize(2);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(34, 350);
  gfx->print("BATTERY VOLTAGE");

  uint16_t bc = C_WHITE;
  if (remoteBatteryMv > 3000 && remoteBatteryMv < 5000) {
    bc = remoteBatteryMv <= 3500 ? C_RED : (remoteBatteryMv <= 3650 ? C_ORANGE : C_GREEN);
    snprintf(buf, sizeof(buf), "%dmV %s", remoteBatteryMv,
             remoteBatteryCharging ? "CHG" : (remoteUsbPresent ? "USB" : "BAT"));
  } else {
    snprintf(buf, sizeof(buf), "----mV %s",
             remoteBatteryCharging ? "CHG" : (remoteUsbPresent ? "USB" : "BAT"));
  }

  gfx->setTextSize(3);
  gfx->setTextColor(bc, C_BLACK);
  gfx->setCursor(34, 380);
  gfx->print(buf);

  centerText("Swipe to return", 430, 1, C_GRAY);
}

void toggleLiveStatusPage() {
  if (pluggedScreenOff) return;

  liveStatusPage = !liveStatusPage;
  liveStatusDirty = true;
  lastLiveStatusDrawMs = 0;
  markRemoteActivity();

  if (liveStatusPage) {
    drawLiveStatusPage();
  } else {
    dashboardBaseDrawn = false;
    dashboardDirty = true;
    drawRemoteDashboard();
  }
}

void refreshRemoteBatteryBadgeIfChanged() {
  readRemoteBattery();

  if (remoteBatteryPercent != lastDrawnBatteryPercent ||
      remoteBatteryMv != lastDrawnBatteryMv ||
      remoteBatteryCharging != lastDrawnBatteryCharging ||
      remoteUsbPresent != lastDrawnUsbPresent) {
    drawRemoteBatteryBadge();
  }
}


// =====================================================
// AXP2101 PHYSICAL POWER KEY WAKE
// =====================================================

void setupRemotePowerKeyWake() {
  if (!remoteBatteryReady) return;

  // Use the AXP2101 PEK / physical power-key events.
  // No GPIO guessing. This reads the PMU power button event over I2C.
  PMU.clearIrqStatus();
  PMU.enableIRQ(XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ);
}

void enterRemoteSleepScreen() {
  // V27 FINAL: sleep mode is disabled. Any sleep request becomes full shutdown.
  shutdownRemoteDisplay();
}

void wakeRemoteScreenFromPowerKey() {
  screenAwake = true;
  gfx->setBrightness(REMOTE_AWAKE_BRIGHTNESS);
  lastUserActivityMs = millis();
  dashboardBaseDrawn = false;
  dashboardDirty = true;
}


void shutdownRemoteDisplay() {
  // Full firmware-controlled shutdown through AXP2101.
  // After this call, firmware is no longer running.
  // Turn back on with the physical board power button or USB power.
  saveLastRemotePacket();

  drawSimplePowerLogo("POWER DOWN", C_RED);
  delay(2000);

  gfx->setBrightness(0);
  delay(80);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(80);

  USBSerial.println("AXP2101 full shutdown requested");

  PMU.clearIrqStatus();
  delay(30);

  PMU.shutdown();

  // If shutdown does not happen for any reason, fall back to ESP32 deep sleep.
  delay(500);
  esp_deep_sleep_start();
}



void pluggedScreenOffMode() {
  // V34:
  // Plugged in screen-off charge mode.
  // This does NOT shut the board down. USB stays alive. Charging continues.
  pluggedScreenOff = true;
  liveStatusPage = false;
  screenAwake = false;

  gfx->fillScreen(C_BLACK);
  gfx->setBrightness(0);
}

void wakeFromPluggedScreenOff() {
  pluggedScreenOff = false;
  screenAwake = true;
  liveStatusPage = false;
  liveStatusDirty = true;
  dashboardBaseDrawn = false;
  dashboardDirty = true;
  waitingScreenDrawn = false;

  gfx->setBrightness(REMOTE_AWAKE_BRIGHTNESS);
  lastUserActivityMs = millis();

  if (havePacket) {
    drawRemoteDashboard();
  } else {
    drawWaitingScreen();
  }
}


void checkRemotePowerKeyWake() {
  if (!remoteBatteryReady) return;

  uint32_t status = PMU.getIrqStatus();
  if (status == 0) return;

  if (PMU.isPekeyNegativeIrq()) {
    remotePowerKeyDown = true;
    remotePowerKeyDownMs = millis();
  }

  if (PMU.isPekeyPositiveIrq()) {
    if (remotePowerKeyDown) {
      readRemoteBattery();

      if (remoteUsbPresent || remoteBatteryCharging) {
        // Plugged in:
        // Bottom power button tap toggles screen off/on for charging.
        // No full shutdown while USB/charging is present.
        if (pluggedScreenOff || !screenAwake) {
          wakeFromPluggedScreenOff();
        } else {
          pluggedScreenOffMode();
        }
      } else {
        // Battery only:
        // Bottom power button tap while ON = full shutdown.
        if (screenAwake) {
          shutdownRemoteDisplay();
        }
      }
    }

    remotePowerKeyDown = false;
  }

  PMU.clearIrqStatus();
}









// =====================================================
// LAST DATA MEMORY + SCREEN / DEVICE POWER
// =====================================================

void saveLastRemotePacket() {
  if (!havePacket) return;

  unsigned long now = millis();
  if (now - lastPacketSaveMs < SAVE_PACKET_INTERVAL_MS) return;
  lastPacketSaveMs = now;

  remotePrefs.begin("wwp_remote", false);
  remotePrefs.putBool("hasPacket", true);
  remotePrefs.putBytes("lastPacket", &lastPacket, sizeof(lastPacket));
  remotePrefs.putUInt("lastSeq", lastPacket.seq);
  remotePrefs.end();
}

void loadLastRemotePacket() {
  remotePrefs.begin("wwp_remote", true);
  bool hasSaved = remotePrefs.getBool("hasPacket", false);

  if (hasSaved) {
    size_t got = remotePrefs.getBytes("lastPacket", &lastPacket, sizeof(lastPacket));
    if (got == sizeof(lastPacket) && lastPacket.magic == 0x57A7 && lastPacket.version == 1) {
      havePacket = true;
      loadedSavedPacket = true;
      lastPacketMs = millis() - (LOST_SIGNAL_MS + 1000UL);  // show as out of range until live packet arrives
      lastSeq = lastPacket.seq;
      lastDrawnSeq = 0;
      dashboardDirty = true;
      waitingScreenDrawn = true;
    }
  }

  remotePrefs.end();
}

void wakeRemoteScreen() {
  if (!screenAwake) {
    screenAwake = true;
    gfx->setBrightness(REMOTE_AWAKE_BRIGHTNESS);
    dashboardBaseDrawn = false;
    dashboardDirty = true;
  }
}

void markRemoteActivity() {
  // Touch counts as activity ONLY while the screen is already awake.
  // It must NOT wake the display from sleep.
  if (!screenAwake) return;
  lastUserActivityMs = millis();
}

void checkRemotePowerTimers() {
  if (remoteOtaActive) {
    lastUserActivityMs = millis();
    return;
  }

  unsigned long now = millis();

  readRemoteBattery();

  // Plugged in:
  // Never full-shutdown while USB/charging is present.
  // If screen was manually turned off for charging, keep it off.
  if (remoteUsbPresent || remoteBatteryCharging) {
    lastUserActivityMs = now;
    return;
  }

  // Battery only:
  // There is no screen-only sleep on battery. If USB is removed while the remote
  // is in plugged-in screen-off charge mode, perform a real controlled shutdown.
  if (pluggedScreenOff || !screenAwake) {
    pluggedScreenOff = false;
    screenAwake = true;
    gfx->setBrightness(REMOTE_AWAKE_BRIGHTNESS);
    shutdownRemoteDisplay();
    return;
  }

  // Battery only:
  // No dim-only mode. 30 seconds idle = full AXP2101 shutdown.
  if (now - lastUserActivityMs >= SCREEN_SLEEP_MS) {
    shutdownRemoteDisplay();
  }
}

void enterRemoteDeepSleep() {
  saveLastRemotePacket();

  drawSimplePowerLogo("POWER DOWN", C_RED);
  delay(700);

  gfx->setBrightness(0);
  delay(50);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(50);

  USBSerial.println("Remote deep sleep - no activity for 2 hours");
  esp_deep_sleep_start();
}


// =====================================================
// BUTTON PENDING / SPINNER FEEDBACK
// =====================================================

const char *spinnerGlyphRemote() {
  switch (pendingSpinnerFrame & 0x03) {
    case 0: return "|";
    case 1: return "/";
    case 2: return "-";
    default: return "\\";
  }
}

void drawSpinnerButtonRemote(int x, int y, int w, int h, const String &label, uint16_t color) {
  gfx->fillRoundRect(x, y, w, h, 18, C_CARD2);
  gfx->drawRoundRect(x, y, w, h, 18, color);

  gfx->setTextSize(2);
  gfx->setTextColor(color, C_CARD2);

  String txt = String(spinnerGlyphRemote()) + " " + label;

  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(txt, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(x + ((w - (int)tw) / 2), y + ((h - (int)th) / 2));
  gfx->print(txt);
}

void drawPendingCommandButtons() {
  if (pendingCommandId == 0) return;

  if (pendingCommandType == CMD_AUX_TOGGLE) {
    drawSpinnerButtonRemote(196, 30, 148, 38, "SENDING", C_CYAN);
  } else if (pendingCommandType == CMD_RELAY_MODE_CYCLE) {
    drawSpinnerButtonRemote(24, 398, 148, 38, "SENDING", C_CYAN);
  }
}

void updatePendingSpinner() {
  if (pendingCommandId == 0) return;
  if (!screenAwake) return;

  unsigned long now = millis();
  if (now - lastPendingSpinnerMs < PENDING_SPINNER_INTERVAL_MS) return;

  lastPendingSpinnerMs = now;
  pendingSpinnerFrame++;
  drawPendingCommandButtons();
}



// =====================================================
// REMOTE OTA UPDATE MODE
// =====================================================

void drawRemoteUpdatePage() {
  if (pluggedScreenOff) return;

  readRemoteBattery();
  gfx->fillScreen(C_BLACK);

  centerText("UPDATE MODE", 18, 3, C_CYAN);
  centerText("REMOTE DISPLAY", 54, 2, C_WHITE);
  centerText(REMOTE_FIRMWARE_VERSION, 78, 1, C_ORANGE);

  if (!remoteOtaActive) {
    centerText("Wireless firmware update", 96, 1, C_GRAY);
    centerText("Plug USB in before updating", 114, 1, C_ORANGE);

    gfx->fillRoundRect(44, 170, 280, 64, 20, C_CARD2);
    gfx->drawRoundRect(44, 170, 280, 64, 20, C_GREEN);
    centerText("START UPDATE", 190, 2, C_GREEN);

    centerText("Swipe to leave", 330, 1, C_GRAY);
    centerText(remoteOtaStatus, 360, 1, remoteOtaStatusColor);
    return;
  }

  centerText("HOTSPOT ACTIVE", 88, 2, C_GREEN);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(30, 132);
  gfx->print("WIFI NAME");
  gfx->setTextSize(2);
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(30, 150);
  gfx->print(REMOTE_OTA_AP_SSID);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(30, 196);
  gfx->print("PASSWORD");
  gfx->setTextSize(2);
  gfx->setTextColor(C_WHITE, C_BLACK);
  gfx->setCursor(30, 214);
  gfx->print(REMOTE_OTA_AP_PASS);

  gfx->setTextSize(1);
  gfx->setTextColor(C_GRAY, C_BLACK);
  gfx->setCursor(30, 260);
  gfx->print("BROWSER");
  gfx->setTextSize(3);
  gfx->setTextColor(C_CYAN, C_BLACK);
  gfx->setCursor(30, 282);
  gfx->print("192.168.4.1");

  centerText(remoteOtaStatus, 350, 1, remoteOtaStatusColor);
  centerText("Do not power off", 394, 1, C_RED);
  centerText("Upload REMOTE .bin only", 414, 1, C_ORANGE);
}

void cycleRemotePage() {
  if (pluggedScreenOff || remoteOtaActive) return;

  markRemoteActivity();

  if (!liveStatusPage && !updateModePage) {
    liveStatusPage = true;
    updateModePage = false;
    liveStatusDirty = true;
    lastLiveStatusDrawMs = 0;
    drawLiveStatusPage();
    return;
  }

  if (liveStatusPage) {
    liveStatusPage = false;
    updateModePage = true;
    drawRemoteUpdatePage();
    return;
  }

  updateModePage = false;
  liveStatusPage = false;
  dashboardBaseDrawn = false;
  dashboardDirty = true;
  drawRemoteDashboard();
}

void handleRemoteUpdateTap(int x, int y) {
  markRemoteActivity();
  if (remoteOtaActive) return;

  if (hitBoxRemote(x, y, 44, 170, 324, 234)) {
    startRemoteOtaMode();
    return;
  }

  remoteOtaStatus = "TAP START UPDATE";
  remoteOtaStatusColor = C_GRAY;
  drawRemoteUpdatePage();
}

String remoteOtaUploadPageHtml() {
  String html;
  html.reserve(1800);
  html += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Water Watch Pro Remote Update</title>";
  html += "<style>body{font-family:Arial;background:#061018;color:#fff;padding:20px;}";
  html += "h1{color:#00e5ff}.box{max-width:520px;margin:auto;background:#111;padding:20px;border-radius:14px;}";
  html += "input,button{font-size:18px;margin-top:12px;width:100%;padding:12px;border-radius:10px;}";
  html += "button{background:#00c853;color:#000;font-weight:bold;border:0}.warn{color:#ffb300}.bad{color:#ff5252}</style></head><body><div class='box'>";
  html += "<h1>Water Watch Pro Remote</h1><h2>Firmware Update</h2>";
  html += "<p class='warn'>Upload REMOTE firmware .bin only.</p>";
  html += "<p class='bad'>Do not power off during update.</p>";
  html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
  html += "<input type='file' name='firmware' accept='.bin' required>";
  html += "<button type='submit'>Upload Firmware</button></form>";
  html += "</div></body></html>";
  return html;
}

void setupRemoteOtaServer() {
  remoteUpdateServer.on("/", HTTP_GET, []() {
    remoteUpdateServer.send(200, "text/html", remoteOtaUploadPageHtml());
  });

  remoteUpdateServer.on("/update", HTTP_POST, []() {
    bool ok = !Update.hasError();
    String msg = ok ? "Update OK. Rebooting remote..." : "Update FAILED.";
    remoteUpdateServer.send(ok ? 200 : 500, "text/plain", msg);
    delay(900);
    if (ok) ESP.restart();
  }, []() {
    HTTPUpload &upload = remoteUpdateServer.upload();

    if (upload.status == UPLOAD_FILE_START) {
      remoteOtaStatus = "UPLOAD STARTED";
      remoteOtaStatusColor = C_ORANGE;
      drawRemoteUpdatePage();
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        remoteOtaStatus = "UPDATE BEGIN FAILED";
        remoteOtaStatusColor = C_RED;
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        remoteOtaStatus = "WRITE FAILED";
        remoteOtaStatusColor = C_RED;
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        remoteOtaStatus = "UPDATE OK - REBOOTING";
        remoteOtaStatusColor = C_GREEN;
      } else {
        remoteOtaStatus = "UPDATE END FAILED";
        remoteOtaStatusColor = C_RED;
      }
      drawRemoteUpdatePage();
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      Update.abort();
      remoteOtaStatus = "UPLOAD ABORTED";
      remoteOtaStatusColor = C_RED;
      drawRemoteUpdatePage();
    }
  });
}

void startRemoteOtaMode() {
  if (remoteOtaActive) return;

  readRemoteBattery();

  if (!remoteUsbPresent && !remoteBatteryCharging) {
    remoteOtaStatus = "PLUG USB IN FIRST";
    remoteOtaStatusColor = C_RED;
    drawRemoteUpdatePage();
    return;
  }

  saveLastRemotePacket();
  remoteOtaStatus = "STARTING HOTSPOT";
  remoteOtaStatusColor = C_ORANGE;
  drawRemoteUpdatePage();

  esp_now_deinit();
  delay(100);
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  delay(100);

  bool apOk = WiFi.softAP(REMOTE_OTA_AP_SSID, REMOTE_OTA_AP_PASS);
  if (!apOk) {
    remoteOtaStatus = "HOTSPOT FAILED";
    remoteOtaStatusColor = C_RED;
    drawRemoteUpdatePage();
    return;
  }

  setupRemoteOtaServer();
  remoteUpdateServer.begin();

  remoteOtaActive = true;
  remoteOtaStartedMs = millis();
  remoteOtaStatus = "CONNECT PHONE NOW";
  remoteOtaStatusColor = C_GREEN;
  updateModePage = true;
  liveStatusPage = false;
  screenAwake = true;
  pluggedScreenOff = false;
  gfx->setBrightness(REMOTE_AWAKE_BRIGHTNESS);
  lastUserActivityMs = millis();
  drawRemoteUpdatePage();
}

void stopRemoteOtaModeAndReturn() {
  if (remoteOtaActive) {
    remoteUpdateServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    remoteOtaActive = false;
    delay(100);
    initEspNowReceiver();
  }

  updateModePage = false;
  liveStatusPage = false;
  dashboardBaseDrawn = false;
  dashboardDirty = true;
  remoteOtaStatus = "READY";
  remoteOtaStatusColor = C_GRAY;

  if (havePacket) drawRemoteDashboard();
  else drawWaitingScreen();
}

void handleRemoteOtaServer() {
  if (!remoteOtaActive) return;

  lastUserActivityMs = millis();
  remoteUpdateServer.handleClient();

  if (millis() - remoteOtaStartedMs > REMOTE_OTA_TIMEOUT_MS) {
    remoteOtaStatus = "UPDATE TIMEOUT";
    remoteOtaStatusColor = C_RED;
    drawRemoteUpdatePage();
    delay(900);
    stopRemoteOtaModeAndReturn();
  }
}

// =====================================================
// TWO-WAY COMMAND HELPERS
// =====================================================

bool hitBoxRemote(int x, int y, int l, int t, int r, int b) {
  return x >= l && x <= r && y >= t && y <= b;
}

void setCommandStatusRemote(const String &msg, uint16_t color, unsigned long holdMs = 2000UL) {
  commandStatus = msg;
  commandStatusColor = color;
  commandStatusUntilMs = millis() + holdMs;
  dashboardDirty = true;
}

bool sendRemoteCommand(uint8_t command) {
  unsigned long now = millis();
  bool lost = !havePacket || ((now - lastPacketMs) > LOST_SIGNAL_MS);

  if (lost) {
    setCommandStatusRemote("NO SIGNAL - COMMAND BLOCKED", C_RED);
    return false;
  }

  if (!commandPeerReady) {
    setCommandStatusRemote("COMMAND RADIO NOT READY", C_RED);
    return false;
  }

  if (pendingCommandId != 0 && (now - pendingCommandMs) < 3000UL) {
    setCommandStatusRemote("WAITING FOR MAIN CONFIRM", C_ORANGE);
    return false;
  }

  WaterWatchCommandPacket cmd = {};
  cmd.magic = WWP_COMMAND_MAGIC;
  cmd.version = WWP_COMMAND_VERSION;
  cmd.command = command;
  cmd.commandId = nextCommandId++;
  cmd.senderMs = now;

  esp_err_t result = esp_now_send(mainBroadcastAddress, (uint8_t *)&cmd, sizeof(cmd));

  if (result == ESP_OK) {
    pendingCommandId = cmd.commandId;
    pendingCommandType = command;
    pendingCommandMs = now;
    pendingSpinnerFrame = 0;
    lastPendingSpinnerMs = 0;

    if (command == CMD_AUX_TOGGLE) setCommandStatusRemote("AUX REQUEST SENT", C_ORANGE);
    else if (command == CMD_RELAY_MODE_CYCLE) setCommandStatusRemote("MODE REQUEST SENT", C_ORANGE);
    else setCommandStatusRemote("COMMAND SENT", C_ORANGE);

    drawPendingCommandButtons();

    return true;
  }

  setCommandStatusRemote("COMMAND SEND FAILED", C_RED);
  return false;
}

void handleAckPacketSimple(uint8_t command, uint8_t accepted, uint32_t commandId, uint8_t ackAuxOn, uint8_t ackRelayMode, uint8_t ackPumpOn) {
  if (pendingCommandId != 0 && commandId == pendingCommandId) {
    if (accepted) {
      setCommandStatusRemote("MAIN CONFIRMED", C_GREEN);

      lastPacket.auxOn = ackAuxOn;
      lastPacket.relayMode = ackRelayMode;
      lastPacket.pumpOn = ackPumpOn;
    } else {
      setCommandStatusRemote("MAIN REJECTED COMMAND", C_RED);
    }

    pendingCommandId = 0;
    pendingCommandType = CMD_NONE;
    dashboardDirty = true;
  }
}

void handleRemoteTap(int x, int y) {
  markRemoteActivity();
  unsigned long now = millis();
  if (now - lastTapMs < 350UL) return;
  lastTapMs = now;

  USBSerial.print("REMOTE TAP X=");
  USBSerial.print(x);
  USBSerial.print(" Y=");
  USBSerial.println(y);

  if (isLowWaterAlarmActive()) {
    acknowledgeLowWaterAlarm();
    setCommandStatusRemote("LOW WATER ACKNOWLEDGED", C_GREEN, 2500UL);
    return;
  }

  // Same dashboard layout:
  // Top-right AUX button.
  if (hitBoxRemote(x, y, 196, 30, 344, 68)) {
    sendRemoteCommand(CMD_AUX_TOGGLE);
    return;
  }

  // Bottom-left relay mode button.
  if (hitBoxRemote(x, y, 24, 398, 172, 438)) {
    sendRemoteCommand(CMD_RELAY_MODE_CYCLE);
    return;
  }

  setCommandStatusRemote("TAP AUX OR MODE", C_GRAY, 1200UL);
}

void finishRemoteTouchAsTap() {
  if (!touchDown) return;

  int dx = touchLastX - touchStartX;
  int dy = touchLastY - touchStartY;
  unsigned long held = millis() - touchStartMs;

  // Horizontal swipe cycles dashboard -> live status -> update mode -> dashboard.
  if (abs(dx) >= 80 && abs(dx) > abs(dy) * 2 && held <= 1500UL) {
    cycleRemotePage();
    touchDown = false;
    return;
  }

  if (abs(dx) <= 35 && abs(dy) <= 35 && held <= 1200UL) {
    if (updateModePage) {
      handleRemoteUpdateTap(touchStartX, touchStartY);
    } else if (!liveStatusPage) {
      handleRemoteTap(touchStartX, touchStartY);
    }
  }

  touchDown = false;
}


void checkRemoteTouch() {
  // Screen asleep means touch is ignored. Only the physical board/power button hardware can wake it.
  if (!screenAwake) {
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
      markRemoteActivity();
      if (!touchDown) {
        touchDown = true;
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
    } else {
      finishRemoteTouchAsTap();
    }
  }

  if (touchDown && (millis() - touchLastSeenMs > 180UL)) {
    finishRemoteTouchAsTap();
  }
}

void initRemoteTouch() {
  if (FT3168->begin() == false) {
    USBSerial.println("Remote touch init failed");
    setCommandStatusRemote("TOUCH INIT FAILED", C_RED, 4000UL);
  } else {
    USBSerial.println("Remote touch init OK");
    USBSerial.printf("Touch ID: %#X\n", (int32_t)FT3168->IIC_Read_Device_ID());
  }
}


// =====================================================
// ESP-NOW RECEIVE CALLBACK
// Works with ESP32 Arduino core 3.x.
// If your Arduino core is older and this callback signature errors,
// update ESP32 boards package or tell me and I will send the legacy callback version.
// =====================================================

void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(WaterWatchRemotePacket)) {
    WaterWatchRemotePacket pkt;
    memcpy(&pkt, incomingData, sizeof(pkt));

    if (pkt.magic != 0x57A7 || pkt.version != 1) {
      return;
    }

    lastPacket = pkt;
    havePacket = true;
    loadedSavedPacket = false;
    lastPacketMs = millis();
    lastSeq = pkt.seq;
    packetsReceived++;
    dashboardDirty = true;
    liveStatusDirty = true;
    waitingScreenDrawn = false;
    saveLastRemotePacket();
    handleLowWaterAlarmWake();
    return;
  }

  if (len == sizeof(WaterWatchAckPacket)) {
    WaterWatchAckPacket ack;
    memcpy(&ack, incomingData, sizeof(ack));

    if (ack.magic == WWP_ACK_MAGIC && ack.version == WWP_ACK_VERSION) {
      handleAckPacketSimple(ack.command, ack.accepted, ack.commandId, ack.auxOn, ack.relayMode, ack.pumpOn);
    }
    return;
  }
}

void initEspNowReceiver() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);  // remote only needs short command bursts; saves power
  WiFi.setSleep(false);                // keep ESP-NOW receive reliable
  WiFi.disconnect();
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  if (esp_now_init() != ESP_OK) {
    USBSerial.println("ESP-NOW receiver init failed");
    return;
  }

  esp_now_register_recv_cb(onEspNowReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mainBroadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(mainBroadcastAddress)) {
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      commandPeerReady = true;
    } else {
      commandPeerReady = false;
      USBSerial.println("ESP-NOW command peer add failed");
    }
  } else {
    commandPeerReady = true;
  }

  USBSerial.println("ESP-NOW remote receiver two-way ready");
  USBSerial.print("Remote display MAC: ");
  USBSerial.println(WiFi.macAddress());
}

// =====================================================
// DISPLAY DRAWING
// =====================================================

void drawWaitingScreen() {
  if (pluggedScreenOff) return;
  if (waitingScreenDrawn) return;
  waitingScreenDrawn = true;
  gfx->fillScreen(C_BLACK);
  centerText("WATER WATCH", 40, 3, C_CYAN);
  centerText("REMOTE", 82, 3, C_WHITE);

  centerText("NO SAVED DATA", 160, 2, C_ORANGE);
  centerText("YET", 190, 2, C_ORANGE);

  centerText("Last dashboard will", 270, 1, C_GRAY);
  centerText("show after first live packet", 286, 1, C_GRAY);
}

void drawRemoteDashboard() {
  if (updateModePage) { drawRemoteUpdatePage(); return; }
  if (liveStatusPage) { drawLiveStatusPage(); return; }
  if (pluggedScreenOff) return;
  drawSenderMatchDashboard();
}

// =====================================================
// SETUP / LOOP
// =====================================================

void setupDisplayHardware() {
  Wire.begin(IIC_SDA, IIC_SCL);

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

  gfx->setBrightness(REMOTE_AWAKE_BRIGHTNESS);
  gfx->fillScreen(C_BLACK);
}

void setup() {
  USBSerial.begin(115200);
  USBSerial.println("Water Watch Pro 1.8 Remote Receiver - V40 OTA FULL BASE");

  setupDisplayHardware();
  showPowerUpLogoBrief();

  initRemoteBatteryGauge();
  initRemoteTouch();

  lastUserActivityMs = millis();
  loadLastRemotePacket();

  if (havePacket) {
    drawRemoteDashboard();
  } else {
    drawWaitingScreen();
  }

  initEspNowReceiver();
}

void loop() {
  unsigned long now = millis();
  checkRemotePowerKeyWake();
  checkRemoteTouch();

  if (remoteOtaActive) {
    handleRemoteOtaServer();
    return;
  }

  checkRemotePowerTimers();

  if (liveStatusPage && screenAwake && !pluggedScreenOff &&
      (liveStatusDirty || (now - lastLiveStatusDrawMs >= LIVE_STATUS_REFRESH_MS))) {
    drawLiveStatusPage();
  }

  updatePendingSpinner();

  if (screenAwake && !pluggedScreenOff && !liveStatusPage && !updateModePage) {
    refreshRemoteBatteryBadgeIfChanged();
  }

  if (now - lastBatteryReadMs >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryReadMs = now;
    int oldPct = remoteBatteryPercent;
    int oldMv = remoteBatteryMv;
    bool oldCharging = remoteBatteryCharging;
    bool oldUsb = remoteUsbPresent;
    readRemoteBattery();

    if (screenAwake && (oldPct != remoteBatteryPercent || oldMv != remoteBatteryMv ||
                        oldCharging != remoteBatteryCharging || oldUsb != remoteUsbPresent)) {
      if (liveStatusPage) liveStatusDirty = true;
      else dashboardDirty = true;
    }
  }

  if (!screenAwake) {
    return;
  }

  if (liveStatusPage) {
    if (pendingCommandId != 0 && (now - pendingCommandMs > 4000UL)) {
      setCommandStatusRemote("MAIN CONFIRM TIMEOUT", C_RED);
      pendingCommandId = 0;
      pendingCommandType = CMD_NONE;
    }
    return;
  }

  if (pendingCommandId != 0 && (now - pendingCommandMs > 4000UL)) {
    setCommandStatusRemote("MAIN CONFIRM TIMEOUT", C_RED);
    pendingCommandId = 0;
    pendingCommandType = CMD_NONE;
    dashboardDirty = true;
  }

  if (!havePacket) {
    drawWaitingScreen();
    return;
  }

  unsigned long ageMs = now - lastPacketMs;
  bool lost = ageMs > LOST_SIGNAL_MS;

  // Draw only when new data arrives, when signal state changes, or occasionally while offline.
  // This avoids pointless full-dashboard repainting every loop.
  bool lostChanged = (lost != lastLostState);
  bool newPacket = (lastPacket.seq != lastDrawnSeq);

  updateLowWaterAlarmState();

  if ((dashboardDirty || lostChanged || (newPacket && (now - lastDrawMs >= DASHBOARD_MIN_REFRESH_MS)) || (lost && (now - lastDrawMs >= DASHBOARD_MIN_REFRESH_MS))) &&
      (now - lastDrawMs >= DASHBOARD_MIN_REFRESH_MS)) {
    lastDrawMs = now;
    dashboardDirty = false;
    drawRemoteDashboard();
  }
}
