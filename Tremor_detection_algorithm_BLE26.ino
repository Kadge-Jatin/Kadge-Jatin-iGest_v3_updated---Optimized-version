/*
* Tremor_Detection_RTOS_V2_UI.ino
* Merges V2 UI (identical fonts/positions/behaviour) with a FreeRTOS-based,
* non-invasive display task (as in V1). Sampling loop logic preserved so
* 10 ms (100 Hz) sampling remains accurate.
*
* Author: adapted for user by ChatGPT
* Based on: V1 (RTOS display) + V2 (UI & layout)
* Date: 2026-02-09
*/

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <esp_partition.h>
#include <esp_spi_flash.h>

#include "config.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>



#define IIR_ORDER 8

TTGOClass *watch;
TFT_eSPI *tft;
BMA *sensor;

// UUIDs for services and characteristics
#define IMU_SERVICE_UUID        "19B10000-E8F2-537E-4F6C-D104768A1214"
#define IMU_CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
#define TIME_SERVICE_UUID       "19B20000-E8F2-537E-4F6C-D104768A1214"
#define TIME_CHARACTERISTIC_UUID "19B20001-E8F2-537E-4F6C-D104768A1214"

// BLE server, services, and characteristics pointers
BLEServer* pServer = nullptr;
BLEService* imuService = nullptr;
BLECharacteristic* imuCharacteristic = nullptr;

BLEService* timeService = nullptr;
BLECharacteristic* timeCharacteristic = nullptr;

// Timing variables
unsigned long lastReadTime = 0;
const unsigned long readInterval = 10; // 10ms for 100Hz

#define IMU_AXIS 3
#define CHANNELS 1 // accel magnitude

bool transmitBLE = false;
unsigned long bleStartTime = 0;
const unsigned long BLE_DURATION = 0.5 * 60 * 1000;  // 30 seconds in milliseconds


// ===== DISPLAY POWER SAVE (touch double-tap wake) =====
static const uint32_t DISPLAY_ON_WINDOW_MS = 5000; // 10s screen-on after wake
static const uint32_t DOUBLE_TAP_GAP_MS    = 350;   // max gap between taps
static const uint32_t TAP_DEBOUNCE_MS      = 120;   // ignore re-triggers

// ===== DISPLAY POWER SAVE STATE =====
volatile bool displayOn = true;                 // screen/backlight state
volatile bool displayWakeRequested = false;     // set by loop(), consumed by displayTask()

static uint32_t lastDisplayOnMs = 0;            // refreshed while screen is on


volatile bool forceRedraw = false;

volatile bool pmuIRQ = false;   // set by AXP202 interrupt, handled in loop()


// tap detection (non-blocking)
static bool touchWasDown = false;
static uint32_t lastTapRegisteredMs = 0;
static bool waitingSecondTap = false;
static uint32_t firstTapMs = 0;

// bool tremor_status = false;

bool inTremorWindow = false;
unsigned long tremorWindowStartMs = 0;

// Configurable parameters
const int MAX_DATA_POINTS = 210;
const float BASELINE_TIMEFRAME = 2.0;
const float CROSSING_TIMEFRAME = 1.0;
const int CROSSING_THRESHOLD = 6;

const float DEFAULT_TREMOR_THRESHOLD = 100.0f;
const float DEFAULT_ANXIETY_THRESHOLD = 200.0f;
const float DEFAULT_THRESHOLD = 100.0f;

float current_ACCL_NOISE_THRESHOLD;

float Accel_Mag[MAX_DATA_POINTS];

// Raw IMU data and filtered data
float IMU_data[IMU_AXIS+1];
float IMU_filtered[IMU_AXIS];
float IMU_magnitude[CHANNELS];

// Updated filter coefficients (8th order)
float b[9] = {0.00293679, 0.0, -0.01174716, 0.0, 0.01762074, 0.0, -0.01174716, 0.0, 0.00293679};
float a[9] = {1.0, -6.14789409, 16.90682328, -27.19829387, 28.01743143, -18.93040648, 8.1935523, -2.07753259, 0.23647338};

// History buffers for IIR filter state (length = IIR_ORDER)
float x_history[IMU_AXIS][IIR_ORDER] = {0}; // For 3 axes, 8 history points each
float y_history[IMU_AXIS][IIR_ORDER] = {0};

unsigned long realw_timestamps[MAX_DATA_POINTS];
unsigned long timestamps[MAX_DATA_POINTS];
int dataIndex = 0;
bool bufferFilled = false;

int tremor_count = 0;
bool prev_tremor_state = false;

bool first_receive = true;

bool wasConnected = false; // global or static variable

// State variables
bool start_reading = false;
uint8_t readData = 0;
int count = 0;
int tremor_data_count = 1;
int fail_count = 0;

unsigned long time_elapsed = 0;
unsigned long time_elapsed_BLE = 0;
unsigned long last_sample_time = 0;
unsigned long time_difference;
unsigned long currentTime;

// For handling & showing mode and last shown time
String currentMode = "";         // current mode to display below Count
String lastDisplayedTime = "";   // last time shown (so a mode-only write won't blank time)
String lastDisplayedDate = "";   // e.g. "15-02-2026"

int crossingCount_Amag = 0;

bool Amag_tremor = false;
bool overallTremor = false;

const char* resultCharArray;

// ===================== BLE Data Buffering & Packet Functions =====================
float data_buffer[30];
uint16_t data_buffer_uint[30];
uint8_t data_buffer_status[30];

uint8_t buffer_fill = 0;
unsigned long packets_sent = 0;
unsigned long last_marker_sent = 0;

// For ESP32 BLE Write Handling
volatile bool timeCharWritten = false;
char receivedTimeBuffer[21] = {0};

// TFT and shared mutex handles (for the UI task)
SemaphoreHandle_t tftMutex = NULL;
SemaphoreHandle_t sharedMutex = NULL;
SemaphoreHandle_t displayStateMutex = NULL;  // FIX 2: protects displayOn/wake flags/timers



// ===== RTC FALLBACK ADDON VARIABLES (NEW) =====
struct RTCFallbackState {
  bool isUsingRTC;                    // true when disconnected, false when using BLE
  int referenceHour;                  // Last hour from BLE (0-23)
  int referenceMinute;                // Last minute from BLE (0-59)
  int referenceDay;                   // Last day from BLE (1-31)
  int referenceMonth;                 // Last month from BLE (1-12)
  int referenceYear;                  // Last year from BLE (2000+)
  unsigned long rtcModeStartMS;       // millis() when switched to RTC mode
  bool wasConnectedLastCheck;         // Track disconnection event
};

RTCFallbackState rtcState = {
  false,    // isUsingRTC
  0, 0,     // referenceHour, referenceMinute
  0, 0, 0,  // referenceDay, referenceMonth, referenceYear
  0,        // rtcModeStartMS
  true      // wasConnectedLastCheck (start as if connected)
};

SemaphoreHandle_t rtcStateMutex = NULL;
// ===== END RTC ADDON VARIABLES =====


// ===== FLASH RECORDING VARIABLES =====
// ===== RAW FLASH RECORDING VARIABLES =====
#define FLASH_RECORD_SIZE       9               // bytes per record (4+2+2+1)
#define FLASH_BUFFER_SAMPLES    300             // flush every 200 samples = 2 second
#define FLASH_BUFFER_SIZE       (FLASH_BUFFER_SAMPLES * FLASH_RECORD_SIZE)  // 1600 bytes

#define FLASH_SECTOR_SIZE       4096            // ESP32 flash sector = 4 KB
// #define FLASH_MAX_BYTES         3550000UL       // 3.5 MB limit
#define FLASH_MAX_BYTES  3200000UL                 // Leave 94KB margin below proven 3,294,000 limit

// RAM buffer — samples collect here, then get written to flash in one shot
uint8_t flash_buffer[FLASH_BUFFER_SIZE];
uint16_t flash_buffer_fill = 0;

// Raw partition tracking
const esp_partition_t* dataPartition = NULL;    // found in flashInit()
uint32_t flash_write_offset = 0;               // current write position in partition
uint32_t flash_bytes_written = 0;              // total bytes written (for limit check)
bool flash_recording = false;
bool flash_full = false;

// ===== SLOT-BASED HEADER TRACKING =====
uint16_t header_slot_index = 1;  // slot 0 = magic, slots 1-1023 = offset bookmarks

// Pre-erase tracking
// We keep a "cursor" of how far ahead we've erased.
// Erasing happens in background or during BLE time.
// uint32_t flash_erased_up_to = 0;               // offset up to which sectors are erased
// #define FLASH_PRE_ERASE_AHEAD  (FLASH_SECTOR_SIZE * 20)  // pre-erase 20 sectors = 80 KB ahead

// // Background pre-erase task
// TaskHandle_t flashEraseTaskHandle = NULL;
// volatile bool flash_erase_task_running = false;

// RTC cache for flash path
uint16_t flash_cached_rtcTime = 0;
unsigned long flash_rtcTime_lastUpdate = 0;
// ===== END RAW FLASH RECORDING VARIABLES =====


// ===== END FLASH RECORDING VARIABLES =====


// ========== BLE PACKET SENDING ==========
void sendPacket21() {
  uint8_t packet[210];
  for (int i = 0; i < 30; i++) {
    memcpy(&packet[i*7 + 0], &data_buffer[i], 4);    // IMU_magnitude[0]
    memcpy(&packet[i*7 + 4], &data_buffer_uint[i], 2);  // time_difference
    memcpy(&packet[i*7 + 6], &data_buffer_status[i], 1);  // time_difference

    // Serial.print("Sample ");
    // Serial.print(i);
    // Serial.print(": ");
    // Serial.print(data_buffer[i], 2);
    // Serial.print(" | ");
    // Serial.print(data_buffer_uint[i]);
    // Serial.print(" | ");
    // Serial.println(data_buffer_status[i]);
  }
  imuCharacteristic->setValue(packet, 210);
  imuCharacteristic->notify();
  packets_sent++;
  buffer_fill = 0;
}

void sendTremorStartMarker() {
  uint8_t hdr[2] = {0x89, 0x67}; // 0x6789
  uint8_t pkt[6] = {0};
  memcpy(&pkt[0], hdr, 2);
  imuCharacteristic->setValue(pkt, 6);
  imuCharacteristic->notify();
  packets_sent++;
}

void flushPartialPacketWithZeros() {
  // If we have a partially-filled 30-sample packet, pad it and send it now.
  // This does NOT drop real samples; it just ends the chunk early.
  if (buffer_fill > 0 && buffer_fill < 30) {
    for (int i = buffer_fill; i < 30; i++) {
      data_buffer[i] = 0;
      data_buffer_uint[i] = 0;
      data_buffer_status[i] = 0;
    }
    sendPacket21(); // resets buffer_fill = 0
  }
}

void sendTremorMarker(uint16_t tremor_count_local, uint16_t tremor_data_count_local) {
  uint8_t marker_header[2] = {0xCD, 0xAB}; // Unique marker header
  uint8_t marker_packet[6] = {0}; 
  memcpy(&marker_packet[0], marker_header, 2);
  memcpy(&marker_packet[2], &tremor_count_local, 2);
  memcpy(&marker_packet[4], &tremor_data_count_local, 2);
  imuCharacteristic->setValue(marker_packet, 6);
  imuCharacteristic->notify();

  uint8_t zero_marker_header[2] = {0x34, 0x12};
  uint8_t zero_marker_packet[6] = {0};
  memcpy(&zero_marker_packet[0], zero_marker_header, 2);
  imuCharacteristic->setValue(zero_marker_packet, 6);
  imuCharacteristic->notify();

  packets_sent += 2;
}

// ========== BLE CALLBACKS ==========

class MyServerCallbacks : public BLEServerCallbacks {
  
  // void onConnect(BLEServer* pServer) {
  //   wasConnected = true;
  //   Serial.println("Central connected.");
  // }

  void onConnect(BLEServer* pServer) 
  {
    wasConnected = true;
    Serial.println("Central connected.");

    // Save any unsaved flash data before switching to BLE mode.
    // flashStopRecording() flushes the RAM buffer to flash and closes the file.
    // Without this, up to 99 samples in RAM would be lost on connect.
    if (flash_recording) {
      flashStopRecording();
      Serial.println("FLASH: Stopped recording (BLE connected)");
    }
  }
  void onDisconnect(BLEServer* pServer) {
    wasConnected = false;
    Serial.println("Central disconnected.");
    BLEDevice::getAdvertising()->start(); // Re-advertise
  }
};

class TimeCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    memset(receivedTimeBuffer, 0, sizeof(receivedTimeBuffer));
    memcpy(receivedTimeBuffer, value.data(), std::min((int)value.length(), 20));
    timeCharWritten = true;
  }
};

// ===================== UI HELPER FUNCTIONS (V2 visuals preserved) =====================

static int16_t centerXForText(const char* txt, int textsize) {
  int len = strlen(txt);
  int w = len * 6 * textsize;
  return (240 - w) / 2;
}


// Returns true on a tap press-edge (debounced)
static bool detectTap(uint32_t now) {
  bool touchDown = (digitalRead(TOUCH_INT) == LOW);
  bool tap = false;

  if (touchDown && !touchWasDown) {
    if (now - lastTapRegisteredMs >= TAP_DEBOUNCE_MS) {
      tap = true;
      lastTapRegisteredMs = now;
    }
  }
  touchWasDown = touchDown;
  return tap;
}


static void requestDisplayWake(uint32_t now) {
  if (displayStateMutex && xSemaphoreTake(displayStateMutex, 0) == pdTRUE) {
    displayWakeRequested = true;
    forceRedraw = true;
    displayOn = true;
    lastDisplayOnMs = now;
    xSemaphoreGive(displayStateMutex);
  } else {
    displayWakeRequested = true;
    forceRedraw = true;
    displayOn = true;
    lastDisplayOnMs = now;
  }
}


static void handleTap(uint32_t now) {
  // If display already ON: any tap just refreshes timer (no redraw)
  if (displayOn) 
  {
    if (displayStateMutex && xSemaphoreTake(displayStateMutex, 0) == pdTRUE) 
    {
      lastDisplayOnMs = now;
      xSemaphoreGive(displayStateMutex);
    } 
    else {
      lastDisplayOnMs = now;
    }
    waitingSecondTap = false;
    return;
  }

  // display OFF: only double tap wakes
  if (!waitingSecondTap) {
    waitingSecondTap = true;
    firstTapMs = now;
  } else {
    if (now - firstTapMs <= DOUBLE_TAP_GAP_MS) {
      requestDisplayWake(now);
    }
    waitingSecondTap = false;
  }
}



// draw a simple Bluetooth glyph; green when connected, grey when not
static void drawBluetoothIcon(int x, int y, bool connected) {
  if (connected) {
    // Connected: solid light blue circle with black BT symbol
    tft->fillCircle(x, y, 11, 0x5DFF);  // Light blue
    
    // Black BT symbol
    tft->drawLine(x, y-7, x, y+7, TFT_BLACK);
    tft->drawLine(x, y-7, x+5, y-3, TFT_BLACK);
    tft->drawLine(x, y+7, x+5, y+3, TFT_BLACK);
    tft->drawLine(x+5, y-3, x, y, TFT_BLACK);
    tft->drawLine(x+5, y+3, x, y, TFT_BLACK);
  } else {
    // Disconnected: just outline with grey BT symbol  
    tft->drawCircle(x, y, 11, 0x4208);
    
    // Grey BT symbol
    tft->drawLine(x, y-7, x, y+7, 0x6B4D);
    tft->drawLine(x, y-7, x+5, y-3, 0x6B4D);
    tft->drawLine(x, y+7, x+5, y+3, 0x6B4D);
    tft->drawLine(x+5, y-3, x, y, 0x6B4D);
    tft->drawLine(x+5, y+3, x, y, 0x6B4D);
  }
}

static void drawBatteryIcon(int x_right, int y_top) {
  int pct = 0;
  bool charging = false;
  
  // Get battery info
  if (watch->power) {
    pct = watch->power->getBattPercentage();
    charging = watch->power->isChargeing();
  }
  
  // Clamp percentage
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  const int bodyW = 24, bodyH = 12;
  const int tipW = 2, tipH = 6;
  int bodyX = x_right - bodyW;
  int bodyY = y_top + 4;

  // Draw battery outline
  tft->drawRoundRect(bodyX, bodyY, bodyW, bodyH, 2, TFT_WHITE);
  tft->fillRect(x_right + 1, bodyY + (bodyH - tipH)/2, tipW, tipH, TFT_WHITE);

  // Draw battery fill level
  int innerW = bodyW - 4;
  int fillW = (innerW * pct) / 100;
  uint16_t lvlColor = pct > 50 ? TFT_GREEN : (pct > 20 ? TFT_YELLOW : TFT_RED);
  if (fillW > 0) {
    tft->fillRect(bodyX + 2, bodyY + 2, fillW, bodyH - 4, lvlColor);
  }

  // Draw percentage text INSIDE battery icon
  char pbuf[5];
  snprintf(pbuf, sizeof(pbuf), "%d", pct);
  tft->setTextSize(1);
  tft->setTextColor(TFT_WHITE, TFT_BLUE);
  
  int textW = strlen(pbuf) * 6;
  int textX = bodyX + (bodyW - textW) / 2;
  tft->setCursor(textX, bodyY + 2);
  tft->print(pbuf);
}

static void drawStatusBar() {
  // Draw blue status bar background
  tft->fillRect(0, 0, 240, 24, TFT_BLUE);
  tft->drawLine(0, 24, 240, 24, TFT_NAVY);
  
  // Draw Mode on left side
  tft->setTextSize(1);
  tft->setTextColor(TFT_WHITE, TFT_BLUE);
  tft->setCursor(4, 8);
  if (currentMode.length() > 0) {
    tft->print(currentMode);
  } else {
    tft->print("--");
  }
  
  // Draw BLE connection status in middle
  // tft->setCursor(110, 8);
  // if (wasConnected) {
  //   tft->setTextColor(TFT_GREEN, TFT_BLUE);
  //   tft->print("BLE");
  // } else {
  //   tft->setTextColor(TFT_DARKGREY, TFT_BLUE);
  //   tft->print("---");
  // }

  drawBluetoothIcon(225, 40, wasConnected); // adjust x,y to position as desired
  
  // Draw battery on right side
  if (watch->power) {
    drawBatteryIcon(240 - 4, 2);
  }
}

// updateDisplay is made thread-safe: it snapshots protected data under sharedMutex,
// while drawing is protected by tftMutex. Visuals & positions identical to V2.
static void updateDisplay() {

  if (!displayOn) return;

  // Snapshot shared data first (short critical section)
  String snapTime;
  String snapDate;
  String snapMode;
  int snapCount;
  bool snapWasConnected;

  if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    snapTime = lastDisplayedTime;
    snapDate = lastDisplayedDate;
    snapMode = currentMode;
    snapCount = tremor_count;
    snapWasConnected = wasConnected;
    xSemaphoreGive(sharedMutex);
  } else {
    snapTime = lastDisplayedTime;
    snapDate = lastDisplayedDate;
    snapMode = currentMode;
    snapCount = tremor_count;
    snapWasConnected = wasConnected;
  }

  // Static caches to avoid frequent redraws
  static unsigned long lastUpdate = 0;
  static String lastDisplayedTimeCache = "";
  static String lastDisplayedDateCache = "";
  static int lastTremorCount = -1;
  static String lastModeCache = "";
  static bool lastBLEStatus = false;

  unsigned long currentMillis = millis();
  bool timeToUpdate = (currentMillis - lastUpdate >= 60000); // 60 seconds

  bool dataChanged = (snapTime != lastDisplayedTimeCache) ||
                     (snapDate != lastDisplayedDateCache) ||
                     (snapCount != lastTremorCount) ||
                     (snapMode != lastModeCache) ||
                     (snapWasConnected != lastBLEStatus);

  if (!forceRedraw && !timeToUpdate && !dataChanged) return;

  lastUpdate = currentMillis;
  lastDisplayedTimeCache = snapTime;
  lastDisplayedDateCache = snapDate;
  lastTremorCount = snapCount;
  lastModeCache = snapMode;
  lastBLEStatus = snapWasConnected;

  if (xSemaphoreTake(tftMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    tft->fillScreen(TFT_BLACK);

    drawStatusBar();

    // Title
    tft->setTextColor(TFT_BLUE, TFT_BLACK);
    tft->setTextSize(2);
    int16_t x = centerXForText("iGest", 2);
    tft->setCursor(x, 40);
    tft->print("iGest");

    // Passcode
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setTextSize(2);
    x = centerXForText("6373", 2);
    tft->setCursor(x, 70);
    tft->print("6373");

    // TIME (reduced font) + DATE (shifted up)
        // ===== RTC ADDON: Calculate display time (BLE or RTC fallback) =====
    String displayTime = snapTime;
    String displayDate = snapDate;
    
    if (!snapWasConnected) {
    calculateDisplayTimeWithRTCFallback(displayTime, displayDate);
    }
    // ===== END RTC ADDON =====

    // TIME (reduced font) + DATE (shifted up)
    if (displayTime.length() > 0) {
      // Reduced font from 4 -> 3
      tft->setTextSize(3);
      tft->setTextColor(TFT_WHITE, TFT_BLACK);
      char timeBuf[32];
      displayTime.toCharArray(timeBuf, sizeof(timeBuf));
      x = centerXForText(timeBuf, 3);
      tft->setCursor(x, 112);   // slightly lower than before to balance smaller font
      tft->print(displayTime);

      // Date below time (shift upward a bit)
      if (displayDate.length() > 0) {
        tft->setTextSize(2);
        tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
        char dateBuf[32];
        displayDate.toCharArray(dateBuf, sizeof(dateBuf));
        int16_t dx = centerXForText(dateBuf, 2);
        tft->setCursor(dx, 145); // moved up from 155 -> 145 to create more space above Count
        tft->print(displayDate);
      }
    }

    // Count (keep clearly separated from date)
    if (snapWasConnected) {
      char countStr[20];
      snprintf(countStr, sizeof(countStr), "Count: %d", snapCount);
      tft->setTextSize(2);
      tft->setTextColor(TFT_WHITE, TFT_BLACK);
      x = centerXForText(countStr, 2);
      tft->setCursor(x, 190);   // moved down from 180 -> 190 to increase gap from date
      tft->print(countStr);
    }

    // forceRedraw = false;   // <--- move here, after drawing is done

    if (displayStateMutex && xSemaphoreTake(displayStateMutex, 0) == pdTRUE) 
    {
      forceRedraw = false;
      xSemaphoreGive(displayStateMutex);
    } 
    else 
    {
      forceRedraw = false;
    }

    xSemaphoreGive(tftMutex);
  }
}

// Display RTOS task: calls updateDisplay() every second (non-invasive)
// void displayTask(void *pvParameters) {
//   const TickType_t delayTicks = pdMS_TO_TICKS(1000); // 1 Hz
//   for (;;) {
//     vTaskDelay(delayTicks);
//     updateDisplay();
//   }
// }

void displayTask(void *pvParameters) {
  bool blState = true; // tracks what we last applied to hardware backlight

  for (;;) {
    // ---- FIX 2: snapshot shared display state safely ----
    bool wakeReq = false;
    bool dispOn = false;

    if (displayStateMutex && xSemaphoreTake(displayStateMutex, 0) == pdTRUE) {
      wakeReq = displayWakeRequested;
      dispOn  = displayOn;
      xSemaphoreGive(displayStateMutex);
    } else {
      wakeReq = displayWakeRequested;
      dispOn  = displayOn;
    }

    // ---- Handle wake request ----
    if (wakeReq) {
      if (displayStateMutex && xSemaphoreTake(displayStateMutex, 0) == pdTRUE) {
        displayWakeRequested = false;
        displayOn = true;
        xSemaphoreGive(displayStateMutex);
      } else {
        displayWakeRequested = false;
        displayOn = true;
      }

      if (!blState) {
        watch->openBL();
        blState = true;
      }

      updateDisplay();                 // one immediate draw on wake
      vTaskDelay(pdMS_TO_TICKS(200));  // small yield
      continue;
    }

    // ---- Normal display ON/OFF behavior ----
    if (dispOn) {
      if (!blState) {
        watch->openBL();
        blState = true;
      }
      updateDisplay();
      vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz update while on
    } else {
      if (blState) {
        watch->closeBL();
        blState = false;
      }
      vTaskDelay(pdMS_TO_TICKS(500));  // sleep while off
    }
  }
}

// ========== Core algorithm & helpers (unchanged logic) ==========

float calculateBaseline(float dataArray[], unsigned long currentTime) 
{
  float sum = 0;
  int baseline_count = 0;
  unsigned long startTime = currentTime - (BASELINE_TIMEFRAME * 1000);
  
  int index = dataIndex - 1;
  if (index < 0) index = MAX_DATA_POINTS - 1;
  
  for (int i = 0; i < MAX_DATA_POINTS; i++) 
  {
    if (realw_timestamps[index] >= startTime) {
      sum += dataArray[index];
      baseline_count++;
    } else if (realw_timestamps[index] < startTime) {
      break;
    }
    index--;
    if (index < 0) index = MAX_DATA_POINTS - 1;
  }
  return (baseline_count > 0) ? (sum / baseline_count) : 0;
}

int countCrossings(float dataArray[], float baseline, unsigned long currentTime, float threshold) 
{
  int crossings = 0;
  char prevState = 'N';
  unsigned long startTime = currentTime - (CROSSING_TIMEFRAME * 1000);
  
  int index = dataIndex - 1;
  if (index < 0) index = MAX_DATA_POINTS - 1;
  
  for (int i = 0; i < MAX_DATA_POINTS; i++) 
  {
    if (realw_timestamps[index] >= startTime) 
    {
      float value = dataArray[index];
      char currentState;
      if (value > baseline + threshold) 
        currentState = 'A';
      else if (value < baseline - threshold) 
        currentState = 'B';
      else 
        currentState = 'N';
      if ((prevState == 'A' && currentState == 'B') || 
          (prevState == 'B' && currentState == 'A')) {
        crossings++;
      }
      if (currentState != 'N') {
        prevState = currentState;
      }
    } else {
      break;
    }
    index--;
    if (index < 0) index = MAX_DATA_POINTS - 1;
  }
  return crossings;
}

void calculateMagnitudes() 
{
  IMU_magnitude[0] = sqrt(
    IMU_filtered[0] * IMU_filtered[0] + 
    IMU_filtered[1] * IMU_filtered[1] + 
    IMU_filtered[2] * IMU_filtered[2]
  );
}

float applyIIRFilter(float x, int index) 
{
  float y = b[0]*x;
  for (int i = 0; i < IIR_ORDER; i++) 
  {
    y += b[i+1]*x_history[index][i];
    y -= a[i+1]*y_history[index][i];
  }

  // Shift histories
  for(int i = IIR_ORDER-1; i > 0; i--) 
  {
    x_history[index][i] = x_history[index][i-1];
    y_history[index][i] = y_history[index][i-1];
  }

  x_history[index][0] = x;
  y_history[index][0] = y;
  
  return y;
}

// ===== RTC FALLBACK ADDON HELPER FUNCTIONS =====



void syncRTCWithTime(int hh, int mm, int ss, int dd, int mon, int yyyy) {
  if (watch && watch->rtc) {
    watch->rtc->setDateTime(yyyy, mon, dd, hh, mm, ss);
    Serial.print("RTC: Hardware synced to ");
    Serial.print(hh); Serial.print(":");
    Serial.print(mm); Serial.print(":");
    Serial.print(ss); Serial.print(" on ");
    Serial.print(dd); Serial.print("-");
    Serial.print(mon); Serial.print("-");
    Serial.println(yyyy);
  }
}

// Update RTC reference state (called when BLE time received)
void updateRTCReference(int hh, int mm, int ss, int dd, int mon, int yyyy) {
  if (xSemaphoreTake(rtcStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    rtcState.referenceHour = hh;
    rtcState.referenceMinute = mm;
    rtcState.referenceDay = dd;
    rtcState.referenceMonth = mon;
    rtcState.referenceYear = yyyy;
    xSemaphoreGive(rtcStateMutex);
  }
  syncRTCWithTime(hh, mm, ss, dd, mon, yyyy);
}

// Monitor connection state changes and switch RTC mode
void monitorConnectionStateForRTC(bool currentConnection) {
  if (xSemaphoreTake(rtcStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    // Detect disconnection
    if (rtcState.wasConnectedLastCheck && !currentConnection) {
      Serial.println("RTC: BLE disconnected - switching to RTC fallback mode");
      rtcState.isUsingRTC = true;
      rtcState.rtcModeStartMS = millis();
    }
    // Detect reconnection
    if (!rtcState.wasConnectedLastCheck && currentConnection) {
      Serial.println("RTC: BLE reconnected - switching back to BLE time mode");
      rtcState.isUsingRTC = false;
    }
    rtcState.wasConnectedLastCheck = currentConnection;
    xSemaphoreGive(rtcStateMutex);
  }
}

// Calculate display time: returns BLE time or RTC-calculated time based on connection state
// Calculate display time: returns BLE time or RTC-calculated time based on connection state
void calculateDisplayTimeWithRTCFallback(String& outTime, String& outDate) {
  if (xSemaphoreTake(rtcStateMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

  if (!rtcState.isUsingRTC) {
    xSemaphoreGive(rtcStateMutex);
    return;
  }

  // Calculate elapsed minutes since disconnect
  unsigned long elapsedMS = millis() - rtcState.rtcModeStartMS;
  unsigned long elapsedMin = elapsedMS / 60000UL;

  // Start from reference date/time (from last BLE time)
  int year  = rtcState.referenceYear;
  int month = rtcState.referenceMonth;   // 1..12
  int day   = rtcState.referenceDay;     // 1..31
  int refHour  = rtcState.referenceHour;    // 0..23
  int refMinute= rtcState.referenceMinute;  // 0..59

  // Compute how many whole days have passed (for date advance)
  unsigned long totalMinutes = (unsigned long)refHour * 60UL + (unsigned long)refMinute + elapsedMin;
  unsigned long daysToAdd = totalMinutes / (24UL * 60UL);

  // ---- TIME: prefer hardware RTC "HH:MM:SS" -> display "HH:MM" ----
  int dispHH = refHour;
  int dispMM = refMinute;

  if (watch && watch->rtc) {
    const char* rtcStr = watch->rtc->formatDateTime(); // expected "HH:MM:SS"
    if (rtcStr && strlen(rtcStr) >= 5 && rtcStr[2] == ':') {
      int h = (isDigit(rtcStr[0]) && isDigit(rtcStr[1])) ? (rtcStr[0]-'0')*10 + (rtcStr[1]-'0') : -1;
      int m = (isDigit(rtcStr[3]) && isDigit(rtcStr[4])) ? (rtcStr[3]-'0')*10 + (rtcStr[4]-'0') : -1;
      if (h >= 0 && h <= 23 && m >= 0 && m <= 59) {
        dispHH = h;
        dispMM = m;
      }
    }
  }

  char timeBuf[6];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", dispHH, dispMM);
  outTime = String(timeBuf);

  // ---- DATE: advance from reference date using daysToAdd (multi-day safe) ----
  auto isLeapYear = [](int y) -> bool {
    return (y % 4 == 0) && ((y % 100 != 0) || (y % 400 == 0));
  };

  auto daysInMonth = [&](int y, int m) -> int {
    static const int dim[13] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    if (m == 2) return isLeapYear(y) ? 29 : 28;
    if (m >= 1 && m <= 12) return dim[m];
    return 31;
  };

  while (daysToAdd > 0) {
    int dim = daysInMonth(year, month);
    int remainingInMonth = dim - day;

    if (daysToAdd <= (unsigned long)remainingInMonth) {
      day += (int)daysToAdd;
      daysToAdd = 0;
    } else {
      daysToAdd -= (unsigned long)(remainingInMonth + 1);
      day = 1;
      month++;
      if (month > 12) {
        month = 1;
        year++;
      }
    }
  }

  char dateBuf[11];
  snprintf(dateBuf, sizeof(dateBuf), "%02d-%02d-%04d", day, month, year);
  outDate = String(dateBuf);

  xSemaphoreGive(rtcStateMutex);
}

// ===== END RTC FALLBACK ADDON HELPER FUNCTIONS =====


// ===== FLASH RECORDING HELPER FUNCTIONS =====



// Called once in setup(). Finds the data partition and reads the
// stored write offset from the first 8 bytes (header).
//
// Partition layout:
//   Byte 0-3: magic number 0xACCE1DAT
//   Byte 4-7: write_offset (where to resume writing)
//   Byte 8+:  actual data records
//
#define FLASH_HEADER_SIZE    4096
#define FLASH_MAGIC          0xACCE1DA7

void flashInit() {
  dataPartition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 
                                            ESP_PARTITION_SUBTYPE_DATA_SPIFFS, 
                                            NULL);
  if (!dataPartition) {
    dataPartition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 
                                              ESP_PARTITION_SUBTYPE_ANY, 
                                              "spiffs");
  }
  
  if (!dataPartition) {
    Serial.println("FLASH: No partition found! Recording disabled.");
    flash_full = true;
    return;
  }
  
  Serial.printf("FLASH: Found partition '%s' at 0x%X, size %lu bytes\n",
                dataPartition->label, dataPartition->address, dataPartition->size);
  
  // Check for valid header
    // Check for valid header and scan for latest offset slot
  uint32_t magic = 0;
  esp_partition_read(dataPartition, 0, &magic, 4);
  
  if (magic == FLASH_MAGIC) {
    // Scan slots 1–1023 to find the last written offset
    uint32_t last_valid_offset = FLASH_HEADER_SIZE;  // default = no data
    header_slot_index = 1;
    
    for (uint16_t i = 1; i < 1024; i++) {
      uint32_t val = 0;
      esp_partition_read(dataPartition, i * 4, &val, 4);
      if (val == 0xFFFFFFFF) {
        // Empty slot — previous slot was the last written
        break;
      }
      last_valid_offset = val;
      header_slot_index = i + 1;  // next write goes to the slot after this one
    }
    
    if (last_valid_offset >= FLASH_HEADER_SIZE && 
        last_valid_offset < dataPartition->size) {
      flash_write_offset = last_valid_offset;
      flash_bytes_written = last_valid_offset - FLASH_HEADER_SIZE;
      Serial.printf("FLASH: Existing data found — %lu bytes used (slot %u)\n", 
                     flash_bytes_written, header_slot_index - 1);
    } else {
      flash_write_offset = FLASH_HEADER_SIZE;
      flash_bytes_written = 0;
      Serial.println("FLASH: Valid magic but invalid offset — starting fresh");
    }
  } else {
    // No valid header — erase sector 0 and write fresh header
    esp_partition_erase_range(dataPartition, 0, FLASH_SECTOR_SIZE);
    magic = FLASH_MAGIC;
    esp_partition_write(dataPartition, 0, &magic, 4);
    flash_write_offset = FLASH_HEADER_SIZE;
    flash_bytes_written = 0;
    header_slot_index = 1;
    Serial.println("FLASH: No valid header — starting fresh");
  }
  
  if (flash_bytes_written >= FLASH_MAX_BYTES) {
    flash_full = true;
    Serial.println("FLASH: Full (3.5 MB reached)");
    return;
  }
  
  // ── SMART ERASE: erase only the UNUSED portion ahead of write_offset ──
  // Round write_offset UP to next sector boundary (don't erase the sector we're writing in)
  uint32_t erase_start = ((flash_write_offset / FLASH_SECTOR_SIZE) + 1) * FLASH_SECTOR_SIZE;
  
  if (erase_start < dataPartition->size) {
    // Round erase_end DOWN to sector boundary (erase_range needs aligned size)
    uint32_t erase_size = dataPartition->size - erase_start;
    // erase_size must be multiple of FLASH_SECTOR_SIZE
    erase_size = (erase_size / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
    
    if (erase_size > 0) {
      Serial.printf("FLASH: Erasing unused space from 0x%lX, size %lu bytes (%lu sectors)...\n",
                    erase_start, erase_size, erase_size / FLASH_SECTOR_SIZE);
      
      unsigned long eraseStartMs = millis();
      esp_err_t err = esp_partition_erase_range(dataPartition, erase_start, erase_size);
      unsigned long eraseTimeMs = millis() - eraseStartMs;
      
      if (err == ESP_OK) {
        Serial.printf("FLASH: Erase complete in %lu ms\n", eraseTimeMs);
      } else {
        Serial.printf("FLASH: Erase FAILED with error %d\n", err);
      }
    } else {
      Serial.println("FLASH: No unused space to erase");
    }
  } else {
    Serial.println("FLASH: Write offset at end of partition, nothing to erase");
  }
  
  Serial.printf("FLASH: Ready — offset %lu, %lu bytes used, partition pre-erased\n",
                flash_write_offset, flash_bytes_written);
}

// Saves the current write_offset to the partition header.
// Called when stopping recording so we can resume later.
// This writes 4 bytes to offset 4 in the header sector.
// The header sector is already erased (we only write once at init),
// BUT flash can only transition bits from 1→0, so subsequent writes
// to the same bytes won't work unless we erase first.
// Solution: we erase sector 0 and rewrite both magic + offset.
void flashSaveHeader() {
  if (!dataPartition) return;
  
  if (header_slot_index >= 1024) {
    // All 1023 slots used — must erase once (~65 ms, happens every ~8.5 hours)
    esp_partition_erase_range(dataPartition, 0, FLASH_SECTOR_SIZE);
    uint32_t magic = FLASH_MAGIC;
    esp_partition_write(dataPartition, 0, &magic, 4);
    header_slot_index = 1;
    Serial.println("FLASH: Header sector full — erased and reset");
  }
  
  // Write offset to next empty slot (no erase needed — just a write)
  uint32_t slot_byte_offset = header_slot_index * 4;
  esp_partition_write(dataPartition, slot_byte_offset, &flash_write_offset, 4);
  header_slot_index++;
}

void flashStartRecording() {
  if (flash_full || !dataPartition) return;
  
  flash_recording = true;
  flash_buffer_fill = 0;
  
  // No erasing here — partition was pre-erased in flashInit()
  Serial.printf("FLASH: Recording started (offset %lu, %lu bytes written)\n", 
                flash_write_offset, flash_bytes_written);
}

void flashStopRecording() {
  if (!flash_recording) return;
  
  // Flush remaining samples
  if (flash_buffer_fill > 0) {
    flashFlushBuffer();
  }
  
  // Save current write position to header so we can resume after reboot
  flashSaveHeader();
  
  flash_recording = false;
  Serial.printf("FLASH: Recording stopped (offset %lu, %lu bytes written)\n",
                flash_write_offset, flash_bytes_written);
}

// THE KEY FUNCTION. Writes buffer to raw flash.
// Only writes to PRE-ERASED sectors — so this is just a page-program
// operation (~0.3 ms per 256 bytes, ~1.2 ms for 900 bytes).
// No sector erase happens here = no 65 ms block.
void flashFlushBuffer() {
  if (flash_buffer_fill == 0) return;
  if (!dataPartition) return;
  if (flash_full) return;
  
  uint16_t bytes_to_write = flash_buffer_fill * FLASH_RECORD_SIZE;
  
  // Check 3.5 MB limit
  if (flash_bytes_written + bytes_to_write > FLASH_MAX_BYTES) {
    uint32_t remaining = FLASH_MAX_BYTES - flash_bytes_written;
    uint16_t samples_that_fit = remaining / FLASH_RECORD_SIZE;
    bytes_to_write = samples_that_fit * FLASH_RECORD_SIZE;
    if (bytes_to_write == 0) {
      flash_full = true;
      flash_buffer_fill = 0;
      Serial.println("FLASH: Storage full (3.5 MB reached)");
      flashStopRecording();
      return;
    }
  }
  
  // Check partition boundary
  if (flash_write_offset + bytes_to_write > dataPartition->size) {
    flash_full = true;
    flash_buffer_fill = 0;
    Serial.println("FLASH: Partition full");
    flashStopRecording();
    return;
  }
  
  // WRITE — partition was pre-erased in flashInit(), pure page-program only
  esp_err_t err = esp_partition_write(dataPartition, flash_write_offset, 
                                       flash_buffer, bytes_to_write);
  if (err != ESP_OK) {
    Serial.printf("FLASH: Write error %d at offset %lu\n", err, flash_write_offset);
  }
  
  flash_write_offset += bytes_to_write;
  flash_bytes_written += bytes_to_write;
  flash_buffer_fill = 0;

  // ===== PERIODIC HEADER SAVE (every 30 seconds = every 30th flush) =====
  static uint8_t flush_count_since_header_save = 0;
  flush_count_since_header_save++;
  if (flush_count_since_header_save >= 30) {
    flashSaveHeader();
    flush_count_since_header_save = 0;
  }

}

// Packs one 9-byte data sample into the RAM buffer (same as before)
void flashWriteDataSample(float mag, uint16_t timeDiff, uint16_t rtcTime, uint8_t status) {
  if (!flash_recording || flash_full) return;
  
  uint16_t offset = flash_buffer_fill * FLASH_RECORD_SIZE;
  
  memcpy(&flash_buffer[offset + 0], &mag,      4);
  memcpy(&flash_buffer[offset + 4], &timeDiff,  2);
  memcpy(&flash_buffer[offset + 6], &rtcTime,   2);
  memcpy(&flash_buffer[offset + 8], &status,    1);
  
  flash_buffer_fill++;
  
  if (flash_buffer_fill >= FLASH_BUFFER_SAMPLES) {
    flashFlushBuffer();
  }
}

// Writes a 9-byte marker record (same as before)
void flashWriteMarker(uint8_t h1, uint8_t h2, uint8_t* payload, uint8_t payloadLen) {
  if (!flash_recording || flash_full) return;
  
  uint16_t offset = flash_buffer_fill * FLASH_RECORD_SIZE;
  memset(&flash_buffer[offset], 0, FLASH_RECORD_SIZE);
  flash_buffer[offset + 0] = h1;
  flash_buffer[offset + 1] = h2;
  
  if (payload && payloadLen > 0) {
    uint8_t maxPayload = FLASH_RECORD_SIZE - 2;
    if (payloadLen > maxPayload) payloadLen = maxPayload;
    memcpy(&flash_buffer[offset + 2], payload, payloadLen);
  }
  
  flash_buffer_fill++;
  
  if (flash_buffer_fill >= FLASH_BUFFER_SAMPLES) {
    flashFlushBuffer();
  }
}

// Pad and flush partial buffer (same as before)
void flashFlushPartialBuffer() {
  if (flash_buffer_fill > 0 && flash_buffer_fill < FLASH_BUFFER_SAMPLES) {
    uint16_t offset = flash_buffer_fill * FLASH_RECORD_SIZE;
    uint16_t remaining_bytes = (FLASH_BUFFER_SAMPLES - flash_buffer_fill) * FLASH_RECORD_SIZE;
    memset(&flash_buffer[offset], 0, remaining_bytes);
    flash_buffer_fill = FLASH_BUFFER_SAMPLES;
    flashFlushBuffer();
  }
}

// RTC time reader (same as before)
uint16_t flashGetRTCTime() {
  uint8_t hh = 0, mm = 0;
  if (watch && watch->rtc) {
    const char* rtcStr = watch->rtc->formatDateTime();
    if (rtcStr && strlen(rtcStr) >= 5 && rtcStr[2] == ':') {
      if (isDigit(rtcStr[0]) && isDigit(rtcStr[1]))
        hh = (rtcStr[0] - '0') * 10 + (rtcStr[1] - '0');
      if (isDigit(rtcStr[3]) && isDigit(rtcStr[4]))
        mm = (rtcStr[3] - '0') * 10 + (rtcStr[4] - '0');
    }
  }
  return (uint16_t)(hh * 100 + mm);
}

// ===== END RAW FLASH RECORDING HELPER FUNCTIONS =====

// ===== END FLASH RECORDING HELPER FUNCTIONS =====


// ===================== setup() =====================

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Tremor Detection System Starting...");
  // Get TTGOClass instance
  watch = TTGOClass::getWatch();
  // Initialize the hardware, the BMA423 sensor has been initialized internally
  watch->begin();
  // Turn on the backlight
  watch->openBL();
      
  //Receive objects for easy writing
  tft = watch->tft;
  sensor = watch->bma;
  
  // Create mutexes before any UI or shared access
  tftMutex = xSemaphoreCreateMutex();
  sharedMutex = xSemaphoreCreateMutex();
  rtcStateMutex = xSemaphoreCreateMutex();  // RTC ADDON
  displayStateMutex = xSemaphoreCreateMutex();

  // Initialize raw flash partition for recording
  flashInit();


  // ---- PEK (side button) short press IRQ setup ----
  pinMode(AXP202_INT, INPUT_PULLUP);
  attachInterrupt(AXP202_INT, [] {
    pmuIRQ = true;
  }, FALLING);

  // Enable PEK short press interrupt in PMU and clear pending IRQ
  if (watch->power) {
    watch->power->enableIRQ(AXP202_PEK_SHORTPRESS_IRQ, true);
    watch->power->clearIRQ();
  }

  // Test battery availability
  if (watch->power) {
    int pct = watch->power->getBattPercentage();
    bool charging = watch->power->isChargeing();
    Serial.print("Battery: ");
    Serial.print(pct);
    Serial.print("% ");
    Serial.println(charging ? "(Charging)" : "(Not Charging)");
  } else {
    Serial.println("Warning: Power module not available");
  }
  
  // Accel parameter structure
  Acfg cfg;
  cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
  cfg.range = BMA4_ACCEL_RANGE_2G;
  cfg.perf_mode = BMA4_CONTINUOUS_MODE;
  
  // Configure the BMA423 accelerometer
  sensor->accelConfig(cfg);
  // Enable BMA423 accelerometer
  sensor->enableAccel();
      
  // Initial display setup (protected)
  if (xSemaphoreTake(tftMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    tft->fillScreen(TFT_BLACK);
    drawStatusBar();
    xSemaphoreGive(tftMutex);
  }

  Serial.println("Accelerometer reading at 100Hz using millis() timing");

  start_reading = true;
  time_elapsed = millis();
  time_difference = 0;

  // 1. Initialize BLE and set device name
  BLEDevice::init("Tremor Detection v6");

  // 2. Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 3. IMU Service and Characteristic
  imuService = pServer->createService(IMU_SERVICE_UUID);
  imuCharacteristic = imuService->createCharacteristic(
    IMU_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  imuCharacteristic->addDescriptor(new BLE2902());  // <-- Add this line!
  imuService->start();


  // 4. Time Service and Characteristic
  timeService = pServer->createService(TIME_SERVICE_UUID);
  timeCharacteristic = timeService->createCharacteristic(
    TIME_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  timeCharacteristic->setCallbacks(new TimeCharacteristicCallbacks());
  timeService->start();

  // 5. Start advertising both services
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(IMU_SERVICE_UUID);
  pAdvertising->addServiceUUID(TIME_SERVICE_UUID);
  pAdvertising->start();

  // current_ACCL_NOISE_THRESHOLD = DEFAULT_ANXIETY_THRESHOLD;
  current_ACCL_NOISE_THRESHOLD = DEFAULT_THRESHOLD;

  Serial.println("Bluetooth device active, waiting for connections...");

  // Create the UI display task which updates at 1 Hz (1000 ms)
  // Lower priority than sampling to avoid preemption of real-time loop
  xTaskCreatePinnedToCore(displayTask, "displayTask", 4096, NULL, 1, NULL, 1);


  pinMode(TOUCH_INT, INPUT);

  uint32_t now = millis();
  lastDisplayOnMs = now;
  displayOn = true;
  displayWakeRequested = true; // force one initial draw shortly after boot
  forceRedraw = true;





}

// ===================== loop() - sampling & processing (keeps 10ms timing) =====================

void loop() 
{

  // // Add this at the very top of loop(), temporarily:
  // static unsigned long lastLoopTime = 0;
  // static unsigned long normalCount = 0;  // counts iterations ≤ 15 ms
  // unsigned long loopNow = millis();

  // if (lastLoopTime > 0) {
  //   if ((loopNow - lastLoopTime) > 10) {
  //     Serial.printf("LOOP GAP: %lu ms  (after %lu normal iterations)\n", 
  //                    loopNow - lastLoopTime, normalCount);
  //     normalCount = 0;  // reset counter after each gap
  //   } else {
  //     normalCount++;
  //     Serial.printf("LOOP GAP: %lu ms  (after %lu normal iterations)\n", 
  //                    loopNow - lastLoopTime, normalCount);
  //   }
  
  // }
  // lastLoopTime = loopNow;  // ← MISSING!

  // // ===== SAFE LOOP TIMING MONITOR =====
  
  // static unsigned long loopStartMs = 0;
  // static unsigned long iterCount = 0;
  // static unsigned long lastPrintMs = 0;
  // static unsigned long maxIterTimeMs = 0;
  // static unsigned long minIterTimeMs = 999;
  
  // unsigned long loopNow = millis();
  
  // if (loopStartMs > 0) {
  //   unsigned long iterTimeMs = loopNow - loopStartMs;
    
  //   // Track min/max
  //   if (iterTimeMs > maxIterTimeMs) maxIterTimeMs = iterTimeMs;
  //   if (iterTimeMs < minIterTimeMs) minIterTimeMs = iterTimeMs;
    
  //   iterCount++;
    
  //   // Print ONLY unusual iterations
  //   if (iterTimeMs > 12) {
  //     Serial.printf("[UNUSUAL ITER] Time: %lu ms (normal: ~10ms)\n", iterTimeMs);
  //   }
  // }
  
  // loopStartMs = loopNow;
  
  // // Print summary every 5 seconds (WITH SAFETY CHECK)
  // if (loopNow - lastPrintMs >= 5000) {
  //   if (iterCount > 0) {  // ← SAFETY CHECK!
  //     unsigned long avgMs = (loopNow - lastPrintMs) / iterCount;
  //     Serial.printf("[SUMMARY] Iterations: %lu | Min: %lu ms | Max: %lu ms | Avg: %lu ms\n",
  //                    iterCount, minIterTimeMs, maxIterTimeMs, avgMs);
  //   }
  //   lastPrintMs = loopNow;
  //   iterCount = 0;
  //   maxIterTimeMs = 0;
  //   minIterTimeMs = 999;
  // }
  
  // ===== REST OF YOUR LOOP CODE STARTS HERE =====
  
  // ... all your existing code ...

  // ===== DISPLAY POWER SAVE: touch tap handling (non-blocking) =====
  uint32_t nowTap = millis();

  if (detectTap(nowTap)) {
    handleTap(nowTap);
  }

  // cancel double-tap wait window if expired (display OFF case)
  if (waitingSecondTap && (nowTap - firstTapMs > DOUBLE_TAP_GAP_MS)) {
    waitingSecondTap = false;
  }


  // ===== PEK (side button) short press wake =====
  if (pmuIRQ) {
    pmuIRQ = false;

    if (watch->power) {
      watch->power->readIRQ();

      if (watch->power->isPEKShortPressIRQ()) {
        watch->power->clearIRQ();

        // Wake screen same as double-tap wake
        uint32_t nowBtn = millis();
        requestDisplayWake(nowBtn);
      } else {
        // clear any other IRQ flags
        watch->power->clearIRQ();
      }
    }
  }
  // ===== END PEK wake =====

  // auto-off after window
  // if (displayOn && (nowTap - lastDisplayOnMs >= DISPLAY_ON_WINDOW_MS)) {
  //   displayOn = false;
  // }

  // bool doAutoOff = false;
  uint32_t lastOnLocal = 0;
  bool displayOnLocal = false;

  if (displayStateMutex && xSemaphoreTake(displayStateMutex, 0) == pdTRUE) 
  {
    displayOnLocal = displayOn;
    lastOnLocal = lastDisplayOnMs;
    xSemaphoreGive(displayStateMutex);
  } else {
    displayOnLocal = displayOn;
    lastOnLocal = lastDisplayOnMs;
  }

  if (displayOnLocal && (nowTap - lastOnLocal >= DISPLAY_ON_WINDOW_MS)) {
    if (displayStateMutex && xSemaphoreTake(displayStateMutex, 0) == pdTRUE) {
      displayOn = false;
      xSemaphoreGive(displayStateMutex);
    } else {
      displayOn = false;
    }
  }
  // ===== END DISPLAY POWER SAVE =====

  // ===== RTC ADDON: Monitor connection changes =====
  monitorConnectionStateForRTC(wasConnected);
  // ===== END RTC ADDON =====


  // Process incoming BLE Time characteristic writes (this updates lastDisplayedTime and currentMode)
  if (wasConnected && timeCharWritten)
  {
    String receivedTime = String(receivedTimeBuffer);
    receivedTime.trim();
    Serial.print("Received time: ");
    Serial.println(receivedTime);

    // Case-insensitive search for "MODE:"
    String upper = receivedTime;
    upper.toUpperCase();
    int idx = upper.indexOf("MODE:");
    if (idx >= 0) {
      // Find start of mode value (skip "MODE:" and any spaces)
      int valStart = idx + 5;
      while (valStart < (int)receivedTime.length() && isSpace(receivedTime[valStart])) valStart++;

      // Find end of mode value (space/newline or end)
      int valEnd = valStart;
      while (valEnd < (int)receivedTime.length() &&
            receivedTime[valEnd] != ' ' &&
            receivedTime[valEnd] != '\n' &&
            receivedTime[valEnd] != '\r') {
        valEnd++;
      }

      // Extract mode value
      String modeValue = "";
      if (valEnd > valStart) {
        modeValue = receivedTime.substring(valStart, valEnd);
        modeValue.trim();
      }

      // Normalize and apply mode (protect with sharedMutex)
      String m = modeValue;
      m.toUpperCase();
      if (m.length() > 0) {
        if (m.indexOf("ANX") >= 0 || m.indexOf("ANXIETY") >= 0) {
          current_ACCL_NOISE_THRESHOLD = DEFAULT_ANXIETY_THRESHOLD;
          if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            currentMode = "ANXIETY";
            tremor_count = 0;
            xSemaphoreGive(sharedMutex);
          } else {
            currentMode = "ANXIETY";
            tremor_count = 0;
          }
          Serial.print("Switched to ANXIETY mode. Threshold = ");
          Serial.println(current_ACCL_NOISE_THRESHOLD);
        } else if (m.indexOf("TREM") >= 0 || m.indexOf("TREMOR") >= 0) {
          current_ACCL_NOISE_THRESHOLD = DEFAULT_TREMOR_THRESHOLD;
          if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            currentMode = "TREMOR";
            tremor_count = 0;
            xSemaphoreGive(sharedMutex);
          } else {
            currentMode = "TREMOR";
            tremor_count = 0;
          }
          Serial.print("Switched to TREMOR mode. Threshold = ");
          Serial.println(current_ACCL_NOISE_THRESHOLD);
        } else {
          if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            currentMode = m;
            xSemaphoreGive(sharedMutex);
          } else {
            currentMode = m;
          }
          Serial.print("MODE command received: ");
          Serial.println(m);
        }
      }

      // Remove the entire "MODE:...token" from the receivedTime string
      int removeEnd = (valEnd > valStart) ? valEnd : (idx + 5);
      String tokenToRemove = receivedTime.substring(idx, removeEnd);
      receivedTime.replace(tokenToRemove, "");
      receivedTime.trim();
    }

    // ------------------------------
    // Parse SENS:<value> token (only applies in ANXIETY mode)
    // ------------------------------
    String upper2 = receivedTime;
    upper2.toUpperCase();
    int sidx = upper2.indexOf("SENS:");
    if (sidx >= 0) {
      int valStart = sidx + 5;
      while (valStart < (int)receivedTime.length() && isSpace(receivedTime[valStart])) valStart++;

      int valEnd = valStart;
      while (valEnd < (int)receivedTime.length() &&
            receivedTime[valEnd] != ' ' &&
            receivedTime[valEnd] != '\n' &&
            receivedTime[valEnd] != '\r') {
        valEnd++;
      }

      String sensValueStr = "";
      if (valEnd > valStart) {
        sensValueStr = receivedTime.substring(valStart, valEnd);
        sensValueStr.trim();
      }

      // Apply only if we are in ANXIETY mode
      String cmUpper = currentMode;
      cmUpper.toUpperCase();
      bool inAnxiety = (cmUpper.indexOf("ANX") >= 0);

      if (inAnxiety && sensValueStr.length() > 0) {
        int sensInt = sensValueStr.toInt(); // expects 50..500
        if (sensInt < 50) sensInt = 50;
        if (sensInt > 500) sensInt = 500;

        current_ACCL_NOISE_THRESHOLD = (float)sensInt;

        Serial.print("Applied ANXIETY sensitivity threshold = ");
        Serial.println(current_ACCL_NOISE_THRESHOLD);
      } else {
        Serial.print("Ignoring SENS (mode not ANXIETY). currentMode=");
        Serial.println(currentMode);
      }

      // Remove the entire "SENS:...token" so it doesn't pollute time display
      int removeEnd = (valEnd > valStart) ? valEnd : (sidx + 5);
      String tokenToRemove = receivedTime.substring(sidx, removeEnd);
      receivedTime.replace(tokenToRemove, "");
      receivedTime.trim();
    }

    // If message contains a time, update lastDisplayedTime; protected by sharedMutex
    // If message contains time|date, update lastDisplayedTime and lastDisplayedDate (no placeholders)
        // ===== RTC ADDON: Parse time, sync RTC, and update display =====
    if (receivedTime.length() > 0) {
      // Parse received format: HH:mm|ddMMyyyy
      String timePart = receivedTime;
      String datePartDigits = "";

      int bar = receivedTime.indexOf('|');
      if (bar >= 0) {
        timePart = receivedTime.substring(0, bar);      // "HH:mm"
        datePartDigits = receivedTime.substring(bar + 1); // "ddMMyyyy"
        timePart.trim();
        datePartDigits.trim();
      } else {
        timePart.trim();
      }

      // Extract HH, MM, SS from "HH:mm:ss" (or fallback "HH:mm")
      int hh = 0, mm = 0, ss = 0;

      int c1 = timePart.indexOf(':');
      int c2 = (c1 >= 0) ? timePart.indexOf(':', c1 + 1) : -1;

      if (c1 >= 0 && c2 >= 0) {
        hh = timePart.substring(0, c1).toInt();
        mm = timePart.substring(c1 + 1, c2).toInt();
        ss = timePart.substring(c2 + 1).toInt();
      } else if (c1 >= 0) {
        hh = timePart.substring(0, c1).toInt();
        mm = timePart.substring(c1 + 1).toInt();
        ss = 0;
      }

      // Extract DD, MM, YYYY from "ddMMyyyy"
      int dd = 0, mon = 0, yyyy = 0;
      String dateFormatted = "";
      if (datePartDigits.length() >= 8) {
        String ddStr = datePartDigits.substring(0, 2);
        String mmStr = datePartDigits.substring(2, 4);
        String yyyyStr = datePartDigits.substring(4, 8);
        dd = ddStr.toInt();
        mon = mmStr.toInt();
        yyyy = yyyyStr.toInt();
        
        // Format for display: DD-MM-YYYY
        dateFormatted = ddStr + "-" + mmStr + "-" + yyyyStr;
      }

      // Validate parsed values
      if (hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59 && ss >= 0 && ss <= 59 && dd >= 1 && dd <= 31 && mon >= 1 && mon <= 12 && yyyy >= 2000) 
      {  
        // RTC ADDON: Update RTC reference and sync hardware
        updateRTCReference(hh, mm, ss, dd, mon, yyyy);
        
        // Update display with parsed time and date
        if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          char hhmmBuf[6];
          snprintf(hhmmBuf, sizeof(hhmmBuf), "%02d:%02d", hh, mm);
          lastDisplayedTime = String(hhmmBuf);
          if (dateFormatted.length() > 0) {
            lastDisplayedDate = dateFormatted;
          }
          xSemaphoreGive(sharedMutex);
        } else {
          char hhmmBuf[6];
          snprintf(hhmmBuf, sizeof(hhmmBuf), "%02d:%02d", hh, mm);
          lastDisplayedTime = String(hhmmBuf);
          if (dateFormatted.length() > 0) {
            lastDisplayedDate = dateFormatted;
          }
        }
        
        Serial.print("BLE: Time updated - ");
        Serial.print(timePart);
        Serial.print(" | Date: ");
        Serial.println(dateFormatted);
      } else {
        Serial.println("BLE: Invalid time values, ignoring");
      }
    }
    // ===== END RTC ADDON =====

    timeCharWritten = false;
  }

  // Ensure 10ms sampling interval
  unsigned long startWait = millis();
  while (millis() - lastReadTime < readInterval) {
    // allow other tasks to run briefly
    delay(0);
    // avoid tight busy loop
    if (millis() - startWait > 20) break;
  }

  // Read accelerometer data
  Accel acc;
  bool res = sensor->getAccel(acc);
  if (!res) 
  {
    Serial.println("getAccel FAIL");
  } 
  else 
  {
    IMU_data[1] = acc.x;
    IMU_data[2] = acc.y;
    IMU_data[3] = acc.z;
  }
  
  // Update last read time
  lastReadTime = millis();
  
  currentTime = millis();
  count++;

  // if(transmitBLE)
  // {
  //   tremor_data_count++;
  //   time_difference = millis() - bleStartTime;
  //   data_buffer_uint[buffer_fill] = (uint16_t)time_difference;
  // }

  realw_timestamps[dataIndex] = currentTime;

  for (int i = 0; i < IMU_AXIS; i++) 
  {
    IMU_filtered[i] = applyIIRFilter(IMU_data[i + 1], i);
  }

  calculateMagnitudes();

  // Protect Accel_Mag and index updates so display snapshot is consistent
  if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    Accel_Mag[dataIndex] = IMU_magnitude[0];
    dataIndex = (dataIndex + 1) % MAX_DATA_POINTS;
    if (dataIndex == 0) bufferFilled = true;
    xSemaphoreGive(sharedMutex);
  } else {
    // fallback without mutex if busy (very rare)
    Accel_Mag[dataIndex] = IMU_magnitude[0];
    dataIndex = (dataIndex + 1) % MAX_DATA_POINTS;
    if (dataIndex == 0) bufferFilled = true;
  }

  // ========================================================================
  // DETECTION + OUTPUT
  // 
  // The key idea: tremor detection runs ONCE using ONE set of shared
  // variables (tremor_count, prev_tremor_state, inTremorWindow, bleStartTime,
  // tremor_data_count). This means when you switch between BLE and flash,
  // all counts and states carry over seamlessly — no resets, no gaps.
  //
  // Only the OUTPUT differs:
  //   BLE connected  → send packets over Bluetooth (your existing code)
  //   BLE disconnected → save 9-byte records to flash (new code)
  // ========================================================================


  if (bufferFilled)
  {
    // ── SHARED: Tremor detection (runs ALWAYS, regardless of BLE/flash) ──
    // This was previously inside "if (bufferFilled && wasConnected)" which
    // meant it stopped running when BLE disconnected. Now it runs always.
    float Amag_Baseline = calculateBaseline(Accel_Mag, currentTime);
    crossingCount_Amag = countCrossings(Accel_Mag, Amag_Baseline, currentTime, current_ACCL_NOISE_THRESHOLD);
    Amag_tremor = (crossingCount_Amag >= CROSSING_THRESHOLD);

    bool current_tremor_state = Amag_tremor;

    // ── SHARED: Tremor onset detection ──
    // Uses the SAME prev_tremor_state, tremor_count, inTremorWindow for both
    // paths. If tremor starts during BLE, continues into flash. And vice versa.
    if (current_tremor_state && !prev_tremor_state && !inTremorWindow)
    {
      // Flush partial data on whichever path is currently active
      if (wasConnected) {
        flushPartialPacketWithZeros();
      } else if (flash_recording) {
        flashFlushPartialBuffer();
      }

      // Shared tremor count — protected for display task to read safely
      if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        tremor_count++;
        xSemaphoreGive(sharedMutex);
      } else {
        tremor_count++;
      }

      // Write START marker to whichever path is active
      if (wasConnected) {
        sendTremorStartMarker();
      } else if (flash_recording) {
        flashWriteMarker(0x89, 0x67, NULL, 0);
      }

      // Shared tremor window state — carries across BLE/flash transitions
      inTremorWindow = true;
      tremorWindowStartMs = millis();
      bleStartTime = tremorWindowStartMs;  // timeDiff resets to ~0 for BOTH paths
      tremor_data_count = 1;
    }
    prev_tremor_state = current_tremor_state;

    // ── SHARED: Compute time_difference and status ──
    // Uses the SAME bleStartTime for both BLE and flash.
    // This is why time_difference is continuous across BLE/flash switches.
    time_difference = millis() - bleStartTime;
    uint8_t sample_status = (uint8_t)(inTremorWindow ? 1 : (Amag_tremor ? 1 : 0));

    // ════════════════════════════════════════════════════════════════════
    // OUTPUT PATH A: BLE (when connected)
    // This is your existing BLE code, moved here. Logic is identical.
    // ════════════════════════════════════════════════════════════════════
    if (wasConnected)
    {
      transmitBLE = true;

      // If flash was recording, stop it (safety — onConnect already does this,
      // but in case loop() runs before the callback fires)
      if (flash_recording) {
        flashStopRecording();
      }

      // Buffer sample into BLE packet (identical to your original code)
      data_buffer[buffer_fill] = IMU_magnitude[0];
      data_buffer_uint[buffer_fill] = (uint16_t)time_difference;
      data_buffer_status[buffer_fill] = sample_status;

      tremor_data_count++;
      buffer_fill++;

      // 30 samples collected → send BLE packet
      if (buffer_fill == 30) {
        sendPacket21();
      }

      // End tremor window after BLE_DURATION (30 sec)
      if (inTremorWindow && (millis() - tremorWindowStartMs > BLE_DURATION))
      {
        // Flush partial BLE packet with zeros
        if (buffer_fill > 0 && buffer_fill < 30) {
          for (int i = buffer_fill; i < 30; i++) {
            data_buffer[i] = 0;
            data_buffer_uint[i] = 0;
            data_buffer_status[i] = 0;
          }
          sendPacket21();
        }

        // Send END marker then ZERO marker (your original order)
        sendTremorMarker((uint16_t)tremor_count, (uint16_t)tremor_data_count);

        tremor_data_count = 1;
        last_marker_sent = millis();
        inTremorWindow = false;
      }
    }
    
    // ════════════════════════════════════════════════════════════════════
    // OUTPUT PATH B: Flash (when disconnected)
    // Mirrors the BLE path above but writes 9-byte records to flash.
    // Uses the SAME shared variables — nothing resets on transition.
    // ════════════════════════════════════════════════════════════════════
    else
    {

      transmitBLE = false;
      buffer_fill = 0;  // keep BLE buffer reset so it's clean when BLE reconnects

      if (!flash_full)
      {
        // Start flash recording if not already running
        // (first loop iteration after BLE disconnects)
        if (!flash_recording) {
          flashStartRecording();
        }


        // Write one 9-byte data sample to RAM buffer
        // Same data as BLE sends, plus RTC time (extra 2 bytes)
        uint16_t flash_rtcTime = flashGetRTCTime();
        flashWriteDataSample(
          IMU_magnitude[0],
          (uint16_t)time_difference,
          flash_rtcTime,
          sample_status
        );
        tremor_data_count++;

        // End tremor window after BLE_DURATION (30 sec) — same logic as BLE
        if (inTremorWindow && (millis() - tremorWindowStartMs > BLE_DURATION))
        {
          // Flush partial flash buffer (mirrors BLE's flushPartialPacketWithZeros)
          flashFlushPartialBuffer();

          // END marker FIRST (0xCD, 0xAB) — same order as your BLE sendTremorMarker()
          uint8_t endPayload[4];
          uint16_t tc = (uint16_t)tremor_count;
          uint16_t tdc = (uint16_t)tremor_data_count;
          memcpy(&endPayload[0], &tc, 2);
          memcpy(&endPayload[2], &tdc, 2);
          flashWriteMarker(0xCD, 0xAB, endPayload, 4);

          // ZERO marker SECOND (0x34, 0x12) — same order as your BLE sendTremorMarker()
          flashWriteMarker(0x34, 0x12, NULL, 0);

          tremor_data_count = 1;
          inTremorWindow = false;
        }
      }
    }
  }

  else
  {
    // Circular buffer not yet filled (first ~2 seconds after boot).
    // Neither BLE nor flash path can run yet — detection needs history.
    transmitBLE = false;
  }



  // loop() keeps sampling; display updates are handled in the RTOS displayTask.
}