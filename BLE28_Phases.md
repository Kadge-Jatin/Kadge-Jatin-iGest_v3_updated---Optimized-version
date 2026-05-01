# BLE28 — Implementation Phases

---

## Phase 0 — Config and Constants

**File section:** Includes + defines + global variables

### Tasks
- [ ] Define all constants in one block at top of file
- [ ] Define all global variables
- [ ] Define SyncState enum
- [ ] Define ConfigSlot struct

### Constants to Define
```cpp
// ── Timing ──────────────────────────────────────────
#define SAMPLE_INTERVAL_MS        10       // 100 Hz
#define T1_INTERVAL_MS            (15UL * 60UL * 1000UL)  // 15 minutes
#define T2_SYNC_INTERVAL_MS       50       // inter-packet during sync

// ── Buffer sizes ─────────────────────────────────────
#define B1_SAMPLES                300      // flash write buffer
#define B1_RECORD_SIZE            9
#define B1_BUFFER_SIZE            (B1_SAMPLES * B1_RECORD_SIZE)  // 2700 bytes

#define B2_SAMPLES                34       // anxiety live BLE buffer
#define B2_RECORD_SIZE            7
#define B2_BUFFER_SIZE            (B2_SAMPLES * B2_RECORD_SIZE)  // 238 bytes

#define B3_RECORDS_PER_PKT        27       // sync chunk
#define B3_PKT_SIZE               (B3_RECORDS_PER_PKT * B1_RECORD_SIZE)  // 243 bytes

// ── Flash layout ─────────────────────────────────────
#define FLASH_MAGIC               0xACCE1DA7
#define FLASH_CONFIG_MAGIC        0xCF28CF07
#define FLASH_SECTOR_SIZE         4096
#define FLASH_HEADER_HALF_SIZE    2048
#define FLASH_OFFSET_HDR_START    0
#define FLASH_CONFIG_HDR_START    2048
#define FLASH_DATA_START          4096
#define FLASH_MAX_BYTES           3200000UL
#define FLASH_WARN_PERCENT        90
#define FLASH_OFFSET_SLOTS        511      // (2048-4)/4
#define FLASH_CONFIG_SLOTS        102      // (2048-4)/20

// ── Thresholds ───────────────────────────────────────
#define DEFAULT_TREMOR_THRESHOLD  100.0f
#define DEFAULT_ANXIETY_THRESHOLD 200.0f
#define CROSSING_THRESHOLD        6
#define MAX_DATA_POINTS           210
#define BASELINE_TIMEFRAME        2.0f
#define CROSSING_TIMEFRAME        1.0f
#define IIR_ORDER                 8
#define IMU_AXIS                  3

// ── Modes ────────────────────────────────────────────
#define MODE_TREMOR               0
#define MODE_ANXIETY              1

// ── Display ──────────────────────────────────────────
#define DISPLAY_ON_WINDOW_MS      5000
#define DOUBLE_TAP_GAP_MS         350
#define TAP_DEBOUNCE_MS           120

// ── Markers ──────────────────────────────────────────
#define MARKER_TREMOR_START_H1    0x89
#define MARKER_TREMOR_START_H2    0x67
#define MARKER_TREMOR_END_H1      0xCD
#define MARKER_TREMOR_END_H2      0xAB
#define MARKER_ZERO_H1            0x34
#define MARKER_ZERO_H2            0x12
#define MARKER_B2_START_H1        0xB2
#define MARKER_B2_START_H2        0xB2
#define MARKER_B2_END_H1          0xB3
#define MARKER_B2_END_H2          0xB3

static const uint8_t SYNC_BLE_START[4] = {0xAA, 0xBB, 0xCC, 0xDD};
static const uint8_t SYNC_BLE_END[4]   = {0xFF, 0xEE, 0xDD, 0xCC};
```

### SyncState Enum
```cpp
typedef enum {
  SYNC_IDLE,
  SYNC_WAITING_BLE,
  SYNC_STARTING,
  SYNC_ACTIVE,
  SYNC_INTERRUPTED,
  SYNC_COMPLETE
} SyncState;
```

### Key Global Variables
```cpp
// ── Mode ─────────────────────────────────────────────
uint8_t  currentMode      = MODE_TREMOR;
uint8_t  lastConnectedMode = MODE_TREMOR;
float    current_ACCL_NOISE_THRESHOLD;

// ── B1 ───────────────────────────────────────────────
uint8_t  b1_buffer[B1_BUFFER_SIZE];
uint16_t b1_fill = 0;

// ── B2 ───────────────────────────────────────────────
uint8_t  b2_buffer[B2_BUFFER_SIZE];
uint8_t  b2_fill = 0;

// ── Flash ────────────────────────────────────────────
const esp_partition_t* dataPartition = NULL;
uint32_t flash_write_offset  = FLASH_DATA_START;
uint32_t flash_bytes_written = 0;
bool     flash_recording     = false;
bool     flash_full          = false;
bool     flash_recording_paused = false;
bool     flash_erase_in_progress = false;
bool     flash_near_full_notified = false;
uint16_t header_slot_index   = 1;
uint16_t config_slot_index   = 1;

// ── Sync ─────────────────────────────────────────────
SyncState syncState           = SYNC_IDLE;
uint32_t  sync_read_offset    = FLASH_DATA_START;
uint32_t  sync_mark_offset    = FLASH_DATA_START;
bool      sync_pending        = false;
bool      sync_interrupted    = false;
unsigned long lastSyncPktMs   = 0;
unsigned long t1_start_ms     = 0;

// ── BLE flags ────────────────────────────────────────
volatile bool flashRefreshRequested = false;
volatile bool timeCharWritten       = false;
char receivedTimeBuffer[21]         = {0};
bool wasConnected                   = false;
```

### Deliverable
All constants and variables compile without error. No logic yet.

---

## Phase 1 — Flash Manager

**Replaces/extends** BLE26 flash functions.

### Tasks
- [ ] `flashInit()` — find partition, scan offset slots, scan config slots, pre-erase
- [ ] `flashSaveOffsetHeader()` — write to Half 1 slot mechanism
- [ ] `flashSaveConfig()` — write to Half 2 slot mechanism
- [ ] `flashLoadConfig()` — read latest config slot on boot
- [ ] `flashStartRecording()` — set flash_recording = true
- [ ] `flashStopRecording()` — flush B1, save offset header, save config
- [ ] `flashFlushB1()` — burst write b1_buffer to flash (pure page-program)
- [ ] `flashWriteMarkerRecord()` — write 9-byte marker directly to b1_buffer
- [ ] `flashEraseDataTask()` — FreeRTOS task, erases data area sector by sector
- [ ] `flashGetRTCTime()` — returns uint16 HHMM

### Key Design Points
```
flashInit():
  1. Find SPIFFS partition
  2. Read magic from offset 0
     Valid?  → scan Half 1 slots 1..511 → find flash_write_offset
             → scan Half 2 slots for config → flashLoadConfig()
     Invalid → erase sector 0 → write both magics → start fresh
  3. Pre-erase data area from write_offset+1 sector to end
     (same as BLE26, ~51 sec at boot, shown on TFT)

flashSaveOffsetHeader():
  header_slot_index >= 511?
    → erase sector 0 entirely
    → rewrite both magics + both current states
    → reset both slot indices to 1
  Write flash_write_offset to slot header_slot_index × 4
  header_slot_index++

flashSaveConfig():
  config_slot_index >= 102?
    → same full-sector-0 refresh as above
  Write ConfigSlot struct to FLASH_CONFIG_HDR_START + config_slot_index × 20
  config_slot_index++

flashFlushB1():
  Pure page-program write (no erase — pre-erased at boot)
  ~1ms for 2700 bytes
  Every 10 flushes → flashSaveOffsetHeader()
```

### Deliverable
Flash init works on boot. Data records write and survive reboot. Offset correctly restored.

---

## Phase 2 — Sampling, Filter, Detection

**Unchanged from BLE26.** Port directly.

### Tasks
- [ ] Port `applyIIRFilter()` (8th order, per-axis history buffers)
- [ ] Port `calculateMagnitudes()` (3-axis magnitude)
- [ ] Port `calculateBaseline()` (rolling 2-sec mean)
- [ ] Port `countCrossings()` (1-sec window, threshold ± baseline)
- [ ] Port BMA423 init in `setup()` (100 Hz ODR, 2G range, continuous mode)
- [ ] Port 10ms sampling loop logic in `loop()`

### Deliverable
Serial monitor shows accelerometer magnitude at 100 Hz. Tremor detection prints to serial.

---

## Phase 3 — B1 Buffer Integration

### Tasks
- [ ] `b1AddRecord()` — pack one 9-byte record into b1_buffer, call flashFlushB1() when full
- [ ] `b1AddMarker()` — pack a marker record (h1, h2, payload) into b1_buffer
- [ ] `b1FlushPartial()` — pad remaining slots with 0x00, flush
- [ ] `b1Reset()` — set b1_fill = 0

### Deliverable
TREMOR mode: every 3 seconds, 2700 bytes written to flash. Verified on serial.

---

## Phase 4 — BLE Stack + Characteristics

**Extends BLE27 setup.**

### Tasks
- [ ] `BLEDevice::init("iGest v1")` with all three services
- [ ] IMU service + imuChar (NOTIFY, BLE2902)
- [ ] TIME service + timeChar (WRITE, TimeCharacteristicCallbacks)
- [ ] FLASH service + flashDataChar (NOTIFY, BLE2902) + flashCmdChar (WRITE)
- [ ] `MyServerCallbacks::onConnect()` — set wasConnected, stop flash recording
- [ ] `MyServerCallbacks::onDisconnect()` — set wasConnected=false, re-advertise
- [ ] `TimeCharacteristicCallbacks::onWrite()` — set timeCharWritten flag
- [ ] `FlashCmdCharacteristicCallbacks::onWrite()` — set flashRefreshRequested on "REFRESH"
- [ ] Advertising all three service UUIDs

### Deliverable
Device advertises. Mobile connects. All three services visible. REFRESH command received.

---

## Phase 5 — B2 Buffer + Anxiety Live Stream

### Tasks
- [ ] `b2AddRecord()` — pack one 7-byte record into b2_buffer
- [ ] `b2SendPacket()` — notify imuChar with 238 bytes, clear b2_fill = 0
- [ ] `b2TransferToFlash()` — on disconnect:
  ```
  1. Write 0xB2 0xB2 START marker via b1AddMarker()
  2. For each b2_buffer record:
       extract mag, timeDiff, status
       rtcTime = flashGetRTCTime()
       b1AddRecord(mag, timeDiff, rtcTime, status)
  3. Write 0xB3 0xB3 END marker via b1AddMarker()
  4. b2_fill = 0
  ```
- [ ] In loop(): ANXIETY + BLE → b2AddRecord() per sample → b2SendPacket() when full
- [ ] In onDisconnect(): call b2TransferToFlash()

### Deliverable
Anxiety mode: mobile receives 238-byte packets at ~3.4 Hz (34 samples @ 100 Hz).
On disconnect: B2 contents appear in flash as B2-block.

---

## Phase 6 — Time/Mode/SENS Command Parser

**Port directly from BLE26** with additions.

### Tasks
- [ ] Parse `HH:mm|ddMMyyyy` → update lastDisplayedTime, lastDisplayedDate, sync RTC
- [ ] Parse `MODE:TREMOR` → set currentMode, threshold, reset tremor_count
- [ ] Parse `MODE:ANXIETY` → set currentMode, threshold, reset tremor_count
- [ ] Parse `SENS:<value>` → override threshold (ANXIETY only, 50–500)
- [ ] Save mode + threshold to config sector on change
- [ ] Port RTC sync helpers (`syncRTCWithTime`, `updateRTCReference`, `monitorConnectionStateForRTC`)

### Deliverable
Mode switches correctly. Config sector updated. Confirmed via serial print.

---

## Phase 7 — Sync State Machine

**Core new feature of BLE28.**

### Tasks
- [ ] Implement `syncStep()` — non-blocking function called every loop()
- [ ] `syncTrigger()` — sets sync_pending, sync_mark, saves config
- [ ] `syncSendChunk()` — reads B3=243 bytes, notifies flashDataChar
- [ ] T1 timer check in loop() (Tremor only)
- [ ] Anxiety on-connect check: if sync_read_offset < flash_write_offset → trigger sync
- [ ] REFRESH command handling → immediate syncTrigger()
- [ ] Save sync_read_offset to config sector after every chunk

### syncStep() Logic
```cpp
void syncStep() {
  unsigned long now = millis();

  switch (syncState) {

    case SYNC_IDLE:
      // Tremor: check T1
      if (currentMode == MODE_TREMOR &&
          (now - t1_start_ms) >= T1_INTERVAL_MS) {
        t1_start_ms = now;  // reset T1 immediately
        syncTrigger();
      }
      break;

    case SYNC_WAITING_BLE:
      if (wasConnected) {
        sync_mark_offset = flash_write_offset;  // extend to latest
        flashSaveConfig();
        syncState = SYNC_STARTING;
      }
      break;

    case SYNC_STARTING:
      if (!wasConnected) { syncState = SYNC_INTERRUPTED; break; }
      flashDataCharacteristic->setValue((uint8_t*)SYNC_BLE_START, 4);
      flashDataCharacteristic->notify();
      uint8_t tb[4]; memcpy(tb, &sync_mark_offset, 4);
      flashDataCharacteristic->setValue(tb, 4);
      flashDataCharacteristic->notify();
      lastSyncPktMs = millis();
      syncState = SYNC_ACTIVE;
      break;

    case SYNC_ACTIVE:
      if (!wasConnected) {
        flashSaveConfig();
        syncState = SYNC_INTERRUPTED;
        break;
      }
      if (now - lastSyncPktMs < T2_SYNC_INTERVAL_MS) break;
      if (sync_read_offset >= sync_mark_offset) {
        flashDataCharacteristic->setValue((uint8_t*)SYNC_BLE_END, 4);
        flashDataCharacteristic->notify();
        syncState = SYNC_COMPLETE;
        break;
      }
      syncSendChunk();
      lastSyncPktMs = now;
      break;

    case SYNC_INTERRUPTED:
      if (wasConnected) {
        sync_mark_offset = flash_write_offset;  // extend to latest
        flashSaveConfig();
        syncState = SYNC_STARTING;
      }
      break;

    case SYNC_COMPLETE:
      sync_pending     = false;
      sync_interrupted = false;
      flashSaveConfig();
      syncState = SYNC_IDLE;
      break;
  }
}
```

### Deliverable
Tremor mode: after 15 minutes, flash data streams to mobile in 243-byte chunks at 50ms intervals.
Confirmed on serial: chunk count, bytes sent, START/END markers received by mobile.

---

## Phase 8 — Flash Full Handlers

### Tasks
- [ ] TREMOR 90% check: `if (flash_bytes_written >= FLASH_MAX_BYTES * 9/10)`
  → set flash_near_full_notified, show TFT warning, do NOT stop recording
- [ ] TREMOR 100% check: `if (flash_bytes_written >= FLASH_MAX_BYTES)`
  → set flash_recording_paused = true, show TFT "STORAGE FULL"
- [ ] ANXIETY 100% check:
  → call `flashAnxietyFullHandler()`:
    ```cpp
    void flashAnxietyFullHandler() {
      flash_recording = false;
      flash_erase_in_progress = true;
      showFlashWarningOnTFT("STORAGE FULL - RESETTING");
      xTaskCreatePinnedToCore(flashEraseTask, "flashErase",
                              4096, NULL, 1, NULL, 1);
    }

    void flashEraseTask(void* param) {
      for (uint32_t off = FLASH_DATA_START;
           off < dataPartition->size;
           off += FLASH_SECTOR_SIZE) {
        esp_partition_erase_range(dataPartition, off, FLASH_SECTOR_SIZE);
        vTaskDelay(pdMS_TO_TICKS(5));  // yield to loop()
      }
      flash_write_offset       = FLASH_DATA_START;
      flash_bytes_written      = 0;
      sync_read_offset         = FLASH_DATA_START;
      flash_erase_in_progress  = false;
      flash_near_full_notified = false;
      flash_recording          = true;
      flashSaveConfig();
      vTaskDelete(NULL);
    }
    ```

### Deliverable
TREMOR: TFT warning at 90%, stops at 100%. ANXIETY: erases and resumes in ~51 sec background task, sampling uninterrupted.

---

## Phase 9 — Display Task + TFT Warnings

**Port from BLE26 with additions.**

### Tasks
- [ ] Port `displayTask()` FreeRTOS task (Core 1, priority 1)
- [ ] Port `updateDisplay()` with tftMutex + sharedMutex
- [ ] Port `drawStatusBar()`, `drawBatteryIcon()`, `drawBluetoothIcon()`
- [ ] Port PEK button wake (AXP202 interrupt)
- [ ] Port double-tap touch wake
- [ ] Port auto-off after 5 sec idle
- [ ] Add sync indicator: show "SYNCING" text when syncState == SYNC_ACTIVE
- [ ] Add flash warning display for 90% and full states
- [ ] Port RTC fallback display (`calculateDisplayTimeWithRTCFallback`)

### Deliverable
Display shows time, date, mode, tremor count, BT icon, battery. Backlight sleeps. Sync indicator appears during sync.

---

## Phase 10 — Integration + Full loop()

### Tasks
- [ ] Assemble complete `loop()` with all three streams in correct order:
  ```
  Stream 1: detectTap() + PEK + display auto-off
  Stream 1: timeCharWritten parser (mode/time/sens)
  Stream 1: 10ms sampling wait
  Stream 1: BMA423 read + IIR filter + magnitude + detection
  Stream 2: Mode-aware output (B1/B2 fill + flush)
  Stream 3: syncStep() non-blocking call
  Stream 3: flashRefreshRequested handler
  ```
- [ ] Verify no blocking calls in loop()
- [ ] Verify B1 flush (~1ms) does not break 10ms timing
- [ ] Verify sync chunk send (one notify per 50ms) does not block

### Deliverable
Full system running. Timing verified on serial (loop gap < 12ms in normal operation).

---

## Phase 11 — Persistence Verification

### Tasks
- [ ] Simulate power cycle mid-recording → verify write_offset restored
- [ ] Simulate power cycle mid-sync → verify sync resumes from saved sync_read_offset
- [ ] Simulate mode switch → verify mode survives reboot
- [ ] Simulate threshold change → verify threshold survives reboot
- [ ] Verify config slot index advances correctly
- [ ] Verify sector 0 refresh when both slot indices near full

### Deliverable
All state survives reboot. No data loss. No duplicate sync data.

---

## Phase 12 — End-to-End Testing

### Test Cases
| Test | Expected |
|------|----------|
| Anxiety live stream | 238-byte packets on imuChar at ~3.4 Hz |
| Anxiety sync on connect | 243-byte chunks on flashDataChar at 50ms intervals |
| Tremor T1 trigger | Sync starts after 15 min, T1 resets immediately |
| Sync with mid-disconnect | Resumes from saved offset on reconnect |
| REFRESH command | Immediate sync trigger |
| B2 → B1 transfer | B2 block appears in flash with 0xB2/0xB3 markers |
| Anxiety flash full | 51-sec background erase, sampling continues |
| Tremor flash 90% | TFT warning shown |
| Tremor flash 100% | Recording paused |
| Mode persist | Mode correct after reboot |
| sync_read_offset persist | Sync resumes correctly after reboot |
| Flash wear | Verified via sector write count on serial |

---

## Phase Summary

| Phase | Feature | Based On |
|-------|---------|----------|
| 0 | Constants + variables | New |
| 1 | Flash manager | BLE26 extended |
| 2 | Sampling + detection | BLE26 ported |
| 3 | B1 buffer | BLE26 extended |
| 4 | BLE stack | BLE27 extended |
| 5 | B2 buffer + anxiety stream | New |
| 6 | Time/mode/SENS parser | BLE26 ported |
| 7 | Sync state machine | New |
| 8 | Flash full handlers | New |
| 9 | Display task | BLE26 ported |
| 10 | Full loop() integration | New |
| 11 | Persistence verification | New |
| 12 | End-to-end testing | New |