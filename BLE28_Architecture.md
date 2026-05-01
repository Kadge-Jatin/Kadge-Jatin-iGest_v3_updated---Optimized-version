# BLE28 — iGest System Architecture

## 1. Overview

BLE28 is the unified firmware for the iGest wearable tremor/anxiety detection system.
It merges BLE26 (recording + detection) and BLE27 (flash-over-BLE transfer) into a
single optimised `.ino` file with mode-aware data routing, non-blocking sync state
machine, and persistent config stored entirely within the raw flash partition.

---

## 2. Hardware Target

| Item | Value |
|------|-------|
| Device | LilyGo T-Watch 2020 V3 |
| MCU | ESP32 |
| Flash chip | SPI NOR Flash (Winbond W25Qxx) |
| Flash size | 16 MB |
| Flash endurance | ~100,000 erase cycles per 4KB sector |
| Accelerometer | BMA423 |
| Display | TFT 240×240 |
| PMU | AXP202 |
| RTC | PCF8563 |

---

## 3. Confirmed Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| N1 | 300 samples | B1 flash write buffer size |
| T1 | 15 minutes | Tremor periodic sync trigger |
| T2 | 50 ms | Inter-packet delay during sync only |
| B2 | 34 × 7 = 238 bytes | Anxiety live BLE stream buffer |
| B3 | 27 × 9 = 243 bytes | Flash-over-BLE sync chunk size |
| Sample rate | 100 Hz | BMA423 ODR |
| Sample interval | 10 ms | loop() timing |
| IIR order | 8 | Bandpass filter order |
| Tremor threshold | 100.0f | DEFAULT_TREMOR_THRESHOLD |
| Anxiety threshold | 200.0f | DEFAULT_ANXIETY_THRESHOLD |
| Crossing threshold | 6 | CROSSING_THRESHOLD |
| Baseline window | 2.0 sec | BASELINE_TIMEFRAME |
| Crossing window | 1.0 sec | CROSSING_TIMEFRAME |

---

## 4. Flash Partition Layout

```
Partition: SPIFFS data partition (found by esp_partition_find_first)

┌─────────────────────────────────────────────────────┐
│ SECTOR 0 (bytes 0 – 4095)   HEADER SECTOR           │
│                                                     │
│  HALF 1: Offset Header  (bytes 0 – 2047)            │
│    [0..3]      magic = 0xACCE1DA7                   │
│    [4..7]      slot 1  = flash_write_offset (uint32)│
│    [8..11]     slot 2  = flash_write_offset         │
│    ...                                              │
│    [2044..2047] slot 511                            │
│    Slot size  = 4 bytes                             │
│    Usable     = 511 slots                           │
│    Coverage   = 511 × 90 sec = 12.8 hours           │
│                                                     │
│  HALF 2: Config/Sync Header (bytes 2048 – 4095)     │
│    [2048..2051] config magic = 0xCF28CF07           │
│    [2052..2071] config slot 1 (20 bytes)            │
│    [2072..2091] config slot 2 (20 bytes)            │
│    ...                                              │
│    [4076..4095] config slot 102                     │
│    Slot size  = 20 bytes                            │
│    Usable     = 102 slots                           │
│    Coverage   = 102 / 10 saves/hr = 10.2 hours      │
├─────────────────────────────────────────────────────┤
│ DATA AREA (bytes 4096 – 3,203,999)                  │
│   FLASH_DATA_START = 4096                           │
│   FLASH_MAX_BYTES  = 3,200,000                      │
│   Record size      = 9 bytes                        │
│   Max records      = 355,555                        │
│   Duration @ 100Hz = ~59 min ≈ 1 hour               │
└─────────────────────────────────────────────────────┘
```

### Config Slot Structure (20 bytes)

```cpp
struct ConfigSlot {
  uint8_t  last_mode;          // 0=TREMOR, 1=ANXIETY        (1 byte)
  uint32_t sync_read_offset;   // resume pointer             (4 bytes)
  uint32_t sync_mark_offset;   // end of current sync window (4 bytes)
  uint8_t  sync_pending;       // 1 = T1 passed, not sent    (1 byte)
  uint8_t  sync_interrupted;   // 1 = dropped mid-sync       (1 byte)
  float    threshold;          // current noise threshold     (4 bytes)
  uint8_t  sync_state;         // SyncState enum value        (1 byte)
  uint8_t  valid;              // 0xAA = written              (1 byte)
  uint8_t  reserved[3];        // padding to 20 bytes         (3 bytes)
};
```

### Header Coverage Summary

| Section | Bytes | Slots | Covers |
|---------|-------|-------|--------|
| Offset header (Half 1) | 2048 | 511 × 4 bytes | 12.8 hours |
| Config header (Half 2) | 2048 | 102 × 20 bytes | 10.2 hours |
| Flash data | 3.2 MB | 355,555 records | ~1 hour @ 100 Hz |
| **Data starts at** | **offset 4096** | — | — |

> Both header halves (12.8h and 10.2h) exceed the 1-hour data limit.
> The flash data will always fill before either header overflows.

---

## 5. Record Formats (9 bytes each)

### Normal Data Record
```
[0..3]  float    mag        (accelerometer magnitude)
[4..5]  uint16   timeDiff   (ms since tremor window start)
[6..7]  uint16   rtcTime    (HHMM from RTC, e.g. 1430 = 14:30)
[8]     uint8    status     (1=in tremor window, 0=normal)
```

### Marker Records (9 bytes, special byte[0..1])
```
Tremor START:     0x89 0x67 + 0x00 × 7
Tremor END:       0xCD 0xAB + tremor_count(2) + tremor_data_count(2) + 0x00 × 3
Zero marker:      0x34 0x12 + 0x00 × 7
B2 block START:   0xB2 0xB2 + 0x00 × 7
B2 block END:     0xB3 0xB3 + 0x00 × 7
```

### BLE Sync Wire Framing (4 bytes, sent on flashDataChar)
```
START marker:  0xAA 0xBB 0xCC 0xDD
END   marker:  0xFF 0xEE 0xDD 0xCC
```

---

## 6. Buffer Definitions

### B1 — Flash Write Buffer (RAM)
```
Size:    300 samples × 9 bytes = 2700 bytes
Purpose: Accumulates IMU records, flushed to flash in one burst write
Used by: TREMOR mode (always) + offline ANXIETY mode
Flush:   When full (300 samples = every 3 seconds @ 100 Hz)
```

### B2 — Anxiety Live BLE Buffer (RAM)
```
Size:    34 samples × 7 bytes = 238 bytes
Purpose: Accumulates anxiety samples, sent as one 238-byte BLE notify
Used by: ANXIETY mode when BLE connected only
Record:  [float mag(4)][uint16 timeDiff(2)][uint8 status(1)] = 7 bytes
Flush:   When full (34 samples), sent on imuChar, then cleared immediately
```

### B3 — Sync Chunk (Wire)
```
Size:    27 records × 9 bytes = 243 bytes
Purpose: One BLE notify packet during flash-over-BLE sync
Sent on: flashDataChar every T2=50ms during SYNC_ACTIVE state
```

---

## 7. BLE Services and Characteristics

```
IMU Service        UUID: 19B10000-E8F2-537E-4F6C-D104768A1214
  imuChar          UUID: 19B10001-...   NOTIFY   238 bytes
                   ← Anxiety live stream (34×7 byte packets)

TIME Service       UUID: 19B20000-E8F2-537E-4F6C-D104768A1214
  timeChar         UUID: 19B20001-...   WRITE    up to 20 bytes
                   ← Accepts: HH:mm|ddMMyyyy
                              MODE:TREMOR / MODE:ANXIETY
                              SENS:<value>

FLASH Service      UUID: 19B30000-E8F2-537E-4F6C-D104768A1214
  flashDataChar    UUID: 19B30001-...   NOTIFY   243 bytes
                   ← Sync chunks + START/END framing markers
  flashCmdChar     UUID: 19B30002-...   WRITE    "REFRESH"
                   ← REFRESH triggers immediate sync
```

---

## 8. Mode-Aware Data Flow

### ANXIETY MODE — BLE Connected

```
Every 10ms:
  Sample → IIR filter → magnitude → detection

  OUTPUT PATH:
    Fill B2 (34 × 7 bytes)
    B2 full? → Send 238-byte packet on imuChar → Clear B2

  SYNC PATH (parallel, different characteristic):
    Check: sync_read_offset < flash_write_offset?
      YES → sync state machine active
            every T2=50ms → send 243-byte chunk on flashDataChar
            NO CLASH: imuChar and flashDataChar are independent
      NO  → no sync needed, skip

  Flash: NOT written during BLE connection
```

### ANXIETY MODE — BLE Disconnected

```
On disconnect:
  B2 has data?
    YES → Write to flash:
          1. Write 0xB2 0xB2 START marker record (9 bytes)
          2. For each B2 record:
               pad 7-byte record to 9 bytes
               add rtcTime captured at transfer time
               write to flash via B1 path
          3. Write 0xB3 0xB3 END marker record (9 bytes)
          4. Clear B2

Every 10ms:
  Sample → detection
  Fill B1 (300 × 9 bytes)
  B1 full? → Burst write to flash → Clear B1

Flash ≥ 100% (3.2 MB filled):
  Show TFT warning: "STORAGE FULL - RESETTING"
  Launch flashEraseTask (FreeRTOS, low priority, Core 1)
    → Erases data area sector by sector (780 sectors, ~51 sec total)
    → Yields 5ms between sectors so loop() keeps running at 100 Hz
  Reset flash_write_offset = FLASH_DATA_START
  Resume recording
```

### TREMOR MODE — BLE Connected OR Disconnected

```
Every 10ms (ALWAYS, regardless of connection):
  Sample → detection
  Fill B1 (300 × 9 bytes)
  B1 full? → Burst write to flash → Clear B1

Flash ≥ 90%:
  Show TFT warning: "STORAGE 90% - CONNECT TO SYNC"

Flash = 100%:
  flash_recording_paused = true
  Stop writing until RESET received

T1 SYNC LOGIC (see Section 9):
  Runs in parallel via non-blocking state machine
  Never interrupts sampling loop
```

---

## 9. Sync State Machine

### States

```cpp
typedef enum {
  SYNC_IDLE,           // Normal, T1 counting
  SYNC_WAITING_BLE,    // T1 passed, no BLE available
  SYNC_STARTING,       // BLE connected, sending START marker
  SYNC_ACTIVE,         // Sending 243-byte chunks every T2=50ms
  SYNC_INTERRUPTED,    // BLE dropped mid-sync
  SYNC_COMPLETE        // All chunks sent, sending END marker
} SyncState;
```

### Tremor Sync Flow (T1=15 min)

```
SYNC_IDLE:
  T1 elapsed?
    YES → sync_pending = true
          T1 RESETS IMMEDIATELY (new 15-min cycle starts)
          BLE connected?
            YES → sync_mark = flash_write_offset (snapshot NOW)
                  state = SYNC_STARTING
            NO  → state = SYNC_WAITING_BLE

SYNC_WAITING_BLE:
  Keep writing flash, keep advertising
  BLE connects?
    YES → sync_mark = flash_write_offset (extended to latest)
          state = SYNC_STARTING

SYNC_STARTING:
  Send START marker (0xAA 0xBB 0xCC 0xDD) on flashDataChar
  Send total_bytes (4 bytes, little-endian) on flashDataChar
  state = SYNC_ACTIVE

SYNC_ACTIVE:
  Every T2=50ms AND wasConnected:
    Read B3=243 bytes from flash[sync_read_offset]
    Notify flashDataChar
    sync_read_offset += 243
    Save sync_read_offset to config sector

    sync_read_offset >= sync_mark?
      YES → Send END marker (0xFF 0xEE 0xDD 0xCC)
            state = SYNC_COMPLETE

  BLE drops?
    → Save sync_read_offset to config sector
    → T1 already reset (was reset at trigger)
    → state = SYNC_INTERRUPTED
    → Markers intact in config sector

SYNC_INTERRUPTED:
  BLE reconnects?
    YES → sync_mark = flash_write_offset (extend to latest)
          state = SYNC_STARTING (resume from saved sync_read_offset)

SYNC_COMPLETE:
  Clear sync_pending
  Save config sector
  state = SYNC_IDLE

REFRESH command received (any time):
  → sync_mark = flash_write_offset
  → sync_pending = true
  → state = SYNC_STARTING (if BLE connected)
     or SYNC_WAITING_BLE (if not)
```

### Anxiety Sync Flow

```
On BLE connect:
  sync_read_offset < flash_write_offset?
    YES → sync_mark = flash_write_offset
          state = SYNC_STARTING
          (runs alongside live B2 stream — different characteristic)
    NO  → no sync needed

Sync proceeds identically to Tremor sync state machine above.
sync_read_offset persists in config sector, advances forward only.
```

### Sync Timeline Example (Tremor)

```
2:00  Watch starts
      flash_write_offset = A = FLASH_DATA_START
      sync_read_offset   = A
      T1 timer starts

2:15  T1 elapses, NO BLE
      sync_pending = true
      T1 RESETS → new cycle starts from 2:15
      state = SYNC_WAITING_BLE
      Flash keeps writing A → B → C...

2:20  BLE found
      sync_mark = flash_write_offset = C  ← 20 min of data
      state = SYNC_STARTING
      Sends flash[A → C] (2:00 to 2:20, all data)
      Flash keeps writing C → D...

2:20+ Sync complete
      sync_read_offset = C
      state = SYNC_IDLE
      T1 already counting since 2:15

2:35  T1 elapses (15 min since 2:15)
      sync_mark = flash_write_offset = D
      Sync sends flash[C → D] (2:20 to 2:35)
      sync_read_offset = D
```

> No data is ever lost. sync_read_offset is a forward-only watermark.

---

## 10. B2 → B1 Transfer on Disconnect (Anxiety)

```
B2 record (7 bytes):  [float mag(4)][uint16 timeDiff(2)][uint8 status(1)]
B1 record (9 bytes):  [float mag(4)][uint16 timeDiff(2)][uint16 rtcTime(2)][uint8 status(1)]

Flash layout after transfer:

[0xB2 0xB2 record]   9 bytes   ← B2 block START marker
[B2 record 0]        9 bytes   ← mag + timeDiff + rtcTime(now) + status
[B2 record 1]        9 bytes
...
[B2 record N]        9 bytes
[0xB3 0xB3 record]   9 bytes   ← B2 block END marker

Receiver sees 0xB2 0xB2 → knows next N records are padded 7-byte anxiety records
Receiver sees 0xB3 0xB3 → block ends
```

---

## 11. Flash Wear Analysis

| Metric | Value |
|--------|-------|
| Flash endurance | 100,000 erase cycles/sector |
| Sector size | 4,096 bytes |
| Sector 0 erase rate | Once per 10–13 hours |
| Sector 0 life | **114 years** |
| Data sector erase rate (continuous) | Once per hour |
| Data sector life (continuous 24h/day) | **11.4 years** |
| Data sector life (typical 8h/day) | **34 years** |
| Boot pre-erase time | ~51 seconds (780 sectors) |
| Anxiety full erase time | ~51 seconds (background task) |
| Sampling during erase | **Uninterrupted** (FreeRTOS task) |

### Erase Strategy
```
AT BOOT:
  Pre-erase all unused data sectors from write_offset onwards
  One-time ~51 sec penalty — show on TFT

DURING RECORDING:
  Pure append writes (~1ms per 300-sample flush)
  Zero erase penalty

WHEN FLASH FULL:
  ANXIETY → background erase task, sampling continues
  TREMOR  → pause logging, notify user
```

---

## 12. Flash Full Handling

| Mode | At 90% | At 100% |
|------|--------|---------|
| TREMOR | TFT warning "STORAGE 90%" | Pause logging, wait for sync + RESET |
| ANXIETY | (no 90% check) | TFT warning → full data erase → restart |

---

## 13. Persistence — Config Sector (No NVS)

All persistent state stored in flash Half 2 (bytes 2048–4095).
Uses same slot-scan pattern as BLE26 offset header.
Survives power cycles, deep sleep, and watchdog resets.

Persistent fields:
- Last connected mode (TREMOR / ANXIETY)
- Current threshold value
- sync_read_offset (sync resume pointer)
- sync_mark_offset (sync end pointer)
- sync_pending flag
- sync_interrupted flag
- sync_state enum value

---

## 14. Detection Algorithm

Unchanged from BLE26/BLE27.
Only the threshold value differs between modes.

```
1. IIR bandpass filter (8th order) applied to each axis
2. Compute acceleration magnitude: sqrt(x² + y² + z²)
3. calculateBaseline(): rolling mean over last 2.0 seconds
4. countCrossings(): zero-crossings above threshold in last 1.0 second
5. Tremor detected: crossingCount >= 6

Thresholds:
  TREMOR:  100.0f  (DEFAULT_TREMOR_THRESHOLD)
  ANXIETY: 200.0f  (DEFAULT_ANXIETY_THRESHOLD)
  SENS command can override ANXIETY threshold (50–500)
```

---

## 15. Three Parallel Streams in loop()

```
Stream 1: Sampling + Detection   [always, every 10ms]
  Read BMA423 → IIR filter → magnitude → crossing count → tremor state

Stream 2: Live Output             [mode + connection dependent]
  ANXIETY + BLE:    fill B2 → send 238-byte imuChar packet
  TREMOR + BLE:     fill B1 → burst write flash
  Any + offline:    fill B1 → burst write flash

Stream 3: Sync Step               [non-blocking, T2=50ms throttle]
  if (wasConnected && syncState==SYNC_ACTIVE && millis()-lastSyncPkt >= T2)
    read 243 bytes from flash[sync_read_offset]
    notify flashDataChar
    advance sync_read_offset
    save to config sector

No clashing:
  Stream 1 owns the 10ms sample window
  Stream 2 fills a buffer (no BLE call until full)
  Stream 3 fires at most once every 50ms
  All three are sequential within loop()
```

---

## 16. Display Task (FreeRTOS)

Unchanged from BLE26. Runs on Core 1 at priority 1.
- 1 Hz update while screen on
- Backlight wake: double-tap touch OR PEK button short press
- Screen off after 5 seconds idle
- TFT warnings added:
  - "STORAGE 90%" (Tremor mode)
  - "STORAGE FULL - RESETTING" (Anxiety mode)
  - "SYNCING..." indicator during SYNC_ACTIVE

---

## 17. RTC Fallback

Unchanged from BLE26.
- Hardware RTC synced from BLE time on each connection
- When disconnected: RTC provides time for display and rtcTime field in records
- Config sector persists last known time reference

---

## 18. Code Structure

Single `.ino` file (no `.cpp` splits), same pattern as BLE26/BLE27.

Logical sections within the file:
```
1.  Includes + defines
2.  Global variables
3.  BLE packet functions (sendPacket, markers)
4.  BLE callbacks (server, time, flashCmd)
5.  UI helpers (drawStatusBar, drawBattery, drawBluetooth)
6.  Display functions (updateDisplay, displayTask)
7.  Detection algorithm (calculateBaseline, countCrossings)
8.  IIR filter (applyIIRFilter, calculateMagnitudes)
9.  RTC fallback helpers
10. Flash manager (flashInit, flashStartRecording, flashFlushBuffer, etc.)
11. B2 transfer helper (b2TransferToFlash)
12. Config sector helpers (flashSaveConfig, flashLoadConfig)
13. Sync state machine (syncStep — called every loop())
14. Flash full handlers (Anxiety erase task, Tremor pause)
15. Flash-over-BLE (flashSendChunk — non-blocking, replaces blocking flashSendOverBLE)
16. setup()
17. loop()
```