// solar_protocol.h — shared ESP-NOW wire protocol for the Solar System installation.
// Included by BOTH src/sun_main.cpp and src/planet_main.cpp.
//
// Transport: ESP-NOW. The Sun (ESP32 WROOM-32) is the master; up to 8 planet carts
// (ESP32-C3) are addressed by MAC. Messages are fixed-size packed structs; a one-byte
// `type` field discriminates, and `magic` guards against stray/foreign packets.
#pragma once

#include <stdint.h>

#define SOLAR_MAGIC        0x53          // 'S' — sanity marker on every packet
#define SOLAR_MSG_CMD      'C'           // CmdMsg    (Sun  -> Planet)
#define SOLAR_MSG_STATUS   'S'           // StatusMsg (Planet -> Sun)
#define SOLAR_MSG_FW_REQ   'R'           // FwReqMsg  (Planet -> Sun)
#define SOLAR_MSG_FW_DATA  'D'           // FwDataMsg (Sun  -> Planet)

#define SOLAR_PLANET_COUNT 8             // number of orbits / planet carts
#define SOLAR_AP_SSID_DEFAULT "SolarSystem"   // planets scan for this to find the channel

// Overall show mode (Sun-side state machine; also surfaced in the web UI).
enum SolarMode : uint8_t {
  MODE_IDLE = 0,
  MODE_SOLAR,
  MODE_COOLDOWN,
  MODE_MENORAH,
  MODE_TEST,
};

// Command verbs sent from Sun to a planet.
enum PlanetCmd : uint8_t {
  CMD_STOP = 0,        // disable motors, lights off
  CMD_RUN,             // run wheels at given rpm + optional light
  CMD_HOME_AND_STOP,   // drive until HOME pulse, then stop and disable motors
  CMD_LIGHT,           // set only the light state (no motion change)
};

// Fault codes reported by a planet. (Motor stuck/disconnected sensing was dropped — the TMC2209
// flags aren't reliable in the quiet/low-speed operating regime — so a planet currently only ever
// reports ERR_NONE. The codes are kept for wire compatibility / future use.)
enum PlanetErr : uint8_t {
  ERR_NONE = 0,
  ERR_MOTOR_DISCONNECTED,
  ERR_MOTOR_STUCK,
};

// ------------------------ Packed driver-config byte -------------------------
// CmdMsg.driver_cfg carries all three per-move TMC parameters in one uint8:
//   bits [0..3]  microsteps as an enum = log2(microsteps): 0->1, 1->2, ... 8->256
//   bit  [4]     chopper mode: 0 = stealthChop (quiet), 1 = spreadCycle (torque/noise)
//   bit  [5]     MicroPlyer: 1 = interpolate microsteps up to 256 internally
#define SOLAR_DRVCFG_CHOP    0x10
#define SOLAR_DRVCFG_INTERP  0x20

static inline uint8_t solarMicrostepsToEnum(uint16_t m) {
  uint8_t e = 0;
  while (m > 1 && e < 8) { m >>= 1; e++; }        // 1->0, 2->1, ... 256->8
  return e;
}
static inline uint16_t solarMicrostepsFromEnum(uint8_t e) {
  return (uint16_t)1u << (e & 0x0F);              // 0->1, 1->2, ... 8->256
}
static inline uint8_t solarPackDriverCfg(uint16_t microsteps, bool spread, bool interpolate) {
  return (uint8_t)((solarMicrostepsToEnum(microsteps) & 0x0F) |
                   (spread      ? SOLAR_DRVCFG_CHOP   : 0) |
                   (interpolate ? SOLAR_DRVCFG_INTERP : 0));
}

// Sun -> Planet, unicast to the planet's MAC.
struct __attribute__((packed)) CmdMsg {
  uint8_t  magic;         // = SOLAR_MAGIC
  uint8_t  type;          // = SOLAR_MSG_CMD
  uint8_t  planet_index;  // 0..7, informational (Sun maps index->MAC)
  uint8_t  command;       // PlanetCmd
  float    wheel_l_rpm;   // left wheel (outer track, faster); sign = direction
  float    wheel_r_rpm;   // right wheel (inner track, slower); sign = direction
  uint8_t  light_on;      // 0/1
  uint8_t  light_brightness; // 0..100 % when light_on (perceptual; the cart applies gamma)
  uint16_t current_ma;    // TMC rms current to apply to all drivers
  uint8_t  driver_cfg;    // packed TMC params: see SOLAR_DRVCFG_* / solarPackDriverCfg()
  float    step_degree;   // motor step angle (deg/full-step, e.g. 1.8) -> steps/rev
  // Cart firmware on offer (see "Cart firmware update" below). fw_stamp = 0: nothing offered now
  // (no image, auto-update off, or the show is running) -> a cart pauses any download.
  uint32_t fw_stamp;      // build stamp of the Sun's cart image (UTC Unix seconds)
  uint32_t fw_size;       // image length, bytes
  uint8_t  fw_md5[16];    // MD5 of the image; the cart verifies it before booting into it
};

// ------------------------ Cart firmware update ------------------------------
// The Sun keeps the last uploaded cart image (LittleFS) and offers it in every CmdMsg. A cart
// running an OLDER build pulls the image over ESP-NOW while the show is idle: it asks for bytes at
// an offset (FwReqMsg) and the Sun answers with up to `count` chunks (FwDataMsg). The Sun keeps
// no per-cart state, so any number of carts download at once. No Wi-Fi join. A cart flashed with
// a newer build (e.g. over USB while testing) is left alone.
//
// Build stamp: the build time in UTC Unix seconds (SOLAR_FW_BUILD_UTC, set by
// scripts/fw_build_stamp.py), so it only ever increases. Every cart build embeds SOLAR_FW_MARK
// followed by the stamp as 10 decimal digits; the Sun reads it out of an uploaded image.
#define SOLAR_FW_MARK      "SOLAR-PLANET-FW "
#define SOLAR_FW_STAMP_LEN 10                        // decimal digits of the Unix time

static inline uint32_t solarFwStamp(const char *digits) {   // the 10 digits after SOLAR_FW_MARK
  uint32_t v = 0;
  for (int k = 0; k < SOLAR_FW_STAMP_LEN; k++) {
    if (digits[k] < '0' || digits[k] > '9') return 0;
    v = v * 10 + (uint32_t)(digits[k] - '0');
  }
  return v;
}

// Cart download progress, reported in StatusMsg.ota_state.
enum OtaState : uint8_t {
  OTA_IDLE = 0,
  OTA_DOWNLOADING,     // ota_progress = 0..100 % (also while paused for a running show)
  OTA_DONE,            // image written and MD5-verified; rebooting into it
  OTA_FAILED,          // ota_err says why; the cart retries later with its current build running
};

#define SOLAR_FW_CHUNK 232                           // image bytes per FwDataMsg (fits 250 B)

// Planet -> Sun: "send me `count` chunks of image `fw_stamp` starting at `offset`".
struct __attribute__((packed)) FwReqMsg {
  uint8_t  magic;         // = SOLAR_MAGIC
  uint8_t  type;          // = SOLAR_MSG_FW_REQ
  uint32_t fw_stamp;
  uint32_t offset;
  uint8_t  count;
};

// Sun -> Planet: one chunk. Sent with only `len` bytes of data (the tail of the struct is cut).
struct __attribute__((packed)) FwDataMsg {
  uint8_t  magic;         // = SOLAR_MAGIC
  uint8_t  type;          // = SOLAR_MSG_FW_DATA
  uint32_t fw_stamp;
  uint32_t offset;
  uint8_t  len;
  uint8_t  data[SOLAR_FW_CHUNK];
};
#define SOLAR_FW_DATA_HDR (sizeof(FwDataMsg) - SOLAR_FW_CHUNK)

// Planet -> Sun. Broadcast until the Sun's MAC is learned, then unicast to it.
struct __attribute__((packed)) StatusMsg {
  uint8_t  magic;         // = SOLAR_MAGIC
  uint8_t  type;          // = SOLAR_MSG_STATUS
  uint8_t  planet_index;  // echo of last commanded index (0xFF if never commanded)
  uint8_t  home_pulsed;   // 1 if a HOME pulse occurred since the previous status
  uint8_t  error;         // PlanetErr
  uint8_t  light_on;      // current light state
  uint8_t  moving;        // 1 if any wheel motor is currently running
  int8_t   temp_c;        // ESP32-C3 internal die temperature, °C (INT8_MIN = unknown)
  int8_t   rssi;          // last measured Sun-AP signal, dBm (0 = unknown)
  uint8_t  cfg_ok;        // 1 = last driver-param apply verified over UART (read-back + IFCNT)
  uint32_t fw_stamp;      // running build stamp, UTC Unix seconds (see SOLAR_FW_MARK)
  uint8_t  ota_state;     // OtaState
  uint8_t  ota_progress;  // 0..100 % while OTA_DOWNLOADING
  int16_t  ota_err;       // why OTA_FAILED: an Update library error, or OTA_ERR_* below
};
#define OTA_ERR_NO_DATA  100     // the Sun stopped answering chunk requests
#define OTA_ERR_BEGIN    101     // couldn't open the cart's OTA partition (image too big?)

#define PLANET_INDEX_UNKNOWN 0xFF
