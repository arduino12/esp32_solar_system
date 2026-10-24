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
#define SOLAR_MSG_OTA      'O'           // OtaMsg    (Sun  -> Planet)

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
  uint8_t  light_reserved[2]; // was the WS2812 G/B bytes: kept so the message size is unchanged
  uint16_t current_ma;    // TMC rms current to apply to all drivers
  uint8_t  driver_cfg;    // packed TMC params: see SOLAR_DRVCFG_* / solarPackDriverCfg()
  float    step_degree;   // motor step angle (deg/full-step, e.g. 1.8) -> steps/rev
};

// ------------------------ Cart firmware update (OTA) ------------------------
// Every cart build embeds the text SOLAR_FW_MARK followed by __DATE__ " " __TIME__
// ("Oct 25 2026 14:30:05"). The cart reports that build as a YYMMDDhhmm number (2610251430);
// the Sun finds the same text in an uploaded image, so both sides agree on which build it is.
#define SOLAR_FW_MARK      "SOLAR-PLANET-FW "
#define SOLAR_FW_STAMP_LEN 20                        // strlen("Oct 25 2026 14:30:05")

static inline uint32_t solarFwStamp(const char *dt) { // "Mmm dd yyyy hh:mm:ss" -> YYMMDDhhmm
  static const char MONTHS[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  uint32_t mo = 0;
  for (uint32_t k = 0; k < 12; k++)
    if (dt[0] == MONTHS[3 * k] && dt[1] == MONTHS[3 * k + 1] && dt[2] == MONTHS[3 * k + 2]) mo = k + 1;
  auto d2 = [](const char *p) -> uint32_t {            // two digits, a leading space counts as 0
    return (uint32_t)((p[0] == ' ' ? 0 : p[0] - '0') * 10 + (p[1] - '0'));
  };
  return d2(dt + 9) * 100000000u + mo * 1000000u + d2(dt + 4) * 10000u + d2(dt + 12) * 100u + d2(dt + 15);
}

// Cart OTA progress, reported in StatusMsg.ota_state.
enum OtaState : uint8_t {
  OTA_IDLE = 0,
  OTA_JOINING,         // joining the Sun's AP to download
  OTA_DOWNLOADING,     // ota_progress = 0..100 %
  OTA_DONE,            // image written and verified; rebooting into it
  OTA_FAILED,          // ota_err says why; the cart carries on with its current build
};

// Sun -> Planet: fetch a new firmware image from the Sun and reboot into it. Sent only while the
// show is idle, to one cart at a time. The carts have no Wi-Fi credentials, so the Sun's AP
// SSID/password travel here (the AP is the Sun's own installation network).
struct __attribute__((packed)) OtaMsg {
  uint8_t  magic;         // = SOLAR_MAGIC
  uint8_t  type;          // = SOLAR_MSG_OTA
  uint8_t  planet_index;
  uint32_t fw_stamp;      // build on offer (YYMMDDhhmm); a cart already running it ignores this
  char     ssid[33];
  char     pass[65];      // "" = open AP
  char     url[64];       // e.g. "http://192.168.4.1/planet.bin"
};

// Planet -> Sun. Broadcast until the Sun's MAC is learned, then unicast to it.
// The fields from fw_stamp on were added later: carts on older firmware send the message without
// them, and the Sun accepts that (missing fields read as 0, i.e. "old build").
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
  uint32_t fw_stamp;      // running build, YYMMDDhhmm (see SOLAR_FW_MARK)
  uint8_t  ota_state;     // OtaState
  uint8_t  ota_progress;  // 0..100 % while OTA_DOWNLOADING
  int16_t  ota_err;       // why OTA_FAILED: 1 = couldn't join the AP, else HTTPUpdate's error code
};
#define SOLAR_STATUS_V1_LEN 10   // StatusMsg size before fw_stamp (older cart firmware)

#define PLANET_INDEX_UNKNOWN 0xFF
