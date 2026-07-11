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
  CMD_RUN,             // run wheels at given rpm + spin the ball + optional light
  CMD_HOME_AND_STOP,   // drive until HOME pulse, then stop and disable motors
  CMD_LIGHT,           // set only the light state (no motion change)
};

// Fault codes reported by a planet.
enum PlanetErr : uint8_t {
  ERR_NONE = 0,
  ERR_MOTOR_DISCONNECTED,   // a TMC driver failed test_connection()
  ERR_MOTOR_STUCK,          // commanded to move but no motion / stall detected
};

// Sun -> Planet, unicast to the planet's MAC.
struct __attribute__((packed)) CmdMsg {
  uint8_t  magic;         // = SOLAR_MAGIC
  uint8_t  type;          // = SOLAR_MSG_CMD
  uint8_t  planet_index;  // 0..7, informational (Sun maps index->MAC)
  uint8_t  command;       // PlanetCmd
  float    wheel_l_rpm;   // left wheel (outer track); sign = direction
  float    wheel_r_rpm;   // right wheel (inner track); sign = direction
  float    spin_rpm;      // planet-ball axial spin (P motor); sign = direction
  uint8_t  light_on;      // 0/1
  uint16_t current_ma;    // TMC rms current to apply to all drivers
  uint8_t  microsteps;    // TMC microsteps to apply (0 = full step, per TMCStepper)
};

// Planet -> Sun. Broadcast until the Sun's MAC is learned, then unicast to it.
struct __attribute__((packed)) StatusMsg {
  uint8_t  magic;         // = SOLAR_MAGIC
  uint8_t  type;          // = SOLAR_MSG_STATUS
  uint8_t  planet_index;  // echo of last commanded index (0xFF if never commanded)
  uint8_t  home_pulsed;   // 1 if a HOME pulse occurred since the previous status
  uint8_t  error;         // PlanetErr
  uint8_t  light_on;      // current light state
  uint8_t  moving;        // 1 if any wheel motor is currently running
};

#define PLANET_INDEX_UNKNOWN 0xFF
