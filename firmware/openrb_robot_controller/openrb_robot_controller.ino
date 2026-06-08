#include <Servo.h>
#include <TinyGPS++.h>
#include <math.h>

// ---- IMU diagnostic integration (Task D) ----
// Read-only IMU diagnostics on the Wire bus (D11 SDA / D12 SCL). Disabled by
// default: with IMU_ENABLE=0 there is no Wire include, no IMU code, and the
// default controller behaviour is unchanged. IMU yaw here is a RELATIVE,
// drifting diagnostic and is never used to drive motors.
#ifndef IMU_ENABLE
#define IMU_ENABLE 0
#endif
#ifndef IMU_HEADING_DRYRUN
#define IMU_HEADING_DRYRUN 0
#endif
#ifndef IMU_YAW_DIAG
#define IMU_YAW_DIAG 0
#endif
#ifndef IMU_CALIBRATION_SECONDS
#define IMU_CALIBRATION_SECONDS 5
#endif
#ifndef IMU_YAW_AXIS
#define IMU_YAW_AXIS 2
#endif
#ifndef IMU_YAW_SIGN
#define IMU_YAW_SIGN 1
#endif
#ifndef IMU_I2C_ADDR
#define IMU_I2C_ADDR 0x69
#endif
#ifndef IMU_GYRO_LSB_PER_DPS
#define IMU_GYRO_LSB_PER_DPS 131.0
#endif
#ifndef IMU_ACCEL_LSB_PER_G
#define IMU_ACCEL_LSB_PER_G 16384.0
#endif
// HEADING_SOURCE_MODE: 0=GPS_COURSE (default/existing), 1=IMU_RELATIVE,
// 2=GPS_COURSE_WITH_IMU_DIAG, 3=MOCK_HEADING, 4=FUSION_DIAG. Only GPS-course
// modes (0/2) are approved to drive motors; the rest are diagnostic-only.
#ifndef HEADING_SOURCE_MODE
#define HEADING_SOURCE_MODE 0
#endif
#ifndef MOCK_HEADING_DEG
#define MOCK_HEADING_DEG 0.0
#endif
#if IMU_ENABLE
#include <Wire.h>
#endif

#ifndef FIXED_WIRING_GPS_SERIAL2_DIAG
#define FIXED_WIRING_GPS_SERIAL2_DIAG 0
#endif

#ifndef FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN
#define FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN 0
#endif

#ifndef FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT
#define FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT 0
#endif

#ifndef MOTOR_PULSE_TEST_MODE
#define MOTOR_PULSE_TEST_MODE 0
#endif

#ifndef MANUAL_RC_RECOVERY
#define MANUAL_RC_RECOVERY 0
#endif

#ifndef AUTO_MOTION_ARMED
#define AUTO_MOTION_ARMED 0
#endif

#ifndef MOTOR_PULSE_CMD
#define MOTOR_PULSE_CMD 0.15
#endif

#ifndef MOTOR_PULSE_LEFT_CMD
#define MOTOR_PULSE_LEFT_CMD MOTOR_PULSE_CMD
#endif

#ifndef MOTOR_PULSE_RIGHT_CMD
#define MOTOR_PULSE_RIGHT_CMD MOTOR_PULSE_CMD
#endif

#ifndef MOTOR_PULSE_MS
#define MOTOR_PULSE_MS 300
#endif

#ifndef DRIVE_CALIBRATION_ENABLE
#define DRIVE_CALIBRATION_ENABLE 0
#endif

#ifndef LEFT_MOTOR_SIGN
#define LEFT_MOTOR_SIGN 1
#endif

#ifndef RIGHT_MOTOR_SIGN
#define RIGHT_MOTOR_SIGN 1
#endif

#ifndef LEFT_MOTOR_SCALE
#define LEFT_MOTOR_SCALE 1.0
#endif

#ifndef RIGHT_MOTOR_SCALE
#define RIGHT_MOTOR_SCALE 1.0
#endif

#ifndef LEFT_MOTOR_MIN_CMD
#define LEFT_MOTOR_MIN_CMD 0.0
#endif

#ifndef RIGHT_MOTOR_MIN_CMD
#define RIGHT_MOTOR_MIN_CMD 0.0
#endif

#ifndef MOTOR_OUTPUT_SWAP_LR
#define MOTOR_OUTPUT_SWAP_LR 0
#endif

#ifndef MANUAL_FORWARD_SIGN
#define MANUAL_FORWARD_SIGN -1
#endif

#ifndef MANUAL_TURN_SIGN
#define MANUAL_TURN_SIGN 1
#endif

#ifndef GROUND_CRAWL_TEST_MODE
#define GROUND_CRAWL_TEST_MODE 0
#endif

#ifndef STAGE15_GUARDED_CRAWL_TEST
#define STAGE15_GUARDED_CRAWL_TEST 0
#endif

#ifndef STAGE15_MAX_CMD
#define STAGE15_MAX_CMD 0.06
#endif

#ifndef STAGE15_PULSE_MS
#define STAGE15_PULSE_MS 200
#endif

#ifndef STAGE15_INTER_PULSE_MS
#define STAGE15_INTER_PULSE_MS 2000
#endif

#ifndef STAGE15_REQUIRE_RC_NEUTRAL
#define STAGE15_REQUIRE_RC_NEUTRAL 0
#endif

#ifndef STAGE16_USB_GUARDED_CRAWL
#define STAGE16_USB_GUARDED_CRAWL 0
#endif

#ifndef STAGE16_MAX_CMD
#define STAGE16_MAX_CMD 0.04
#endif

#ifndef STAGE16_MAX_MS
#define STAGE16_MAX_MS 150
#endif

#ifndef STAGE17_FIRST_PRIMITIVE_CRAWL
#define STAGE17_FIRST_PRIMITIVE_CRAWL 0
#endif

#ifndef STAGE17_MAX_CMD
#define STAGE17_MAX_CMD 0.06
#endif

#ifndef STAGE17_MAX_MS
#define STAGE17_MAX_MS 500
#endif

#ifndef STAGE18_MOTOR_MAPPING_PROBE
#define STAGE18_MOTOR_MAPPING_PROBE 0
#endif

#ifndef STAGE18_MAX_CMD
#define STAGE18_MAX_CMD 0.08
#endif

#ifndef STAGE18_MAX_MS
#define STAGE18_MAX_MS 800
#endif

#ifndef STATION_DRIVE_GUARDED
#define STATION_DRIVE_GUARDED 0
#endif

#ifndef STATION_DRIVE_IGNORE_RC_INPUT
#define STATION_DRIVE_IGNORE_RC_INPUT 0
#endif

#ifndef STATION_DRIVE_MAX_ABS_A
#define STATION_DRIVE_MAX_ABS_A 0.35
#endif

#ifndef STATION_DRIVE_MAX_ABS_B
#define STATION_DRIVE_MAX_ABS_B 0.35
#endif

#ifndef STATION_DRIVE_MAX_MS
#define STATION_DRIVE_MAX_MS 1000
#endif

#ifndef USB_PULSE_TEST_GUARDED
#define USB_PULSE_TEST_GUARDED 0
#endif

#ifndef USB_PULSE_TEST_IGNORE_RC_INPUT
#define USB_PULSE_TEST_IGNORE_RC_INPUT 0
#endif

#ifndef USB_PULSE_TEST_MAX_ABS_A
#define USB_PULSE_TEST_MAX_ABS_A 0.35
#endif

#ifndef USB_PULSE_TEST_MAX_ABS_B
#define USB_PULSE_TEST_MAX_ABS_B 0.35
#endif

#ifndef USB_PULSE_TEST_MAX_MS
#define USB_PULSE_TEST_MAX_MS 1000
#endif

#ifndef STATION_HW_MANUAL_ENABLE
#define STATION_HW_MANUAL_ENABLE 0
#endif

#ifndef STATION_HW_MANUAL_A_B_MAPPING
#define STATION_HW_MANUAL_A_B_MAPPING 0
#endif

#ifndef STATION_HW_MANUAL_IGNORE_RC_INPUT
#define STATION_HW_MANUAL_IGNORE_RC_INPUT 0
#endif

#ifndef STATION_HW_MANUAL_DIAGNOSE_ONLY
#define STATION_HW_MANUAL_DIAGNOSE_ONLY 0
#endif

#ifndef STATION_HW_MANUAL_MAX_ABS_A
#define STATION_HW_MANUAL_MAX_ABS_A 0.35
#endif

#ifndef STATION_HW_MANUAL_MAX_ABS_B
#define STATION_HW_MANUAL_MAX_ABS_B 0.35
#endif

#ifndef STAGE20_PHYSICAL_AB_GUARDED_CRAWL
#define STAGE20_PHYSICAL_AB_GUARDED_CRAWL 0
#endif

#ifndef STAGE20_MAX_ABS_A
#define STAGE20_MAX_ABS_A 0.25
#endif

#ifndef STAGE20_MAX_ABS_B
#define STAGE20_MAX_ABS_B 0.25
#endif

#ifndef STAGE20_MAX_MS
#define STAGE20_MAX_MS 300
#endif

#ifndef STATION_MANUAL_IGNORE_RC_INPUT
#define STATION_MANUAL_IGNORE_RC_INPUT 0
#endif

#if STATION_DRIVE_GUARDED
#undef STAGE20_PHYSICAL_AB_GUARDED_CRAWL
#define STAGE20_PHYSICAL_AB_GUARDED_CRAWL 1
#undef STAGE20_MAX_ABS_A
#define STAGE20_MAX_ABS_A STATION_DRIVE_MAX_ABS_A
#undef STAGE20_MAX_ABS_B
#define STAGE20_MAX_ABS_B STATION_DRIVE_MAX_ABS_B
#undef STAGE20_MAX_MS
#define STAGE20_MAX_MS STATION_DRIVE_MAX_MS
#undef STATION_MANUAL_IGNORE_RC_INPUT
#define STATION_MANUAL_IGNORE_RC_INPUT STATION_DRIVE_IGNORE_RC_INPUT
#endif

#if USB_PULSE_TEST_GUARDED
#undef STAGE20_PHYSICAL_AB_GUARDED_CRAWL
#define STAGE20_PHYSICAL_AB_GUARDED_CRAWL 1
#undef STAGE20_MAX_ABS_A
#define STAGE20_MAX_ABS_A USB_PULSE_TEST_MAX_ABS_A
#undef STAGE20_MAX_ABS_B
#define STAGE20_MAX_ABS_B USB_PULSE_TEST_MAX_ABS_B
#undef STAGE20_MAX_MS
#define STAGE20_MAX_MS USB_PULSE_TEST_MAX_MS
#undef STATION_MANUAL_IGNORE_RC_INPUT
#define STATION_MANUAL_IGNORE_RC_INPUT USB_PULSE_TEST_IGNORE_RC_INPUT
#endif

#if STAGE15_GUARDED_CRAWL_TEST && \
    (PHYSICAL_PATH_FOLLOWING_ENABLE != 0 || PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT != 0 || \
     PATH_FOLLOWING_DRYRUN != 0 || GROUND_CRAWL_TEST_MODE != 0 || AUTO_MOTION_ARMED != 0 || \
     STAGE16_USB_GUARDED_CRAWL != 0 || STAGE17_FIRST_PRIMITIVE_CRAWL != 0 || \
     STAGE18_MOTOR_MAPPING_PROBE != 0 || STAGE20_PHYSICAL_AB_GUARDED_CRAWL != 0)
#error "Stage 15 guarded crawl must not enable path following, autonomous motion, or path motor-output gates."
#endif

#if STAGE16_USB_GUARDED_CRAWL && \
    (PHYSICAL_PATH_FOLLOWING_ENABLE != 0 || PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT != 0 || \
     PATH_FOLLOWING_DRYRUN != 0 || GROUND_CRAWL_TEST_MODE != 0 || AUTO_MOTION_ARMED != 0 || \
     STAGE15_GUARDED_CRAWL_TEST != 0 || STAGE17_FIRST_PRIMITIVE_CRAWL != 0 || \
     STAGE18_MOTOR_MAPPING_PROBE != 0 || STAGE20_PHYSICAL_AB_GUARDED_CRAWL != 0)
#error "Stage 16 guarded crawl must not enable path following, autonomous motion, Stage 15, or path motor-output gates."
#endif

#if STAGE17_FIRST_PRIMITIVE_CRAWL && \
    (PHYSICAL_PATH_FOLLOWING_ENABLE != 0 || PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT != 0 || \
     PATH_FOLLOWING_DRYRUN != 0 || GROUND_CRAWL_TEST_MODE != 0 || AUTO_MOTION_ARMED != 0 || \
     STAGE15_GUARDED_CRAWL_TEST != 0 || STAGE16_USB_GUARDED_CRAWL != 0 || \
     STAGE18_MOTOR_MAPPING_PROBE != 0 || STAGE20_PHYSICAL_AB_GUARDED_CRAWL != 0)
#error "Stage 17 first-primitive crawl must not enable path following, autonomous motion, Stage 15, Stage 16, or path motor-output gates."
#endif

#if STAGE18_MOTOR_MAPPING_PROBE && \
    (PHYSICAL_PATH_FOLLOWING_ENABLE != 0 || PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT != 0 || \
     PATH_FOLLOWING_DRYRUN != 0 || GROUND_CRAWL_TEST_MODE != 0 || AUTO_MOTION_ARMED != 0 || \
     STAGE15_GUARDED_CRAWL_TEST != 0 || STAGE16_USB_GUARDED_CRAWL != 0 || \
     STAGE17_FIRST_PRIMITIVE_CRAWL != 0 || STAGE20_PHYSICAL_AB_GUARDED_CRAWL != 0)
#error "Stage 18 motor mapping probe must not enable path following, autonomous motion, Stage 15, Stage 16, Stage 17, or path motor-output gates."
#endif

#if STAGE20_PHYSICAL_AB_GUARDED_CRAWL && \
    (PHYSICAL_PATH_FOLLOWING_ENABLE != 0 || PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT != 0 || \
     PATH_FOLLOWING_DRYRUN != 0 || GROUND_CRAWL_TEST_MODE != 0 || AUTO_MOTION_ARMED != 0 || \
     STAGE15_GUARDED_CRAWL_TEST != 0 || STAGE16_USB_GUARDED_CRAWL != 0 || \
     STAGE17_FIRST_PRIMITIVE_CRAWL != 0 || STAGE18_MOTOR_MAPPING_PROBE != 0)
#error "Stage 20 physical A/B guarded crawl must not enable path following, autonomous motion, Stage 15, Stage 16, Stage 17, Stage 18, or path motor-output gates."
#endif

#if MANUAL_RC_RECOVERY && \
    (PHYSICAL_PATH_FOLLOWING_ENABLE != 0 || PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT != 0 || \
     PATH_FOLLOWING_DRYRUN != 0 || GROUND_CRAWL_TEST_MODE != 0 || AUTO_MOTION_ARMED != 0 || \
     STAGE15_GUARDED_CRAWL_TEST != 0 || STAGE16_USB_GUARDED_CRAWL != 0 || \
     STAGE17_FIRST_PRIMITIVE_CRAWL != 0 || STAGE18_MOTOR_MAPPING_PROBE != 0 || \
     STAGE20_PHYSICAL_AB_GUARDED_CRAWL != 0)
#error "Manual RC recovery must not enable path following, autonomous motion, or guarded station pulse modes."
#endif

#ifndef GROUND_CRAWL_MAX_CMD
#define GROUND_CRAWL_MAX_CMD 0.08
#endif

#ifndef GROUND_CRAWL_MAX_AUTO_MS
#define GROUND_CRAWL_MAX_AUTO_MS 1200
#endif

#ifndef SINGLE_WP_CRAWL_BASE_CMD
#define SINGLE_WP_CRAWL_BASE_CMD 0.100
#endif

#ifndef SINGLE_WP_STEERING_DRYRUN
#define SINGLE_WP_STEERING_DRYRUN 0
#endif

#ifndef COURSE_MIN_DISPLACEMENT_M
#define COURSE_MIN_DISPLACEMENT_M 2.0
#endif

#ifndef GPS_COURSE_LATCH_DIAG
#define GPS_COURSE_LATCH_DIAG 1
#endif

#ifndef GPS_COURSE_LATCH_DIAG_MAX_AGE_MS
#define GPS_COURSE_LATCH_DIAG_MAX_AGE_MS 5000
#endif

#ifndef GPS_COURSE_OUTPUT_LATCH_MS
#define GPS_COURSE_OUTPUT_LATCH_MS GPS_COURSE_LATCH_DIAG_MAX_AGE_MS
#endif

// 0-based PPM index of the Manual/Auto switch channel. Default 4 = receiver CH5.
// Override only after firmware/ppm_channel_map_probe proves a stable 2-position
// switch channel. This does not weaken failsafe or change motion gates.
#ifndef MODE_CHANNEL_INDEX
#define MODE_CHANNEL_INDEX 4
#endif

// ---- Path-following dry-run / guarded execution (Tasks E/F/G) ----
// PATH_FOLLOWING_DRYRUN is a self-contained top-level mode. It computes waypoint
// distance/bearing/heading/steering diagnostics and NEVER drives motors unless
// every gate below is satisfied. All four motion gates default to 0, so the
// default build (and the dry-run build) can never move motors.
#ifndef PATH_FOLLOWING_DRYRUN
#define PATH_FOLLOWING_DRYRUN 0
#endif
#ifndef PHYSICAL_PATH_FOLLOWING_ENABLE
#define PHYSICAL_PATH_FOLLOWING_ENABLE 0
#endif
#ifndef PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT
#define PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT 0
#endif
// Operator acknowledgement that the RC/PPM Manual/Auto channel is stable. The
// current hardware blocker (CH5 did not hold HIGH) means this must stay 0 until
// firmware/ppm_channel_map_probe proves a stable 2-position switch.
#ifndef PATH_FOLLOWING_MODE_CHANNEL_STABLE
#define PATH_FOLLOWING_MODE_CHANNEL_STABLE 0
#endif
// Allow a compile-time mock current position so the dry-run works indoors with
// no GPS fix. Mock position cannot produce a heading, so it can never satisfy
// the physical-output gate.
#ifndef PATH_FOLLOWING_ALLOW_MOCK_POSITION
#define PATH_FOLLOWING_ALLOW_MOCK_POSITION 1
#endif
#ifndef PATH_FOLLOWING_MOCK_CURRENT_LAT
#define PATH_FOLLOWING_MOCK_CURRENT_LAT 35.5705500
#endif
#ifndef PATH_FOLLOWING_MOCK_CURRENT_LON
#define PATH_FOLLOWING_MOCK_CURRENT_LON 129.1871000
#endif
// Hard caps for any guarded physical output.
#ifndef PATH_FOLLOWING_MAX_FORWARD_CMD
#define PATH_FOLLOWING_MAX_FORWARD_CMD 0.18
#endif
#ifndef PATH_FOLLOWING_MAX_TURN_CMD
#define PATH_FOLLOWING_MAX_TURN_CMD 0.04
#endif
#ifndef PATH_FOLLOWING_MAX_AUTO_MS
#define PATH_FOLLOWING_MAX_AUTO_MS 500
#endif
#ifndef PATH_FOLLOWING_ARRIVAL_RADIUS_M
#define PATH_FOLLOWING_ARRIVAL_RADIUS_M 2.0
#endif
#ifndef PATH_FOLLOWING_MIN_TARGET_DISTANCE_M
#define PATH_FOLLOWING_MIN_TARGET_DISTANCE_M 3.0
#endif
#ifndef PATH_FOLLOWING_MAX_TARGET_DISTANCE_M
#define PATH_FOLLOWING_MAX_TARGET_DISTANCE_M 20.0
#endif
// HC-12 waypoint protocol for the dry-run. It must be on a UART different from
// GPS (Serial2); 1=Serial1, 3=Serial3. Serial2 is rejected at compile time.
#ifndef PATH_FOLLOWING_HC12_ENABLED
#define PATH_FOLLOWING_HC12_ENABLED 1
#endif
#ifndef PATH_FOLLOWING_HC12_SERIAL_PORT
#define PATH_FOLLOWING_HC12_SERIAL_PORT 3
#endif
#ifndef PATH_FOLLOWING_HC12_CMD_STALE_MS
#define PATH_FOLLOWING_HC12_CMD_STALE_MS 2000
#endif
// Compile-time path (override per-waypoint with -D...). Defaults are real nearby
// coordinates from the documented test site, forming a short multi-leg path.
#ifndef PATH_FOLLOWING_WP_COUNT
#define PATH_FOLLOWING_WP_COUNT 3
#endif
#ifndef PATH_FOLLOWING_WP1_LAT
#define PATH_FOLLOWING_WP1_LAT 35.5706361
#endif
#ifndef PATH_FOLLOWING_WP1_LON
#define PATH_FOLLOWING_WP1_LON 129.1870540
#endif
#ifndef PATH_FOLLOWING_WP2_LAT
#define PATH_FOLLOWING_WP2_LAT 35.5707680
#endif
#ifndef PATH_FOLLOWING_WP2_LON
#define PATH_FOLLOWING_WP2_LON 129.1867906
#endif
#ifndef PATH_FOLLOWING_WP3_LAT
#define PATH_FOLLOWING_WP3_LAT 35.5705010
#endif
#ifndef PATH_FOLLOWING_WP3_LON
#define PATH_FOLLOWING_WP3_LON 129.1872696
#endif

#define STRINGIFY_VALUE_IMPL(value) #value
#define STRINGIFY_VALUE(value) STRINGIFY_VALUE_IMPL(value)

#define ENABLE_GPS_TELEMETRY 1
#if MOTOR_PULSE_TEST_MODE || FIXED_WIRING_GPS_SERIAL2_DIAG || \
    FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN || \
    FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT || PATH_FOLLOWING_DRYRUN || \
    STAGE15_GUARDED_CRAWL_TEST || STAGE16_USB_GUARDED_CRAWL || \
    STAGE17_FIRST_PRIMITIVE_CRAWL || STAGE18_MOTOR_MAPPING_PROBE || \
    STAGE20_PHYSICAL_AB_GUARDED_CRAWL || MANUAL_RC_RECOVERY
// GPS owns Serial2 here. The legacy station HC-12 command stack (which can drive
// motors via CMD,MANUAL/CMD,AUTO) stays disabled. Path-following mode runs its
// own motor-free HC-12 waypoint protocol on a separate UART (Serial1/Serial3).
#define HC12_LINK_ENABLED 0
#define GPS_SERIAL Serial2
#else
#define HC12_LINK_ENABLED 1
#define HC12_SERIAL Serial2
#define GPS_SERIAL Serial3
#endif

constexpr const char *FIRMWARE_ID = "openrb_robot_controller station-hw-manual usb-pulse-test rc-arcade-manual-fwdneg 2026-05-30";
constexpr uint8_t PPM_PIN = 6;
constexpr uint8_t ESC_LEFT_PIN = 4;
constexpr uint8_t ESC_RIGHT_PIN = 5;
constexpr long HC12_BAUD = 9600;
constexpr long GPS_BAUD = 9600;
constexpr long USB_BAUD = 115200;

constexpr uint32_t GPS_DRYRUN_STALE_MS = 2000;
constexpr uint8_t GPS_DRYRUN_MIN_SATS = 4;
constexpr double GPS_DRYRUN_MAX_HDOP = 6.0;
constexpr uint32_t GPS_MOTION_STALE_MS = 2000;
constexpr uint8_t GPS_MOTION_MIN_SATS = 5;
constexpr double GPS_MOTION_MAX_HDOP = 2.5;
constexpr uint32_t GPS_STALE_MS = GPS_MOTION_STALE_MS;
constexpr uint8_t GPS_MIN_SATS = GPS_MOTION_MIN_SATS;
constexpr double GPS_MAX_HDOP = GPS_MOTION_MAX_HDOP;
constexpr bool MOTOR_PULSE_ENABLED = MOTOR_PULSE_TEST_MODE != 0;
constexpr bool MANUAL_RC_RECOVERY_ENABLED = MANUAL_RC_RECOVERY != 0;
constexpr float MOTOR_PULSE_CMD_VALUE = MOTOR_PULSE_CMD;
constexpr float MOTOR_PULSE_LEFT_CMD_VALUE = MOTOR_PULSE_LEFT_CMD;
constexpr float MOTOR_PULSE_RIGHT_CMD_VALUE = MOTOR_PULSE_RIGHT_CMD;
constexpr uint32_t MOTOR_PULSE_MS_VALUE = MOTOR_PULSE_MS;
constexpr bool DRIVE_CALIBRATION_ENABLED = DRIVE_CALIBRATION_ENABLE != 0;
constexpr float LEFT_MOTOR_SIGN_VALUE = LEFT_MOTOR_SIGN;
constexpr float RIGHT_MOTOR_SIGN_VALUE = RIGHT_MOTOR_SIGN;
constexpr float LEFT_MOTOR_SCALE_VALUE = LEFT_MOTOR_SCALE;
constexpr float RIGHT_MOTOR_SCALE_VALUE = RIGHT_MOTOR_SCALE;
constexpr float LEFT_MOTOR_MIN_CMD_VALUE = LEFT_MOTOR_MIN_CMD;
constexpr float RIGHT_MOTOR_MIN_CMD_VALUE = RIGHT_MOTOR_MIN_CMD;
constexpr bool MOTOR_OUTPUT_SWAP_LR_ENABLED = MOTOR_OUTPUT_SWAP_LR != 0;
constexpr float MANUAL_FORWARD_SIGN_VALUE = MANUAL_FORWARD_SIGN;
constexpr float MANUAL_TURN_SIGN_VALUE = MANUAL_TURN_SIGN;
constexpr bool STAGE15_GUARDED_CRAWL_ENABLED = STAGE15_GUARDED_CRAWL_TEST != 0;
constexpr float STAGE15_MAX_CMD_VALUE = STAGE15_MAX_CMD;
constexpr uint32_t STAGE15_PULSE_MS_VALUE = STAGE15_PULSE_MS;
constexpr uint32_t STAGE15_INTER_PULSE_MS_VALUE = STAGE15_INTER_PULSE_MS;
constexpr bool STAGE15_REQUIRE_RC_NEUTRAL_VALUE = STAGE15_REQUIRE_RC_NEUTRAL != 0;
constexpr bool STAGE16_USB_GUARDED_CRAWL_ENABLED = STAGE16_USB_GUARDED_CRAWL != 0;
constexpr float STAGE16_MAX_CMD_VALUE = STAGE16_MAX_CMD;
constexpr uint32_t STAGE16_MAX_MS_VALUE = STAGE16_MAX_MS;
constexpr bool STAGE17_FIRST_PRIMITIVE_CRAWL_ENABLED = STAGE17_FIRST_PRIMITIVE_CRAWL != 0;
constexpr float STAGE17_MAX_CMD_VALUE = STAGE17_MAX_CMD;
constexpr uint32_t STAGE17_MAX_MS_VALUE = STAGE17_MAX_MS;
constexpr bool STAGE18_MOTOR_MAPPING_PROBE_ENABLED = STAGE18_MOTOR_MAPPING_PROBE != 0;
constexpr float STAGE18_MAX_CMD_VALUE = STAGE18_MAX_CMD;
constexpr uint32_t STAGE18_MAX_MS_VALUE = STAGE18_MAX_MS;
constexpr bool STATION_DRIVE_GUARDED_ENABLED = STATION_DRIVE_GUARDED != 0;
constexpr bool USB_PULSE_TEST_GUARDED_ENABLED = USB_PULSE_TEST_GUARDED != 0;
constexpr bool STATION_HW_MANUAL_ENABLED = STATION_HW_MANUAL_ENABLE != 0;
constexpr bool STATION_HW_MANUAL_A_B_MAPPING_ENABLED =
    STATION_HW_MANUAL_ENABLE != 0 && STATION_HW_MANUAL_A_B_MAPPING != 0;
constexpr bool STATION_HW_MANUAL_DIAGNOSE_ONLY_ENABLED =
    STATION_HW_MANUAL_ENABLE != 0 && STATION_HW_MANUAL_DIAGNOSE_ONLY != 0;
constexpr float STATION_HW_MANUAL_MAX_ABS_A_VALUE = STATION_HW_MANUAL_MAX_ABS_A;
constexpr float STATION_HW_MANUAL_MAX_ABS_B_VALUE = STATION_HW_MANUAL_MAX_ABS_B;
constexpr bool STAGE20_PHYSICAL_AB_GUARDED_CRAWL_ENABLED = STAGE20_PHYSICAL_AB_GUARDED_CRAWL != 0;
constexpr bool STATION_MANUAL_IGNORE_RC_INPUT_ENABLED =
    STAGE20_PHYSICAL_AB_GUARDED_CRAWL != 0 && STATION_MANUAL_IGNORE_RC_INPUT != 0;
constexpr float STAGE20_MAX_ABS_A_VALUE = STAGE20_MAX_ABS_A;
constexpr float STAGE20_MAX_ABS_B_VALUE = STAGE20_MAX_ABS_B;
constexpr uint32_t STAGE20_MAX_MS_VALUE = STAGE20_MAX_MS;
constexpr bool STAGE_USB_GUARDED_CRAWL_ENABLED =
    STAGE16_USB_GUARDED_CRAWL != 0 || STAGE17_FIRST_PRIMITIVE_CRAWL != 0 ||
    STAGE18_MOTOR_MAPPING_PROBE != 0 || STAGE20_PHYSICAL_AB_GUARDED_CRAWL != 0;
constexpr float STAGE_USB_GUARDED_MAX_CMD_VALUE =
    STAGE20_PHYSICAL_AB_GUARDED_CRAWL ? STAGE20_MAX_ABS_A :
    (STAGE18_MOTOR_MAPPING_PROBE ? STAGE18_MAX_CMD : (STAGE17_FIRST_PRIMITIVE_CRAWL ? STAGE17_MAX_CMD : STAGE16_MAX_CMD));
constexpr float STAGE_USB_GUARDED_MAX_B_VALUE =
    STAGE20_PHYSICAL_AB_GUARDED_CRAWL ? STAGE20_MAX_ABS_B : STAGE_USB_GUARDED_MAX_CMD_VALUE;
constexpr uint32_t STAGE_USB_GUARDED_MAX_MS_VALUE =
    STAGE20_PHYSICAL_AB_GUARDED_CRAWL ? STAGE20_MAX_MS :
    (STAGE18_MOTOR_MAPPING_PROBE ? STAGE18_MAX_MS : (STAGE17_FIRST_PRIMITIVE_CRAWL ? STAGE17_MAX_MS : STAGE16_MAX_MS));
constexpr bool DRYRUN_TARGET_AVAILABLE = true;
constexpr double DRYRUN_TARGET_LAT = 35.571120;
constexpr double DRYRUN_TARGET_LON = 129.186050;
constexpr uint32_t DRYRUN_GPS_READY_MAX_AGE_MS = GPS_DRYRUN_STALE_MS;
constexpr bool SINGLE_WAYPOINT_TARGET_AVAILABLE = true;
constexpr double SINGLE_WAYPOINT_FALLBACK_TARGET_LAT = 35.571120;
constexpr double SINGLE_WAYPOINT_FALLBACK_TARGET_LON = 129.186050;
#if defined(SINGLE_WP_TARGET_LAT) && defined(SINGLE_WP_TARGET_LON)
constexpr bool SINGLE_WAYPOINT_TARGET_OVERRIDE_ENABLED = true;
constexpr double SINGLE_WAYPOINT_TARGET_LAT = SINGLE_WP_TARGET_LAT;
constexpr double SINGLE_WAYPOINT_TARGET_LON = SINGLE_WP_TARGET_LON;
constexpr const char *SINGLE_WAYPOINT_TARGET_SOURCE = "compile_time";
#else
constexpr bool SINGLE_WAYPOINT_TARGET_OVERRIDE_ENABLED = false;
constexpr double SINGLE_WAYPOINT_TARGET_LAT = SINGLE_WAYPOINT_FALLBACK_TARGET_LAT;
constexpr double SINGLE_WAYPOINT_TARGET_LON = SINGLE_WAYPOINT_FALLBACK_TARGET_LON;
constexpr const char *SINGLE_WAYPOINT_TARGET_SOURCE = "fallback";
#endif
#if defined(SINGLE_WP_TARGET_LAT)
constexpr const char *SINGLE_WAYPOINT_TARGET_LAT_MACRO = STRINGIFY_VALUE(SINGLE_WP_TARGET_LAT);
#else
constexpr const char *SINGLE_WAYPOINT_TARGET_LAT_MACRO = "NA";
#endif
#if defined(SINGLE_WP_TARGET_LON)
constexpr const char *SINGLE_WAYPOINT_TARGET_LON_MACRO = STRINGIFY_VALUE(SINGLE_WP_TARGET_LON);
#else
constexpr const char *SINGLE_WAYPOINT_TARGET_LON_MACRO = "NA";
#endif
constexpr bool SINGLE_WAYPOINT_AUTO_MOTION_ARMED = AUTO_MOTION_ARMED != 0;
constexpr float SINGLE_WAYPOINT_CRAWL_BASE_CMD = SINGLE_WP_CRAWL_BASE_CMD;
constexpr bool SINGLE_WAYPOINT_STEERING_DRYRUN_ENABLED = SINGLE_WP_STEERING_DRYRUN != 0;
constexpr double SINGLE_WAYPOINT_ARRIVAL_RADIUS_M = 2.5;
constexpr double SINGLE_WAYPOINT_MAX_TARGET_DISTANCE_M = 30.0;
constexpr double SINGLE_WAYPOINT_MAX_COORD_SANITY_DISTANCE_M = 1000.0;
constexpr uint32_t SINGLE_WAYPOINT_GPS_STALE_MS = GPS_DRYRUN_STALE_MS;
constexpr double SINGLE_WAYPOINT_MAX_HDOP = GPS_DRYRUN_MAX_HDOP;
constexpr uint32_t SINGLE_WAYPOINT_AUTO_TIMEOUT_MS = 15000;
constexpr double SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M = COURSE_MIN_DISPLACEMENT_M;
constexpr const char *SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_SOURCE = STRINGIFY_VALUE(COURSE_MIN_DISPLACEMENT_M);
constexpr bool GPS_COURSE_LATCH_DIAG_ENABLED = GPS_COURSE_LATCH_DIAG != 0;
constexpr uint32_t GPS_COURSE_OUTPUT_LATCH_MS_VALUE = GPS_COURSE_OUTPUT_LATCH_MS;

constexpr bool     GROUND_CRAWL_ENABLED = GROUND_CRAWL_TEST_MODE != 0;
constexpr float    GROUND_CRAWL_MAX_CMD_VALUE = GROUND_CRAWL_MAX_CMD;
constexpr uint32_t GROUND_CRAWL_MAX_AUTO_MS_VALUE = GROUND_CRAWL_MAX_AUTO_MS;
constexpr double   GROUND_CRAWL_MIN_TARGET_DISTANCE_M = 5.0;
constexpr double   GROUND_CRAWL_MAX_TARGET_DISTANCE_M = 20.0;

constexpr uint8_t CHANNEL_COUNT = 8;
// Printed transmitter labels are 1-based (CH1-CH8), while ppmChannels[] uses 0-based indexes.
// The integrated station controller can physically look like CH7 at the panel,
// but the receiver PPM mapping used by firmware is CH1 steering, CH2 throttle,
// and CH5 Manual/Auto. CH7 is reserved/unused for now.
constexpr uint8_t STEERING_CHANNEL_INDEX = 0;  // Station joystick horizontal: PPM CH1
constexpr uint8_t THROTTLE_CHANNEL_INDEX = 1;  // Station joystick vertical: PPM CH2
constexpr uint8_t MODE_CHANNEL_INDEX_VALUE = MODE_CHANNEL_INDEX;  // Manual/Auto switch (default PPM CH5)
constexpr uint16_t STEERING_CENTER_US = 1504;
constexpr uint16_t THROTTLE_CENTER_US = 1500;
constexpr uint16_t RC_DEADBAND_US = 80;
constexpr uint16_t RC_MIN_VALID_US = 900;
constexpr uint16_t RC_MAX_VALID_US = 2100;
constexpr uint16_t RC_AUTO_SWITCH_ON_US = 1600;
constexpr float RC_MANUAL_AXIS_ROTATION_SCALE = 0.70710678f;
constexpr uint16_t ESC_NEUTRAL_US = 1500;
constexpr uint16_t ESC_RANGE_US = 300;
constexpr uint32_t STATION_TIMEOUT_MS = 500;
constexpr float STATION_MANUAL_MAX_OUTPUT = 0.25f;
constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;
constexpr uint32_t STATUS_PERIOD_MS = 500;
constexpr bool ENABLE_USB_DEBUG = true;
constexpr uint32_t USB_DEBUG_PERIOD_MS = MOTOR_PULSE_TEST_MODE ? 100 : 500;

enum RobotMode {
  DISARMED,
  MANUAL,
  AUTO_READY,
  AUTO_RUNNING,
  LINK_LOST,
  FAILSAFE
};

enum ControlSource {
  CONTROL_SOURCE_STOP,
  CONTROL_SOURCE_RC_MANUAL,
  CONTROL_SOURCE_STATION_MANUAL,
  CONTROL_SOURCE_AUTO
};

struct StationManualCommand {
  float steer;
  float throttle;
  bool deadman;
  uint32_t lastFrameMs;
};

#if ENABLE_GPS_TELEMETRY
TinyGPSPlus gps;
String gpsNmeaLine;
char lastRmcStatus = '\0';
int lastGgaFixQuality = -1;
#endif
Servo escLeft;
Servo escRight;

volatile uint16_t ppmChannels[CHANNEL_COUNT] = {0};
volatile uint8_t ppmIndex = 0;
volatile uint32_t lastPpmEdgeMicros = 0;
volatile uint32_t lastPpmFrameMs = 0;

RobotMode currentMode = DISARMED;
String hc12Line;
uint32_t lastStationFrameMs = 0;
long lastStationSeq = -1;
uint32_t stationHc12RxCount = 0;
uint32_t stationHc12TxCount = 0;
uint32_t stationFrameCount = 0;
uint32_t stationParseOkCount = 0;
uint32_t stationParseErrorCount = 0;
StationManualCommand stationManual = {0.0f, 0.0f, false, 0};
bool stationEstop = false;
bool autoCommandActive = false;
uint32_t lastTelemetryMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastUsbDebugMs = 0;
ControlSource currentControlSource = CONTROL_SOURCE_STOP;
float autoLeftCmd = 0.0f;
float autoRightCmd = 0.0f;
float lastLeftOutputCmd = 0.0f;
float lastRightOutputCmd = 0.0f;
float lastLogicalLeftCmd = 0.0f;
float lastLogicalRightCmd = 0.0f;
float lastRawLeftCmd = 0.0f;
float lastRawRightCmd = 0.0f;
float lastCalibratedLeftCmd = 0.0f;
float lastCalibratedRightCmd = 0.0f;
float lastOutputLeftPinCmd = 0.0f;
float lastOutputRightPinCmd = 0.0f;
bool lastMixerBypassedForMotorPulse = false;

#if FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT
bool singleWaypointAutoTimingActiveFlag = false;
uint32_t singleWaypointAutoEntryMs = 0;
uint32_t singleWaypointAutoElapsedMs = 0;
bool singleWaypointGpsReadyFlag = false;
bool singleWaypointDryrunReadyFlag = false;
bool singleWaypointMotionReadyFlag = false;
bool singleWaypointTargetReadyFlag = false;
bool singleWaypointTimeoutOkFlag = false;
bool singleWaypointDistanceAllowedFlag = false;
bool singleWaypointSafetyReadyFlag = false;
bool singleWaypointArrivedFlag = false;
bool singleWaypointAutoMotorInhibitFlag = true;
bool singleWaypointTargetComputedFlag = false;
bool singleWaypointGpsCoordSaneFlag = false;
double singleWaypointTargetDistanceM = 0.0;
double singleWaypointTargetBearingDeg = 0.0;
float singleWaypointCandidateLeftCmd = 0.0f;
float singleWaypointCandidateRightCmd = 0.0f;
bool steeringCourseReferenceValidFlag = false;
double steeringCourseReferenceLat = 0.0;
double steeringCourseReferenceLon = 0.0;
bool steeringHeadingReadyFlag = false;
double steeringEstimatedCourseDeg = 0.0;
double steeringBearingErrorDeg = 0.0;
float steeringDesiredForwardCmd = 0.0f;
float steeringDesiredTurnCmd = 0.0f;
float steeringDesiredLogicalLeftCmd = 0.0f;
float steeringDesiredLogicalRightCmd = 0.0f;
float steeringDesiredPhysicalACmd = 0.0f;
float steeringDesiredPhysicalBCmd = 0.0f;
double steeringCourseDisplacementM = 0.0;
const char *steeringBlockReason = "MODE_OFF";
bool groundCrawlNeutralOkFlag = false;
bool groundCrawlReadyFlag = false;
bool groundCrawlLatchedStopFlag = false;
uint32_t groundCrawlElapsedMs = 0;
float groundCrawlUnclampedLeftCmd = 0.0f;
float groundCrawlUnclampedRightCmd = 0.0f;
const char *groundCrawlBlockReason = "MODE_OFF";
#endif

#if MOTOR_PULSE_TEST_MODE
bool motorPulseTimingActiveFlag = false;
uint32_t motorPulseStartMs = 0;
uint32_t motorPulseElapsedMs = 0;
bool motorPulseLatchedStopFlag = false;
bool motorPulseReadyFlag = false;
const char *motorPulseBlockReason = "MODE_OFF";
#endif

#if STAGE15_GUARDED_CRAWL_TEST
String stage15UsbLine;
bool stage15PulseActiveFlag = false;
uint32_t stage15PulseStartMs = 0;
uint32_t stage15LastStopMs = 0;
uint32_t stage15PulseElapsedMs = 0;
const char *stage15PulseType = "test_stop";
float stage15PulseLeftCmd = 0.0f;
float stage15PulseRightCmd = 0.0f;
const char *stage15BlockReason = "WAITING_FOR_TRIGGER";
bool stage15FailsafeSeenFlag = false;
uint32_t stage15PulseSequence = 0;
#endif

#if STAGE16_USB_GUARDED_CRAWL || STAGE17_FIRST_PRIMITIVE_CRAWL || STAGE18_MOTOR_MAPPING_PROBE || STAGE20_PHYSICAL_AB_GUARDED_CRAWL
String stage16UsbLine;
bool stage16PulseActiveFlag = false;
uint32_t stage16PulseStartMs = 0;
uint32_t stage16PulseElapsedMs = 0;
int32_t stage16CmdSeq = -1;
float stage16LeftCmd = 0.0f;
float stage16RightCmd = 0.0f;
uint32_t stage16CmdMs = 0;
const char *stage16CmdState = "IDLE";
const char *stage16RejectReason = "NONE";
bool stage16LatchedStopFlag = false;
bool stage16ArmedFlag = false;
uint32_t stage16LastCmdMs = 0;
uint32_t stage16LastHeartbeatMs = 0;
#endif

void printImuDiagFields();

void ppmISR() {
  uint32_t now = micros();
  uint32_t width = now - lastPpmEdgeMicros;
  lastPpmEdgeMicros = now;

  if (width > 3000) {
    ppmIndex = 0;
    lastPpmFrameMs = millis();
    return;
  }

  if (ppmIndex < CHANNEL_COUNT) {
    ppmChannels[ppmIndex] = width;
    ppmIndex++;
  }
}

uint8_t checksumXor(const String &body) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < body.length(); ++i) {
    checksum ^= static_cast<uint8_t>(body[i]);
  }
  return checksum;
}

void writeFrame(const char *type, uint32_t seq, const String &payload) {
#if HC12_LINK_ENABLED
  String body = String(type) + "," + String(seq);
  if (payload.length() > 0) {
    body += "," + payload;
  }

  uint8_t checksum = checksumXor(body);
  HC12_SERIAL.print("@");
  HC12_SERIAL.print(body);
  HC12_SERIAL.print("*");
  if (checksum < 0x10) {
    HC12_SERIAL.print("0");
  }
  HC12_SERIAL.println(checksum, HEX);
  stationHc12TxCount++;
#else
  (void)type;
  (void)seq;
  (void)payload;
#endif
}

void motorStop() {
  escLeft.writeMicroseconds(ESC_NEUTRAL_US);
  escRight.writeMicroseconds(ESC_NEUTRAL_US);
  lastLogicalLeftCmd = 0.0f;
  lastLogicalRightCmd = 0.0f;
  lastRawLeftCmd = 0.0f;
  lastRawRightCmd = 0.0f;
  lastCalibratedLeftCmd = 0.0f;
  lastCalibratedRightCmd = 0.0f;
  lastLeftOutputCmd = 0.0f;
  lastRightOutputCmd = 0.0f;
  lastOutputLeftPinCmd = 0.0f;
  lastOutputRightPinCmd = 0.0f;
  lastMixerBypassedForMotorPulse = false;
}

void clearAutoCommand() {
  autoLeftCmd = 0.0f;
  autoRightCmd = 0.0f;
  autoCommandActive = false;
}

uint16_t clampPulse(int pulse) {
  if (pulse < ESC_NEUTRAL_US - ESC_RANGE_US) {
    return ESC_NEUTRAL_US - ESC_RANGE_US;
  }
  if (pulse > ESC_NEUTRAL_US + ESC_RANGE_US) {
    return ESC_NEUTRAL_US + ESC_RANGE_US;
  }
  return pulse;
}

float clampUnit(float value) {
  if (value > 1.0f) {
    return 1.0f;
  }
  if (value < -1.0f) {
    return -1.0f;
  }
  return value;
}

float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

#if FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN || \
    FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT || PATH_FOLLOWING_DRYRUN
double degreesToRadians(double degrees) {
  return degrees * 0.017453292519943295;
}

double radiansToDegrees(double radians) {
  return radians * 57.29577951308232;
}

double normalizeBearingDegrees(double degrees) {
  while (degrees < 0.0) {
    degrees += 360.0;
  }
  while (degrees >= 360.0) {
    degrees -= 360.0;
  }
  return degrees;
}

double normalizeBearingErrorDegrees(double degrees) {
  while (degrees <= -180.0) {
    degrees += 360.0;
  }
  while (degrees > 180.0) {
    degrees -= 360.0;
  }
  return degrees;
}

double dryrunDistanceMeters(double fromLat, double fromLon, double toLat, double toLon) {
  constexpr double earthRadiusMeters = 6371000.0;
  double lat1 = degreesToRadians(fromLat);
  double lat2 = degreesToRadians(toLat);
  double dLat = degreesToRadians(toLat - fromLat);
  double dLon = degreesToRadians(toLon - fromLon);
  double sinHalfLat = sin(dLat * 0.5);
  double sinHalfLon = sin(dLon * 0.5);
  double a = sinHalfLat * sinHalfLat + cos(lat1) * cos(lat2) * sinHalfLon * sinHalfLon;
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return earthRadiusMeters * c;
}

double dryrunBearingDegrees(double fromLat, double fromLon, double toLat, double toLon) {
  double lat1 = degreesToRadians(fromLat);
  double lat2 = degreesToRadians(toLat);
  double dLon = degreesToRadians(toLon - fromLon);
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  return normalizeBearingDegrees(radiansToDegrees(atan2(y, x)));
}

bool dryrunGpsReady() {
  return gpsDryrunReady();
}
#endif

#if ENABLE_GPS_TELEMETRY
bool gpsLocationValid() {
  return gps.location.isValid();
}

bool gpsAgeOk(uint32_t staleMs) {
  return gpsLocationValid() && gps.location.age() <= staleMs;
}

bool gpsAgeOk() {
  return gpsAgeOk(GPS_MOTION_STALE_MS);
}

bool gpsNmeaFixKnown() {
  return lastRmcStatus != '\0' || lastGgaFixQuality >= 0;
}

bool gpsNmeaFixOk() {
  return lastRmcStatus == 'A' || lastGgaFixQuality >= 1;
}

bool gpsSolutionValid() {
  return gpsLocationValid() && gpsAgeOk(GPS_DRYRUN_STALE_MS) &&
         (!gpsNmeaFixKnown() || gpsNmeaFixOk());
}

bool gpsSatsOk() {
  return gps.satellites.isValid() && gps.satellites.value() >= GPS_MOTION_MIN_SATS;
}

bool gpsHdopOk() {
  return gps.hdop.isValid() && gps.hdop.hdop() <= GPS_MOTION_MAX_HDOP;
}

bool gpsDryrunSatsOk() {
  return gps.satellites.isValid() && gps.satellites.value() >= GPS_DRYRUN_MIN_SATS;
}

bool gpsDryrunHdopOk() {
  return gps.hdop.isValid() && gps.hdop.hdop() <= GPS_DRYRUN_MAX_HDOP;
}

bool gpsMotionSatsOk() {
  return gps.satellites.isValid() && gps.satellites.value() >= GPS_MOTION_MIN_SATS;
}

bool gpsMotionHdopOk() {
  return gps.hdop.isValid() && gps.hdop.hdop() <= GPS_MOTION_MAX_HDOP;
}

bool gpsDryrunReady() {
  return gpsSolutionValid() && gpsAgeOk(GPS_DRYRUN_STALE_MS) &&
         gpsDryrunSatsOk() && gpsDryrunHdopOk();
}

bool gpsMotionReady() {
  return gpsSolutionValid() && gpsAgeOk(GPS_MOTION_STALE_MS) &&
         gpsMotionSatsOk() && gpsMotionHdopOk();
}

bool gpsReady() {
  return gpsMotionReady();
}

const char *gpsTierBlockReason(uint32_t staleMs, uint8_t minSats, double maxHdop) {
  if (!gpsLocationValid()) {
    return "NO_LOCATION";
  }
  if (!gpsAgeOk(staleMs)) {
    return "STALE_LOCATION";
  }
  if (gpsNmeaFixKnown() && !gpsNmeaFixOk()) {
    return "NO_FIX_STATUS";
  }
  if (!gps.satellites.isValid() || gps.satellites.value() < minSats) {
    return "NO_SATS";
  }
  if (!gps.hdop.isValid() || gps.hdop.hdop() > maxHdop) {
    return "BAD_HDOP";
  }
  return "OK";
}

const char *gpsDryrunBlockReason() {
  const char *reason = gpsTierBlockReason(GPS_DRYRUN_STALE_MS, GPS_DRYRUN_MIN_SATS, GPS_DRYRUN_MAX_HDOP);
  return gpsDryrunReady() ? "OK" : reason;
}

const char *gpsMotionBlockReason() {
  const char *reason = gpsTierBlockReason(GPS_MOTION_STALE_MS, GPS_MOTION_MIN_SATS, GPS_MOTION_MAX_HDOP);
  return gpsMotionReady() ? "OK" : reason;
}

const char *gpsBlockReason() {
  return gpsMotionBlockReason();
}

bool nmeaSentenceType(const String &sentence, char a, char b, char c) {
  return sentence.length() >= 6 && sentence.charAt(0) == '$' &&
         ((sentence.charAt(1) == 'G' && sentence.charAt(2) == 'P') ||
          (sentence.charAt(1) == 'G' && sentence.charAt(2) == 'N')) &&
         sentence.charAt(3) == a && sentence.charAt(4) == b && sentence.charAt(5) == c;
}

bool nmeaField(const String &sentence, uint8_t index, String &field) {
  uint8_t current = 0;
  int start = 0;
  while (start < sentence.length()) {
    int end = sentence.indexOf(',', start);
    int checksum = sentence.indexOf('*', start);
    if (end < 0 || (checksum >= 0 && checksum < end)) {
      end = checksum >= 0 ? checksum : sentence.length();
    }
    if (current == index) {
      field = sentence.substring(start, end);
      return true;
    }
    if (end >= sentence.length() || sentence.charAt(end) == '*') {
      return false;
    }
    start = end + 1;
    current++;
  }
  return false;
}

void updateNmeaStatus(const String &sentence) {
  String field;
  if (nmeaSentenceType(sentence, 'R', 'M', 'C') && nmeaField(sentence, 2, field) && field.length() > 0) {
    lastRmcStatus = field.charAt(0);
  } else if (nmeaSentenceType(sentence, 'G', 'G', 'A') && nmeaField(sentence, 6, field) && field.length() > 0) {
    lastGgaFixQuality = field.toInt();
  }
}

void processGpsChar(char c) {
  gps.encode(c);
  if (c == '\n') {
    if (gpsNmeaLine.length() > 0) {
      updateNmeaStatus(gpsNmeaLine);
      gpsNmeaLine = "";
    }
  } else if (c != '\r') {
    if (gpsNmeaLine.length() < 96) {
      gpsNmeaLine += c;
    } else {
      gpsNmeaLine = "";
    }
  }
}
#endif

#if FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT
bool singleWaypointGpsReady() {
  return SINGLE_WAYPOINT_AUTO_MOTION_ARMED ? gpsMotionReady() : gpsDryrunReady();
}

void resetSteeringDryrunOutputs(const char *reason) {
  steeringHeadingReadyFlag = false;
  steeringEstimatedCourseDeg = 0.0;
  steeringBearingErrorDeg = 0.0;
  steeringDesiredForwardCmd = 0.0f;
  steeringDesiredTurnCmd = 0.0f;
  steeringDesiredLogicalLeftCmd = 0.0f;
  steeringDesiredLogicalRightCmd = 0.0f;
  steeringDesiredPhysicalACmd = 0.0f;
  steeringDesiredPhysicalBCmd = 0.0f;
  steeringCourseDisplacementM = 0.0;
  steeringBlockReason = reason;
}

void updateSingleWaypointSteeringDryrun() {
  if (!SINGLE_WAYPOINT_STEERING_DRYRUN_ENABLED) {
    resetSteeringDryrunOutputs("MODE_OFF");
    return;
  }

  if (!singleWaypointGpsReadyFlag || !gps.location.isValid()) {
    steeringCourseReferenceValidFlag = false;
    resetSteeringDryrunOutputs("GPS_NOT_READY");
    return;
  }

  double currentLat = gps.location.lat();
  double currentLon = gps.location.lng();
  if (!steeringCourseReferenceValidFlag) {
    steeringCourseReferenceLat = currentLat;
    steeringCourseReferenceLon = currentLon;
    steeringCourseReferenceValidFlag = true;
    resetSteeringDryrunOutputs("NO_HEADING");
    return;
  }

  steeringCourseDisplacementM =
      dryrunDistanceMeters(steeringCourseReferenceLat, steeringCourseReferenceLon, currentLat, currentLon);

  if (steeringCourseDisplacementM < SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M) {
    double displacementM = steeringCourseDisplacementM;
    resetSteeringDryrunOutputs("NO_HEADING");
    steeringCourseDisplacementM = displacementM;
    return;
  }

  steeringEstimatedCourseDeg =
      dryrunBearingDegrees(steeringCourseReferenceLat, steeringCourseReferenceLon, currentLat, currentLon);
  steeringCourseReferenceLat = currentLat;
  steeringCourseReferenceLon = currentLon;

  if (!singleWaypointTargetComputedFlag) {
    double displacementM = steeringCourseDisplacementM;
    double estimatedCourseDeg = steeringEstimatedCourseDeg;
    resetSteeringDryrunOutputs("NO_TARGET");
    steeringCourseDisplacementM = displacementM;
    steeringEstimatedCourseDeg = estimatedCourseDeg;
    return;
  }

  steeringHeadingReadyFlag = true;
  steeringBearingErrorDeg =
      normalizeBearingErrorDegrees(singleWaypointTargetBearingDeg - steeringEstimatedCourseDeg);
  steeringDesiredForwardCmd = SINGLE_WAYPOINT_CRAWL_BASE_CMD;
  steeringDesiredTurnCmd =
      clampUnit(static_cast<float>(steeringBearingErrorDeg / 90.0)) * SINGLE_WAYPOINT_CRAWL_BASE_CMD;
  steeringDesiredLogicalLeftCmd = clampUnit(steeringDesiredForwardCmd + steeringDesiredTurnCmd);
  steeringDesiredLogicalRightCmd = clampUnit(steeringDesiredForwardCmd - steeringDesiredTurnCmd);
  steeringDesiredPhysicalACmd =
      clampUnit((steeringDesiredLogicalLeftCmd + steeringDesiredLogicalRightCmd) * 0.5f);
  steeringDesiredPhysicalBCmd =
      clampUnit((steeringDesiredLogicalRightCmd - steeringDesiredLogicalLeftCmd) * 0.5f);
  steeringBlockReason = "OK";
}

void updateSingleWaypointExperimentState(bool rcValid, bool autoSwitchOn, uint32_t now) {
  singleWaypointTargetReadyFlag = SINGLE_WAYPOINT_TARGET_AVAILABLE;
  singleWaypointDryrunReadyFlag = gpsDryrunReady();
  singleWaypointMotionReadyFlag = gpsMotionReady();
  singleWaypointGpsReadyFlag = singleWaypointGpsReady();
  singleWaypointTargetComputedFlag = singleWaypointTargetReadyFlag && singleWaypointGpsReadyFlag;
  singleWaypointGpsCoordSaneFlag = false;
  singleWaypointTargetDistanceM = 0.0;
  singleWaypointTargetBearingDeg = 0.0;
  singleWaypointArrivedFlag = false;
  singleWaypointDistanceAllowedFlag = false;

  if (singleWaypointTargetComputedFlag) {
    singleWaypointTargetDistanceM = dryrunDistanceMeters(
        gps.location.lat(), gps.location.lng(), SINGLE_WAYPOINT_TARGET_LAT, SINGLE_WAYPOINT_TARGET_LON);
    singleWaypointTargetBearingDeg = dryrunBearingDegrees(
        gps.location.lat(), gps.location.lng(), SINGLE_WAYPOINT_TARGET_LAT, SINGLE_WAYPOINT_TARGET_LON);
    singleWaypointGpsCoordSaneFlag =
        singleWaypointTargetDistanceM <= SINGLE_WAYPOINT_MAX_COORD_SANITY_DISTANCE_M;
    singleWaypointArrivedFlag = singleWaypointTargetDistanceM <= SINGLE_WAYPOINT_ARRIVAL_RADIUS_M;
    singleWaypointDistanceAllowedFlag =
        singleWaypointGpsCoordSaneFlag &&
        singleWaypointTargetDistanceM > SINGLE_WAYPOINT_ARRIVAL_RADIUS_M &&
        singleWaypointTargetDistanceM <= SINGLE_WAYPOINT_MAX_TARGET_DISTANCE_M;
  }

  bool autoCandidateActive = rcValid && autoSwitchOn;
  if (autoCandidateActive) {
    if (!singleWaypointAutoTimingActiveFlag) {
      singleWaypointAutoTimingActiveFlag = true;
      singleWaypointAutoEntryMs = now;
    }
    singleWaypointAutoElapsedMs = now - singleWaypointAutoEntryMs;
    singleWaypointTimeoutOkFlag = singleWaypointAutoElapsedMs <= SINGLE_WAYPOINT_AUTO_TIMEOUT_MS;
  } else {
    singleWaypointAutoTimingActiveFlag = false;
    singleWaypointAutoEntryMs = 0;
    singleWaypointAutoElapsedMs = 0;
    singleWaypointTimeoutOkFlag = true;
  }

  singleWaypointSafetyReadyFlag =
      rcValid && autoSwitchOn && singleWaypointGpsReadyFlag && singleWaypointTargetReadyFlag &&
      singleWaypointGpsCoordSaneFlag && singleWaypointDistanceAllowedFlag && singleWaypointTimeoutOkFlag;

  if (singleWaypointSafetyReadyFlag) {
    // Heading control is not implemented yet; this candidate is straight low-speed only.
    singleWaypointCandidateLeftCmd = SINGLE_WAYPOINT_CRAWL_BASE_CMD;
    singleWaypointCandidateRightCmd = SINGLE_WAYPOINT_CRAWL_BASE_CMD;
  } else {
    singleWaypointCandidateLeftCmd = 0.0f;
    singleWaypointCandidateRightCmd = 0.0f;
  }

  singleWaypointAutoMotorInhibitFlag =
      !SINGLE_WAYPOINT_AUTO_MOTION_ARMED || !singleWaypointSafetyReadyFlag;

  updateSingleWaypointSteeringDryrun();
}
#endif

#if MOTOR_PULSE_TEST_MODE
void updateMotorPulseState(bool rcValid, bool autoSwitchOn, bool rcManualActive,
                           uint16_t steeringUs, uint16_t throttleUs, uint32_t now) {
  bool neutralOk =
      normRcCentered(steeringUs, STEERING_CENTER_US) == 0.0f &&
      normRcCentered(throttleUs, THROTTLE_CENTER_US) == 0.0f;

  motorPulseReadyFlag = false;

  if (rcManualActive) {
    motorPulseLatchedStopFlag = false;
    motorPulseTimingActiveFlag = false;
    motorPulseStartMs = 0;
    motorPulseElapsedMs = 0;
    motorPulseBlockReason = "MODE_OFF";
    return;
  }

  if (!rcValid) {
    motorPulseTimingActiveFlag = false;
    motorPulseStartMs = 0;
    motorPulseElapsedMs = 0;
    motorPulseBlockReason = "RC_INVALID";
    return;
  }

  if (!autoSwitchOn) {
    motorPulseTimingActiveFlag = false;
    motorPulseStartMs = 0;
    motorPulseElapsedMs = 0;
    motorPulseBlockReason = "MODE_OFF";
    return;
  }

  if (motorPulseLatchedStopFlag) {
    motorPulseTimingActiveFlag = false;
    motorPulseBlockReason = "LATCHED_STOP";
    return;
  }

  if (!neutralOk) {
    motorPulseTimingActiveFlag = false;
    motorPulseStartMs = 0;
    motorPulseElapsedMs = 0;
    motorPulseBlockReason = "RC_NOT_NEUTRAL";
    return;
  }

  if (!motorPulseTimingActiveFlag) {
    motorPulseTimingActiveFlag = true;
    motorPulseStartMs = now;
  }

  motorPulseElapsedMs = now - motorPulseStartMs;
  if (motorPulseElapsedMs >= MOTOR_PULSE_MS_VALUE) {
    motorPulseLatchedStopFlag = true;
    motorPulseReadyFlag = false;
    motorPulseBlockReason = "LATCHED_STOP";
    return;
  }

  motorPulseReadyFlag = true;
  motorPulseBlockReason = "OK";
}
#endif

float clampGroundCrawl(float value) {
  if (value > GROUND_CRAWL_MAX_CMD_VALUE) return GROUND_CRAWL_MAX_CMD_VALUE;
  if (value < -GROUND_CRAWL_MAX_CMD_VALUE) return -GROUND_CRAWL_MAX_CMD_VALUE;
  return value;
}

float applyMotorCalibration(float raw, float sign, float scale, float minCmd) {
  if (!DRIVE_CALIBRATION_ENABLED) {
    return clampUnit(raw);
  }

  float calibrated = raw * sign * scale;
  float minMagnitude = absFloat(minCmd);
  if (calibrated > 0.0f && calibrated < minMagnitude) {
    calibrated = minMagnitude;
  } else if (calibrated < 0.0f && -calibrated < minMagnitude) {
    calibrated = -minMagnitude;
  }
  return clampUnit(calibrated);
}

void writeEscOutputPins(float physicalACmd, float physicalBCmd) {
  lastOutputLeftPinCmd = physicalACmd;
  lastOutputRightPinCmd = physicalBCmd;

  int leftPulse = ESC_NEUTRAL_US + static_cast<int>(physicalACmd * ESC_RANGE_US);
  int rightPulse = ESC_NEUTRAL_US + static_cast<int>(physicalBCmd * ESC_RANGE_US);
  escLeft.writeMicroseconds(clampPulse(leftPulse));
  escRight.writeMicroseconds(clampPulse(rightPulse));
}

void applyDriveCommandInternal(float logicalLeft, float logicalRight, bool motorPulseDirectWheelMode) {
  // Inputs are direct logical wheel commands. Mixers, if any, must run before this function.
  lastLogicalLeftCmd = logicalLeft;
  lastLogicalRightCmd = logicalRight;
  lastRawLeftCmd = logicalLeft;
  lastRawRightCmd = logicalRight;
  lastMixerBypassedForMotorPulse = motorPulseDirectWheelMode;

  float calibratedLeft =
      applyMotorCalibration(logicalLeft, LEFT_MOTOR_SIGN_VALUE, LEFT_MOTOR_SCALE_VALUE, LEFT_MOTOR_MIN_CMD_VALUE);
  float calibratedRight =
      applyMotorCalibration(logicalRight, RIGHT_MOTOR_SIGN_VALUE, RIGHT_MOTOR_SCALE_VALUE, RIGHT_MOTOR_MIN_CMD_VALUE);

  lastCalibratedLeftCmd = calibratedLeft;
  lastCalibratedRightCmd = calibratedRight;

  float outputLeft = calibratedLeft;
  float outputRight = calibratedRight;
  if (MOTOR_OUTPUT_SWAP_LR_ENABLED) {
    outputLeft = calibratedRight;
    outputRight = calibratedLeft;
  }

  lastLeftOutputCmd = outputLeft;
  lastRightOutputCmd = outputRight;

  // Physical output channel A is throttle and channel B is turn.
  // Probe-confirmed wheel model:
  //   physical_left_wheel  = A - B
  //   physical_right_wheel = A + B
  // Therefore the inverse from logical wheel commands is:
  //   A = (left + right) / 2
  //   B = (right - left) / 2
  float physicalACmd = clampUnit((outputLeft + outputRight) * 0.5f);
  float physicalBCmd = clampUnit((outputRight - outputLeft) * 0.5f);

  writeEscOutputPins(physicalACmd, physicalBCmd);
}

void applyDriveCommand(float logicalLeft, float logicalRight) {
  applyDriveCommandInternal(logicalLeft, logicalRight, false);
}

void applyMotorPulseDirectWheelCommand(float logicalLeft, float logicalRight) {
  applyDriveCommandInternal(logicalLeft, logicalRight, true);
}

void applyPhysicalABManualEquivalentCommand(float physicalACmd, float physicalBCmd) {
  float a = clampUnit(physicalACmd);
  float b = clampUnit(physicalBCmd);
  float logicalLeft = clampUnit(a - b);
  float logicalRight = clampUnit(a + b);
  lastLogicalLeftCmd = logicalLeft;
  lastLogicalRightCmd = logicalRight;
  lastRawLeftCmd = logicalLeft;
  lastRawRightCmd = logicalRight;
  lastCalibratedLeftCmd = logicalLeft;
  lastCalibratedRightCmd = logicalRight;
  lastLeftOutputCmd = logicalLeft;
  lastRightOutputCmd = logicalRight;
  lastMixerBypassedForMotorPulse = true;
  writeEscOutputPins(a, b);
}

void applyAutoCommand(float left, float right) {
  // Wheel-off-ground only for ESC validation.
  applyDriveCommand(left, right);
}

bool stationLinkValid() {
  return lastStationFrameMs != 0 && millis() - lastStationFrameMs <= STATION_TIMEOUT_MS;
}

bool stationManualValid() {
  return stationManual.lastFrameMs != 0 && millis() - stationManual.lastFrameMs <= STATION_TIMEOUT_MS;
}

void clearStationManualCommand() {
  stationManual.steer = 0.0f;
  stationManual.throttle = 0.0f;
  stationManual.deadman = false;
  stationManual.lastFrameMs = 0;
}

bool rcFrameRecent() {
  return millis() - lastPpmFrameMs < 200;
}

void readRcChannels(uint16_t &steering, uint16_t &throttle, uint16_t &mode) {
  noInterrupts();
  steering = ppmChannels[STEERING_CHANNEL_INDEX];
  throttle = ppmChannels[THROTTLE_CHANNEL_INDEX];
  mode = ppmChannels[MODE_CHANNEL_INDEX_VALUE];
  interrupts();
}

bool rcPulseValid(uint16_t pulseUs) {
  return pulseUs >= RC_MIN_VALID_US && pulseUs <= RC_MAX_VALID_US;
}

float normRcCentered(uint16_t pulseUs, uint16_t centerUs) {
  int delta = static_cast<int>(pulseUs) - static_cast<int>(centerUs);

  if (abs(delta) <= RC_DEADBAND_US) {
    return 0.0f;
  }

  float denom = delta > 0 ? (2000.0f - centerUs) : (centerUs - 1000.0f);
  if (denom < 1.0f) {
    return 0.0f;
  }

  return clampUnit(static_cast<float>(delta) / denom);
}

// Deprecated legacy diagonal remap. Do not use for the final MANUAL drive path.
void mapRcManualAxes(float rawSteering, float rawThrottle, float &steeringOut, float &throttleOut) {
  steeringOut = clampUnit((rawSteering + rawThrottle) * RC_MANUAL_AXIS_ROTATION_SCALE);
  throttleOut = clampUnit((rawSteering - rawThrottle) * RC_MANUAL_AXIS_ROTATION_SCALE);
}

void computeManualArcadeCommands(float rawSteering, float rawThrottle,
                                 float &forwardOut, float &turnOut,
                                 float &logicalLeftOut, float &logicalRightOut) {
  forwardOut = clampUnit(MANUAL_FORWARD_SIGN_VALUE * rawThrottle);
  turnOut = clampUnit(MANUAL_TURN_SIGN_VALUE * rawSteering);
  logicalLeftOut = clampUnit(forwardOut + turnOut);
  logicalRightOut = clampUnit(forwardOut - turnOut);
}

#if FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT
// Guarded ground crawl: the ONLY path to armed AUTO motion. Clamps the candidate
// command to +/-GROUND_CRAWL_MAX_CMD, latches to a hard stop after
// GROUND_CRAWL_MAX_AUTO_MS of continuous AUTO, and requires neutral RC sticks plus
// motion-grade GPS and a near-field target before motion is permitted.
void updateGroundCrawlState(bool rcValid, bool autoSwitchOn, bool rcManualActive,
                            uint16_t steeringUs, uint16_t throttleUs) {
  groundCrawlNeutralOkFlag =
      (normRcCentered(steeringUs, STEERING_CENTER_US) == 0.0f) &&
      (normRcCentered(throttleUs, THROTTLE_CENTER_US) == 0.0f);

  groundCrawlElapsedMs = singleWaypointAutoElapsedMs;
  groundCrawlUnclampedLeftCmd = singleWaypointCandidateLeftCmd;
  groundCrawlUnclampedRightCmd = singleWaypointCandidateRightCmd;

  // Latch clears only on a return to MANUAL.
  if (rcManualActive) {
    groundCrawlLatchedStopFlag = false;
  }

  bool crawlAutoActive = GROUND_CRAWL_ENABLED && SINGLE_WAYPOINT_AUTO_MOTION_ARMED &&
                         rcValid && autoSwitchOn && !rcManualActive;
  if (crawlAutoActive && groundCrawlElapsedMs > GROUND_CRAWL_MAX_AUTO_MS_VALUE) {
    groundCrawlLatchedStopFlag = true;
  }

  bool crawlDistanceOk =
      singleWaypointGpsCoordSaneFlag &&
      singleWaypointTargetDistanceM >= GROUND_CRAWL_MIN_TARGET_DISTANCE_M &&
      singleWaypointTargetDistanceM <= GROUND_CRAWL_MAX_TARGET_DISTANCE_M;

  if (!GROUND_CRAWL_ENABLED) {
    groundCrawlBlockReason = "MODE_OFF";
  } else if (groundCrawlLatchedStopFlag) {
    groundCrawlBlockReason = "LATCHED_STOP";
  } else if (!groundCrawlNeutralOkFlag) {
    groundCrawlBlockReason = "RC_NOT_NEUTRAL";
  } else if (!singleWaypointMotionReadyFlag) {
    groundCrawlBlockReason = "GPS_NOT_MOTION_READY";
  } else if (!singleWaypointSafetyReadyFlag) {
    groundCrawlBlockReason = "SAFETY_NOT_READY";
  } else if (!crawlDistanceOk) {
    groundCrawlBlockReason = "DISTANCE_OUT_OF_RANGE";
  } else {
    groundCrawlBlockReason = "OK";
  }

  groundCrawlReadyFlag = GROUND_CRAWL_ENABLED && !groundCrawlLatchedStopFlag &&
                         groundCrawlNeutralOkFlag && singleWaypointMotionReadyFlag &&
                         singleWaypointSafetyReadyFlag && crawlDistanceOk;
}
#endif

bool rcChannelsValid(uint16_t steering, uint16_t throttle, uint16_t mode) {
  if (!rcFrameRecent()) {
    return false;
  }

  return rcPulseValid(steering) && rcPulseValid(throttle) && rcPulseValid(mode);
}

bool rcChannelsValid() {
  uint16_t steering = 0;
  uint16_t throttle = 0;
  uint16_t mode = 0;
  readRcChannels(steering, throttle, mode);
  return rcChannelsValid(steering, throttle, mode);
}

bool rcAutoSwitchOn(uint16_t mode) {
  return mode > RC_AUTO_SWITCH_ON_US;
}

bool rcAutoSwitchOn() {
  uint16_t steering = 0;
  uint16_t throttle = 0;
  uint16_t mode = 0;
  readRcChannels(steering, throttle, mode);
  return rcAutoSwitchOn(mode);
}

#if STAGE15_GUARDED_CRAWL_TEST
bool stage15NeutralOk(uint16_t steeringUs, uint16_t throttleUs) {
  return normRcCentered(steeringUs, STEERING_CENTER_US) == 0.0f &&
         normRcCentered(throttleUs, THROTTLE_CENTER_US) == 0.0f;
}

void stage15PrintEvent(const char *event, bool rcValid, bool neutralOk, const char *reason) {
  Serial.print(F("STAGE15 stage15_guarded_crawl=true event="));
  Serial.print(event);
  Serial.print(F(" path_following_enable=false"));
  Serial.print(F(" physical_path_following_enable=false"));
  Serial.print(F(" allow_motor_output=false"));
  Serial.print(F(" path_package_used=false"));
  Serial.print(F(" virtual_controller_used=false"));
  Serial.print(F(" autonomous_mode=false"));
  Serial.print(F(" pulse_sequence="));
  Serial.print(stage15PulseSequence);
  Serial.print(F(" pulse_type="));
  Serial.print(stage15PulseType);
  Serial.print(F(" pulse_cmd_left="));
  Serial.print(stage15PulseLeftCmd, 3);
  Serial.print(F(" pulse_cmd_right="));
  Serial.print(stage15PulseRightCmd, 3);
  Serial.print(F(" pulse_ms="));
  Serial.print(STAGE15_PULSE_MS_VALUE);
  Serial.print(F(" pulse_elapsed_ms="));
  Serial.print(stage15PulseElapsedMs);
  Serial.print(F(" max_cmd="));
  Serial.print(STAGE15_MAX_CMD_VALUE, 3);
  Serial.print(F(" physical_output_active="));
  Serial.print(stage15PulseActiveFlag ? F("true") : F("false"));
  Serial.print(F(" stop_confirmed="));
  Serial.print(stage15PulseActiveFlag ? F("false") : F("true"));
  Serial.print(F(" rc_ok="));
  Serial.print(rcValid ? F("true") : F("false"));
  Serial.print(F(" neutral_ok="));
  Serial.print(neutralOk ? F("true") : F("false"));
  Serial.print(F(" failsafe_seen="));
  Serial.print(stage15FailsafeSeenFlag ? F("true") : F("false"));
  Serial.print(F(" final_left_cmd="));
  Serial.print(lastLeftOutputCmd, 3);
  Serial.print(F(" final_right_cmd="));
  Serial.print(lastRightOutputCmd, 3);
  Serial.print(F(" block_reason="));
  Serial.print(reason);
  Serial.println();
}

void stage15Stop(bool rcValid, bool neutralOk, const char *reason) {
  stage15PulseActiveFlag = false;
  stage15PulseLeftCmd = 0.0f;
  stage15PulseRightCmd = 0.0f;
  stage15PulseType = "test_stop";
  stage15BlockReason = reason;
  stage15LastStopMs = millis();
  motorStop();
  stage15PrintEvent("pulse_stop", rcValid, neutralOk, reason);
}

bool stage15StartPulse(const char *pulseType, float leftCmd, float rightCmd,
                       bool rcValid, bool neutralOk, uint32_t now) {
  if (stage15PulseActiveFlag) {
    stage15PrintEvent("pulse_rejected", rcValid, neutralOk, "PULSE_ALREADY_ACTIVE");
    return false;
  }
  if (STAGE15_REQUIRE_RC_NEUTRAL_VALUE && (!rcValid || !neutralOk)) {
    stage15PrintEvent("pulse_rejected", rcValid, neutralOk, rcValid ? "RC_NOT_NEUTRAL" : "RC_INVALID");
    return false;
  }
  if (stage15LastStopMs != 0 && now - stage15LastStopMs < STAGE15_INTER_PULSE_MS_VALUE) {
    stage15PrintEvent("pulse_rejected", rcValid, neutralOk, "INTER_PULSE_WAIT");
    return false;
  }

  float maxCmd = absFloat(STAGE15_MAX_CMD_VALUE);
  if (maxCmd > 0.10f) {
    maxCmd = 0.10f;
  }
  stage15PulseLeftCmd = clampUnit(leftCmd > maxCmd ? maxCmd : (leftCmd < -maxCmd ? -maxCmd : leftCmd));
  stage15PulseRightCmd = clampUnit(rightCmd > maxCmd ? maxCmd : (rightCmd < -maxCmd ? -maxCmd : rightCmd));
  stage15PulseType = pulseType;
  stage15PulseStartMs = now;
  stage15PulseElapsedMs = 0;
  stage15PulseActiveFlag = true;
  stage15PulseSequence++;
  stage15BlockReason = "OK";
  applyMotorPulseDirectWheelCommand(stage15PulseLeftCmd, stage15PulseRightCmd);
  stage15PrintEvent("pulse_start", rcValid, neutralOk, "OK");
  return true;
}

void stage15HandleCommand(const String &line, bool rcValid, bool neutralOk, uint32_t now) {
  if (line == "test_stop" || line == "STOP" || line == "stop") {
    stage15Stop(rcValid, neutralOk, "USB_STOP");
  } else if (line == "test_forward_pulse") {
    stage15StartPulse("test_forward_pulse", STAGE15_MAX_CMD_VALUE, STAGE15_MAX_CMD_VALUE, rcValid, neutralOk, now);
  } else if (line == "test_backward_pulse") {
    stage15StartPulse("test_backward_pulse", -STAGE15_MAX_CMD_VALUE, -STAGE15_MAX_CMD_VALUE, rcValid, neutralOk, now);
  } else if (line == "test_rotate_left_pulse") {
    stage15StartPulse("test_rotate_left_pulse", -STAGE15_MAX_CMD_VALUE, STAGE15_MAX_CMD_VALUE, rcValid, neutralOk, now);
  } else if (line == "test_rotate_right_pulse") {
    stage15StartPulse("test_rotate_right_pulse", STAGE15_MAX_CMD_VALUE, -STAGE15_MAX_CMD_VALUE, rcValid, neutralOk, now);
  } else if (line.length() > 0) {
    stage15PrintEvent("command_ignored", rcValid, neutralOk, "UNKNOWN_COMMAND");
  }
}

void stage15ReadUsbCommands(bool rcValid, bool neutralOk, uint32_t now) {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      stage15UsbLine.trim();
      stage15HandleCommand(stage15UsbLine, rcValid, neutralOk, now);
      stage15UsbLine = "";
    } else if (c != '\r') {
      stage15UsbLine += c;
      if (stage15UsbLine.length() > 80) {
        stage15UsbLine = "";
      }
    }
  }
}

void stage15UpdatePulse(bool rcValid, bool neutralOk, uint32_t now) {
  stage15FailsafeSeenFlag = stage15FailsafeSeenFlag || !rcValid;
  if (!stage15PulseActiveFlag) {
    motorStop();
    return;
  }
  stage15PulseElapsedMs = now - stage15PulseStartMs;
  if (stage15PulseElapsedMs >= STAGE15_PULSE_MS_VALUE || STAGE15_PULSE_MS_VALUE > 300) {
    stage15Stop(rcValid, neutralOk, "PULSE_COMPLETE");
    return;
  }
  applyMotorPulseDirectWheelCommand(stage15PulseLeftCmd, stage15PulseRightCmd);
}
#endif

#if STAGE16_USB_GUARDED_CRAWL || STAGE17_FIRST_PRIMITIVE_CRAWL || STAGE18_MOTOR_MAPPING_PROBE || STAGE20_PHYSICAL_AB_GUARDED_CRAWL
bool stage16NeutralOk(uint16_t steeringUs, uint16_t throttleUs) {
  return normRcCentered(steeringUs, STEERING_CENTER_US) == 0.0f &&
         normRcCentered(throttleUs, THROTTLE_CENTER_US) == 0.0f;
}

bool stage16ExtractToken(const String &line, const char *key, String &out) {
  String prefix = String(key) + "=";
  int start = line.indexOf(prefix);
  if (start < 0) {
    return false;
  }
  start += prefix.length();
  int end = line.indexOf(' ', start);
  if (end < 0) {
    end = line.length();
  }
  out = line.substring(start, end);
  return out.length() > 0;
}

bool stage16ParseFloatToken(const String &line, const char *key, float &out) {
  String token;
  if (!stage16ExtractToken(line, key, token)) {
    return false;
  }
  char *endPtr = nullptr;
  out = strtof(token.c_str(), &endPtr);
  return endPtr != token.c_str() && *endPtr == '\0';
}

bool stage16ParseIntToken(const String &line, const char *key, int32_t &out) {
  String token;
  if (!stage16ExtractToken(line, key, token)) {
    return false;
  }
  char *endPtr = nullptr;
  long parsed = strtol(token.c_str(), &endPtr, 10);
  if (endPtr == token.c_str() || *endPtr != '\0') {
    return false;
  }
  out = parsed;
  return true;
}

bool stage16RcInputRequiredForStationPulse() {
  return !STATION_MANUAL_IGNORE_RC_INPUT_ENABLED;
}

void stage16PrintStatus(const char *event, bool rcValid, bool neutralOk) {
  uint32_t now = millis();
  if (USB_PULSE_TEST_GUARDED_ENABLED || STATION_DRIVE_GUARDED_ENABLED) {
    Serial.print(F("USB_PULSE_TEST usb_pulse_test_mode=true event="));
    Serial.print(event);
    Serial.print(F(" usb_pulse_test_ready=true"));
    Serial.print(F(" usb_pulse_test_ignore_rc_input="));
    Serial.print(STATION_MANUAL_IGNORE_RC_INPUT_ENABLED ? F("true") : F("false"));
    Serial.print(F(" usb_pulse_test_cmd_state="));
    Serial.print(stage16CmdState);
    Serial.print(F(" usb_pulse_test_armed="));
    Serial.print(stage16ArmedFlag ? F("true") : F("false"));
    Serial.print(F(" usb_pulse_test_cmd_seq="));
    Serial.print(stage16CmdSeq < 0 ? 0 : stage16CmdSeq);
    Serial.print(F(" usb_pulse_test_last_cmd_age_ms="));
    if (stage16LastCmdMs == 0) {
      Serial.print(F("NA"));
    } else {
      Serial.print(now - stage16LastCmdMs);
    }
    Serial.print(F(" usb_pulse_test_ms="));
    Serial.print(stage16CmdMs);
    Serial.print(F(" requested_a_cmd="));
    Serial.print(stage16LeftCmd, 3);
    Serial.print(F(" requested_b_cmd="));
    Serial.print(stage16RightCmd, 3);
    Serial.print(F(" physical_a_cmd="));
    Serial.print(lastOutputLeftPinCmd, 3);
    Serial.print(F(" physical_b_cmd="));
    Serial.print(lastOutputRightPinCmd, 3);
    Serial.print(F(" usb_pulse_test_reject_reason="));
    Serial.print(stage16RejectReason);
    Serial.print(F(" usb_pulse_test_physical_output_active="));
    Serial.print(stage16PulseActiveFlag ? F("true") : F("false"));
    Serial.print(F(" physical_output_active="));
    Serial.print(stage16PulseActiveFlag ? F("true") : F("false"));
    Serial.print(F(" motor_write_called="));
    Serial.print(stage16PulseActiveFlag ? F("true") : F("false"));
    Serial.print(F(" final_left_cmd="));
    Serial.print(lastLeftOutputCmd, 3);
    Serial.print(F(" final_right_cmd="));
    Serial.print(lastRightOutputCmd, 3);
    Serial.print(F(" physical_a_role=throttle physical_b_role=turn"));
    Serial.print(F(" wheel_to_physical_mapping=physical_ab_manual_equivalent"));
    Serial.print(F(" rc_ok="));
    Serial.print(rcValid ? F("true") : F("false"));
    Serial.print(F(" neutral_ok="));
    Serial.print(neutralOk ? F("true") : F("false"));
    Serial.print(F(" latched_stop="));
    Serial.print(stage16LatchedStopFlag ? F("true") : F("false"));
    Serial.print(F(" physical_path_following_enable=false allow_motor_output=false path_package_used=false hc12_used=false"));
    Serial.print(F(" ready_for_full_path_following=false"));
    Serial.println();
    return;
  }
#if STAGE20_PHYSICAL_AB_GUARDED_CRAWL
  Serial.print(F("STAGE20 stage20_physical_ab_guarded_crawl=true stage18_motor_mapping_probe=false stage17_first_primitive_crawl=false stage16_guarded_crawl=false event="));
#elif STAGE18_MOTOR_MAPPING_PROBE
  Serial.print(F("STAGE18 stage18_motor_mapping_probe=true stage17_first_primitive_crawl=false stage16_guarded_crawl=false event="));
#elif STAGE17_FIRST_PRIMITIVE_CRAWL
  Serial.print(F("STAGE17 stage17_first_primitive_crawl=true stage16_guarded_crawl=false event="));
#else
  Serial.print(F("STAGE16 stage16_guarded_crawl=true stage17_first_primitive_crawl=false event="));
#endif
  Serial.print(event);
  Serial.print(F(" stage16_firmware_ready=true stage17_firmware_ready="));
  Serial.print(STAGE17_FIRST_PRIMITIVE_CRAWL_ENABLED ? F("true") : F("false"));
  Serial.print(F(" stage18_firmware_ready="));
  Serial.print(STAGE18_MOTOR_MAPPING_PROBE_ENABLED ? F("true") : F("false"));
  Serial.print(F(" stage20_firmware_ready="));
  Serial.print(STAGE20_PHYSICAL_AB_GUARDED_CRAWL_ENABLED ? F("true") : F("false"));
  Serial.print(F(" stage20_cmd_state="));
  Serial.print(stage16CmdState);
  Serial.print(F(" stage20_armed="));
  Serial.print(stage16ArmedFlag ? F("true") : F("false"));
  Serial.print(F(" station_manual_ignore_rc_input="));
  Serial.print(STATION_MANUAL_IGNORE_RC_INPUT_ENABLED ? F("true") : F("false"));
  Serial.print(F(" stage16_cmd_seq="));
  Serial.print(stage16CmdSeq < 0 ? 0 : stage16CmdSeq);
  Serial.print(F(" stage16_cmd_state="));
  Serial.print(stage16CmdState);
  Serial.print(F(" stage16_last_cmd_age_ms="));
  if (stage16LastCmdMs == 0) {
    Serial.print(F("NA"));
  } else {
    Serial.print(now - stage16LastCmdMs);
  }
  Serial.print(F(" stage16_left_cmd="));
  Serial.print(stage16LeftCmd, 3);
  Serial.print(F(" stage16_right_cmd="));
  Serial.print(stage16RightCmd, 3);
  Serial.print(F(" stage16_ms="));
  Serial.print(stage16CmdMs);
  Serial.print(F(" requested_left_cmd="));
  Serial.print(stage16LeftCmd, 3);
  Serial.print(F(" requested_right_cmd="));
  Serial.print(stage16RightCmd, 3);
  Serial.print(F(" requested_a_cmd="));
  Serial.print(stage16LeftCmd, 3);
  Serial.print(F(" requested_b_cmd="));
  Serial.print(stage16RightCmd, 3);
  Serial.print(F(" manual_equivalent_forward_cmd="));
  Serial.print(stage16LeftCmd, 3);
  Serial.print(F(" manual_equivalent_turn_cmd="));
  Serial.print(stage16RightCmd, 3);
  Serial.print(F(" stage16_reject_reason="));
  Serial.print(stage16RejectReason);
  Serial.print(F(" stage16_physical_output_active="));
  Serial.print(stage16PulseActiveFlag ? F("true") : F("false"));
  Serial.print(F(" physical_output_active="));
  Serial.print(stage16PulseActiveFlag ? F("true") : F("false"));
  Serial.print(F(" final_left_cmd="));
  Serial.print(lastLeftOutputCmd, 3);
  Serial.print(F(" final_right_cmd="));
  Serial.print(lastRightOutputCmd, 3);
  Serial.print(F(" logical_left_cmd="));
  Serial.print(lastLogicalLeftCmd, 3);
  Serial.print(F(" logical_right_cmd="));
  Serial.print(lastLogicalRightCmd, 3);
  Serial.print(F(" raw_left_cmd="));
  Serial.print(lastRawLeftCmd, 3);
  Serial.print(F(" raw_right_cmd="));
  Serial.print(lastRawRightCmd, 3);
  Serial.print(F(" calibrated_left_cmd="));
  Serial.print(lastCalibratedLeftCmd, 3);
  Serial.print(F(" calibrated_right_cmd="));
  Serial.print(lastCalibratedRightCmd, 3);
  Serial.print(F(" physical_a_cmd="));
  Serial.print(lastOutputLeftPinCmd, 3);
  Serial.print(F(" physical_b_cmd="));
  Serial.print(lastOutputRightPinCmd, 3);
  Serial.print(F(" motor_write_called="));
  Serial.print(stage16PulseActiveFlag ? F("true") : F("false"));
  Serial.print(F(" motor_backend=servo_pwm"));
  Serial.print(F(" motor_enable_state=attached"));
  Serial.print(F(" pwm_or_dynamixel_write_status=OK"));
  Serial.print(F(" physical_a_role=throttle physical_b_role=turn"));
  Serial.print(F(" wheel_to_physical_mapping="));
  Serial.print(STAGE20_PHYSICAL_AB_GUARDED_CRAWL_ENABLED ? F("physical_ab_manual_equivalent") : F("diff_to_throttle_turn"));
  Serial.print(F(" rc_ok="));
  Serial.print(rcValid ? F("true") : F("false"));
  Serial.print(F(" neutral_ok="));
  Serial.print(neutralOk ? F("true") : F("false"));
  Serial.print(F(" latched_stop="));
  Serial.print(stage16LatchedStopFlag ? F("true") : F("false"));
  Serial.print(F(" stage16_armed="));
  Serial.print(stage16ArmedFlag ? F("true") : F("false"));
  Serial.print(F(" motor_command_source="));
  Serial.print(stage16PulseActiveFlag ? (STAGE20_PHYSICAL_AB_GUARDED_CRAWL_ENABLED ? F("STAGE20_PHYSICAL_AB_GUARDED_CRAWL") : (STAGE18_MOTOR_MAPPING_PROBE_ENABLED ? F("STAGE18_MOTOR_MAPPING_PROBE") : (STAGE17_FIRST_PRIMITIVE_CRAWL_ENABLED ? F("STAGE17_FIRST_PRIMITIVE_CRAWL") : F("STAGE16_USB_GUARDED_CRAWL")))) : F("NONE"));
  Serial.print(F(" active_target_source="));
  Serial.print(stage16PulseActiveFlag ? (STAGE20_PHYSICAL_AB_GUARDED_CRAWL_ENABLED ? F("station_physical_ab_guarded") : (STAGE18_MOTOR_MAPPING_PROBE_ENABLED ? F("station_motor_mapping_probe") : (STAGE17_FIRST_PRIMITIVE_CRAWL_ENABLED ? F("station_first_primitive_guarded") : F("station_path_package_guarded")))) : F("NONE"));
  Serial.print(F(" physical_path_following_enable=false allow_motor_output=false path_package_used=false hc12_used=false"));
  Serial.print(F(" gps_block_reason="));
  Serial.print(gpsDryrunBlockReason());
  Serial.print(F(" position_source="));
  bool gpsPositionReady = gpsDryrunReady() && gps.location.isValid();
  Serial.print(gpsPositionReady ? F("gps") : F("NA"));
  Serial.print(F(" current_lat="));
  if (gpsPositionReady) {
    Serial.print(gps.location.lat(), 7);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" current_lon="));
  if (gpsPositionReady) {
    Serial.print(gps.location.lng(), 7);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_sats="));
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_hdop="));
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.hdop(), 2);
  } else {
    Serial.print(F("NA"));
  }
  printImuDiagFields();
  Serial.println();
}

void stage16Stop(bool rcValid, bool neutralOk, const char *reason) {
  stage16PulseActiveFlag = false;
  stage16ArmedFlag = false;
  stage16LatchedStopFlag = true;
  stage16LeftCmd = 0.0f;
  stage16RightCmd = 0.0f;
  stage16CmdMs = 0;
  stage16CmdState = "STOPPED";
  stage16RejectReason = reason;
  motorStop();
  stage16PrintStatus("STOP", rcValid, neutralOk);
}

void stage16Reject(bool rcValid, bool neutralOk, const char *reason) {
  stage16CmdState = "REJECTED";
  stage16RejectReason = reason;
  stage16PulseActiveFlag = false;
  stage16ArmedFlag = false;
  stage16LatchedStopFlag = true;
  motorStop();
  stage16PrintStatus("REJECT", rcValid, neutralOk);
}

void stage16Arm(bool rcValid, bool neutralOk, uint32_t now, const String &line) {
  int32_t seq = 0;
  if (stage16ParseIntToken(line, "seq", seq)) {
    stage16CmdSeq = seq;
  }
  stage16LastCmdMs = now;
  bool requireRcInput = stage16RcInputRequiredForStationPulse();
  if (requireRcInput && !rcValid) {
    stage16Reject(rcValid, neutralOk, "ARM_RC_NOT_OK");
    return;
  }
  if (requireRcInput && !neutralOk) {
    stage16Reject(rcValid, neutralOk, "ARM_NEUTRAL_NOT_OK");
    return;
  }
  if (stage16PulseActiveFlag || absFloat(lastLeftOutputCmd) > 0.0001f || absFloat(lastRightOutputCmd) > 0.0001f) {
    stage16Reject(rcValid, neutralOk, "ARM_OUTPUT_ACTIVE");
    return;
  }
  stage16LatchedStopFlag = false;
  stage16ArmedFlag = true;
  stage16CmdState = "ARMED";
  stage16RejectReason = "NONE";
  motorStop();
  stage16PrintStatus("ARM", rcValid, neutralOk);
}

void stage16HandleCommand(const String &line, bool rcValid, bool neutralOk, uint32_t now) {
  if (line.startsWith("USB_PULSE_TEST_STOP") || line.startsWith("STATION_DRIVE_STOP") || line.startsWith("STAGE16_STOP") || line.startsWith("STAGE17_STOP") || line.startsWith("STAGE18_STOP") || line.startsWith("STAGE20_STOP")) {
    int32_t seq = 0;
    if (stage16ParseIntToken(line, "seq", seq)) {
      stage16CmdSeq = seq;
    }
    stage16LastCmdMs = now;
    stage16Stop(rcValid, neutralOk, "USB_STOP");
    return;
  }
  if (line.startsWith("USB_PULSE_TEST_ARM") || line.startsWith("STATION_DRIVE_ARM") || line.startsWith("STAGE16_ARM") || line.startsWith("STAGE17_ARM") || line.startsWith("STAGE18_ARM") || line.startsWith("STAGE20_ARM")) {
    stage16Arm(rcValid, neutralOk, now, line);
    return;
  }
  if (!line.startsWith("USB_PULSE_TEST_CMD") && !line.startsWith("STATION_DRIVE_CMD") && !line.startsWith("STAGE16_CMD") && !line.startsWith("STAGE17_CMD") && !line.startsWith("STAGE18_CMD") && !line.startsWith("STAGE20_CMD")) {
    stage16Reject(rcValid, neutralOk, "UNKNOWN_COMMAND");
    return;
  }

  int32_t seq = 0;
  float left = 0.0f;
  float right = 0.0f;
  int32_t ms = 0;
  bool parsedMotion = false;
  if (line.startsWith("USB_PULSE_TEST_CMD") || line.startsWith("STATION_DRIVE_CMD") || line.startsWith("STAGE20_CMD")) {
    parsedMotion = stage16ParseIntToken(line, "seq", seq) &&
                   stage16ParseFloatToken(line, "a", left) &&
                   stage16ParseFloatToken(line, "b", right) &&
                   stage16ParseIntToken(line, "ms", ms);
  } else {
    parsedMotion = stage16ParseIntToken(line, "seq", seq) &&
                   stage16ParseFloatToken(line, "left", left) &&
                   stage16ParseFloatToken(line, "right", right) &&
                   stage16ParseIntToken(line, "ms", ms);
  }
  if (!parsedMotion) {
    stage16Reject(rcValid, neutralOk, "PARSE_ERROR");
    return;
  }
  stage16CmdSeq = seq;
  stage16LastCmdMs = now;
  if (stage16RcInputRequiredForStationPulse() && !rcValid) {
    stage16Reject(rcValid, neutralOk, "RC_INVALID");
    return;
  }
  if (stage16LatchedStopFlag) {
    stage16Reject(rcValid, neutralOk, "LATCHED_STOP");
    return;
  }
  if (!stage16ArmedFlag) {
    stage16Reject(rcValid, neutralOk, "NOT_ARMED");
    return;
  }
  if (stage16PulseActiveFlag) {
    stage16Reject(rcValid, neutralOk, "PULSE_ALREADY_ACTIVE");
    return;
  }
  if (absFloat(left) > STAGE_USB_GUARDED_MAX_CMD_VALUE || absFloat(right) > STAGE_USB_GUARDED_MAX_B_VALUE) {
    stage16Reject(rcValid, neutralOk, "COMMAND_EXCEEDS_MAX_CMD");
    return;
  }
  if (ms < 1 || static_cast<uint32_t>(ms) > STAGE_USB_GUARDED_MAX_MS_VALUE) {
    stage16Reject(rcValid, neutralOk, "COMMAND_EXCEEDS_MAX_MS");
    return;
  }

  stage16LeftCmd = left;
  stage16RightCmd = right;
  stage16CmdMs = static_cast<uint32_t>(ms);
  stage16PulseStartMs = now;
  stage16PulseElapsedMs = 0;
  stage16PulseActiveFlag = true;
  stage16ArmedFlag = false;
  stage16CmdState = "ACTIVE";
  stage16RejectReason = "NONE";
  if (STAGE20_PHYSICAL_AB_GUARDED_CRAWL_ENABLED) {
    applyPhysicalABManualEquivalentCommand(stage16LeftCmd, stage16RightCmd);
  } else {
    applyMotorPulseDirectWheelCommand(stage16LeftCmd, stage16RightCmd);
  }
  stage16PrintStatus("ACK", rcValid, neutralOk);
}

void stage16ReadUsbCommands(bool rcValid, bool neutralOk, uint32_t now) {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      stage16UsbLine.trim();
      if (stage16UsbLine.length() > 0) {
        stage16HandleCommand(stage16UsbLine, rcValid, neutralOk, now);
      }
      stage16UsbLine = "";
    } else if (c != '\r') {
      stage16UsbLine += c;
      if (stage16UsbLine.length() > 120) {
        stage16UsbLine = "";
      }
    }
  }
}

void stage16UpdatePulse(bool rcValid, bool neutralOk, uint32_t now) {
  if (stage16RcInputRequiredForStationPulse() && !rcValid) {
    stage16LatchedStopFlag = true;
    stage16ArmedFlag = false;
    if (stage16PulseActiveFlag) {
      stage16Stop(rcValid, neutralOk, "RC_INVALID");
    } else {
      motorStop();
    }
    return;
  }
  if (!stage16PulseActiveFlag) {
    motorStop();
    return;
  }
  stage16PulseElapsedMs = now - stage16PulseStartMs;
  if (stage16PulseElapsedMs >= stage16CmdMs || stage16CmdMs > STAGE_USB_GUARDED_MAX_MS_VALUE) {
    stage16Stop(rcValid, neutralOk, "PULSE_COMPLETE");
    return;
  }
  if (STAGE20_PHYSICAL_AB_GUARDED_CRAWL_ENABLED) {
    applyPhysicalABManualEquivalentCommand(stage16LeftCmd, stage16RightCmd);
  } else {
    applyMotorPulseDirectWheelCommand(stage16LeftCmd, stage16RightCmd);
  }
}

void stage16Heartbeat(bool rcValid, bool neutralOk, uint32_t now) {
  if (now - stage16LastHeartbeatMs < 500) {
    return;
  }
  stage16LastHeartbeatMs = now;
  stage16PrintStatus("HEARTBEAT", rcValid, neutralOk);
}
#endif

void applyManualOverride(uint16_t steeringUs, uint16_t throttleUs) {
  float rawSteering = normRcCentered(steeringUs, STEERING_CENTER_US);
  float rawThrottle = normRcCentered(throttleUs, THROTTLE_CENTER_US);
  float forward = 0.0f;
  float turn = 0.0f;
  float left = 0.0f;
  float right = 0.0f;
  computeManualArcadeCommands(rawSteering, rawThrottle, forward, turn, left, right);
  applyDriveCommand(left, right);
}

void applyStationManualCommand() {
  if (STATION_HW_MANUAL_DIAGNOSE_ONLY_ENABLED) {
    motorStop();
    return;
  }
  if (STATION_HW_MANUAL_A_B_MAPPING_ENABLED) {
    float physicalA = stationManual.throttle;
    float physicalB = stationManual.steer;
    if (absFloat(physicalA) > STATION_HW_MANUAL_MAX_ABS_A_VALUE) {
      physicalA = (physicalA > 0.0f ? 1.0f : -1.0f) * STATION_HW_MANUAL_MAX_ABS_A_VALUE;
    }
    if (absFloat(physicalB) > STATION_HW_MANUAL_MAX_ABS_B_VALUE) {
      physicalB = (physicalB > 0.0f ? 1.0f : -1.0f) * STATION_HW_MANUAL_MAX_ABS_B_VALUE;
    }
    applyPhysicalABManualEquivalentCommand(physicalA, physicalB);
    return;
  }
  float left = stationManual.throttle - stationManual.steer;
  float right = stationManual.throttle + stationManual.steer;
  float maxMagnitude = absFloat(left);
  if (absFloat(right) > maxMagnitude) {
    maxMagnitude = absFloat(right);
  }
  if (maxMagnitude > STATION_MANUAL_MAX_OUTPUT && maxMagnitude > 0.0f) {
    float scale = STATION_MANUAL_MAX_OUTPUT / maxMagnitude;
    left *= scale;
    right *= scale;
  }
  applyDriveCommand(left, right);
}

const char *modeName(RobotMode mode) {
  switch (mode) {
    case DISARMED: return "DISARMED";
    case MANUAL: return "MANUAL";
    case AUTO_READY: return "AUTO_READY";
    case AUTO_RUNNING: return "AUTO_RUNNING";
    case LINK_LOST: return "LINK_LOST";
    case FAILSAFE: return "FAILSAFE";
    default: return "UNKNOWN";
  }
}

const char *controlSourceName(ControlSource source) {
  switch (source) {
    case CONTROL_SOURCE_RC_MANUAL: return "RC_MANUAL";
    case CONTROL_SOURCE_STATION_MANUAL: return STATION_HW_MANUAL_ENABLED ? "STATION_HW_MANUAL" : "STATION_MANUAL";
    case CONTROL_SOURCE_AUTO: return "AUTO";
    case CONTROL_SOURCE_STOP:
    default: return "STOP";
  }
}

bool parseFloatToken(const String &token, float &valueOut) {
  char *endPtr = nullptr;
  valueOut = strtof(token.c_str(), &endPtr);
  return endPtr != token.c_str() && *endPtr == '\0';
}

bool parseBoolToken(const String &token, bool &valueOut) {
  if (token == "1" || token.equalsIgnoreCase("true")) {
    valueOut = true;
    return true;
  }
  if (token == "0" || token.equalsIgnoreCase("false")) {
    valueOut = false;
    return true;
  }
  return false;
}

bool splitCsvFields(const String &text, String *fields, size_t maxFields, size_t &fieldCount) {
  fieldCount = 0;
  int start = 0;
  while (start <= text.length()) {
    if (fieldCount >= maxFields) {
      return false;
    }
    int comma = text.indexOf(',', start);
    if (comma < 0) {
      fields[fieldCount++] = text.substring(start);
      return true;
    }
    fields[fieldCount++] = text.substring(start, comma);
    start = comma + 1;
  }
  return true;
}

void debugPrintStatus() {
  if (!ENABLE_USB_DEBUG) {
    return;
  }

  uint32_t now = millis();
  if (now - lastUsbDebugMs < USB_DEBUG_PERIOD_MS) {
    return;
  }
  lastUsbDebugMs = now;

  uint16_t steeringUs = 0;
  uint16_t throttleUs = 0;
  uint16_t modeUs = 0;
  uint32_t ppmFrameMs = 0;
  uint16_t rawChannelUs[CHANNEL_COUNT] = {0};
  noInterrupts();
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    rawChannelUs[i] = ppmChannels[i];
  }
  ppmFrameMs = lastPpmFrameMs;
  interrupts();
  steeringUs = rawChannelUs[STEERING_CHANNEL_INDEX];
  throttleUs = rawChannelUs[THROTTLE_CHANNEL_INDEX];
  modeUs = rawChannelUs[MODE_CHANNEL_INDEX_VALUE];

  bool rcValid = rcChannelsValid(steeringUs, throttleUs, modeUs);
  bool autoSwitchOn = rcAutoSwitchOn(modeUs);
  float steeringNorm = normRcCentered(steeringUs, STEERING_CENTER_US);
  float throttleNorm = normRcCentered(throttleUs, THROTTLE_CENTER_US);
  float manualForward = 0.0f;
  float manualTurn = 0.0f;
  float manualLogicalLeft = 0.0f;
  float manualLogicalRight = 0.0f;
  computeManualArcadeCommands(steeringNorm, throttleNorm,
                              manualForward, manualTurn,
                              manualLogicalLeft, manualLogicalRight);
  bool gpsLocValid = gpsLocationValid();
  bool gpsLocFresh = gpsAgeOk();
  bool gpsSatellitesOk = gpsSatsOk();
  bool gpsDilutionOk = gpsHdopOk();
  bool gpsIsReady = gpsReady();
  bool gpsSolutionOk = gpsSolutionValid();
  bool gpsDryrunOk = gpsDryrunReady();
  bool gpsMotionOk = gpsMotionReady();

  Serial.print(F("USBDBG manual_rc_recovery="));
  Serial.print(MANUAL_RC_RECOVERY_ENABLED ? F("true") : F("false"));
  Serial.print(F(" mode="));
  Serial.print(modeName(currentMode));
  Serial.print(F(" rc_ok="));
  Serial.print(rcValid ? F("true") : F("false"));
  Serial.print(F(" auto_sw="));
  Serial.print(autoSwitchOn ? F("true") : F("false"));
  Serial.print(F(" ppm_age_ms="));
  if (ppmFrameMs == 0) {
    Serial.print(F("NA"));
  } else {
    Serial.print(now - ppmFrameMs);
  }
  Serial.print(F(" steer_us="));
  Serial.print(steeringUs);
  Serial.print(F(" throttle_us="));
  Serial.print(throttleUs);
  Serial.print(F(" mode_us="));
  Serial.print(modeUs);
  Serial.print(F(" mode_channel_index="));
  Serial.print(MODE_CHANNEL_INDEX_VALUE);
  Serial.print(F(" mode_channel_label=CH"));
  Serial.print(MODE_CHANNEL_INDEX_VALUE + 1);
  Serial.print(F(" raw_mode_channel_us="));
  Serial.print(modeUs);
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    Serial.print(F(" raw_ch"));
    Serial.print(i + 1);
    Serial.print(F("_us="));
    Serial.print(rawChannelUs[i]);
  }
  Serial.print(F(" steer_norm="));
  Serial.print(steeringNorm, 3);
  Serial.print(F(" throttle_norm="));
  Serial.print(throttleNorm, 3);
  Serial.print(F(" manual_steer_cmd="));
  Serial.print(manualTurn, 3);
  Serial.print(F(" manual_throttle_cmd="));
  Serial.print(manualForward, 3);
  Serial.print(F(" manual_forward_cmd="));
  Serial.print(manualForward, 3);
  Serial.print(F(" manual_turn_cmd="));
  Serial.print(manualTurn, 3);
  Serial.print(F(" manual_forward_sign="));
  Serial.print(MANUAL_FORWARD_SIGN_VALUE, 1);
  Serial.print(F(" manual_turn_sign="));
  Serial.print(MANUAL_TURN_SIGN_VALUE, 1);
  Serial.print(F(" manual_logical_left_cmd="));
  Serial.print(manualLogicalLeft, 3);
  Serial.print(F(" manual_logical_right_cmd="));
  Serial.print(manualLogicalRight, 3);
  Serial.print(F(" old_angle_remap_active=false"));
  Serial.print(F(" station_hw_manual_mode="));
  Serial.print(STATION_HW_MANUAL_ENABLED ? F("true") : F("false"));
  Serial.print(F(" station_link_seen="));
  Serial.print(lastStationFrameMs != 0 ? F("true") : F("false"));
  Serial.print(F(" station_frame_count="));
  Serial.print(stationFrameCount);
  Serial.print(F(" station_parse_ok_count="));
  Serial.print(stationParseOkCount);
  Serial.print(F(" station_parse_error_count="));
  Serial.print(stationParseErrorCount);
  Serial.print(F(" hc12_rx_count="));
  Serial.print(stationHc12RxCount);
  Serial.print(F(" hc12_tx_count="));
  Serial.print(stationHc12TxCount);
  Serial.print(F(" hc12_last_rx_age_ms="));
  if (lastStationFrameMs == 0) {
    Serial.print(F("NA"));
  } else {
    Serial.print(now - lastStationFrameMs);
  }
  Serial.print(F(" station_age_ms="));
  if (lastStationFrameMs == 0) {
    Serial.print(F("NA"));
  } else {
    Serial.print(now - lastStationFrameMs);
  }
  Serial.print(F(" station_seq="));
  if (lastStationSeq < 0) {
    Serial.print(F("NA"));
  } else {
    Serial.print(lastStationSeq);
  }
  Serial.print(F(" station_manual_valid="));
  Serial.print(stationManualValid() ? F("true") : F("false"));
  Serial.print(F(" station_deadman="));
  Serial.print(stationManual.deadman ? F("true") : F("false"));
  Serial.print(F(" station_estop="));
  Serial.print(stationEstop ? F("true") : F("false"));
  Serial.print(F(" station_forward_cmd="));
  Serial.print(stationManual.throttle, 3);
  Serial.print(F(" station_turn_cmd="));
  Serial.print(stationManual.steer, 3);
  Serial.print(F(" station_physical_a_cmd="));
  Serial.print(STATION_HW_MANUAL_A_B_MAPPING_ENABLED ? stationManual.throttle : lastOutputLeftPinCmd, 3);
  Serial.print(F(" station_physical_b_cmd="));
  Serial.print(STATION_HW_MANUAL_A_B_MAPPING_ENABLED ? stationManual.steer : lastOutputRightPinCmd, 3);
  Serial.print(F(" physical_a_role=throttle physical_b_role=turn"));
  Serial.print(F(" wheel_to_physical_mapping=physical_ab_manual_equivalent"));
  Serial.print(F(" control_source="));
  Serial.print(controlSourceName(currentControlSource));
  Serial.print(F(" left_cmd="));
  Serial.print(lastLeftOutputCmd, 3);
  Serial.print(F(" right_cmd="));
  Serial.print(lastRightOutputCmd, 3);
  Serial.print(F(" final_left_cmd="));
  Serial.print(lastLeftOutputCmd, 3);
  Serial.print(F(" final_right_cmd="));
  Serial.print(lastRightOutputCmd, 3);
  Serial.print(F(" logical_left_cmd="));
  Serial.print(lastLogicalLeftCmd, 3);
  Serial.print(F(" logical_right_cmd="));
  Serial.print(lastLogicalRightCmd, 3);
  Serial.print(F(" raw_left_cmd="));
  Serial.print(lastRawLeftCmd, 3);
  Serial.print(F(" raw_right_cmd="));
  Serial.print(lastRawRightCmd, 3);
  Serial.print(F(" calibrated_left_cmd="));
  Serial.print(lastCalibratedLeftCmd, 3);
  Serial.print(F(" calibrated_right_cmd="));
  Serial.print(lastCalibratedRightCmd, 3);
  Serial.print(F(" output_left_cmd="));
  Serial.print(lastLeftOutputCmd, 3);
  Serial.print(F(" output_right_cmd="));
  Serial.print(lastRightOutputCmd, 3);
  Serial.print(F(" output_left_pin_cmd="));
  Serial.print(lastOutputLeftPinCmd, 3);
  Serial.print(F(" output_right_pin_cmd="));
  Serial.print(lastOutputRightPinCmd, 3);
  Serial.print(F(" physical_a_cmd="));
  Serial.print(lastOutputLeftPinCmd, 3);
  Serial.print(F(" physical_b_cmd="));
  Serial.print(lastOutputRightPinCmd, 3);
  Serial.print(F(" motor_write_called="));
  Serial.print((abs(lastLeftOutputCmd) > 0.001f || abs(lastRightOutputCmd) > 0.001f) ? F("true") : F("false"));
  Serial.print(F(" physical_output_active="));
  Serial.print((abs(lastLeftOutputCmd) > 0.001f || abs(lastRightOutputCmd) > 0.001f) ? F("true") : F("false"));
  Serial.print(F(" active_target_source="));
  Serial.print(currentControlSource == CONTROL_SOURCE_RC_MANUAL ? F("manual_rc") : F("NONE"));
  Serial.print(F(" latched_stop="));
  Serial.print(F("false"));
  Serial.print(F(" physical_path_following_enable="));
  Serial.print((PHYSICAL_PATH_FOLLOWING_ENABLE != 0) ? F("true") : F("false"));
  Serial.print(F(" allow_motor_output="));
  Serial.print((PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT != 0) ? F("true") : F("false"));
  Serial.print(F(" physical_a_role=throttle"));
  Serial.print(F(" physical_b_role=turn"));
  Serial.print(F(" wheel_to_physical_mapping="));
  Serial.print(MANUAL_RC_RECOVERY_ENABLED ? F("physical_ab_manual_equivalent") : F("diff_to_throttle_turn"));
  Serial.print(F(" motor_output_swap_lr="));
  Serial.print(MOTOR_OUTPUT_SWAP_LR_ENABLED ? F("true") : F("false"));
  Serial.print(F(" mixer_bypassed_for_motor_pulse="));
  Serial.print(lastMixerBypassedForMotorPulse ? F("true") : F("false"));
  Serial.print(F(" drive_calibration_enable="));
  Serial.print(DRIVE_CALIBRATION_ENABLED ? F("true") : F("false"));
  Serial.print(F(" left_motor_sign="));
  Serial.print(LEFT_MOTOR_SIGN_VALUE, 1);
  Serial.print(F(" right_motor_sign="));
  Serial.print(RIGHT_MOTOR_SIGN_VALUE, 1);
  Serial.print(F(" left_motor_scale="));
  Serial.print(LEFT_MOTOR_SCALE_VALUE, 3);
  Serial.print(F(" right_motor_scale="));
  Serial.print(RIGHT_MOTOR_SCALE_VALUE, 3);
  Serial.print(F(" left_motor_min_cmd="));
  Serial.print(LEFT_MOTOR_MIN_CMD_VALUE, 3);
  Serial.print(F(" right_motor_min_cmd="));
  Serial.print(RIGHT_MOTOR_MIN_CMD_VALUE, 3);
  Serial.print(F(" fixed_wiring_gps_serial2_diag="));
  Serial.print(FIXED_WIRING_GPS_SERIAL2_DIAG ? F("true") : F("false"));
  Serial.print(F(" hc12_enabled="));
  Serial.print(HC12_LINK_ENABLED ? F("true") : F("false"));
#if MOTOR_PULSE_TEST_MODE
  Serial.print(F(" motor_pulse_test_mode="));
  Serial.print(MOTOR_PULSE_ENABLED ? F("true") : F("false"));
  Serial.print(F(" motor_pulse_cmd="));
  Serial.print(MOTOR_PULSE_CMD_VALUE, 3);
  Serial.print(F(" motor_pulse_left_cmd="));
  Serial.print(MOTOR_PULSE_LEFT_CMD_VALUE, 3);
  Serial.print(F(" motor_pulse_right_cmd="));
  Serial.print(MOTOR_PULSE_RIGHT_CMD_VALUE, 3);
  Serial.print(F(" motor_pulse_ms="));
  Serial.print(MOTOR_PULSE_MS_VALUE);
  Serial.print(F(" motor_pulse_elapsed_ms="));
  Serial.print(motorPulseElapsedMs);
  Serial.print(F(" motor_pulse_latched_stop="));
  Serial.print(motorPulseLatchedStopFlag ? F("true") : F("false"));
  Serial.print(F(" motor_pulse_ready="));
  Serial.print(motorPulseReadyFlag ? F("true") : F("false"));
  Serial.print(F(" motor_pulse_block_reason="));
  Serial.print(motorPulseBlockReason);
#endif
  Serial.print(F(" gps_chars="));
  Serial.print(gps.charsProcessed());
  Serial.print(F(" gps_location_valid="));
  Serial.print(gpsLocValid ? F("true") : F("false"));
  Serial.print(F(" gps_location_fresh="));
  Serial.print(gpsLocFresh ? F("true") : F("false"));
  Serial.print(F(" gps_age_ok="));
  Serial.print(gpsLocFresh ? F("true") : F("false"));
  Serial.print(F(" gps_sats_ok="));
  Serial.print(gpsSatellitesOk ? F("true") : F("false"));
  Serial.print(F(" gps_hdop_ok="));
  Serial.print(gpsDilutionOk ? F("true") : F("false"));
  Serial.print(F(" gps_solution_valid="));
  Serial.print(gpsSolutionOk ? F("true") : F("false"));
  Serial.print(F(" gps_dryrun_ready="));
  Serial.print(gpsDryrunOk ? F("true") : F("false"));
  Serial.print(F(" gps_motion_ready="));
  Serial.print(gpsMotionOk ? F("true") : F("false"));
  Serial.print(F(" gps_dryrun_block_reason="));
  Serial.print(gpsDryrunBlockReason());
  Serial.print(F(" gps_motion_block_reason="));
  Serial.print(gpsMotionBlockReason());
  Serial.print(F(" gps_ready="));
  Serial.print(gpsIsReady ? F("true") : F("false"));
  Serial.print(F(" gps_block_reason="));
  Serial.print(gpsBlockReason());
  Serial.print(F(" gps_stale_ms="));
  Serial.print(GPS_STALE_MS);
  Serial.print(F(" gps_min_sats="));
  Serial.print(GPS_MIN_SATS);
  Serial.print(F(" gps_max_hdop="));
  Serial.print(GPS_MAX_HDOP, 1);
  Serial.print(F(" gps_dryrun_stale_ms="));
  Serial.print(GPS_DRYRUN_STALE_MS);
  Serial.print(F(" gps_dryrun_min_sats="));
  Serial.print(GPS_DRYRUN_MIN_SATS);
  Serial.print(F(" gps_dryrun_max_hdop="));
  Serial.print(GPS_DRYRUN_MAX_HDOP, 1);
  Serial.print(F(" gps_motion_stale_ms="));
  Serial.print(GPS_MOTION_STALE_MS);
  Serial.print(F(" gps_motion_min_sats="));
  Serial.print(GPS_MOTION_MIN_SATS);
  Serial.print(F(" gps_motion_max_hdop="));
  Serial.print(GPS_MOTION_MAX_HDOP, 1);
  Serial.print(F(" gps_lat="));
  if (gpsIsReady) {
    Serial.print(gps.location.lat(), 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_lon="));
  if (gpsIsReady) {
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_cached_lat="));
  if (gpsLocValid) {
    Serial.print(gps.location.lat(), 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_cached_lon="));
  if (gpsLocValid) {
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_cached_age_ms="));
  if (gpsLocValid) {
    Serial.print(gps.location.age());
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_sats="));
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_hdop="));
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.hdop(), 2);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_age_ms="));
  if (gpsLocValid) {
    Serial.print(gps.location.age());
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" last_rmc_status="));
  if (lastRmcStatus != '\0') {
    Serial.print(lastRmcStatus);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" last_gga_fix_quality="));
  if (lastGgaFixQuality >= 0) {
    Serial.print(lastGgaFixQuality);
  } else {
    Serial.print(F("NA"));
  }
#if FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN
  bool gpsReady = dryrunGpsReady();
  bool targetReady = DRYRUN_TARGET_AVAILABLE;
  bool autonomyReady = rcValid && autoSwitchOn && gpsReady && targetReady;
  Serial.print(F(" autonomy_dryrun=true"));
  Serial.print(F(" target_lat="));
  if (targetReady) {
    Serial.print(DRYRUN_TARGET_LAT, 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" target_lon="));
  if (targetReady) {
    Serial.print(DRYRUN_TARGET_LON, 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" target_distance_m="));
  if (targetReady && gpsReady) {
    Serial.print(dryrunDistanceMeters(gps.location.lat(), gps.location.lng(), DRYRUN_TARGET_LAT, DRYRUN_TARGET_LON), 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" target_bearing_deg="));
  if (targetReady && gpsReady) {
    Serial.print(dryrunBearingDegrees(gps.location.lat(), gps.location.lng(), DRYRUN_TARGET_LAT, DRYRUN_TARGET_LON), 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_ready="));
  Serial.print(gpsReady ? F("true") : F("false"));
  Serial.print(F(" target_ready="));
  Serial.print(targetReady ? F("true") : F("false"));
  Serial.print(F(" autonomy_ready="));
  Serial.print(autonomyReady ? F("true") : F("false"));
#endif
#if FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT
  Serial.print(F(" single_waypoint_experiment=true"));
  Serial.print(F(" target_override_enabled="));
  Serial.print(SINGLE_WAYPOINT_TARGET_OVERRIDE_ENABLED ? F("true") : F("false"));
  Serial.print(F(" target_source="));
  Serial.print(SINGLE_WAYPOINT_TARGET_SOURCE);
  Serial.print(F(" target_lat_macro="));
  Serial.print(SINGLE_WAYPOINT_TARGET_LAT_MACRO);
  Serial.print(F(" target_lon_macro="));
  Serial.print(SINGLE_WAYPOINT_TARGET_LON_MACRO);
  Serial.print(F(" auto_motion_armed="));
  Serial.print(SINGLE_WAYPOINT_AUTO_MOTION_ARMED ? F("true") : F("false"));
  Serial.print(F(" single_wp_crawl_base_cmd="));
  Serial.print(SINGLE_WAYPOINT_CRAWL_BASE_CMD, 3);
  Serial.print(F(" auto_motor_inhibit="));
  Serial.print(singleWaypointAutoMotorInhibitFlag ? F("true") : F("false"));
  Serial.print(F(" active_gps_ready="));
  Serial.print(singleWaypointGpsReadyFlag ? F("true") : F("false"));
  Serial.print(F(" dryrun_ready="));
  Serial.print(singleWaypointDryrunReadyFlag ? F("true") : F("false"));
  Serial.print(F(" motion_ready="));
  Serial.print(singleWaypointMotionReadyFlag ? F("true") : F("false"));
  Serial.print(F(" safety_ready_source="));
  Serial.print(SINGLE_WAYPOINT_AUTO_MOTION_ARMED ? F("motion_when_armed") : F("dryrun_when_inhibited"));
  Serial.print(F(" gps_coord_sane="));
  Serial.print(singleWaypointGpsCoordSaneFlag ? F("true") : F("false"));
  Serial.print(F(" target_ready="));
  Serial.print(singleWaypointTargetReadyFlag ? F("true") : F("false"));
  Serial.print(F(" timeout_source=auto_entry"));
  Serial.print(F(" auto_entry_ms="));
  if (singleWaypointAutoTimingActiveFlag) {
    Serial.print(singleWaypointAutoEntryMs);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" auto_elapsed_ms="));
  if (singleWaypointAutoTimingActiveFlag) {
    Serial.print(singleWaypointAutoElapsedMs);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" timeout_limit_ms="));
  Serial.print(SINGLE_WAYPOINT_AUTO_TIMEOUT_MS);
  Serial.print(F(" timeout_ok="));
  Serial.print(singleWaypointTimeoutOkFlag ? F("true") : F("false"));
  Serial.print(F(" max_target_distance_m="));
  Serial.print(SINGLE_WAYPOINT_MAX_TARGET_DISTANCE_M, 1);
  Serial.print(F(" max_coord_sanity_distance_m="));
  Serial.print(SINGLE_WAYPOINT_MAX_COORD_SANITY_DISTANCE_M, 1);
  Serial.print(F(" arrival_radius_m="));
  Serial.print(SINGLE_WAYPOINT_ARRIVAL_RADIUS_M, 1);
  Serial.print(F(" distance_allowed="));
  Serial.print(singleWaypointDistanceAllowedFlag ? F("true") : F("false"));
  Serial.print(F(" safety_ready="));
  Serial.print(singleWaypointSafetyReadyFlag ? F("true") : F("false"));
  Serial.print(F(" arrived="));
  Serial.print(singleWaypointArrivedFlag ? F("true") : F("false"));
  Serial.print(F(" target_lat="));
  if (singleWaypointTargetReadyFlag) {
    Serial.print(SINGLE_WAYPOINT_TARGET_LAT, 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" target_lon="));
  if (singleWaypointTargetReadyFlag) {
    Serial.print(SINGLE_WAYPOINT_TARGET_LON, 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" target_distance_m="));
  if (singleWaypointTargetComputedFlag) {
    Serial.print(singleWaypointTargetDistanceM, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" target_bearing_deg="));
  if (singleWaypointTargetComputedFlag) {
    Serial.print(singleWaypointTargetBearingDeg, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" single_wp_steering_dryrun="));
  Serial.print(SINGLE_WAYPOINT_STEERING_DRYRUN_ENABLED ? F("true") : F("false"));
  Serial.print(F(" current_gps_lat="));
  if (singleWaypointGpsReadyFlag && gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" current_gps_lon="));
  if (singleWaypointGpsReadyFlag && gps.location.isValid()) {
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" steering_target_lat="));
  if (singleWaypointTargetReadyFlag) {
    Serial.print(SINGLE_WAYPOINT_TARGET_LAT, 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" steering_target_lon="));
  if (singleWaypointTargetReadyFlag) {
    Serial.print(SINGLE_WAYPOINT_TARGET_LON, 6);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" heading_ready="));
  Serial.print(steeringHeadingReadyFlag ? F("true") : F("false"));
  Serial.print(F(" heading_source="));
  Serial.print(steeringHeadingReadyFlag ? F("course_over_ground") : F("NA"));
  Serial.print(F(" course_min_displacement_m="));
  Serial.print(SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M, 1);
  Serial.print(F(" course_min_displacement_source="));
  Serial.print(SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_SOURCE);
  Serial.print(F(" course_displacement_m="));
  Serial.print(steeringCourseDisplacementM, 2);
  Serial.print(F(" estimated_course_deg="));
  if (steeringHeadingReadyFlag) {
    Serial.print(steeringEstimatedCourseDeg, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" bearing_error_deg="));
  if (steeringHeadingReadyFlag) {
    Serial.print(steeringBearingErrorDeg, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" desired_forward_cmd="));
  Serial.print(steeringDesiredForwardCmd, 3);
  Serial.print(F(" desired_turn_cmd="));
  Serial.print(steeringDesiredTurnCmd, 3);
  Serial.print(F(" desired_logical_left_cmd="));
  Serial.print(steeringDesiredLogicalLeftCmd, 3);
  Serial.print(F(" desired_logical_right_cmd="));
  Serial.print(steeringDesiredLogicalRightCmd, 3);
  Serial.print(F(" desired_physical_a_cmd="));
  Serial.print(steeringDesiredPhysicalACmd, 3);
  Serial.print(F(" desired_physical_b_cmd="));
  Serial.print(steeringDesiredPhysicalBCmd, 3);
  Serial.print(F(" steering_block_reason="));
  Serial.print(steeringBlockReason);
  Serial.print(F(" candidate_left_cmd="));
  Serial.print(singleWaypointCandidateLeftCmd, 3);
  Serial.print(F(" candidate_right_cmd="));
  Serial.print(singleWaypointCandidateRightCmd, 3);
  Serial.print(F(" ground_crawl_test_mode="));
  Serial.print(GROUND_CRAWL_ENABLED ? F("true") : F("false"));
  Serial.print(F(" ground_crawl_max_cmd="));
  Serial.print(GROUND_CRAWL_MAX_CMD_VALUE, 3);
  Serial.print(F(" ground_crawl_max_auto_ms="));
  Serial.print(GROUND_CRAWL_MAX_AUTO_MS_VALUE);
  Serial.print(F(" ground_crawl_elapsed_ms="));
  Serial.print(groundCrawlElapsedMs);
  Serial.print(F(" ground_crawl_latched_stop="));
  Serial.print(groundCrawlLatchedStopFlag ? F("true") : F("false"));
  Serial.print(F(" ground_crawl_neutral_ok="));
  Serial.print(groundCrawlNeutralOkFlag ? F("true") : F("false"));
  Serial.print(F(" ground_crawl_ready="));
  Serial.print(groundCrawlReadyFlag ? F("true") : F("false"));
  Serial.print(F(" ground_crawl_block_reason="));
  Serial.print(groundCrawlBlockReason);
  Serial.print(F(" ground_crawl_min_target_distance_m="));
  Serial.print(GROUND_CRAWL_MIN_TARGET_DISTANCE_M, 1);
  Serial.print(F(" ground_crawl_max_target_distance_m="));
  Serial.print(GROUND_CRAWL_MAX_TARGET_DISTANCE_M, 1);
  Serial.print(F(" unclamped_final_left_cmd="));
  Serial.print(groundCrawlUnclampedLeftCmd, 3);
  Serial.print(F(" unclamped_final_right_cmd="));
  Serial.print(groundCrawlUnclampedRightCmd, 3);
#endif
  Serial.println();
}

void sendStatus(uint32_t refSeq) {
  String payload = String(modeName(currentMode));
  payload += rcChannelsValid() ? ",RC_OK" : ",RC_BAD";
  payload += stationLinkValid() ? ",LINK_OK," : ",LINK_LOST,";
  payload += String(refSeq);
  writeFrame("STAT", refSeq, payload);
}

void sendAck(uint32_t seq, const char *result) {
  writeFrame("ACK", seq, String(result));
}

void sendErr(uint32_t seq, const char *reason) {
  writeFrame("ERR", seq, String(reason));
}

void sendGpsTelemetry() {
#if ENABLE_GPS_TELEMETRY
  String payload;
  payload.reserve(128);
  bool ready = gpsReady();
  payload += "fix=";
  payload += ready ? "1" : "0";
  payload += ",lat=";
  if (ready) {
    payload += String(gps.location.lat(), 6);
  } else {
    payload += "NA";
  }
  payload += ",lon=";
  if (ready) {
    payload += String(gps.location.lng(), 6);
  } else {
    payload += "NA";
  }
  payload += ",block_reason=";
  payload += gpsBlockReason();
  payload += ",sats=";
  if (gps.satellites.isValid()) {
    payload += String(gps.satellites.value());
  } else {
    payload += "NA";
  }
  payload += ",hdop=";
  if (gps.hdop.isValid()) {
    payload += String(gps.hdop.hdop(), 2);
  } else {
    payload += "NA";
  }
  payload += ",age_ms=";
  if (gps.location.isValid()) {
    payload += String(gps.location.age());
  } else {
    payload += "NA";
  }
  writeFrame("GPS", millis(), payload);
#else
  writeFrame("GPS", millis(), "GPS_DISABLED");
#endif
}

bool decodeFrame(const String &line, String &typeOut, long &seqOut, String &payloadOut) {
  if (!line.startsWith("@")) {
    return false;
  }

  int star = line.lastIndexOf('*');
  if (star < 0) {
    return false;
  }

  String body = line.substring(1, star);
  String checksumText = line.substring(star + 1);
  checksumText.trim();
  if (checksumText.length() == 0) {
    return false;
  }

  char *endPtr = nullptr;
  long expected = strtol(checksumText.c_str(), &endPtr, 16);
  if (endPtr == checksumText.c_str() || *endPtr != '\0') {
    return false;
  }
  if (checksumXor(body) != static_cast<uint8_t>(expected)) {
    return false;
  }

  int firstComma = body.indexOf(',');
  int secondComma = body.indexOf(',', firstComma + 1);
  if (firstComma <= 0 || secondComma <= firstComma) {
    return false;
  }

  typeOut = body.substring(0, firstComma);
  String seqText = body.substring(firstComma + 1, secondComma);
  payloadOut = body.substring(secondComma + 1);
  seqOut = seqText.toInt();
  return seqText == String(seqOut);
}

void handleCommand(long seq, const String &payload) {
  String fields[5];
  size_t fieldCount = 0;
  if (!splitCsvFields(payload, fields, 5, fieldCount) || fieldCount == 0) {
    stationParseErrorCount++;
    sendErr(seq, "CMD_PAYLOAD");
    return;
  }

  String command = fields[0];

  if (command == "STOP") {
    stationEstop = true;
    clearStationManualCommand();
    clearAutoCommand();
    motorStop();
    currentControlSource = CONTROL_SOURCE_STOP;
    stationParseOkCount++;
    sendAck(seq, "OK");
    return;
  }

  if (command == "MANUAL") {
    if (fieldCount != 5) {
      stationParseErrorCount++;
      sendErr(seq, "MANUAL_PAYLOAD");
      return;
    }

    float steer = 0.0f;
    float throttle = 0.0f;
    bool deadman = false;
    bool estop = false;
    if (!parseFloatToken(fields[1], steer) || !parseFloatToken(fields[2], throttle) ||
        !parseBoolToken(fields[3], deadman) || !parseBoolToken(fields[4], estop)) {
      stationParseErrorCount++;
      sendErr(seq, "MANUAL_PAYLOAD");
      return;
    }

    clearAutoCommand();
    stationManual.steer = clampUnit(steer);
    stationManual.throttle = clampUnit(throttle);
    stationManual.deadman = deadman;
    stationManual.lastFrameMs = millis();
    stationEstop = estop;
    if (stationEstop) {
      motorStop();
      currentControlSource = CONTROL_SOURCE_STOP;
    }
    stationParseOkCount++;
    sendAck(seq, "OK");
    return;
  }

  if (command == "AUTO" || command == "START") {
    if (fieldCount != 3) {
      stationParseErrorCount++;
      sendErr(seq, "AUTO_PAYLOAD");
      return;
    }

    if (!rcChannelsValid() || !rcAutoSwitchOn()) {
      clearAutoCommand();
      motorStop();
      stationParseErrorCount++;
      sendErr(seq, "AUTO_REJECTED");
      return;
    }

    float left = 0.0f;
    float right = 0.0f;
    if (!parseFloatToken(fields[1], left) || !parseFloatToken(fields[2], right)) {
      stationParseErrorCount++;
      sendErr(seq, "AUTO_PAYLOAD");
      return;
    }

    autoLeftCmd = clampUnit(left);
    autoRightCmd = clampUnit(right);
    autoCommandActive = true;
    clearStationManualCommand();
    stationEstop = false;
    currentMode = AUTO_RUNNING;
    currentControlSource = CONTROL_SOURCE_AUTO;
    applyAutoCommand(autoLeftCmd, autoRightCmd);
    stationParseOkCount++;
    sendAck(seq, "OK");
    return;
  }

  stationParseErrorCount++;
  sendErr(seq, "UNKNOWN_CMD");
}

void processHC12Line(const String &line) {
  String type;
  String payload;
  long seq = 0;
  stationHc12RxCount++;
  if (!decodeFrame(line, type, seq, payload)) {
    stationParseErrorCount++;
    sendErr(0, "BAD_FRAME");
    return;
  }

  stationFrameCount++;
  lastStationFrameMs = millis();
  lastStationSeq = seq;

  if (type == "HB") {
    stationParseOkCount++;
    sendAck(seq, "OK");
  } else if (type == "CMD") {
    handleCommand(seq, payload);
  } else {
    stationParseErrorCount++;
    sendErr(seq, "UNSUPPORTED");
  }
}

// ===================== IMU diagnostic integration (Task D) =====================
// IMU diagnostics. Compiled only when IMU_ENABLE=1. Never drives motors.
// Gyro-integrated yaw is RELATIVE and DRIFTS (diagnostic, not a heading).
// Supports:
//   - BMI160 auto-detected at 0x68/0x69 via CHIP_ID register 0x00 == 0xD1
//   - legacy MPU/ICM path at IMU_I2C_ADDR using WHO_AM_I register 0x75
#if IMU_ENABLE
constexpr uint8_t  IMU_LEGACY_CTL_ADDR = IMU_I2C_ADDR;
constexpr uint8_t  IMU_REG_WHO_AM_I = 0x75;
constexpr uint8_t  IMU_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t  IMU_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t  IMU_MPU_BURST_LEN = 14;
constexpr double   IMU_GYRO_LSB_PER_DPS_VALUE = IMU_GYRO_LSB_PER_DPS;
constexpr double   IMU_ACCEL_LSB_PER_G_VALUE = IMU_ACCEL_LSB_PER_G;
constexpr uint8_t  IMU_YAW_AXIS_VALUE = IMU_YAW_AXIS;
constexpr float    IMU_YAW_SIGN_VALUE = IMU_YAW_SIGN;
constexpr uint32_t IMU_SAMPLE_PERIOD_MS = 20;
constexpr uint32_t IMU_CAL_MS = static_cast<uint32_t>(IMU_CALIBRATION_SECONDS) * 1000UL;
constexpr uint8_t  BMI160_ADDR_A = 0x68;
constexpr uint8_t  BMI160_ADDR_B = 0x69;
constexpr uint8_t  BMI160_CHIP_ID_EXPECTED = 0xD1;
constexpr uint8_t  BMI160_REG_CHIP_ID = 0x00;
constexpr uint8_t  BMI160_REG_ERR_REG = 0x02;
constexpr uint8_t  BMI160_REG_PMU_STATUS = 0x03;
constexpr uint8_t  BMI160_REG_GYRO_X_LSB = 0x0C;
constexpr uint8_t  BMI160_REG_ACCEL_X_LSB = 0x12;
constexpr uint8_t  BMI160_REG_ACC_RANGE = 0x41;
constexpr uint8_t  BMI160_REG_GYR_RANGE = 0x43;
constexpr uint8_t  BMI160_REG_CMD = 0x7E;
constexpr uint8_t  BMI160_CMD_ACC_NORMAL = 0x11;
constexpr uint8_t  BMI160_CMD_GYR_NORMAL = 0x15;

bool imuPresentFlag = false;
bool imuWhoamiOkFlag = false;
uint8_t imuWhoamiValue = 0;
bool imuChipIdOkFlag = false;
uint8_t imuChipIdValue = 0;
uint8_t imuActiveAddr = 0;
uint8_t imuErrRegValue = 0;
bool imuErrRegOkFlag = false;
uint8_t imuPmuStatusValue = 0;
bool imuPmuStatusOkFlag = false;
bool imuPmuNormalFlag = false;
bool imuAccelRangeDefaultFlag = false;
bool imuGyroRangeDefaultFlag = false;
bool imuRawAllZeroFlag = false;
bool imuDataPlausibleFlag = false;
int16_t imuAccelRawX = 0, imuAccelRawY = 0, imuAccelRawZ = 0;
int16_t imuGyroRawX = 0, imuGyroRawY = 0, imuGyroRawZ = 0;
int16_t imuTempRaw = 0;
double imuAccelMagG = 0.0, imuGyroMagDps = 0.0;
double imuAccelLsbPerGActive = IMU_ACCEL_LSB_PER_G_VALUE;
double imuGyroLsbPerDpsActive = IMU_GYRO_LSB_PER_DPS_VALUE;
bool imuStationaryFlag = false;
bool imuCalibratingFlag = false;
bool imuCalibratedFlag = false;
uint32_t imuCalStartMs = 0;
double imuGyroSumX = 0.0, imuGyroSumY = 0.0, imuGyroSumZ = 0.0;
uint32_t imuCalCount = 0;
double imuGyroBiasX = 0.0, imuGyroBiasY = 0.0, imuGyroBiasZ = 0.0;
double imuRelativeYawDeg = 0.0;
bool imuHaveLastSample = false;
uint32_t imuLastSampleMicros = 0;
uint32_t imuLastSampleMs = 0;
const char *imuHeadingBlockReason = "INIT";

enum ImuControllerType {
  IMU_TYPE_NONE,
  IMU_TYPE_MPU_ICM,
  IMU_TYPE_BMI160
};
ImuControllerType imuType = IMU_TYPE_NONE;

const char *imuTypeName() {
  switch (imuType) {
    case IMU_TYPE_BMI160: return "BMI160";
    case IMU_TYPE_MPU_ICM: return "MPU_ICM";
    default: return "NONE";
  }
}

double imuWrap180(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

bool imuCtlReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t n = Wire.requestFrom(addr, len);
  if (n != len) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

bool imuCtlWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

int16_t imuCtlBe16(uint8_t hi, uint8_t lo) {
  return static_cast<int16_t>(static_cast<uint16_t>(hi) << 8 | lo);
}

int16_t imuCtlLe16(uint8_t lo, uint8_t hi) {
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

int16_t imuGyroAxisRaw(uint8_t axis) {
  return axis == 0 ? imuGyroRawX : (axis == 1 ? imuGyroRawY : imuGyroRawZ);
}
double imuGyroAxisBias(uint8_t axis) {
  return axis == 0 ? imuGyroBiasX : (axis == 1 ? imuGyroBiasY : imuGyroBiasZ);
}

double bmi160AccelLsbPerG(uint8_t accRangeReg, bool &defaulted) {
  defaulted = false;
  switch (accRangeReg & 0x0F) {
    case 0x03: return 16384.0;  // +/-2g
    case 0x05: return 8192.0;   // +/-4g
    case 0x08: return 4096.0;   // +/-8g
    case 0x0C: return 2048.0;   // +/-16g
    default:
      defaulted = true;
      return 16384.0;           // diagnostic fallback
  }
}

double bmi160GyroLsbPerDps(uint8_t gyroRangeReg, bool &defaulted) {
  defaulted = false;
  switch (gyroRangeReg & 0x07) {
    case 0x00: return 16.4;    // +/-2000 dps
    case 0x01: return 32.8;    // +/-1000 dps
    case 0x02: return 65.6;    // +/-500 dps
    case 0x03: return 131.2;   // +/-250 dps
    case 0x04: return 262.4;   // +/-125 dps
    default:
      defaulted = true;
      return 16.4;             // diagnostic fallback
  }
}

const char *bmi160PmuModeName(uint8_t mode) {
  switch (mode & 0x03) {
    case 0x00: return "SUSPEND";
    case 0x01: return "NORMAL";
    case 0x02: return "LOW_POWER_OR_FAST_START";
    default: return "RESERVED";
  }
}

uint8_t bmi160AccelPmuBits(uint8_t pmuStatus) {
  return (pmuStatus >> 4) & 0x03;
}

uint8_t bmi160GyroPmuBits(uint8_t pmuStatus) {
  return (pmuStatus >> 2) & 0x03;
}

void imuStartCalibration(const char *reason) {
  imuCalibratingFlag = true;
  imuCalibratedFlag = false;
  imuCalStartMs = millis();
  imuGyroSumX = imuGyroSumY = imuGyroSumZ = 0.0;
  imuCalCount = 0;
  imuRelativeYawDeg = 0.0;
  imuHaveLastSample = false;
  imuHeadingBlockReason = reason;
}

bool imuTryInitBmi160(uint8_t addr) {
  uint8_t chipId = 0;
  if (!imuCtlReadRegs(addr, BMI160_REG_CHIP_ID, &chipId, 1)) {
    return false;
  }
  if (chipId != BMI160_CHIP_ID_EXPECTED) {
    return false;
  }

  imuType = IMU_TYPE_BMI160;
  imuActiveAddr = addr;
  imuChipIdValue = chipId;
  imuChipIdOkFlag = true;
  imuWhoamiOkFlag = false;
  imuWhoamiValue = 0;

  bool accelCmdOk = imuCtlWriteReg(addr, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
  delay(10);
  bool gyroCmdOk = imuCtlWriteReg(addr, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL);
  delay(100);
  imuPmuStatusOkFlag = imuCtlReadRegs(addr, BMI160_REG_PMU_STATUS, &imuPmuStatusValue, 1);
  imuPmuNormalFlag = imuPmuStatusOkFlag &&
                     bmi160AccelPmuBits(imuPmuStatusValue) == 0x01 &&
                     bmi160GyroPmuBits(imuPmuStatusValue) == 0x01;
  imuErrRegOkFlag = imuCtlReadRegs(addr, BMI160_REG_ERR_REG, &imuErrRegValue, 1);

  uint8_t accRange = 0x03;
  if (imuCtlReadRegs(addr, BMI160_REG_ACC_RANGE, &accRange, 1)) {
    imuAccelLsbPerGActive = bmi160AccelLsbPerG(accRange, imuAccelRangeDefaultFlag);
  } else {
    imuAccelRangeDefaultFlag = true;
    imuAccelLsbPerGActive = 16384.0;
  }
  uint8_t gyrRange = 0x00;
  if (imuCtlReadRegs(addr, BMI160_REG_GYR_RANGE, &gyrRange, 1)) {
    imuGyroLsbPerDpsActive = bmi160GyroLsbPerDps(gyrRange, imuGyroRangeDefaultFlag);
  } else {
    imuGyroRangeDefaultFlag = true;
    imuGyroLsbPerDpsActive = 16.4;
  }

  if (!accelCmdOk || !gyroCmdOk || !imuPmuNormalFlag) {
    imuPresentFlag = false;
    imuHeadingBlockReason = "BMI160_PMU_NOT_NORMAL";
    return true;
  }

  imuPresentFlag = true;
  imuStartCalibration("CALIBRATING");
  return true;
}

bool imuTryInitMpuIcm(uint8_t addr) {
  imuWhoamiOkFlag = imuCtlReadRegs(addr, IMU_REG_WHO_AM_I, &imuWhoamiValue, 1);
  if (!imuWhoamiOkFlag) {
    return false;
  }
  imuType = IMU_TYPE_MPU_ICM;
  imuActiveAddr = addr;
  imuPresentFlag = true;
  imuChipIdOkFlag = false;
  imuChipIdValue = 0;
  imuPmuNormalFlag = true;  // not a BMI160 PMU concept
  imuAccelLsbPerGActive = IMU_ACCEL_LSB_PER_G_VALUE;
  imuGyroLsbPerDpsActive = IMU_GYRO_LSB_PER_DPS_VALUE;
  imuCtlWriteReg(addr, IMU_REG_PWR_MGMT_1, 0x00);  // wake (clear SLEEP)
  delay(10);
  imuStartCalibration("CALIBRATING");
  return true;
}

void imuControllerInit() {
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif

  if (!imuTryInitBmi160(BMI160_ADDR_A) && !imuTryInitBmi160(BMI160_ADDR_B) &&
      !imuTryInitMpuIcm(IMU_LEGACY_CTL_ADDR)) {
    imuType = IMU_TYPE_NONE;
    imuActiveAddr = IMU_LEGACY_CTL_ADDR;
    imuPresentFlag = false;
    imuHeadingBlockReason = "IMU_NOT_PRESENT";
  }

  Serial.print(F("imu_init imu_type="));
  Serial.print(imuTypeName());
  Serial.print(F(" imu_present="));
  Serial.print(imuPresentFlag ? F("true") : F("false"));
  Serial.print(F(" imu_i2c_addr=0x"));
  if (imuActiveAddr < 0x10) Serial.print('0');
  Serial.print(imuActiveAddr, HEX);
  Serial.print(F(" imu_chip_id="));
  if (imuChipIdOkFlag) {
    Serial.print(F("0x"));
    if (imuChipIdValue < 0x10) Serial.print('0');
    Serial.print(imuChipIdValue, HEX);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_whoami="));
  if (imuType == IMU_TYPE_BMI160) {
    Serial.print(F("NA_BMI160"));
  } else if (imuWhoamiOkFlag) {
    Serial.print(F("0x"));
    if (imuWhoamiValue < 0x10) Serial.print('0');
    Serial.print(imuWhoamiValue, HEX);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_pmu_status=0x"));
  if (imuPmuStatusValue < 0x10) Serial.print('0');
  Serial.print(imuPmuStatusValue, HEX);
  Serial.print(F(" imu_pmu_normal="));
  if (imuType == IMU_TYPE_BMI160) {
    Serial.print(imuPmuNormalFlag ? F("true") : F("false"));
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_heading_block_reason="));
  Serial.print(imuHeadingBlockReason);
  Serial.println();
}

bool imuControllerReadRaw() {
  if (imuType == IMU_TYPE_BMI160) {
    uint8_t gyroBuf[6] = {0};
    uint8_t accelBuf[6] = {0};
    if (!imuCtlReadRegs(imuActiveAddr, BMI160_REG_GYRO_X_LSB, gyroBuf, sizeof(gyroBuf))) return false;
    if (!imuCtlReadRegs(imuActiveAddr, BMI160_REG_ACCEL_X_LSB, accelBuf, sizeof(accelBuf))) return false;
    imuGyroRawX = imuCtlLe16(gyroBuf[0], gyroBuf[1]);
    imuGyroRawY = imuCtlLe16(gyroBuf[2], gyroBuf[3]);
    imuGyroRawZ = imuCtlLe16(gyroBuf[4], gyroBuf[5]);
    imuAccelRawX = imuCtlLe16(accelBuf[0], accelBuf[1]);
    imuAccelRawY = imuCtlLe16(accelBuf[2], accelBuf[3]);
    imuAccelRawZ = imuCtlLe16(accelBuf[4], accelBuf[5]);
    imuTempRaw = 0;
    return true;
  }

  uint8_t buf[IMU_MPU_BURST_LEN];
  if (!imuCtlReadRegs(imuActiveAddr, IMU_REG_ACCEL_XOUT_H, buf, IMU_MPU_BURST_LEN)) return false;
  imuAccelRawX = imuCtlBe16(buf[0], buf[1]);
  imuAccelRawY = imuCtlBe16(buf[2], buf[3]);
  imuAccelRawZ = imuCtlBe16(buf[4], buf[5]);
  imuTempRaw = imuCtlBe16(buf[6], buf[7]);
  imuGyroRawX = imuCtlBe16(buf[8], buf[9]);
  imuGyroRawY = imuCtlBe16(buf[10], buf[11]);
  imuGyroRawZ = imuCtlBe16(buf[12], buf[13]);
  return true;
}

// Read-only sampling: gyro-bias calibration, motion diagnostic, and relative-yaw
// integration. Runs whenever IMU_ENABLE; it never affects motor output.
void imuControllerUpdate(uint32_t now) {
  if (!imuPresentFlag) {
    imuHeadingBlockReason = "IMU_NOT_PRESENT";
    return;
  }
  if (now - imuLastSampleMs < IMU_SAMPLE_PERIOD_MS) return;
  imuLastSampleMs = now;
  if (!imuControllerReadRaw()) {
    imuHeadingBlockReason = "IMU_READ_FAILED";
    imuDataPlausibleFlag = false;
    return;
  }
  uint32_t nowMicros = micros();

  imuRawAllZeroFlag =
      imuAccelRawX == 0 && imuAccelRawY == 0 && imuAccelRawZ == 0 &&
      imuGyroRawX == 0 && imuGyroRawY == 0 && imuGyroRawZ == 0;

  double ax = imuAccelRawX / imuAccelLsbPerGActive;
  double ay = imuAccelRawY / imuAccelLsbPerGActive;
  double az = imuAccelRawZ / imuAccelLsbPerGActive;
  imuAccelMagG = sqrt(ax * ax + ay * ay + az * az);
  double gx = (imuGyroRawX - imuGyroBiasX) / imuGyroLsbPerDpsActive;
  double gy = (imuGyroRawY - imuGyroBiasY) / imuGyroLsbPerDpsActive;
  double gz = (imuGyroRawZ - imuGyroBiasZ) / imuGyroLsbPerDpsActive;
  imuGyroMagDps = sqrt(gx * gx + gy * gy + gz * gz);
  imuDataPlausibleFlag =
      !imuRawAllZeroFlag &&
      imuAccelMagG >= 0.5 && imuAccelMagG <= 1.5 &&
      imuGyroMagDps < 500.0 &&
      (imuType != IMU_TYPE_BMI160 || (imuPmuNormalFlag && imuChipIdOkFlag));
  if (!imuDataPlausibleFlag) {
    imuHeadingBlockReason = imuRawAllZeroFlag ? "IMU_RAW_ALL_ZERO" : "IMU_RAW_NOT_PLAUSIBLE";
    return;
  }

  imuStationaryFlag = (imuGyroMagDps < 2.0) && (fabs(imuAccelMagG - 1.0) < 0.12);

  if (imuCalibratingFlag) {
    imuGyroSumX += imuGyroRawX;
    imuGyroSumY += imuGyroRawY;
    imuGyroSumZ += imuGyroRawZ;
    imuCalCount++;
    if (now - imuCalStartMs >= IMU_CAL_MS && imuCalCount > 0) {
      imuGyroBiasX = imuGyroSumX / imuCalCount;
      imuGyroBiasY = imuGyroSumY / imuCalCount;
      imuGyroBiasZ = imuGyroSumZ / imuCalCount;
      imuCalibratingFlag = false;
      imuCalibratedFlag = true;
    }
    imuHeadingBlockReason = "CALIBRATING";
  }

  if (imuCalibratedFlag) {
    if (imuHaveLastSample) {
      double dtS = (nowMicros - imuLastSampleMicros) / 1000000.0;
      double rate = (imuGyroAxisRaw(IMU_YAW_AXIS_VALUE) - imuGyroAxisBias(IMU_YAW_AXIS_VALUE)) /
                    imuGyroLsbPerDpsActive;
      imuRelativeYawDeg = imuWrap180(imuRelativeYawDeg + IMU_YAW_SIGN_VALUE * rate * dtS);
    }
    imuHaveLastSample = true;
    imuHeadingBlockReason = "RELATIVE_YAW_DIAG_ONLY";
  }
  imuLastSampleMicros = nowMicros;
}

// imu_heading_ready means the relative-yaw diagnostic is producing values. It is
// NOT an absolute heading and is NOT approved to drive motors.
bool imuHeadingReady() { return imuPresentFlag && imuDataPlausibleFlag && imuCalibratedFlag; }
#endif  // IMU_ENABLE

void printImuDiagFields() {
#if IMU_ENABLE
  Serial.print(F(" imu_enabled=true imu_type="));
  Serial.print(imuTypeName());
  Serial.print(F(" imu_present="));
  Serial.print(imuPresentFlag ? F("true") : F("false"));
  Serial.print(F(" imu_i2c_addr=0x"));
  if (imuActiveAddr < 0x10) Serial.print('0');
  Serial.print(imuActiveAddr, HEX);
  Serial.print(F(" imu_chip_id="));
  if (imuChipIdOkFlag) {
    Serial.print(F("0x"));
    if (imuChipIdValue < 0x10) Serial.print('0');
    Serial.print(imuChipIdValue, HEX);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_pmu_normal="));
  if (imuType == IMU_TYPE_BMI160) {
    Serial.print(imuPmuNormalFlag ? F("true") : F("false"));
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_data_plausible="));
  Serial.print(imuDataPlausibleFlag ? F("true") : F("false"));
  Serial.print(F(" imu_calibrated="));
  Serial.print(imuCalibratedFlag ? F("true") : F("false"));
  Serial.print(F(" imu_stationary="));
  Serial.print(imuStationaryFlag ? F("true") : F("false"));
  Serial.print(F(" imu_gyro_mag_dps="));
  Serial.print(imuGyroMagDps, 2);
  Serial.print(F(" imu_relative_yaw_deg="));
  Serial.print(imuRelativeYawDeg, 2);
  Serial.print(F(" imu_yaw_axis="));
  Serial.print(IMU_YAW_AXIS_VALUE);
  Serial.print(F(" imu_yaw_sign="));
  Serial.print(static_cast<int>(IMU_YAW_SIGN_VALUE));
  Serial.print(F(" imu_heading_ready="));
  Serial.print(imuHeadingReady() ? F("true") : F("false"));
  Serial.print(F(" imu_heading_block_reason="));
  Serial.print(imuHeadingBlockReason);
#else
  Serial.print(F(" imu_enabled=false imu_type=NA imu_present=false imu_i2c_addr=NA imu_chip_id=NA imu_pmu_normal=NA imu_data_plausible=false imu_calibrated=false imu_stationary=NA imu_gyro_mag_dps=NA imu_relative_yaw_deg=NA imu_yaw_axis=NA imu_yaw_sign=NA imu_heading_ready=false imu_heading_block_reason=IMU_DISABLED"));
#endif
}

// ============== Path-following dry-run + guarded execution (Tasks E/F/G) ==============
// Compiled only when PATH_FOLLOWING_DRYRUN=1. Self-contained and motor-safe by
// default: physical output is impossible unless PATH_FOLLOWING_PHYSICAL_COMPILE_GATE
// (all four motion flags = 1) AND every runtime gate passes. See firmware/README.md.
#if PATH_FOLLOWING_DRYRUN

#if PATH_FOLLOWING_HC12_ENABLED
#if PATH_FOLLOWING_HC12_SERIAL_PORT == 1
#define PF_HC12_SERIAL Serial1
constexpr const char *PF_HC12_PORT_NAME = "Serial1";
#elif PATH_FOLLOWING_HC12_SERIAL_PORT == 3
#define PF_HC12_SERIAL Serial3
constexpr const char *PF_HC12_PORT_NAME = "Serial3";
#else
#error "PATH_FOLLOWING_HC12_SERIAL_PORT must be 1 (Serial1) or 3 (Serial3); Serial2 is reserved for GPS."
#endif
#else
constexpr const char *PF_HC12_PORT_NAME = "disabled";
#endif

constexpr float    PATH_FOLLOWING_MAX_FORWARD_CMD_VALUE = PATH_FOLLOWING_MAX_FORWARD_CMD;
constexpr float    PATH_FOLLOWING_MAX_TURN_CMD_VALUE = PATH_FOLLOWING_MAX_TURN_CMD;
constexpr uint32_t PATH_FOLLOWING_MAX_AUTO_MS_VALUE = PATH_FOLLOWING_MAX_AUTO_MS;
constexpr double   PATH_FOLLOWING_ARRIVAL_RADIUS_M_VALUE = PATH_FOLLOWING_ARRIVAL_RADIUS_M;
constexpr double   PATH_FOLLOWING_MIN_TARGET_DISTANCE_M_VALUE = PATH_FOLLOWING_MIN_TARGET_DISTANCE_M;
constexpr double   PATH_FOLLOWING_MAX_TARGET_DISTANCE_M_VALUE = PATH_FOLLOWING_MAX_TARGET_DISTANCE_M;
constexpr bool     PATH_FOLLOWING_ALLOW_MOCK_POSITION_ENABLED = PATH_FOLLOWING_ALLOW_MOCK_POSITION != 0;
constexpr double   PATH_FOLLOWING_MOCK_CURRENT_LAT_VALUE = PATH_FOLLOWING_MOCK_CURRENT_LAT;
constexpr double   PATH_FOLLOWING_MOCK_CURRENT_LON_VALUE = PATH_FOLLOWING_MOCK_CURRENT_LON;
constexpr bool     PATH_FOLLOWING_MODE_CHANNEL_STABLE_ACK = PATH_FOLLOWING_MODE_CHANNEL_STABLE != 0;
constexpr uint32_t PATH_FOLLOWING_HC12_CMD_STALE_MS_VALUE = PATH_FOLLOWING_HC12_CMD_STALE_MS;

// The single hard compile gate. False unless ALL FOUR motion flags are 1.
constexpr bool PATH_FOLLOWING_PHYSICAL_COMPILE_GATE =
    (PHYSICAL_PATH_FOLLOWING_ENABLE != 0) && (PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT != 0) &&
    (GROUND_CRAWL_TEST_MODE != 0) && (AUTO_MOTION_ARMED != 0);

struct PfWaypoint {
  double lat;
  double lon;
};
PfWaypoint pfWaypoints[] = {
    {PATH_FOLLOWING_WP1_LAT, PATH_FOLLOWING_WP1_LON},
    {PATH_FOLLOWING_WP2_LAT, PATH_FOLLOWING_WP2_LON},
    {PATH_FOLLOWING_WP3_LAT, PATH_FOLLOWING_WP3_LON},
};
constexpr size_t PF_WAYPOINT_CAPACITY = sizeof(pfWaypoints) / sizeof(pfWaypoints[0]);
size_t pfWaypointCount =
    (PATH_FOLLOWING_WP_COUNT < 1)
        ? 1
        : ((static_cast<size_t>(PATH_FOLLOWING_WP_COUNT) > PF_WAYPOINT_CAPACITY)
               ? PF_WAYPOINT_CAPACITY
               : static_cast<size_t>(PATH_FOLLOWING_WP_COUNT));
size_t pfActiveIndex = 0;

bool pfPositionValidFlag = false;
double pfCurrentLat = 0.0;
double pfCurrentLon = 0.0;
const char *pfPositionSource = "none";
const char *pfActiveTargetSource = "compile_time";
double pfTargetDistanceM = 0.0;
double pfTargetBearingDeg = 0.0;
bool pfArrivedFlag = false;
bool pfCourseRefValid = false;
double pfCourseRefLat = 0.0;
double pfCourseRefLon = 0.0;
double pfCourseDisplacementM = 0.0;
bool pfHeadingReadyFlag = false;
const char *pfHeadingSource = "NONE";
double pfEstimatedCourseDeg = 0.0;
double pfBearingErrorDeg = 0.0;
float pfDesiredForwardCmd = 0.0f;
float pfDesiredTurnCmd = 0.0f;
float pfDesiredLogicalLeftCmd = 0.0f;
float pfDesiredLogicalRightCmd = 0.0f;
float pfDesiredPhysicalACmd = 0.0f;
float pfDesiredPhysicalBCmd = 0.0f;
const char *pfBlockReason = "MODE_OFF";
bool pfGpsCourseReadyCandidateFlag = false;
const char *pfGpsCourseBlockReason = "NO_ANCHOR";
const char *pfGpsCourseAnchorResetReason = "BOOT";
uint32_t pfGpsCourseAnchorResetCount = 0;
uint32_t pfGpsCourseAnchorMs = 0;
bool pfGpsCourseLastPositionValid = false;
double pfGpsCourseLastLat = 0.0;
double pfGpsCourseLastLon = 0.0;
double pfGpsCourseTotalDisplacementSinceBootM = 0.0;
double pfGpsCourseTotalDisplacementSinceLastResetM = 0.0;
bool pfGpsCourseOutputValid = false;
double pfGpsCourseOutputDeg = 0.0;
uint32_t pfGpsCourseOutputMs = 0;
bool pfGpsCourseCandidateValid = false;
double pfGpsCourseCandidateDeg = 0.0;
bool pfGpsCourseLastAcceptedValid = false;
double pfGpsCourseLastAcceptedDeg = 0.0;
double pfGpsCourseLastAcceptedDisplacementM = 0.0;
uint32_t pfGpsCourseLastAcceptedMs = 0;
uint32_t pfGpsCourseLastAcceptedAnchorResetCount = 0;

const char *headingSourceModeName() {
  switch (HEADING_SOURCE_MODE) {
    case 1: return "IMU_RELATIVE";
    case 2: return "GPS_COURSE_WITH_IMU_DIAG";
    case 3: return "MOCK_HEADING";
    case 4: return "FUSION_DIAG";
    default: return "GPS_COURSE";
  }
}
// Only GPS-course-based heading is approved to drive motors; IMU/MOCK/FUSION are
// diagnostic-only. This keeps physical motion tied to GPS course-over-ground.
constexpr bool PF_HEADING_SOURCE_APPROVED_FOR_MOTION =
    (HEADING_SOURCE_MODE == 0) || (HEADING_SOURCE_MODE == 2);
#if IMU_ENABLE
double pfHeadingAgreementErrorDeg = 0.0;
const char *pfHeadingAgreementDiag = "NA";
bool pfHeadingSeedValid = false;
double pfHeadingSeedGpsCourse = 0.0;
double pfHeadingSeedImuYaw = 0.0;
#endif

bool pfNeutralOkFlag = false;
bool pfAutoTimingActive = false;
uint32_t pfAutoEntryMs = 0;
uint32_t pfAutoElapsedMs = 0;
bool pfLatchedStopFlag = false;
bool pfPhysicalOutputActiveFlag = false;

bool pfHc12TargetValid = false;
double pfHc12TargetLat = 0.0;
double pfHc12TargetLon = 0.0;
bool pfHc12Estop = false;
uint32_t pfHc12RxCount = 0;
uint32_t pfHc12TxCount = 0;
uint32_t pfHc12LastRxMs = 0;
bool pfHc12AnyRx = false;
bool pfHc12ParseOk = false;
const char *pfHc12ParseError = "NONE";
const char *pfHc12LastCmd = "NONE";
long pfHc12LastSeq = -1;
String pfHc12RxLine;

void pfMarkCourseAnchorReset(const char *reason, uint32_t now);
const char *pfCourseNoPositionBlockReason();
const char *pfCourseOutputBlockReason(uint32_t now);
bool pfCourseOutputDiagValid(uint32_t now);

bool pfParseDoubleToken(const String &token, double &out) {
  if (token.length() == 0) {
    return false;
  }
  char *endPtr = nullptr;
  double value = strtod(token.c_str(), &endPtr);
  if (endPtr == token.c_str() || *endPtr != '\0') {
    return false;
  }
  out = value;
  return true;
}

#if PATH_FOLLOWING_HC12_ENABLED
void pfHc12SendFrame(const char *type, long seq, const char *payload) {
  String body = String(type) + "," + String(seq) + "," + String(payload);
  uint8_t checksum = checksumXor(body);
  PF_HC12_SERIAL.print('@');
  PF_HC12_SERIAL.print(body);
  PF_HC12_SERIAL.print('*');
  if (checksum < 0x10) {
    PF_HC12_SERIAL.print('0');
  }
  PF_HC12_SERIAL.println(checksum, HEX);
  pfHc12TxCount++;
}
#else
void pfHc12SendFrame(const char *, long, const char *) {}
#endif

// HC-12 waypoint protocol (dry-run only). Updates a dry-run target / estop flag /
// counters and replies, but never drives motors and never touches stationManual.
void pfHandleHc12Frame(const String &type, long seq, const String &payload) {
  pfHc12LastSeq = seq;
  if (type == "PING") {
    pfHc12LastCmd = "PING";
    pfHc12SendFrame("PONG", seq, "PF");
  } else if (type == "SET_TARGET") {
    pfHc12LastCmd = "SET_TARGET";
    String fields[2];
    size_t count = 0;
    double lat = 0.0;
    double lon = 0.0;
    if (splitCsvFields(payload, fields, 2, count) && count == 2 &&
        pfParseDoubleToken(fields[0], lat) && pfParseDoubleToken(fields[1], lon) &&
        lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
      pfHc12TargetLat = lat;
      pfHc12TargetLon = lon;
      pfHc12TargetValid = true;
      pfCourseRefValid = false;  // re-seed heading reference for the new target
      pfMarkCourseAnchorReset("HC12_TARGET", millis());
      pfHc12SendFrame("ACK", seq, "SET_TARGET");
    } else {
      pfHc12SendFrame("ERR", seq, "SET_TARGET_PAYLOAD");
    }
  } else if (type == "STATUS") {
    pfHc12LastCmd = "STATUS";
    String status = String("dist=") + String(pfTargetDistanceM, 1) +
                    " heading_ready=" + (pfHeadingReadyFlag ? "1" : "0") + " block=" + pfBlockReason;
    pfHc12SendFrame("STAT", seq, status.c_str());
  } else if (type == "ESTOP") {
    pfHc12LastCmd = "ESTOP";
    pfHc12Estop = true;  // blocks the (already-gated) physical-output path
    pfHc12SendFrame("ACK", seq, "ESTOP");
  } else if (type == "CLEAR") {
    pfHc12LastCmd = "CLEAR";
    pfHc12TargetValid = false;
    pfHc12Estop = false;
    pfCourseRefValid = false;
    pfMarkCourseAnchorReset("HC12_CLEAR", millis());
    pfHc12SendFrame("ACK", seq, "CLEAR");
  } else {
    pfHc12LastCmd = "UNSUPPORTED";
    pfHc12SendFrame("ERR", seq, "UNSUPPORTED");
  }
}

void pfProcessHc12Line(const String &line) {
  if (line.length() == 0) {
    return;
  }
  pfHc12RxCount++;
  pfHc12LastRxMs = millis();
  pfHc12AnyRx = true;
  String type;
  String payload;
  long seq = 0;
  if (!decodeFrame(line, type, seq, payload)) {
    pfHc12ParseOk = false;
    pfHc12ParseError = "BAD_FRAME";
    return;
  }
  pfHc12ParseOk = true;
  pfHc12ParseError = "NONE";
  pfHandleHc12Frame(type, seq, payload);
}

void pfReadHc12() {
#if PATH_FOLLOWING_HC12_ENABLED
  while (PF_HC12_SERIAL.available() > 0) {
    char c = static_cast<char>(PF_HC12_SERIAL.read());
    if (c == '\n') {
      pfProcessHc12Line(pfHc12RxLine);
      pfHc12RxLine = "";
    } else if (c != '\r') {
      pfHc12RxLine += c;
      if (pfHc12RxLine.length() > 120) {
        pfHc12RxLine = "";  // overflow guard
      }
    }
  }
#endif
}

bool pfResolveCurrentPosition() {
  if (gpsDryrunReady() && gps.location.isValid()) {
    pfCurrentLat = gps.location.lat();
    pfCurrentLon = gps.location.lng();
    pfPositionSource = "gps";
    return true;
  }
  if (PATH_FOLLOWING_ALLOW_MOCK_POSITION_ENABLED) {
    pfCurrentLat = PATH_FOLLOWING_MOCK_CURRENT_LAT_VALUE;
    pfCurrentLon = PATH_FOLLOWING_MOCK_CURRENT_LON_VALUE;
    pfPositionSource = "mock";
    return true;
  }
  pfPositionSource = "none";
  return false;
}

void pfMarkCourseAnchorReset(const char *reason, uint32_t now) {
  pfGpsCourseAnchorResetReason = reason;
  pfGpsCourseAnchorResetCount++;
  pfGpsCourseAnchorMs = now;
  pfGpsCourseTotalDisplacementSinceLastResetM = 0.0;
}

const char *pfCourseNoPositionBlockReason() {
  const char *reason = gpsDryrunBlockReason();
  if (strcmp(reason, "STALE_LOCATION") == 0) {
    return "GPS_AGE_BLOCK";
  }
  if (strcmp(reason, "OK") == 0) {
    return "NO_POSITION";
  }
  return "GPS_QUALITY_BLOCK";
}

bool pfCourseOutputDiagValid(uint32_t now) {
  if (!GPS_COURSE_LATCH_DIAG_ENABLED) {
    return false;
  }
  if (!pfGpsCourseLastAcceptedValid) {
    return false;
  }
  if (now - pfGpsCourseLastAcceptedMs > GPS_COURSE_OUTPUT_LATCH_MS_VALUE) {
    return false;
  }
  return gpsDryrunReady();
}

const char *pfCourseOutputBlockReason(uint32_t now) {
  if (!GPS_COURSE_LATCH_DIAG_ENABLED) {
    return "DIAG_LATCH_DISABLED";
  }
  if (!pfGpsCourseLastAcceptedValid) {
    return "NO_ACCEPTED_COURSE_YET";
  }
  if (now - pfGpsCourseLastAcceptedMs > GPS_COURSE_OUTPUT_LATCH_MS_VALUE) {
    return "ACCEPTED_COURSE_EXPIRED";
  }
  if (!gpsDryrunReady()) {
    const char *reason = gpsDryrunBlockReason();
    if (strcmp(reason, "STALE_LOCATION") == 0) {
      return "GPS_AGE_BLOCK";
    }
    return "GPS_QUALITY_BLOCK";
  }
  return "OK";
}

void pfUpdateCourseDistanceDiagnostics() {
  if (!pfPositionValidFlag || strcmp(pfPositionSource, "gps") != 0) {
    pfGpsCourseLastPositionValid = false;
    return;
  }

  if (pfGpsCourseLastPositionValid) {
    double stepM = dryrunDistanceMeters(
        pfGpsCourseLastLat, pfGpsCourseLastLon, pfCurrentLat, pfCurrentLon);
    if (stepM >= 0.0 && stepM < 100.0) {
      pfGpsCourseTotalDisplacementSinceBootM += stepM;
      pfGpsCourseTotalDisplacementSinceLastResetM += stepM;
    }
  }
  pfGpsCourseLastLat = pfCurrentLat;
  pfGpsCourseLastLon = pfCurrentLon;
  pfGpsCourseLastPositionValid = true;
}

void pfResolveActiveTarget(double &lat, double &lon) {
  if (pfHc12TargetValid) {
    lat = pfHc12TargetLat;
    lon = pfHc12TargetLon;
    pfActiveTargetSource = "hc12";
    return;
  }
  lat = pfWaypoints[pfActiveIndex].lat;
  lon = pfWaypoints[pfActiveIndex].lon;
  pfActiveTargetSource = "compile_time";
}

void pfResetSteeringOutputs(const char *reason) {
  pfHeadingReadyFlag = false;
  pfBearingErrorDeg = 0.0;
  pfDesiredForwardCmd = 0.0f;
  pfDesiredTurnCmd = 0.0f;
  pfDesiredLogicalLeftCmd = 0.0f;
  pfDesiredLogicalRightCmd = 0.0f;
  pfDesiredPhysicalACmd = 0.0f;
  pfDesiredPhysicalBCmd = 0.0f;
  pfBlockReason = reason;
}

void updatePathFollowingDryrun(bool rcValid, bool autoSwitchOn, bool rcManualActive,
                               uint16_t steeringUs, uint16_t throttleUs, uint32_t now) {
  pfNeutralOkFlag = (normRcCentered(steeringUs, STEERING_CENTER_US) == 0.0f) &&
                    (normRcCentered(throttleUs, THROTTLE_CENTER_US) == 0.0f);

  pfPositionValidFlag = pfResolveCurrentPosition();
  pfUpdateCourseDistanceDiagnostics();
  double targetLat = 0.0;
  double targetLon = 0.0;
  pfResolveActiveTarget(targetLat, targetLon);

  pfArrivedFlag = false;
  pfTargetDistanceM = 0.0;
  pfTargetBearingDeg = 0.0;
  pfGpsCourseCandidateValid = false;

  if (!pfPositionValidFlag) {
    if (pfCourseRefValid) {
      pfMarkCourseAnchorReset("NO_POSITION", now);
    }
    pfGpsCourseReadyCandidateFlag = false;
    pfGpsCourseBlockReason = pfCourseNoPositionBlockReason();
    pfCourseRefValid = false;
    pfHeadingSource = "NONE";
    pfResetSteeringOutputs("NO_POSITION");
  } else {
    pfTargetDistanceM = dryrunDistanceMeters(pfCurrentLat, pfCurrentLon, targetLat, targetLon);
    pfTargetBearingDeg = dryrunBearingDegrees(pfCurrentLat, pfCurrentLon, targetLat, targetLon);
    pfArrivedFlag = pfTargetDistanceM <= PATH_FOLLOWING_ARRIVAL_RADIUS_M_VALUE;

    if (strcmp(pfPositionSource, "gps") != 0) {
      // Mock/static position cannot yield a course-over-ground heading.
      if (pfCourseRefValid) {
        pfMarkCourseAnchorReset("MOCK_POSITION", now);
      }
      pfGpsCourseReadyCandidateFlag = false;
      pfGpsCourseBlockReason = "MOCK_POSITION";
      pfCourseRefValid = false;
      pfHeadingSource = "NONE_MOCK_NO_COURSE";
      pfResetSteeringOutputs("NO_HEADING");
    } else if (!pfCourseRefValid) {
      pfCourseRefLat = pfCurrentLat;
      pfCourseRefLon = pfCurrentLon;
      pfCourseRefValid = true;
      pfMarkCourseAnchorReset("ANCHOR_SEEDED", now);
      pfGpsCourseReadyCandidateFlag = false;
      pfGpsCourseBlockReason = "ANCHOR_SEEDED";
      pfHeadingSource = "GPS_COURSE";
      pfResetSteeringOutputs("NO_HEADING");
    } else {
      pfCourseDisplacementM =
          dryrunDistanceMeters(pfCourseRefLat, pfCourseRefLon, pfCurrentLat, pfCurrentLon);
      pfGpsCourseReadyCandidateFlag = pfCourseDisplacementM >= SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M;
      if (pfCourseDisplacementM < SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M) {
        pfGpsCourseBlockReason = "BELOW_MIN_DISPLACEMENT";
        pfHeadingSource = "GPS_COURSE";
        pfResetSteeringOutputs("NO_HEADING");
      } else {
        double acceptedDisplacementM = pfCourseDisplacementM;
        pfEstimatedCourseDeg =
            dryrunBearingDegrees(pfCourseRefLat, pfCourseRefLon, pfCurrentLat, pfCurrentLon);
        pfGpsCourseCandidateValid = true;
        pfGpsCourseCandidateDeg = pfEstimatedCourseDeg;
        pfCourseRefLat = pfCurrentLat;
        pfCourseRefLon = pfCurrentLon;
        pfMarkCourseAnchorReset("COURSE_ACCEPTED_RESEED", now);
        pfGpsCourseBlockReason = "OK";
        pfGpsCourseOutputValid = true;
        pfGpsCourseOutputDeg = pfEstimatedCourseDeg;
        pfGpsCourseOutputMs = now;
        pfGpsCourseLastAcceptedValid = true;
        pfGpsCourseLastAcceptedDeg = pfEstimatedCourseDeg;
        pfGpsCourseLastAcceptedDisplacementM = acceptedDisplacementM;
        pfGpsCourseLastAcceptedMs = now;
        pfGpsCourseLastAcceptedAnchorResetCount = pfGpsCourseAnchorResetCount;
        pfHeadingReadyFlag = true;
        pfHeadingSource = "GPS_COURSE";
        pfBearingErrorDeg = normalizeBearingErrorDegrees(pfTargetBearingDeg - pfEstimatedCourseDeg);

        float forward = PATH_FOLLOWING_MAX_FORWARD_CMD_VALUE;
        float turn =
            clampUnit(static_cast<float>(pfBearingErrorDeg / 90.0)) * PATH_FOLLOWING_MAX_TURN_CMD_VALUE;
        if (turn > PATH_FOLLOWING_MAX_TURN_CMD_VALUE) {
          turn = PATH_FOLLOWING_MAX_TURN_CMD_VALUE;
        } else if (turn < -PATH_FOLLOWING_MAX_TURN_CMD_VALUE) {
          turn = -PATH_FOLLOWING_MAX_TURN_CMD_VALUE;
        }
        pfDesiredForwardCmd = forward;
        pfDesiredTurnCmd = turn;
        pfDesiredLogicalLeftCmd = clampUnit(forward + turn);
        pfDesiredLogicalRightCmd = clampUnit(forward - turn);
        pfDesiredPhysicalACmd = clampUnit((pfDesiredLogicalLeftCmd + pfDesiredLogicalRightCmd) * 0.5f);
        pfDesiredPhysicalBCmd = clampUnit((pfDesiredLogicalRightCmd - pfDesiredLogicalLeftCmd) * 0.5f);
        pfBlockReason = "OK";
      }
    }

    // Advance along the compile-time path on arrival (an HC-12 override holds one target).
    if (pfArrivedFlag && !pfHc12TargetValid && (pfActiveIndex + 1 < pfWaypointCount)) {
      pfActiveIndex++;
      pfCourseRefValid = false;
      pfMarkCourseAnchorReset("WAYPOINT_ADVANCE", now);
    }
  }

#if IMU_ENABLE
  // GPS-course vs IMU-yaw seeded delta comparison (diagnostic only; never drives
  // motors). IMU yaw is relative, so we compare CHANGE in GPS course against
  // CHANGE in IMU yaw since the first moment both were available.
  bool gpsCourseForAgreementValid = pfHeadingReadyFlag || pfCourseOutputDiagValid(now);
  double gpsCourseForAgreementDeg = pfHeadingReadyFlag ? pfEstimatedCourseDeg : pfGpsCourseLastAcceptedDeg;
  if (gpsCourseForAgreementValid && imuHeadingReady()) {
    if (!pfHeadingSeedValid) {
      pfHeadingSeedValid = true;
      pfHeadingSeedGpsCourse = gpsCourseForAgreementDeg;
      pfHeadingSeedImuYaw = imuRelativeYawDeg;
      pfHeadingAgreementErrorDeg = 0.0;
      pfHeadingAgreementDiag = pfHeadingReadyFlag ? "SEEDED" : "GPS_COURSE_LATCH_AVAILABLE";
    } else {
      double gpsDelta = normalizeBearingErrorDegrees(gpsCourseForAgreementDeg - pfHeadingSeedGpsCourse);
      double imuDelta = normalizeBearingErrorDegrees(imuRelativeYawDeg - pfHeadingSeedImuYaw);
      pfHeadingAgreementErrorDeg = normalizeBearingErrorDegrees(gpsDelta - imuDelta);
      pfHeadingAgreementDiag = "HEADING_AGREEMENT_READY";
    }
  } else if (!imuHeadingReady()) {
    pfHeadingAgreementDiag = "IMU_NOT_READY";
  } else if (pfGpsCourseLastAcceptedValid &&
             now - pfGpsCourseLastAcceptedMs > GPS_COURSE_OUTPUT_LATCH_MS_VALUE) {
    pfHeadingAgreementDiag = "GPS_COURSE_EXPIRED";
  } else {
    pfHeadingAgreementDiag = "WAITING_GPS_COURSE";
  }
#endif

  bool autoActive = rcValid && autoSwitchOn && !rcManualActive;
  if (rcManualActive) {
    pfLatchedStopFlag = false;  // latch clears only on MANUAL
  }
  if (autoActive) {
    if (!pfAutoTimingActive) {
      pfAutoTimingActive = true;
      pfAutoEntryMs = now;
    }
    pfAutoElapsedMs = now - pfAutoEntryMs;
    if (pfAutoElapsedMs > PATH_FOLLOWING_MAX_AUTO_MS_VALUE) {
      pfLatchedStopFlag = true;
    }
  } else {
    pfAutoTimingActive = false;
    pfAutoEntryMs = 0;
    pfAutoElapsedMs = 0;
  }
}

bool pfDistanceInBounds() {
  return pfPositionValidFlag &&
         pfTargetDistanceM >= PATH_FOLLOWING_MIN_TARGET_DISTANCE_M_VALUE &&
         pfTargetDistanceM <= PATH_FOLLOWING_MAX_TARGET_DISTANCE_M_VALUE;
}

bool pfHc12TargetFreshIfUsed() {
  if (!pfHc12TargetValid) {
    return true;
  }
  return pfHc12AnyRx && (millis() - pfHc12LastRxMs) <= PATH_FOLLOWING_HC12_CMD_STALE_MS_VALUE;
}

// The full physical-output gate (Task F). Returns "OK" only when every compile
// AND runtime condition is satisfied; otherwise the string is the block reason.
// Default build -> COMPILE_GATE_OFF, so motors can never move.
const char *pathFollowingPhysicalBlockReason(bool rcValid, bool autoSwitchOn, bool rcManualActive) {
  if (!PATH_FOLLOWING_PHYSICAL_COMPILE_GATE) {
    return "COMPILE_GATE_OFF";
  }
  if (!PF_HEADING_SOURCE_APPROVED_FOR_MOTION) {
    // IMU_RELATIVE / MOCK_HEADING / FUSION_DIAG are diagnostic-only.
    return "HEADING_SOURCE_NOT_APPROVED_FOR_MOTION";
  }
  if (!PATH_FOLLOWING_MODE_CHANNEL_STABLE_ACK) {
    return "MODE_CHANNEL_NOT_STABLE";
  }
  if (!rcValid) {
    return "RC_INVALID";
  }
  if (!autoSwitchOn || rcManualActive) {
    return "NOT_AUTO";
  }
  if (pfHc12Estop) {
    return "HC12_ESTOP";
  }
  if (pfLatchedStopFlag) {
    return "LATCHED_STOP";
  }
  if (!pfNeutralOkFlag) {
    return "RC_NOT_NEUTRAL";
  }
  if (!gpsMotionReady()) {
    return "GPS_NOT_MOTION_READY";
  }
  if (!pfHeadingReadyFlag) {
    return "NO_HEADING";
  }
  if (pfArrivedFlag) {
    return "ARRIVED";
  }
  if (!pfDistanceInBounds()) {
    return "DISTANCE_OUT_OF_RANGE";
  }
  if (!pfHc12TargetFreshIfUsed()) {
    return "HC12_TARGET_STALE";
  }
  return "OK";
}

void pathFollowingDebugPrint(bool rcValid, bool autoSwitchOn, uint16_t steeringUs,
                             uint16_t throttleUs, uint16_t modeUs, const char *physBlockReason) {
  if (!ENABLE_USB_DEBUG) {
    return;
  }
  uint32_t now = millis();
  if (now - lastUsbDebugMs < USB_DEBUG_PERIOD_MS) {
    return;
  }
  lastUsbDebugMs = now;

  Serial.print(F("USBDBG "));
  Serial.print(FIRMWARE_ID);
  Serial.print(F(" path_following_dryrun=true mode="));
  Serial.print(modeName(currentMode));
  Serial.print(F(" control_source="));
  Serial.print(controlSourceName(currentControlSource));
  Serial.print(F(" rc_ok="));
  Serial.print(rcValid ? F("true") : F("false"));
  Serial.print(F(" auto_sw="));
  Serial.print(autoSwitchOn ? F("true") : F("false"));
  Serial.print(F(" steer_us="));
  Serial.print(steeringUs);
  Serial.print(F(" throttle_us="));
  Serial.print(throttleUs);
  Serial.print(F(" mode_us="));
  Serial.print(modeUs);

  Serial.print(F(" gps_dryrun_ready="));
  Serial.print(gpsDryrunReady() ? F("true") : F("false"));
  Serial.print(F(" gps_motion_ready="));
  Serial.print(gpsMotionReady() ? F("true") : F("false"));
  Serial.print(F(" gps_block_reason="));
  Serial.print(gpsBlockReason());
  Serial.print(F(" gps_sats="));
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_hdop="));
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.hdop(), 2);
  } else {
    Serial.print(F("NA"));
  }

  Serial.print(F(" position_source="));
  Serial.print(pfPositionSource);
  Serial.print(F(" current_lat="));
  if (pfPositionValidFlag) {
    Serial.print(pfCurrentLat, 7);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" current_lon="));
  if (pfPositionValidFlag) {
    Serial.print(pfCurrentLon, 7);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" active_target_source="));
  Serial.print(pfActiveTargetSource);
  Serial.print(F(" wp_index="));
  Serial.print(static_cast<unsigned long>(pfActiveIndex));
  Serial.print(F(" wp_count="));
  Serial.print(static_cast<unsigned long>(pfWaypointCount));
  Serial.print(F(" target_distance_m="));
  Serial.print(pfTargetDistanceM, 2);
  Serial.print(F(" target_bearing_deg="));
  Serial.print(pfTargetBearingDeg, 1);
  Serial.print(F(" arrived="));
  Serial.print(pfArrivedFlag ? F("true") : F("false"));
  Serial.print(F(" heading_ready="));
  Serial.print(pfHeadingReadyFlag ? F("true") : F("false"));
  Serial.print(F(" heading_source="));
  Serial.print(pfHeadingSource);
  Serial.print(F(" course_displacement_m="));
  Serial.print(pfCourseDisplacementM, 2);
  bool gpsCourseOutputDiagValid = pfCourseOutputDiagValid(now);
  Serial.print(F(" gps_course_ready_candidate="));
  Serial.print(pfGpsCourseReadyCandidateFlag ? F("true") : F("false"));
  Serial.print(F(" gps_course_block_reason="));
  Serial.print(pfGpsCourseBlockReason);
  Serial.print(F(" gps_course_min_displacement_m="));
  Serial.print(SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M, 1);
  Serial.print(F(" gps_course_anchor_reset_reason="));
  Serial.print(pfGpsCourseAnchorResetReason);
  Serial.print(F(" gps_course_anchor_reset_count="));
  Serial.print(pfGpsCourseAnchorResetCount);
  Serial.print(F(" gps_course_anchor_age_ms="));
  if (pfCourseRefValid && pfGpsCourseAnchorMs > 0) {
    Serial.print(now - pfGpsCourseAnchorMs);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_segment_displacement_m="));
  Serial.print(pfCourseDisplacementM, 2);
  Serial.print(F(" gps_course_total_displacement_m_since_boot="));
  Serial.print(pfGpsCourseTotalDisplacementSinceBootM, 2);
  Serial.print(F(" gps_course_total_displacement_m_since_last_reset="));
  Serial.print(pfGpsCourseTotalDisplacementSinceLastResetM, 2);
  Serial.print(F(" gps_course_latch_diag="));
  Serial.print(GPS_COURSE_LATCH_DIAG_ENABLED ? F("true") : F("false"));
  Serial.print(F(" gps_course_output_latch_ms="));
  Serial.print(GPS_COURSE_OUTPUT_LATCH_MS_VALUE);
  Serial.print(F(" gps_course_candidate_deg="));
  if (pfGpsCourseCandidateValid) {
    Serial.print(pfGpsCourseCandidateDeg, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_last_accepted_valid="));
  Serial.print(pfGpsCourseLastAcceptedValid ? F("true") : F("false"));
  Serial.print(F(" gps_course_last_accepted_deg="));
  if (pfGpsCourseLastAcceptedValid) {
    Serial.print(pfGpsCourseLastAcceptedDeg, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_last_accepted_age_ms="));
  if (pfGpsCourseLastAcceptedValid) {
    Serial.print(now - pfGpsCourseLastAcceptedMs);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_last_accepted_displacement_m="));
  if (pfGpsCourseLastAcceptedValid) {
    Serial.print(pfGpsCourseLastAcceptedDisplacementM, 2);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_last_accepted_anchor_reset_count="));
  if (pfGpsCourseLastAcceptedValid) {
    Serial.print(pfGpsCourseLastAcceptedAnchorResetCount);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_output_valid="));
  Serial.print(gpsCourseOutputDiagValid ? F("true") : F("false"));
  Serial.print(F(" gps_course_estimated_deg="));
  if (pfGpsCourseOutputValid) {
    Serial.print(pfGpsCourseOutputDeg, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_output_deg="));
  if (gpsCourseOutputDiagValid) {
    Serial.print(pfGpsCourseLastAcceptedDeg, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_output_block_reason="));
  Serial.print(pfCourseOutputBlockReason(now));
  Serial.print(F(" gps_course_output_age_ms="));
  if (gpsCourseOutputDiagValid) {
    Serial.print(now - pfGpsCourseLastAcceptedMs);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_age_ms="));
  if (pfGpsCourseOutputValid) {
    Serial.print(now - pfGpsCourseOutputMs);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" gps_course_hdop_ok="));
  Serial.print(gpsMotionHdopOk() ? F("true") : F("false"));
  Serial.print(F(" gps_course_sats_ok="));
  Serial.print(gpsMotionSatsOk() ? F("true") : F("false"));
  Serial.print(F(" estimated_course_deg="));
  Serial.print(pfEstimatedCourseDeg, 1);
  Serial.print(F(" bearing_error_deg="));
  Serial.print(pfBearingErrorDeg, 1);
  Serial.print(F(" desired_forward_cmd="));
  Serial.print(pfDesiredForwardCmd, 3);
  Serial.print(F(" desired_turn_cmd="));
  Serial.print(pfDesiredTurnCmd, 3);
  Serial.print(F(" desired_logical_left_cmd="));
  Serial.print(pfDesiredLogicalLeftCmd, 3);
  Serial.print(F(" desired_logical_right_cmd="));
  Serial.print(pfDesiredLogicalRightCmd, 3);
  Serial.print(F(" desired_physical_a_cmd="));
  Serial.print(pfDesiredPhysicalACmd, 3);
  Serial.print(F(" desired_physical_b_cmd="));
  Serial.print(pfDesiredPhysicalBCmd, 3);
  Serial.print(F(" path_following_block_reason="));
  Serial.print(pfBlockReason);

  Serial.print(F(" physical_compile_gate="));
  Serial.print(PATH_FOLLOWING_PHYSICAL_COMPILE_GATE ? F("true") : F("false"));
  Serial.print(F(" physical_path_following_enable="));
  Serial.print((PHYSICAL_PATH_FOLLOWING_ENABLE != 0) ? F("true") : F("false"));
  Serial.print(F(" allow_motor_output="));
  Serial.print((PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT != 0) ? F("true") : F("false"));
  Serial.print(F(" ground_crawl_test_mode="));
  Serial.print((GROUND_CRAWL_TEST_MODE != 0) ? F("true") : F("false"));
  Serial.print(F(" auto_motion_armed="));
  Serial.print((AUTO_MOTION_ARMED != 0) ? F("true") : F("false"));
  Serial.print(F(" mode_channel_stable_ack="));
  Serial.print(PATH_FOLLOWING_MODE_CHANNEL_STABLE_ACK ? F("true") : F("false"));
  Serial.print(F(" neutral_ok="));
  Serial.print(pfNeutralOkFlag ? F("true") : F("false"));
  Serial.print(F(" latched_stop="));
  Serial.print(pfLatchedStopFlag ? F("true") : F("false"));
  Serial.print(F(" auto_elapsed_ms="));
  Serial.print(pfAutoElapsedMs);
  Serial.print(F(" max_auto_ms="));
  Serial.print(PATH_FOLLOWING_MAX_AUTO_MS_VALUE);
  Serial.print(F(" max_forward_cmd="));
  Serial.print(PATH_FOLLOWING_MAX_FORWARD_CMD_VALUE, 3);
  Serial.print(F(" max_turn_cmd="));
  Serial.print(PATH_FOLLOWING_MAX_TURN_CMD_VALUE, 3);
  Serial.print(F(" physical_output_active="));
  Serial.print(pfPhysicalOutputActiveFlag ? F("true") : F("false"));
  Serial.print(F(" physical_block_reason="));
  Serial.print(physBlockReason);
  Serial.print(F(" final_left_cmd="));
  Serial.print(lastLeftOutputCmd, 3);
  Serial.print(F(" final_right_cmd="));
  Serial.print(lastRightOutputCmd, 3);
  Serial.print(F(" physical_a_cmd="));
  Serial.print(lastOutputLeftPinCmd, 3);
  Serial.print(F(" physical_b_cmd="));
  Serial.print(lastOutputRightPinCmd, 3);

  Serial.print(F(" hc12_pf_port="));
  Serial.print(PF_HC12_PORT_NAME);
  Serial.print(F(" hc12_rx_count="));
  Serial.print(pfHc12RxCount);
  Serial.print(F(" hc12_tx_count="));
  Serial.print(pfHc12TxCount);
  Serial.print(F(" hc12_last_rx_age_ms="));
  if (pfHc12AnyRx) {
    Serial.print(millis() - pfHc12LastRxMs);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" hc12_last_cmd="));
  Serial.print(pfHc12LastCmd);
  Serial.print(F(" hc12_parse_ok="));
  Serial.print(pfHc12ParseOk ? F("true") : F("false"));
  Serial.print(F(" hc12_parse_error="));
  Serial.print(pfHc12ParseError);
  Serial.print(F(" active_target_source_hc12="));
  Serial.print(pfHc12TargetValid ? F("true") : F("false"));
  Serial.print(F(" hc12_estop="));
  Serial.print(pfHc12Estop ? F("true") : F("false"));
  Serial.print(F(" hc12_enabled="));
  Serial.print(PATH_FOLLOWING_HC12_ENABLED ? F("true") : F("false"));
  Serial.print(F(" hc12_port="));
  Serial.print(PF_HC12_PORT_NAME);

  Serial.print(F(" heading_source="));
  Serial.print(headingSourceModeName());
#if IMU_ENABLE
  Serial.print(F(" imu_enabled=true imu_type="));
  Serial.print(imuTypeName());
  Serial.print(F(" imu_present="));
  Serial.print(imuPresentFlag ? F("true") : F("false"));
  Serial.print(F(" imu_i2c_addr=0x"));
  if (imuActiveAddr < 0x10) Serial.print('0');
  Serial.print(imuActiveAddr, HEX);
  Serial.print(F(" imu_chip_id="));
  if (imuChipIdOkFlag) {
    Serial.print(F("0x"));
    if (imuChipIdValue < 0x10) Serial.print('0');
    Serial.print(imuChipIdValue, HEX);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_whoami="));
  if (imuType == IMU_TYPE_BMI160) {
    Serial.print(F("NA_BMI160"));
  } else if (imuWhoamiOkFlag) {
    Serial.print(F("0x"));
    if (imuWhoamiValue < 0x10) Serial.print('0');
    Serial.print(imuWhoamiValue, HEX);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_pmu_status="));
  if (imuType == IMU_TYPE_BMI160 && imuPmuStatusOkFlag) {
    Serial.print(F("0x"));
    if (imuPmuStatusValue < 0x10) Serial.print('0');
    Serial.print(imuPmuStatusValue, HEX);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_accel_pmu="));
  if (imuType == IMU_TYPE_BMI160 && imuPmuStatusOkFlag) {
    Serial.print(bmi160PmuModeName(bmi160AccelPmuBits(imuPmuStatusValue)));
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_gyro_pmu="));
  if (imuType == IMU_TYPE_BMI160 && imuPmuStatusOkFlag) {
    Serial.print(bmi160PmuModeName(bmi160GyroPmuBits(imuPmuStatusValue)));
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_pmu_normal="));
  if (imuType == IMU_TYPE_BMI160) {
    Serial.print(imuPmuNormalFlag ? F("true") : F("false"));
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" imu_accel_scale_defaulted="));
  Serial.print(imuAccelRangeDefaultFlag ? F("true") : F("false"));
  Serial.print(F(" imu_gyro_scale_defaulted="));
  Serial.print(imuGyroRangeDefaultFlag ? F("true") : F("false"));
  Serial.print(F(" imu_raw_all_zero="));
  Serial.print(imuRawAllZeroFlag ? F("true") : F("false"));
  Serial.print(F(" imu_data_plausible="));
  Serial.print(imuDataPlausibleFlag ? F("true") : F("false"));
  Serial.print(F(" imu_calibrated="));
  Serial.print(imuCalibratedFlag ? F("true") : F("false"));
  Serial.print(F(" imu_gyro_bias_x="));
  Serial.print(imuGyroBiasX, 2);
  Serial.print(F(" imu_gyro_bias_y="));
  Serial.print(imuGyroBiasY, 2);
  Serial.print(F(" imu_gyro_bias_z="));
  Serial.print(imuGyroBiasZ, 2);
  Serial.print(F(" imu_gyro_cal_x="));
  Serial.print(imuGyroRawX - imuGyroBiasX, 1);
  Serial.print(F(" imu_gyro_cal_y="));
  Serial.print(imuGyroRawY - imuGyroBiasY, 1);
  Serial.print(F(" imu_gyro_cal_z="));
  Serial.print(imuGyroRawZ - imuGyroBiasZ, 1);
  Serial.print(F(" imu_accel_mag_g="));
  Serial.print(imuAccelMagG, 3);
  Serial.print(F(" imu_gyro_mag_dps="));
  Serial.print(imuGyroMagDps, 2);
  Serial.print(F(" imu_stationary="));
  Serial.print(imuStationaryFlag ? F("true") : F("false"));
  Serial.print(F(" imu_relative_yaw_deg="));
  Serial.print(imuRelativeYawDeg, 2);
  Serial.print(F(" imu_yaw_axis="));
  Serial.print(IMU_YAW_AXIS_VALUE);
  Serial.print(F(" imu_yaw_sign="));
  Serial.print(static_cast<int>(IMU_YAW_SIGN_VALUE));
  Serial.print(F(" imu_heading_ready="));
  Serial.print(imuHeadingReady() ? F("true") : F("false"));
  Serial.print(F(" imu_heading_block_reason="));
  Serial.print(imuHeadingBlockReason);
  Serial.print(F(" gps_course_deg="));
  if (gpsCourseOutputDiagValid) {
    Serial.print(pfGpsCourseLastAcceptedDeg, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(F(" heading_agreement_diag="));
  Serial.print(pfHeadingAgreementDiag);
  Serial.print(F(" heading_agreement_error_deg="));
  if (pfHeadingSeedValid) {
    Serial.print(pfHeadingAgreementErrorDeg, 1);
  } else {
    Serial.print(F("NA"));
  }
#else
  Serial.print(F(" imu_enabled=false"));
#endif
  Serial.println();
}
#endif  // PATH_FOLLOWING_DRYRUN

void setup() {
  Serial.begin(USB_BAUD);
#if HC12_LINK_ENABLED
  HC12_SERIAL.begin(HC12_BAUD);
#endif
#if ENABLE_GPS_TELEMETRY && !MOTOR_PULSE_TEST_MODE && !STAGE15_GUARDED_CRAWL_TEST
  GPS_SERIAL.begin(GPS_BAUD);
#endif

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  escLeft.attach(ESC_LEFT_PIN);
  escRight.attach(ESC_RIGHT_PIN);
  motorStop();

  Serial.print("Firmware: ");
  Serial.println(FIRMWARE_ID);
  Serial.println("OpenRB robot controller starting.");
#if IMU_ENABLE
  imuControllerInit();
#endif
#if MOTOR_PULSE_TEST_MODE
  Serial.println("MOTOR_PULSE_TEST_MODE enabled.");
  Serial.println("HC-12 link is disabled/ignored.");
  Serial.println("GPS is not used for this motor pulse calibration mode.");
  Serial.println("RC MANUAL mode can drive normally; AUTO emits one guarded motor pulse if sticks are neutral.");
  Serial.print("MOTOR_PULSE_CMD=");
  Serial.println(MOTOR_PULSE_CMD_VALUE, 3);
  Serial.print("MOTOR_PULSE_LEFT_CMD=");
  Serial.println(MOTOR_PULSE_LEFT_CMD_VALUE, 3);
  Serial.print("MOTOR_PULSE_RIGHT_CMD=");
  Serial.println(MOTOR_PULSE_RIGHT_CMD_VALUE, 3);
  Serial.print("MOTOR_PULSE_MS=");
  Serial.println(MOTOR_PULSE_MS_VALUE);
  Serial.print("MOTOR_OUTPUT_SWAP_LR=");
  Serial.println(MOTOR_OUTPUT_SWAP_LR_ENABLED ? "true" : "false");
#elif STAGE15_GUARDED_CRAWL_TEST
  Serial.println("STAGE15_GUARDED_CRAWL_TEST enabled.");
  Serial.println("Stage 15 is bounded motor sanity only: no path following, no path package, no HC-12 target commands.");
  Serial.println("USB commands accepted: test_forward_pulse, test_backward_pulse, test_rotate_left_pulse, test_rotate_right_pulse, test_stop.");
  Serial.print("STAGE15_MAX_CMD=");
  Serial.println(STAGE15_MAX_CMD_VALUE, 3);
  Serial.print("STAGE15_PULSE_MS=");
  Serial.println(STAGE15_PULSE_MS_VALUE);
  Serial.print("STAGE15_INTER_PULSE_MS=");
  Serial.println(STAGE15_INTER_PULSE_MS_VALUE);
  Serial.print("STAGE15_REQUIRE_RC_NEUTRAL=");
  Serial.println(STAGE15_REQUIRE_RC_NEUTRAL_VALUE ? "true" : "false");
  Serial.println("PHYSICAL_PATH_FOLLOWING_ENABLE=0 PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 required by compile guard.");
#elif STAGE16_USB_GUARDED_CRAWL || STAGE17_FIRST_PRIMITIVE_CRAWL || STAGE18_MOTOR_MAPPING_PROBE || STAGE20_PHYSICAL_AB_GUARDED_CRAWL
#if USB_PULSE_TEST_GUARDED || STATION_DRIVE_GUARDED
  Serial.println("USB_PULSE_TEST enabled.");
  Serial.println("usb-pulse-test is bounded laptop USB physical A/B pulse validation only.");
  Serial.println("USB commands accepted: USB_PULSE_TEST_ARM seq=<n>, USB_PULSE_TEST_CMD seq=<n> a=<throttle> b=<turn> ms=<ms>, USB_PULSE_TEST_STOP seq=<n>.");
  Serial.print("USB_PULSE_TEST_MAX_ABS_A=");
  Serial.println(STAGE20_MAX_ABS_A_VALUE, 3);
  Serial.print("USB_PULSE_TEST_MAX_ABS_B=");
  Serial.println(STAGE20_MAX_ABS_B_VALUE, 3);
  Serial.print("USB_PULSE_TEST_MAX_MS=");
  Serial.println(STAGE20_MAX_MS_VALUE);
  Serial.print("USB_PULSE_TEST_IGNORE_RC_INPUT=");
  Serial.println(STATION_MANUAL_IGNORE_RC_INPUT_ENABLED ? "true" : "false");
#elif STAGE20_PHYSICAL_AB_GUARDED_CRAWL
  Serial.println("STAGE20_PHYSICAL_AB_GUARDED_CRAWL enabled.");
  Serial.println("Stage 20 is direct physical A/B manual-equivalent guarded crawl only.");
  Serial.println("USB commands accepted: STAGE20_ARM seq=<n>, STAGE20_CMD seq=<n> a=<throttle> b=<turn> ms=<ms>, STAGE20_STOP seq=<n>.");
  Serial.print("STAGE20_MAX_ABS_A=");
  Serial.println(STAGE20_MAX_ABS_A_VALUE, 3);
  Serial.print("STAGE20_MAX_ABS_B=");
  Serial.println(STAGE20_MAX_ABS_B_VALUE, 3);
  Serial.print("STAGE20_MAX_MS=");
  Serial.println(STAGE20_MAX_MS_VALUE);
#elif STAGE18_MOTOR_MAPPING_PROBE
  Serial.println("STAGE18_MOTOR_MAPPING_PROBE enabled.");
  Serial.println("Stage 18 is bounded motor mapping and power diagnostics only.");
  Serial.println("USB commands accepted: STAGE18_ARM seq=<n>, STAGE18_CMD seq=<n> left=<cmd> right=<cmd> ms=<ms>, STAGE18_STOP seq=<n>.");
  Serial.print("STAGE18_MAX_CMD=");
  Serial.println(STAGE18_MAX_CMD_VALUE, 3);
  Serial.print("STAGE18_MAX_MS=");
  Serial.println(STAGE18_MAX_MS_VALUE);
#elif STAGE17_FIRST_PRIMITIVE_CRAWL
  Serial.println("STAGE17_FIRST_PRIMITIVE_CRAWL enabled.");
  Serial.println("Stage 17 is USB-tethered station-supervised first-primitive crawl only.");
  Serial.println("USB commands accepted: STAGE17_ARM seq=<n>, STAGE17_CMD seq=<n> left=<cmd> right=<cmd> ms=<ms>, STAGE17_STOP seq=<n>.");
  Serial.print("STAGE17_MAX_CMD=");
  Serial.println(STAGE17_MAX_CMD_VALUE, 3);
  Serial.print("STAGE17_MAX_MS=");
  Serial.println(STAGE17_MAX_MS_VALUE);
#else
  Serial.println("STAGE16_USB_GUARDED_CRAWL enabled.");
  Serial.println("Stage 16 is USB-tethered station-supervised bounded crawl only.");
  Serial.println("USB commands accepted: STAGE16_ARM seq=<n>, STAGE16_CMD seq=<n> left=<cmd> right=<cmd> ms=<ms>, STAGE16_STOP seq=<n>.");
  Serial.print("STAGE16_MAX_CMD=");
  Serial.println(STAGE16_MAX_CMD_VALUE, 3);
  Serial.print("STAGE16_MAX_MS=");
  Serial.println(STAGE16_MAX_MS_VALUE);
#endif
  Serial.println("OpenRB does not read path packages and does not run compile-time waypoint following.");
  Serial.println("PHYSICAL_PATH_FOLLOWING_ENABLE=0 PATH_FOLLOWING_ALLOW_MOTOR_OUTPUT=0 required by compile guard.");
#elif FIXED_WIRING_GPS_SERIAL2_DIAG
  Serial.println("FIXED_WIRING_GPS_SERIAL2_DIAG enabled.");
  Serial.println("HC-12 link is disabled/ignored to avoid Serial2 conflict.");
  Serial.println("Motor outputs are forced neutral; station and RC drive commands are ignored.");
  Serial.println("GPS diagnostic uses current fixed wiring: OpenRB-150 Serial2 at 9600 baud.");
#elif FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN
  Serial.println("FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN enabled.");
  Serial.println("HC-12 link is disabled/ignored to avoid Serial2 conflict.");
  Serial.println("GPS uses current fixed wiring: OpenRB-150 Serial2 at 9600 baud.");
  Serial.println("RC MANUAL mode can drive normally; AUTO mode is computation-only with motor outputs neutral.");
  Serial.println("No autonomous waypoint following is implemented in this dry-run build.");
#elif FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT
  Serial.println("FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT enabled.");
  Serial.println("HC-12 link is disabled/ignored to avoid Serial2 conflict.");
  Serial.println("GPS uses current fixed wiring: OpenRB-150 Serial2 at 9600 baud.");
  Serial.println("RC MANUAL mode can drive normally; AUTO mode uses one placeholder waypoint only.");
  Serial.print("AUTO_MOTION_ARMED=");
  Serial.println(SINGLE_WAYPOINT_AUTO_MOTION_ARMED ? "1" : "0");
  Serial.print("SINGLE_WP_CRAWL_BASE_CMD=");
  Serial.println(SINGLE_WAYPOINT_CRAWL_BASE_CMD, 3);
  Serial.print("SINGLE_WP_STEERING_DRYRUN=");
  Serial.println(SINGLE_WAYPOINT_STEERING_DRYRUN_ENABLED ? "1" : "0");
  Serial.print("SINGLE_WP_COURSE_MIN_DISPLACEMENT_M=");
  Serial.println(SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_M, 1);
  Serial.print("SINGLE_WP_COURSE_MIN_DISPLACEMENT_SOURCE=");
  Serial.println(SINGLE_WAYPOINT_COURSE_MIN_DISPLACEMENT_SOURCE);
  Serial.print("target_override_enabled=");
  Serial.println(SINGLE_WAYPOINT_TARGET_OVERRIDE_ENABLED ? "true" : "false");
  Serial.print("target_source=");
  Serial.println(SINGLE_WAYPOINT_TARGET_SOURCE);
  Serial.print("target_lat_macro=");
  Serial.println(SINGLE_WAYPOINT_TARGET_LAT_MACRO);
  Serial.print("target_lon_macro=");
  Serial.println(SINGLE_WAYPOINT_TARGET_LON_MACRO);
  Serial.print("target_lat=");
  Serial.println(SINGLE_WAYPOINT_TARGET_LAT, 7);
  Serial.print("target_lon=");
  Serial.println(SINGLE_WAYPOINT_TARGET_LON, 7);
  Serial.println("AUTO_MOTION_ARMED=0 computes candidate commands only and forces motor outputs neutral.");
  Serial.println("No multi-waypoint, mission.json, or coverage/lawnmower driving is implemented.");
#else
  Serial.println("GPS telemetry uses OpenRB-150 Serial3 (D13/RX) at 9600 baud.");
#endif
  Serial.println("Motor tests are wheel-off-ground only.");
  Serial.print("mode_channel_index=");
  Serial.print(MODE_CHANNEL_INDEX_VALUE);
  Serial.print(" (receiver CH");
  Serial.print(MODE_CHANNEL_INDEX_VALUE + 1);
  Serial.println("); override with -DMODE_CHANNEL_INDEX=<0-based> after probe.");
  Serial.println("RC mode input uses the mode channel above; PPM CH7 is reserved/unused.");
  Serial.println("Mode HIGH enters AUTO_READY only; drive stays STOP until explicit AUTO.");
  Serial.println("Station manual accepts CMD,MANUAL only when fresh frames and deadman=1.");
}

void loop() {
#if ENABLE_GPS_TELEMETRY && !MOTOR_PULSE_TEST_MODE && !STAGE15_GUARDED_CRAWL_TEST
  while (GPS_SERIAL.available() > 0) {
    processGpsChar(static_cast<char>(GPS_SERIAL.read()));
  }
#endif

#if HC12_LINK_ENABLED
  while (HC12_SERIAL.available() > 0) {
    char c = static_cast<char>(HC12_SERIAL.read());
    if (c == '\n') {
      processHC12Line(hc12Line);
      hc12Line = "";
    } else if (c != '\r') {
      hc12Line += c;
    }
  }
#endif

  uint16_t steeringUs = 0;
  uint16_t throttleUs = 0;
  uint16_t modeUs = 0;
  readRcChannels(steeringUs, throttleUs, modeUs);

  uint32_t now = millis();
#if IMU_ENABLE
  imuControllerUpdate(now);  // read-only IMU diagnostics; never affects motors
#endif
  bool rcValid = rcChannelsValid(steeringUs, throttleUs, modeUs);
  bool autoSwitchOn = rcAutoSwitchOn(modeUs);
  bool stationManualFresh = stationManualValid();
  bool stationManualActive = stationManualFresh && stationManual.deadman && !stationEstop;
  bool rcManualActive = rcValid && !autoSwitchOn;

#if STAGE15_GUARDED_CRAWL_TEST
  clearAutoCommand();
  clearStationManualCommand();
  bool neutralOk = stage15NeutralOk(steeringUs, throttleUs);
  stage15ReadUsbCommands(rcValid, neutralOk, now);
  stage15UpdatePulse(rcValid, neutralOk, now);
  currentControlSource = stage15PulseActiveFlag ? CONTROL_SOURCE_AUTO : CONTROL_SOURCE_STOP;
  currentMode = rcValid ? (stage15PulseActiveFlag ? AUTO_RUNNING : AUTO_READY) : FAILSAFE;
  return;
#endif

#if STAGE16_USB_GUARDED_CRAWL || STAGE17_FIRST_PRIMITIVE_CRAWL || STAGE18_MOTOR_MAPPING_PROBE || STAGE20_PHYSICAL_AB_GUARDED_CRAWL
  clearAutoCommand();
  clearStationManualCommand();
  bool neutralOk = stage16NeutralOk(steeringUs, throttleUs);
  if (STAGE20_PHYSICAL_AB_GUARDED_CRAWL_ENABLED && rcManualActive && !neutralOk) {
    stage16PulseActiveFlag = false;
    stage16ArmedFlag = false;
    stage16CmdState = "STOPPED";
    stage16LeftCmd = 0.0f;
    stage16RightCmd = 0.0f;
    stage16CmdMs = 0;
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_RC_MANUAL;
    applyManualOverride(steeringUs, throttleUs);
    debugPrintStatus();
    return;
  }
  stage16ReadUsbCommands(rcValid, neutralOk, now);
  stage16UpdatePulse(rcValid, neutralOk, now);
  stage16Heartbeat(rcValid, neutralOk, now);
  currentControlSource = stage16PulseActiveFlag ? CONTROL_SOURCE_AUTO : CONTROL_SOURCE_STOP;
  currentMode = rcValid ? (stage16PulseActiveFlag ? AUTO_RUNNING : AUTO_READY) : FAILSAFE;
  return;
#endif

#if MANUAL_RC_RECOVERY
  clearAutoCommand();
  clearStationManualCommand();
  if (!rcValid) {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = FAILSAFE;
    motorStop();
  } else if (rcManualActive) {
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_RC_MANUAL;
    applyManualOverride(steeringUs, throttleUs);
  } else {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = AUTO_READY;
    motorStop();
  }
  debugPrintStatus();
  return;
#endif

#if MOTOR_PULSE_TEST_MODE
  clearAutoCommand();
  clearStationManualCommand();
  updateMotorPulseState(rcValid, autoSwitchOn, rcManualActive, steeringUs, throttleUs, now);
  if (!rcValid) {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = FAILSAFE;
    motorStop();
  } else if (rcManualActive) {
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_RC_MANUAL;
    applyManualOverride(steeringUs, throttleUs);
  } else if (autoSwitchOn) {
    if (motorPulseReadyFlag) {
      currentMode = AUTO_RUNNING;
      currentControlSource = CONTROL_SOURCE_AUTO;
      applyMotorPulseDirectWheelCommand(MOTOR_PULSE_LEFT_CMD_VALUE, MOTOR_PULSE_RIGHT_CMD_VALUE);
    } else {
      currentMode = AUTO_READY;
      currentControlSource = CONTROL_SOURCE_STOP;
      motorStop();
    }
  } else {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = DISARMED;
    motorStop();
  }
  debugPrintStatus();
  return;
#endif

#if FIXED_WIRING_GPS_SERIAL2_DIAG
  clearAutoCommand();
  clearStationManualCommand();
  currentControlSource = CONTROL_SOURCE_STOP;
  currentMode = rcValid ? DISARMED : FAILSAFE;
  motorStop();
  debugPrintStatus();
  return;
#endif

#if FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN
  clearAutoCommand();
  clearStationManualCommand();
  if (!rcValid) {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = FAILSAFE;
    motorStop();
  } else if (rcManualActive) {
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_RC_MANUAL;
    applyManualOverride(steeringUs, throttleUs);
  } else if (autoSwitchOn) {
    currentMode = AUTO_READY;
    currentControlSource = CONTROL_SOURCE_STOP;
    motorStop();
  } else {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = DISARMED;
    motorStop();
  }
  debugPrintStatus();
  return;
#endif

#if FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT
  clearAutoCommand();
  clearStationManualCommand();
  updateSingleWaypointExperimentState(rcValid, autoSwitchOn, now);
  updateGroundCrawlState(rcValid, autoSwitchOn, rcManualActive, steeringUs, throttleUs);
  if (!rcValid) {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = FAILSAFE;
    motorStop();
  } else if (rcManualActive) {
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_RC_MANUAL;
    applyManualOverride(steeringUs, throttleUs);
  } else if (autoSwitchOn) {
    // Armed AUTO motion is permitted ONLY through the guarded ground crawl harness.
    // Any armed build without GROUND_CRAWL_TEST_MODE=1 (or a failed/latched crawl
    // gate) holds the final commands at zero.
    if (singleWaypointSafetyReadyFlag && SINGLE_WAYPOINT_AUTO_MOTION_ARMED &&
        GROUND_CRAWL_ENABLED && groundCrawlReadyFlag) {
      currentMode = AUTO_RUNNING;
      currentControlSource = CONTROL_SOURCE_AUTO;
      applyAutoCommand(clampGroundCrawl(singleWaypointCandidateLeftCmd),
                       clampGroundCrawl(singleWaypointCandidateRightCmd));
    } else {
      currentMode = AUTO_READY;
      currentControlSource = CONTROL_SOURCE_STOP;
      motorStop();
    }
  } else {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = DISARMED;
    motorStop();
  }
  debugPrintStatus();
  return;
#endif

#if PATH_FOLLOWING_DRYRUN
  clearAutoCommand();
  clearStationManualCommand();
  pfReadHc12();
  updatePathFollowingDryrun(rcValid, autoSwitchOn, rcManualActive, steeringUs, throttleUs, now);
  const char *pfPhysReason =
      pathFollowingPhysicalBlockReason(rcValid, autoSwitchOn, rcManualActive);
  bool pfPhysAllowed = (strcmp(pfPhysReason, "OK") == 0);
  pfPhysicalOutputActiveFlag = false;
  if (!rcValid) {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = FAILSAFE;
    motorStop();
  } else if (rcManualActive) {
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_RC_MANUAL;
    applyManualOverride(steeringUs, throttleUs);
  } else if (autoSwitchOn) {
    // Guarded physical path-following. pfPhysAllowed is true only when every
    // compile gate AND runtime gate passed; the default build keeps it false.
    if (pfPhysAllowed) {
      float left = clampUnit(pfDesiredForwardCmd + pfDesiredTurnCmd);
      float right = clampUnit(pfDesiredForwardCmd - pfDesiredTurnCmd);
      currentMode = AUTO_RUNNING;
      currentControlSource = CONTROL_SOURCE_AUTO;
      pfPhysicalOutputActiveFlag = true;
      applyAutoCommand(left, right);
    } else {
      currentMode = AUTO_READY;
      currentControlSource = CONTROL_SOURCE_STOP;
      motorStop();
    }
  } else {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = DISARMED;
    motorStop();
  }
  pathFollowingDebugPrint(rcValid, autoSwitchOn, steeringUs, throttleUs, modeUs, pfPhysReason);
  return;
#endif

  if (autoCommandActive && !stationLinkValid()) {
    currentMode = LINK_LOST;
    clearAutoCommand();
    motorStop();
  }

  bool autoActive = rcValid && autoSwitchOn && autoCommandActive && stationLinkValid() && !stationEstop;

  if (stationEstop) {
    currentControlSource = CONTROL_SOURCE_STOP;
    currentMode = DISARMED;
    motorStop();
  } else if (stationManualActive) {
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_STATION_MANUAL;
    applyStationManualCommand();
  } else if (rcManualActive) {
    currentMode = MANUAL;
    currentControlSource = CONTROL_SOURCE_RC_MANUAL;
    clearAutoCommand();
    applyManualOverride(steeringUs, throttleUs);
  } else if (autoActive) {
    currentMode = AUTO_RUNNING;
    currentControlSource = CONTROL_SOURCE_AUTO;
    applyAutoCommand(autoLeftCmd, autoRightCmd);
  } else {
    currentControlSource = CONTROL_SOURCE_STOP;
    motorStop();
    if (!rcValid) {
      currentMode = FAILSAFE;
    } else if (autoSwitchOn) {
      if (currentMode != LINK_LOST) {
        currentMode = AUTO_READY;
      }
    } else {
      currentMode = DISARMED;
    }
  }

  if (now - lastTelemetryMs > TELEMETRY_PERIOD_MS) {
    sendGpsTelemetry();
    lastTelemetryMs = now;
  }

  if (now - lastStatusMs > STATUS_PERIOD_MS) {
    sendStatus(lastStationSeq >= 0 ? static_cast<uint32_t>(lastStationSeq) : 0);
    lastStatusMs = now;
  }

  debugPrintStatus();
}
