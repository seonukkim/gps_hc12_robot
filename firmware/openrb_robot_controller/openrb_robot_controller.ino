#include <Servo.h>
#include <TinyGPS++.h>
#include <math.h>

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

#define STRINGIFY_VALUE_IMPL(value) #value
#define STRINGIFY_VALUE(value) STRINGIFY_VALUE_IMPL(value)

#define ENABLE_GPS_TELEMETRY 1
#if MOTOR_PULSE_TEST_MODE || FIXED_WIRING_GPS_SERIAL2_DIAG || \
    FIXED_WIRING_GPS_SERIAL2_RC_AUTONOMY_DRYRUN || \
    FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT
#define HC12_LINK_ENABLED 0
#define GPS_SERIAL Serial2
#else
#define HC12_LINK_ENABLED 1
#define HC12_SERIAL Serial2
#define GPS_SERIAL Serial3
#endif

constexpr const char *FIRMWARE_ID = "openrb_robot_controller station-manual rc-arcade-manual-fwdneg 2026-05-30";
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
constexpr uint8_t MODE_CHANNEL_INDEX = 4;      // Station Manual/Auto switch: PPM CH5
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
    FIXED_WIRING_GPS_SERIAL2_SINGLE_WAYPOINT_EXPERIMENT
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
  mode = ppmChannels[MODE_CHANNEL_INDEX];
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
    case CONTROL_SOURCE_STATION_MANUAL: return "STATION_MANUAL";
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
  noInterrupts();
  steeringUs = ppmChannels[STEERING_CHANNEL_INDEX];
  throttleUs = ppmChannels[THROTTLE_CHANNEL_INDEX];
  modeUs = ppmChannels[MODE_CHANNEL_INDEX];
  ppmFrameMs = lastPpmFrameMs;
  interrupts();

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

  Serial.print(F("USBDBG mode="));
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
  Serial.print(F(" physical_a_role=throttle"));
  Serial.print(F(" physical_b_role=turn"));
  Serial.print(F(" wheel_to_physical_mapping=diff_to_throttle_turn"));
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
    sendAck(seq, "OK");
    return;
  }

  if (command == "MANUAL") {
    if (fieldCount != 5) {
      sendErr(seq, "MANUAL_PAYLOAD");
      return;
    }

    float steer = 0.0f;
    float throttle = 0.0f;
    bool deadman = false;
    bool estop = false;
    if (!parseFloatToken(fields[1], steer) || !parseFloatToken(fields[2], throttle) ||
        !parseBoolToken(fields[3], deadman) || !parseBoolToken(fields[4], estop)) {
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
    sendAck(seq, "OK");
    return;
  }

  if (command == "AUTO" || command == "START") {
    if (fieldCount != 3) {
      sendErr(seq, "AUTO_PAYLOAD");
      return;
    }

    if (!rcChannelsValid() || !rcAutoSwitchOn()) {
      clearAutoCommand();
      motorStop();
      sendErr(seq, "AUTO_REJECTED");
      return;
    }

    float left = 0.0f;
    float right = 0.0f;
    if (!parseFloatToken(fields[1], left) || !parseFloatToken(fields[2], right)) {
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
    sendAck(seq, "OK");
    return;
  }

  sendErr(seq, "UNKNOWN_CMD");
}

void processHC12Line(const String &line) {
  String type;
  String payload;
  long seq = 0;
  if (!decodeFrame(line, type, seq, payload)) {
    sendErr(0, "BAD_FRAME");
    return;
  }

  lastStationFrameMs = millis();
  lastStationSeq = seq;

  if (type == "HB") {
    sendAck(seq, "OK");
  } else if (type == "CMD") {
    handleCommand(seq, payload);
  } else {
    sendErr(seq, "UNSUPPORTED");
  }
}

void setup() {
  Serial.begin(USB_BAUD);
#if HC12_LINK_ENABLED
  HC12_SERIAL.begin(HC12_BAUD);
#endif
#if ENABLE_GPS_TELEMETRY && !MOTOR_PULSE_TEST_MODE
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
  Serial.println("RC mode input uses receiver PPM CH5; PPM CH7 is reserved/unused.");
  Serial.println("CH5 high enters AUTO_READY only; drive stays STOP until explicit AUTO.");
  Serial.println("Station manual accepts CMD,MANUAL only when fresh frames and deadman=1.");
}

void loop() {
#if ENABLE_GPS_TELEMETRY && !MOTOR_PULSE_TEST_MODE
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
  bool rcValid = rcChannelsValid(steeringUs, throttleUs, modeUs);
  bool autoSwitchOn = rcAutoSwitchOn(modeUs);
  bool stationManualFresh = stationManualValid();
  bool stationManualActive = stationManualFresh && stationManual.deadman && !stationEstop;
  bool rcManualActive = rcValid && !autoSwitchOn;

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
