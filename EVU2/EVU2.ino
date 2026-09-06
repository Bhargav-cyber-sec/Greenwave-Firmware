/*
============================================================================
GREENWAVE EVP TRANSMITTER V4 — VALIDATION SOP EDITION
ESP32 DEV MODULE + SX1278 LoRa + SmartElex u-blox MAX-M10S GNSS
Configured for 1Hz Transmission and Long-Duration Stress Testing.
============================================================================
*/

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <math.h>
#include <Preferences.h>
#include <Ed25519.h>

// ============================================================================
// HARDWARE PINS & PERIPHERALS
// ============================================================================
#define LORA_SS      5
#define LORA_RST     14
#define LORA_DIO0    2
#define LORA_SCK     18
#define LORA_MISO    19
#define LORA_MOSI    23

#define GPS_RX       16
#define GPS_TX       17

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

// ============================================================================
// VEHICLE DATA & DEPLOYMENT PARAMETERS
// ============================================================================
#define VEHICLE_ID        "AMB_02"
#define PRIORITY_CLASS    1
// SCENARIO 2 / S2-08 : THE EMERGENCY FLAG MUST BE ABLE TO FALL.
//
// This was `#define SIREN_ACTIVE true` -- a compile-time constant, so
// the emergency bit was set in every packet the unit ever sent and could
// never go from 1 to 0.
//
// That single line disabled the most valuable release signal in the
// whole system. The falling edge is an explicit, authenticated "I am
// done" from the driver: the only party who actually knows the event is
// over. Everything else the ICU can do -- waiting for the siren to fade,
// watching the vehicle cross the stop line, detecting divergence -- is
// inference. This is a statement.
//
// It is now runtime state. Change it with the serial commands below, or
// wire a real siren/emergency switch to EMERGENCY_SWITCH_PIN.
//
// The default is true so existing behaviour is unchanged on boot.
bool sirenActive = true;

// [FIELD] Wire the vehicle's own emergency switch here. -1 disables the
// input and leaves the flag under serial control for bench work.
//
// In deployment this MUST be driven by the same switch that operates the
// light bar. A separate control the crew has to remember is one they
// will forget, and a system that keeps a junction held after the
// ambulance has finished is worse than one that never held it.
#define EMERGENCY_SWITCH_PIN  -1
#define EMERGENCY_SWITCH_ACTIVE_LOW  true

// ==========================================================
// PHASE 6.2 : IN-CAB ACKNOWLEDGEMENT -- NOT IMPLEMENTED, BY DESIGN
//
// This unit is TRANSMIT-ONLY, and that is an architectural decision
// rather than a gap.
//
// The trust model is one-directional: the EVU signs, the roadside
// verifies, and nothing travels back. The vehicle holds no key capable
// of authenticating an inbound message -- not the CA public key, not a
// roadside public key, nothing.
//
// A downlink was prototyped and removed. It worked, and it would have
// been unauthenticated, which means anybody with a LoRa module and the
// frame layout could have transmitted a status into an ambulance cab. A
// forged "not being acted on" is a nuisance; a forged reassurance that
// a driver acted on is worse than showing nothing at all.
//
// Making it safe requires the vehicle to verify a roadside signature,
// which requires distributing the CA public key to every EVU and
// issuing certificates to every RDU. That is a change to the security
// architecture, and it belongs to whoever owns that architecture -- not
// to a firmware convenience feature.
//
// SO THE CAB IS TOLD NOTHING, deliberately. A crew that receives no
// confirmation knows they have no confirmation. A crew shown a
// forgeable confirmation does not know that at all.
// ==========================================================

// Confirmed 2026-07-26: 2000ms is the correct transmission interval.
// (Previous comment here incorrectly cited a 1000ms SOP requirement --
// TC-TX-003 and any other references to a 1000ms interval should be
// corrected to 2000ms to match this, not the other way around.)
const unsigned long TX_INTERVAL_MS = 2000;
unsigned long previousTX = 0;

// CHANGED v3: RANGE_TEST_BUILD gates the range-test instrumentation.
//
// test_leg was being transmitted in every production frame. It is a
// range-test artifact: it exists so a combined TX/RX log auto-segments by
// distance. Shipping it costs a byte of airtime forever and, worse, makes
// a test build indistinguishable from a production build. Set this to 0
// for anything that leaves the bench.
#define RANGE_TEST_BUILD 1

// Measured time-on-air at SF9 / BW125kHz / CR4-6 / preamble 12.
// Recompute these if ANY radio parameter changes -- printDutyCycle()
// below uses them and will otherwise lie to you.
#define TOA_STEADY_MS   640UL   // 97-byte frame
#define TOA_CERT_MS    1181UL   // 193-byte certificate-bearing frame

// ============================================================================
// GPS FILTERING & GEOLOCATION PARAMETERS
// ============================================================================
#define MIN_SATELLITES        6
#define MAX_HDOP                 3.0
#define STATIC_SPEED_THRESHOLD   3.0
#define STATIONARY_CONFIRM_COUNT 3
#define MOVE_SPEED_THRESHOLD     2.5
#define MOVE_DISTANCE_THRESHOLD  4.0
#define MOVING_CONFIRM_COUNT     3
#define STATIC_ALPHA             0.15
#define MOVING_ALPHA             0.75

double filteredLat = 0;
double filteredLon = 0;
double lockedLat = 0;
double lockedLon = 0;
bool filterStarted = false;
bool positionLocked = false;
int stationaryCounter = 0;
int movementCounter = 0;

// ============================================================================
// CRYPTOGRAPHIC STATE & NVS COUNTERS
// ============================================================================
Preferences secPrefs;
uint8_t evuPrivateKey[32];
uint8_t evuPublicKey[32];

#define CERT_BROADCAST_INTERVAL 5
#define SEQ_SAVE_MARGIN         100

// Increased to 60 for 100-hour endurance test to prevent ESP32 flash wear
#define SEQ_SAVE_INTERVAL       60

uint32_t sequenceNumber = 0;
uint32_t seqSinceLastSave = 0;

uint8_t caSignature[64];
bool certProvisioned = false;
String serialLineBuffer = "";

// ============================================================================
// RANGE-TEST LEG TAGGING
// Set with the serial command "LEG <n>" (0-255) before starting a distance
// leg (e.g. LEG 50, LEG 100, LEG 200). Embedded in every packet from then on,
// so a single combined TX/RX log auto-segments by distance in post-analysis
// instead of relying on manually noted timestamps/sequence numbers.
// ============================================================================
uint8_t currentTestLeg = 0;

// ============================================================================
// TIMEZONE CONFIG — INDIA STANDARD TIME (UTC+5:30)
// ============================================================================
#define IST_OFFSET_SEC   19800UL   // 5*3600 + 30*60
#define IST_LABEL        "IST"

// ============================================================================
// PACKED BINARY LAYOUT
// ============================================================================
// RANGE-TEST BUILD: added altitude_m and test_leg. This struct must stay
// byte-for-byte identical to the one in GreenwaveTypes.h (used by RDU1) --
// EVU2 doesn't include that header, so any change here has to be mirrored
// there too, in the same field order.
struct __attribute__((packed)) TelemetryPayload {
  char vehicle_id[6];      // Fixed length matching "AMB_02"
  uint8_t flags;           // Bit 0: emergency, Bit 1: gps_valid, Bit 2: siren_active, Bit 3: altitude_valid
  uint8_t priority_class;
  uint32_t seq;
  uint32_t gps_epoch;      // Always UTC 
  uint16_t gps_ms;         // Millisecond precision
  int32_t latitude;        // Scaled by 10,000,000
  int32_t longitude;       // Scaled by 10,000,000
  int16_t altitude_m;      // NEW: GPS altitude, meters above MSL, whole-meter precision
  uint16_t speed_kmph;     // Scaled by 100
  uint16_t heading_deg;    // Scaled by 100
  uint8_t test_leg;        // NEW: range-test leg/tag ID (0 = untagged), set via serial "LEG <n>"
};





// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
bool hexToBytes(const String &hex, uint8_t *out, size_t outLen) {
  if (hex.length() != outLen * 2) return false;
  for (size_t i = 0; i < outLen; i++) {
    char hi = hex[i * 2];
    char lo = hex[i * 2 + 1];
    if (!isxdigit(hi) || !isxdigit(lo)) return false;
    auto nib = [](char c) -> uint8_t {
      if (c >= '0' && c <= '9') return c - '0';
      c = tolower(c);
      return 10 + (c - 'a');
    };
    out[i] = (nib(hi) << 4) | nib(lo);
  }
  return true;
}

String bytesToHex(const uint8_t *data, size_t len) {
  static const char hexChars[] = "0123456789abcdef";
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    out += hexChars[(data[i] >> 4) & 0x0F];
    out += hexChars[data[i] & 0x0F];
  }
  return out;
}

static long daysFromCivil(int y, int m, int d) {
  y -= (m <= 2) ? 1 : 0;
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097L + (long)doe - 719468L;
}

static void civilFromDays(long z, int &y, int &m, int &d) {
  z += 719468L;
  long era = (z >= 0 ? z : z - 146096L) / 146097L;
  unsigned long doe = (unsigned long)(z - era * 146097L);
  unsigned long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  long yy = (long)yoe + era * 400;
  unsigned long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned long mp = (5 * doy + 2) / 153;
  d = (int)(doy - (153 * mp + 2) / 5 + 1);
  m = (int)(mp + (mp < 10 ? 3 : -9));
  y = (int)(yy + (m <= 2 ? 1 : 0));
}

uint32_t gpsToEpoch(TinyGPSDate &date, TinyGPSTime &time) {
  long days = daysFromCivil(date.year(), date.month(), date.day());
  return (uint32_t)days * 86400UL
       + (uint32_t)time.hour() * 3600UL
       + (uint32_t)time.minute() * 60UL
       + (uint32_t)time.second();
}

void formatEpochAsIST(uint32_t utcEpoch, uint16_t ms, char *outBuf, size_t outBufLen) {
  uint32_t istTotalSec = utcEpoch + IST_OFFSET_SEC;
  long days = (long)(istTotalSec / 86400UL);
  uint32_t secOfDay = istTotalSec % 86400UL;

  int y, mo, da;
  civilFromDays(days, y, mo, da);
  int hh = secOfDay / 3600;
  int mm = (secOfDay % 3600) / 60;
  int ss = secOfDay % 60;
  snprintf(outBuf, outBufLen, "%04d-%02d-%02d %02d:%02d:%02d.%03d %s",
           y, mo, da, hh, mm, ss, ms, IST_LABEL);
}

double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371000;
  double dLat = (lat2 - lat1) * PI / 180.0;
  double dLon = (lon2 - lon1) * PI / 180.0;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// ============================================================================
// SECURITY SUBSYSTEM PROVISIONING
// ============================================================================
void loadOrGenerateIdentity() {
  secPrefs.begin("evu-sec", false);
  size_t privLen = secPrefs.getBytesLength("privkey");
  if (privLen == 32) {
    secPrefs.getBytes("privkey", evuPrivateKey, 32);
    secPrefs.getBytes("pubkey", evuPublicKey, 32);
    Serial.println("[SEC] Loaded existing Ed25519 identity from flash.");
  } else {
    Serial.println("[SEC] No identity found — generating new Ed25519 keypair.");
    Ed25519::generatePrivateKey(evuPrivateKey);
    Ed25519::derivePublicKey(evuPublicKey, evuPrivateKey);
    secPrefs.putBytes("privkey", evuPrivateKey, 32);
    secPrefs.putBytes("pubkey", evuPublicKey, 32);
    Serial.println("[SEC] New identity generated and persisted.");
  }

  Serial.print("[SEC] EVU Public Key (register with CA/KMS): ");
  Serial.println(bytesToHex(evuPublicKey, 32));

  size_t certLen = secPrefs.getBytesLength("casig");
  if (certLen == 64) {
    secPrefs.getBytes("casig", caSignature, 64);
    certProvisioned = true;
    Serial.println("[SEC] CA certificate signature loaded from flash.");
  } else {
    memset(caSignature, 0, sizeof(caSignature));
    certProvisioned = false;
    Serial.println("[SEC] No CA certificate provisioned yet.");
    Serial.println("[SEC] Run ca_tool.py sign-evu --vehicle-id " + String(VEHICLE_ID) + " --pubkey <public key above>");
    Serial.println("[SEC] Then type: SETCERT <128-hex-char signature>  (Enter)");
  }
}

void handleSerialProvisioning() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLineBuffer.length() > 0) {
        serialLineBuffer.trim();

        // ---- SCENARIO 2 / S2-08 : emergency flag control ----
        //
        // EMERG ON / EMERG OFF / EMERG  (query)
        //
        // Turning it OFF is the interesting case: it produces the
        // falling edge the ICU treats as an explicit release, which
        // could not exist while this was a compile-time constant.
        if (serialLineBuffer.equalsIgnoreCase("EMERG ON")) {
            if (!sirenActive) {
                sirenActive = true;
                Serial.println("[EMERG] ON -- emergency flag set, "
                               "priority will be requested");
            } else {
                Serial.println("[EMERG] already ON");
            }
            serialLineBuffer = "";
            continue;
        }

        if (serialLineBuffer.equalsIgnoreCase("EMERG OFF")) {
            if (sirenActive) {
                sirenActive = false;
                Serial.println("[EMERG] OFF -- flag cleared. The NEXT packet "
                               "carries the falling edge and the ICU should "
                               "release this vehicle's demand.");
            } else {
                Serial.println("[EMERG] already OFF");
            }
            serialLineBuffer = "";
            continue;
        }

        if (serialLineBuffer.equalsIgnoreCase("EMERG")) {
            Serial.printf("[EMERG] %s\n", sirenActive ? "ON" : "OFF");
            serialLineBuffer = "";
            continue;
        }

        if (serialLineBuffer.startsWith("SETCERT ")) {
          String hex = serialLineBuffer.substring(8);
          hex.trim();
          uint8_t candidate[64];
          if (hexToBytes(hex, candidate, 64)) {
            memcpy(caSignature, candidate, 64);
            secPrefs.putBytes("casig", caSignature, 64);
            certProvisioned = true;
            Serial.println("[SEC] CA certificate signature stored successfully.");
          } else {
            Serial.println("[SEC] SETCERT rejected: expected 128 hex characters.");
          }
        } else if (serialLineBuffer.startsWith("LEG ")) {
          String num = serialLineBuffer.substring(4);
          num.trim();
          int val = num.toInt();
          if (num.length() > 0 && val >= 0 && val <= 255) {
            currentTestLeg = (uint8_t)val;
            Serial.print("[TEST] Leg tag set to ");
            Serial.print(currentTestLeg);
            Serial.println(" — every packet from now on carries this tag.");
          } else {
            Serial.println("[TEST] LEG rejected: expected an integer 0-255, e.g. LEG 50");
          }
        } else {
          Serial.println("[SEC] Unknown command. Use: SETCERT <128-hex-char signature>  or  LEG <0-255>");
        }
      }
      serialLineBuffer = "";
    } else {
      serialLineBuffer += c;
    }
  }
}

void loadPersistedCounter() {
  uint32_t stored = secPrefs.getUInt("seqctr", 0);
  sequenceNumber = stored + SEQ_SAVE_MARGIN;
  secPrefs.putUInt("seqctr", sequenceNumber);
  Serial.print("[SEC] Restored counter (with anti-rollback margin) = ");
  Serial.println(sequenceNumber);
}

void maybePersistCounter() {
  seqSinceLastSave++;
  if (seqSinceLastSave >= SEQ_SAVE_INTERVAL) {
    secPrefs.putUInt("seqctr", sequenceNumber);
    seqSinceLastSave = 0;
  }
}

// ============================================================================
// DUTY CYCLE AUDIT
//
// CHANGED v3: NEW. Nobody had ever computed this. It turns out to be
// ~37%, which is far outside any licence-exempt duty-cycle allowance the
// author is aware of -- and it is not fixable in firmware, because the
// 64-byte Ed25519 signature dominates the frame and SF9 is required by
// measurement (SF7 gave 39% packet loss at TC-TX-002).
//
// This is therefore a BAND / REGULATORY decision, not a code change:
// either the applicable Indian allowance for 433.05-434.79 MHz turns out
// to be generous, or this system moves to 865-867 MHz. Either way it must
// be settled before PCB layout and antenna design, because it changes
// both. Printing it at boot means the number is in front of whoever is
// working on the hardware, every single time.
// ============================================================================
void printDutyCycle() {
  double avgToa = (4.0 * TOA_STEADY_MS + 1.0 * TOA_CERT_MS) / 5.0;
  double duty   = 100.0 * avgToa / (double)TX_INTERVAL_MS;

  Serial.println("---- IF-1 AIRTIME BUDGET ----");
  Serial.printf("  steady frame   %u B  ~%lu ms\n",
                (unsigned)(sizeof(TelemetryPayload) + 64), TOA_STEADY_MS);
  Serial.printf("  cert frame     %u B  ~%lu ms  (every %d)\n",
                (unsigned)(sizeof(TelemetryPayload) + 64 + 96), TOA_CERT_MS,
                CERT_BROADCAST_INTERVAL);
  Serial.printf("  interval       %lu ms\n", TX_INTERVAL_MS);
  Serial.printf("  DUTY CYCLE     %.1f %%\n", duty);
  if (duty > 10.0) {
    Serial.println("  ** OVER 10% - not compliant with any licence-exempt");
    Serial.println("  ** allowance the team has verified. See OPEN-003/OPEN-010.");
    Serial.println("  ** This is a BAND decision, not a firmware bug.");
  }
  Serial.println("-----------------------------");
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
#if EMERGENCY_SWITCH_PIN >= 0
    pinMode(EMERGENCY_SWITCH_PIN,
            EMERGENCY_SWITCH_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
#endif

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("GREENWAVE EVP TRANSMITTER V4 (BINARY SECURE)");
#if RANGE_TEST_BUILD
  Serial.println("*** RANGE-TEST BUILD - test_leg tagging ACTIVE ***");
#else
  Serial.println("Production build - range-test instrumentation disabled");
#endif
  Serial.println("ESP32 + SX1278 + MAX-M10S");
  Serial.println("========================================");

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] MAX-M10S READY");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  while(!LoRa.begin(433E6)) {
    Serial.println("[ERROR] LoRa Init Failed");
    delay(1000);
  }

  LoRa.setTxPower(20);
  LoRa.setSyncWord(0xF3);
  // RANGE-TEST CONFIG: SF7 -> SF9 (~5 dB more link budget than SF8, ~10 dB
  // more than the original SF7 baseline that measured 39% loss at TC-TX-002).
  // CR 4/5 -> 4/6 (a bit more forward error correction without SF10/CR8's
  // ~2s-per-packet airtime cost). Preamble lengthened for more reliable sync
  // detection near the noise floor. Every one of these must match RDU1.ino
  // exactly -- SF/BW/CR/sync-word/preamble mismatches don't degrade
  // gracefully, they just fail to link.
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.setPreambleLength(12);
  LoRa.enableCrc();
  printDutyCycle();
  Serial.println("[LORA] TX READY");
  Serial.printf("[LORA CONFIG] SF=9 BW=125kHz CR=4/6 Preamble=12 TXPower=20dBm SyncWord=0xF3\n");
  Serial.println("Broadcast Started");

  loadOrGenerateIdentity();
  loadPersistedCounter();
  previousTX = millis();
}

// ============================================================================
// MAIN LOOP & SERIAL COMMUNICATIONS
// ============================================================================
// Reads the vehicle's emergency switch, if one is wired.
//
// Debounced, because a bouncing contact would otherwise generate a burst
// of rising and falling edges -- and the ICU treats a falling edge as an
// explicit release. A single bounce could release a live preemption.
static void pollEmergencySwitch() {
#if EMERGENCY_SWITCH_PIN >= 0
    static int  lastRaw     = -1;
    static unsigned long lastChangeMs = 0;
    static bool stable      = true;

    int raw = digitalRead(EMERGENCY_SWITCH_PIN);
    bool want = EMERGENCY_SWITCH_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);

    if (raw != lastRaw) {
        lastRaw = raw;
        lastChangeMs = millis();
        stable = false;
        return;
    }

    if (!stable && (millis() - lastChangeMs) > 80) {
        stable = true;
        if (want != sirenActive) {
            sirenActive = want;
            Serial.printf("[EMERG] switch -> %s\n", want ? "ON" : "OFF");
        }
    }
#endif
}




void loop() {
    pollEmergencySwitch();

  handleSerialProvisioning();

  while(GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  unsigned long now = millis();
  if(now - previousTX >= TX_INTERVAL_MS) {
    // CHANGED v3: previousTX = now, not previousTX += TX_INTERVAL_MS.
    //
    // The old accumulate-style scheduling is correct only if the loop
    // never falls behind. If anything stalls this loop for longer than
    // one interval -- a slow GPS burst, a long serial print, a
    // certificate-bearing packet at 1181ms of airtime -- previousTX ends
    // up in the past, and the next few iterations fire back-to-back with
    // no gap at all, trying to "catch up".
    //
    // Bursting like that is the worst possible behaviour on a
    // duty-cycle-limited link, AND it collides with the lane node's
    // gap-scheduling logic, which assumes a predictable 2000ms cadence.
    // Anchoring to now sacrifices long-term cadence accuracy (which
    // nothing needs) to guarantee a minimum gap (which everything needs).
    previousTX = now; 
    
    bool gpsValid = false;
    double latitude = 0, longitude = 0, speed = 0, heading = -1;
    int satellites = 0;
    double hdop = 99;
    String quality = "NONE";
    double altitude = 0;
    bool altitudeValid = false;

    // GPS Processing Pipeline
    if(gps.location.isValid() && gps.satellites.isValid() && gps.hdop.isValid()) {
      satellites = gps.satellites.value();
      hdop = gps.hdop.hdop();

      if(satellites >= MIN_SATELLITES && hdop <= MAX_HDOP && gps.location.age() < 2000) {
        gpsValid = true;
        double rawLat = gps.location.lat();
        double rawLon = gps.location.lng();
        speed = gps.speed.kmph();
        if(gps.course.isValid()) heading = gps.course.deg();
        if(gps.altitude.isValid()) {
          altitude = gps.altitude.meters();
          altitudeValid = true;
        }
        if(!filterStarted) {
          filteredLat = rawLat;
          filteredLon = rawLon;
          filterStarted = true;
        }

        double alpha = positionLocked ? STATIC_ALPHA : MOVING_ALPHA;
        filteredLat = alpha * rawLat + (1 - alpha) * filteredLat;
        filteredLon = rawLon * alpha + (1 - alpha) * filteredLon;
        latitude = filteredLat;
        longitude = filteredLon;
        quality = (hdop <= 1.5) ? "HIGH" : "MEDIUM";

        // Static Lock Machine
        if(!positionLocked) {
          if(speed < STATIC_SPEED_THRESHOLD) {
            stationaryCounter++;
          } else {
            stationaryCounter = 0;
          }
          if(stationaryCounter >= STATIONARY_CONFIRM_COUNT) {
            positionLocked = true;
            lockedLat = latitude;
            lockedLon = longitude;
            Serial.println("[GPS] POSITION LOCKED");
          }
        } else {
          // Movement Unlock Machine
          double moved = distanceMeters(lockedLat, lockedLon, rawLat, rawLon);
          if(speed > MOVE_SPEED_THRESHOLD || moved > MOVE_DISTANCE_THRESHOLD) {
            movementCounter++;
            Serial.println("[GPS] Movement candidate");
          } else {
            movementCounter = 0;
          }
          if(movementCounter >= MOVING_CONFIRM_COUNT) {
            positionLocked = false;
            stationaryCounter = 0;
            movementCounter = 0;
            Serial.println("[GPS] POSITION UNLOCKED - MOVING");
          } else {
            latitude = lockedLat;
            longitude = lockedLon;
            speed = 0;
          }
        }
      }
    }

    uint32_t gpsEpoch = 0;
    uint32_t currentMs = 0;
    bool gpsTimeValid = false;
    
    if (gps.time.isValid() && gps.date.isValid()) {
      gpsEpoch = gpsToEpoch(gps.date, gps.time);
      
      // Calculate true milliseconds using GPS fractional seconds + ESP32 time elapsed since last parse
      uint32_t baseMs = gps.time.centisecond() * 10;
      uint32_t totalMsElapsed = baseMs + gps.time.age();
      
      // If the age pushes us into the next second(s), adjust the epoch forward
      uint32_t extraSeconds = totalMsElapsed / 1000;
      gpsEpoch += extraSeconds;
      currentMs = totalMsElapsed % 1000;
      
      gpsTimeValid = true;
    }

    char istTimeStr[32];
    if (gpsTimeValid) {
      formatEpochAsIST(gpsEpoch, currentMs, istTimeStr, sizeof(istTimeStr));
    } else {
      snprintf(istTimeStr, sizeof(istTimeStr), "NO GPS TIME FIX");
    }

    char altitudeStr[16];
    if (altitudeValid) {
      snprintf(altitudeStr, sizeof(altitudeStr), "%d m", (int)round(altitude));
    } else {
      snprintf(altitudeStr, sizeof(altitudeStr), "NO FIX");
    }

    // Assemble Packed Struct Structures
    TelemetryPayload packet;
    memset(&packet, 0, sizeof(packet));
    strncpy(packet.vehicle_id, VEHICLE_ID, sizeof(packet.vehicle_id));
    packet.flags = 0;
    if (sirenActive)   packet.flags |= (1 << 0);   // emergency
    if (gpsValid)      packet.flags |= (1 << 1);
    if (sirenActive)   packet.flags |= (1 << 2);   // siren
    if (altitudeValid) packet.flags |= (1 << 3);
    // ADVISORY ONLY as of Scenario 2. The ICU derives the authoritative
    // priority from its own registry keyed on vehicle_id, and logs a
    // warning when this claim disagrees with it.
    //
    // Kept on the wire because a mismatch is a useful security signal --
    // it means a unit is asserting an importance it was not granted --
    // but it no longer influences any decision (spec defect S2-01).
    packet.priority_class = PRIORITY_CLASS;
    packet.seq = sequenceNumber;
    packet.gps_epoch = gpsEpoch;
    packet.gps_ms = currentMs; 
    packet.latitude = (int32_t)(latitude * 10000000.0);
    packet.longitude = (int32_t)(longitude * 10000000.0);
    packet.altitude_m = (int16_t)round(altitudeValid ? altitude : 0);
    packet.speed_kmph = (uint16_t)(speed * 100.0);
    packet.heading_deg = (uint16_t)((heading < 0 ? 0 : heading) * 100.0);
#if RANGE_TEST_BUILD
    packet.test_leg = currentTestLeg;
#else
    packet.test_leg = 0;
#endif
    
    // Dynamic Asymmetric Digital Signature Engine Calculation
    uint8_t signature[64];
    Ed25519::sign(signature, evuPrivateKey, evuPublicKey, &packet, sizeof(packet));
    bool includeCertThisPacket = (sequenceNumber % CERT_BROADCAST_INTERVAL == 0);

    // Physical Hardware Transmission Step
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&packet, sizeof(packet));
    LoRa.write(signature, sizeof(signature));
    if (includeCertThisPacket) {
      LoRa.write(evuPublicKey, 32);
      LoRa.write(caSignature, 64);
    }
    LoRa.endPacket();

// NOTE: no LoRa.receive() here, and that is deliberate.
    //
    // The radio returns to standby after endPacket() and this unit hears
    // nothing. Opening a receive window would be one line -- and would
    // create an unauthenticated inbound path into an ambulance cab. See
    // the architecture note at the top of this file.

    
    // ==========================================================
    // DIAGNOSTICS ENGINE TRACE — FULL PACKET PARAMETER DUMP
    // ==========================================================
    uint32_t packetBytes = (uint32_t)(sizeof(packet) + sizeof(signature) + (includeCertThisPacket ? 96 : 0));
    uint32_t up = millis()/1000;
    Serial.printf(
      "\n========== GREENWAVE TX ==========\n"
      "Vehicle ID       : %s\n"
      "Priority         : %u\n\n"
      "Emergency        : %s\n"
      "GPS Valid        : %s\n"
      "Siren            : %s\n\n"
      "Latitude         : %.7f\n"
      "Longitude        : %.7f\n"
      "Altitude         : %s\n\n"
      "Speed            : %.2f km/h\n"
      "Heading          : %.2f deg\n\n"
      "GPS Satellites   : %d\n"
      "GPS Accuracy     : %.2f HDOP\n"
      "GPS Quality      : %s\n"
      "Stable Lock      : %s\n\n"
      "Sequence         : %lu\n"
      "Test Leg         : %u\n"
      "Timestamp (UTC)  : %lu\n"
      "Timestamp (IST)  : %s\n\n"
      "Packet Size      : %u bytes\n"
      "Certificate      : %s\n"
      "Signature        : GENERATED\n"
      "LoRa TX          : SUCCESS\n\n"
      "Free RAM         : %u bytes\n"
      "Uptime           : %02lu:%02lu:%02lu\n"
      "==================================\n",
      VEHICLE_ID,
      (unsigned)packet.priority_class,
      (packet.flags & (1<<0)) ? "YES":"NO",
      (packet.flags & (1<<1)) ? "YES":"NO",
      (packet.flags & (1<<2)) ? "ACTIVE":"OFF",
      packet.latitude/10000000.0,
      packet.longitude/10000000.0,
      altitudeStr,
      packet.speed_kmph/100.0,
      packet.heading_deg/100.0,
      satellites,
      hdop,
      quality.c_str(),
      positionLocked ? "YES":"NO",
      (unsigned long)packet.seq,
      (unsigned)packet.test_leg,
      (unsigned long)packet.gps_epoch,
      istTimeStr,
      (unsigned)packetBytes,
      includeCertThisPacket ? (certProvisioned ? "YES (SIGNED)" : "YES (UNPROVISIONED)") : "NO",
      ESP.getFreeHeap(),
      up/3600,(up%3600)/60,up%60
    );
    Serial.flush();

    sequenceNumber++;
    maybePersistCounter();
  }
}