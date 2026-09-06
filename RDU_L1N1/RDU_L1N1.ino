/***********************************************************************
 * GREENWAVE EVP
 * Lane Node V3.8  --  wire protocol v4
 *
 * Secure V2I + Smart Road Geofence + Acoustic AI
 *
 * v3.9 CHANGE (roadside): audio capture is now a continuous producer task
 * feeding a ring buffer, because inference (~600 ms) previously blocked all
 * I2S reads and left a real-time hole between windows. Scoring is
 * confidence-weighted with hysteresis and a minimum window count, and every
 * window is logged as [AIW] for site tuning. Measured tensor arena: 56204 B.
 *
 * v3.8 CHANGE: the acoustic path no longer uses Edge Impulse. The Phase 6
 * INT8 DS-CNN runs directly on TFLite Micro, fed by a C port of the frozen
 * Phase 5 log-mel front end (gw_frontend.c). Edge Impulse's MFE block could
 * not reproduce that pipeline: no per-window power_to_db(ref=max), no global
 * mean/std normalisation, and a power-of-two FFT where ours is 400 points.
 *
 * Front end validated against the Phase 5 Python reference:
 *   18/20 golden vectors byte-identical at the INT8 tensor
 *   20/20 golden vectors classify identically through the real model
 * See 05_Validation in the Phase 6 release package.
 ***********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/i2s_std.h"
#include "esp_heap_caps.h"

#include <Ed25519.h>

// CHANGED v3.6: declare this build's role BEFORE including GreenwaveCrypto.h.
// RESOLVED IN v4: GreenwaveCrypto.h now #errors if this is not defined, and
// role-gates all key material. A lane-node image contains ONLY this node's
// own X25519 private key plus the ICU's PUBLIC key -- no other node's
// secret, and not the ICU's private key. The v3 GW_NODE_KEYS table (all six
// symmetric keys in every image) is gone.
#define GW_ROLE_LANE_NODE

#include "GreenwaveTypes.h"
#include "GreenwaveCrypto.h"
// PATH B: the Edge Impulse inferencing library is gone. Its DSP block cannot
// reproduce the frozen Phase 5 front end -- no per-window power_to_db(ref=max),
// no global mean/std normalisation, and its FFT length must be a power of two
// while ours is 400. We now run the Phase 5 pipeline ourselves and feed the
// Phase 6 INT8 model directly through TFLite Micro.
//
// Validation: gw_frontend_run() reproduces the Phase 5 Python pipeline on
// 18/20 golden vectors byte-identically, and all 20 golden vectors classify
// identically through the real model (19 of 20 to the bit).
#include "gw_frontend.h"
#include "gw_model.h"

/***********************************************************************
 * NODE CONFIGURATION
 ***********************************************************************/
// CHANGED v3: numeric, not a string. The ICU now actually checks this
// (it never did before), and 2 bytes on the wire beats 8.
#define INTERSECTION_ID 1
#define LANE_ID  GW_KEY_LANE_ID
#define NODE_ID  GW_KEY_NODE_ID

// v6: GreenwaveKeys.h is the SINGLE SOURCE OF TRUTH for node identity.
// It holds the private key, so it is the one field that cannot be wrong
// without the ICU noticing -- a mismatched key fails authentication
// immediately and loudly. A separately declared LANE_ID could be wrong
// silently, and the node would sign as one identity while announcing
// another: every frame rejected as BAD TAG, nothing in any log saying why.
//
// These asserts catch a key file that is missing, stale, or from another
// project BEFORE the image is built, rather than at a junction.
static_assert(GW_KEY_LANE_ID >= 1 && GW_KEY_LANE_ID <= 3,
              "GW_KEY_LANE_ID out of range - wrong or missing key file?");
static_assert(GW_KEY_NODE_ID >= 1 && GW_KEY_NODE_ID <= 2,
              "GW_KEY_NODE_ID out of range - wrong or missing key file?");
static_assert(GW_KEY_INTERSECTION == INTERSECTION_ID,
              "Key file intersection does not match INTERSECTION_ID");

// CHANGED v3.6: consolidated. There were TWO definitions of this node's own
// position in this file and they disagreed:
//
//   LANE_NODE_LATITUDE / LANE_NODE_LONGITUDE = 0.0, 0.0  -- never read by
//       anything, gated behind LANE_NODE_POSITION_KNOWN
//   NODE_LATITUDE / NODE_LONGITUDE = 13.0415, 77.6928    -- used by
//       calculateETA(), i.e. the one that actually affected behaviour
//
// Two sources of truth for one physical fact, one of them a point in the
// Atlantic, is the same shape as the phase-anchor bug: both were "correct",
// they just meant different things. NODE_LATITUDE/NODE_LONGITUDE wins -- it
// has real coordinates and real callers. LANE_NODE_POSITION_KNOWN survives
// as what it should always have been: an assertion that those coordinates
// were SURVEYED rather than read off a map. The geofence below refuses to
// run on unsurveyed coordinates.
//
// The two byte-identical haversine implementations (calculateDistanceMeters
// and calculateDistance) are collapsed into one. The float-returning copy
// had no callers.
#define LANE_NODE_POSITION_KNOWN false   // set true ONLY once surveyed

// SOP-EVP-LN-001: geofence is the ONLY check bypassed for validation
// testing. Everything else -- signature verification, anti-replay,
// GPS-validity check, TinyML siren detection -- stays fully live.
// This is checked and logged on every packet (not just a boot banner),
// so the workbook/report can show exactly when it applied.
#define GEOFENCE_BYPASSED_FOR_SOP true

#define NODE_LATITUDE   13.041500
#define NODE_LONGITUDE  77.692800

/***********************************************************************
 * v5 / PHASE 4 : SURVEYED GEOMETRY
 *
 * THE MOST IMPORTANT PER-NODE SETTING IN THIS FILE.
 *
 * This node's distance from the stop line, in metres, measured along the
 * road -- not straight-line, and not read off a map.
 *
 * THIS NODE DOES NOT KNOW WHETHER IT IS "FAR" OR "NEAR", AND IS NEVER
 * TOLD. It knows only its own distance. It reports that distance in
 * every heartbeat, and the ICU sorts each approach's two nodes at
 * runtime: larger distance = RDU_FAR, smaller = RDU_NEAR.
 *
 * WHY IT IS DONE THAT WAY
 *
 * v4 had the ICU infer position from NODE_ID, and NODE_ID is an
 * install-order label with no physical meaning. Spec Scenario 1 defect
 * S1-01 is exactly this: the v1.0 logic document had the geometry
 * INVERTED, and the consequence is silent and symmetric -- the system
 * preempts for traffic LEAVING the junction and ignores traffic
 * arriving. No error is raised. Nothing in any log looks wrong. It
 * simply behaves backwards, forever.
 *
 * Sorting by a surveyed distance the node reports itself makes that
 * failure impossible to express: swap two enclosures and the ICU
 * follows the swap, because the distance travels with the box.
 *
 * DYNAMIC PER JUNCTION. Every intersection has a different layout, so
 * this number belongs in per-node configuration and never in shared
 * firmware. Two nodes on one approach MUST NOT have the same value --
 * the ICU cannot order them and will mark the approach MISCONFIGURED
 * and disable automatic action on it, which is the correct response to
 * geometry it was not given.
 *
 * [FIELD] Set per node at install. Bench defaults for this deployment
 * are 250 m for the outer node and 150 m for the inner one.
 ***********************************************************************/
// FALLBACK ONLY. The live value comes from NVS -- set it over serial
// with `dist <metres>`, which survives reflashing.
//
// This was a compile-time constant, which meant moving a node to
// another junction required an edit-compile-flash cycle, and gave four
// chances per site to flash the wrong number without noticing.
//
// That failure is silent: this value sets the acoustic correlation
// window, so a wrong distance produces systematically wrong direction
// inference with nothing in any log to show for it.
#define NODE_DISTANCE_FALLBACK_M  250

// Set true ONLY when the distance above was actually measured on site.
//
// A separate assertion from the value itself, on purpose. A plausible
// number that nobody surveyed is more dangerous than an obviously
// missing one: it passes every range check and quietly becomes the
// basis for direction inference. False here makes the ICU treat this
// node as MISCONFIGURED rather than believing an estimate.
// FALLBACK ONLY -- NVS overrides this. `dist <m>` sets both the value
// and the surveyed flag, because entering a measurement IS the act of
// surveying it.
#define NODE_DISTANCE_SURVEYED_FALLBACK  true

// Which approach this node watches. Cross-checked by the ICU against
// hdr.lane_id; a mismatch is MISCONFIGURED, never a guess.
#define NODE_APPROACH_ID  LANE_ID

/***********************************************************************
 * MICROPHONE SELF-TEST BOUNDS
 *
 * A dead, unplugged, or taped-over microphone reports a noise floor at
 * or near ZERO. Real roadside ambient never does.
 *
 * This is the fault v4 had no way to represent (spec defect S1-04). Such
 * a node passes every other health check -- it boots, it heartbeats, its
 * radio works -- and simply never detects anything. It presents as a
 * healthy node on a quiet road, indefinitely.
 *
 * It is also the fault a two-node design is least able to notice on its
 * own: the partner node keeps working, so the approach looks merely
 * quiet rather than half-blind.
 *
 * An implausibly HIGH floor matters too. It means the classifier is
 * working against a noise wall, and its confidence numbers no longer
 * mean what they meant when the thresholds were tuned.
 *
 * [FIELD] Both bounds should be re-measured on site. The low bound in
 * particular is the one that catches a failed microphone, and setting it
 * from a bench measurement in a quiet room will make it too low to fire.
 ***********************************************************************/
#define MIC_NOISE_FLOOR_MIN_RMS  0.00030f   // below this: mic is not hearing
#define MIC_NOISE_FLOOR_MAX_RMS  0.20000f   // above this: saturated / very noisy

/***********************************************************************
 * GREENWAVE SMART ROAD GEOFENCE ENGINE
 ***********************************************************************/
GPSPoint roadPath[] = {
    {13.042500,77.693400},
    {13.042000,77.693100},
    {13.041500,77.692800}
};
int totalRoadPoints = sizeof(roadPath) / sizeof(roadPath[0]);

#define ROAD_WIDTH_METERS      15
#define HEADING_LIMIT_DEG      45
#define PREPARE_DISTANCE       500
#define PRIORITY_DISTANCE      200
#define HOLD_DISTANCE          50

double smoothLat = 0;
double smoothLon = 0;
bool gpsFilterStarted=false;

void smoothGPS(double &lat, double &lon) {
    float alpha=0.2;
    if(!gpsFilterStarted) {
        smoothLat=lat;
        smoothLon=lon;
        gpsFilterStarted=true;
    }
    smoothLat = alpha*lat + (1-alpha)*smoothLat;
    smoothLon = alpha*lon + (1-alpha)*smoothLon;
    lat=smoothLat;
    lon=smoothLon;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double R=6371000;
    double dLat = radians(lat2-lat1);
    double dLon = radians(lon2-lon1);
    double a = sin(dLat/2)*sin(dLat/2) + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R*c;
}

float calculateETA(double lat, double lon, float speed) {
    double distance = calculateDistance(lat, lon, NODE_LATITUDE, NODE_LONGITUDE);
    if(speed < 2) return 999;
    return distance / (speed/3.6);
}

bool priorityLocked=false;

// Initial bearing from (lat1,lon1) to (lat2,lon2), degrees clockwise from
// true north. Used to test whether a vehicle is pointed AT this node rather
// than merely near it.
double bearingDegrees(double lat1, double lon1, double lat2, double lon2) {
    double dLon = radians(lon2 - lon1);
    double y = sin(dLon) * cos(radians(lat2));
    double x = cos(radians(lat1)) * sin(radians(lat2)) -
               sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
    double b = degrees(atan2(y, x));
    return (b < 0) ? b + 360.0 : b;
}

// Smallest absolute difference between two bearings, 0..180 degrees.
// Inputs are normalised first: heading arrives from the EVU as
// heading_deg/100 and a corrupt or unfixed field can present values well
// outside 0..360. Without normalisation this returns a negative "delta",
// which then compares as <= HEADING_LIMIT_DEG and silently PASSES the
// heading check on garbage input. Fail closed on nonsense, not open.
double bearingDelta(double a, double b) {
    a = fmod(fmod(a, 360.0) + 360.0, 360.0);
    b = fmod(fmod(b, 360.0) + 360.0, 360.0);
    double d = fabs(a - b);
    if (d > 180.0) d = 360.0 - d;
    return d;
}

// Perpendicular distance in metres from point P to segment AB, using a local
// equirectangular projection. Valid because every distance involved here is
// under a kilometre, where the error against a true geodesic is centimetres.
double distanceToSegmentMeters(double pLat, double pLon,
                               double aLat, double aLon,
                               double bLat, double bLon) {
    const double MPD = 111320.0;                  // metres per degree latitude
    double cosLat = cos(radians(pLat));
    double px = (pLon - aLon) * MPD * cosLat, py = (pLat - aLat) * MPD;
    double bx = (bLon - aLon) * MPD * cosLat, by = (bLat - aLat) * MPD;

    double segLenSq = bx*bx + by*by;
    if (segLenSq < 1e-6) return sqrt(px*px + py*py);   // degenerate segment

    double t = (px*bx + py*by) / segLenSq;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    double cx = px - t*bx, cy = py - t*by;
    return sqrt(cx*cx + cy*cy);
}

/***********************************************************************
 * CHANGED v3.6: this function is now IMPLEMENTED.
 *
 * It previously read, in its entirety:
 *
 *     Serial.println("[SMART GEOFENCE - BYPASSED FOR SOP VALIDATION]");
 *     return true;
 *
 * That is worse than having no geofence at all. Every caller, every log
 * line, the EVF_GEOFENCE_PASS flag on the wire, and the ICU's own log
 * output all said a geofence decision had been made. None had been. A
 * function that pretends to decide is a lie the whole system repeats
 * downstream -- and `roadPath`, ROAD_WIDTH_METERS, HEADING_LIMIT_DEG and
 * PREPARE_DISTANCE all existed, unused, immediately above it.
 *
 * Three independent conditions, all of which must hold:
 *
 *   1. CORRIDOR   - the vehicle is within half of ROAD_WIDTH_METERS of the
 *                   surveyed road polyline. Rejects a vehicle on a parallel
 *                   street, which GPS alone cannot distinguish by distance.
 *   2. RANGE      - within PREPARE_DISTANCE of this node. Rejects an
 *                   ambulance kilometres away that LoRa can still hear.
 *   3. HEADING    - travelling toward this node within HEADING_LIMIT_DEG.
 *                   Rejects one that has already passed and is receding,
 *                   which is the single most common false preemption in
 *                   commercial EVP.
 *
 * Heading is only checked above MOVE-ish speed, because GNSS course is
 * meaningless at a standstill and would otherwise reject a stopped
 * ambulance -- exactly the vehicle most in need of a green.
 *
 * NOT ENABLED BY DEFAULT. It refuses to run unless the node's own position
 * has been surveyed (LANE_NODE_POSITION_KNOWN), because a geofence run
 * against guessed coordinates rejects real ambulances, and a security
 * control that produces false negatives on an emergency system is worse
 * than an absent one. Failing OPEN here is deliberate and is logged.
 *
 * REVIEW THIS BEFORE TRUSTING IT: it is new logic, it has never executed
 * on hardware, and its thresholds have never been validated against a real
 * road. Bring it up with GEOFENCE_BYPASSED_FOR_SOP still true and compare
 * its verdicts against reality in the log before letting it gate anything.
 ***********************************************************************/
bool smartGeofenceDecision(double lat, double lon, float speed, float heading) {
    if (!LANE_NODE_POSITION_KNOWN) {
        Serial.println("[GEOFENCE] SKIPPED - node position not surveyed "
                       "(LANE_NODE_POSITION_KNOWN=false). Failing OPEN.");
        return true;
    }

    // 1. Corridor
    double minCorridor = 1e9;
    for (int i = 0; i < totalRoadPoints - 1; i++) {
        double d = distanceToSegmentMeters(lat, lon,
                                           roadPath[i].lat,   roadPath[i].lon,
                                           roadPath[i+1].lat, roadPath[i+1].lon);
        if (d < minCorridor) minCorridor = d;
    }
    bool corridorOK = (minCorridor <= (ROAD_WIDTH_METERS / 2.0));

    // 2. Range
    double range = calculateDistance(lat, lon, NODE_LATITUDE, NODE_LONGITUDE);
    bool rangeOK = (range <= PREPARE_DISTANCE);

    // 3. Heading -- only meaningful when actually moving.
    double toNode = bearingDegrees(lat, lon, NODE_LATITUDE, NODE_LONGITUDE);
    double hDelta = bearingDelta(heading, toNode);
    bool movingEnough = (speed >= 5.0f);
    bool headingOK = (!movingEnough) || (hDelta <= HEADING_LIMIT_DEG);

    bool accepted = corridorOK && rangeOK && headingOK;

    Serial.printf("[GEOFENCE] corridor=%.1fm(%s) range=%.1fm(%s) "
                  "hdgDelta=%.0fdeg(%s%s) -> %s\n",
                  minCorridor, corridorOK ? "OK" : "FAIL",
                  range,       rangeOK    ? "OK" : "FAIL",
                  hDelta,      headingOK  ? "OK" : "FAIL",
                  movingEnough ? "" : ",stationary-exempt",
                  accepted ? "ACCEPT" : "REJECT");

    return accepted;
}

/***********************************************************************
 * SECURITY LAYER & ROOT CA
 ***********************************************************************/
const uint8_t ROOT_CA_PUBLIC_KEY[32] = {
  0xff, 0x73, 0xff, 0xd1, 0xda, 0x93, 0x6c, 0xb8, 0x50, 0x79, 0x9b, 0x92, 0x20, 0x93, 0xdd, 0x4c, 0xb9, 0x90, 0x3d, 0xf8, 0x45, 0x31, 0x65, 0xd0, 0x2e, 0x07, 0xe9, 0x7c, 0x46, 0x22, 0x5e, 0x4e
};

VehicleTrust trustTable[MAX_TRACKED_VEHICLES];
RetroPacket retroBuffer[RETRO_BUFFER_SIZE];

String bytesToHex(const uint8_t *data, size_t len) {
    static const char chars[]="0123456789abcdef";
    String out;
    for(size_t i=0;i<len;i++) {
        out += chars[(data[i]>>4)&0x0F];
        out += chars[data[i]&0x0F];
    }
    return out;
}

bool hexToBytes(String hex, uint8_t *out, size_t len) {
    if(hex.length()!=len*2) return false;
    for(size_t i=0;i<len;i++) {
        String part= hex.substring(i*2, i*2+2);
        out[i]= strtol(part.c_str(), NULL, 16);
    }
    return true;
}

// CHANGED v3.
//
// The old version, when the 8-slot table was full, did `empty = 0` and
// wiped slot 0 -- silently evicting whatever trusted vehicle happened to
// live there. Since findVehicle() is called BEFORE any signature is
// checked (we need the entry to hold the key we are about to check
// against), an attacker could transmit 8 unsigned frames with 8 made-up
// vehicle IDs and evict the real ambulance's verified key. The ambulance
// then falls back to WAITING_CERT and no preemption happens.
//
// Now: never evict a slot whose key is valid in favour of an unverified
// newcomer. Prefer an empty slot, then the oldest UNVERIFIED slot, and if
// every slot holds a verified vehicle, refuse (return NULL) rather than
// destroy trust state.
VehicleTrust* findVehicle(const char *id) {
    int empty = -1;
    int oldestUnverified = -1;
    unsigned long oldestSeen = 0xFFFFFFFFUL;

    for(int i=0; i<MAX_TRACKED_VEHICLES; i++) {
        if(trustTable[i].used && strncmp(id, trustTable[i].vehicleID, 8) == 0)
            return &trustTable[i];
        if(!trustTable[i].used && empty == -1) empty = i;
        if(trustTable[i].used && !trustTable[i].keyValid &&
           trustTable[i].lastSeen <= oldestSeen) {
            oldestSeen = trustTable[i].lastSeen;
            oldestUnverified = i;
        }
    }

    int slot = (empty != -1) ? empty : oldestUnverified;
    if(slot == -1) {
        Serial.println("[SECURITY] Trust table full of VERIFIED vehicles - refusing to evict");
        return NULL;   // callers must handle this
    }

    memset(&trustTable[slot], 0, sizeof(VehicleTrust));
    strncpy(trustTable[slot].vehicleID, id, 7);
    trustTable[slot].vehicleID[7] = '\0';
    trustTable[slot].used = true;
    trustTable[slot].lastSeen = millis();
    return &trustTable[slot];
}

bool verifyCertificate(const char *vehicleID, const uint8_t *pub, const uint8_t *signature, VehicleTrust *entry) {
    String message = String(vehicleID) + ":" + bytesToHex(pub, 32);
    bool result = Ed25519::verify(signature, ROOT_CA_PUBLIC_KEY, message.c_str(), message.length());
    if(result) {
        memcpy(entry->publicKey, pub, 32);
        entry->keyValid=true;
        Serial.println("[CERT VERIFIED]");
    } else {
        Serial.println("[CERT] REJECTED - INVALID CA SIGNATURE");
    }
    return result;
}

bool verifySignature(const uint8_t *data, size_t dataLen, const uint8_t *sig, uint8_t *publicKey) {
    return Ed25519::verify(sig, publicKey, data, dataLen);
}

/***********************************************************************
 * TIMEZONE CONFIG — INDIA STANDARD TIME (UTC+5:30)
 ***********************************************************************/
#define IST_OFFSET_SEC   19800UL
#define IST_LABEL        "IST"

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

void formatEpochAsIST(uint32_t utcEpoch, uint16_t ms, char *outBuf, size_t outBufLen) {
  uint32_t istTotalSec = utcEpoch + IST_OFFSET_SEC;
  long days = (long)(istTotalSec / 86400UL);
  uint32_t secOfDay = istTotalSec % 86400UL;
  int y, mo, da;
  civilFromDays(days, y, mo, da);
  int hh = secOfDay / 3600;
  int mm = (secOfDay % 3600) / 60;
  int ss = secOfDay % 60;
  snprintf(outBuf, outBufLen, "%04d-%02d-%02d %02d:%02d:%02d.%03d %s", y, mo, da, hh, mm, ss, ms, IST_LABEL);
}

/***********************************************************************
 * GLOBAL VARIABLES
 ***********************************************************************/
// CHANGED v3: replaced the plain RAM counter with a NVS-backed epoch
// counter. The old sequenceCounter restarted at 0 on every reboot, so
// after a power blip every previously-recorded frame became replayable.
GwCounter gwCounter;

// v4 (SOP 3.2/3.4). Holds this node's derived session key and drives the
// 60 s re-key. Declared extern in GreenwaveCrypto.h.
GwRduSession gwSession;
portMUX_TYPE seqMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t loraMutex;
SemaphoreHandle_t serialMutex;
QueueHandle_t eventQueue;

bool loraHealthy=false;
bool micHealthy=false;
double vehicleLatitude=0;
double vehicleLongitude=0;
float vehicleAltitude=0;
bool vehicleAltitudeValid=false;
float vehicleSpeed=0;
float vehicleHeading=0;
float vehicleDistance=0;
float vehicleETA=0;
String motionState="UNKNOWN";
float batteryVoltage=0;
unsigned long lastEmergency=0;
uint16_t currentVehicleMs=0; // NEW: global variable to hold millisecond



void retryBufferedPackets(const char *vehicleID);

// v6: forward declaration. gwFallingEdgeNow is defined below with the
// transmit-scheduling constants, but queueFrame() (above it in the file)
// must capture it into the OutFrame. Declared here rather than moved, so
// the constant stays next to the comment block that explains it.
extern bool gwFallingEdgeNow;

/***********************************************************************
 * LORA CONFIGURATION & BUFFERING
 ***********************************************************************/
#define AMBULANCE_FREQ 433E6
// TIER-A ISOLATION FIX: moved from 433.5E6 to 434.5E6 to widen the guard
// band against AMBULANCE_FREQ. MUST match ICU.ino's CENTRAL_FREQ exactly.
#define CENTRAL_FREQ   434.5E6

// TIER-A ISOLATION FIX: gap-aware ICU transmit scheduling.
// EVU2.ino transmits every 2000ms. While radiateToICU() is retuned off
// AMBULANCE_FREQ, an EVU packet physically cannot be received (known
// single-radio tradeoff, see radiateToICU() below). Rather than
// transmitting to the ICU at any arbitrary moment, traffic is deferred to
// land in the gap between predicted EVU arrivals.
//
// CHANGED v3.6: this comment block used to describe the OLD edge-based
// scheduler and cited "~150ms worst-case EVU airtime". That figure is
// wrong by more than 4x -- a steady frame is 640ms and a cert frame is
// 1181ms -- and it is the exact wrong number that produced the 32.6%
// reception bug in TC-LN-001 v3. It sat directly above the corrected code
// for three revisions. Deleted rather than adjusted, because a comment
// that has already misled once has no credit left.
//
// ICU_TX_GUARD_MS absorbs jitter and clock drift between the EVU's
// free-running millis() and this node's. The EVU is modelled as an
// OCCUPANCY INTERVAL, not an instant -- see icuTransmitWindowOpen().
#define EVU_TX_INTERVAL_MS   2000UL

// CHANGED v3. The old code guarded +/-300ms around a modelled *instant*.
// The EVU does not transmit at an instant -- at SF9/CR4-6 a 97-byte frame
// occupies 640ms, and a certificate-bearing frame occupies 1181ms, out of
// every 2000ms. So the old guard was smaller than the thing it was
// guarding against, and no safe window existed at all. We now model the
// EVU as an OCCUPANCY INTERVAL and guard the whole of it plus jitter.
#define EVU_AIRTIME_MS       640UL    // steady-state frame, SF9/BW125/CR4-6
#define EVU_CERT_AIRTIME_MS  1181UL   // certificate-bearing frame
#define ICU_TX_GUARD_MS      200UL    // clock-drift / jitter margin

// Longest IF-2 frame we might send, so we can check it actually fits in
// the gap before we start transmitting into it.
// v5: must be >= the LARGEST frame this node sends to the ICU, which is
// now the 58-byte LoRaEventFrame at 443 ms. The old 400 was already
// short for the 56-byte frame and is now short by 75 ms.
// Must be >= the largest frame sent to the ICU. SF7 (FIX 3).
#define ICU_TX_MAX_AIRTIME_MS 165UL

// Must match CERT_BROADCAST_INTERVAL in EVU2.ino.
#define CERT_CADENCE 5

// CHANGED v3.1: the EVU sends a certificate on every 5th frame, and that
// frame occupies 1181ms instead of 640ms. Tracking the cadence lets the
// guard widen only for the frames that need it, instead of using the
// worst case always (which would leave a ~19ms window, i.e. none).
volatile unsigned long lastEVUTxStartMillis = 0;   // START of frame, not end
volatile uint8_t       evuFramesSinceCert   = 0;

// v6: the EVU sequence number belonging to the frame that set
// lastEVUTxStartMillis.
//
// lastEVUTxStartMillis gives all six nodes a common time ANCHOR. It does
// not give them a common PERIOD LABEL: a node that missed the last two
// frames counts periods from a different anchor than its neighbour, and
// the two would silently choose different slots. The EVU's sequence
// number is the only period label every node agrees on, because it comes
// from the frame itself rather than from a local clock.
//
// Set only on live frames, never on retro replays -- same rule, and same
// reason, as the anchor it accompanies.
volatile uint32_t lastEVUSeq      = 0;
volatile bool     lastEVUSeqValid = false;

// Timestamp of the most recent live (non-retro-replay) EVU packet
// reception, used to predict the next expected arrival. 0 = no EVU seen
// yet, in which case scheduling doesn't block (nothing to avoid yet).
volatile unsigned long lastEVURxMillis = 0;

// Returns true if we are currently at least ICU_TX_GUARD_MS away (in
// either direction) from the nearest predicted EVU transmission edge,
// i.e. it's safe to retune off AMBULANCE_FREQ for an ICU transmission.
// Returns true if a transmission of txAirtimeMs starting NOW would finish
// before the EVU's next predicted transmission begins.
//
// Timeline within one 2000ms EVU period:
//   [0 .. 640ms]      EVU is on air -- we must be silent and listening
//   [640 .. 1800ms]   free gap (1160ms usable after the 200ms guard)
//   [1800 .. 2000ms]  guard before the next EVU frame
// Returns true if a transmission of txAirtimeMs starting NOW would finish
// before the EVU's next frame begins.
//
// Timeline within one 2000ms EVU period, measured from TRANSMIT START:
//   [0 .. 640ms]      EVU steady frame on air  (1181ms if cert-bearing)
//   [+200ms guard]
//   [840 .. 1400ms]   usable gap, 560ms wide
//   [1400 .. 2000ms]  guard before the next EVU frame
//
// CHANGED v3.1 -- THE ROOT CAUSE OF TC-LN-001's 32.6% RECEPTION RATE.
//
// This used to anchor on lastEVURxMillis, which is set when a packet
// finishes ARRIVING. But the arithmetic below assumes the anchor marks
// when the EVU STARTED transmitting. At SF9 a frame takes 640ms to
// arrive, so the anchor was 640ms late, and the "safe" window
// [840..1400ms] was really [1480..2040ms] measured from the true start.
// The next EVU frame starts at 2000ms.
//
// So every transmit window overlapped the start of the next EVU frame by
// 40ms, every single cycle. Measured result: 32.6% packet reception at
// -29 dBm, and not one instance of two consecutive beacons being
// received across the entire capture.
//
// Both the old code and the arithmetic were "correct". The bug was that
// the VARIABLE MEANT SOMETHING DIFFERENT from what the function assumed.
// Name timestamps for the event they mark.
bool icuTransmitWindowOpen(unsigned long txAirtimeMs) {
    if (lastEVUTxStartMillis == 0) return true;   // no EVU seen yet

    unsigned long elapsed = (millis() - lastEVUTxStartMillis) % EVU_TX_INTERVAL_MS;

    // Will the NEXT frame be a cert frame? Certs come every CERT_CADENCE.
    // If so it runs long, so start the guard earlier.
    bool nextIsCert = (evuFramesSinceCert >= (CERT_CADENCE - 1));
    unsigned long thisFrameAirtime = nextIsCert ? EVU_CERT_AIRTIME_MS
                                                : EVU_AIRTIME_MS;

    if (elapsed < (thisFrameAirtime + ICU_TX_GUARD_MS)) return false;

    unsigned long untilNextEVU = EVU_TX_INTERVAL_MS - elapsed;
    return untilNextEVU > (txAirtimeMs + ICU_TX_GUARD_MS);
}

#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_SS    10
#define LORA_RST   9
#define LORA_DIO0  8

// Must match the LoRa.setSpreadingFactor() call in initLoRa() below --
// used only to compute the "Link Margin" diagnostic, has no effect on
// the radio itself.
#define LORA_SF 9          // IF-1, the ambulance link. DO NOT LOWER.

/***********************************************************************
 * FIX 3 : THE TWO LINKS GET DIFFERENT SPREADING FACTORS.
 *
 * Both links ran at SF9. That was wrong, and it is the single biggest
 * cause of the channel congestion measured on this bench.
 *
 * The two links have nothing in common:
 *
 *   IF-1  ambulance -> node    up to 2 km, one end MOVING, unknown
 *                              position, must not be missed. Needs SF9.
 *
 *   IF-2  node -> ICU          ~250 m, BOTH ENDS BOLTED DOWN at a
 *                              distance known at install time.
 *
 * SF9 buys range that IF-2 does not need and cannot use, and charges
 * 4x the airtime for it. A 58-byte relay costs 443 ms at SF9 and
 * 135 ms at SF7 -- for a link where the far end is 250 m away and
 * never moves.
 *
 * That airtime is what two nodes were fighting over. Measured duty on
 * 434.5 during a live event was 41% across the two nodes, against a
 * 10% ceiling, which is why a node showing 100% delivery in a
 * standalone radio test showed 30% inside the running system. The
 * radio was never the problem; the nodes were talking over each other.
 *
 * IS SF7 ENOUGH? Bench measurement gave -34 to -42 dBm node-to-ICU. At
 * 250 m outdoors expect roughly -75 dBm. SF7 sensitivity is -117 dBm,
 * leaving about 42 dB of margin -- a factor of 16,000 in power.
 *
 * [VERIFY BEFORE TRUSTING] That is arithmetic, not measurement. Run the
 * standalone link test at your real node spacing with SF7 before
 * relying on it. If it disappoints, SF8 still halves the airtime
 * against SF9 and costs only 3 dB.
 *
 * MUST MATCH ICU_LORA_SF IN ICU.ino. A node at SF7 and an ICU at SF9
 * are completely deaf to each other -- there is no partial failure and
 * no error message, they simply never hear one another.
 ***********************************************************************/
#define LORA_SF_ICU 7      // IF-2, the short fixed link to the cabinet

// CHANGED v3.7: sync words were four scattered literals (0xF3 in initLoRa()
// and in the retune-back path, 0xA5 in radiateToICU()) plus a comment on
// one of them reading "must match ICU's LoRa.setSyncWord(0xA5)". A sync
// word mismatch is silent -- the radio simply never reports a packet -- so
// this is the worst class of constant to duplicate. Named once, used
// everywhere, printed in the banner from the same symbol.
#define LORA_SYNC_WORD_EVU 0xF3    // IF-1, ambulance link
#define LORA_SYNC_WORD_ICU 0xA5    // IF-2, must match ICU.ino

/***********************************************************************
 * v3 TRANSMIT SCHEDULING AND POWER
 *
 * NOTE ON SPREADING FACTOR: SF9/CR4-6 is KEPT, not changed. It looks
 * expensive in airtime terms, but EVU2.ino records that SF7 was the
 * original baseline and measured 39% packet loss at TC-TX-002. That is
 * measurement, and measurement beats arithmetic. Do not "optimise" this
 * back to SF7 without re-running the range test.
 *
 * Airtimes below are computed for SF9 / BW125kHz / CR4-6 / preamble 12
 * and MUST be recomputed if any of those change.
 ***********************************************************************/
// CHANGED v3.2 note, CORRECTED v3.6: observed spacing in TC-LN-001 was
// 5.8-8.1s (v3.3) and later 5.42-13.65s (v3.5), not the designed 5.0-6.5s,
// because heartbeats also wait for a free transmit window.
//
// The v3.2 text here read "raised to 2 to match the observed upper bound",
// which was both garbled and contradicted ICU.ino (which said 25000ms for
// 3 misses). Neither number is current: the ICU is now 40000ms and the
// spacing is BOUNDED by HEARTBEAT_MAX_DEFER_MS below rather than left to
// whatever the window scheduler allows. Two files quoting each other's
// constants from memory is how this drifted; both now name the constant.
/***********************************************************************
 * FIX 2 : HEARTBEAT SLOWED 5s -> 10s.
 *
 * The ICU does not mark a node SUSPECT until 26 s of silence, or FAILED
 * until 40 s. A 5 s heartbeat was reporting roughly four times more
 * often than anything consumed it, and paying full airtime for each one
 * on a channel two nodes share.
 *
 * At 10 s the ICU still sees two heartbeats before it would consider a
 * node suspect and four before failed -- unchanged detection behaviour,
 * half the channel occupancy.
 *
 * If NODE_SUSPECT_MS or NODE_FAILED_MS in ICU_Geometry.ino are ever
 * shortened, revisit this: the relationship is what matters, not the
 * number.
 ***********************************************************************/
#define HEARTBEAT_BASE_MS      10000UL
#define HEARTBEAT_JITTER_MS    1500UL   // random 0..this, re-rolled each cycle

/***********************************************************************
 * v6 : SIX-NODE TRANSMIT SLOTTING   (replaces FIX 4 and FIX 5)
 *
 * WHY THE TWO-NODE SCHEME DOES NOT EXTEND.
 *
 * FIX 4 offset the heartbeat by (NODE_ID-1) * half an interval. FIX 5
 * delayed node 2 by 250 ms inside radiateToICU(). Three problems once
 * there are six nodes on three approaches:
 *
 *   1. Both key on NODE_ID alone, so they only separate node 1 from
 *      node 2. All three lane-N1 nodes fire together, and all three
 *      lane-N2 nodes fire together. The lane is invisible to both.
 *
 *   2. MORE SERIOUS -- and this is a defect at TWO nodes, today:
 *      FIX 5's delay is applied AFTER icuTransmitWindowOpen() has
 *      already approved the transmission. The usable window is only
 *      795 ms wide (see the timeline above icuTransmitWindowOpen()). A
 *      frame approved at elapsed=1600 ms and then delayed 250 ms starts
 *      at 1850 ms, inside the guard band. The stagger invalidates the
 *      check that permitted it.
 *
 *      This is the same shape as the phase-anchor bug: the value was
 *      correct, but it was consumed at a point where it no longer meant
 *      what the caller assumed. Widening the stagger makes it worse, not
 *      better -- a 500 ms stagger puts the frame at 2100 ms, i.e.
 *      directly on top of the next EVU frame, every time.
 *
 *   3. A NEW CONTENTION SOURCE APPEARS AT THREE LANES. FIX 1 splits
 *      relay duty by NODE_ID parity against the EVU sequence number --
 *      not by lane. All six nodes hear the same EVU beacon. So on
 *      sequence N, all three lane-N1 nodes independently decide "my
 *      turn" and transmit in the same few milliseconds. At two nodes
 *      that was one transmitter; at six it is three.
 *
 * WHY 100 ms SLOTS DO NOT WORK.
 *
 * The tempting fix is slot = (lane-1)*2 + (node-1), 100 ms apart,
 * spreading six nodes over 500 ms -- "wider than three frame lengths,
 * so no two can overlap". That reasoning is wrong. Slot SPACING must
 * exceed frame AIRTIME; total span is irrelevant. At 100 ms spacing and
 * 165 ms frames every adjacent pair overlaps by 65 ms. It also ignores
 * the listen-before-talk backoff already inside radiateToICU(), which
 * was up to 3 x 50 = 150 ms of entirely unmodelled delay.
 *
 * WHAT THIS DOES INSTEAD: PHASE, NOT DELAY.
 *
 * A node no longer waits. It transmits only while millis() lies inside
 * its assigned PHASE of the EVU period, and if that phase has passed it
 * waits for the next one. Nothing is ever delayed past a window it was
 * already cleared for, because the clearance and the separation are now
 * the same test.
 *
 * SIX SLOTS DO NOT FIT IN ONE WINDOW. 795 ms usable, and a slot needs
 * 165 ms airtime + bounded LBT + margin = 220 ms. Six would need
 * 1320 ms. So the six slots fold across TWO EVU periods:
 *
 *   period parity 0 (even EVU seq)   L1N1 @840   L2N1 @1060   L3N1 @1280
 *   period parity 1 (odd  EVU seq)   L1N2 @840   L2N2 @1060   L3N2 @1280
 *
 * The fold is not arbitrary. FIX 1 already alternates the node-1s and
 * node-2s against the EVU sequence number, so they ALREADY occupy
 * alternate periods. This map follows the traffic pattern that exists
 * rather than imposing a second, conflicting one -- and it is what
 * resolves problem 3 above: the three simultaneous relayers land in
 * three different slots.
 *
 * COST: a node's transmit opportunity is once per 4000 ms rather than
 * once per 2000 ms. HEARTBEAT_BASE_MS is 10000 and EVENT_MIN_INTERVAL_MS
 * is 4000, so neither is constrained by this. The two frames that cannot
 * wait -- the emergency falling edge and an overdue heartbeat -- are
 * exempt; see the bypass argument to icuSlotOpen().
 ***********************************************************************/
#define NODE_TX_SLOT       ((((LANE_ID) - 1) * 2) + ((NODE_ID) - 1))   // 0..5
#define NODE_TX_PERIOD     ((NODE_TX_SLOT) & 1u)                       // 0 or 1
#define NODE_TX_POSITION   ((NODE_TX_SLOT) >> 1)                       // 0..2

static_assert(NODE_TX_SLOT >= 0 && NODE_TX_SLOT <= 5,
              "NODE_TX_SLOT out of range - check LANE_ID / NODE_ID");

// Slot width must exceed the largest IF-2 frame plus the worst bounded
// LBT backoff plus margin:  165 + 40 + 15 = 220.
//
// Three slots from 840 ms end at 1500 ms, inside the 1635 ms close for a
// 165 ms frame -- 135 ms of headroom.
//
// RECOMPUTE THIS if ICU_TX_MAX_AIRTIME_MS, LORA_SF_ICU, EVU_AIRTIME_MS or
// ICU_TX_GUARD_MS change. The number is derived, not chosen.
#define ICU_SLOT_WIDTH_MS    220UL
#define ICU_WINDOW_OPEN_MS   (EVU_AIRTIME_MS + ICU_TX_GUARD_MS)        // 840
#define NODE_SLOT_START_MS   (ICU_WINDOW_OPEN_MS + \
                              (unsigned long)(NODE_TX_POSITION) * ICU_SLOT_WIDTH_MS)

static_assert(NODE_SLOT_START_MS + ICU_SLOT_WIDTH_MS
                  <= EVU_TX_INTERVAL_MS - ICU_TX_GUARD_MS,
              "Slot runs past the EVU guard band - narrow ICU_SLOT_WIDTH_MS");

// Free-running fallback, used ONLY before any EVU has been heard -- at
// which point no shared epoch exists and slotting is meaningless.
// Heartbeats are the only traffic in that state. Six-way phase split,
// replacing FIX 4's two-way one.
#define NODE_TX_OFFSET_MS    ((unsigned long)(NODE_TX_SLOT) * (HEARTBEAT_BASE_MS / 6))

// v6: bounded listen-before-talk.
//
// The old loop allowed 3 x random(5,50) = up to 150 ms of delay that
// nothing accounted for -- 68% of a slot. With deterministic slots the
// only collisions LBT can still catch are UNSCHEDULED ones (a foreign
// transmitter, or a node whose slot arithmetic disagrees), so one short
// retry is enough, and its whole budget must fit inside the slot
// alongside the frame.
#define ICU_LBT_MAX_BACKOFF_MS  40UL
#define ICU_LBT_ATTEMPTS         2

/***********************************************************************
 * v6 : icuSlotOpen()
 *
 * Returns true if a txAirtimeMs frame started NOW would fit entirely
 * inside THIS NODE'S slot, in an EVU period this node owns.
 *
 * Two gates, and the order matters:
 *
 *   icuTransmitWindowOpen()  -- "is the EVU off the air?"  NEVER bypassed.
 *   slot phase               -- "is it my turn?"           bypassable.
 *
 * bypass = true for the emergency FALLING EDGE and for a heartbeat past
 * HEARTBEAT_MAX_DEFER_MS. Both are single, unrepeatable frames whose loss
 * costs more than a possible collision does. Bypass skips the slot and
 * NOTHING ELSE: violating the EVU guard would deafen this node to the
 * ambulance, which is the one thing the slot scheme exists to protect.
 ***********************************************************************/
bool icuSlotOpen(unsigned long txAirtimeMs, bool bypass) {

    if (!icuTransmitWindowOpen(txAirtimeMs)) return false;
    if (bypass)                              return true;

    // No EVU heard yet: there is no shared epoch to slot against. Fall
    // back to NODE_TX_OFFSET_MS, applied at task start.
    if (lastEVUTxStartMillis == 0 || !lastEVUSeqValid) return true;

    unsigned long sinceAnchor = millis() - lastEVUTxStartMillis;
    unsigned long periodsOn   = sinceAnchor / EVU_TX_INTERVAL_MS;
    unsigned long elapsed     = sinceAnchor % EVU_TX_INTERVAL_MS;

    // Period label every node agrees on: the anchor frame's sequence
    // number, advanced by however many whole periods have since elapsed.
    // Deliberately NOT a locally counted period index -- see lastEVUSeq.
    uint32_t periodIndex = (uint32_t)lastEVUSeq + (uint32_t)periodsOn;
    if ((periodIndex & 1u) != (uint32_t)NODE_TX_PERIOD) return false;

    unsigned long slotStart = NODE_SLOT_START_MS;
    unsigned long slotEnd   = slotStart + ICU_SLOT_WIDTH_MS;

    if (elapsed < slotStart)                return false;
    if (elapsed + txAirtimeMs > slotEnd)    return false;

    return true;
}

// CHANGED v3.6: NEW. Bounds how long a due heartbeat may be held waiting
// for a clear transmit window.
//
// TC-LN-001 (v3.5, 2026-07-29) measured heartbeat spacing AT THE ICU of
// min 5.42s / mean 6.96s / max 13.65s -- against an ICU timeout of 25000ms,
// i.e. 1.8 clean misses of margin on a link with no acknowledgement and no
// retry. The designed spacing is 5.0-6.5s. The gap is icuTransmitWindowOpen()
// repeatedly declining: the usable gap is 560ms out of every 2000ms, so a
// little jitter between the EVU's free-running millis() and this node's is
// enough to miss several cycles in a row.
//
// And that run had ZERO acoustic traffic. Periodic re-assert adds a third
// frame class contending for the same window, so 13.65s is a floor.
//
// The tempting fix was to raise the ICU's timeout. That treats the symptom:
// the number goes stale again the moment traffic changes, exactly as the
// v3.2 figure did. Bounding the deferral instead makes worst-case spacing a
// DESIGNED quantity that the ICU timeout can be derived from.
//
// The cost is explicit and small: after this long, a heartbeat transmits
// even if the window is not clear, and this node is deaf to the EVU for
// ~271ms. At a 2000ms beacon that risks at most one beacon, at most once
// per 12s, and only when windows are already being missed. A missed
// heartbeat costs node-offline margin; a missed beacon costs nothing,
// because the next one arrives in 2s and the ICU holds loraDetected for
// 20s. Trading the cheap loss for the expensive one is the right way round.
//
// SIZING -- GET THIS RIGHT, IT IS EASY TO GET WRONG:
//
// The deferral clock starts at (lastHeartbeat + hbInterval), NOT at
// lastHeartbeat. So worst-case spacing is the SUM:
//
//   HEARTBEAT_BASE_MS + HEARTBEAT_JITTER_MS + HEARTBEAT_MAX_DEFER_MS + airtime
//   = 5000 + 1500 + 6000 + 271  =  ~12.8 s
//
// The first draft of this used 12000UL on the belief that worst spacing
// would be ~12.5s. Simulation measured 18050ms, i.e. 2.2 misses against the
// ICU's 40000ms -- WORSE than the 1.8 misses this change existed to fix,
// while looking like an improvement. The arithmetic was wrong, not the
// mechanism. Same lesson as the phase anchor: the variable did not mean
// what the formula assumed.
//
// At 6000UL: ~12.8s worst spacing, 3.1 clean misses against 40000ms.
//
// PAIRED WITH HEARTBEAT_TIMEOUT_MS IN ICU.ino (40000, derived from this).
// Change one, recompute the other -- and recompute the SUM, not the term.
#define HEARTBEAT_MAX_DEFER_MS 6000UL
// v4: 28 -> 32 bytes (session_epoch). 271 -> 296 ms. Under-reserving the
// gap is precisely the drift class that caused TC-LN-001.
// v5: HeartbeatFrame grew 32 -> 42 bytes (surveyed geometry + self-test).
// Recomputed for SF9/BW125/CR4-6, 12-symbol preamble: 345.1 ms, plus
// margin. The old 296 was measured against the 32-byte frame and would
// now under-reserve the channel by ~50 ms on every heartbeat.
//
// This is not bookkeeping. The RDU shares ONE radio between listening
// for the ambulance on 433.0 and talking to the ICU on 434.5, so any
// millisecond it under-books is a millisecond it believes it is
// listening while it is still transmitting.
// SF7 (FIX 3). 42-byte HeartbeatFrame computes to 105 ms; margin added.
// Was 375 at SF9.
#define HEARTBEAT_AIRTIME_MS    130UL
// v4: 52 -> 56 bytes. Real airtime 419 ms; 425 keeps the same ~6 ms margin
// the v3 value had over its own 394 ms.
// v5: LoRaEventFrame grew 56 -> 58 bytes (heading_deg added).
// 443.4 ms computed, plus margin.
// SF7 (FIX 3). 58-byte LoRaEventFrame computes to 135 ms; margin added.
// Was 475 at SF9.
#define EVENT_AIRTIME_MS        165UL
// CHANGED v3.1: 3 -> 1.
//
// Repetition protects against losing a RARE detection. Detections here
// are not rare -- the EVU beacons every 2s, so the next beacon regenerates
// the event anyway. The natural beacon cadence IS the redundancy.
//
// Each repeat needed its own transmit window, so 3 repeats spanned 3 EVU
// cycles (~6s) with the radio retuned away for part of each. TC-LN-001
// measured this as a ~6 second lockout after every single detection.
#define EVENT_REPEAT_COUNT         1

// Do not generate a new event more often than this. The ICU holds
// loraDetected for EVENT_TIMEOUT_MS, so one event covers that much
// decision state. Generating one every 2s and repeating it 3x was roughly
// 30x more traffic than any decision could consume.
//
// v3.5 note: EVENT_TIMEOUT_MS went 20000 -> 35000 in v3.4 and back to
// 20000 in v3.5, so this comment is accurate again. It was briefly wrong.
// If you move EVENT_TIMEOUT_MS again, this ratio (4000 : 20000 = 5 covers
// per timeout) is the thing to preserve.
#define EVENT_MIN_INTERVAL_MS   4000UL

// Set for the single packet that carries a 1 -> 0 emergency transition.
//
// File-scope rather than a parameter because the relay decision and the
// transmit rate limiter live in different functions, and the limiter
// must be able to see that this particular packet is not replaceable by
// the next one -- there will not be a next one.
bool gwFallingEdgeNow = false;

// v3.3: how many times to re-attempt an event whose transmission was
// blocked by radio contention before giving up on it. Bounded so a stuck
// or permanently busy radio cannot pin the queue forever and block every
// subsequent detection behind one undeliverable frame.
#define EVENT_MAX_RETRIES          5

// Explicit TX power on both links. IF-2 is a short fixed link to a
// surveyed node -- it does not need the EVU link's power, and radiating
// harder than necessary makes the city-scale channel congestion problem
// worse for every neighbouring intersection.
#define EVU_LINK_TX_POWER_DBM     17
#define ICU_TX_POWER_DBM          14

// Diagnostics, published so a degrading link is visible before it fails.
volatile uint32_t icuTxSkipped       = 0;   // mutex contention
volatile uint32_t eventsDropped      = 0;   // v3.3: detections never radiated
volatile uint32_t icuTxForcedOnBusy  = 0;   // CAD said busy, we sent anyway
volatile uint32_t icuTxNoSession     = 0;   // v4: frames not sent, no session key
volatile uint32_t hbForcedOnDeadline = 0;   // v3.6: heartbeat sent past HEARTBEAT_MAX_DEFER_MS
volatile uint32_t hbMaxSpacingMs     = 0;   // v3.6: worst spacing seen, for sizing the ICU timeout

// v3.6: IF-1 link quality accumulators. TC-LN-001 v3.5 showed IF-1 SNR
// ranging 2.5-12.8 dB at ~2 metres, with 5 of 36 packets under 6 dB, and a
// documented drift from ~13.8 dB earlier in the project. A 10 dB spread at
// two metres is not a link-budget problem, it is interference or antenna
// coupling -- and it is invisible unless somebody diffs two logs by hand.
// Summarising it here puts the trend in front of whoever is running the
// test, which is the only reason the phase-anchor bug was ever found.
volatile uint32_t if1Packets   = 0;
float             if1SnrSum    = 0.0f;
float             if1SnrMin    =  999.0f;
float             if1SnrMax    = -999.0f;
int               if1RssiMin   =  999;
int               if1RssiMax   = -999;
volatile uint32_t if1LowSnrPkts = 0;        // SNR < 6 dB
#define IF1_LOW_SNR_DB 6.0f

// CHANGED v3.7: rolling window alongside the session totals.
//
// The session mean hid a real trend. Across three TC-LN-001 sessions the
// IF-1 SNR MINIMUM went 8.0 dB -> 2.5 dB -> -0.5 dB at roughly two metres,
// while the session MEAN barely moved (11.0 -> 10.9). A mean over 400
// packets cannot show a degrading floor, and the floor is the number that
// decides whether SF8 is viable.
//
// This is not a link-budget problem -- SF9 demodulates to -12.5 dB and IF-1
// delivered 100% in the v3.6 run. Something is desensitising the receiver
// or the antennas are coupling. Firmware cannot fix that. What firmware CAN
// do is refuse to let the trend stay invisible until somebody diffs two
// logs by hand, which is how it was found in the first place.
//
// The window is the last IF1_WINDOW_PKTS packets, printed next to the
// session figures so drift is visible WITHIN one run and not only across
// them. A warning fires when the windowed low-SNR fraction exceeds
// IF1_DEGRADED_FRAC.
//
// DO NOT ACT ON SF8 vs SF9 UNTIL THE BAND HAS BEEN SWEPT AT THE BENCH.
// The TC-TX-002 measurement that fixed SF9 was taken when this floor was
// 8 dB; it is not a valid basis for a parameter decision at -0.5 dB,
// because it may have been measuring the interferer rather than the link.
#define IF1_WINDOW_PKTS   30
#define IF1_DEGRADED_FRAC 0.10f

float             if1WinSnr[IF1_WINDOW_PKTS] = {0};
uint8_t           if1WinIdx   = 0;
uint32_t          if1WinCount = 0;

// Channel Activity Detection wrapper. The Arduino LoRa library does not
// expose CAD directly, so we approximate it by reading the modem's RSSI:
// if the channel is materially above the noise floor, somebody else is
// probably talking. Crude, but it is the difference between "we never
// look before transmitting" and "we usually look".
#define CAD_RSSI_BUSY_DBM   (-95)
bool isChannelBusy() {
    return LoRa.rssi() > CAD_RSSI_BUSY_DBM;
}

// Typical SX127x SNR demodulation floor per spreading factor (dB), BW125kHz.
// Link Margin = measured SNR - this floor: positive and growing = comfortable,
// hovering near 0 = right at the edge, negative = you got lucky this packet
// decoded at all. This is what should be watched closely as distance increases
// during the 50/100/200m sweep.
float snrFloorForSF(int sf) {
    switch(sf) {
        case 6:  return -5.0;
        case 7:  return -7.5;
        case 8:  return -10.0;
        case 9:  return -12.5;
        case 10: return -15.0;
        case 11: return -17.5;
        case 12: return -20.0;
        default: return -7.5;
    }
}

// CHANGED v3: builds a 52-byte LoRaEventFrame instead of a 136-byte
// EventPacket, and signs it.
//
// Airtime at SF9/BW125/CR4-6: 861ms -> 394ms. That matters more than it
// looks, because this node has ONE radio: every millisecond spent
// transmitting on 434.5 is a millisecond it cannot hear the ambulance on
// 433.0. The old 3x861ms event burst blinded the node for 2.6 seconds at
// exactly the moment it had just detected an ambulance.
//
// Fields removed and why:
//   distance_m, eta_s  - the ICU knows its own surveyed position and the
//                        vehicle position, so it should compute these
//                        itself. One authority computing a value beats
//                        six nodes each asserting their own.
//   altitude_m         - nothing read it
//   tx_uptime          - always 0, there is no uptime field in the EVU
//                        payload to source it from
//   uptime_ms          - nothing read it
//   snr (float)        - now int16 x10
//   motion_state[16], signature_status[16], vehicle_id[32] -> enums / char[8]
LoRaEventFrame buildLoRaEvent(const char *vehicleID, int rssi, float snr, uint32_t txSeq,
                              bool emergencyFlag, uint8_t priorityValue, bool gpsValidFlag,
                              uint8_t sigStatus, uint32_t gpsEpoch, bool geofencePass) {
    LoRaEventFrame f;
    memset(&f, 0, sizeof(f));

    f.hdr.proto_version   = GW_PROTO_VERSION;
    f.hdr.packet_type     = LORA_EVENT_PACKET;
    f.hdr.intersection_id = INTERSECTION_ID;
    f.hdr.lane_id         = LANE_ID;
    f.hdr.node_id         = NODE_ID;
    f.hdr.counter         = 0;   // stamped at transmit time -- see radiateToICU()

    f.flags = EVF_LORA_DETECTED;
    if (emergencyFlag)            f.flags |= EVF_EMERGENCY;
    if (gpsValidFlag)             f.flags |= EVF_GPS_VALID;
    if (geofencePass)             f.flags |= EVF_GEOFENCE_PASS;
    if (GEOFENCE_BYPASSED_FOR_SOP) f.flags |= EVF_GEOFENCE_BYPASS;

    f.sig_status   = sigStatus;
    f.priority     = priorityValue;
    f.motion_state = MOTION_APPROACHING;

    strncpy(f.vehicle_id, vehicleID, 7);
    f.vehicle_id[7] = '\0';

    // CHANGED v3: carried as scaled ints, exactly as they arrived from the
    // EVU. The old code converted double -> float, which silently threw
    // away about 1 metre of position precision on every hop for no reason.
    f.latitude     = (int32_t)lround(vehicleLatitude  * 10000000.0);
    f.longitude    = (int32_t)lround(vehicleLongitude * 10000000.0);
    f.speed_kmph   = (uint16_t)lround(vehicleSpeed * 100.0);

    /*******************************************************************
     * v5 / PHASE 4 : RELAY THE HEADING
     *
     * The EVU has always sent heading in IF-1. v4 read it here (for the
     * geofence heading check) and then THREW IT AWAY instead of passing
     * it on, so the ICU never saw it.
     *
     * Two things are impossible without it, and both are in the spec as
     * defects against v1.0:
     *
     *   S2-14  Approach association. Corridor membership alone puts two
     *          EVUs travelling in OPPOSITE directions on the same road
     *          into the same approach -- and they need opposite phases.
     *          Position says which road; only heading says which way.
     *
     *   S2-09  Divergence. A vehicle that turns away otherwise holds its
     *          demand until an undefined timeout, because nothing in the
     *          system can observe that it left.
     *
     * Degrees x100, matching IF-1. 0xFFFF means "no valid heading",
     * which is NOT the same as 0 -- 0 is due north and is a perfectly
     * good heading. Collapsing the two would make every unfixed GPS look
     * like a vehicle driving due north.
     *******************************************************************/
    // A STATIONARY GPS REPORTS HEADING 0, AND 0 IS DUE NORTH.
    //
    // GPS derives heading from movement. With no movement there is no
    // heading, and the receiver reports 0.00 -- which is a perfectly
    // valid bearing. A bench run showed a stationary EVU transmitting
    // heading=0 continuously, and the ICU had no way to tell that from
    // a vehicle genuinely driving north.
    //
    // That matters because the ICU uses heading to decide WHICH APPROACH
    // a vehicle is on (spec S2-14). Believing a fabricated north would
    // associate a stopped ambulance with whichever approach happens to
    // run north-south, and could point an officer at the wrong road.
    //
    // Below the speed threshold the heading is reported as UNKNOWN
    // (0xFFFF) rather than as 0. The ICU then declines to associate an
    // approach at all, which is the honest answer: a stationary vehicle
    // has no direction of travel.
    //
    // 3 km/h is walking pace -- below it, reported heading is noise.
    if (vehicleSpeed >= 3.0 &&
        vehicleHeading >= 0.0f && vehicleHeading < 360.0f) {
        f.heading_deg = (uint16_t)lround(vehicleHeading * 100.0f);
    } else {
        f.heading_deg = 0xFFFF;   // no valid heading, NOT due north
    }

    f.gps_epoch    = gpsEpoch;
    f.tx_seq       = txSeq;
    f.rssi_if1     = (int16_t)rssi;
    f.snr_if1_x10  = (int16_t)lround(snr * 10.0);

    // NOT signed here. The counter is stamped at transmit time, and the
    // CMAC covers the counter, so signing must happen after stamping.
    // radiateToICU() does both.
    return f;
}

/***********************************************************************
 * OUTBOUND FRAME QUEUE
 *
 * CHANGED v3: the queue used to hold EventPacket structs, so it could
 * only ever carry one frame type. Now it carries an opaque byte buffer
 * plus a length, so heartbeats, LoRa events and acoustic events all go
 * through one path -- which means the CAD/backoff and window-scheduling
 * logic below only has to exist once.
 ***********************************************************************/
struct OutFrame {
    uint8_t len;
    uint8_t buf[GW_MAX_FRAME_LEN];
    uint8_t attempts;      // repeats that ACTUALLY radiated
    uint8_t retries;       // v3.3: times the radio was unavailable
    // v6: does this frame carry the emergency falling edge?
    //
    // Captured AT QUEUE TIME, not read from gwFallingEdgeNow at transmit
    // time. The frame sits in a queue and the flag is file-scope, so by
    // the time transportTask picks it up the flag may describe a
    // completely different event. The property belongs to the frame.
    bool    fallingEdge;
};

void queueFrame(const void *frame, size_t len) {
    OutFrame o;
    if (len > GW_MAX_FRAME_LEN) { Serial.println("[QUEUE] FRAME TOO LARGE"); return; }
    o.len = (uint8_t)len;
    o.attempts = 0;
    o.retries = 0;
    o.fallingEdge = gwFallingEdgeNow;
    memcpy(o.buf, frame, len);
    if(xQueueSend(eventQueue, &o, 0) == pdPASS) {
        Serial.printf("[QUEUE] FRAME ADDED | len=%u depth=%d\n",
                      (unsigned)len, (int)uxQueueMessagesWaiting(eventQueue));
    } else {
        Serial.println("[QUEUE] FULL");
    }
}

void bufferPacket(const char *id, const uint8_t *data, size_t len, int rssi, float snr) {
    for(int i=0; i<RETRO_BUFFER_SIZE; i++) {
        if(!retroBuffer[i].used) {
            retroBuffer[i].used = true;

            // CHANGED v3: fixed char[8] instead of an Arduino String. String
            // inside a struct that lives for the life of the program is a
            // heap-fragmentation source on a device expected to run for
            // months without a reboot.
            strncpy(retroBuffer[i].vehicleID, id, 7);
            retroBuffer[i].vehicleID[7] = '\0'; 
            
            retroBuffer[i].len = len;
            if(retroBuffer[i].len > sizeof(retroBuffer[i].buffer)) 
                retroBuffer[i].len = sizeof(retroBuffer[i].buffer);
            memcpy(retroBuffer[i].buffer, data, retroBuffer[i].len);
            retroBuffer[i].rssi = rssi;
            retroBuffer[i].snr = snr;
            retroBuffer[i].time = millis();
            return;
        }
    }
}

// Forward declaration of the updated packet processor
void processLoRaPacket(uint8_t *rxBuffer, size_t len, int rssi, float snr, bool isRetroReplay);

void retryBufferedPackets(const char *vehicleID) {
    uint32_t highestRetroSeq = 0;
    bool updatedSeq = false;
    VehicleTrust *trust = findVehicle(vehicleID);
    if (trust == NULL) return;

    // ===================================================================
    // SECURITY FIX v3 -- this was the worst bug on the ambulance link.
    //
    // The old code did this here:
    //     trust->sequenceStarted = false;
    //     trust->lastSequence    = 0;
    //
    // i.e. it threw away the anti-replay baseline on EVERY certificate
    // handshake. Certificates arrive on every 5th frame, so at a 2s
    // beacon that is every 10 seconds, forever.
    //
    // Concretely: an attacker records one old certificate-bearing frame
    // and replays it. Because sequenceStarted was just cleared, the
    // replay check is skipped, the frame is accepted, and lastSequence is
    // then set BACKWARDS to that old value. Every recorded frame newer
    // than it now replays successfully. The counter was rolled back by
    // the attacker, using nothing but a recording.
    //
    // The baseline is now only established once, for a vehicle we have
    // never seen before. A certificate refresh for an already-known
    // vehicle changes nothing about replay state -- which is correct,
    // because re-presenting a certificate says nothing about freshness.
    // ===================================================================
    bool firstEnrolment = !trust->sequenceStarted;

    for (int i = 0; i < RETRO_BUFFER_SIZE; i++) {
        // FIXED: Replaced standard C strcmp with the native .equals() method of Arduino String objects
        if (retroBuffer[i].used && strncmp(vehicleID, retroBuffer[i].vehicleID, 8) == 0) {
            bool stillFresh = (millis() - retroBuffer[i].time) <= RETRO_TIMEOUT_MS;
            retroBuffer[i].used = false; 
            if (stillFresh) {
                Serial.print("[RETRO VERIFY] Replaying buffered packet for ");
                Serial.println(vehicleID);
                
                // Peek at sequence number inside payload to find maximum bound
                if (retroBuffer[i].len >= sizeof(TelemetryPayload)) {
                    TelemetryPayload tmp;
                    memcpy(&tmp, retroBuffer[i].buffer, sizeof(TelemetryPayload));
                    if (tmp.seq > highestRetroSeq) {
                        highestRetroSeq = tmp.seq;
                        updatedSeq = true;
                    }
                }
                
                // Process with isRetroReplay context flag set to TRUE
                processLoRaPacket(retroBuffer[i].buffer, retroBuffer[i].len, retroBuffer[i].rssi, retroBuffer[i].snr, true);
            }
        }
    }

    // Establish the baseline ONLY on first enrolment, and only ever move it
    // forward -- never backwards.
    if (updatedSeq && firstEnrolment && highestRetroSeq > trust->lastSequence) {
        trust->lastSequence = highestRetroSeq;
        trust->sequenceStarted = true;
        Serial.printf("[SECURITY] Post-Retro Anti-Replay Baseline set to Sequence: %lu\n", (unsigned long)trust->lastSequence);
    }
}

String getLinkQuality(int rssi, float snr) {
    if(rssi > -70 && snr > 7) return "EXCELLENT";
    if(rssi > -100) return "GOOD";
    return "WEAK";
}

String formatUptime() {
    unsigned long totalSeconds = millis() / 1000;
    unsigned long hrs = totalSeconds / 3600;
    unsigned long mins = (totalSeconds % 3600) / 60;
    unsigned long secs = totalSeconds % 60;
    char buf[12];
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hrs, mins, secs);
    return String(buf);
}

void processLoRaPacket(uint8_t *rxBuffer, size_t len, int rssi, float snr, bool isRetroReplay) {
    if(len < sizeof(TelemetryPayload) + 64) {
        Serial.println("[BINARY DECODE FAILED]");
        return;
    }

    // TIER-A ISOLATION FIX: record live EVU arrival time for gap-aware ICU
    // scheduling (see icuTransmitWindowOpen()). Retro replays are buffered
    // packets being reprocessed after the fact, not live radio traffic, so
    // they must not be used to predict the EVU's live transmission schedule.
    if (!isRetroReplay) {
        lastEVURxMillis = millis();

        // CHANGED v3.1: back-date the anchor by this frame's airtime so it
        // marks when the EVU STARTED transmitting. We know which kind of
        // frame this was from its length, so we know its airtime exactly.
        bool wasCert = (len >= sizeof(TelemetryPayload) + 64 + 32 + 64);
        lastEVUTxStartMillis = millis() - (wasCert ? EVU_CERT_AIRTIME_MS
                                                   : EVU_AIRTIME_MS);
        evuFramesSinceCert = wasCert ? 0 : (evuFramesSinceCert + 1);
    }

    TelemetryPayload ev;
    memcpy(&ev, rxBuffer, sizeof(TelemetryPayload));

    uint8_t signature[64];
    memcpy(signature, rxBuffer + sizeof(TelemetryPayload), 64);

    size_t normalPacketLen = sizeof(TelemetryPayload) + 64;
    size_t certPacketLen = normalPacketLen + 32 + 64;
    bool certPresent = (len >= certPacketLen);
    uint8_t certPubKey[32];
    uint8_t certSignature[64];

    if(certPresent) {
        memcpy(certPubKey, rxBuffer + normalPacketLen, 32);
        memcpy(certSignature, rxBuffer + normalPacketLen + 32, 64);
    }

    char vehicleID[8];
    memcpy(vehicleID, ev.vehicle_id, 6);
    vehicleID[6]='\0';
    vehicleID[7]='\0';

    uint8_t priority = ev.priority_class;
    bool emergency = ev.flags & (1<<0);
    bool gpsValid = ev.flags & (1<<1);
    bool siren = ev.flags & (1<<2);
    uint32_t seq = ev.seq;

    // v6: period label for transmit slotting. Guarded by !isRetroReplay to
    // match lastEVUTxStartMillis above -- a buffered replay is not live
    // radio traffic and must not shift this node's slot.
    if (!isRetroReplay) {
        lastEVUSeq      = seq;
        lastEVUSeqValid = true;
    }
    uint32_t timestamp = ev.gps_epoch;
    uint16_t milliseconds = ev.gps_ms; // Extracted Milliseconds
    double latitude = ev.latitude / 10000000.0;
    double longitude = ev.longitude / 10000000.0;
    bool altValid = ev.flags & (1<<3);
    int16_t altitude = ev.altitude_m;
    float speed = ev.speed_kmph / 100.0;
    float heading = ev.heading_deg / 100.0;
    uint8_t testLeg = ev.test_leg;

    uint8_t securityStatus = SIG_UNVERIFIED;
    VehicleTrust *trust = NULL;
    bool sigValid = false;

    // WAS `if(emergency)`. THIS IS THE LINE THAT CRASHED THE NODE.
    //
    // trust is initialised to NULL above and was only ever populated
    // when the emergency flag was set. Further down, the relay path
    // dereferences trust->keyValid unconditionally -- which was safe
    // ONLY because a bare `if(!emergency) return;` sat between the two
    // and made that code unreachable for non-emergency packets.
    //
    // Adding the falling-edge relay removed that guard, so the first
    // emergency=0 packet reached trust->keyValid with trust == NULL,
    // panicked the ESP32 and rebooted the node. A bench run caught it
    // exactly: the reboot banner appears in the log immediately after
    // "[RX1] seq=96090 emg=0", and the reboot then reset the node's
    // anti-replay counter, which the ICU correctly rejected as a replay
    // storm -- a second failure that looked unrelated to the first.
    //
    // The lesson is about the coupling, not the crash: a security-
    // critical pointer's validity was being maintained by an early
    // return several hundred lines away, with nothing marking the
    // dependency. Removing one guard silently invalidated the other.
    //
    // The lookup now runs for ANY packet that reaches this point.
    // Non-emergency packets are still dropped later by the relay gate;
    // this only ensures trust is valid for the ones that continue.
    if(true) {
        trust = findVehicle(vehicleID);
        // CHANGED v3: findVehicle() can now legitimately return NULL (it
        // refuses to evict a verified vehicle to make room for an
        // unverified one). Every caller must handle that.
        if(trust == NULL) {
            Serial.println("[SECURITY] No trust slot available - packet dropped");
            return;
        }
        if(certPresent) {
            bool certOK = verifyCertificate(vehicleID, certPubKey, certSignature, trust);
            if(!certOK) {
                securityStatus = SIG_CERT_REJECTED;
            } else {
                retryBufferedPackets(vehicleID);
            }
        }

        if(!trust->keyValid) {
            if(securityStatus != SIG_CERT_REJECTED) securityStatus = SIG_WAITING_CERT;
        } else {
            sigValid = verifySignature(rxBuffer, sizeof(TelemetryPayload), signature, trust->publicKey);
            securityStatus = sigValid ? SIG_VERIFIED : SIG_FAILED;
        }
    }

    String linkQuality = getLinkQuality(rssi, snr);
    float linkMargin = snr - snrFloorForSF(LORA_SF);

    // v3.6: accumulate IF-1 quality for the periodic [IF1 HEALTH] summary.
    // Live receptions only -- a retro replay is a buffered packet being
    // reprocessed, and counting it would double-count one radio event and
    // corrupt the trend.
    if(!isRetroReplay) {
        if1Packets++;
        if1SnrSum += snr;
        if(snr < if1SnrMin)  if1SnrMin  = snr;
        if(snr > if1SnrMax)  if1SnrMax  = snr;
        if(rssi < if1RssiMin) if1RssiMin = rssi;
        if(rssi > if1RssiMax) if1RssiMax = rssi;
        if(snr < IF1_LOW_SNR_DB) if1LowSnrPkts++;

        // v3.7: rolling window for trend visibility within a single run.
        if1WinSnr[if1WinIdx] = snr;
        if1WinIdx = (if1WinIdx + 1) % IF1_WINDOW_PKTS;
        if(if1WinCount < IF1_WINDOW_PKTS) if1WinCount++;
    }
    char istTimeStr[32];
    if (timestamp > 0) {
        formatEpochAsIST(timestamp, milliseconds, istTimeStr, sizeof(istTimeStr));
    } else {
        snprintf(istTimeStr, sizeof(istTimeStr), "NO GPS TIME FIX");
    }
    char altitudeStr[16];
    if (altValid) {
        snprintf(altitudeStr, sizeof(altitudeStr), "%d m", (int)altitude);
    } else {
        snprintf(altitudeStr, sizeof(altitudeStr), "NO FIX");
    }

    // ==================================================================
    // CHANGED v3.6: single-line machine-parseable record, emitted BEFORE
    // the human-readable block.
    //
    // TC-LN-001 v3.5 exposed a measurement problem, not a firmware one: the
    // serial capture fragmented mid-string ("** OVER 10% - not compliant
    // with an" / "y licence-exempt"), and "Certificate : YES (SIGNED)"
    // appeared truncated as N / Y / YE / "YES [" across the file. The
    // EVU log held 68 of ~94 frames it had actually sent. The PRR method
    // documented in the handover doc consequently returned 131.8%, which is
    // not a number, and anyone trusting it would conclude the link was
    // fine when they had in fact measured their terminal emulator.
    //
    // A 30-line block per packet at 115200 baud is the cause. This one line
    // survives a dropped chunk (a corrupted line fails to parse and is
    // visibly absent, rather than silently merging with its neighbour), and
    // it makes PRR a one-line grep instead of a multiline regex:
    //
    //   grep -o 'seq=[0-9]*' RDULOG.txt | cut -d= -f2 | sort -n | uniq
    //
    // Keep the verbose block for eyeballing on the bench; parse THIS.
    // ==================================================================
    Serial.printf("[RX1] seq=%lu veh=%s emg=%d gps=%d sec=%u retro=%d "
                  "rssi=%d snr=%.2f margin=%.2f epoch=%lu ms=%u leg=%u len=%u\n",
                  (unsigned long)seq, vehicleID, emergency ? 1 : 0,
                  gpsValid ? 1 : 0, (unsigned)securityStatus,
                  isRetroReplay ? 1 : 0, rssi, snr, linkMargin,
                  (unsigned long)timestamp, (unsigned)milliseconds,
                  (unsigned)testLeg, (unsigned)len);

    Serial.printf(
        "\n========== GREENWAVE RX ==========\n"
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
        "Sequence         : %lu\n"
        "Test Leg         : %u\n"
        "Timestamp (UTC)  : %lu\n"
        "Timestamp (IST)  : %s\n\n"
        "Packet Size      : %u bytes\n"
        "Certificate      : %s\n\n"
        "RSSI             : %d dBm\n"
        "SNR              : %.2f dB\n"
        "Link Margin      : %.2f dB\n"
        "Link Quality     : %s\n\n"
        "Security         : %u\n\n"
        "Free RAM         : %u bytes\n"
        "Uptime           : %s\n"
        "==================================\n",
        vehicleID, (unsigned)priority, emergency ? "YES" : "NO",
        gpsValid ? "YES" : "NO", siren ? "ACTIVE" : "OFF",
        latitude, longitude, altitudeStr, speed, heading,
        (unsigned long)seq, (unsigned)testLeg, (unsigned long)timestamp, istTimeStr,
        (unsigned)len, certPresent ? "YES" : "NO",
        rssi, snr, linkMargin, linkQuality.c_str(), (unsigned)securityStatus,
        ESP.getFreeHeap(), formatUptime().c_str()
    );

    // ------------------------------------------------------------
    // RELAY THE FALLING EDGE  (spec S2-08)
    //
    // This used to be a bare `if(!emergency) return;`, which discarded
    // every non-emergency packet at the relay.
    //
    // That deleted the most reliable release signal in the entire
    // system, one hop before the ICU could see it. When the driver
    // switches the siren off, the EVU immediately starts sending
    // emergency=0 -- an explicit, authenticated "I am done" from the
    // only party who actually knows. The old line threw those away, so
    // the ICU never saw the 1 -> 0 transition and had to wait out a
    // 20 s expiry timeout instead.
    //
    // A bench run showed exactly this: the EVU transmitted seven
    // emergency=0 packets and not one reached the ICU.
    //
    // Hard to catch in the field, because nothing looks broken. The
    // junction just stays held for twenty seconds after the ambulance
    // has finished with it, and every log reads normally.
    //
    // NOW: a non-emergency packet is relayed ONCE, only when the
    // previous packet from that vehicle WAS an emergency. That carries
    // the edge to the ICU without turning this relay into a general
    // vehicle tracker -- ordinary non-emergency traffic is still
    // dropped here, which is what keeps the 434.5 channel clear.
    // ------------------------------------------------------------

    static char     lastEmergVeh[8] = {0};
    static bool     lastEmergState  = false;

    bool sameVehicle = (strncmp(lastEmergVeh, vehicleID, 7) == 0);
    bool fallingEdge = sameVehicle && lastEmergState && !emergency;

    // Visible to the rate limiter further down, which must not be
    // allowed to discard it. See the comment there.
    gwFallingEdgeNow = fallingEdge;

    if (emergency) {
        strncpy(lastEmergVeh, vehicleID, 7);
        lastEmergVeh[7] = '\0';
        lastEmergState  = true;
    } else if (fallingEdge) {
        // Relay this one, then stop. Further non-emergency packets from
        // the same vehicle are dropped again.
        lastEmergState = false;
        Serial.println("[RELAY] emergency cleared -- forwarding the falling "
                       "edge so the ICU can release immediately");
    }

    if(!emergency && !fallingEdge) return;

    // Belt and braces. The block above now populates trust for every
    // packet that reaches here, but this dereference is the one that
    // panicked the node, and a null check costs nothing next to a
    // roadside unit rebooting mid-event.
    //
    // A reboot here is not a contained failure: it resets the node's
    // anti-replay counter, and the ICU then rejects everything it sends
    // until the ICU is itself restarted. One crash takes the node off
    // the network indefinitely.
    if(trust == NULL) {
        Serial.println("[SECURITY] no trust slot - packet dropped");
        return;
    }

    if(!trust->keyValid) {
        Serial.println(certPresent ? "[SECURITY] NO TRUSTED KEY - CERT REJECTED THIS PACKET" : "[SECURITY] NO TRUSTED KEY - WAITING FOR NEXT CERTIFICATE PACKET");
        bufferPacket(vehicleID, rxBuffer, len, rssi, snr);
        return;
    }

    if(!sigValid) {
        Serial.println("[SECURITY] BAD SIGNATURE");
        return;
    }

    // 3. REPLAY DETECTOR: Ignore replay logic if processing the retro-verification loop
    if(!isRetroReplay) {
        if(trust->sequenceStarted && seq <= trust->lastSequence) {
            Serial.println("[SECURITY] REPLAY BLOCKED");
            return;
        }
        // Advance tracking strictly for live streams
        trust->lastSequence = seq;
        trust->sequenceStarted = true;
    }

    trust->lastSeen = millis();

    // --- NEW FIX: DROP STALE RETRO-BUFFER GPS DATA ---
    if(isRetroReplay) {
        return; // Security baseline is updated; discard the stale payload.
    }
    // -------------------------------------------------

    vehicleLatitude = latitude;
    vehicleLongitude = longitude;
    vehicleAltitude = altitude;
    vehicleAltitudeValid = altValid;
    vehicleSpeed = speed;
    vehicleHeading = heading;
    currentVehicleMs = milliseconds; // Store for event processing

    if(!gpsValid) {
        Serial.println("[GPS INVALID]");
        return;
    }

    bool accepted = GEOFENCE_BYPASSED_FOR_SOP
        ? true
        : smartGeofenceDecision(vehicleLatitude, vehicleLongitude, vehicleSpeed, vehicleHeading);

    Serial.print("Geofence result: ");
    Serial.println(GEOFENCE_BYPASSED_FOR_SOP ? "BYPASSED_FOR_SOP" : (accepted ? "ACCEPTED" : "REJECTED"));

    if(!accepted) {
        Serial.println("[SMART GEO REJECT]");
        return;
    }

    vehicleETA = calculateETA(vehicleLatitude, vehicleLongitude, vehicleSpeed);
    motionState = "APPROACHING";
    lastEmergency = millis();

    // CHANGED v3.1: rate limit event generation. See EVENT_MIN_INTERVAL_MS.
    // Note lastEmergency is updated ABOVE this check -- suppressing a
    // redundant transmission must not also suppress the liveness timer.
    /*******************************************************************
     * FIX 1 : THE TWO NODES SPLIT THE RELAY DUTY.
     *
     * Both nodes hear the same ambulance packet and, until now, both
     * relayed it. The ICU discarded one of every pair:
     *
     *     [EVU] AMB_02 stale seq 97275 < 97278 -- ignored
     *
     * Half the relay airtime on a shared channel was spent transmitting
     * information that was thrown away on arrival. It was also the
     * largest single term in the duty-cycle measurement -- and the two
     * nodes were doing it simultaneously, having both heard the same
     * packet at the same moment, which is the worst possible timing.
     *
     * Now each node relays alternate sequence numbers: node 1 takes the
     * odd ones, node 2 the even ones. No coordination is needed -- each
     * node looks only at the sequence number in the packet it already
     * has.
     *
     * The ICU receives updates just as often as before. Each node
     * transmits half as much, and the two are never sending the same
     * packet at the same instant.
     *
     * TWO DELIBERATE EXCEPTIONS, because these must never be dropped:
     *
     *   the emergency FALLING EDGE -- carried by exactly one packet.
     *   Losing it means the junction stays held after the ambulance has
     *   finished (spec S2-08), which is the whole reason that signal was
     *   rescued in the first place.
     *
     *   a SINGLE-NODE approach -- if this node's partner is not present
     *   or not working, alternating would silently halve the ICU's
     *   update rate with nothing to show for it. A node cannot easily
     *   know its partner's health, so the safe default is: if in doubt,
     *   relay. Sending a duplicate costs airtime; dropping the only
     *   copy costs the event.
     *******************************************************************/

    bool myTurn = ((seq & 1u) == (uint32_t)((NODE_ID - 1) & 1));

    if (!myTurn && !gwFallingEdgeNow) {
        Serial.printf("[EVENT] seq %lu is the partner node's turn -- skipped\n",
                      (unsigned long)seq);
        return;
    }

    static unsigned long lastEventSent = 0;

    // ------------------------------------------------------------
    // THE FALLING EDGE BYPASSES THE RATE LIMIT.
    //
    // This limiter exists because the EVU transmits every 2 s and the
    // ICU does not need a relay that often -- suppressing roughly half
    // of a repeating "still approaching" message costs nothing, because
    // the next one arrives in two seconds.
    //
    // The falling edge is not a repeating message. It is carried by
    // EXACTLY ONE packet: the gate above forwards the first
    // emergency=0 and drops every one after it. If the limiter happens
    // to be inside its 4 s window when that single packet arrives, the
    // only copy in existence is discarded and the release signal is
    // gone permanently.
    //
    // That is a coin flip, and a bench run lost it -- the EVU sent ten
    // emergency=0 packets, the relay forwarded the edge, and the ICU
    // still never saw it.
    //
    // The failure is invisible in the field. The ICU falls back to a
    // 20 s expiry timeout, the junction stays held after the ambulance
    // has finished, and every log reads normally.
    //
    // One extra transmission per event, only when a vehicle stands
    // down. That is not a channel-budget concern; losing the signal is.
    // ------------------------------------------------------------

    if (lastEventSent != 0 &&
        (millis() - lastEventSent) < EVENT_MIN_INTERVAL_MS &&
        !gwFallingEdgeNow) {
        Serial.println("[EVENT SUPPRESSED - within rate limit]");
        return;
    }

    if (gwFallingEdgeNow &&
        lastEventSent != 0 &&
        (millis() - lastEventSent) < EVENT_MIN_INTERVAL_MS) {
        Serial.println("[EVENT] rate limit BYPASSED -- carrying the "
                       "emergency falling edge");
    }

    lastEventSent = millis();

    LoRaEventFrame event = buildLoRaEvent(vehicleID, rssi, snr, seq, emergency, priority,
                                          gpsValid, securityStatus, timestamp, accepted);
    queueFrame(&event, sizeof(event));
    Serial.println("[GREENWAVE EVENT CREATED]");
}

void initLoRa()
{
    SPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_SS
    );

    LoRa.setPins(
        LORA_SS,
        LORA_RST,
        LORA_DIO0
    );

    if(!LoRa.begin(AMBULANCE_FREQ))
    {
        Serial.println("[LORA FAILED]");
        loraHealthy = false;
        return;
    }

    //-------------------------
    // LoRa Configuration
    //-------------------------

    LoRa.setTxPower(EVU_LINK_TX_POWER_DBM);
    LoRa.setSyncWord(LORA_SYNC_WORD_EVU);

    // RANGE-TEST CONFIG: must match EVU2.ino exactly (SF/BW/CR/preamble/
    // sync word). SF7 -> SF9 for ~5 dB more link budget than the SF8 fix,
    // CR 4/5 -> 4/6 for a moderate FEC bump, longer preamble for more
    // reliable sync at the edge of range.
    // CHANGED v3.7: was a literal 9 while LORA_SF (used by snrFloorForSF()
    // for every printed link margin) was a separate #define, with a comment
    // asking a human to keep the two in sync. The very next planned
    // experiment is SF8 vs SF9. Change the setter, forget the #define, and
    // every link margin in the log is quietly wrong by 2.5 dB while the
    // banner still says SF9 -- during the run whose whole purpose is
    // deciding SF. One source of truth.
    LoRa.setSpreadingFactor(LORA_SF);

    LoRa.setSignalBandwidth(125E3);

    LoRa.setCodingRate4(6);

    LoRa.setPreambleLength(12);

    LoRa.enableCrc();

    //-------------------------

    LoRa.receive();

    loraHealthy = true;

    Serial.println("[LORA READY]");
    // v3.7: derived from the constants actually programmed, not retyped.
    Serial.printf("[LORA CONFIG] SF=%d BW=125kHz CR=4/6 Preamble=12 "
                  "SyncWord=0x%02X  snrFloor=%.1fdB\n",
                  LORA_SF, LORA_SYNC_WORD_EVU, snrFloorForSF(LORA_SF));
}

void loraTask(void *parameter) {
    Serial.println("[LORA TASK START]");
    initLoRa();
    Serial.println("[LORA TASK ENTER LOOP]");
    while(true) {
        // loraMutex also guards sendToICU()'s frequency retune (transportTask,
        // core 0). Without this, a heartbeat/event transmit could retune the
        // radio to CENTRAL_FREQ in the middle of this task reading an
        // in-flight EVU packet on the other core.
        if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            int size = LoRa.parsePacket();
            if(size) {
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                static uint8_t rxBuffer[255];
                size_t index=0;
                while(LoRa.available()) {
                    if(index < sizeof(rxBuffer)) rxBuffer[index++] = (uint8_t)LoRa.read();
                    else LoRa.read();
                }
                // Inside loraTask() loop, change the call to pass "false" for normal live transmissions:
                processLoRaPacket(rxBuffer, index, LoRa.packetRssi(), LoRa.packetSnr(), false);
                xSemaphoreGive(serialMutex);
            }
            xSemaphoreGive(loraMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/***********************************************************************
 * ACOUSTIC SENTINEL
 ***********************************************************************/
#define I2S_BCK_IO GPIO_NUM_4
#define I2S_WS_IO  GPIO_NUM_5
#define I2S_SD_IO  GPIO_NUM_6
#define SAMPLE_RATE 16000

i2s_chan_handle_t rx_handle=NULL;

// PATH B buffers. All heap-allocated once in tinymlTask(); none of this
// belongs on a task stack.
//   pcmBuffer   32000 int16 =  64000 B  raw 2 s window straight from I2S
//   featureInt8 40*161      =   6440 B  quantised tensor for the model
//   feScratch   gw_scratch_t = 157772 B front-end working memory
int16_t      *pcmBuffer   = NULL;
int8_t       *featureInt8 = NULL;
gw_scratch_t *feScratch   = NULL;

// Published in the acoustic log line. The front end runs its IIR in double,
// which the S3 emulates in software, and the TFLM arena was never measured
// before Path B -- these two numbers are how both stop being guesses.
volatile uint32_t lastFrontEndMs = 0;
volatile uint32_t lastInferMs    = 0;

/***********************************************************************
 * v3.9 ROADSIDE ACOUSTIC TUNING
 *
 * The old scheme was a single threshold (0.85) with fixed +3/-4 scoring
 * and CONFIRM_SCORE 15. Three problems for a roadside deployment:
 *
 *  1. SCORE_DECREMENT (4) was LARGER than SCORE_INCREMENT (3). One
 *     marginal window undid more than a good window gained. That was
 *     harmless when a window cost milliseconds; at 2.6 s per window it
 *     cost 3.5 s of net progress. Measured: detections fired at SCORE=17,
 *     not 15, meaning EIGHT windows (~21 s), not the five the constants
 *     imply. The arithmetic never changed; what it measured did.
 *
 *  2. All evidence weighed the same. A 0.99 window and a 0.86 window both
 *     scored +3, so the classifier's own confidence was discarded at
 *     exactly the point it was most useful.
 *
 *  3. One threshold cannot both resist traffic noise and hold onto a
 *     receding siren. Hysteresis needs two.
 *
 * NONE OF THESE NUMBERS ARE VALIDATED ON YOUR ROAD. The model's 4.1%
 * INT8 false-positive rate was measured on UrbanSound8K clips through a
 * different microphone. Every window is now logged (see [AIW] below) so
 * these can be set from site data instead of from this comment.
 ***********************************************************************/

// Evidence tiers. A window scores by HOW confident it was, not merely
// whether it crossed a line.
#define P_STRONG        0.92f   // unambiguous siren
#define P_MEDIUM        0.80f   // probable
#define P_HOLD          0.70f   // supports an episode, cannot start one

#define W_STRONG        5
#define W_MEDIUM        3
#define W_HOLD          1
#define W_MISS         (-3)     // smaller than W_STRONG on purpose (see 1 above)

#define CONFIRM_SCORE   12
#define SCORE_CEILING   24      // bounds how long a passing siren lingers

// A burst of horn/airbrake noise can span two consecutive windows. Three
// is the minimum that cannot be produced by one acoustic event, so score
// alone must never confirm -- MIN_CONFIRM_WINDOWS must also be satisfied.
#define MIN_CONFIRM_WINDOWS 3

// Legacy name kept: still the floor below which a window is scored a miss.
#define AI_THRESHOLD    P_HOLD

// How often to run inference. Compute measured at ~600 ms/window
// (front end ~445 ms + inference ~155 ms), so 1000 ms is ~60% of one
// core. Do not go below ~700 ms without re-measuring.
#define INFER_HOP_MS    1000UL

// Absolute input-level gate, applied BEFORE inference.
//
// The frozen Phase 5 front end peak-normalises every window and then takes
// power_to_db(ref=max). Absolute sound level is discarded TWICE, so on a
// quiet road at night the noise floor is amplified to full scale and
// presented to the model as though it were a loud signal. That is a
// structural false-positive path, not a tuning issue.
//
// DISABLED BY DEFAULT (0) because the correct value is a property of your
// microphone, its gain and its mounting, and I do not have those. Every
// window logs its RMS in [AIW]; collect an hour of site audio, look at the
// RMS distribution for genuine sirens versus quiet periods, and set this
// BELOW the quietest true siren you care about. Setting it from anything
// other than site data will cost you real detections.
#define MIN_WINDOW_RMS  0.0f

// CHANGED v3.5: how often to RE-ASSERT acoustic detection while the siren
// is still sounding. See tinymlTask() for why this exists.
//
// Sizing: the ICU holds acousticDetected for EVENT_TIMEOUT_MS (20000).
// 8000 gives 2.5 clean misses of margin on a link with no acknowledgement
// and no retry -- deliberately the same margin rule as HEARTBEAT_TIMEOUT_MS.
//
// Airtime cost: AcousticEventFrame is 24 B = ~247 ms at SF9/BW125/CR4-6
// (v4 grew it from 20 B), which fits the 560 ms EVU gap with room. One frame
// per 8 s per node is 3.1% duty on IF-2, and only while a siren is audible.
//
// If you change EVENT_TIMEOUT_MS in ICU.ino, change this with it. They are
// one decision expressed in two files.
#define ACOUSTIC_REASSERT_MS 8000UL

// v4 CONSISTENCY FIX: AcousticEventFrame grew 20 -> 24 bytes when
// FrameHeader gained session_epoch. HEARTBEAT_AIRTIME_MS and
// EVENT_AIRTIME_MS were updated to match; this frame's airtime was left as
// a bare literal 222 in two places inside printAirtimeBudget() and in the
// comment above. GreenwaveTypes.h states these MUST be kept in step -- so
// name it once, like the other two.
// 24 B at SF9/BW125/CR4-6/preamble 12 = ~247 ms.
// v5: AcousticEventFrame grew 24 -> 30 bytes (event_id + age_ms).
// 271.4 ms computed, plus margin.
// SF7 (FIX 3). 30-byte AcousticEventFrame computes to 86 ms; margin
// added. Was 305 at SF9.
#define ACOUSTIC_AIRTIME_MS   110UL

int sirenScore=0;
bool sirenActive=false;

// CHANGED v3.5: timestamp of the last acoustic frame QUEUED (not radiated --
// transportTask owns radiation and may defer it into a free window). Drives
// the periodic re-assert in tinymlTask(). 0 = none sent since boot.
unsigned long lastAcousticSent=0;

// CHANGED v3: 20 bytes instead of 136.
//
// The old acoustic event sent a full EventPacket -- 136 bytes, zero in
// all but four fields -- to convey one confidence value. That cost 861ms
// of airtime, three times over, to transmit 1 byte of actual meaning.
// This frame costs ~247ms (24 B since v4).
/***********************************************************************
 * v5 / PHASE 4 : ACOUSTIC EPISODE IDENTITY
 *
 * An "episode" is one continuous siren detection. Every report about
 * the same unbroken detection carries the same event_id; silence long
 * enough to end the episode starts a new one.
 *
 * WHY THIS IS NEEDED
 *
 * Without it, the periodic re-assert stream from the two nodes is just
 * a sequence of undifferentiated "siren now" pulses. The ICU cannot
 * tell one vehicle's sustained siren from a second vehicle arriving
 * behind it, and spec Scenario 1 section 9 forbids collapsing
 * concurrent sirens into a single event -- two ambulances converging on
 * one junction from different approaches is precisely the case where
 * getting it wrong matters most.
 *
 * Node-local, not global. The key is (lane_id, node_id, event_id); the
 * ICU correlates across nodes by time and geometry, not by identifier.
 * Two nodes hearing the SAME siren will assign it DIFFERENT event_ids,
 * and that is correct -- neither node knows what the other heard, and a
 * node that pretended to would be inventing a correlation the ICU
 * exists to perform.
 ***********************************************************************/

static uint32_t      currentEventId    = 0;   // 0 = no episode in progress
static uint32_t      eventIdCounter    = 0;
static unsigned long episodeFirstMs    = 0;   // first detection of this episode

// Best confidence seen during the current episode.
//
// Reported instead of the instantaneous window value, because the ICU
// uses confidence to decide whether evidence is strong enough to
// COMMIT -- a question about the episode, not about the current 125 ms.
// A siren briefly masked by a passing vehicle should not lose its
// commitment; the accumulator already models that, and the reported
// confidence should agree with it.
static float         episodePeakConf   = 0.0f;

AcousticEventFrame buildAcousticEvent(float confidence, bool sustained) {
    AcousticEventFrame f;
    memset(&f, 0, sizeof(f));

    f.hdr.proto_version   = GW_PROTO_VERSION;
    f.hdr.packet_type     = ACOUSTIC_EVENT_PACKET;
    f.hdr.intersection_id = INTERSECTION_ID;
    f.hdr.lane_id         = LANE_ID;
    f.hdr.node_id         = NODE_ID;
    f.hdr.counter         = 0;   // stamped at transmit time -- see radiateToICU()

    // 0.0-1.0 -> 0-255. One byte is plenty: the decision threshold is
    // 0.85 and nothing downstream needs more than ~0.4% resolution.
    if (confidence < 0.0f) confidence = 0.0f;
    if (confidence > 1.0f) confidence = 1.0f;
    f.confidence = (uint8_t)lround(confidence * 255.0f);

    f.flags = 0;
    if (micHealthy) f.flags |= ACF_MIC_OK;
    if (sustained)  f.flags |= ACF_SUSTAINED;

    // ---- v5: episode identity ----
    f.event_id = currentEventId;

    // Milliseconds from the FIRST detection of this episode to the
    // moment this frame was built.
    //
    // This exists because direction of travel is a TIME DELTA between
    // the two nodes, and measuring that delta from frame ARRIVAL at the
    // ICU measures the radio rather than the road. Both nodes share one
    // 434.5 channel with heartbeats and relays, and a frame can wait
    // seconds for a clear window.
    //
    // The scale of the problem: at 60 km/h across a 100 m separation the
    // true delta is about 6 s. Transport jitter of the same order would
    // not perturb that measurement, it would DOMINATE it -- and the
    // resulting error is not random noise, it is a wrong direction
    // classification, which is the one thing this measurement exists to
    // establish.
    //
    // Saturates rather than wrapping. A wrapped value would read as a
    // near-zero age and could manufacture an AMBIGUOUS classification --
    // the signature of two microphones hearing one loud siren -- out of
    // an episode that had simply lasted a long time.
    unsigned long age = millis() - episodeFirstMs;
    f.age_ms = (age > 65535UL) ? 65535 : (uint16_t)age;

    // NOT signed here. The counter is stamped at transmit time, and the
    // CMAC covers the counter, so signing must happen after stamping.
    // radiateToICU() does both.
    return f;
}

// Builds the I2S standard-mode configuration. One definition, used by
// both the initial bring-up and the recovery path, so the two can never
// drift apart -- a recovery that reconfigures the peripheral slightly
// differently from boot is worse than no recovery at all.
static i2s_std_config_t gwBuildMicConfig() {
    i2s_std_config_t config = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws   = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD_IO,
            .invert_flags = {.mclk_inv=false, .bclk_inv=false, .ws_inv=false}
        }
    };
    config.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    return config;
}

// Tears the I2S channel down and brings it back up.
//
// WHY THIS EXISTS
//
// The capture loop used to respond to a failed read by waiting 100 ms
// and retrying the SAME read, forever. If the channel itself has faulted
// -- a DMA stall, a clock glitch, a brownout on the microphone's supply
// -- retrying cannot fix it, so a transient fault became permanent until
// somebody power-cycled the node.
//
// A roadside unit that goes deaf until a site visit is, for this
// system's purposes, a failed node. It still heartbeats, still relays
// EVU packets, and contributes nothing acoustic -- which under the
// degraded-mode rules also denies its partner the confirmation it needs.
//
// Returns false if recovery fails; the caller keeps trying on a slower
// cadence rather than giving up.
//
// Deliberately does NOT use ESP_ERROR_CHECK: that aborts the chip on
// failure, which is the correct response during boot and precisely the
// wrong one here. Rebooting a node because its microphone hiccuped
// would take out the LoRa relay too.
static bool gwRecoverMicrophone() {
    Serial.println("[MIC] attempting I2S recovery...");

    if (rx_handle != NULL) {
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
    }

    i2s_chan_config_t chanCfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    if (i2s_new_channel(&chanCfg, NULL, &rx_handle) != ESP_OK) {
        Serial.println("[MIC] recovery FAILED at i2s_new_channel");
        rx_handle = NULL;
        return false;
    }

    i2s_std_config_t stdCfg = gwBuildMicConfig();

    if (i2s_channel_init_std_mode(rx_handle, &stdCfg) != ESP_OK) {
        Serial.println("[MIC] recovery FAILED at init_std_mode");
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
        return false;
    }

    if (i2s_channel_enable(rx_handle) != ESP_OK) {
        Serial.println("[MIC] recovery FAILED at channel_enable");
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
        return false;
    }

    Serial.println("[MIC] I2S recovered");
    return true;
}

void initMicrophone() {
    i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&channelConfig, NULL, &rx_handle));
    i2s_std_config_t config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_SD_IO,
            .invert_flags = {.mclk_inv=false, .bclk_inv=false, .ws_inv=false}
        }
    };
    config.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &config));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    Serial.println("[MIC READY]");
}

/***********************************************************************
 * CONTINUOUS AUDIO CAPTURE  (v3.9)
 *
 * THE BUG THIS FIXES: inference takes ~600 ms, and while it runs nothing
 * reads I2S, so the DMA ring overflows and that audio is gone. Successive
 * 2-second windows were therefore separated by a ~600 ms REAL-TIME HOLE.
 *
 * Nothing detected this, because each window was internally contiguous and
 * looked perfectly normal. It only becomes fatal the moment you try to
 * slide the window: shifting the buffer would splice two non-adjacent
 * seconds together and hand the model a spectrogram with a discontinuity
 * in the middle -- something it has never seen in training.
 *
 * So capture is now a separate producer task that reads I2S continuously
 * into a ring buffer. tinymlTask snapshots the newest 2 seconds whenever
 * it wants one. Audio is never lost, and windows can overlap honestly.
 ***********************************************************************/
#define AUDIO_CHUNK_SAMPLES 2000              // 125 ms per I2S read

// ---- microphone fault detection and recovery ----
//
// [FIELD] These bound how long a node may be deaf before it says so.
// Each chunk is 125 ms, so the silence threshold below is about two
// seconds -- long enough that a genuine momentary dropout does not
// raise an alarm, short enough that a dead microphone is reported
// within one heartbeat interval.

// Consecutive failed reads before attempting a channel restart.
#define MIC_FAIL_BEFORE_RECOVERY   8      // ~1 s of failures

// Once past that, retry recovery this often (in failed reads) rather
// than hammering the peripheral every 100 ms.
#define MIC_RECOVERY_RETRY_EVERY   40     // ~5 s between attempts

// Consecutive all-zero chunks before the microphone is declared dead.
#define MIC_SILENT_CHUNKS_FAIL     16     // ~2 s of pure silence

// Once declared dead, attempt a channel restart this often.
#define MIC_SILENT_RECOVERY_EVERY  240    // ~30 s between attempts

// ---- per-junction geometry, held in NVS ----
//
// Loaded at boot, overridden by `dist <metres>` on the serial console.
// Survives reflashing, so updating firmware does not lose the survey.
static uint16_t nodeDistanceM   = NODE_DISTANCE_FALLBACK_M;
static bool     nodeSurveyed    = NODE_DISTANCE_SURVEYED_FALLBACK;
static bool     nodeFromNvs     = false;

static uint32_t micReadFails    = 0;
static uint32_t micSilentChunks = 0;

static int16_t          *ringBuffer = NULL;   // GW_N_SAMPLES, 64000 B
static volatile uint32_t ringHead   = 0;      // next write index
static volatile uint32_t ringFilled = 0;      // samples written since boot
static SemaphoreHandle_t ringMutex  = NULL;

void audioCaptureTask(void *parameter) {
    static int16_t chunk[AUDIO_CHUNK_SAMPLES];
    Serial.println("[AUDIO TASK START]");
    while (true) {
        size_t bytesRead = 0;
        esp_err_t r = i2s_channel_read(rx_handle, chunk,
                                       sizeof(chunk), &bytesRead,
                                       pdMS_TO_TICKS(1000));

        // ------------------------------------------------------------
        // READ FAILURE -> RECOVER, DO NOT JUST RETRY.
        //
        // This used to wait 100 ms and retry the same read indefinitely.
        // If the channel itself has faulted, retrying cannot fix it, so
        // a transient glitch left the node permanently deaf until
        // somebody power-cycled it.
        // ------------------------------------------------------------
        if (r != ESP_OK || bytesRead == 0) {
            micHealthy = false;
            micReadFails++;

            if (micReadFails == MIC_FAIL_BEFORE_RECOVERY ||
                (micReadFails > MIC_FAIL_BEFORE_RECOVERY &&
                 (micReadFails % MIC_RECOVERY_RETRY_EVERY) == 0)) {

                Serial.printf("[MIC] %lu consecutive read failures (err %d)\n",
                              (unsigned long)micReadFails, (int)r);

                if (gwRecoverMicrophone()) {
                    micReadFails = 0;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        micReadFails = 0;

        // ------------------------------------------------------------
        // A SUCCESSFUL READ IS NOT A WORKING MICROPHONE.
        //
        // The old code set micHealthy = true here and stopped thinking.
        // But I2S happily returns buffers full of zeros when the data
        // line is dead, unplugged, or the microphone has lost power --
        // the peripheral is fine, it is just clocking in nothing.
        //
        // A bench node showed exactly this: heartbeats reporting MIC=1
        // while its reported noise floor sat at 0. The node believed it
        // was healthy and was completely deaf, which is the specific
        // fault spec defect S1-04 is about -- and the one a two-node
        // design is least able to notice, because the partner keeps
        // working and the approach merely looks quiet.
        //
        // The ICU already catches it from the noise floor. The NODE
        // should catch it too, and faster: this is a per-chunk test
        // against exact silence, not a slow filtered average.
        //
        // Exact zero is the right test. Real ambient, even in a silent
        // room, dithers around zero from thermal noise and the mic's own
        // self-noise. A chunk of IDENTICAL zeros is not a quiet road; it
        // is no signal at all.
        // ------------------------------------------------------------

        size_t n = bytesRead / sizeof(int16_t);

        // Two distinct fault signatures, plus one this CANNOT catch.
        //
        //  allZero  -- every sample exactly 0. An absent microphone whose
        //              data line is held LOW.
        //
        //  allSame  -- every sample the same non-zero value. A latched
        //              line, a stuck bit, or a microphone that powered up
        //              and froze. Real audio never holds one exact value
        //              for two seconds, so this is unambiguous.
        //
        // WHAT NEITHER CATCHES: a FLOATING data line.
        //
        // A disconnected CMOS input is not silent. It picks up mains hum,
        // the board's own switching noise and crosstalk from the adjacent
        // clock lines, producing samples with real amplitude, real
        // variance and broadband structure. A bench test with BOTH
        // microphones removed showed one node reading exact zeros and
        // reporting mic:FAIL, and the other reading floating noise at a
        // completely plausible ambient level and reporting mic:OK.
        //
        // No sample-domain test separates that from a quiet site. Any
        // threshold aggressive enough to reject floating noise also
        // rejects a working microphone on a genuinely quiet road -- and
        // that error is worse, because it disables a healthy node and
        // denies its partner confirmation.
        //
        // [HARDWARE] Fit a ~10k pull-down on the I2S data line. An absent
        // microphone then reads exact zeros deterministically and the
        // allZero test above catches it every time. One resistor turns a
        // probabilistic fault into a certain one.
        bool allZero = true;
        bool allSame = true;
        int16_t first = (n > 0) ? chunk[0] : 0;

        for (size_t i = 0; i < n; i++) {
            if (chunk[i] != 0)     allZero = false;
            if (chunk[i] != first) allSame = false;
            if (!allZero && !allSame) break;
        }

        // A latched non-zero line is treated exactly like silence: the
        // node is not hearing anything, whatever value it is repeating.
        if (allSame && !allZero) {
            if (micSilentChunks == MIC_SILENT_CHUNKS_FAIL - 1) {
                Serial.printf("[MIC] *** every sample identical (%d) -- the "
                              "data line is LATCHED, not carrying audio ***\n",
                              (int)first);
            }
            allZero = true;   // share the counter and recovery path
        }

        if (allZero) {
            if (micSilentChunks < 0xFFFFFFFF) micSilentChunks++;

            if (micSilentChunks == MIC_SILENT_CHUNKS_FAIL) {
                Serial.printf("[MIC] *** %d consecutive all-zero chunks -- "
                              "the I2S reads are succeeding but the "
                              "microphone is producing NOTHING. Check the "
                              "data line, the supply, and the connector. ***\n",
                              MIC_SILENT_CHUNKS_FAIL);
            }

            if (micSilentChunks >= MIC_SILENT_CHUNKS_FAIL) {
                micHealthy = false;

                // Try a channel restart periodically. If the fault is in
                // the peripheral this fixes it; if the microphone is
                // physically disconnected it will not, and the node goes
                // on reporting itself unhealthy -- which is the honest
                // outcome and exactly what the ICU needs to hear.
                if ((micSilentChunks % MIC_SILENT_RECOVERY_EVERY) == 0) {
                    gwRecoverMicrophone();
                }
            }
        } else {
            if (micSilentChunks >= MIC_SILENT_CHUNKS_FAIL) {
                Serial.printf("[MIC] signal returned after %lu silent chunks\n",
                              (unsigned long)micSilentChunks);
            }
            micSilentChunks = 0;
            micHealthy      = true;
        }

        if (xSemaphoreTake(ringMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (size_t i = 0; i < n; i++) {
                ringBuffer[ringHead] = chunk[i];
                ringHead = (ringHead + 1) % GW_N_SAMPLES;
            }
            if (ringFilled < GW_N_SAMPLES) {
                ringFilled += n;
                if (ringFilled > GW_N_SAMPLES) ringFilled = GW_N_SAMPLES;
            }
            xSemaphoreGive(ringMutex);
        }
    }
}

// Copy the newest GW_N_SAMPLES into pcmBuffer, oldest sample first.
// Returns false until the ring has filled once after boot.
bool snapshotAudio() {
    if (ringFilled < GW_N_SAMPLES) return false;
    if (xSemaphoreTake(ringMutex, pdMS_TO_TICKS(200)) != pdTRUE) return false;

    uint32_t start = ringHead;                       // oldest sample
    uint32_t first = GW_N_SAMPLES - start;
    memcpy(pcmBuffer,         ringBuffer + start, first * sizeof(int16_t));
    memcpy(pcmBuffer + first, ringBuffer,         start * sizeof(int16_t));

    xSemaphoreGive(ringMutex);
    return true;
}

// RMS of the raw window, before any normalisation. This is the only place
// absolute sound level survives -- the front end discards it.
float windowRms(const int16_t *pcm, size_t n) {
    double acc = 0.0;
    for (size_t i = 0; i < n; i++) {
        double v = (double)pcm[i] / 32768.0;
        acc += v * v;
    }
    return (float)sqrt(acc / (double)n);
}

// Full Path B chain:
//   ring snapshot -> frozen Phase 5 front end -> INT8 tensor -> DS-CNN -> p
//
// Returns false only on a capture or interpreter FAILURE. A low score is a
// successful inference, not an error -- collapsing the two would make a
// dead microphone indistinguishable from silence.
volatile float lastWindowRms = 0.0f;


/***********************************************************************
 * v5 / PHASE 4 : MICROPHONE SELF-TEST STATE
 ***********************************************************************/

// Slow-moving estimate of ambient level, updated ONLY while no siren
// episode is in progress.
//
// Gating on that matters: a siren is loud, and folding it into the
// ambient estimate would raise the floor every time the node did its
// job. Over a busy shift the floor would climb until the node declared
// itself saturated -- a self-test that fails because the system is
// working is worse than no self-test.
volatile float micNoiseFloorRms = 0.0f;

// Set once enough quiet windows have been averaged for the estimate to
// mean anything. Before that the node reports "not yet known" rather
// than reporting an unsettled number that reads as a dead microphone
// during the first seconds after boot.
volatile bool  micNoiseFloorValid = false;

// Free-running counter, incremented once per completed inference.
//
// Proves the DETECTION path is alive, not merely the radio path. A hung
// or crashed inference task alongside a healthy transport task still
// heartbeats perfectly: that node is deaf and looks entirely fine. The
// ICU watches this value across consecutive heartbeats -- unchanged
// means the node is not listening, whatever else it says about itself.
//
// Wraps freely. The ICU compares for INEQUALITY, never magnitude, so a
// wrap is indistinguishable from ordinary progress and needs no handling.
volatile uint16_t inferenceLiveness = 0;

// Converts normalised RMS (0.0-1.0) to the 0-255 byte on the wire.
//
// Logarithmic, because loudness is. A linear byte would spend almost
// its whole range on levels far above ambient and compress the entire
// interesting region -- the difference between a working microphone and
// a dead one -- into the bottom two or three counts.
//
// Maps -100 dBFS..0 dBFS onto 0..255. Exact zero returns 0, which is
// reserved for "silent or unmeasured" and is precisely the reading a
// disconnected microphone produces.
static uint8_t encodeNoiseFloor(float rms) {
    if (rms <= 0.0f) return 0;

    float db = 20.0f * log10f(rms);          // -inf .. 0
    if (db < -100.0f) db = -100.0f;
    if (db > 0.0f)    db = 0.0f;

    float scaled = (db + 100.0f) * (255.0f / 100.0f);

    // Never return 0 for a real measurement, however faint -- 0 means
    // "no measurement" and the two must stay distinguishable.
    int v = (int)lroundf(scaled);
    if (v < 1)   v = 1;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

// FNV-1a over this node's entire identity and geometry.
//
// The ICU compares it against what it expects for this node. What this
// catches, and nothing else does, is a node flashed with a NEIGHBOURING
// node's configuration: every field is individually plausible, so no
// range check fires, and the node reports confidently and accurately
// about the wrong piece of road.
//
// Computed at runtime from the same macros the rest of the file uses,
// so it cannot drift from the values actually compiled in.
static uint32_t computeConfigHash() {
    uint32_t h = 2166136261UL;

    uint32_t fields[] = {
        (uint32_t)INTERSECTION_ID,
        (uint32_t)LANE_ID,
        (uint32_t)NODE_ID,
        (uint32_t)NODE_APPROACH_ID,
        (uint32_t)nodeDistanceM,
        (uint32_t)(nodeSurveyed ? 1 : 0),
        (uint32_t)GW_PROTO_VERSION,
        (uint32_t)GW_KEY_SET_ID
    };

    for (unsigned i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        uint32_t v = fields[i];
        for (int b = 0; b < 4; b++) {
            h ^= (v >> (b * 8)) & 0xFF;
            h *= 16777619UL;
        }
    }
    return h;
}

bool runSirenInference(float &confidence) {
    if (!snapshotAudio()) return false;

    lastWindowRms = windowRms(pcmBuffer, GW_N_SAMPLES);
    if (MIN_WINDOW_RMS > 0.0f && lastWindowRms < MIN_WINDOW_RMS) {
        confidence = 0.0f;          // below the level gate: a real, low, result
        lastFrontEndMs = 0;
        lastInferMs    = 0;
        return true;
    }

    uint32_t t0 = millis();
    gw_frontend_run(pcmBuffer, GW_N_SAMPLES, featureInt8, feScratch);
    uint32_t t1 = millis();

    float p = gw_model_infer(featureInt8);
    uint32_t t2 = millis();

    lastFrontEndMs = t1 - t0;
    lastInferMs    = t2 - t1;

    if (p < 0.0f) return false;      // interpreter error
    confidence = p;
    return true;
}

void tinymlTask(void *parameter) {
    Serial.println("[TINYML TASK START]");
    
    // PSRAM first for the two large buffers -- 157 KB of scratch plus 64 KB
    // of PCM will not sit comfortably in internal SRAM alongside the TFLM
    // arena. featureInt8 is small and hot, so it stays internal.
    pcmBuffer = (int16_t*)heap_caps_malloc(GW_N_SAMPLES * sizeof(int16_t),
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (pcmBuffer == NULL)
        pcmBuffer = (int16_t*)heap_caps_malloc(GW_N_SAMPLES * sizeof(int16_t),
                                               MALLOC_CAP_8BIT);

    feScratch = (gw_scratch_t*)heap_caps_malloc(sizeof(gw_scratch_t),
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (feScratch == NULL)
        feScratch = (gw_scratch_t*)heap_caps_malloc(sizeof(gw_scratch_t),
                                                    MALLOC_CAP_8BIT);

    featureInt8 = (int8_t*)heap_caps_malloc(GW_N_MELS * GW_N_FRAMES,
                                            MALLOC_CAP_8BIT);

    // v3.9: ring buffer for continuous capture. Internal RAM preferred --
    // the producer touches it every 125 ms and the consumer memcpys all of
    // it every second, so PSRAM latency here is paid constantly.
    ringBuffer = (int16_t*)heap_caps_malloc(GW_N_SAMPLES * sizeof(int16_t),
                                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (ringBuffer == NULL)
        ringBuffer = (int16_t*)heap_caps_malloc(GW_N_SAMPLES * sizeof(int16_t),
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ringMutex = xSemaphoreCreateMutex();

    if (pcmBuffer == NULL || feScratch == NULL || featureInt8 == NULL ||
        ringBuffer == NULL || ringMutex == NULL) {
        Serial.printf("[AUDIO MEMORY ERROR] pcm=%p scratch=%p(%u B) feat=%p\n",
                      pcmBuffer, feScratch, (unsigned)sizeof(gw_scratch_t),
                      featureInt8);
        if (pcmBuffer)   heap_caps_free(pcmBuffer);
        if (feScratch)   heap_caps_free(feScratch);
        if (featureInt8) heap_caps_free(featureInt8);
        if (ringBuffer)  heap_caps_free(ringBuffer);
        micHealthy = false;
        vTaskDelete(NULL);
    }

    Serial.printf("[AI] buffers: pcm=%u B ring=%u B scratch=%u B feat=%u B\n",
                  (unsigned)(GW_N_SAMPLES * sizeof(int16_t)),
                  (unsigned)(GW_N_SAMPLES * sizeof(int16_t)),
                  (unsigned)sizeof(gw_scratch_t),
                  (unsigned)(GW_N_MELS * GW_N_FRAMES));

    // Loads the model, verifies its quantisation parameters against
    // gw_frontend_tables.h, and prints the measured arena size.
    //
    // FAILS CLOSED. If the model's scale or zero point disagree with what the
    // front end produces, this node runs deaf rather than running wrong -- a
    // silent mismatch there is exactly what the golden vectors exist to catch,
    // and it must not survive boot.
    if (!gw_model_init()) {
        Serial.println("[AI] MODEL INIT FAILED - acoustic path disabled");
        micHealthy = false;
        vTaskDelete(NULL);
    }

    initMicrophone();

    // Producer must be running before the first snapshot is attempted.
    // Priority 3 (above tinymlTask at 2): losing audio is unrecoverable,
    // being late with an inference is not. Core 1, alongside loraTask,
    // because core 0 already carries the front end and the transport.
    xTaskCreatePinnedToCore(audioCaptureTask, "Audio", 4096, NULL, 3, NULL, 1);

    Serial.printf("[AI] tuning: strong>=%.2f(+%d) med>=%.2f(+%d) hold>=%.2f(+%d) "
                  "miss(%d) confirm=%d minWin=%d hop=%lums rmsGate=%.4f\n",
                  P_STRONG, W_STRONG, P_MEDIUM, W_MEDIUM, P_HOLD, W_HOLD,
                  W_MISS, CONFIRM_SCORE, MIN_CONFIRM_WINDOWS,
                  INFER_HOP_MS, MIN_WINDOW_RMS);
    Serial.println("[AI READY]");

    // v3.9: confidence-weighted, hysteretic scoring with a hard minimum
    // window count. See the tuning block above for why each part exists.
    uint8_t  confirmWindows = 0;      // windows >= P_HOLD in this episode
    uint32_t windowSeq      = 0;
    uint32_t nextInferAt    = 0;

    while(true) {
        // Fixed cadence. snapshotAudio() always returns the newest 2 s, so
        // the hop is set here rather than by how long the last inference
        // took -- otherwise the decision rate would drift with CPU load.
        uint32_t now = millis();
        if ((int32_t)(now - nextInferAt) < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        nextInferAt = now + INFER_HOP_MS;

        float confidence = 0;
        bool  ok = runSirenInference(confidence);

        if (!ok) {
            // Capture or interpreter failure. NOT the same as a low score:
            // decaying the episode here would let a failing microphone look
            // exactly like a quiet road.
            static unsigned long lastFailLog = 0;
            if (millis() - lastFailLog > 5000) {
                lastFailLog = millis();
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                Serial.printf("[AI] inference unavailable (mic=%d ringFilled=%lu)\n",
                              micHealthy ? 1 : 0, (unsigned long)ringFilled);
                xSemaphoreGive(serialMutex);
            }
            continue;
        }

        windowSeq++;

        // ---- v5: liveness + ambient tracking --------------------------
        //
        // Incremented here, after a SUCCESSFUL inference, and never on
        // the failure path above. That placement is the point: if the
        // capture or the interpreter is broken this counter stops moving
        // even though the task is still looping, and the ICU sees a node
        // that is running but not listening.
        inferenceLiveness++;

        // Update the ambient estimate only while nothing is being
        // detected. sirenScore > 0 means an episode is in progress, and
        // folding a siren into "ambient" would raise the floor every
        // time the node did its job.
        if (sirenScore == 0 && lastWindowRms > 0.0f) {
            if (!micNoiseFloorValid) {
                // Seed on the first quiet window rather than easing up
                // from zero. Starting at zero and filtering toward the
                // true level would spend the first minute after boot
                // reporting a floor low enough to look like a dead
                // microphone.
                micNoiseFloorRms   = lastWindowRms;
                micNoiseFloorValid = true;
            } else {
                // Slow EMA, alpha = 1/64. Ambient changes over minutes
                // (traffic, weather); a fast filter would track
                // individual passing vehicles and the value would say
                // more about the last truck than about the microphone.
                micNoiseFloorRms += (lastWindowRms - micNoiseFloorRms) / 64.0f;
            }
        }

        // ---- weight this window by how confident it actually was -------
        int delta;
        const char *tier;
        if      (confidence >= P_STRONG) { delta = W_STRONG; tier = "STRONG"; }
        else if (confidence >= P_MEDIUM) { delta = W_MEDIUM; tier = "MEDIUM"; }
        else if (confidence >= P_HOLD)   { delta = W_HOLD;   tier = "HOLD";   }
        else                             { delta = W_MISS;   tier = "-";      }

        if (delta > 0) { if (confirmWindows < 255) confirmWindows++; }

        sirenScore += delta;
        if (sirenScore > SCORE_CEILING) sirenScore = SCORE_CEILING;
        if (sirenScore < 0)             sirenScore = 0;

        if (sirenScore == 0) confirmWindows = 0;   // episode over

        // ---- PER-WINDOW FIELD RECORD ----------------------------------
        // This is the whole point of the roadside instrumentation. Every
        // window, detected or not, one grep-able line. Without it you are
        // tuning P_STRONG and MIN_WINDOW_RMS from a comment in this file
        // rather than from your own road.
        //
        //   grep '\[AIW\]' log.txt | awk '{print $3}' | cut -d= -f2 | sort -n
        //
        // Set MIN_WINDOW_RMS below the quietest true siren you see here,
        // and P_STRONG above the loudest false positive.
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.printf("[AIW] n=%lu p=%.4f rms=%.5f tier=%s d=%+d score=%d win=%u "
                      "act=%d fe=%lums inf=%lums\n",
                      (unsigned long)windowSeq, confidence, lastWindowRms, tier,
                      delta, sirenScore, (unsigned)confirmWindows,
                      sirenActive ? 1 : 0,
                      (unsigned long)lastFrontEndMs, (unsigned long)lastInferMs);
        xSemaphoreGive(serialMutex);

        // ---- confirmation ---------------------------------------------
        // BOTH conditions. Score alone is not enough: a horn or an air
        // brake can span two consecutive windows and, at W_STRONG each,
        // would reach 10 -- close to CONFIRM_SCORE. Requiring three
        // separate supporting windows means no single acoustic event can
        // confirm on its own.
        bool confirmed = (sirenScore >= CONFIRM_SCORE) &&
                         (confirmWindows >= MIN_CONFIRM_WINDOWS);

        if (confirmed) {
            // Refresh the local liveness timer so the 40 s [EVENT RESET]
            // in transportTask() does not tear down siren state under a
            // genuinely sustained siren.
            lastEmergency = millis();

            // Rising edge, then re-assert every ACOUSTIC_REASSERT_MS for as
            // long as the episode holds. The ICU's acousticDetected is a
            // latch with a timeout; the node that can still hear the siren
            // is the only thing that knows to re-arm it.
            bool firstAssert = !sirenActive;

            // Open a new episode on the rising edge, before the frame is
            // built -- buildAcousticEvent() reads both of these.
            if (firstAssert) {
                eventIdCounter++;
                currentEventId = eventIdCounter;
                episodeFirstMs = millis();
                episodePeakConf = 0.0f;
            }

            // Peak confidence across the episode, not the latest window.
            if (confidence > episodePeakConf) episodePeakConf = confidence;

            if (firstAssert ||
                (millis() - lastAcousticSent) >= ACOUSTIC_REASSERT_MS) {

                // ---- what SUSTAINED and CONFIDENCE actually report ----
                //
                // sustained was previously `!firstAssert`, which meant
                // "this is not the first frame of the episode" -- a
                // statement about frame ordering, not about the signal.
                //
                // And confidence was the CURRENT window's value. Those
                // two together produced the anomaly seen twice on the
                // bench: "conf=0.02 sust=1", a node asserting a
                // detection was sustained while being 98% sure the
                // current window was not a siren.
                //
                // That happens legitimately: sirenScore is an
                // accumulator, so it can still exceed CONFIRM_SCORE from
                // earlier strong windows while the latest window scores
                // near zero -- a siren momentarily masked by a passing
                // truck, for instance. The episode really is still live.
                // The reporting was just describing the wrong thing.
                //
                // ACF_SUSTAINED now means what the ICU assumes it means:
                // the episode has been supported by at least
                // MIN_CONFIRM_WINDOWS separate windows AND still holds a
                // confirming score. That is a claim about evidence, and
                // it is the claim Phase 5 carries into the evidence
                // tuple.
                //
                // Confidence now reports the episode's PEAK rather than
                // the current window. The ICU uses it to gate COMMIT,
                // and the question there is "how good was the evidence
                // for this vehicle", not "how good is this millisecond".
                // Reporting a momentary dip would demote a well
                // evidenced siren to PREPARE for the duration of a
                // passing truck.
                bool trulySustained = (confirmWindows >= MIN_CONFIRM_WINDOWS) &&
                                      (sirenScore >= CONFIRM_SCORE);

                float reportConf = (episodePeakConf > confidence)
                                 ? episodePeakConf : confidence;

                AcousticEventFrame event =
                    buildAcousticEvent(reportConf, trulySustained);
                queueFrame(&event, sizeof(event));
                lastAcousticSent = millis();
                sirenActive = true;

                xSemaphoreTake(serialMutex, portMAX_DELAY);
                Serial.printf("%s CONF=%.4f SCORE=%d WINDOWS=%u rms=%.5f "
                              "fe=%lums inf=%lums\n",
                              firstAssert ? "[SIREN DETECTED]" : "[SIREN SUSTAINED]",
                              confidence, sirenScore, (unsigned)confirmWindows,
                              lastWindowRms,
                              (unsigned long)lastFrontEndMs,
                              (unsigned long)lastInferMs);
                xSemaphoreGive(serialMutex);
            }
        } else if (sirenScore == 0 && sirenActive) {
            sirenActive = false;

            // Close the episode. The next detection gets a NEW event_id,
            // which is how the ICU tells a second vehicle from the same
            // vehicle still sounding.
            //
            // currentEventId is deliberately NOT reset to 0 here: the
            // ICU may still be holding a track for this episode, and
            // zeroing the identifier while that track is live would
            // orphan it. eventIdCounter only ever increments, so the
            // next episode is unambiguous either way.
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.printf("[SIREN CLEARED] event=%lu duration=%lums\n",
                          (unsigned long)currentEventId,
                          (unsigned long)(millis() - episodeFirstMs));
            xSemaphoreGive(serialMutex);
        }
    }
}

/***********************************************************************
 * TRANSPORT
 ***********************************************************************/
#define BATTERY_ADC_PIN 7
uint8_t batteryLevel=0;
float filteredBattery=-1;

// Single SX1278 shared between two links: AMBULANCE_FREQ/0xF3 (listening
// for the EVU) and CENTRAL_FREQ/0xA5 (talking to the ICU, now 434.5MHz
// as of the Tier-A isolation fix). Only one of those can be active at a
// time, so this briefly steals the radio, retunes, sends, and retunes
// back. loraMutex already existed (declared + created in setup()) but
// was never actually taken anywhere -- this is what it was for.
// loraTask() must also take it around its parsePacket()/read() calls
// (see below) or the two tasks will race on the same SPI/radio
// registers from two different cores.
//
// Known tradeoff of a single-radio design: while this function is
// mid-transmission (retuned off AMBULANCE_FREQ), an incoming EVU packet
// physically cannot be received. Time-on-air at SF9/125kHz for a
// heartbeat/event-sized payload is on the order of tens-to-~150ms.
// TIER-A ISOLATION FIX: transportTask() now gates heartbeat transmission
// (the frequent, non-urgent traffic) on icuTransmitWindowOpen(), landing
// it in the gap between predicted EVU arrivals instead of at an
// arbitrary moment. Emergency event packets are deliberately NOT gated --
// a live detection is time-critical, and deferring it to dodge a
// millisecond-scale radio window is the wrong tradeoff. The residual
// EVU-miss risk during an event transmission should be quantified by a
// dedicated dual-channel isolation test (see SOP) rather than assumed
// away.
// CHANGED v3. Three things were wrong here.
//
// 1. NO MEDIA ACCESS CONTROL. Six nodes shared one channel and simply
//    transmitted whenever they felt like it. Added Channel Activity
//    Detection with randomised backoff, so a node that hears another
//    node mid-transmission waits instead of colliding with it.
//
// 2. TX POWER WAS NEVER SET on this link, so it ran at whatever the LoRa
//    library defaults to. Now set explicitly. Radiating harder than the
//    link needs wastes battery and makes the city-scale channel
//    congestion problem worse, because a node's frames reach further and
//    interfere with more neighbouring intersections.
//
// 3. THE RETURN VALUE LIED. LoRa.endPacket() confirms that the modem
//    finished radiating. It is not delivery confirmation -- IF-2 is
//    simplex, nobody acknowledges anything. The old code logged
//    "[EVENT SENT ICU]" which reads like delivery. Renamed to RADIATED
//    so nobody debugging this at 2am believes the ICU got it.
// =====================================================
// PHASE 6.2 : DOWNLINK ACK -- WITHDRAWN
//
// A downlink transmitter lived here and has been removed.
//
// It was technically sound: the radio is already tuned to
// AMBULANCE_FREQ when listening, the EVU leaves a 960 ms gap between
// transmissions, and a 305 ms acknowledgement fitted inside it with
// margin.
//
// It was removed because the SECURITY ARCHITECTURE IS DELIBERATELY
// ONE-DIRECTIONAL. The vehicle signs and the roadside verifies; nothing
// travels back, and the EVU holds no key able to authenticate an
// inbound frame. Any acknowledgement this node transmitted would be
// forgeable by anyone with a LoRa module.
//
// That is a poor trade. A forged status in an ambulance cab is worse
// than an empty one, because a crew with no confirmation KNOWS they
// have none.
//
// It also cost listening time on the uplink that carries the
// ambulance's position -- spending a safety-critical resource on an
// advisory display that could not be trusted.
//
// Reinstating it requires the vehicle to verify a roadside signature,
// which means distributing the CA public key to every EVU and issuing
// certificates to every RDU. That is an architectural decision, not a
// firmware one.
// =====================================================

bool radiateToICU(uint8_t *data, size_t length) {
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.println("[ICU TX] SKIPPED - RADIO BUSY");
        xSemaphoreGive(serialMutex);
        icuTxSkipped++;
        return false;
    }

    // ------------------------------------------------------------
    // FIX 5 : PER-NODE TRANSMIT STAGGER ON THE ICU LINK.
    //
    // Both nodes hear the same EVU packet at the same instant, and both
    // react to it in the same few milliseconds. The relay split means
    // they carry alternate sequence numbers -- but heartbeats, acoustic
    // reports and retries are not alternated, and those still coincide.
    //
    // A bench run measured it: 58 frames radiated by the pair, 46
    // received by the ICU. Both nodes losing a similar share is the
    // signature of two transmitters overlapping, not of one weak radio,
    // and the standalone link test had already shown each node reaching
    // the ICU at 100% on its own.
    //
    // A frame at SF7 is about 165 ms. A fixed 250 ms stagger on node 2
    // is therefore longer than a whole frame, so the two nodes'
    // transmissions cannot overlap even when they decide to send at the
    // identical millisecond.
    //
    // Node 1 gets zero delay and is unaffected.
    //
    // PLACED BEFORE THE RETUNE, DELIBERATELY. Waiting after the radio
    // has already moved to 434.5 would leave this node deaf to the
    // ambulance for an extra 250 ms while doing nothing at all --
    // spending listening time on the link that matters to solve a
    // problem on the one that does not. Here the node stays in
    // receive on the ambulance frequency throughout the wait.
    // ------------------------------------------------------------
    // v6: THE PER-NODE STAGGER IS GONE FROM HERE, DELIBERATELY.
    //
    // A delay applied after icuSlotOpen() has approved the transmission
    // moves the frame out of the very slot that approval was granted for.
    // Separation is now expressed as a PHASE the caller must already be
    // inside, so by the time execution reaches this line the node is
    // provably in its own slot and must transmit immediately.
    //
    // ANYTHING THAT SLEEPS BETWEEN THE GATE AND THE TRANSMIT RE-CREATES
    // THE DEFECT THIS REPLACED. Do not add one. If a future change needs
    // to wait, it must re-test icuSlotOpen() afterwards.

    LoRa.idle();                       // leave receive-continuous mode
    LoRa.setFrequency(CENTRAL_FREQ);
    LoRa.setSyncWord(LORA_SYNC_WORD_ICU);
    LoRa.setSpreadingFactor(LORA_SF_ICU);   // FIX 3: fast, short link
    LoRa.setTxPower(ICU_TX_POWER_DBM);

    // --- Listen Before Talk -------------------------------------------
    bool channelClear = false;
    for (int attempt = 0; attempt < ICU_LBT_ATTEMPTS && !channelClear; attempt++) {
        if (!LoRa.available() && !isChannelBusy()) {
            channelClear = true;
        } else {
            // v6: bounded. The whole backoff budget must fit inside the
            // slot alongside the frame -- see ICU_SLOT_WIDTH_MS. The old
            // 3 x random(5,50) could spend 150 ms of a 220 ms slot before
            // transmitting a 165 ms frame.
            uint32_t backoff = random(5, ICU_LBT_MAX_BACKOFF_MS);
            vTaskDelay(pdMS_TO_TICKS(backoff));
        }
    }
    if (!channelClear) icuTxForcedOnBusy++;   // transmit anyway; counted, not hidden

    // ==================================================================
    // CHANGED v3.2: stamp the counter and sign HERE, not at build time.
    //
    // Counters used to be assigned inside build*(), but frames do not
    // transmit in the order they are built. Events go into a queue and
    // wait for a transmit window; heartbeats are built and radiated
    // immediately in transportTask. So an event could take counter N, be
    // overtaken by a heartbeat taking N+1, and then arrive at the ICU
    // with a LOWER counter than one already accepted -- where it was
    // correctly rejected as a replay.
    //
    // TC-LN-001 (v3.1) lost 1 of 12 real detections this way, and it
    // presented as "[SECURITY] REPLAY", which is the worst possible
    // disguise for a functional bug.
    //
    // The counter is a replay defence, and replay is about transmission
    // order, so the counter must be assigned at transmission.
    //
    // ORDER IS CRITICAL: stamp first, sign second. The CMAC covers the
    // counter bytes. Sign before stamping and every frame fails its MAC
    // at the ICU.
    // ==================================================================
    // v4 FIX: bail out BEFORE gwCounter.next().
    //
    // The first cut called next() and only then discovered signing would
    // fail, so every failed attempt burned a counter value. With the
    // heartbeat retrying each 50 ms loop pass that is ~20 counters/second:
    // the 16-bit seq reaches its 65534 wrap in about 55 minutes, forcing an
    // NVS epoch write, then does it again, forever. Counter space is a
    // finite persisted resource -- never spend it on a frame that cannot be
    // transmitted.
    if (!gwSession.valid()) {
        LoRa.setFrequency(AMBULANCE_FREQ);
        LoRa.setSyncWord(0xF3);
        LoRa.setSpreadingFactor(LORA_SF);   // back to the ambulance link
        LoRa.setTxPower(EVU_LINK_TX_POWER_DBM);
        LoRa.receive();
        xSemaphoreGive(loraMutex);
        icuTxNoSession++;
        static unsigned long lastNoSessLog = 0;
        if (millis() - lastNoSessLog > 10000) {   // this condition persists
            lastNoSessLog = millis();
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.printf("[ICU TX] ABORTED - no valid session key (%lu so far)\n",
                          (unsigned long)icuTxNoSession);
            gwPrintSessionFailure();   // reprint the latched reason
            xSemaphoreGive(serialMutex);
        }
        return false;
    }

    uint32_t txCounter = gwCounter.next();
    memcpy(data + GW_COUNTER_OFFSET, &txCounter, sizeof(txCounter));
    // v4: gwSignFrame() now also stamps session_epoch (the tag covers it) and
    // returns false if no valid session key exists. FAIL CLOSED -- radiating
    // an unauthenticated frame is worse than radiating nothing: the ICU
    // rejects it anyway and we spent the airtime and the ambulance-deafness
    // for nothing.
    if (!gwSignFrame(LANE_ID, NODE_ID, data, length)) {
        LoRa.setFrequency(AMBULANCE_FREQ);
        LoRa.setSyncWord(0xF3);
        LoRa.setSpreadingFactor(LORA_SF);   // back to the ambulance link
        LoRa.setTxPower(EVU_LINK_TX_POWER_DBM);
        LoRa.receive();
        xSemaphoreGive(loraMutex);
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.println("[ICU TX] ABORTED - gwSignFrame failed (not a session issue)");
        xSemaphoreGive(serialMutex);
        icuTxNoSession++;
        return false;
    }

        LoRa.beginPacket();
    LoRa.write(data, length);
    bool radiated = LoRa.endPacket(false);   // blocking

    // Retune back to the EVU-facing link before releasing the radio
    // Restore the ambulance link. NOT OPTIONAL and not deferrable: this
    // node is deaf to vehicles until every one of these is back.
    LoRa.setFrequency(AMBULANCE_FREQ);
    LoRa.setSyncWord(LORA_SYNC_WORD_EVU);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setTxPower(EVU_LINK_TX_POWER_DBM);
    LoRa.receive();

    xSemaphoreGive(loraMutex);

    if (!radiated) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.println("[ICU TX] LORA RADIATE FAILED");
        xSemaphoreGive(serialMutex);
    }

    return radiated;
}

HeartbeatFrame buildHeartbeat() {
    HeartbeatFrame f;
    memset(&f, 0, sizeof(f));

    f.hdr.proto_version   = GW_PROTO_VERSION;
    f.hdr.packet_type     = HEARTBEAT_PACKET;
    f.hdr.intersection_id = INTERSECTION_ID;
    f.hdr.lane_id         = LANE_ID;
    f.hdr.node_id         = NODE_ID;
    f.hdr.counter         = 0;   // stamped at transmit time -- see radiateToICU()

    // CHANGED v3: seconds, not milliseconds. millis() as uint32 wraps at
    // 49.7 days; this system is meant to run for months, and a heartbeat
    // whose uptime silently jumps back to zero makes any endurance log
    // useless. Seconds gives 136 years.
    f.uptime_s      = (uint32_t)(millis() / 1000UL);
    f.free_heap_kb  = (uint16_t)(ESP.getFreeHeap() / 1024UL);
    f.battery_mv    = (uint16_t)lround(batteryVoltage * 1000.0f);
    f.battery_level = batteryLevel;

    f.flags = 0;
    if (loraHealthy) f.flags |= HBF_LORA_OK;
    if (micHealthy)  f.flags |= HBF_MIC_OK;
    // Tell the ICU, on every heartbeat, whether this node is a bench
    // build. Previously a node running with the geofence bypassed was
    // indistinguishable from a production node by inspection.
    if (GEOFENCE_BYPASSED_FOR_SOP) f.flags |= HBF_BENCH_BUILD;

    // CHANGED v3.1: distinguish "no measurement" from "critically low".
    // TC-LN-001 logged BATT=0mV LVL=1 for the whole run, which reads as a
    // dying battery. The divider simply isn't connected. Reporting an
    // unwired sensor as a critical reading is how you train people to
    // ignore critical readings.
    if (batteryVoltage > 0.10f) f.flags |= HBF_BATT_VALID;
    else f.battery_level = 0;          // 0 = unmeasured, 1..5 = real

    /*******************************************************************
     * v5 / PHASE 4 : SURVEYED GEOMETRY
     *
     * The node states where it is. It does NOT state whether it is the
     * far node or the near node -- it does not know, and must not
     * guess. The ICU sorts each approach's two nodes by these distances
     * at runtime.
     *
     * Sent on EVERY heartbeat rather than announced once at boot. A
     * one-time announcement is lost forever if the ICU reboots or the
     * frame is dropped, and it cannot detect an enclosure swapped or
     * re-flashed while the ICU was running. Repeating it makes the
     * geometry continuously verified instead of assumed.
     *
     * The heartbeat is authenticated like every other frame, so this
     * cannot be injected to invert a lane's direction logic -- which
     * would be an unusually elegant attack, since it makes the junction
     * preempt for departing traffic and ignore arrivals while every log
     * continues to look correct.
     *******************************************************************/

    f.approach_id            = NODE_APPROACH_ID;
    f.distance_to_stopline_m = nodeDistanceM;
    f.config_hash            = computeConfigHash();

    // The flag is a SEPARATE assertion from the value.
    //
    // A plausible distance nobody surveyed passes every range check and
    // quietly becomes the basis for direction inference. Requiring an
    // explicit "this was measured" means an unsurveyed node presents as
    // MISCONFIGURED rather than as confidently wrong. It also survives
    // a corrupted field that happens to land inside the plausible range.
    if (nodeSurveyed &&
        nodeDistanceM >= GW_DISTANCE_MIN_M &&
        nodeDistanceM <= GW_DISTANCE_MAX_M) {
        f.flags |= HBF_DISTANCE_VALID;
    } else {
        // Report the sentinel, not the number. If the node is not
        // entitled to be believed, it should not put a usable-looking
        // value on the wire at all -- a receiver that ignores the flag
        // would otherwise still find something plausible to sort by.
        f.distance_to_stopline_m = GW_DISTANCE_UNSET;
    }

    /*******************************************************************
     * v5 / PHASE 4 : MICROPHONE SELF-TEST
     *
     * Catches the fault v4 could not represent: a node whose radio,
     * battery and firmware are all healthy, and whose microphone is
     * dead, unplugged, or taped over. It heartbeats perfectly and
     * simply never detects anything -- indistinguishable, under v4,
     * from a node on a quiet road (spec defect S1-04).
     *******************************************************************/

    f.mic_noise_floor = micNoiseFloorValid
                      ? encodeNoiseFloor(micNoiseFloorRms)
                      : 0;                    // 0 = no measurement yet

    f.loop_liveness   = inferenceLiveness;

    // HBF_MIC_SELFTEST_OK is a STRICTER claim than HBF_MIC_OK.
    //
    // HBF_MIC_OK says the I2S driver initialised. That is true of a
    // microphone with tape over it, and true of one whose cable was cut
    // after boot.
    //
    // This says: the ambient level has settled, it sits inside the
    // plausible band for a roadside site, and the inference loop is
    // actually running. All three, or the flag stays clear.
    bool floorPlausible = micNoiseFloorValid &&
                          micNoiseFloorRms >= MIC_NOISE_FLOOR_MIN_RMS &&
                          micNoiseFloorRms <= MIC_NOISE_FLOOR_MAX_RMS;

    if (micHealthy && floorPlausible) {
        f.flags |= HBF_MIC_SELFTEST_OK;
    }

    // Whether this node's clock is disciplined well enough for its
    // age_ms values to mean anything. The ICU uses age_ms to reconstruct
    // when a detection actually happened, so a node that cannot make
    // that claim must say so rather than have its timings quietly
    // trusted.
    //
    // [FIELD] There is no time source on this node yet. Left clear
    // deliberately: age_ms is still useful as a relative measure within
    // one node, and claiming disciplined time we do not have would be
    // worse than admitting we lack it.

    // NOT signed here. The counter is stamped at transmit time, and the
    // CMAC covers the counter, so signing must happen after stamping.
    // radiateToICU() does both.
    return f;
}

void transportTask(void *parameter) {
    // FIX 4, NOW ACTUALLY APPLIED.
    //
    // NODE_TX_OFFSET_MS was defined in the previous revision and never
    // referenced -- the constant existed, nothing used it, and the two
    // nodes went on transmitting in step. A bench run showed the result:
    // 58 frames sent by the pair, 46 received by the ICU, with both
    // nodes losing a similar share. That is the signature of two
    // transmitters coinciding, not of one weak radio.
    //
    // Seeding lastHeartbeat NEGATIVE by the offset makes node 2's first
    // heartbeat fall half an interval after node 1's, and every one
    // after it stays out of phase. Node 1 gets an offset of zero and is
    // unaffected.
    //
    // The random jitter still runs on top, so nothing is perfectly
    // periodic -- but the two nodes now start from opposite points in
    // the cycle rather than from the same one.
    unsigned long lastHeartbeat = 0 - (unsigned long)NODE_TX_OFFSET_MS;

    unsigned long hbInterval    = HEARTBEAT_BASE_MS + random(0, HEARTBEAT_JITTER_MS);
    OutFrame pending;
    bool     hasPending = false;

    while(true) {
        // ---------------- heartbeat ----------------------------------
        // CHANGED v3: was a flat 20000ms with no jitter, against an ICU
        // timeout of 60000ms -- so two lost heartbeats put a node offline,
        // on a link with no acknowledgement and no retry.
        //
        // Now 5000ms base with up to 1500ms of random jitter, re-rolled
        // every cycle. The jitter is the important part: six nodes on a
        // fixed identical period will eventually drift into phase and then
        // STAY collided, because nothing breaks the symmetry. Randomising
        // guarantees they walk apart again.
        // CHANGED v3.6: deadline override. See HEARTBEAT_MAX_DEFER_MS.
        //
        // `due` is when the heartbeat SHOULD have gone out. Past
        // HEARTBEAT_MAX_DEFER_MS beyond that we stop asking politely for a
        // clear window and transmit anyway, accepting ~271ms of deafness to
        // the EVU. Without this the spacing is unbounded: it is whatever
        // icuTransmitWindowOpen() happens to allow, which measured 13.65s
        // in TC-LN-001 v3.5 and gets worse with acoustic traffic added.
        bool hbDue     = (millis() - lastHeartbeat) > hbInterval;
        bool hbOverdue = (millis() - lastHeartbeat) > (hbInterval + HEARTBEAT_MAX_DEFER_MS);

        // v6: slot gate. hbOverdue bypasses the SLOT but never the EVU
        // guard inside icuSlotOpen() -- the previous form (|| hbOverdue)
        // bypassed both, which is what made an overdue heartbeat able to
        // transmit on top of the ambulance.
        if(hbDue && icuSlotOpen(HEARTBEAT_AIRTIME_MS, hbOverdue)) {
            if(hbOverdue) hbForcedOnDeadline++;

            HeartbeatFrame hb = buildHeartbeat();
            bool ok = radiateToICU((uint8_t*)&hb, sizeof(hb));

            // CHANGED v3.3: same class of bug as the event path. The old
            // code advanced lastHeartbeat whether or not the frame went
            // out, so a mutex timeout cost a full heartbeat interval. Less
            // damaging than a lost detection, but it eats the offline-
            // detection margin for free. On failure we retry on the next
            // loop pass instead.
            if(ok) {
                // v3.6: record actual spacing. This is the number the ICU's
                // HEARTBEAT_TIMEOUT_MS is derived from, so measure it here
                // rather than reconstructing it from ICU timestamps later.
                unsigned long spacing = millis() - lastHeartbeat;
                if(spacing > hbMaxSpacingMs) hbMaxSpacingMs = spacing;

                lastHeartbeat = millis();
                hbInterval = HEARTBEAT_BASE_MS + random(0, HEARTBEAT_JITTER_MS);
            } else if(!gwSession.valid()) {
                // v4: a missing session key is not a transient radio problem
                // -- it will not clear by retrying 20x/second. Back off to the
                // normal cadence so the node keeps its loop, its log and its
                // NVS wear budget while somebody fixes the key material.
                // Radio-contention failures still retry immediately; that IS
                // transient and that behaviour is unchanged.
                lastHeartbeat = millis();
                hbInterval = HEARTBEAT_BASE_MS;
            }

            xSemaphoreTake(serialMutex, portMAX_DELAY);
            if(ok) {
                Serial.printf("[HEARTBEAT RADIATED] ctr=%lu%s\n",
                              (unsigned long)hb.hdr.counter,
                              hbOverdue ? "  *FORCED - window never cleared*" : "");
            } else {
                Serial.printf("[HEARTBEAT RADIATE FAILED - %s] ctr=%lu\n",
                              gwSession.valid() ? "will retry" : "NO SESSION KEY",
                              (unsigned long)hb.hdr.counter);
            }
            xSemaphoreGive(serialMutex);
        }

        // ---------------- events -------------------------------------
        // CHANGED v3: the old code did `for(i=0;i<3;i++) sendToICU(...)`
        // back to back. At the old 136-byte frame size that was
        // 3 x 861ms + delays = about 2.6 SECONDS during which this node
        // was retuned to 434.5 MHz and therefore completely deaf to the
        // ambulance on 433.0 -- at precisely the moment it had just
        // detected one.
        //
        // Repeats still happen (there are no acknowledgements on IF-2, so
        // a single collision would otherwise lose a detection forever),
        // but each repeat is now scheduled INDEPENDENTLY into a free gap
        // rather than fired as one uninterruptible burst.
        if(!hasPending) {
            if(xQueueReceive(eventQueue, &pending, 0) == pdTRUE) hasPending = true;
        }

        // ==============================================================
        // CHANGED v3.3: only count an attempt that ACTUALLY radiated.
        //
        // radiateToICU() returns false when it cannot acquire loraMutex
        // within 200ms -- the radio is mid-transaction on the EVU-facing
        // link. The old code incremented attempts regardless of the
        // return value, so with EVENT_REPEAT_COUNT=1 a single mutex
        // timeout silently DISCARDED A REAL DETECTION while logging
        // "[EVENT RADIATED x1]" as though it had succeeded.
        //
        // TC-LN-001 (v3.2) exposed this: one event logged "ctr=0", which
        // is impossible for a transmitted frame because the counter is
        // stamped inside radiateToICU(). An unstamped counter meant the
        // stamping code never ran, which meant nothing was transmitted.
        //
        // Note this bug predates v3.2 -- it has been present since the v3
        // transportTask rewrite. It only became VISIBLE once the counter
        // was stamped at transmit time, because before that a skipped
        // frame carried a plausible build-time counter and its log line
        // was indistinguishable from a real transmission.
        //
        // Lesson worth keeping: instrumentation that can only produce a
        // valid-looking value cannot tell you when it failed. The ctr=0
        // here is diagnostic precisely BECAUSE it is impossible.
        // ==============================================================
        if(hasPending && icuSlotOpen(EVENT_AIRTIME_MS, pending.fallingEdge)) {
            if(radiateToICU(pending.buf, pending.len)) {
                pending.attempts++;
            } else {
                pending.retries++;
            }

            bool done      = (pending.attempts >= EVENT_REPEAT_COUNT);
            bool exhausted = (pending.retries  >= EVENT_MAX_RETRIES);

            if(done || exhausted) {
                hasPending = false;
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                if(pending.attempts == 0) {
                    // Never got on the air at all. This is a lost
                    // detection and must be loud, not silent.
                    Serial.printf("[EVENT DROPPED - radio unavailable after %d retries]\n",
                                  pending.retries);
                    eventsDropped++;
                } else {
                    uint32_t sentCtr;
                    memcpy(&sentCtr, pending.buf + GW_COUNTER_OFFSET, sizeof(sentCtr));
                    Serial.printf("[EVENT RADIATED x%d] ctr=%lu%s\n",
                                  pending.attempts, (unsigned long)sentCtr,
                                  pending.retries ? "  (after retry)" : "");
                }
                xSemaphoreGive(serialMutex);
            }
        }

        // v4 (SOP 3.4): check the 60 s re-key here, in the task loop, not in
        // radiateToICU(). Deriving inside the transmit path would put an ECDH
        // inside a scheduled window and could push a frame past its reserved
        // airtime.
        gwSession.tick();

        // v3.3: local health line. eventsDropped and icuTxForcedOnBusy are
        // the only signs that radio contention is costing us detections,
        // and neither is visible from the ICU side -- a dropped event
        // simply never arrives, which looks identical to no detection.
        static unsigned long lastTxHealth = 0;
        if(millis() - lastTxHealth > 60000) {
            lastTxHealth = millis();
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            Serial.printf("[TX HEALTH] dropped=%lu skipped=%lu forcedOnBusy=%lu "
                          "hbForced=%lu hbMaxGap=%lums nosession=%lu epoch=%lu\n",
                          (unsigned long)eventsDropped,
                          (unsigned long)icuTxSkipped,
                          (unsigned long)icuTxForcedOnBusy,
                          (unsigned long)hbForcedOnDeadline,
                          (unsigned long)hbMaxSpacingMs,
                          (unsigned long)icuTxNoSession,
                          (unsigned long)gwSession.epoch());

            // v3.6: IF-1 quality summary. See the accumulator declarations.
            if(if1Packets > 0) {
                Serial.printf("[IF1 HEALTH] n=%lu rssi=%d..%d snr=%.1f..%.1f "
                              "mean=%.1f lowSnr(<%.0fdB)=%lu\n",
                              (unsigned long)if1Packets, if1RssiMin, if1RssiMax,
                              if1SnrMin, if1SnrMax,
                              if1SnrSum / (float)if1Packets,
                              IF1_LOW_SNR_DB, (unsigned long)if1LowSnrPkts);

                // v3.7: rolling window. The FLOOR is the number that decides
                // SF8 vs SF9; a session mean cannot show it moving.
                if(if1WinCount > 0) {
                    float wMin = 999.0f, wSum = 0.0f;
                    uint32_t wLow = 0;
                    for(uint32_t i = 0; i < if1WinCount; i++) {
                        float v = if1WinSnr[i];
                        if(v < wMin) wMin = v;
                        wSum += v;
                        if(v < IF1_LOW_SNR_DB) wLow++;
                    }
                    float frac = (float)wLow / (float)if1WinCount;
                    Serial.printf("[IF1 TREND] last%lu: min=%.1f mean=%.1f "
                                  "lowSnr=%lu/%lu (%.0f%%)%s\n",
                                  (unsigned long)if1WinCount, wMin,
                                  wSum / (float)if1WinCount,
                                  (unsigned long)wLow, (unsigned long)if1WinCount,
                                  100.0f * frac,
                                  (frac > IF1_DEGRADED_FRAC) ? "  ** DEGRADED **" : "");
                    if(frac > IF1_DEGRADED_FRAC) {
                        Serial.println("[IF1 TREND] Sweep the band before trusting "
                                       "any SF measurement taken in this session.");
                    }
                }
            }
            xSemaphoreGive(serialMutex);
        }

        if(lastEmergency!=0 && millis()-lastEmergency>40000) {
            priorityLocked=false;
            sirenActive=false;
            sirenScore=0;
            lastEmergency=0;
            Serial.println("[EVENT RESET]");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void updateBattery() {
    uint32_t sum = 0;
    
    // Take 20 samples to smooth out ADC noise
    for(int i = 0; i < 20; i++) {
        sum += analogRead(BATTERY_ADC_PIN);
        delay(2);
    }
    
    int raw = sum / 20;
    
    float adcVoltage = (raw / 4095.0f) * 3.3f;
    adcVoltage *= 1.08f; // Hardware calibration multiplier

    batteryVoltage = adcVoltage * ((68.0f + 33.0f) / 33.0f);

    // Apply software low-pass filter
    if(filteredBattery < 0) {
        filteredBattery = batteryVoltage;
    } else {
        filteredBattery = 0.9f * filteredBattery + 0.1f * batteryVoltage;
    }
    
    batteryVoltage = filteredBattery;

    // Map to discrete battery levels
    if(batteryVoltage >= 8.0f) {
        batteryLevel = 5;
    } else if(batteryVoltage >= 7.4f) {
        batteryLevel = 4;
    } else if(batteryVoltage >= 7.0f) {
        batteryLevel = 3;
    } else if(batteryVoltage >= 6.4f) {
        batteryLevel = 2;
    } else {
        batteryLevel = 1;
    }

    xSemaphoreTake(serialMutex, portMAX_DELAY);
    // v4.5: the heartbeat already reports BATT=n/a when the divider is
    // unwired (HBF_BATT_VALID). This local line still printed LEVEL=1,
    // which reads as "critically low" rather than "not measured" -- the
    // exact confusion the heartbeat flag was added to prevent. Same rule,
    // same threshold, both places.
    if (batteryVoltage > 0.10f) {
        Serial.printf("[BATTERY] RAW=%d ADC=%.3fV PACK=%.2fV LEVEL=%d\n",
                      raw, adcVoltage, batteryVoltage, batteryLevel);
    } else {
        Serial.printf("[BATTERY] RAW=%d ADC=%.3fV PACK=unmeasured LEVEL=- "
                      "(divider not connected)\n", raw, adcVoltage);
    }
    xSemaphoreGive(serialMutex);
}

// CHANGED v3: prints the airtime budget at boot.
//
// The duty cycle of this system was never computed by anyone until it was
// measured off the source, and it turned out to be 37% on the EVU link --
// far outside any licence-exempt allowance. Printing it at every boot
// means nobody can change a frame size or a spreading factor again
// without immediately seeing what it did to the budget.
void printAirtimeBudget() {
    Serial.println("---- IF-2 AIRTIME BUDGET (SF9/BW125/CR4-6/pre12) ----");
    Serial.printf("  HeartbeatFrame      %2u B  ~%4lu ms\n",
                  (unsigned)sizeof(HeartbeatFrame), HEARTBEAT_AIRTIME_MS);
    Serial.printf("  AcousticEventFrame  %2u B  ~%4lu ms\n",
                  (unsigned)sizeof(AcousticEventFrame), ACOUSTIC_AIRTIME_MS);
    Serial.printf("  LoRaEventFrame      %2u B  ~%4lu ms\n",
                  (unsigned)sizeof(LoRaEventFrame), EVENT_AIRTIME_MS);
    Serial.printf("  Heartbeat duty      ~%.2f %%  (1 node)\n",
                  100.0 * HEARTBEAT_AIRTIME_MS / (double)HEARTBEAT_BASE_MS);
    Serial.printf("  Event repeat        %d   max retries %d\n",
                  EVENT_REPEAT_COUNT, EVENT_MAX_RETRIES);

    // CHANGED v3.6: print the acoustic and heartbeat scheduling constants.
    //
    // The ICU has printed its timer set since v3.5 and it immediately paid
    // off -- TC-LN-001 v3.5 proved the ICU build from its banner alone. The
    // lane node printed nothing about the acoustic path, so when that run
    // produced zero siren events there was NO WAY to tell whether the
    // re-assert firmware was flashed or simply never triggered. Build
    // provenance was verifiable at one end of a two-ended change.
    //
    // These two constants are paired with EVENT_TIMEOUT_MS and
    // HEARTBEAT_TIMEOUT_MS in ICU.ino and nothing can check that pairing
    // across two sketch folders. Printing both ends makes a mismatched
    // flash visible in the first ten lines of each log instead of showing
    // up later as intermittent missed preemptions.
    Serial.printf("  Acoustic re-assert  %lu ms  (ICU expects eventTimeout=20000)\n",
                  ACOUSTIC_REASSERT_MS);
    // v3.7: the ICU prints the values it EXPECTS from this node
    // (EXPECTED_ACOUSTIC_REASSERT_MS / EXPECTED_LANE_NODE_DEFER_MS). Compare
    // those two banners at the start of every run. Nothing can check this
    // pairing at compile time across two sketch folders; two banners and one
    // pair of eyes is the whole mechanism, so it has to be legible.
    Serial.printf("  Acoustic duty       ~%.2f %%  while a siren is audible\n",
                  100.0 * (double)ACOUSTIC_AIRTIME_MS / (double)ACOUSTIC_REASSERT_MS);
    Serial.printf("  Event min interval  %lu ms\n", EVENT_MIN_INTERVAL_MS);
    Serial.printf("  Heartbeat max defer %lu ms -> worst spacing ~%lu ms "
                  "(ICU expects hbTimeout=40000)\n",
                  HEARTBEAT_MAX_DEFER_MS,
                  HEARTBEAT_BASE_MS + HEARTBEAT_JITTER_MS +
                  HEARTBEAT_MAX_DEFER_MS + HEARTBEAT_AIRTIME_MS);
    Serial.printf("  Geofence            %s\n",
                  GEOFENCE_BYPASSED_FOR_SOP ? "BYPASSED (bench)"
                                            : (LANE_NODE_POSITION_KNOWN
                                               ? "ACTIVE"
                                               : "ACTIVE but position UNSURVEYED - fails open"));
    Serial.printf("  EVU occupancy       %lu ms of every %lu ms -> free gap %lu ms\n",
                  EVU_AIRTIME_MS, EVU_TX_INTERVAL_MS,
                  EVU_TX_INTERVAL_MS - EVU_AIRTIME_MS - 2*ICU_TX_GUARD_MS);
    if (EVENT_AIRTIME_MS + 2*ICU_TX_GUARD_MS > EVU_TX_INTERVAL_MS - EVU_AIRTIME_MS) {
        Serial.println("  ** WARNING: event frame does NOT fit the EVU gap **");
    }
    Serial.println("-----------------------------------------------------");
}

void initializeSystem() {
    Serial.begin(115200);
    delay(5000);
    Serial.println();
    Serial.println("==============================");
    // v4 CONSISTENCY FIX: this said "proto v3" while GW_PROTO_VERSION is 4 and
    // the [BUILD] line two lines below printed the real value -- a banner that
    // contradicted itself within three lines, in a file whose own comments
    // argue that build provenance must be legible in the first ten lines.
    Serial.printf(" GREENWAVE LANE NODE  proto v%d \n", GW_PROTO_VERSION);
    Serial.println(" Secure + Smart Geofence ");
    Serial.println("==============================");
    Serial.printf("[BUILD] L%dN%d  intersection=%u  proto=%d\n",
                  LANE_ID, NODE_ID, (unsigned)INTERSECTION_ID, GW_PROTO_VERSION);
    if (GEOFENCE_BYPASSED_FOR_SOP)
        Serial.println("[BUILD] *** BENCH BUILD: GEOFENCE BYPASSED - NOT FOR DEPLOYMENT ***");
    memset(trustTable, 0, sizeof(trustTable));
    memset(retroBuffer, 0, sizeof(retroBuffer));

    // Seed the RNG used for heartbeat jitter and CAD backoff from a
    // hardware entropy source. Without this every node boots with an
    // identical random sequence and the "jitter" would be identical
    // across all six -- which is exactly the collision pattern we are
    // trying to break.
    randomSeed(esp_random());

    gwCounter.begin("gw-ln-ctr");

    // v4: derive the first session key. Must run BEFORE any frame is built;
    // gwSignFrame() fails closed if the session is invalid, so a derivation
    // failure means silence rather than unauthenticated traffic.
    gwSession.begin(INTERSECTION_ID, LANE_ID, NODE_ID, "gw-ln-sess");
    printAirtimeBudget();
    Serial.println("[SECURITY READY]");
}

void createQueues() {
    eventQueue=xQueueCreate(20, sizeof(OutFrame));
    if(eventQueue) Serial.println("[QUEUE READY]");
}

void createTasks() {
    xTaskCreatePinnedToCore(loraTask, "LoRa", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(tinymlTask, "TinyML", 24576, NULL, 2, NULL, 0);
    
    // FIXED: Uncommented so parser can access telemetry for stress testing
    xTaskCreatePinnedToCore(transportTask, "Transport", 8192, NULL, 1, NULL, 0); 

    Serial.println("[TASKS STARTED ON ESP32-S3 DUAL CORES]");
}


// =====================================================
// PER-JUNCTION GEOMETRY  --  NVS + SERIAL CONSOLE
// =====================================================
//
// The node's distance to the stop line is the only value that differs
// between junctions, and it was a compile-time constant. Moving a node
// to another site meant edit, compile, flash -- with a chance each time
// of flashing the wrong number and not noticing.
//
// That failure is silent and consequential. This value sets the
// acoustic correlation window, so a wrong distance produces
// systematically wrong direction inference with nothing in any log to
// show for it.
//
// It now lives in NVS. Set it once per site over the serial console:
//
//     dist 250
//
// NVS survives reflashing, so updating firmware does not lose the
// survey.

#include <Preferences.h>

static Preferences nodePrefs;

static void gwNodeConfigLoad() {
    nodePrefs.begin("gw-node", true);          // read-only

    bool haveNvs = nodePrefs.getBool("prov", false);

    if (haveNvs) {
        nodeDistanceM = nodePrefs.getUShort("dist", NODE_DISTANCE_FALLBACK_M);
        nodeSurveyed  = true;                  // entering it IS surveying it
        nodeFromNvs   = true;
    }

    nodePrefs.end();

    // A stored value outside plausible bounds is treated as absent.
    // Corrupted NVS should fall back to a known number rather than to
    // one that will quietly skew the correlation window.
    if (nodeFromNvs &&
        (nodeDistanceM < GW_DISTANCE_MIN_M || nodeDistanceM > GW_DISTANCE_MAX_M)) {
        Serial.printf("[NODE] stored distance %um is out of range -- ignoring\n",
                      (unsigned)nodeDistanceM);
        nodeDistanceM = NODE_DISTANCE_FALLBACK_M;
        nodeSurveyed  = NODE_DISTANCE_SURVEYED_FALLBACK;
        nodeFromNvs   = false;
    }
}

static void gwNodeConfigSave(uint16_t metres) {
    nodePrefs.begin("gw-node", false);
    nodePrefs.putBool("prov", true);
    nodePrefs.putUShort("dist", metres);
    nodePrefs.end();

    nodeDistanceM = metres;
    nodeSurveyed  = true;
    nodeFromNvs   = true;
}

static void gwNodePrintConfig() {
    Serial.println();
    Serial.println("---------------- NODE CONFIGURATION ----------------");
    Serial.printf("  lane %d  node %d  intersection %d\n",
                  LANE_ID, NODE_ID, INTERSECTION_ID);
    Serial.printf("  distance to stop line : %u m  [%s]\n",
                  (unsigned)nodeDistanceM,
                  nodeSurveyed ? "SURVEYED" : "NOT SURVEYED");
    Serial.printf("  source                : %s\n",
                  nodeFromNvs ? "NVS (set on site)" : "compiled fallback");
    Serial.printf("  config hash           : 0x%08lX\n",
                  (unsigned long)computeConfigHash());

    if (!nodeSurveyed) {
        Serial.println();
        Serial.println("  *** NOT SURVEYED -- the ICU will mark this approach");
        Serial.println("  *** MISCONFIGURED and take no automatic action on it.");
        Serial.println("  *** Measure along the road and set it:  dist <metres>");
    }
    Serial.println("----------------------------------------------------");
    Serial.println();
}

// Non-blocking serial console. Called from loop().
static void gwNodeConsole() {
    static char buf[48];
    static uint8_t len = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\r') continue;

        if (c != '\n') {
            if (len < sizeof(buf) - 1) buf[len++] = c;
            else len = 0;                        // overlong: discard whole
            continue;
        }

        buf[len] = '\0';
        len = 0;
        if (buf[0] == '\0') continue;

        // ---- dist <metres> ----
        if (strncasecmp(buf, "dist", 4) == 0) {
            const char *arg = buf + 4;
            while (*arg == ' ') arg++;

            if (*arg == '\0') {
                Serial.printf("[NODE] distance is %u m (%s)\n",
                              (unsigned)nodeDistanceM,
                              nodeFromNvs ? "from NVS" : "compiled fallback");
                Serial.println("[NODE] set with:  dist <metres>");
                continue;
            }

            long m = atol(arg);

            if (m < GW_DISTANCE_MIN_M || m > GW_DISTANCE_MAX_M) {
                Serial.printf("[NODE] %ld m is outside the plausible range "
                              "(%d..%d)\n",
                              m, GW_DISTANCE_MIN_M, GW_DISTANCE_MAX_M);
                continue;
            }

            gwNodeConfigSave((uint16_t)m);

            Serial.printf("[NODE] distance set to %ld m and saved to NVS\n", m);
            Serial.println("[NODE] the ICU re-derives FAR/NEAR from the next "
                           "heartbeat -- no reset needed");
            Serial.println("[NODE] the two nodes on an approach MUST differ, "
                           "and by at least ~85 m for reliable direction");
            continue;
        }

        // ---- show ----
        if (strncasecmp(buf, "show", 4) == 0) {
            gwNodePrintConfig();
            continue;
        }

        // ---- clear ----
        if (strncasecmp(buf, "clear", 5) == 0) {
            nodePrefs.begin("gw-node", false);
            nodePrefs.clear();
            nodePrefs.end();

            nodeDistanceM = NODE_DISTANCE_FALLBACK_M;
            nodeSurveyed  = NODE_DISTANCE_SURVEYED_FALLBACK;
            nodeFromNvs   = false;

            Serial.println("[NODE] NVS cleared -- back to compiled fallback");
            gwNodePrintConfig();
            continue;
        }

        Serial.println("[NODE] commands:  dist <metres> | show | clear");
    }
}

void setup() {
    loraMutex=xSemaphoreCreateMutex();
    serialMutex=xSemaphoreCreateMutex();
    pinMode(BATTERY_ADC_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    // Load the surveyed distance BEFORE anything transmits. The config
    // hash and every heartbeat carry it, so it must be settled first.
    gwNodeConfigLoad();

    initializeSystem();
    createQueues();
    createTasks();
    Serial.println();
    Serial.println("GREENWAVE SYSTEM ONLINE");

    /*******************************************************************
     * v5 / PHASE 4 : CONFIGURATION BANNER
     *
     * Every RDU runs IDENTICAL firmware and differs only by the macros
     * at the top of this file. That is the right design, and it has one
     * predictable failure: flashing the wrong build to the wrong
     * enclosure. Nothing about a running node makes that visible --
     * every field is individually plausible, so it reports confidently
     * and accurately about the wrong piece of road.
     *
     * Printing the whole identity at boot makes it a five-second check
     * with a serial cable, instead of a fault discovered when the
     * junction behaves backwards.
     *******************************************************************/
    Serial.println();
    Serial.println("================ NODE CONFIGURATION ================");
    Serial.printf("  intersection=%d  lane=%d  node=%d  approach=%d\n",
                  INTERSECTION_ID, LANE_ID, NODE_ID, NODE_APPROACH_ID);
    // v6: print the slot. A mis-flashed node is otherwise only visible as
    // a degraded if2= at the ICU, hours later. Six boxes must show six
    // different lines here.
    // v6: provisioning state. A bench image built with a placeholder key
    // is indistinguishable from a real one in every other log line, and
    // the ICU reports only BAD TAG. Say it here, once, unmissably.
#ifdef GW_BENCH_UNPROVISIONED
    Serial.println("**********************************************************");
    Serial.println("*  UNPROVISIONED BUILD - PLACEHOLDER KEY - BENCH ONLY     *");
    Serial.println("*  This node CANNOT authenticate to the ICU. Every frame  *");
    Serial.println("*  will be rejected as BAD TAG. Do not deploy.            *");
    Serial.println("**********************************************************");
#endif
    Serial.printf("key set               : %s  %s\n", GW_KEY_SET_ID,
#ifdef GW_BENCH_UNPROVISIONED
                  "UNPROVISIONED");
#else
                  "provisioned");
#endif
    Serial.printf("tx slot               : %u  (period parity %u, position %u)\n",
                  (unsigned)NODE_TX_SLOT, (unsigned)NODE_TX_PERIOD,
                  (unsigned)NODE_TX_POSITION);
    Serial.printf("slot phase            : %lu..%lu ms after EVU frame start\n",
                  (unsigned long)NODE_SLOT_START_MS,
                  (unsigned long)(NODE_SLOT_START_MS + ICU_SLOT_WIDTH_MS));
    Serial.printf("tx opportunity        : 1 per %lu ms (2 EVU periods)\n",
                  (unsigned long)(2UL * EVU_TX_INTERVAL_MS));
    Serial.printf("  distance to stop line : %d m  [%s]\n",
                  nodeDistanceM,
                  nodeSurveyed ? "SURVEYED"
                                         : "NOT SURVEYED - ICU WILL REJECT");
    Serial.printf("  config hash           : 0x%08lX\n",
                  (unsigned long)computeConfigHash());
    Serial.printf("  proto version         : %d   frames HB=%uB ACO=%uB EVT=%uB\n",
                  GW_PROTO_VERSION,
                  (unsigned)sizeof(HeartbeatFrame),
                  (unsigned)sizeof(AcousticEventFrame),
                  (unsigned)sizeof(LoRaEventFrame));
    Serial.printf("  airtime reserved      : HB=%lums ACO=%lums EVT=%lums\n",
                  HEARTBEAT_AIRTIME_MS, ACOUSTIC_AIRTIME_MS, EVENT_AIRTIME_MS);
    Serial.printf("  mic self-test band    : %.5f .. %.5f rms\n",
                  MIC_NOISE_FLOOR_MIN_RMS, MIC_NOISE_FLOOR_MAX_RMS);

    // This node does NOT decide, and does not display, whether it is the
    // far or the near node. The ICU derives that at runtime by sorting
    // the two distances. Printing a guess here would be the first step
    // toward someone trusting it.
    Serial.println("  role (FAR/NEAR)       : decided by the ICU, not here");

    if (!nodeSurveyed) {
        Serial.println();
        Serial.println("  *** WARNING: distance not marked as surveyed.");
        Serial.println("  *** This node reports GW_DISTANCE_UNSET and the ICU");
        Serial.println("  *** will mark this approach MISCONFIGURED and disable");
        Serial.println("  *** automatic action on it. Set NODE_DISTANCE_SURVEYED");
        Serial.println("  *** true only AFTER measuring on site.");
    }
    Serial.println("====================================================");
    Serial.println();
}

void loop() {
    // Poll the console often enough to feel responsive. Battery is
    // sampled on its own slower cadence -- a 5 s loop delay made typing
    // unusable, and this is the interface used to commission a site.
    static unsigned long lastBattery = 0;

    gwNodeConsole();

    if (millis() - lastBattery >= 5000) {
        lastBattery = millis();
        updateBattery();
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}