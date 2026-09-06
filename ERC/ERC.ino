#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <math.h>
#include "greenwave_logo.h"
#include "GreenwaveLink.h"
#include <HardwareSerial.h>

// ============================================================
// GREENWAVE TECHLABS
// EMERGENCY RESPONSE CONSOLE - ERC V3
//
// DIRECT ICU UART
// EMERGENCY ALERT
// ACKNOWLEDGEMENT
// FAR -> NEAR PRIORITY SEQUENCE
// DYNAMIC COUNTDOWN
// AUTO / MANUAL RETURN TO NORMAL
// ============================================================


// ============================================================
// LCD GPIO MAP
// ============================================================

#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   5
#define TFT_DC   27
#define TFT_RST  33
#define TFT_BL   32


// ============================================================
// TOUCH GPIO MAP
// ============================================================

#define TP_SDA 21
#define TP_SCL 22
#define TP_RST 26

#define FT6336_ADDR 0x38


// ============================================================
// LED GPIO MAP
// ============================================================

#define LEFT_FAR_LED     17
#define LEFT_NEAR_LED    16

#define RIGHT_FAR_LED    13
#define RIGHT_NEAR_LED   14

#define BOTTOM_FAR_LED   25
#define BOTTOM_NEAR_LED  19


// ============================================================
// BUZZER
// ============================================================

#define BUZZER 4


// ============================================================
// ICU <-> ERC DIRECT UART
//
// ICU GPIO18 TX ---> ERC GPIO34 RX
// ICU GPIO17 RX <--- ERC GPIO15 TX
// ICU GND        --- ERC GND
// ============================================================

#define ICU_RX 34
#define ICU_TX 15

HardwareSerial ICU_UART(1);


// ============================================================
// COLORS
// ============================================================

#define C_BLACK       0x0000
#define C_WHITE       0xFFFF

// ---- Redesigned console palette (RGB565) ----
// Deep slate base with a cyan accent, amber for
// caution and a desaturated red for alerts.

#define C_BG          0x0884   // #0B1220 page background
#define C_PANEL       0x1107   // #16203A card surface
#define C_RAISED      0x1949   // #1E2B4A raised chip
#define C_BORDER      0x29CB   // #2C3A5E hairline
#define C_ACCENT      0x269D   // #22D3EE cyan accent
#define C_ACCENT_DIM  0x0BB2   // #0E7490 accent shadow
#define C_TEXT        0xEF7E   // #E8EEF7 primary text
#define C_MUTED       0x8AB6   // #8A97B0 secondary text
#define C_AMBER       0xF4E1   // #F59E0B caution
#define C_ALERT       0xEC48   // #EF4444 alert
#define C_ALERT_DK    0x78E3   // #7F1D1D alert shadow
#define C_OK          0x262B   // #22C55E confirm
#define C_OK_DK       0x1285   // #14532D confirm shadow
#define C_INFO        0x3C1E   // #3B82F6 info

// ---- Legacy names kept so existing call sites
// ---- pick up the new palette automatically ----
#define C_NAVY        C_BG
#define C_GREEN       C_OK
#define C_DARKGREEN   C_OK_DK
#define C_RED         C_ALERT
#define C_BLUE        C_INFO
#define C_YELLOW      C_AMBER
#define C_CYAN        C_ACCENT
#define C_GRAY        0x52AA
#define C_LIGHTGRAY   C_MUTED
#define C_ORANGE      C_AMBER
#define C_BUTTON      C_RAISED
#define C_STATUSGREEN C_OK


// ============================================================
// DISPLAY
// ============================================================

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  TFT_DC,
  TFT_CS,
  TFT_SCLK,
  TFT_MOSI,
  GFX_NOT_DEFINED
);

Arduino_GFX *gfx = new Arduino_ST7796(
  bus,
  TFT_RST,
  1,
  true
);


// ============================================================
// SCREEN MODES
// ============================================================

enum ScreenMode
{
  ERC_MAIN,
  ERC_EMERGENCY,
  ERC_TEST,
  ERC_WIFI_LIST,
  ERC_WIFI_KEY
};

ScreenMode currentScreen = ERC_MAIN;


// ============================================================
// ICU LINK / WIFI PROVISIONING STATE
//
// The ICU owns the radio; this console is the
// operator surface for it. Everything below is
// mirrored state pushed over the UART.
// ============================================================

#define MAX_NETS 18

bool  icuOnline    = false;          // ICU has an internet connection
bool  icuLinkAlive = false;          // we have heard from the ICU at all
char  icuSsid[33]  = "";             // network it is joined to
char  wifiStatus[24] = "OFFLINE";    // OFFLINE / CONNECTING / ONLINE / FAILED

unsigned long lastIcuMessage = 0;
// Must stay comfortably longer than the ICU's
// keepalive interval, or the console declares the
// link dead in the quiet gap between messages and
// the offline banner flashes up for no reason.
// PHASE 2: was 25000, sized when the ICU only spoke during an incident
// and 25 s of silence on a quiet link was completely normal.
//
// The ICU now sends PING every second whether or not anything is
// happening, so silence is never normal and the timeout can be tight.
// GW_LINK_TIMEOUT_MS is 5000 -- five missed PINGs.
//
// This is the primary fail-safe of the advisory design. An ICU that
// raises an alert and then hangs must not leave that alert standing on
// the console, so the ERC decides for itself when the ICU has stopped
// talking rather than waiting to be told. See checkIcuLinkTimeout().
const unsigned long ICU_LINK_TIMEOUT = GW_LINK_TIMEOUT_MS;

char netSsid[MAX_NETS][33];
int  netRssi[MAX_NETS];
bool netLock[MAX_NETS];
int  netCount     = 0;
int  netPage      = 0;
int  netSelected  = -1;
bool scanRunning  = false;

char passBuf[65]  = "";
int  passLen      = 0;
bool keyShift     = false;
bool keySymbols   = false;

const int NETS_PER_PAGE = 5;


// ============================================================
// APPROACH
// ============================================================

enum Approach
{
  APPROACH_NONE,
  APPROACH_LEFT,
  APPROACH_RIGHT,
  APPROACH_BOTTOM
};

Approach activeApproach = APPROACH_NONE;


// ============================================================
// PRIORITY SEQUENCE
// ============================================================

// PHASE 3: THE STAGES ARE NOW DRIVEN BY THE ICU, NOT BY A LOCAL TIMER.
//
// The names are kept so the existing drawing code continues to work, but
// their MEANING has changed and the mapping is:
//
//     PRIORITY_FAR_BLINK   <-  ICU stage PREPARE
//     PRIORITY_NEAR_BLINK  <-  ICU stage COMMIT
//     PRIORITY_COMPLETE    <-  ICU stage CLEARING
//
// WHAT WAS WRONG BEFORE
//
// The console received one word -- "LEFT" -- and then ran a fixed
// 7 s / 20 s / 7 s sequence to completion on its own clock. The ICU
// could not extend it, could not end it early, and could not change it.
//
// That made every release rule in the specification unimplementable.
// Scenario 2 v2.0 section 10.1 lists five ways an event can end: the
// driver switching the siren off, the vehicle crossing the stop line,
// the siren fading at the near node, the vehicle turning away, and a
// hard maximum. A fixed timer expresses none of them. It expresses only
// the last one, and it expresses it badly, because 34 seconds is a
// guess rather than a bound.
//
// The failure that matters most is the ordinary one: an ambulance in
// the gridlock this product exists to clear routinely takes longer than
// 34 seconds to travel the last 250 m. The console would stand down
// while the vehicle was still approaching, in precisely the conditions
// the system was bought for.
//
// The opposite error was equally available. A vehicle that turned off
// the approach after 5 seconds still held the console -- and an officer
// holding a manual override -- for the full 34.
//
// NOW: the ICU owns the stage. The ERC renders it, times its own blink
// animation, and independently fails safe if the ICU goes quiet
// (see the link-timeout handler in updateMainHeartbeat).
enum PriorityStage
{
  PRIORITY_IDLE,
  PRIORITY_WAIT_ACK,      // retained for compatibility; unused
  PRIORITY_FAR_BLINK,     // = PREPARE   : probable approach, no buzzer
  PRIORITY_NEAR_BLINK,    // = COMMIT    : confirmed, buzzer, override now
  PRIORITY_COMPLETE       // = CLEARING  : released, winding down
};

PriorityStage priorityStage = PRIORITY_IDLE;

// ------------------------------------------------------------
// PHASE 3: WHAT THE ICU LAST TOLD US ABOUT THIS ALERT
// ------------------------------------------------------------

// Registered priority class of the controlling vehicle. 0 = not stated.
// This is the class from the ICU's own registry, NOT the one the vehicle
// claimed in its packet -- a stolen or modified unit can set the packet
// field to anything, and under the v1.0 design that would have outranked
// every genuine emergency in the network (spec defect S2-01).
uint8_t alertPriority = 0;

// Evidence class. See LinkEvidence in GreenwaveLink.h.
//
// Displayed prominently, and that is a deliberate design position. An
// authenticated ambulance 1.2 km out and a siren that one working
// microphone heard once both produce a red screen, and only one of them
// justifies stopping cross traffic. The project's own IEEE paper
// concludes that acoustic evidence alone is not a sufficient basis for
// preemption; Scenario 1 exists anyway because most vehicles will not
// carry an EVU during rollout. Showing the operator which case they are
// looking at is how that compromise is made honest rather than hidden.
uint8_t alertEvidence = 0;

// Seconds until arrival, from the ICU. -1 means NOT COMPUTABLE.
//
// -1 is displayed as "--", never as 0 and never as a guess. In the
// acoustic-only case the ICU genuinely has no position telemetry and no
// sensor past the stop line, so it cannot know (spec S1-07). A
// fabricated countdown would be read by an officer as a measurement.
int16_t alertEtaS = -1;

// Wall time the current alert began, for the elapsed display.
unsigned long alertStartMs = 0;

// Last release reason, for the CLEARING screen.
char alertReleaseReason[16] = "";

// True once the ICU has sent at least one ALERT for the live event.
// Distinguishes an ICU-driven alert from one started by a bare legacy
// "LEFT" command, which carries no stage, priority or evidence.
bool alertFromIcu = false;

// ------------------------------------------------------------
// PHASE 3.6: APPROACHES WAITING BEHIND THE LIVE ALERT
//
// A live alert is never displaced mid-event -- swapping the screen out
// from under an officer who has begun an override would be worse than
// making the second vehicle wait.
//
// But that leaves a real gap in what the operator can see. A second
// emergency vehicle can be approaching a different arm of the same
// junction, authenticated and higher priority, and nothing on this
// display would indicate it exists. The officer clears the current
// alert, stands down, and only then discovers a second vehicle was
// queued the whole time. A bench run showed exactly that: LEFT held the
// console while BOTTOM carried a live P1 demand, invisibly.
//
// This is the difference between "nothing else is happening" and
// "BOTTOM is next".
// ------------------------------------------------------------
#define MAX_QUEUED_APPROACHES 3

char    queuedName[MAX_QUEUED_APPROACHES][8] = {{0}};
uint8_t queuedPriority[MAX_QUEUED_APPROACHES] = {0};
uint8_t queuedCount = 0;


// ============================================================
// PHASE 3: NODE HEALTH, AS REPORTED BY THE ICU
//
// The console shows this so a commissioning engineer can stand at the
// cabinet and see the geometry the ICU is actually reasoning with,
// without a laptop.
//
// That matters more than it sounds. If the two nodes on an approach are
// provisioned with swapped distances, the ICU's direction logic inverts:
// it preempts for traffic LEAVING the junction and ignores traffic
// arriving. Nothing in any log looks wrong, no error is raised, and the
// system simply behaves backwards -- silently and symmetrically. That is
// spec defect S1-01, and it survived a full review of v1.0 undetected.
//
// Printing FAR/NEAR next to the surveyed metres makes it a five-second
// visual check instead of an invisible failure.
// ============================================================

struct NodeHealthView
{
  uint8_t  role;        // 0 unknown, 1 FAR, 2 NEAR
  uint8_t  state;       // NodeHealth values, 0 = never reported
  uint16_t distanceM;
  bool     reported;
};

// [lane][node], both 1-based; index 0 unused.
NodeHealthView nodeHealth[4][3];

// True if ANY node on ANY approach is in a state that needs attention.
// Drives a persistent banner rather than a transient message, because a
// degraded node is a condition, not an event.
bool anyNodeDegraded = false;


// ============================================================
// TIMING
// ============================================================

// FAR flashes while the TCU clears the
// intersection (5s yellow + 2s all-red)
const unsigned long FAR_BLINK_DURATION =
  7000;

// NEAR flashes for the whole TCU
// priority green window
const unsigned long NEAR_BLINK_DURATION =
  20000;

// Both remain solid while the TCU winds
// back down (5s yellow + 2s all-red)
const unsigned long COMPLETE_HOLD_DURATION =
  7000;

// Seconds shown on screen, derived so the
// display can never drift from the timers
const int FAR_SECONDS =
  FAR_BLINK_DURATION / 1000;

const int NEAR_SECONDS =
  NEAR_BLINK_DURATION / 1000;

const int COMPLETE_SECONDS =
  COMPLETE_HOLD_DURATION / 1000;

// Siren silences itself if no operator
// acknowledges within this window
const unsigned long AUTO_ACK_DELAY =
  10000;

// PHASE 3: LOCAL SAFETY BACKSTOP.
//
// Maximum time an alert may stand, enforced by the ERC alone, with no
// reference to the ICU.
//
// This is NOT the normal way an alert ends -- normally the ICU sends
// RELEASE. It covers the case the link timeout cannot: an ICU that is
// alive and still PINGing but whose decision loop is wedged. From here
// that looks exactly like a genuine emergency lasting an hour, and
// without a local bound the console would hold the alert forever.
//
// Spec Scenario 1 section 11.1 requires a hard maximum enforced
// independently of the decision logic, and never optional.
//
// Five minutes is deliberately generous. This bounds a FAULT, not an
// ambulance. Set it near a plausible journey time and it fires during
// normal slow approaches in heavy traffic, which turns a safety net into
// a bug and trains everyone to ignore it.
//
// [FIELD] The real value belongs to the traffic authority, alongside
// MAX_PREEMPT_DURATION and the preemptions-per-hour cap (spec decision
// D11). Five minutes is a working default, not an agreed figure.
const unsigned long MAX_ALERT_DURATION =
  300000;

bool emergencyAcknowledged = false;

unsigned long emergencyStartMillis = 0;

// LED blink interval
const unsigned long PRIORITY_BLINK_INTERVAL =
  500;

unsigned long priorityStageStart = 0;
unsigned long lastPriorityBlink = 0;

bool priorityBlinkState = false;


// ============================================================
// COUNTDOWN
// ============================================================

int lastDisplayedCountdown = -1;


// ============================================================
// ACTIVE LED PINS
// ============================================================

int activeFarPin = -1;
int activeNearPin = -1;


// ============================================================
// SIREN
// ============================================================

bool sirenActive = false;
bool buzzerState = false;

unsigned long lastSirenToggle = 0;

const unsigned long SIREN_INTERVAL =
  120;


// ============================================================
// ENGINEERING LED TEST
// ============================================================

bool ledTestActive = false;

int activeLEDPin = -1;

bool ledState = false;

int ledToggleCount = 0;

unsigned long lastLEDToggle = 0;

const unsigned long LED_INTERVAL =
  150;

const int LED_TOTAL_TOGGLES =
  12;


// ============================================================
// TEST BUTTON LONG PRESS
// ============================================================

bool testButtonHolding = false;
bool testModeTriggered = false;

unsigned long testButtonPressStart = 0;

const unsigned long TEST_HOLD_TIME =
  3000;


// ============================================================
// TOUCH STATE
// ============================================================

bool previousTouchState = false;


// ============================================================
// FORWARD DECLARATIONS
// ============================================================

void drawMainScreen();
void drawEmergencyScreen();
void drawTestScreen();

void allLEDsOff();

void safeWrite(int pin, bool on);
void handleNodeCommand(String command);
void updateCountdownCell();
void drawLiveSegments();

// UI toolkit
uint16_t mixColor(uint16_t a, uint16_t b, uint8_t t);
void vGradient(int x, int y, int w, int h, uint16_t top, uint16_t bottom);
void drawPanel(int x, int y, int w, int h, uint16_t fill, uint16_t border);
int  trackedWidth(const char *text, int size, int track);
void trackedText(const char *text, int x, int y, int size, uint16_t color, int track);
void centerTracked(const char *text, int cx, int y, int size, uint16_t color, int track);
void drawSoftButton(int x, int y, int w, int h, uint16_t base, uint16_t border,
                    const char *label, int size, uint16_t textColor);
void drawPill(int x, int y, int w, int h, uint16_t dot, const char *label, uint16_t textColor);
void drawSegments(int x, int y, int w, int h, int activeIndex, int percent);
void drawAlertGlyph(int cx, int cy, int r, uint16_t color);
void drawEdgeRails(bool on);
void drawMainStatusLine();
void updateMainHeartbeat();
void drawReturnButton();

// Heartbeat dot state for the idle screen
bool mainHeartbeatOn = true;
unsigned long lastHeartbeatUpdate = 0;

// ------------------------------------------------------------
// SOFT CLOCK
//
// There is no RTC on this board. The clock
// free-runs from millis() and is corrected
// whenever the ICU sends:
//     TIME:HH:MM:SS
//     DATE:14 MAY 2026
// Until then it shows placeholders rather
// than inventing a time.
// ------------------------------------------------------------

// Tracks what the live region last rendered, so a
// per-second refresh only repaints the countdown
// rather than the entire data row.
int  lastLiveStage = -1;
bool lastLiveAcked = false;

bool clockValid = false;
int  clockH = 0;
int  clockM = 0;
int  clockS = 0;

unsigned long clockAnchor = 0;
long          clockBase   = 0;

char dateText[16] = "-- --- ----";

void drawStatusBar();
void drawLogoMark(int cx, int cy, int r);
void drawSplashScreen();
void updateEmergencyLive();
void drawFooterBar(const char *primaryLabel, uint16_t primaryColor);
int  activeLaneNumber();
const char *activeApproachName();
const char *phaseLabel();

void drawPhaseArea(
  const char *phaseText,
  int secondsRemaining
);

void drawAckButton();
void drawAcknowledgedBanner();

void startLEDTest(int pin);
void updateLEDTest();

void startSiren();
void stopSiren();
void updateSiren();

void processMainTouch(int x, int y);
void processEmergencyTouch(int x, int y);
void processTestTouch(int x, int y);
void processWifiListTouch(int x, int y);
void processWifiKeyTouch(int x, int y);
void drawWifiListScreen();
void drawWifiKeyScreen();
void drawOfflineBanner();
void requestWifiScan();
void sendWifiJoin();
void drawPassField();
void drawKeyboard();

void enterTestMode();
void returnToMainScreen();

void processICUCommunication();
void handleIcuLine(String command);
void receiveEmergencyCommand(String command);

// PHASE 3 handlers
void handleAlertCommand(String command);
void handleHoldCommand(String command);
void handleReleaseCommand(String command);
void handleHealthCommand(String command);
void handleQueueCommand(String command);
void drawQueueLine();
const char *evidenceShortName(uint8_t e);
void drawMainStatusLine();

void acknowledgeEmergency();
void updatePrioritySequence();

void resetToNormalOperation();

void drawCountdown(
  const char *phaseText,
  int secondsRemaining
);

void drawCompleteScreen(
  int secondsRemaining
);

void centerText(
  const char *text,
  int centerX,
  int y,
  int textSize,
  uint16_t color
);


// ============================================================
// TOUCH RESET
// ============================================================

void resetTouchController()
{
  pinMode(
    TP_RST,
    OUTPUT
  );

  digitalWrite(
    TP_RST,
    LOW
  );

  delay(20);

  digitalWrite(
    TP_RST,
    HIGH
  );

  delay(200);
}


// ============================================================
// TOUCH REGISTER
// ============================================================

uint8_t touchReadRegister(
  uint8_t reg
)
{
  Wire.beginTransmission(
    FT6336_ADDR
  );

  Wire.write(reg);

  if (
    Wire.endTransmission(false) != 0
  )
  {
    return 0;
  }

  Wire.requestFrom(
    FT6336_ADDR,
    1
  );

  if (Wire.available())
  {
    return Wire.read();
  }

  return 0;
}


// ============================================================
// RAW TOUCH
// ============================================================

bool getRawTouch(
  uint16_t &rawX,
  uint16_t &rawY
)
{
  uint8_t touches =
    touchReadRegister(0x02);

  if (
    (touches & 0x0F) == 0
  )
  {
    return false;
  }

  Wire.beginTransmission(
    FT6336_ADDR
  );

  Wire.write(0x03);

  if (
    Wire.endTransmission(false) != 0
  )
  {
    return false;
  }

  Wire.requestFrom(
    FT6336_ADDR,
    4
  );

  if (
    Wire.available() < 4
  )
  {
    return false;
  }

  uint8_t xh = Wire.read();
  uint8_t xl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t yl = Wire.read();

  rawX =
    ((xh & 0x0F) << 8) | xl;

  rawY =
    ((yh & 0x0F) << 8) | yl;

  return true;
}


// ============================================================
// LANDSCAPE TOUCH CONVERSION
// ============================================================

bool getTouch(
  uint16_t &x,
  uint16_t &y
)
{
  uint16_t rawX;
  uint16_t rawY;

  if (
    !getRawTouch(
      rawX,
      rawY
    )
  )
  {
    return false;
  }

  x = rawY;
  y = 319 - rawX;

  if (x > 479)
  {
    x = 479;
  }

  if (y > 319)
  {
    y = 319;
  }

  return true;
}


// ============================================================
// TOUCH RECTANGLE
// ============================================================

bool inside(
  int tx,
  int ty,
  int x,
  int y,
  int w,
  int h
)
{
  return (
    tx >= x &&
    tx <= (x + w) &&
    ty >= y &&
    ty <= (y + h)
  );
}



// ============================================================
// ============================================================
//  UI TOOLKIT
//
//  Small drawing primitives that give the
//  console a consistent visual language:
//  gradients, cards, letter-spaced type,
//  bevelled buttons and progress segments.
// ============================================================
// ============================================================


// ------------------------------------------------------------
// Blend two RGB565 colours. t = 0 gives a,
// t = 255 gives b. Used for gradients and
// for deriving highlight / shadow edges.
// ------------------------------------------------------------

uint16_t mixColor(
  uint16_t a,
  uint16_t b,
  uint8_t t
)
{
  int ar = (a >> 11) & 0x1F;
  int ag = (a >> 5)  & 0x3F;
  int ab =  a        & 0x1F;

  int br = (b >> 11) & 0x1F;
  int bg = (b >> 5)  & 0x3F;
  int bb =  b        & 0x1F;

  int r = ar + (((br - ar) * t) / 255);
  int g = ag + (((bg - ag) * t) / 255);
  int c = ab + (((bb - ab) * t) / 255);

  return (uint16_t)((r << 11) | (g << 5) | c);
}


// ------------------------------------------------------------
// Vertical gradient fill.
// ------------------------------------------------------------

void vGradient(
  int x,
  int y,
  int w,
  int h,
  uint16_t top,
  uint16_t bottom
)
{
  int span =
    (h > 1)
      ? (h - 1)
      : 1;

  for (int i = 0; i < h; i++)
  {
    gfx->fillRect(
      x,
      y + i,
      w,
      1,
      mixColor(
        top,
        bottom,
        (uint8_t)((i * 255) / span)
      )
    );
  }
}


// ------------------------------------------------------------
// Card surface with hairline border and a
// soft top highlight for a sense of depth.
// ------------------------------------------------------------

void drawPanel(
  int x,
  int y,
  int w,
  int h,
  uint16_t fill,
  uint16_t border
)
{
  gfx->fillRoundRect(
    x,
    y,
    w,
    h,
    8,
    fill
  );

  gfx->drawRoundRect(
    x,
    y,
    w,
    h,
    8,
    border
  );

  gfx->fillRect(
    x + 9,
    y + 1,
    w - 18,
    1,
    mixColor(fill, C_WHITE, 26)
  );
}


// ------------------------------------------------------------
// Letter-spaced text.
//
// The built-in 5x7 font looks cramped when
// scaled up. Adding tracking to headings is
// the single biggest readability win here.
// ------------------------------------------------------------

int trackedWidth(
  const char *text,
  int size,
  int track
)
{
  int n = strlen(text);

  if (n == 0)
  {
    return 0;
  }

  return (n * 6 * size) + ((n - 1) * track);
}


void trackedText(
  const char *text,
  int x,
  int y,
  int size,
  uint16_t color,
  int track
)
{
  gfx->setTextSize(size);
  gfx->setTextColor(color);

  int cx = x;

  for (const char *p = text; *p; p++)
  {
    char one[2];

    one[0] = *p;
    one[1] = 0;

    gfx->setCursor(cx, y);
    gfx->print(one);

    cx += (6 * size) + track;
  }
}


void centerTracked(
  const char *text,
  int centerX,
  int y,
  int size,
  uint16_t color,
  int track
)
{
  trackedText(
    text,
    centerX - (trackedWidth(text, size, track) / 2),
    y,
    size,
    color,
    track
  );
}


// ------------------------------------------------------------
// Bevelled button: flat fill, lighter top
// edge, darker bottom edge, hairline border.
// ------------------------------------------------------------

void drawSoftButton(
  int x,
  int y,
  int w,
  int h,
  uint16_t base,
  uint16_t border,
  const char *label,
  int size,
  uint16_t textColor
)
{
  gfx->fillRoundRect(
    x,
    y,
    w,
    h,
    7,
    base
  );

  gfx->drawRoundRect(
    x,
    y,
    w,
    h,
    7,
    border
  );

  gfx->fillRect(
    x + 8,
    y + 1,
    w - 16,
    1,
    mixColor(base, C_WHITE, 55)
  );

  gfx->fillRect(
    x + 8,
    y + h - 2,
    w - 16,
    1,
    mixColor(base, C_BLACK, 70)
  );

  centerTracked(
    label,
    x + (w / 2),
    y + ((h - (8 * size)) / 2),
    size,
    textColor,
    1
  );
}


// ------------------------------------------------------------
// Status pill with a leading state dot.
// ------------------------------------------------------------

void drawPill(
  int x,
  int y,
  int w,
  int h,
  uint16_t dotColor,
  const char *label,
  uint16_t textColor
)
{
  gfx->fillRoundRect(
    x,
    y,
    w,
    h,
    h / 2,
    C_RAISED
  );

  gfx->drawRoundRect(
    x,
    y,
    w,
    h,
    h / 2,
    C_BORDER
  );

  gfx->fillCircle(
    x + 13,
    y + (h / 2),
    4,
    dotColor
  );

  trackedText(
    label,
    x + 24,
    y + ((h - 8) / 2),
    1,
    textColor,
    1
  );
}


// ------------------------------------------------------------
// Three-segment phase progress bar.
//
// Completed phases fill solid, the running
// phase fills proportionally, the rest stay
// as empty tracks.
// ------------------------------------------------------------

void drawSegments(
  int x,
  int y,
  int w,
  int h,
  int activeIndex,
  int percent
)
{
  int gap  = 7;
  int segW = (w - (2 * gap)) / 3;

  for (int i = 0; i < 3; i++)
  {
    int sx = x + (i * (segW + gap));

    gfx->fillRect(
      sx,
      y,
      segW,
      h,
      C_BORDER
    );

    int fillW = 0;

    uint16_t c = C_AMBER;

    if (i < activeIndex)
    {
      fillW = segW;
      c     = C_OK;
    }

    else if (i == activeIndex)
    {
      fillW = (segW * percent) / 100;

      if (fillW > segW)
      {
        fillW = segW;
      }
    }

    if (fillW > 0)
    {
      gfx->fillRect(
        sx,
        y,
        fillW,
        h,
        c
      );
    }
  }
}


// ------------------------------------------------------------
// Warning triangle glyph.
// ------------------------------------------------------------

void drawAlertGlyph(
  int cx,
  int cy,
  int r,
  uint16_t color
)
{
  for (int i = 0; i < r; i++)
  {
    gfx->fillRect(
      cx - i,
      cy + i - (r / 2),
      (2 * i) + 1,
      1,
      color
    );
  }

  gfx->fillRect(
    cx - 1,
    cy - (r / 2) + 5,
    3,
    r - 10,
    C_ALERT_DK
  );

  gfx->fillRect(
    cx - 1,
    cy + (r / 2) - 4,
    3,
    3,
    C_ALERT_DK
  );
}


// ------------------------------------------------------------
// Pulsing side rails on the emergency screen.
// Driven by the same blink flag as the LEDs,
// so the panel and the console agree.
// ------------------------------------------------------------

void drawEdgeRails(
  bool on
)
{
  uint16_t c =
    on
      ? C_ALERT
      : mixColor(C_ALERT, C_BG, 200);

  gfx->fillRect(0,   50, 5, 262, c);
  gfx->fillRect(475, 50, 5, 262, c);
}


// ============================================================
// GREENWAVE LOGO MARK
//
// Drawn procedurally: gradient ring, leaf
// S-curve, signal arcs and circuit traces.
// ============================================================

#define LOGO_LEAF  0x4D6A   // #4CAF50
#define LOGO_TEAL  0x044F   // #00897B


void drawLogoArc(
  int cx,
  int cy,
  int r,
  int startDeg,
  int endDeg,
  int thickness,
  uint16_t color
)
{
  for (int a = startDeg; a <= endDeg; a++)
  {
    float rad = a * 0.01745329f;

    int px = cx + (int)(cosf(rad) * r);
    int py = cy + (int)(sinf(rad) * r);

    gfx->fillRect(
      px,
      py,
      thickness,
      thickness,
      color
    );
  }
}


void drawLogoMark(
  int cx,
  int cy,
  int r
)
{
  // Real artwork, blitted from greenwave_logo.h.
  // The radius argument selects which prepared
  // size to use rather than drawing shapes.

  if (r >= 30)
  {
    gfx->draw16bitRGBBitmap(
      cx - (LOGO_MARK_W / 2),
      cy - (LOGO_MARK_H / 2),
      (uint16_t *)LOGO_MARK,
      LOGO_MARK_W,
      LOGO_MARK_H
    );
  }

  else
  {
    gfx->draw16bitRGBBitmap(
      cx - (LOGO_TINY_W / 2),
      cy - (LOGO_TINY_H / 2),
      (uint16_t *)LOGO_TINY,
      LOGO_TINY_W,
      LOGO_TINY_H
    );
  }
}



// ============================================================
// ============================================================
//  WIFI PROVISIONING SCREENS
// ============================================================
// ============================================================

const char *KEYS_LOWER[4] =
{
  "1234567890",
  "qwertyuiop",
  "asdfghjkl",
  "zxcvbnm"
};

const char *KEYS_UPPER[4] =
{
  "1234567890",
  "QWERTYUIOP",
  "ASDFGHJKL",
  "ZXCVBNM"
};

const char *KEYS_SYM[4] =
{
  "1234567890",
  "!@#$%^&*()",
  "-_=+[]{};",
  ":'\",.?/~"
};


int keyRowY(int row)
{
  return 80 + (row * 44);
}


// ------------------------------------------------------------
// Big offline call to action on the idle screen
// ------------------------------------------------------------

void drawOfflineBanner()
{
  gfx->fillRect(12, 168, 456, 52, C_PANEL);

  drawSoftButton(
    20, 172, 440, 44,
    C_ALERT,
    mixColor(C_ALERT, C_WHITE, 60),
    "ICU OFFLINE - TAP TO SET UP WIFI",
    2,
    C_WHITE
  );
}


// ------------------------------------------------------------
// Ask the ICU to scan
// ------------------------------------------------------------

void requestWifiScan()
{
  netCount    = 0;
  netPage     = 0;
  netSelected = -1;
  scanRunning = true;

  ICU_UART.println("NET:SCAN");
  ICU_UART.flush();

  Serial.println("WIFI SCAN REQUESTED");
}


void sendWifiJoin()
{
  ICU_UART.print("NET:SSID:");
  ICU_UART.println(netSsid[netSelected]);

  ICU_UART.print("NET:PASS:");
  ICU_UART.println(passBuf);

  ICU_UART.println("NET:CONNECT");
  ICU_UART.flush();

  strncpy(wifiStatus, "CONNECTING", sizeof(wifiStatus) - 1);

  Serial.print("JOIN REQUEST: ");
  Serial.println(netSsid[netSelected]);
}


// ------------------------------------------------------------
// NETWORK LIST
// ------------------------------------------------------------

void drawWifiListScreen()
{
  gfx->fillScreen(C_BG);

  vGradient(0, 0, 480, 42, C_PANEL, C_BG);
  gfx->fillRect(0, 42, 480, 2, C_ACCENT_DIM);

  trackedText("WIFI SETUP", 16, 8, 2, C_TEXT, 2);
  trackedText("SELECT A NETWORK FOR THE ICU", 16, 28, 1, C_MUTED, 1);

  drawPill(
    346, 10, 120, 22,
    icuOnline ? C_OK : C_ALERT,
    icuOnline ? "ONLINE" : "OFFLINE",
    C_TEXT
  );

  drawPanel(8, 50, 464, 202, C_PANEL, C_BORDER);

  if (scanRunning && netCount == 0)
  {
    centerTracked("SCANNING...", 240, 140, 2, C_MUTED, 3);
  }

  else if (netCount == 0)
  {
    centerTracked("NO NETWORKS FOUND", 240, 132, 2, C_MUTED, 2);
    centerTracked("TAP RESCAN TO TRY AGAIN", 240, 158, 1, C_MUTED, 1);
  }

  else
  {
    int start = netPage * NETS_PER_PAGE;

    for (int i = 0; i < NETS_PER_PAGE; i++)
    {
      int idx = start + i;

      if (idx >= netCount)
      {
        break;
      }

      int ry = 58 + (i * 38);

      gfx->fillRoundRect(16, ry, 448, 34, 6, C_RAISED);
      gfx->drawRoundRect(16, ry, 448, 34, 6, C_BORDER);

      // Signal strength bars
      int bars = 1;

      if (netRssi[idx] > -80) bars = 2;
      if (netRssi[idx] > -70) bars = 3;
      if (netRssi[idx] > -60) bars = 4;

      for (int b = 0; b < 4; b++)
      {
        gfx->fillRect(
          28 + (b * 5),
          ry + 24 - (b * 4) - 2,
          3,
          4 + (b * 4),
          (b < bars) ? C_OK : C_BORDER
        );
      }

      trackedText(netSsid[idx], 60, ry + 12, 1, C_TEXT, 1);

      if (netLock[idx])
      {
        trackedText("LOCKED", 392, ry + 12, 1, C_MUTED, 1);
      }
      else
      {
        trackedText("OPEN", 400, ry + 12, 1, C_OK, 1);
      }
    }
  }

  // Footer
  drawSoftButton(16,  262, 136, 40, C_RAISED, C_ACCENT_DIM, "RESCAN", 2, C_TEXT);
  drawSoftButton(172, 262, 136, 40, C_RAISED, C_BORDER,     "MORE",   2, C_MUTED);
  drawSoftButton(328, 262, 136, 40, C_RAISED, C_BORDER,     "BACK",   2, C_MUTED);
}


// ------------------------------------------------------------
// PASSWORD ENTRY
// ------------------------------------------------------------

void drawPassField()
{
  gfx->fillRect(12, 34, 456, 38, C_BG);

  gfx->fillRoundRect(12, 34, 456, 36, 6, C_RAISED);
  gfx->drawRoundRect(12, 34, 456, 36, 6, C_ACCENT_DIM);

  if (passLen == 0)
  {
    trackedText("ENTER PASSWORD", 24, 46, 1, C_MUTED, 1);
  }

  else
  {
    // Show the password so it can actually be checked
    char shown[42];

    int n = passLen;

    if (n > 40)
    {
      n = 40;
    }

    strncpy(shown, passBuf + (passLen - n), n);
    shown[n] = '\0';

    trackedText(shown, 24, 44, 1, C_TEXT, 2);
  }
}


void drawKeyboard()
{
  const char **rows =
    keySymbols
      ? KEYS_SYM
      : (keyShift ? KEYS_UPPER : KEYS_LOWER);

  for (int r = 0; r < 4; r++)
  {
    int n  = strlen(rows[r]);
    int ry = keyRowY(r);

    for (int c = 0; c < n; c++)
    {
      int kx = 2 + (c * 47);

      gfx->fillRoundRect(kx, ry, 44, 40, 5, C_RAISED);
      gfx->drawRoundRect(kx, ry, 44, 40, 5, C_BORDER);

      char lab[2];

      lab[0] = rows[r][c];
      lab[1] = 0;

      centerTracked(lab, kx + 22, ry + 13, 2, C_TEXT, 0);
    }
  }

  // Backspace sits at the end of row 3
  gfx->fillRoundRect(332, keyRowY(3), 66, 40, 5, C_RAISED);
  gfx->drawRoundRect(332, keyRowY(3), 66, 40, 5, C_BORDER);
  centerTracked("DEL", 365, keyRowY(3) + 16, 1, C_MUTED, 2);

  // Bottom row
  drawSoftButton(2,   268, 78,  42, C_RAISED, C_BORDER, keyShift ? "ABC" : "SHIFT", 1, C_TEXT);
  drawSoftButton(84,  268, 78,  42, C_RAISED, C_BORDER, keySymbols ? "abc" : "?123", 1, C_TEXT);
  drawSoftButton(166, 268, 106, 42, C_RAISED, C_BORDER, "SPACE", 1, C_MUTED);
  drawSoftButton(276, 268, 96,  42, C_RAISED, C_BORDER, "CANCEL", 1, C_MUTED);
  drawSoftButton(376, 268, 102, 42, C_OK_DK,  C_OK,     "CONNECT", 1, C_TEXT);
}


void drawWifiKeyScreen()
{
  gfx->fillScreen(C_BG);

  char title[48];

  snprintf(title, sizeof(title), "PASSWORD FOR %s",
           (netSelected >= 0) ? netSsid[netSelected] : "");

  trackedText(title, 12, 10, 1, C_MUTED, 1);

  drawPassField();
  drawKeyboard();
}


// ------------------------------------------------------------
// TOUCH: NETWORK LIST
// ------------------------------------------------------------

void processWifiListTouch(
  int x,
  int y
)
{
  if (inside(x, y, 16, 262, 136, 40))
  {
    requestWifiScan();
    drawWifiListScreen();
    return;
  }

  if (inside(x, y, 172, 262, 136, 40))
  {
    int pages = (netCount + NETS_PER_PAGE - 1) / NETS_PER_PAGE;

    if (pages > 0)
    {
      netPage = (netPage + 1) % pages;
      drawWifiListScreen();
    }

    return;
  }

  if (inside(x, y, 328, 262, 136, 40))
  {
    currentScreen = ERC_MAIN;
    drawMainScreen();
    return;
  }

  // Network rows
  int start = netPage * NETS_PER_PAGE;

  for (int i = 0; i < NETS_PER_PAGE; i++)
  {
    int idx = start + i;

    if (idx >= netCount)
    {
      break;
    }

    if (inside(x, y, 16, 58 + (i * 38), 448, 34))
    {
      netSelected = idx;

      passBuf[0]  = '\0';
      passLen     = 0;
      keyShift    = false;
      keySymbols  = false;

      if (!netLock[idx])
      {
        // Open network: join immediately
        sendWifiJoin();

        currentScreen = ERC_MAIN;
        drawMainScreen();
        return;
      }

      currentScreen = ERC_WIFI_KEY;
      drawWifiKeyScreen();
      return;
    }
  }
}


// ------------------------------------------------------------
// TOUCH: KEYBOARD
// ------------------------------------------------------------

void processWifiKeyTouch(
  int x,
  int y
)
{
  // Bottom row first
  if (y >= 268)
  {
    if (inside(x, y, 2, 268, 78, 42))
    {
      keyShift = !keyShift;
      drawKeyboard();
      return;
    }

    if (inside(x, y, 84, 268, 78, 42))
    {
      keySymbols = !keySymbols;
      drawKeyboard();
      return;
    }

    if (inside(x, y, 166, 268, 106, 42))
    {
      if (passLen < 64)
      {
        passBuf[passLen++] = ' ';
        passBuf[passLen]   = '\0';
        drawPassField();
      }

      return;
    }

    if (inside(x, y, 276, 268, 96, 42))
    {
      currentScreen = ERC_WIFI_LIST;
      drawWifiListScreen();
      return;
    }

    if (inside(x, y, 376, 268, 102, 42))
    {
      if (netSelected >= 0)
      {
        sendWifiJoin();
      }

      currentScreen = ERC_MAIN;
      drawMainScreen();
      return;
    }

    return;
  }

  // Backspace
  if (inside(x, y, 332, keyRowY(3), 66, 40))
  {
    if (passLen > 0)
    {
      passBuf[--passLen] = '\0';
      drawPassField();
    }

    return;
  }

  // Character keys
  const char **rows =
    keySymbols
      ? KEYS_SYM
      : (keyShift ? KEYS_UPPER : KEYS_LOWER);

  for (int r = 0; r < 4; r++)
  {
    int ry = keyRowY(r);

    if (y < ry || y > ry + 40)
    {
      continue;
    }

    int n = strlen(rows[r]);

    for (int c = 0; c < n; c++)
    {
      int kx = 2 + (c * 47);

      if (x >= kx && x <= kx + 44)
      {
        if (passLen < 64)
        {
          passBuf[passLen++] = rows[r][c];
          passBuf[passLen]   = '\0';

          drawPassField();
        }

        return;
      }
    }
  }
}


// ============================================================
// CENTER TEXT
// ============================================================

void centerText(
  const char *text,
  int centerX,
  int y,
  int textSize,
  uint16_t color
)
{
  gfx->setTextSize(
    textSize
  );

  gfx->setTextColor(
    color
  );

  int textWidth =
    strlen(text) *
    6 *
    textSize;

  gfx->setCursor(
    centerX -
    textWidth / 2,
    y
  );

  gfx->print(text);
}


// ============================================================
// BUTTON
// ============================================================

void drawButton(
  int x,
  int y,
  int w,
  int h,
  uint16_t fillColor,
  const char *text
)
{
  drawSoftButton(
    x,
    y,
    w,
    h,
    fillColor,
    mixColor(fillColor, C_WHITE, 70),
    text,
    2,
    C_TEXT
  );
}


// ============================================================
// TWO-LINE TEST BUTTON
// ============================================================

void drawTestButton(
  int x,
  int y,
  int w,
  int h,
  uint16_t color,
  const char *line1,
  const char *line2
)
{
  // Dark chip, colour used as an accent rather
  // than a flat fill - reads far cleaner.
  gfx->fillRoundRect(
    x,
    y,
    w,
    h,
    7,
    C_RAISED
  );

  gfx->drawRoundRect(
    x,
    y,
    w,
    h,
    7,
    mixColor(color, C_BG, 120)
  );

  // Accent stripe down the left edge
  gfx->fillRect(
    x + 3,
    y + 8,
    3,
    h - 16,
    color
  );

  trackedText(
    line1,
    x + 16,
    y + 11,
    1,
    C_MUTED,
    2
  );

  trackedText(
    line2,
    x + 16,
    y + 27,
    2,
    C_TEXT,
    1
  );

  // Indicator dot, top right
  gfx->fillCircle(
    x + w - 16,
    y + 17,
    5,
    mixColor(color, C_BG, 90)
  );

  gfx->drawCircle(
    x + w - 16,
    y + 17,
    5,
    color
  );
}


// ============================================================
// ALL LEDs OFF
// ============================================================

void allLEDsOff()
{
  digitalWrite(
    LEFT_FAR_LED,
    LOW
  );

  digitalWrite(
    LEFT_NEAR_LED,
    LOW
  );

  digitalWrite(
    RIGHT_FAR_LED,
    LOW
  );

  digitalWrite(
    RIGHT_NEAR_LED,
    LOW
  );

  digitalWrite(
    BOTTOM_FAR_LED,
    LOW
  );

  digitalWrite(
    BOTTOM_NEAR_LED,
    LOW
  );
}


// ============================================================
// GUARDED LED WRITE
//
// Never writes to an unassigned (-1) pin.
// ============================================================

void safeWrite(
  int pin,
  bool on
)
{
  if (pin < 0)
  {
    return;
  }

  digitalWrite(
    pin,
    on
      ? HIGH
      : LOW
  );
}


// ============================================================
// START ENGINEERING LED TEST
// ============================================================

void startLEDTest(
  int pin
)
{
  allLEDsOff();

  activeLEDPin = pin;

  ledTestActive = true;

  ledState = true;

  ledToggleCount = 0;

  lastLEDToggle =
    millis();

  digitalWrite(
    activeLEDPin,
    HIGH
  );
}


// ============================================================
// UPDATE ENGINEERING LED TEST
// ============================================================

void updateLEDTest()
{
  if (
    !ledTestActive
  )
  {
    return;
  }

  unsigned long now =
    millis();

  if (
    now - lastLEDToggle >=
    LED_INTERVAL
  )
  {
    lastLEDToggle = now;

    ledState =
      !ledState;

    digitalWrite(
      activeLEDPin,
      ledState
        ? HIGH
        : LOW
    );

    ledToggleCount++;

    if (
      ledToggleCount >=
      LED_TOTAL_TOGGLES
    )
    {
      digitalWrite(
        activeLEDPin,
        LOW
      );

      ledTestActive =
        false;

      activeLEDPin =
        -1;

      ledState =
        false;
    }
  }
}


// ============================================================
// START SIREN
// ============================================================

void startSiren()
{
  sirenActive = true;

  buzzerState = true;

  lastSirenToggle =
    millis();

  digitalWrite(
    BUZZER,
    HIGH
  );

  Serial.println(
    "WARNING BUZZER STARTED"
  );
}


// ============================================================
// STOP SIREN
// ============================================================

void stopSiren()
{
  sirenActive = false;

  buzzerState = false;

  digitalWrite(
    BUZZER,
    LOW
  );

  Serial.println(
    "BUZZER STOPPED"
  );
}


// ============================================================
// UPDATE SIREN
// ============================================================

void updateSiren()
{
  if (
    !sirenActive
  )
  {
    return;
  }

  unsigned long now =
    millis();

  if (
    now - lastSirenToggle >=
    SIREN_INTERVAL
  )
  {
    lastSirenToggle =
      now;

    buzzerState =
      !buzzerState;

    digitalWrite(
      BUZZER,
      buzzerState
        ? HIGH
        : LOW
    );
  }
}


// ============================================================
// DRAW AMBULANCE
// ============================================================

void drawAmbulance(
  int x,
  int y
)
{
  // Side elevation of a box-body ambulance.
  // Drawn in layers: shadow, chassis, body
  // panels, glazing, livery, then wheels.

  uint16_t body    = 0xFFFF;
  uint16_t bodySh  = mixColor(0xFFFF, C_PANEL, 70);
  uint16_t bodyDk  = mixColor(0xFFFF, C_PANEL, 130);
  uint16_t glass   = mixColor(C_INFO,  C_WHITE, 90);
  uint16_t glassDk = mixColor(C_INFO,  C_BLACK, 60);
  uint16_t tyre    = mixColor(C_BLACK, C_PANEL, 60);


  // Ground shadow
  gfx->fillRoundRect(
    x + 8,
    y + 60,
    134,
    5,
    2,
    mixColor(C_PANEL, C_BLACK, 90)
  );


  // ---------- Rear box body ----------
  gfx->fillRoundRect(x + 48, y + 10, 96, 42, 4, body);

  // Lower shading band
  gfx->fillRect(x + 48, y + 42, 96, 10, bodySh);

  // Waist line
  gfx->fillRect(x + 48, y + 30, 96, 1, bodyDk);


  // ---------- Cab ----------
  gfx->fillRoundRect(x + 6, y + 24, 46, 28, 3, body);
  gfx->fillRect(x + 6, y + 44, 46, 8, bodySh);

  // Bonnet taper
  gfx->fillRect(x + 4, y + 32, 6, 14, body);
  gfx->fillRect(x + 4, y + 44, 6, 3, bodySh);

  // Windscreen, raked
  for (int i = 0; i < 13; i++)
  {
    gfx->fillRect(
      x + 12 + (i / 3),
      y + 27 + i,
      20 - (i / 3),
      1,
      (i < 3)
        ? glassDk
        : glass
    );
  }

  // Door glass
  gfx->fillRect(x + 36, y + 28, 13, 12, glass);
  gfx->fillRect(x + 36, y + 28, 13, 2,  glassDk);

  // Door seam + handle
  gfx->fillRect(x + 34, y + 26, 1, 24, bodyDk);
  gfx->fillRect(x + 40, y + 43, 6, 2,  bodyDk);


  // ---------- Rear compartment glazing ----------
  gfx->fillRect(x + 58, y + 16, 30, 14, glass);
  gfx->fillRect(x + 58, y + 16, 30, 2,  glassDk);
  gfx->fillRect(x + 72, y + 16, 1,  14, bodyDk);

  // Rear door seam
  gfx->fillRect(x + 122, y + 12, 1, 38, bodyDk);


  // ---------- Livery ----------
  // Battenburg-style stripe
  gfx->fillRect(x + 4, y + 33, 140, 8, C_ALERT);
  gfx->fillRect(x + 4, y + 33, 140, 1, mixColor(C_ALERT, C_BLACK, 70));

  for (int i = 0; i < 7; i++)
  {
    gfx->fillRect(
      x + 10 + (i * 20),
      y + 33,
      9,
      8,
      mixColor(C_ALERT, C_WHITE, 120)
    );
  }

  // Medical cross, rear quarter
  gfx->fillRect(x + 99, y + 14, 7, 21, C_ALERT);
  gfx->fillRect(x + 92, y + 21, 21, 7, C_ALERT);


  // ---------- Light bar ----------
  gfx->fillRoundRect(x + 76, y + 3, 40, 8, 3, mixColor(C_PANEL, C_BLACK, 40));
  gfx->fillRect(x + 79, y + 5, 12, 4, C_ALERT);
  gfx->fillRect(x + 94, y + 5, 6,  4, C_AMBER);
  gfx->fillRect(x + 103, y + 5, 11, 4, C_INFO);

  // Headlamp + grille
  gfx->fillRect(x + 3, y + 36, 4, 5, C_AMBER);
  gfx->fillRect(x + 3, y + 46, 5, 4, bodyDk);


  // ---------- Wheels ----------
  int wheelY = y + 52;

  // Arches
  gfx->fillCircle(x + 30,  wheelY, 14, C_PANEL);
  gfx->fillCircle(x + 112, wheelY, 14, C_PANEL);

  gfx->fillCircle(x + 30,  wheelY, 12, tyre);
  gfx->fillCircle(x + 112, wheelY, 12, tyre);

  gfx->fillCircle(x + 30,  wheelY, 6, mixColor(C_MUTED, C_WHITE, 90));
  gfx->fillCircle(x + 112, wheelY, 6, mixColor(C_MUTED, C_WHITE, 90));

  gfx->fillCircle(x + 30,  wheelY, 2, tyre);
  gfx->fillCircle(x + 112, wheelY, 2, tyre);
}



// ============================================================
// SHARED CHROME
// ============================================================

int activeLaneNumber()
{
  if (activeApproach == APPROACH_LEFT)   return 1;
  if (activeApproach == APPROACH_BOTTOM) return 2;
  if (activeApproach == APPROACH_RIGHT)  return 3;
  return 0;
}


const char *activeApproachName()
{
  if (activeApproach == APPROACH_LEFT)   return "LEFT";
  if (activeApproach == APPROACH_BOTTOM) return "BOTTOM";
  if (activeApproach == APPROACH_RIGHT)  return "RIGHT";
  return "---";
}


const char *phaseLabel()
{
  // PHASE 3: renamed to match what the stages now MEAN.
  //
  // The old labels described the traffic controller's internal phases --
  // "CLEARANCE" was the 5 s yellow plus 2 s all-red of a preempt cycle
  // this build no longer commands. They described a machine's state.
  //
  // These describe what the OPERATOR should do, which is the only thing
  // an advisory console has any business displaying.
  if (priorityStage == PRIORITY_FAR_BLINK)  return "PREPARE";
  if (priorityStage == PRIORITY_NEAR_BLINK) return "COMMIT";
  if (priorityStage == PRIORITY_COMPLETE)   return "CLEARING";
  return "STANDBY";
}


// ------------------------------------------------------------
// Advance the soft clock from millis().
// ------------------------------------------------------------

void serviceClock()
{
  if (!clockValid)
  {
    return;
  }

  long elapsed =
    (long)((millis() - clockAnchor) / 1000UL);

  long total =
    (clockBase + elapsed) % 86400L;

  clockH = total / 3600;
  clockM = (total / 60) % 60;
  clockS = total % 60;
}


// ------------------------------------------------------------
// Top status bar: date left, time right.
// ------------------------------------------------------------

void drawStatusBar()
{
  gfx->fillRect(0, 0, 480, 26, C_PANEL);
  gfx->fillRect(0, 26, 480, 1, C_BORDER);

  gfx->draw16bitRGBBitmap(
    10,
    3,
    (uint16_t *)LOGO_TINY,
    LOGO_TINY_W,
    LOGO_TINY_H
  );

  trackedText(
    dateText,
    38,
    9,
    1,
    C_TEXT,
    1
  );

  char timeText[16];

  if (clockValid)
  {
    int h12 = clockH % 12;

    if (h12 == 0)
    {
      h12 = 12;
    }

    sprintf(
      timeText,
      "%d:%02d:%02d %s",
      h12,
      clockM,
      clockS,
      (clockH < 12) ? "AM" : "PM"
    );
  }

  else
  {
    sprintf(timeText, "--:--:--");
  }

  int w = trackedWidth(timeText, 1, 1);

  trackedText(
    timeText,
    468 - w,
    9,
    1,
    C_TEXT,
    1
  );
}


// ------------------------------------------------------------
// Footer control bar, identical on every
// screen so the buttons never move.
// ------------------------------------------------------------

void drawFooterBar(
  const char *primaryLabel,
  uint16_t primaryColor
)
{
  // Clear the queue band too. drawFooterBar() runs on every screen
  // change, and a stale "ALSO WAITING" left behind on the idle screen
  // would claim a vehicle is queued when nothing is active at all.
  gfx->fillRect(0, 254, 480, 10, C_BG);

  gfx->fillRect(0, 264, 480, 56, C_BG);

  drawSoftButton(
    16, 272, 136, 38,
    primaryColor,
    mixColor(primaryColor, C_WHITE, 70),
    primaryLabel, 2, C_TEXT
  );

  drawSoftButton(
    172, 272, 136, 38,
    C_RAISED, C_BORDER,
    "TEST", 2, C_MUTED
  );

  drawSoftButton(
    328, 272, 136, 38,
    C_RAISED, C_BORDER,
    "LOGS", 2, C_MUTED
  );
}


// ------------------------------------------------------------
// Label above value, as used in the data row.
// ------------------------------------------------------------

void drawDataCell(
  int cx,
  int y,
  const char *label,
  const char *value,
  uint16_t valueColor,
  int valueSize
)
{
  centerTracked(label, cx, y, 1, C_MUTED, 2);
  centerTracked(value, cx, y + 16, valueSize, valueColor, 2);
}


// ============================================================
// SPLASH SCREEN
// ============================================================

void drawSplashScreen()
{
  gfx->fillScreen(C_BG);

  gfx->draw16bitRGBBitmap(
    (480 - LOGO_SPLASH_W) / 2,
    54,
    (uint16_t *)LOGO_SPLASH,
    LOGO_SPLASH_W,
    LOGO_SPLASH_H
  );

  gfx->fillRect(160, 214, 160, 1, C_BORDER);

  centerTracked(
    "EMERGENCY RESPONSE CONSOLE",
    240,
    228,
    1,
    C_MUTED,
    2
  );

  centerTracked(
    "INITIALISING",
    240,
    252,
    1,
    mixColor(C_MUTED, C_BG, 90),
    2
  );
}


// ============================================================
// DRAW MAIN SCREEN
// ============================================================

void drawMainScreen()
{
  gfx->fillScreen(C_BG);

  drawStatusBar();


  // Instrument card
  drawPanel(6, 32, 468, 226, C_PANEL, C_BORDER);


  // Branding block
  drawLogoMark(48, 76, 32);

  trackedText("GREENWAVE", 82, 60, 2, C_TEXT, 3);
  trackedText("TECHLABS",  82, 80, 1, LOGO_LEAF, 5);

  trackedText(
    "EMERGENCY RESPONSE CONSOLE",
    96,
    94,
    1,
    C_MUTED,
    1
  );

  gfx->fillRect(16, 112, 448, 1, C_BORDER);


  // Primary state
  centerTracked("SYSTEM STANDBY", 240, 126, 3, C_TEXT, 4);

  centerTracked(
    "MONITORING ALL APPROACHES",
    240,
    158,
    1,
    C_MUTED,
    2
  );


  // When the ICU has no internet the console's
  // job is to get it back online, so that takes
  // the place of the approach chips.
  if (!icuOnline)
  {
    drawOfflineBanner();

    gfx->fillRect(16, 228, 448, 1, C_BORDER);

    drawMainStatusLine();

    drawFooterBar("ACK", C_RAISED);

    return;
  }

  // Approach chips
  const char *chipLabels[3] = { "LANE 1", "LANE 2", "LANE 3" };
  const char *chipNames[3]  = { "LEFT", "BOTTOM", "RIGHT" };

  for (int i = 0; i < 3; i++)
  {
    int cx = 20 + (i * 148);

    gfx->fillRoundRect(cx, 176, 136, 40, 6, C_RAISED);
    gfx->drawRoundRect(cx, 176, 136, 40, 6, C_BORDER);

    gfx->fillCircle(cx + 18, 196, 5, mixColor(C_OK, C_PANEL, 130));
    gfx->drawCircle(cx + 18, 196, 5, C_OK);

    trackedText(chipLabels[i], cx + 33, 183, 1, C_TEXT,  2);
    trackedText(chipNames[i],  cx + 33, 199, 1, C_MUTED, 1);
  }


  gfx->fillRect(16, 228, 448, 1, C_BORDER);

  drawMainStatusLine();

  drawFooterBar("ACK", C_RAISED);
}


// ============================================================
// LIVE STATUS LINE
//
// Uptime plus a heartbeat dot, so the panel
// visibly proves it is running.
// ============================================================

void drawMainStatusLine()
{
  gfx->fillRect(12, 234, 456, 20, C_PANEL);

  trackedText("STATUS:", 20, 240, 1, C_MUTED, 1);

  // ----------------------------------------------------------
  // PHASE 3: DEGRADED SENSING IS SHOWN ON THE IDLE SCREEN.
  //
  // A failed, suspect or misconfigured roadside node changes what the
  // system is capable of, and it does so silently. With one microphone
  // down an approach can no longer establish direction of travel at all
  // -- and direction is the ONLY second factor Scenario 1 has, since
  // there is no camera and no authenticated vehicle.
  //
  // Left invisible, a junction can sit for weeks in a state where it
  // will either miss approaching ambulances or, if degraded COMMIT is
  // enabled, act on evidence it was never designed to act on alone. The
  // console is the only thing anyone looks at, so it is where this has
  // to appear.
  //
  // It takes priority over the routine "MONITORING" text because a
  // maintenance condition that only shows when nothing else is happening
  // is a maintenance condition nobody sees.
  // ----------------------------------------------------------

  const char *statusText;
  uint16_t    statusColour;

  if (anyNodeDegraded)
  {
    statusText   = "DEGRADED SENSING - CHECK NODES";
    statusColour = C_AMBER;
  }
  else if (icuOnline)
  {
    statusText   = "MONITORING";
    statusColour = C_OK;
  }
  else
  {
    statusText   = wifiStatus;
    statusColour = C_ALERT;
  }

  trackedText(
    statusText,
    20 + trackedWidth("STATUS:", 1, 1) + 8,
    240,
    1,
    statusColour,
    1
  );


  unsigned long up = millis() / 1000;

  char buf[32];

  sprintf(
    buf,
    "UP %02lu:%02lu:%02lu",
    (up / 3600),
    ((up / 60) % 60),
    (up % 60)
  );

  int wv = trackedWidth(buf, 1, 1);

  trackedText(buf, 460 - wv, 240, 1, C_MUTED, 1);


  int wl = trackedWidth("ICU LINK:", 1, 1);

  trackedText(
    "ICU LINK:",
    460 - wv - 24 - wl - trackedWidth("ONLINE", 1, 1) - 8,
    240,
    1,
    C_MUTED,
    1
  );

  trackedText(
    icuLinkAlive ? "UP" : "DOWN",
    460 - wv - 24 - trackedWidth("ONLINE", 1, 1),
    240,
    1,
    icuLinkAlive
      ? (mainHeartbeatOn ? C_OK : mixColor(C_OK, C_PANEL, 90))
      : C_ALERT,
    1
  );
}


// ============================================================
// IDLE SCREEN HEARTBEAT
//
// Refreshes uptime once a second and blinks
// the link dot, so the console never looks
// like a frozen static image.
// ============================================================

void updateMainHeartbeat()
{
  if (millis() - lastHeartbeatUpdate < 1000)
  {
    return;
  }

  lastHeartbeatUpdate = millis();

  mainHeartbeatOn = !mainHeartbeatOn;

  serviceClock();

  // No UART traffic for a while means the ICU is
  // off, reset, or unplugged - not just offline.
  if (icuLinkAlive &&
      (millis() - lastIcuMessage > ICU_LINK_TIMEOUT))
  {
    icuLinkAlive = false;
    icuOnline    = false;

    strncpy(wifiStatus, "NO LINK", sizeof(wifiStatus) - 1);

    // ------------------------------------------------------------
    // PHASE 2 FAIL-SAFE  --  THE MOST IMPORTANT LINES IN THIS FILE
    //
    // If an alert is live when the ICU goes silent, END IT. Do not
    // leave it standing waiting for a CLEAR that is never coming.
    //
    // Before this, a live alert was driven entirely by the ERC's own
    // fixed sequence timer, so an ICU that crashed mid-event left the
    // console holding a red screen and a buzzer for an incident nobody
    // was tracking any more. The officer has no way to tell a live
    // emergency from a dead controller, and the natural reading of a
    // red screen is that the ambulance is still coming.
    //
    // Spec Scenario 1 v2.0 section 11.1 requires the interface to the
    // signal side to be a HEARTBEAT-GATED HOLD rather than a latching
    // command: the correct behaviour of an intersection with a dead ICU
    // is NORMAL OPERATION, not a held preemption. Neither v1.0 document
    // said what happens when the ICU dies mid-event, and it is the first
    // question a traffic-authority safety reviewer asks.
    //
    // In advisory mode the consequence of getting this wrong is milder
    // than a stuck conflicting green -- but it is the same defect, and
    // it becomes that defect verbatim the day GW_ENABLE_TCU is set to 1.
    // Building the habit now is free; retrofitting it later is not.
    //
    // resetToNormalOperation() stops the buzzer, clears the LEDs and
    // returns to the main screen. It also emits NORMAL_MODE, which is
    // harmless when nobody is listening and correct if the ICU is in
    // fact still receiving on a one-directional fault.
    // ------------------------------------------------------------

    if (priorityStage != PRIORITY_IDLE)
    {
      Serial.println(
        "ICU LINK LOST DURING LIVE ALERT - FAIL-SAFE TO NORMAL"
      );

      resetToNormalOperation();

      // resetToNormalOperation() already repaints, so fall through
      // without a second draw.
      return;
    }

    if (currentScreen == ERC_MAIN)
    {
      drawMainScreen();
    }
  }

  // The clock runs on every screen
  if (currentScreen == ERC_MAIN ||
      currentScreen == ERC_EMERGENCY)
  {
    drawStatusBar();
  }

  if (currentScreen == ERC_MAIN)
  {
    drawMainStatusLine();
  }
}


// ============================================================
// DRAW EMERGENCY SCREEN
// ============================================================

void drawEmergencyScreen()
{
  gfx->fillScreen(C_BG);

  drawStatusBar();


  // Instrument card with an alert border
  drawPanel(6, 32, 468, 226, C_PANEL, C_ALERT_DK);


  // ---------- Title ----------
  centerTracked(
    "EMERGENCY VEHICLE APPROACHING",
    240,
    44,
    2,
    C_TEXT,
    2
  );

  gfx->fillRect(16, 68, 448, 1, C_BORDER);


  // ---------- Lane block ----------
  char laneText[4];

  sprintf(laneText, "%d", activeLaneNumber());

  centerTracked("LANE", 66, 82, 2, C_TEXT, 3);
  centerTracked(laneText, 66, 104, 6, C_TEXT, 0);

  centerTracked(
    activeApproachName(),
    66,
    152,
    1,
    C_MUTED,
    2
  );

  gfx->fillRect(118, 78, 1, 84, C_BORDER);


  // ---------- Vehicle type ----------
  trackedText("VEHICLE TYPE", 136, 96, 1, C_MUTED, 2);
  trackedText("AMBULANCE",   136, 114, 2, C_TEXT,  2);


  // ---------- Vehicle illustration ----------
  drawAmbulance(310, 84);


  gfx->fillRect(16, 168, 448, 1, C_BORDER);


  // ---------- Live data row + status ----------
  lastLiveStage = -1;          // force a full paint

  updateEmergencyLive();


  // ---------- Controls ----------
  drawAckButton();
}


// ============================================================
// PHASE PROGRESS BAR
// ============================================================

void drawLiveSegments()
{
  // PHASE 3: this is now a STAGE INDICATOR, not a progress bar.
  //
  // It used to fill proportionally to a local countdown: 7 s of FAR,
  // 20 s of NEAR, 7 s of hold. That bar was a lie in the ordinary case.
  // It showed an officer how much of a fixed timer had elapsed, and they
  // would read it as how close the ambulance was -- two completely
  // different things, and the difference is largest exactly when traffic
  // is worst.
  //
  // There is no honest percentage to draw. The ICU knows the stage; when
  // it can compute an ETA it sends one, and that goes in the ETA cell
  // where a number belongs. So the bar shows WHICH stage is live and
  // fills that segment completely -- a position, not a prediction.

  int activeIndex = 0;

  if (priorityStage == PRIORITY_NEAR_BLINK)     activeIndex = 1;
  else if (priorityStage == PRIORITY_COMPLETE)  activeIndex = 2;

  drawSegments(20, 224, 440, 6, activeIndex, 100);
}


// ============================================================
// COUNTDOWN CELL ONLY
//
// The one thing that changes every second.
// Repainting just this cell removes the 1 Hz
// flicker across the whole data row.
// ============================================================

void drawLiveSegments();

void updateCountdownCell()
{
  // PHASE 3: refreshes the ETA cell, which is the only value that moves
  // between stage changes.
  //
  // The ETA comes from the ICU and is recomputed there on every accepted
  // packet. This does not decrement it locally: a locally-counted-down
  // ETA would keep falling smoothly while the vehicle sat stationary in
  // traffic, and would reach zero while it was still 200 m away. The
  // number on screen is the last thing the ICU actually measured.

  char etaText[12];

  if (alertEtaS >= 0)
  {
    sprintf(etaText, "%d s", (int)alertEtaS);
  }
  else
  {
    strcpy(etaText, "--");
  }

  gfx->fillRect(376, 176, 92, 40, C_PANEL);

  drawDataCell(416, 178, "ETA", etaText, C_AMBER, 2);

  drawLiveSegments();
}


// ============================================================
// LIVE EMERGENCY DATA
//
// Everything that changes second to second
// lives in one repaint region, so the rest
// of the panel never flickers.
// ============================================================

// PHASE 3: short evidence label for the console.
//
// The full names ("ACOUSTIC_AMBIGUOUS") do not fit the cell, and this
// field must never be the one that wraps or truncates -- it is the field
// that tells the operator how far to trust the alert.
//
// "2/2" and "1/2" read as what they are: how many of the two roadside
// microphones agree. An officer does not need the specification to
// understand that one is weaker than two.
const char *evidenceShortName(uint8_t e)
{
  switch (e)
  {
    case 1:  return "EVU";        // authenticated, received directly
    case 2:  return "EVU-RLY";    // authenticated, via a roadside relay
    case 3:  return "EVU-DEG";    // was authenticated, now in grace
    case 4:  return "SIREN 2/2";  // both nodes, valid progression
    case 5:  return "SIREN 1/2";  // one node only, partner failed
    case 6:  return "SIREN ?";    // heard at both, direction unknown
    default: return "-";
  }
}


void updateEmergencyLive()
{
  int  stageNow = (int)priorityStage;
  bool ackNow   = emergencyAcknowledged;

  bool fullRedraw =
    (stageNow != lastLiveStage) ||
    (ackNow   != lastLiveAcked);

  lastLiveStage = stageNow;
  lastLiveAcked = ackNow;

  if (!fullRedraw)
  {
    updateCountdownCell();
    return;
  }

  gfx->fillRect(10, 172, 460, 82, C_PANEL);


  // ----------------------------------------------------------
  // ETA CELL
  //
  // -1 from the ICU means NOT COMPUTABLE, and it renders as "--".
  //
  // Never as 0, and never as an elapsed timer dressed up to look like a
  // prediction. In the acoustic-only case the ICU genuinely cannot know:
  // it has no position telemetry and no sensor beyond the stop line, so
  // there is nothing to compute an arrival time from (spec S1-07).
  //
  // A fabricated countdown would be read by an officer as a measurement,
  // and they would time a manual override against it. "--" tells them
  // the truth, which is that the system knows a vehicle is coming but
  // not when it will arrive.
  // ----------------------------------------------------------

  char etaText[12];

  if (alertEtaS >= 0)
  {
    sprintf(etaText, "%d s", (int)alertEtaS);
  }
  else
  {
    strcpy(etaText, "--");
  }


  // ----------------------------------------------------------
  // PRIORITY CELL
  //
  // This is the REGISTERED class held by the ICU, not the class the
  // vehicle asked for. A unit that is stolen, decommissioned, or has had
  // its firmware read can put any number it likes in its own packet, and
  // that packet passes authentication because it genuinely is signed --
  // it is simply lying about its own importance (spec defect S2-01).
  //
  // 0 means the ICU has no registry entry, shown as "?" rather than as
  // a class. An unregistered but authenticated vehicle is admitted at
  // the lowest class and flagged, so the operator should see that this
  // one is unusual.
  // ----------------------------------------------------------

  char priText[8];

  if (alertPriority > 0)
  {
    sprintf(priText, "P%u", (unsigned)alertPriority);
  }
  else
  {
    strcpy(priText, "?");
  }


  // Colour follows evidence strength, not stage. A COMMIT on one
  // surviving microphone and a COMMIT on an authenticated ambulance
  // are the same red banner; this cell is where they differ.
  uint16_t evColour = C_OK;

  if (alertEvidence >= 5)       evColour = C_AMBER;   // degraded/ambiguous
  else if (alertEvidence == 4)  evColour = C_OK;      // both nodes agree
  else if (alertEvidence >= 1)  evColour = C_ACCENT;  // authenticated EVU


  drawDataCell(
    72,
    178,
    "DIRECTION",
    activeApproachName(),
    C_OK,
    2
  );

  drawDataCell(
    192,
    178,
    "EVIDENCE",
    evidenceShortName(alertEvidence),
    evColour,
    2
  );

  drawDataCell(
    312,
    178,
    "CLASS",
    priText,
    C_TEXT,
    2
  );

  drawDataCell(
    416,
    178,
    "ETA",
    etaText,
    C_AMBER,
    2
  );

  gfx->fillRect(132, 176, 1, 42, C_BORDER);
  gfx->fillRect(252, 176, 1, 42, C_BORDER);
  gfx->fillRect(372, 176, 1, 42, C_BORDER);


  // ---------- Phase progress ----------
  drawLiveSegments();


  // ---------- Status line ----------
  gfx->fillRect(16, 236, 448, 1, C_BORDER);

  trackedText("STATUS:", 20, 242, 1, C_MUTED, 1);

  // ----------------------------------------------------------
  // STATUS TEXT
  //
  // PREPARE says "MONITORING", not "AWAITING ACK". PREPARE does not ask
  // the officer for anything -- it informs. Only COMMIT is a request to
  // act, and only COMMIT sounds the buzzer.
  //
  // Wording the reversible stage as though it needed acknowledgement
  // would train the operator to acknowledge everything, which destroys
  // the distinction the two-stage model exists to create.
  // ----------------------------------------------------------

  const char *statusText;
  uint16_t    statusColour;

  if (priorityStage == PRIORITY_FAR_BLINK)
  {
    statusText   = "MONITORING - NO ACTION NEEDED";
    statusColour = C_AMBER;
  }
  else if (priorityStage == PRIORITY_COMPLETE)
  {
    statusText   = "CLEARING";
    statusColour = C_MUTED;
  }
  else if (emergencyAcknowledged)
  {
    statusText   = "PRIORITY ACTIVE";
    statusColour = C_OK;
  }
  else
  {
    statusText   = "AWAITING ACK";
    statusColour = C_ALERT;
  }

  trackedText(
    statusText,
    20 + trackedWidth("STATUS:", 1, 1) + 8,
    242,
    1,
    statusColour,
    1
  );

  const char *sysLabel = "LINK:";
  const char *sysValue = icuLinkAlive ? "UP" : "DOWN";

  int wv = trackedWidth(sysValue, 1, 1);
  int wl = trackedWidth(sysLabel, 1, 1);

  trackedText(sysLabel, 460 - wv - 8 - wl, 242, 1, C_MUTED, 1);
  trackedText(sysValue, 460 - wv,          242, 1,
              icuLinkAlive ? C_OK : C_ALERT, 1);
}


// ============================================================
// PHASE + COUNTDOWN AREA
//
// Occupies y = 212 .. 253 only, so the
// button strip underneath is never erased.
// ============================================================

void drawPhaseArea(
  const char *phaseText,
  int secondsRemaining
)
{
  (void)phaseText;

  lastDisplayedCountdown = secondsRemaining;

  updateEmergencyLive();
}


// ============================================================
// BUTTON STRIP - ACKNOWLEDGE
// ============================================================

void drawAckButton()
{
  drawFooterBar("ACK", C_ALERT);
}


// ============================================================
// BUTTON STRIP - ACKNOWLEDGED BANNER
// ============================================================

void drawAcknowledgedBanner()
{
  drawFooterBar("ACKED", C_OK_DK);

  updateEmergencyLive();
}


// ============================================================
// DRAW COUNTDOWN
// ============================================================

void drawCountdown(
  const char *phaseText,
  int secondsRemaining
)
{
  drawPhaseArea(
    phaseText,
    secondsRemaining
  );
}


// ============================================================
// DRAW COMPLETE SCREEN
// ============================================================

void drawCompleteScreen(
  int secondsRemaining
)
{
  lastDisplayedCountdown = secondsRemaining;

  updateEmergencyLive();
}


// ============================================================
// BUTTON STRIP - RETURN TO NORMAL
// ============================================================

void drawReturnButton()
{
  drawFooterBar("RETURN", C_OK_DK);
}


// ============================================================
// DRAW TEST SCREEN
// ============================================================

void drawTestScreen()
{
  gfx->fillScreen(C_BG);


  // ----------------------------------------------------------
  // HEADER
  // ----------------------------------------------------------

  vGradient(0, 0, 480, 42, C_PANEL, C_BG);

  gfx->fillRect(0, 42, 480, 2, C_AMBER);

  trackedText(
    "ENGINEERING TEST MODE",
    16,
    11,
    2,
    C_AMBER,
    1
  );

  trackedText(
    "HARDWARE DIAGNOSTICS",
    16,
    30,
    1,
    C_MUTED,
    1
  );

  drawPill(
    360,
    11,
    106,
    22,
    C_AMBER,
    "SERVICE",
    C_TEXT
  );


  // ----------------------------------------------------------
  // INDICATOR TEST GRID
  // ----------------------------------------------------------

  drawPanel(8, 52, 300, 200, C_PANEL, C_BORDER);

  drawTestButton( 15,  60, 135, 55, C_INFO,  "LEFT",   "FAR");
  drawTestButton(165,  60, 135, 55, C_OK,    "LEFT",   "NEAR");

  drawTestButton( 15, 125, 135, 55, C_INFO,  "RIGHT",  "FAR");
  drawTestButton(165, 125, 135, 55, C_OK,    "RIGHT",  "NEAR");

  drawTestButton( 15, 190, 135, 55, C_INFO,  "BOTTOM", "FAR");
  drawTestButton(165, 190, 135, 55, C_OK,    "BOTTOM", "NEAR");


  // ----------------------------------------------------------
  // AUDIBLE WARNING
  // ----------------------------------------------------------

  drawPanel(316, 52, 156, 200, C_PANEL, C_BORDER);

  centerTracked(
    "SIREN",
    394,
    64,
    2,
    C_TEXT,
    3
  );

  gfx->fillRect(336, 84, 116, 1, C_BORDER);

  drawSoftButton(
    320,
    90,
    145,
    60,
    C_ALERT_DK,
    C_ALERT,
    "PLAY",
    2,
    C_TEXT
  );

  drawSoftButton(
    320,
    165,
    145,
    60,
    C_RAISED,
    C_BORDER,
    "STOP",
    2,
    C_TEXT
  );


  // ----------------------------------------------------------
  // FOOTER
  // ----------------------------------------------------------

  trackedText(
    "TAP AN INDICATOR TO PULSE IT FOR 3 SECONDS",
    16,
    276,
    1,
    mixColor(C_MUTED, C_BG, 90),
    0
  );

  drawSoftButton(
    340,
    255,
    120,
    45,
    C_RAISED,
    C_ACCENT_DIM,
    "RETURN",
    2,
    C_TEXT
  );
}


// ============================================================
// NODE COMMAND
//
// NODE:<approach>:<node>:<0|1> from the
// dashboard's FAR / NEAR chips.
//
// Manual LED control is only honoured while no
// sequence is running: updatePrioritySequence()
// writes these pins every loop and would
// overwrite a manual value within milliseconds.
// NODE_BUSY makes that explicit.
// ============================================================

void handleNodeCommand(
  String command
)
{
  int p1 = command.indexOf(':');
  int p2 = command.indexOf(':', p1 + 1);
  int p3 = command.indexOf(':', p2 + 1);

  if (p1 < 0 || p2 < 0 || p3 < 0)
  {
    ICU_UART.println("NODE_BAD");
    ICU_UART.flush();
    return;
  }

  String approach = command.substring(p1 + 1, p2);
  String node     = command.substring(p2 + 1, p3);
  bool   state    = command.substring(p3 + 1).toInt() != 0;

  approach.trim();
  node.trim();

  int pin = -1;

  if (approach == "LEFT")
  {
    pin = (node == "FAR") ? LEFT_FAR_LED : LEFT_NEAR_LED;
  }

  else if (approach == "RIGHT")
  {
    pin = (node == "FAR") ? RIGHT_FAR_LED : RIGHT_NEAR_LED;
  }

  else if (approach == "BOTTOM")
  {
    pin = (node == "FAR") ? BOTTOM_FAR_LED : BOTTOM_NEAR_LED;
  }

  if (pin < 0)
  {
    ICU_UART.println("NODE_BAD");
    ICU_UART.flush();
    return;
  }

  if (priorityStage != PRIORITY_IDLE)
  {
    ICU_UART.println("NODE_BUSY");
    ICU_UART.flush();
    return;
  }

  safeWrite(pin, state);

  ICU_UART.print("NODE_OK ");
  ICU_UART.println(command);
  ICU_UART.flush();

  Serial.print("NODE_OK ");
  Serial.println(command);
}


// ============================================================
// RECEIVE EMERGENCY COMMAND
// ============================================================

// ============================================================
// PHASE 3: ICU-DRIVEN ALERT HANDLERS
// ============================================================

// Splits a colon-delimited line into up to 8 fields.
// Returns the number of fields found.
static int splitFields(const String &s, String *out, int maxOut)
{
  int n     = 0;
  int start = 0;

  while (n < maxOut)
  {
    int c = s.indexOf(':', start);

    if (c < 0)
    {
      out[n++] = s.substring(start);
      break;
    }

    out[n++] = s.substring(start, c);
    start    = c + 1;
  }

  return n;
}


static uint8_t parseEvidenceName(const String &s)
{
  if (s == "EVU_DIRECT")         return 1;
  if (s == "EVU_INDIRECT")       return 2;
  if (s == "EVU_DEGRADED")       return 3;
  if (s == "ACOUSTIC_CONFIRMED") return 4;
  if (s == "ACOUSTIC_DEGRADED")  return 5;
  if (s == "ACOUSTIC_AMBIGUOUS") return 6;
  return 0;
}


static uint8_t parseHealthName(const String &s)
{
  if (s == "HEALTHY")      return 1;
  if (s == "SUSPECT")      return 2;
  if (s == "FAILED")       return 3;
  if (s == "RECOVERING")   return 4;
  if (s == "MISCONFIG")    return 5;
  if (s == "LINK_SUSPECT") return 6;
  return 0;
}


// Maps an approach name onto the LED pins and the enum.
// Returns false for a name this console does not serve.
static bool selectApproach(const String &appr)
{
  if (appr == "LEFT")
  {
    activeApproach = APPROACH_LEFT;
    activeFarPin   = LEFT_FAR_LED;
    activeNearPin  = LEFT_NEAR_LED;
    return true;
  }

  if (appr == "RIGHT")
  {
    activeApproach = APPROACH_RIGHT;
    activeFarPin   = RIGHT_FAR_LED;
    activeNearPin  = RIGHT_NEAR_LED;
    return true;
  }

  // CENTER is the traffic controller's name for the approach this
  // console calls BOTTOM. Accepted so a TCU-shaped name does not
  // silently fail to match.
  if (appr == "BOTTOM" || appr == "CENTER")
  {
    activeApproach = APPROACH_BOTTOM;
    activeFarPin   = BOTTOM_FAR_LED;
    activeNearPin  = BOTTOM_NEAR_LED;
    return true;
  }

  return false;
}


// ------------------------------------------------------------
// ALERT:<APPROACH>:<STAGE>:<PRIORITY>:<EVIDENCE>:<ETA>
//
// Creates the alert, or updates one already running. Idempotent: the
// ICU re-sends this on every change and periodically thereafter, so
// arriving twice with the same content must be harmless.
// ------------------------------------------------------------

void handleAlertCommand(String command)
{
  String f[8];
  int n = splitFields(command, f, 8);

  if (n < 3)
  {
    Serial.println("ALERT: malformed, ignored");
    return;
  }

  String appr  = f[1];
  String stage = f[2];

  uint8_t pri = (n > 3) ? (uint8_t)f[3].toInt() : 0;
  uint8_t ev  = (n > 4) ? parseEvidenceName(f[4]) : 0;

  // -1 means the ICU cannot compute an ETA and is saying so. Preserved
  // as -1 and rendered "--". Never coerced to 0, which would display as
  // "arriving now".
  int16_t eta = (n > 5) ? (int16_t)f[5].toInt() : -1;

  if (!selectApproach(appr))
  {
    Serial.print("ALERT: unknown approach ");
    Serial.println(appr);
    return;
  }

  PriorityStage newStage = PRIORITY_IDLE;

  if      (stage == "PREPARE")  newStage = PRIORITY_FAR_BLINK;
  else if (stage == "COMMIT")   newStage = PRIORITY_NEAR_BLINK;
  else if (stage == "CLEARING") newStage = PRIORITY_COMPLETE;
  else if (stage == "MONITOR")
  {
    // MONITOR means the ICU is tracking something but has taken no
    // action. Nothing is shown. Displaying every tracked siren would
    // train the operator to ignore the screen, which is the failure
    // mode that matters most in a device whose only output is a human's
    // attention.
    if (priorityStage != PRIORITY_IDLE)
    {
      resetToNormalOperation();
    }
    return;
  }
  else
  {
    Serial.print("ALERT: unknown stage ");
    Serial.println(stage);
    return;
  }

  bool wasIdle       = (priorityStage == PRIORITY_IDLE);
  bool stageChanged  = (priorityStage != newStage);

  priorityStage    = newStage;
  alertPriority    = pri;
  alertEvidence    = ev;
  alertEtaS        = eta;
  alertFromIcu     = true;
  alertReleaseReason[0] = '\0';

  if (wasIdle)
  {
    alertStartMs          = millis();
    emergencyStartMillis  = millis();
    emergencyAcknowledged = false;

    ledTestActive = false;
    activeLEDPin  = -1;

    allLEDsOff();
  }

  // ----------------------------------------------------------
  // THE BUZZER RULE
  //
  // Sound on COMMIT only. Never on PREPARE.
  //
  // PREPARE means "probably approaching, not yet confirmed" and is
  // reversible by design -- spec Scenario 1 section 7 exists precisely
  // so that weak evidence can produce a cheap action instead of being
  // forced to choose between an expensive one and nothing at all.
  //
  // A buzzer is not cheap. It is a demand for attention, and an alarm
  // that sounds for events that then evaporate is an alarm that gets
  // ignored within a week. The screen informs on PREPARE; only COMMIT
  // asks the officer to act.
  // ----------------------------------------------------------

  if (newStage == PRIORITY_NEAR_BLINK)
  {
    if (!emergencyAcknowledged && !sirenActive)
    {
      startSiren();
    }
  }
  else
  {
    stopSiren();
  }

  if (wasIdle || stageChanged)
  {
    priorityStageStart = millis();
    lastPriorityBlink  = millis();
    priorityBlinkState = false;

    allLEDsOff();

    currentScreen = ERC_EMERGENCY;
    drawEmergencyScreen();
    drawQueueLine();

    Serial.print("ALERT ");
    Serial.print(appr);
    Serial.print(" stage=");
    Serial.print(stage);
    Serial.print(" pri=");
    Serial.print(pri);
    Serial.print(" eta=");
    Serial.println(eta);

    ICU_UART.print("RECEIVED:");
    ICU_UART.println(appr);
    ICU_UART.flush();
  }
  else
  {
    // Same stage, refreshed values. Repaint only the live row, not the
    // whole screen -- a full repaint at the ICU's refresh rate would
    // make the display visibly flicker.
    updateEmergencyLive();
  }
}


// ------------------------------------------------------------
// HOLD:<APPROACH>
//
// "Still true." Refreshes the alert and changes nothing else.
//
// This is what replaces the old fixed 20-second green: an ambulance
// held in traffic keeps the console alive for as long as the ICU keeps
// saying so, with no arbitrary ceiling.
// ------------------------------------------------------------

void handleHoldCommand(String command)
{
  if (priorityStage == PRIORITY_IDLE)
  {
    return;
  }

  // The timestamp that matters is lastIcuMessage, already stamped by the
  // receive path for any traffic. Nothing further is needed here: HOLD
  // exists to BE that traffic.
  updateEmergencyLive();
}


// ------------------------------------------------------------
// RELEASE:<APPROACH>:<REASON>
//
// Ends the alert. The reason is shown and logged.
//
// It travels because "the ambulance arrived" and "we gave up waiting"
// look identical on a screen that returns to idle either way, yet they
// say opposite things about whether the system is working. A site with
// a failing microphone that quietly times out every event would
// otherwise be indistinguishable from a site running perfectly.
// ------------------------------------------------------------

void handleReleaseCommand(String command)
{
  String f[4];
  int n = splitFields(command, f, 4);

  String reason = (n > 2) ? f[2] : String("EXPIRED");

  strncpy(alertReleaseReason, reason.c_str(),
          sizeof(alertReleaseReason) - 1);
  alertReleaseReason[sizeof(alertReleaseReason) - 1] = '\0';

  Serial.print("RELEASE reason=");
  Serial.println(reason);

  // The waiting list belonged to the alert that just ended. Whatever is
  // queued will be re-sent by the ICU against the NEXT alert; carrying
  // it across would show a stale approach behind an unrelated event.
  queuedCount = 0;

  if (priorityStage == PRIORITY_IDLE)
  {
    return;
  }

  resetToNormalOperation();
}


// ------------------------------------------------------------
// HEALTH:<LANE>:<NODE>:<ROLE>:<STATE>:<DIST_M>
// ------------------------------------------------------------

// QUEUE:<n>[:<APPROACH>:<PRIORITY>]...
// ------------------------------------------------------------
// PHASE 3.6: THE WAITING-APPROACH LINE
//
// Drawn in the 10-pixel band between the data panel (ends y=254) and
// the footer bar (starts y=264). Deliberately small and low-contrast:
// this is context, not a demand. The live alert must stay the loudest
// thing on the screen.
//
// Priority is shown because it is what decides who takes the console
// next. "BOTTOM P1" behind a P2 alert tells the officer that the queued
// vehicle outranks the current one and will be served the moment this
// clears -- which changes whether they stand down or stay ready.
// ------------------------------------------------------------

void drawQueueLine()
{
  gfx->fillRect(0, 254, 480, 10, C_PANEL);

  if (queuedCount == 0)
  {
    return;
  }

  char buf[64];
  int  pos = 0;

  pos += snprintf(buf + pos, sizeof(buf) - pos, "ALSO WAITING: ");

  for (uint8_t i = 0; i < queuedCount; i++)
  {
    if (pos >= (int)sizeof(buf) - 14) break;

    if (i > 0)
    {
      pos += snprintf(buf + pos, sizeof(buf) - pos, "  ");
    }

    if (queuedPriority[i] > 0)
    {
      pos += snprintf(buf + pos, sizeof(buf) - pos, "%s P%u",
                      queuedName[i], (unsigned)queuedPriority[i]);
    }
    else
    {
      // 0 means unregistered or acoustic-only. Shown as "?" for the
      // same reason the CLASS cell does: an unclassified demand must
      // not be made to look like a registered one.
      pos += snprintf(buf + pos, sizeof(buf) - pos, "%s ?", queuedName[i]);
    }
  }

  trackedText(buf, 20, 255, 1, C_AMBER, 1);
}


void handleQueueCommand(String command)
{
  String f[8];
  int n = splitFields(command, f, 8);

  if (n < 2)
  {
    queuedCount = 0;
    return;
  }

  int count = f[1].toInt();

  if (count < 0) count = 0;
  if (count > MAX_QUEUED_APPROACHES) count = MAX_QUEUED_APPROACHES;

  queuedCount = 0;

  for (int i = 0; i < count; i++)
  {
    int nameIdx = 2 + i * 2;
    int priIdx  = nameIdx + 1;

    // Stop at the first incomplete pair rather than reading past the
    // fields that actually arrived. A truncated line must show fewer
    // entries, never a garbage one.
    if (priIdx >= n) break;

    strncpy(queuedName[queuedCount], f[nameIdx].c_str(),
            sizeof(queuedName[0]) - 1);
    queuedName[queuedCount][sizeof(queuedName[0]) - 1] = '\0';

    queuedPriority[queuedCount] = (uint8_t)f[priIdx].toInt();
    queuedCount++;
  }

  if (currentScreen == ERC_EMERGENCY)
  {
    drawQueueLine();
  }
}


void handleHealthCommand(String command)
{
  String f[8];
  int n = splitFields(command, f, 8);

  if (n < 6)
  {
    return;
  }

  int lane = f[1].toInt();
  int node = f[2].toInt();

  if (lane < 1 || lane > 3 || node < 1 || node > 2)
  {
    return;
  }

  uint8_t role = 0;
  if (f[3] == "FAR")  role = 1;
  if (f[3] == "NEAR") role = 2;

  nodeHealth[lane][node].role      = role;
  nodeHealth[lane][node].state     = parseHealthName(f[4]);
  nodeHealth[lane][node].distanceM = (uint16_t)f[5].toInt();
  nodeHealth[lane][node].reported  = true;

  // Recompute the banner condition across every node.
  //
  // HEALTHY and RECOVERING do not raise it; everything else does.
  // SUSPECT is included deliberately even though it is not yet a fault:
  // it is the state that precedes one, and the entire reason SUSPECT
  // exists as a separate state is that treating a briefly-silent node as
  // failed would quietly lower the system's own evidence threshold.
  // Showing it means a link degrading toward that point is visible
  // before it gets there.
  anyNodeDegraded = false;

  for (int l = 1; l <= 3; l++)
  {
    for (int nd = 1; nd <= 2; nd++)
    {
      if (!nodeHealth[l][nd].reported) continue;

      uint8_t s = nodeHealth[l][nd].state;

      if (s != 1 && s != 4 && s != 0)
      {
        anyNodeDegraded = true;
      }
    }
  }

  if (currentScreen == ERC_MAIN)
  {
    drawMainStatusLine();
  }
}


void receiveEmergencyCommand(
  String command
)
{
  command.trim();
  command.toUpperCase();


  // ----------------------------------------------------------
  // LEFT
  // ----------------------------------------------------------

  if (
    command == "LEFT"
  )
  {
    activeApproach =
      APPROACH_LEFT;

    activeFarPin =
      LEFT_FAR_LED;

    activeNearPin =
      LEFT_NEAR_LED;
  }


  // ----------------------------------------------------------
  // RIGHT
  // ----------------------------------------------------------

  else if (
    command == "RIGHT"
  )
  {
    activeApproach =
      APPROACH_RIGHT;

    activeFarPin =
      RIGHT_FAR_LED;

    activeNearPin =
      RIGHT_NEAR_LED;
  }


  // ----------------------------------------------------------
  // BOTTOM
  // ----------------------------------------------------------

  else if (
    command == "BOTTOM"
  )
  {
    activeApproach =
      APPROACH_BOTTOM;

    activeFarPin =
      BOTTOM_FAR_LED;

    activeNearPin =
      BOTTOM_NEAR_LED;
  }


  else
  {
    return;
  }


  // Stop any previous outputs
  stopSiren();
  allLEDsOff();


  // Reset engineering LED test
  ledTestActive =
    false;

  activeLEDPin =
    -1;


  // ----------------------------------------------------------
  // New priority event
  //
  // The indicator sequence starts NOW so it
  // stays locked to the traffic controller.
  // Operator acknowledgement only silences
  // the buzzer - it does not gate the LEDs.
  // ----------------------------------------------------------

  priorityStage =
    PRIORITY_FAR_BLINK;

  emergencyAcknowledged =
    false;

  emergencyStartMillis =
    millis();

  priorityStageStart =
    millis();

  lastPriorityBlink =
    millis();

  priorityBlinkState =
    true;

  lastDisplayedCountdown =
    -1;

  // PHASE 3: a bare legacy "LEFT" command carries no stage, priority,
  // evidence or ETA. Mark the alert as not ICU-driven and show the
  // fields as unknown rather than inventing plausible values.
  alertFromIcu  = false;
  alertPriority = 0;
  alertEvidence = 0;
  alertEtaS     = -1;
  alertStartMs  = millis();

  currentScreen =
    ERC_EMERGENCY;


  // Ignore a finger already on the glass
  previousTouchState =
    false;


  // Start warning
  startSiren();


  // FAR starts ON
  safeWrite(
    activeFarPin,
    true
  );


  // Show warning screen
  drawEmergencyScreen();


  Serial.println();

  Serial.println(
    "================================"
  );

  Serial.print(
    "EMERGENCY COMMAND RECEIVED: "
  );

  Serial.println(
    command
  );

  Serial.println(
    "WAITING FOR OPERATOR ACK"
  );

  Serial.println(
    "================================"
  );


  // Tell ICU
  ICU_UART.print(
    "RECEIVED:"
  );

  ICU_UART.println(
    command
  );

  ICU_UART.flush();
}


// ============================================================
// ACKNOWLEDGE EMERGENCY
// ============================================================

void acknowledgeEmergency()
{
  // Only meaningful during a live event
  if (
    priorityStage ==
    PRIORITY_IDLE ||
    emergencyAcknowledged
  )
  {
    return;
  }


  emergencyAcknowledged =
    true;


  Serial.println();

  Serial.println(
    "EMERGENCY ACKNOWLEDGED - BUZZER SILENCED"
  );


  // Stop buzzer immediately
  stopSiren();


  // PHASE 3 FIX: report ACKNOWLEDGED, not a bare "ACK".
  //
  // "ACK" is a command the ICU SENDS to this console to acknowledge
  // remotely. Echoing the same word back up the wire meant the ICU was
  // receiving its own vocabulary as a reply and had no handler for it --
  // it logged "(unhandled) ACK" and threw away the single most important
  // fact the console produces.
  //
  // ACKNOWLEDGED is unambiguous and directional: it only ever travels
  // ERC -> ICU, and it means a human pressed the button. That matters
  // because AUTO_ACK -- the console silencing its own buzzer after ten
  // seconds in an empty cabin -- must never be recorded as the same
  // event. The difference is whether anyone was present, which is the
  // first question anyone would ask after a missed emergency.
  ICU_UART.println(
    "ACKNOWLEDGED"
  );

  ICU_UART.flush();


  // Swap the button strip for a banner.
  // LED phases are untouched.
  if (
    priorityStage !=
    PRIORITY_COMPLETE
  )
  {
    drawAcknowledgedBanner();
  }
}


// ============================================================
// AUTO ACKNOWLEDGE
//
// If nobody is at the console the buzzer
// silences itself and the ICU is told.
// ============================================================

void checkAutoAcknowledge()
{
  if (
    priorityStage ==
    PRIORITY_IDLE
  )
  {
    return;
  }

  // ----------------------------------------------------------
  // PHASE 3: AUTO-ACK APPLIES TO COMMIT ONLY.
  //
  // Its whole purpose is to stop the buzzer sounding indefinitely in an
  // empty cabin, and PREPARE has no buzzer to stop.
  //
  // Letting it fire during PREPARE would be actively harmful. The alert
  // would be marked acknowledged BEFORE the COMMIT that actually asks
  // the officer to do something -- so when COMMIT arrived it would find
  // emergencyAcknowledged already true, skip the buzzer, and the one
  // stage that genuinely needs attention would arrive in silence.
  //
  // A slow approach makes this the normal case, not an edge case:
  // PREPARE can easily run longer than AUTO_ACK_DELAY while the vehicle
  // covers the outer 250 m.
  // ----------------------------------------------------------

  if (
    priorityStage !=
    PRIORITY_NEAR_BLINK
  )
  {
    return;
  }

  if (emergencyAcknowledged)
  {
    return;
  }

  // Measured from entry to COMMIT (priorityStageStart), not from the
  // start of the whole alert. An alert that spent two minutes in
  // PREPARE would otherwise satisfy this the instant COMMIT began and
  // silence the buzzer immediately.
  if (
    millis() -
    priorityStageStart <
    AUTO_ACK_DELAY
  )
  {
    return;
  }

  Serial.println(
    "NO OPERATOR - AUTO ACKNOWLEDGE"
  );

  acknowledgeEmergency();

  ICU_UART.println(
    "AUTO_ACK"
  );

  ICU_UART.flush();
}


// ============================================================
// RESET TO NORMAL OPERATION
// ============================================================

void resetToNormalOperation()
{
  Serial.println();

  Serial.println(
    "================================"
  );

  Serial.println(
    " RETURNING TO NORMAL MODE"
  );

  Serial.println(
    "================================"
  );


  // Stop warning
  stopSiren();


  // Turn indicators OFF
  allLEDsOff();


  // Reset emergency state
  activeApproach =
    APPROACH_NONE;

  activeFarPin =
    -1;

  activeNearPin =
    -1;

  priorityStage =
    PRIORITY_IDLE;

  emergencyAcknowledged =
    false;

  lastDisplayedCountdown =
    -1;


  // Reset engineering test state
  ledTestActive =
    false;

  activeLEDPin =
    -1;


  // Return screen
  currentScreen =
    ERC_MAIN;


  // Reset touch
  previousTouchState =
    false;

  testButtonHolding =
    false;

  testModeTriggered =
    false;


  // Main display
  drawMainScreen();


  // Inform ICU
  ICU_UART.println(
    "NORMAL_MODE"
  );

  ICU_UART.flush();


  Serial.println(
    "ERC NORMAL OPERATION RESTORED"
  );

  Serial.println(
    "SEARCHING FOR NEXT EMERGENCY VEHICLE"
  );
}


// ============================================================
// UPDATE PRIORITY SEQUENCE
// ============================================================

void updatePrioritySequence()
{
  // ============================================================
  // PHASE 3: REWRITTEN. THE ICU OWNS THE STAGE; THIS OWNS THE PIXELS.
  //
  // This function used to be a 388-line state machine that advanced
  // itself: FAR for exactly 7 s, then NEAR for exactly 20 s, then a 7 s
  // hold, then back to idle -- regardless of anything the ICU knew.
  //
  // All of that is gone. Stage changes now arrive as ALERT commands and
  // the alert ends on RELEASE. What remains here is presentation:
  // blinking the right LED, refreshing the live row, and running the two
  // local safety timers that must not depend on the ICU being alive.
  //
  // WHY THE TIMERS HAD TO GO
  //
  // A fixed sequence cannot express any of the five release conditions
  // in spec Scenario 2 section 10.1. Worse, its central assumption is
  // wrong in the case the product exists to serve: an ambulance crawling
  // through gridlock takes far longer than 34 seconds to cover the last
  // 250 m, so the console stood down while the vehicle was still
  // approaching -- exactly when it was needed. The mirror-image error
  // was just as available: a vehicle that turned away after 5 seconds
  // still held the officer's override for the full 34.
  //
  // WHAT STAYS LOCAL, AND WHY
  //
  // Two timers below are deliberately NOT delegated to the ICU:
  // the blink animation, and the maximum alert duration. Both must keep
  // working when the ICU is the thing that has failed. Handing the
  // maximum duration to the ICU would mean an ICU that hangs mid-alert
  // leaves the console lit forever -- which is the precise failure that
  // spec section 11.1 requires a heartbeat-gated design to prevent.
  // ============================================================

  if (priorityStage == PRIORITY_IDLE)
  {
    return;
  }

  if (priorityStage == PRIORITY_WAIT_ACK)
  {
    return;
  }

  unsigned long now = millis();


  // ----------------------------------------------------------
  // LED PATTERN
  //
  //   PREPARE   FAR blinks.   Something is probably coming.
  //   COMMIT    FAR solid, NEAR blinks. Confirmed; act now.
  //   CLEARING  both solid.   Winding down.
  //
  // The pattern escalates rather than switching, so the transition from
  // PREPARE to COMMIT reads as "more", not "different". An officer
  // glancing across a cabin should not have to decode which of two
  // similar-looking states is the urgent one.
  // ----------------------------------------------------------

  if (now - lastPriorityBlink >= PRIORITY_BLINK_INTERVAL)
  {
    lastPriorityBlink  = now;
    priorityBlinkState = !priorityBlinkState;

    if (priorityStage == PRIORITY_FAR_BLINK)
    {
      safeWrite(activeFarPin,  priorityBlinkState);
      safeWrite(activeNearPin, false);
    }
    else if (priorityStage == PRIORITY_NEAR_BLINK)
    {
      safeWrite(activeFarPin,  true);
      safeWrite(activeNearPin, priorityBlinkState);
    }
    else
    {
      safeWrite(activeFarPin,  true);
      safeWrite(activeNearPin, true);
    }
  }


  // ----------------------------------------------------------
  // LIVE ROW, ONCE PER SECOND
  //
  // Repainting only the changing cell rather than the screen. A full
  // repaint at this rate visibly flickers on this display.
  // ----------------------------------------------------------

  static unsigned long lastLiveTick = 0;

  if (now - lastLiveTick >= 1000)
  {
    lastLiveTick = now;

    if (currentScreen == ERC_EMERGENCY)
    {
      updateEmergencyLive();
    }
  }


  // ----------------------------------------------------------
  // LOCAL MAXIMUM ALERT DURATION  --  SAFETY BACKSTOP
  //
  // Independent of the ICU, and that independence is the entire point.
  //
  // The link timeout already covers an ICU that stops transmitting. This
  // covers the nastier case: an ICU that is alive, still PINGing, and
  // stuck -- a decision loop wedged in a state it cannot leave. From the
  // console's side that is indistinguishable from a genuine emergency
  // that has lasted an hour, and without a local bound the alert would
  // simply never end.
  //
  // Spec section 11.1 requires exactly this shape: a hard maximum
  // enforced independently of the decision logic, and where the
  // controller supports its own limit, that limit configured as a second
  // backstop outside our software. Two independent limits, one of them
  // not ours.
  //
  // MAX_ALERT_DURATION is generous on purpose. It is a backstop against
  // a fault, not a policy about how long an ambulance may take, and
  // setting it near a plausible journey time would make it fire during
  // normal slow approaches -- converting a safety net into a bug.
  // ----------------------------------------------------------

  if (now - alertStartMs > MAX_ALERT_DURATION)
  {
    Serial.println(
      "MAX ALERT DURATION EXCEEDED - LOCAL BACKSTOP FIRED"
    );

    // Named distinctly in the reply. If this ever appears in a log it
    // means the ICU did not release when it should have, and that is a
    // fault to investigate rather than a normal ending.
    ICU_UART.println("ALERT_TIMEOUT");
    ICU_UART.flush();

    resetToNormalOperation();
    return;
  }
}


// ============================================================
// PROCESS ICU UART
// ============================================================

// ============================================================
// PHASE 3 FIX: NON-BLOCKING LINE ASSEMBLY
//
// This used to be readStringUntil('\n') with a 50 ms timeout, taking
// ONE line per loop() pass.
//
// Two failures came out of that, and a bench run showed both:
//
//   1. PONG return rate fell from 99% to 57%. While the ERC is
//      repainting the screen or servicing the LED blink, bytes arrive
//      and queue. When the read finally happens it can catch a line
//      half-arrived, block for the full 50 ms, and return a TRUNCATED
//      string. A truncated line matches no command, so no PONG is sent,
//      and the ICU scores it as a lost packet on a wire that never
//      dropped a bit.
//
//   2. Mangled lines in the log -- "(unhandled) ?NORMAL_MODE". Same
//      cause: a fragment of one line concatenated with the next.
//
//      That is the worse of the two. A silently truncated command is
//      not a missing command; it is a DIFFERENT command. "RELEASE:LEFT"
//      arriving as "RELEASE:LEF" fails harmlessly, but the general case
//      is a line that parses into something nobody sent.
//
// Now: characters are drained into a buffer whenever they are
// available, and a line is dispatched only when its terminating newline
// actually arrives. Nothing blocks, nothing truncates, and a slow
// screen repaint costs latency instead of data.
//
// The whole buffer is drained each pass rather than one line, so a
// burst that lands during a repaint is caught up in a single visit
// instead of one line per loop iteration.
// ============================================================

void processICUCommunication()
{
  static char lineBuf[GW_LINK_MAX_LINE];
  static uint8_t lineLen = 0;

  while (ICU_UART.available())
  {
    char c = (char)ICU_UART.read();

    if (c == '\r')
    {
      continue;
    }

    if (c != '\n')
    {
      if (lineLen < sizeof(lineBuf) - 1)
      {
        lineBuf[lineLen++] = c;
      }
      else
      {
        // Overlong line: discard the whole thing rather than act on a
        // fragment. A half-parsed command is worse than a missing one.
        lineLen = 0;
      }

      continue;
    }

    // Newline reached: we have a complete line.
    lineBuf[lineLen] = '\0';

    String command = String(lineBuf);

    lineLen = 0;

    command.trim();

    if (command.length() == 0)
    {
      continue;
    }

    handleIcuLine(command);
  }
}


void handleIcuLine(String command)
{
  // ----------------------------------------------------------
  // REJECT CORRUPTED LINES BEFORE ANYTHING ELSE.
  //
  // Dispatch below is prefix-based (startsWith), so a line carrying
  // noise can still match a command prefix and then be acted upon with
  // a corrupted payload. On this end that could mean an ALERT for the
  // wrong approach, or a RELEASE that was never sent -- pointing an
  // officer at the wrong road, or standing down while a vehicle is
  // still coming.
  //
  // Note this runs BEFORE the liveness stamp. Noise on an otherwise
  // dead wire must not be able to hold the link timeout open and
  // suppress the fail-safe; only a well-formed line counts as the ICU
  // being alive.
  // ----------------------------------------------------------

  for (unsigned int i = 0; i < command.length(); i++)
  {
    char c = command.charAt(i);

    if (c < 0x20 || c > 0x7E)
    {
      Serial.println("ICU line rejected: corrupted bytes");
      return;
    }
  }


  // ----------------------------------------------------------
  // REJECT OUR OWN REPLIES ECHOED BACK.
  //
  // These words only ever travel ERC -> ICU. Seeing one arriving means
  // it is this console's own transmission returning on its own receive
  // pin -- crosstalk between two wires in the same bundle, which was
  // observed at the ICU end during the ERC's twelve-second boot.
  //
  // ALSO BEFORE THE LIVENESS STAMP, for the same reason as the ICU side:
  // if the ICU dies while this console keeps replying, an echo of our
  // own traffic must not be able to hold icuLinkAlive true. That would
  // suppress the fail-safe and leave a stale alert on screen with
  // nothing coming to clear it.
  //
  // Liveness may only be established by something ONLY the ICU could
  // have sent.
  // ----------------------------------------------------------

  if (command.startsWith("PONG")        ||
      command.startsWith("ACKNOWLEDGED")||
      command.startsWith("AUTO_ACK")    ||
      command.startsWith("RECEIVED:")   ||
      command.startsWith("NORMAL_MODE") ||
      command.startsWith("CLEARED")     ||
      command.startsWith("ALERT_TIMEOUT"))
  {
    return;
  }


  // Any traffic proves the UART link is alive.
  lastIcuMessage = millis();
  icuLinkAlive   = true;


  // SSIDs and passwords are CASE SENSITIVE. Folding
  // "Saiyuuuu" to "SAIYUUUU" makes the ICU try to
  // join a network that does not exist, which then
  // reports as a wrong password. Leave these lines
  // exactly as received; every other command is
  // a fixed keyword and folds safely.
  if (!command.startsWith("NET:") &&
      !command.startsWith("WIFI:") &&
      !command.startsWith("DATE:"))
  {
    command.toUpperCase();
  }


  if (
    command.length() == 0
  )
  {
    return;
  }


  // PHASE 3 FIX: PING is not logged.
  //
  // At 1 Hz it is 86,400 lines a day of "nothing happened", which buries
  // the handful of lines an engineer is actually reading the capture
  // for. The ICU already suppresses it on its own side; this is the
  // matching half.
  //
  // Liveness is still tracked -- lastIcuMessage was stamped above, for
  // ANY traffic, before this point. Only the printing is skipped.
  if (!command.startsWith("PING"))
  {
    Serial.print(
      "ICU -> ERC: "
    );

    Serial.println(
      command
    );
  }


  // ----------------------------------------------------------
  // EMERGENCY COMMANDS
  // ----------------------------------------------------------

  // ----------------------------------------------------------
  // PING:<seq>   ->   PONG:<seq>            (PHASE 2)
  //
  // Handled FIRST and returned from immediately. At 1 Hz this is by far
  // the most frequent line on the link, and every check placed above it
  // would run 86,400 times a day to fail.
  //
  // The reply echoes the sequence number rather than being a bare
  // "PONG", which lets the ICU measure the RETURN path specifically.
  // That path is the one carrying the operator's ACK -- the only
  // evidence anywhere in this system that a human actually saw an alert
  // -- so a link that transmits perfectly and receives nothing is a
  // failure worth distinguishing from a link that is simply idle.
  //
  // Note lastIcuMessage was already stamped above, before this point,
  // for ANY traffic. PING therefore feeds the liveness timer even when
  // nothing else is being said, which is the whole reason it exists.
  // ----------------------------------------------------------

  if (command.startsWith("PING"))
  {
    int c = command.indexOf(':');

    if (c >= 0)
    {
      ICU_UART.print("PONG:");
      ICU_UART.println(command.substring(c + 1));
    }
    else
    {
      ICU_UART.println("PONG:0");
    }

    ICU_UART.flush();

    return;
  }


  // ----------------------------------------------------------
  // WIFI STATUS + SCAN RESULTS FROM THE ICU
  //   WIFI:ONLINE:<ssid>  WIFI:OFFLINE
  //   WIFI:CONNECTING     WIFI:FAILED
  //   NET:BEGIN / NET:<i>:<rssi>:<lock>:<ssid> / NET:END
  // ----------------------------------------------------------

  if (command.startsWith("WIFI:"))
  {
    String body = command.substring(5);

    // Remember what we were showing. The ICU repeats
    // this line on a keepalive, so redrawing blindly
    // makes the whole screen flash every few seconds.
    bool wasOnline = icuOnline;

    char wasStatus[24];
    strncpy(wasStatus, wifiStatus, sizeof(wasStatus));

    if (body.startsWith("ONLINE"))
    {
      icuOnline = true;

      strncpy(wifiStatus, "ONLINE", sizeof(wifiStatus) - 1);

      int c = body.indexOf(':');

      if (c >= 0)
      {
        body.substring(c + 1).toCharArray(icuSsid, sizeof(icuSsid));
      }
    }

    else if (body.startsWith("CONNECTING"))
    {
      icuOnline = false;
      strncpy(wifiStatus, "CONNECTING", sizeof(wifiStatus) - 1);
    }

    else if (body.startsWith("FAILED"))
    {
      icuOnline = false;
      strncpy(wifiStatus, "FAILED", sizeof(wifiStatus) - 1);
    }

    else if (body.startsWith("OFFLINE"))
    {
      icuOnline = false;
      strncpy(wifiStatus, "OFFLINE", sizeof(wifiStatus) - 1);
    }

    else
    {
      // Garbled or truncated line. Keep showing what we
      // have rather than announcing a false outage.
      Serial.print("IGNORED WIFI LINE: ");
      Serial.println(command);

      return;
    }

    bool onlineChanged = (icuOnline != wasOnline);
    bool statusChanged = (strcmp(wasStatus, wifiStatus) != 0);

    if (currentScreen == ERC_MAIN)
    {
      if (onlineChanged)
      {
        // Banner and approach chips swap places,
        // so this one genuinely needs a full paint.
        drawMainScreen();
      }

      else if (statusChanged)
      {
        drawMainStatusLine();
      }
    }

    else if (currentScreen == ERC_WIFI_LIST &&
             (onlineChanged || statusChanged))
    {
      drawWifiListScreen();
    }

    return;
  }

  if (command == "NET:BEGIN")
  {
    netCount    = 0;
    scanRunning = true;
    return;
  }

  if (command == "NET:END")
  {
    scanRunning = false;

    if (currentScreen == ERC_WIFI_LIST)
    {
      drawWifiListScreen();
    }

    return;
  }

  if (command.startsWith("NET:") && netCount < MAX_NETS)
  {
    // NET:<index>:<rssi>:<lock>:<ssid>
    int p1 = command.indexOf(':');
    int p2 = command.indexOf(':', p1 + 1);
    int p3 = command.indexOf(':', p2 + 1);
    int p4 = command.indexOf(':', p3 + 1);

    if (p2 > 0 && p3 > 0 && p4 > 0)
    {
      netRssi[netCount] = command.substring(p2 + 1, p3).toInt();
      netLock[netCount] = command.substring(p3 + 1, p4).toInt() != 0;

      command.substring(p4 + 1).toCharArray(
        netSsid[netCount], sizeof(netSsid[netCount]));

      netCount++;
    }

    return;
  }


  // ----------------------------------------------------------
  // Clock sync from the ICU (which has NTP).
  //   TIME:HH:MM:SS      DATE:14 MAY 2026
  // ----------------------------------------------------------

  if (command.startsWith("TIME:"))
  {
    int h = command.substring(5, 7).toInt();
    int m = command.substring(8, 10).toInt();
    int sec = command.substring(11, 13).toInt();

    if (h >= 0 && h < 24 && m >= 0 && m < 60 && sec >= 0 && sec < 60)
    {
      clockBase   = ((long)h * 3600L) + ((long)m * 60L) + sec;
      clockAnchor = millis();
      clockValid  = true;

      // Only repaint if a screen that owns the
      // status bar is showing. The test screen
      // has its own header in the same band.
      if (currentScreen == ERC_MAIN ||
          currentScreen == ERC_EMERGENCY)
      {
        drawStatusBar();
      }

      Serial.println("CLOCK SYNCED");
    }

    return;
  }

  if (command.startsWith("DATE:"))
  {
    String d = command.substring(5);

    d.trim();

    d.toCharArray(dateText, sizeof(dateText));

    if (currentScreen == ERC_MAIN ||
        currentScreen == ERC_EMERGENCY)
    {
      drawStatusBar();
    }

    return;
  }


  // ----------------------------------------------------------
  // REMOTE ACKNOWLEDGE  (ERC_SIMULATION.md 5a)
  //
  // The documented version gates on
  // PRIORITY_WAIT_ACK. This firmware no longer
  // has that stage - the sequence starts on
  // command arrival and ACK only silences the
  // alarm - so the gate is "is there an
  // unacknowledged alert", same intent and the
  // same replies back to the ICU.
  // ----------------------------------------------------------

  // ----------------------------------------------------------
  // PHASE 3 COMMANDS FROM THE ICU
  // ----------------------------------------------------------

  // ALERT:<APPROACH>:<STAGE>:<PRIORITY>:<EVIDENCE>:<ETA>
  if (command.startsWith("ALERT:"))
  {
    handleAlertCommand(command);
    return;
  }

  // HOLD:<APPROACH>
  if (command.startsWith("HOLD:"))
  {
    handleHoldCommand(command);
    return;
  }

  // RELEASE:<APPROACH>:<REASON>
  if (command.startsWith("RELEASE:"))
  {
    handleReleaseCommand(command);
    return;
  }

  // QUEUE:<n>[:<APPROACH>:<PRIORITY>]...
  if (command.startsWith("QUEUE:"))
  {
    handleQueueCommand(command);
    return;
  }

  // HEALTH:<LANE>:<NODE>:<ROLE>:<STATE>:<DIST_M>
  if (command.startsWith("HEALTH:"))
  {
    handleHealthCommand(command);
    return;
  }


  if (command == "ACK")
  {
    if (priorityStage != PRIORITY_IDLE &&
        !emergencyAcknowledged)
    {
      // acknowledgeEmergency() now emits ACKNOWLEDGED itself, so this
      // path no longer sends it a second time. Two identical replies to
      // one command would make the ICU's link statistics disagree with
      // the number of events that actually occurred.
      acknowledgeEmergency();
    }

    else
    {
      ICU_UART.println("ACK_IGNORED");
    }

    ICU_UART.flush();

    return;
  }


  // ----------------------------------------------------------
  // DIRECT INDICATOR CONTROL  (ERC_SIMULATION.md 5b)
  //   NODE:LEFT:FAR:1
  // ----------------------------------------------------------

  if (command.startsWith("NODE:"))
  {
    handleNodeCommand(command);

    return;
  }


  // CENTER is the traffic controller's name
  // for the approach the ERC calls BOTTOM
  if (
    command == "CENTER"
  )
  {
    command = "BOTTOM";
  }


  if (
    command == "LEFT" ||
    command == "RIGHT" ||
    command == "BOTTOM"
  )
  {
    receiveEmergencyCommand(
      command
    );
  }


  // ----------------------------------------------------------
  // CLEAR / RESET
  // ----------------------------------------------------------

  else if (
    command == "CLEAR" ||
    command == "NORMAL"
  )
  {
    Serial.println(
      "CLEAR COMMAND RECEIVED"
    );

    resetToNormalOperation();

    ICU_UART.println(
      "CLEARED"
    );

    ICU_UART.flush();
  }
}


// ============================================================
// MAIN TOUCH
// ============================================================

void processMainTouch(
  int x,
  int y
)
{
  // Offline banner occupies the chip row
  if (!icuOnline && inside(x, y, 20, 172, 440, 44))
  {
    currentScreen = ERC_WIFI_LIST;

    requestWifiScan();
    drawWifiListScreen();

    return;
  }

  if (
    inside(
      x,
      y,
      16,
      272,
      136,
      38
    )
  )
  {
    Serial.println(
      "ACK PRESSED - NO ACTIVE EMERGENCY"
    );
  }

  else if (
    inside(
      x,
      y,
      328,
      272,
      136,
      38
    )
  )
  {
    Serial.println(
      "LOGS BUTTON PRESSED"
    );
  }
}


// ============================================================
// EMERGENCY TOUCH
// ============================================================

void processEmergencyTouch(
  int x,
  int y
)
{
  // ----------------------------------------------------------
  // MANUAL RETURN BUTTON
  //
  // Checked first: in the COMPLETE stage the
  // strip holds RETURN, not ACKNOWLEDGE.
  // ----------------------------------------------------------

  if (
    priorityStage ==
    PRIORITY_COMPLETE
  )
  {
    if (
      inside(
        x,
        y,
        16,
        272,
        136,
        38
      )
    )
    {
      Serial.println(
        "RETURN TO NORMAL BUTTON PRESSED"
      );

      resetToNormalOperation();
    }

    return;
  }


  // ----------------------------------------------------------
  // ACK BUTTON
  //
  // Silences the buzzer. The LED sequence is
  // already running and is not affected.
  // ----------------------------------------------------------

  if (
    !emergencyAcknowledged
  )
  {
    if (
      inside(
        x,
        y,
        16,
        272,
        136,
        38
      )
    )
    {
      Serial.println(
        "ACKNOWLEDGE BUTTON PRESSED"
      );

      acknowledgeEmergency();
    }
  }
}


// ============================================================
// TEST TOUCH
// ============================================================

void processTestTouch(
  int x,
  int y
)
{
  // LEFT FAR
  if (
    inside(
      x,
      y,
      15,
      60,
      135,
      55
    )
  )
  {
    Serial.println(
      "LEFT FAR -> GPIO17"
    );

    startLEDTest(
      LEFT_FAR_LED
    );
  }


  // LEFT NEAR
  else if (
    inside(
      x,
      y,
      165,
      60,
      135,
      55
    )
  )
  {
    Serial.println(
      "LEFT NEAR -> GPIO16"
    );

    startLEDTest(
      LEFT_NEAR_LED
    );
  }


  // RIGHT FAR
  else if (
    inside(
      x,
      y,
      15,
      125,
      135,
      55
    )
  )
  {
    Serial.println(
      "RIGHT FAR -> GPIO13"
    );

    startLEDTest(
      RIGHT_FAR_LED
    );
  }


  // RIGHT NEAR
  else if (
    inside(
      x,
      y,
      165,
      125,
      135,
      55
    )
  )
  {
    Serial.println(
      "RIGHT NEAR -> GPIO14"
    );

    startLEDTest(
      RIGHT_NEAR_LED
    );
  }


  // BOTTOM FAR
  else if (
    inside(
      x,
      y,
      15,
      190,
      135,
      55
    )
  )
  {
    Serial.println(
      "BOTTOM FAR -> GPIO25"
    );

    startLEDTest(
      BOTTOM_FAR_LED
    );
  }


  // BOTTOM NEAR
  else if (
    inside(
      x,
      y,
      165,
      190,
      135,
      55
    )
  )
  {
    Serial.println(
      "BOTTOM NEAR -> GPIO19"
    );

    startLEDTest(
      BOTTOM_NEAR_LED
    );
  }


  // PLAY SIREN
  else if (
    inside(
      x,
      y,
      320,
      90,
      145,
      60
    )
  )
  {
    Serial.println(
      "PLAY SIREN PRESSED"
    );

    startSiren();
  }


  // STOP SIREN
  else if (
    inside(
      x,
      y,
      320,
      165,
      145,
      60
    )
  )
  {
    Serial.println(
      "STOP SIREN PRESSED"
    );

    stopSiren();
  }


  // RETURN
  else if (
    inside(
      x,
      y,
      340,
      255,
      120,
      45
    )
  )
  {
    returnToMainScreen();
  }
}


// ============================================================
// ENTER TEST MODE
// ============================================================

void enterTestMode()
{
  allLEDsOff();

  stopSiren();


  ledTestActive =
    false;

  activeLEDPin =
    -1;


  activeApproach =
    APPROACH_NONE;

  activeFarPin =
    -1;

  activeNearPin =
    -1;


  priorityStage =
    PRIORITY_IDLE;

  emergencyAcknowledged =
    false;


  currentScreen =
    ERC_TEST;


  testButtonHolding =
    false;

  testModeTriggered =
    true;


  drawTestScreen();


  Serial.println();

  Serial.println(
    "ERC ENGINEERING TEST MODE"
  );
}


// ============================================================
// RETURN FROM TEST MODE
// ============================================================

void returnToMainScreen()
{
  stopSiren();

  allLEDsOff();


  ledTestActive =
    false;

  activeLEDPin =
    -1;


  activeApproach =
    APPROACH_NONE;

  activeFarPin =
    -1;

  activeNearPin =
    -1;


  priorityStage =
    PRIORITY_IDLE;

  emergencyAcknowledged =
    false;


  currentScreen =
    ERC_MAIN;


  testButtonHolding =
    false;

  testModeTriggered =
    false;


  drawMainScreen();


  Serial.println(
    "RETURNED TO ERC MAIN SCREEN"
  );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  // ----------------------------------------------------------
  // SERIAL MONITOR
  // ----------------------------------------------------------

  Serial.begin(
    115200
  );

  delay(500);


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    " GREENWAVE TECHLABS"
  );

  Serial.println(
    " EMERGENCY RESPONSE CONSOLE"
  );

  Serial.println(
    " ERC V3"
  );

  Serial.println(
    "========================================"
  );


  // ----------------------------------------------------------
  // LED OUTPUTS
  // ----------------------------------------------------------

  pinMode(
    LEFT_FAR_LED,
    OUTPUT
  );

  pinMode(
    LEFT_NEAR_LED,
    OUTPUT
  );

  pinMode(
    RIGHT_FAR_LED,
    OUTPUT
  );

  pinMode(
    RIGHT_NEAR_LED,
    OUTPUT
  );

  pinMode(
    BOTTOM_FAR_LED,
    OUTPUT
  );

  pinMode(
    BOTTOM_NEAR_LED,
    OUTPUT
  );

  allLEDsOff();


  // ----------------------------------------------------------
  // BUZZER
  // ----------------------------------------------------------

  pinMode(
    BUZZER,
    OUTPUT
  );

  digitalWrite(
    BUZZER,
    LOW
  );


  // ----------------------------------------------------------
  // LCD BACKLIGHT
  // ----------------------------------------------------------

  pinMode(
    TFT_BL,
    OUTPUT
  );

  digitalWrite(
    TFT_BL,
    HIGH
  );


  // ----------------------------------------------------------
  // DISPLAY
  // ----------------------------------------------------------

  Serial.println(
    "Starting ST7796 LCD..."
  );

  gfx->begin();


  Serial.print(
    "Display size: "
  );

  Serial.print(
    gfx->width()
  );

  Serial.print(
    " x "
  );

  Serial.println(
    gfx->height()
  );


  // ----------------------------------------------------------
  // TOUCH
  // ----------------------------------------------------------

  Wire.begin(
    TP_SDA,
    TP_SCL
  );

  resetTouchController();


  Wire.beginTransmission(
    FT6336_ADDR
  );

  if (
    Wire.endTransmission() == 0
  )
  {
    Serial.println(
      "FT6336U FOUND @ 0x38"
    );
  }

  else
  {
    Serial.println(
      "ERROR: FT6336U NOT FOUND"
    );
  }


  // ----------------------------------------------------------
  // ICU UART
  // ----------------------------------------------------------

  // PHASE 3 FIX: enlarge the RX buffer BEFORE begin().
  //
  // The default is 256 bytes, which at 9600 baud is about 266 ms of
  // traffic. A full emergency-screen repaint can take longer than that,
  // and anything arriving while the display is busy is silently dropped
  // by the driver -- no error, no flag, just missing commands.
  //
  // 1024 gives roughly a second of slack. That is not a fix for a slow
  // draw; it is headroom so a slow draw costs latency instead of data.
  //
  // Must be called before begin() or it has no effect.
  ICU_UART.setRxBufferSize(1024);

  ICU_UART.begin(
    9600,
    SERIAL_8N1,
    ICU_RX,
    ICU_TX
  );

  // Retained, though nothing reads with a timeout any more --
  // processICUCommunication() assembles lines byte by byte and never
  // blocks. Harmless, and correct if a blocking read is ever added back.
  ICU_UART.setTimeout(50);

  // Discard whatever the receiver latched while the pin was floating
  // between reset and begin(). Same reasoning as the ICU end: a
  // corrupted line is not a missing line, it is a different one.
  delay(50);
  while (ICU_UART.available()) ICU_UART.read();


  Serial.println();

  Serial.println(
    "ICU DIRECT UART INITIALIZED"
  );

  Serial.println(
    "ERC RX = GPIO34"
  );

  Serial.println(
    "ERC TX = GPIO15"
  );

  Serial.println(
    "UART = 9600 8N1"
  );


  // ----------------------------------------------------------
  // INITIAL SCREEN
  // ----------------------------------------------------------

  currentScreen =
    ERC_MAIN;

  priorityStage =
    PRIORITY_IDLE;

  // Branding splash, then the idle screen
  drawSplashScreen();

  delay(1800);

  drawMainScreen();


  Serial.println();

  Serial.println(
    "ERC OPERATIONAL"
  );

  Serial.println(
    "SEARCHING FOR EMERGENCY VEHICLE..."
  );

  Serial.println(
    "Waiting for LEFT / RIGHT / BOTTOM from ICU."
  );
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // BACKGROUND SERVICES
  // ==========================================================

  processICUCommunication();

  updateSiren();

  checkAutoAcknowledge();

  updateMainHeartbeat();

  updatePrioritySequence();

  updateLEDTest();

  // PHASE 3 FIX: service the UART a SECOND time, after the drawing work.
  //
  // updatePrioritySequence() and updateMainHeartbeat() can both repaint
  // regions of the display, and a TFT write over SPI is slow enough that
  // bytes accumulate underneath it. Draining once at the top of the loop
  // means everything arriving during a repaint waits a full pass.
  //
  // The cost of the extra call is nothing when the buffer is empty --
  // it is a single `available()` check. The benefit is that a PING which
  // lands mid-repaint is answered on this pass rather than the next,
  // which is the difference the bench run measured as a 43% PONG loss
  // on a wire that was not dropping anything.
  processICUCommunication();


  // ==========================================================
  // TOUCH
  // ==========================================================

  uint16_t x;
  uint16_t y;

  bool touching =
    getTouch(
      x,
      y
    );


  // ==========================================================
  // MAIN SCREEN
  // ==========================================================

  if (
    currentScreen ==
    ERC_MAIN
  )
  {
    if (touching)
    {
      bool onTestButton =
        inside(
          x,
          y,
          172,
          272,
          136,
          38
        );


      // -------------------------------------------------------
      // TEST BUTTON LONG PRESS
      // -------------------------------------------------------

      if (onTestButton)
      {
        if (
          !testButtonHolding
        )
        {
          testButtonHolding =
            true;

          testModeTriggered =
            false;

          testButtonPressStart =
            millis();

          Serial.println(
            "TEST HOLD STARTED"
          );
        }


        if (
          !testModeTriggered &&
          millis() -
          testButtonPressStart >=
          TEST_HOLD_TIME
        )
        {
          testModeTriggered =
            true;

          enterTestMode();

          previousTouchState =
            true;

          delay(100);

          return;
        }
      }

      else
      {
        testButtonHolding =
          false;

        testModeTriggered =
          false;
      }


      // -------------------------------------------------------
      // NORMAL MAIN BUTTON PRESS
      // -------------------------------------------------------

      if (
        !previousTouchState &&
        !onTestButton
      )
      {
        processMainTouch(
          x,
          y
        );
      }
    }

    else
    {
      testButtonHolding =
        false;

      testModeTriggered =
        false;
    }
  }


  // ==========================================================
  // EMERGENCY SCREEN
  // ==========================================================

  else if (
    currentScreen ==
    ERC_EMERGENCY
  )
  {
    if (
      touching &&
      !previousTouchState
    )
    {
      Serial.print(
        "EMERGENCY TOUCH X:"
      );

      Serial.print(x);

      Serial.print(
        " Y:"
      );

      Serial.println(y);


      processEmergencyTouch(
        x,
        y
      );
    }
  }


  // ==========================================================
  // ENGINEERING TEST SCREEN
  // ==========================================================

  else if (
    currentScreen ==
    ERC_TEST
  )
  {
    if (
      touching &&
      !previousTouchState
    )
    {
      processTestTouch(
        x,
        y
      );
    }
  }


  // ==========================================================
  // WIFI SETUP SCREENS
  // ==========================================================

  else if (
    currentScreen ==
    ERC_WIFI_LIST
  )
  {
    if (
      touching &&
      !previousTouchState
    )
    {
      processWifiListTouch(
        x,
        y
      );
    }
  }

  else if (
    currentScreen ==
    ERC_WIFI_KEY
  )
  {
    if (
      touching &&
      !previousTouchState
    )
    {
      processWifiKeyTouch(
        x,
        y
      );
    }
  }


  // ==========================================================
  // SAVE TOUCH STATE
  // ==========================================================

  previousTouchState =
    touching;


  delay(10);
}