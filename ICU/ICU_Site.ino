/***********************************************************************
 * ICU_Site.ino  --  PER-JUNCTION CONFIGURATION, HELD IN NVS
 *
 * A new tab in the ICU sketch folder.
 *
 * ---------------------------------------------------------------------
 * WHY THIS EXISTS
 *
 * The stop-line position and the approach bearings are the only values
 * in this system that are genuinely different at every junction. They
 * were compile-time constants, which meant testing four sites required
 * four edit-compile-flash cycles -- and, worse, four opportunities to
 * flash the wrong site's numbers and not notice.
 *
 * That last risk is not hypothetical. A stop line 200 m out does not
 * produce an error. It biases every distance, every ETA and every
 * divergence test in the same direction, quietly, and the system goes
 * on looking like it works.
 *
 * These now live in NVS and are set over the serial console. The
 * compile-time values remain as a FALLBACK ONLY, and the ICU says
 * loudly at boot when it is running on them.
 *
 * ---------------------------------------------------------------------
 * THE WORKFLOW THIS IS BUILT FOR
 *
 * Testing several junctions in a day, without reflashing:
 *
 *   At each site, once:
 *     site name MG Road North
 *     site here 13.02610 77.66300
 *     site bearing 1 274
 *
 *   Then keep a record:
 *     site export
 *
 *   which prints a single line you can paste back later:
 *     site set 13.026100 77.663000 274 180 90
 *
 * Returning to a junction is one paste. Keep the exported line in your
 * notes next to the site name and the whole configuration is one line
 * of text per junction.
 *
 * NVS survives reflashing, so updating firmware does not lose the site.
 ***********************************************************************/

#include <Preferences.h>
#include "GreenwaveLink.h"

static Preferences sitePrefs;

// Sentinel for "never provisioned".
//
// 0.0 would be a poor choice: it is a real coordinate in the Gulf of
// Guinea, and a receiver that silently used it would place every vehicle
// thousands of kilometres away rather than failing visibly.
#define SITE_LAT_UNSET  1000.0
#define SITE_LON_UNSET  1000.0

struct SiteConfig {
    bool     provisioned = false;
    char     name[24]    = "(not set)";
    double   lat         = SITE_LAT_UNSET;
    double   lon         = SITE_LON_UNSET;
    float    bearing[GW_NUM_APPROACHES + 1] = {0};
};

static SiteConfig gwSite;

// Compile-time fallbacks, from ICU_EVU.ino. Used ONLY when NVS is empty,
// and the ICU announces that it is doing so.
extern const double  GW_FALLBACK_LAT;
extern const double  GW_FALLBACK_LON;
extern const float   GW_FALLBACK_BEARING[GW_NUM_APPROACHES + 1];

// =====================================================
// LOAD / SAVE
// =====================================================

void gwSiteLoad() {
    sitePrefs.begin("gw-site", true);          // read-only

    gwSite.provisioned = sitePrefs.getBool("prov", false);

    if (gwSite.provisioned) {
        String n = sitePrefs.getString("name", "(unnamed)");
        strncpy(gwSite.name, n.c_str(), sizeof(gwSite.name) - 1);
        gwSite.name[sizeof(gwSite.name) - 1] = '\0';

        gwSite.lat = sitePrefs.getDouble("lat", SITE_LAT_UNSET);
        gwSite.lon = sitePrefs.getDouble("lon", SITE_LON_UNSET);

        for (int i = 1; i <= GW_NUM_APPROACHES; i++) {
            char key[8];
            snprintf(key, sizeof(key), "brg%d", i);
            gwSite.bearing[i] = sitePrefs.getFloat(key, GW_FALLBACK_BEARING[i]);
        }
    }

    sitePrefs.end();

    // A stored position outside plausible bounds is treated as absent.
    // Corrupted NVS should fall back to a known value, not to a
    // coordinate somewhere in the ocean.
    if (gwSite.provisioned &&
        (gwSite.lat < -90.0 || gwSite.lat > 90.0 ||
         gwSite.lon < -180.0 || gwSite.lon > 180.0)) {
        Serial.println("[SITE] stored position is out of range -- ignoring it");
        gwSite.provisioned = false;
    }

    if (!gwSite.provisioned) {
        gwSite.lat = GW_FALLBACK_LAT;
        gwSite.lon = GW_FALLBACK_LON;
        for (int i = 1; i <= GW_NUM_APPROACHES; i++) {
            gwSite.bearing[i] = GW_FALLBACK_BEARING[i];
        }
        strncpy(gwSite.name, "(compiled default)", sizeof(gwSite.name) - 1);
    }
}

static void gwSiteSave() {
    sitePrefs.begin("gw-site", false);         // read-write

    sitePrefs.putBool("prov", true);
    sitePrefs.putString("name", gwSite.name);
    sitePrefs.putDouble("lat", gwSite.lat);
    sitePrefs.putDouble("lon", gwSite.lon);

    for (int i = 1; i <= GW_NUM_APPROACHES; i++) {
        char key[8];
        snprintf(key, sizeof(key), "brg%d", i);
        sitePrefs.putFloat(key, gwSite.bearing[i]);
    }

    sitePrefs.end();
    gwSite.provisioned = true;
}

// =====================================================
// ACCESSORS  -- used by ICU_EVU.ino
// =====================================================

double gwSiteLat() { return gwSite.lat; }
double gwSiteLon() { return gwSite.lon; }

float gwSiteBearing(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return 0.0f;
    return gwSite.bearing[lane];
}

bool gwSiteProvisioned() { return gwSite.provisioned; }

// =====================================================
// REPORTING
// =====================================================

void gwSitePrint() {
    Serial.println("[SITE]");
    Serial.printf("  name        : %s\n", gwSite.name);
    Serial.printf("  stop line   : %.6f, %.6f\n", gwSite.lat, gwSite.lon);

    for (int i = 1; i <= GW_NUM_APPROACHES; i++) {
        const char *nm = gwErcApproachName(i);
        Serial.printf("  approach %d  : %-6s bearing %.0f deg\n",
                      i, nm ? nm : "?", gwSite.bearing[i]);
    }

    if (!gwSite.provisioned) {
        Serial.println();
        Serial.println("  *** RUNNING ON COMPILED DEFAULTS -- NOT A SURVEYED SITE ***");
        Serial.println("  *** Every distance, ETA and divergence test is measured");
        Serial.println("  *** from the position above. Set it with:");
        Serial.println("  ***     site here <lat> <lon>");
        Serial.println("  ***     site bearing <lane> <degrees>");
    }
}

// One line containing the whole configuration, in a form that can be
// pasted straight back in.
//
// This is what makes several junctions practical. Keep the exported line
// in your notes beside the site name, and returning to that junction is
// a single paste rather than a survey.
void gwSiteExport() {
    Serial.println();
    Serial.println("--- copy this line into your notes for this junction ---");
    Serial.printf("site set %.6f %.6f", gwSite.lat, gwSite.lon);
    for (int i = 1; i <= GW_NUM_APPROACHES; i++) {
        Serial.printf(" %.0f", gwSite.bearing[i]);
    }
    Serial.println();
    Serial.printf("  (site: %s)\n", gwSite.name);
    Serial.println();
}

// =====================================================
// CONSOLE COMMANDS
// =====================================================
//
// Returns true if the line was a site command.

bool gwSiteCommand(int argc, char **argv) {
    if (argc < 1 || strcasecmp(argv[0], "site") != 0) return false;

    // ---- site ----
    if (argc == 1) {
        gwSitePrint();
        return true;
    }

    // ---- site here <lat> <lon> ----
    if (strcasecmp(argv[1], "here") == 0) {
        if (argc < 4) {
            Serial.println("[SITE] site here <lat> <lon>");
            Serial.println("       stand AT the stop line and use a GPS fix");
            return true;
        }

        double la = atof(argv[2]);
        double lo = atof(argv[3]);

        if (la < -90.0 || la > 90.0 || lo < -180.0 || lo > 180.0) {
            Serial.printf("[SITE] %.6f, %.6f is not a valid position\n", la, lo);
            return true;
        }

        gwSite.lat = la;
        gwSite.lon = lo;
        gwSiteSave();

        Serial.printf("[SITE] stop line set to %.6f, %.6f and saved\n", la, lo);
        Serial.println("[SITE] every distance and ETA is now measured from here");
        return true;
    }

    // ---- site bearing <lane> <deg> ----
    if (strcasecmp(argv[1], "bearing") == 0) {
        if (argc < 4) {
            Serial.println("[SITE] site bearing <lane> <degrees>");
            Serial.println("       stand at the stop line, face AWAY down the");
            Serial.println("       approach, and read the compass");
            return true;
        }

        int   lane = atoi(argv[2]);
        float deg  = atof(argv[3]);

        if (lane < 1 || lane > GW_NUM_APPROACHES) {
            Serial.printf("[SITE] lane %d out of range (1..%d)\n",
                          lane, GW_NUM_APPROACHES);
            return true;
        }

        while (deg < 0.0f)     deg += 360.0f;
        while (deg >= 360.0f)  deg -= 360.0f;

        gwSite.bearing[lane] = deg;
        gwSiteSave();

        const char *nm = gwErcApproachName(lane);
        Serial.printf("[SITE] approach %d (%s) bearing set to %.0f deg and saved\n",
                      lane, nm ? nm : "?", deg);
        return true;
    }

    // ---- site name <text> ----
    if (strcasecmp(argv[1], "name") == 0) {
        if (argc < 3) {
            Serial.println("[SITE] site name <text>");
            return true;
        }

        gwSite.name[0] = '\0';
        for (int i = 2; i < argc; i++) {
            if (i > 2) strncat(gwSite.name, " ",
                               sizeof(gwSite.name) - strlen(gwSite.name) - 1);
            strncat(gwSite.name, argv[i],
                    sizeof(gwSite.name) - strlen(gwSite.name) - 1);
        }

        gwSiteSave();
        Serial.printf("[SITE] name set to \"%s\"\n", gwSite.name);
        return true;
    }

    // ---- site set <lat> <lon> <b1> <b2> <b3> ----
    //
    // The whole configuration in one line. This is the paste-back form
    // produced by 'site export'.
    if (strcasecmp(argv[1], "set") == 0) {
        if (argc < 4 + GW_NUM_APPROACHES) {
            Serial.printf("[SITE] site set <lat> <lon>");
            for (int i = 1; i <= GW_NUM_APPROACHES; i++) Serial.printf(" <brg%d>", i);
            Serial.println();
            return true;
        }

        double la = atof(argv[2]);
        double lo = atof(argv[3]);

        if (la < -90.0 || la > 90.0 || lo < -180.0 || lo > 180.0) {
            Serial.printf("[SITE] %.6f, %.6f is not a valid position\n", la, lo);
            return true;
        }

        gwSite.lat = la;
        gwSite.lon = lo;

        for (int i = 1; i <= GW_NUM_APPROACHES; i++) {
            float deg = atof(argv[3 + i]);
            while (deg < 0.0f)    deg += 360.0f;
            while (deg >= 360.0f) deg -= 360.0f;
            gwSite.bearing[i] = deg;
        }

        gwSiteSave();
        Serial.println("[SITE] configuration loaded and saved:");
        gwSitePrint();
        return true;
    }

    // ---- site export ----
    if (strcasecmp(argv[1], "export") == 0) {
        gwSiteExport();
        return true;
    }

    // ---- site clear ----
    if (strcasecmp(argv[1], "clear") == 0) {
        sitePrefs.begin("gw-site", false);
        sitePrefs.clear();
        sitePrefs.end();

        gwSite.provisioned = false;
        gwSiteLoad();

        Serial.println("[SITE] cleared -- back to compiled defaults");
        gwSitePrint();
        return true;
    }

    Serial.println("[SITE] site | here | bearing | name | set | export | clear");
    return true;
}
