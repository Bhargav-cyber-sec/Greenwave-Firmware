/***********************************************************************
 * test_icu_rx_pipeline.cpp  --  host test for the v7.1 receive pipeline
 *
 * Build and run:
 *
 *     g++ -std=gnu++17 -O2 -pthread -DGW_HOST_TEST \
 *         -I../ICU tests/test_icu_rx_pipeline.cpp -o /tmp/t && /tmp/t
 *
 * WHAT THIS TESTS, AND WHAT IT DOES NOT
 *
 * It includes the REAL ICU/ICU_RxPipeline.h -- the rings, the memory
 * orderings and the claim protocol under test are the ones the firmware
 * compiles. Threads are real, the races are real, and the duplicate
 * cases are driven by two threads contending for the same node.
 *
 * It does NOT include ICU.ino: that needs Arduino, LoRa and mbedTLS. So
 * gwValidateSlot()'s ORDER OF OPERATIONS is mirrored here, in
 * validateSlot() below, the same way tests/test_bypass_cap.cpp mirrors
 * the geofence cap. Mirrors drift, so the mirror is not the only
 * defence: verify_rdu_tree.py's check_rx_pipeline_wired() asserts that
 * the real header is actually included by ICU.ino and that the real
 * validation path still has the claim around it. Read that check as
 * part of this test.
 *
 * The HMAC is a real SHA256-based HMAC (compact implementation below),
 * not a stub, so case 8 rejects on genuine cryptographic failure and the
 * measured HMAC cost in case 13 is a real number rather than a sleep.
 ***********************************************************************/

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <algorithm>

#define GW_MAX_FRAME_LEN 128
#include "ICU_RxPipeline.h"

// ---------------------------------------------------------------------
// globals the header declares
// ---------------------------------------------------------------------
GwRxRing  gwRxRing[GW_RX_NODES];
GwValRing gwValRing;
GwLogRing gwLogRing;
std::atomic<uint8_t> gwNodeOwner[GW_RX_NODES];

void gwPipeLogf(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); gwLogRing.pushv(fmt, ap); va_end(ap);
}

static int g_fail = 0, g_pass = 0;
static void CHECK(bool c, const char *what) {
    if (c) { g_pass++; printf("  [ok]   %s\n", what); }
    else   { g_fail++; printf("  [FAIL] %s\n", what); }
}

// =====================================================================
// COMPACT SHA-256 + HMAC  (real, not a stub)
// =====================================================================
struct Sha256 {
    uint32_t s[8]; uint64_t len; uint8_t buf[64]; size_t n;
    static uint32_t ror(uint32_t x,int r){return (x>>r)|(x<<(32-r));}
    void init(){ static const uint32_t iv[8]={0x6a09e667,0xbb67ae85,0x3c6ef372,
        0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
        memcpy(s,iv,32); len=0; n=0; }
    void block(const uint8_t*p){
        static const uint32_t K[64]={
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
        uint32_t w[64];
        for(int i=0;i<16;i++) w[i]=(p[i*4]<<24)|(p[i*4+1]<<16)|(p[i*4+2]<<8)|p[i*4+3];
        for(int i=16;i<64;i++){
            uint32_t a=ror(w[i-15],7)^ror(w[i-15],18)^(w[i-15]>>3);
            uint32_t b=ror(w[i-2],17)^ror(w[i-2],19)^(w[i-2]>>10);
            w[i]=w[i-16]+a+w[i-7]+b; }
        uint32_t a=s[0],b=s[1],c=s[2],d=s[3],e=s[4],f=s[5],g=s[6],h=s[7];
        for(int i=0;i<64;i++){
            uint32_t S1=ror(e,6)^ror(e,11)^ror(e,25);
            uint32_t ch=(e&f)^((~e)&g);
            uint32_t t1=h+S1+ch+K[i]+w[i];
            uint32_t S0=ror(a,2)^ror(a,13)^ror(a,22);
            uint32_t mj=(a&b)^(a&c)^(b&c);
            uint32_t t2=S0+mj;
            h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2; }
        s[0]+=a;s[1]+=b;s[2]+=c;s[3]+=d;s[4]+=e;s[5]+=f;s[6]+=g;s[7]+=h;
    }
    void update(const uint8_t*p,size_t l){ len+=l;
        while(l){ size_t k=std::min(l,64-n); memcpy(buf+n,p,k); n+=k; p+=k; l-=k;
            if(n==64){ block(buf); n=0; } } }
    void final(uint8_t*out){ uint64_t bits=len*8; uint8_t pad=0x80; update(&pad,1);
        uint8_t z=0; while(n!=56) update(&z,1);
        uint8_t b8[8]; for(int i=0;i<8;i++) b8[i]=(uint8_t)(bits>>(56-8*i));
        update(b8,8);
        for(int i=0;i<8;i++){ out[i*4]=(uint8_t)(s[i]>>24); out[i*4+1]=(uint8_t)(s[i]>>16);
            out[i*4+2]=(uint8_t)(s[i]>>8); out[i*4+3]=(uint8_t)s[i]; } }
};

static void hmac_sha256(const uint8_t*key,size_t kl,const uint8_t*m,size_t ml,uint8_t*out){
    uint8_t k[64]={0};
    if(kl>64){ Sha256 h; h.init(); h.update(key,kl); h.final(k); } else memcpy(k,key,kl);
    uint8_t ip[64],op[64];
    for(int i=0;i<64;i++){ ip[i]=k[i]^0x36; op[i]=k[i]^0x5c; }
    uint8_t in[32];
    Sha256 h1; h1.init(); h1.update(ip,64); h1.update(m,ml); h1.final(in);
    Sha256 h2; h2.init(); h2.update(op,64); h2.update(in,32); h2.final(out);
}

// =====================================================================
// FRAME MODEL  -- mirrors FrameHeader's fields that the pipeline reads.
// Layout is local to this test; the wire format is not under test here
// (verify_rdu_tree.py's check_wire_types owns that).
// =====================================================================
#define TAG_LEN 8
struct TestHdr {
    uint8_t  lane, node, type;
    uint32_t counter;
    uint32_t epoch;
};
#define FRAME_LEN 48

static void buildFrame(uint8_t *f, uint8_t lane, uint8_t node, uint8_t type,
                       uint32_t ctr, uint32_t epoch, const uint8_t *key,
                       bool forgeTag = false) {
    memset(f, 0, FRAME_LEN);
    TestHdr h{lane, node, type, ctr, epoch};
    memcpy(f, &h, sizeof(h));
    // Truncate exactly as gwComputeTag() does: full 32-byte digest into a
    // temp, then copy GW_TAG_LEN bytes. Writing the digest straight into
    // the 8-byte tag field overflows the frame by 24 bytes -- which is
    // what the first draft of this test did, and ASan caught it.
    uint8_t full[32];
    hmac_sha256(key, 32, f, FRAME_LEN - TAG_LEN, full);
    memcpy(f + FRAME_LEN - TAG_LEN, full, TAG_LEN);
    if (forgeTag) f[FRAME_LEN - 1] ^= 0xFF;
}

// =====================================================================
// PER-NODE SECURITY STATE  -- the [W] fields of NodeState, and the
// GwIcuSessionCache Slot, in the shape the real ICU has them.
// =====================================================================
struct SessSlot {
    uint8_t  key[32], prevKey[32];
    uint32_t epoch = 0, prevEpoch = 0;
    bool     have = false, havePrev = false;
};

struct NodeSec {
    uint32_t lastCounter = 0;
    bool     counterStarted = false;
    uint32_t framesAccepted = 0, framesLostIf2 = 0;
    uint32_t rejectedMac = 0, rejectedReplay = 0;
    SessSlot slot;
    bool     revoked = false;
};
static NodeSec g_sec[GW_RX_NODES];

static std::atomic<uint32_t> c_dup{0}, c_replay{0}, c_mac{0},
                             c_revoked{0}, c_accepted{0}, c_badlen{0};

// Simulated X25519+HKDF cost on epoch change. Configurable so the
// head-of-line-blocking case can be driven deterministically instead of
// hoped for.
static std::atomic<int> g_deriveUs{0};

static void deriveKey(uint8_t lane, uint8_t node, uint32_t epoch, uint8_t *out) {
    int us = g_deriveUs.load();
    if (us > 0) {
        auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - t0).count() < us) { }
    }
    uint8_t seed[16];
    memcpy(seed, &lane, 1); memcpy(seed + 1, &node, 1);
    memcpy(seed + 2, &epoch, 4); memset(seed + 6, 0xA5, 10);
    Sha256 h; h.init(); h.update(seed, sizeof(seed)); h.final(out);
}

// Mirrors GwIcuSessionCache::keyFor(), including the overlap window and
// the forward-only epoch rule.
enum VR { V_OK, V_TAG, V_EPOCH, V_REVOKED };
static VR keyFor(int k, uint8_t lane, uint8_t node, uint32_t epoch, const uint8_t **keyOut) {
    NodeSec &n = g_sec[k];
    if (n.revoked) return V_REVOKED;
    SessSlot &s = n.slot;
    if (s.have && s.epoch == epoch)         { *keyOut = s.key;     return V_OK; }
    if (s.havePrev && s.prevEpoch == epoch) { *keyOut = s.prevKey; return V_OK; }
    if (s.have && epoch < s.epoch)          { return V_EPOCH; }

    uint8_t fresh[32];
    deriveKey(lane, node, epoch, fresh);
    if (s.have) { memcpy(s.prevKey, s.key, 32); s.prevEpoch = s.epoch; s.havePrev = true; }
    memcpy(s.key, fresh, 32);
    s.epoch = epoch; s.have = true;
    *keyOut = s.key;
    return V_OK;
}

// =====================================================================
// MIRROR OF gwValidateSlot()  -- same order of operations.
// Called ONLY with node k claimed by the caller.
// =====================================================================
static void validateSlot(int k, const GwRxSlot *s) {
    if (s->len < sizeof(TestHdr) + TAG_LEN) { c_badlen++; return; }

    TestHdr h; memcpy(&h, s->bytes, sizeof(h));
    NodeSec &n = g_sec[k];

    // replay / duplicate BEFORE the MAC
    if (n.counterStarted) {
        if (h.counter == n.lastCounter) { c_dup++; return; }
        if (h.counter <  n.lastCounter) { n.rejectedReplay++; c_replay++; return; }
    }

    const uint8_t *key = nullptr;
    VR r = keyFor(k, h.lane, h.node, h.epoch, &key);
    if (r == V_REVOKED) { c_revoked++; return; }
    if (r != V_OK)      { c_mac++; return; }

    uint8_t exp[32];
    hmac_sha256(key, 32, s->bytes, s->len - TAG_LEN, exp);
    if (memcmp(exp, s->bytes + s->len - TAG_LEN, TAG_LEN) != 0) {
        n.rejectedMac++; c_mac++; return;
    }

    if (n.counterStarted && h.counter > n.lastCounter + 1)
        n.framesLostIf2 += (h.counter - n.lastCounter - 1);
    n.framesAccepted++;
    n.lastCounter = h.counter;
    n.counterStarted = true;
    c_accepted++;

    GwValEvent e{};
    memcpy(e.bytes, s->bytes, s->len);
    e.len = s->len; e.nodeIdx = (uint8_t)k; e.counter = h.counter;
    e.sessionEpoch = h.epoch;
    gwValRing.push(e);
}

// ---- claim-exclusivity instrumentation ------------------------------
static std::atomic<int> g_inNode[GW_RX_NODES];
static std::atomic<int> g_exclusivityViolations{0};

static void drainNode(int k) {
    if (g_inNode[k].fetch_add(1) != 0) g_exclusivityViolations++;
    const GwRxSlot *s;
    while ((s = gwRxRing[k].peek()) != nullptr) { validateSlot(k, s); gwRxRing[k].pop(); }
    g_inNode[k].fetch_sub(1);
}

static std::atomic<bool> g_stop{false};

static void workerTask(uint8_t id) {
    while (!g_stop.load(std::memory_order_relaxed)) {
        for (int i = 0; i < GW_RX_NODES; i++) {
            int k = (i + id) % GW_RX_NODES;
            if (gwRxRing[k].depth() == 0) continue;
            if (!gwClaimNode(k, id)) continue;      // never waits
            drainNode(k);
            gwReleaseNode(k);
        }
        std::this_thread::yield();
    }
}

// ---- helpers ---------------------------------------------------------
static void resetAll() {
    for (int i = 0; i < GW_RX_NODES; i++) {
        gwRxRing[i].head = 0; gwRxRing[i].tail = 0;
        gwRxRing[i].dropped = 0; gwRxRing[i].maxDepth = 0;
        gwNodeOwner[i] = 0; g_inNode[i] = 0;
        g_sec[i] = NodeSec{};
    }
    gwValRing.head = gwValRing.tail = 0; gwValRing.dropped = 0;
    gwLogRing.head = gwLogRing.tail = 0; gwLogRing.dropped = 0;
    c_dup = c_replay = c_mac = c_revoked = c_accepted = c_badlen = 0;
    g_exclusivityViolations = 0;
    g_deriveUs = 0;
}

static bool produce(int k, const uint8_t *frame, int len) {
    GwRxSlot *s = gwRxRing[k].reserve();
    if (!s) return false;
    memcpy(s->bytes, frame, len);
    s->len = (uint16_t)len; s->rssi = -70; s->snr_x10 = 90;
    s->t_rx_us = 0; s->t_q_us = 0;
    gwRxRing[k].commit();
    return true;
}

static void nodeKey(int k, uint32_t epoch, uint8_t *out) {
    deriveKey((uint8_t)(k / 2 + 1), (uint8_t)(k % 2 + 1), epoch, out);
}

static void runWorkersUntilDrained(int nWorkers, int maxMs = 2000) {
    g_stop = false;
    std::vector<std::thread> th;
    for (int i = 0; i < nWorkers; i++) th.emplace_back(workerTask, (uint8_t)i);
    auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        uint32_t d = 0;
        for (int i = 0; i < GW_RX_NODES; i++) d += gwRxRing[i].depth();
        if (d == 0) break;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count() > maxMs) break;
        std::this_thread::yield();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    g_stop = true;
    for (auto &t : th) t.join();
}

// =====================================================================
// CASES
// =====================================================================
int main() {
    uint8_t f[FRAME_LEN], k0[32], k1[32];

    printf("\n=== ICU v7.1 receive pipeline ===\n");
    printf("sizeof(GwRxSlot)=%zu  rings=%zu B  val=%zu B  log=%zu B\n\n",
           sizeof(GwRxSlot), sizeof(gwRxRing), sizeof(gwValRing), sizeof(gwLogRing));

    // ---- Case 1: one RDU, one frame ---------------------------------
    printf("Case 1  -- 1 RDU, single request\n");
    resetAll(); nodeKey(0, 1, k0);
    buildFrame(f, 1, 1, 2, 100, 1, k0);
    produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(2);
    CHECK(c_accepted == 1, "exactly one accepted transaction");
    CHECK(g_sec[0].lastCounter == 100, "replay counter advanced to 100");

    // ---- Case 2: duplicate, sequential ------------------------------
    printf("Case 2  -- same RDU sends request X twice (sequential)\n");
    resetAll(); nodeKey(0, 1, k0);
    buildFrame(f, 1, 1, 2, 100, 1, k0);
    produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(2);
    produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(2);
    CHECK(c_accepted == 1, "one accepted, not two");
    CHECK(c_dup == 1, "second counted as benign duplicate, not replay");
    CHECK(c_replay == 0, "same counter is NOT reported as a replay");

    // ---- Case 3: duplicate, simultaneous ----------------------------
    printf("Case 3  -- same RDU, two copies of X in flight at once\n");
    resetAll(); nodeKey(0, 1, k0);
    buildFrame(f, 1, 1, 2, 100, 1, k0);
    produce(0, f, FRAME_LEN); produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(4);           // 4 workers contending for node 0
    CHECK(c_accepted == 1, "still exactly one accepted under contention");
    CHECK(c_dup == 1, "the losing copy is suppressed");
    CHECK(g_exclusivityViolations == 0, "never two owners of one node's state");

    // ---- Case 4: two different RDUs concurrently --------------------
    printf("Case 4  -- RDU1 sends X while RDU2 sends Y\n");
    resetAll(); nodeKey(0, 1, k0); nodeKey(2, 1, k1);
    buildFrame(f, 1, 1, 2, 100, 1, k0); produce(0, f, FRAME_LEN);
    buildFrame(f, 2, 1, 2, 500, 1, k1); produce(2, f, FRAME_LEN);
    runWorkersUntilDrained(2);
    CHECK(c_accepted == 2, "both independent requests accepted");
    CHECK(g_sec[0].lastCounter == 100 && g_sec[2].lastCounter == 500,
          "each node advanced its own counter only");

    // ---- Case 5: epoch transition on one node, other unaffected -----
    printf("Case 5  -- RDU1 advances session epoch while RDU2 validates\n");
    resetAll(); nodeKey(0, 2, k0); nodeKey(2, 1, k1);
    buildFrame(f, 1, 1, 2, 101, 2, k0); produce(0, f, FRAME_LEN);
    buildFrame(f, 2, 1, 2, 501, 1, k1); produce(2, f, FRAME_LEN);
    runWorkersUntilDrained(2);
    CHECK(c_accepted == 2, "both accepted across an epoch change");
    CHECK(g_sec[0].slot.epoch == 2, "node 0 session epoch advanced");
    CHECK(g_sec[2].slot.epoch == 1, "node 2 session state untouched by node 0");

    // ---- Case 5b: overlap window ------------------------------------
    printf("Case 5b -- frame from the PRIOR epoch still verifies (SOP 5.3)\n");
    resetAll();
    nodeKey(0, 1, k0); buildFrame(f, 1, 1, 2, 100, 1, k0); produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(1);
    nodeKey(0, 2, k1); buildFrame(f, 1, 1, 2, 101, 2, k1); produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(1);
    nodeKey(0, 1, k0); buildFrame(f, 1, 1, 2, 102, 1, k0); produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(1);
    CHECK(c_accepted == 3, "in-flight frame from the prior epoch is accepted");

    // ---- Case 6: same-RDU sequential ordering -----------------------
    printf("Case 6  -- two different packets from the same RDU, close together\n");
    resetAll(); nodeKey(0, 1, k0);
    buildFrame(f, 1, 1, 2, 200, 1, k0); produce(0, f, FRAME_LEN);
    buildFrame(f, 1, 1, 2, 201, 1, k0); produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(4);
    CHECK(c_accepted == 2, "both accepted");
    CHECK(g_sec[0].lastCounter == 201, "counter ends at the LATER value");
    CHECK(c_replay == 0, "in-order arrival never looks like a replay");

    // ---- Case 7: invalid HMAC ---------------------------------------
    printf("Case 7  -- forged tag\n");
    resetAll(); nodeKey(0, 1, k0);
    buildFrame(f, 1, 1, 2, 300, 1, k0, /*forgeTag=*/true);
    produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(2);
    CHECK(c_accepted == 0 && c_mac == 1, "rejected as bad tag");
    CHECK(g_sec[0].counterStarted == false,
          "replay counter NOT advanced by an unauthenticated frame");

    // ---- Case 8: replayed (lower) counter ---------------------------
    printf("Case 8  -- replayed counter (goes backwards)\n");
    resetAll(); nodeKey(0, 1, k0);
    buildFrame(f, 1, 1, 2, 400, 1, k0); produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(2);
    buildFrame(f, 1, 1, 2, 399, 1, k0); produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(2);
    CHECK(c_accepted == 1 && c_replay == 1, "lower counter rejected as REPLAY");
    CHECK(c_dup == 0, "a rollback is not filed as a benign duplicate");

    // ---- Case 9: revoked node ---------------------------------------
    printf("Case 9  -- revoked node\n");
    resetAll(); nodeKey(0, 1, k0); g_sec[0].revoked = true;
    buildFrame(f, 1, 1, 2, 100, 1, k0); produce(0, f, FRAME_LEN);
    runWorkersUntilDrained(2);
    CHECK(c_accepted == 0 && c_revoked == 1, "refused at session establishment");

    // ---- Case 10: malformed / short ---------------------------------
    printf("Case 10 -- malformed short frame\n");
    resetAll();
    memset(f, 0, sizeof(f));
    produce(0, f, 6);
    runWorkersUntilDrained(2);
    CHECK(c_accepted == 0 && c_badlen == 1, "rejected on length before any crypto");

    // ---- Case 11: queue overflow ------------------------------------
    printf("Case 11 -- RX ring overflow is bounded and counted\n");
    resetAll(); nodeKey(0, 1, k0);
    int okCount = 0;
    for (int i = 0; i < GW_RX_RING_DEPTH + 3; i++) {
        buildFrame(f, 1, 1, 2, 600 + i, 1, k0);
        if (produce(0, f, FRAME_LEN)) okCount++;
    }
    CHECK(okCount == GW_RX_RING_DEPTH, "producer accepted exactly DEPTH frames");
    CHECK(gwRxRing[0].dropped.load() == 3, "the excess is counted, not silently lost");
    runWorkersUntilDrained(2);
    CHECK(c_accepted == (uint32_t)GW_RX_RING_DEPTH, "queued frames all processed");

    // ---- Case 12: claim exclusivity under stress --------------------
    printf("Case 12 -- claim exclusivity, 6 nodes x 4 workers, sustained\n");
    resetAll();
    for (int k = 0; k < GW_RX_NODES; k++) nodeKey(k, 1, k0);
    {
        g_stop = false;
        std::vector<std::thread> th;
        for (int i = 0; i < 4; i++) th.emplace_back(workerTask, (uint8_t)i);
        std::thread prod([&]{
            uint8_t fr[FRAME_LEN], kk[32];
            for (int i = 0; i < 400; i++) {
                int k = i % GW_RX_NODES;
                nodeKey(k, 1, kk);
                buildFrame(fr, (uint8_t)(k / 2 + 1), (uint8_t)(k % 2 + 1), 2,
                           1000 + i / GW_RX_NODES, 1, kk);
                produce(k, fr, FRAME_LEN);
                std::this_thread::yield();
            }
        });
        prod.join();
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        g_stop = true;
        for (auto &t : th) t.join();
    }
    CHECK(g_exclusivityViolations == 0,
          "no two workers ever inside the same node's security state");
    CHECK(c_replay == 0, "per-node ordering preserved under 4-way contention");

    // ---- Case 13: does adding RDUs serialise validation? ------------
    printf("Case 13 -- latency scaling, 1 vs 2 vs 3 concurrent RDUs\n");
    printf("           (derive cost forced to 3000 us to model X25519 on epoch change)\n");
    for (int nRdu = 1; nRdu <= 3; nRdu++) {
        resetAll();
        g_deriveUs = 3000;                       // every frame here is a new epoch
        for (int k = 0; k < nRdu; k++) {
            uint8_t kk[32]; nodeKey(k * 2, 1, kk);
            buildFrame(f, (uint8_t)(k + 1), 1, 2, 100, 1, kk);
            produce(k * 2, f, FRAME_LEN);
        }
        auto t0 = std::chrono::steady_clock::now();
        runWorkersUntilDrained(2);
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now() - t0).count() - 20000;
        printf("           %d RDU(s): all validated in %5lld us  (accepted=%u)\n",
               nRdu, (long long)(us < 0 ? 0 : us), c_accepted.load());
        CHECK(c_accepted == (uint32_t)nRdu, "all concurrent RDUs validated");
    }
    printf("           2 workers bound the pile-up: 3 RDUs is ~2 derivations deep,\n");
    printf("           not 3. Raise GW_VAL_WORKERS only if [PERF] wait= tracks key=.\n");

    // ---- HMAC cost, for the record ----------------------------------
    {
        uint8_t key[32] = {1}, msg[FRAME_LEN - TAG_LEN] = {2}, out[32];
        const int N = 200000;
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < N; i++) hmac_sha256(key, 32, msg, sizeof(msg), out);
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now() - t0).count() / N;
        printf("\nHMAC-SHA256 over %d B, this host: %lld ns\n",
               (int)sizeof(msg), (long long)ns);
        printf("For scale: one LoRaEventFrame is 129300000 ns of airtime.\n");
    }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
