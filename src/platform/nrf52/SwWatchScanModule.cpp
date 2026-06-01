#include "configuration.h"

#if defined(ARCH_NRF52) && defined(SWWATCH_SCAN)
#include "MeshService.h"
#include "main.h" // nrf52Bluetooth
#include "mesh/Channels.h"
#include "modules/esp32/SwWatchScanModule.h"
#include <bluefruit.h>
#include <string.h>
#ifdef BLE_HRM_CONNECT
#include "modules/esp32/GarminPairStore.h"
#endif
#if HAS_SCREEN
#include "graphics/Screen.h"
#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#ifdef BLE_HRM_CONNECT
#include "graphics/draw/MenuHandler.h"
#endif
#endif

SwWatchScanModule *swWatchScanModule;

// Send watch vitals only on this channel (must exist with a matching PSK on all
// your nodes; falls back to the primary channel if not present).
#define SWWATCH_CHANNEL_NAME "WatchData"

// How often to forward each watch's vitals into the mesh (ms). Default 30 s.
#define SWWATCH_FORWARD_INTERVAL_MS 30000
// Runtime-configurable forward interval when Garmin HR is built in.
#ifdef BLE_HRM_CONNECT
#define FWD_INTERVAL_MS (garminIntervalGet())
#else
#define FWD_INTERVAL_MS SWWATCH_FORWARD_INTERVAL_MS
#endif

// Support multiple sw_watch devices simultaneously (keyed by 8-byte device id).
#define SWWATCH_MAX_DEVICES 4
#define SWWATCH_STALE_MS 120000 // free a slot if a watch hasn't been heard for 2 min

// --- sw_watch advertisement constants (see Oliver0804/sw_watch VitalsAdvPacket.kt) ---
static constexpr uint8_t SWW_COMPANY_LO = 0xFF;
static constexpr uint8_t SWW_COMPANY_HI = 0xFF; // company id 0xFFFF
static constexpr uint8_t SWW_DEVID_LEN = 8;
static constexpr uint8_t SWW_TYPE_CORE = 0x01;   // 20 bytes
static constexpr uint8_t SWW_TYPE_MOTION = 0x02; // 21 bytes
static constexpr uint8_t SWW_TYPE_ENV = 0x03;    // 20 bytes

// Per-watch state. Written from the BLE scan callback (SoftDevice context), read
// by runOnce()/drawFrame(). The SD serialises scan callbacks so writes aren't
// re-entrant; reads race benignly (display only).
struct WatchSlot {
    bool used;
    uint8_t id[SWW_DEVID_LEN];
    uint8_t hr, spo2, accM;
    int32_t lat, lon;
    int32_t steps;
    int16_t skinTempX100;
    bool haveEnv;
    bool coreReady; // a fresh CORE frame is pending forward
    bool hasSent;
    uint32_t lastSeenMs;
    uint32_t lastSendMs;
};
static WatchSlot slots[SWWATCH_MAX_DEVICES];

static inline int32_t be32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]);
}

// Find the slot for this device id, or allocate a free/stale one. -1 if full of fresh devices.
static int findOrAllocSlot(const uint8_t *id)
{
    int freeIdx = -1;
    for (int i = 0; i < SWWATCH_MAX_DEVICES; i++) {
        if (slots[i].used && memcmp(slots[i].id, id, SWW_DEVID_LEN) == 0)
            return i;
        if (freeIdx < 0 && !slots[i].used)
            freeIdx = i;
    }
    if (freeIdx < 0) { // all used: evict the most-stale one if any is stale
        uint32_t now = millis(), oldest = 0;
        for (int i = 0; i < SWWATCH_MAX_DEVICES; i++) {
            uint32_t age = now - slots[i].lastSeenMs;
            if (age > SWWATCH_STALE_MS && age > oldest) { oldest = age; freeIdx = i; }
        }
    }
    if (freeIdx < 0)
        return -1;
    memset(&slots[freeIdx], 0, sizeof(WatchSlot));
    slots[freeIdx].used = true;
    memcpy(slots[freeIdx].id, id, SWW_DEVID_LEN);
    return freeIdx;
}

static void parseSwwBody(const uint8_t *p, size_t len)
{
    uint8_t type = p[0];
    const uint8_t *id = p + 1;
    const uint8_t *b = p + 1 + SWW_DEVID_LEN; // skip type + 8-byte device id

    int s = findOrAllocSlot(id);
    if (s < 0)
        return;
    WatchSlot &w = slots[s];
    w.lastSeenMs = millis();

    if (type == SWW_TYPE_CORE && len == 20) {
        w.hr = b[0];
        w.spo2 = b[1];
        w.lat = be32(b + 2);
        w.lon = be32(b + 6);
        w.accM = b[10];
        w.coreReady = true;
        LOG_DEBUG("sw_watch[%02x%02x%02x%02x] CORE hr=%d spo2=%d", id[0], id[1], id[2], id[3], (int)w.hr, (int)w.spo2);
    } else if (type == SWW_TYPE_ENV && len == 20) {
        w.steps = be32(b + 4); // pressure(2) alt(2) steps(4) skin(2)
        w.skinTempX100 = (int16_t)((b[8] << 8) | b[9]);
        w.haveEnv = true;
    }
}

#ifdef BLE_HRM_CONNECT
// --- Optional: also connect to a standard BLE HR sensor (Garmin) as central ---
// Same single scanner: the scan callback detects the HR service and kicks off a
// central connect; sw_watch broadcasts keep arriving on the same callback.
static BLEClientService garminHrSvc(UUID16_SVC_HEART_RATE);                  // 0x180D
static BLEClientCharacteristic garminHrChr(UUID16_CHR_HEART_RATE_MEASUREMENT); // 0x2A37
static bool garminConnected = false;
static uint16_t garminHr = 0;
static volatile bool garminReady = false;
static volatile bool garminValid = false; // last notify carried a real on-wrist reading
static uint32_t garminLastSeenMs = 0, garminLastChangeMs = 0, garminLastSendMs = 0;
static bool garminHasSent = false;
static char garminMacStr[18] = {0}; // "aa:bb:cc:dd:ee:ff" of the locked-onto watch

// Pairing mode: a short scan collects nearby HR sensors into a candidate list for
// the on-screen picker (triggered from the system menu).
#define GARMIN_PAIR_SCAN_MS 6000
#define GARMIN_MAX_CAND 6
static volatile bool garminPairing = false;
static volatile bool garminPairReq = false;
static uint32_t garminPairStartMs = 0;
static char garminCand[GARMIN_MAX_CAND][18];
static volatile uint8_t garminCandCount = 0;

// Optionally lock onto one specific watch. Define at build time, e.g.
//   -D GARMIN_TARGET_MAC='"64:a3:37:00:e8:5c"'
// Garmin advertises a STATIC public address (verified type=0), so a fixed MAC is reliable.
#ifndef GARMIN_TARGET_MAC
#define GARMIN_TARGET_MAC ""
#endif

// SoftDevice stores addr bytes LSB-first; format MSB-first like a printed MAC.
static void garminFormatAddr(const uint8_t *a, char *out)
{
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x", a[5], a[4], a[3], a[2], a[1], a[0]);
}

// Off-wrist behaviour is watch-dependent: some Garmins STOP notifying, others keep
// re-sending the SAME frozen value with the contact flag still set. Both look stale
// as "value hasn't changed for a while", so we gate freshness on the last value CHANGE.
#define GARMIN_FRESH_MS 12000

static void garmin_notify_cb(BLEClientCharacteristic *, uint8_t *data, uint16_t len)
{
    if (len < 2)
        return;
    uint8_t flags = data[0];
    uint16_t v = (flags & 0x01) ? (uint16_t)(data[1] | (data[2] << 8)) : data[1];
    bool contactSupported = flags & 0x04; // bit2
    bool contactDetected = flags & 0x02;  // bit1
    garminLastSeenMs = millis();
    if ((contactSupported && !contactDetected) || v == 0) { // off-wrist or no reading
        garminValid = false;
        return;
    }
    if (v != garminHr) // real HR jitters; a changing value means it's live
        garminLastChangeMs = millis();
    garminHr = v;
    garminValid = true;
    garminReady = true;
    LOG_DEBUG("garmin notify hr=%d", (int)garminHr);
}

static inline bool garminHasFresh() { return garminValid && (millis() - garminLastChangeMs) < GARMIN_FRESH_MS; }

static void garmin_connect_cb(uint16_t conn_handle)
{
    if (!garminHrSvc.discover(conn_handle) || !garminHrChr.discover()) {
        LOG_WARN("garmin: HR service/char not found, dropping");
        Bluefruit.disconnect(conn_handle);
        return;
    }
    garminHrChr.enableNotify();
    garminConnected = true;
    garminLastSeenMs = millis();
    LOG_INFO("garmin connected, subscribed 0x2A37");
}

static void garmin_disconnect_cb(uint16_t, uint8_t)
{
    garminConnected = false;
    LOG_INFO("garmin disconnected -> scanner keeps running");
}
#endif

// Bluefruit scan callback (SoftDevice context). Pull the manufacturer-specific
// data field straight out of the advertising report; match company id 0xFFFF.
static void swwatch_scan_cb(ble_gap_evt_adv_report_t *report)
{
    uint8_t md[31];
    uint8_t mdLen = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, md, sizeof(md));
    if (mdLen >= 4 && md[0] == SWW_COMPANY_LO && md[1] == SWW_COMPANY_HI)
        parseSwwBody(md + 2, (size_t)mdLen - 2); // skip 2-byte company id
#ifdef BLE_HRM_CONNECT
    // HR sensor seen and we're free: start a central connection. Bluefruit allows
    // connect() straight from the scan callback; it pauses scanning until connected.
    else if (Bluefruit.Scanner.checkReportForUuid(report, garminHrSvc.uuid)) {
        char mac[18];
        garminFormatAddr(report->peer_addr.addr, mac);
        // Pairing mode: collect unique candidates for the picker, don't connect.
        if (garminPairing) {
            bool dup = false;
            for (uint8_t i = 0; i < garminCandCount; i++)
                if (strcasecmp(garminCand[i], mac) == 0) { dup = true; break; }
            if (!dup && garminCandCount < GARMIN_MAX_CAND) {
                strncpy(garminCand[garminCandCount], mac, sizeof(garminCand[0]) - 1);
                garminCandCount++;
                LOG_INFO("garmin pair candidate %s type=%d", mac, (int)report->peer_addr.addr_type);
            }
            Bluefruit.Scanner.resume();
            return;
        }
        // Normal: only auto-connect to a remembered (paired) watch.
        if (!garminConnected && garminPairContains(mac)) {
            strncpy(garminMacStr, mac, sizeof(garminMacStr) - 1);
            Bluefruit.Central.connect(report);
            return; // connect() handles scan restart via restartOnDisconnect
        }
        Bluefruit.Scanner.resume();
        return;
    }
#endif
    // The Adafruit Bluefruit scanner does NOT auto-rearm after a report; every
    // central-scan example re-arms it here, else scanning stalls after one packet.
    Bluefruit.Scanner.resume();
}

#ifdef BLE_HRM_CONNECT
// After a pairing scan, show found watches; the chosen one is remembered and
// auto-connected on the next scan pass. Falls back to auto-add if headless.
static void garminShowPairPicker()
{
    if (garminCandCount == 0) {
        LOG_INFO("garmin pair: no watches found");
#if HAS_SCREEN
        if (screen)
            screen->showSimpleBanner("No Garmin found", 3000);
#endif
        return;
    }
#if HAS_SCREEN
    static const char *opts[GARMIN_MAX_CAND + 1];
    static char labels[GARMIN_MAX_CAND][24];
    static char title[20];
    opts[0] = "Cancel";
    uint8_t n = garminCandCount;
    for (uint8_t i = 0; i < n; i++) {
        snprintf(labels[i], sizeof(labels[i]), "%s", garminCand[i]);
        opts[i + 1] = labels[i];
    }
    snprintf(title, sizeof(title), "Pick Garmin (%d)", (int)n);
    graphics::BannerOverlayOptions b;
    b.message = title;
    b.optionsArrayPtr = opts;
    b.optionsCount = n + 1;
    b.durationMs = 30000;
    b.bannerCallback = [](int sel) {
        if (sel >= 1 && sel <= garminCandCount) {
            garminPairAdd(garminCand[sel - 1]);
            LOG_INFO("garmin paired %s (%d total)", garminCand[sel - 1], (int)garminPairCount());
        }
    };
    if (screen)
        screen->showOverlayBanner(b);
#else
    garminPairAdd(garminCand[0]);
#endif
}
#endif

int32_t SwWatchScanModule::runOnce()
{
    // Don't touch the BLE stack until Bluefruit has been brought up.
    if (!nrf52Bluetooth)
        return 5000;

    if (!scanning) {
#ifdef BLE_HRM_CONNECT
        // Bring up the Garmin HR central client once, before scanning starts.
        garminHrChr.setNotifyCallback(garmin_notify_cb);
        garminHrSvc.begin();
        garminHrChr.begin();
        Bluefruit.Central.setConnectCallback(garmin_connect_cb);
        Bluefruit.Central.setDisconnectCallback(garmin_disconnect_cb);
#endif
        Bluefruit.Scanner.setRxCallback(swwatch_scan_cb);
        Bluefruit.Scanner.restartOnDisconnect(true);
        Bluefruit.Scanner.setInterval(160, 80); // 0.625 ms units: 100 ms interval / 50 ms window
        Bluefruit.Scanner.useActiveScan(false); // passive: data is in the adv itself, saves power
        Bluefruit.Scanner.start(0);             // 0 = continuous, never stop
        scanning = true;
        LOG_INFO("sw_watch scanner started (passive, Bluefruit)");
#ifdef BLE_HRM_CONNECT
        garminPairLoad();
        garminIntervalLoad();
        if (((const char *)GARMIN_TARGET_MAC)[0])
            garminPairAdd(GARMIN_TARGET_MAC);
        LOG_INFO("garmin: %d paired watch(es), tx interval %lus", (int)garminPairCount(),
                 (unsigned long)(garminIntervalGet() / 1000));
#endif
    }

#ifdef BLE_HRM_CONNECT
    // Pairing: run a short scan, then show the candidate picker.
    if (garminPairReq && !garminPairing) {
        garminPairReq = false;
        garminCandCount = 0;
        garminPairStartMs = millis();
        garminPairing = true;
        LOG_INFO("garmin: pairing scan started");
    }
    if (garminPairing && (millis() - garminPairStartMs > GARMIN_PAIR_SCAN_MS)) {
        garminPairing = false;
        garminShowPairPicker();
    }
#endif

#if HAS_SCREEN
    // Once a watch is heard, ask Screen to (re)build its frameset so our page appears.
    if (!framePosted) {
        bool any = false;
        for (int i = 0; i < SWWATCH_MAX_DEVICES; i++)
            any |= slots[i].used;
#ifdef BLE_HRM_CONNECT
        any |= garminConnected;
#endif
        if (any) {
            UIFrameEvent e;
            // BACKGROUND = FOCUS_PRESERVE: add our frame without yanking the user to
            // page 1 (plain REGENERATE_FRAMESET steals focus to the first frame).
            e.action = UIFrameEvent::Action::REGENERATE_FRAMESET_BACKGROUND;
            notifyObservers(&e);
            framePosted = true;
            LOG_INFO("sw_watch: requested screen frame");
        }
    }
#endif

    // Forward every watch that has fresh data and is past its own throttle window.
    uint32_t now = millis();
    int ch = channels.getByName(SWWATCH_CHANNEL_NAME).index; // encrypted private channel
    for (int i = 0; i < SWWATCH_MAX_DEVICES; i++) {
        WatchSlot &w = slots[i];
        if (!w.used || !w.coreReady)
            continue;
        if (w.hasSent && (now - w.lastSendMs) < FWD_INTERVAL_MS)
            continue;
        w.coreReady = false;

        char msg[80];
        double lat = (w.lat == INT32_MIN) ? 0.0 : w.lat / 1e7;
        double lon = (w.lon == INT32_MIN) ? 0.0 : w.lon / 1e7;
        int len = snprintf(msg, sizeof(msg), "%02x%02x%02x%02x%02x%02x%02x%02x hr=%d spo2=%d %.6f,%.6f ~%dm", w.id[0],
                           w.id[1], w.id[2], w.id[3], w.id[4], w.id[5], w.id[6], w.id[7], (int)w.hr, (int)w.spo2, lat,
                           lon, (int)w.accM);

        meshtastic_MeshPacket *pkt = allocDataPacket();
        pkt->to = NODENUM_BROADCAST;
        pkt->channel = ch;
        pkt->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP; // show as chat message on the channel
        pkt->decoded.payload.size = len;
        memcpy(pkt->decoded.payload.bytes, msg, len);
        service->sendToMesh(pkt);
        w.lastSendMs = now;
        w.hasSent = true;
        LOG_INFO("sw_watch -> mesh: %s", msg);
    }

#ifdef BLE_HRM_CONNECT
    // Drop a stale Garmin link, then forward fresh HR on the same channel/throttle.
    if (garminConnected && (now - garminLastSeenMs > SWWATCH_STALE_MS)) {
        if (garminHrChr.connHandle() != BLE_CONN_HANDLE_INVALID)
            Bluefruit.disconnect(garminHrChr.connHandle());
        garminConnected = false;
        LOG_WARN("garmin stale, dropped");
    }
    if (garminReady && garminHasFresh() && (!garminHasSent || (now - garminLastSendMs) >= FWD_INTERVAL_MS)) {
        garminReady = false;
        char gmsg[64];
        // Include the MAC so multiple Garmins across the mesh stay distinguishable.
        int glen = snprintf(gmsg, sizeof(gmsg), "garmin %s hr=%d", garminMacStr, (int)garminHr);
        meshtastic_MeshPacket *gp = allocDataPacket();
        gp->to = NODENUM_BROADCAST;
        gp->channel = ch;
        gp->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        gp->decoded.payload.size = glen;
        memcpy(gp->decoded.payload.bytes, gmsg, glen);
        service->sendToMesh(gp);
        garminLastSendMs = now;
        garminHasSent = true;
        LOG_INFO("garmin -> mesh: %s", gmsg);
    }
#endif
    return 1500;
}

#if HAS_SCREEN
bool SwWatchScanModule::wantUIFrame()
{
    for (int i = 0; i < SWWATCH_MAX_DEVICES; i++)
        if (slots[i].used)
            return true;
#ifdef BLE_HRM_CONNECT
    if (garminConnected)
        return true;
#endif
    return false;
}

void SwWatchScanModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->clear();
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);

    int count = 0;
    for (int i = 0; i < SWWATCH_MAX_DEVICES; i++)
        if (slots[i].used)
            count++;

    char title[24];
    snprintf(title, sizeof(title), "Watch x%d", count);
    graphics::drawCommonHeader(display, x, y, title);

    char buf[64];
    int line = 1;
    const int *rows = graphics::getTextPositions(display);
    uint32_t now = millis();

    // One line per watch: id, HR, SpO2, last-seen age / next-TX countdown.
    for (int i = 0; i < SWWATCH_MAX_DEVICES && line <= 6; i++) {
        WatchSlot &w = slots[i];
        if (!w.used)
            continue;
        uint32_t age = (now - w.lastSeenMs) / 1000;
        uint32_t tx = 0; // seconds until this watch's next forward
        if (w.hasSent) {
            uint32_t elapsed = now - w.lastSendMs;
            tx = (elapsed >= FWD_INTERVAL_MS) ? 0 : (FWD_INTERVAL_MS - elapsed) / 1000;
        }
        snprintf(buf, sizeof(buf), "%02x%02x%02x%02x HR%d %lus/%lus", w.id[0], w.id[1], w.id[2], w.id[3], (int)w.hr,
                 (unsigned long)age, (unsigned long)tx);
        display->drawString(x, rows[line++], buf);
    }

#ifdef BLE_HRM_CONNECT
    if (garminConnected && line <= 6) {
        snprintf(buf, sizeof(buf), "G %s", garminMacStr); // full MAC of the locked-onto watch
        display->drawString(x, rows[line++], buf);
        if (line <= 6) {
            uint32_t age = (now - garminLastChangeMs) / 1000; // s since HR last updated
            uint32_t tx = 0;                                  // s until next forward
            if (garminHasSent) {
                uint32_t el = now - garminLastSendMs;
                tx = (el >= FWD_INTERVAL_MS) ? 0 : (FWD_INTERVAL_MS - el) / 1000;
            }
            if (garminHasFresh())
                snprintf(buf, sizeof(buf), "  HR%d %lus/%lus", (int)garminHr, (unsigned long)age, (unsigned long)tx);
            else
                snprintf(buf, sizeof(buf), "  HR-- %lus/%lus", (unsigned long)age, (unsigned long)tx);
            display->drawString(x, rows[line++], buf);
        }
    }
#endif

    graphics::drawCommonFooter(display, x, y);
}
#endif // HAS_SCREEN

#ifdef BLE_HRM_CONNECT
// --- Garmin pairing API (called from the system menu) --------------------
void SwWatchScanModule::startGarminPairing() { garminPairReq = true; }
uint8_t SwWatchScanModule::garminPairedCount() { return garminPairCount(); }
const char *SwWatchScanModule::garminPairedGet(uint8_t i) { return garminPairGet(i); }
void SwWatchScanModule::garminPairedRemove(uint8_t i) { garminPairRemoveAt(i); }
void SwWatchScanModule::garminPairedClearAll() { garminPairClear(); }
uint32_t SwWatchScanModule::garminGetIntervalMs() { return garminIntervalGet(); }
void SwWatchScanModule::garminSetIntervalMs(uint32_t ms) { garminIntervalSet(ms); }
#if HAS_SCREEN
bool SwWatchScanModule::onFrameSelectPress()
{
    graphics::menuHandler::garminMenu(); // open pair/forget menu from the watch frame
    return true;
}
#endif
#endif
#endif
