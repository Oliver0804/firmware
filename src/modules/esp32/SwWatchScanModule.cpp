#include "configuration.h"

#if defined(ARCH_ESP32) && defined(SWWATCH_SCAN)
#include "MeshService.h"
#include "SwWatchScanModule.h"
#include "main.h" // nimbleBluetooth
#include "mesh/Channels.h"
#include <NimBLEDevice.h>
#include <string.h>
#ifdef BLE_HRM_CONNECT
#include "GarminPairStore.h"
#endif
#if HAS_SCREEN
#include "graphics/Screen.h"
#include "graphics/SharedUIDisplay.h"
#include "graphics/ScreenFonts.h"
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
// Runtime-configurable forward interval when Garmin HR is built in (set via the
// Watch menu, persisted); otherwise the compile-time default applies.
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

// Per-watch state. Written from the BLE host callback, read by runOnce()/drawFrame().
// NimBLE serialises onResult() so writes aren't re-entrant; reads race benignly (display only).
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
// Shares this module's single scanner: the callback below flags an HR-service
// device, runOnce() does the connect/subscribe, and we keep scanning so sw_watch
// broadcasts still arrive while the Garmin link is up.
static NimBLEUUID HRM_SVC((uint16_t)0x180D);
static NimBLEUUID HRM_CHR((uint16_t)0x2A37);
static volatile bool hrmPending = false; // scan saw an HR device, want to connect
static NimBLEAddress hrmAddr;
static char hrmMacStr[18] = {0}; // "aa:bb:cc:dd:ee:ff" of the device we locked onto
static NimBLEClient *hrmClient = nullptr;

// Pairing mode: double-press starts a short scan that collects nearby HR sensors
// into a candidate list; the user then picks one from an on-screen banner.
#define HRM_PAIR_SCAN_MS 6000
#define HRM_MAX_CAND 6
static volatile bool hrmPairing = false;
static volatile bool hrmPairReq = false; // set by input observer, consumed by runOnce
static uint32_t hrmPairStartMs = 0;
static char hrmCand[HRM_MAX_CAND][18];
static volatile uint8_t hrmCandCount = 0;

// Optionally lock onto one specific watch. Define at build time, e.g.
//   -D GARMIN_TARGET_MAC='"d1:22:33:44:55:66"'
// Leave undefined to connect to the first HR sensor seen (current behaviour).
// NOTE: only reliable if the watch advertises a STATIC address; Garmin may use a
// rotating random/resolvable address (RPA), which a fixed MAC filter can't track.
#ifndef GARMIN_TARGET_MAC
#define GARMIN_TARGET_MAC ""
#endif
static volatile bool hrmConnected = false;
static uint16_t hrmHr = 0;
static volatile bool hrmReady = false;
static volatile bool hrmValid = false; // last notify carried a real on-wrist reading
static uint32_t hrmLastSeenMs = 0, hrmLastChangeMs = 0, hrmLastSendMs = 0;
static bool hrmHasSent = false;

// Off-wrist behaviour is watch-dependent: some Garmins STOP notifying, others keep
// re-sending the SAME frozen value with the contact flag still set. Both look stale
// as "value hasn't changed for a while", so we gate freshness on the last value CHANGE.
#define HRM_FRESH_MS 12000

static void hrmNotifyCb(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool)
{
    if (len < 2)
        return;
    uint8_t flags = data[0];
    uint16_t v = (flags & 0x01) ? (uint16_t)(data[1] | (data[2] << 8)) : data[1];
    bool contactSupported = flags & 0x04; // bit2
    bool contactDetected = flags & 0x02;  // bit1
    hrmLastSeenMs = millis();
    // Off-wrist (contact lost) or zero reading -> not a usable HR.
    if ((contactSupported && !contactDetected) || v == 0) {
        hrmValid = false;
        return;
    }
    if (v != hrmHr) // real HR jitters; a changing value means it's live
        hrmLastChangeMs = millis();
    hrmHr = v;
    hrmValid = true;
    hrmReady = true;
    LOG_DEBUG("garmin notify hr=%d", (int)hrmHr);
}

// Fresh only if valid AND the value has actually changed recently (catches both the
// "stopped notifying" and the "frozen-repeat" off-wrist modes).
static inline bool hrmHasFresh() { return hrmValid && (millis() - hrmLastChangeMs) < HRM_FRESH_MS; }

// Clear the connected flag the instant NimBLE reports a dropped link, so the
// always-on scanner can re-detect and reconnect immediately (else we'd wait for
// the 2-min stale timeout). Mirrors nRF52's restartOnDisconnect behaviour.
class SwwHrmClientCb : public NimBLEClientCallbacks
{
    void onDisconnect(NimBLEClient *) override
    {
        hrmConnected = false;
        hrmValid = false;
        LOG_INFO("garmin link dropped, will rescan/reconnect");
    }
};
#endif

class SwWatchAdvCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
    void onResult(NimBLEAdvertisedDevice *dev) override
    {
#ifdef BLE_HRM_CONNECT
        // Note the first HR-service device for runOnce() to connect (don't connect
        // from inside the scan callback — NimBLE forbids re-entrant host calls).
        if (dev->isAdvertisingService(HRM_SVC)) {
            NimBLEAddress a = dev->getAddress();
            std::string as = a.toString();
            const char *mac = as.c_str();
            // Pairing mode: collect unique candidates for the picker, don't connect.
            if (hrmPairing) {
                bool dup = false;
                for (uint8_t i = 0; i < hrmCandCount; i++)
                    if (strcasecmp(hrmCand[i], mac) == 0) { dup = true; break; }
                if (!dup && hrmCandCount < HRM_MAX_CAND) {
                    strncpy(hrmCand[hrmCandCount], mac, sizeof(hrmCand[0]) - 1);
                    hrmCandCount++;
                    LOG_INFO("garmin pair candidate %s type=%d", mac, (int)a.getType());
                }
                return;
            }
            // Normal: only auto-connect to a remembered (paired) watch.
            if (!hrmConnected && !hrmPending && garminPairContains(mac)) {
                hrmAddr = a;
                strncpy(hrmMacStr, mac, sizeof(hrmMacStr) - 1);
                hrmPending = true;
            }
        }
#endif
        uint8_t *pl = dev->getPayload();
        size_t n = dev->getPayloadLength();
        if (!pl || n < 5)
            return;
        size_t i = 0;
        while (i + 1 < n) {
            uint8_t adLen = pl[i];
            if (adLen == 0 || i + 1 + adLen > n)
                break;
            uint8_t adType = pl[i + 1];
            if (adType == 0xFF && adLen >= 4) { // manufacturer specific data
                const uint8_t *md = pl + i + 2; // company id (LE) + payload
                size_t mdLen = adLen - 1;
                if (md[0] == SWW_COMPANY_LO && md[1] == SWW_COMPANY_HI)
                    parseSwwBody(md + 2, mdLen - 2);
            }
            i += adLen + 1;
        }
    }
};

// After a pairing scan, show the found watches; the chosen one is remembered and
// will be auto-connected on the next scan pass. Falls back to auto-add if headless.
static void showPairPicker()
{
    if (hrmCandCount == 0) {
        LOG_INFO("garmin pair: no watches found");
#if HAS_SCREEN
        if (screen)
            screen->showSimpleBanner("No Garmin found", 3000);
#endif
        return;
    }
#if HAS_SCREEN
    static const char *opts[HRM_MAX_CAND + 1];
    static char labels[HRM_MAX_CAND][24];
    static char title[20];
    opts[0] = "Cancel";
    uint8_t n = hrmCandCount;
    for (uint8_t i = 0; i < n; i++) {
        snprintf(labels[i], sizeof(labels[i]), "%s", hrmCand[i]);
        opts[i + 1] = labels[i];
    }
    snprintf(title, sizeof(title), "Pick Garmin (%d)", (int)n);
    graphics::BannerOverlayOptions b;
    b.message = title;
    b.optionsArrayPtr = opts;
    b.optionsCount = n + 1;
    b.durationMs = 30000;
    b.bannerCallback = [](int sel) {
        if (sel >= 1 && sel <= hrmCandCount) {
            garminPairAdd(hrmCand[sel - 1]);
            LOG_INFO("garmin paired %s (%d total)", hrmCand[sel - 1], (int)garminPairCount());
        }
    };
    if (screen)
        screen->showOverlayBanner(b);
#else
    garminPairAdd(hrmCand[0]); // headless: remember the first one found
#endif
}

int32_t SwWatchScanModule::runOnce()
{
    // Don't touch the BLE stack unless it is up (PowerFSM may have torn it down).
    if (!nimbleBluetooth || !nimbleBluetooth->isActive())
        return 5000;

    if (!scanning) {
        NimBLEScan *scan = NimBLEDevice::getScan();
        scan->setAdvertisedDeviceCallbacks(new SwWatchAdvCallbacks(), /*wantDuplicates=*/true);
        scan->setActiveScan(false); // passive: data is in the adv itself, saves power
        scan->setInterval(160);
        scan->setWindow(80);
        scan->start(0, nullptr, false); // continuous, non-blocking
        scanning = true;
        LOG_INFO("sw_watch scanner started (passive)");
#ifdef BLE_HRM_CONNECT
        garminPairLoad(); // restore remembered watches
        garminIntervalLoad();
        if (((const char *)GARMIN_TARGET_MAC)[0])
            garminPairAdd(GARMIN_TARGET_MAC); // optional build-time seed
        LOG_INFO("garmin: %d paired watch(es), tx interval %lus", (int)garminPairCount(),
                 (unsigned long)(garminIntervalGet() / 1000));
#endif
    }

#ifdef BLE_HRM_CONNECT
    // Pairing: double-press requested a scan. Run it for HRM_PAIR_SCAN_MS, then
    // present the candidate list so the user can pick one to remember.
    if (hrmPairReq && !hrmPairing) {
        hrmPairReq = false;
        hrmCandCount = 0;
        hrmPairStartMs = millis();
        hrmPairing = true;
        LOG_INFO("garmin: pairing scan started");
    }
    if (hrmPairing && (millis() - hrmPairStartMs > HRM_PAIR_SCAN_MS)) {
        hrmPairing = false;
        showPairPicker();
    }

    // A scan spotted an HR sensor: connect, subscribe, then resume scanning so
    // sw_watch broadcasts keep flowing alongside the Garmin link.
    if (hrmPending && !hrmConnected) {
        hrmPending = false;
        NimBLEScan *scan = NimBLEDevice::getScan();
        scan->stop(); // NimBLE can't initiate a connection mid-scan
        if (!hrmClient) {
            hrmClient = NimBLEDevice::createClient();
            hrmClient->setClientCallbacks(new SwwHrmClientCb(), false);
        }
        bool ok = false;
        if (hrmClient->connect(hrmAddr)) {
            NimBLERemoteService *svc = hrmClient->getService(HRM_SVC);
            NimBLERemoteCharacteristic *c = svc ? svc->getCharacteristic(HRM_CHR) : nullptr;
            if (c && c->canNotify() && c->subscribe(true, hrmNotifyCb)) {
                hrmConnected = true;
                hrmLastSeenMs = millis();
                ok = true;
                LOG_INFO("garmin connected, subscribed 0x2A37");
            }
        }
        if (!ok) {
            if (hrmClient->isConnected())
                hrmClient->disconnect();
            LOG_WARN("garmin connect failed, keep scanning");
        }
        scan->start(0, nullptr, false); // resume passive scan for sw_watch + retries
    }
    // Drop a stale Garmin link so the scanner can re-pair it.
    if (hrmConnected && (millis() - hrmLastSeenMs > SWWATCH_STALE_MS)) {
        if (hrmClient && hrmClient->isConnected())
            hrmClient->disconnect();
        hrmConnected = false;
        LOG_WARN("garmin stale, dropped");
    }
    // Watchdog: after a dropped link NimBLE may leave the scan stopped. If we're not
    // connected and not scanning, restart it so re-detection/reconnect can happen.
    if (!hrmConnected && !NimBLEDevice::getScan()->isScanning()) {
        NimBLEDevice::getScan()->start(0, nullptr, false);
        LOG_INFO("garmin: scan restarted for reconnect");
    }
#endif

#if HAS_SCREEN
    // Once a watch is heard, ask Screen to (re)build its frameset so our page appears.
    if (!framePosted) {
        bool any = false;
        for (int i = 0; i < SWWATCH_MAX_DEVICES; i++)
            any |= slots[i].used;
#ifdef BLE_HRM_CONNECT
        any |= hrmConnected;
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
    // Forward the Garmin HR on the same channel/throttle as the sw_watch slots.
    if (hrmReady && hrmHasFresh() && (!hrmHasSent || (now - hrmLastSendMs) >= FWD_INTERVAL_MS)) {
        hrmReady = false;
        char msg[64];
        // Include the MAC so multiple Garmins across the mesh stay distinguishable.
        int len = snprintf(msg, sizeof(msg), "garmin %s hr=%d", hrmMacStr, (int)hrmHr);
        meshtastic_MeshPacket *pkt = allocDataPacket();
        pkt->to = NODENUM_BROADCAST;
        pkt->channel = ch;
        pkt->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        pkt->decoded.payload.size = len;
        memcpy(pkt->decoded.payload.bytes, msg, len);
        service->sendToMesh(pkt);
        hrmLastSendMs = now;
        hrmHasSent = true;
        LOG_INFO("garmin -> mesh: %s", msg);
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
    if (hrmConnected)
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
    if (hrmConnected && line <= 6) {
        snprintf(buf, sizeof(buf), "G %s", hrmMacStr); // full MAC of the locked-onto watch
        display->drawString(x, rows[line++], buf);
        if (line <= 6) {
            uint32_t age = (now - hrmLastChangeMs) / 1000; // s since HR last updated
            uint32_t tx = 0;                               // s until next forward
            if (hrmHasSent) {
                uint32_t el = now - hrmLastSendMs;
                tx = (el >= FWD_INTERVAL_MS) ? 0 : (FWD_INTERVAL_MS - el) / 1000;
            }
            if (hrmHasFresh())
                snprintf(buf, sizeof(buf), "  HR%d %lus/%lus", (int)hrmHr, (unsigned long)age, (unsigned long)tx);
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
void SwWatchScanModule::startGarminPairing() { hrmPairReq = true; }
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
