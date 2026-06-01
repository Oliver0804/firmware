#pragma once
#include "configuration.h"

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52)) && (defined(SWWATCH_SCAN) || defined(BLE_HRM_CONNECT))
#include "FSCommon.h"
#include <string.h>

#ifdef BLE_HRM_CONNECT
// Header-only persistent store for paired Garmin (BLE HR) MAC addresses.
// One MAC ("aa:bb:cc:dd:ee:ff") per line in /prefs/garmin.dat. Header-only with
// internal-linkage state: only one platform's SwWatchScanModule TU uses it per
// build, so the per-TU copy is intentional and harmless.

#define GARMIN_MAX_PAIRED 8
#define GARMIN_MAC_LEN 18 // "aa:bb:cc:dd:ee:ff" + NUL
static const char *GARMIN_PAIR_FILE = "/prefs/garmin.dat";

static char garminPaired[GARMIN_MAX_PAIRED][GARMIN_MAC_LEN];
static uint8_t garminPairedCount = 0;

static inline uint8_t garminPairCount() { return garminPairedCount; }
static inline const char *garminPairGet(uint8_t i) { return (i < garminPairedCount) ? garminPaired[i] : ""; }

static inline bool garminPairContains(const char *mac)
{
    for (uint8_t i = 0; i < garminPairedCount; i++)
        if (strcasecmp(garminPaired[i], mac) == 0)
            return true;
    return false;
}

static inline void garminPairSave()
{
#ifdef FSCom
    auto f = FSCom.open(GARMIN_PAIR_FILE, FILE_O_WRITE);
    if (!f)
        return;
    for (uint8_t i = 0; i < garminPairedCount; i++) {
        f.print(garminPaired[i]);
        f.print("\n");
    }
    f.close();
#endif
}

static inline void garminPairLoad()
{
#ifdef FSCom
    garminPairedCount = 0;
    auto f = FSCom.open(GARMIN_PAIR_FILE, FILE_O_READ);
    if (!f)
        return;
    while (f.available() && garminPairedCount < GARMIN_MAX_PAIRED) {
        char line[GARMIN_MAC_LEN];
        size_t n = f.readBytesUntil('\n', line, sizeof(line) - 1);
        line[n] = 0;
        // trim trailing CR/space
        while (n && (line[n - 1] == '\r' || line[n - 1] == ' '))
            line[--n] = 0;
        if (n >= 11) // looks like a MAC
            strncpy(garminPaired[garminPairedCount++], line, GARMIN_MAC_LEN - 1);
    }
    f.close();
#endif
}

// Add a MAC (idempotent). Returns true if added/already present, false if full.
static inline bool garminPairAdd(const char *mac)
{
    if (garminPairContains(mac))
        return true;
    if (garminPairedCount >= GARMIN_MAX_PAIRED)
        return false;
    strncpy(garminPaired[garminPairedCount++], mac, GARMIN_MAC_LEN - 1);
    garminPairSave();
    return true;
}

static inline void garminPairRemoveAt(uint8_t idx)
{
    if (idx >= garminPairedCount)
        return;
    for (uint8_t i = idx; i + 1 < garminPairedCount; i++)
        strncpy(garminPaired[i], garminPaired[i + 1], GARMIN_MAC_LEN - 1);
    garminPairedCount--;
    garminPairSave();
}

static inline void garminPairClear()
{
    garminPairedCount = 0;
    garminPairSave();
}

// --- Forward (text TX) interval, runtime-configurable + persisted -----------
static const char *GARMIN_IV_FILE = "/prefs/garmin_iv.dat";
static uint32_t garminFwdIntervalMs = 30000; // default 30 s

static inline uint32_t garminIntervalGet() { return garminFwdIntervalMs; }

static inline void garminIntervalLoad()
{
#ifdef FSCom
    auto f = FSCom.open(GARMIN_IV_FILE, FILE_O_READ);
    if (!f)
        return;
    char buf[12];
    size_t n = f.readBytes(buf, sizeof(buf) - 1);
    buf[n] = 0;
    f.close();
    uint32_t v = (uint32_t)strtoul(buf, nullptr, 10);
    if (v >= 5000 && v <= 600000)
        garminFwdIntervalMs = v;
#endif
}

static inline void garminIntervalSet(uint32_t ms)
{
    garminFwdIntervalMs = ms;
#ifdef FSCom
    auto f = FSCom.open(GARMIN_IV_FILE, FILE_O_WRITE);
    if (!f)
        return;
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)ms);
    f.print(buf);
    f.close();
#endif
}
#endif // BLE_HRM_CONNECT

#ifdef SWWATCH_SCAN
// --- sw_watch device allow-list: which broadcast devices forward to the mesh ---
// Stored as 16-hex device ids (8 bytes), one per line in /prefs/swwatch.dat.
// Empty list = forward nothing (devices must be enabled from the Watch menu).
#define SWW_MAX_ENABLED 8
#define SWW_ID_LEN 17 // 16 hex + NUL
static const char *SWW_ENABLED_FILE = "/prefs/swwatch.dat";
static char swwEnabled[SWW_MAX_ENABLED][SWW_ID_LEN];
static uint8_t swwEnabledCount = 0;

static inline bool swwEnabledContains(const char *id)
{
    for (uint8_t i = 0; i < swwEnabledCount; i++)
        if (strcasecmp(swwEnabled[i], id) == 0)
            return true;
    return false;
}

static inline void swwEnabledSave()
{
#ifdef FSCom
    auto f = FSCom.open(SWW_ENABLED_FILE, FILE_O_WRITE);
    if (!f)
        return;
    for (uint8_t i = 0; i < swwEnabledCount; i++) {
        f.print(swwEnabled[i]);
        f.print("\n");
    }
    f.close();
#endif
}

static inline void swwEnabledLoad()
{
#ifdef FSCom
    swwEnabledCount = 0;
    auto f = FSCom.open(SWW_ENABLED_FILE, FILE_O_READ);
    if (!f)
        return;
    while (f.available() && swwEnabledCount < SWW_MAX_ENABLED) {
        char line[SWW_ID_LEN];
        size_t n = f.readBytesUntil('\n', line, sizeof(line) - 1);
        line[n] = 0;
        while (n && (line[n - 1] == '\r' || line[n - 1] == ' '))
            line[--n] = 0;
        if (n == 16)
            strncpy(swwEnabled[swwEnabledCount++], line, SWW_ID_LEN - 1);
    }
    f.close();
#endif
}

// Toggle a device id in the allow-list; persists. Returns the new enabled state.
static inline bool swwEnabledToggle(const char *id)
{
    for (uint8_t i = 0; i < swwEnabledCount; i++) {
        if (strcasecmp(swwEnabled[i], id) == 0) { // remove
            for (uint8_t j = i; j + 1 < swwEnabledCount; j++)
                strncpy(swwEnabled[j], swwEnabled[j + 1], SWW_ID_LEN - 1);
            swwEnabledCount--;
            swwEnabledSave();
            return false;
        }
    }
    if (swwEnabledCount < SWW_MAX_ENABLED) { // add
        strncpy(swwEnabled[swwEnabledCount++], id, SWW_ID_LEN - 1);
        swwEnabledSave();
        return true;
    }
    return false;
}
#endif // SWWATCH_SCAN
#endif
