#pragma once
#include "configuration.h"

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52)) && defined(SWWATCH_SCAN)
#include "SinglePortModule.h"

/**
 * Passively scans for sw_watch (Galaxy Watch) connectionless BLE broadcasts and
 * forwards decoded vitals into the mesh.
 *
 * sw_watch advertises manufacturer-specific data under company id 0xFFFF, round-
 * robin over three frame types (CORE / MOTION / ENV). All payload integers are
 * big-endian. We match on company id + type byte + length, decode, and broadcast
 * a compact text summary of the CORE frame (HR, SpO2, GPS) on PRIVATE_APP.
 */
class SwWatchScanModule : public SinglePortModule,
                          private concurrency::OSThread
#if HAS_SCREEN
    ,
                          public Observable<const UIFrameEvent *>
#endif
{
  public:
    SwWatchScanModule()
        : SinglePortModule("swwatch", meshtastic_PortNum_PRIVATE_APP), OSThread("SwWatchScan")
    {
    }

#ifdef BLE_HRM_CONNECT
    // Garmin pairing API, driven from the system menu (graphics/draw/MenuHandler).
    void startGarminPairing();              // begin a scan + on-screen device picker
    uint8_t garminPairedCount();            // number of remembered watches
    const char *garminPairedGet(uint8_t i); // MAC string of paired watch i
    void garminPairedRemove(uint8_t i);     // forget paired watch i
    void garminPairedClearAll();            // forget all
    uint32_t garminGetIntervalMs();         // current text-forward interval
    void garminSetIntervalMs(uint32_t ms);  // set + persist forward interval
#endif

  protected:
    virtual int32_t runOnce() override;

#if HAS_SCREEN
  public:
    virtual bool wantUIFrame() override;
    virtual void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) override;
    virtual Observable<const UIFrameEvent *> *getUIFrameObservable() override { return this; }
#ifdef BLE_HRM_CONNECT
    virtual bool onFrameSelectPress() override; // SELECT on the watch frame -> Garmin menu
#endif
#endif

  private:
    bool scanning = false;
    bool framePosted = false; // have we asked Screen to add our frame yet
};

extern SwWatchScanModule *swWatchScanModule;
#endif
