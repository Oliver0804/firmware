#pragma once
#include "configuration.h"

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52)) && defined(BLE_HRM_CONNECT)
#include "SinglePortModule.h"

/**
 * Connects to a standard BLE Heart Rate sensor (Service 0x180D / Measurement
 * 0x2A37) as a GATT central, subscribes to HR notifications, and forwards the
 * value into the mesh.
 *
 * Unlike SwWatchScanModule (passive, connectionless, custom adv format), this
 * targets devices that only expose HR over a connection -- e.g. Garmin
 * Vivoactive with "Broadcast Heart Rate" enabled. Requires the NimBLE CENTRAL
 * role to be compiled in (CONFIG_BT_NIMBLE_ROLE_CENTRAL=y) and coexists with
 * the peripheral (phone) link as a dual-role device.
 */
class BleHrmConnectModule : public SinglePortModule, private concurrency::OSThread
#if HAS_SCREEN
    ,
                            public Observable<const UIFrameEvent *>
#endif
{
  public:
    BleHrmConnectModule() : SinglePortModule("blehrm", meshtastic_PortNum_PRIVATE_APP), OSThread("BleHrm") {}

    // Called from the NimBLE notify callback (host task context).
    void onHrNotify(const uint8_t *data, size_t len);
    // Called from connect/disconnect client callbacks.
    void onConnected();
    void onDisconnected();

  protected:
    virtual int32_t runOnce() override;

#if HAS_SCREEN
  public:
    virtual bool wantUIFrame() override;
    virtual void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) override;
    virtual Observable<const UIFrameEvent *> *getUIFrameObservable() override { return this; }
#endif

  private:
    void maybeForward(); // throttle + push the latest HR into the mesh

    enum State { ST_IDLE, ST_SCANNING, ST_CONNECTING, ST_CONNECTED } state = ST_IDLE;

    uint16_t hr = 0;        // last decoded bpm
    bool hrReady = false;   // a fresh value is pending forward
    uint32_t lastSeenMs = 0;
    uint32_t lastSendMs = 0;
    bool hasSent = false;
    bool framePosted = false;
};

extern BleHrmConnectModule *bleHrmConnectModule;
#endif
