#include "configuration.h"

#if defined(ARCH_NRF52) && defined(BLE_HRM_CONNECT)
#include "MeshService.h"
#include "main.h" // nrf52Bluetooth
#include "mesh/Channels.h"
#include "modules/esp32/BleHrmConnectModule.h"
#include <bluefruit.h>
#if HAS_SCREEN
#include "graphics/Screen.h"
#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#endif

BleHrmConnectModule *bleHrmConnectModule;

#define HRM_CHANNEL_NAME "WatchData"
#define HRM_FORWARD_INTERVAL_MS 30000
#define HRM_STALE_MS 60000

// Standard Heart Rate service/characteristic clients (UUID16 macros from BLEUuid.h).
static BLEClientService hrService(UUID16_SVC_HEART_RATE);                  // 0x180D
static BLEClientCharacteristic hrMeasure(UUID16_CHR_HEART_RATE_MEASUREMENT); // 0x2A37
static bool bleSetupDone = false;

void BleHrmConnectModule::onHrNotify(const uint8_t *data, size_t len)
{
    if (len < 2)
        return;
    uint8_t flags = data[0];
    hr = (flags & 0x01) ? (uint16_t)(data[1] | (data[2] << 8)) : data[1];
    hrReady = true;
    lastSeenMs = millis();
    LOG_DEBUG("BleHrm notify hr=%d", (int)hr);
}

void BleHrmConnectModule::onConnected()
{
    state = ST_CONNECTED;
    lastSeenMs = millis();
    LOG_INFO("BleHrm connected");
}

void BleHrmConnectModule::onDisconnected()
{
    state = ST_IDLE; // Scanner.restartOnDisconnect(true) re-arms the scan for us
    LOG_INFO("BleHrm disconnected -> will rescan");
}

// --- Bluefruit central callbacks (SoftDevice context) --------------------
static void hrm_notify_cb(BLEClientCharacteristic *, uint8_t *data, uint16_t len)
{
    bleHrmConnectModule->onHrNotify(data, (size_t)len);
}

static void scan_cb(ble_gap_evt_adv_report_t *report)
{
    // Scanner is filtered to the HR service UUID, so any report here qualifies.
    Bluefruit.Central.connect(report);
}

static void connect_cb(uint16_t conn_handle)
{
    if (!hrService.discover(conn_handle)) {
        LOG_WARN("BleHrm: no 0x180D, dropping");
        Bluefruit.disconnect(conn_handle);
        return;
    }
    if (!hrMeasure.discover()) {
        LOG_WARN("BleHrm: no 0x2A37, dropping");
        Bluefruit.disconnect(conn_handle);
        return;
    }
    hrMeasure.enableNotify();
    bleHrmConnectModule->onConnected();
}

static void disconnect_cb(uint16_t, uint8_t)
{
    bleHrmConnectModule->onDisconnected();
}

// --- main loop -----------------------------------------------------------
int32_t BleHrmConnectModule::runOnce()
{
    if (!nrf52Bluetooth)
        return 5000;

    if (!bleSetupDone) {
        hrMeasure.setNotifyCallback(hrm_notify_cb);
        hrService.begin();
        hrMeasure.begin();
        Bluefruit.Central.setConnectCallback(connect_cb);
        Bluefruit.Central.setDisconnectCallback(disconnect_cb);
        Bluefruit.Scanner.setRxCallback(scan_cb);
        Bluefruit.Scanner.restartOnDisconnect(true);
        Bluefruit.Scanner.filterUuid(hrService.uuid); // only HR-service peripherals
        Bluefruit.Scanner.setInterval(160, 80);
        Bluefruit.Scanner.useActiveScan(true);
        Bluefruit.Scanner.start(0);
        bleSetupDone = true;
        state = ST_SCANNING;
        LOG_INFO("BleHrm scanning for 0x180D (Bluefruit central)");
    }

    if (state == ST_CONNECTED && millis() - lastSeenMs > HRM_STALE_MS) {
        LOG_WARN("BleHrm: stale, dropping link");
        Bluefruit.disconnect(hrMeasure.connHandle());
        state = ST_IDLE;
    }

#if HAS_SCREEN
    if (!framePosted && state == ST_CONNECTED) {
        UIFrameEvent e;
        e.action = UIFrameEvent::Action::REGENERATE_FRAMESET;
        notifyObservers(&e);
        framePosted = true;
    }
#endif

    if (state == ST_CONNECTED)
        maybeForward();
    return 1000;
}

void BleHrmConnectModule::maybeForward()
{
    uint32_t now = millis();
    if (!hrReady)
        return;
    if (hasSent && (now - lastSendMs) < HRM_FORWARD_INTERVAL_MS)
        return;
    hrReady = false;

    char msg[48];
    int len = snprintf(msg, sizeof(msg), "garmin hr=%d", (int)hr);

    meshtastic_MeshPacket *pkt = allocDataPacket();
    pkt->to = NODENUM_BROADCAST;
    pkt->channel = channels.getByName(HRM_CHANNEL_NAME).index;
    pkt->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    pkt->decoded.payload.size = len;
    memcpy(pkt->decoded.payload.bytes, msg, len);
    service->sendToMesh(pkt);
    lastSendMs = now;
    hasSent = true;
    LOG_INFO("BleHrm -> mesh: %s", msg);
}

#if HAS_SCREEN
bool BleHrmConnectModule::wantUIFrame() { return state == ST_CONNECTED; }

void BleHrmConnectModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state_, int16_t x, int16_t y)
{
    display->clear();
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);
    graphics::drawCommonHeader(display, x, y, "Garmin HR");
    const int *rows = graphics::getTextPositions(display);
    char buf[32];
    snprintf(buf, sizeof(buf), "HR %d bpm", (int)hr);
    display->drawString(x, rows[1], buf);
    graphics::drawCommonFooter(display, x, y);
}
#endif
#endif
