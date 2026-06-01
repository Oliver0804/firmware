#include "configuration.h"

#if defined(ARCH_ESP32) && defined(BLE_HRM_CONNECT)
#include "BleHrmConnectModule.h"
#include "MeshService.h"
#include "main.h" // nimbleBluetooth
#include "mesh/Channels.h"
#include <NimBLEDevice.h>
#if HAS_SCREEN
#include "graphics/Screen.h"
#include "graphics/SharedUIDisplay.h"
#include "graphics/ScreenFonts.h"
#endif

BleHrmConnectModule *bleHrmConnectModule;

#define HRM_CHANNEL_NAME "WatchData"          // same private channel as sw_watch
#define HRM_FORWARD_INTERVAL_MS 30000         // throttle forwards (ms)
#define HRM_STALE_MS 60000                    // no notify for this long -> drop & rescan

// Standard BLE Heart Rate GATT UUIDs.
static NimBLEUUID UUID_HR_SERVICE((uint16_t)0x180D);
static NimBLEUUID UUID_HR_MEASUREMENT((uint16_t)0x2A37);

static NimBLEAdvertisedDevice *foundDev = nullptr; // set by scan callback, consumed by runOnce()
static NimBLEClient *client = nullptr;

// Decode Heart Rate Measurement (0x2A37): flags byte, then uint8 or uint16 HR.
void BleHrmConnectModule::onHrNotify(const uint8_t *data, size_t len)
{
    if (len < 2)
        return;
    uint8_t flags = data[0];
    uint16_t v = (flags & 0x01) ? (uint16_t)(data[1] | (data[2] << 8)) : data[1];
    hr = v;
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
    state = ST_IDLE;
    LOG_INFO("BleHrm disconnected -> will rescan");
}

// --- NimBLE callbacks ----------------------------------------------------
class HrmClientCallbacks : public NimBLEClientCallbacks
{
    void onConnect(NimBLEClient *) override { bleHrmConnectModule->onConnected(); }
    void onDisconnect(NimBLEClient *) override { bleHrmConnectModule->onDisconnected(); }
};

class HrmScanCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
    void onResult(NimBLEAdvertisedDevice *dev) override
    {
        if (dev->isAdvertisingService(UUID_HR_SERVICE) && !foundDev) {
            foundDev = dev; // grab first HR sensor; runOnce() does the actual connect
            NimBLEDevice::getScan()->stop();
            LOG_INFO("BleHrm: found HR sensor %s", dev->getAddress().toString().c_str());
        }
    }
};

static void hrNotifyCb(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool)
{
    bleHrmConnectModule->onHrNotify(data, len);
}

// --- main loop -----------------------------------------------------------
int32_t BleHrmConnectModule::runOnce()
{
    if (!nimbleBluetooth || !nimbleBluetooth->isActive())
        return 5000;

    switch (state) {
    case ST_IDLE:
    case ST_SCANNING: {
        if (state == ST_IDLE) {
            NimBLEScan *scan = NimBLEDevice::getScan();
            scan->setAdvertisedDeviceCallbacks(new HrmScanCallbacks(), false);
            scan->setActiveScan(true);
            scan->start(0, nullptr, false); // continuous, non-blocking
            state = ST_SCANNING;
            LOG_INFO("BleHrm scanning for 0x180D");
        }
        if (foundDev) {
            state = ST_CONNECTING; // fall through next tick
        }
        break;
    }
    case ST_CONNECTING: {
        if (!client)
            client = NimBLEDevice::createClient();
        client->setClientCallbacks(new HrmClientCallbacks(), false);
        if (client->connect(foundDev)) {
            NimBLERemoteService *svc = client->getService(UUID_HR_SERVICE);
            NimBLERemoteCharacteristic *ch = svc ? svc->getCharacteristic(UUID_HR_MEASUREMENT) : nullptr;
            if (ch && ch->canNotify() && ch->subscribe(true, hrNotifyCb)) {
                onConnected();
            } else {
                LOG_WARN("BleHrm: no notifiable 0x2A37, disconnecting");
                client->disconnect();
                state = ST_IDLE;
            }
        } else {
            LOG_WARN("BleHrm: connect failed");
            state = ST_IDLE;
        }
        foundDev = nullptr;
        break;
    }
    case ST_CONNECTED: {
        if (millis() - lastSeenMs > HRM_STALE_MS) {
            LOG_WARN("BleHrm: stale, dropping link");
            if (client)
                client->disconnect();
            state = ST_IDLE;
            break;
        }
        maybeForward();
        break;
    }
    }

#if HAS_SCREEN
    if (!framePosted && state == ST_CONNECTED) {
        UIFrameEvent e;
        e.action = UIFrameEvent::Action::REGENERATE_FRAMESET;
        notifyObservers(&e);
        framePosted = true;
    }
#endif
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
