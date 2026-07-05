#!/usr/bin/env bash
#
# flash-tbeam.sh — 一鍵 flash + 灌 config 到目前接著的 T-Beam
#
# 用法:
#   ./flash-tbeam.sh
#
# 需要:
#   - tbeam-config.stripped.yaml               (剝過的 Meshtastic config)
#   - release/firmware-tbeam-*.bin             (factory bin, 不要 -update)
#   - ~/.platformio/penv/bin/{esptool.py, ...} (pio env)
#   - meshtastic CLI 在 PATH
#
# 抓到「唯一一台」CH340 T-Beam (/dev/cu.wchusbserial*) 就開燒, 否則報錯。
#
set -euo pipefail
export PATH="$HOME/.platformio/penv/bin:$PATH"

CONFIG=tbeam-config.stripped.yaml
FIRMWARE="$(ls -t release/firmware-tbeam-*.bin 2>/dev/null | grep -v update | head -1 || true)"
WATCHDATA_PSK="base64:He3VNIzgvm07GNUJYBPCf3B6zjGJUx5yHEbwMq301/c="

# --- 找 T-Beam port ---
shopt -s nullglob
PORTS=(/dev/cu.wchusbserial*)
shopt -u nullglob
if [ ${#PORTS[@]} -eq 0 ]; then
    echo "找不到 T-Beam (無 /dev/cu.wchusbserial*, CH340 沒插好?)"
    exit 1
fi
if [ ${#PORTS[@]} -gt 1 ]; then
    echo "同時抓到多台 T-Beam, 這個腳本只支援一台:"
    printf '  %s\n' "${PORTS[@]}"
    exit 1
fi
DST="${PORTS[0]}"

[ -f "$CONFIG" ] || { echo "找不到 $CONFIG"; exit 1; }
[ -n "$FIRMWARE" ] && [ -f "$FIRMWARE" ] || { echo "找不到 release/firmware-tbeam-*.bin (跑 bin/build-esp32.sh tbeam)"; exit 1; }

echo "port     : $DST"
echo "firmware : $FIRMWARE"
echo "config   : $CONFIG"
echo

echo "=== [1/3] flash ==="
esptool.py --chip esp32 --port "$DST" --baud 460800 erase_flash
esptool.py --chip esp32 --port "$DST" --baud 460800 \
    write_flash --flash_mode dio --flash_freq 40m --flash_size detect \
    0x0 "$FIRMWARE"

echo "=== [2/3] wait 20s boot ==="
sleep 20

echo "=== [3/3] configure ==="
meshtastic --port "$DST" --configure "$CONFIG"
sleep 6
# --configure / --seturl 都只寫前 4 條 channel, WatchData (idx=4) 手動補
meshtastic --port "$DST" --ch-add WatchData
sleep 6
meshtastic --port "$DST" --ch-index 4 --ch-set psk "$WATCHDATA_PSK"
sleep 6

echo
echo "=== done ==="
meshtastic --port "$DST" --get lora 2>&1 | grep -E "modem_preset|region|channel_num"
meshtastic --port "$DST" --info 2>&1 | \
    awk '/^Channels/,/^Primary channel URL/' | \
    grep -oE 'Index [0-9]+[^{]*"name":[[:space:]]*"[^"]*"' | head
