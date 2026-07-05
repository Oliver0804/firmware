#!/usr/bin/env bash
#
# dump-and-clone-config.sh
#
# 從 SRC_PORT 上一台已配置好的 Meshtastic 節點抓 config，
# 剝除裝置身份欄位，然後 flash + 灌到 DST_PORT 上的目標裝置。
#
# 用法:
#   bin/dump-and-clone-config.sh <BOARD> <SRC_PORT> <DST_PORT> [FIRMWARE_BIN]
#
# 範例:
#   bin/dump-and-clone-config.sh tbeam \
#       /dev/cu.usbmodem112201 \
#       /dev/cu.wchusbserial595D0258371
#
#   # 指定現成 firmware（跳過 build）
#   bin/dump-and-clone-config.sh tbeam SRC DST \
#       release/firmware-tbeam-2.7.15.61f672b5.bin
#
# 會做的事:
#   1. meshtastic --port SRC --export-config > /tmp/<board>-config.yaml
#   2. 剝掉 owner / owner_short / security.privateKey / security.publicKey /
#      position.gpsMode(NOT_PRESENT) → /tmp/<board>-config.stripped.yaml
#   3. 若沒給 FIRMWARE_BIN，跑 bin/build-esp32.sh <board>
#   4. esptool erase_flash + write factory.bin @ 0x0 → DST_PORT
#   5. 等 20s reboot
#   6. meshtastic --port DST --configure /tmp/<board>-config.stripped.yaml
#   7. meshtastic --port DST --seturl <complete_url> — 補寫 channel index >= 4
#      （--configure 的 channel_url 目前只吃前 4 條）
#
# 注意:
#   - 只支援 ESP32 (T-Beam / Heltec-V3 等)，nRF52 板走 UF2 不同流程
#   - factory.bin 內含 bootloader + partitions + app，開機夠用
#     但 littlefs 分區保留白版 → 沒有 on-device web dashboard
#   - 剝除的欄位:
#       owner / owner_short         保留目標裝置自己的名字
#       security.privateKey/publicKey  裝置身份 keypair 絕不共用
#       position.gpsMode: NOT_PRESENT  避免 T-Echo 導出的值把 T-Beam GPS 關掉
#
set -euo pipefail

if [ $# -lt 3 ]; then
    sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//; /^set -euo/d'
    exit 1
fi

BOARD="$1"
SRC_PORT="$2"
DST_PORT="$3"
FIRMWARE_BIN="${4:-}"

FULL_YAML="/tmp/${BOARD}-config.yaml"
STRIPPED_YAML="/tmp/${BOARD}-config.stripped.yaml"

# pio 環境內的 esptool + python 通常最保險
export PATH="${HOME}/.platformio/penv/bin:${PATH}"

command -v meshtastic >/dev/null || { echo "meshtastic CLI 不在 PATH"; exit 1; }
command -v esptool.py >/dev/null || { echo "esptool.py 不在 PATH（試試把 ~/.platformio/penv/bin 加進 PATH）"; exit 1; }

echo "==> [1/6] 從 $SRC_PORT 讀 config + 完整 channel 表"
meshtastic --port "$SRC_PORT" --export-config > "$FULL_YAML"
SRC_INFO="/tmp/${BOARD}-src-info.txt"
meshtastic --port "$SRC_PORT" --info > "$SRC_INFO" 2>/dev/null
wc -l "$FULL_YAML"

echo "==> [2/6] 剝掉 owner / privateKey / publicKey / gpsMode(NOT_PRESENT)"
python3 - "$FULL_YAML" "$STRIPPED_YAML" << 'PY'
import re, sys, pathlib
src = pathlib.Path(sys.argv[1]).read_text()
out = ["# stripped: owner, owner_short, security.privateKey/publicKey, position.gpsMode(NOT_PRESENT)"]
for line in src.splitlines():
    if re.match(r"^owner(_short)?:", line): continue
    if re.match(r"^\s+(privateKey|publicKey):\s*base64:", line): continue
    if re.match(r"^\s+gpsMode:\s*NOT_PRESENT", line): continue
    out.append(line)
pathlib.Path(sys.argv[2]).write_text("\n".join(out) + "\n")
PY

# 提出 complete URL（有的話）供最後 --seturl 用
COMPLETE_URL="$(grep -oE 'Complete URL[^:]*: (https://[^ ]+)' <(meshtastic --port "$SRC_PORT" --info 2>/dev/null) | awk '{print $NF}' | tail -1 || true)"
if [ -z "$COMPLETE_URL" ]; then
    COMPLETE_URL="$(grep -E '^channel_url:' "$FULL_YAML" | awk '{print $2}')"
fi
echo "channel URL: ${COMPLETE_URL:-<none>}"

if [ -z "$FIRMWARE_BIN" ]; then
    echo "==> [3/6] build ${BOARD} firmware"
    bin/build-esp32.sh "$BOARD"
    FIRMWARE_BIN="release/firmware-${BOARD}-$(bin/buildinfo.py long).bin"
else
    echo "==> [3/6] 用現成 firmware: $FIRMWARE_BIN"
fi

if [ ! -f "$FIRMWARE_BIN" ]; then
    echo "找不到 $FIRMWARE_BIN"; exit 1
fi

echo "==> [4/6] erase_flash + write $FIRMWARE_BIN @ 0x0 → $DST_PORT"
esptool.py --chip esp32 --port "$DST_PORT" --baud 460800 erase_flash
esptool.py --chip esp32 --port "$DST_PORT" --baud 460800 \
    write_flash --flash_mode dio --flash_freq 40m --flash_size detect \
    0x0 "$FIRMWARE_BIN"

echo "==> [5/6] 等 20s 讓 $DST_PORT 開機"
sleep 20

echo "==> [6/6] apply config"
meshtastic --port "$DST_PORT" --configure "$STRIPPED_YAML"

if [ -n "$COMPLETE_URL" ]; then
    echo "==> 額外用 --seturl 補寫 channel"
    sleep 5
    meshtastic --port "$DST_PORT" --seturl "$COMPLETE_URL"
fi

# --configure 和 --seturl 目前 (meshtastic-python 2.7.x) 都只寫前 4 條 channel,
# 手動用 --ch-add 補寫 index >= 4 的 channel。從 $SRC_INFO 解析原始表。
echo "==> [7/6] 補寫 channel index >= 4 (--configure / --seturl 的已知 gap)"
python3 - "$SRC_INFO" << 'PY' > /tmp/extra-channels.sh
import json, re, sys, pathlib
info = pathlib.Path(sys.argv[1]).read_text()
in_ch = False
for line in info.splitlines():
    if line.startswith("Channels:"):
        in_ch = True; continue
    if in_ch:
        m = re.match(r"\s*Index (\d+):\s+\S+\s+psk=\S+\s+(\{.*\})\s*$", line)
        if not m:
            if line.strip() == "" or line.startswith("Primary channel URL"):
                break
            continue
        idx = int(m.group(1))
        if idx < 4:
            continue
        try:
            ch = json.loads(m.group(2))
        except json.JSONDecodeError:
            continue
        name = ch.get("name", f"ch{idx}")
        psk_b64 = ch.get("psk", "")
        uplink = ch.get("uplinkEnabled", True)
        downlink = ch.get("downlinkEnabled", True)
        # --ch-add 會把新頻道 append 到下一個空 slot, 然後把 --ch-index 定成該 slot
        print(f'echo "  + adding channel {idx} {name!r}"')
        print(f'meshtastic --port "$1" --ch-add {name!r} 2>&1 | tail -1')
        print('sleep 6')
        print(f'meshtastic --port "$1" --ch-index {idx} --ch-set psk base64:{psk_b64} 2>&1 | tail -1')
        if not uplink:
            print(f'meshtastic --port "$1" --ch-index {idx} --ch-set uplink_enabled false 2>&1 | tail -1')
        if not downlink:
            print(f'meshtastic --port "$1" --ch-index {idx} --ch-set downlink_enabled false 2>&1 | tail -1')
        print('sleep 4')
PY
if [ -s /tmp/extra-channels.sh ]; then
    bash /tmp/extra-channels.sh "$DST_PORT"
else
    echo "  (無 index >= 4 頻道, 跳過)"
fi

echo
echo "==> 完成。目標裝置狀態:"
meshtastic --port "$DST_PORT" --info 2>&1 | grep -E "hwModel|firmwareVersion|region|modemPreset|Primary channel URL" | head -10
