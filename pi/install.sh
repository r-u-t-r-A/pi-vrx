#!/usr/bin/env bash
# Install c5div on Raspberry Pi OS (Bookworm or later, Lite recommended).
# Builds the binary, installs the SMI overlay, sets up config.txt
# (with a backup) and installs the systemd service, started at boot.
#
#   sudo ./pi/install.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo" >&2
    exit 1
fi
HERE=$(cd "$(dirname "$0")/.." && pwd)

echo "== packages"
apt-get update
apt-get install -y build-essential cmake pkg-config libdrm-dev libjpeg62-turbo-dev device-tree-compiler

echo "== build"
cmake -S "$HERE" -B "$HERE/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HERE/build" -j"$(nproc)"
install -m 755 "$HERE/build/c5div" /usr/local/bin/c5div

echo "== device tree overlay"
BOOT=/boot/firmware
[ -d "$BOOT/overlays" ] || BOOT=/boot
dtc -@ -I dts -O dtb -o "$BOOT/overlays/c5div-smi.dtbo" "$HERE/pi/c5div-smi.dts"
"$HERE/pi/config-txt.sh" "$BOOT/config.txt"

echo "== service"
install -m 644 "$HERE/pi/c5div.service" /etc/systemd/system/c5div.service
DEF=/etc/default/c5div
# Options nobody changed (the first release's, or none): use the current ones.
if [ ! -f "$DEF" ] || [ "$(grep '^C5DIV_ARGS=' "$DEF")" = 'C5DIV_ARGS="--source smi --output hdmi,web --web-port 80"' ]; then
    [ -f "$DEF" ] && cp "$DEF" "$DEF.bak"
    install -m 644 "$HERE/pi/c5div.default" "$DEF"
else
    echo "kept your $DEF (current defaults: pi/c5div.default)"
fi
systemctl daemon-reload
systemctl enable c5div
systemctl is-active -q c5div && systemctl restart c5div

cat <<EOF

Installed. Next:
  1. sudo reboot
  2. ls -l /dev/smi                  (must exist; see docs/WIRING.md if not)
  3. sudo systemctl stop c5div; sudo c5div probe      (wiring + signal check, C5s running)
  4. optional Wi-Fi AP:  sudo ./pi/setup-ap.sh MYSSID MYPASSWORD

c5div starts at boot (options in /etc/default/c5div). To run it by hand, or
probe/record, stop the service first: sudo systemctl stop c5div
EOF
