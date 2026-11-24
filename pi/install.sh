#!/usr/bin/env bash
# Install c5div on Raspberry Pi OS (Bookworm or later, Lite recommended).
# Builds the binary, installs the SMI overlay, sets up config.txt
# (with a backup) and installs the systemd service (not enabled).
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
[ -f /etc/default/c5div ] || install -m 644 "$HERE/pi/c5div.default" /etc/default/c5div
systemctl daemon-reload

cat <<EOF

Installed. Next:
  1. sudo reboot
  2. ls -l /dev/smi                  (must exist; see docs/WIRING.md if not)
  3. sudo c5div probe                (wiring + signal check, C5s running)
  4. sudo c5div run -o jumpers       (HDMI or AV, web viewer: by jumper)
  5. optional Wi-Fi AP:  sudo ./pi/setup-ap.sh MYSSID MYPASSWORD
  6. optional autostart: sudo systemctl enable --now c5div   (options in /etc/default/c5div)
EOF
