#!/usr/bin/env bash
# Put the c5div settings into a Raspberry Pi config.txt (install.sh calls this).
# Safe to run again: replaces its own section, leaves the rest alone, and
# keeps a one-time backup.
#
#   sudo ./pi/config-txt.sh [/boot/firmware/config.txt]
set -euo pipefail
CFG=${1:-/boot/firmware/config.txt}
[ -f "$CFG" ] || { echo "$CFG not found" >&2; exit 1; }
[ -f "$CFG.c5div.bak" ] || cp "$CFG" "$CFG.c5div.bak"

TMP=$(mktemp)
trap 'rm -f "$TMP"' EXIT
# Drop our old section, and enable the composite connector next to HDMI
# (the plain overlay line only; other variants and comments stay).
sed -e '/^# c5div begin/,/^# c5div end/d' \
    -e '/^dtoverlay=vc4-kms-v3d\(,[^#]*\)\?$/{/composite/!s/$/,composite/}' "$CFG" > "$TMP"
grep -q '^dtoverlay=vc4-kms-v3d\(,.*\)\?$' "$TMP" || echo 'dtoverlay=vc4-kms-v3d,composite' > "$TMP.add"
{
    echo "# c5div begin"
    echo "[all]"
    [ -f "$TMP.add" ] && cat "$TMP.add"
    for l in dtoverlay=c5div-smi dtoverlay=smi-dev dtparam=spi=off dtparam=i2s=off; do
        grep -qxF "$l" "$TMP" || echo "$l"
    done
    cat <<'EOT'
# output jumpers: GPIO4 (pin 7-9) fitted = AV instead of HDMI,
# GPIO5 (pin 29-30) fitted = Wi-Fi viewer
gpio=4,5=ip,pu
[gpio4=0]
enable_tvout=1
[all]
# c5div end
EOT
} >> "$TMP"
rm -f "$TMP.add"
cat "$TMP" > "$CFG"
echo "updated $CFG (first backup: $CFG.c5div.bak)"
