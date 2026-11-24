#!/usr/bin/env bash
# Turn the Pi's Wi-Fi into an access point (NetworkManager, Raspberry Pi OS
# Bookworm+). Phones/laptops join it and open http://192.168.50.1/ (or :8080).
#
#   sudo ./pi/setup-ap.sh SSID PASSWORD [CHANNEL] [--yes]
#
# The Zero 2W / Pi 3B radio is 2.4 GHz only, which keeps it well away from
# the 5.8 GHz video band. Wi-Fi power saving is disabled for lower latency.
# Undo with: sudo nmcli con delete c5div-ap
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo" >&2
    exit 1
fi
SSID=${1:-}
PSK=${2:-}
CH=${3:-6}
YES=0
for a in "$@"; do [ "$a" = "--yes" ] && YES=1; done
[ "$CH" = "--yes" ] && CH=6

if [ -z "$SSID" ] || [ -z "$PSK" ]; then
    echo "usage: sudo $0 SSID PASSWORD [CHANNEL] [--yes]" >&2
    exit 2
fi
if [ "${#PSK}" -lt 8 ]; then
    echo "password must be at least 8 characters" >&2
    exit 2
fi
if ! command -v nmcli >/dev/null; then
    echo "NetworkManager (nmcli) not found" >&2
    exit 1
fi

if [ "$YES" -ne 1 ]; then
    echo "This turns wlan0 into an access point '$SSID' on channel $CH."
    echo "If you are connected to this Pi over Wi-Fi, that connection will drop."
    read -r -p "Continue? [y/N] " ans
    case "$ans" in y|Y|yes) ;; *) echo "aborted"; exit 1 ;; esac
fi

nmcli con delete c5div-ap >/dev/null 2>&1 || true
nmcli con add type wifi ifname wlan0 con-name c5div-ap autoconnect yes ssid "$SSID" \
    802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel "$CH" \
    802-11-wireless.powersave 2 \
    ipv4.method shared ipv4.addresses 192.168.50.1/24 ipv6.method disabled \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK"
nmcli con up c5div-ap

echo
echo "Access point '$SSID' is up. Join it and open http://192.168.50.1:8080/"
echo "(or http://192.168.50.1/ when the service runs with --web-port 80)"
