#! /usr/bin/env bash
set -euo pipefail

# Autodetect XTRX devices with `lspci` filtering on our vendor ID:
XTRX_DEVICE_ADDRS=( $(lspci -d '10ee:7022:' -mm | awk '{ print $1 }') )

if [ "${#XTRX_DEVICE_ADDRS[@]}" -lt 1 ]; then
    echo "ERROR: No XTRX devices found!" >&2
    exit 1
fi

if [ "${EUID}" -ne "0" ]; then
    echo "ERROR: Must run this script as root!" >&2
    exit 1
fi

echo "Found ${#XTRX_DEVICE_ADDRS[@]} XTRX device(s)"

RESET=1

while getopts "r" arg; do
  case $arg in
    r) RESET=0;;
  esac
done

if [ $RESET = 0 ]; then
    for ADDR in "${XTRX_DEVICE_ADDRS[@]}"; do
        echo "Removing'ing 0000:${ADDR}..."
        # Bizarre addressing, to cause a removal of the entire root bus
        (cd "$(realpath /sys/bus/pci/devices/"0000:${ADDR}")"; echo 1 > ../remove) || true
    done
    echo "Rescanning..."
    echo 1 > /sys/bus/pci/rescan
else
    for ADDR in "${XTRX_DEVICE_ADDRS[@]}"; do
        echo "Resetting 0000:${ADDR}..."
        echo "1" > /sys/bus/pci/devices/"0000:${ADDR}"/reset
    done
fi
