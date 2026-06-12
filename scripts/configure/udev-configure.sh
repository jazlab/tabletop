#!/usr/bin/env bash
# Install udev rules for Teensy, PlatformIO, and FLIR camera device access
# Usage: udev-configure.sh
# Runs on: host (requires sudo and internet for downloading PlatformIO rules)
# Environment: None
# Notes: Creates device symlinks in /dev/flir/*, sets permissions for hidraw/ttyACM devices

set -eo pipefail

PLATFORMIO_FILENAME=98-platformio.rules
TEENSY_FILNAME=00-teensy.rules
FLIR_FILENAME=40-flir.rules

# PlatformIO rules (disabled in lieu of custom teensy rule below)
sudo curl -fsSL -o /etc/udev/rules.d/$PLATFORMIO_FILENAME https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules

# Teensy microcontroller udev rules (vendor ID 16c0, product ID 04xx; also covers Flic Micro)
sudo tee /etc/udev/rules.d/$TEENSY_FILNAME > /dev/null <<EOF
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04[789a]*", ENV{MTP_NO_PROBE}="1"
KERNEL=="ttyACM*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", MODE:="0666", RUN:="/bin/stty -F /dev/%k raw -echo"
KERNEL=="hidraw*", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="04*", MODE:="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="013*", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1fc9", ATTRS{idProduct}=="013*", MODE:="0666"
EOF

# FLIR camera udev rules (creates /dev/flir/<serial_number> symlinks)
sudo tee /etc/udev/rules.d/$FLIR_FILENAME > /dev/null <<EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", MODE="0666" SYMLINK+="flir/%s{serial}"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1724", MODE="0666" SYMLINK+="flir/%s{serial}"
EOF

# Reload udev rules and trigger re-enumeration of connected devices
sudo udevadm control --reload
sudo udevadm trigger
