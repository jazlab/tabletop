#!/usr/bin/env bash
# Configure USBFS memory limit for high-bandwidth USB devices (FLIR cameras)
# Usage: usbfs-configure.sh [<usbfs_memory_mb>] (default: 5000)
# Runs on: host (requires sudo and /etc/default/grub.d access)
# Environment: None
# Notes: Sets kernel module parameter immediately (current session) and permanently via GRUB

set -eo pipefail

bin_dir=$(dirname $(realpath ${BASH_SOURCE[0]}))
source $bin_dir/../../bin/utils.sh

if [[ $# -gt 1 ]]; then
    print_error "Usage: $0 [<usbfs_memory_mb>(default 1000)]"
    exit 1
fi

USBFS_MEMORY_MB=${1:-5000}
GRUBD_FILE=/etc/default/grub.d/99-usbfs-memory.cfg
USBFS_MODULE_FILE=/sys/module/usbcore/parameters/usbfs_memory_mb

# Temporarily change USBFS memory limit for current session (takes immediate effect)
sudo sh -c "echo $USBFS_MEMORY_MB > $USBFS_MODULE_FILE"

# Permanently change limit via GRUB kernel command line (persists across reboots)
sudo tee $GRUBD_FILE > /dev/null <<EOF
GRUB_CMDLINE_LINUX_DEFAULT="\$GRUB_CMDLINE_LINUX_DEFAULT usbcore.usbfs_memory_mb=$USBFS_MEMORY_MB"
EOF
sudo update-grub

print_status "Successfully updated USBFS Memory Limit."
print_status "This should work for the current session, but if not, simply reboot and changes will be made permanent"
