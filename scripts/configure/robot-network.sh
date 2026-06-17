#!/usr/bin/env bash
# Configure reverse SSH tunnel IP address for robot communication
# Usage: robot-network.sh
# Runs on: host (requires sudo)
# Environment: ROBOT_IP, REVERSE_IP (from setup.bash)
# Notes: Validates IP subnet matching, auto-detects ethernet interface, adds reverse IP via sudo

bin_dir=$(dirname $(realpath ${BASH_SOURCE[0]}))
source $bin_dir/../../setup.bash
source $bin_dir/../../bin/utils.sh

# Validate that ROBOT_IP and REVERSE_IP are in the same subnet (first 3 octets)
if [ "${ROBOT_IP%.*}" != "${REVERSE_IP%.*}" ]; then
    print_status "Error: ROBOT_IP and REVERSE_IP must share the same first 3 octets"
    exit 1
fi

# Auto-detect first ethernet interface using 'ip link show'
# Explanation of awk:
#   -F': ' sets field separator to ': '
#   /^[0-9]+: e/ matches lines starting with digit(s), then ': e' (identifies ethernet interfaces)
#   {print $2} extracts interface name
# Then head -n 1 selects the first match
eth_interface=$(ip link show | awk -F': ' '/^[0-9]+: e/{print $2}' | head -n 1)
if [ -n "$eth_interface" ]; then
    print_status "Using ethernet interface: $eth_interface"
else
    print_status "Error: Could not find ethernet interface"
    exit 1
fi

# Add reverse connection IP address to the ethernet interface (/24 subnet)
sudo ip addr add "$REVERSE_IP/24" dev $eth_interface
# sudo ip route add "${REVERSE_IP%.*}.0/24" dev $eth_interface
