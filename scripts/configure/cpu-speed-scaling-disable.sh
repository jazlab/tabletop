#!/usr/bin/env bash
# Disable CPU frequency scaling for deterministic real-time performance
# Usage: cpu-speed-scaling-disable.sh
# Runs on: host (requires sudo and systemctl)
# Environment: None
# Notes: Installs cpufrequtils, sets CPU governor to 'performance', requires reboot for permanent effect

set -eo pipefail

sudo apt-get install -y cpufrequtils
sudo systemctl disable ondemand &> /dev/null || true
sudo systemctl enable cpufrequtils
if grep -q "^GOVERNOR=" /etc/default/cpufrequtils; then
    sudo sed -i 's/^GOVERNOR=.*/GOVERNOR=performance/' /etc/default/cpufrequtils
else
    echo "GOVERNOR=performance" | sudo tee -a /etc/default/cpufrequtils
fi
sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils
