#!/usr/bin/env bash

sudo apt-get install -y cpufrequtils
sudo systemctl enable cpufrequtils
if grep -q "^GOVERNOR=" /etc/default/cpufrequtils; then
    sudo sed -i 's/^GOVERNOR=.*/GOVERNOR=performance/' /etc/default/cpufrequtils
else
    echo "GOVERNOR=performance" | sudo tee -a /etc/default/cpufrequtils
fi
sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils
