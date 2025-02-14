#!/bin/bash

sudo apt-get install -y cpufrequtils
sudo systemctl enable cpufrequtils
echo "GOVERNOR=performance" | sudo tee -a /etc/default/cpufrequtils
sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils
