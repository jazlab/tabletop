#!/bin/bash

systemctl enable cpufrequtils
echo "GOVERNOR=performance" > /etc/default/cpufrequtils
systemctl daemon-reload && systemctl restart cpufrequtils
