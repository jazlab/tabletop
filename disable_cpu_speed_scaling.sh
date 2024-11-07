sudo systemctl disable ondemand
sudo systemctl enable cpufrequtils
sudo echo "GOVERNOR=performance" > /etc/default/cpufrequtils
sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils