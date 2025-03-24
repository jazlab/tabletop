#!/bin/bash

sudo curl -fsSL -o /etc/udev/rules.d/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
sudo curl -fsSL -o /etc/udev/rules.d/99-platformio-udev.rules https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
