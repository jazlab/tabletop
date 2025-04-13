#!/usr/bin/env bash

sudo apt update
sudo apt install -y git cmake python3-pip
curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
rm /tmp/get-platformio.py
mkdir -p $HOME/.local/bin
ln -sf $HOME/.platformio/penv/bin/platformio $HOME/.local/bin/platformio
ln -sf $HOME/.platformio/penv/bin/pio $HOME/.local/bin/pio
ln -sf $HOME/.platformio/penv/bin/piodebuggdb $HOME/.local/bin/piodebuggdb
