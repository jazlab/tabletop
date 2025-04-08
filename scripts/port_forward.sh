#!/usr/bin/env bash

# Forward local port 8081 to remote port 8080 on tabletop.valmikikothare.com
ssh -N -L 8081:localhost:8080 valmiki@tabletop.valmikikothare.com &

# NoVNC is then available at http://localhost:8081/vnc.html
