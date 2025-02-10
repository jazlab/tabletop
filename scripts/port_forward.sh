#! /bin/bash

# Forward local port 8081 to remote port 8080 on tabletop.valmikikothare.com
ssh -N -L 8081:localhost:8080 valmiki@tabletop.valmikikothare.com &
