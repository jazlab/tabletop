#!/bin/bash

# This script is a modification of the entrypoint script for the
# unversalrobots/ursim_e-series docker image which removes the
# Xvfb and x11vnc servers, as well as the webserver interface.
# This allows for the use of a single vnc server across all containers
# in the system, consolidating all GUI functionality into a single window.

LOG_OUTPUT="NONE"
if [[ ! $1 == "" ]]; then
        LOG_OUTPUT=$1
fi

# file=/tmp/.X1-lock
# if test -f $file; then
#   rm /tmp/.X1-lock
# fi

# Setup VNC server
# Xvfb :0 -screen 0 1920x1080x24 &
# x11vnc -bg -quiet -forever -shared -display :1 -snapfb >/dev/null 2>/dev/null

# Copy urcaps into bundle, to be installed properly, when the simulator is started
cp -r /urcaps/*.jar /ursim/GUI/bundle/ 2>/dev/null

# # find path to daemon run file
# runsvdir=$(find /etc/service/ -name "runsvdir*")
# run_file="$runsvdir/run"

# # Correct path in run file and make executable
# mkdir -p /home/root/service
# sed -i 's|/ursim/service|/home/root/service|g' $run_file
# chmod +x $run_file

# # Run daemon service
# runsv $runsvdir/ &
# rm -r /ursim/service

# Create webserver interface for vnc
# sed -i 's/$(hostname)/localhost/g' /usr/share/novnc/utils/novnc_proxy
# /usr/share/novnc/utils/novnc_proxy --vnc localhost:5900 >/dev/null 2>/dev/null &

# Get container ip address
docker_ip=$(hostname -i)

# User instructions
echo -e "Universal Robots simulator for e-Series:${VERSION}\n\n"

echo -e "IP address of the simulator\n"
echo -e "     $docker_ip\n\n"

echo -e "NOTE: This is a headless container. No X server is running.\n"
echo -e "     You need to run the novnc docker container on the same network\n"
echo -e "     as this container and set the DISPLAY environment variable of this\n"
echo -e "     container to 'novnc:0.0'.\n\n"

echo -e "Access the robots user interface through the novnc docker container."

echo -e "You can find documentation on how to use this container on dockerhub:\n"
echo -e "     https://hub.docker.com/r/universalrobots/ursim_e-series\n\n"

echo -e "Press Crtl-C to exit\n\n"

polyscope_file=/ursim/polyscope.log
urcontrol_file=/ursim/URControl.log

# Execute URSim
if [ ${LOG_OUTPUT} == "polyscope_log" ]; then
  /ursim/start-ursim.sh ${ROBOT_MODEL} >${polyscope_file} 2>${polyscope_file} &
  tail -f -n10 ${polyscope_file}
elif [ ${LOG_OUTPUT} == "control_log" ]; then
  /ursim/start-ursim.sh ${ROBOT_MODEL} >${polyscope_file} 2>${polyscope_file} &
  while ! tail -f ${urcontrol_file} 2>/dev/null; do sleep 1 ; done
  tail -f -n10 ${urcontrol_file}
else
  /ursim/start-ursim.sh ${ROBOT_MODEL} >${polyscope_file} 2>${polyscope_file}
fi

echo -e "\n"
