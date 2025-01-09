docker container rm tabletop_server tabletop_robot tabletop_novnc tabletop_devcontainer > /dev/null 2>&1
docker network rm tabletop_net
docker container prune -f
docker network prune -f
docker image prune -f