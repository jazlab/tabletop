docker container rm tabletop_server tabletop_robot tabletop_novnc tabletop_devcontainer > /dev/null 2>&1
docker network rm tabletop_net > /dev/null 2>&1

if [[ "$1" =~ ^(all|full|system)$ ]]; then
    docker system prune -f
    exit 0
fi

docker container prune -f
docker network prune -f
docker image prune -f
docker volume prune -f