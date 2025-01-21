#!/bin/bash

echo "Removing containers..."
docker container rm tabletop_server tabletop_robot tabletop_novnc tabletop_devcontainer > /dev/null 2>&1
echo "Removing network..."
docker network rm tabletop_net > /dev/null 2>&1

if [[ "$1" =~ ^(all|full|system)$ ]]; then
    echo "Pruning system..."
    docker system prune -f
    exit 0
fi

echo "Pruning containers..."
docker container prune -f
echo "Pruning networks..."
docker network prune -f
echo "Pruning images..."
docker image prune -f
echo "Pruning volumes..."
docker volume prune -f
