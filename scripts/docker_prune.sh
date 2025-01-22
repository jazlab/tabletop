#!/bin/bash

# Parse argument
all_arg=""
while [[ $# -gt 0 ]]; do
    case $1 in
        -a)
            echo "Pruning all..."
            all_arg="-a"
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [-a]"
            exit 1
            ;;
    esac
done

containers_to_kill=$(docker container ls -a --format "{{.Names}}" | grep -i tabletop | tr '\n' ' ')
docker container kill $containers_to_kill
docker system prune -f $all_arg --filter "label=com.docker.compose.project=tabletop"
