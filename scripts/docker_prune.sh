#!/usr/bin/env bash

# Parse argument
_all_arg=""
while [[ $# -gt 0 ]]; do
    case $1 in
        -a)
            echo "Pruning all..."
            _all_arg="-a"
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [-a]"
            exit 1
            ;;
    esac
done

docker system prune -f $_all_arg --filter "label=com.docker.compose.project=tabletop"
