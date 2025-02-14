#!/bin/bash

while [[ $# -gt 0 ]]; do
    case $1 in
        --pattern)
            pattern="$2"
            shift
            shift
            ;;
        --new-prefix)
            new_prefix="$2"
            shift
            shift
            ;;
        --dir)
            dir="$2"
            shift
            shift
            ;;
        *)
            echo "Error: Unknown argument $1"
            echo "Usage: $0 [--dir <directory>] [--pattern <pattern>] [--new-prefix <new_prefix>]"
            exit 1
            ;;
    esac
done

if [ -z "$dir" ]; then
    dir="."
fi

if [ -z "$pattern" ]; then
    pattern="*.stl"
fi

if [ -z "$new_prefix" ]; then
    new_prefix="rig_mesh"
fi

pushd "$dir" > /dev/null

i=1
for f in $(find . -maxdepth 1 -name "$pattern" -print); do
    if [ -f "$f" ]; then
        new_name="${new_prefix}${i}.${f##*.}"
        mv "$f" "$new_name"
        ((i++))
    fi
done

popd > /dev/null

# i=1; for f in *.stl; do mv "$f" "rig_mesh$i.stl"; ((i++)); done