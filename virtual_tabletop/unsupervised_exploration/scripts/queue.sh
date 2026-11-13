#!/bin/bash
# Run training configs in order, keeping at most MAX runs alive at once
# (each run is learner-bound at ~1.1 cores, so 8 runs ~ 9-10 cores).
# Run names: r<round>_<config stem> (round from the configs/round<N>/
# folder); outputs go to the config's version directory.
# usage: scripts/queue.sh MAX version_<v>/configs/round<N>/<cfg>.yaml ...
MAX=$1; shift
cd "$(dirname "$0")/.."
export VIRTUAL_ENV= PYTHONPATH=$PWD
for cfg in "$@"; do
  while [ "$(pgrep -f 'bin/python -m ue.learn.train' | wc -l)" -ge "$MAX" ]; do
    sleep 20
  done
  vdir=$(echo "$cfg" | grep -o '^.*version_[^/]*')
  r=$(basename "$(dirname "$cfg")")
  name="r${r#round}_$(basename "$cfg" .yaml)"
  mkdir -p "$vdir/logs"
  echo "$(date +%H:%M:%S) launch $name"
  nohup uv run --project .. python -m ue.learn.train "$cfg" --name "$name" \
    > "$vdir/logs/$name.log" 2>&1 &
  sleep 30
done
wait
echo "$(date +%H:%M:%S) queue done"
