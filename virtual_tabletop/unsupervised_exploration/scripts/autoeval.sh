#!/bin/bash
# Every minute, evaluate any finished run of a version that lacks
# eval.json (1 at a time), until version_<v>/STOP_AUTOEVAL exists.
# usage: scripts/autoeval.sh version_<v>
VDIR=$1
cd "$(dirname "$0")/.."
while [ ! -f "$VDIR/STOP_AUTOEVAL" ]; do
  for run in "$VDIR"/runs/r*_*; do
    if [ -f "$run/agent.pt" ] && [ ! -f "$run/eval.json" ] && \
       ! pgrep -f "ue.analysis.evaluate $run" > /dev/null; then
      scripts/eval_runs.sh 1 "$run"
    fi
  done
  sleep 60
done
