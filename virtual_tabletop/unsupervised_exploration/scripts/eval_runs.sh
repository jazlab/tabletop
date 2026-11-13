#!/bin/bash
# Evaluate finished runs (agent.pt present, no eval.json yet) on held-out
# shapes with videos; at most PAR evaluations at a time.
# usage: scripts/eval_runs.sh PAR version_<v>/runs/<name>...
PAR=$1; shift
cd "$(dirname "$0")/.."
export VIRTUAL_ENV= PYTHONPATH=$PWD
for run in "$@"; do
  [ -f "$run/agent.pt" ] || { echo "skip $run (not finished)"; continue; }
  [ -f "$run/eval.json" ] && { echo "skip $run (done)"; continue; }
  while [ "$(pgrep -f 'bin/python -m ue.analysis.evaluate' | wc -l)" -ge "$PAR" ]; do
    sleep 5
  done
  vdir=$(echo "$run" | grep -o '^.*version_[^/]*')
  name=$(basename "$run")
  echo "$(date +%H:%M:%S) eval $name"
  ( uv run --project .. python -m ue.analysis.evaluate "$run" --trials 120 \
      --video --n-video 9 > "$vdir/logs/eval_$name.log" 2>&1
    uv run --project .. python -m ue.analysis.evaluate "$run" --trials 60 \
      --deterministic >> "$vdir/logs/eval_$name.log" 2>&1 ) &
  sleep 2
done
wait
echo "$(date +%H:%M:%S) evals done"
