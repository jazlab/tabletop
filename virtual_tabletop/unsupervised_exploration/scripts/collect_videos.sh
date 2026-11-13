#!/bin/bash
# Copy every evaluation video of a version into its report/videos/, and
# render large showcase videos (8 held-out trials, 360x480 tiles, 2 per
# row) for the given runs (PAR at a time).
# usage: scripts/collect_videos.sh version_<v> PAR [run_dir...]
VDIR=$1; PAR=$2; shift 2
cd "$(dirname "$0")/.."
export VIRTUAL_ENV= PYTHONPATH=$PWD
mkdir -p "$VDIR/report/videos"
for v in "$VDIR"/runs/*/video.mp4; do
  n=$(basename "$(dirname "$v")")
  cp "$v" "$VDIR/report/videos/video_${n}.mp4"
done
for run in "$@"; do
  while [ "$(pgrep -f 'bin/python -m ue.analysis.evaluate' | wc -l)" -ge "$PAR" ]; do
    sleep 3
  done
  n=$(basename "$run")
  ( uv run --project .. python -m ue.analysis.evaluate "$run" --trials 8 \
      --video --n-video 8 --size 360 480 --cols 2 --tag _showcase \
      > /dev/null 2>&1
    cp "$run/video_showcase.mp4" "$VDIR/report/videos/showcase_${n}.mp4" ) &
  sleep 1
done
wait
ls "$VDIR/report/videos" | wc -l
