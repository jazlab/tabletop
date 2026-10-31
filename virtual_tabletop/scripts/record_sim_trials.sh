#!/usr/bin/env bash
# Record a simulated TableTop session of N trials to a ROS bag.
#
# Usage (from anywhere, on the host, with the sim profile already up):
#   virtual_tabletop/scripts/record_sim_trials.sh N [TASK]
#
#   N      number of trials (the task stops by itself after N)
#   TASK   task config name in tabletop_tasks/config (default foraging_ordered)
#
# Optional: RESPONSE_TIMEOUT=SECONDS overrides the task's response_timeout in
# the copied config. foraging_ordered uses 0.5 s, but the simulated Flic
# button presses after 1-3 s, so by default every sim trial times out with no
# press or reward; e.g. RESPONSE_TIMEOUT=5 gives rewarded trials.
#
# Steps: writes a copy of the task config with `num_trials: N` to the
# git-ignored .cache/tabletop/, starts the rosbag recorder (FLIR camera
# preflight off: there are no cameras in sim), runs the task in mock mode,
# then stops the recorder cleanly. The session lands in bags/latest.
#
# Requires: `tt-compose --profile=sim up -d` and a manipulation preflight
# report (`tt-preflight run`), see gitignore/instructions_from_claude/.

set -eo pipefail

n_trials=${1:?usage: $0 N [TASK]}
task=${2:-foraging_ordered}
[[ $n_trials =~ ^[1-9][0-9]*$ ]] || { echo "N must be a positive integer" >&2; exit 2; }

repo=$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/../.." && pwd)
source "$repo/setup.bash"
cd "$repo"

src="src/ros/tabletop/tabletop_tasks/config/$task.yaml"
[[ -f $src ]] || { echo "No task config $src" >&2; exit 2; }
grep -q '^ *num_trials:' "$src" || { echo "$src has no num_trials" >&2; exit 2; }
if [[ ! -f .cache/tabletop/manipulation_preflight.yaml ]] && grep -q '^manipulation_preflight:' "$src"; then
    echo "No manipulation preflight report: run 'tt-preflight run' first" >&2
    exit 1
fi

name="sim_${task}_${n_trials}"
mkdir -p .cache/tabletop
edits=(-e "s/^( *num_trials:) *[0-9]+/\1 $n_trials/")
if [[ -n $RESPONSE_TIMEOUT ]]; then
    [[ $RESPONSE_TIMEOUT =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo "RESPONSE_TIMEOUT must be a number" >&2; exit 2; }
    grep -q '^ *response_timeout:' "$src" || { echo "$src has no response_timeout" >&2; exit 2; }
    edits+=(-e "s/^( *response_timeout:) *[0-9.]+/\1 $RESPONSE_TIMEOUT/")
    name="${name}_rt${RESPONSE_TIMEOUT}"
fi
sed -E "${edits[@]}" "$src" > ".cache/tabletop/$name.yaml"

recorder=vtt-sim-recorder
log=$(mktemp -t vtt-recorder)
stop_recorder() {
    if docker ps --format '{{.Names}}' | grep -qx "$recorder"; then
        echo "Stopping the recorder (waiting for it to finalize the bag)..."
        docker kill --signal=SIGINT "$recorder" >/dev/null
        while docker ps --format '{{.Names}}' | grep -qx "$recorder"; do
            sleep 1
        done
    fi
    grep -E 'Recording stopped cleanly|Integrity report' "$log" || {
        echo "Recorder did not report a clean stop; see $log" >&2
    }
}
trap stop_recorder EXIT

echo "Starting the recorder..."
tt-compose run --rm --name "$recorder" commander \
    tt-launch rosbag flir_preflight:=false > "$log" 2>&1 &
until grep -q 'Starting recording to' "$log"; do
    if ! kill -0 $! 2>/dev/null; then
        cat "$log" >&2
        echo "Recorder failed to start" >&2
        exit 1
    fi
    sleep 1
done
grep -o "Starting recording to '.*'" "$log"

echo "Running $n_trials trial(s) of $task (mock robots)..."
tt-compose run --rm commander tt-launch tasks \
    "task:=/tabletop/.cache/tabletop/$name" robot_mode:=mock
echo "Task finished."
stop_recorder
trap - EXIT
echo "Session: $(readlink -f bags/latest)"
