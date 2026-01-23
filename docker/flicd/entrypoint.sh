#!/bin/bash

set -eo pipefail

#
# Executes a command, monitors its output in real time, and terminates it
# if the word "Warning" appears on any line. This version is more robust
# and uses a coprocess to explicitly manage the command's lifecycle.
#
# HOW IT WORKS:
# 1. `coproc THE_CMD { ... }`: Starts the command as an asynchronous coprocess.
#    Bash creates a named process `THE_CMD` and provides its PID in the
#    `THE_CMD_PID` variable and its output stream as a file descriptor in
#    `THE_CMD[0]`.
# 2. `while IFS= read -r line <&"${THE_CMD[0]}"`: We read the command's output
#    line by line directly from its output file descriptor. This happens in
#    the main shell, so we don't have subshell scoping problems.
# 3. `printf "%s\n" "$line"`: We print each line to the terminal so the user
#    can see the output in real time.
# 4. `if [[ "$line" == *"Warning"* ]]`: We check each line for the keyword.
# 5. `pkill -P "$cmd_pid"` & `kill "$cmd_pid"`: If a warning is found, we
#    explicitly terminate the command. `pkill -P` is used first to terminate
#    any child processes the command may have spawned, which is a very
#    robust way to clean up. `kill` terminates the main process.
# 6. `wait "$cmd_pid"`: If the loop finishes without finding a warning, it means
#    the command exited on its own. `wait` retrieves its exit code.
#
# USAGE:
#   watch_for_warning <your_command> [your_arguments...]
#
# RETURNS:
#   - 2: If "Warning" was detected and the process was terminated.
#   - The original command's exit code: If the command completes without warnings.
#   - 127: If no command is provided.
#
exit_on_disconnect() {
    if [ "$#" -eq 0 ]; then
        echo "Usage: exit_on_disconnect <command> [args...]" >&2
        return 127
    fi

    local pattern="disconnected"

    # Start the command as a coprocess, redirecting its stderr to stdout
    coproc THE_CMD { "$@" 2>&1; }

    local cmd_pid=$THE_CMD_PID
    local line

    # Read output line-by-line from the coprocess's stdout
    while IFS= read -r line <&"${THE_CMD[0]}"; do
        # Print the line to the screen for real-time viewing
        printf "%s\n" "$line"

        # Check for the trigger word
        if [[ "$line" == *"$pattern"* ]]; then
            echo "---" >&2
            echo "DISCONNECTED: '$pattern' detected." >&2
            echo "Terminating process tree for PID $cmd_pid..." >&2
            # Terminate the process and its children for a clean exit
            pkill -P "$cmd_pid" &>/dev/null
            kill "$cmd_pid" &>/dev/null
            # Return a unique exit code indicating a warning was found
            return 2
        fi
    done

    # If the loop completes, the process finished on its own. Get its exit code.
    wait "$cmd_pid"
    local cmd_exit_code=$?

    if [ $cmd_exit_code -ne 0 ]; then
        echo "---" >&2
        echo "Command finished with an error (exit code $cmd_exit_code)," >&2
        echo "but no '$pattern' was found." >&2
    else
        echo "---" >&2
        echo "Command finished successfully without warnings." >&2
    fi
    return $cmd_exit_code
}

# --wait-for-hci necessary, see musings.md
app_dir=$(dirname $(realpath ${BASH_SOURCE[0]}))
exec $app_dir/flicd \
    --db-file $app_dir/flic.db \
    --my-bdaddr $BD_ADDR \
    --server-addr $SERVER_ADDR \
    --server-port $PORT \
    --hci-dev $HCI_DEV \
    --wait-for-hci
