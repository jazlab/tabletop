# Utility functions for environment, configuration, and output formatting
# Not a standalone script; provides helper functions used by other scripts
# Helper functions: dotenv_set_if_empty, get_parent_dir, print_status, print_error, print_warning

dotenv_set_if_empty() {
    # Check if the correct number of arguments are provided
    if [ $# -ne 2 ]; then
        print_error "Usage: set_if_empty KEY VALUE"
        return 1
    fi

    local key=$1
    local value=$2

    if ! dotenv get $key >/dev/null 2>&1; then
        dotenv set $key $value
    fi
}

get_parent_dir() {
    # Check if the correct number of arguments are provided
    if [ $# -ne 2 ]; then
        print_error "Usage: get_parent_dir <path> <n>"
        exit 1
    fi
    local path=$1
    local n=$2

    # If path is a file, start from its directory
    if [ -f "$path" ]; then
        path=$(dirname $path)
    fi

    # Move up n directories
    for ((i=0; i<n; i++)); do
        path=$(dirname $path)
    done

    echo $path
}

print_status() {
    echo -e "\033[1;34m$@\033[0m"
}

print_error() {
    echo -e "\033[1;31m$@\033[0m"
}

print_warning() {
    echo -e "\033[1;33m$@\033[0m"
}
