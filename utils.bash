# Utility functions
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
