# Functions
get_parent_dir() {
    # Check for exactly 2 arguments
    if [ $# -ne 2 ]; then
        echo "Error: get_parent_dir requires exactly 2 arguments" >&2
        echo "Usage: get_parent_dir <path> <number_of_levels>" >&2
        return 1
    fi
    local path=$1
    local n=$2
    
    # If path is a file, start from its directory
    if [ -f "$path" ]; then
        path=$(dirname $(readlink -f $path))
    fi

    # Get absolute path
    local abs_path=$(readlink -f $path)
    
    # Move up n directories
    for ((i=0; i<n; i++)); do
        abs_path=$(dirname $abs_path)
    done
    
    echo $abs_path
}