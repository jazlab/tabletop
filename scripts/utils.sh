# Functions
get_parent_dir() {
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