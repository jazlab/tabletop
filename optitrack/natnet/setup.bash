cur_dir=$(dirname $(realpath ${BASH_SOURCE[0]}))
lib_dir=$(realpath $cur_dir/lib)
export LD_LIBRARY_PATH="$lib_dir:$LD_LIBRARY_PATH"
