#!/usr/bin/env bash

set -e

# Define the results directory relative to the script's location
# Get workspace directory
script_dir=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
source $script_dir/utils.sh

# Parse command line arguments
results_dir=$(get_parent_dir $script_dir 1)/results/eyelink
while [[ $# -gt 0 ]]; do
    case $1 in
        -o|--output)
            if [[ -z "$2" ]]; then
                print_error "Error: --output requires a results directory argument."
                exit 1
            fi
            results_dir="$2"
            shift # past argument
            shift # past value
            ;;
        *)
            print_error "Unknown option: $1"
            print_error "Usage: $0 [-o|--output results_directory]"
            exit 1
            ;;
    esac
done

# Check if the results directory exists
if [ ! -d "$results_dir" ]; then
  print_error "Error: Results directory '$results_dir' not found."
  exit 1
fi

print_status "Checking for EDF files to convert in $results_dir..."

# Loop through all .edf files in the results directory
find "$results_dir" -maxdepth 1 -name '*.edf' -print0 | while IFS= read -r -d $'\0' edf_file; do
  # Construct the corresponding .asc filename
  base_name=$(basename "$edf_file" .edf)
  asc_file="$results_dir/$base_name.asc"

  # Check if the .asc file already exists
  if [ ! -f "$asc_file" ]; then
    print_status "Converting '$edf_file' to '$asc_file'..."
    # Run the edf2asc conversion tool
    # Assumes edf2asc is in the PATH and outputs the .asc file
    # in the same directory as the input .edf file.
    edf2asc "$edf_file" || true
    print_status "Conversion attempt completed for '$edf_file'."
  else
    print_status "Skipping '$edf_file', already converted ('$asc_file' exists)."
  fi
done

print_status "Conversion complete."

# Remove existing latest.asc symlink if it exists
if [ -L "$results_dir/latest.asc" ]; then
    print_status "Removing existing latest.asc symlink..."
    rm "$results_dir/latest.asc"
fi

# Find the latest .asc file (based on filename timestamp, assuming YYYY-MM-DD_HH-MM format)
latest_asc=$(ls -1 "$results_dir"/*.asc 2>/dev/null | sort -r | head -n 1)

# Create a relative symlink named latest.asc pointing to the latest file
target_file=$(basename "$latest_asc")
link_name="$results_dir/latest.asc"

print_status "Creating/updating symlink '$link_name' -> '$target_file'"
# Use -f to overwrite existing symlink, -s for symbolic
# Use relative path for the target within the symlink command's context directory
(cd "$results_dir" && ln -sf "$target_file" latest.asc)

if [ $? -eq 0 ]; then
  print_status "Symlink 'latest.asc' created successfully."
else
  print_error "Error creating symlink 'latest.asc'."
  exit 1
fi

exit 0
