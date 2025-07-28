#!/bin/bash
# This script finds all setup.py files in its subdirectories and runs
# 'pip install -e .' in each of those directories.

set -e # Exit immediately if a command exits with a non-zero status.

# Find all directories that directly contain a setup.py file
# and loop through them.
for setup_file in $(find . -maxdepth 2 -name setup.py); do
    # Get the directory containing the setup.py file
    DIR=$(dirname "${setup_file}")
    
    echo "----------------------------------------------------"
    echo "Installing package in: ${DIR}"
    echo "----------------------------------------------------"
    
    # Use a subshell to change directory, so we don't have to 'cd' back.
    # The '&&' ensures pip only runs if the 'cd' was successful.
    (cd "${DIR}" && python3 -m pip install -e .)
    
    echo "Done!"
    echo ""
done

echo "All drivers have been installed in editable mode."
