#!/bin/bash

# Change the working directory to the script's directory
SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd $SCRIPTPATH

# Find all files with ".hpp" or ".cpp" extensions in the current directory and subdirectories,
# excluding certain paths (.rosflight_io/include/mavlink/v1.0/, ./rosflight_firmware/firmware/, and ./.git)
find . -iname "*.hpp" -o -iname "*.cpp" | \
egrep -v "^(.rosflight_io/include/mavlink/v1.0/|./rosflight_firmware/firmware/|./.git)" | \

# Format the files according to the rules specified in .clang-format
xargs clang-format -i
