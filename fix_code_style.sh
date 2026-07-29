#!/bin/bash

# Change the working directory to the script's directory
SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd $SCRIPTPATH

# format c/c++ code
find . \( -path "./.git" -o -path "./rosflight_firmware/" \) -prune \
  -o \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) -print \
  | xargs clang-format -i --verbose -style=file
