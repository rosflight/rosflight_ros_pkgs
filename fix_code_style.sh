#!/bin/bash

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
cd $SCRIPTPATH

find . -iname "*.h" -o -iname "*.cpp" | \
egrep -v "^(.rosflight/include/mavlink/v1.0/|./rosflight_firmware/firmware/|./.git)" | \
xargs clang-format -i
