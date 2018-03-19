#!/bin/bash

if [ -z $1 ]; then
  echo "USAGE: $0 branch"
  exit 1
fi

BRANCH=$1
EXIT_CODE=0

function echo_gray   { echo -e "\033[1;31m$@\033[0m"; }
function echo_red    { echo -e "\033[1;31m$@\033[0m"; }
function echo_green  { echo -e "\033[1;32m$@\033[0m"; }
function echo_yellow { echo -e "\033[1;33m$@\033[0m"; }
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }
function echo_purple { echo -e "\033[1;35m$@\033[0m"; }
function echo_cyan   { echo -e "\033[1;36m$@\033[0m"; }

function run_test() {
  DISTRO=$1
  OS=$2
  OS_VERSION=$3
  ARCH=$4

  echo ""
  echo_blue "Testing \"$DISTRO $OS $OS_VERSION $ARCH\""

  TEST_NAME=${DISTRO}_${OS}_${OS_VERSION}_${ARCH}

  pushd . > /dev/null
  mkdir $TEST_NAME
  cd $TEST_NAME


  echo "Generating prerelease scripts..."
  generate_prerelease_script.py \
    https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml \
    $DISTRO default $OS $OS_VERSION $ARCH \
    --custom-repo \
    rosflight__custom-1:git:https://github.com/rosflight/rosflight.git:$BRANCH \
    --level 0 \
    --output-dir ./ \
    &> ../${TEST_NAME}-generate.log

  if [ ! -f prerelease.sh ]; then
    echo_yellow "Generating prerelease scripts failed!"
    echo_red "[Failed]"
    EXIT_CODE=2
    popd > /dev/null
    return 2
  fi


  echo "Running prerelease tests..."
  ./prerelease.sh &> ../${TEST_NAME}-test.log

  if [ $? -eq 0 ]; then
    echo_green "[Passed]"
    popd > /dev/null
    return 0
  else
    echo_red "[Failed]"
    EXIT_CODE=1
    popd > /dev/null
    return 1
  fi
}

if [ "$(ls -A)" ]; then
  read -p "Non-empty directory, delete all files and continue? [y/N]: " CONTINUE
  if [ "$CONTINUE" == 'y' ] || [ "$CONTINUE" == 'Y' ]
  then
    rm -rf *
  else
    echo "Aborting."
    exit 0
  fi
fi

echo_purple "Running all tests for the \"$BRANCH\" branch"

run_test kinetic ubuntu xenial i386
run_test kinetic ubuntu xenial amd64
run_test kinetic debian jessie amd64
run_test kinetic debian jessie arm64
run_test kinetic ubuntu xenial armhf
run_test kinetic ubuntu xenial arm64

run_test lunar ubuntu xenial amd64
run_test lunar debian stretch amd64
run_test lunar debian stretch arm64
run_test lunar ubuntu xenial armhf
run_test lunar ubuntu xenial arm64

#run_test melodic ubuntu artful amd64
#run_test melodic ubuntu bionic amd64
#run_test melodic debian stretch amd64
#run_test melodic debian stretch arm64
#run_test melodic ubuntu bionic armhf
#run_test melodic ubuntu bionic arm64

echo ""
echo_cyan "Exit code: $EXIT_CODE"
exit $EXIT_CODE

