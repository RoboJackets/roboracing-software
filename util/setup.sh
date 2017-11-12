#!/bin/bash
# A small script to automate installation of dependencies
# This enables "unofficial Bash strict mode", which Jay recommended for debugging
set -euo pipefail
IFS=$'\n\t'


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR/../.."

echo "WHERE IS THIS RUN?"

catkin_make || true
sudo apt-get update

sudo rosdep init
rosdep update
rosdep install --from-path src --ignore-src -y
