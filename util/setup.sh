#!/bin/bash
# A small script to automate installation of dependencies



DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR/../.."

catkin_make || true
sudo apt-get update

# Install MRAA for Joule
sudo add-apt-repository ppa:mraa/mraa
sudo apt-get update
sudo apt-get install -y libmraa1 libmraa-dev mraa-tools python-mraa python3-mraa

sudo rosdep init
rosdep update
rosdep install --from-path src --ignore-src -y
