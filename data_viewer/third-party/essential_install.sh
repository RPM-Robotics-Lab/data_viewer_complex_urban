#!/usr/bin/env bash
PACKAGES=""
function addpkg {
    PACKAGES="$PACKAGES $@"
}

#sudo apt-get install bc python-software-properties

addpkg\
    build-essential\
    git\
    libboost-dev\
    libboost-all-dev\
    cmake\
    cmake-curses-gui\
    ros-kinetic-serial\
    python-wstool\
    doxygen\
    libpcap-dev\

# go forth!
echo "apt-get install $PACKAGES"
sudo apt-get update
sudo apt-get install $PACKAGES
