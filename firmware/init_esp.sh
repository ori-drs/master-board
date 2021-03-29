#!/bin/bash
# This script initializes the esp-idf to prepare for master-board flashing.
# just type ./init_esp to run the script

source ~/python-environments/env-python3/bin/activate # switch to python3 (has to be prepared before running the script)
cd ~/esp/esp-idf
git checkout v4.0 # The export.sh is only available at newer versions.
source export.sh
git checkout 8d1a9c0 # Need to checkout this old version for now
git submodule update --init --recursive
cd ~/solo12_hardware/master-board/firmware
