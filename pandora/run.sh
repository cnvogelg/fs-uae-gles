#!/bin/bash
#
# run.sh - pandora runner for FS-UAE
#

# ensure configuration directory is here
if [ ! -d Configurations ]; then
    mkdir Configurations
fi

# copy default config if none is available
if [ ! -e Configurations/Default.fs-uae ]; then
    cp Default.fs-uae.templ Configurations/Default.fs-uae
fi

# run fs-uae
export LD_LIBRARY_PATH=.
./fs-uae --base-dir=$HOME
