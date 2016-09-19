#!/bin/bash


SCRIPT_PATH=$1
VARIANCE=$2
N_COUNT=$3


if [ "${SCRIPT_PATH}" != "" ]; then
    if [ "${N_COUNT}" == "" ]; then
	N_COUNT=1
    fi
    
    for i in `seq ${N_COUNT}`; do
	rosrun auto_experimenter load_experiment_description.py ${SCRIPT_PATH}
	roslaunch auto_experimenter auto_experimenter.launch variance:="${VARIANCE}"
    done
else
    echo "Usage: $0 <script-path> [n=1]"
fi
