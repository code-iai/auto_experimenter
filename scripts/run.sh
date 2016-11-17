#!/bin/bash


SCRIPT_PATH=$1
VARIANCE=$2
N_COUNT=$3


if [ "${SCRIPT_PATH}" != "" ]; then
    if [ "${N_COUNT}" == "" ]; then
	N_COUNT=1
    fi
    
    echo "Running ${N_COUNT} experiments"
    
    for i in `seq ${N_COUNT}`; do
	echo "Starting auxiliary roscore"
	roscore &
	CORE_PID=$!
	sleep 2
	
	echo "Parameterizing and starting experiment"
	rosrun auto_experimenter load_experiment_description.py ${SCRIPT_PATH}
	roslaunch auto_experimenter auto_experimenter.launch variance:="${VARIANCE}"
	
	echo "Killing auxiliary roscore"
	kill -s SIGKILL $CORE_PID
    done
else
    echo "Usage: $0 <script-path> [n=1]"
fi
