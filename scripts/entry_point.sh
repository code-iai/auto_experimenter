#!/bin/bash

PACKAGE_PATH=$1
COMMAND=$2

case $COMMAND in
    run)
	# Run the experiment here
	PACKAGE_PATH=$3
	rosrun auto_experimenter run.sh "--package ${PACKAGE_PATH}"
	;;
    
    info)
	# Extract information
	SUBCOMMAND=$3
	rosrun auto_experimenter info.py --package ${PACKAGE_PATH} ${SUBCOMMAND}
	;;
    *)
	# Unknown command
	echo $"Usage: $0 <package>/path/to/description.yaml <command> [subcommand]"
	exit 1

esac
