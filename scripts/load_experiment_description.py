#!/usr/bin/python

import os
import sys
import rospy
from rospkg import RosPack


#
# Return value of this script in the shell:
#
#  0 : Success
#  1 : Parameter missing (printing usage)
#  2 : File path given not found or not accessible or is not a file
#  3 : ROS parameter server not found when trying to write parameter
#


def parseArguments(argv):
    is_package = False
    filepath = ""
    parameter = "/experiment_description"
    
    if len(argv) > 0:
        if argv[0] == "--package":
            is_package = True
            argv = argv[1:]
        
        if len(argv) > 0:
            filepath = argv[0]
            argv = argv[1:]
        
        if len(argv) > 0:
            parameter = argv[0]
            argv = argv[1:]
    
    return (filepath, parameter)


if __name__=="__main__":
    rp = RosPack()
    (filepath, parameter) = parseArguments(sys.argv[1:])
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--package":
            if len(sys.argv) > 2:
                pkg_path = sys.argv[2]
                tokens = pkg_path.split("/")
                
                path = rp.get_path(tokens[0])
                for token in tokens[1:]:
                    path += "/" + token
        else:
            path = sys.argv[1]
            
            if len(sys.argv) > 2:
                parameter = sys.argv[2]
            else:
                parameter = "/experiment_description"
        
        if os.path.isfile(path):
            try:
                with open(path, 'r') as content_file:
                    content = content_file.read()
                    
                    try:
                        rospy.set_param(parameter, content)
                        sys.exit(0)
                    except IOError:
                        print "Error: No ROS parameter server found"
                        sys.exit(3)
            except IOError:
                print "Error: File '" + path + "' not accessible"
                sys.exit(2)
        elif os.path.isdir(path):
            print "Error: '" + path + "' is a directory (expected a file)"
            sys.exit(2)
        else:
            print "Error: Cannot access '" + path + "'"
            sys.exit(2)
    else:
        me = sys.argv[0].split("/")[-1]
        
        print "Experiment Description Loader (ver 0.2) by Jan Winkler <winkler@cs.uni-bremen.de>"
        print "This script is part of the `auto_experimenter` package (for more information, see https://github.com/code-iai/auto_experimenter)"
        print
        print "Usage: ./" + me + " [--package] <path-to/description.yaml> [ros-parameter]"
        print
        print "Options for specifying the file path:"
        print
        print "  (1) ./" + me + " /absolute/path/to/file.yaml"
        print "  (2) ./" + me + " relative/path/to/file.yaml"
        print "  (3) ./" + me + " --package <ros_package>/relative/path/to/file.yaml"
        print
        print "Specifying the path relative to a ROS package using the `--package` switch resolves the first path component of the following path into the absolute location of that ROS package. You can specify any relative path thereafter."
        print
        print "The optional [ros-parameter] argument allows to change the parameter on the ROS Parameter Server to populate with the contents of the YAML file to load. The default is '/experiment_description'."
        print
        print "In case of problems with this script, send a mail to winkler@cs.uni-bremen.de or open an issue on the GitHub page mentioned above."
        
        sys.exit(1)
