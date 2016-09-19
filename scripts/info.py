#!/usr/bin/python

import os
import sys
import yaml
from rospkg import RosPack


#
# This script can be parameterized with a number of keywords that each
# identify a portion of an automation YAML.
#
# Return value of this script in the shell, based on a given keyword:
#
#  Keyword         Return Value
#  -------         ------------
#  meta            Meta Information, such as author, version, website, license
#  changelog       Changelog information on what changed when in this
#                  experiment, and who introduces the change (plus comments)
#  task-variances  Task variances defined for this specific experiment
#


def processKeyword(doc, keyword):
    if keyword in doc:
        print doc[keyword]
    else:
        print []


def parseArguments(argv):
    is_package = False
    filepath = ""
    keyword = ""
    
    if len(argv) > 0:
        if argv[0] == "--package":
            is_package = True
            argv = argv[1:]
        
        if len(argv) > 0:
            filepath = argv[0]
            argv = argv[1:]
        
        if len(argv) > 0:
            keyword = argv[0]
            argv = argv[1:]
    
    return (filepath, keyword)


if __name__=="__main__":
    rp = RosPack()
    (filepath, keyword) = parseArguments(sys.argv[1:])
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--package":
            if len(sys.argv) > 2:
                pkg_path = sys.argv[2]
                tokens = pkg_path.split("/")
                
                path = rp.get_path(tokens[0])
                for token in tokens[1:]:
                    path += "/" + token
        else:
            path = filepath
        
        if os.path.isfile(path):
            try:
                with open(path, 'r') as content_file:
                    content = content_file.read()
                
                doc = yaml.load(content)
                
                if doc:
                    processKeyword(doc, keyword)
            except IOError:
                print "Error: File '" + path + "' not accessible"
                sys.exit(2)
        elif os.path.isdir(path):
            print "Error: '" + path + "' is a directory (expected a file)"
            sys.exit(2)
        else:
            print "Error: Cannot access '" + path + "'"
            sys.exit(2)
