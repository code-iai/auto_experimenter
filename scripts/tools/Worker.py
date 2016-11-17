# Copyright (c) 2016, Jan Winkler <jan.winkler.84@gmail.com>
# All rights reserved.

# This file is part of auto_experimenter.

# auto_experimenter is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.

# auto_experimenter is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with auto_experimenter.  If not, see
# <http://www.gnu.org/licenses/>.

# Author: Jan Winkler


import os
import sys
import time
import subprocess
from Queue import Queue, Empty
from threading import Thread, Timer


def enqueueWorkerOutput(source, queue):
    for line in iter(source.readline, b''):
        queue.put(line.rstrip())


class Worker(object):
    def __init__(self, executable):
        self.executable = executable
        self.done = True
        self.lines = []
        self.process = None
    
    def fullName(self):
        return self.executable
    
    def run(self, args = []):
        if self.done:
            self.done = False
            
            strargs = []
            for arg in args:
                strargs.append(str(arg))
            args = strargs
            
            try:
                self.process = subprocess.Popen([self.executable] + args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)
                self.done = False
                self.lines = []
                self.lineQueue = Queue()
                
                self.waitDoneThread = Thread(target=self.waitDone)
                self.waitDoneThread.daemon = True
                self.waitDoneThread.start()
                
                self.enqueueOutputThreadOut = Thread(target=enqueueWorkerOutput, args=(self.process.stdout, self.lineQueue))
                self.enqueueOutputThreadOut.daemon = True
                self.enqueueOutputThreadOut.start()
                
                self.enqueueOutputThreadErr = Thread(target=enqueueWorkerOutput, args=(self.process.stderr, self.lineQueue))
                self.enqueueOutputThreadErr.daemon = True
                self.enqueueOutputThreadErr.start()
                
                while not self.done:
                    try:
                        line = self.lineQueue.get_nowait()
                        
                        self.lines.append(line)
                    except Empty:
                        pass
                
                self.lineQueue.put("no-error")
            except OSError:
                self.done = True
                self.lineQueue.put("fail-popen")
        else:
            print "A process is already running on this worker:", [self.executable] + args
    
    def nextLine(self):
        if len(self.lines) > 0:
            line = self.lines[0]
            self.lines = self.lines[1:]
            
            return line
        else:
            return None
    
    def hasLines(self):
        return len(self.lines) > 0
    
    def waitDone(self):
        self.process.wait()
        self.done = True
    
    def isDone(self):
       return self.done 
    
    def kill(self):
        self.process.terminate()
        print "\rTerminated '" + self.executable + "', wait for it to shut down"
        self.process.wait()
        print "Shutdown complete for " + self.executable
