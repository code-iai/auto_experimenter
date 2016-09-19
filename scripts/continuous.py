#!/usr/bin/python

#  Software License Agreement (BSD License)
#  
#  Copyright (c) 2016, Institute for Artificial Intelligence,
#  Universitaet Bremen.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

# Author: Jan Winkler


import os.path
import time
import signal
import sys
import yaml
import multiprocessing
import subprocess
import rospy
from threading import Thread
from tools.Worker import Worker
from Queue import Queue, Empty


def printLoggedLines(name):
    message(name, "Failure output", "Last 50 lines of output")
    
    for line in logged_lines:
        print line


def logLine(line):
    global logged_lines
    
    logged_lines.append(line)
    length = len(logged_lines)
    max_len = 50
    
    if length > max_len:
        logged_lines = logged_lines[length - max_len:]


def globalKill():
    signalHandler(None, None)


def getTerminalWidth():
    rows, columns = subprocess.check_output(['stty', 'size']).split()
    
    return int(columns)


def message(sender, subject, msg, do_newline = True):
    global last_message_did_newline
    max_width_no_newline = getTerminalWidth() - 15 # Terminal width - ("[Line] Out: " + some safety)
    
    sys.stdout.write('\r')
    
    for i in range(0, max_width_no_newline):
        sys.stdout.write(" ")
    
    sys.stdout.write('\r')
    
    last_message_did_newline = do_newline
    
    color0 = "\033[0;37m" # paranthesises
    color1 = "\033[0;32m" # sender
    color2 = "\033[1;33m" # subject
    color3 = "\033[1;37m" # message
    
    sender = time.strftime("%H:%M:%S", time.gmtime()) + " " + color1 + sender
    
    fullmsg = color0 + "[" + sender + color0 + "] " + color2 + subject + ": " + color3 + msg
    
    if not do_newline:
        if len(fullmsg) > max_width_no_newline:
            fullmsg = fullmsg[:max_width_no_newline]
    
    sys.stdout.write(fullmsg)
    
    if do_newline:
        sys.stdout.write('\n')
    
    sys.stdout.flush()


def addWorker(cmd, args, checklist, quithooks, timeout = None):
    workers_schedule.append([cmd, args, checklist, quithooks, timeout])


def addCleaner(cmd, args, checklist, quithooks, timeout = None):
    cleaners_schedule.append([cmd, args, checklist, quithooks, timeout])


def run(w, args):
    w.run(args)


def kill_with_children(pid, sig):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    
    for pid_str in ps_output.strip().split("\n")[:-1]:
        os.kill(int(pid_str), sig)
    
    os.kill(pid, sig)


def signalHandlerProxy(signal, frame):
    printLoggedLines("SIGINT")
    
    signalHandler(signal, frame)


def signalHandler(sig, frame):
    global killed
    global workers
    global processes
    
    killed = True
    
    for p in processes:
        p.queue.put("quit")
        p.queue.get()
        p.terminate()
        
        p.join(10)
        if p.is_alive():
            message("Core", "Process", "Process '" + p.name + "' (pid " + str(p.pid) + ") persists after termination, killing it softly.")
            kill_with_children(p.pid, signal.SIGTERM)
        
        p.join(2)
        if p.is_alive():
            message("Core", "Process", "Process '" + p.name + "' (pid " + str(p.pid) + ") persists after soft kill, killing it hard.")
            kill_with_children(p.pid, signal.SIGKILL)
    
    for w in workers:
        w.kill()
    
    workers = []
    processes = []


def addToChecklist(checklist, item, matchmode, template, message):
    checklist[item] = {}
    checklist[item]["matchmode"] = matchmode
    checklist[item]["template"] = template
    checklist[item]["message"] = message
    checklist[item]["matched"] = False


def maintainChecklist(w, checklist, line):
    for item in checklist:
        if not checklist[item]["matched"]:
            match = False
            
            if checklist[item]["matchmode"] == "match":
                if line == checklist[item]["template"]:
                    match = True
            elif checklist[item]["matchmode"] == "contains":
                if checklist[item]["template"] in line:
                    match = True
            
            if match:
                checklist[item]["matched"] = True
                message(w.fullName(), "Checklist", checklist[item]["message"])
                
                return True
    
    if len(checklist) == 0:
        return True
    
    return False


def isChecklistDone(checklist):
    all_match = True
    
    for item in checklist:
        if not checklist[item]["matched"]:
            all_match = False
            break
    
    return all_match


def checkQuitHooks(w, quithooks, line):
    for item in quithooks:
        match = False
        
        if quithooks[item]["matchmode"] == "match":
            if line == quithooks[item]["template"]:
                match = True
        elif quithooks[item]["matchmode"] == "contains":
            if quithooks[item]["template"] in line:
                match = True
        
        if match:
            message(w.fullName(), "Quithooks", quithooks[item]["message"])
            printLoggedLines(w.fullName())
            
            return True
    
    return False


def runWorker(w, args, checklist, quithooks, queue=None):
    global killed
    global workers
    
    thrdRun = Thread(target=run, args=(w, args))
    thrdRun.daemon = True
    
    thrdRun.start()
    time.sleep(1)
    
    while (w.hasLines() or not w.isDone()) and not killed:
        line = w.nextLine()
        
        if line != None:
            message("Line", "Out", line.strip(), False)
            logLine(line)
            
            if maintainChecklist(w, checklist, line):
                if isChecklistDone(checklist):
                    message(w.fullName(), "Run complete", "Moving into background")
                    break
            else:
                if checkQuitHooks(w, quithooks, line):
                    globalKill()
        
        if queue:
            try:
                queued_command = queue.get_nowait()
                
                if queued_command == "quit":
                    w.kill()
                    queue.put("ok")
            except Empty:
                pass


def runWorkerWithTimeout(w, args = [], checklist = {}, quithooks = {}, timeout = None):
    global killed
    global workers
    global processes
    
    if timeout:
        queue = Queue()
        # TODO: Handle return value of runWorker here!
        p = multiprocessing.Process(target=runWorker, args=(w, args, checklist, quithooks, queue))
        p.queue = queue
        p.start()
        
        processes.append(p)
        
        p.join(timeout)
        
        if p.is_alive():
            message(w.fullName(), "Run", "Timeout reached, shutting down.")
            globalKill()
    else:
        workers.append(w)
        runWorker(w, args, checklist, quithooks)


def runNextWorker():
    global workers_schedule
    
    if len(workers_schedule) > 0 and not killed:
        current_worker = workers_schedule[0]
        workers_schedule = workers_schedule[1:]
        
        w = Worker(current_worker[0])
        
        if current_worker[3]:
            to_msg = " and a timeout of " + str(current_worker[4]) + " sec"
        else:
            to_msg = ""
        
        message(w.fullName(), "Run worker with parameters", str(current_worker[1]) + to_msg)
        runWorkerWithTimeout(w, current_worker[1], current_worker[2], current_worker[3], current_worker[4])
        message(w.fullName(), "Run complete", "Advancing pipeline")
        
        return True
    
    return False


def runNextCleaner():
    global cleaners_schedule
    
    if len(cleaners_schedule) > 0:
        current_cleaner = cleaners_schedule[0]
        cleaners_schedule = cleaners_schedule[1:]
        
        w = Worker(current_cleaner[0])
        
        if current_cleaner[3]:
            to_msg = " and a timeout of " + str(current_cleaner[4]) + " sec"
        else:
            to_msg = ""
        
        message(w.fullName(), "Run cleaner with parameters", str(current_cleaner[1]) + to_msg)
        runWorkerWithTimeout(w, current_cleaner[1], current_cleaner[2], current_cleaner[3], current_cleaner[4])
        message(w.fullName(), "Run complete", "Advancing pipeline")
        
        return True
    
    return False


def loadWorker(worker, is_cleaner=False):
    checklist = {}
    quithooks = {}
    
    if "checklist" in worker:
        for item in worker["checklist"]:
            addToChecklist(checklist, item["name"], item["matchmode"], item["template"], item["message"])
    
    if "quithooks" in worker:
        for item in worker["quithooks"]:
            addToChecklist(quithooks, item["name"], item["matchmode"], item["template"], item["message"])
    
    if not is_cleaner:
        addWorker(worker["command"], worker["parameters"], checklist, quithooks, worker["timeout"])
    else:
        addCleaner(worker["command"], worker["parameters"], checklist, quithooks, worker["timeout"])


def loadWorkersFromYaml(doc):
    if "workers" in doc:
        for worker in doc["workers"]:
            loadWorker(worker)
    
    if "cleaners" in doc:
        for cleaner in doc["cleaners"]:
            loadWorker(cleaner, True)


def loadWorkersFromYaml(doc):
    if "workers" in doc:
        for worker in doc["workers"]:
            for worker_wrap in doc:
                if "worker" in worker_wrap:
                    loadWorker(worker)


if __name__ == "__main__":
    rospy.init_node("auto_experimenter", anonymous=True)
    
    global workers_schedule
    global cleaners_schedule
    global workers
    global killed
    global last_message_did_newline
    global processes
    global logged_lines
    
    workers_schedule = []
    cleaners_schedule = []
    workers = []
    killed = False
    last_message_did_newline = True
    processes = []
    logged_lines = []
    
    doc = None
    
    param = rospy.get_param("~experiment_description_parameter", "/experiment_description")
    yaml_contents = rospy.get_param(param)
    doc = yaml.load(yaml_contents)
    
    if doc:
        loadWorkersFromYaml(doc)
        
        signal.signal(signal.SIGINT, signalHandlerProxy)
        
        while runNextWorker() and not killed:
            pass
        
        message("Core", "All tasks completed", "Tearing down workers")
        globalKill()
        
        message("Core", "Cleanup procedure", "Running cleaners")
        while runNextCleaner():
            pass
    else:
        message("Core", "Invalid", "No or no valid yaml configuration found (using parameter '" + param + "')")
