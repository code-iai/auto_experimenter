#!/usr/bin/python

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


import os.path
import time
import signal
import sys
import yaml
import json
import random
import multiprocessing
import subprocess
import rospy
from threading import Thread
from tools.Worker import Worker
from Queue import Queue, Empty


def printLoggedLines(name):
    message(name, "Failure output", "Last 250 lines of output")
    
    for line in logged_lines:
        print line


def logLine(line):
    global logged_lines
    
    logged_lines.append(line)
    length = len(logged_lines)
    max_len = 250
    
    if length > max_len:
        logged_lines = logged_lines[length - max_len:]


def globalKill():
    signalHandler(None, None)


def getTerminalWidth():
    #rows, columns = subprocess.check_output(['stty', 'size']).split()
    columns = 120 # Otherwise, this doesn't work in docker; stty size is knowingly buggy in docker.
    
    return int(columns)


def message(sender, subject, msg, do_newline = True):
    global last_message_did_newline
    max_width_no_newline = getTerminalWidth()# - 15 # Terminal width - ("[Line] Out: " + some safety)
    
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


def addWorker(cmd, args, checklist, quithooks, timeout = None, append_variance = False):
    if append_variance:
        args = args + [json.dumps(global_variance)]
    
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
        echo "Funnily, this checklist is empty (although it shouldn't, as that's not allowed)."
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
            printLoggedLines("Timeout")
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
        
        message(w.fullName(), "Run worker with parameters", str(current_worker[1]) + to_msg), "(checklist length =", str(len(current_cleaner[2])) + ")"
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
    
    append_variance = False
    if "append-variance" in worker:
        if worker["append-variance"] == True:
            append_variance = True
    
    timeout = ""
    if "timeout" in worker:
        timeout = worker["timeout"]
    
    if "checklist" in worker:
        for item in worker["checklist"]:
            addToChecklist(checklist, item["name"], item["matchmode"], item["template"], item["message"])
    
    if "quithooks" in worker:
        for item in worker["quithooks"]:
            addToChecklist(quithooks, item["name"], item["matchmode"], item["template"], item["message"])
    
    if not is_cleaner:
        addWorker(worker["command"], worker["parameters"], checklist, quithooks, timeout, append_variance)
    else:
        addCleaner(worker["command"], worker["parameters"], checklist, quithooks, timeout)


def loadWorkersFromYaml(doc):
    if "workers" in doc:
        for worker in doc["workers"]:
            loadWorker(worker)
    
    if "cleaners" in doc:
        for cleaner in doc["cleaners"]:
            loadWorker(cleaner, True)


def loadVariances(doc, settings):
    yaml_variances = {}
    
    if "task-variances" in doc:
        yaml_variances = doc["task-variances"]
    
    final_variances = {}
    for variance in yaml_variances:
        final_variances[variance] = {}
        final_variances[variance]["value"] = yaml_variances[variance]["default"]
        
        if variance in settings:
            if "value" in settings[variance]:
                final_variances[variance]["value"] = settings[variance]["value"]
        
        if "distribution" in yaml_variances[variance]:
            final_variances[variance]["distribution"] = yaml_variances[variance]["distribution"]
        
        if "value-range" in yaml_variances[variance]:
            final_variances[variance]["value-range"] = yaml_variances[variance]["value-range"]
        
        if variance in settings:
            if "distribution" in settings[variance]:
                final_variances[variance]["distribution"] = settings[variance]["distribution"]
            
            if "value-range" in settings[variance]:
                final_variances[variance]["value-range"] = settings[variance]["value-range"]
        
        final_variances[variance]["value-type"] = yaml_variances[variance]["value-type"]
    
    return final_variances


def instantiateVariance(variances):
    final_variances = {}
    random.seed()
    
    for variance in variances:
        final_variances[variance] = {}
        
        if "range" in variances[variance]["value-type"]:
            val = 0
            range_type = variances[variance]["value-type"][0]
            
            if "value-range" in variances[variance]:
                low = variances[variance]["value-range"][0]
                high = variances[variance]["value-range"][1]
            
            if range_type == "integer":
                val = random.randrange(low, high + 1)
            elif range_type == "percentage":
                val = random.randrange(low * 100, high * 100 + 1) / 100.0
            
            final_variances[variance] = val
        else:
            final_variances[variance] = variances[variance]["value"]
    
    return final_variances


if __name__ == "__main__":
    rospy.init_node("auto_experimenter", anonymous=True)
    
    global workers_schedule
    global cleaners_schedule
    global workers
    global killed
    global last_message_did_newline
    global processes
    global logged_lines
    global global_variance
    
    workers_schedule = []
    cleaners_schedule = []
    workers = []
    killed = False
    last_message_did_newline = True
    processes = []
    logged_lines = []
    global_variance = {}
    
    doc = None
    
    param = rospy.get_param("~experiment_description_parameter", "/experiment_description")
    yaml_contents = rospy.get_param(param)
    doc = yaml.load(yaml_contents)
    
    if doc:
        try:
            param_val = rospy.get_param("~task_variance", "{}")
            variances_settings = json.loads(param_val)
        except ValueError:
            variances_settings = {}
        
        variances = loadVariances(doc, variances_settings)
        global_variance = instantiateVariance(variances)
        
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
