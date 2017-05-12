#!/usr/bin/env python

import os
import subprocess
import sys
import time
import shutil
from subprocess import PIPE
sys.path.append(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools"))
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    os.path.dirname(__file__), "..", "..", "..")), "tools"))
from sumolib import checkBinary


netconvertBinary = checkBinary('netconvert')
sumoBinary = checkBinary('sumo')
# build/check network
retcode = subprocess.call(
    #[netconvertBinary, "-c", "data/platoon.netccfg"], stdout=sys.stdout, stderr=sys.stderr)
    [netconvertBinary, "-c", "data/platoon.netccfg"], stdout=PIPE, stderr=PIPE)
try:
    shutil.copy("data/platoon.net.xml", "net.net.xml")
except:
    print "Missing 'platoon.net.xml'"
print ">> Netbuilding closed with status %s" % retcode
sys.stdout.flush()
# run simulation
retcode = subprocess.call(
    #[sumoBinary, "-c", "data/platoon.sumocfg", "--no-step-log"], stdout=sys.stdout, stderr=sys.stderr)
    [sumoBinary, "-c", "data/platoon.sumocfg", "--no-step-log"], stdout=PIPE, stderr=PIPE)
print ">> Simulation closed with status %s" % retcode
sys.stdout.flush()
