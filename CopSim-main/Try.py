from cmath import pi
import time
import sys
import struct
import os
sys.path.append('remote_api/')

from zmqRemoteApi import RemoteAPIClient
import server_req
import server

import math

import math_op

import time

import fuzzy_logic_python

import map

import os
import time

client = RemoteAPIClient()
sim = client.getObject('sim')

client.setStepping(True)

sim.startSimulation()
objHandles = sim.getCollectionObjects(6)
while (t := sim.getSimulationTime()) < 3:
    s = f'Simulation time: {t:.2f} [s]'
    
    print(s)
    client.step()
sim.stopSimulation()
print(objHandles)
# запуск цикла управления 


