#this is the simulation module, in which the simulation parameters and the simulation itself is initiated
import simpy
import network
import psutil
import utility as util
import networkx as nx
import simpy
import functools
import random as np
import time
from enum import Enum
import numpy
from scipy.stats import norm

#simpy environment variable
env = simpy.Environment()
#reads the XML configuration file
parameters = util.xmlParser('configurations.xml')


#initiate the simulation parameters
util.createSimulation(env, parameters)


#starts the simulation
print("------------------------------------------------------------SIMULATION STARTED AT {}------------------------------------------------------------".format(env.now))
startMemory = psutil.virtual_memory().percent
util.startSimulation(env, 1)
endMemory = psutil.virtual_memory().percent
print("------------------------------------------------------------SIMULATION ENDED AT {}------------------------------------------------------------".format(env.now))
#print(psutil.virtual_memory())#print the memory consumption for testing

print("\n-------------------------------------------------------------------------------")
print("------------------------------Simulation results:------------------------------")
print("-------------------------------------------------------------------------------")
print("Total of frames generated: {}".format(network.generatedCPRI))
print("Start memory usage was {} and final memory usage was {}".format(startMemory, endMemory))
print("\n")


