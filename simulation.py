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

#print the neighbors of each base stations
#util.printNeighbors(network.elements)

#starts the simulation
print("------------------------------------------------------------SIMULATION STARTED AT {}------------------------------------------------------------".format(env.now))
startMemory = psutil.virtual_memory().percent
util.startSimulation(env, 1500)
endMemory = psutil.virtual_memory().percent
print("------------------------------------------------------------SIMULATION ENDED AT {}------------------------------------------------------------".format(env.now))
#print(psutil.virtual_memory())#print the memory consumption for testing
print("Start memory usage was {} and final memory usage was {}".format(startMemory, endMemory))
print("Total of CPRI basic frames: {}".format(network.generatedCPRI))

'''
#Tests
#print the graph
#print([i for i in nx.edges(G)])
print(G.edges())
#print(G["RRH:0"]["Switch:0"]["weight"])
#print(G.graph)
#for i in nx.edges(G):
#	print("{} --> {} Weight: {}".format(i[0], i[1], G[i[0]][i[1]]["weight"]))

#calling Dijkstra to calculate the shortest path. Returning variables "length" and "path" are the total cost of the path and the path itself, respectively
#length, path = nx.single_source_dijkstra(G, "RRH:0", "Cloud:0")
#print(path)

#for i in range(len(rrhs)):
#  print(g["s"]["RRH{}".format(i)]["capacity"])


print("-----------------Input Parameters-------------------")
for i in inputParameters:
	print("{}: {}".format(i.tag, i.text))

print("-----------------RRHs-------------------")
for i in rrhsParameters:
	print(i)

print("-----------------Network Nodes-------------------")
for i in netNodesParameters:
	print(i)

print("-----------------Processing Nodes-------------------")
for i in procNodesParameters:
	print(i)

print("-----------------Edges-------------------")
for i in networkEdges:
	print(i)
'''