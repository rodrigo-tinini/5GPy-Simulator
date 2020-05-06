#this is the utility module, where every utility method must be put, including methods that calculates metrics from the simulation
import xml.etree.ElementTree as ET
import networkx as nx
import random as np
import network

#XML parser
def xmlParser(xmlFile):
	#keep the configuration parameters
	parameters = {}
	#construct the XML tree
	tree = ET.parse(xmlFile)
	#get the root
	root = tree.getroot()
	#return root
	##iterate over the nodes and  store each one into the parameters dictionaire
	for child in root:
		parameters[child.tag] = child
	return parameters

#call Dijkstra shortest path algorithm
def dijkstraShortestPath(G, source, destiny):
	length, path = nx.single_source_dijkstra(G, source, destiny)
	return length, path

#create the limits of each base station/RRH following a cartesian plane - obsolete, not being used
def createNetworkLimits(limitX, limitY, stepX, stepY, elements):
	#to place each base station in a dictionaire position
	i = 0
	#until the limit of axis x, go upside until the limite of axis y
	x = 0
	while x < limitX:
		y = 0
		while y < limitY:
			elements["RRH:{}".format(i)].x1 = x
			elements["RRH:{}".format(i)].y1 = y
			elements["RRH:{}".format(i)].x2 = x + 1
			elements["RRH:{}".format(i)].y2 = y + 1
			y += stepY
			i += 1
		x += stepX

#print the coordinates of each base station
def printBaseStationCoordinates(baseStations, elements):
	for r in baseStations:
		print("{} coordinates are: \n X1: {}\n Y1: {}\n X2: {}\n Y2: {}\n".format(elements["RRH:{}".format(r["aId"])].aId,
			elements["RRH:{}".format(r["aId"])].x1, elements["RRH:{}".format(r["aId"])].y1, elements["RRH:{}".format(r["aId"])].x2, elements["RRH:{}".format(r["aId"])].y2))

#start the simulation
def startSimulation(env, limit):
	env.run(until = limit)

#initiate simulation parameters
def createSimulation(env, parameters):
	#initiate input parameters from the entries on the XML file
	switchTime = float(parameters["InputParameters"].find("switchTime").text)
	frameProcTime = float(parameters["InputParameters"].find("frameProcTime").text)
	transmissionTime = float(parameters["InputParameters"].find("transmissionTime").text)
	localTransmissionTime = float(parameters["InputParameters"].find("localTransmissionTime").text)
	cpriFrameGenerationTime = float(parameters["InputParameters"].find("cpriFrameGenerationTime").text)
	distributionAverage = float(parameters["InputParameters"].find("distributionAverage").text)
	cpriMode = parameters["InputParameters"].find("cpriMode").text
	eCpriEncap = parameters["InputParameters"].find("eCpriEncap").text
	distribution = lambda x: np.expovariate(1000)
	limitAxisY = int(parameters["InputParameters"].find("limitAxisY").text)#limit of axis Y of the network topology on a cartesian plane
	limitAxisX = int(parameters["InputParameters"].find("limitAxisX").text)#limit of axis X of the network topology on a cartesian plane
	stepAxisY = int(parameters["InputParameters"].find("stepAxisY").text)#increasing step on axis Y when defining the size of the base station
	stepAxisX = int(parameters["InputParameters"].find("stepAxisX").text)#increasing step on axis X when defining the size of the base station
	CoordinateX1 = int(parameters["InputParameters"].find("CoordinateX1").text)
	CoordinateX2 = int(parameters["InputParameters"].find("CoordinateX2").text)
	CoordinateY1 = int(parameters["InputParameters"].find("CoordinateY1").text)
	CoordinateY2 = int(parameters["InputParameters"].find("CoordinateY2").text)
	signalStrength = (parameters["InputParameters"].find("signalStrength").text)
	network.wavelengthsAmount = int(parameters["InputParameters"].find("wavelengthsAmount").text)
	network.chosenAlgorithm = (parameters["InputParameters"].find("Algorithm").text)

	#keep the input parameters for visualization or control purposes
	inputParameters = []
	for p in parameters["InputParameters"]:
		inputParameters.append(p)

	#get the attributes of each RRH
	rrhsParameters = []
	for r in parameters["RRHs"]:
		rrhsParameters.append(r.attrib)

	#get the attributes of each node to be created
	netNodesParameters = []
	for node in parameters["NetworkNodes"]:
		netNodesParameters.append(node.attrib)

	#get the attributes of each processing node to be created
	procNodesParameters = []
	for proc in parameters["ProcessingNodes"]:
		procNodesParameters.append(proc.attrib)

	controlPlaneParameters = []
	#get the attributes of each control plane node to be created
	for cp in parameters["ControlPlane"]:
		controlPlaneParameters.append(cp.attrib)

	#get the edges for the graph representation
	networkEdges = []
	for e in parameters["Edges"]:
		networkEdges.append(e.attrib)

	#save the id of each element to create the graph
	vertex = []
	#RRHs
	for r in rrhsParameters:
		vertex.append("RRH:"+str(r["aId"]))
	#Network nodes
	for node in netNodesParameters:
		vertex.append(node["aType"]+":"+str(node["aId"]))
	#Processing nodes
	for proc in procNodesParameters:
		vertex.append(proc["aType"]+":"+str(proc["aId"]))
	#Control plane node(s)
	for cp in controlPlaneParameters:
		vertex.append(cp["aType"]+":"+str(cp["aId"]))

	#create the graph
	G = nx.Graph()
	#add the nodes to the graph
	for u in vertex:
		G.add_node(u)
	#add the edges and weights to the graph
	for edge in networkEdges:
		G.add_edge(edge["source"], edge["destiny"], weight= float(edge["weight"]))

	# create the control plane node(s)
	for cp in controlPlaneParameters:
		cp_node = network.ControlPlane(env, cp["aId"], cp["aType"], G)
		network.elements[cp_node.aId] = cp_node

	#create the elements
	#create the RRHs
	if cpriMode == "CPRI":
		for r in rrhsParameters:
			fog_node = None
			if r["fogNode"] != "None":
				fog_node = r["fogNode"]
			rrh = network.RRH(env, r["aId"], distribution, cpriFrameGenerationTime, transmissionTime, localTransmissionTime, cpriMode, CoordinateX1,
							  CoordinateX2, CoordinateY1, CoordinateY2, signalStrength, network.elements["ControlPlane:0"], fog_node)
			network.elements[rrh.aId] = rrh
	elif cpriMode == "eCPRI":
		for r in rrhsParameters:
			fog_node = None
			if r["fogNode"] != "None":
				fog_node = r["fogNode"]
			rrh = network.eRRH(env, r["aId"], distribution, cpriFrameGenerationTime, transmissionTime, localTransmissionTime,
							   cpriMode, eCpriEncap, CoordinateX1, CoordinateX2, CoordinateY1, CoordinateY2, signalStrength,
							   network.elements["ControlPlane:0"], fog_node)
			network.elements[rrh.aId] = rrh

	#create the network nodes
	for node in netNodesParameters:
		net_node = network.NetworkNode(env, node["aId"], node["aType"], float(node["capacity"]), node["qos"], switchTime, transmissionTime, G)
		network.elements[net_node.aId] = net_node
		if net_node.aType != "Cloud":
			network.fogNodes.append(net_node)

	#create the processing nodes
	for proc in procNodesParameters:
		proc_node = network.ProcessingNode(env, proc["aId"], proc["aType"], float(proc["capacity"]), proc["qos"], frameProcTime, transmissionTime, G)
		network.elements[proc_node.aId] = proc_node

	#create the neighbors of each RRH
	createNeighbors(parameters, network.elements)

#create the neighbors of each base station
def createNeighbors(parameters, elements):
	neighbors = []
	for n in parameters["Neighbors"]:
		neighbors.append(n.attrib)
	#get the neighborhood of each base station
	for n in neighbors:
		if n["RightRRH"] != None:
			elements[n["src"]].adjacencies["RightRRH"] = n["RightRRH"]
		else: 
			elements[n["src"]].adjacencies["RightRRH"] = None 
		if n["LeftRRH"] != None:
			elements[n["src"]].adjacencies["LeftRRH"] = n["LeftRRH"]
		else:
			elements[n["src"]].adjacencies["LeftRRH"] = None
		if n["RightSupDiagRRH"] != None:
			elements[n["src"]].adjacencies["RightSupDiagRRH"] = n["RightSupDiagRRH"]
		else:
			elements[n["src"]].adjacencies["RightSupDiagRRH"] = None
		if n["RightInfDiagRRH"] != None:
			elements[n["src"]].adjacencies["RightInfDiagRRH"] = n["RightInfDiagRRH"]
		else:
			elements[n["src"]].adjacencies["RightInfDiagRRH"] = None
		if n["LeftSupDiagRRH"] != None:
			elements[n["src"]].adjacencies["LeftSupDiagRRH"] = n["LeftSupDiagRRH"]
		else:
			elements[n["src"]].adjacencies["LeftSupDiagRRH"] = None
		if n["LeftInfDiagRRH"] != None:
			elements[n["src"]].adjacencies["LeftInfDiagRRH"] = n["LeftInfDiagRRH"]
		else:
			elements[n["src"]].adjacencies["LeftInfDiagRRH"] = None
		if n["UpSideRRH"] != None:
			elements[n["src"]].adjacencies["UpSideRRH"] = n["UpSideRRH"]
		else:
			elements[n["src"]].adjacencies["UpSideRRH"] = None
		if n["DownSideRRH"] != None:
			elements[n["src"]].adjacencies["DownSideRRH"] = n["DownSideRRH"]
		else:
			elements[n["src"]].adjacencies["DownSideRRH"] = None

#print the list of nighbors
def printNeighbors(elements):
	for i in elements:
		if i.startswith("RRH"):
			print("Neighbors of {} are:".format(i))
			print(elements[i].adjacencies)