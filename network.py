#This is the network module. It keeps all network elements, such as, processing nodes, RRHs, network nodes
import copy
import sys
import abc
import simpy
import functools
import networkx as nx
import random
import time
from enum import Enum
import numpy
import utility as util
from scipy.stats import norm
import psutil
from functools import reduce
import algorithms

#some network parameters and useful variables
#amount of available wavelengths
wavelengthsAmount = None
#this dictionaire keeps all created network objects
elements = {}
#list to keeps all fog nodes
fogNodes = []
#amount of generated CPRI/eCPRI frames
generatedCPRI = 0
#algorithm to be used
chosenAlgorithm = None
#downlink and uplink split option
dl_split = None
ul_split = None
#time stamp to change the load
timeStamp = None
#keep the total generated frames/packets
ip_packets = []
ethernet_frames = []
#keep the occupied bandwidth on the fronthaul
uplink_fronthaul_bandwidth = []
downlink_fronthaul_bandwidth = []
avg_ul_fronthaul = []
avg_dl_fronthaul = []
#keep the consumed memory and cpu
consumed_memory = []
consumed_cpu = []

#this class represents a general frame
#aId is the fram id, payLoad is its data, src and dst is the source and destiny element, nextHop keeps the path from src to dst, procTime is the average time to process this frame
class Frame(object):
	def __init__(self, aId, src, dst):
		self.aId = aId
		self.payLoad = []
		self.load = 0
		self.originalLoad = 0#it is used to keep the original load of the frame in the case of modifying it on splits
		self.src = src
		self.dst = dst
		self.nextHop = []
		self.inversePath = []#return path
		self.aType = None
		self.splitPart = 1

#this class extends the basic frame to represent a basic eCPRI frame
#the ideia is that it carries payload from several users equipments and can carry one or more QoS classes of service
#users is a list of UEs being carried, 
#QoS are the classes of service carried on this fram and size is the bit rate of the frame
class cpriFrame(Frame):
	def __init__(self, aId, src, dst, users, QoS):
		super().__init__(aId, src, dst)
		self.users = users
		self.QoS = QoS

#this class represents a simple ACK message
class AckMessage(object):
	def __init__(self):
		self.nextHop = None

#this class represents a basic user equipment
#aId is the UE identification, posY and posX are the locations of the UE in a cartesian plane, applicationType is the kind of application accessed by the UE (e.g., video, messaging)
class UserEquipment(object):
	def __init__(self, env, aId, servingRRH, applicationType, localTransmissionTime):
		self.env = env
		self.aId = aId
		self.servingRRH = servingRRH
		#set the beginning position of each UE as the middle of its base station area
		self.posY = 0 
		self.posX = 0
		self.localTransmissionTime = localTransmissionTime
		self.applicationType = applicationType
		self.initiation = self.env.process(self.run())
		self.latency = 0.0
		self.jitter = 0.0
		self.lastLatency = 0.0
		self.signalStrength = self.servingRRH.signalStrength
		self.interRRHs = []
		self.upLinkBitRate = 1500#uplink bit rate per ms (ms because of the generation interval for frames, so it may change)
		self.QoS = None

	#this method verifies the RSSI of the signal of the UE and triggers the CoMP process if it is below the threshold (however, the processing of the CoMP set will be delegated to another method)
	#when triggering the CoMP process, the UE will send its position
	def checkInterference(self):
		#check the position of the RRHs
		self.signalStrength = self.checkPosition()
		#print("UE {} is being interfered by {} RRHs".format(self.aId, len(self.interRRHs)))
		if self.signalStrength < 5000:#an arbitrary value for the RSSI threshold
			#print("RRH {} Sending CSI to {}".format(self.aId, self.servingRRH.aId))
			self.servingRRH.comp_notifications.put(self)

	#this method verifies the position of the UE and set how much interference it is receiving/how many RRHs are interacting with it
	def checkPosition(self):
		#possible positions considering positive X and Y axis
		if self.posX >= 50 and 0 <=self.posY <= 25:
			#print("Right")
			self.interRRHs.append(self.servingRRH.adjacencies["RightRRH"])
		elif self.posX >= 50 and 25 < self.posY <= 50:
			#print("Right and SupRight")
			self.interRRHs.extend((self.servingRRH.adjacencies["RightRRH"], self.servingRRH.adjacencies["RightSupDiagRRH"]))
		elif self.posX >= 50 and 50 < self.posY:
			#print("Right and SupRight and Upside")
			self.interRRHs.extend((self.servingRRH.adjacencies["RightRRH"], self.servingRRH.adjacencies["RightSupDiagRRH"], self.servingRRH.adjacencies["UpSideRRH"]))
		elif 0 <= self.posX < 50 and 50 < self.posY:
			#print("Upside, and right and left sup")
			self.interRRHs.extend((self.servingRRH.adjacencies["UpSideRRH"], self.servingRRH.adjacencies["RightSupDiagRRH"], self.servingRRH.adjacencies["LeftSupDiagRRH"]))
		#possible positions considering positive X axis and negativa Y axis
		elif self.posX >= 50 and 0 > self.posY >= - 25:
			#print("Right")
			self.interRRHs.append(self.servingRRH.adjacencies["RightRRH"])
		elif self.posX >= 50 and -25 > self.posY >= -50:
			#print("Right and InfRight")
			self.interRRHs.extend((self.servingRRH.adjacencies["RightRRH"], self.servingRRH.adjacencies["RightInfDiagRRH"]))
		elif self.posX >= 50 and -50 > self.posY:
			#print("Right and InfRight and Downside")
			self.interRRHs.extend((self.servingRRH.adjacencies["RightRRH"], self.servingRRH.adjacencies["RightInfDiagRRH"], self.servingRRH.adjacencies["DownSideRRH"]))
		elif 0 <= self.posX < 50 and -50 > self.posY:
			#print("Downside, left and right inf")
			self.interRRHs.extend((self.servingRRH.adjacencies["DownSideRRH"], self.servingRRH.adjacencies["RightInfDiagRRH"], self.servingRRH.adjacencies["LeftInfDiagRRH"]))
		#possible possitions considering negative X axis and positive Y axis
		elif self.posX <= -50 and 0 <= self.posY <= 25:
			#print("Left")
			self.interRRHs.append(self.servingRRH.adjacencies["LeftRRH"])
		elif self.posX <= -50 and 25 < self.posY < 50:
			#print("Left and SupLeft")
			self.interRRHs.extend((self.servingRRH.adjacencies["LeftRRH"], self.servingRRH.adjacencies["LeftSupDiagRRH"]))
		elif self.posX <= 50 and 50 < self.posY:
			#print("Left and SupLeft and Upside")
			self.interRRHs.extend((self.servingRRH.adjacencies["LeftRRH"], self.servingRRH.adjacencies["LeftSupDiagRRH"], self.servingRRH.adjacencies["UpSideRRH"]))
		elif 0 >= self.posX > -50 and 50 < self.posY:
			#print("Upside, SupLeft and SupRight")
			self.interRRHs.extend((self.servingRRH.adjacencies["UpSideRRH"], self.servingRRH.adjacencies["LeftSupDiagRRH"], self.servingRRH.adjacencies["RightSupDiagRRH"]))
		#possible possitions considering negative X and Y axis
		elif self.posX <= -50 and 0 >= self.posY >= -25:
			#print("Left")
			self.interRRHs.append(self.servingRRH.adjacencies["LeftRRH"])
		elif self.posX <= -50 and -25 > self.posY > -50:
			#print("Left and InfLeft")
			self.interRRHs.extend((self.servingRRH.adjacencies["LeftRRH"], self.servingRRH.adjacencies["LeftInfDiagRRH"]))
		elif self.posX <= 50 and -50 > self.posY:
			#print("Leftm InfLeft and Downside")
			self.interRRHs.extend((self.servingRRH.adjacencies["LeftRRH"], self.servingRRH.adjacencies["LeftInfDiagRRH"], self.servingRRH.adjacencies["DownSideRRH"]))
		elif 0 >= self.posX > -50 and -50 > self.posY:
			#print("Downside and InfLeft and InfRight")
			self.interRRHs.extend((self.servingRRH.adjacencies["DownSideRRH"], self.servingRRH.adjacencies["LeftInfDiagRRH"], self.servingRRH.adjacencies["RightInfDiagRRH"]))
		#now, calculate the RSSI of the UE considering how many RRHs are interfering in its signal
		return self.calculateRSSI()

	#this method calculates the RSSI of the signal #TODO implement the calculation
	def calculateRSSI(self):
		return 10000

	#this method causes UEs to move
	def run(self):
		i = 0
		while True:
			#timeout for the UE to move
			yield self.env.timeout(0.05)
			self.randomWalk()
			#print("UE {} moved to position X = {} and Y = {} at {}".format(hash(self), self.posX, self.posY, self.env.now))
			i += 1

	#moves the UE
	def randomWalk(self):
		val = random.randint(1, 4)
		if val == 1:
			if self.posX + 1 <= self.servingRRH.CoordinateX1:
				self.posX += 1
				self.posY = self.posY 
		if val == 2:
			if self.posX -1 >= self.servingRRH.CoordinateX2:
				self.posX -= 1
				self.posY = self.posY 
		elif val == 3:
			if self.posY + 1 <= self.servingRRH.CoordinateY1:
				self.posX = self.posX 
				self.posY += 1
		else:
			if self.posY - 1 >= self.servingRRH.CoordinateY2:
				self.posX = self.posX 
				self.posY -= 1
		#check the interference
		self.checkInterference()

#TODO: Implement the placement of the RRH processing considering the different eCPRI option
#this class represents a RRH that generates user equipments and frames (CPRI or eCPRI)
class RRH(object):
	def __init__(self, env, aId, distribution, cpriFrameGenerationTime, transmissionTime, localTransmissionTime, cpriMode,
				 x1, x2, y1, y2, signalStrength, controlPlane, fogNode, bsLoads):
		self.env = env
		self.aType = "RRH"
		self.aId = "RRH"+":"+str(aId)
		self.users = []#list of active UEs served by this RRH
		self.cpriMode = cpriMode#type of cpri frames to be generated by the RRH
		self.currentLoad = 0#buffer load
		self.bsLoads = copy.copy(bsLoads)
		self.maximumLoad = self.bsLoads.pop()
		self.distribution = distribution#the distribution for the traffic generator distribution
		self.trafficGen = self.env.process(self.run())#initiate the built-in traffic generator
		self.startOperation()
		self.loadManager = self.env.process(self.changeLoad(timeStamp))
		#self.uplinkTransmitCPRI = self.env.process(self.uplinkTransmitCPRI())#send eCPRI frames to a processing node
		#self.downlinkTransmitUE = self.env.process(self.downlinkTransmitUE())#send frames to the UEs
		self.processingQueue = simpy.Store(self.env)#keep downlink frames received
		self.comp_monitor = self.env.process(self.triggerCoMP())#waits for a RSSI notification from the UE
		self.cpriFrameGenerationTime = cpriFrameGenerationTime
		self.transmissionTime = transmissionTime
		self.localTransmissionTime = localTransmissionTime
		self.comp_notifications = simpy.Store(self.env)#waits for a CoMP solicitation from a UE
		self.CoordinateX1 = x1#limiting coordinates of the base station area
		self.CoordinateX2 = x2#limiting coordinates of the base station area
		self.CoordinateY1 = y1#limiting coordinates of the base station area
		self.CoordinateY2 = y2#limiting coordinates of the base station area
		self.adjacencies = {}#each key is a range of coordinates expressed as a tuple, for instance, {("x1")}
		self.signalStrength = signalStrength
		self.path = None#the path to be calculated between source and destiny
		self.length = None#the length of the path used by this RRH
		self.allocatedWavelength = None#the wavelength used by this RRH to transmit traffic
		self.controlPlane = controlPlane#reference to the control plane
		self.fogNode = fogNode#the serving fog node of this RRH (for instance, the closer RRH)
		self.processingNode = None#the processing node that hosts the RRH's vBBU
		self.procDemand = 0


	#starts important methods
	def startOperation(self):
		print("{} starts operating with {}".format(self.aId, self.maximumLoad))
		self.uplinkTransmitCPRI = self.env.process(self.uplinkTransmitCPRI())  # send eCPRI frames to a processing node
		self.downlinkTransmitUE = self.env.process(self.downlinkTransmitUE())#send frames to the UEs

	#account the generated frames (to be invoked when traffic changes)
	def accountFrames(self):
		global ethernet_frames
		ethernet_frames.append(generatedCPRI)
		consumed_memory.append(psutil.virtual_memory().percent)
		consumed_cpu.append(psutil.cpu_percent())

	#changes the maximum amount of UEs
	def changeLoad(self, timeStep):#time step is the time to change the load of base station; bsLoad is a list containing the loads
		while True:
			#wait for the time to change the load on the base station
			yield self.env.timeout(timeStep)
			#account the current amount of generated frames
			self.accountFrames()
			if self.bsLoads:#if list happens to be empty, continue with the current load
				self.maximumLoad = self.bsLoads.pop()#TODO implement this on the configuration and utility modules
			#print("Now operating with {} UEs on {} at {}".format(self.maximumLoad, self.aId, self.env.now))
			#print("{} had {} UEs at {}".format(self.aId, len(self.users), self.env.now))
			#self.maximumLoad += 10

	#send a message/frame to its connected switch
	def sendRequest(self, request):
		request.nextHop = copy.copy(self.path)
		request.path = copy.copy(self.path)
		request.path.pop(0)
		request.inversePath = list(request.nextHop)
		request.inversePath.reverse()
		request.inversePath.pop(0)
		# takes the next hop
		request.nextHop.pop(0)
		destiny = elements[request.nextHop.pop(0)]
		# yield self.env.timeout(self.transmissionTime)
		destiny.processingQueue.put(request)
		destiny.currentLoad += 1

	#this method generates users equipments
	def run(self):
		i = 0
		while True:
			yield self.env.timeout(self.distribution(self))
			#a limit for the generation of UEs for testing purposes
			if len(self.users) < self.maximumLoad:
				ue = UserEquipment(self.env, i, self, "Messaging", self.localTransmissionTime)
				self.users.append(ue)
				#print("{} generated UE {} at {}".format(self.aId, hash(ue), self.env.now))
				i += 1


	#request the lightpath wavelength from the control plane
	def requestLightpath(self):
		path_copy = copy.copy(self.path)
		#remove the own RRH and the processing node from the path (as the dijkstra returns them) to keep only the intermediate nodes
		path_copy.pop()
		path_copy.pop(0)
		wavelength = self.controlPlane.allocateWavelength(path_copy)
		#check if a lambda was found
		if wavelength != -1:
			self.allocatedWavelength = wavelength
		# if no, send back an error message
		else:
			print("Lightpath failed")#blocks

	#every time a frame is received from a UE, keep it to generate the eCPRI frame later
	def takeFrameUE(self):
		while True:
			r = yield self.received_users_frames.get()
			self.frames.append(r)

	#generate a CPRI frame
	def cpriFrameGeneration(self, aId, src, dst, QoS, frame_id):
		#take each UE and put it into the CPRI frame
		activeUsers = []
		if src.users:
			for i in src.users:
				activeUsers.append(i)
		#Cloud:0 is the generic destiny for tests purposes - An algorithm will be used to decide in which node it will be placed
		CPRIFrame = cpriFrame(src.aId+"->"+str(frame_id), src, dst, activeUsers, QoS)
		#TODO atualizar o tempo em que cada UE mandou o quadro para o RRH em função da sua distância até ele (ex. env.now - transmissiontTime,  transmissionTime vai ser dinâmico)
		if src.users:
			for i in src.users:
				i.lastLatency = i.latency
				i.latency = (i.latency + self.env.now)/frame_id
		return CPRIFrame

	#this method triggers the CoMP process for an UE
	def triggerCoMP(self):
		while True:
			#waits for an UE to send a message informing that
			comp_request = yield self.comp_notifications.get()
			print("Processing CoMP for UE {}".format(comp_request.aId))
			#TODO Implement the CoMP set generation

	#calls the placement algorithm
	def findProcessingNode(self, chosenAlgorithm):
		node = None
		if chosenAlgorithm == "cloudPlacement":
			node = algorithms.cloudPlacement(elements["Cloud:0"], self)
		if node != None:
			return True
		else:
			return False

	#this method builds a eCPRI frame and uplink transmits it to a optical network element
	#TODO: Update the send time of each UE frame in function of the distance of each UE from the RRH 
	def uplinkTransmitCPRI(self):
		global generatedCPRI
		frame_id = 1
		#self.length, self.path = nx.single_source_dijkstra(self.controlPlane.graph, self.aId, "Cloud:0")#For now, cloud is the default destiny
		#self.length, self.path = self.shorstestPath("Cloud:0")
		while True:
			if self.processingNode == None:
				if self.findProcessingNode(chosenAlgorithm):
					print("{} allocated at {}".format(self.aId, self.processingNode.aId))
					self.length, self.path = util.dijkstraShortestPath(self.controlPlane.graph, self.aId, self.processingNode.aId)#calculate the path to the node
				else:
					print("vBBU from {} was not allocated".format(self.aId))
			else:
				if self.allocatedWavelength != None:
					yield self.env.timeout(self.cpriFrameGenerationTime)
					#If traditional CPRI is used, create a frame with fixed bandwidth
					if self.cpriMode == "CPRI":
						#print(psutil.virtual_memory().percent)
						print("{} generating CPRI frame {} at {}".format(self.aId, self.aId+"->"+str(frame_id), self.env.now))
						eCPRIFrame = self.cpriFrameGeneration(self.aId+"->"+str(frame_id), self, self.processingNode.aId, None, frame_id)
						generatedCPRI += 1
					elif self.cpriMode == "eCPRI":
						#print("{} generating eCPRI frame {} at {}".format(self.aId, self.aId+"->"+str(frame_id), self.env.now))
						eCPRIFrame = self.e_CpriFrameGeneration(frame_id, None, self, "Cloud:0", None, frame_size, frame_id)
						generatedCPRI += 1
					#send the request to the next connected network element
					self.sendRequest(eCPRIFrame)
					frame_id += 1
				else:
					self.requestLightpath()

	#This method hipothetically sends an ACK to each UE. The ACK message is modeled as an update on the received time attribute of each UE.
	#TODO: Implement the received time for eacj UE in function of its distance to the RRH
	def downlinkTransmitUE(self):
		frame_id = 1
		while True:
			#print(psutil.virtual_memory())
			received_frame = yield self.processingQueue.get()
			#check if it is an Harq Ack message
			if isinstance(received_frame, AckMessage):
				print("{} received an ack from processing node".format(self.aId))
			#print("{} transmitting to its UEs".format(self.aId))
			else:
				if received_frame.users:
					for i in received_frame.users:
						yield self.env.timeout(self.localTransmissionTime)
						#TODO: Implement the jitter calculation, using the lastLatency variable
						i.jitter = (i.latency + self.env.now)/frame_id
					#update the load on the buffer after processing the frame
					self.currentLoad -= 1
					frame_id += 1
			del received_frame

#basic network node to be extended
class ActiveNode(metaclass=abc.ABCMeta):
	def __init__(self, env, aId, aType, capacity):
		self.env = env
		self.aType = aType
		self.aId = aType+":"+str(aId)
		self.processingCapacity = capacity
		self.currentLoad = 0
		self.processingQueue = simpy.Store(self.env)
		self.lastNode = None
		self.startOperation()
		self.procQueue = []

	#process each frame
	@abc.abstractmethod
	def processRequest(self):
		pass

	#transmit each frame after processing
	@abc.abstractmethod
	def sendRequest(self, request):
		pass

	#test the processing capacity
	def hasCapacity(self):
		if self.currentLoad <= self.processingCapacity:
			return True
		else:
			return False

	#start the operation
	@abc.abstractmethod
	def startOperation(self):
		pass

#a general processing node
class ProcessingNode(ActiveNode):
	def __init__(self, env, aId, aType, capacity, qos, procTime, transmissionTime, graph):
		super().__init__(env, aId, aType, capacity)
		self.qos = qos#list of class of service suppoerted by this node
		self.procTime = procTime
		self.transmissionTime = transmissionTime
		self.graph = graph
		self.vBBUs = {}#keeps the RRHs being processed. Key: RRH.aId, value: processing being consumed by each RRH

	#starts the operation
	def startOperation(self):
		self.toProcess = self.env.process(self.processRequest())

	#process a request
	def processRequest(self):
		while True:
			request = yield self.processingQueue.get()
			if self.aId == request.dst:#this is the destiny node. Process it and compute the downlink path
				#print("Request {} arrived at destination {}".format(request.aId, self.aId))
				request.nextHop = request.inversePath
				# wait for the time to send the Harq ack frame
				ackFrame = AckMessage()
				# yield self.env.timeout(0.002750)
				ackFrame.nextHop = copy.copy(request.nextHop)
				self.sendRequest(ackFrame)
			#print("{} buffer load is {}".format(self.aId, self.currentLoad))
			#print("{} processing request {} at {}".format(self.aId, request.aId, self.env.now))
			yield self.env.timeout(self.procTime)
			#self.processingCapacity -= 1
			#update the load on the buffer after processing the frame
			self.currentLoad -= 1
			self.sendRequest(request)

	#transmit a request to its destiny	
	def sendRequest(self, request):
		nextHop = request.nextHop.pop(0)#returns the id of the next hop
		destiny = elements[nextHop]#retrieve the next hop object searching by its id
		#print("{} sending request {} to {}".format(self.aId, request.aId, destiny.aId))
		#print("{} buffer load is {}".format(self.aId, self.currentLoad))
		self.env.timeout(self.transmissionTime)
		destiny.processingQueue.put(request)
		#update the load on the buffer of the destiny node
		destiny.currentLoad += 1

#a general processing node
class NetworkNode(ActiveNode):
	def __init__(self, env, aId, aType, capacity, qos, switchTime, transmissionTime, graph):
		super().__init__(env, aId, aType, capacity)
		self.qos = qos#list of class of service suppoerted by this node
		self.switchTime = switchTime
		self.transmissionTime = transmissionTime
		self.graph = graph
		self.lambdasList = []
		# initiate the list of available wavelengths
		for i in range(wavelengthsAmount):
			self.lambdasList.append(i)

	# starts the operation
	def startOperation(self):
		self.toProcess = self.env.process(self.processRequest())

	#process a request
	def processRequest(self):
		while True:
			request = yield self.processingQueue.get()
			#if not isinstance(request, AckMessage):
			#	print("{} got request {} from {}".format(self.aId, request.aId, request.src.aId))
			#yield self.env.timeout(self.switchTime)
			#update the load on the buffer after processing the frame
			self.currentLoad -= 1
			#self.processingCapacity -= 1
			self.sendRequest(request)

	#transmit a request to its destiny
	def sendRequest(self, request):
		nextHop = request.nextHop.pop(0)#returns the id of the next hop
		destiny = elements[nextHop]#retrieve the next hop object searching by its id
		#if not isinstance(request, AckMessage):
		#	print("{} sending request {} to {}".format(self.aId, request.aId, destiny.aId))
		#self.env.timeout(self.transmissionTime)
		destiny.processingQueue.put(request)
		#update the load on the buffer of the destiny node
		destiny.currentLoad += 1

#this class represents a control plane to perform centralized decisions
class ControlPlane(object):
	def __init__(self, env, aId, aType, graph):
		self.env = env
		self.aId = aType+":"+str(aId)
		self.aType = aType
		self.graph = graph

	#this method returns a wavelength available in a path of network nodes
	def getPath(self, path):
		#create a list of the lists of available wavelengths
		lambdas = []
		for i in path:
			if elements[i].aType == "Switch":
				lambdas.append(elements[i].lambdasList)
		#get the first common wavelength for network nodes in the path
		res = list(reduce(lambda i, j: i & j, (set(x) for x in lambdas)))
		return res

	#this method allocates the wavelength for a path
	def allocateWavelength(self, path):
		#gets the wavelength
		res =  self.getPath(path)
		if res:
			wavelength = res.pop()
			#pop the wavelength from each network node in path
			for i in path:
				if elements[i].aType == "Switch":
					elements[i].lambdasList.remove(wavelength)
			return wavelength
		else:
			return -1

#******************************************************eCPRI***************************************************************************
#size of the ethernet pay load
maximumEthernetPayload = 1500#default value in bytes

#enhanced CPRI packet
class eCPRIFrame(Frame):
	def __init__(self, aId, src, dst, QoS, srcUser, load):
		super().__init__(aId, src, dst)
		self.QoS = QoS
		self.srcUser = srcUser
		self.load = load
		self.splitPart = 1#to identify to each part of the split this frame/packet is part
		self.secondDst = "Cloud:0"#processing node of the second part of the split (the cloud is the default

# this class represents a base station operating under eCPRI
class eRRH(RRH):
	def __init__(self, env, aId, distribution, cpriFrameGenerationTime, transmissionTime, localTransmissionTime,
				 cpriMode, eCpriEncap, x1, x2, y1, y2, signalStrength, controlPlane, fogNode, bsLoads):
		self.eCpriEncap = eCpriEncap
		super().__init__(env, aId, distribution, cpriFrameGenerationTime, transmissionTime, localTransmissionTime,
						 cpriMode, x1, x2, y1, y2, signalStrength, controlPlane, fogNode, bsLoads)
		self.secondProcessingNode = None
		self.splitOption = None
		self.secondPath = None#path for the second part of the split
		self.returningPath = None#return pat hfrom the second node to the source
		self.frameId = 0

	#account the ethernet frames or IP packets generated
	def accountFrames(self):#TODO move it to a control plane method that accounts for the entire network
		global ethernet_frames, ip_packets, avg_dl_fronthaul, avg_ul_fronthaul, uplink_fronthaul_bandwidth, downlink_fronthaul_bandwidth
		#check which type of encapsulation was used in this RRH
		if self.eCpriEncap == "Ethernet":
			ethernet_frames.append(generatedCPRI)
		elif self.eCpriEncap == "IP":
			ip_packets.append(generatedCPRI)

	#override the start operation method
	def startOperation(self):
		#print("{} starts operating with {}".format(self.aId, self.maximumLoad))
		if self.eCpriEncap == "IP":
			self.uplinkTransmitCPRI = self.env.process(self.uplinkTransmitCPRI())  # send eCPRI frames as IP to a processing node
			self.downlinkTransmitUE = self.env.process(self.downlinkTransmitUE())  # send frames to the UEs
		elif self.eCpriEncap == "Ethernet":
			self.uplinkTransmitEcpri = self.env.process(self.eCpriEthernetUplink())
			self.downlinkTransmitUE = self.env.process(self.eCpriEthernetDownlink())

	#calculate the two paths for this RRH
	def getPaths(self):
		#print("Getting paths for {}".format(self.aId))
		self.path = util.dijkstraShortestPathOnly(self.controlPlane.graph, self.aId, self.processingNode.aId)
		#print("First path is {}".format(self.path))
		self.secondPath = util.dijkstraShortestPathOnly(self.controlPlane.graph, self.processingNode.aId, self.secondProcessingNode.aId)
		#print("Second path is {}".format(self.secondPath))
		self.returningPath = util.dijkstraShortestPathOnly(self.controlPlane.graph, self.aId, self.secondProcessingNode.aId)
		#print("Returning path is {}".format(self.returningPath))

	# send a message/frame to its connected switch
	def sendRequest(self, request):
		util.formatPath(request, self.path)
		#request.nextHop = copy.copy(self.path)
		#request.path = copy.copy(self.path)
		#request.path.pop(0)
		#request.inversePath = list(request.nextHop)
		#request.inversePath.reverse()
		#request.inversePath.pop(0)
		# takes the next hop
		request.nextHop.pop(0)
		destiny = elements[request.nextHop.pop(0)]
		# yield self.env.timeout(self.transmissionTime)
		destiny.processingQueue.put(request)
		destiny.currentLoad += 1

	#create an eCPRI IP flow
	def eCpriIpFlow(self):
		# prepare the size of the flow - each UE will demand a separated packet in this approach (no QoS distinction)
		frameId = 1
		packets = []  # keep all the generated packets
		for i in self.users:
			bitRate = 0
			# calculate the amount of packets (its bitRate per ms/ip packet size to generate to this user
			bitRate = int(i.upLinkBitRate / 150)#150 is a default packet size (defined by me) in bytes
			# create the amount of packets for this user
			for j in range(bitRate):
				packets.append(eCPRIFrame(frameId, self, None, i.QoS, i, i.upLinkBitRate))
				# update the latency of each user when sending the packet to the RRH (to account latency and jitter later)
				i.lastLatency = i.latency
				i.latency = (i.latency + self.env.now)
				frameId += 1
		return packets

		#send each eCPRI packet as an IP packet for each user
	def uplinkTransmitCPRI(self):
		global generatedCPRI
		#sets any fog node as the first processing node of this RRH and the cloud as the second (placement algorithm must do it)
		self.processingNode = elements["Fog:1"]
		self.secondProcessingNode = elements["Cloud:0"]
		self.getPaths()
		#self.path = util.dijkstraShortestPathOnly(self.controlPlane.graph, self.aId, "Fog:1")
		#self.secondPath = util.dijkstraShortestPathOnly(self.controlPlane.graph, "Fog:1", "Cloud:0")
		#self.returningPath = util.dijkstraShortestPathOnly(self.controlPlane.graph, "Cloud:0", self.aId)
		#print("First path is {}".format(self.path))
		#print("Second path is {}".format(self.secondPath))
		#print("Returning path is {}".format(self.returningPath))
		while True:
			yield self.env.timeout(self.cpriFrameGenerationTime)
			# only send traffic if there are UEs connected
			if self.users:
				packets = self.eCpriIpFlow()
				#send each packet
				#print("Total flow of {} is {}".format(self.aId, len(packets)*1500))
				#TODO define the split and destiny of each flow of packets
				for i in packets:
					#for testing purposes, each packet will be destinated to the cloud
					i.dst = "Cloud:0"
					generatedCPRI += 1
					print(psutil.virtual_memory().percent)
					print("Sending IP packets {} from {} at {}".format(i.aId, self.aId, self.env.now))
					#before sending, calculate the split regarding the QoS of the packets
					#after calculating the split, send a message to the SDN controller
					#format the path
					self.sendRequest(i)

		#send the downlink packets back to the UEs
	def downlinkTransmitUE(self):
		while True:
			received_frame = yield self.processingQueue.get()
			# check if it is an Harq Ack message
			if isinstance(received_frame, AckMessage):
				pass
				#print("{} received an ack from processing node".format(self.aId))
			else:
				#print("{} transmitting IP packets to its UEs at {}".format(self.aId, self.env.now))
				yield self.env.timeout(self.localTransmissionTime)
				# TODO: Implement the jitter calculation, using the lastLatency variable
				received_frame.srcUser.jitter = (received_frame.srcUser.latency + self.env.now)
				# update the load on the buffer after processing the frame
				self.currentLoad -= 1
			del received_frame

		# create a eCPRI Ethernet flow
	def eCpriEthernetFlow(self):
		# prepare the size of the flow - all UEs packets will be put on the same Ethernet frame (no QoS distinction)
		packets = []  # keep all the generated packets
		ethernetFrames = []#keep the ethernet frames containing the generated packets to be returned
		currentFrame = None#to keep track of the current ethernet frame being populated by eCPRI packets
		#frameId = 1
		#create a first ethernet frame
		eFrame = Frame(self.frameId, self, None)
		currentFrame = eFrame
		ethernetFrames.append(eFrame)
		for i in self.users:
			bitRate = 0
			# calculate the amount of packets (its bitRate per ms/ip packet size to generate to this user)
			bitRate = int(i.upLinkBitRate / 150)#150 is the default ip packet size set by me
			# create the amount of packets for this user
			for j in range(bitRate):
				packets.append(eCPRIFrame(self.frameId, self, None, i.QoS, i, i.upLinkBitRate))
				# update the latency of each user when sending the packet to the RRH (to account latency and jitter later)
				i.lastLatency = i.latency
				i.latency = (i.latency + self.env.now)
		#put packets on the pay load of the ethernet frame
		while packets:
			# while there is packets to be encapsulated and capacity on an ethernet frame, put eCPRI packets on it
			if currentFrame.load < maximumEthernetPayload:
				p = packets.pop()
				currentFrame.payLoad.append(p)
				currentFrame.load += p.load
				currentFrame.originalLoad = currentFrame.load#keep the original load
			#if pay load limit if the ethernet frame was reached, create another one
			else:
				self.frameId += 1
				newFrame = Frame(self.frameId, self, None)
				currentFrame = newFrame
				p = packets.pop()
				currentFrame.payLoad.append(p)
				currentFrame.load += p.load
				currentFrame.originalLoad = currentFrame.load  # keep the original load
				ethernetFrames.append(newFrame)
		return ethernetFrames

		#send the flow of eCPRI packets as a flow of ethernet frames
	def eCpriEthernetUplink(self):
		global generatedCPRI
		#self.length, self.path = util.dijkstraShortestPath(self.controlPlane.graph, self.aId, "Cloud:0")#cloud is the default
		self.processingNode = elements["Fog:1"]
		self.secondProcessingNode = elements["Cloud:0"]
		self.getPaths()
		#frameId = 1
		#packetLimit = 0#test variable to limit the generation of frames for debbuging purposes
		while True:
			yield self.env.timeout(self.cpriFrameGenerationTime)
			#only send traffic if there are UEs connected
			#if packetLimit < 1:
			if self.users:
				frames = self.eCpriEthernetFlow()
				#print("Total Ethernet flow of {} is {}".format(self.aId, len(frames)))
				#TODO define the split and destiny of each flow of packets
				#send each ethernet frame
				for i in frames:
					# for testing purposes, each packet will be destinated to the cloud
					i.dst = "Cloud:0"
					generatedCPRI += 1
					#print("Sending Ethernet frame {} from {} at {}".format(i.aId, self.aId, self.env.now))
					#print("{} has {} UEs".format(self.aId, len(self.users)))
					#packetLimit += 1
					self.sendRequest(i)

		# send the downlink packets back to the UEs
	def eCpriEthernetDownlink(self):
		while True:
			received_frame = yield self.processingQueue.get()
			# check if it is an Harq Ack message
			if isinstance(received_frame, AckMessage):
				pass
				#print("{} received an ack from processing node".format(self.aId))
			else:
				print("{} transmitting Ethernet frames {} (actually, ip packets :) ) to its UEs at {}".format(self.aId, received_frame.aId, self.env.now))
				# extract each UE packet
				for i in received_frame.payLoad:
					yield self.env.timeout(self.localTransmissionTime)
					# TODO: Implement the jitter calculation, using the lastLatency variable
					i.srcUser.jitter = (i.srcUser.latency + self.env.now)
					# update the load on the buffer after processing the frame
					self.currentLoad -= 1
			del received_frame

#processing node supporting eCPRI
class eProcessingNode(ProcessingNode):
	def __init__(self, env, aId, aType, capacity, qos, procTime, transmissionTime, graph):
		super().__init__(env, aId, aType, capacity, qos, procTime, transmissionTime, graph)
		self.packets = 0
		self.aggregatedDownlinkBandwidth = []
		self.bitRate = []
		self.monitorBandwidth = self.env.process(self.monitorBitrate())

	#this method accounts the bit rate at each seconds (or any desired time interval)
	def monitorBitrate(self):#TODO move this to a control plane method that accounts for the entire network
		global downlink_fronthaul_bandwidth
		while True:
			yield self.env.timeout(timeStamp)
			if self.aId == "Cloud:0":
				#print("Downlink bitrate was", downlink_fronthaul_bandwidth)
				#print("Got here at", self.env.now)
				downlink_fronthaul_bandwidth.append((sum(self.aggregatedDownlinkBandwidth)/timeStamp)*8)
				self.aggregatedDownlinkBandwidthBandwidth = []
			# account the memory usage
			consumed_memory.append(psutil.virtual_memory().percent)
			consumed_cpu.append(psutil.cpu_percent())

	#process each part of the split of an eCPRI flow
	def processRequest(self):
		while True:
			request = yield self.processingQueue.get()
			#print("{} get request {} from {} with part {} and dst {}".format(self.aId, request.aId, request.src.aId, request.splitPart, request.dst))
			#after getting the request, check which part of the split is
			#if it is the first part, process it, reduce the size of the pay load (regarding the split option)
			#then, sends to the next destiny
			if request.splitPart == 1:
				#process it
				#yield self.env.timeout(self.proc)#arbitrary value for testing
				#decrease the pay load of the packet/frame regarding the uplink eCPRI split
				#self.reduceUplinkPayload(request)
				request.splitPart = 2
				#get the path to the next processing node
				#print("This is node {} and path is {}".format(self.aId, request.src.secondPath))
				#util.formatPath(request, request.src.secondPath)#note that the RRH (request.src) keeps the paths
				util.setSecondPath(request)
				#print("UP",uplink_fronthaul_bandwidth)
			elif request.splitPart == 2:#frame reached its second processing node
				#process it and then send back
				#yield self.env.timeout(self.transmissionTime)#arbitrary value for testing
				#decrease the pay load of the packet/frame regarding the downlink eCPRI split
				self.reduceDownLinkPayload(request)
				util.reversePath(request, request.src.returningPath)
				# wait for the time to send an Har ack message
				ackFrame = AckMessage()
				# yield self.env.timeout(0.002750)
				ackFrame.nextHop = copy.copy(request.nextHop)
				self.sendRequest(ackFrame)
				#accounts the aggregated bandwidth
				self.aggregatedDownlinkBandwidth.append(request.load)
			self.packets += 1
			#print("{} switched {} packets".format(self.aId, self.packets))
			#get the path to the src
			self.currentLoad -= 1
			#print("Sending request {} to {}".format(request.aId, request.nextHop))
			#exit()
			self.sendRequest(request)

	#TODO This method simply reduce the load parameter of the packet/frame. Implement the generation of the amount of packets
	#TODO regarding the new total pay load (for instance, RRH generate 100 Mbps in 100 packet, now uplink traffic is 50 Mbps and needs only 50 packets of full size
	#reduce the pay load size of an eCPRI packet/frame on uplink direction
	def reduceUplinkPayload(self, request):
		request.load = (request.load/100)*12

	#TODO Similarly to the reduceUplinkPayload case, implement the generation of new packets/frames regarding the new bit rate
	#reduce the pay load of an eCPRI packet/frame on downlink direction
	def reduceDownLinkPayload(self, request):
		#check which split option is being used
		if dl_split == "Option1":
			request.load = (request.load / 100) * 6
		elif dl_split == "Option2":
			request.load = (request.load / 100) * 12

#eCPRI SDN control plane
class sdnController(ControlPlane):
	def __init__(self, env, aId, aType, graph):
		super().__init__(env, aId, aType, graph)

	#calculate the two paths for a split eCPRI flow
	def getSplitPaths(baseStation):
		pass
