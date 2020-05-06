#this module contains algorithms for the network operation

#this method places vBBUs first on the cloud and then on fog nodes
#for the requesting base station, place on the cloud if there is capacity; otherwise, place on its serving fog node
#with this algorithm, CoMP sets will be processed on the same place of the vBBU of the RRHs
#if no node has capacity, it will return None
def minEnergyPlacement(networkNodes, baseStation):
    #first checks if the cloud has capacity
    if baseStation.procDemand <= networkNodes["Cloud:0)"].processingCapacity:
        networkNodes["Cloud:0"].vBBUs[baseStation.aId] = baseStation.procDemand
        networkNodes["Cloud:0"].processingCapacity -= baseStation.procDemand
        baseStation.processingNode = networkNodes["Cloud:0)"]
        return networkNodes["Cloud:0)"]
    #otherwise, check the serving fog node of the RRH
    elif baseStation.procDemand <= networkNodes[baseStation.fogNode].processingCapacity:
        networkNodes[baseStation.fogNode].vBBUs[baseStation.aId] = baseStation.procDemand
        networkNodes[baseStation.fogNode].processingCapacity -= baseStation.procDemand
        baseStation.processingNode = networkNodes[baseStation.fogNode]
        return networkNodes[baseStation.fogNode]

#put only on the cloud; if no capacity is found, put on a queue that will increase latency
def cloudPlacement(cloud, baseStation):
    # first checks if the cloud has capacity
    if baseStation.procDemand <= cloud.processingCapacity:
        cloud.vBBUs[baseStation.aId] = baseStation.procDemand
        cloud.processingCapacity -= baseStation.procDemand
        baseStation.processingNode = cloud
        return cloud
    #otherwise, put on the queue
    else:
        baseStation.processingNode = cloud
        cloud.procQueue.append(baseStation)
        print("{} put on the queue of {}".format(baseStation.aId, cloud.aId))
        return cloud

#this method first tries to allocate a vBBU on the cloud and then on a fog node that has more free processing capacity
def cloudLeastLoadedFog(networkNodes, fogNodes, baseStation):
    #first,check if the cloud has capacity
    if baseStation.procDemand <= networkNodes["Cloud:0)"].processingCapacity:
        networkNodes["Cloud:0"].vBBUs[baseStation.aId] = baseStation.procDemand
        networkNodes["Cloud:0"].processingCapacity -= baseStation.procDemand
        baseStation.processingNode = networkNodes["Cloud:0)"]
        return networkNodes["Cloud:0)"]
    #otherwise, check every fog node
    else:
        #sort processing nodes in least loaded order (after reversing it)
        fogNodes.sort(key = lambda p: p.processingCapacity)
        fogNodes.reverse()
        for i in fogNodes:
            if baseStation.procDemand <= i.processingCapacity:
                i.vBBUs[baseStation.aId] = baseStation.procDemand
                i.processingCapacity -= baseStation.procDemand
                baseStation.processingNode = i
                return i

#this method first tries to allocate a vBBU on the cloud and then on a fog node that has least free processing capacity
def cloudMostLoadedFog(networkNodes, fogNodes, baseStation):
    #first,check if the cloud has capacity
    if baseStation.procDemand <= networkNodes["Cloud:0)"].processingCapacity:
        networkNodes["Cloud:0"].vBBUs[baseStation.aId] = baseStation.procDemand
        networkNodes["Cloud:0"].processingCapacity -= baseStation.procDemand
        baseStation.processingNode = networkNodes["Cloud:0)"]
        return networkNodes["Cloud:0)"]
    #otherwise, check every fog node
    else:
        #sort processing nodes in most loaded order
        fogNodes.sort(key = lambda p: p.processingCapacity)
        for i in fogNodes:
            if baseStation.procDemand <= i.processingCapacity:
                i.vBBUs[baseStation.aId] = baseStation.procDemand
                i.processingCapacity -= baseStation.procDemand
                baseStation.processingNode = i
                return i

#put first on the base station serving fog node and then on the processing node with most free processing capacity
def servingFogFirst(networkNodes, baseStation):
    #first, check the serving fog node
    if baseStation.procDemand <= networkNodes[baseStation.fogNode].processingCapacity:
        networkNodes[baseStation.fogNode].vBBUs[baseStation.aId] = baseStation.procDemand
        networkNodes[baseStation.fogNode].processingCapacity -= baseStation.procDemand
        baseStation.processingNode = networkNodes[baseStation.fogNode]
        return networkNodes[baseStation.fogNode]
    #otherwise, take the least loaded processing node
    else:
        nodes = []#list of processing nodes
        for k, proc in networkNodes.items():
            if proc.aId != baseStation.fogNode:
                nodes.append(proc)
        nodes.sort(key=lambda p: p.processingCapacity)#sort processing nodes in least loaded order (after reversing it)
        nodes.reverse()
        #search for a proper processing node
        for i in nodes:
            if baseStation.procDemand <= i.processingCapacity:
                i.vBBUs[baseStation.aId] = baseStation.procDemand
                i.processingCapacity -= baseStation.procDemand
                baseStation.processingNode = i
                return i