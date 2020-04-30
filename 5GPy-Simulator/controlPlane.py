#this module presents a control plane for the network
#the objective of this module is to centralize the network state and take centralize decisions

#main control plane class
class ControlPlane(object):
    def __init__(self, aId, networkElements, networkGraph):
        self.aId = aId
        self.networkElements = networkElements
        self.networkGraph = networkGraph