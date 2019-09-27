class Bot:
	def __init__(self, endPointAddress, botType, status, arucoNumber, port, lastKnownCamera):
		self.endPointAddress = endPointAddress
		self.botType = botType
		self.status = status
		self.arucoNumber = arucoNumber
		self.port = port
		self.lastKnownCamera = lastKnownCamera

	def getAddress(self):
		return self.endPointAddress

	def getPort(self):
		return str(self.port)

	def getBotType(self):
		return self.botType

	def getArucoNumber(self):
		return self.arucoNumber

	def getLastKnownCamera(self):
		return self.lastKnownCamera

	def setStatus(self, status):
		self.status = status

	def getStatus(self):
		return self.status

	def setArucoNumber(self, number):
		self.arucoNumber = number

	def setLastKnownCamera(self, camera):
		self.lastKnownCamera = camera

class Camera:
	def __init__(self, endPointAddress, port, cameraNumber):
		self.endPointAddress = endPointAddress
		self.neghbors = []
		self.cameraNumber = cameraNumber
		self.port = port

	def getAddress(self):
		return self.endPointAddress

	def getPort(self):
		return self.port

	def getAllNeighbors(self):
		return self.neighbors

	def setAllNeighbors(self, neighborsInput):
		self.neighbors = neighborsInput

	def getNeighbor(self, whichNeighbor):
		return neighbors[whichNeighbor]

	def setNeighbor(self, whichNeighbor, neighborEndpoint):
		self.neighbor[whichNeighbor]=neighborEndpoint

	def getCameraNumber(self):
		return self.cameraNumber

	def setCameraNumber(self, cameraNumber):
		self.cameraNumber=cameraNumber

	def addNeighbor(self, cameraAdded):
		self.neighbors.append(cameraAdded) #Need to figure out how to organize the cameras to tell the direction in which they are neighbors

class Host:
	def __init__(self, hostAddress, port):
		self.hostAddress = hostAddress
		self.port = port

	def getPort(self):
		return self.port

	def setAddress(self, hostAddress):
		self.hostAddress = hostAddress

	def getAddress(self):
		return self.hostAddress
	
