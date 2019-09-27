from flask import Flask, jsonify, request
from threading import Thread

import json, pickle, requests, socket, time, traceback, logging
from classDefs import *



app = Flask(__name__)

isSimulation = False           #(Has issues currently) Set true if you want to run both camera and client on local machine

if isSimulation:
	print('In Sumulation Mode')
	myIp='127.0.0.1'
else:
	myIp = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]


botsList = []		      #List of ALL bots known to the camera
continuePollingBots = True    #CHANGE TO TRUE FOR BOT POLLING TO MAINTAIN ACCURATE LIST OF BOTS
myCameraNumber = 1            #To be read from file later?
myPort = 5000                 #To be read from file later?

myCamera = Camera(myIp, myPort, myCameraNumber)

def addCamera(cameraIpInput, cameraPortInput, cameraNumberInput):
	wasFound = False
	for camera in myCamera.getAllNeighbors():
		if((camera.getAddress == cameraIpInput) and (camera.getPort == cameraPortInput)):
			wasFound = True
		if wasFound:
			print('Camera already registered with camera: ' + str(myCamera.getCameraNumber()))
			return 'ALREADY_INITIALIZED'
		if not wasFound:
			try:
				myCamera.addNeighbor(Camera(cameraIpInput, cameraPortInput, cameraNumberInput))
				return 'SUCCESS'
			except Exception as e:
				logging.error(traceback.format_exc())
				return 'NOT_INITIALIZED'

def addBot(botIpInput, botTypeInput, arucoNumberInput, botPortInput):
	wasFound = False
	for bot in botsList:
		if ((bot.getAddress() == botIpInput) and (bot.getPort() == botPortInput)):
			wasFound = True
	if wasFound:
		print('Bot already Exists in List')
		return 'ALREADY_INITIALIZED'
	if not wasFound:
		botsList.append(Bot(botIpInput, botTypeInput, 'INITIALIZED', arucoNumberInput, botPortInput, 'DEFAULT'))
		wasFound = False
		for bot in botsList:
			if bot.getAddress() == botIpInput:
				wasFound = True
		if wasFound:
			return 'SUCCESS'
		if not wasFound:
			return 'NOT_INITIALIZED'

def pollBots():
	global continuePollingBots
	global botsList
	while(continuePollingBots):
		for bot in botsList:
			url = 'http://' + bot.getAddress() + ':' + bot.getPort() + '/status/'
			print(url)
			try:
				#print(time.clock())
				r = requests.get(url)
				#print(time.clock())
				resp = r.json()
				print(resp)
			except Exception as e:
				print('Removing bot ' + bot.getAddress())
				botsList.remove(bot)
				#logging.error(traceback.format_exc())
		time.sleep(10)
		
def setThreadRunning(runBool):
	global continuePollingBots
	continuePollingBots = runBool
	if (continuePollingBots):
		pollThread = Thread(target=pollBots, )
		pollThread.setDaemon(True)
		pollThread.start()

def stressResult(requestIp):
	url = ('http://'+requestIp+':5000/results/stress/')
	payload = {'result' : 'success'}
	headers = {'content-type': 'application/json'}
	try:
		r=requests.post(url, data=json.dumps(payload), headers=headers)
		print(r)
	except Exception as e:
		logging.error(traceback.format_exc())

setThreadRunning(True)



##ENDPOINTS

@app.route('/', methods=['GET'])
def test():

	return jsonify({'message' : 'It works'})

@app.route('/', methods=['POST'])
def killThreads():
	global continuePollingBots
	global pollThread
	continuePollingBots = False
	print('Threads STOPPED')
	return jsonify([{'threadsStopped' : 'SUCCESS'}])

@app.route('/stress/', methods=['POST'])
def stressPoint():
	stressReceived = request.get_json(force=True)
	requestAddress = stressReceived['endPointAddress']
	resultThread = Thread(target=stressResult, args=(requestAddress, ))
	resultThread.setDaemon(True)
	resultThread.start()
	#whichThread = stressReceived['requestNum']
	#print('Stress received.')
	#return jsonify([{'requestNum' : 'SUCCESS'}])
	return '200'

@app.route('/bots/', methods=['POST'])
def newBotIntialize():

	newBotReceived = request.get_json(force=True)
	newAddress = newBotReceived['endPointAddress']
	newType = newBotReceived['botType']
	newAruco = newBotReceived['arucoNumber']
	newPort = newBotReceived['port']
	newType.rstrip('\n')
	return jsonify([{'endPointAddress' : newAddress, 'STATUS' : addBot(newAddress, newType, newAruco, newPort)}])
	
@app.route('/cameras/neighbor/', methods=['POST'])
def addNewNeighborCamera():
	return 'need to write this function'

@app.route('/requests/visibility/', methods=['POST'])
def isBotVisible():                   #Want to see if this camera can see a given bot (maybe poll neighbor cameras if it cannot see it)
	botRequested = request.get_json(force=True)
	botArucoNumber = botRequested['arucoNumber']
	print('Implement code to check if bot is visible to camera')
	return 'NOT_IMPLEMENTED'

@app.route('/cameras/massRelay/', methods=['POST'])
def massRelayInformation():
	print('Need to implement mass relay of information (pyramid?)')
	return 'NOT_IMPLEMENTED'

@app.route('/cameras/receiveRelay/', methods=['POST'])
def massRelayReceived():
	print('Need to implement mass relay received (Then repeating requests)')
	return 'NOT_IMPLEMENTED'

if __name__ == '__main__':

	if isSimulation:
		app.run(port=myPort)
	#app.run(debug=True, host=myIp)
	else:
		app.run(host=myIp, port=myPort)
