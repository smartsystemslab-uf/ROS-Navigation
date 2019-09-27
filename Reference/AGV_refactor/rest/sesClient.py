
from flask import Flask, jsonify, request
from threading import Thread

import json, pickle, requests, socket, time, threading, traceback, logging
from classDefs import  *



app = Flask(__name__)

isSimulation = False     #(Currently has issues) Set to True to run both camera and client on same local host IP

myPort=8887 #Maybe change this to read from file with a loop that sees if initialization is successful
if isSimulation:
	print('In Simulation Mode.')
	myIp='127.0.0.l'
else:
	myIp = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]
	print(myIp)


current_milli_time = lambda: int(round(time.time()*1000))

with open('botInfo.txt', 'rb') as rf:
	line = rf.readline()
	myType = line.rstrip('\n')
	line = rf.readline()
	myAruco = line.rstrip('\n')

with open('hostIP.txt', 'rb') as rf:
	line = rf.readline()
	myHost = line.rstrip('\n')

refTime = 0
firstCheckTime = 0


myBot = Bot(myIp, myType, 'NOT_INITIALIZED', myAruco, myPort, myHost)

def requestLocationCheck():
	global myHost
	global myBot	
	lastKnownCamera = myBot.getLastKnownCamera()
	arucoNumber = myBot.getArucoNumber()
	url = 'http://' + lastKnownCamera + '/requests/visibility/'
	payload = {'endPointAddress' : mybot.getAddress(), 'arucoNumber' : arucoNumber}

def selfInitialize(repeat, isRepeating):
	global myHost
	global myBot
	if isRepeating:
		print('repeating the self initialize thing. times left to repeat: ' + str(repeat-1))
	url = 'http://' + myHost + '/bots/'
	payload = {'endPointAddress' : myBot.getAddress(), 'botType' : myBot.getBotType(), 'arucoNumber' : myBot.getArucoNumber(), 'port' : myBot.getPort()}
	headers = {'content-type': 'application/json'}

	try:
		r = requests.post(url, data=json.dumps(payload), headers=headers)
		resp = r.json()
		myBot.setStatus(resp[0]['STATUS'])
		print(resp)
		if((myBot.getStatus() == 'NOT_INITIALIZED') and (repeat > 1)):
			selfInitialize(repeat-1, True)
		elif((myBot.getStatus == 'NOT_INITIALIZED') and (repeat <= 1)):
			myBot.setStatus('NOT_INITIALIZED')
		else:
			myBot.setStatus('INITIALIZED')
	except Exception as e:
		#print('Couldn't reach the server. Trying again.')
		logging.error(traceback.format_exc())

def simpleRequest(address, requestNum):
	global refTime
	global firstCheckTime
	global myHost
	url = 'http://' + myHost + '/stress/'
	#print(address, requestNum)
	payload = {'endPointAddress' : myBot.getAddress(), 'port' : myBot.getPort(), 'botType' : myBot.getBotType(), 'arucoNumber' : myBot.getArucoNumber(), 'requestNum' : requestNum}
	headers = {'content-type': 'application/json'}
	try:
		refTime = current_milli_time()
		#print('refTime: ' + str(refTime))
		r = requests.post(url, data=json.dumps(payload), headers=headers)
		firstCheckTime = current_milli_time()
		#print('firstTime: ' + str(firstCheckTime))
		#print(r)
		#resp = r.json()
		#myBot.setStatus(resp[0]['requestNum'])
		#print(resp)
	except Exception as e:
		print('Could not reach the camera. Try again.')
		logging.error(traceback.format_exc())

def stressCamera(address, numRequests):
	for _ in range(numRequests):
		pollThread = Thread(target=simpleRequest, args=(address, _))
		pollThread.setDaemon(True)
		pollThread.start()
			

selfInitialize(2, False)
#stressCamera(myHost, 100)

while(myBot.getStatus() == 'NOT_INITIALIZED'):
	time.sleep(5)
	selfInitialize(2, False)

##ENDPOINTS
@app.route('/', methods=['GET'])
def test():

	return jsonify({'Bot: ' : myBot.getBotType(), 'EndPoint ' : myBot.getAddress()})
@app.route('/results/stress/', methods=['POST'])
def stressResult():
	global refTime
	global firstCheckTime
	curTime = current_milli_time()
	#print('refInside: ' + str(refTime))
	#print('firstInside: ' + str(firstCheckTime))
	#print('curtime is: '+ str(curTime))
	firstCheckDelta = firstCheckTime-refTime
	completeRequestDelta = curTime-refTime
	print('First Check Delta: ' + str(firstCheckDelta) + '. Complete Request Delta: ' + str(completeRequestDelta))
	resultReceived = request.get_json(force=True)
	print(resultReceived)
	return '200'

@app.route('/status/', methods=['GET'])
def requestBotStatus():

	return jsonify([{'endPointAddress' : myBot.getAddress(), 'port' : myBot.getPort(), 'botType' : myBot.getBotType(), 'status' : myBot.getStatus()}])

@app.route('/host/', methods=['PUT'])
def setHostAddress():
	hostAddressReceived = request.get_json(force=True)

	myHost.setAddress(hostAddressReceived['hostAddress'])
	
	return jsonify({'BotAddress' : myBot.getAddress(), 'STATUS' : 'SUCCESS', 'RecievedAddress' : myHost.getAddress()})



if __name__ == '__main__':
	
	#app.run(debug=True, port=myPort)
	if isSimulation:
		app.run(port=myPort)
	else:
		#app.run(debug=True, host=myIp, port=myPort)
		app.run(host=myIp, port=myPort)
