
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <map>
#include <list>
#include <cmath>
#include <vector>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <errno.h>

#include <cstring>


#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;


#define portno 60000 // port number to use
#define buf_size 4096 // max file size to receive


int rc; // holds return code of system calls
int sock; // socket desciptor
int desc; // file descriptor for socket

char const* fr_name = "./path.txt"; // path to file
	
	
	struct hostent *server;
	struct sockaddr_in serv_addr; 

/********************************************************
 *
 * Create socket and bind if successful created
 *
 *********************************************************/
void bindSocket() {
    struct sockaddr_in serv_addr;       // structure containing server address

    /* enable keep-alive on the socket */
    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

    int idletime = 120; /* in seconds */
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idletime, sizeof(idletime));

    /* First call to socket() function */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        fprintf(stderr, "unable to create socket: %s\n", strerror(errno));
        exit(1);
    }

    /* fill in socket structure */
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);

    /* Now bind the host address using bind() call.*/
    rc = bind(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    if (rc == -1) {
        fprintf(stderr, "unable to bind to socket: %s\n", strerror(errno));
        exit(1);
    }
}

bool setupServer(string cam) {

	/* create Internet domain socket */
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		cout << "ERROR, no Connection established" << endl;
		return 0;
	}

	const char* cludgeTemp = cam.c_str();
	server = gethostbyname(cludgeTemp); // enter IP-address of Raspberry network 

	if (server == NULL) {
		cout << "ERROR, no such server" << endl;
		return 0;
	}

	bzero((char *) &serv_addr, sizeof(serv_addr)); // sets all values of the client address to zero
	serv_addr.sin_family = AF_INET; // set address family
	serv_addr.sin_port = htons(portno); // set port number

	bcopy((char *) server->h_addr,  // copy new values into buffer
	(char *)&serv_addr.sin_addr.s_addr,
	server->h_length);

	/* connect to server*/
	
	if (connect(sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		cout << "ERROR connecting" << endl;
		return 0;
	} else
		cout << "Connection established " << endl;

	return 1;
}

/********************************************************
 *
 * write receiving data into text-file
 *
 *********************************************************/
void receiveData() {

    FILE *data = fopen(fr_name, "w"); // open txt file in read write mode rewrite the file every time

    char revbuf[buf_size]; // receive buffer for file
    if (data == NULL) {
        fprintf(stderr, "unable to open '%s': %s\n", data, strerror(errno));
        exit(1);
    } else {
        bzero(revbuf, buf_size);
        int fr_block_sz = 0;
        while ((fr_block_sz = recv(sock, revbuf, sizeof(revbuf), 0)) > 0) {
            int write_sz = fwrite(revbuf, sizeof(char), fr_block_sz, data);
            if (write_sz < fr_block_sz) {
                cout << "File write failed." << endl;
            }
            bzero(revbuf, buf_size);
            if (fr_block_sz == 0 || fr_block_sz != buf_size) {
                break;
            }
        }
        if (fr_block_sz < 0) {
            if (errno == EAGAIN) {
                printf("recv() timed out.\n");
            } else {
                fprintf(stderr, "recv() failed due to errno = %d\n", errno);
            }
        }
        cout << "Writing successfully done!" << endl;
        fclose(data);
    }
}


/********************************************************
 *
 * read data from text-file and write into vector
 *
 *********************************************************/
vector<u_char> readData() {
    char * line = NULL;
    size_t len = 0;
    FILE *path;
	vector<u_char> pathVector;

    path = fopen(fr_name, "r");
    if (path == NULL) {
        fprintf(stderr, "unable to open '%s': %s\n", path, strerror(errno));
        exit(1);
    }

    // read txt file line by line and write it into vector
    while ((getline(&line, &len, path)) != -1) {
        //read file into array
        pathVector.push_back(*line); //use vector for easier processing
    }

    fclose(path);
    cout << "close File" << endl;
    if (line)
        free(line);
	return pathVector;
}


void sendInt( int propComms, int value){
	char a[sizeof(int)];
	
	
	sprintf(a, "%d", value);
	for(int i = 0; i < sizeof(int); i++){
		serialPutchar(propComms, a[i]);
	}
}

int getInt(int propComms){
	char a[sizeof(int)];
	int returnValue;
	for(int i = 0; i < sizeof(int); i++){
			a[i] = serialGetchar(propComms);

		}
	sscanf(a, "%d", &returnValue);

return returnValue;
}

int setPath(int propComms, vector<u_char> path ){
	int numSteps;

	//digitalWrite(1, LOW);
  	digitalWrite(23, HIGH);
  	digitalWrite(24, HIGH);

	//serialFlush(propComms);

	int catchKey = serialGetchar(propComms);

//	int wtf = atoi(catchKey);
//	cin >> catchKey;
	cout << "char received indicating propellor serial ready : ";
	cout << (char) catchKey << endl;
	int    leftTicks = getInt(propComms);
	int    rightTicks = getInt(propComms);
		cout << "left : " << leftTicks << " right : " << rightTicks << endl;

	digitalWrite(23, LOW);
    	digitalWrite(24, LOW);

	for(int i = 0; i < path.size(); i++){
		serialPutchar(propComms, path.at(i));
	}

	

	serialPutchar(propComms, '9');

 

	 numSteps = getInt(propComms) ;

//	int wtf = atoi(catchKey);
//	cin >> catchKey;
	cout << "Numsteps returned by propellor : ";
	cout << numSteps << endl;

	serialPutchar(propComms, '9');

	return numSteps;
}

void runPath(int propComms, vector<u_char> path){


	digitalWrite(1, HIGH);
	int catchKey;
	int leftTicks;
	int rightTicks;


	for(int i = 0; i < path.size()+1; i++){
	
	
	    catchKey = serialGetchar(propComms);
	    cout << "Value returned by propellor : ";
	    cout <<(char) catchKey << endl;

	    leftTicks = getInt(propComms);
	    rightTicks = getInt(propComms);
		cout << "left : " << leftTicks << " right : " << rightTicks << endl;
 	}


	digitalWrite(1, LOW);



	//serialFlush(propComms);

}

void closeSerial(int propComms){

	
	serialFlush(propComms);

//	printf("%c - %d", catchKey, catchKey);
	serialClose(propComms);

	cout << "closed serial \n";

}


int main(){



//
// serial test
//
//
	wiringPiSetup();
	pinMode(1, OUTPUT);
	pinMode(23, OUTPUT);
	pinMode(24, OUTPUT);

	digitalWrite(1, LOW);
  	digitalWrite(23, LOW);
  	digitalWrite(24, LOW);


	int propComms = serialOpen("/dev/ttyUSB0", 115200);

	if( propComms == -1 )
		cout << "Failed to open serial port. :( \n";



	serialFlush(propComms);

//start of client sockets
/*
	int fd_path; // filedescriptor for path
    struct sockaddr_in client_addr; // structure containing client address
    unsigned int clilen = sizeof(client_addr);

      bindSocket();

        rc = listen(sock, 1); // listening for the client only one client is allowed
        if (rc == -1) {
            fprintf(stderr, "listen failed: %s\n", strerror(errno));
            exit(1);
        }

   desc = accept(sock, (struct sockaddr *) &client_addr, &clilen);
        if (desc == -1) {
            fprintf(stderr, "accept failed: %s\n", strerror(errno));
            exit(1);
        } else {
            cout << "Connection accept from address "
                 << inet_ntoa(client_addr.sin_addr) << endl;
        }
*/

//end of client socket s
	string cam[4];
	cam[0] = "";
	cam[1] = "192.168.1.146";
	cam[2] = "192.168.1.141";
	cam[3] = ""; //"192.168.1.144"; // currently on the ceiling

	string taskGoal = "";
	char detected[4] = {0};
	int nextCam = -1;
	int currentCam = -1;
	int goalCam = -1;
	bool wasDetected = false;
	int xGoal;
	int yGoal;

	bool isConnected = false;
	
	cout << "Server Socket setup" << endl;


	cout << "Enter a destination ( Camera,X,Y ) with no spaces :";
		cin >> taskGoal;
		taskGoal += ","; // evil socket delimiter bull shit buffer overflow 2000 goals


	while( taskGoal != "-1" ){

		int j = 0;
		for( int i = 0; i < 3; i++){ // tokenize the taskGoal string
			string s;
			while(taskGoal[j] != ',' && j < strlen(taskGoal.c_str())) {
				s += taskGoal[j];
				j++;
			}
			cout << "S[" << i << "] = " << s << endl;
			j++; // discard the ','
			switch(i) {

				case 0:
					goalCam = atoi(s.c_str());
					break;
				case 1:
					xGoal = atoi(s.c_str());
					break;
				case 2:
					yGoal = atoi(s.c_str());
					break;
			}
		}

		// connect to the goal camera first, in case we are already in his region
		if( goalCam == 1){
			isConnected = setupServer(cam[goalCam]);
			currentCam = goalCam;
			nextCam = 2;
		}
		else{
			isConnected = setupServer(cam[goalCam]);
			currentCam = goalCam;
			nextCam = 1;
		}
/*	
		while( !isConnected ){
			cout << "Failed to connect to cam1, attempting to connect to cam2" << endl;
			isConnected = setupServer(cam2);
			currentCam = 2;
			if( !isConnected ) {
				cout << "Failed to connect to cam2, attempting to connect to cam1" << endl;
				isConnected = setupServer(cam1);
				currentCam = 1;
			}

		}
*/
		send(sock, taskGoal.c_str(), strlen(taskGoal.c_str()), 0); // send a msg to goalCam to see if it can see me

		int recv_sz = 0;
		recv_sz = recv(sock, detected, sizeof(detected), 0); // receive the response from goalCam
		 j = 0;
		for( int i = 0; i < 2; i++){
			string s;
			while(detected[j] != ',' && j < 3) {
				s += detected[j];
				if (detected[j] == '-')
					s+= detected[j+1];				
				j++;
			}
			cout << "S[" << i << "] = " << s << endl;
			j++; // discard the ','
			switch(i) {

				case 0:
					wasDetected = s[0] == 'T';
					break;
				case 1:
					if (wasDetected)
						nextCam = atoi(s.c_str());
					break;
			}
		}
		cout << "nextCam: " << nextCam << endl;

		bool taskComplete = false;

	while( !taskComplete){

		if( !wasDetected ){ // if the goal camera didn't detect me
			close(sock);
			
			isConnected = setupServer(cam[nextCam]);
			currentCam = nextCam;
			nextCam = goalCam;
			send(sock, taskGoal.c_str(), strlen(taskGoal.c_str()), 0); // send taskGoal to other cam
		
			recv_sz = recv(sock, detected, sizeof(detected), 0); // receive his response

			 j = 0;
			for( int i = 0; i < 2; i++){ // tokenize
				string s;
				while(detected[j] != ',' && j < 3) {
					s += detected[j];
					if (detected[j] == '-')
						s+= detected[j+1];
					j++;
				}
				cout << "S[i] = " << s << endl;
				j++; // discard the ','
				switch(i) {

					case 0:
						wasDetected = s[0] == 'T';
						break;
					case 1:
						if (wasDetected)
							nextCam = atoi(s.c_str());
						break;
				}
			}
			cout << "was not detected and nextCam: " << nextCam << endl;
			
		} 
		else {	// if goal camera detected me
			const char* imReady = "T";
			cout << "I'm asking ceiling camera for path" << endl;
			send(sock, imReady, strlen(imReady), 0); // ask for the path
			receiveData();
			vector<u_char> path;
			path.clear();
			path = readData();
			u_char extraStepForward = '0';


			vector<u_char> finalPath;
			cout << " Received path : " ;
			for(int i = 0; i < path.size(); i++){
				cout << path.at(i);			
			}
			int i = 0;
				while( i < path.size() && (path.at(i) == '0' || path.at(i) == '1' || path.at(i) == '7' ) ){
						finalPath.push_back(path.at(i));
						i++;				
					} 

			finalPath.push_back(extraStepForward);//manual append of a step forward for transitions between cameras
			cout << endl << " Filtered path : " ;
			for(int i = 0; i < finalPath.size(); i++){
				cout << finalPath.at(i);			
			}


			cout << "path.size() = " << finalPath.size() << endl;
			int numSteps = setPath(propComms, finalPath);
			runPath(propComms, finalPath);
			if( nextCam == -1 )
				taskComplete = true;
			wasDetected = false;
			cout << "was detected and nextCam: " << nextCam << endl;
		}	
			
	}// end of while ( currentCam != goalCam)		
		close(sock);

		cout << "Enter a destination ( Camera,X,Y ) with no spaces :";
		cin >> taskGoal;	
		taskGoal += ","; // evil socket delimiter bull shit buffer overflow 2000 goals
	}//End of while( taskGoal != "-1" ) loop

	closeSerial(propComms);

 	
//
return 0;
}
