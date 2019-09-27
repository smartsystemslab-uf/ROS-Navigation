/*
 * SocketConnection.cpp
 *
 *	TCP Socket Client class
 *
 *  Created on: May 9, 2016
 *      Author: Streit FJ
 *      email: StreitFr50552@th-nuernberg.de
 */

#include "SocketConnection.h"

SocketConnection::SocketConnection() {
}

SocketConnection::~SocketConnection() {
	/* close socket */
	close(sock);
}

bool SocketConnection::setupServer(const char* robIP) {

	/* create Internet domain socket */
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		cout << "ERROR, no Connection established" << endl;
		return 0;
	}

	//server = gethostbyname("192.168.1.151"); // IP-address of rob1
	server = gethostbyname(robIP);

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
	}
	else
		cout << "Connection established " << endl;
	return 1;
}

bool SocketConnection::sendPath(char const* filename) {
	/* commenting out franz's code which reads the path from a file. we will not have a file
	 * that contains the most recent path. only file that contains all paths. i sent a string.c_str()
	 * nevermind. joe wants me to write it to a file.
	 */
	/* open the file to be sent */
	path = open(filename, O_RDONLY);

	if (path == -1) {
		cout << "ERROR unable to open or find file " << filename << endl;
		return 0;
	}

	/* get the size of the file to be sent */
	fstat(path, &stat_buf);

	offset = 0; // set offset to zero
	/* copy file using sendfile */
	for (size_t size_to_send = stat_buf.st_size; size_to_send > 0;) {
		ssize_t send = sendfile(sock, path, &offset, stat_buf.st_size);
			if (send == -1) {
				cout << "ERROR from sendfile" << endl;
				return 0;
			}
		offset += send;
		size_to_send -= send;
	}

	cout << "Done sending" << endl;
	/* close descriptor for file that was sent */
	close(path);

	return 1;
}
