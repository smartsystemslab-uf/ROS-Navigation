/*
 * SocketConnection.h
 *
 *  Created on: May 9, 2016
 *      Author: Streit FJ
 *      email: StreitFr50552@th-nuernberg.de
 */

#ifndef SOCKETCONNECTION_H_
#define SOCKETCONNECTION_H_

#include <iostream>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <netdb.h>

#define portno 60000 	// port number to use @Joe maybe /* FIXME */

using namespace std;

class SocketConnection {
	public:
		SocketConnection();
		virtual ~SocketConnection();
		bool setupServer(const char* robIP);
		bool sendPath(char const*);

	private:
		int sock;                           // socket desciptor
		int desc;                           // file descriptor for socket
		int path;                           // file descriptor for file to send
		off_t offset; 						// file offset
		struct stat stat_buf;               // argument to fstat
		struct sockaddr_in serv_addr;
		struct hostent *server;
};

#endif /* SOCKETCONNECTION_H_ */
