#pragma once

#include "sockpp/tcp_acceptor.h"

class ScannerClient
{
public:
	ScannerClient(sockpp::tcp_socket *socket);
	~ScannerClient();

	sockpp::tcp_socket *socket; // Socket the client is connected on

	bool send = false;			// Indicates if we should be sending things to this client (client has made an API request for send)
	std::string nameFilter;		// If a name filter is set, this is the substring to check for
	std::string macFilter;		// If a MAC filter is set, this is the partial MAC address to check for (starting at the beginning)
};

