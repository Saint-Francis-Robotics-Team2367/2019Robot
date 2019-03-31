
#include <stdio.h> 
#include <stdlib.h>
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>
#include <iostream>
#include <sstream>
using namespace std; 
#define PORT 5801
#define MAX_BUFF 100000   

class UDPClient{

    public:
     int bCastSock = 0;
    int srcaddrSize;
    struct sockaddr_in localUdp; 
    struct sockaddr_in bCastRecv;
    socklen_t addrlen = sizeof(struct sockaddr_in); 
    char  *buffHolder; 
    char *hostIpString;
    const int bCastPort = 5801;
    char buffer[MAX_BUFF] = {0}; 
    struct sockaddr_in serv_addr; 
    ssize_t currPacket;
    
    UDPClient();
    void setup_socket();
    void read();
    int angle();
    int distance();

};