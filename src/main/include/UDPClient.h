
#include <stdio.h> 
#include <stdlib.h>
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>
#define PORT 9999
#define MAX_BUFF 100000   

class UDPClient{
    int bCastSock =0;
    int srcaddrSize;
    struct sockadrr_in localUDP;
    struct sockadrr_in bCastRecv;
    socklen_t addrlen = sizeof( sockadrr_in);
    char * buffHolder;
    char * hostIpString;
    const int bCastPort = 5801;
    string data;
    
    
    UDPClient();
    void setup_socket();

};