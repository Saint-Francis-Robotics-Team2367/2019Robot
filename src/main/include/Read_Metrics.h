#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#define PORT 8888
#define MAX_BUFF 100000

using namespace std;

class read_metrics{

    public:
    read_metrics();
    int setUpTCP();

    int read_dist(int dist);
    int read_angle(int angle);
    void configDistance();
    
    private:
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    const char *hello = "Hello from client";
    char buffer[MAX_BUFF] = {0};
    int bCastSock = 0;
    int srcaddrSize;
    struct sockaddr_in localUdp; 
    struct sockaddr_in bCastRecv;
    socklen_t addrlen = sizeof(struct sockaddr_in); 
    char  *buffHolder; 
    char *hostIpString;
    const int bCastPort = 9999;

   


};

