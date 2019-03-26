
#include <stdio.h> 
#include <stdlib.h>
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
  

#define PORT 5353
#define MAX_BUFF 100000   

class UDPClient{
    int sockfd; 
    char buffer[MAX_BUFF]; 
    char *hello = "Hello from client"; 
    struct sockaddr_in   servaddr; //is this corredct MRP help me
    
    UDPClient();
    void setup_socket();

};