#include "UDPClient.h"

UDPClient:: UDPClient(){
    setup_socket(); 

}

void UDPClient:: setup_socket(){
    if ( (bCastSock = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("UDP socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 

    if ( setsockopt(bCastSock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &srcaddrSize, sizeof(srcaddrSize)) ) { 
        perror("UDP socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
  
    memset(&localUDP,0, sizeof(serv_addr)); 
      
    // Filling server information 
    localUDP.sin_family = AF_INET; 
    localUDP.sin_port = htons(bCastPort); 
    localUDP.sin_addr.s_addr = INADDR_ANY; 

    bind(bCastSock, (struct sockaddr *)&localUDP, sizeof(localUDP));

    do{
        memset(&bCastRecv, '0', sizeof(bCastRecv));
        recvfrom(BCastSock, buffer, MAX_BUFF, 0, (struct sockaddr*) &bCastRecv, &addrlen);
    }  while(strcmp(data), buffer);
     

}