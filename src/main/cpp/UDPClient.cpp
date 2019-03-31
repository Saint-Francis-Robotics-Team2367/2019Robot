#include "UDPClient.h"

/*UDPClient::UDPClient(){
    cout<<"stuff";
}*/

void UDPClient::read(){

   do{
        cout << "Waiting for broadcast..." <<endl;
        memset(&bCastRecv, '0', sizeof(bCastRecv));
        currPacket = recvfrom(bCastSock, buffer, MAX_BUFF, 0, (struct sockaddr *) &bCastRecv, &addrlen);
    }while(currPacket != -1);
     
}

void UDPClient::setup_socket(){
    if ((bCastSock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) 
    { 
        perror("UDP socket failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (setsockopt(bCastSock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &srcaddrSize, sizeof(srcaddrSize))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    memset(&localUdp, '0', sizeof(serv_addr)); 
    localUdp.sin_family = AF_INET; 
    localUdp.sin_addr.s_addr = INADDR_ANY;
    localUdp.sin_port = htons( bCastPort ); 

    bind(bCastSock, (struct sockaddr *)&localUdp,sizeof(localUdp)); 
}

int UDPClient::angle(){
    int ang, d;
    istringstream iss(buffer);
    iss >> ang >> d;
    return ang;
}

int UDPClient::distance(){
    int ang, d;
    istringstream iss(buffer);
    iss >> ang >> d;
    return d;
}