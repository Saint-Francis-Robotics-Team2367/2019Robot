#include "Read_Metrics.h"

read_metrics::read_metrics(){
    
}

int read_metrics::setUpTCP(){
     if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    } 
    //setup udp sock
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
    //wait for beacon from server
    do{
        cout << "Waiting for broadcast..." <<endl;
        memset(&bCastRecv, '0', sizeof(bCastRecv));
        recvfrom(bCastSock, buffer, MAX_BUFF, 0, (struct sockaddr *) &bCastRecv, &addrlen);
    }while(strcmp("Host", buffer));

    hostIpString = inet_ntoa(bCastRecv.sin_addr);

   //setup and connect to tcp sock
    memset(&serv_addr, '0', sizeof(serv_addr)); 
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, hostIpString, &serv_addr.sin_addr)<=0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
        return -1; 
    } 
    cout << "connecting to ip: " << hostIpString << endl;
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n"); 
        return -1; 
    } 
    printf("Connected to server!");

}

int read_metrics::read_dist(int dist){
    read(sock, &dist, sizeof(int));
    return dist;
}

int read_metrics::read_angle(int angle){
    read(sock, &angle, sizeof(int));
    return angle;
}






