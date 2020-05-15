#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <open_critter/protocol.h>

void error(char *msg) {
    perror(msg);
    exit(0);
}

int main(int argc, char **argv) {
    int sockfd, portno, n, joint_id;
    double joint_state;
    int serverlen;
    ServoStatePacket packet;
    struct sockaddr_in serveraddr;
    struct hostent *server;
    char *hostname;

    if (argc != 5) {
       fprintf(stderr,"Usage: %s <hostname> <port> <joint_id> <angle>\n", argv[0]);
       exit(0);
    }
    hostname = argv[1];
    portno = atoi(argv[2]);
    joint_id = atoi(argv[3]);
    joint_state = atof(argv[4]);

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) error("ERROR opening socket");

    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host as %s\n", hostname);
        exit(0);
    }

    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
	  (char *)&serveraddr.sin_addr.s_addr, server->h_length);
    serveraddr.sin_port = htons(portno);

    printf("Setting joint: %d to %f\r\n", joint_id, joint_state);
    packet.header.packetId = PID_SET_SERVO;
    packet.jointId = joint_id;
    packet.state   = joint_state;
    uint8_t buf[sizeof(ServoStatePacket)];
    memcpy( buf, &packet, sizeof(ServoStatePacket) );

    serverlen = sizeof(serveraddr);
    n = sendto(sockfd, buf, sizeof(buf), 0, &serveraddr, serverlen);
    if (n < 0) error("ERROR in sendto");

    n = recvfrom(sockfd, buf, sizeof(buf), 0, &serveraddr, &serverlen);
    if (n < 0) error("ERROR in recvfrom");

    printf("Echo received.\r\n");
    return 0;
}
