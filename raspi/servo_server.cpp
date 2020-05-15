#include <PCA9685.h>
#include <protocol.h>
#include <iostream>
#include <stdio.h>
#include <thread>
#include <vector>
#include <string>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MIN_PULSE_WIDTH 900
#define MAX_PULSE_WIDTH 2100
#define FREQUENCY 50
#define BUFFERSIZE 1024

using namespace std;

void error(char const *msg) {
  perror(msg);
  exit(1);
}

int map (int x, int in_min, int in_max, int out_min, int out_max) {
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int set_joint(PCA9685 pwm, int channel, int angle ) {
    if (angle > 180) { angle = 180; }
    if (angle < 0) { angle = 0; }
    float pulse_wide = map(angle,0,180,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    int analog_value = int(float(pulse_wide) /  1000000 * FREQUENCY * 4096);
    pwm.setPWM(channel, 0, analog_value);
    cout << "Joint: " << channel << "\tAngle: " << angle << "\tAnalog: " << analog_value << endl;
    return(0);
}

int main(int argc, char **argv) {

  PCA9685 pwm;
  ServoStatePacket singleJoint;
  RobotStatePacket allJoints;
  int dgram_socket;
  int port_number;
  socklen_t client_addr_len;
  struct sockaddr_in server_address;
  struct sockaddr_in client_address;
  struct hostent *host_info;
  char message_buffer[BUFFERSIZE];
  char *dot_dec_host_address;
  int socket_option_flag;
  int n;

  if (argc != 2) {
    fprintf(stderr, "usage: %s <port>\n", argv[0]);
    exit(1);
  }
  port_number = atoi(argv[1]);

  dgram_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (dgram_socket < 0) error("ERROR opening socket");

  socket_option_flag = 1;
  setsockopt(dgram_socket, SOL_SOCKET, SO_REUSEADDR, (const void *)&socket_option_flag , sizeof(int));

  bzero((char *) &server_address, sizeof(server_address));
  server_address.sin_family = AF_INET;
  server_address.sin_addr.s_addr = htonl(INADDR_ANY);
  server_address.sin_port = htons((unsigned short)port_number);

  if (bind(dgram_socket, (struct sockaddr *) &server_address, sizeof(server_address)) < 0) error("ERROR on binding");
  client_addr_len = sizeof(client_address);
	cout << "Server started." << endl;

	pwm.init(1,0x40);
	usleep(1000 * 100);
	cout << "PWM frequency: " << FREQUENCY << endl;
	pwm.setPWMFreq (FREQUENCY);

  while (1) {

    bzero(message_buffer, BUFFERSIZE);
    n = recvfrom(dgram_socket, message_buffer, BUFFERSIZE, 0, (struct sockaddr *) &client_address, &client_addr_len);

    if (n < 0) error("ERROR in recvfrom");
    host_info = gethostbyaddr((const char *)&client_address.sin_addr.s_addr, sizeof(client_address.sin_addr.s_addr), AF_INET);
    if (host_info == NULL) error("ERROR on gethostbyaddr");
    dot_dec_host_address = inet_ntoa(client_address.sin_addr);
    if (dot_dec_host_address == NULL) error("ERROR on inet_ntoa\n");

    // no error
    printf("Server received datagram from %s (%s)\n", host_info->h_name, dot_dec_host_address);
    printf("Server received %d/%d bytes.\r\n", strlen(message_buffer), n);

    switch (message_buffer[PID]){
      case PID_SET_SERVO:
        memcpy(&singleJoint, &message_buffer, sizeof(ServoStatePacket));
        set_joint(pwm, singleJoint.jointId-1, singleJoint.angle);
      break;
      case PID_SET_ALL_SERVOS:
        memcpy(&allJoints, &message_buffer, sizeof(RobotStatePacket));
      break;
    }

    // response
    n = sendto(dgram_socket, message_buffer, strlen(message_buffer), 0, (struct sockaddr *) &client_address, client_addr_len);
    if (n < 0) error("ERROR in sendto");
  }

	return 0;
};
