#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <open_critter/protocol.h>

int sockfd, portno, serverlen;
struct sockaddr_in serveraddr;
struct hostent *server;

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};

void callback(const std_msgs::Float64::ConstPtr &msg, const int &jointId) {
    ROS_INFO_STREAM("Joint id: " << jointId << "\t Norm:" << msg->data);
    ServoStatePacket packet;
    packet.header.packetId = PID_SET_SERVO;
    packet.jointId = jointId;
    packet.state   = msg->data;
    uint8_t buf[sizeof(ServoStatePacket)];
    memcpy( buf, &packet, sizeof(ServoStatePacket) );
    int ret = sendto(sockfd, buf, sizeof(buf), 0, (struct sockaddr *)&serveraddr, sizeof(serveraddr));
    if (ret < 0) ROS_ERROR("Can not send.");
}

// TODO: joint state update

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_bridge");
  ros::NodeHandle n;
  ros::NodeHandle nhLocal("~");

  std::string cs = "/command";
  std::string ns;
  std::string hostname;

  nhLocal.param("namespace", ns, std::string("open_critter"));
  nhLocal.param("hostname", hostname, std::string("zero"));
  nhLocal.param("portnumber", portno, 9001);

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) ROS_ERROR("ERROR opening socket");

  server = gethostbyname(hostname.c_str());
  if (server == NULL) {
      ROS_ERROR("ERROR, no such host as %s\n", hostname.c_str());
      exit(0);
  }

  bzero((char *) &serveraddr, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
  (char *)&serveraddr.sin_addr.s_addr, server->h_length);
  serveraddr.sin_port = htons(portno);

  // Coxae
  ros::Subscriber FR_COX_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"front_right_coxa"+cs, 1, boost::bind(callback, _1, FR_Coxa));
  ros::Subscriber FL_COX_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"front_left_coxa"+cs,  1, boost::bind(callback, _1, FL_Coxa));
  ros::Subscriber RR_COX_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"rear_right_coxa"+cs,  1, boost::bind(callback, _1, RR_Coxa));
  ros::Subscriber RL_COX_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"rear_left_coxa"+cs,   1, boost::bind(callback, _1, RL_Coxa));

  // Femurae
  ros::Subscriber FR_FEM_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"front_right_femur"+cs, 1, boost::bind(callback, _1, FR_Femur));
  ros::Subscriber FL_FEM_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"front_left_femur"+cs,  1, boost::bind(callback, _1, FL_Femur));
  ros::Subscriber RR_FEM_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"rear_right_femur"+cs,  1, boost::bind(callback, _1, RR_Femur));
  ros::Subscriber RL_FEM_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"rear_left_femur"+cs,   1, boost::bind(callback, _1, RL_Femur));

  // Tibiae
  ros::Subscriber FR_TIB_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"front_right_tibia"+cs, 1, boost::bind(callback, _1, FR_Tibia));
  ros::Subscriber FL_TIB_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"front_left_tibia"+cs,  1, boost::bind(callback, _1, FL_Tibia));
  ros::Subscriber RR_TIB_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"rear_right_tibia"+cs,  1, boost::bind(callback, _1, RR_Tibia));
  ros::Subscriber RL_TIB_sub = n.subscribe<std_msgs::Float64>(ns+"/"+"rear_left_tibia"+cs,   1, boost::bind(callback, _1, RL_Tibia));

  ros::spin();
  // while(n.ok()){
  //   ros::spinOnce();
  //   int ret = recvfrom(sockfd, buf, sizeof(buf), 0, (struct sockaddr *)&serveraddr, &serverlen);
  //   if (ret < 0) ROS_ERROR("Can not receive.");
  // }
  return 0;
}
