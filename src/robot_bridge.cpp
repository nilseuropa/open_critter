#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <open_critter/protocol.h>

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};

void callback(const std_msgs::Float64::ConstPtr &msg, const int &jointId) {
    ROS_INFO_STREAM("Joint id: " << jointId << "\t Norm:" << msg->data);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_bridge");
  ros::NodeHandle n;
  ros::NodeHandle nhLocal("~");

  double publish_rate;
  std::string ns;
  std::string cs = "/command";

  nhLocal.param("namespace", ns, std::string("open_critter"));

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
  return 0;
}
