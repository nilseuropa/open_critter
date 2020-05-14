#include <ros/ros.h>
#include <std_msgs/Float64.h>

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "gait_controller");
  ros::NodeHandle n;
  ros::NodeHandle nhLocal("~");

  double publish_rate;
  std::string ns;
  std::string cs = "/command";

  nhLocal.param("publish_rate", publish_rate, 10.0);
  nhLocal.param("namespace", ns, std::string("open_critter"));

  // Coxae
  ros::Publisher FR_COX_pub = n.advertise<std_msgs::Float64>(ns+"/"+"front_right_coxa"+cs, 1);
  ros::Publisher FL_COX_pub = n.advertise<std_msgs::Float64>(ns+"/"+"front_left_coxa" +cs, 1);
  ros::Publisher RR_COX_pub = n.advertise<std_msgs::Float64>(ns+"/"+"rear_right_coxa" +cs, 1);
  ros::Publisher RL_COX_pub = n.advertise<std_msgs::Float64>(ns+"/"+"rear_left_coxa"  +cs, 1);

  // Femurae
  ros::Publisher FR_FEM_pub = n.advertise<std_msgs::Float64>(ns+"/"+"front_right_femur"+cs, 1);
  ros::Publisher FL_FEM_pub = n.advertise<std_msgs::Float64>(ns+"/"+"front_left_femur" +cs, 1);
  ros::Publisher RR_FEM_pub = n.advertise<std_msgs::Float64>(ns+"/"+"rear_right_femur" +cs, 1);
  ros::Publisher RL_FEM_pub = n.advertise<std_msgs::Float64>(ns+"/"+"rear_left_femur"  +cs, 1);

  // Tibiae
  ros::Publisher FR_TIB_pub = n.advertise<std_msgs::Float64>(ns+"/"+"front_right_tibia"+cs, 1);
  ros::Publisher FL_TIB_pub = n.advertise<std_msgs::Float64>(ns+"/"+"front_left_tibia" +cs, 1);
  ros::Publisher RR_TIB_pub = n.advertise<std_msgs::Float64>(ns+"/"+"rear_right_tibia" +cs, 1);
  ros::Publisher RL_TIB_pub = n.advertise<std_msgs::Float64>(ns+"/"+"rear_left_tibia"  +cs, 1);

  ros::Rate r(publish_rate);
  double x = 0;
  bool state = true;

  while(n.ok()){
    ros::spinOnce();
    std_msgs::Float64 msg;

    if (state) x = 30; else x = 90;
    state = !state;

    msg.data = map(x, 0, 180, -1.0, 1.0);
    FL_COX_pub.publish(msg);
    msg.data = map(x, 0, 180, -2.0, 0.7);
    FL_FEM_pub.publish(msg);
    msg.data = map(x, 0, 180, -0.5, 1.0);
    FL_TIB_pub.publish(msg);

    msg.data = map(x, 0, 180, -1.0, 1.0);
    FR_COX_pub.publish(msg);
    msg.data = map(x, 0, 180, -2.0, 0.7);
    FR_FEM_pub.publish(msg);
    msg.data = map(x, 0, 180, -0.5, 1.0);
    FR_TIB_pub.publish(msg);

    msg.data = map(x, 0, 180, -1.0, 1.0);
    RR_COX_pub.publish(msg);
    msg.data = map(x, 0, 180, -2.0, 0.7);
    RR_FEM_pub.publish(msg);
    msg.data = map(x, 0, 180, -0.5, 1.0);
    RR_TIB_pub.publish(msg);

    msg.data = map(x, 0, 180, -1.0, 1.0);
    RL_COX_pub.publish(msg);
    msg.data = map(x, 0, 180, -2.0, 0.7);
    RL_FEM_pub.publish(msg);
    msg.data = map(x, 0, 180, -0.5, 1.0);
    RL_TIB_pub.publish(msg);

    ros::Duration(1.0).sleep();
    //r.sleep();
  }
}
