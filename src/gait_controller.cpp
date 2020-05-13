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
  ros::Publisher FRCp = n.advertise<std_msgs::Float64>(ns+"/"+"front_right_coxa"+cs, 1);
  ros::Publisher FLCp = n.advertise<std_msgs::Float64>(ns+"/"+"front_left_coxa" +cs, 1);
  ros::Publisher RRCp = n.advertise<std_msgs::Float64>(ns+"/"+"rear_right_coxa" +cs, 1);
  ros::Publisher RLCp = n.advertise<std_msgs::Float64>(ns+"/"+"rear_left_coxa"  +cs, 1);

  // Femurae
  ros::Publisher FRFp = n.advertise<std_msgs::Float64>(ns+"/"+"front_right_femur"+cs, 1);
  ros::Publisher FLFp = n.advertise<std_msgs::Float64>(ns+"/"+"front_left_femur" +cs, 1);
  ros::Publisher RRFp = n.advertise<std_msgs::Float64>(ns+"/"+"rear_right_femur" +cs, 1);
  ros::Publisher RLFp = n.advertise<std_msgs::Float64>(ns+"/"+"rear_left_femur"  +cs, 1);

  // Tibiae
  ros::Publisher FRTp = n.advertise<std_msgs::Float64>(ns+"/"+"front_right_tibia"+cs, 1);
  ros::Publisher FLTp = n.advertise<std_msgs::Float64>(ns+"/"+"front_left_tibia" +cs, 1);
  ros::Publisher RRTp = n.advertise<std_msgs::Float64>(ns+"/"+"rear_right_tibia" +cs, 1);
  ros::Publisher RLTp = n.advertise<std_msgs::Float64>(ns+"/"+"rear_left_tibia"  +cs, 1);

  ros::Rate r(publish_rate);
  double x = 0;
  bool state = true;

  while(n.ok()){
    ros::spinOnce();
    std_msgs::Float64 msg;

    if (state) x = 30; else x = 90;
    state = !state;

    msg.data = map(x, 0, 180, -1.7, 0.7);
    FLCp.publish(msg);
    msg.data = map(x, 0, 180, -2.0, 0.7);
    FLFp.publish(msg);
    msg.data = map(x, 0, 180, -0.5, 1.0);
    FLTp.publish(msg);

    msg.data = map(x, 0, 180, -0.7, 1.7);
    FRCp.publish(msg);
    msg.data = map(x, 0, 180, -2.0, 0.7);
    FRFp.publish(msg);
    msg.data = map(x, 0, 180, -0.5, 1.0);
    FRTp.publish(msg);

    msg.data = map(x, 0, 180, -1.7, 0.7);
    RRCp.publish(msg);
    msg.data = map(x, 0, 180, -2.0, 0.7);
    RRFp.publish(msg);
    msg.data = map(x, 0, 180, -0.5, 1.0);
    RRTp.publish(msg);

    msg.data = map(x, 0, 180, -0.7, 1.7);
    RLCp.publish(msg);
    msg.data = map(x, 0, 180, -2.0, 0.7);
    RLFp.publish(msg);
    msg.data = map(x, 0, 180, -0.5, 1.0);
    RLTp.publish(msg);

    ros::Duration(1.0).sleep();
    //r.sleep();
  }
}
