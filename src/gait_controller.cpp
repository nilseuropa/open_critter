#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <open_critter/protocol.h>

std::vector<ros::Publisher> joint_publishers;

std_msgs::Float64 msg[12];

std::string topics[12] = {
  "front_left_coxa",  "front_left_femur",   "front_left_tibia",
  "front_right_coxa", "front_right_femur",  "front_right_tibia",
  "rear_right_coxa",  "rear_right_femur",   "rear_right_tibia",
  "rear_left_coxa",   "rear_left_femur",    "rear_left_tibia"
};

float up    = -0.5;
float down  =  0.5;
float reach = -0.5;
float push  =  0.5;
float mid   =  0.0;

float state[12] = {
// C     F     T
   0.0,  0.0,  0.0, // FL
   0.0,  0.0,  0.0, // FR
   0.0,  0.0,  0.0, // RR
   0.0,  0.0,  0.0  // RL
};

float sequence[4][12] = {
   {
      reach,  up,  up,
      reach,  up,  up,
      reach,  up,  up,
      reach,  up,  up
   },
   {
      reach,  down,  down,
      reach,  down,  down,
      reach,  down,  down,
      reach,  down,  down
   },
   {
      push,  down,  down,
      push,  down,  down,
      push,  down,  down,
      push,  down,  down
   },
   {
      push,  up,  up,
      push,  up,  up,
      push,  up,  up,
      push,  up,  up
   }
};

void publish_joint_states(float stm[])
{
  for (uint8_t i=0; i<12; i++) {
         // hardware inverted joints
         if (i==FL_Femur) msg[i].data=-stm[i];
    else if (i==FR_Tibia) msg[i].data=-stm[i];
    else if (i==RR_Femur) msg[i].data=-stm[i];
    else if (i==RL_Tibia) msg[i].data=-stm[i];
    // inverse operation
    else if (i==FR_Coxa) msg[i].data=-stm[i];
    else if (i==RR_Coxa) msg[i].data=-stm[i];
    //
    else msg[i].data = stm[i];
    joint_publishers[i].publish(msg[i]);
  }
}

void initialize_joints()
{
  for (uint8_t i=0; i<12; i++) state[i] = 0.0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gait_controller");
  ros::NodeHandle n;
  ros::NodeHandle nhLocal("~");

  std::string robot_name;
  std::string cmd_str = "/command";
  double publish_rate;

  nhLocal.param("namespace", robot_name, std::string("open_critter"));
  nhLocal.param("publish_rate", publish_rate, 1.0);

  for (uint8_t i=0; i<12; i++) {
    std::stringstream ss;
    ss << robot_name << "/" << topics[i] << cmd_str;
    joint_publishers.push_back( n.advertise<std_msgs::Float64>(ss.str(), 50));
  }

  bool initialized = false;
  while(n.ok()&&!initialized){
    ros::spinOnce();
    initialize_joints();
    publish_joint_states(state);
    ros::Duration(1.0).sleep();
    initialized = true;
  }
  ROS_INFO("Joint states initialized.");
  ros::Rate r(publish_rate);

  uint8_t sq=0;

  while(n.ok()){
    ros::spinOnce();
    publish_joint_states(sequence[sq]);
    sq++; sq>3?sq=0:sq;
    r.sleep();
  }
}
