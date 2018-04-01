#include "grasp_type.h"
#include <stdio.h>
//#include <algorithm>
#include <iterator>

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)



int joint[16];
int stop_table[16];
int condinit;
float speed_Percentage=1;
float hand_Direction=0;
double desired_position[DOF_JOINTS] = {0.0};
double current_position[DOF_JOINTS] = {0.0};
double previous_position[DOF_JOINTS] = {0.0};
double distance[DOF_JOINTS] = {0.0};

AllegroNodeGraspController::AllegroNodeGraspController()
         {

  initControllerxx();

  grasp_type_sub = nh.subscribe("allegroHand_0/libsss_cmd", 1, &AllegroNodeGraspController::graspTypeControllerCallback, this);

  SpeedPer_sub = nh.subscribe("/lwr/speedPercentage", 10, &AllegroNodeGraspController::speedPerCallback, this);

  desired_state_pub = nh.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 1);

  next_state_sub = nh.subscribe(NEXT_STATE_TOPIC, 1, &AllegroNodeGraspController::nextStateCallback, this);

  current_state_pub = nh.advertise<sensor_msgs::JointState>(CURRENT_LISTENER_TOPIC, 1);

  stop_pub = nh.advertise<std_msgs::String>(STOP_TOPIC, 1);

  
}

AllegroNodeGraspController::~AllegroNodeGraspController() {
  delete mutex;
}

void AllegroNodeGraspController::speedPerCallback(const handtracker::spper &msg) {
  speed_Percentage = msg.sPer;
  hand_Direction = msg.dir;
}

void AllegroNodeGraspController::graspTypeControllerCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
  const std::string grasp_type = msg->data;

  std_msgs::String stop_msg;
  std::stringstream stop_ss;
  
  condinit = 0;

  if (grasp_type.compare("home") == 0) {
    condinit = 1;

    //mutex->lock();
      for (int i = 0; i < DOF_JOINTS; i++)
        desired_position[i] = DEGREES_TO_RADIANS(home_pose[i]); 
    //mutex->unlock();  

    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("power") == 0) {
    condinit = 1;

    //mutex->lock();
      for (int i = 0; i < DOF_JOINTS; i++)
        desired_position[i] = power[i];
    //mutex->unlock();

    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("thumb") == 0) {
    condinit = 1;

    //mutex->lock();
      for (int i = 0; i < DOF_JOINTS; i++)
        desired_position[i] = thumb[i];
    //mutex->unlock();

    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("pinch") == 0) {
    condinit = 1;

    
      for (int i = 0; i < DOF_JOINTS; i++)
        desired_position[i] = pinch[i];
  

    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("lateral") == 0) {
    condinit = 1;

  
      for (int i = 0; i < DOF_JOINTS; i++)
        desired_position[i] = lateral[i]; 
   

    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
  }

  else if (grasp_type.compare("little_tactile") == 0) {
    stop_ss << "little_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 8; i < 12; i++) {
      stop_table[i] = 1;
    }
  }

  else if (grasp_type.compare("middle_tactile") == 0) {
    stop_ss << "middle_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 4; i < 8; i++) {
      stop_table[i] = 1;
    }
  }

  else if (grasp_type.compare("index_tactile") == 0) {
    stop_ss << "index_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 0; i < 4; i++) {
      stop_table[i] = 1;
    }
  }

  else if (grasp_type.compare("thumb_tactile") == 0) {
    stop_ss << "thumb_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 12; i < 16; i++) {
      stop_table[i] = 1;
    }
  }

  if (condinit == 1) {


    current_state.position.resize(DOF_JOINTS);
    for (int i = 0; i < 12; i++) {
      current_state.position[i] = 0.0;
    }
    
    current_state.position[12] = 1.05;
  
    for (int i = 13; i < DOF_JOINTS; i++) {
      current_state.position[i] = 0.0;
    }
   
    for (int i = 0; i < DOF_JOINTS; i++) {
      distance[i] = std::abs(desired_position[i] - current_state.position[i]);
      current_state.velocity[i] = (distance[i]/8000)*speed_Percentage;
      joint[i] = 0;
      stop_table[i] = 0;
    }
    

    condinit = 0;
    current_state_pub.publish(current_state);
  }
}

void AllegroNodeGraspController::nextStateCallback(const sensor_msgs::JointState &msg) {
  current_state = msg;
  for (int i = 0; i < DOF_JOINTS; i++) {
    if (current_state.position[i] >= desired_position[i]) 
      joint[i] = 1;
  }
  
  for (int i = 0; i < DOF_JOINTS; i++) {
    if (joint[i] == 1 && stop_table[i] == 0) {
      current_state.position[i] = desired_position[i];
    }
  }

  for (int i = 0; i < (DOF_JOINTS); i++)
  {
    if (joint[i] != 1 &&  stop_table[i] != 1 ) {
      current_state_pub.publish(current_state);
    }
  }
 

  desired_state_pub.publish(current_state);
}

void AllegroNodeGraspController::initControllerxx() {
  std::cout<<"okokokok\n";

  current_state.position.resize(DOF_JOINTS);
   ROS_INFO("dghsadhsa");
  current_state.velocity.resize(DOF_JOINTS);
  
  desired_state.position.resize(DOF_JOINTS);
  desired_state.velocity.resize(DOF_JOINTS);
  ROS_INFO("543545");

  printf("*************************************\n");
  printf("         Grasp (BHand) Method        \n");
  printf("-------------------------------------\n");
  printf("         Every command works.        \n");
  printf("*************************************\n");
}
void AllegroNodeGraspController::doIt() {
  ros::Rate rate(250.0);
  while (ros::ok()) {
    //updateController();
    rate.sleep();
    ros::spinOnce();
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_grasp");
  AllegroNodeGraspController grasping;


  grasping.doIt();
}
