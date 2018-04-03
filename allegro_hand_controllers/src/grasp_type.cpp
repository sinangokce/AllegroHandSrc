#include "grasp_type.h"
#include <stdio.h>
//#include <algorithm>
#include <iterator>

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

int joint[16];
int stop_table[16];
int condinit;
int dir;
float speed_Percentage=1;
float hand_Direction=0;
double desired_position[DOF_JOINTS] = {0.0};
double current_position[DOF_JOINTS] = {0.0};
double previous_position[DOF_JOINTS] = {0.0};
double distance[DOF_JOINTS] = {0.0};

std::vector< std::vector<double> > history;



int s = 0;

AllegroNodeGraspController::AllegroNodeGraspController() {
         
  initControllerxx();

  grasp_type_sub = nh.subscribe("allegroHand_0/libsss_cmd", 1, &AllegroNodeGraspController::graspTypeControllerCallback, this);

  SpeedPer_sub = nh.subscribe("/lwr/speedPercentage", 10, &AllegroNodeGraspController::speedPerCallback, this);

  desired_state_pub = nh.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 1);

  next_state_sub = nh.subscribe(NEXT_STATE_TOPIC, 1, &AllegroNodeGraspController::nextStateCallback, this);

  current_state_pub = nh.advertise<sensor_msgs::JointState>(CURRENT_LISTENER_TOPIC, 1);

  stop_pub = nh.advertise<std_msgs::String>(STOP_TOPIC, 1); 
  dir_pub = nh.advertise<std_msgs::String>(DIR_TOPIC, 1);
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

  std_msgs::String dir_msg;
  std::stringstream dir_ss;
  
  condinit = 0;
  dir = 0;

  if (grasp_type.compare("home") == 0) {
    condinit = 1;

    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = DEGREES_TO_RADIANS(home_pose[i]); 

    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);

    dir = 1;
    dir_ss << "false";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }


   else if (grasp_type.compare("open") == 0) {
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);

    condinit = 0;
    for(int i = 0; i<DOF_JOINTS; i++)
      joint[i] = 0;

    dir = -1;
    dir_ss << "open";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("stay") == 0) {
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);

    dir = 0;
    dir_ss << "stay";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("close") == 0) {
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);

    dir = 1;
    dir_ss << "close";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("power") == 0) {
    condinit = 1;

    
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = power[i];
   
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);

    dir = 1;
    dir_ss << "false";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("thumb") == 0) {
    condinit = 1;

    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = thumb[i];
    
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);

    dir = 1;
    dir_ss << "false";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("pinch") == 0) {
    condinit = 1;

    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = pinch[i];
  
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);

    dir = 1;
    dir_ss << "false";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("lateral") == 0) {
    condinit = 1;

    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = lateral[i]; 
   
    stop_ss << "false";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);

    dir = 1;
    dir_ss << "false";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("little_tactile") == 0) {
    stop_ss << "little_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 8; i < 12; i++) {
      stop_table[i] = 1;
    }

    dir = 1;
    dir_ss << "false";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("middle_tactile") == 0) {
    stop_ss << "middle_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 4; i < 8; i++) {
      stop_table[i] = 1;
    }

    dir = 1;
    dir_ss << "false";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("index_tactile") == 0) {
    stop_ss << "index_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 0; i < 4; i++) {
      stop_table[i] = 1;
    }

    dir = 1;
    dir_ss << "false";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
  }

  else if (grasp_type.compare("thumb_tactile") == 0) {
    stop_ss << "thumb_tactile";
    stop_msg.data = stop_ss.str();
    stop_pub.publish(stop_msg);
    for (int i = 12; i < 16; i++) {
      stop_table[i] = 1;
    }

    dir = 1;
    dir_ss << "false";
    dir_msg.data = dir_ss.str();
    dir_pub.publish(dir_msg);
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
      current_state.velocity[i] = (distance[i]/8000);
      joint[i] = 0;
      stop_table[i] = 0;
    }
    
    //condinit = 0;

    current_state_pub.publish(current_state);
  }
}

void AllegroNodeGraspController::nextStateCallback(const sensor_msgs::JointState &msg) {

  //std::vector<double> each_step;

  current_state = msg;

  /*if (condinit == 1) {
    history.clear();

    for (int i = 0; i < DOF_JOINTS; i++) {
      each_step.push_back(current_state.position[i]); //first step is initialization
    }

    history.push_back(each_step);

    each_step.clear();

    condinit = 0;
  }*/

  //ROS_INFO("%d",s);
  //s = s+1;

  /*for (int i = 0; i < DOF_JOINTS; i++) {
      if (current_state.position[i] >= desired_position[i])   //if the joint arrives the desired position it should stop
        joint[i] = 1;
    }*/
  
  

  switch(dir) {
    case -1:
      ROS_INFO("opening");

      for (int i = 0; i < 12; i++) {
        if(current_state.position[i] <= 0.0);
          joint[i] = 1;
      }

      if(current_state.position[12] <= 1.05 )
        joint[12] = 1;

      for (int i = 13; i < DOF_JOINTS; i++) {
        if(current_state.position[i] <= 0.0);
          joint[i] = 1;
      }
      
      break;

    case 0:
      //ROS_INFO("stoped");
      break;  

    case 1:
      //ROS_INFO("closing");
      for (int i = 0; i < DOF_JOINTS; i++) {
        if (current_state.position[i] >= desired_position[i])   //if the joint arrives the desired position it should stop
          joint[i] = 1;
      }
      break;   
  }

  //for(int i = 0; i<DOF_JOINTS; i++)
    //ROS_INFO("%f", current_state.position[i]);

  if(dir == -1) {

    /*for(int i = 0; i < DOF_JOINTS; i++) {
      ROS_INFO("%f", current_state.position[i]);
    }

    for(int p = 0; p<10000000; p++)
      int n = 0;*/

    /*for (int i = 0; i < 12; i++) {
      if (joint[i] == 1 )                //When the joint arrived the desired position, it should stay at this position 
        current_state.position[i] = 0.0;
    }

    if (joint[12] == 1)                //When the joint arrived the desired position, it should stay at this position 
      current_state.position[12] = 1.05;

    for (int i = 13; i < DOF_JOINTS; i++) {
      if (joint[i] == 1 && stop_table[i] == 0)                //When the joint arrived the desired position, it should stay at this position 
        current_state.position[i] = 0.0;
    }  */
  }

  else {
    for (int i = 0; i < DOF_JOINTS; i++) {
      if (joint[i] == 1 && stop_table[i] == 0)                 //When the joint arrived the desired position, it should stay at this position 
        current_state.position[i] = desired_position[i];
    }
  }  

  for (int i = 0; i < DOF_JOINTS; i++) 
        ROS_INFO("%d :%d",i, joint[i]);

  for (int i = 0; i < (DOF_JOINTS); i++)
  {
    if (joint[i] != 1 &&  stop_table[i] != 1 )
      current_state_pub.publish(current_state);
  }

  /*if(close == 1){

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
      if (joint[i] != 1 &&  stop_table[i] != 1 )
        current_state_pub.publish(current_state);
    }
  }

  else if(open == 1) {
    for (int i = 0; i < DOF_JOINTS; i++) {
      if (current_state.position[i] <= DEGREES_TO_RADIANS(home_pose[i])) 
        joint[i] = 1;
    }*/
  
    /*for (int i = 0; i < DOF_JOINTS; i++) {
      if (joint[i] == 1 && stop_table[i] == 0) {
        current_state.position[i] = desired_position[i];
      }
    }*/

    for (int i = 0; i < (DOF_JOINTS); i++)
    {
      if (joint[i] != 1 )
        current_state_pub.publish(current_state);
    }

  

  
  

  /*for (int i = 0; i < (DOF_JOINTS); i++) {
        each_step.push_back(current_state.position[i]);
      }
  history.push_back(each_step);*/

  /*if(back == 1) {
    std::vector<int>::size_type i = (history.size()-1);
    for(i; i>0; --i) {
      for(int j=0; j< DOF_JOINTS; j++)
        current_state.position[j] = history[i][j];
      desired_state_pub.publish(current_state);
      for(int p=0; p < 1000; p++)
        int x=0;
    }
  }
  else*/
  desired_state_pub.publish(current_state);
}

void AllegroNodeGraspController::initControllerxx() {
  current_state.position.resize(DOF_JOINTS);
  current_state.velocity.resize(DOF_JOINTS);
  
  desired_state.position.resize(DOF_JOINTS);
  desired_state.velocity.resize(DOF_JOINTS);

  printf("*************************************\n");
  printf("         Grasp (BHand) Method        \n");
  printf("-------------------------------------\n");
  printf("         Every command works.        \n");
  printf("*************************************\n");
}
void AllegroNodeGraspController::doIt() {
  ros::Rate rate(250.0);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_grasp");
  AllegroNodeGraspController grasping;

  grasping.doIt();
}