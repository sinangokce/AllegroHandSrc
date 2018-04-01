#ifndef PROJECT_GRASP_TYPE_H
#define PROJECT_GRASP_TYPE_H

//#include "allegro_node.h"
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"






#include <string>
#include <boost/thread/thread.hpp>

#include "sensor_msgs/JointState.h"


#define DOF_JOINTS 16












const std::string STOP_TOPIC = "allegroHand_0/stop_topic";
const std::string CURRENT_LISTENER_TOPIC = "allegroHand_0/current_listener";
const std::string NEXT_STATE_TOPIC = "allegroHand_0/next_state";

//const int DOF_JOINTS = 16;




// Grasping controller that uses the BHand library for commanding various
// pre-defined grasp (e.g., three-finger ping, envelop, etc...).
//
// This node is most useful when run with the keyboard node (the keyboard node
// sends the correct String to this node). A map from String command -> Grasp
// type is defined in the implementation (cpp) file.
//
// This node can also save & hold a position, but in constrast to the PD node
// you do not have any control over the controller gains.
//
// Author: Felix Duvallet
//
class AllegroNodeGraspController {

 public:

    AllegroNodeGraspController();

    ~AllegroNodeGraspController();

    void graspTypeControllerCallback(const std_msgs::String::ConstPtr &msg);

    void nextStateCallback(const sensor_msgs::JointState &msg);

    void initControllerxx();

    void doIt();


    //void jointStop(void);

    //void computeDesiredTorque();

    

    //void tactileInfoCallback(const std_msgs::String::ConstPtr &msg);

    

    //void setJointCallback(const sensor_msgs::JointState &msg);

    //void envelopTorqueCallback(const std_msgs::Float32 &msg);

    
 protected:
    ros::NodeHandle nh;

    // Handles defined grasp types (std_msgs/String).
    ros::Subscriber grasp_type_sub;
    ros::Subscriber next_state_sub;
    ros::Subscriber tactile_sub;

    ros::Publisher current_state_pub;
    ros::Publisher desired_state_pub;
    ros::Publisher stop_pub;


    boost::mutex *mutex;

    

    //ros::Subscriber envelop_torque_sub;



    sensor_msgs::JointState desired_state;
    sensor_msgs::JointState current_state;
    
    double home_pose[16] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };

   /* double power[16] =
        {
                5e-324, 1.5938999999999, 1.24506845099999, 0.150626025000005,  5e-324, 1.5938999999999, 1.24506845099999, 0.150626025000005,
                5e-324, 1.5938999999999, 1.24506845099999, 0.150626025000005, 0.26037000005, 0.9277975079999999, 1.62755999999997, 1.26810594900000002
        };  
        */

    /*double power[16] =
        {
                0, 1.5938999999999, 1.24506845099999, 0.150626025000005,  0, 1.5938999999999, 1.24506845099999, 0.150626025000005,
                0, 1.5938999999999, 1.24506845099999, 0.150626025000005, 0.26037000005, 0.9277975079999999, 1.62755999999997, 1.26810594900000002
        };   
        */  

    double power[16] =
        {
                -0.15, 1.08, 1.07, 0.75,  0.04, 1.03, 1.07, 0.75,
                0.21, 1.08, 1.07, 0.75, 1.40, 0.54, 0.34, 1.29
        }; 

    /*double lateral[16] =
        {
                5e-324, 1.5938999999999, 1.589753484, -6e-5,  5e-324, 1.5938999999999, 1.589753484, -6e-5,
                5e-324, 1.5938999999999, 1.589753484, -6e-5, 0.375565508999996, 0.24000768000003, 1.50325510499996, -4e-5
        };*/

    double lateral[16] =
        {
                -0.06, 1.60, 1.60, 0,  0, 1.60, 1.60, 0,
                0, 1.60, 1.60, 0, 0.89, -0.10, 1.02, 0.70
        };   

    /*double pinch[16] =
        {
                5e-324, 0.834561882, 0.887520644999999, 0.9011554200000005,  0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.351306242, -9e-6, 0.757425734999998, 0.28598694300000005
        };
        */

    double pinch[16] =
        {
            0, 0.834561882, 0.887520644999999, 0.9011554200000005,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.351306242, 0, 0.757425734999998, 0.28598694300000005
        };

    double thumb[16] =
        {
                0.1082287799999993, 0.834561882, 0.88752064499999, 0.9011554200005,  0.29956013999999986, 0.883551438000002, 0.8235796139999, 0.9886471649999,
                0.0, 0.0, 0.0, 0.0, 1.3820399999995, -0.0, 0.75742573499999998, 0.3498600599999                                 
        }; 

  double desired_position[DOF_JOINTS] = {0.0};
};

#endif //PROJECT_GRASP_TYPE_H
