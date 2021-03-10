#include "ros/ros.h"

#include <webots_ros/set_int.h>

#include <webots_ros/set_float.h>

#include <sensor_msgs/Range.h>

#include <signal.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cstdlib>

#define TIME_STEP 32
#define JOINT_COUNT 7

// --------------------------------------------------------------
// ROS callbacks
// --------------------------------------------------------------

static int step = TIME_STEP;
static int count = 0;
static char modelList[10][100];
ros::ServiceClient setTimeStepClient;
webots_ros::set_int setTimeStepSrv;

// callback functions
void modelNameCallback(const std_msgs::String::ConstPtr &name){
  count++;
  strcpy(modelList[count], name->data.c_str());
  ROS_INFO("Model #%d: %s.", count, name->data.c_str());
}
void quit(int sig) {
  setTimeStepSrv.request.value = 0;
  setTimeStepClient.call(setTimeStepSrv);
  ROS_INFO("User stopped the 'robot_state_initializer_node' node.");
  ros::shutdown();
  exit(0);
}



int main(int argc, char **argv) {
  int nStep = 0;
  int stepMax = 500;
  ros::init(argc, argv, "robot_state_initializer_node");
  ros::NodeHandle node_handle;
  // initial state of panda
  std::vector<float> panda_ready_state{0, 0, 0, -2.356, 0, 1.571, 0};

  // declaration of varible names to define services and topics name dynamically, the webots reconize the name
  std::string modelName;
  // get the name of robot
  ros::Subscriber nameSub = node_handle.subscribe("model_name", 100, modelNameCallback);
  while (count == 0||count<nameSub.getNumPublishers())
  {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();
  if (count == 1)
  {
    modelName = modelList[1];
  }
  else
  {
    int wantedModel = 0;
    std::cout << "Choose the # of the model you want to use:\n";
    std::cin >> wantedModel;
    if (1 <= wantedModel && wantedModel <= count)
      modelName = modelList[wantedModel];
    else {
      ROS_ERROR("Invalid choice.");
      return 1;
    }
  }
  nameSub.shutdown();
  count = 0;
  // send robot time step to webots
  ros::ServiceClient setTimeStepClient = node_handle.serviceClient<webots_ros::set_int>(modelName + "/robot/time_step");
  setTimeStepSrv.request.value = step;
  if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success)
  {
    nStep++;
    ROS_INFO("set time step to webots");
  }
  else
  {
    ROS_ERROR("Failed to call service time_step to update robot's time step.");
  }
  
  


  // ros::ServiceClient jointMotorPositionClient;
  // ros::ServiceClient jointMotorVelocityClient;
  webots_ros::set_float motorVelocity;
  motorVelocity.request.value = 1.0;
  webots_ros::set_float motorPosition;
  // motorPosition.request.value = 0.0;
  char deviceName[20];
  

  for (int i = 0; i < JOINT_COUNT; i++)
    {
      int joint = i+1;
      sprintf(deviceName, "panda_joint%d", joint);
      ROS_INFO(deviceName);
      ros::ServiceClient jointMotorPositionClient = node_handle.serviceClient<webots_ros::set_float>(modelName + "/" + deviceName + "/set_position");
      ros::ServiceClient jointMotorVelocityClient = node_handle.serviceClient<webots_ros::set_float>(modelName + "/" + deviceName + "/set_velocity");
      if(jointMotorVelocityClient.call(motorVelocity)){
        ROS_INFO("Set the velocity to 1.0");
      }else
      {
        ROS_ERROR("Can't call the velocity!");
      }
      
      motorPosition.request.value = panda_ready_state[i];
      if (jointMotorPositionClient.call(motorPosition))
      {
        ROS_INFO("Set the position to initial pose");
      }else
      {
        ROS_ERROR("Can't call the position!");
      }    
      
    }
  
  // main loop
  while (nStep <= stepMax)
  {
    if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success)
      nStep++;
    else
      ROS_ERROR("Failed to call service time_step to update robot's time step.");
  }
  
  
  
  
  ros::shutdown();
  return (0);
}
