/*
 * PushExp is the class that does a chain of random of pushes.
 */

#ifndef PUSH_EXP_H
#define PUSH_EXP_H

#include <iostream>
#include <vector>
// ROS related. 
#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <Mocap/mocap_comm.h>
#include <matVec/matVec.h>

class PushExp {
 public:
  PushExp(ros::NodeHandle *np);
  ~PushExp();
  void SubscribeServices();

 private:
  RobotComm* robot;
  ros::AsyncSpinner asyn_spinner;

  // Mocap subscription.
  ros::Subscriber mocap_sub;
  // Force subscription.
  ros::Subscriber force_sub;
  int num_pushes;
  // Command the robot to a resting position that won't block the mocap nor touch the object.
  void GotoRobotRestingState();
  // Let the mocap acquire STATIC object poses assuming the robot is not blocking view.
  void AcquireObjectStablePose();
  
  // 1) Move from the robot from rest state to pre-approach state.
  // 2) Approach object and push.
  // 3) Leave contact. 
  void ExecuteSingleRobotPush();
  // Log the pushing force while robot is pushing/in contact with the object.
  // Uses async spinner for logging. Call this function before calling robotSetCartesian.
  // Remember to stop the Async spinner after robotSetCartesian.
  void LogPushForce();
  
  
  
};

#endif

