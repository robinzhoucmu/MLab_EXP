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

#include <push_action/PushGenerator.h>

class PushExp {
 public:
  PushExp();
  ~PushExp();
  void SubscribeServices();

 private:
  RobotComm* robot;
  PushGenerator push_plan_gen;

  ros::AsyncSpinner async_spinner;
  // Mocap comm.
  MocapComm mocap_comm;
  // Force subscription.
  ros::Subscriber force_sub;
  // Number of push trials.
  int num_pushes;
  
  // Flag indicating whether the robot is away from camera view, hence not
  // blocking the mocap cameras view. 
  bool flag_robot_away;
  std::vector<geometry_msgs::Pose> obj_poses;
 
  // Command the robot to a resting position that won't block the mocap nor touch the object.
  void GotoRobotRestingState();
  // Let the mocap acquire STATIC object poses assuming the robot is not blocking view.
  void AcquireObjectStablePose();
  void ComputeAveragePose();

  // 1) Move from the robot from rest state to pre-approach state.
  // 2) Approach object and push.
  // 3) Leave contact. 
  void GeneratePushPlan();
  void ExecuteRobotPushTraj();
  
  // Log the pushing force while robot is pushing/in contact with the object.
  // Uses async spinner for logging. Call this function before calling robotSetCartesian.
  // Remember to stop the Async spinner after robotSetCartesian.
  void LogPushForceAsync();
  
  
  
};

#endif

