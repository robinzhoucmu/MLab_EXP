/*
 * PushExp is the class that does a chain of random of pushes.
 * Note that Mocap should be calibrated in robot frame already.
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

#include "gl_parameter.h"
#include <push_action/PushGenerator.h>

class PushExp {
 public:
  PushExp(ros::NodeHandle *n);
  ~PushExp();
  void InitPushObject(std::string file_name_cali, std::string file_name_geo);
  

 private:
  ros::NodeHandle *nodeHandle;
  RobotComm* robot;
  PushGenerator push_plan_gen;
  PushObject push_object;

  ros::AsyncSpinner async_spinner;
  // Mocap comm.
  // MocapComm mocap_comm;
  
  // Force subscription.
  ros::Subscriber force_sub;
  // Number of push trials.
  int num_pushes;
  
  // Flag indicating whether the robot is away from camera view, i.e., not
  // blocking the mocap cameras view. 
  bool flag_robot_away;
  std::vector<HomogTransf> obj_poses;
  
  // Acquiring pose: read kNumMocapReadings number of frames in kReadDuration secs.
  static const int kNumMocapReadings = 5;
  static const double kReadDuration = 1.0;

  // Robot movement trajectory for one single push.
  std::vector<HomogTransf> robot_push_traj;

  // Command the robot to a resting position that won't block the mocap nor touch the object.
  void GotoRobotRestingState();
  // Let the mocap acquire STATIC object poses assuming the robot is not blocking view.
  bool AcquireObjectStablePose(HomogTransf* pose_tf);
  bool ComputeAveragePose(HomogTransf* avg_pose_tf);

  // 1) Move from the robot from rest state to pre-approach state.
  // 2) Approach object and push.
  // 3) Leave contact. 
  bool GeneratePushPlan();
  bool ExecuteRobotPushTraj();  

  // Log the pushing force while robot is pushing/in contact with the object.
  // Uses async spinner for logging. Call this function before calling robotSetCartesian.
  // Remember to stop the Async spinner after robotSetCartesian.
  void LogPushForceAsync();
  
  // Check for reset. Read Pose information and decide whether to initiate reset action.
  bool CheckForReset();
  
};

#endif

