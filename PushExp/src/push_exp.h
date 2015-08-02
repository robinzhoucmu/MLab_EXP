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
#include "PushGenerator.h"

class PushExp {
 public:
  PushExp(ros::NodeHandle *n);
  ~PushExp(){};
  void Initialize();
  bool ConfirmStart();
  void Run();  

 private:
  bool execution_flag;
  ros::NodeHandle *nh;
  RobotComm* robot;
  PushGenerator* push_plan_gen;
  PushObject* push_object;

  ros::AsyncSpinner* async_spinner; 
  // Force subscription.
  ros::Subscriber force_sub;
  // Number of push trials.
  int num_pushes;  
  
  // Safe height above the working object surface. 
  double safe_height;
  
  // Flag indicating whether the robot is away from camera view, i.e., not
  // blocking the mocap cameras view. 
  bool flag_robot_away;
  std::vector<HomogTransf> obj_poses;
  
  // Acquiring pose: read kNumMocapReadings number of frames in kReadDuration secs.
  int kNumMocapReadings;
  double kReadDuration;

  // Robot movement trajectory for one single push.
  std::vector<HomogTransf> robot_push_traj;
  
  // Robot resting cartesian position.
  double robot_rest_cart[7];

  void InitializeRobot();
  void InitializeMocapTransform();
  
  // A wrapper function over robot_comm SetCartesian. 
  bool SetCartesian(HomogTransf tf);

  // Command the robot to a resting position that won't block the mocap nor touch the object.
  bool RobotMoveToRestingState();
  // Let the mocap acquire STATIC object poses assuming the robot is not blocking view.
  bool AcquireObjectStablePose(HomogTransf* pose_tf);
  bool ComputeAveragePose(HomogTransf* pose_tf);
  // 1) Move from the robot from rest state to pre-approach state.
  // 2) Approach object and push.
  // 3) Leave contact. 
  bool GeneratePushPlan(HomogTransf pre_push_obj_pose);
  bool ExecRobotPushAndLogForce();  
  bool RobotMoveToAbove(HomogTransf pose_below);

  // Pipeline for single push and data logging.
  bool SinglePushPipeline();
  // Log the pushing force while robot is pushing/in contact with the object.
  // Uses async spinner for logging. Call this function before calling robotSetCartesian.
  // Remember to stop the Async spinner after robotSetCartesian.
  void LogPushForceAsync();
  
  // Check for reset. Read Pose information and decide whether to initiate reset action.
  bool CheckForReset();
  
};

#endif

