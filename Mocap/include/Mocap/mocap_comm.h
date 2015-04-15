/**
 * Mocap comm provide several wrapper for to interact with mocap services.
 * 
 */
#ifndef MOCAP_COMM_H
#define MOCAP_COMM_H

#include <ros/ros.h>
#include <Mocap/mocap_GetMocapFrame.h>
#include <Mocap/mocap_GetMocapTransformation.h>
#include <Mocap/mocap_SetMocapTransformation.h>
#include <Mocap/mocap_SetObjTransformation.h>
#include <Mocap/mocap_frame.h>
#include <matVec/matVec.h>

class MocapComm {
 public:
  MocapComm(ros::NodeHandle *np);
  ~MocapComm();
  void SubscribeServices();

  bool SetMocapTransformation(const double pose[7]);
  bool SetMocapTransformation(const HomogTransf& tf);

  bool GetMocapFrame(Mocap::mocap_frame* mocap_frame);
  bool GetMocapTransformation(HomogTransf* tf);
  bool GetMocapTransformation(double pose[7]);
  
 private:
  ros::NodeHandle *nodeHandle;
  
  // Ros services(get).
  ros::ServiceClient handle_mocap_GetMocapFrame;
  ros::ServiceClient handle_mocap_GetMocapTransformation;

  // Ros services(set).
  ros::ServiceClient handle_mocap_SetMocapTransformation;

  // Ros service data structures.
  Mocap::mocap_GetMocapFrame mocap_GetMocapFrame_srv;
  Mocap::mocap_GetMocapTransformation mocap_GetMocapTransformation_srv;
  Mocap::mocap_SetMocapTransformation mocap_SetMocapTransformation_srv;

  /*
  // HomogTransf representing tracked object local/self frame in mocap object tracking frame. 
  HomogTransf tfobj_mocap_loc;
  */
  
};

#endif //MOCAP_COMM_H
