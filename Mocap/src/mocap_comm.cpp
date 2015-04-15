#include "mocap_comm.h"
#include <iostream>

MocapComm::MocapComm(ros::NodeHandle *np) {
  nodeHandle = np;
  SubscribeServices();
}

MocapComm::~MocapComm() {
  // Shutdown services.
  handle_mocap_GetMocapFrame.shutdown();
  handle_mocap_GetMocapTransformation.shutdown();
  handle_mocap_SetMocapTransformation.shutdown();
}

void MocapComm::SubscribeServices() {
  handle_mocap_SetMocapTransformation = 
    nodeHandle->serviceClient<Mocap::mocap_SetMocapTransformation>("mocap_SetMocapTransformation");
  handle_mocap_GetMocapTransformation = 
    nodeHandle->serviceClient<Mocap::mocap_GetMocapTransformation>("mocap_GetMocapTransformation");
  handle_mocap_GetMocapFrame = 
    nodeHandle->serviceClient<Mocap::mocap_GetMocapFrame>("mocap_GetMocapFrame");
}

bool MocapComm::SetMocapTransformation(const double pose[7]) {
  mocap_SetMocapTransformation_srv.request.x = pose[0];
  mocap_SetMocapTransformation_srv.request.y = pose[1];
  mocap_SetMocapTransformation_srv.request.z = pose[2];
  mocap_SetMocapTransformation_srv.request.q0 = pose[3];
  mocap_SetMocapTransformation_srv.request.qx = pose[4];
  mocap_SetMocapTransformation_srv.request.qy = pose[5];
  mocap_SetMocapTransformation_srv.request.qz = pose[6];
  return handle_mocap_SetMocapTransformation.call(mocap_SetMocapTransformation_srv);
}

bool MocapComm::SetMocapTransformation(const HomogTransf& tf) {
  double pose[7];
  Vec tf_trans = tf.getTranslation();
  pose[0] = tf_trans[0];
  pose[1] = tf_trans[1];
  pose[2] = tf_trans[2];
  Quaternion tf_quat = tf.getQuaternion();
  pose[3] = tf_quat[0];
  pose[4] = tf_quat[1];
  pose[5] = tf_quat[2];
  pose[6] = tf_quat[3];
  return SetMocapTransformation(pose);
}

bool MocapComm::GetMocapFrame(Mocap::mocap_frame* mocap_frame) {
  if (handle_mocap_GetMocapFrame.call(mocap_GetMocapFrame_srv)) {
    *mocap_frame = mocap_GetMocapFrame_srv.response.mf;
    return true;
  } else {
    return false;
  }
}

bool MocapComm::GetMocapTransformation(double pose[7]) {
  if (handle_mocap_GetMocapTransformation.call(mocap_GetMocapTransformation_srv)) {
    pose[0] = mocap_GetMocapTransformation_srv.response.x;
    pose[1] = mocap_GetMocapTransformation_srv.response.y;
    pose[2] = mocap_GetMocapTransformation_srv.response.z;
    pose[3] = mocap_GetMocapTransformation_srv.response.q0;
    pose[4] = mocap_GetMocapTransformation_srv.response.qx;
    pose[5] = mocap_GetMocapTransformation_srv.response.qy;
    pose[6] = mocap_GetMocapTransformation_srv.response.qz;
    return true;
  } else {
    return false;
  }
}

bool MocapComm::GetMocapTransformation(HomogTransf* tf) {
  double pose[7];
  if (GetMocapTransformation(pose)) {
    tf->setPose(pose);
    return true;
  } else {
    return false;
  }
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "MocapComm");
  ros::NodeHandle node;
  MocapComm mocap_comm(&node);

  // testing code.
  Mocap::mocap_frame mf;
  mocap_comm.GetMocapFrame(&mf);
  std::cout << mf << std::endl;

  double test_pose[7] = {600, 150, 60, 0, 0, 0.707, 0.707};
  HomogTransf tf(test_pose);
  mocap_comm.SetMocapTransformation(tf);

  mocap_comm.GetMocapFrame(&mf);
  std::cout << mf << std::endl;

  HomogTransf tf2;
  mocap_comm.GetMocapTransformation(&tf2);
  std::cout << tf2 << std::endl;
  return 0;
}
