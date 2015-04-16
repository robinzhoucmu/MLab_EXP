#ifndef OBJ_REJ_H
#define OBJ_REJ_H

// This header file defines the object registration class.
// ObjectReg instance contains  mocap readings and 
// computed transformations to register the object.
// We will require the mocap system to be calibrated w.r.t the robot
// base frame beforehand. 

#include <fstream>
#include <iostream>
#include <string>
#include <assert.h>
#include <Mocap/mocap_comm.h>
#include <Mocap/mocap_frame.h>
#include <matVec/matVec.h>

class ObjectReg {
 public:
  ObjectReg();
  
  void Serialize(std::ostream& fout);
  void Deserialize(std::istream& fin);

  // An interactive process to register the object from terminal.
  // Include a mocap calibration w.r.t robot base step.
  bool Register(MocapComm& mocap_comm);

  bool ReadCaliMarkersFromMocap(MocapComm& mocap_comm);

  // Read the pose of the tractable rigid body from mocap.
  bool ReadTractablePoseFromMocap(MocapComm& mocap_comm);
  
  void ComputeTransformation();

  // Get local object pose in robot base frame by transforming mocap pose with tf_mctractable_obj;
  bool GetLocalObjectPose(MocapComm& mocap_comm, HomogTransf* obj_pose);
  
  // Get the key transformation between mocap marker frame, which we don't exactly 
  // know how opti-track is forming it, to object local frame, which we will pre-specify. 
  const HomogTransf GetTransformation() {
    return tf_mctractable_obj;
  }
  const std::string GetObjName() {
    return obj_name;
  }
  void SetObjName(std::string name) {
    obj_name = name;
  }
  
 private:
  // Compute the tf_robot_calimarkers from calibration marker point mocap readings.
  void FormCaliMarkerCoordinateFrame();

  void SerializeHomog(std::ostream& fout, const HomogTransf& tf);
  void DeserializeHomog(std::istream& fin, HomogTransf* tf);
  
  std::string obj_name;
  std::vector<Vec> cali_markers_pos;

  // Transformation from robot base to motion capture tractable.
  HomogTransf tf_robot_mctractable;

  // Transformation from robot base to calibration markers.
  // If mocap publisher is calibrated to robot base, we could directly read off  
  // from mocap topic information. 
  // We will use this as one sample of known T_{R}^{obj}.
  HomogTransf tf_robot_calimarkers;

  // Transformation (T_{mct}^{obj}) from mocap frame tractable to local object frame. 
  // T_{R}^{mct} * T_{mct}^{obj} = T_{R}^{obj}.
  // T_{mct}^{obj} = inv(T_{R}^{mct}) * T_{R}^{obj}
  HomogTransf tf_mctractable_obj;
  
  // Pairwise distance matrix that characterize the rigid body for association when
  // mocap is tracking multiple objects.
  Mat cm_dis_mat;
  
};

#endif
