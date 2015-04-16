#ifndef PUSH_OBJ
#define PUSH_OBJ

// This header is a hyper class containing object property information
// for carrying out push actions.

#include "obj_geometry.h"
#include "obj_reg.h"


class PushObject {
 public:
  PushObject();
  // Get pose in robot frame after transforming mocap readings.
  // Assume mocap system is calibrated to robot base frame already.
  bool GetObjPose(HomogTransf* tf);
  const std::vector<Vec> GetVertices() {return obj_geo.vertices;}
  const std::vector<Edge> GetEdges() {return obj_geo.edges;}
  
 private:
  MocapComm* mocap_comm;
  ObjectGeometry obj_geo;
  ObjectReg obj_reg;
  
};

#endif
