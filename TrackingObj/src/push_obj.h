#ifndef PUSH_OBJ
#define PUSH_OBJ

// This header is a hyper class containing object property information
// for carrying out push actions.
// Function designed to be a light weight class mainly used for interfacing
// and extract object informations.

#include "obj_geometry.h"
#include "obj_reg.h"


class PushObject {
 public:
  PushObject();
  // Construct from a single file with serialized obj_reg & obj_geo; 
  PushObject(std::string file_name);
  // Construct from separate files.
  PushObject(std::string file_name_reg, std::string file_name_obj);

  // Get pose in robot frame (global frame) after transforming mocap readings.
  // Assume mocap system is calibrated to robot base frame already.
  // The procedure is based on sending service request via mocap_comm in obj_reg.
  bool GetGlobalObjPose(HomogTransf* tf);

  const std::vector<Vec>& GetVertices() const {return obj_geo.vertices;}
  const std::vector<Edge>& GetEdges() const {return obj_geo.edges;}
  
  // Extract the transformation from object mocap marker frame to object local frame. 
  HomogTransf GetIntrinsicTf() const {return obj_reg.GetTransformation();} 
  // Get object name.
  std::string GetObjName() const {return obj_reg.GetObjName();}
  // Get object tractable id.
  int GetTractableId() const {return obj_reg.tractable_id;}

  void Serialize(std::ostream& fout);
  void Deserialize(std::istream &fin);

 private:
  // Mocap stuffs.
  void InitMocapComm();
  ros::NodeHandle np;
  MocapComm* mocap_comm;

  // Object stuffs.
  ObjectGeometry obj_geo;
  ObjectReg obj_reg;
  
};

#endif
