#include "push_obj.h"

PushObject::PushObject() {
  ros::NodeHandle np;
  mocap_comm = new MocapComm(&np); 
}

bool PushObject::GetObjPose(HomogTransf *tf) {
  if (obj_reg.GetLocalObjectPose(*mocap_comm, tf)) {
    return true;
  } else {
    return false;
  }
}

void PushObject::Serialize(std::ostream& fout) {
  // Serialize object registration(transformation) information.
  obj_reg.Serialize(fout);
  // Serialize object geometry information.
  obj_geo.Serialize(fout);
}

void PushObject::Deserialize(std::istream& fin) {
  // Deserialize object registration/transformation.
  obj_reg.Deserialize(fin);
  // Deserialize object geometry info.
  obj_geo.Deserialize(fin);
}



