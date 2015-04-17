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



