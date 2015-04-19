#include "push_obj.h"

PushObject::PushObject() {
  InitMocapComm();
}

PushObject::PushObject(std::string file_name) {
  InitMocapComm();
  std::ifstream fin;
  fin.open(file_name.c_str());
  Deserialize(fin);
  fin.close();
}

PushObject::PushObject(std::string file_name_reg, std::string file_name_geo) {
  InitMocapComm();
  std::ifstream fin_reg;
  fin_reg.open(file_name_reg.c_str());
  obj_reg.Deserialize(fin_reg);
  fin_reg.close();
  
  std::ifstream fin_obj;
  fin_obj.open(file_name_geo.c_str());
  obj_geo.Deserialize(fin_obj);
  fin_obj.close();
}

bool PushObject::GetGlobalObjPose(HomogTransf *tf) {
  if (obj_reg.GetGlobalObjectPose(*mocap_comm, tf)) {
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

void PushObject::InitMocapComm() {
  mocap_comm = new MocapComm(&np); 
}


