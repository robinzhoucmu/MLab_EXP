#include "obj_reg.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ObjectRegistration");
  ros::NodeHandle np;
  MocapComm mocap_comm(&np);
  ObjectReg obj_reg;
  obj_reg.Register(mocap_comm);

  std::cout << "Saving information to \"" << obj_reg.GetObjName() << ".txt\"  on disk";
  std::string output_file_name = obj_reg.GetObjName() + ".txt";
  std::ofstream fout;
  fout.open(output_file_name.c_str());
  obj_reg.Serialize(fout);
  fout.close();

  return 0;
}
