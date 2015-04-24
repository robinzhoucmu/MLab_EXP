#include "obj_reg.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ObjectRegistration");
  ros::NodeHandle np;
  MocapComm mocap_comm(&np);
  ObjectReg obj_reg;

  obj_reg.Register(mocap_comm);
  std::cout << "Type in the file name to save registered information" << std::endl;
  std::string output_file_name;
  std::cin >> output_file_name;

  std::ofstream fout;
  fout.open(output_file_name.c_str());
  obj_reg.Serialize(fout);
  fout.close();

  return 0;
}
