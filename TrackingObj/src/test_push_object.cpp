#include "push_obj.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_push_obj");
  // Read calibration information to transform mocap coordinate to global robot base
  // frame coordinate. 
  ros::NodeHandle np;
  MocapComm mocap_comm(&np);
  double pose_robot_mocap[7];
  std::ifstream fin_mocap_cali;
  fin_mocap_cali.open("../Mocap/matlab_cali_result.txt");
  for (int i = 0; i < 7; ++i) {
    fin_mocap_cali >> pose_robot_mocap[i];
  }
  fin_mocap_cali.close();
  mocap_comm.SetMocapTransformation(pose_robot_mocap);
    
  //PushObject triangle("object_db/triangle_reg.txt", "object_db/triangle_geo.txt");
  PushObject triangle("object_db/triangle.txt");
  // Print out vertices information.
  const std::vector<Vec> & vertices = triangle.GetVertices();
  for (int i = 0; i < vertices.size(); ++i) {
    std::cout << vertices[i] << std::endl;
  }

  // Print out current pose in robot frame(global).
  HomogTransf tf;
  triangle.GetGlobalObjPose(&tf);
  std::cout << tf << std::endl;
}
