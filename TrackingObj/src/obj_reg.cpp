#include "obj_reg.h"

ObjectReg::ObjectReg() {
}

void ObjectReg::Serialize(std::ostream & fout) {
  fout << obj_name << std::endl;
  /*
  // First 3 lines of cali marker positions.
  for (int i = 0; i < 3; ++i) {
    const Vec& p = cali_markers_pos[i];
    for (int j = 0; j < 3; ++j ) {
      fout << p[j] << " ";
    }
    fout << std::endl;
  }
  // Serialize relevant homogenious transformations.
  SerializeHomog(fout, tf_robot_mctractable);
  SerializeHomog(fout, tf_robot_calimarkers);
  */
  SerializeHomog(fout, tf_mctractable_obj);
}

void ObjectReg::Deserialize(std::istream& fin) {
  fin >> obj_name;
  /*
  // Deserialize vector of cali marker positions.
  cali_markers_pos.clear();
  for (int i = 0; i < 3; ++i) {
    Vec p(3);
    for (int j = 0; j < 3; ++j) {
      fin >> p[j];
    }
    cali_markers_pos.push_back(p);
  }
  DeserializeHomog(fin, &tf_robot_mctractable);
  DeserializeHomog(fin, &tf_robot_calimarkers);
  */
  DeserializeHomog(fin, &tf_mctractable_obj);
}

void ObjectReg::DeserializeHomog(std::istream& fin, HomogTransf* tf) {
  double tf_mat[4][4];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      fin >> tf_mat[i][j];
    }
  }
  *tf = HomogTransf(tf_mat);
}

void ObjectReg::SerializeHomog(std::ostream& fout, const HomogTransf& tf) {
  assert(tf.nn == 4 && tf.mm == 4);
  for (int i = 0; i < tf.nn; ++i) {
    for (int j = 0; j < tf.mm; ++j) {
      fout << tf[i][j]<< " ";
    }
    fout << std::endl;
  }
}

void ObjectReg::ReadCaliMarkersFromMocap(MocapComm& mocap_comm) {
  cali_markers_pos.clear();
  Mocap::mocap_frame mocap_msg;
  mocap_comm.GetMocapFrame(&mocap_msg);
  
  // Extract cali markers(which NatNet will treat as unidenfied markers).
  int num_markers = mocap_msg.uid_markers.markers.size();
  // Check for size.
  assert(num_markers == 3);
  for (int i = 0; i < num_markers; ++i) {
    const geometry_msgs::Point& pt_mocap = mocap_msg.uid_markers.markers[i];
    Vec pt(3);
    pt[0] = pt_mocap.x;
    pt[1] = pt_mocap.y;
    pt[2] = pt_mocap.z;
    cali_markers_pos.push_back(pt);
  }
  FormCaliMarkerCoordinateFrame();
}

void ObjectReg::FormCaliMarkerCoordinateFrame() {
  int num_markers = cali_markers_pos.size();
  assert(num_markers == 3);
  int id_origin = -1;
  const double eps_dot_product = 0.1; 
  Vec axis_x;
  Vec axis_y;
  for (int i = 0; i < num_markers; ++i) {
    // Determine the ith point can be set as the origin.
    // Check the angle between axis is close to right angle or not.
    axis_x = cali_markers_pos[(i+1) % num_markers] - cali_markers_pos[i];
    axis_y = cali_markers_pos[(i+2) % num_markers] - cali_markers_pos[i];
    const double norm_axis_x = sqrt(axis_x * axis_x);
    const double norm_axis_y = sqrt(axis_y * axis_y);
    axis_x = axis_x / norm_axis_x;
    axis_y = axis_y / norm_axis_y;

    // Use the longer axis as the y axis.
    if (norm_axis_x > norm_axis_y) {
      Vec tmp = axis_x;
      axis_x = axis_y;
      axis_y = tmp;
    }

    // Check dot product.
    double dot_product = 
      (axis_x * axis_y) / (sqrt(axis_x * axis_x) * sqrt(axis_y * axis_y));  
    if (dot_product < eps_dot_product) {
      id_origin = i;
      break;
    }
  }
  double z[3] = {0, 0, 1};
  Vec axis_z(z,3);
  // axis_x and axis_y may not be perfectly perpendicular, we reset axis_x as 
  // the cross-product of axis_z and axis_y.
  // Todo(Jiaji): Find the nearest rotation matrix by projection.
  axis_x = axis_y ^ axis_z;
  // Form the RefFrame.
  RotMat rot_mat(axis_x, axis_y, axis_z);
  // Form the Translation.
  Vec trans(cali_markers_pos[id_origin]);
  // Update the homogenious transformation.
  tf_robot_calimarkers.setRotation(rot_mat);
  tf_robot_calimarkers.setTranslation(trans);
}

void ObjectReg::ReadTractablePoseFromMocap(MocapComm& mocap_comm) {
  Mocap::mocap_frame mocap_msg;
  mocap_comm.GetMocapFrame(&mocap_msg);
  // We are assuming during registration process, there is only one tractable in view.
  assert(mocap_msg.body_poses.poses.size() == 1);
  // Extract pose from mocap output.
  double tractable_pose[7];
  geometry_msgs::Pose pose = mocap_msg.body_poses.poses[0];
  tractable_pose[0] = pose.position.x;
  tractable_pose[1] = pose.position.y;
  tractable_pose[2] = pose.position.z;
  tractable_pose[3] = pose.orientation.x;
  tractable_pose[4] = pose.orientation.y;
  tractable_pose[5] = pose.orientation.z;
  tractable_pose[6] = pose.orientation.w;
  // Update the tf.
  tf_robot_mctractable.setPose(tractable_pose);
}

void ObjectReg::ComputeTransformation() {
  tf_mctractable_obj = tf_robot_mctractable.inv() * tf_robot_calimarkers;
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ObjectRegistration");
  ros::NodeHandle np;
  MocapComm mocap_comm(&np);
  ObjectReg obj_reg;
  // Shell based interactive process.
  std::cout << "Object Registration Process Starts." << std::endl;

  std::cout << "Input Object Name" << std::endl;
  std::string name;
  std::cin >> name;

  std::cout << "First we need to calibrate mocap frame to robot base frame. Type in the pose(7 numbers):" << std::endl;
  double pose_robot_mocap[7];
  for (int i = 0; i < 7; ++i) {
    std::cin >> pose_robot_mocap[i];
  }
  mocap_comm.SetMocapTransformation(pose_robot_mocap); 

  std::cout << "Place JUST the paper cali markers on the table." << std::endl;
  
  obj_reg.SetObjName(name);
  
  std::cout << "Setting Up Cali Marker Frame" << std::endl;
  std::cout << "Enter any key to start acquiring" << std::endl;
  std::cin.ignore();
  std::cin.get();
  obj_reg.ReadCaliMarkersFromMocap(mocap_comm);
  
  std::cout << "Now put the object on the paper." << std::endl;
  std::cout << "Enter any key to start acquiring" << std::endl;
  std::cin.get();
  obj_reg.ReadTractablePoseFromMocap(mocap_comm);
  
  std::cout << "Computing transformation." << std::endl;
  obj_reg.ComputeTransformation();
  std::cout << obj_reg.GetTransformation() << std::endl;
  
  std::cout << "Saving information to \"" << obj_reg.GetObjName() << ".txt\"  on disk";
  std::string output_file_name = name + ".txt";
  std::ofstream fout;
  fout.open(output_file_name.c_str());
  obj_reg.Serialize(fout);
  fout.close();

  return 0;
}
