#include "obj_reg.h"
#include<stdlib.h>

ObjectReg::ObjectReg() {
}

void ObjectReg::Serialize(std::ostream & fout) {
  fout << obj_name << std::endl;
  fout << tractable_id << std::endl; 
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
  fin >> tractable_id;
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

bool ObjectReg::ReadCaliMarkersFromMocap(MocapComm& mocap_comm) {
  cali_markers_pos.clear();
  Mocap::mocap_frame mocap_msg;
  bool flag = mocap_comm.GetMocapFrame(&mocap_msg);
  if (flag) {
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
  } else {
    return false;
  }
}

void ObjectReg::FormCaliMarkerCoordinateFrame() {
  int num_markers = cali_markers_pos.size();
  assert(num_markers == 3);
  int id_origin = -1;
  const double eps_dot_product = 0.1; 
  Vec axis_x;
  Vec axis_y;
  for (int i = 0; i < num_markers; ++i) {
    // Determine whether the ith point can be set as the origin.
    // Check the angle between axis is close to right angle or not.
    axis_x = cali_markers_pos[(i+1) % num_markers] - cali_markers_pos[i];
    axis_y = cali_markers_pos[(i+2) % num_markers] - cali_markers_pos[i];
    const double norm_axis_x = sqrt(axis_x * axis_x);
    const double norm_axis_y = sqrt(axis_y * axis_y);
    // Normalize to unit length.
    axis_x = axis_x / norm_axis_x;
    axis_y = axis_y / norm_axis_y;

    // Convention: Use the longer axis as the y axis.
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

bool ObjectReg::ReadTractablePoseFromMocap(MocapComm& mocap_comm) {
  Mocap::mocap_frame mocap_msg;
  bool flag = mocap_comm.GetMocapFrame(&mocap_msg);
  if (flag) {
    // Extract pose from mocap output.
    double tractable_pose[7];
    // TODO(Jiaji): Better handling of id matching without assuming consecutive id from 1.
    geometry_msgs::Pose pose = mocap_msg.body_poses[tractable_id - 1];
    tractable_pose[0] = pose.position.x;
    tractable_pose[1] = pose.position.y;
    tractable_pose[2] = pose.position.z;
    tractable_pose[3] = pose.orientation.x;
    tractable_pose[4] = pose.orientation.y;
    tractable_pose[5] = pose.orientation.z;
    tractable_pose[6] = pose.orientation.w;
    // Update the tf.
    tf_robot_mctractable.setPose(tractable_pose);
    return true;
  } else {
    return false;
  }
}

bool ObjectReg::GetGlobalObjectPose(MocapComm& mocap_comm, HomogTransf* obj_pose) {
  // Read tractable pose from mocap first.
  if (ReadTractablePoseFromMocap(mocap_comm)) {
    // Transform to the local object frame.
    *obj_pose = tf_robot_mctractable * tf_mctractable_obj;
    return true;
  } else {
    return false;
  }
}

void ObjectReg::ComputeTransformation() {
  tf_mctractable_obj = tf_robot_mctractable.inv() * tf_robot_calimarkers;
}

bool ObjectReg::Register(MocapComm& mocap_comm) {
  // Shell based interactive process.
  std::cout << "Object Registration Process Starts." << std::endl;
  
  // Set object name.
  std::cout << "Input Object Name" << std::endl;
  std::string name;
  std::cin >> name;
  SetObjName(name);

  // Set object tractable id.
  std::cout << "Input Tractable id. Make sure it's the same with Opti-track Windows setting!" << std::endl;
  std::cin >> tractable_id;

  // Calibrate mocap frame to robot base frame.
  std::cout << "Do you need to calibrate mocap frame w.r.t robot base frame first? Type y for yes and n for no." << std::endl;
  char ch;
  std::cin >> ch;
  if (ch == 'Y' || ch == 'y') {
    std::cout << "Type in the pose(7 numbers):" << std::endl;
    double pose_robot_mocap[7];
    for (int i = 0; i < 7; ++i) {
      std::cin >> pose_robot_mocap[i];
    }
    mocap_comm.SetMocapTransformation(pose_robot_mocap); 
  } else if (!(ch == 'N' || ch == 'n')) {
    std::cerr << "Cannot recognize input" << std::endl;
    return false;
  }
  // Read calibration marker paper and form local object frame.
  std::cout << "Place JUST the paper cali markers on the table." << std::endl;  
  std::cout << "Setting Up Cali Marker Frame" << std::endl;
  std::cout << "Enter any key to start acquiring" << std::endl;
  std::cin.ignore();
  std::cin.get();
  if (!ReadCaliMarkersFromMocap(mocap_comm)) {
    std::cerr << "Failed to read calibration markers" << std::endl;
    return false;
  } 
  
  // Put the object to align with paper markers and compute transformation.
  std::cout << "Now put the object on the paper." << std::endl;
  std::cout << "Note that the object should be registered in opti-track as tractable rigid body already." << std::endl;
  std::cout << "Enter any key to start acquiring" << std::endl;
  std::cin.ignore();
  std::cin.get();

  if (!ReadTractablePoseFromMocap(mocap_comm)) {
    std::cerr << "Failed to read tractable pose from mocap" << std::endl;
    return false;
  }
  
  std::cout << "Computing transformation." << std::endl;
  ComputeTransformation();
  std::cout << GetTransformation() << std::endl;

}

