#include "push_exp.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <ctime>
// Global flag indicating whether the robot is pushing the object.
bool flag_is_pushing = false;
std::vector<geometry_msgs::WrenchStamped> ft_wrenches;
std::vector<geometry_msgs::PoseStamped> robot_poses;

// Asynchronous force logging.
void CallBackForceLogging(const geometry_msgs::WrenchStamped& msg_force) {
  if (flag_is_pushing) {
    geometry_msgs::WrenchStamped contact_wrench;
    contact_wrench.wrench = msg_force.wrench;
    contact_wrench.header.stamp = ros::Time::now();
    ft_wrenches.push_back(contact_wrench);
  }
}

void CallBackRobotPoseLogging(const robot_comm::robot_CartesianLog& msg_robot_pose) {
  if (flag_is_pushing) {
    geometry_msgs::PoseStamped robot_pose_stamped;
    // Extract cartesian pose information.
    geometry_msgs::Point pt;
    pt.x = msg_robot_pose.x;
    pt.y = msg_robot_pose.y;
    pt.z = msg_robot_pose.z;
    robot_pose_stamped.pose.position = pt;
    geometry_msgs::Quaternion q;
    q.w = msg_robot_pose.q0;
    q.x = msg_robot_pose.qx;
    q.y = msg_robot_pose.qy;
    q.z = msg_robot_pose.qz;
    robot_pose_stamped.pose.orientation = q;
    // Add time stamp.
    robot_pose_stamped.header.stamp = ros::Time::now();
    robot_poses.push_back(robot_pose_stamped);
  }
}

PushExp::PushExp(ros::NodeHandle *n) {
  nh = n;
  Initialize();
}

void PushExp::Initialize() {
  // Extract the flag indicating whether to execute the robot to goal position.
  execution_flag = GLParameters::execution_flag;
  read_pushaction_flag = GLParameters::read_pushaction_flag;

  num_pushes = GLParameters::num_pushes;
  kNumMocapReadings = GLParameters::mocap_num_readings;
  kReadDuration = GLParameters::mocap_read_duration;
  safe_height = GLParameters::safe_height;

  memcpy(robot_rest_cart, GLParameters::robot_rest_cart, sizeof(GLParameters::robot_rest_cart));
  
  std::cout << "constructing push object from file" << std::endl;
  std::string obj_file_cali = GLParameters::workobj_file_cali;
  std::string obj_file_geo = GLParameters::workobj_file_geometry;
  std::cout << obj_file_cali << "," << obj_file_geo << std::endl;
  push_object = new PushObject(obj_file_cali, obj_file_geo);
  
  std::cout << "new push generator" << std::endl;
  push_plan_gen = new PushGenerator();
  
  robot = new RobotComm(nh);
  // Set tool and work object transform.
  InitializeRobot();
  // Initialize mocap transform.
  InitializeMocapTransform();
  // Initialize force logging related stuffs.
  std::cout << "async spinner" << std::endl;
  async_spinner = new ros::AsyncSpinner(2);
  flag_is_pushing = false;
  force_sub = nh->subscribe("netft_data", 1, CallBackForceLogging);
  robot_pose_sub = nh->subscribe("robot_CartesianLog", 1, CallBackRobotPoseLogging);
}

void PushExp::InitializeRobot() {
  // Set tool.
  double tf_tool[7];
  memcpy(tf_tool, GLParameters::robot_set_tool, sizeof(GLParameters::robot_set_tool));
  robot->SetTool(tf_tool[0], tf_tool[1], tf_tool[2], 
		 tf_tool[3], tf_tool[4], tf_tool[5], tf_tool[6]);
  // Set workobj.
  double tf_workobj[7];
  memcpy(tf_workobj, GLParameters::robot_set_workobj, sizeof(GLParameters::robot_set_workobj));
  robot->SetWorkObject(tf_workobj[0], tf_workobj[1], tf_workobj[2], 
		       tf_workobj[3], tf_workobj[4], tf_workobj[5], tf_workobj[6]);
  // Set speed.
  double tcp_speed = GLParameters::robot_tcp_speed;
  double ori_speed = GLParameters::robot_ori_speed;
  robot->SetSpeed(tcp_speed, ori_speed);
}

void PushExp::InitializeMocapTransform() {
  ros::NodeHandle n;
  MocapComm mocap_comm(&n);
  double mocap_cali_tf[7];
  memcpy(mocap_cali_tf, GLParameters::mocap_cali_tf, sizeof(GLParameters::mocap_cali_tf));
  mocap_comm.SetMocapTransformation(mocap_cali_tf);
}

// Remember to stop async spinner after robot finishes pushing. 
void PushExp::LogPushForceAndRobotPoseAsync() {
  // Clear previous logged data.
  ft_wrenches.clear();
  robot_poses.clear();
  // Start async thread for logging force.
  async_spinner->start();
}

bool PushExp::SetCartesian(HomogTransf tf, bool use_joint) {
  // Check for z value greater than minimum height.
  assert(tf.getTranslation()[2] > GLParameters::min_height);

  std::cout <<"Robot Set Cartesian: ";
  std::cout << tf.getTranslation() << "," <<tf.getQuaternion() << std::endl;
  if (execution_flag) {
    bool flag_robot_move = false;
    if (use_joint) {
      flag_robot_move = robot->SetCartesianJ(tf);
    } else {
      flag_robot_move = robot->SetCartesian(tf);
    }
    return flag_robot_move;
  } else {
    return true;
  }
}

bool PushExp::SetJoints(double joints[6]) {
  HomogTransf tf;
  robot->GetFK(joints, tf);
  // Check for z value greater than minimum height.
  assert(tf.getTranslation()[2] > GLParameters::min_height);
  if (execution_flag) {
    return robot->SetJoints(joints);
  }
  return true;
}

// Average over a bunch of planar poses recorded in obj_poses. 
bool PushExp::ComputeAveragePose(HomogTransf* pose_tf) {
  // Convert vector of homogtransf to quaternion.
  std::cout << "compute average pose" << std::endl;
  const int num_poses = obj_poses.size();
  assert(num_poses >= 1);
  Vec sum_trans(0.0, 3);
  Quaternion sum_quat(0.0);
  for (int i = 0; i < num_poses; ++i) {
    // TODO(Jiaji): Add consistency check logic.
    const HomogTransf& pose_i = obj_poses[i];
    // convert each homogtransformation to quaternion.
    sum_trans = sum_trans + pose_i.getTranslation();
    Quaternion q = pose_i.getQuaternion();
    if (q[0] < 0) {
      q = -q;
    }
    sum_quat = sum_quat + q;
  }
  // Compute average transformation and quaternion.
  Vec avg_trans = sum_trans / num_poses;
  Quaternion avg_quat = sum_quat / num_poses;
  // Normalize the quaternion.
  avg_quat.normalize();
  pose_tf->setTranslation(avg_trans);
  pose_tf->setQuaternion(avg_quat);
  return true;
}

bool PushExp::AcquireObjectStablePose(HomogTransf* pose_tf) {
  if (flag_robot_away) {
    //Mocap::mocap_frame mocap_msg;
    HomogTransf obj_pose_tf;
    //const int tractable_id = push_object->GetTractableId();
    double t_sleep = kReadDuration / (kNumMocapReadings + 1);
    int num_acquired_frames = 0;
    double start_time = ros::Time::now().toSec();
    double elapsed_time = 0.0;
    obj_poses.clear();
    while (num_acquired_frames < kNumMocapReadings && elapsed_time < kReadDuration) {
      if (push_object->GetGlobalObjPose(&obj_pose_tf)) {
	//std::cout << obj_pose_tf << std::endl;
	obj_poses.push_back(obj_pose_tf);
	++num_acquired_frames;
      }
      ros::Duration(t_sleep).sleep();
      elapsed_time = ros::Time::now().toSec() - start_time;
    } // while
    // Average Reading.
    bool flag_average = ComputeAveragePose(pose_tf);
    if (!flag_average) {
      std::cerr << "Acquired readings are problematic" << std::endl;
      return false;
    } else {
      std::cout << "Successfully acquired pose" << std::endl;
      //std::cout << *pose_tf << std::endl;
      return true;
    }
  } else {       
    std::cerr << "Robot is NOT in a far away position" << std::endl;
    return false;
  }
}

bool PushExp::SinglePushPipeline() {
  // Check whether the object needs to be reset.
  CheckForReset();
  
  // Read object pose.
 
  if (!AcquireObjectStablePose(&pre_push_obj_pose)) {
    std::cerr << "Before Push: Cannot acquire object pose from mocap" << std::endl; 
    return false;
  } else {
    std::cout << "Before Push Pose" << std::endl;
    std::cout << pre_push_obj_pose << std::endl;
  }
  // Generate trajectory plan.
  assert(GeneratePushPlan(pre_push_obj_pose));
  // Execute trajectory and log force data.
  assert(ExecRobotPushAndLogForce());
  // Read stable object pose after robot move back to initial position.
  if (!AcquireObjectStablePose(&post_push_obj_pose)) {
    std::cerr << "After Push: Cannot acquire object pose from mocap" << std::endl; 
    return false;
  } else {
    std::cout << "After Push Pose" << std::endl;
    std::cout << post_push_obj_pose << std::endl;
  }
  
  return true;
}

bool PushExp::GeneratePushPlan(HomogTransf pre_push_obj_pose) {
  bool flag_kinematic_feasible = false;
  while (!flag_kinematic_feasible) {
    // Generate a random local pushing direction.
    //PushAction push_action;
    if (!read_pushaction_flag) {
      if (!push_plan_gen->generateRandomPush(
	    *push_object, &push_action, GLParameters::push_type)) {
	std::cerr << "Cannot generate a random push direction" << std::endl;
	return false;  
      }
    } else {
      DeserializePushAction(fin_pushaction, &push_action);
    }
    // Generate robot trajectory in global robot frame.
    const Vec tableNormal = Vec("0 0 1", 3);
    // Lift up the pushing plane a bit from the ground surface.
    // To do so, lift up the Z value for object pose.
    HomogTransf push_obj_pose_lifted;
    Quaternion q = pre_push_obj_pose.getQuaternion();
    Vec v = pre_push_obj_pose.getTranslation();
    // Add part of the object height to Z.
    const int ind_z = 2;
    const double height_lift_ratio = 0.25;
    v[ind_z] = v[ind_z] + push_object->GetHeight() * height_lift_ratio;
    push_obj_pose_lifted.setTranslation(v);
    push_obj_pose_lifted.setQuaternion(q);

    if (!push_plan_gen->generateTrajectory(
	     push_action, push_obj_pose_lifted, tableNormal, &robot_push_traj)) {
      std::cerr << "Cannot generate robot trajectory" << std::endl;
      return false;
    }
    bool tmp_flag = true;
    double dummy_joints[6];
    // Check for the first move above pose feasibility.
    const int ind_approach = 0;
    HomogTransf tf_above = GetPoseAbove(robot_push_traj[ind_approach]);
    if (!robot->GetIK(tf_above, dummy_joints)) {
      tmp_flag = false;
    }
    // Check for trajectory kinematic feasiblity.
    for (int i = 0; i < robot_push_traj.size(); ++i) {
      if (!robot->GetIK(robot_push_traj[i], dummy_joints)) {
	tmp_flag = false;
	break;
      }
    }
    flag_kinematic_feasible = tmp_flag;
  } // While
  return true;
}

bool PushExp::ExecRobotPushAndLogForce() {
  const int ind_approach = 0;
  const int ind_close = 1;
  const int ind_push = 2;
  const int ind_retract = 3;
  
  assert(robot_push_traj.size() == 4);

  HomogTransf approach_pose = robot_push_traj[ind_approach];
  std::cout << "Move to the resting position first" << std::endl;
  assert(RobotMoveToRestingState());
   // Move to above. 
  std::cout << "Move to Above" << std::endl;
  assert(RobotMoveToAbove(approach_pose));
  // Move to relatively far approach_pose.
  std::cout << "Approach" << std::endl;
  // Set slower speed for moving downwards to approach pose.
  robot->SetSpeed(10.0,10.0);
  assert(SetCartesian(robot_push_traj[ind_approach]));
  // Recover original speed. 
  robot->SetSpeed(GLParameters::robot_tcp_speed, GLParameters::robot_ori_speed);
  // Move close to the object to prepare pushing.
  std::cout << "Getting close" << std::endl;
  assert(SetCartesian(robot_push_traj[ind_close]));
  ros::Duration(2.0).sleep();
  const double slow_speed_tcp = 2.5;
  const double slow_speed_ori = 2.5;
  robot->SetSpeed(slow_speed_tcp, slow_speed_ori);
  flag_is_pushing = true;
  // Reset tcp point for two point cor-based push.
  if (push_action.pushType == "TwoPointRotation") {
    double tf_tool[7];
    memcpy(tf_tool, GLParameters::robot_set_tool, sizeof(GLParameters::robot_set_tool));
    HomogTransf tool_tf_old = HomogTransf(tf_tool);
    HomogTransf tool_tf_new;
    assert(push_plan_gen->getToolTransformTwoPointsPush(tool_tf_old, push_action, &tool_tf_new ));
    Vec tcp_new = tool_tf_new.getTranslation();
    Quaternion q_new = tool_tf_new.getQuaternion();
    robot->SetTool(tcp_new[0],tcp_new[1],tcp_new[2],
		   q_new[0],q_new[1],q_new[2],q_new[3]);
  }
  // Now start logging. 
  LogPushForceAndRobotPoseAsync();
  // Push the object.
  std::cout << "Push into" << std::endl;
  assert(SetCartesian(robot_push_traj[ind_push]));
  // After the robot pushes in, finish async force logging.
  async_spinner->stop();
  flag_is_pushing = false;
  // Recover original speed. 
  robot->SetSpeed(GLParameters::robot_tcp_speed, GLParameters::robot_ori_speed);
  // Retract the robot to break contact.
  std::cout << "Retract" << std::endl;
  assert(SetCartesian(robot_push_traj[ind_retract]));
  // Robot Leave Contact.
  std::cout << "Move back to resting position" << std::endl;
  // Reset back to original tool transform.
  if (push_action.pushType == "TwoPointRotation") {
    double tf_tool[7];
    memcpy(tf_tool, GLParameters::robot_set_tool, sizeof(GLParameters::robot_set_tool));
    robot->SetTool(tf_tool[0], tf_tool[1], tf_tool[2], 
		   tf_tool[3], tf_tool[4], tf_tool[5], tf_tool[6]);
  }
  assert(RobotMoveToRestingState());
  return true;
}

// Get current robot pose. Move straight up and then move to resting state.
bool PushExp::RobotMoveToRestingState() {
  // Read current pose, move up along z. 
  const int ind_z = 2;
  double robot_nxt_cart[7];
  robot->GetCartesian(robot_nxt_cart);
  robot_nxt_cart[ind_z] = robot_rest_cart[ind_z];
  HomogTransf robot_nxt_tf(robot_nxt_cart);
  assert(SetCartesian(robot_nxt_tf));

  HomogTransf robot_rest_tf(robot_rest_cart);
  //bool flag_move_success = SetCartesian(robot_rest_tf, true);
  // A hack: joint 6 could be \theta +- 360. We always choose the angle closer to 0 so
  // it's harder to hit joint limit. 
  double joints[6];
  assert(robot->GetIK(robot_rest_tf, joints));
  const int ind_last_j = 5;
  joints[ind_last_j] = 
    (joints[ind_last_j] > 360)?(joints[ind_last_j] - 360):joints[ind_last_j];
  joints[ind_last_j] = 
    (joints[ind_last_j] < -360)?(joints[ind_last_j] + 360):joints[ind_last_j];

  bool flag_move_success = SetJoints(joints);

  if (!flag_move_success) {
    flag_robot_away = false;
    std::cerr << "Robot did not successfully move to resting position" << std::endl;
  } else {
    flag_robot_away = true;
  }
  return flag_move_success;
}

bool PushExp::RobotMoveToAbove(HomogTransf pose_below) {
  HomogTransf pose_above = GetPoseAbove(pose_below);
  bool flag = SetCartesian(pose_above, true); 
  if (!flag) {
    std::cerr << "Robot fail to move above " << std::endl;
    std::cerr << pose_below << std::endl;
  } 
  return flag;
}

HomogTransf PushExp::GetPoseAbove(HomogTransf pose_below) {
  Vec trans = pose_below.getTranslation();
  Quaternion quat = pose_below.getQuaternion();
  // Set the z compenent to safe_height.
  trans[2] = safe_height;
  HomogTransf pose_above(quat, trans);
  return pose_above;
}
void PushExp::SerializeSensorInfo(std::ostream& fout) {
  // Output pre and post object poses.
  SerializeHomogTransf(pre_push_obj_pose, fout);
  SerializeHomogTransf(post_push_obj_pose, fout);
  // Output force signals.
  fout << ft_wrenches.size() << std::endl;
  for (int i = 0; i < ft_wrenches.size(); ++i) {
    const geometry_msgs::WrenchStamped& w= ft_wrenches[i];
    fout << w.header.stamp << " " << w.wrench.force.x << " " 
	 << w.wrench.force.y << std::endl;
  }
  // Output robot poses.
  fout << robot_poses.size() << std::endl;
  for (int i = 0; i < robot_poses.size(); ++i) {
    geometry_msgs::PoseStamped p = robot_poses[i];
    fout << p.header.stamp << " ";
    fout << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " ";
    fout << p.pose.orientation.w << " " << p.pose.orientation.x << " " 
	 <<  p.pose.orientation.y << " " << p.pose.orientation.z << std::endl; 
    
  }
}
void PushExp::SerializePushActionInfo(std::ostream& fout) {
  // Output pushType.
  fout << push_action.pushType << std::endl;
  // Output pushPoint.
  Vec v = push_action.pushPoint;
  fout << v[0] << " " << v[1] << " " << v[2] << std::endl;
  double dist;
  if (push_action.pushType == "TwoPointRotation") {
    // Output COR.
    v = push_action.cor;
    dist = push_action.rotAngle;
  } else {
    // Output pushVector.
    v = push_action.pushVector;
    dist = push_action.penetrationDist;
  }
  fout << v[0] << " " << v[1] << " " << v[2] << std::endl;
  // Output approachVector.
  v = push_action.approachVector;
  fout << v[0] << " " << v[1] << " " << v[2] << std::endl;
  // Output penetration distance/rotAngle and moveClostDist.
  fout << dist << " " << push_action.moveCloseDist << std::endl;
}

bool PushExp::DeserializePushAction(std::istream& fin, PushAction *push) {
  fin >> push->pushType;
  Vec push_point(3);
  fin >> push_point[0] >> push_point[1] >> push_point[2];
  push->pushPoint = push_point;
  double move_amount;
  double moveClose_dist;
  Vec move_vector(3);
  fin >> move_vector[0] >> move_vector[1] >> move_vector[2];
  Vec approach_vector(3);
  fin >> approach_vector[0] >> approach_vector[1] >> approach_vector[2];
  fin >> move_amount >> moveClose_dist;
  if (push->pushType == "TwoPointRotation") {
    push->cor = move_vector;
    push->rotAngle = move_amount;
  } else {
    push->pushVector = move_vector;
    push->penetrationDist = move_amount;
  }
  push->approachVector = approach_vector;
  push->moveCloseDist = moveClose_dist;
  push->initialDist = GLParameters::default_init_dist;
  push->retractionDist = GLParameters::default_retraction_dist;
  return true;
}

void PushExp::SerializeHomogTransf(const HomogTransf& tf, std::ostream& fout) {
  Vec trans = tf.getTranslation();
  Quaternion quat = tf.getQuaternion();
  fout << trans[0] << " " << trans[1] << " " << trans[2] << " ";
  fout << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << std::endl;
 
}

bool PushExp::CheckForReset() {
  return true;
}

bool PushExp::ConfirmStart() {
  std::cout << "Print diagnostic information to confirm starting experiment" << std::endl;
 
  // Print robot information.
  HomogTransf tf_tool;
  HomogTransf tf_workobj;
  double tcp, ori;
  int zone;
  robot->GetState(tcp, ori, zone, tf_workobj, tf_tool);
  std::cout << "Robot tool setting:" << std::endl;
  std::cout << tf_tool.getTranslation() << "," << tf_tool.getQuaternion() << std::endl;
  std::cout << "Robot work object setting:" << std::endl;
  std::cout << tf_workobj.getTranslation() << "," << tf_workobj.getQuaternion() << std::endl;
  std::cout << "tpc, ori, zone:" << std::endl;
  std::cout << tcp << "," << ori << "," << zone << std::endl;
  HomogTransf robot_tf;
  robot->GetCartesian(robot_tf);
  std::cout << "Robot pose now:" << std::endl;
  std::cout << robot_tf.getTranslation() << "," << robot_tf.getQuaternion() << std::endl;
 
  // Print object pose.
  HomogTransf obj_pose_tf;
  push_object->GetGlobalObjPose(&obj_pose_tf);
  std::cout << "Object pose in global robot frame:" << std::endl;
  std::cout << obj_pose_tf.getTranslation() <<"," << obj_pose_tf.getQuaternion() << std::endl;
  std::cout << "axis: " << obj_pose_tf.getQuaternion().getAxis() 
	    << ", angle: " << obj_pose_tf.getQuaternion().getAngle() << std::endl;
  // Ask to confirm start running experiments.
  std::cout << "Press y for start and n for stop" << std::endl;
  char ch;
  std::cin >> ch;
  if (ch == 'Y' || ch == 'y') {
    return true;
  } else {
    return false;
  }
}

void PushExp::Run() {
  assert(num_pushes >= 1);      
  assert(RobotMoveToRestingState());
  std::string file_name = GLParameters::sensor_log_file;
  double secs = ros::Time::now().toSec();
  std::stringstream convert;
  std::time_t result = std::time(NULL);
  std::string str_cur_time =  std::string(std::asctime(std::localtime(&result)));
  str_cur_time = str_cur_time.substr(0, str_cur_time.size()-1);
  //std::cout << str_cur_time;
  convert << num_pushes;
  std::string str_num_pushes = convert.str();

  std::string file_name_sensor = 
    file_name + str_cur_time + "_sensor" + str_num_pushes + ".txt";
  std::string file_name_push_action = 
    file_name + str_cur_time + "_pushaction" + str_num_pushes + ".txt";
  //std::cout << file_name_sensor << std::endl;

  if (ConfirmStart()) {
    std::ofstream fout(file_name_sensor.c_str());
    std::ofstream fout_push_action(file_name_push_action.c_str());
    
    fout << num_pushes << std::endl;
    fout_push_action << num_pushes << std::endl;
    if (read_pushaction_flag) {
      fin_pushaction.open(GLParameters::pushaction_file.c_str());
      int total_input_pushes;
      fin_pushaction >> total_input_pushes;
      std::cout << total_input_pushes << std::endl;
      assert(total_input_pushes >= num_pushes);
    }
    for (int i = 0; i < num_pushes; ++i) {
      std::cout << "Started to run push " << i << std::endl;
      SinglePushPipeline();
      SerializeSensorInfo(fout);
      SerializePushActionInfo(fout_push_action);
      ros::Duration(3.0).sleep();
    }
    fout.close();
    fout_push_action.close();
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "PushExp");
  GLParameters::ReadParameters();
  ros::NodeHandle n;
  PushExp push_exp_inst(&n);
  push_exp_inst.Run();
  return 0;
}
