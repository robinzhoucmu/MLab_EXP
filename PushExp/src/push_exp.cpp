#include "push_exp.h"
#include "geometry_msgs/WrenchStamped.h"

// Global flag indicating whether the robot is pushing the object.
bool flag_is_pushing;
std::vector<geometry_msgs::Wrench> ft_wrenches;

// Asynchronous force logging.
void CallBackForceLogging(const geometry_msgs::WrenchStamped& msg_force) {
  if (flag_is_pushing) {
    ft_wrenches.push_back(msg_force.wrench);
  }
}

PushExp::PushExp(ros::NodeHandle *n) {
  nh = n;
  Initialize();
}

void PushExp::Initialize() {
  num_pushes = GLParameters::num_pushes;
  kNumMocapReadings = GLParameters::mocap_num_readings;
  kReadDuration = GLParameters::mocap_read_duration;
  safe_height = GLParameters::safe_height;

  memcpy(robot_rest_cart, GLParameters::robot_rest_cart, sizeof(robot_rest_cart));
  
  std::string obj_file_cali = GLParameters::workobj_file_cali;
  std::string obj_file_geo = GLParameters::workobj_file_geometry;
  push_object = new PushObject(obj_file_cali, obj_file_geo);
  
  push_plan_gen = new PushGenerator();
  
  robot = new RobotComm(nh);
  
  async_spinner = new ros::AsyncSpinner(1);
  force_sub = nh->subscribe("netft_data", 1000, CallBackForceLogging);
}


// Remember to stop async spinner after robot finishes pushing. 
void PushExp::LogPushForceAsync() {
  // Clear previous logged data.
  ft_wrenches.clear();
  // Start async thread for logging force.
  async_spinner->start();
}

bool PushExp::ComputeAveragePose(HomogTransf* avg_pose_tf) {
  return true;
}

bool PushExp::AcquireObjectStablePose(HomogTransf* pose_tf) {
  if (flag_robot_away) {
    //Mocap::mocap_frame mocap_msg;
    HomogTransf obj_pose_tf;
    const int tractable_id = push_object->GetTractableId();
    double t_sleep = kReadDuration / (kNumMocapReadings + 1);
    int num_acquired_frames = 0;
    double start_time = ros::Time::now().toSec();
    double elapsed_time = 0.0;
    obj_poses.clear();
    while (num_acquired_frames < kNumMocapReadings && elapsed_time < kReadDuration) {
      // if (mocap_comm.GetMocapFrame(&mocap_msg)) {
      // 	++num_acquired_frames;
      // 	assert(tractable_id <= mocap_msg.body_poses.size());
      // 	const geometry_msgs::Pose pose = mocap_msg.body_poses[tractable_id - 1];
      // 	// Add to obj_poses vector.
      // 	obj_poses.push_back(pose);
      // }
      if (push_object->GetGlobalObjPose(&obj_pose_tf)) {
	obj_poses.push_back(obj_pose_tf);
	++num_acquired_frames;
      }
      ros::Duration(t_sleep).sleep();
      elapsed_time = ros::Time::now().toSec() - start_time;
    }
    // Average Reading.
    bool flag_average = ComputeAveragePose(pose_tf);
    if (!flag_average) {
      std::cerr << "Acquired readings are problematic" << std::endl;
      return false;
    } 
    return true;
  } else {
    std::cerr << "Robot is NOT in a far away position" << std::endl;
    return false;
  }
}

bool PushExp::SinglePushPipeline() {
  // Check whether the object needs to be reset.
  CheckForReset();
  
  // Read object pose.
  HomogTransf pre_push_obj_pose;
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
  HomogTransf post_push_obj_pose;
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
  // Generate a random local pushing direction.
  PushAction push_action;
  if (!push_plan_gen->generateRandomPush(*push_object, &push_action)) {
    std::cerr << "Cannot generate a random push direction" << std::endl;
    return false;  
  }
  
  // Generate robot trajectory in global robot frame.
  const Vec tableNormal = Vec("0 0 1", 3);
  if (!push_plan_gen->generateTrajectory(push_action, pre_push_obj_pose, 
					tableNormal, &robot_push_traj)) {
    std::cerr << "Cannot generate robot trajectory" << std::endl;
    return false;
  }
  return true;
}

bool PushExp::ExecRobotPushAndLogForce() {
  const int ind_approach = 0;
  const int ind_close = 1;
  const int ind_push = 2;
  const int ind_retract = 3;
  
  assert(robot_push_traj.size() == 4);

  HomogTransf approach_pose = robot_push_traj[ind_approach];
  // Move to above. 
  assert(RobotMoveToAbove(approach_pose));
  // Move to relatively far approach_pose.
  assert(robot->SetCartesian(robot_push_traj[ind_approach]));
  // Move close to the object to prepare pushing.
  assert(robot->SetCartesian(robot_push_traj[ind_close]));
  // Now start logging. 
  LogPushForceAsync();
  // Push the object.
  assert(robot->SetCartesian(robot_push_traj[ind_push]));
  // After the robot pushes in, finish async force logging.
  async_spinner->stop();
  // Retract the robot to break contact.
  assert(robot->SetCartesian(robot_push_traj[ind_retract]));
  // Robot Leave Contact.
  assert(RobotMoveToRestingState());
  return true;
}

bool PushExp::RobotMoveToRestingState() {
  bool flag_move_success = robot->SetCartesian(robot_rest_cart);
  if (!flag_move_success) {
    std::cerr << "Robot did not successfully move to resting position" << std::endl;
  }
  return flag_move_success;
}

bool PushExp::RobotMoveToAbove(HomogTransf pose_below) {
  Vec trans = pose_below.getTranslation();
  Quaternion quat = pose_below.getQuaternion();
  // Set the z compenent to safe_height.
  trans[2] = safe_height;
  HomogTransf pose_above(quat, trans);
  
  bool flag = robot->SetCartesian(pose_above); 
  if (!flag) {
    flag_robot_away = false;
    std::cerr << "Robot fail to move above " << std::endl;
    std::cerr << pose_below << std::endl;
  } else {
    flag_robot_away = true;
  }
  return flag;
}

bool PushExp::CheckForReset() {
  return true;
}

void PushExp::Run() {
  assert(num_pushes >= 1);
  for (int i = 0; i < num_pushes; ++i) {
    std::cout << "Started to run push " << i << std::endl;
    SinglePushPipeline();
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
