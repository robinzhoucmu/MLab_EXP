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
// Remember to stop async spinner after robot finishes pushing. 
void PushExp::LogPushForceAsync() {
  // Clear previous logged data.
  ft_wrenches.clear();
  // Start async thread for logging force.
  async_spinner.start();
}

bool PushExp::ComputeAveragePose(HomogTransf* avg_pose_tf) {
  return true;
}

bool PushExp::AcquireObjectStablePose(HomogTransf* pose_tf) {
  if (flag_robot_away) {
    //Mocap::mocap_frame mocap_msg;
    HomogTransf obj_pose_tf;
    const int tractable_id = push_object.GetTractableId();
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
      if (push_object.GetGlobalObjPose(&obj_pose_tf)) {
	obj_poses.push_back(obj_pose_tf);
	++num_acquired_frames;
      }
      ros::Duration(t_sleep).sleep();
      elapsed_time = ros::Time::now().toSec() - start_time;
    }
    // Average Reading.
    bool flag_average = ComputeAveragePose(pose_tf);
    if (!flag_average) {
      std::cerr << "Acquired readings are wrong" << std::endl;
      return false;
    } 
    return true;
  } else {
    std::cerr << "Robot is NOT in a far away position" << std::endl;
    return false;
  }
}

bool PushExp::GeneratePushPlan() {
  // Check whether the object needs to be reset.
  CheckForReset();

  // Generate a random local pushing direction.
  PushAction push_action;
  if (!push_plan_gen.generateRandomPush(push_object, &push_action)) {
    std::cerr << "Cannot generate a random push direction" << std::endl;
    return false;  
  }
  // Read object pose.
  HomogTransf pre_push_obj_pose;
  if (!AcquireObjectStablePose(&pre_push_obj_pose)) {
    std::cerr << "Cannot acquire object pose from mocap" << std::endl; 
    return false;
  }
  // Generate robot trajectory in global robot frame.
  const Vec tableNormal = Vec("0 0 1", 3);
  if (!push_plan_gen.generateTrajectory(push_action, pre_push_obj_pose, 
					tableNormal, &robot_push_traj)) {
    std::cerr << "Cannot generate robot trajectory" << std::endl;
    return false;
  }
  return true;
}

bool PushExp::CheckForReset() {
  return true;
}


int main(int argc, char* argv[]) {
  return 0;
}
