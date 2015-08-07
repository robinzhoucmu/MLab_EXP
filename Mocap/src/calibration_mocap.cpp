#include "calibration_mocap.h"
#include <time.h>
#include <assert.h>
#include <math.h>


// kCenter plus/minus kDelta as boundary.
const double kDelta[3] = {145, 225, 25};
// A bit bigger than the measured length between the center of tool to the mocap marker. 
const double safe_height = 150 + 45;
// Hand chosen values (in robot base frame) that avoid collision.
// Robot Moves randomly around kCenter.
const double kCenter[3] = {500, 0, safe_height + kDelta[2]};

// Base frame: x pointing out to the vision node; 
// Have the robot tool frame mostly facing downward.
// Correspond to rotating the base frame about y axis for 180 degree.
const double kCenterAngles[3] = {0, M_PI, 0};
const double angle_range = 25;
const double kDeltaAngles[3] = {angle_range * M_PI / 180.0, 
				angle_range * M_PI / 180.0, 
				angle_range * M_PI / 180.0};

// Global mocap coords.
double global_mocap_cord[3];
// Global boolean to indicate whether the mocap is successfully captured.
bool flag_mocap_captured;

void Callback(const Mocap::mocap_frame& msg){
  global_mocap_cord[0] = global_mocap_cord[1] = global_mocap_cord[2] = -1;
  // Check the number of unidentified markers.
  // During calibration, we expect to see only one unidentified marker.
  int num_markers = msg.uid_markers.markers.size();
  flag_mocap_captured = (num_markers == 1);
  
  // Extract marker x y z coordinates.
  if (flag_mocap_captured) {
    const geometry_msgs::Point pt = msg.uid_markers.markers[0];
    global_mocap_cord[0] = pt.x;
    global_mocap_cord[1] = pt.y;
    global_mocap_cord[2] = pt.z;
  }
}

MocapCalibration::MocapCalibration() {
  ros::NodeHandle node;
  robot = new RobotComm(&node);
  InitRobotTransformation();
  // Subscribe to "Mocap" topic
  ros::NodeHandle node_sub;
  mocap_sub =  node_sub.subscribe("Mocap", 1000, Callback);
  
}

void MocapCalibration::InitRobotTransformation() {
  // Initialize tool frame to default (center of phalange).
  robot->SetTool(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  // Set identical between work object frame and robot base frame.
  robot->SetWorkObject(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  // Set relatively high speed.
  const double tcp = 50;
  const double ori = 10;
  robot->SetSpeed(tcp, ori);
}

void MocapCalibration::GenRandomTrajectory(const double center[3], 
					   const double delta[3], 
					   const double delta_angles[3],
					   int n_samples) {
  num_samples = n_samples;
  traj.clear();
  int counter = 0;
  while (counter < num_samples) {
    const HomogTransf rand_pose = GenerateRandomPose(center, delta, delta_angles);
    // Check joint limit.
    double dummy_joints[6];
    if (robot->GetIK(rand_pose, dummy_joints)) {
      traj.push_back(rand_pose);
      std::cout << counter << "th pose " << std::endl;
      std::cout << traj[counter];
      std::cout << std::endl;
      counter++;
    }
  } 
}

HomogTransf MocapCalibration::GenerateRandomPose(const double center[3], const double delta[3],
						 const double delta_angles[3]) {
  const int kMaxR = 1000;

  // Random xyz around center.
  Vec trans(3);

  // Random xyz rotation angles.
  double rand_angles[3];

  for (int j = 0; j < 3; ++j) {
    // Generate random number in [-1,1];
    const double r = 2 * (double(rand() % kMaxR) / kMaxR - 0.5);
    trans[j] = center[j] + r * delta[j];
    rand_angles[j] = kCenterAngles[j] + r * delta_angles[j];
  }
  
  // Generate corresponding random rotation matrix.
  RotMat rotMX;
  rotMX.rotX(rand_angles[0]);
  RotMat rotMY;
  rotMY.rotY(rand_angles[1]);
  RotMat rotMZ;
  rotMZ.rotZ(rand_angles[2]);
  RotMat rotM = rotMZ * rotMY * rotMX; 

  // Set random Homogenous Transformation.
  HomogTransf rand_pose(rotM, trans);
  //std::cout << rand_pose.getQuaternion() << std::endl;

  return rand_pose;
}

void MocapCalibration::RunTrajectory(std::ostream& out) {
  assert(traj.size() == num_samples);
  mocap_cords.clear();
  for (int i = 0; i < num_samples; ++i) {
    // Move the robot.
    MoveRobotOneStep(i);

    // Sleep for 0.5 second before start logging mocap data.
    ros::Duration(k_cusion_time).sleep();
    
    // Acquire Mocap Data.
    if (AcquireMocapData(i)) {
      std::cout << "Outputing " << i << "th way point" << std::endl;
      // Output to disk/stdout.
      OutputPoseAndMocap(i, out);
    }
  }
  // Set back to slow speed.
  robot->SetSpeed(20.0, 10.0);

}

void MocapCalibration::OutputPoseAndMocap(int step_id, std::ostream& out) {
  // Output the Rotation Matrix.
  const RotMat rotM = traj[step_id].getRotation();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out << rotM.v[i][j] << " ";
    }
    out << std::endl;
  }

  // Output the Translation.
  const Vec trans = traj[step_id].getTranslation();
  for (int i = 0; i < 3; ++i) {
    out << trans[i] << " ";
  }
  out << std::endl;
 
  // Output the Mocap data.
  for (int i = 0; i < 3; ++i) {
    out << mocap_cords[step_id][i] << " ";
  }
  out << std::endl;
}

void MocapCalibration::MoveRobotOneStep(int step_id) {
  // Move the robot to the next trajectory point.
  robot->SetCartesian(traj[step_id]);
  // Wait until robot reaches goal.
  while(robot->IsMoving());
}

bool MocapCalibration::AcquireMocapData(int step_id) {
    // Start acquiring mocap data and average them.
    int num_received_frames = 0; 
    bool flag_response = true;
    std::vector<double> avg_mocap_cords(3,0);
    std::vector<double> last_mocap_cords(3,0);
    const int freqHz = int (1.0 / k_read_period);
    ros::Rate r(freqHz);
    double last_secs = ros::Time::now().toSec();   
    while (num_received_frames < k_num_collections && flag_response) {
      ros::spinOnce();
      double cur_secs = ros::Time::now().toSec();
      if (flag_mocap_captured) {
	flag_response = true;
	// Update last_secs.
	last_secs = cur_secs;
	num_received_frames++;
	for (int i = 0; i < 3; ++i) {
	  avg_mocap_cords[i] += global_mocap_cord[i];
	  last_mocap_cords[i] = global_mocap_cord[i];
	}
	// Check for sudden jump.
	const double eps_jump = 1.5;
	for (int i = 0; i < 3; ++i) {
	  double diff = global_mocap_cord[i] - last_mocap_cords[i];
	  if (diff > eps_jump || diff < -eps_jump) {
	    flag_response = false;
	  } 
	}
      }
      r.sleep();
      if (cur_secs - last_secs > k_max_wait_time) {
	flag_response = false;
      }
    }
    for (int i = 0; i < 3; ++i) {
      avg_mocap_cords[i] = avg_mocap_cords[i] / num_received_frames; 
    }
    // Store into corresponding position in mocap_cords.
    mocap_cords.push_back(avg_mocap_cords);
    
    return flag_response; 
}

int main(int argc, char* argv[]) {
  srand (time(NULL));
  ros::init(argc, argv, "MocapCalibration");
  MocapCalibration mocap_cali;
  const int kNumRandSamples = 300;
  mocap_cali.GenRandomTrajectory(kCenter, kDelta, kDeltaAngles, kNumRandSamples);
  
  std::ofstream fout;
  fout.open("mocap_log_2.txt");
  mocap_cali.RunTrajectory(fout);
  // Debugging. 
  //mocap_cali.RunTrajectory(std::cout);
  fout.close();
  return 0;
}
