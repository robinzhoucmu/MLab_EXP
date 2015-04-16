#include "publisher.h"

MocapPublisher::MocapPublisher(ros::NodeHandle *n) {
  nodeHandle = n;
  // Set as default frequency.
  SetPublishFrequency(Globals::kPublishFreq);
  // Without given transformation: set transformation (from robot base to mocap frame) 
  // as identity. Therefore mocap just return in its (unknown/uncalibrated) native 
  // coordinate frame.
  double pose[7] = {0,0,0,1,0,0,0};
  setMocapTransformation(pose);
}

MocapPublisher::~MocapPublisher() {
  handle_mocap_GetMocapFrame.shutdown();
  handle_mocap_SetMocapTransformation.shutdown();
}

void MocapPublisher::SetPublishFrequency(int _pub_freq_hz) {
  pub_freq_hz = _pub_freq_hz;
}

void MocapPublisher::EstablishCommunications(int argc, char* argv[]) {  
  // Sockets Initialization.
  sdCommand = -1;
  sdData = -1;
   
  // Catch ctrl-c and terminate gracefully.
  signal(SIGINT, Globals::Terminate);

  // Set addresses
  Globals::ReadOpts(argc, argv);
   
  // Create sockets
  sdCommand = NatNet::createCommandSocket( Globals::localAddress );
  sdData = NatNet::createDataSocket( Globals::localAddress );

  // Ensure that the socket creation was successful
  assert(sdCommand != -1); 
  assert(sdData != -1);
  
  // Create Listeners with the sockets.
  CreateListener();
}

void MocapPublisher::CreateListener() {
  // Version number of the NatNet protocol, as reported by the server.
  unsigned char natNetMajor;
  unsigned char natNetMinor;
  
  // Use this socket address to send commands to the server.
  // Default NatNet command port is 1510.
  struct sockaddr_in serverCommands = 
    NatNet::createAddress(Globals::serverAddress, NatNet::commandPort);

  printf("About to start command listener\n");
   
  // Start the CommandListener in a new thread.
  commandListener = new CommandListener(sdCommand);
  commandListener->start();
  printf("cmd listener started. about to ping\n");

  // Send a ping packet to the server so that it sends us the NatNet version
  // in its response to commandListener.
  NatNetPacket ping = NatNetPacket::pingPacket();
  ping.send(sdCommand, serverCommands);
  printf("ping sent. waiting...\n");
   
  // Wait here for ping response to give us the NatNet version.
  commandListener->getNatNetVersion(&natNetMajor, &natNetMinor);
  printf("ping recieved. Major = %u, Minor = %u \n",natNetMajor,natNetMinor);
   
  // Start up a FrameListener in a new thread.
  frameListener = new FrameListener(sdData, natNetMajor, natNetMinor);
  frameListener->start();  
}

void MocapPublisher::ExtractPoseMsg(const MocapFrame& mframe, 
				    Mocap::mocap_frame* msg) {
  // See NatNet.h file for relevant data structures of MocapFrame.
  // Extract unidenfied markers (e.g., single calibration marker).
  const int num_uid_markers = mframe.unIdMarkers().size();
  msg->uid_markers.markers.clear();
  for (int i = 0; i < num_uid_markers; ++i) {
    const Point3f& pt = mframe.unIdMarkers()[i];
    geometry_msgs::Point uid_marker_pt;
    uid_marker_pt.x = k_unit * pt.x;
    uid_marker_pt.y = k_unit * pt.y;
    uid_marker_pt.z = k_unit * pt.z;
    // Transform the point.
    transformPoint(&uid_marker_pt);
    msg->uid_markers.markers.push_back(uid_marker_pt);
  }
  // Extract idenfied marker sets.
  const int num_id_marker_sets = mframe.markerSets().size();
  msg->id_marker_sets.clear();
  for (int i = 0; i < num_id_marker_sets; ++i) {
    const MarkerSet marker_set = mframe.markerSets()[i]; 
    const int marker_set_size = marker_set.markers().size();
    Mocap::marker_set id_marker_set;
    for (int j = 0; j < marker_set_size; ++j) {
      const Point3f& pt = marker_set.markers()[j];
      geometry_msgs::Point id_marker_pt;
      id_marker_pt.x = k_unit * pt.x;
      id_marker_pt.y = k_unit * pt.y;
      id_marker_pt.z = k_unit * pt.z;
      // Transform the point.
      transformPoint(&id_marker_pt);
      id_marker_set.markers.push_back(id_marker_pt);
    }
    msg->id_marker_sets.push_back(id_marker_set);
  }
  // Extract tracked body/ies poses.
  const int num_bodies = mframe.rigidBodies().size();
  for (int i = 0; i < num_bodies; i++) {
    const RigidBody& body = mframe.rigidBodies()[i];
    geometry_msgs::Pose pose;
    // Copy x,y,z.
    pose.position.x = k_unit * body.location().x;
    pose.position.y = k_unit * body.location().y;
    pose.position.z = k_unit * body.location().z;
    // Copy Quaternion.
    pose.orientation.x = body.orientation().qx;
    pose.orientation.y = body.orientation().qy;
    pose.orientation.z = body.orientation().qz;
    pose.orientation.w = body.orientation().qw;
    // Transform the pose.
    transformPose(&pose);
    msg->body_poses.poses.push_back(pose);    
    msg->header.stamp = ros::Time::now();

  }
}

void MocapPublisher::PublishToTopic() {
  // Set the publiser queue size.
  const int kQueueSize = 100;
  // Set the publishing frequency.
  ros::Rate loop_rate(pub_freq_hz);
  ros::Publisher mocapPub = nodeHandle->advertise<Mocap::mocap_frame>("Mocap", kQueueSize);
  unsigned int rosMsgCount = 0;

  Globals::run = true;    
  while( ros::ok() && Globals::run) {
    // Try to get a new frame from the listener.
    bool valid;
    MocapFrame frame(frameListener->pop(&valid).first);
    // Quit if the listener has no more frames.
    if(!valid) {
      fprintf(stderr,"No valid frame available \n");       
    }
    else {
      std::cout << frame << std::endl;
      // Extract rigid bodies and publish to ROS.
      Mocap::mocap_frame msg;
      ExtractPoseMsg(frame, &msg);
      //std::cout << ros::Time::now() << std::endl;
      mocapPub.publish(msg);
    }
    // All callbacks/services will be processed here.
    ros::spinOnce();
    loop_rate.sleep();
    rosMsgCount++;
  }

  printf("after ROS loop: ros::ok =  %i, Globals::run = %i \n",ros::ok(),Globals::run);

  // Wait for threads to finish.
  frameListener->stop();
  commandListener->stop();
  frameListener->join();
  commandListener->join();
   
  // Epilogue (Close sockets)
  close(sdData);
  close(sdCommand);
  
  // Close topic.
  mocapPub.shutdown();
}

void MocapPublisher::AdvertiseServices() {
  
  handle_mocap_GetMocapFrame = 
    nodeHandle->advertiseService("mocap_GetMocapFrame", 
				 &MocapPublisher::GetMocapFrame, this);
  handle_mocap_GetMocapTransformation = 
    nodeHandle->advertiseService("mocap_GetMocapTransformation",
				 &MocapPublisher::GetMocapTransformation, this);

  handle_mocap_SetMocapTransformation = 
    nodeHandle->advertiseService("mocap_SetMocapTransformation",
				 &MocapPublisher::SetMocapTransformation, this);
}

bool MocapPublisher::GetMocapFrame(Mocap::mocap_GetMocapFrame::Request& req, 
				   Mocap::mocap_GetMocapFrame::Response& res) {
  double start_secs = ros::Time::now().toSec();
  bool flag_tle = false;
  bool flag_captured_frame = false;
  while (!flag_tle && !flag_captured_frame) {
    double cur_secs = ros::Time::now().toSec();
    if (cur_secs - start_secs > k_max_wait) {
      flag_tle = true;
    }
    // Try to get a new frame from the listener.
    bool valid;
    MocapFrame frame(frameListener->pop(&valid).first);
    // Quit if the listener has no more frames.
    if(!valid) {
      fprintf(stderr,"No valid frame available \n");       
    }
    else {
      // Extract rigid bodies and publish to ROS.
      Mocap::mocap_frame msg;
      ExtractPoseMsg(frame, &msg);
      res.mf = msg;
      flag_captured_frame = true;
    }
  }
  if (flag_captured_frame) {
    res.msg = "OK.";
    res.ret = 1;
    return true;
  } else {
    res.msg = "Time limit exceeded.";
    res.ret = 1;
    return false;
  }
}

bool MocapPublisher::GetMocapTransformation(Mocap::mocap_GetMocapTransformation::Request& req, 
					    Mocap::mocap_GetMocapTransformation::Response& res) {
  Vec tf_trans = tf_mocap.getTranslation();
  res.x = tf_trans[0];
  res.y = tf_trans[1];
  res.z = tf_trans[2];

  Quaternion tf_quat = tf_mocap.getQuaternion();
  res.q0 = tf_quat[0];
  res.qx = tf_quat[1];
  res.qy = tf_quat[2];
  res.qz = tf_quat[3];

  res.msg = "Ok.";
  res.ret = 1;
  return true;
}

// TODO: the return false logic.
bool MocapPublisher::SetMocapTransformation(Mocap::mocap_SetMocapTransformation::Request& req, 
					    Mocap::mocap_SetMocapTransformation::Response& res) {
  double pose[7];
  pose[0] = req.x;
  pose[1] = req.y;
  pose[2] = req.z;
  pose[3] = req.q0;
  pose[4] = req.qx;
  pose[5] = req.qy;
  pose[6] = req.qz;
  setMocapTransformation(pose);
  res.msg = "OK.";
  res.ret = 1;
  return true;
}

void MocapPublisher::setMocapTransformation(double pose[7]) {
  tf_mocap = HomogTransf(pose);
}

void MocapPublisher::transformPoint(geometry_msgs::Point* pt) {
  double p[3];
  p[0] = pt->x;
  p[1] = pt->y;
  p[2] = pt->z;
  Vec vec_p(p, 3);
  vec_p = tf_mocap * vec_p;
  pt->x = vec_p[0];
  pt->y = vec_p[1];
  pt->z = vec_p[2];
}

void MocapPublisher::transformPose(geometry_msgs::Pose* pose) {
  geometry_msgs::Point& trans = pose->position;
  geometry_msgs::Quaternion& quat = pose->orientation;
  double tmp_pose[7] = {trans.x, trans.y, trans.z,
			quat.x, quat.y, quat.z, quat.w};
  HomogTransf tf = tf_mocap * HomogTransf(tmp_pose);

  Vec tf_trans = tf.getTranslation();
  trans.x = tf_trans[0];
  trans.y = tf_trans[1];
  trans.z = tf_trans[2];

  Quaternion tf_quat = tf.getQuaternion();
  quat.x = tf_quat[0];
  quat.y = tf_quat[1];
  quat.z = tf_quat[2];
  quat.w = tf_quat[3];
}

// Main function for the Mocap Node. 
// The Mocap node is publishing Mocap frames to the Mocap Topic.


int main(int argc, char* argv[]) {
  // Initialize the ROS node.
  ros::init(argc, argv, "Mocap");
  ros::NodeHandle node;
  MocapPublisher mocap_pub(&node);
  mocap_pub.EstablishCommunications(argc, argv);
  mocap_pub.AdvertiseServices();
  mocap_pub.PublishToTopic();
}

