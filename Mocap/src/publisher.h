/*
 * publisher extract mocap frame information from socket and 
 * publish mocap frame information.
 */

// System includes
//******************
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

// NatNet Linux includes
//*************************
#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>

// ROS Includes
//*************************
#include <ros/ros.h>
#include <Mocap/mocap_frame.h>
#include <Mocap/marker_set.h>
#include "mocap_comm.h"

// Misc.
//****************************
#include <boost/program_options.hpp>
#include <time.h>

//const int kPublishFreq = 60;

class Globals {
 public: 
  // Parameters read from the command line
  static uint32_t localAddress;
  static uint32_t serverAddress;
   
  // State of the main() thread.
  static bool run;
  
  // Publishing frequency
  static int kPublishFreq;
  
  // Initialize the sockets arguments.
  static void ReadOpts(int argc, char* argv[]) {
    namespace po = boost::program_options;
    po::options_description desc("mocap-publisheru sing NatNetLinux");
    desc.add_options()
      ("help", "Display help message")
      ("local-addr,l", po::value<std::string>(), "Local IPv4 address")
      ("server-addr,s", po::value<std::string>(), "Server IPv4 address")
      ("pub-freq,f", po::value<int>(), "Publishing Frequency(<=100Hz)");
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc,argv,desc), vm);
   
    if (argc < 5 || vm.count("help") || !vm.count("local-addr") ||
	!vm.count("server-addr")) {
      std::cout << desc << std::endl;
      exit(1);
    }
    
    Globals::localAddress = inet_addr( vm["local-addr"].as<std::string>().c_str() );
    Globals::serverAddress = inet_addr( vm["server-addr"].as<std::string>().c_str() );
    if (!vm["pub-freq"].empty()) {
      Globals::kPublishFreq = vm["pub-freq"].as<int>();
    } else {
      Globals::kPublishFreq = 60;
    }
  }

  // End the program gracefully.
  static void Terminate(int)
  {
    // Tell the publishing loop and corresponding sockets to close.
    Globals::run = false;
  }
  
};
// Static member variable initializations.
uint32_t Globals::localAddress = 0;
uint32_t Globals::serverAddress = 0;
bool Globals::run = false;
int Globals::kPublishFreq = 60;

// Mocap Publisher. 
class MocapPublisher {
 public:
  // Initialize ROS, socket stuffs and publish frequency(default).
  MocapPublisher(ros::NodeHandle *n);
  ~MocapPublisher();
  void SetPublishFrequency(int pub_freq_hz);
  // Establish socket communication to the NatNet Windows Server.
  
  void EstablishCommunications(int argc, char* argv[]);
  // Publish mocap frame in a loop.
  
  void PublishToTopic();
 
  void AdvertiseServices();

  /*
  bool SetObjTransformation(Mocap::mocap_SetObjTransformation::Request& req,
			    Mocap::mocap_SetObjTransformation::Response& res);
  */
 private:
  // Create frame packet listener to listen to the server.
  void CreateListener();

  // Parse the NatNet MocapFrame data structure and store in a ros message.
  void ExtractPoseMsg(const MocapFrame& mframe, Mocap::mocap_frame* msg);
  
  // Service Callbacks.
  // Extract the mocap information(markers, rigid bodies) at this time frame.
  bool GetMocapFrame(Mocap::mocap_GetMocapFrame::Request& req, 
		     Mocap::mocap_GetMocapFrame::Response& res);
  
  bool GetMocapTransformation(Mocap::mocap_GetMocapTransformation::Request& req,
			      Mocap::mocap_GetMocapTransformation::Response& res); 

  // Set the transformation from robot base to the mocap frame.
  bool SetMocapTransformation(Mocap::mocap_SetMocapTransformation::Request& req, 
			      Mocap::mocap_SetMocapTransformation::Response& res);

  void setMocapTransformation(double pose[7]);  

  // Helper function to transform points in the mocap frame to robot base frame
  // given tf_mocap.
  void transformPoint(geometry_msgs::Point* pt);
  void transformPose(geometry_msgs::Pose* pose);

  // Publishing frequency to the Mocap Topic.
  int pub_freq_hz;
  
  // Socket stuffs.
  int sdCommand;
  int sdData;
  CommandListener* commandListener;
  FrameListener* frameListener;

  // Ros node.
  ros::NodeHandle *nodeHandle;

  // Handles to ROS stuff.
  ros::ServiceServer handle_mocap_GetMocapFrame;
  ros::ServiceServer handle_mocap_GetMocapTransformation;
  
  ros::ServiceServer handle_mocap_SetMocapTransformation;
  
  HomogTransf tf_mocap; 
  
  // For GetMocapFrame service call: if we wait for more than k_max_wait time,
  // will return false.
  static const double k_max_wait = 2.0;
  // Conversion from raw mocap output(in meters) to robot frame unit(milimeters).
  static const double k_unit = 1000;

};
