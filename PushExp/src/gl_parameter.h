/*
 * Static global class to store experiments config. 
 * Read from ros parameters specified in a yaml file.
 */
#ifndef GL_PARAMETER_H
#define GL_PARAMETER_H
#include <ros/ros.h>
#include <string>
#include <iostream>

class GLParameters {
 public:

  static double safe_height; // in mm.
  
  static int mocap_num_readings;
  static double mocap_read_duration; // in seconds.

  // Pushing parameters.
  static double min_edge_dist;  // in mm.
  static double min_push_angle;  // in degree.
  static double default_init_dist; // in mm.
  
  // Default robot resting position.
  static double robot_rest_cart[7]; // in mm.
  
  // Initial robot settings.
  static double robot_set_workobj[7];  // in mm.
  static double robot_set_tool[7]; //in mm.

  // Files that store work object geometry and calibration information.
  static std::string workobj_file_cali;
  static std::string workobj_file_geometry;

  static void ReadParameters() {
    ros::param::get("/PushExp/safe_height", GLParameters::safe_height);
    ros::param::get("/PushExp/mocap_num_readings", GLParameters::mocap_num_readings);
    ros::param::get("/PushExp/mocap_read_duration", GLParameters::mocap_read_duration);
    ros::param::get("/PushExp/default_init_dist", GLParameters::default_init_dist);
    ros::param::get("/PushExp/workobj_file_cali", GLParameters::workobj_file_cali);
    ros::param::get("/PushExp/workobj_file_geometry", GLParameters::workobj_file_geometry);

    // Read the robot resting position. 
    //GLParameters::ReadList("/PushExp/robot_rest_cart", GLParameters::robot_rest_cart);
    XmlRpc::XmlRpcValue list;
    ros::param::get("/PushExp/robot_rest_cart", list);
    for (int i = 0; i < list.size(); i++) {
      ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      robot_rest_cart[i] = static_cast<double>(list[i]);
    }
  }

  static void ReadList(std::string name, double data[]) {
    XmlRpc::XmlRpcValue list;
    ros::param::get(name.c_str(), list);
    for (int i = 0; i < list.size(); i++) {
      ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      data[i] = static_cast<double>(list[i]);
      std::cout << data[i] << std::endl;
    }
  }
};

// Need to be "initialized" outside of class. Otherwise won't compile.
double GLParameters::safe_height = 300.0;
int GLParameters::mocap_num_readings = 5;
double GLParameters::mocap_read_duration = 1.0;
double GLParameters::min_edge_dist = 10.0;
double GLParameters::min_push_angle = 20.0;
double GLParameters::default_init_dist = 50.0;
double GLParameters::robot_rest_cart[7] = {300, 0, 400.0, 1, 0, 0, 0};
std::string GLParameters::workobj_file_cali = "";
std::string GLParameters::workobj_file_geometry = "";

#endif
