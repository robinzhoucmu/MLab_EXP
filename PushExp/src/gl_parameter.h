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
  static double mocap_cali_tf[7];

  // Pushing parameters.
  static bool execution_flag;
  static int num_pushes;
  static double min_edge_dist;  // in mm.
  static double min_push_angle;  // in degree.
  static double default_init_dist; // in mm.
  static double default_penetration_dist; // in mm.
  static double default_retraction_dist; // in mm.
  static double default_move_close_dist; // in mm.

  // Default robot resting position.
  static double robot_rest_cart[7]; // in mm.
  
  // Initial robot settings.
  static double robot_tcp_speed; // in mm.
  static double robot_ori_speed; // in degree.
  static double robot_set_workobj[7];  // in mm.
  static double robot_set_tool[7]; //in mm.

  // Files that store work object geometry and calibration information.
  static std::string workobj_file_cali;
  static std::string workobj_file_geometry;

  static void ReadParameters() {
    ros::param::get("/PushExp/execution_mode", GLParameters::execution_flag);
    ros::param::get("/PushExp/safe_height", GLParameters::safe_height);
    ros::param::get("/PushExp/mocap_num_readings", GLParameters::mocap_num_readings);
    ros::param::get("/PushExp/mocap_read_duration", GLParameters::mocap_read_duration);

    ros::param::get("/PushExp/num_pushes", GLParameters::num_pushes);
    ros::param::get("/PushExp/min_edge_dist", GLParameters::min_edge_dist);
    ros::param::get("/PushExp/min_push_angle", GLParameters::min_push_angle);
    ros::param::get("/PushExp/default_init_dist", GLParameters::default_init_dist);
    ros::param::get("/PushExp/default_penetration_dist", GLParameters::default_penetration_dist);
    ros::param::get("/PushExp/default_retraction_dist", GLParameters::default_retraction_dist);
    ros::param::get("/PushExp/default_move_close_dist", GLParameters::default_move_close_dist);
    ros::param::get("/PushExp/workobj_file_cali", GLParameters::workobj_file_cali);
    ros::param::get("/PushExp/workobj_file_geometry", GLParameters::workobj_file_geometry);
    ros::param::get("/PushExp/robot_tcp_speed", GLParameters::robot_tcp_speed);
    ros::param::get("/PushExp/robot_ori_speed", GLParameters::robot_ori_speed);

    // Read the robot resting position. 
    GLParameters::ReadList("/PushExp/robot_rest_cart", GLParameters::robot_rest_cart);
    // Read robot set tool.
    GLParameters::ReadList("/PushExp/robot_set_workobj", GLParameters::robot_set_workobj);
    // Read robot set workobj.
    GLParameters::ReadList("/PushExp/robot_set_tool", GLParameters::robot_set_tool);

    // Read mocap calibration transform.
    GLParameters::ReadList("/PushExp/mocap_cali_tf", GLParameters::mocap_cali_tf);
    
  }

  static void ReadList(std::string name, double data[]) {
    // Caveat:: ros::init() must be called before to enable correct XmlRpc list reading.
    XmlRpc::XmlRpcValue list;
    ros::param::get(name, list);
    for (int i = 0; i < list.size(); i++) {
      ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      data[i] = static_cast<double>(list[i]);
      //std::cout << data[i] << std::endl;
    }
  }
};

// Need to be "initialized" outside of class. Otherwise won't compile.
bool GLParameters::execution_flag = false;
double GLParameters::safe_height = 300.0;
int GLParameters::mocap_num_readings = 5;
double GLParameters::mocap_read_duration = 1.0;

int GLParameters::num_pushes = 1;
double GLParameters::min_edge_dist = 10.0;
double GLParameters::min_push_angle = 20.0;
double GLParameters::default_init_dist = 50.0;
double GLParameters::default_penetration_dist = 25.0;
double GLParameters::default_retraction_dist = 5.0;
double GLParameters::default_move_close_dist = 15.0;
double GLParameters::robot_tcp_speed = 10.0;
double GLParameters::robot_ori_speed = 5.0;
double GLParameters::robot_rest_cart[7] = {300, 0, 400.0, 1, 0, 0, 0};
double GLParameters::robot_set_workobj[7] = {0, 0, 0, 1, 0, 0, 0};
double GLParameters::robot_set_tool[7] = {-125, 0, 115, 0, 0, 1, 0};
double GLParameters::mocap_cali_tf[7] = {668.54, -260.52, 56.021, 0.49366, 0.49611, 0.50222, 0.50788};
std::string GLParameters::workobj_file_cali = "";
std::string GLParameters::workobj_file_geometry = "";

#endif
