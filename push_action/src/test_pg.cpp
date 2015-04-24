#include <iostream>
#include <TrackingObj/push_obj.h>
#include "PushGenerator.h"
#include <ros/package.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_pg");

  // Load our push object
  std::string to_path = ros::package::getPath("TrackingObj");
  PushObject triangle(to_path + "/object_db/triangle.txt");

  // Our push generator class
  PushGenerator pg;

  // A push action that will be set
  PushAction push;

  bool ret;
  
  const std::vector<Vec> & vertices = triangle.GetVertices();
  std::cout << "Vertices: " << std::endl;
  for (size_t i = 0; i < vertices.size(); ++i) {
    std::cout << vertices[i] << std::endl;
  }
  
  std::cout << "*******************************" << std::endl;

  for (int i=0; i < 10; ++i)
  {
    ret = pg.generateRandomPush(triangle, &push);
    std::cout << "Push Point: " << push.pushPoint << " Push Dir: " << push.pushVector << std::endl;
  }

  return 0;
}
