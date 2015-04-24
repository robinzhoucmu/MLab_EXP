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
    std::cout << "Push Point: " << push.pushPoint << " Push Dir: " << push.pushVector << " ret = " << ret << std::endl;
  }


  std::cout << "*******************************" << std::endl;
  push.pushPoint = Vec("50 50 0",3);
  push.pushVector = Vec("-0.7071 -0.7071 0",3);
  HomogTransf objectPose(Quaternion("0.3827 0 0 0.9239"), Vec("100 200 300",3));
  std::cout << "Push Point: " << push.pushPoint << " Push Dir: " << push.pushVector << " objectPose: " << objectPose.getTranslation() << objectPose.getQuaternion() << std::endl;

  std::vector<HomogTransf> robotPoses;

  ret = pg.generateTrajectory(push, objectPose, Vec("0 0 1",3), &robotPoses);

  for (size_t i=0; i < robotPoses.size(); ++i)
  {
    std::cout << robotPoses[i].getTranslation() << robotPoses[i].getQuaternion() << std::endl;
  }

  std::cout << "*******************************" << std::endl;
  push.pushPoint = Vec("0 40 0",3);
  push.pushVector = Vec("1 0 0",3);
  objectPose = HomogTransf(Quaternion("0 0 0 1"), Vec("-200 100 300",3));
  std::cout << "Push Point: " << push.pushPoint << " Push Dir: " << push.pushVector << " objectPose: " << objectPose.getTranslation() << objectPose.getQuaternion() << std::endl;

  ret = pg.generateTrajectory(push, objectPose, Vec("0 0 1",3), &robotPoses);

  for (size_t i=0; i < robotPoses.size(); ++i)
  {
    std::cout << robotPoses[i].getTranslation() << robotPoses[i].getQuaternion() << std::endl;
  }


  return 0;
}
