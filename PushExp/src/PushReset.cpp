#include "PushReset.h"

void PushReset::InitParameters()
{
}

bool PushReset::checkReset(const PushObject obj, const HomogTransf objectPose)
{
  return true;
}

bool PushReset::generateTrajectory(const PushObject obj, const HomogTransf objectPose, std::vector<HomogTransf> *robotPoses)
{
  return false;
}




