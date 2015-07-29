#ifndef PUSH_RESET_H
#define PUSH_RESET_H

#include <matVec/matVec.h>
#include <TrackingObj/push_obj.h>
#include "gl_parameter.h"

#include <vector>
#include <assert.h>
#include <cstdlib>
#include <ctime>


class PushReset
{
  public:

    PushReset() { 
      srand(time(NULL)); 
      InitParameters();
    }
    ~PushReset(){}
    void InitParameters();

    // Return true if all-clear, return false if need to reset
    bool checkReset(const PushObject obj, const HomogTransf objectPose);

    /** Generate a series of Homogeneous Transforms for how the robot should
     * move to reset an object.
     *
     * \return returns false if it is not possible to reset the object,
     * because for example the object is too far away
     */
    bool generateTrajectory(const PushObject obj, const HomogTransf objectPose, std::vector<HomogTransf> *robotPoses);

  private:
};

#endif /* PUSH_GENERATOR_H */
