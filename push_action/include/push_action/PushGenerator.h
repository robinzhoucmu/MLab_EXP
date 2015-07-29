#ifndef PUSH_GENERATOR_H
#define PUSH_GENERATOR_H

#include <matVec/matVec.h>
#include <TrackingObj/push_obj.h>
#include <PushExp/gl_parameters.h>

#include <vector>
#include <assert.h>
#include <cstdlib>
#include <ctime>

struct PushAction{
  Vec pushPoint;
  Vec pushVector;
  double initialDist;
  double penetrationDist;
  double retractionDist;
};

class PushGenerator
{
  public:
    // Minimum distance from the corner of a block that we will push (mm)
    const static double MIN_EDGE_DISTANCE = GLParameters::min_edge_dist;
    // Minimum angle the push direction makes with an edge (degrees)
    const static double MIN_PUSH_ANGLE = 20.0;

    // The distance away from the push point to start the object (mm)
    const static double DEFAULT_INITIAL_DISTANCE = 50.0;
    // The distance past the push point to push the object (mm)
    const static double DEFAULT_PENETRATION_DISTANCE = 20.0;
    // The distance to retract after pushing (mm)
    const static double DEFAULT_RETRACTION_DISTANCE = 5.0;

    PushGenerator(){ srand(time(NULL)); }
    ~PushGenerator(){}


    /** Determines whether a specified push of an object is valid
     * 
     * \param[in] obj the object we will be pushing
     * \param[in] pushPoint a 2d vector containing the x-y point in mm on 
     *             the edge of the object where we will be making a push
     * \param[in] pushVector a 2d vector containing the direction of the 
     *             push in the frame of the PushObject
     * \return boolean for whether or not this is a valid push
     */
    bool checkPush(const PushObject obj, const PushAction push);

    /** Generates a push action in the frame of the object
     * 
     * \param[in] obj The object we will be pushing
     * \param[out] push A push action. This contains a vector containing 
     *            the point in mm on the edge of the object where we will 
     *            be making a push, a vector containing the direction of
     *            the push in the frame of the object, and then the default 
     *            push action distances described in 'generateRandomPush'  
     * \return boolean for whether or not we successfully generated a push
     */
    bool generateRandomPush(const PushObject obj, PushAction *push);

    /** Generates a series of Homogeneous Transforms for how the robot 
     * should move to push an object.
     * 
     * \param[in] push The push action we want the robot to execute
     * \param[in] objectPose A homogeneous transform of the object pose in 
     *            the frame used by the robot. Note that the local object 
     *            frame used here is also used by the push action.
     * \param[in] tableNormal A 3d vector representing the normal vector the 
     *            table in the frame used by the robot, pointing upwards 
     * \param[out] robotPoses A vector of poses to move the robot through to 
     *              execute the desired push. The robot will start an 
     *              'initialDist' away from the object, then move to contact
     *              the object and push it a 'penetrationDist' amount, and 
     *              then retract a 'retractionDist' away from the object to 
     *              complete the push. Note that we assume the tool frame of
     *              the robot is the following: origin is exactly at the tip
     *              of the pusher, x-axis is the direction of the push, 
     *              z-axis is normal to the ground the object is resting on, 
     *              pointing upwards, and y-axis satisfies right hand rule. 
     */
    bool generateTrajectory(const PushAction push, const HomogTransf objectPose, const Vec tableNormal, 
        std::vector<HomogTransf> *robotPoses);

  private:


};

#endif /* PUSH_GENERATOR_H */
