#ifndef PUSH_GENERATOR_H
#define PUSH_GENERATOR_H

#include <matVec/matVec.h>
#include <TrackingObj/push_obj.h>
#include "gl_parameter.h"

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
  double moveCloseDist;
  // Added for two points pushing.
  std::string pushType;
  Vec approachVector;
  // Added for stable pushing actions with rotation parts.
  Vec cor;
  double rotAngle;
};

class PushGenerator
{
  public:
    // Minimum distance from the corner of a block that we will push (mm)
    double MIN_EDGE_DISTANCE;
    // Minimum angle the push direction makes with an edge (degrees)
    double MIN_PUSH_ANGLE;

    // The distance away from the push point to start the object (mm)
    double DEFAULT_INITIAL_DISTANCE;
    // The distance past the push point to push the object (mm)
    double DEFAULT_PENETRATION_DISTANCE;
    // The distance to retract after pushing (mm)
    double DEFAULT_RETRACTION_DISTANCE;
    // The distance to move the robot close to the object (mm)
    double DEFAULT_MOVECLOSE_DISTANCE;

    // The angle to rotate about a cor for two points push. (degree)
    double DEFAULT_ROT_ANGLE;
    // The characteristic length (e.g. radius of gyration) for sampling CORs.
    double DEFAULT_PHO; 
    // Range of COR samplings (multiples of pho).
    double DEFAULT_COR_RANGE;

    PushGenerator() { 
      srand(time(NULL)); 
      InitParameters();
    }
    ~PushGenerator(){}
    void InitParameters();

    /** Determines whether a specified push of an object is valid
     * 
     * \param[in] obj the object we will be pushing
     * \param[in] pushPoint a 2d vector containing the x-y point in mm on 
     *             the edge of the object where we will be making a push
     * \param[in] pushVector a 2d vector containing the direction of the 
     *             push in the frame of the PushObject
     * \return boolean for whether or not this is a valid push. False will
     * be returned for any of the following reasons: (1) pushPoint is not on
     * the surface of the object, (2) pushVector is not within
     * MIN_PUSH_ANGLE, (3) any of the distances are negative
     */
    bool checkPush(const PushObject obj, const PushAction push);

    /** Generate a push action specified by type in the frame of object.
     */
    bool generateRandomPush(const PushObject obj, PushAction *push, 
			    std::string push_type = "Point");

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
    bool generateRandomPointPush(const PushObject obj, PushAction *push);
    
    bool generateRandomTwoPointsPushTrans(const PushObject obj, PushAction *push);

    bool generateRandomTwoPointsPushRot(const PushObject obj, PushAction *push);

    /** Generates a series of Homogeneous Transforms for how the robot 
     * should move to push an object.
     * 
     * \param[in] push The push action we want the robot to execute
     * \param[in] objectPose A homogeneous transform of the object pose in 
     *            the frame used by the robot. Note that the local object 
     *            frame used here is also used by the push action.
     * \param[in] tableNormal A 3d vector representing the normal vector the 
     *            table in the frame used by the robot, pointing upwards 
     * \param[out] robotPoses A length 4 vector of poses to move the robot through to 
     *              execute the desired push. The robot will start an 
     *              'initialDist' away from the object, then move close to be 'move_close_dist'
     *              away from the object before move to contact
     *              the object and push it a 'penetrationDist' amount, and 
     *              then retract a 'retractionDist' away from the object to 
     *              complete the push. Note that we assume the tool frame of
     *              the robot is the following: origin is exactly at the tip
     *              of the pusher, x-axis is the direction of the push, 
     *              z-axis is normal to the ground the object is resting on, 
     *              pointing upwards, and y-axis satisfies right hand rule. 
     */
    bool generateTrajectoryPointPush(const PushAction push, const HomogTransf objectPose, 
				     const Vec tableNormal, std::vector<HomogTransf> *robotPoses);
    bool generateTrajectoryTwoPointsPushTrans(const PushAction push, const HomogTransf objectPose, const Vec tableNormal, std::vector<HomogTransf> *robotPoses);
    
    bool generateTrajectoryTwoPointsPushRot(const PushAction push, const HomogTransf objectPose, const Vec tableNormal, std::vector<HomogTransf> *robotPoses);    

    bool generateTrajectory(const PushAction push, const HomogTransf objectPose, const Vec tableNormal, std::vector<HomogTransf> *robotPoses);
    
    // Given original tool transform, change to the tool transform to rotate about the COR
    // specified by push_action.
    bool getToolTransformTwoPointsPush(const Vec tool_tf, const PushAction push_action, Vec* tcp_new);
    
  
 private:
    // Randomly sample an edge and push point on the edge, return the push point 
    // and edge outward normal in the local object frame.
    bool sampleEdgeAndPushPoint(const PushObject obj, Vec* pushPoint, Vec* EdgeNormal);
    Vec sampleCORInPushFrame();
    // Compute the push pose in object local frame given approach vector.
    HomogTransf computePushPose(Vec pushPoint, Vec appoachVector);
    // The distance off of an edge a point can be where check_push will
    // still return true (mm)
    const static double MAX_DIST_OFF_EDGE = 0.1;
    // Minimum norm of push vector
    const static double MIN_VECTOR_NORM = 0.001;
};

#endif /* PUSH_GENERATOR_H */
