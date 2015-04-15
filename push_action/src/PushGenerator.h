#ifndef PUSH_GENERATOR_H
#define PUSH_GENERATOR_H

public class PushGenerator
{
  public:
    /** Determines whether a specified push of an object is valid
     * 
     * \param[in] obj the object we will be pushing
     * \param[in] pushPoint a 2d vector containing the x-y point in mm on 
     *             the edge of the object where we will be making a push
     * \param[in] pushVector a 2d vector containing the direction of the 
     *             push in the frame of the PushObject
     * \return boolean for whether or not this is a valid push
     */
    bool checkPush(const PushObject obj, 
        const std::vector<double> pushPoint, 
        const std::vector<double> pushVector);

    /** Generates a push point and direction in the frame of the object
      * 
      * \param[in] obj the object we will be pushing
      * \param[out] pushPoint a 2d vector containing the x-y point in mm on 
      *             the edge of the object where we will be making a push
      * \param[out] pushVector a 2d vector containing the direction of the 
      *             push in the frame of the PushObject
      * \return boolean for whether or not we successfully generated a push
      */
    bool generateRandomPush(const PushObject obj, 
        std::vector<double> *pushPoint, 
        std::vector<double> *pushVector);

    /** Generates a series of Homogeneous Transforms for how the robot 
      * should move to push an object
      * 
      * \param objectPose A homogeneous transform of the object pose in 
      *                   the frame of the robot. Note that the reference point 
     */
    bool generateTrajectory(const std::vector<double> pushPoint, 
        const std::vector<double> pushVector, 
        const HomogTransf objectPose, 
        std::vector<HomogTransf> *robotPoses);

  private:

    // The distance away from the push point to start the object (mm)
    const double INITIAL_DISTANCE = 100.0;
    // The distance past the push point to push the object (mm)
    const double PENETRATION_DISTANCE = 20.0;
    // The distance to retract after pushing (mm)
    const double RETRACTION_DISTANCE = 5.0;
};

#endif /* PUSH_GENERATOR_H */
