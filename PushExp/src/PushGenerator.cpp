#include "PushGenerator.h"



bool PushGenerator::checkPush(const PushObject obj, const PushAction push)
{
  // First, let's confirm that the distances used by the push are
  // non-negative
  if (initialDist < 0 || penetrationDist < 0 || retractionDist < 0)
    return false;

  // Next, let's check that the push point is on the object
  const std::vector<Edge>& edges = obj.GetEdges();  
  size_t num_edges = edges.size();
  double perimeter = 0.0; 
  for (size_t i=0; i < num_edges; ++i)
  {
    // Compute where the push point is relative to this edge
    // First, compute a vector from the left end of this edge to the push
    // point, so we can decompose it into lengths along the edge and
    // perpendicular to the edge
    Vec rel_vec = pushPoint - edges[i].left_end;
    double edge_length = edges[i].edge_vec.norm();
    double along_edge_length = rel_vec * edges[i].edge_vec / edge_length; // project relative vector onto edge_vec
    Vec perp_vec = pushPoint - (left_end + along_edge_length * edges[i].edge_vec / edge_length);
    double perp_edge_length = 

    // Subtract out the border region on each edge and 
    //  use this as our useable length
    edge_lengths[i] = edges[i].edge_vec.norm() - 2 * PushGenerator::MIN_EDGE_DISTANCE;
    
    // If the border regions overlap, then set our useable length to 0
    if (edge_lengths[i] < 0)
      edge_lengths[i] = 0;

    // Compute the perimeter, and maintain a cumulative sum which will 
    //  be used for picking a random point
    perimeter += edge_lengths[i];
    cum_sum[i] = perimeter;
  }


  return true;
}

bool PushGenerator::generateRandomPush(const PushObject obj, PushAction *push)
{
  const std::vector<Edge>& edges = obj.GetEdges();

  // First, get the useable lengths of all of the edges
  size_t num_edges = edges.size();
  std::vector<double> edge_lengths(num_edges);
  std::vector<double> cum_sum(num_edges);
  double perimeter = 0.0; 
  for (size_t i=0; i < num_edges; ++i)
  {
    // Subtract out the border region on each edge and 
    //  use this as our useable length
    edge_lengths[i] = edges[i].edge_vec.norm() - 2 * PushGenerator::MIN_EDGE_DISTANCE;
    
    // If the border regions overlap, then set our useable length to 0
    if (edge_lengths[i] < 0)
      edge_lengths[i] = 0;

    // Compute the perimeter, and maintain a cumulative sum which will 
    //  be used for picking a random point
    perimeter += edge_lengths[i];
    cum_sum[i] = perimeter;
  }

  // If no useable location was found, we cannot generate a push. Return false
  if (perimeter == 0)
  {
    return false;
  }

  // Now, let's randomly pick a useable point by choosing a 
  // random distance within the useable perimeter
  double position = ((double)rand()) / ((double)RAND_MAX) * perimeter;

  // Determine which edge this distance falls in
  size_t edge_idx;
  for (edge_idx=0; edge_idx < num_edges; ++edge_idx)
  {
    if (position < cum_sum[edge_idx])
      break;
  }
  assert(edge_idx < num_edges);

  // Now, compute how far along this edge this position is, 
  //  adding back in the buffer distances
  double total_edge_length = (edge_lengths[edge_idx] + 2 * PushGenerator::MIN_EDGE_DISTANCE);
  double distance_along_edge = position + PushGenerator::MIN_EDGE_DISTANCE - (cum_sum[edge_idx] - edge_lengths[edge_idx]);
  double ratio_on_edge = distance_along_edge / total_edge_length;

  // Generate our push point by using this ratio
  push->pushPoint = edges[edge_idx].GetSample(ratio_on_edge);

  // Now let's randomly pick a direction to push
  assert(PushGenerator::MIN_PUSH_ANGLE >= 0.0 && PushGenerator::MIN_PUSH_ANGLE < 90.0);
  double angle = ((double)rand()) / ((double)RAND_MAX) * (180 - 2 * PushGenerator::MIN_PUSH_ANGLE) - 90.0 + PushGenerator::MIN_PUSH_ANGLE;

  // The edge normal is outward facing, so we'll make it inward facing, 
  //  and then rotate it by the randomly chosen amount
  RotMat R;
  R.rotZ(angle / 180.0 * PI); // Convert from degrees to radians
  push->pushVector = R * edges[edge_idx].normal_dir * -1;

  // Finally let's set default parameters for pushing distances
  push->initialDist = PushGenerator::DEFAULT_INITIAL_DISTANCE;
  push->penetrationDist = PushGenerator::DEFAULT_PENETRATION_DISTANCE;
  push->retractionDist = PushGenerator::DEFAULT_RETRACTION_DISTANCE;

  return true;
}

bool PushGenerator::generateTrajectory(const PushAction push, const HomogTransf objectPose, const Vec tableNormal, 
    std::vector<HomogTransf> *robotPoses)
{
  robotPoses->clear();
  // Compute the push point and push direction in the robot frame
  Vec robotPoint = objectPose * push.pushPoint;
  Vec robotXDir = objectPose.getRotation() * push.pushVector;

  // Compute the frame of the robot
  Vec robotZDir = tableNormal;
  Vec robotYDir = robotZDir ^ robotXDir;

  // Make sure the axes are normalized so we create our 
  //  rotation matrix correctly
  robotXDir.normalize();
  robotYDir.normalize();
  robotZDir.normalize();

  RotMat robot_orient(robotXDir, robotYDir, robotZDir);

  // Now find the 3 translations of the robot. 
  Vec initialPoint = robotPoint - robotXDir * push.initialDist;
  Vec penetrationPoint = robotPoint + robotXDir * push.penetrationDist;
  Vec retractionPoint = penetrationPoint - robotXDir * push.retractionDist;

  // Store the poses in our array, and we're done
  robotPoses->resize(3);
  (*robotPoses)[0] = HomogTransf(robot_orient, initialPoint);
  (*robotPoses)[1] = HomogTransf(robot_orient, penetrationPoint);
  (*robotPoses)[2] = HomogTransf(robot_orient, retractionPoint);

  return true;
}





