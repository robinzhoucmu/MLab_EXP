#include "PushGenerator.h"

void PushGenerator::InitParameters() {
  MIN_EDGE_DISTANCE = GLParameters::min_edge_dist;
  MIN_PUSH_ANGLE = GLParameters::min_push_angle;
  DEFAULT_INITIAL_DISTANCE = GLParameters::default_init_dist;
  DEFAULT_PENETRATION_DISTANCE = GLParameters::default_penetration_dist;
  DEFAULT_RETRACTION_DISTANCE = GLParameters::default_retraction_dist;
  DEFAULT_MOVECLOSE_DISTANCE = GLParameters::default_move_close_dist;
  DEFAULT_ROT_ANGLE = GLParameters::default_rot_angle;
  DEFAULT_PHO = GLParameters::default_pho;
  DEFAULT_COR_RANGE = GLParameters::default_cor_range;
}

bool PushGenerator::checkPush(const PushObject obj, const PushAction push)
{
  //***********************************
  // First, let's confirm that the distances used by the push are
  // non-negative
  if (push.initialDist < 0 || push.penetrationDist < 0 || push.retractionDist < 0)
    return false;

  //***********************************
  // Next, let's check that the push point is on the object
  int edge_idx = -1;
  double best_perp_edge_dist = -1;
  const std::vector<Edge>& edges = obj.GetEdges();  
  for (size_t i=0; i < edges.size(); ++i)
  {
    // Compute where the push point is relative to this edge
    // First, compute a vector from the left end of this edge to the push
    // point, so we can decompose it into lengths along the edge and
    // perpendicular to the edge
    Vec rel_vec = push.pushPoint - edges[i].left_end;
    double edge_length = edges[i].edge_vec.norm();
    double along_edge_length = rel_vec * edges[i].edge_vec / edge_length; // project relative vector onto edge_vec

    // Next compute the perpendicular vector and distance the point is away
    // from the edge
    Vec perp_vec = push.pushPoint - (edges[i].left_end + (edges[i].edge_vec * along_edge_length) / edge_length);
    double perp_edge_length = perp_vec.norm();

    // Now check if the point lies on this edge
    if (fabs(perp_edge_length) > MAX_DIST_OFF_EDGE)
    {
      // If the perpendicular distance is too large, the point is not on
      // this edge
      continue;
    }
    else if (along_edge_length < MIN_EDGE_DISTANCE || along_edge_length > edge_length - MIN_EDGE_DISTANCE)
    {
      // If the perpendicular distance is ok, but the point doesn't lie
      // within the pushable region of the edge, then this is not a valid
      // edge to push
      continue;
    }
    else
    {
      // Otherwise, this is a valid push. Do some additional logic to make
      // sure we pick the edge that the point is closest to.
      if (edge_idx == -1 || perp_edge_length < best_perp_edge_dist)
      {
        edge_idx = i;
        best_perp_edge_dist = perp_edge_length;
      }
    }
  }

  // If we couldn't find an edge that the push point was close to, then
  // this is not a valid push
  if (edge_idx == -1)
    return false;

  //**********************************
  // Finally, check that the direction of the push is not too far away from
  // the normal direction, which will result in a collision or a missed
  // push

  // Initially, make sure pushVector is not null
  if (push.pushVector.norm() < MIN_VECTOR_NORM)
    return false;

  // Compute the angle between the pushVector and the normal vector by
  // taking the dot product of the vectors (Note that normal_dir from the
  // object is an outward facing normal, so we'll multiply by -1 to make it
  // inward facing)
  double angle = 180/PI * acos(
      push.pushVector * (edges[edge_idx].normal_dir * -1) / 
      (push.pushVector.norm() * edges[edge_idx].normal_dir.norm())); 
  
  // If the angle between the push vector and the normal vector is too
  // large, this is not a valid push
  if (90 - angle < MIN_PUSH_ANGLE)
    return false;
  
  //********************************
  // If we have passed all of the above tests, then this is a valid push
  return true;
}

Vec PushGenerator::sampleCORInPushFrame() {
  // -0.5 to 0.5.
  double r_x =  ((double)rand()) / ((double)RAND_MAX) - 0.5;
  // -1.5 to 1.5
  double r_y = 3 * (((double)rand()) / ((double)RAND_MAX) - 0.5);
  Vec sampled_cor(3);
  sampled_cor[0] = DEFAULT_PHO * DEFAULT_COR_RANGE * r_x;
  sampled_cor[1] = DEFAULT_PHO * DEFAULT_COR_RANGE * r_y;
  sampled_cor[2] = 0;
  return sampled_cor;
}

HomogTransf PushGenerator::computePushPose(Vec pushPoint, Vec approachVector) {
  // Form push frame.
  Vec push_frame_x = approachVector;
  double z[3] = {0,0,1};
  Vec push_frame_z(z,3);
  Vec push_frame_y = push_frame_z ^ push_frame_x;
  RotMat R_push(push_frame_x, push_frame_y, push_frame_z);
  return HomogTransf(R_push, pushPoint);
}

bool PushGenerator::sampleEdgeAndPushPoint(const PushObject obj, Vec* pushPoint, Vec* edgeNormal) {
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
  *pushPoint =  edges[edge_idx].GetSample(ratio_on_edge);
  *edgeNormal = edges[edge_idx].normal_dir;
  return true;

}
bool PushGenerator::getToolTransformTwoPointsPush(const HomogTransf tool_tf_old, const PushAction push_action, HomogTransf* tool_tf_new) {
  if (push_action.pushType!= "TwoPointRotation") {
    std::cerr << "Wrong push type for resetting COR as tcp" << std::endl;
    return false;
  }
  // Get COR in local object frame from push action.
  Vec cor = push_action.cor;
  HomogTransf push_pose = computePushPose(push_action.pushPoint, push_action.approachVector);
  // Change COR to pusher frame. 
  cor = push_pose.inv() * cor;
  tool_tf_new->setRotation(tool_tf_old.getRotation());
  Vec tcp_new = tool_tf_old * cor;
  std::cout << tcp_new << std::endl;
  tool_tf_new->setTranslation(tcp_new);
  return true;
}

bool PushGenerator::generateRandomPush(const PushObject obj, PushAction *push, std::string push_type) {
  bool flag = false;
  if (push_type == "Point") {
    flag = generateRandomPointPush(obj, push);
  } else if (push_type == "TwoPointTranslation") {
    flag = generateRandomTwoPointsPushTrans(obj, push);
  } else if (push_type == "TwoPointRotation") {
    flag = generateRandomTwoPointsPushRot(obj, push);
  }
  if (flag) {
    push->pushType = push_type;
  }
  return flag;
}

bool PushGenerator::generateRandomTwoPointsPushTrans(const PushObject obj, PushAction *push) {
  // Same push action logic generation, differs in how to generate trajectory.
  return generateRandomPointPush(obj, push);
}

bool PushGenerator::generateRandomTwoPointsPushRot(const PushObject obj, PushAction *push) {
  // Sample push point and approach direction.
  Vec edge_normal;
  bool flag_sample = sampleEdgeAndPushPoint(obj, &(push->pushPoint), &edge_normal);
  if (!flag_sample) {
    return false;
  }
  push->approachVector = edge_normal * -1;
  // Parameters for approaching close.
  push->initialDist = PushGenerator::DEFAULT_INITIAL_DISTANCE;
  push->moveCloseDist = PushGenerator::DEFAULT_MOVECLOSE_DISTANCE;

  // Sample cor in push frame.
  Vec cor = sampleCORInPushFrame();
  std::cout << "sampled COR in push frame: " << cor << std::endl;
  // Check for y component to determine CCW or CW.
  if (cor[1] > 0) {
    // CCW rotation. 
    push->rotAngle = DEFAULT_ROT_ANGLE;
  } else {
    // CW rotation.
    push->rotAngle = -DEFAULT_ROT_ANGLE;
  }
  HomogTransf pushPose = computePushPose(push->pushPoint, push->approachVector);
  std::cout << "pushPose in local obj frame: " << pushPose << std::endl;
  push->cor = pushPose * cor;
  std::cout << "push COR in local obj frame: " << push->cor << std::endl;
  return true;
}

bool PushGenerator::generateRandomPointPush(const PushObject obj, PushAction *push)
{
  Vec edge_normal;
  bool flag_sample = sampleEdgeAndPushPoint(obj, &(push->pushPoint), &edge_normal);
  if (!flag_sample) {
    return false;
  }
  // Now let's randomly pick a direction to push
  assert(PushGenerator::MIN_PUSH_ANGLE >= 0.0 && PushGenerator::MIN_PUSH_ANGLE < 90.0);
  double angle = ((double)rand()) / ((double)RAND_MAX) * (180 - 2 * PushGenerator::MIN_PUSH_ANGLE) - 90.0 + PushGenerator::MIN_PUSH_ANGLE;

  // The edge normal is outward facing, so we'll make it inward facing, 
  // and then rotate it by the randomly chosen amount.
  RotMat R;
  R.rotZ(angle / 180.0 * PI); // Convert from degrees to radians
  push->pushVector = R * edge_normal * -1;

  push->approachVector = edge_normal * -1;

  // Finally let's set default parameters for pushing distances
  push->initialDist = PushGenerator::DEFAULT_INITIAL_DISTANCE;
  push->penetrationDist = PushGenerator::DEFAULT_PENETRATION_DISTANCE;
  push->retractionDist = PushGenerator::DEFAULT_RETRACTION_DISTANCE;
  push->moveCloseDist = PushGenerator::DEFAULT_MOVECLOSE_DISTANCE;
  return true;
}
bool PushGenerator::generateTrajectory(const PushAction push, const HomogTransf objectPose, const Vec tableNormal, std::vector<HomogTransf> *robotPoses) {
  bool flag = false;
  std::string push_type = push.pushType;
  if (push_type == "Point") {
    flag = generateTrajectoryPointPush(push, objectPose, tableNormal, robotPoses);
  } else if (push_type == "TwoPointTranslation") {
    flag = generateTrajectoryTwoPointsPushTrans(push, objectPose, tableNormal, robotPoses);
  } else if (push_type == "TwoPointRotation") {
    flag = generateTrajectoryTwoPointsPushRot(push, objectPose, tableNormal, robotPoses);
  }
  return flag;
}

bool PushGenerator::generateTrajectoryPointPush(const PushAction push, const HomogTransf objectPose, const Vec tableNormal, 
    std::vector<HomogTransf> *robotPoses)
{
  robotPoses->clear();
  // Compute the push point and push direction in the robot frame
  Vec robotPoint = objectPose * push.pushPoint;
  Vec robotXDir = objectPose.getRotation() * push.pushVector;
  // Eliminate the z component(exists due to perception error: XoY plane tilted a bit).
  const int ind_z = 2;
  robotXDir[ind_z] = 0;
  robotXDir.normalize();
  // Set z to be the same as object z. 
  const double obj_position_z = objectPose.getTranslation()[ind_z];
  robotPoint[ind_z] = obj_position_z; 
  std::cout << "push point: " << push.pushPoint << std::endl;
  std::cout << "robot Point: " << robotPoint << std::endl;
  std::cout << "robotXDir: " << robotXDir << std::endl;
  // Compute the frame of the robot
  Vec robotZDir = tableNormal;
  Vec robotYDir = robotZDir ^ robotXDir;

  // Make sure the axes are normalized so we create our 
  // rotation matrix correctly.
  robotXDir.normalize();
  robotYDir.normalize();
  robotZDir.normalize();

  RotMat robot_orient(robotXDir, robotYDir, robotZDir);

  // Now find the 3 translations of the robot. 
  Vec initialPoint = robotPoint - robotXDir * push.initialDist;
  Vec moveClosePoint = robotPoint - robotXDir * push.moveCloseDist;
  Vec penetrationPoint = robotPoint + robotXDir * push.penetrationDist;
  Vec retractionPoint = penetrationPoint - robotXDir * push.retractionDist;

  // Store the poses in our array, and we're done
  robotPoses->resize(4);
  (*robotPoses)[0] = HomogTransf(robot_orient, initialPoint);
  (*robotPoses)[1] = HomogTransf(robot_orient, moveClosePoint);
  (*robotPoses)[2] = HomogTransf(robot_orient, penetrationPoint);
  (*robotPoses)[3] = HomogTransf(robot_orient, retractionPoint);

  return true;
}

bool PushGenerator::generateTrajectoryTwoPointsPushTrans(const PushAction push, const HomogTransf objectPose, const Vec tableNormal, std::vector<HomogTransf> *robotPoses) {
  robotPoses->clear();
  // Compute the push point and push direction in the robot frame
  Vec robotPoint = objectPose * push.pushPoint;
  // Robot approaches the object in the direction of inward edge normal.
  Vec robotXDir = objectPose.getRotation() * push.approachVector;
  // Robot pushes the object (translational move) in the direction of pushvector 
  // without changing the orientation.
  Vec robotPushXDir = objectPose.getRotation() * push.pushVector;
  // Eliminate the z component(exists due to perception error: XoY plane tilted a bit).
  const int ind_z = 2;
  robotXDir[ind_z] = 0;
  robotXDir.normalize();
  // Set z to be the same as object z. 
  const double obj_position_z = objectPose.getTranslation()[ind_z];
  robotPoint[ind_z] = obj_position_z; 
  std::cout << "push point: " << push.pushPoint << std::endl;
  std::cout << "robot Point: " << robotPoint << std::endl;
  std::cout << "robotXDir: " << robotXDir << std::endl;
  std::cout << "robotPushXDir: " << robotPushXDir << std::endl;
  // Compute the frame of the robot pushing pose.
  Vec robotZDir = tableNormal;
  Vec robotYDir = robotZDir ^ robotXDir;

  // Make sure the axes are normalized so we create our 
  // rotation matrix correctly.
  robotXDir.normalize();
  robotYDir.normalize();
  robotZDir.normalize();
  robotPushXDir.normalize();
  
  RotMat robot_orient(robotXDir, robotYDir, robotZDir);

  // Now find the 3 translations of the robot. 
  Vec initialPoint = robotPoint - robotXDir * push.initialDist;
  Vec moveClosePoint = robotPoint - robotXDir * push.moveCloseDist;
  Vec penetrationPoint = robotPoint + robotPushXDir * push.penetrationDist;
  Vec retractionPoint = penetrationPoint - robotPushXDir * push.retractionDist;

  // Store the poses in our array, and we're done
  robotPoses->resize(4);
  (*robotPoses)[0] = HomogTransf(robot_orient, initialPoint);
  (*robotPoses)[1] = HomogTransf(robot_orient, moveClosePoint);
  (*robotPoses)[2] = HomogTransf(robot_orient, penetrationPoint);
  (*robotPoses)[3] = HomogTransf(robot_orient, retractionPoint);
  return true;
}

bool PushGenerator::generateTrajectoryTwoPointsPushRot(const PushAction push, const HomogTransf objectPose, const Vec tableNormal, std::vector<HomogTransf> *robotPoses) {
  robotPoses->clear();
  robotPoses->resize(4);
  HomogTransf push_pose = computePushPose(push.pushPoint, push.approachVector);
  Vec trans = push_pose.getTranslation();
  RotMat rot = push_pose.getRotation();
  // The very initial pose far from object.
  Vec init_trans = trans - push.approachVector * push.initialDist;
  HomogTransf push_init_pose(rot, init_trans);
  // The move close pose.
  Vec move_close_trans = trans - push.approachVector * push.moveCloseDist;
  HomogTransf move_close_pose(rot, move_close_trans);
  
  // Note that we now enter into a new tcp point (COR) mode. We will perform a pure 
  // rotation to command the robot to rotate about COR. 
  // Caveat:: check that tcp point has been reset.
  RotMat Rz;
  Rz.rotZ(push.rotAngle / 180.0 * PI); // Convert from degrees to radians
  RotMat cor_rot = rot * Rz; 
  HomogTransf cor_pose_pushin(cor_rot, push.cor);
  HomogTransf cor_pose_retract(rot, push.cor);

  // Map to global frame.
  // Project object pose onto the xoy plane.
  Quaternion q_obj = objectPose.getQuaternion();
  q_obj[1] = 0;
  q_obj[2] = 0;
  q_obj.normalize();
  HomogTransf projected_objectPose(q_obj, objectPose.getTranslation());

  (*robotPoses)[0] = projected_objectPose * push_init_pose;
  (*robotPoses)[1] = projected_objectPose * move_close_pose;
  (*robotPoses)[2] = projected_objectPose * cor_pose_pushin;
  (*robotPoses)[3] = projected_objectPose * cor_pose_retract;
  return true;
}




