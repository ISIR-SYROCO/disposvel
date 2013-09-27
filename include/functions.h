#include <tf/tf.h>
#include <tf/tfMessage.h>
#include "segment.h"

#define NUMBER_OF_ROBOT_LINKS 7
#define NUMBER_OF_HUMAN_LINKS 7
/** @todo fill human_links[] */
void resetHumanBodySegments(const std::vector<tf::StampedTransform> & user_transform,Segment * user_limbes);

/** @todo fill robot_links[] */
//void resetRobotSegments(std::vector<tf::StampedTransform> & robot_transform_,Segment robot_links_[]);


