#include <functions.h>
/** @todo fill user_limbes[] */

void resetHumanBodySegments(const std::vector<tf::StampedTransform> & user_transform,Segment * user_limbes)
{
	if(user_transform.empty())
	{
		std::cout << "le vector user_tranform est vide" << std::endl;
	}
	else
	{

	//	user_limbes[0].setSegmentAttribut(user_transform[2],user_transform[0],"head_neck_torso");
	//	user_limbes[1].setSegmentAttribut(user_transform[3],user_transform[4],"left_arm");
	//	user_limbes[2].setSegmentAttribut(user_transform[4],user_transform[5],"left_forearm");
	//	user_limbes[3].setSegmentAttribut(user_transform[6],user_transform[7],"right_arm");
	//	user_limbes[4].setSegmentAttribut(user_transform[7],user_transform[8],"right_forearm");
	}

}

/** @todo fill robot_links[] */
/*
void resetRobotSegments(std::vector<tf::StampedTransform> &  robot_transform_,Segment robot_links_[])
{
	for(unsigned int j = 0; j < NUMBER_OF_ROBOT_LINKS-1; j++)
	{
		robot_links_[j].setTransforms(robot_transform_[j],robot_transform_[j+1]);
	}
}
*/
//------------> calcul des distances
// void resetHumanBodySegments(std::vector<tf::StampedTransform> user_transform_,std::vector<Segment> user_limbes_);
//resetHumanBodySegments(human_transform, human_limbes);
//resetHumanBodySegments(robot_transform, robot_links);
//------------------------------------------------------
