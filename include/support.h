#include <rtt/RTT.hpp>
#include <geometry_msgs/TransformStamped.h>
//#include <disposvel/disposvel-types.hpp>
#include <ros/ros.h>
#include <rtt/Component.hpp>
//#include "distance2.h"
#include "distance.h"
#include <iostream>

/** @todo fill human_links and robot_links*/
void initSegments(std::vector<tf::StampedTransform>& human_transform_now, 
		  std::vector<tf::StampedTransform>& human_transform, 
				std::vector<tf::StampedTransform>& robot_transform_now, 
				std::vector<tf::StampedTransform>& robot_transform, 
					unsigned int num_human_seg, 
					unsigned int rob_human_seg,
						std::vector<Segment>& human_limbes_now, 
						std::vector<Segment>& human_limbes_,
							std::vector<Segment>& robot_links_now, 
							std::vector<Segment>& robot_links_);



