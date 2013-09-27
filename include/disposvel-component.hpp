#ifndef OROCOS_DISPOSVEL_COMPONENT_HPP
#define OROCOS_DISPOSVEL_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <disposvel/disposvel-types.hpp>
#include <ros/ros.h>
#include <rtt/Component.hpp>
#include "distance2.h"
#include "distance.h"
#include <iostream>

class Disposvel : public RTT::TaskContext
{
	public:
		RTT::OperationCaller <geometry_msgs::TransformStamped(const std::string &, const std::string &)> lookup_now; //service to lookup (listen to tf topic) transform at time = now

		RTT::OperationCaller <geometry_msgs::TransformStamped(const std::string &, const std::string &, const ros::Time &)> lookup;//service to lookup (listen to tf topic) transform at desired time
		
		geometry_msgs::TransformStamped tform_rob, tform_now_rob, tform_hum, tform_now_hum; //transforms for robot and human coordinates
	
		std::vector<tf::StampedTransform> robot_transform_now, human_transform_now, robot_transform, human_transform; //vector of transform in which transforms are stored

		unsigned int num_rob_links, num_hum_limb, num_rob_frame,num_hum_frame; 

		RTT::OutputPort< std::vector<ControlData> > sortie; //distance, position and velocity (of each pair of segment)
  
		Disposvel(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		void addPorts();
};
#endif
