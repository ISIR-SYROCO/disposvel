#ifndef DISTANCE_H
#define DISTANCE_H
#include "segment.h"
/*!
 * \file distance.h
 * \brief calculate the distance between two segments
 * \author Karama Sriti
 * \version 0.1
 * \date June 14th 2013
 */
class Distance
{
	public :
		Segment human_limb;	
		Segment robot_link;
		double distance;				/**< the distance between the two segments */
		tf::Vector3 point_belong_to_human;		/**< the coordiantes of the point that belong to the human for this distance */		
		tf::Vector3 point_belong_to_robot;		/**< the coordiantes of the point that belong to the robot for this distance */

		/** @todo calculate the distance and so will fill the corresponding attribut */
		void CalculateDistance();
	
	/*!
   	* \brief constructor and methods
	*/
		Distance();
		Distance(Segment human_limb_,Segment robot_link_);

		/** @todo returns the distance */
		double getDistance() const;

		/** @todo returns coordinates of the point that belong to the humain  */
		tf::Vector3 getPointBelongToHuman() const;

		/** @todo returns coordinates of the point that belong to the robot */
		tf::Vector3 getPointBelongToRobot() const;

		/** @todo set the elements of this class */
		void SetSegments(Segment human_limb_,Segment robot_link_);
};

#endif 
