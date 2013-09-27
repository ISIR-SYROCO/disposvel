/* Distance : calculate the distance between two segments */

#ifndef DISTANCE2_H
#define DISTANCE2_H

#include "cylinder.h"
class Distance2
{
	public :
		Cylinder human_limb;	
		Cylinder robot_link;
		double distance;				/**< the distance between the two Cylinders */
		tf::Vector3 point_belong_to_human;		/**< the coordiantes of the point that belong to the human for this distance */		
		tf::Vector3 point_belong_to_robot;		/**< the coordiantes of the point that belong to the robot for this distance */

		/** @todo calculate the distance and so will fill the corresponding attribut */
		void CalculateDistance();
	
	/*!
   	* \brief constructor and methods
	*/
		Distance2();
		Distance2(Cylinder human_limb_,Cylinder robot_link_);

		/** @todo returns the distance */
		double getDistance() const;

		/** @todo returns coordinates of the point that belong to the humain  */
		tf::Vector3 getPointBelongToHuman() const;

		/** @todo returns coordinates of the point that belong to the robot */
		tf::Vector3 getPointBelongToRobot() const;

		/** @todo set the elements of this class */
		void SetCylinders(Cylinder human_limb_,Cylinder robot_link_);
};

#endif 
