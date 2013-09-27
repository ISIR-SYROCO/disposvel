#ifndef CYLINDER_H
#define CYLINDER_H
/*!
 * \file cylinder.h
 * \brief each Cylindert will be defined by one Segment, a radius and an id 
 * \author Karama Sriti
 * \version 0.1
 * \date June 14th 2013
 */
#include <ros/ros.h>
#include <tf/transform_listener.h> 

#include <segment.h>
using namespace std;

/*! \class Cylinder
*   \brief Class that represents a 3D cylinder, from anthropometric  datas 
*/
class Cylinder
{
	public :
		Segment axe;		/*!< l'axe du cylindre porte par le segment */ 
		string cylinder_id;	/*!< l'identite du membre dont va dependre le rayon du cylindre  */
		double radius;		/*!< rayon du cylindre */

	public :
	/*! 	\brief Constructor from Segment 
	* 	\param Segment  : represents the axe of the cylinder 
	*/
	
		Cylinder(){}

		Cylinder(tf::StampedTransform distal_transform_,tf::StampedTransform proximal_transform_,string segment_id_)
		{ 
			axe.setSegmentAttribut(distal_transform_,proximal_transform_,segment_id_);				
			cylinder_id = segment_id_;
			std::map<string,int> map_id;

			if ((cylinder_id == "link1")||(cylinder_id == "link2")||(cylinder_id == "link3")||(cylinder_id == "link4")||(cylinder_id == "link5")||(cylinder_id == "link6")||(cylinder_id == "link7"))
			cylinder_id = "link";

			map_id["HN"]   =  0; //head_neck
			map_id["TO"]   =  1; //torso
			map_id["LA"]   =  2; //arm
			map_id["LH"]   =  3; //hand
			map_id["LF"]   =  4; //forearm
			map_id["RA"]   =  5; //arm
			map_id["RF"]   =  6; //forearm
			map_id["RH"]   =  7; //hand
			map_id["LT"]   =  8; //thigh
			map_id["LL"]   =  9; //leg
			map_id["LF"]   = 10; //foot
			map_id["RT"]   = 11; //thigh
			map_id["RL"]   = 12; //leg
			map_id["RF"]   = 13; //foot
			map_id["UB"]   = 14; //upper body
			map_id["link"] = 15; //foot



			
			// see http://msis.jsc.nasa.gov/sections/section03.htm
			
			switch (map_id[cylinder_id])
			{
				case 0 :		//HN
					radius = 0.075 ;
					break;
				case 1 :		//TO
					radius = 0.500 ;
					break;
				case 2 :		//LA
					radius = 0.117 ;
					break;
				case 3 :		//LF
					radius = 0.104 ;
					break;
				case 4 :		//LH
					radius = 0.074 ;
					break;
				case 5	:   		//RA
					radius = 0.117 ;
					break;
				case 6 :		//RF
					radius = 0.104 ;
					break;
				case 7 :		//RH
					radius = 0.074 ;
					break;
				case 8 :		//LT
					radius = 0.214 ;
					break;
				case 9 :		//LL
					radius = 0.131 ;
					break;
				case 10:		//LF
					radius = 0.086 ;
					break;
				case 11:		//RT
					radius = 0.214 ;
					break;
				case 12:		//RL
					radius = 0.131 ;
					break;
				case 13:		//RF
					radius = 0.086 ;
					break;
				case 14:		//upper-body
					radius = 0.500 ;
					break;
				case 15:		//robot link
					radius = 1 ;
					break;
				default:
					radius = 0;
					break;
			} // switch 
				
		}

		/*! \brief returns height */
		double getHeight() const;

		/*! \brief returns radius */
		double getRadius() const;

		/*! \brief returns height */
		string getCylinderId() const;

		/*! \brief returns Segment */
		Segment getSegment() const;

		/*! \brief set cylinder arguments */
		void setCylinderAttribut(tf::StampedTransform distal_transform_,tf::StampedTransform proximal_transform_,string segment_id_);


};


#endif 
