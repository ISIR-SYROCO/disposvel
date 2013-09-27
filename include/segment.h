#ifndef SEGMENT_H
#define SEGMENT_H
/*!
 * \file segment.h
 * \brief segment constrution from tf datas
 * \author Karama Sriti
 * \version 0.1
 * \date June 14th 2013
 */
#include <ros/ros.h>
#include <tf/transform_listener.h> 


using namespace std;
	/*! \class Segment
	*   \brief Class that represents a segment 
	*
	*   This class allows to construct segments from tf datas 
	*/
	class Segment
	{
		public :
			tf::Vector3 vec_seg;		/*!< the "segment" */
			tf::Vector3 vec_uni;		/*!< the mormalized vector */ 
			double p_limite;		/*!< the condition limit of the p parameter */ 
			string segment_id;		/*!< the name id of this segments */ 

			tf::StampedTransform proximal_transform ;	/*!< the frame that is proximal respect to the human\robot body */
			tf::StampedTransform distal_transform ;		/*!< the frame that is distal respect to the human\robot body */

			tf::Vector3 proximal_joint;			/*!< the position of the proximal point of the segment that belong to the human\robot body */		
			tf::Vector3 distal_joint;			/*!< the position of the distal point of the segment that belong to the human\robot body */		

		
			
	
			Segment();

			/*!	\brief Constructor from transforms with a segment Id
			* 	\param proximal_transform  :	coordinates of the proximal point respect to the body
			* 	\param distal_transform    :	coordinates of the distal point respect to the body
			* 	\param segment_id	   : 	the id of the segment
			*/
			Segment(tf::StampedTransform proximal_transform_,tf::StampedTransform distal_transform_,string segments_id_);

			/*!	\brief Constructor from transforms without a segment Id
			* 	\param proximal_transform  :	coordinates of the proximal point respect to the body
			* 	\param distal_transform    :	coordinates of the distal point respect to the body
			*/
			Segment(tf::StampedTransform proximal_transform_,tf::StampedTransform distal_transform_);
			
			/*!	\brief Constructor vectors and with a segment Id
			* 	\param proximal_joint      : 	coordinates of the proximal point respect to the body
			* 	\param distal_joint        :	coordinates of the distal point respect to the body
			* 	\param segment_id	   : 	the id of the segment
			*/
			Segment(tf::Vector3 proximal_joint_,tf::Vector3 distal_joint_,string segments_id_) ;
		
			/*!	\brief Constructor vectors and without a segment Id
			* 	\param proximal_joint      : 	coordinates of the proximal point respect to the body
			* 	\param distal_joint        :	coordinates of the distal point respect to the body
			* 	\param segment_id	   : 	the id of the segment
			*/
			Segment(tf::Vector3 proximal_joint_,tf::Vector3 distal_joint_) ;

			//-----------------------------------------------------------------------
			/*! \brief returns the length of the segment */
			double getLength() const;

			/*! 	\brief returns the id of the segment */
			string getSegmentId() const;

			/*! 	\brief returns the normalized vector of the segment */
			tf::Vector3 getDirectionVector() const;

			/*! 	\brief returns the limite of the parameter p */
			double getPrameterLimit() const;
			//----------------------
			/*! 	\brief returns x coordinate of the vector */
			double getVx() const;

			/*! 	\brief returns y coordinate of the  vector */
			double getVy() const;

			/*! 	\brief returns z coordinate of the  vector */
			double getVz() const;
			//----------------------
			/*! 	\brief returns x coordinate of the normalized vector */
			double getUx() const;

			/*! 	\brief returns y coordinate of the normalized vector */
			double getUy() const;

			/*! 	\brief returns z coordinate of the normalized vector */
			double getUz() const;
			//----------------------
			/*! 	\brief returns x coordinate of the points belong to this segments */
			double getPx() const;

			/*! 	\brief returns y coordinate of the points belong to this segments */
			double getPy() const;

			/*! 	\brief returns z coordinate of the points belong to this segments */
			double getPz() const;
			//-----------------------------------------------------------------------
			/*! 	\brief set the transforms value 
			* 	\param proximal_transform  :	coordinates of the proximal point respect to the body
			* 	\param distal_transform    :	coordinates of the distal point respect to the body
			*/
			void setTransforms(tf::StampedTransform distal_transform_,tf::StampedTransform proximal_transform_);

			//----------------------
			/*! 	\brief set the vector value 
			* 	\param proximal_joint      : 	coordinates of the proximal point respect to the body
			* 	\param distal_joint        :	coordinates of the distal point respect to the body
			*/
			void setVector(tf::Vector3 proximal_joint_,tf::Vector3 distal_joint_);
			//----------------------
			/*! \brief set the class' attribut 
			* 	\param proximal_transform  :	coordinates of the proximal point respect to the body
			* 	\param distal_transform    :	coordinates of the distal point respect to the body
			* 	\param segment_id	   : 	the id of the segment
			*/
			void setSegmentAttribut(tf::StampedTransform distal_transform_,tf::StampedTransform proximal_transform_,string segment_id_);
	};

#endif 
