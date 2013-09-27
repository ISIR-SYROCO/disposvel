/*!
 * \file segment.cpp
 * \brief segment constrution from tf datas
 * \author Karama Sriti
 * \version 0.1
 * \date June 14th 2013
 */
#include "segment.h"
//-----------------------------------------------------------------------
Segment::Segment(){}
//constructor with transforms
Segment::Segment(tf::StampedTransform proximal_transform_,tf::StampedTransform distal_transform_,string segment_id_)
{
	this->proximal_joint.setValue(proximal_transform_.getOrigin().x(),proximal_transform_.getOrigin().y(),proximal_transform_.getOrigin().z());
	this->distal_joint.setValue(distal_transform_.getOrigin().x(),distal_transform_.getOrigin().y(),distal_transform_.getOrigin().z()) ;
	this->vec_seg 	   = -(this->proximal_joint-this->distal_joint);
	this->vec_uni 	   = this->vec_seg.normalized() ;
	this->p_limite 	   = this->vec_seg.getX()/this->vec_uni.getX();
	this->segment_id = segment_id_;

	//-------------------------> affichage des variables du segment
/*
	std::cout <<"prox_x =" << proximal_transform_.getOrigin().x() << std::endl;	
	std::cout <<"prox_y =" << proximal_transform_.getOrigin().y() << std::endl;	
	std::cout <<"prox_z =" << proximal_transform_.getOrigin().z() << std::endl;	

	std::cout <<"dist_x =" << distal_transform_.getOrigin().x() << std::endl;	
	std::cout <<"dist_y =" << distal_transform_.getOrigin().y() << std::endl;	
	std::cout <<"dist_z =" << distal_transform_.getOrigin().z() << std::endl;

	std::cout <<"vec_seg = (" << vec_seg.x() << ";" << vec_seg.y() << ";" << vec_seg.z() << ")" << std::endl;	
	std::cout <<"vec_uni = (" << vec_uni.x() << ";" << vec_uni.y() << ";" << vec_uni.z() << ")" << std::endl;	
	std::cout <<"p_limite =" << p_limite << std::endl;	
	std::cout <<"segment_id =" << segment_id << std::endl;
*/
	//------------------------------------------------------------------------------------------------------------
}

/* ! \brief with transforms without segment_id */
Segment::Segment(tf::StampedTransform proximal_transform_,tf::StampedTransform distal_transform_)
{
	this->proximal_joint.setValue(proximal_transform_.getOrigin().x(),proximal_transform_.getOrigin().y(),proximal_transform_.getOrigin().z());
	this->distal_joint.setValue(distal_transform_.getOrigin().x(),distal_transform_.getOrigin().y(),distal_transform_.getOrigin().z()) ;
	this->vec_seg 	   = -(proximal_joint-distal_joint);
	this->vec_uni 	   = this->vec_seg.normalized() ;
	if(this->vec_uni.getX() != 0)   this->p_limite = this->vec_seg.getX()/this->vec_uni.getX();


}
/* ! \brief constructor with vectors and the segment id */
Segment::Segment(tf::Vector3 proximal_joint_,tf::Vector3 distal_joint_,string segments_id_) 
{
		this->proximal_joint     = proximal_joint_;
		this->distal_joint       = distal_joint_;
		this->vec_seg            = -(this->proximal_joint-this->distal_joint);
		this->vec_uni            = this->vec_seg.normalized() ;
		if(this->vec_uni.getX() != 0)   this->p_limite = this->vec_seg.getX()/this->vec_uni.getX() ;
		//else p_limite = 0;
		this->segment_id =  segments_id_;
}

/* ! \brief constructor with vectors without segment_id */
Segment::Segment(tf::Vector3 proximal_joint_,tf::Vector3 distal_joint_) 
{
		this->proximal_joint = proximal_joint_;
		this->distal_joint   = distal_joint_;
		this->vec_seg        = -(this->proximal_joint-this->distal_joint);
		this->vec_uni        = this->vec_seg.normalized() ;
		if(this->vec_uni.getX() != 0)   this->p_limite = this->vec_seg.getX()/this->vec_uni.getX();
}
/*! \brief returns the length of the segment  */
double Segment::getLength() const
{	
	return this->vec_seg.length();
}
//----------------------
/*! \brief returns the id of the segment */
string Segment::getSegmentId() const
{
	return this->segment_id;
}
/*! \brief returns the normalized vector of the segment */
tf::Vector3 Segment::getDirectionVector() const
{
	return this->vec_uni;
}
//----------------------
/*! \brief returns the limite of the parameter p */
double Segment::getPrameterLimit() const
{
	return this->p_limite; 
}
//-----------------------------------------------------------------------
/*! \brief returns x coordinate of the vector */
double Segment::getVx() const
{
	return this->vec_seg.getX();
}

/*! \brief returns y coordinate of the  vector */
double Segment::getVy() const
{
	return this->vec_seg.getY();
}

/*! \brief returns z coordinate of the  vector */
double Segment::getVz() const
{
	return this->vec_seg.getZ();
}
//----------------------
/*! \brief returns x coordinate of the normalized vector */
double Segment::getUx() const
{
	return this->vec_uni.getX();
}
/*! \brief returns y coordinate of the normalized vector */
double Segment::getUy() const
{
	return this->vec_uni.getY();
}
/*! \brief returns z coordinate of the normalized vector */
double Segment::getUz() const
{
	return this->vec_uni.getZ();
}
//----------------------
/*! \brief returns x coordinate of the points belong to this segments */
double Segment::getPx() const
{
	return this->proximal_joint.getX();
}

/*! \brief returns y coordinate of the points belong to this segments */
double Segment::getPy() const
{
	return this->proximal_joint.getY();
}

/*! \brief returns z coordinate of the points belong to this segments */
double Segment::getPz() const
{
	return this->proximal_joint.getZ();
}
//-----------------------------------------------------------------------
/*! \brief set the transforms value */
void Segment::setTransforms(tf::StampedTransform distal_transform_,tf::StampedTransform proximal_transform_)
{
	this->proximal_joint.setValue(proximal_transform_.getOrigin().x(),proximal_transform_.getOrigin().y(),proximal_transform_.getOrigin().z());
	this->distal_joint.setValue(distal_transform_.getOrigin().x(),distal_transform_.getOrigin().y(),distal_transform_.getOrigin().z()) ;
}
//----------------------
/*! \brief set the vector value */
void Segment::setVector(tf::Vector3 proximal_joint_,tf::Vector3 distal_joint_)
{
	this->proximal_joint = proximal_joint_; //
	this->distal_joint = distal_joint_;	//
	this->vec_seg = proximal_joint_- distal_joint_;  
}
/*! \brief set the class attribut */
void Segment::setSegmentAttribut(tf::StampedTransform distal_transform_,tf::StampedTransform proximal_transform_,string segment_id_)
{
	this->proximal_joint.setValue(proximal_transform_.getOrigin().x(),proximal_transform_.getOrigin().y(),proximal_transform_.getOrigin().z());
	this->distal_joint.setValue(distal_transform_.getOrigin().x(),distal_transform_.getOrigin().y(),distal_transform_.getOrigin().z()) ;
	this->vec_seg 	   = -(this->proximal_joint - this->distal_joint);
	this->vec_uni 	   = this->vec_seg.normalized() ;
	if(this->vec_uni.getX() != 0)   this->p_limite = this->vec_seg.getX()/this->vec_uni.getX();
	this->segment_id = segment_id_;	
	//-------------------------> affichage des variables du segment
/*
	std::cout <<"prox_x =" << proximal_transform_.getOrigin().x() << std::endl;	
	std::cout <<"prox_y =" << proximal_transform_.getOrigin().y() << std::endl;	
	std::cout <<"prox_z =" << proximal_transform_.getOrigin().z() << std::endl;	

	std::cout <<"dist_x =" << distal_transform_.getOrigin().x() << std::endl;	
	std::cout <<"dist_y =" << distal_transform_.getOrigin().y() << std::endl;	
	std::cout <<"dist_z =" << distal_transform_.getOrigin().z() << std::endl;

	std::cout <<"vec_seg = (" << vec_seg.x() << ";" << vec_seg.y() << ";" << vec_seg.z() << ")" << std::endl;	
	std::cout <<"vec_uni = (" << vec_uni.x() << ";" << vec_uni.y() << ";" << vec_uni.z() << ")" << std::endl;	
	std::cout <<"p_limite =" << this->vec_seg.getX()/this->vec_uni.getX() << std::endl;	
	std::cout <<"segment_id =" << segment_id << std::endl;
*/
	//------------------------------------------------------------------------------------------------------------
}
