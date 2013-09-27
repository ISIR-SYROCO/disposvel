/*!
 * \file Distance.cpp
 * \brief calculate the distance between two segments
 * \author Karama Sriti
 * \version 0.1
 * \date June 14th 2013
 */
#include "distance.h"
#include <cmath>

Distance::Distance() {}
//-------------------------------------------------------
Distance::Distance(Segment human_limb_,Segment robot_link_)
{
	this->human_limb = human_limb_;
	this->robot_link = robot_link_;
	this->CalculateDistance();
}
//--------------------------------------------------------------------------------------------------------------------------------------
/*! \brief calculate the distance and so will fill the corresponding attribut */
void Distance::CalculateDistance()
{
	// some necessary variables for the distance equation
	// distance = sqrt((a^2+b^2+c^2)*t1^2 + (d^2+e^2+f^2)*t2^2 + 2*t1(a*(xa-xb)+b*(ya-yb)+c*(za-zb)) + 2*t2(d*(xb-xa)+e*(yb-ya)+f*(zb-za)) + (xa-xb)^2 + (ya-yb)^2 + (za-zb)^2-t1*t2*(cf+ad+bc))
	// simplication :
	//	a = a^2+b^2+c^2 
	//	b = d^2+e^2+f^2
	// 	c = a*(xa-xb)+b*(ya-yb)+c*(za-zb)
	//	d = d*(xb-xa)+e*(yb-ya)+f*(zb-za)
	// 	e = (xa-xb)^2 + (ya-yb)^2 + (za-zb)^2
	// 	f = c*f+a*d+b*c
	//	then the equation becoms : distance = a*(x^2) + b*(y^2) + 2*c*x +2*d*y + e -f*x*y        with x=t1 and y=t2 (the parameeters of the the two segments)
	this->distance = -1;

	double Ua(this->human_limb.getUx()), Ub(this->human_limb.getUy()), Uc(this->human_limb.getUz());				//a = Ua, b = Ub, c = Uc)
	double Ud(this->robot_link.getUx()), Ue(this->robot_link.getUy()), Uf(this->robot_link.getUz());				//d = Ud, e = Ue, f = Uf)

	double Px_human(this->human_limb.getPx()), Py_human(this->human_limb.getPy()), Pz_human(this->human_limb.getPz());	//xa = Px_human, ya = Py_human, za = Pz_human
	double Px_robot(this->robot_link.getPx()), Py_robot(this->robot_link.getPy()), Pz_robot(this->robot_link.getPz());	//xb = Px_robot, yb = Py_robot, zb = Pz_robot

	double a = pow(Ua,2)+pow(Ub,2)+pow(Uc,2);
	double b = pow(Ud,2)+pow(Ue,2)+pow(Uf,2);

	double c = Ua*(Px_human-Px_robot)+Ub*(Py_human-Py_robot)+Uc*(Pz_human-Pz_robot); 
	double d = Ud*(Px_robot-Px_human)+Ue*(Py_robot-Py_human)+Uf*(Pz_robot-Pz_human); 
	double e = pow((Px_human-Px_robot),2)+pow((Py_human-Py_robot),2)+pow((Pz_human-Pz_robot),2);
	double f = Ua*Ud+Ub*Ue+Uc*Uf;

	double param_human_segment(-1) ; // ***
	double param_robot_segment(-1) ;

	double param_human_limit = this->human_limb.getPrameterLimit();
	double param_robot_limit = this->robot_link.getPrameterLimit();


	// the parameter of the segment that belong to the robot
	param_robot_segment = ((c/a)-(2*d/f))/(((2*b)/f)-(f/(2*a)));

	if (param_robot_segment < 0)	param_robot_segment = 0;
		else if (param_robot_segment > param_robot_limit)	param_robot_segment =  param_robot_limit;
			else	param_robot_segment = param_robot_segment; 	

	// the parameter of the segment that belong to the robot
	param_human_limit = ((2*c) + (f*param_robot_segment))/(2*a);

	if (param_human_segment < 0)	param_human_segment = 0;
		else if (param_human_segment > param_human_limit)	param_human_segment =  param_human_limit;
			else	param_human_segment = param_human_segment; 
	 

	this->distance = sqrt( a*(pow(param_human_segment,2)) + b*(pow(param_robot_segment,2)) + 2*c*param_human_segment + 2*d*param_robot_segment + e - 2*f*param_human_segment*param_robot_segment);
	

	// the points coordinates, using the pametric equation of a line : 
	// X=Xa+at   
	// Y=Ya+bt
	// Z=za+ct
	// with A(Xa,Ya,Za) a point of belong to the line : Px_,Py_,Pz_
	// U(a,b,c) the normalized direction vector : Ua,Ub,Uc and Ud,Ue,Uf
	// t the parameeter of the equation : param_...

	this->point_belong_to_human.setX(Px_human + Ua*param_human_segment); 
	this->point_belong_to_human.setY(Py_human + Ub*param_human_segment); 
	this->point_belong_to_human.setZ(Pz_human + Uc*param_human_segment);

	this->point_belong_to_robot.setX(Px_robot + Ud*param_robot_segment); 
	this->point_belong_to_robot.setY(Py_robot + Ue*param_robot_segment); 
	this->point_belong_to_robot.setZ(Pz_robot + Uf*param_robot_segment);

	//-------------------------------------------------------------	
//	std::cout <<"Distance = " << this->distance << std::endl;
//	std::cout <<"P_human = (" << this->point_belong_to_human.getX() <<" ; "<< this->point_belong_to_human.getY() <<" ; "<< this->point_belong_to_human.getZ() <<  ")" << std::endl;
//	std::cout <<"P_robot = (" << this->point_belong_to_robot.getX() <<" ; "<< this->point_belong_to_robot.getY() <<" ; "<< this->point_belong_to_robot.getZ() <<  ")" << std::endl;
}
//-------------------------------------------------------------------------------------------------------------------------------------------
/*! \brief returns the distance */
double Distance::getDistance() const
{
	return this->distance ;
}
/*! \brief returns coordinates of the point that belong to the humain  */
tf::Vector3 Distance::getPointBelongToHuman() const
{
	return this->point_belong_to_human ;
}
/*! \brief returns coordinates of the point that belong to the robot */
tf::Vector3 Distance::getPointBelongToRobot() const
{
	return this->point_belong_to_robot ;
}
/*! \brief set the elements of this class */
void Distance::SetSegments(Segment human_limb_,Segment robot_link_)
{
	this->human_limb = human_limb_ ;
	this->robot_link = robot_link_ ;
	Distance::CalculateDistance();
}
