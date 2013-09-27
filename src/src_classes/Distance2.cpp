/*!
 * \file Distance.cpp
 * \brief calculate the distance between two cylinders
 * \author Karama Sriti
 * \version 0.1
 * \date June 14th 2013
 */
#include "distance2.h"
#include <cmath>
#include <vector>

Distance2::Distance2() {}
//-------------------------------------------------------
Distance2::Distance2(Cylinder human_limb_,Cylinder robot_link_)
{
	this->human_limb = human_limb_;
	this->robot_link = robot_link_;
	this->CalculateDistance();
}
//--------------------------------------------------------------------------------------------------------------------------------------
/*! \brief calculate the distance and so will fill the corresponding attribut */
void Distance2::CalculateDistance()
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

	//-------------------------------------------------------------
	std::cout << "-----in CalculateDistance() ------" << std::endl;
	std::cout <<"d = " << distance << std::endl;
	//-------------------------------------------------------------

	double Ua(this->human_limb.getSegment().getUx()), Ub(this->human_limb.getSegment().getUy()), Uc(this->human_limb.getSegment().getUz());				//a = Ua, b = Ub, c = Uc)
	double Ud(this->robot_link.getSegment().getUx()), Ue(this->robot_link.getSegment().getUy()), Uf(this->robot_link.getSegment().getUz());				//d = Ud, e = Ue, f = Uf)
	//-------------------------------------------------------------

	std::cout <<"Uab = (" << Ua <<";"<< Ub <<";"<< Uc << ")" << std::endl;
	std::cout <<"Ucd = (" << Ud <<";"<< Ue <<";"<< Uf << ")" << std::endl;
	//-------------------------------------------------------------

	double Px_human(this->human_limb.getSegment().getPx()), Py_human(this->human_limb.getSegment().getPy()), Pz_human(this->human_limb.getSegment().getPz());	//xa = Px_human, ya = Py_human, za = Pz_human
	double Px_robot(this->robot_link.getSegment().getPx()), Py_robot(this->robot_link.getSegment().getPy()), Pz_robot(this->robot_link.getSegment().getPz());	//xb = Px_robot, yb = Py_robot, zb = Pz_robot

	//-------------------------------------------------------------
	std::cout <<"Porigine_human = (" << Px_human <<";"<< Py_human <<";"<< Pz_human << ")" << std::endl;
	std::cout <<"Porigine_robot = (" << Px_robot <<";"<< Py_robot <<";"<< Pz_robot << ")" << std::endl;
	//-------------------------------------------------------------	

	double a = pow(Ua,2)+pow(Ub,2)+pow(Uc,2);
	double b = pow(Ud,2)+pow(Ue,2)+pow(Uf,2);

	double c = Ua*(Px_human-Px_robot)+Ub*(Py_human-Py_robot)+Uc*(Pz_human-Pz_robot); 
	double d = Ud*(Px_robot-Px_human)+Ue*(Py_robot-Py_human)+Uf*(Pz_robot-Pz_human); 
	double e = pow((Px_human-Px_robot),2)+pow((Py_human-Py_robot),2)+pow((Pz_human-Pz_robot),2);
	double f = Ua*Ud+Ub*Ue+Uc*Uf;

	double param_human_segment(-1) ; // ***
	double param_robot_segment(-1) ;

	double param_human_limit = this->human_limb.getSegment().getPrameterLimit();
	double param_robot_limit = this->robot_link.getSegment().getPrameterLimit();

	// the parameter of the segment that belong to the robot
	param_robot_segment = ((c/a)-(2*d/f))/(((2*b)/f)-(f/(2*a)));

	// pour un segement : 0 < parama_equation_de_droite < param_limite
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

	this->point_belong_to_robot.setX(Px_human - Ua*param_human_segment); 
	this->point_belong_to_robot.setY(Py_human - Ub*param_human_segment); 
	this->point_belong_to_robot.setZ(Pz_human - Uc*param_human_segment);

	this->point_belong_to_human.setX(Px_robot - Ud*param_robot_segment); 
	this->point_belong_to_human.setY(Py_robot - Ue*param_robot_segment); 
	this->point_belong_to_human.setZ(Pz_robot - Uf*param_robot_segment);

	//-------------------------------------------------------------
	std::cout <<"P_human = (" << point_belong_to_human.getX() <<";"<< point_belong_to_human.getY() <<";"<< point_belong_to_human.getZ() << ")" << std::endl;
	std::cout <<"P_robot = (" << point_belong_to_robot.getX() <<";"<< point_belong_to_robot.getY() <<";"<< point_belong_to_robot.getZ() << ")" << std::endl;
	//-------------------------------------------------------------	

	//-------------------------------------------------------------
	std::cout << "d = " << distance << std::endl;
	//std::cout << "-----out CalculateDistance() ------" << std::endl;
	//-------------------------------------------------------------	

//----------------------> calcul des positions et distances en prenant compte le volume du cylinder capsule

	//calcul de la nouvelle position, en prenant compe le volume du cylindre 
	//this->point_belong_to_robot;  
	//this->point_belong_to_human;

	//-------------------------------------------------------------
	std::cout << "prise en compte du volume " <<  std::endl;
	
	//-------------------------------------------------------------	
	struct point 
	{
	    	double x;
	    	double y;
		double z;
	};

	point m, n;
	m.x = point_belong_to_robot.getX() ; m.y = point_belong_to_robot.getY() ; m.z = point_belong_to_robot.getZ();		//M
	std::cout << "m(" << m.x <<";"<< m.y <<";"<< m.z << ")" << std::endl;
	//-----------------------------------------
	n.x = point_belong_to_human.getX() ; n.y = point_belong_to_human.getY() ; n.z = point_belong_to_human.getZ();		//N
	std::cout << "n(" << n.x <<";"<< n.y <<";"<< n.z << ")" << std::endl;


	 
	double dpchn = this->human_limb.getRadius();
	double dpcrm = this->robot_link.getRadius();
	//-------------------------------------------------------------
	std::cout << "radius human = " << this->human_limb.getRadius() << std::endl;
	std::cout << "radius robot = " << this->robot_link.getRadius() << std::endl;
	//-------------------------------------------------------------
	tf::Vector3  dirHR; dirHR.setX(m.x-n.x); dirHR.setX(m.x-n.x); dirHR.setX(m.x-n.x);
	dirHR  = dirHR.normalized() ;  // dirHR le vecteur directeur de la droite qui porte la distance
	this->point_belong_to_robot.setX(m.x - dirHR.getX()*dpcrm); 
	this->point_belong_to_robot.setY(m.y - dirHR.getX()*dpcrm); 
	this->point_belong_to_robot.setZ(m.z - dirHR.getX()*dpcrm);

	this->point_belong_to_human.setX(n.x - dirHR.getX()*dpchn); 
	this->point_belong_to_human.setY(n.y - dirHR.getX()*dpchn); 
	this->point_belong_to_human.setZ(n.z - dirHR.getX()*dpchn);

	// le calcul de la distance finale
	this->distance = sqrt( (pow((point_belong_to_human.getX()-point_belong_to_robot.getX()),2)) + (pow((point_belong_to_human.getY()-point_belong_to_robot.getY()),2)) + (pow((point_belong_to_human.getZ()-point_belong_to_robot.getZ()),2)));
	//-------------------------------------------------------------
	std::cout <<"P_human = (" << point_belong_to_human.getX() <<";"<< point_belong_to_human.getY() <<";"<< point_belong_to_human.getZ() << ")" << std::endl;
	std::cout <<"P_robot = (" << point_belong_to_robot.getX() <<";"<< point_belong_to_robot.getY() <<";"<< point_belong_to_robot.getZ() << ")" << std::endl;
	//-------------------------------------------------------------	
	std::cout << "-----out CalculateDistance() ------" << std::endl;
}
//-------------------------------------------------------------------------------------------------------------------------------------------
/*! \brief returns the distance */
double Distance2::getDistance() const
{
	return this->distance ;
}
/*! \brief returns coordinates of the point that belong to the humain  */
tf::Vector3 Distance2::getPointBelongToHuman() const
{
	return this->point_belong_to_human ;
}
/*! \brief returns coordinates of the point that belong to the robot */
tf::Vector3 Distance2::getPointBelongToRobot() const
{
	return this->point_belong_to_robot ;
}
/*! \brief set the elements of this class */
void Distance2::SetCylinders(Cylinder human_limb_,Cylinder robot_link_)
{
	this->human_limb = human_limb_ ;
	this->robot_link = robot_link_ ;
	Distance2::CalculateDistance();
}
