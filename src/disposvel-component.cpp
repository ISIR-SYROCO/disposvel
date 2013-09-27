/*!
 * \file Disposvel.cpp
 * \brief Orocos component that provides to Antoine's controller the distance, position and velocity between robot's segments and human's limbe
 * \author Karama Sriti
 * \version 0.1
 * \date June 14th 2013
 */
#include <disposvel-component.hpp>
#include <support.h>
/*! \brief constructor of the component disposvel, that computes distances, positions and velocities  */
Disposvel::Disposvel(const std::string &name) :
	RTT::TaskContext(name, RTT::TaskContext::PreOperational)
	, lookup_now("lookupTransform")
	, lookup("lookupTransformAtTime")
	, num_rob_links(0)
	, num_hum_limb(0)
	, num_rob_frame(0)
	, num_hum_frame(0)
{
	//tell the RTT the name and type of this struct
	RTT::types::Types()->addType(new ControlDataTypeInfo());
	RTT::types::Types()->type("ControlData")->addConstructor( RTT::types::newConstructor(&createCD) );
	
	//register a std::vector (or compatible) of some type
	RTT::types::Types()->addType(new RTT::types::SequenceTypeInfo< std::vector<ControlData> > ("std::vector<ControlData>"));

	//------------------------------------------------
	this->addProperty("robot_transform",robot_transform)
		.doc("the vector of transform recoverd from tf topic");

	this->addProperty("robot_transform_now",robot_transform_now)
		.doc("the vector of transform recoverd from tf topic");

	this->addProperty("human_transform",human_transform)
		.doc("the vector of transform recoverd from tf topic");

	this->addProperty("human_transform_now",human_transform_now)
		.doc("the vector of transform recoverd from tf topic");

	this->addProperty("number_of_robot_frames",num_rob_frame)
		.doc("reperes lie au robot");

	this->addProperty("number_of_human_frames",num_hum_frame)
		.doc("reperes lie a l'humain");

	this->addProperty("number_of_robot_links",num_rob_links)
		.doc("les segments du robot");

	this->addProperty("number_of_human_limbes",num_hum_limb)
		.doc("les membres de l'humain");

	this->addProperty("tform_now_rob",tform_now_rob)
		.doc("variable that recover transform from tf topic");
	this->addProperty("tform_rob",tform_rob)
		.doc("variable that recover transform from tf topic");

	this->addProperty("tform_now_hum",tform_now_hum)
		.doc("variable that recover transform from tf topic");
	this->addProperty("tform_hum",tform_hum)
		.doc("variable that recover transform from tf topic");

	this->requires("tf")->addOperationCaller(lookup_now);
	this->requires("tf")->addOperationCaller(lookup);
	//------------------------------------------------
	std::cout << "Disposvel constructed !" << std::endl;
}//constructor
//virtual Disposvel::~Disposvel()  { }
//----------------------------------> configure
/*! \brief configuration of the component */
bool Disposvel::configureHook() 
{ 
	//--------------------	
	if(num_rob_frame == 0)
	{
		std::cout << "Please define the number of the robot frames" << std::endl;
		return false;
	}
	else 
	{	
		robot_transform_now.resize(num_rob_frame);
		robot_transform.resize(num_rob_frame);

		for(unsigned int i=0 ; i < robot_transform_now.size(); i++)
					{
						// transform now
						robot_transform_now[i].setRotation( tf::Quaternion( 0, 0, 0, 0 ) );
						robot_transform_now[i].setOrigin( tf::Vector3(0, 0, 0 ) );
						// past transform
						robot_transform[i].setRotation( tf::Quaternion( 0, 0, 0, 0 ) );
						robot_transform[i].setOrigin( tf::Vector3(0, 0, 0 ) );
					}	
	}
	//--------------------
	if(num_hum_frame == 0)
	{
		std::cout << "Please define the number of the human frames" << std::endl;
		return false;
	}
	else 
	{	
		human_transform_now.resize(num_hum_frame);
		human_transform.resize(num_hum_frame);

		for(unsigned int i=0 ; i < human_transform_now.size() ; i++)
					{
						//transform now
						human_transform_now[i].setRotation( tf::Quaternion( 0, 0, 0, 0 ) );
						human_transform_now[i].setOrigin( tf::Vector3(0, 0, 0 ) );
						//past transform
						human_transform[i].setRotation( tf::Quaternion( 0, 0, 0, 0 ) );
						human_transform[i].setOrigin( tf::Vector3(0, 0, 0 ) );
					}	
	}
	if(num_hum_limb == 0)
	{
		std::cout << "Please define the number of the human limbes" << std::endl;
		return false;
	}
	if(num_rob_links == 0)
	{
		std::cout << "Please define the number of the robot links" << std::endl;
		return false;
	}
	addPorts();
	std::cout << "Disposvel component configured" << std::endl;

	
	return true;
}//configure
//----------------------------------> start
/*! \brief Start of the component  */
bool Disposvel::startHook() 
{	
	if(num_rob_frame == 0)
	{
		std::cout << "Please define the number of the robot frames" << std::endl;
		return false;
	}
	
	std::cout << "Disposvel component started" << std::endl;
	return true;
}//start
//----------------------------------> update
/*! \brief The updateHook of the component */
void Disposvel::updateHook() 
{
	double duree(1.0);

	std::string robot_frame_id[] = {"/segment_1","/segment_2","/segment_3","/segment_4","/segment_5","/segment_6","/segment_7"}; // if tf are recovered from tf_robot

	std::string human_frame_id[] = { "/head_1","/torso_1","/left_elbow_1" ,"/left_shoulder_1" ,"/left_hand_1" ,
	  	  		         "/right_elbow_1","/right_shoulder_1","/right_hand_1"}; // only 7DOF are considered

	tf::StampedTransform world;
		world.setRotation( tf::Quaternion( 0, 0, 0, 1 ) );
		world.setOrigin( tf::Vector3(0, 0, 0 ) );

	//Segment robot_links_now[num_rob_links], robot_links_[num_rob_links],human_limbes_now[num_hum_limb], human_limbes_[num_hum_limb] ;
	std::vector<Segment> robot_links_now, robot_links_,human_limbes_now, human_limbes_ ;

	//-----------------------------------> recuperation des TF	
	try
	{
		for(unsigned int i=0 ; i < robot_transform_now.size() ; i++)
		{
			tform_now_rob = lookup_now("/world",robot_frame_id[i]);
			tform_rob     = lookup("/world",robot_frame_id[i],ros::Time::now()-ros::Duration(duree));

			tf::transformStampedMsgToTF(tform_now_rob, robot_transform_now[i]);
			tf::transformStampedMsgToTF(tform_rob, robot_transform[i]);
		}

		for(unsigned int j=0 ; j < human_transform_now.size() ; j++)
		{
			tform_now_hum = lookup_now("/world",human_frame_id[j]);
			tform_hum     = lookup("/world",human_frame_id[j],ros::Time::now()-ros::Duration(duree));

			tf::transformStampedMsgToTF(tform_now_hum, human_transform_now[j]);
			tf::transformStampedMsgToTF(tform_hum, human_transform[j]);
		}
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());	
	}
	//------------------------------------------------------<
	//-----------------------------------> Segments Set for now transforms 
	initSegments(human_transform_now, 
		     human_transform, 
			robot_transform_now, 
			robot_transform, 
				num_hum_limb, 
				num_rob_links,
					human_limbes_now, 
					human_limbes_,
						robot_links_now, 
						robot_links_);
	//------------------------------------------------------<

	//-----------------------------------> calcul des distances et envoie des donnees		
	Distance dist_now[num_rob_links*num_hum_limb];
	Distance dist[num_rob_links*num_hum_limb];

	unsigned int n(0), size(num_rob_links*num_hum_limb);
	std::vector<ControlData> donnee;
	donnee.resize(size);

	for(unsigned int i=0 ; i < num_hum_limb-1 ; i++)
	{
		for(unsigned int j=0 ; j < num_rob_links ; j++)
		{
			dist[n].SetSegments(human_limbes_[i],robot_links_[j]);
			dist_now[n].SetSegments(human_limbes_now[i],robot_links_now[j]);

			donnee[n].distance_id = human_limbes_now[i].getSegmentId()+"-"+robot_links_now[j].getSegmentId();
			donnee[n].distance = dist_now[n].getDistance();

			donnee[n].ph[0] = dist_now[n].getPointBelongToHuman().getX();
			donnee[n].ph[1] = dist_now[n].getPointBelongToHuman().getY();
			donnee[n].ph[2] = dist_now[n].getPointBelongToHuman().getZ();

			donnee[n].pr[0] = dist_now[n].getPointBelongToRobot().getX();
			donnee[n].pr[1] = dist_now[n].getPointBelongToRobot().getY();
			donnee[n].pr[2] = dist_now[n].getPointBelongToRobot().getZ();

			donnee[n].vph[0] = (dist_now[n].getPointBelongToHuman().getX() - dist[n].getPointBelongToHuman().getX());// /duree;
			donnee[n].vph[1] = (dist_now[n].getPointBelongToHuman().getY() - dist[n].getPointBelongToHuman().getY());// /duree;
			donnee[n].vph[2] = (dist_now[n].getPointBelongToHuman().getZ() - dist[n].getPointBelongToHuman().getZ());// /duree;

			donnee[n].vpr[0] = (dist_now[n].getPointBelongToRobot().getX() - dist[n].getPointBelongToRobot().getX());// /duree; 
			donnee[n].vpr[1] = (dist_now[n].getPointBelongToRobot().getY() - dist[n].getPointBelongToRobot().getY());// /duree; 
			donnee[n].vpr[2] = (dist_now[n].getPointBelongToRobot().getZ() - dist[n].getPointBelongToRobot().getZ());// /duree; 
			//--------------------------------------------------
			std::cout << "-----------------------------------" <<  std::endl;
			std::cout << "ID : " << donnee[n].distance_id << std::endl;
			std::cout << "distance : " << donnee[n].distance << std::endl;
			std::cout << "position human : " << donnee[n].ph[0]   <<","<< donnee[n].ph[1]   <<","<< donnee[n].ph[2]   << std::endl;
			std::cout << "position robot : " << donnee[n].pr[0]   <<","<< donnee[n].pr[1]   <<","<< donnee[n].pr[2]   << std::endl;
			std::cout << "velocity human : " << donnee[n].vph[0]  <<","<< donnee[n].vph[1]  <<","<< donnee[n].vph[2]  << std::endl;
			std::cout << "velocity robot : " << donnee[n].vpr[0]  <<","<< donnee[n].vpr[1]  <<","<< donnee[n].vpr[2]  << std::endl;
			std::cout << "-----------------------------------" <<  std::endl;
			//--------------------------------------------------
			n++;
		}
	}		
	sortie.write(donnee);
}//update
//----------------------------------> stop
/*! \brief Stop of the component  */
void Disposvel::stopHook() 
{
	std::cout << "Disposvel component stoped" << std::endl;
}//stop
void Disposvel::cleanupHook()
{
	std::cout << "Disposvel component cleanupHook" << std::endl;
}
//----------------------------------> addPorts
/*! \brief Configures and add the OutPut ports of the component  */
void Disposvel::addPorts()
{
	unsigned int size = num_rob_links*num_hum_limb;
	std::vector<ControlData> donnee;
	donnee.resize(size);
	for(unsigned int i = 0 ;  i<size ; i++)
	{
		donnee[i] = ControlData("humanLimb-robotLink",100.00,100.0,100.0,0.0,0.0);
	}
	this->addPort("sortie",sortie);
	sortie.setDataSample(donnee);
}//addPorts
//------------------------------>
ORO_CREATE_COMPONENT(Disposvel);
//------------------------------<
