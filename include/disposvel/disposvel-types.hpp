#include <vector>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/TemplateConstructor.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>

/*!
 * \file disposvel-types.hpp
 * \brief construction of typekit for the communication between distance and controller components
 * \author Karama Sriti
 * \version 0.1
 * \date June 14th 2013
 */
struct ControlData
{
	ControlData():distance_id("humanLimb-robotLink"), distance(100.0), ph(3,100.0), pr(3,100.0), vph(3,100.0), vpr(3,100.0){}
	ControlData(std::string distance_id, double distance, double ph, double pr, double vph, double pvr):distance_id("humanLimb-robotLink"), distance(100.0), ph(3,100.0), pr(3,100.0), vph(3,100.0), vpr(3,100.0){}


	std::string distance_id;
	double distance;
	std::vector<double> ph;		//position of the point that belong to the human segment
	std::vector<double> pr;		//position of the point that belong to the robot segment

	std::vector<double> vph;	//velocity of the point that belong to the human segment
	std::vector<double> vpr;	//velocity of the point that belong to the robot segment
};

//this 'factory' function contructs one object
//ControlData createCD(std::string distance_id,double distance,double ph,double pr)
ControlData createCD(std::string distance_id, double distance, double ph, double pr, double vph, double vpr)
{
	return ControlData(distance_id, distance, ph, pr, vph, vpr);
}

namespace boost
{
	namespace serialization 
	{
		//the helper function
		template<class Archive>
		void serialize(Archive & a, ControlData & cd, unsigned int) 
		{
			using boost::serialization::make_nvp;
			a & make_nvp("distance_id",cd.distance_id);
			a & make_nvp("distance",cd.distance);
			a & make_nvp("ph",cd.ph);
			a & make_nvp("pr",cd.pr);
			a & make_nvp("vph",cd.vph);
			a & make_nvp("vpr",cd.vpr);
		}
	}
}

//RTT helper class which uses the above function behind the scences 
struct ControlDataTypeInfo
	:public RTT::types::StructTypeInfo<ControlData>
{
	ControlDataTypeInfo()
		:RTT::types::StructTypeInfo<ControlData>("ControlData"){}
};

