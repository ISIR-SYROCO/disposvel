disposvel
---------
	- orocos component 
	- get back the related frames linked to the human body and the robot segments
	- calculates the distances between each pair "human limbe" - "robot segment"
	- returns the distance struct :
	 (std::vector<RTT::OutputPOrt <CondtrolData> *> port)
		struct ControlData
		{
			ControlData():distance_id("humanLimb-robotLink"), distance(100.00),ph(3,0.0), pr(3,0.0){} //intialization
			ControlData(std::string distance_id,double ph,double pr):distance_id("humanLimb-robotLink"), ph(3,0.0), pr(3,0.0){} //intialization

			std::string distance_id;
			double distance; 
			std::vector<double> ph; 	position (x,y,z) of the point that belong to the human limbe
			std::vector<double> pr;		position (x,y,z) of the point that belong to the robot link
		};


