
#include <support.h>
void initSegments(std::vector<tf::StampedTransform>& human_transform_now, 
		  std::vector<tf::StampedTransform>& human_transform, 
				std::vector<tf::StampedTransform>& robot_transform_now, 
				std::vector<tf::StampedTransform>& robot_transform, 
					unsigned int num_human_seg, 
					unsigned int rob_human_seg,
						std::vector<Segment>& human_limbes_now, 
						std::vector<Segment>& human_limbes_,
							std::vector<Segment>& robot_links_now, 
							std::vector<Segment>& robot_links_)
{

	tf::StampedTransform world;
			world.setRotation( tf::Quaternion( 0, 0, 0, 1 ) );
			world.setOrigin( tf::Vector3(0, 0, 0 ) );

	human_limbes_now.resize(num_human_seg);
	human_limbes_.resize(num_human_seg);

	robot_links_now.resize(rob_human_seg);
	robot_links_.resize(rob_human_seg);

	//-----------------------------------> Segments Set for now transforms 	
	if(human_transform_now.empty())
	{
		std::cout << "attention! user_transform is empty" << std::endl;
	}
	else
	{
		human_limbes_now[0].setSegmentAttribut(human_transform_now[1],human_transform_now[0],"UB");//head_neck_torso (upper_body)
		human_limbes_now[1].setSegmentAttribut(human_transform_now[2],human_transform_now[3],"LA");//left_arm
		human_limbes_now[2].setSegmentAttribut(human_transform_now[3],human_transform_now[4],"LF");//left_forearm
		human_limbes_now[3].setSegmentAttribut(human_transform_now[5],human_transform_now[6],"RA");//right_arm
		human_limbes_now[4].setSegmentAttribut(human_transform_now[6],human_transform_now[7],"RF");//right_forearm
		human_limbes_now[5].setSegmentAttribut(human_transform_now[2],human_transform_now[5],"Ch");//chest
		//add here others limbes if needed, dont forget to reset number_of_human_limbes in file.ops
	}
	//-----------------------------------> Segments Set		
	if(human_transform.empty())
	{
		std::cout << "attention! user_transform is empty" << std::endl;
	}
	else
	{
		human_limbes_[0].setSegmentAttribut(human_transform[1],human_transform[0],"UB");//head_neck_torso (upper_body)
		human_limbes_[1].setSegmentAttribut(human_transform[2],human_transform[3],"LA");//left_arm
		human_limbes_[2].setSegmentAttribut(human_transform[3],human_transform[4],"LF");//left_forearm
		human_limbes_[3].setSegmentAttribut(human_transform[5],human_transform[6],"RA");//right_arm
		human_limbes_[4].setSegmentAttribut(human_transform[6],human_transform[7],"RF");//right_forearm
		human_limbes_[5].setSegmentAttribut(human_transform[2],human_transform[5],"Ch");//chest
		//add here others limbes if needed, dont forget to reset number_of_human_limbes in file.ops
	}

	//-----------------------------------> 
	if(robot_transform_now.empty())
	{
		std::cout << "attention! robot_transform is empty" << std::endl;
	}
	else
	{
		robot_links_now[0].setSegmentAttribut(world,robot_transform_now[1],"link1");
		robot_links_now[1].setSegmentAttribut(robot_transform_now[1],robot_transform_now[3],"link2");
		robot_links_now[2].setSegmentAttribut(robot_transform_now[3],robot_transform_now[5],"link3");
		//robot_links_now[3].setSegmentAttribut(robot_transform_now[2],robot_transform_now[3],"link4");
		//robot_links_now[4].setSegmentAttribut(robot_transform_now[3],robot_transform_now[4],"link5");
		//robot_links_now[5].setSegmentAttribut(robot_transform_now[4],robot_transform_now[5],"link6");
		//robot_links_now[6].setSegmentAttribut(robot_transform_now[5],robot_transform_now[6],"link7");
	}
	if(robot_transform.empty())
	{
		std::cout << "attention! robot_transform is empty" << std::endl;
	}
	else
	{
		robot_links_[0].setSegmentAttribut(world,robot_transform[0],"link1");
		robot_links_[1].setSegmentAttribut(robot_transform[0],robot_transform[1],"link2");
		robot_links_[2].setSegmentAttribut(robot_transform[1],robot_transform[2],"link3");
		//robot_links_[3].setSegmentAttribut(robot_transform[2],robot_transform[3],"link4");
		//robot_links_[4].setSegmentAttribut(robot_transform[3],robot_transform[4],"link5");
		//robot_links_[5].setSegmentAttribut(robot_transform[4],robot_transform[5],"link6");
		//robot_links_[6].setSegmentAttribut(robot_transform[5],robot_transform[6],"link7");
	}

}
