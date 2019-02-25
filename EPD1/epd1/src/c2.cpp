#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


void sendTransform(void){

	static tf::TransformBroadcaster br;

	//Name of the coordinate frame	
	//std::ostringstream frame_name;
	//std::ostringstream frame_name_new;
	

	//Represents the full transformation
  	tf::Transform transform [14];
	float angle = M_PI/7; //EN RADIANES
	float height = 0.19;

	for(int i=0;i<14;i++)
	{
		//Translation
		transform[i].setOrigin( tf::Vector3(cos(angle)*0.8, sin(angle)*0.8, height) );

		//Rotation
		tf::Quaternion q; //TF uses quaternions, but we can create them from roll,pitch and yaw (line below)
  		q.setRPY(0, 0, angle);  //Roll, pitch and yaw angles
  		transform[i].setRotation(q);
		
	}

	//Send the transformation between "world" and "frame_0" with a time stamp
  	br.sendTransform(tf::StampedTransform(transform[0], ros::Time::now(), "world", "frame_0"));

	for(int i=1;i<14;i++)
	{
		std::ostringstream frame_name;
		std::ostringstream frame_name_new;
		
		frame_name << "frame_" << i-1; //si se hace respecto al world, hay que hacerlo de otra forma
		frame_name_new << "frame_" << i;
  		br.sendTransform(tf::StampedTransform(transform[i], ros::Time::now(), frame_name.str(), frame_name_new.str()));
		

	}
	
 
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "epd1_c2");

  	ros::NodeHandle node;
  
  	ros::Rate loop_rate(10);

  	while (ros::ok())
  	{

		sendTransform();
    		ros::spinOnce();

    		loop_rate.sleep();
  	}

  	return 0;
};

