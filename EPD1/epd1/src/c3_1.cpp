#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#define NUM_FRAMES 20

float degreesToRadians(float degrees){
	return (degrees*(2*M_PI))/360;
}

void sendTransform(void){

	static tf::TransformBroadcaster br;

	//Name of the coordinate frame	
	std::string frame_name = "frame_1";

	//Represents the full transformation
  	//Represents the full transformation
  	tf::Transform transform [NUM_FRAMES];
	float max_roll = degreesToRadians(15);
	float max_pitch = degreesToRadians(22);

	// 0,0 to 15,22 to 0,0 to -15,-22
	float total_roll_amplitude = max_roll*3;
	float total_pitch_amplitude = max_pitch*3;
	
	//Get delta of roll and pitch in each iteration:
	float roll_delta = -total_roll_amplitude/NUM_FRAMES;
	float pitch_delta = -total_pitch_amplitude/NUM_FRAMES;

	float current_roll = 0;
	float current_pitch = 0;
	
	for(int i=0;i<NUM_FRAMES;i++)
	{
		//Translation
		//transform[i].setOrigin( tf::Vector3(0.5,0,0) ); //0.5m on X
		
		//Translation
	//transform.setOrigin( tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0) );
transform[i].setOrigin( tf::Vector3(0.5,0,0) ); //0.5m on X

		//Rotation
		/*tf::Quaternion q; //TF uses quaternions, but we can create them from roll,pitch and yaw (line below)
  		q.setRPY(roll_delta, pitch_delta, 0);  //Roll, pitch and yaw angles
  		transform[i].setRotation(q);*/
		
		//Rotation
	tf::Quaternion q; //TF uses quaternions, but we can create them from roll,pitch and yaw (line below)
  	q.setRPY(0, 0, 45.0*M_PI/180.0);  //Roll, pitch and yaw angles
  	transform.setRotation(q);
		
		if(fabsf(current_roll + roll_delta) > max_roll){
			roll_delta = -1*roll_delta;
		}
		
		if(fabsf(current_pitch + pitch_delta) > max_roll){
			pitch_delta = -1*pitch_delta;
		}

		current_roll += roll_delta;
		current_pitch += pitch_delta;
		
	}
	

	

	

	//Send the transformation between "world" and "frame_1" with a time stamp
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", frame_name));
 
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "epd1_c3");

  	ros::NodeHandle node;
  
  	ros::Rate loop_rate(1);


  	while (ros::ok())
  	{

		sendTransform();
    		ros::spinOnce();

    		loop_rate.sleep();
  	}

  	return 0;
};
