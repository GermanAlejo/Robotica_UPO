#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
//JAC
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
// Costmap
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include"GlobalPlanner.h"

// Representation (RVIZ)
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>


tf::TransformListener *tf_ptr=NULL;
ros::Publisher *map_pose_pub_ptr=NULL;

//ADDED CODE FROM HERE
//global variables

  	float tempX, tempY;//variables temporales para indicar el objetivo
	float vectorResX = 0;
	float vectorResY = 0;

	float dist=0;//distancia al goal/punto de evasion
	float distToGoal;//variable fija al goal, usada para dar peso al goal cuando hay un objeto cerca del goal
/**
* Our class to control the robot
* It has members to store the robot pose, and
* methods to control the robot by publishing data
*/
class Turtlebot
{
public:
  Turtlebot();
 
  /*
   * This function should command the robot to reach the goal
   * It should compute the commands to the robot by knowing the current position
   * and the goal position.
   * This function will return true only if the goal has been reached.
   */
  bool command(double goal_x, double goal_y);

private:

  
  ros::NodeHandle nh_;
 
  
  //2D robot pose
  double x,y,theta;
  // Scan
  sensor_msgs::LaserScan data_scan;
  ros::Subscriber kinect_sub_;
  
  //Transform listerner to obtain the transform between the world frame (odom) and the robot frame (base_link)
  tf::TransformListener listener;

  //Publisher and subscribers
  ros::Publisher vel_pub_;
 
  //!Publish the command to the turtlebot
  void publish(double angular_vel, double linear_vel);
  
  //!Callback for kinect
  void receiveKinect(const sensor_msgs::LaserScan & laser_kinect);

};

Turtlebot::Turtlebot()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  kinect_sub_ = nh_.subscribe("/scan",1,&Turtlebot::receiveKinect,this);

}


//command function from EPD3
bool Turtlebot::command(double gx, double gy)  
{

	double linear_vel=0.0;
	double angular_vel=0.0;
  	float ang = 0.0;
	bool ret_val = false;

  	//std::cout <<"llamando al command" <<std::endl;
	//Transform the goal to the local frame
	geometry_msgs::PointStamped goal;
	geometry_msgs::PointStamped base_goal;
	
  	goal.header.frame_id = "map";

  	//we'll just use the most recent transform available for our simple example
  	goal.header.stamp = ros::Time();

  	//just an arbitrary point in space
  	goal.point.x = gx;
  	goal.point.y = gy;
  	goal.point.z = 0.0;

	try{
	    
		listener.transformPoint("base_link", goal, base_goal);

	    ROS_INFO("goal: (%.2f, %.2f. %.2f) -----> base_goal: (%.2f, %.2f, %.2f) at time %.2f",
		goal.point.x, goal.point.y, goal.point.z,
		base_goal.point.x, base_goal.point.y, base_goal.point.z, base_goal.header.stamp.toSec());

  	}catch(tf::TransformException& ex){
    		ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
		return ret_val;
  	}
	
	/**
	* This should be completed. You should use a proportional controller
	* The linear velocity should be proportional to the distance to the goal
	* The angular velocity should be proportional to the difference in the angle towards
	* the goal and the current angle of the robot. You should check if you reached the goal
	*/
	 //calculate angle if is minimal then we are looking towards the goal
  
  /*
  *Si hay objeto las variables de vectorRes modificaran las variables auxiliares
  *para que el robot modifique su movimiento para evadir el obstaculo
  *si no hay obscaulo las variables valdran 0 manteniendo el goal original
  */
  tempX = base_goal.point.x + 0.01*vectorResX;
  tempY = base_goal.point.y + 0.01*vectorResY;
  dist = sqrt(pow(tempX, 2) + pow(tempY, 2));

  //calculate the module of base_goal to know distand towards objective
  distToGoal = sqrt(pow(base_goal.point.x, 2) + pow(base_goal.point.y, 2));
  
  //std::cout <<"BASEGOALX "<<base_goal.point.x<<std::endl;
  //std::cout <<"BASEGOALY "<<base_goal.point.y<<std::endl;
  //std::cout <<"command" <<std::endl;
  ang = atan2(tempY, tempX);
 // ang = ang*180/M_PI;
  std::cout <<"DIST "<<dist<<std::endl;
  std::cout <<"ANG "<<ang<<std::endl;
  //controla las velocidades
	/*
	Las velocidades al estar controladas en funcion de la distancia y el angulo
	se realentizan cuando no hay objetos, cuanto mas cerca este del objetivo mas lento
	aumentar la velocidad cuando no haya objetos detectados o el goal esta muy cerca
	*/
	//object
	if(vectorResX != 0 && vectorResY != 0){
	
		angular_vel = 0.2*ang;//velocidad angular controlada en funcion del angulo
  		linear_vel = 0.16*dist;//velocidad linear controlada en funcion de la distancia al goal
		
	//no object
	}else{
		
		angular_vel = 0.2*ang;
		linear_vel = 0.3*dist;
	
	}
	
 
  
  if(angular_vel > 0.8){
    angular_vel = 0.8;
  }
  
  if(linear_vel > 0.6){
    linear_vel = 0.6;
  }
  

  std::cout <<"DISTANCIA AL GOAL"<<distToGoal<<std::endl;
  //goal reached
	if(distToGoal < 0.1){
		linear_vel =0.0;
    	angular_vel=0.0;
    	ret_val = true;
	}
  std::cout <<"LINEAR_VEL"<<linear_vel<<std::endl;
  std::cout <<"ANGULAR_VEL"<<angular_vel<<std::endl;
  publish(angular_vel,linear_vel);    


  return ret_val;
	
}



//Publish the command to the turtlebot
void Turtlebot::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;


    vel_pub_.publish(vel);    


  return;
}

//Callback for robot position
void Turtlebot::receiveKinect(const sensor_msgs::LaserScan& msg)
{
    data_scan=msg;
    // Different variables used to detect obstacles
  	//calcula tamaño del vector
  	int rangeSize = (int) (data_scan.angle_max - data_scan.angle_min)/data_scan.angle_increment;
  	int i = 0;
  	float pX = 0, pY = 0;
  	float rangeDist;
  	float rangeAng;
    bool enc=false;

	//print range size
    //std::cout <<"Range Size"<<rangeSize<<std::endl;
  	
    vectorResX = 0;
    vectorResY = 0;

  //busca obstaculos
	while(i<rangeSize && !enc){

		//no tengas en cuenta objetos que no esten al frente de esta forma no realentizaran el robot
    	if(!std::isnan(data_scan.ranges.at(i)) && (i>100 && i<509)){
        
        	enc=true;
      	}
		
    	i++;
    }
  
  //reset loop var
  i=0;
  
	if(enc){
    
    	std::cout <<"OBJECT!!"<<std::endl;
  
  		while(i<rangeSize){
   
    
			if(!std::isnan(data_scan.ranges[i])){

			  rangeAng = data_scan.angle_min + i * data_scan.angle_increment;//calculo del angulo del vector

			  rangeDist = data_scan.ranges[i];//distancia al punto
				
				//si la distancia al goal es menor que a la del objeto, ignora el objeto	
			  	if(distToGoal>rangeDist){
					//obtenemos todos los puntos 
					pX = rangeDist * cos(rangeAng);
					pY = rangeDist * sin(rangeAng);

					//controla que el calculo del punto no de un numero muy alto
					if(rangeDist<0.0001)rangeDist=0.0001;

					//no tengas en cuenta objetos lejanos
					if(rangeDist<4){  
					  //sum all vectors with objects to obtain the result vector
					  vectorResX += pX/(rangeDist*rangeDist);
					  vectorResY += pY/(rangeDist*rangeDist);
					} 
				 }    

			}
        
        
          i++;
    
  		}
		
		//std::cout <<"Distancia al objeto"<<rangeDist<<std::endl;
		vectorResX = -vectorResX;
		vectorResY = -vectorResY; 
		/*
		*EL robot acelera como si no hubiera ningun objeto aunque lo hubiese
		*este parametro parace arreglarlo
		*/
		enc = false;//not sure
    
	}else{
     std::cout <<"NO OBJECT!!"<<std::endl;
		//theres no object ahead;
		
	}
  

}

//GIVEN CODE FROM HERE


void publishPath(ros::Publisher& path_pub, std::vector<geometry_msgs::PoseStamped>& plan )
{
	visualization_msgs::Marker marker;
  	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "path";
	marker.type = 4;
	marker.id = 0;
	marker.action = 0;

	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b =0;
	marker.color.a = 1;
	marker.scale.x = 0.05;
	for (unsigned i = 0; i< plan.size(); i++) {
		geometry_msgs::Point p;
		p.x = plan[i].pose.position.x;
		p.y =plan[i].pose.position.y;
		p.z = 0;
		marker.points.push_back(p);
	}
	path_pub.publish(marker);	


}


void odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	geometry_msgs::PoseStamped poseIn,poseOut;
	poseIn.pose = odom->pose.pose;
	poseIn.header = odom->header;	
	poseIn.header.stamp = ros::Time(0);
	try{
		tf_ptr->transformPose("map",poseIn,poseOut);
		poseOut.header.stamp = odom->header.stamp;
		map_pose_pub_ptr->publish(poseOut);	
	}catch(std::exception &e) {
		ROS_ERROR("%s",e.what());
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotics_challenge");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	double start_x, start_y;
	double goal_x, goal_y;
	
	//our vars
	Turtlebot robot;
	unsigned int cont_wp = 0;
	bool arrived = false;
	
	tf::TransformListener tf(ros::Duration(10));
	tf_ptr = &tf;

  	costmap_2d::Costmap2DROS cost_map("cost_map", tf);

	// (start_x, start_y) are the coordinates of the initial position of the robot in MAP frame
	pn.param<double>("start_x",start_x,3.46);
	pn.param<double>("start_y",start_y,4.62);
	
	// (goal_x, goal_y) are the coordinates of the goal in MAP frame
	pn.param<double>("goal_x",goal_x,6.0);
	pn.param<double>("goal_y",goal_y,10.0);
	
	ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("/path", 1);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1, odomReceived);
	ros::Publisher map_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/map_pose", 1);
	map_pose_pub_ptr = &map_pose_pub;

	// Start position
  	geometry_msgs::PoseStamped start;
  	start.pose.position.x = start_x;
  	start.pose.position.y = start_y;
  	start.pose.position.z = 0.0;
  	start.header.stamp = ros::Time::now();
  
  	// Goal Position
  	geometry_msgs::PoseStamped goal;
  	goal.pose.position.x = goal_x;
  	goal.pose.position.y = goal_y;
  	goal.pose.position.z = 0.0;
  	goal.header.stamp = ros::Time::now();


	std::vector<geometry_msgs::PoseStamped> plan;
  	global_planner::GlobalPlanner planner("turtle_planner", &cost_map); 
  	planner.makePlan(start, goal, plan);	

	publishPath(path_pub,plan);
	ros::Rate loop_rate(15);

  	/**
  	* Complete the code to follow the path
  	*/

  	while (ros::ok() && cont_wp<3)
  	{

		arrived = robot.command(goal.pose.position.x, goal.pose.position.y);
    
		if(arrived){
		  cont_wp++;
		  std::cout <<"GOAL REACHED!!"<<std::endl;
		}

	 	ros::spinOnce();
       	loop_rate.sleep();
  	}


	return 0;
}
