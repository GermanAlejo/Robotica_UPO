
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/LaserScan.h>

#include <stdio.h>

#include <math.h>
#include <vector>
#include <fstream>

// Representation (RVIZ)
#include <visualization_msgs/Marker.h>

  float tempX, tempY;
	float vectorResX = 0;
	float vectorResY = 0;

	float dist=0;//distancia al goal
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




bool Turtlebot::command(double gx, double gy)  
{

	double linear_vel=0.0;
	double angular_vel=0.0;
  	float ang = 0.0;
	bool ret_val = false;

  	std::cout <<"llamando al command" <<std::endl;
	//Transform the goal to the local frame
	geometry_msgs::PointStamped goal;
	geometry_msgs::PointStamped base_goal;
	
  	goal.header.frame_id = "odom";

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
  
  
  std::cout <<"VECTORRESX "<<vectorResX<<std::endl;
  std::cout <<"VECTORRESY "<<vectorResY<<std::endl;
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
	
		angular_vel = 0.12*ang;//velocidad angular controlada en funcion del angulo
  		linear_vel = 0.09*dist;//velocidad linear controlada en funcion de la distancia al goal
		
	//no object
	}else{
		
		angular_vel = 0.2*ang;
		linear_vel = 0.3*dist;
	
	}
	
 
  
  if(angular_vel > 0.8){
    angular_vel = 0.8;
  }
  
  if(linear_vel > 1){
    linear_vel = 1;
  }
  

  
  //calculate the module of base_goal to know distand towards objective
  dist = sqrt(pow(base_goal.point.x, 2) + pow(base_goal.point.y, 2));
  std::cout <<"DISTANCIA AL GOAL"<<dist<<std::endl;
  //goal reached
	if(dist < 0.5){
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
    std::cout <<"Range Size"<<rangeSize<<std::endl;
  	
    vectorResX = 0;
    vectorResY = 0;

  //busca obstaculos
	while(i<rangeSize && !enc){

		//no tengas en cuenta objetos que no esten al frente de esta forma no realentizaran el robot
    	if(!std::isnan(data_scan.ranges.at(i)) && (i>160 && i<449)){
        
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
			 // if(dist!=0 && rangeDist>dist){
				//obtenemos todos los puntos 
				pX = rangeDist * cos(rangeAng);
				pY = rangeDist * sin(rangeAng);

				//controla que el calculo del punto no de un numero muy alto
				if(rangeDist<0.1)rangeDist=0.1;

				//no tengas en cuenta objetos lejanos
				if(rangeDist<3){  
				  //sum all vectors with objects to obtain the result vector
				  vectorResX += pX/(rangeDist*rangeDist);
				  vectorResY += pY/(rangeDist*rangeDist);
				} 
		   //  }    

			}
        
        
          i++;
    
  		}
		
		std::cout <<"Distancia al objeto"<<rangeDist<<std::endl;
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
	



void visualizePlan(const std::vector<geometry_msgs::Pose> &plan, ros::Publisher &marker_pub );

std::vector<geometry_msgs::Pose> loadPlan(const char *filename);

//this is main fuction
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_control");
  Turtlebot robot;
  ros::NodeHandle n;
  bool aux = false;
  int xGoal, yGoal;
  
  if(argc<2)
  {
    std::cout << "Insufficient number of parameters" << std::endl;
    std::cout << "Usage: robot_control <filename>" << std::endl;
    return 0;
  }

  std::cout <<"Prueba"<< argv[1] <<std::endl;
  std::vector<geometry_msgs::Pose> plan = loadPlan(argv[1]);
  
  //std::cout <<"Prueba despues"<< argv[1] <<std::endl;
  unsigned int cont_wp = 0;

  ros::Rate loop_rate(20);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 4);

  /**
  * Complete the code to follow the path,
  * calling adequately to the command function
  */

  while (ros::ok() && cont_wp<3)
  {
    
      xGoal = plan[cont_wp].position.x;
      yGoal = plan[cont_wp].position.y;
    
      aux = robot.command(xGoal,yGoal);
    
    if(aux){
      cont_wp++;
      aux = false;
      std::cout <<"GOAL REACHED!!"<<std::endl;
    }
    ros::spinOnce();
    
    visualizePlan(plan, marker_pub);
    
    loop_rate.sleep();
  }

  return 0;

}

  std::vector<geometry_msgs::Pose> loadPlan(const char *filename) {
  std::vector<geometry_msgs::Pose> plan;
  double x,y;
  
  std::ifstream is(filename);
  
  while (is.good()) {
    is >> x;
    if (is.good()) {
      is >> y;
      geometry_msgs::Pose curr_way;
      curr_way.position.x = x;
      curr_way.position.y = y;
      plan.push_back(curr_way);
      ROS_INFO("Loaded waypoint (%f, %f).", x , y);
    }
  }
  ROS_INFO("Plan loaded successfully.");
  return plan;
}


void visualizePlan(const std::vector< geometry_msgs::Pose >& plan, ros::Publisher &marker_pub )
{
  ros::NodeHandle n;
  
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  for (unsigned int i = 0; i < plan.size(); i++) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/odom";  // This is the default fixed frame in order to show the move of the robot
    marker.header.stamp = ros::Time::now();
  
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "plan";
    marker.id = i;
  
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
  
    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
  
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = plan[i].position.x;
    marker.pose.position.y = plan[i].position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
  
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
  
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
  
    marker.lifetime = ros::Duration(); // Eternal marker
  
    // Publish the marker
    marker_pub.publish(marker);
  }
}










