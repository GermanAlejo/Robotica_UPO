


#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "student.h"

#define MIN_M00 100000.0

    
  void imageCb(const sensor_msgs::ImageConstPtr & msg)
  {
    cv::Mat rgb_frame; //Input image in matrix form
    cv::Mat out_frame; //Output image

	  
	
  //	std::cout <<"procesando imagen" <<std::endl;
	  
    cv_bridge::CvImagePtr cv_ptr_rgb; //cv_bridge::CvImagePtr is a pointer type that points to NULL by default. You have to allocate storage before you can actually use it
    try
    {
      cv_ptr_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //make a copy of the ROS message data. //bgra8: BGR color image with an alpha channel
                                                         //Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    rgb_frame = cv_ptr_rgb->image; //Here we have the current frame in OpenCV Mat format
	
    processImageColor_c4(rgb_frame, out_frame);
   	
	 //circulo
  // 2. Usando Moments se obtiene el centro de la imagen
  cv::Moments moment = moments(out_frame, false);
  float x = moment.m10 / moment.m00;
  float y = moment.m01 / moment.m00;
  // 3. Draw a circle at that position if the book is in the frame (counting m00)
  if (moment.m00 > MIN_M00) {
    cv::circle(rgb_frame,cv::Point(x,y),10,cv::Scalar(0,0,255),3);
    cv::ellipse(rgb_frame, cv::Point(x, y), cv::Size(70,40), 360.0, 0.0, 360.0, cv::Scalar(0,0,255), 3);

  }
	  /*
	  	double colorMsg;
	  
	  	colorMsg->header.stamp.sec; //(seconds)
		
	    std::cout<<"Seconds"<<colorMsg<<std::endl;;
	  	colorMsg->header.stamp.nsec; //(nanoseconds)
	  
	     std::cout<<"nanoseconds"<<colorMsg<<std::endl;;
	  */
	  
		cv::imshow("input_image", rgb_frame); //Show a window with the input image
		cv::imshow("output_image", out_frame); //Show a window with the output image
		cv::waitKey(3); //Wait for 3 milliseconds
  }
    
 

int main(int argc, char** argv)
{
	  ros::init(argc, argv, "cha2"); 

	  ros::NodeHandle nh_;

		//std::cout <<"procesando imagen" <<std::endl;

	  //image_transport::ImageTransport it_(nh_);

	  ros::Subscriber image_sub_;


	  image_sub_ = nh_.subscribe("/camera/rgb/image_rect_color", 1, imageCb); //subscribes to the Kinect video frames
		//ros::Subscriber sub = nh_.subscribe("/camera/rgb/image_raw", 1, imageCb); //subscribes to the Kinect video frames

	  ros::spin();
	  return 0;
}
