

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


//This is the function to be filled in C4
void processImageColor_c4(cv::Mat &in, cv::Mat &out)
{
	

	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;

	cv::Mat o[] = { b, g, r, alpha };

	cv::Mat res;
	cv::Mat aux_imageR;
	cv::Mat aux_imageB;
	cv::Mat aux_imageG;
	cv::Mat aux_image;
	
	//in is a color image. We split it into the 3 colors (in order BGR) and the alpha channel:
	cv::split(in,o);

	//From now on, we process just one of the channels. For instance, o[2] is the red channel
	cv::threshold(o[0],aux_imageB,70.0,255.0,cv::THRESH_BINARY);
	cv::threshold(o[1],aux_imageG,70.0,255.0,cv::THRESH_BINARY);
	cv::threshold(o[2],aux_imageR,120.0,255.0,cv::THRESH_BINARY);
	
	//cv::bitwise_xor(aux_imageB, aux_imageG, aux_image);
	//cv::bitwise_not(aux_image, aux_image);
	
	cv::bitwise_not(aux_imageB, aux_imageB);
	cv::bitwise_not(aux_imageG, aux_imageG);
	
	cv::Mat aux_GB;
	
	cv::bitwise_and(aux_imageG, aux_imageB, aux_GB);
	//cv::bitwise_and(aux_GB, aux_image, aux_image);
	//cv::bitwise_not(aux_image, aux_image);
	cv::bitwise_and(aux_imageR, aux_GB, res);
	
	//cv::bitwise_or(aux_GB, aux_imageR, res);
	
	cv::erode(res,res,cv::Mat());
	cv::dilate(res,res,cv::Mat());
	cv::dilate(res,res,cv::Mat());
	cv::dilate(res,res,cv::Mat());
	
	cv::erode(res,res,cv::Mat());
	cv::erode(res,res,cv::Mat());
	
	cv::dilate(res,out,cv::Mat());
	
	

	
}
