

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


//This is the function to be filled in C3
void processImage_c3(cv::Mat &in, cv::Mat &out)
{
	double k[3][3]={{-1,-2,-1},{0,0,0},{1,2,1}};

	cv::Mat kernel = cv::Mat(3,3,CV_32F,k);

	//cv::filter2D(in,out,-1,kernel);

	//cv::Sobel(in,out,-1,1,1);

	//cv::threshold(in,out,140.0,255.0,cv::THRESH_TOZERO_INV);
	cv::threshold(in,out,200.0,255.0,cv::THRESH_BINARY);

	//cv::erode(out,out,cv::Mat());
	//cv::dilate(out,out,cv::Mat());

	
}


//This is the function to be filled in C4
void processImageColor_c4(cv::Mat &in, cv::Mat &out)
{

	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;

	cv::Mat o[] = { b, g, r, alpha };

	cv::Mat aux_image;
	cv::Mat aux_imageR;
	cv::Mat aux_imageB;
	cv::Mat aux_imageG;
	
	//in is a color image. We split it into the 3 colors (in order BGR) and the alpha channel:
	cv::split(in,o);

	//From now on, we process just one of the channels. For instance, o[2] is the red channel
	cv::threshold(o[0],aux_imageB,150.0,255.0,cv::THRESH_BINARY);
	cv::threshold(o[1],aux_imageG,150.0,255.0,cv::THRESH_BINARY);
	cv::threshold(o[2],aux_imageR,100.0,255.0,cv::THRESH_BINARY);
	
	//cv::bitwise_or(aux_imageB, aux_imageR, aux_imageB);
	//cv::bitwise_or(aux_imageB, aux_imageG, aux_imageB);
	
	cv::bitwise_xor(aux_imageR, aux_imageG, aux_image);
	
	cv::bitwise_or(aux_imageG, aux_imageR, aux_image);
	cv::bitwise_not(aux_image, aux_image);
	
	cv::bitwise_and(aux_imageB, aux_image, aux_image);
	
	cv::dilate(aux_image,aux_image,cv::Mat());
	cv::erode(aux_image,out,cv::Mat());
	
	//This is an example:
	//cv::filter2D(o[0],out,-1,cv::threshold(in,o[2],200.0,255.0,cv::THRESH_BINARY));

	//cv::filter2D(o[1],out,-1,cv::getGaussianKernel(5,2));
	
	

	
}
