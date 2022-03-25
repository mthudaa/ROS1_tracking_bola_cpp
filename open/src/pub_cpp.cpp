#include <iostream>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "pub_node");
    ros::NodeHandle n;
    ros::Publisher xyz = n.advertise<geometry_msgs::Point>("xyz", 10);
    ros::Rate loop(10);
    
    cv::Mat img, hsv, mask, blur;
    cv::VideoCapture cap(0);
    
    int h_low = 0, s_low = 127, v_low = 140;
    int h_hi = 26, s_hi = 255, v_hi = 255;
    
    cv::namedWindow("slider");
    cv::createTrackbar("hue low : ", "slider", &h_low, 255);
    cv::createTrackbar("sat low : ", "slider", &s_low, 255);
    cv::createTrackbar("val low : ", "slider", &v_low, 255);
    cv::createTrackbar("hue hi : ", "slider", &h_hi, 255);
    cv::createTrackbar("sat hi : ", "slider", &s_hi, 255);
    cv::createTrackbar("val hi : ", "slider", &v_hi, 255);
    
    while(ros::ok){
        geometry_msgs::Point koordinat;
        cv::Scalar low(h_low, s_low, v_low);
	cv::Scalar hi(h_hi, s_hi, v_hi);
	cap.read(img);
	cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
	cv::inRange(hsv, low, hi, mask);
	cv::Canny(mask, mask, 30, 50, 3);
	cv::GaussianBlur(mask, blur, cv::Size(11, 11), 0);
		
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(blur, circles, cv::HOUGH_GRADIENT, 1.25 , 100, 100, 30, 1, 500);

	for (size_t i = 0; i < circles.size(); i++) {
		cv::Vec3i c = circles[i];
		cv::Point cntr = cv::Point(c[0], c[1]);
		cv::circle(img, cntr, 5, cv::Scalar(0, 0, 255), 7, cv::LINE_8);
		cv::circle(img, cntr, c[2], cv::Scalar(0, 255, 0), 3, cv::LINE_8);
		int cx,cy;
		cv::Size sz = img.size();
		cx = c[0]-(sz.width/2);
		cy = (sz.height/2)-c[1];
		int alfa;
		if(cx>0 && cy>0){
			alfa = (180/3.14)*atan(cy/cx);
		}
		else if(cx<0 && cy<0){
			alfa = (180/3.14)*atan(cy/cx)+180;
		}
		if(cx<0 && cy>0){
			alfa = (180/3.14)*atan(cy/cx);
		}
		else if(cx>0 && cy<0){
			alfa = (180/3.14)*atan(cy/cx)+180;
		}
        	koordinat.x = cx;
        	koordinat.y = cy;
        	koordinat.z = alfa;
	}
		
	cv::imshow("image ori", img);
	cv::imshow("image edge", mask);
	cv::waitKey(1);
        
        xyz.publish(koordinat);
        ros::spinOnce();
        loop.sleep();
    }
    
    return 0;
}
