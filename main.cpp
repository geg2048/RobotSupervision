#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "GrapicalDebug.h"

#include "RobotDetection.h"
#include "RobotOverseer.h"
#include "RobotObject.h"
#include "RobotControl.h"

int main(int argc, char** argv)
{
	RobotOverseer *ro = new RobotOverseer();

	RobotObject *r1 = new RobotControl(1);
	RobotObject *r2 = new RobotControl(2);
	RobotObject *r3 = new RobotControl(3);
	RobotObject *r4 = new RobotControl(4);

	ro->initWebcam(0);

	ro->AddRobotForOverseeing(r1);
	ro->AddRobotForOverseeing(r2);
	ro->AddRobotForOverseeing(r3);
	ro->AddRobotForOverseeing(r4);

	ro->startOverseerTread();

	ro->stopOverseerTread();

	printf("end");
}

/*
int main(int argc, char** argv)
{
#if GRAPIC_DEBUG
        
        int ok = 80;
        
        gRobDec = &rb;
        HSV_FILTER hsv;
        
        hsv.iLowH = 55;
        hsv.iHighH = 75;

        hsv.iLowS = 100;
        hsv.iHighS = 255;
        
        hsv.iLowV = 100;
        hsv.iHighV = 255;
        
        rb.setHSV(hsv.iLowH, hsv.iHighH, hsv.iLowS, hsv.iHighS, hsv.iLowV,hsv.iHighV);
        
        cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
        cv::namedWindow("Contures", CV_WINDOW_NORMAL);
        cv::namedWindow("HSV", CV_WINDOW_NORMAL);

        cv::startWindowThread();

        //Create trackbars in "Control" window
        cv::createTrackbar("LowH", "Control", &(hsv.iLowH), 179,updateHSV,&hsv); //Hue (0 - 179)
        cv::createTrackbar("HighH", "Control", &(hsv.iHighH), 179,updateHSV,&hsv);

        cv::createTrackbar("LowS", "Control", &(hsv.iLowS), 255,updateHSV,&hsv); //Saturation (0 - 255)
        cv::createTrackbar("HighS", "Control", &(hsv.iHighS), 255,updateHSV,&hsv);

        cv::createTrackbar("LowV", "Control", &(hsv.iLowV), 255,updateHSV,&hsv); //Value (0 - 255)
        cv::createTrackbar("HighV", "Control", &(hsv.iHighV), 255,updateHSV,&hsv);
        
        cv::createTrackbar(">90 OK","Control",&ok,100);

        //check hsv
        cv::Mat img;
        while(ok < 90){
            rb.readFrame(img);
            cv::imshow("Contures",img);
            rb.applyHsvFilter(img,img);
            cv::imshow("HSV",img);
        }
        
#endif
        


	r1->OpenSocket("30:14:11:26:01:34"); // Bertle_03
	r2->OpenSocket("30:14:08:26:27:65"); // Bertle_11
//	r3->OpenSocket("98:D3:31:30:84:9C"); // Bertle_13
//	r4->OpenSocket("98:D3:34:90:62:36"); // Bertle_12

	//TODO: Handle bt error

	ro.AddRobotForOverseeing(r1);
	ro.AddRobotForOverseeing(r2);
//	ro.AddRobotForOverseeing(r3);
//	ro.AddRobotForOverseeing(r4);

	ro.DetektRobots(rb);

	rb.startDetection(ro);

	delete r1;
	 delete r2;
	// delete r3;
	// delete r4;

    return 0;
}


*/
