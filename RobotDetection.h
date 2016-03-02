#ifndef SRC_ROBOTDETECTION_H_
#define SRC_ROBOTDETECTION_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include "vect2d.h"
#include "RobotOverseer.h"

class RobotDetection;
class RobotOverseer;
class RobotDetection {
public:
	RobotDetection();
	virtual ~RobotDetection();
	bool initCapture(int device);
	bool startDetection(RobotOverseer &ro);
	void setHSV(int LowH, int HighH, int LowS, int HighS, int LowV, int HighV);
	bool readFrame(cv::Mat &img);
	void getFrameSize(int &width, int &hight);
	void calcPolygons(cv::Mat inputImg,
			std::vector<std::vector<Vect2D> > &polygonList);
	void applyHsvFilter(cv::Mat inputImg, cv::Mat &outputImg,
			const bool doMorph = true, const int morphSize = 3);
	bool getCaptureStatus();

private:
	bool _captureStatus;
	cv::VideoCapture _cap;
	cv::Scalar _lowHSV;
	cv::Scalar _hightHSV;

};

#endif /* SRC_ROBOTDETECTION_H_ */
