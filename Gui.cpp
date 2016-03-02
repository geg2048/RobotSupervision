/*
 * Gui.cpp
 *
 *  Created on: 01. März 2016
 *      Author: georg
 */

#include "Gui.h"
#include <string>

cv::Mat GLOBAL_IMAGE;

Gui::Gui(RobotOverseer *robotOverseer) {

	ro = robotOverseer;

	_okVar = 0;

	_hsv.iLowH = 55;
	_hsv.iHighH = 85;

	_hsv.iLowS = 100;
	_hsv.iHighS = 255;

	_hsv.iLowV = 100;
	_hsv.iHighV = 255;

}

Gui::~Gui() {
	// TODO Auto-generated destructor stub
}

void Gui::initGUI() {
	cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	cv::namedWindow(THRESHOLD_WINDOW, CV_WINDOW_NORMAL);
	cv::namedWindow(GLOBAL_WINDOW, CV_WINDOW_NORMAL);

	cv::startWindowThread();

	//Create trackbars in "Control" window
	cv::createTrackbar("LowH", "Control", &(_hsv.iLowH), 179, updateHSV, this); //Hue (0 - 179)
	cv::createTrackbar("HighH", "Control", &(_hsv.iHighH), 179, updateHSV,
			this);

	cv::createTrackbar("LowS", "Control", &(_hsv.iLowS), 255, updateHSV, this); //Saturation (0 - 255)
	cv::createTrackbar("HighS", "Control", &(_hsv.iHighS), 255, updateHSV,
			this);

	cv::createTrackbar("LowV", "Control", &(_hsv.iLowV), 255, updateHSV, this); //Value (0 - 255)
	cv::createTrackbar("HighV", "Control", &(_hsv.iHighV), 255, updateHSV,
			this);

	cv::createTrackbar(">10 OK", "Control", &_okVar, 20);

	updateHSV(0,this);
}

bool Gui::isOK(bool reset) {
	if (_okVar > 10) {
		if (reset) {
			_okVar = 0;
			cv::setTrackbarPos(">10 OK", "Control", 0);
		}
		return true;
	}
	return false;
}

void Gui::setHSV() {
	ro->setHSV(_hsv.iLowH, _hsv.iHighH, _hsv.iLowS, _hsv.iHighS, _hsv.iLowV,
			_hsv.iHighV);
}

void Gui::updateHSV(int s, void *vp) {
	Gui *gui = (Gui*) (vp);
	gui->setHSV();
}

void Gui::drawRobotPolygon(std::vector<std::vector<Vect2D> > polygonList) {
	for (size_t a = 0; a < polygonList.size(); a++) {
		Vect2D alt = polygonList[a][0];
		Vect2D tmp;
		for (size_t b = 0; b < polygonList[a].size(); b++) {
			tmp = polygonList[a][b];
			cv::line(GLOBAL_IMAGE, alt.getCvPoint(), tmp.getCvPoint(),
					cv::Scalar(0, 255, 0));
			alt = tmp;
		}
	}
}

void Gui::drawRobotInformation(RobotControl *rc,RobotCalc *rCalc){
	e_RobotStates state = rc->GetRobotState();
	cv::Point center = rc->GetCenter().getCvPoint();

	std::string name = "Robot: ";
	name.append(std::to_string(rc->GetRobId()));

	cv::putText(GLOBAL_IMAGE,name,center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0));

	switch (state) {
		case DANGERBYROBOT:
		case DANGERBYWALL:
		case LOST:
			cv::circle(GLOBAL_IMAGE,center,rCalc->getBotRad(),cv::Scalar(0,0,255));
			break;
		default:
			break;
	}
}



