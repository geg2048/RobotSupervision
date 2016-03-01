#include "RobotOverseer.h"

#include <chrono>
#include "Gui.h"

RobotOverseer::RobotOverseer() : _threadRunning(false) {
	_robDetect = new RobotDetection();
	_robCalc = new RobotCalc();
}

RobotOverseer::~RobotOverseer(){
	stopOverseerTread();

	delete _robDetect;
	delete _robCalc;
}

bool RobotOverseer::initWebcam(int cam){
	return _robDetect->initCapture(cam);
}

void RobotOverseer::AddRobotForOverseeing(RobotControl *rob){
	_robotList.push_back(rob);
	_robCalc->setRobotList(&_robotList);
}

void RobotOverseer::setHSV(int LowH, int HighH, int LowS, int HighS, int LowV, int HighV){
	_robDetect->setHSV(LowH,HighH,LowS,HighS,LowV,HighV);
}

void RobotOverseer::overseer(){
	cv::Mat img;
	std::vector<std::vector<Vect2D> > polygonList;

	while(_threadRunning){
		if(!_robDetect->readFrame(img)){
			continue;
		}
		_currentFrame = img.clone();
#ifdef GUI_ACTIVE
		GLOBAL_IMAGE = img.clone();
#endif

		_robDetect->applyHsvFilter(img,img);
		_robDetect->calcPolygons(img,polygonList);

		_robCalc->TrackRobots(polygonList);
		_robCalc->StateManager();

		for (size_t i = 0; i < _robotList.size(); ++i) {
			_robotList[i]->controlRobot();
		}

#ifdef GUI_ACTIVE
		cv::imshow(GLOBAL_WINDOW,GLOBAL_IMAGE);
#endif
	}
}

cv::Mat RobotOverseer::getThresholdedImage(){
	cv::Mat img;
	_robDetect->readFrame(img);
	_robDetect->applyHsvFilter(img,img);
	return img.clone();
}

void RobotOverseer::keepRobotsAlive(){
	while(_threadRunning){
		for (size_t i = 0; i < _robotList.size(); ++i) {
			_robotList[i]->keepAlive();
		}
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

void RobotOverseer::startOverseerTread(){
	_threadRunning = true;
 	overseerTread = std::thread(startTread,this);
 	keepAliveTread = std::thread(startKeepAliveTread,this);
}



void RobotOverseer::stopOverseerTread(){
	if(_threadRunning){
		_threadRunning = false;
		overseerTread.join();
		keepAliveTread.join();
	}
}

void RobotOverseer::startTread(RobotOverseer *ro){
	ro->overseer();
}

void RobotOverseer::startKeepAliveTread(RobotOverseer *ro){
	ro->keepRobotsAlive();
}

