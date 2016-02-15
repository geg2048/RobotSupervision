#include "RobotOverseer.h"

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

void RobotOverseer::AddRobotForOverseeing(RobotObject *rob){
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

		_robDetect->applyHsvFilter(img,img);
		_robDetect->calcPolygons(img,polygonList);

		_robCalc->TrackRobots(polygonList);
		_robCalc->StateManager();
	}
}

void RobotOverseer::startOverseerTread(){
	_threadRunning = true;
 	overseerTread = std::thread(startTread,this);
}



void RobotOverseer::stopOverseerTread(){
	if(_threadRunning){
		_threadRunning = false;
		overseerTread.join();
	}
}

void RobotOverseer::startTread(RobotOverseer *ro){
	ro->overseer();
}

