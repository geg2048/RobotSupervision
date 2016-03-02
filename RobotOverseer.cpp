#include "RobotOverseer.h"

#include <chrono>
#include "Gui.h"

RobotOverseer::RobotOverseer(int cam) :
		_threadRunning(false) {

	_robDetect = new RobotDetection();
	if (!_robDetect->initCapture(cam)) {
		return;
	}
	int w = 0, h = 0;
	_robDetect->getFrameSize(w, h);
	_robCalc = new RobotCalc(h, w);
}

RobotOverseer::~RobotOverseer() {
	stopOverseerTread();

	delete _robDetect;
	delete _robCalc;
}

void RobotOverseer::AddRobotForOverseeing(RobotControl *rob) {
	_robotList.push_back(rob);
	_robCalc->setRobotList(&_robotList);
}

void RobotOverseer::setHSV(int LowH, int HighH, int LowS, int HighS, int LowV,
		int HighV) {
	_robDetect->setHSV(LowH, HighH, LowS, HighS, LowV, HighV);
}

void RobotOverseer::overseer() {
	cv::Mat img;
	std::vector<std::vector<Vect2D> > polygonList;

	while (_threadRunning) {
		if (!_robDetect->readFrame(img)) {
			continue;
		}

		GLOBAL_IMAGE = img.clone();

		_robDetect->applyHsvFilter(img, img);
		_robDetect->calcPolygons(img, polygonList);

		_robCalc->TrackRobots(polygonList);
		_robCalc->StateManager();

		for (size_t i = 0; i < _robotList.size(); ++i) {
			_robotList.at(i)->controlRobot();
			if (gui != NULL) {
				gui->drawRobotInformation(_robotList.at(i), _robCalc);
			}
		}
		if (gui != NULL) {
			gui->drawRobotPolygon(polygonList);
			cv::imshow(GLOBAL_WINDOW, GLOBAL_IMAGE);
		}
	}
}

cv::Mat RobotOverseer::getThresholdedImage() {
	cv::Mat img;
	_robDetect->readFrame(img);
	_robDetect->applyHsvFilter(img, img);
	return img.clone();
}

void RobotOverseer::keepRobotsAlive() {
	while (_threadRunning) {
		for (size_t i = 0; i < _robotList.size(); ++i) {
			_robotList[i]->keepAlive();
		}
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

void RobotOverseer::startOverseerTread() {
	_threadRunning = true;
	overseerTread = std::thread(startTread, this);
	keepAliveTread = std::thread(startKeepAliveTread, this);
}

void RobotOverseer::stopOverseerTread() {
	if (_threadRunning) {
		_threadRunning = false;
		overseerTread.join();
		keepAliveTread.join();
	}
}

void RobotOverseer::startTread(RobotOverseer *ro) {
	ro->overseer();
}

void RobotOverseer::startKeepAliveTread(RobotOverseer *ro) {
	ro->keepRobotsAlive();
}

void RobotOverseer::addGUI(Gui *gui) {
	this->gui = gui;
}

void RobotOverseer::assingeRobot(std::vector<Vect2D> robotPoints, int robID) {
	for (size_t j = 0; j < _robotList.size(); ++j) {
		if (_robotList[j]->GetRobId() == robID) {
			_robotList[j]->SetNewPoints(robotPoints);
			_robotList[j]->SetRobotToIDLE();
		}
	}
}

void RobotOverseer::initRobotPos() {
	cv::Mat img;
	std::vector<std::vector<Vect2D> > robotPointList;
	_robDetect->readFrame(img);
	_robDetect->applyHsvFilter(img, img);
	_robDetect->calcPolygons(img, robotPointList);

	int ROB1 = 1;
	int ROB2 = 2;
	int ROB3 = 3;
	int ROB4 = 4;

	int j;
	for (int i = 0; i < robotPointList.size(); i++) {
		if (robotPointList[i][0].m_X < _robCalc->getFieldWidth() / 2) {
			if (robotPointList[i][0].m_Y < _robCalc->getFieldHight() / 2) {
				//Bottom Left Robot
				assingeRobot(robotPointList[i], ROB3);
			} else {
				//Top Left Robot
				assingeRobot(robotPointList[i], ROB1);
			}
		} else {
			if (robotPointList[i][0].m_Y < _robCalc->getFieldHight() / 2) {
				//Bottom Rigth Robot
				assingeRobot(robotPointList[i], ROB4);
			} else {
				//Top Rigth Robot
				assingeRobot(robotPointList[i], ROB2);
			}
		}
	}
}
