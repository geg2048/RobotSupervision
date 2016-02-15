#include "RobotCalc.h"

RobotCalc::RobotCalc() {
	// TODO Auto-generated constructor stub
	_robotList = NULL;

}

RobotCalc::~RobotCalc() {
	// TODO Auto-generated destructor stub
}

void RobotCalc::setRobotList(std::vector<RobotObject*> *robotList){
	_robotList = robotList;
}

void RobotCalc::TrackRobots(std::vector<std::vector<Vect2D> > newPoints) {
}

void RobotCalc::StateManager(){
}
