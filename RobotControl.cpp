#include "RobotControl.h"
#include <stdlib.h>

RobotControl::RobotControl(int robId) : RobotObject(robId) , _defaultSpeed(0.5), _speedMultiplier(0.01) {
	// TODO Auto-generated constructor stub
	_RobSocket = -1;
	_motorLeftSpeed = _defaultSpeed;
	_motorRightSpeed = _defaultSpeed;
	_currentTargetPoint = 0;
}

RobotControl::~RobotControl() {
	// TODO Auto-generated destructor stub
}

void RobotControl::setMotorLeft(float left) {
	char leftCmd = 1;
	sendMotorCmd(leftCmd,left);
}

void RobotControl::setMotorRight(float right) {
	char rightCmd = 2;
	sendMotorCmd(rightCmd,right);
}

void RobotControl::setMotors(float left, float right) {
	setMotorLeft(left);
	setMotorRight(right);
}

void RobotControl::setMotors(){
	setMotorLeft(_motorLeftSpeed);
	setMotorRight(_motorRightSpeed);
}

Vect2D RobotControl::getCurrentTargetPoint() {
	return _tagetPoints.at(_currentTargetPoint);
}

void RobotControl::setNextTargetPoint(){
	if(_tagetPoints.size() >= _currentTargetPoint + 1){
		_currentTargetPoint = 0;
	} else {
		_currentTargetPoint++;
	}
}

bool RobotControl::OpenSocket(char *aMac) {
	struct sockaddr_rc addr = { 0 };
	int status;

	// allocate a socket
	_RobSocket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	if (_RobSocket < 0) {
		perror("no Socket");
		return false;
	} else {
		printf("RobS:%d\n", _RobSocket);
	}

	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba(aMac, &addr.rc_bdaddr);

	// connect to server
	status = connect(_RobSocket, (struct sockaddr *) &addr, sizeof(addr));

	if (status < 0) {
		perror("uh oh");
		return false;
	} else {
		printf("status:%d\n", status);
		return true;
	}
}
void RobotControl::CloseSocket() {
	if (_RobSocket >= 0) {
		close(_RobSocket);
	}
}

void RobotControl::sendCMD(void *cmd, size_t size) {
	if (_RobSocket >= 0) {
		write(_RobSocket, &cmd, size);
	}
}

void RobotControl::controlRobot(){
	switch (controlCmd) {
		case FORWARD:
			if(_tagetPoints.empty()){
				setMotors();
			} else {
				folowPoints();
			}
			break;
		case TURNCLOCKWISE:
			turnRobot(true);
			break;
		case TURNCOUNTERCLOCKWISE:
			turnRobot(false);
		break;
		default:
			stopRobot();
			break;
	}
}

void RobotControl::sendMotorCmd(char motor,float speed){
	if (speed > 1.0) {
		speed = 1;
	} else if (speed < -1.0) {
		speed = -1;
	}

	//printf("ROB%d: motor:%d,speed:%f\n",GetRobId(),motor,speed);
	std::lock_guard<std::mutex> lock(sendCmdMurtex);

	this->sendCMD(&motor, sizeof(char));
	this->sendCMD(&speed, sizeof(float));
}

void RobotControl::stopRobot(){
	setMotors(0,0);
}

void RobotControl::turnRobot(bool clockwise){
	if (clockwise) {
		setMotors(_defaultSpeed,-_defaultSpeed);
	} else {
		setMotors(-_defaultSpeed,_defaultSpeed);
	}
}

void RobotControl::folowPoints(){
	Vect2D tagetPoint = getCurrentTargetPoint();
	Vect2D currentPos = this->GetDirVect();

	double dist = tagetPoint.DistBetweenPoints(currentPos);

	if(dist < _tagetPointTreshold){
		setNextTargetPoint();
		return;
	}

	double angle = currentPos.AngleBetweenVect_Grad(tagetPoint);

	if(std::abs(angle) > 90){
		this->StartRotationBy(angle,(bool)(angle > 0));
	}
	else if(angle < 0){
		_motorRightSpeed = -angle * _speedMultiplier + _defaultSpeed;
	} else {
		_motorLeftSpeed = angle * _speedMultiplier + _defaultSpeed;
	}
}

void RobotControl::keepAlive(){
	std::lock_guard<std::mutex> lock(sendCmdMurtex);
	//printf("ROB%d: KEEP_ALIVE\n",GetRobId());
	char c = 0;
	sendCMD(&c,sizeof(char));
}

void RobotControl::setTargetPoints(std::vector<Vect2D> points){
	_tagetPoints = points;
}
