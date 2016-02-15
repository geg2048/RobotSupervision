#include "RobotControl.h"

RobotControl::RobotControl(int robId) : RobotObject(robId) {
	// TODO Auto-generated constructor stub
	_RobSocket = -1;
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

Vect2D RobotControl::getCurrentTargetPoint() {
	return _tagetPoints.at(_currentTargetPoint);
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
	switch (GetRobotState()) {
		case IDLE:
			folowPoints();
			break;
		case DANGERBYROBOT:
		case DANGERBYWALL:
			turnRobot();
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

	this->sendCMD(&motor, sizeof(char));
	this->sendCMD(&speed, sizeof(float));
}

void RobotControl::stopRobot(){
	setMotors(0,0);
}

void RobotControl::turnRobot(){
	setMotors(-0.5,0.5);
}

void RobotControl::folowPoints(){

}






