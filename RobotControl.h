#ifndef ROBOTCONTROL_H_
#define ROBOTCONTROL_H_

#include <vector>
#include <stdio.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#include "RobotObject.h"

class RobotObject;

class RobotControl : RobotObject{
public:
    RobotControl(int robId);
	virtual ~RobotControl();

	void setMotorLeft(float left);
	void setMotorRight(float right);

	void setMotors(float left,float right);

	Vect2D getCurrentTargetPoint();

	// Socket Verwaltung
	bool OpenSocket(char *aMac);
	void CloseSocket();

	void controlRobot();

private:
	void sendCMD(void *cmd,size_t size);
	void sendMotorCmd(char motor,float speed);
    void folowPoints();
    void turnRobot();
    void stopRobot();

    // TODO: move to robojekt
	std::vector<Vect2D> _tagetPoints;
	size_t _currentTargetPoint;

	// Socket for Bluethoot
	int _RobSocket;
};

#endif /* ROBOTCONTROL_H_ */
