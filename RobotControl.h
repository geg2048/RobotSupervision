#ifndef ROBOTCONTROL_H_
#define ROBOTCONTROL_H_

#include <vector>
#include <stdio.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <thread>
#include <mutex>

#include "RobotObject.h"

class RobotObject;

class RobotControl : public RobotObject {
public:
    RobotControl(int robId);
	virtual ~RobotControl();

	void setMotorLeft(float left);
	void setMotorRight(float right);

	void setMotors(float left,float right);
	void setMotors();

	Vect2D getCurrentTargetPoint();

	void keepAlive();

	// Socket Verwaltung
	bool OpenSocket(char *aMac);
	void CloseSocket();

	void controlRobot();
protected:
	void sendCMD(void *cmd,size_t size);
	void sendMotorCmd(char motor,float speed);
    void folowPoints();
    void turnRobot(bool clockwise = true);
    void stopRobot();

    float _motorLeftSpeed;
    float _motorRightSpeed;

    void setNextTargetPoint();
	std::vector<Vect2D> _tagetPoints;
	size_t _currentTargetPoint;
	int _tagetPointTreshold;

	const float _defaultSpeed;
	const float _speedMultiplier;

	std::mutex sendCmdMurtex;

private:
	// Socket for Bluethoot
	int _RobSocket;
};

#endif /* ROBOTCONTROL_H_ */
