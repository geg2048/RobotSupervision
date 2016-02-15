#ifndef ROBOTOBJECT_H_
#define ROBOTOBJECT_H_

#include "vect2d.h"
#include <stdio.h>
#include <unistd.h>
#include <vector>

enum e_RobotStates {UNKNOWN, IDLE, DANGERBYROBOT, DANGERBYWALL, LOST};

class RobotObject {
public:
	RobotObject();
	RobotObject(int robId);
	RobotObject(std::vector<Vect2D> aPoints, e_RobotStates aState);
	virtual ~RobotObject();
	// Accessing the value of the PointArray
	Vect2D GetPoint(int aIndex);
	// Return true when RobotObject has Points
	bool HasPoints();
	// Accessing the value of the OldPointArray
	Vect2D GetOldPoint(int aIndex);
	// Saving the new position of the Robot
	void SetNewPoints(std::vector<Vect2D> aPoints);
	// Accessing the value of the CenterPoint
	Vect2D GetCenter();
	// Accessing the value of the OldCenterPoint
	Vect2D GetOldCenter();
	// Accessing the value of the SpeedVect
	Vect2D GetSpeed();
	// Returns a Vector of length 1 that points in the direction the Robot points to
	Vect2D GetDirVect();
	// Returns the Current Radius of the Robot
	int GetRobRad();
	// Sets Vars for turning by aGrad
	void StartRotationBy(int aGrad, bool aClockwise);
	// Check if Rotation is finished
	bool CheckIfRotationIsDone();
	// Changing the state of the Robot ( one function per state )
	void SetRobotToIDLE();
	void SetRobotToDANGERBYROBOT();
	void SetRobotToDANGERBYWALL();
	void SetRobotToLOST();
	e_RobotStates GetRobotState();
	// Returns RobotName
	int GetRobId();
	// Flag to remember if the Robot was Found again
	bool HasBeenFound = false;
	bool ReceivedNewPoints = false;
	// Counter For Lost Robot
	int LostCounter;
private:
	// Array which contains the Points the define the Position of the Robot
	std::vector<Vect2D> _PointArray;
	std::vector<Vect2D> _OldPointArray;

	// Center Point of the Robot
	Vect2D _CenterPoint;
	Vect2D _OldCenterPoint;
	// Speed Vector of the Robot
	Vect2D _SpeedVect;
	// enum that contains information about the current state of the Robot
	enum e_RobotStates _RobotState;
	// enum that contains the name of the Robot
	int _robId;
	// Calculating the new CenterPoint of the Robot
	void CalcNewCenter();
	// Calculating the new SpeedVect of the Robot
	void CalcNewSpeed();
	// Vars for Turning
	// Number of Grads to be turned
	int GradToTurn;
	// Vector From which the turning started
	Vect2D StartingVect;
};

#endif /* ROBOTOBJECT_H_ */
