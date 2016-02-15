#include "RobotObject.h"
#include "stdlib.h"

RobotObject::RobotObject() :
		_RobotState(UNKNOWN), _robId(0){
}

RobotObject::RobotObject(int robId) : RobotObject() {
	_robId = robId;
}

RobotObject::~RobotObject() {
}

// Returns Point of aIndex
Vect2D RobotObject::GetPoint(int aIndex) {
	return _PointArray.at(aIndex);
}
// Return true when RobotObject has Points
bool RobotObject::HasPoints() {
	return (_PointArray.size() > 0);
}
// Accessing the value of the OldPointArray
Vect2D RobotObject::GetOldPoint(int aIndex) {
	return _OldPointArray.at(aIndex);
}
// Saving the new position of the Robot
void RobotObject::SetNewPoints(std::vector<Vect2D> aPoints) {
	_OldPointArray = _PointArray;
	_PointArray = aPoints;
	CalcNewCenter();
	CalcNewSpeed();
}
// Accessing the value of the CenterPoint
Vect2D RobotObject::GetCenter() {
	return _CenterPoint;
}
// Accessing the value of the OldCenterPoint
Vect2D RobotObject::GetOldCenter() {
	return _OldCenterPoint;
}
// Accessing the value of the SppedVect
Vect2D RobotObject::GetSpeed() {
	return _SpeedVect;
}
// Returns a Vector that points in the direction the Robot points to
Vect2D RobotObject::GetDirVect() {

	std::vector<double> arr;
	arr.resize(_PointArray.size());
	int i, j, maxFound = 0;
	double maxValue = 0;

	for(i = 0; i < _PointArray.size(); i++)
	{
		double sum = 0;
		for( j = 0; j < _PointArray.size(); j++)
		{
			if( i != j)
			{
				sum += _PointArray[i].DistBetweenPoints(_PointArray[j]);
			}
		}
		arr[i] = sum;
	}

	for (i = 0; i < _PointArray.size(); i++) {
		if (arr[i] > maxValue) {
			maxValue = arr[i];
			maxFound = i;
		}
	}

	return VectBetweenPoints(GetCenter(), _PointArray[maxFound]);
}
int RobotObject::GetRobRad(){
	return this->GetDirVect().VectLength();
}
// Sets Vars for turning by aGrad
void RobotObject::StartRotationBy(int aGrad, bool aClockwise) {
	if(aGrad > 170)
	{
		GradToTurn = 170;
	}
	else
	{
		GradToTurn = aGrad;
	}
	StartingVect = GetDirVect();
	if (aClockwise) {
		//RCTurnClockwise();
	} else {
		//RCTurnCounterClockwise();
	}
}
// Check if Rotation is finished
bool RobotObject::CheckIfRotationIsDone() {
	printf("Angle Between is: %f\n", StartingVect.AngleBetweenVect_Grad(GetDirVect()));
	printf("Desired Angle: %i\n", GradToTurn);
	printf("PosX: %i PosY:%i\n",_PointArray[0].XI(),_PointArray[0].YI());
	if (std::abs(StartingVect.AngleBetweenVect_Grad(GetDirVect())) >= GradToTurn) {
		//RCForward();
		return true;
	} else {
		return false;
	}
}
// Changing the state of the Robot ( one function per state )
void RobotObject::SetRobotToIDLE() {
	_RobotState = IDLE;
}
void RobotObject::SetRobotToDANGERBYROBOT() {
	_RobotState = DANGERBYROBOT;
}
void RobotObject::SetRobotToDANGERBYWALL() {
	_RobotState = DANGERBYWALL;
}
void RobotObject::SetRobotToLOST() {
	_RobotState = LOST;
}
e_RobotStates RobotObject::GetRobotState() {
	return _RobotState;
}
// Socket Verwaltung

void RobotObject::CalcNewCenter() {
	if (_PointArray.size() == 3) {
		int SumOfXCords = 0, SumOfYCords = 0;

		for (int i = 0; i < _PointArray.size(); i++) {
			SumOfXCords += _PointArray[i].m_X;
			SumOfYCords += _PointArray[i].m_Y;
		}
		Vect2D tmp = Vect2D(SumOfXCords / 3, SumOfYCords / 3, false);
		_OldCenterPoint = _CenterPoint;
		_CenterPoint = tmp;
	}
}

void RobotObject::CalcNewSpeed() {
	_SpeedVect = VectBetweenPoints(_OldCenterPoint, _CenterPoint);
}

int RobotObject::GetRobId() {
	return _robId;
}
