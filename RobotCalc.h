#ifndef ROBOTCALC_H_
#define ROBOTCALC_H_

#include <vector>

#include "RobotControl.h"
#include "vect2d.h"

class RobotObject;
class Vect2D;

class RobotCalc {
public:
	Vect2D X_AXIS;
	Vect2D Y_AXIS;

	RobotCalc(const int fieldHight = 480, const int fieldWidth = 960,
			const int maxDistToOld = 30, const int maxDistToLost = 30, const int botRad = 40);

	virtual ~RobotCalc();

	void TrackRobots(std::vector<std::vector<Vect2D> > newPoints);
	void setRobotList(std::vector<RobotControl*> *robotList);
	void StateManager();

	int getMaxDistToLost();
	int getMaxDistToOld();
	int getBotRad();
	int getFieldHight();
	int getFieldWidth();

private:
	std::vector<RobotControl*> *_robotList;

	const int MAXDISTTOOLD;
	const int MAXDISTTOLOST;
	const int BOT_RAD;
	const int FIELD_HEIGTH;
	const int FIELD_WIDTH;

	bool DangerFromRobots(RobotControl *aRob);
	bool DangerFromBorder(RobotControl *aRob);
	void ReleaseSafeBots();
	bool CheckForClearPath(RobotControl *aRob1, RobotControl *aRob2);
	void CheckIfLost();
};

#endif /* ROBOTCALC_H_ */
