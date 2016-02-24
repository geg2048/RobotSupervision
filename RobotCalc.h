#ifndef ROBOTCALC_H_
#define ROBOTCALC_H_

#include <vector>

#include "RobotControl.h"
#include "vect2d.h"

class RobotObject;
class Vect2D;

class RobotCalc {
public:
	RobotCalc();
	virtual ~RobotCalc();

    void TrackRobots(std::vector<std::vector<Vect2D> > newPoints);
    void setRobotList(std::vector<RobotControl*> *robotList);
    void StateManager();

private:
    std::vector<RobotControl*> *_robotList;

};

#endif /* ROBOTCALC_H_ */
