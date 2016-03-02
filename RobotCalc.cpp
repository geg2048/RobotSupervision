#include "RobotCalc.h"

RobotCalc::RobotCalc(const int fieldHight, const int fieldWidth,
		const int maxDistToOld, const int maxDistToLost, const int botRad) :
		FIELD_HEIGTH(fieldHight), FIELD_WIDTH(fieldWidth), MAXDISTTOOLD(
				maxDistToOld), MAXDISTTOLOST(maxDistToLost), BOT_RAD(botRad) {
	// TODO Auto-generated constructor stub
	_robotList = NULL;
	printf("robcalc\n");
}

RobotCalc::~RobotCalc() {
	// TODO Auto-generated destructor stub
}

void RobotCalc::setRobotList(std::vector<RobotControl*> *robotList) {
	_robotList = robotList;
}

void RobotCalc::TrackRobots(std::vector<std::vector<Vect2D> > newPoints) {
	if (newPoints.size() > 0) {
		std::vector<bool> usedPoly;
		usedPoly.resize(newPoints.size());
		for (int i = 0; i < newPoints.size(); i++) {
			usedPoly[i] = false;
		}
		for (int a = 0; a < _robotList->size(); a++) {
			if (_robotList->at(a)->HasPoints()
					&& _robotList->at(a)->GetRobotState() != LOST
					&& _robotList->at(a)->GetRobotState() != UNKNOWN)
				for (int b = 0; b < 3; b++) {
					int sDist =
							_robotList->at(a)->GetPoint(b).DistBetweenPoints(
									newPoints[0][0]);

					int tmp;
					int cF = 0;
					for (int c = 1; c < newPoints.size(); c++) {
						tmp = _robotList->at(a)->GetPoint(b).DistBetweenPoints(
								newPoints[c][0]);
						if (tmp < sDist) {
							sDist = tmp;
							cF = c;
						}
					}
					if (sDist <= MAXDISTTOOLD) {
						_robotList->at(a)->SetNewPoints(newPoints[cF]);
						usedPoly[cF] = true;
						_robotList->at(a)->ReceivedNewPoints = true;
						_robotList->at(a)->LostCounter = 0;
					} else {
						_robotList->at(a)->ReceivedNewPoints = false;
					}
				}
		}
		for (int a = 0; a < _robotList->size(); a++) {
			if ((_robotList->at(a)->GetRobotState() == LOST
					|| !_robotList->at(a)->ReceivedNewPoints)
					&& _robotList->at(a)->GetRobotState() != UNKNOWN) {
				for (int b = 0; b < usedPoly.size(); b++) {
					if (!usedPoly[b]) {
						if (newPoints[b][0].DistBetweenPoints(
								_robotList->at(a)->GetPoint(0))
								<= MAXDISTTOLOST) {
							_robotList->at(a)->SetNewPoints(newPoints[b]);
							usedPoly[b] = true;
							_robotList->at(a)->ReceivedNewPoints = true;
							_robotList->at(a)->HasBeenFound = true;
							_robotList->at(a)->LostCounter = 0;
						}
					}
				}
			}
		}
	}

	CheckIfLost();
}

void RobotCalc::CheckIfLost() {
	for (int a = 0; a < _robotList->size(); a++) {
		if (_robotList->at(a)->GetRobotState() != UNKNOWN
				&& _robotList->at(a)->GetRobotState() != LOST)
			if (_robotList->at(a)->LostCounter >= 30) {
				_robotList->at(a)->SetRobotToLOST();
			} else {
				_robotList->at(a)->LostCounter++;
			}
	}
}

void RobotCalc::StateManager() {
	for (int i = 0; i < _robotList->size(); i++) {
		RobotControl *rob = _robotList->at(i);
		switch (rob->GetRobotState()) {
		case IDLE:
			if (DangerFromRobots(rob)) {
				rob->SetRobotToDANGERBYROBOT();
			} else if (DangerFromBorder(rob)) {
				rob->SetRobotToDANGERBYWALL();
			}
			break;
		case DANGERBYROBOT:
			if (!DangerFromRobots(rob)) {
				rob->SetRobotToIDLE();
			}
			break;
		case DANGERBYWALL:
			if (rob->CheckIfRotationIsDone()) {
				rob->SetRobotToIDLE();
			}
			break;
		case LOST:
			if (rob->HasBeenFound) {
				rob->SetRobotToIDLE();
				rob->LostCounter = 0;
				rob->HasBeenFound = false;
			} else {
				rob->RCHalt();
			}
			break;
		default:
			break;
		}
	}
}

bool RobotCalc::DangerFromRobots(RobotControl *aRob) {

	if (_robotList->size() == 1) {
		aRob->RCForward();
		return false;
	}

	for (int j = 0; j < _robotList->size(); j++) {
		if (aRob->GetRobId() != _robotList->at(j)->GetRobId()) {
			switch(CheckForClearPath(aRob, _robotList->at(j)))
			{
			case 0:
				aRob->RCForward();
				return false;
			case 1:
				aRob->RCTurnCounterClockwise();
				return true;
			case 2:
			case 3:
			case 4:
				aRob->RCTurnClockwise();
				return true;
			default: aRob->RCHalt(); break;
			}
		}
	}
}


int AdjustAngle(int aAngle)
{
	if (aAngle > 30) {
		return 2 * aAngle;
	} else {
		return aAngle + 60;
	}
}

bool RobotCalc::DangerFromBorder(RobotControl *aRob) {
	if (aRob->GetCenter().m_X <= BOT_RAD) {
		if (aRob->GetDirVect().m_X < 0) {
			volatile double angle = std::abs(
					aRob->GetDirVect().AngleBetweenVect_Grad(Y_AXIS));
			double turnangle = AdjustAngle(angle);
			if (aRob->GetDirVect().m_Y >= 0) {
				aRob->StartRotationBy(turnangle, false);
				return true;
			} else {
				aRob->StartRotationBy(turnangle, true);
				return true;
			}
		}
	} else if (aRob->GetCenter().m_X >= FIELD_WIDTH - BOT_RAD) {
		if (aRob->GetDirVect().m_X > 0) {
			volatile double angle = std::abs(
					aRob->GetDirVect().AngleBetweenVect_Grad(Y_AXIS));
			double turnangle = AdjustAngle(angle);
			if (aRob->GetDirVect().m_Y >= 0) {
				aRob->StartRotationBy(turnangle, true);
				return true;
			} else {
				aRob->StartRotationBy(turnangle, false);
				return true;
			}
		}
	} else if (aRob->GetCenter().m_Y <= BOT_RAD) {
		if (aRob->GetDirVect().m_Y < 0) {
			volatile double angle = std::abs(
					aRob->GetDirVect().AngleBetweenVect_Grad(X_AXIS));
			double turnangle = AdjustAngle(angle);
			if (aRob->GetDirVect().m_X >= 0) {
				aRob->StartRotationBy(turnangle, true);
				return true;
			} else {
				aRob->StartRotationBy(turnangle, false);
				return true;
			}
		}
	} else if (aRob->GetCenter().m_Y >= FIELD_HEIGTH - BOT_RAD) {
		if (aRob->GetDirVect().m_Y > 0) {
			volatile double angle = std::abs(
					aRob->GetDirVect().AngleBetweenVect_Grad(X_AXIS));
			double turnangle = AdjustAngle(angle);
			if (aRob->GetDirVect().m_X >= 0) {
				aRob->StartRotationBy(turnangle, false);
				return true;
			} else {
				aRob->StartRotationBy(turnangle, true);
				return true;
			}
		}
	}
	return false;
}

void RobotCalc::ReleaseSafeBots() {
	for (int i = 0; i < _robotList->size(); i++) {
		RobotControl *rob = _robotList->at(i);
		switch (rob->GetRobotState()) {
		case DANGERBYWALL:
		case DANGERBYROBOT: {
			bool IsNotSafe = false;
			for (int j = 0; j < _robotList->size(); j++) {
				if (i != j) {
					if (rob->GetCenter().DistBetweenPoints(
							_robotList->at(j)->GetCenter()) <= BOT_RAD * 2) {
						if (!CheckForClearPath(rob, _robotList->at(j))) {
							IsNotSafe = true;
						}
					}
				}
			}
			if (!IsNotSafe) {
				rob->SetRobotToIDLE();
			}
			break;
		}
		default:
			break;
		}
	}
}

bool RobotCalc::CheckForClearPath(RobotControl *aRob1, RobotControl *aRob2) {
	Vect2D VectBetween = VectBetweenPoints(aRob1->GetCenter(), aRob2->GetCenter());

	if (VectBetween.VectLength() <= aRob1->GetRobRad() * 1.5)
		return 4; // Roboter ist zu nahe

	if (VectBetween.VectLength() > aRob1->GetRobRad() * 5 )
		return 0; // Roboter ist zu weit weg

	double AngleBetween = aRob1->GetDirVect().AngleBetweenVect(VectBetween);
	double DistToAB2 = 0;

	if ((180 / PI) * abs(AngleBetween) >= 90)
		return 0; // Roboter ist nicht im richtígen Winkel

	DistToAB2 = sin(AngleBetween) * VectBetween.VectLength();

	if (abs(DistToAB2) <= aRob1->GetRobRad() * 1.5)
	{
		if (AngleBetween > 0)
			return 1; // Roboter ist Rechts
		else if (AngleBetween < 0)
			return 2; // Roboter ist Links
		else
			return 3; // Roboter ist direkt voraus
	}
	else
	{
		return 0;
	}

}

int RobotCalc::getMaxDistToLost(){
	return MAXDISTTOLOST;
}

int RobotCalc::getMaxDistToOld(){
	return MAXDISTTOOLD;
}

int RobotCalc::getBotRad(){
	return BOT_RAD;
}

int RobotCalc::getFieldHight(){
	return FIELD_HEIGTH;
}
int RobotCalc::getFieldWidth(){
	return FIELD_WIDTH;
}



