#ifndef ROBOTOVERSEER_H_
#define ROBOTOVERSEER_H_

#include <opencv2/core/core.hpp>
#include <vector>
#include <thread>

#include "RobotDetection.h"
#include "RobotControl.h"
#include "RobotObject.h"
#include "RobotCalc.h"
#include "vect2d.h"

class RobotDetection;
class RobotControl;
class RobotObject;
class RobotCalc;
class Vect2D;

class RobotOverseer {
public:
	RobotOverseer();
	~RobotOverseer();
	bool initWebcam(int cam);
	void AddRobotForOverseeing(RobotObject *rob);
	void setHSV(int LowH, int HighH, int LowS, int HighS, int LowV, int HighV);
	void startOverseerTread();
	void stopOverseerTread();
	void overseer();

	cv::Mat * getFrame();
private:
	static void startTread(RobotOverseer *ro);

	bool _threadRunning;
	std::thread overseerTread;

	RobotDetection *_robDetect;
	RobotCalc *_robCalc;

	cv::Mat _currentFrame;

	std::vector<RobotObject*> _robotList;

};

#endif /* ROBOTOVERSEER_H_ */
