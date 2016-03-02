/*
 * Gui.h
 *
 *  Created on: 01. März 2016
 *      Author: georg
 */

#ifndef GUI_H_
#define GUI_H_

#include <opencv2/opencv.hpp>
#include "RobotOverseer.h"

#define GUI_ACTIVE
#define GLOBAL_WINDOW "Camera"
#define THRESHOLD_WINDOW "Threshold"

class RobotOverseer;

extern cv::Mat GLOBAL_IMAGE;

class Gui {
	struct _HSV_FILTER{
		int iLowH;
		int iHighH;

		int iLowS;
		int iHighS;

		int iLowV;
		int iHighV;
	};

public:
	Gui(RobotOverseer *robotOverseer);
	virtual ~Gui();
	void initGUI();
	void setHSV();
	void setupHSV();

	bool isOK(bool reset = true);

	void drawRobotPolygon(std::vector<std::vector<Vect2D> > polygonList);
	void drawRobotInformation(RobotControl *rc,RobotCalc *rCalc);

private:
	static void updateHSV(int s,void *vp);
	RobotOverseer *ro;
	_HSV_FILTER _hsv;
	int _okVar;
};

#endif /* GUI_H_ */
