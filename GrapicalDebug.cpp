#include "GrapicalDebug.h"

#if GRAPIC_DEBUG

RobotDetection *gRobDec;
cv::Mat gIMG;

void updateHSV(int s,void *vp){
	HSV_FILTER hsv = *((HSV_FILTER*)(vp));
	gRobDec->setHSV(hsv.iLowH, hsv.iHighH, hsv.iLowS, hsv.iHighS, hsv.iLowV,hsv.iHighV);
}

#endif