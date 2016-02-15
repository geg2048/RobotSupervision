#ifndef GRAPICALDEBUG_H_
#define GRAPICALDEBUG_H_ 

#define GRAPIC_DEBUG 1
#define DEBUG 1

#if GRAPIC_DEBUG

#include <opencv2/opencv.hpp>
#include "RobotDetection.h"

#define rob2str(x)  (x == 1) ? "ROB1" : \
                    (x == 2) ? "ROB2" : \
                    (x == 2) ? "ROB3" : \
                    (x == 3) ? "ROB4" : "NAME ERROR"

extern cv::Mat gIMG;

void updateHSV(int s,void *vp);
#endif
#endif
