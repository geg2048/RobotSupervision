#include "RobotDetection.h"

RobotDetection::RobotDetection() :
		_cap(), _lowHSV(0,0,0), _hightHSV(0,0,0), _captureStatus(false) {
}

RobotDetection::~RobotDetection() {
	_captureStatus = false;
	_cap.release();
}

bool RobotDetection::initCapture(int device) {
	if (_cap.open(device)) {
		_captureStatus = true;
	} else {
		std::cerr << "Cannot open the WebCam" << std::endl;
		_captureStatus = false;
	}
	return _captureStatus;
}

bool RobotDetection::getCaptureStatus() {
	return _captureStatus;
}

void RobotDetection::setHSV(int LowH, int HighH, int LowS, int HighS, int LowV,
		int HighV) {
	_lowHSV = cv::Scalar(LowH, LowS, LowV);
	_hightHSV = cv::Scalar(HighH, HighS, HighV);
}

bool RobotDetection::readFrame(cv::Mat &img) {
	bool bSuccess;

	if(_captureStatus){
		bSuccess = _cap.read(img); // read a new frame from video
	}else {
		std::cerr << "VideoCaputure is not initalisiert" << std::endl;
		return _captureStatus;
	}
	if (!bSuccess) {
		std::cerr << "Cannot read a frame from video stream" << std::endl;
	}

	return bSuccess;
}

void RobotDetection::applyHsvFilter(cv::Mat inputImg, cv::Mat &outputImg,
		const bool doMorph, const int morphSize) {
	cv::Size morphSizeCV(morphSize, morphSize);

	cv::cvtColor(inputImg, inputImg, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	inRange(inputImg, _lowHSV, _hightHSV, outputImg); //Threshold the image

	for (int i = 0; (i < 4) && doMorph; ++i) {
		//0+1 morphological opening (removes small objects from the foreground)
		//4+3 morphological closing (removes small holes from the foreground)

		switch (i) {
		case 0:
		case 3:
			cv::erode(outputImg, outputImg,
					cv::getStructuringElement(cv::MORPH_ELLIPSE, morphSizeCV));
			break;
		case 1:
		case 2:
			cv::dilate(outputImg, outputImg,
					cv::getStructuringElement(cv::MORPH_ELLIPSE, morphSizeCV));
			break;
		}
	}
}

void RobotDetection::calcPolygons(const cv::Mat inputImg,
		std::vector<std::vector<Vect2D> > &polygonList) {

	std::vector<std::vector<cv::Point> > contours; // std::vector for storing contour
	std::vector<std::vector<cv::Point> > polygons;
	polygonList.clear();

	//Finde die conturen
	cv::findContours(inputImg.clone(), contours, CV_RETR_LIST,
			CV_CHAIN_APPROX_SIMPLE);

	polygons.resize(contours.size());

	for (size_t k = 0; k < contours.size(); k++) {
		//Detect closed polygons
		cv::approxPolyDP(cv::Mat(contours[k]), polygons[k],
				cv::arcLength(cv::Mat(contours[k]), true) * 0.06, true);
	}

	for (size_t a = 0; a < polygons.size(); ++a) {
		size_t tmpS = polygons[a].size();
		std::vector<Vect2D> tmp;
		tmp.resize(tmpS);

		for (size_t b = 0; b < tmpS; ++b) {
			tmp[b].m_X = polygons[a][b].x;
			tmp[b].m_Y = polygons[a][b].y;
		}

		polygonList.push_back(tmp);
	}
}

void RobotDetection::getFrameSize(int &w, int &h){
	w = (int)(_cap.get(CV_CAP_PROP_FRAME_WIDTH));
	h = (int)(_cap.get(CV_CAP_PROP_FRAME_HEIGHT));
}
