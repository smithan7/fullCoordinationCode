/*
 * Pose.h
 *
 *  Created on: Nov 2, 2016
 *      Author: andy
 */

#ifndef POSE_H_
#define POSE_H_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "math.h"

#include "Costmap.h"

using namespace std;
using namespace cv;

class Pose {
public:
	Pose(Point loc, Costmap &costmap);
	virtual ~Pose();
	void initPose(Costmap &costmap);
	void makeMat();
	void addToMat(Mat &matIn);

	Point loc;
	Mat mat;
	vector<Point> obsLim;
	vector<float> obsLen;
	vector<int> obsVal;
	bool needInference;
	float mean;
	float stanDev;

	float radius;
	int nSamples;

	void getMean();
	void getStanDev();
	float getPDF( float x );
	float getCDF( float x );

};

#endif /* POSE_H_ */
