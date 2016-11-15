/*
 * Market.h
 *
 *  Created on: Sep 27, 2016
 *      Author: andy
 */

#ifndef MARKET_H_
#define MARKET_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

using namespace cv;
using namespace std;

class Market {
public:
	Market();
	void init(int nAgents, int myIndex);
	void shareMarket( Market &in );
	void updateMarket( Point cLoc, Point gLoc );
	void iterateTime();
	void printMarket();
	virtual ~Market();

	vector<Point> cLocs;
	vector<Point> gLocs;
	vector<float> values;
	vector<float> rewards;
	vector<float> costs;
	vector<int> times;
	vector<float> batteryLeft;

	int myIndex;

};

#endif /* MARKET_H_ */
