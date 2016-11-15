/*
 * GraphNode.h
 *
 *  Created on: Sep 16, 2016
 *      Author: andy
 */

#ifndef GRAPHNODE_H_
#define GRAPHNODE_H_


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


class GraphNode {
public:
	GraphNode(Point l);
	virtual ~GraphNode();

	Point loc;
	float reward;
	Mat observation;
	vector<int> nbrs;
	vector<float> transitions;
	bool updateView;

};

#endif /* GRAPHNODE_H_ */
