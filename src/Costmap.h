/*
 * Costmap.h
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#ifndef COSTMAP_H_
#define COSTMAP_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

using namespace std;
using namespace cv;

class Costmap {
public:
	Costmap();
	virtual ~Costmap();

	void shareCostmap(Costmap &b);

	void prettyPrintCostmap();

	// useful stuff
	int obsFree, infFree, domFree, unknown, obsWall, infWall, inflatedWall;
	// 1 = free space // 2 = inferred free space // 3 = domFree
	// 101 = unknown
	// 201 = wall // 202 = inferred wall // 203 = inflated wall
	Vec3b cObsFree, cInfFree, cObsWall, cInfWall, cUnknown, cError;

	Mat cells;
	Mat euclidDist; // array of distances
	Mat occ; // floats, holds 0-1 of occupancy of a cell; 0-0.2 = obsFree, 0.2-0.35 = infFree, 0.35-0.65 = unknown, 0.65-0.8=infWall, 0.8-1.0 = obsWall
	// use inference to seed the occGrid and also for mapping, track by 0.1 per observation with p(0.01) of a misreading;
	Mat searchReward;
	Mat reward;

	vector<Point> cellUpdates;
	vector<Point> hullBreaches;
	vector<Point> cellChanges;

	void simulateObservation(Point pose, Mat &resultingView, vector<Point> observedCells, float obsRadius);
	vector<Point> viewPerim;
	void growMatIntoFreeCells( Mat &freeMat );

	void getRewardMat(float w[3], float e[2], float spread);
	void displayThermalMat(Mat &mat);

	void spreadSearchArea(float growthRate);
	void displaySearchReward();

	float getEuclidianDistance(Point a, Point b);
	void getDistGraph();

	float cumulativeAStarDist(Point sLoc, Point gLoc, Mat &cSet, Mat &oSet, vector<Point> &oVec, Mat &fScore, Mat &gScore);
	float aStarDist(Point sLoc, Point gLoc);
	vector<Point> aStarPath(Point sLoc, Point gLoc);

	// TODO only update portion that needs it
	//void updateCostmap(vector<vector<int> > cells, vector<int> value);

	//CumulativePathPlanner observerPathPlanner;
	//CumulativePathPlanner relayPathPlanner;

	Mat displayPlot;
	void buildCellsPlot(); // build nice display plot
	void buildOccPlot(); // build nice display plot
	//void showCostmapPlot(int index); // show nice display plot and number it
	void addAgentToPlot(Scalar color, vector<Point> myPath, Point cLoc);

	Mat createCostMat(); // Largely used for inference
	vector<Point> getImagePointsAt(Mat &image, int intensity); // convert mat to vector of points

	vector<int> publishCostmap();
	void subscribeCostmap( vector<int> &cm);

	/*retired functions

	float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentDominated(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentObservedAndInferred(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentObservedAndInferredCorrectly(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentInferredCorrectly(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentInferredWrongly(Costmap &globalCostmap, Costmap &workingCostmap);

	 */


};

#endif /* COSTMAP_H_ */
