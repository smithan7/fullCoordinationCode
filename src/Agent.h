/*
 * Agent.h
 *
 *  Created on: Jun 8, 2016
 *      Author: andy
 */
/*
 * Agent.h
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */

#ifndef SRC_Agent_H_
#define SRC_Agent_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include "CostmapCoordination.h" // includes - world.h, frontier.h
#include "CostmapPlanning.h" // includes - world.h, frontier.h

#include "Graph.h" // includes - costmap.h, node.h
#include "GraphCoordination.h" // includes - frontier.h, TreeNode.h, Node.h
#include "GraphPlanning.h" // includes - graph.h, node.h

#include "Inference.h" // includes - frontiers.h, contours.h
#include "Market.h" // for graphCoordination

#include "Observer.h"

using namespace std;

class Agent{
public:

	// agent stuff
	Agent(Point sLoc, int myIndex, float batteryInit, float obsThresh, float comThresh, int numAgents, int reportInterval);
	void pickMyColor();
	~Agent();

	void showCellsPlot();
	int myIndex;
	Scalar myColor;
	float comThresh;
	float obsThresh;

	Market market;

	// working
	void communicate(Costmap &cIn, Market &mIn);
	void infer(string inferenceMethod, World &World);
	void plan(string planMethod);
	void planExplore(string planMethod);
	void act(); // make move

	bool checkReportTime();
	bool lastReport(); // is my battery almost dead and I shouldn't report to do last bit of exploring
	Point reportToOperator();
	float getDistanceToReportToOperator(); // report means coms with
	int reportInterval; // how often I am supposed to report
	int reportTime; // next time I'm supposed to report
	int reportCntr;  //use this to cnt time, no need to check the report time if time < this, then reset iter with new dist, check A star only a few times
	bool reportFlag;

	bool checkReturnTime();
	Point returnToOperator();
	void marketReturnInfo(Agent &a);
	bool checkForExplorationFinished();
	float getDistanceToReturnToOperator(); // return means travel to
	float batteryLeft; // counts down
	int returnTime; // counts up with occasional reductions based on travel distance
	bool returnFlag; // time to return

	void marketRelaySacrifice(Agent &a);
	Point reportToRelayPt();
	Point returnToRelayPt();
	float getDistanceToReportToRelayPt(); // report means coms with
	float getDistanceToReturnToRelayPt(); // return means travel to
	float getDistanceToReportToOperatorFromRelayPt();
	float getDistanceToReturnToOperatorFromRelayPt();
	bool relayFlag, sacrificeFlag;
	vector<int> sacrificeList;
	int myRelay;

	Point cLoc, gLoc, oLoc, rLoc; // for map
	vector<Point> myPath, agentLocs;

	// costmap class stuff
	Costmap costmap;
	CostmapCoordination costmapCoordination;
	CostmapPlanning costmapPlanning;

	GraphCoordination graphCoordination;
	GraphPlanning graphPlanning;
	int cNode;
	int gNode;
	vector<int> nodePath;

	// inference stuff
	Inference inference;
	void greedyFrontiers();

	// retired functions
	// 	void shareGoals(vector<int> inG, int index);
	// 	void shareCostmap(Costmap &A, Costmap &B);
	// void soloPlan(string method, int timeLeft);
	// int marketNodeSelect(World &gMap);
	// graph class stuff
	// Graph travelGraph, poseGraph;
};

#endif /* SRC_Agent_H_ */
