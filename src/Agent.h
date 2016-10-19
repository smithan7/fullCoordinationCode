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

#include "World.h" // includes - costmap.h
#include "CostmapCoordination.h" // includes - world.h, frontier.h
#include "CostmapPlanning.h" // includes - world.h, frontier.h

#include "Graph.h" // includes - costmap.h, node.h
#include "GraphCoordination.h" // includes - frontier.h, TreeNode.h, Node.h
#include "GraphPlanning.h" // includes - graph.h, node.h

#include "Inference.h" // includes - frontiers.h, contours.h
#include "Market.h" // for graphCoordination

using namespace std;

class Agent{
public:

	// agent stuff
	Agent(Point sLoc, int myIndex, World &gMap, float obsThresh, float comThresh, int numAgents);
	void pickMyColor();
	~Agent();
	void shareCostmap(Costmap &A, Costmap &B);
	void showCellsPlot();
	void shareGoals(vector<int> inG, int index);
	int myIndex;
	Scalar myColor;
	float comThresh;
	float obsThresh;

	Market market;

	// working
	void soloPlan(string method, int timeLeft);
	void coordinatedPlan(string method, int timeLeft, vector<Agent> &agents);

	void act();

	Point cLoc, gLoc; // for map
	vector<Point> myPath, agentLocs;

	// costmap class stuff
	Costmap costmap;
	CostmapCoordination costmapCoordination;
	CostmapPlanning costmapPlanning;

	// graph class stuff
	//Graph travelGraph, poseGraph;

	GraphCoordination graphCoordination;
	GraphPlanning graphPlanning;
	int cNode;
	int gNode;
	vector<int> nodePath;

	// inference stuff
	Inference inference;

	int marketNodeSelect(World &gMap);
	void greedyFrontiers();
};

#endif /* SRC_Agent_H_ */
