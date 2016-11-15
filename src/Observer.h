/*
 * Observer.h
 *
 *  Created on: Jul 14, 2016
 *      Author: andy
 */

#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "Costmap.h"
#include "Inference.h"
#include "Market.h"

class Observer {
public:
	Observer(Point cLoc, int nAgents, bool global, String name, int myIndex);
	virtual ~Observer();

	bool globalObserver;
	String name;

	Point cLoc;
	int nAgents;
	vector<Scalar> agentColors;

	Costmap costmap;
	Market market;
	void communicate(Costmap &cIn, Market &mIn);
	void showCellsPlot();
	void addAgentsToCostmapPlot();
	Scalar setAgentColor(int i);
	void addSelfToCostmapPlot();

	Inference inference;

	// void shareCostmap(Costmap &A, Costmap &B);
};

#endif /* OBSERVER_H_ */
