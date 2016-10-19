/*
 * Market.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: andy
 */

#include "Market.h"

Market::Market(){}

void Market::init(int nAgents, int myIndex) {
	for(int i=0; i<nAgents; i++){
		rewards.push_back(0);
		costs.push_back(0);
		values.push_back(0);

		Point a(0,0);
		locations.push_back(a);
	}

	this->myIndex = myIndex;
}

Market::~Market() {}

