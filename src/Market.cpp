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
		Point a(0,0);
		cLocs.push_back(a);
		gLocs.push_back(a);
		times.push_back(0);
		rewards.push_back(0);
		costs.push_back(0);
		values.push_back(0);
	}

	this->myIndex = myIndex;
}

void Market::iterateTime(){
	for( size_t i=0; i<this->cLocs.size(); i++){
		times[i]++;
	}
}

void Market::shareMarket( Market &vis ){

	for( size_t i=0; i<this->cLocs.size(); i++){ // for all agents
		if(this->times[i] <= vis.times[i] && i != vis.myIndex){ // have i seen them sooner than they have?
			// yes, give them all my info on them
			vis.cLocs[i] = this->cLocs[i];
			vis.gLocs[i] = this->gLocs[i];
			vis.cLocs[i] = this->cLocs[i];
			vis.costs[i] = this->costs[i];
			vis.times[i] = this->times[i];
		}
	}
}

void Market::updateMarket( Point cLoc, Point gLoc ){
	this->iterateTime();

	cLocs[myIndex] = cLoc;
	gLocs[myIndex] = gLoc;
	times[myIndex] = 0;
}

void Market::printMarket(){

	cout << "time: ";
	for( size_t i=0; i<this->times.size(); i++){
		cout << times[i];
		if(i+1 < times.size() ){
			cout << ", ";
		}
	}
	cout << endl;

	cout << "cLocs: ";
	for( size_t i=0; i<this->cLocs.size(); i++){
		cout << cLocs[i];
		if(i+1 < cLocs.size() ){
			cout << ", ";
		}
	}
	cout << endl;

	cout << "gLocs: ";
	for( size_t i=0; i<this->gLocs.size(); i++){
		cout << gLocs[i];
		if(i+1 < gLocs.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

Market::~Market() {}

