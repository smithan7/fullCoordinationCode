/*
 * Market.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: andy
 */

#include "Market.h"

Market::Market(){}

void Market::init(int nAgents, int myIndex) {

	transmissionLength = 13;

	for(int i=0; i<nAgents; i++){
		Point a(0,0);

		// explore stuff
		cLocs.push_back(a);
		gLocs.push_back(a);
		exploreCosts.push_back(0);

		// report stuff
		reportTimes.push_back(-1);
		reportCosts.push_back(0);
		reportRequests.push_back(0);

		// relay / sacrifice stuff
		rLocs.push_back(a);
		roles.push_back(-1); // -1 = n/a, 0 = sacrifice, 1 = relay
		mates.push_back(-1); // -1 = n/a, # = index of corresponding relay sacrifice

		// general stuff
		times.push_back(0); // how long since I have heard from each person

		for(int j=0; j<transmissionLength; j++){
			transmission.push_back(-1); // the transmitted form for ROS publisher
		}
	}

	this->myIndex = myIndex;
	this->nAgents = nAgents;

}

void Market::iterateTime(){
	for( size_t i=0; i<this->cLocs.size(); i++){
		times[i]++;
	}
}

bool Market::comCheck(int a){
	if(times[a] == 0 && a != myIndex){ // am I in contact with them currently?
		return true;
	}
	return false;
}

void Market::marketRelaySacrifice(){
	for(int a = 0; a<nAgents; a++){
		if( comCheck(a) ){
			if( roles[a] == -1){ // and they're not already a relay or sacrifice

				if( myIndex > a ){
					roles[myIndex] = 1; // I'll be a relay, they'll be a sacrifice
					roles[a] = 0;

					rLocs[myIndex] = cLocs[a];
					rLocs[a] = cLocs[a];

					mates[myIndex] = a;
					mates[a] = myIndex;
				}
				else{
					roles[myIndex] = 0; // I'll be a relay, they'll be a sacrifice
					roles[a] = 1;

					rLocs[myIndex] = cLocs[myIndex];
					rLocs[a] = cLocs[myIndex];

					mates[myIndex] = a;
					mates[a] = myIndex;
				}
			}
		}
	}
}

void Market::marketReturnInfo(){

	for( int a = 0; a<nAgents; a++){
		if( comCheck(a) ){ // am I in contact with them currently?
			if( reportCosts[myIndex] < reportCosts[a] ){ // am I closer to observer?

				if( reportTimes[myIndex] < reportTimes[a] ){ // if I have longer until I report
					reportTimes[myIndex] = reportTimes[a]; // I get their report time
				}
				reportRequests[a] = -1; // they reset their report time
			}
		}
	}
}


void Market::assembleTransmission(){
	// turn the 1d vector into standard market

	for(int i=0; i<nAgents; i++){
		transmission[i*nAgents] = cLocs[i].x;
		transmission[i*nAgents+1] = cLocs[i].y;
		transmission[i*nAgents+2] = gLocs[i].x;
		transmission[i*nAgents+3] = gLocs[i].y;
		transmission[i*nAgents+4] = exploreCosts[i] * 100;

		transmission[i*nAgents+5] = reportTimes[i];
		transmission[i*nAgents+6] = reportCosts[i] * 100;

		transmission[i*nAgents+7] = rLocs[i].x;
		transmission[i*nAgents+8] = rLocs[i].y;
		transmission[i*nAgents+9] = roles[i];
		transmission[i*nAgents+10] = mates[i];

		transmission[i*nAgents+11] = times[i];
	}
}

void Market::dissasembleTransmission(){
	// turn the 1d vector into a standard market

	for(int i=0; i<nAgents; i++){
		cLocs[i].x = transmission[i*nAgents];
		cLocs[i].y = transmission[i*nAgents+1];
		gLocs[i].x = transmission[i*nAgents+2];
		gLocs[i].y = transmission[i*nAgents+3];
		exploreCosts[i] = transmission[i*nAgents+4] / 100;

		reportTimes[i] = transmission[i*nAgents+5];
		reportCosts[i] = transmission[i*nAgents+6] / 100;

		rLocs[i].x = transmission[i*nAgents+7];
		rLocs[i].y = transmission[i*nAgents+8];
		roles[i] = transmission[i*nAgents+9];
		mates[i] = transmission[i*nAgents+10];

		times[i] = transmission[i*nAgents+11];
	}
}


void Market::shareMarket( Market &vis ){
	for( size_t i=0; i<this->cLocs.size(); i++){ // for all agents
		if(this->times[i] <= vis.times[i] && i != vis.myIndex){ // have i seen them sooner than they have?
			// explore info
			vis.cLocs[i] = this->cLocs[i];
			vis.gLocs[i] = this->gLocs[i];
			vis.exploreCosts[i] = this->exploreCosts[i];

			// return info
			vis.reportTimes[i] = this->reportTimes[i];
			vis.reportCosts[i] = this->reportCosts[i];

			// relay info
			vis.rLocs[i] = this->rLocs[i];
			vis.roles[i] = this->roles[i];
			vis.mates[i] = this->mates[i];

			// general info
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

