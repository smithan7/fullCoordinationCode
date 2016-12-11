/*
 * Market.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: andy
 */

#include "Market.h"

void printVector(vector<int> v);
void printVector(vector<Point> v);
void printVector(vector<float> v);

Market::Market(){}

void Market::init(int nAgents, int myIndex, bool enableRelaySacrifice) {

	for(int i=0; i<nAgents+1; i++){
		Point a(0,0);

		// explore stuff
		cLocs.push_back(a);
		gLocs.push_back(a);
		exploreCosts.push_back(0);

		// report stuff
		reportTimes.push_back(1);
		reportCosts.push_back(0);
		reportRequests.push_back(0);

		// relay / sacrifice stuff
		rLocs.push_back(a);
		roles.push_back(-1); // -1 = n/a, 0 = sacrifice, 1 = relay
		mates.push_back(-1); // -1 = n/a, # = index of corresponding relay sacrifice

		// general stuff
		times.push_back(0); // how long since I have heard from each person
	}

	this->myIndex = myIndex;
	this->nAgents = nAgents;
}

void Market::iterateTime(){
	for( int i=0; i<nAgents+1; i++){
		times[i]++;
	}
}

bool Market::comCheck(int a){
	if(times[a] <= 1 && a != myIndex){ // am I in contact with them currently?
		return true;
	}
	return false;
}

vector<int> Market::assembleTransmission(){
	// turn the 1d vector into standard market

	vector<int> transmission;
	for(int i=0; i<nAgents+1; i++){ // +1 is to account for observer
		transmission.push_back( cLocs[i].x ); // 0
		transmission.push_back( cLocs[i].y); // 1
		transmission.push_back( gLocs[i].x); // 2
		transmission.push_back( gLocs[i].y); // 3
		transmission.push_back( int(exploreCosts[i] * 100) ); // 4

		transmission.push_back( reportTimes[i]); // 5
		transmission.push_back( int(reportCosts[i] * 100) ); // 6
		transmission.push_back( reportRequests[i]); // 7

		transmission.push_back( rLocs[i].x); // 8
		transmission.push_back( rLocs[i].y); // 9
		transmission.push_back( roles[i]); // 10
		transmission.push_back( mates[i]); // 11

		transmission.push_back( times[i] ); // 12
	}

	return transmission;
}

void Market::dissasembleTransmission(vector<int> &transmission){

	// turn the 1d vector into a standard market
	int transmissionLength = 13;
	for(int i=0; i<nAgents+1; i++){ // +1 is to account for observer
		if( transmission[i*transmissionLength+12] < times[i] && i != myIndex ){ // only update market if the info is new and not me
			cLocs[i].x = transmission[i*transmissionLength];
			cLocs[i].y = transmission[i*transmissionLength+1];

			gLocs[i].x = transmission[i*transmissionLength+2];
			gLocs[i].y = transmission[i*transmissionLength+3];
			exploreCosts[i] = float(transmission[i*transmissionLength+4]) / 100;

			reportTimes[i] = transmission[i*transmissionLength+5];
			reportCosts[i] = float(transmission[i*transmissionLength+6]) / 100;
			reportRequests[i] = transmission[i*transmissionLength+7];

			rLocs[i].x = transmission[i*transmissionLength+8];
			rLocs[i].y = transmission[i*transmissionLength+9];
			roles[i] = transmission[i*transmissionLength+10];
			mates[i] = transmission[i*transmissionLength+11];

			times[i] = transmission[i*transmissionLength+12]+1; // add 1 for my hop
		}

		if( i == myIndex){ // if it is me check return and relay / sacrfice
			reportRequests[i] = transmission[i*transmissionLength+7]; // check for requests

			if(transmission[i*transmissionLength+10]){ // check if someone has made me a relay / sacrifice
				rLocs[i].x = transmission[i*transmissionLength+8];
				rLocs[i].y = transmission[i*transmissionLength+9];
				roles[i] = transmission[i*transmissionLength+10];
				mates[i] = transmission[i*transmissionLength+11];
			}
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

	cout << "cLocs: ";
	printVector(cLocs);

	cout << "gLocs: ";
	printVector( gLocs );

	cout << "exploreCosts: ";
	printVector(exploreCosts);

	cout << "reportTimes: ";
	printVector(reportTimes);

	cout << "reportCosts: ";
	printVector(reportCosts);

	cout << "reportRequests: ";
	printVector(reportRequests);

	cout << "rLocs: ";
	printVector( rLocs );

	cout << "roles: ";
	printVector(roles);

	cout << "mates: ";
	printVector(mates);

	cout << "times: ";
	printVector(times);

}

void printVector(vector<int> v){
	for( size_t i=0; i<v.size(); i++){
		cout << v[i];
		if(i+1 < v.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

void printVector(vector<Point> v){
	for( size_t i=0; i<v.size(); i++){
		cout << v[i];
		if(i+1 < v.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

void printVector(vector<float> v){
	for( size_t i=0; i<v.size(); i++){
		cout << v[i];
		if(i+1 < v.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

Market::~Market() {}

