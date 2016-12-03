/*
 * Agent.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */

using namespace std;

#include "Agent.h"

Agent::Agent(Point sLoc, int myIndex, float batteryInit, float obsThresh, float comThresh, int numAgents, int reportInterval){
	this->obsThresh = obsThresh;
	this->comThresh = comThresh;

	cLoc= sLoc;
	gLoc = sLoc;
	oLoc = sLoc;

	gNode = -1;
	cNode = 0;

	this->myIndex = myIndex;
	this->pickMyColor();

	for(int i=0; i<numAgents; i++){
		agentLocs.push_back(sLoc);
	}
	market.init(numAgents, myIndex );
	graphCoordination.init(obsThresh, comThresh);

	reportFlag = false;
	this->reportInterval = reportInterval;
	reportCntr = 0;

	returnFlag = false;
	this->batteryLeft = batteryInit;
	returnTime = -1;

	relayFlag = false;
	sacrificeFlag = false;
}

Point Agent::reportToOperator(){

	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( oLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
		}
	}

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(cLoc.x-comPts[i].x,2) + pow(cLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}

	if(mindex >= 0){
		while( true ){ // find closest
			if( trueDists[mindex]){
				return comPts[mindex];
			}
			comDists[mindex] = costmap.aStarDist(cLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return oLoc;
}

Point Agent::returnToOperator(){
	return oLoc;
}

float Agent::getDistanceToReturnToOperator(){
	return costmap.aStarDist(cLoc, oLoc);
}

float Agent::getDistanceToReportToOperator(){

	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( oLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
			//circle(tComs, graphCoordination.thinGraph.nodeLocations[i], 2, Scalar(127), -1, 8);
		}
	}

	//circle(tComs, cLoc, 3, Scalar(0), -1, 8);
	//circle(tComs, cLoc, 1, Scalar(255), -1, 8);

	//namedWindow("tComs", WINDOW_NORMAL);
	//imshow("tComs", tComs);
	//waitKey(0);

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(cLoc.x-comPts[i].x,2) + pow(cLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}


	if(mindex >= 0){
		while( true ){ // find closest
			//circle(tComs, comPts[mindex], 2, Scalar(127), -1, 8);
			//circle(tComs, comPts[mindex], 1, Scalar(0), -1, 8);
			//cout << "comDist[mindex] = " << comDists[mindex] << endl;

			//namedWindow("tComs", WINDOW_NORMAL);
			//imshow("tComs", tComs);
			//waitKey(0);

			if( trueDists[mindex]){
				return comDists[mindex];
			}
			comDists[mindex] = costmap.aStarDist(cLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return costmap.aStarDist(cLoc, oLoc);
}

float Agent::getDistanceToReportToRelayPt(){
	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( rLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
			//circle(tComs, graphCoordination.thinGraph.nodeLocations[i], 2, Scalar(127), -1, 8);
		}
	}

	//circle(tComs, cLoc, 3, Scalar(0), -1, 8);
	//circle(tComs, cLoc, 1, Scalar(255), -1, 8);

	//namedWindow("tComs", WINDOW_NORMAL);
	//imshow("tComs", tComs);
	//waitKey(0);

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(cLoc.x-comPts[i].x,2) + pow(cLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}


	if(mindex >= 0){
		while( true ){ // find closest
			//circle(tComs, comPts[mindex], 2, Scalar(127), -1, 8);
			//circle(tComs, comPts[mindex], 1, Scalar(0), -1, 8);
			//cout << "comDist[mindex] = " << comDists[mindex] << endl;

			//namedWindow("tComs", WINDOW_NORMAL);
			//imshow("tComs", tComs);
			//waitKey(0);

			if( trueDists[mindex]){
				return comDists[mindex];
			}
			comDists[mindex] = costmap.aStarDist(cLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return costmap.aStarDist(cLoc, rLoc);
}

float Agent::getDistanceToReturnToRelayPt(){
	return costmap.aStarDist(cLoc, rLoc);
}

float Agent::getDistanceToReportToOperatorFromRelayPt(){
	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( oLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
			//circle(tComs, graphCoordination.thinGraph.nodeLocations[i], 2, Scalar(127), -1, 8);
		}
	}

	//circle(tComs, cLoc, 3, Scalar(0), -1, 8);
	//circle(tComs, cLoc, 1, Scalar(255), -1, 8);

	//namedWindow("tComs", WINDOW_NORMAL);
	//imshow("tComs", tComs);
	//waitKey(0);

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(rLoc.x-comPts[i].x,2) + pow(rLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}


	if(mindex >= 0){
		while( true ){ // find closest
			//circle(tComs, comPts[mindex], 2, Scalar(127), -1, 8);
			//circle(tComs, comPts[mindex], 1, Scalar(0), -1, 8);
			//cout << "comDist[mindex] = " << comDists[mindex] << endl;

			//namedWindow("tComs", WINDOW_NORMAL);
			//imshow("tComs", tComs);
			//waitKey(0);

			if( trueDists[mindex]){
				return comDists[mindex];
			}
			comDists[mindex] = costmap.aStarDist(rLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return costmap.aStarDist(rLoc, oLoc);
}

float Agent::getDistanceToReturnToOperatorFromRelayPt(){
	return costmap.aStarDist(oLoc, rLoc);
}

void Agent::communicate(Costmap &cIn, Market &mIn){
	this->costmap.shareCostmap(cIn);
	this->market.shareMarket( mIn );
}

bool Agent::checkReportTime(){

	if( lastReport() ){ // would this be my last time to report?
		return false;
	}

	if( market.reportRequests[market.myIndex] == -1){
		market.reportRequests[market.myIndex] = 0;
		market.reportTimes = reportInterval;
		reportCntr = 0;
		reportFlag = false;
	}

	reportCntr++; // tracks me moving away from observer
	market.reportTimes[market.myIndex]--; // tracks time elapsing until I have to report
	if(reportCntr >= reportTime){ // has enough time gone by? if yes check distance to update time
		// may not have travelled in a straight line away from operator location
		float dR = getDistanceToReportToOperator();
		market.reportCosts[market.myIndex] = dR;

		if( dR >= market.reportTimes[market.myIndex]){
			reportFlag = true;
			return true;
		}
		else{
			reportCntr = market.reportTimes[market.myIndex] - dR;
			reportFlag = false;
			return false;
		}
	}
	else{
		reportFlag = false;
		return false;
	}
}

bool Agent::lastReport(){

	// if 1.5 * report interval < return time left then this is the last report, disable reporting
	if( batteryLeft < 1.5*reportInterval ){
		return true;
	}
	return false;
}

bool Agent::checkReturnTime(){

	batteryLeft--; // assume I spent one unit of energy
	returnTime++; // assume I took a step away from operator
	if(returnTime >= batteryLeft){

		float dR;
		if(relayFlag){
			dR = getDistanceToReturnToOperatorFromRelayPt() + getDistanceToReturnToRelayPt();
		}
		else if( sacrificeFlag ){
			dR = getDistanceToReportToRelayPt();
		}
		else{
			dR = getDistanceToReturnToOperator();
		}

		if( dR + 0*dR >= batteryLeft){
			returnFlag = true;
			return true;
		}
		else{
			returnTime = dR + 0*dR; // time it takes to return from current location plus slop
			returnFlag = true;
			return true;
		}
	}
	else{
		returnFlag = false;
		return false;
	}
}

bool Agent::checkForExplorationFinished(){

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point t(i,j);
			if(costmap.cells.at<short>(t) == costmap.infFree || costmap.cells.at<short>(t) == costmap.domFree){
				return false;
			}
		}
	}
	return true;

}

void Agent::infer( string inferenceMethod,  World &world ){
	this->inference.makeInference( inferenceMethod, this->costmap, world);
}

void Agent::plan( string planMethod ){

	// all agents check if they should return, includes relay / sacrifice / normal
	if( checkReturnTime() || checkForExplorationFinished() ){
		if(relayFlag){
			gLoc = returnToRelayPt();
			return;
		}
		else if( sacrificeFlag ){
			gLoc = reportToRelayPt();

			return;
		}
		else{
			gLoc = returnToOperator();
			return;
		}
	}

	// all agents check if they should report
	if( checkReportTime() && !lastReport()){
		gLoc = reportToOperator();
		return;
	}

	// don't return or report, so explore
	planExplore(planMethod);
}

void Agent::marketRelaySacrifice(){

	if(this->returnFlag && this->relayFlag ){ // is it time to return and I'm a relay or sacrifice?

		if(this->relayFlag){ // if I'm a relay
			// is a on my sacrifice list?, if yes, remove them
			if(this->myIndex == a.myRelay && a.batteryLeft <= 0){
				for(size_t i=0; i<this->sacrificeList.size(); i++){
					if(a.myIndex == this->sacrificeList[i]){
						this->sacrificeList.erase(this->sacrificeList.begin() + i);
						break;
					}
				}
			}

			// is my sacrifice list empty? if yes clear my relay flag
			if(this->sacrificeList.size() == 0){
				this->relayFlag = false;
			}

		}
	}
	else if( !market.roles[market.myIndex] == -1 && this->lastReport() ){ // I'm not a sacrifice or relay already and it's last report time
		market.marketRelaySacrifice();
	}
	else if( market.roles[myIndex] == 0 && !market.returnFlags[myIndex] && !market.returnFlags[a] ){ // I'm a sacrifice, update rLoc
		if( a.relayFlag && a.myIndex == this->myRelay ){ // theyre my relay, update rLoc
			a.rLoc = this->cLoc; // rLoc is sacrifices location to extend range
			this->rLoc = this->cLoc;
		}
	}
	else if( this->relayFlag && !this->returnFlag && !a.returnFlag){ // I'm a relay, update rLoc
		if( a.sacrificeFlag && this->myIndex == a.myRelay ){ // theyre my relay, update rLoc
			this->rLoc = a.cLoc; // rLoc is sacrifices location to extend range
			a.rLoc = a.cLoc;
		}
	}
}

Point Agent::returnToRelayPt(){
	return rLoc;
}

Point Agent::reportToRelayPt(){
	// simulate communication of operator, find node on graph that is closest to agent that is in coms of operator, return it
	Mat tComs = Mat::zeros(costmap.cells.size(), CV_8UC1);
	graphCoordination.simulateCommunication( rLoc, tComs, costmap);

	vector<Point> comPts;
	for(size_t i=0; i<graphCoordination.thinGraph.nodeLocations.size(); i++){
		if( tComs.at<uchar>(graphCoordination.thinGraph.nodeLocations[i]) == 255 ){
			comPts.push_back( graphCoordination.thinGraph.nodeLocations[i] );
		}
	}

	vector<float> comDists;
	vector<bool> trueDists;
	float minDist = INFINITY;
	int mindex = -1;
	for(size_t i=0; i<comPts.size(); i++){

		comDists.push_back( sqrt(pow(cLoc.x-comPts[i].x,2) + pow(cLoc.y-comPts[i].y,2) ));
		trueDists.push_back( false );

		if(comDists.back() < minDist){
			minDist = comDists.back();
			mindex = i;
		}
	}

	if(mindex >= 0){
		while( true ){ // find closest
			if( trueDists[mindex]){
				return comPts[mindex];
			}
			comDists[mindex] = costmap.aStarDist(cLoc, comPts[mindex]);
			trueDists[mindex] = true;

			minDist = INFINITY;
			mindex= -1;
			for(size_t i=0; i<comDists.size(); i++){
				if(comDists[i] < minDist){
					minDist = comDists[i];
					mindex = i;
				}
			}
		}
	}

	return rLoc;
}

void Agent::planExplore(string planMethod ){

	if(planMethod.compare("greedy") == 0){
		//gLoc = this->costmapCoordination.greedyFrontierPlanner(costmap, agentLocs, myIndex);
		if(costmapCoordination.standingBids.size() != agentLocs.size()){
			costmapCoordination.initializeMarket(agentLocs.size());
		}
		else{
			for(size_t i=0; i<market.cLocs.size(); i++){
				costmapCoordination.standingBids[i] = market.exploreCosts[i];
				costmapCoordination.goalLocations[i] = market.cLocs[i];
			}
		}
		gLoc = costmapCoordination.marketFrontiers(costmap, cLoc, myIndex);
	}
	else if(planMethod.compare("select") == 0){

		gLoc = costmapPlanning.searchPlanner(costmap, cLoc);
		//gLoc = costmapPlanning.mappingPlanner(costmap, cLoc);

		if(gLoc.x < 0 && gLoc.y < 0){
			cerr << "greedy" << endl;
			if(costmap.cells.at<short>(gLoc) != costmap.infFree || (gLoc.x == cLoc.x && gLoc.y == cLoc.y) ){
				gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
			}
		}
	}
	else if(planMethod.compare("selectPose") == 0){

		float w[3] = {1, 0, 0}; // explore, search, map
		float e[2] = {0.5, 1}; // dominated, breaches
		float spread = 0.3; // spread rate
		costmap.getRewardMat(w, e, spread);
		costmap.displayThermalMat( costmap.reward );

		//gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
		//gLoc = costmapPlanning.searchPlanner(costmap, cLoc);
		//gLoc = costmapPlanning.mappingPlanner(costmap, cLoc);

		clock_t tStart1 = clock();
		graphCoordination.thinGraph.createThinGraph(costmap, 1, 1);
		printf("Time taken to create thinGraph: %.2fs\n", (double)(clock() - tStart1)/CLOCKS_PER_SEC);
		cout << "thinGraph.size(): " << graphCoordination.thinGraph.nodeLocations.size() << endl;


		graphCoordination.findPosesEvolution( costmap );
		cout << "out" << endl;
		cout << "Agent::act::found graphPoses with " << this->graphCoordination.poseGraph.nodeLocations.size() << " nodes" << endl;

		//this->inference.makeVisualInference(costmap, graphCoordination.thinGraph);

		if(graphCoordination.poseGraph.nodeLocations.size() < 1){
			cout << "PoseGraph.size() == 0" << endl;
			gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
			//gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
		}
		else{
			//gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
			graphCoordination.marketPoses( costmap, cLoc, gLoc, market );
		}



		//graphCoordination.displayPoseGraph( costmap );

		if(gLoc.x < 0 || gLoc.y < 0){
			cerr << "greedy" << endl;
			if(costmap.cells.at<short>(gLoc) != costmap.infFree || (gLoc.x == cLoc.x && gLoc.y == cLoc.y) ){
				gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
			}
		}
		cout << "A" << endl;
	}
	else if(planMethod.compare("pose") == 0){
		graphCoordination.travelGraph.createPRMGraph(cLoc, costmap, 3, 9);
		//this->graph.displayCoordMap(this->costmap, false);
		//waitKey(10);

		graphCoordination.findPosesEvolution(graphCoordination.travelGraph, costmap, agentLocs);

		if(this->graphCoordination.poseGraph.nodeLocations.size() == 1){ // only one pose, switch to greedy
			gLoc = this->costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
		}
		else{
			cout << "Agent::coordinatedPlan::found graphPoses with " << this->graphCoordination.poseGraph.nodeLocations.size() << " nodes" << endl;

			gLoc = graphCoordination.posePathPlanningTSP(graphCoordination.travelGraph, costmap, agentLocs, myIndex);
		}
	}
}

void Agent::act(){

	if(cLoc == rLoc && returnFlag && relayFlag){ // if I'm a relay and it's time to relay, land at rLoc and wait
		batteryLeft++;
	}
	else if(batteryLeft > 0){
		if(cLoc == gLoc){
			while(true){
				Point g;
				g.x = gLoc.x + rand() % 5 - 2;
				g.y = gLoc.y + rand() % 5 - 2;

				if(costmap.cells.at<short>(g) == costmap.obsFree){
					gLoc = g;
					break;
				}
			}
		}

		myPath = costmap.aStarPath(cLoc, gLoc);
		cLoc = myPath[1];
		myPath.erase(myPath.begin());
	}

	cout << "Agent[" << myIndex << "]::act::cLoc / gLoc / reportCntr / batteryLeft / returnTime: " << cLoc << " / " << gLoc << " / " << reportCntr << " / " << batteryLeft << " / " << returnTime << endl;
	cout << "Agent[" << myIndex << "]::act::reportFlag / returnFlag: " << reportFlag << " / " << returnFlag << endl;
	cout << "Agent[" << myIndex << "]::act::relayFlag / sacrificeFlag: " << relayFlag << " / " << sacrificeFlag << endl;
	if( relayFlag || sacrificeFlag ){
		cout << "Agent[" << myIndex << "]::act::rLoc / oLoc: " << rLoc << " / " << oLoc << endl;
	}
}

void Agent::showCellsPlot(){
	costmap.buildCellsPlot();

	circle(costmap.displayPlot,cLoc,2, myColor,-1, 8);
	circle(costmap.displayPlot,gLoc,2, Scalar(0,0,255),-1, 8);

	char buffer[50];
	sprintf(buffer,"Agent[%d]::costMat", myIndex);

	namedWindow(buffer, WINDOW_NORMAL);
	imshow(buffer, costmap.displayPlot);
	waitKey(1);
}

void Agent::pickMyColor(){
	this->myColor = {0,0,0};

	if(this->myIndex == 0){
		this->myColor[0] = 255;
	}
	else if(this->myIndex == 1){
		this->myColor[1] = 255;
	}
	else if(this->myIndex == 2){
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 3){
		this->myColor[0] = 255;
		this->myColor[1] = 153;
		this->myColor[2] = 51;
	}
	else if(this->myIndex == 4){
		this->myColor[0] = 255;
		this->myColor[1] = 255;
		this->myColor[2] = 51;
	}
	else if(this->myIndex == 5){
		this->myColor[0] = 255;
		this->myColor[1] = 51;
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 6){
		this->myColor[0] = 51;
		this->myColor[1] = 255;
		this->myColor[2] = 255;
	}
	else if(this->myIndex == 7){
		this->myColor[0] = 153;
		this->myColor[1] = 255;
		this->myColor[2] = 51;
	}
	else if(this->myIndex == 8){
		this->myColor[0] = 127;
		this->myColor[1] = 127;
		this->myColor[2] = 127;
	}
	else if(this->myIndex == 9){
		// white
	}
}

Agent::~Agent() {

}

/*
 *

void Agent::shareCostmap(Costmap &A, Costmap &B){
	for(int i=0; i<A.cells.cols; i++){
		for(int j=0; j<A.cells.rows; j++){
			Point a(i,j);

			// share cells
			if(A.cells.at<short>(a) != B.cells.at<short>(a) ){ // do we think the same thing?
				if(A.cells.at<short>(a) == A.unknown){
					A.cells.at<short>(a) = B.cells.at<short>(a); // if A doesn't know, anything is better
				}
				else if(A.cells.at<short>(a) == A.infFree || A.cells.at<short>(a) == A.infWall){ // A think its inferred
					if(B.cells.at<short>(a) == B.obsFree || B.cells.at<short>(a) == B.obsWall){ // B has observed
						A.cells.at<short>(a) = B.cells.at<short>(a);
					}
				}
				else if(B.cells.at<short>(a) == B.unknown){ // B doesn't know
					B.cells.at<short>(a) = A.cells.at<short>(a); // B doesn't know, anything is better
				}
				else if(B.cells.at<short>(a) == B.infFree || B.cells.at<short>(a) == B.infWall){ // B think its inferred
					if(A.cells.at<short>(a) == A.obsFree || A.cells.at<short>(a) == A.obsWall){ // A has observed
						B.cells.at<short>(a) = A.cells.at<short>(a);
					}
				}
			}

			if(A.cells.at<short>(a) != B.cells.at<short>(a) ){ // do we think the same thing?

				if(A.cells.at<short>(a) != A.obsFree && A.cells.at<short>(a) != A.obsWall){ // if A hasn't observed anything
					if(B.cells.at<short>(a) == B.obsFree || B.cells.at<short>(a) == B.obsWall){ // B has observed
						A.cells.at<short>(a) = B.cells.at<short>(a); // update A
					}
				}

				if(B.cells.at<short>(a) != B.obsFree && B.cells.at<short>(a) != B.obsWall){ // if B hasn't observed anything
					if(A.cells.at<short>(a) == A.obsFree || A.cells.at<short>(a) == A.obsWall){ // A has observed
						B.cells.at<short>(a) = A.cells.at<short>(a); // update B
					}
				}
			}


			// share search
			if(A.searchReward.at<float>(a) < B.searchReward.at<float>(a) ){ // do we think the same thing?
				B.searchReward.at<float>(a) = A.searchReward.at<float>(a);
			}
			else if(A.searchReward.at<float>(a) > B.searchReward.at<float>(a)){
				A.searchReward.at<float>(a) = B.searchReward.at<float>(a);
			}

			// share occ

			float minA = INFINITY;
			float minB = INFINITY;

			if(1-A.occ.at<float>(a) > A.occ.at<float>(a) ){
				minA = 1-A.occ.at<float>(a);
			}
			else{
				minA = A.occ.at<float>(a);
			}

			if(1-B.occ.at<float>(a) > B.occ.at<float>(a) ){
				minB = 1-B.occ.at<float>(a);
			}
			else{
				minB = B.occ.at<float>(a);
			}

			if( minA < minB ){ // do we think the same thing?
				B.occ.at<float>(a) = A.occ.at<float>(a);
			}
			else{
				A.occ.at<float>(a) = B.occ.at<float>(a);
			}
		}
	}
}

void Agent::soloPlan(string method, int timeLeft){

	if(method.compare("greedy") == 0){

		//graphPlanning.prmGraph.updatePRMGraph(costmap, 5, 5, 20);
		//graphPlanning.prmGraph.displayCoordMap(costmap, true);

		if(costmap.cells.at<short>(gLoc) != costmap.infFree || (gLoc.x == cLoc.x && gLoc.y == cLoc.y) ){
			cout << "going into costmapPlanning.GreedyFrontierPlanner" << endl;
			gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
			cout << "gLoc: " << gLoc.x << " , " << gLoc.y << endl;
		}
	}
	else if(method.compare("select") == 0){

		float w[3] = {1, 1, 0.5}; // explore, search, map
		float e[2] = {0.5, 5}; // dominated, breaches
		float spread = 0.5; // spread rate
		costmap.getRewardMat(w, e, spread);
		costmap.displayThermalMat( costmap.reward );

		//gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
		//gLoc = costmapPlanning.searchPlanner(costmap, cLoc);
		//gLoc = costmapPlanning.mappingPlanner(costmap, cLoc);

		gLoc.x = -1;
		gLoc.y = -1;

		if(gLoc.x < 0 || gLoc.y < 0){
			cerr << "greedy" << endl;
			if(costmap.cells.at<short>(gLoc) != costmap.infFree || (gLoc.x == cLoc.x && gLoc.y == cLoc.y) ){
				gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
			}
		}
	}
	else if(method.compare("selectPose") == 0){

		float w[3] = {1, 0, 0}; // explore, search, map
		float e[2] = {0.5, 1}; // dominated, breaches
		float spread = 0; // spread rate
		costmap.getRewardMat(w, e, spread);
		costmap.displayThermalMat( costmap.reward );

		//gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
		//gLoc = costmapPlanning.searchPlanner(costmap, cLoc);
		//gLoc = costmapPlanning.mappingPlanner(costmap, cLoc);

		clock_t tStart1 = clock();
		graphPlanning.thinGraph.createThinGraph(costmap, 1, 1);
		printf("Time taken to create thinGraph: %.2fs\n", (double)(clock() - tStart1)/CLOCKS_PER_SEC);
		cout << "thinGraph.size(): " << graphPlanning.thinGraph.nodeLocations.size() << endl;

		tStart1 = clock();
		graphPlanning.findPosesEvolution( costmap );
		printf("Time taken to find poses 1: %.2fs\n", (double)(clock() - tStart1)/CLOCKS_PER_SEC);
		cerr << "out" << endl;
		cout << "Agent::act::found graphPoses with " << this->graphPlanning.poseGraph.nodeLocations.size() << " nodes" << endl;
		cout << "thinGraph.size(): " << graphPlanning.thinGraph.nodeLocations.size() << endl;


		if(graphPlanning.poseGraph.nodeLocations.size() < 1){
			gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
		}
		else{
			//gLoc = costmapPlanning.explorePlanner(costmap, cLoc);
			graphPlanning.marketPoses( costmap, cLoc, gLoc );
		}

		//graphPlanning.displayPoseGraph( costmap );

		if(gLoc.x < 0 || gLoc.y < 0){
			cerr << "greedy" << endl;
			if(costmap.cells.at<short>(gLoc) != costmap.infFree || (gLoc.x == cLoc.x && gLoc.y == cLoc.y) ){
				gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
			}
		}


	}
	else if(method.compare("pose") == 0){

		clock_t tStart1 = clock();
		graphPlanning.travelGraph.createPRMGraph(cLoc, costmap, 2, 4);
		printf("Time taken to getPRM: %.2fs\n", (double)(clock() - tStart1)/CLOCKS_PER_SEC);
		cout << "travelGraph.size(): " << graphPlanning.travelGraph.nodeLocations.size() << endl;

		graphPlanning.thinGraph.createThinGraph(costmap, 2, 2);

		graphPlanning.findPosesEvolution( costmap );
		cout << "Agent::act::found graphPoses with " << this->graphPlanning.poseGraph.nodeLocations.size() << " nodes" << endl;
		cout << "thinGraph.size(): " << graphPlanning.thinGraph.nodeLocations.size() << endl;



		if(graphPlanning.poseGraph.nodeLocations.size() <= 1){
			gLoc = costmapPlanning.greedyFrontierPlanner(costmap, cLoc);
		}
		else{
			gLoc = graphPlanning.TSPPosePathPlanning(costmap);
			//gLoc = this->graphPlanning.MCTSPosePathPlanning(timeLeft, graph, costmap);
		}
	}
	cerr << "out of solo plan" << endl;
}
 */

