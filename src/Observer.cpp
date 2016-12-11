/*
 * Observer.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: andy
 */

#include "Observer.h"

Observer::Observer(Point cLoc, int nAgents, bool global, String name, int myIndex, bool enableRelaySacrifice){
	this->cLoc = cLoc;
	this->nAgents = nAgents;
	globalObserver = global;
	this->name = name;

	for(int i=0; i<nAgents; i++){
		agentColors.push_back( setAgentColor(i) );
	}

	market.init( nAgents, myIndex, enableRelaySacrifice);
}

Observer::~Observer(){}

Scalar Observer::setAgentColor(int index){
	Scalar color(0,0,0);

	if(index == 0){
		color[0] = 255;
	}
	else if(index == 1){
		color[1] = 255;
	}
	else if(index == 2){
		color[2] = 255;
	}
	else if(index == 3){
		color[0] = 255;
		color[1] = 153;
		color[2] = 51;
	}
	else if(index == 4){
		color[0] = 255;
		color[1] = 255;
		color[2] = 51;
	}
	else if(index == 5){
		color[0] = 255;
		color[1] = 51;
		color[2] = 255;
	}
	else if(index == 6){
		color[0] = 51;
		color[1] = 255;
		color[2] = 255;
	}
	else if(index == 7){
		color[0] = 153;
		color[1] = 255;
		color[2] = 51;
	}
	else if(index == 8){
		color[0] = 127;
		color[1] = 127;
		color[2] = 127;
	}
	else if(index == 9){
		// black
	}

	return color;
}

void Observer::showCellsPlot(){
	costmap.buildCellsPlot();
	addAgentsToCostmapPlot();
	addSelfToCostmapPlot();
	namedWindow(name, WINDOW_NORMAL);
	imshow(name, costmap.displayPlot);
	waitKey(1);
}

void Observer::addAgentsToCostmapPlot(){
	for(int i=0; i<nAgents; i++){
		Scalar black(0,0,0);
		circle(costmap.displayPlot, market.cLocs[i],2, agentColors[i],-1, 8);
	}

	for(int i=0; i<nAgents; i++){
		Scalar black(0,0,0);
		circle(costmap.displayPlot,market.gLocs[i],2, black,-1, 8);
		circle(costmap.displayPlot,market.gLocs[i],1, agentColors[i],-1, 8);
	}
}

void Observer::addSelfToCostmapPlot(){
	rectangle( costmap.displayPlot, Point(cLoc.x-2, cLoc.y-2), Point(cLoc.x+2, cLoc.y+2), Scalar(0,165,255), -1, 8);
}


/*
 void Observer::shareCostmap(Costmap &A, Costmap &B){
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
 */

