/*
 * Pose.cpp
 *
 *  Created on: Nov 2, 2016
 *      Author: andy
 */

#include "Pose.h"

Pose::Pose(Point loc, Costmap &costmap, float radius, int nSamples) {
	this->loc = loc;
	this->needInference = true;

	initPose(costmap, radius, nSamples);
}

void Pose::initPose(Costmap &costmap, float radius, int nSamples){

	Mat ta = Mat::zeros(costmap.cells.size(), CV_8UC1);

	Point p;
	for(int ns=0; ns<nSamples; ns++){
		p.x = loc.x + round( radius*cos( 6.28318*ns/nSamples) ); // end point I am aiming at
		p.y = loc.y + round( radius*sin( 6.28318*ns/nSamples) );

		LineIterator it(ta, loc, p, 4, false);
		for(int i=0; i<it.count; i++, ++it){
			Point pp  = it.pos();
			if(costmap.cells.at<short>(pp) > costmap.infFree){ // detect if a cell is infFree
				this->obsLim.push_back( Point(pp.x-loc.x, pp.y-loc.y) ); // record position of end pt relative to center
				this->obsLen.push_back( sqrt( pow(loc.x - pp.x,2) + pow(loc.y - pp.y,2)) ); // record lengths
				this->obsVal.push_back( costmap.cells.at<short>(pp));
				break;
			}

			if(i+1 >= it.count){ // didn't hit a wall
				this->obsLim.push_back( Point(p.x-loc.x, p.y-loc.y) ); // record position of end pt relative to center
				this->obsLen.push_back( radius ); // record lengths
				this->obsVal.push_back( costmap.cells.at<short>(pp));
			}
		}
	}

	// recenter loc to (0,0)
	loc.x = 0;
	loc.y = 0;
}

void Pose::makeMat(){

	// find min and max x/y vals
	int minx = 1000;
	int miny = 1000;
	int maxx = -1000;
	int maxy = -1000;
	for(size_t i=0; i<obsLim.size(); i++){
		if(obsLim[i].x > maxx){
			maxx = obsLim[i].x;
		}
		else if(obsLim[i].x < minx){
			minx = obsLim[i].x;
		}

		if(obsLim[i].y > maxy){
			maxy = obsLim[i].y;
		}
		else if(obsLim[i].y < miny){
			miny = obsLim[i].y;
		}
	}

	mat = Mat::zeros(maxy - miny+10,maxx - minx+10,CV_8UC1);
	for(size_t i=0; i<obsLim.size(); i++){
		Point t(obsLim[i].x + 5 - minx, obsLim[i].y + 5 -  miny);
		mat.at<uchar>(t) = 255;
	}

	mat.at<uchar>(Point( 5 - minx, 5 - miny) ) = 127;
}

void Pose::addToMat(Mat &matIn){

	// find min and max x/y vals
	int minx = 1000;
	int miny = 1000;
	int maxx = -1000;
	int maxy = -1000;
	for(size_t i=0; i<obsLim.size(); i++){
		if(obsLim[i].x > maxx){
			maxx = obsLim[i].x;
		}
		else if(obsLim[i].x < minx){
			minx = obsLim[i].x;
		}

		if(obsLim[i].y > maxy){
			maxy = obsLim[i].y;
		}
		else if(obsLim[i].y < miny){
			miny = obsLim[i].y;
		}
	}

	for(size_t i=0; i<obsLim.size(); i++){
		Point t(obsLim[i].x + 5 - minx, obsLim[i].y + 5 -  miny);
		if( t.x < matIn.cols && t.y < matIn.rows){
			matIn.at<uchar>(t) = 127;
		}
	}



}

Pose::~Pose() {}

