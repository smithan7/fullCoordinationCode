/*
 * Pose.cpp
 *
 *  Created on: Nov 2, 2016
 *      Author: andy
 */

#include "Pose.h"

Pose::Pose(Point loc, Costmap &costmap) {
	this->loc = loc;
	this->needInference = true;

	initPose(costmap);
}

void Pose::initPose(Costmap &costmap){

	Mat ta = Mat::zeros(costmap.cells.size(), CV_8UC1);

	Point p;
	float sr = 20;
	int nSamples = 300;
	for(int i=0; i<nSamples; i++){
		p.x = loc.x + round( sr*cos( 6.28318*i/nSamples) );
		p.y = loc.y + round( sr*sin( 6.28318*i/nSamples) );

		LineIterator it(ta, loc, p, 4, false);
		for(int i=0; i<it.count; i++, ++it){
			Point pp  = it.pos();
			if(costmap.cells.at<short>(pp) > costmap.infFree){ // detect if a cell is infFree
				this->obsLim.push_back( Point(pp.x-loc.x, pp.y-loc.y) ); // record position of end pt relative to center
				this->obsLen.push_back( sqrt( pow(loc.x - pp.x,2) + pow(loc.y - pp.y,2)) ); // record lengths

				if( isnan(obsLen.back()) ){
					cerr << "isnan" << endl;
				}

				break;
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

