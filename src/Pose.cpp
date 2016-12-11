/*
 * Pose.cpp
 *
 *  Created on: Nov 2, 2016
 *      Author: andy
 */

#include "Pose.h"

Pose::Pose(Point ploc, Costmap &costmap) {
	loc = ploc;
	needInference = false;

	radius = 10;
	nSamples = 36;

	mean = -1;
	stanDev = -1;

	initPose(costmap);

	cerr << "loc: " << loc << endl;
}

void Pose::initPose(Costmap &costmap){

	Mat ta = Mat::zeros(costmap.cells.size(), CV_8UC1);

	Point p;
	for(int ns=0; ns<nSamples; ns++){
		p.x = loc.x + round( radius*cos( 6.28318*ns/nSamples) ); // end point I am aiming at
		p.y = loc.y + round( radius*sin( 6.28318*ns/nSamples) );

		LineIterator it(ta, loc, p, 4, false);
		for(int i=0; i<it.count; i++, ++it){
			Point pp  = it.pos();

			int ppVal = costmap.cells.at<short>(pp);

			if( ppVal == costmap.infWall || ppVal == costmap.inflatedWall || ppVal == costmap.infFree ){
				this->needInference = true;
			}

			if(ppVal > costmap.infFree){ // detect if a cell is infFree
				this->obsLim.push_back( Point(pp.x-loc.x, pp.y-loc.y) ); // record position of end pt relative to center
				this->obsLen.push_back( sqrt( pow(loc.x - pp.x,2) + pow(loc.y - pp.y,2)) ); // record lengths
				this->obsVal.push_back( ppVal );
				break;
			}

			if(i+1 >= it.count){ // didn't hit a wall
				this->obsLim.push_back( Point(p.x-loc.x, p.y-loc.y) ); // record position of end pt relative to center
				this->obsLen.push_back( radius ); // record lengths
				this->obsVal.push_back( costmap.cells.at<short>(pp));
			}
		}
	}

	getMean();
	getStanDev();
}



void Pose::getMean(){

	float sum = 0;
	float N = obsLen.size();
	for(int i=0; i<N; i++){
		sum += obsLen[i];
	}

	this->mean = sum / N;
}

void Pose::getStanDev(){

	float sum = 0;
	float N = obsLen.size();
	for(int i=0; i<N; i++){
		sum += pow(this->mean - obsLen[i],2);
	}

	this->stanDev = sqrt(sum / N);
}

float Pose::getPDF( float x ){
    float pi = 3.14159265358979323846264338327950288419716939937510582;
    return 1 / sqrt(2*pow(stanDev,2)*pi)*exp(-pow(x-mean,2)/(2*pow(stanDev,2)));
}

float Pose::getCDF( float x ){
	// A Sigmoid Approximation of the Standard Normal Integral
	// Gary R. Waissi and Donald F. Rossin

    float z = (x- mean) / stanDev;
    float pi = 3.14159265358979323846264338327950288419716939937510582;
    return 1 / (1+exp(-sqrt(pi)*(-0.0004406*pow(z,5) + 0.0418198*pow(z,3) + 0.9*z) ) );
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

