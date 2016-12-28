//============================================================================
// Name        : discrete_coordination.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include<dirent.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Agent.h"
#include "Observer.h"

using namespace cv;
using namespace std;

int findMapToRun(vector<string> &fName);
void loadMapNames(vector<string> &fName);
float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap);
float getPercentDominated(Costmap &globalCostmap, Costmap &workingCostmap);
float getPrecisionInferredAndObserved(Costmap &workingCostmap, Costmap &globalCostmap);
float getPrecisionInferred(Costmap &workingCostmap, Costmap &globalCostmap);
float getPrecisionObserved(Costmap &workingCostmap, Costmap &globalCostmap);
float getRecallInferred(Costmap &workingCostmap, Costmap &globalCostmap);
float getRecallObserved(Costmap &workingCostmap, Costmap &globalCostmap);

int main(){

	destroyAllWindows();

	vector<int> treePath;
	srand( time(NULL) );
	bool videoFlag = true;

	int numAgents = 1;
	int numIterations = 1;

	int gSpace = 5;
	float obsThresh = 100;
	float comThresh = 100;
	int maxTime = 700;
	int reportInterval = 50000;
	float batteryInit = 500000;
	bool enableRelaySacrifice = false;

	vector<string> fName;
	//fName.push_back("mineMap");
	//fName.push_back("mineMap2");
	//fName.push_back("gmapping");
	//fName.push_back("tunnelTest");


	loadMapNames(fName);

	vector<string> exploreMethod;
	//exploreMethod.push_back("pose");
	//exploreMethod.push_back("select");
	//exploreMethod.push_back("selectPose");
	//exploreMethod.push_back("greedyFrontiers");
	exploreMethod.push_back("marketFrontiers");

	vector<string> inferenceMethod;
	//inferenceMethod.push_back("naive");
	//inferenceMethod.push_back("geometric");
	//inferenceMethod.push_back("structural");
	//inferenceMethod.push_back("global");
	inferenceMethod.push_back("visual");

	srand( time(NULL) );


	for(size_t map_iter = 0; map_iter < 1; map_iter++){

		int mapName = findMapToRun( fName );
		// create world
		World world(fName[mapName], gSpace, obsThresh, comThresh);
		if( world.costmap.cells.cols > 300 || world.costmap.cells.rows > 300){
			return -1;
		}

		cout << "main::loaded world" << fName[ mapName ] << endl;
		vector<Point> sLoc;
		for(int i=0; i<numIterations*numAgents; i++){
			while(true){
				Point tLoc(rand() % world.costmap.cells.cols, rand() % world.costmap.cells.rows);
				//Point tLoc(26,25);
				if(world.costmap.cells.at<short>(tLoc) == world.costmap.obsFree){ // its not an obstacle
					sLoc.push_back(tLoc);
					break;
				}
			}
		}
		cout << "main::Chose starting locations" << endl;

		for(size_t exploreMethod_iter = 0; exploreMethod_iter<exploreMethod.size(); exploreMethod_iter++){
			for(size_t inferenceMethod_iter = 0; inferenceMethod_iter<inferenceMethod.size(); inferenceMethod_iter++){
				for(int iterations_iter = 0; iterations_iter<numIterations; iterations_iter++){

					vector<int> timeLog;
					vector<float> precisionInferredAndObservedLog;
					vector<float> precisionInferredLog;
					vector<float> precisionObservedLog;
					vector<float> recallInferredLog;
					vector<float> recallObservedLog;

					Observer humanObserver(sLoc[iterations_iter], numAgents, false, "operator", numAgents, enableRelaySacrifice);
					if( inferenceMethod.back().compare( "visual" ) == 0){
						//humanObserver.inference.loadVisualInferenceLibrary();
					}
					Observer globalObserver(sLoc[iterations_iter], numAgents, true, "global", -1, enableRelaySacrifice);// make this observer get maps shared with it and NOT share its map with people it
					vector<Agent> agents;
					for(int i=0; i<numAgents; i++){

						// if the same starting locations
						agents.push_back(Agent(sLoc[iterations_iter*numAgents], i, batteryInit, obsThresh, comThresh, numAgents, reportInterval, enableRelaySacrifice ));
						cout << "Main::Agent[" << i << "]: created at : " << sLoc[iterations_iter*numAgents].x << " , " << sLoc[iterations_iter*numAgents].y << endl;
						// if different starting locations
						//agents.push_back(Agent(sLoc[iterations_iter*numAgents+i], i, obsThresh, comThresh, numAgents, reportInterval, returnTime));
						//cout << "Main::Agent[" << i << "]: created at : " << sLoc[iterations_iter*numAgents+i].x << " , " << sLoc[iterations_iter*numAgents+i].y << endl;

						if( inferenceMethod.back().compare( "visual" ) == 0){
							agents.back().inference.loadVisualInferenceLibrary();
						}
					}
					cout << "Main::All agents created" << endl;

					time_t start = clock();

					// video writers
					VideoWriter operatorVideo, globalVideo;
					if(videoFlag){
						Size frameSize = world.costmap.cells.size();
						operatorVideo.open("multiAgentInferenceOperator.avi",CV_FOURCC('M','J','P','G'), 30, frameSize, true );
						globalVideo.open("multiAgentInferenceGlobal.avi",CV_FOURCC('M','J','P','G'), 30, frameSize, true );
						cout << "Main::Videos started" << endl;
					}

					int timeSteps = -1;
					float percentObserved = 0;

					// initialize observer costmaps
					world.observe(humanObserver.cLoc, humanObserver.costmap);
					world.observe(globalObserver.cLoc, globalObserver.costmap);

					globalObserver.showCellsPlot();
					cout << "Main::Ready, press any key to begin." << endl;
					waitKey(1);
					cout << "Main::Here we go!" << endl;

					while(timeSteps < maxTime-1 && percentObserved < 0.99){
						//cout << "Main::Starting while loop" << endl;
						timeSteps++;

						// all agents observe
						for(int i=0; i<numAgents; i++){
							world.observe(agents[i].cLoc, agents[i].costmap);
							agents[i].market.updateMarket(agents[i].cLoc, agents[i].gLoc);
							world.observe(agents[i].cLoc, globalObserver.costmap);
						}
						humanObserver.market.updateMarket( humanObserver.cLoc, humanObserver.cLoc );
						globalObserver.market.iterateTime();
						//cout << "Main::made observations" << endl;

						// all agents communicate
						for(int i=0; i<numAgents; i++){
							// all agents communicate with global observer, always
							vector<int> marketTransmission = agents[i].market.assembleTransmission();
							vector<int> costmapTransmission = agents[i].costmap.publishCostmap();

							globalObserver.market.dissasembleTransmission( marketTransmission );

							// all agents communicate with humanObserver, if possible
							if( world.commoCheck(agents[i].cLoc, humanObserver.cLoc, comThresh) ){
								humanObserver.market.dissasembleTransmission( marketTransmission );
								humanObserver.costmap.subscribeCostmap( costmapTransmission );

								vector<int> obsTran = humanObserver.market.assembleTransmission();
								agents[i].market.dissasembleTransmission( obsTran );

								vector<int> cmTran = humanObserver.costmap.publishCostmap();
								agents[i].costmap.subscribeCostmap( cmTran );
							}
							// all agents communicate with each other if possible
							for(int j=0; j<numAgents; j++){
								if(i!=j){
									if(world.commoCheck(agents[i].cLoc, agents[j].cLoc, comThresh)){
										agents[j].market.dissasembleTransmission( marketTransmission );
										agents[j].costmap.subscribeCostmap( costmapTransmission );
									}
								}
							}
						}
						//cout << "Main::Out of communicate with other agents" << endl;

						//cout << "Main::Into inference" << endl;
						if(humanObserver.inference.visualLibrary.size() == 0){
							humanObserver.inference.buildVisualInferenceLibrary( world );
						}

						//humanObserver.inference.makeInference( inferenceMethod[0], humanObserver.costmap, world);
						humanObserver.showCellsPlot();
						globalObserver.inference.makeInference( "global", globalObserver.costmap, world );
						globalObserver.showCellsPlot();

						// all agents plan for one timestep
						for(int i=0; i<numAgents; i++){
							if( agents[i].inference.visualLibrary.size() == 0){
								agents[i].inference.buildVisualInferenceLibrary( world );
							}
							agents[i].infer( inferenceMethod[0], world); // includes updating internal cLoc
						}
						//cout << "Main::out of inference" << endl;

						//cout << "Main::Into plan for one timestep" << endl;
						// all agents plan for one timestep
						for(int i=0; i<numAgents; i++){
							agents[i].plan(exploreMethod[exploreMethod_iter] ); // includes updating internal cLoc
						}

						// all agents act for one timestep
						for(int i=0; i<numAgents; i++){
							agents[i].act();
						}

						if(videoFlag){
							for(int i =0; i<5; i++){ // slower frmae rate through repeated frames
								operatorVideo << humanObserver.costmap.displayPlot;
								globalVideo << globalObserver.costmap.displayPlot;
							}
						}

						// print out progress
						percentObserved = getPercentObserved(world.costmap, humanObserver.costmap);

						timeLog.push_back( timeSteps );
						precisionInferredAndObservedLog.push_back( getPrecisionInferredAndObserved(agents[0].costmap, world.costmap) );
						precisionInferredLog.push_back( getPrecisionInferred(agents[0].costmap, world.costmap) );
						precisionObservedLog.push_back( getPrecisionObserved(agents[0].costmap, world.costmap) );
						recallInferredLog.push_back( getRecallInferred(agents[0].costmap, world.costmap) );
						recallObservedLog.push_back( getRecallObserved(agents[0].costmap, world.costmap) );

						if(exploreMethod[exploreMethod_iter].compare("pose") == 0){
							float percentDominated = getPercentDominated(world.costmap, agents[0].costmap);
							percentObserved += percentDominated;
						}

						//cout << "------timeSteps & percent observed: " << timeSteps << " & " << percentObserved << " " << map_iter << ": " << fName[map_iter] << endl;

					} // end timeStep in simulation

					cout << "made it to the end of the simulation!" << endl;

					if( percentObserved < 0.98 ){
						return -1;
					}

					ofstream myfile;
					myfile.open ("/home/andy/git/coordination/Results/Visual_inf_27_dec_2016.csv", ofstream::out | ofstream::app);
					for(size_t i=0; i<timeLog.size(); i++){
						myfile << fName[mapName] << ",";
						myfile << inferenceMethod[inferenceMethod_iter] << ",";
						myfile << exploreMethod[exploreMethod_iter] << ",";
						myfile << timeLog[i] << ",";

						myfile << precisionInferredAndObservedLog[i] << ",";
						myfile << precisionInferredLog[i] << ",";
						myfile << precisionObservedLog[i] << ",";

						myfile << recallInferredLog[i] << ",";
						myfile << recallObservedLog[i] << "\n";
					}
					myfile.close();

					cerr << "wrote the file!" << endl;

					//waitKey(0);
				} // end test iterations
			} // end inference iterations
		} // end explore methods
	} // end maps
}// end main

float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap){

	float globalFree = 0;
	float workingFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				workingFree++;
			}
		}
	}

	return workingFree / globalFree;
}

float getPercentDominated(Costmap &globalCostmap, Costmap &workingCostmap){

	float globalFree = 0;
	float dominated = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.domFree){
				dominated++;
			}
		}
	}

	return dominated / globalFree;
}


int findMapToRun(vector<string> &fName){

	/*
	ifstream file ( "/home/andy/git/coordination/Debug/InferenceSept10.csv", ifstream::in ); // declare file stream: http://www.cplusplus.com/reference/iostream/ifstream/
	if ( file.is_open() ){
		string value;
		vector<string> mapNames;
		string mapName;
		int c = 0;
		while ( file.good() ){
			c++;
			getline ( file, value, '\n' ); // read a string until next comma: http://www.cplusplus.com/reference/string/getline/

			int fnd = value.find_first_of(",");

			//if(!mapName.compare( value.substr(0, fnd)) == 0){
				//cout << value << endl; // display value removing the first and the last character from it
				mapName = value.substr(0, fnd);
				//cout << mapName << endl;
				mapNames.push_back( mapName );
			//}
		}
		file.close();

		vector<int> cnt;
		for(size_t i=0; i<fName.size(); i++){
			int fi = fName[i].find_last_of("/");
			string fn = fName[i].substr(fi+1);
			int cntr = 0;
			for(size_t j=0; j<mapNames.size(); j++){
				if( fn.compare( mapNames[j] ) == 0 ){
					cntr++;
				}
			}
			//cout << fn << " : " << cntr << endl;
			cnt.push_back(cntr);
		}

		double minV = INFINITY;
		vector<int> minI;

		for(size_t i=0; i<cnt.size(); i++){
			if(cnt[i] == minV){
				minV = cnt[i];
				minI.push_back(i);
			}
			if(cnt[i] < minV){
				minV = cnt[i];
				minI.clear();
				minI.push_back(i);
			}
		}


		int v = rand() % minI.size();

		return minI[v];
	}
	*/

	return rand() % fName.size();

}


void loadMapNames(vector<string> &fName){
	string dir = "/home/andy/git/fabmap2Test/InferenceLibrary/TestMaps/generated/";
	ifstream fin;
	string filepath;
	DIR *dp;
	struct dirent *dirp;
	struct filestat;

	dp = opendir( dir.c_str() ); // open provided directory
	if (dp == NULL){ // did it open
		cerr << "Error opening " << dir << endl;
		waitKey(0);
	}

	while ((dirp = readdir( dp ))){ // read all files in directory to get filenames
		// get file to open
		filepath = dir + dirp->d_name;
		string fileName = dirp->d_name;
		if(fileName.substr(fileName.find_last_of(".") + 1) != "jpg"){
		    continue;
		}
		fName.push_back( filepath.substr( 0, filepath.find_last_of(".") ) );
	}
}

float getRecallObserved(Costmap &workingCostmap, Costmap &globalCostmap){

	// recall = true positives / all positives

	int globalFree = 0;
	int workingFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				workingFree++;
			}
		}
	}
	return float(workingFree)/float(globalFree);
}

float getRecallInferred(Costmap &workingCostmap, Costmap &globalCostmap){

	// recall = true positives / all positives

	int globalFree = 0;
	int workingFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				workingFree++;
			}
			else if(workingCostmap.cells.at<short>(a) == workingCostmap.infFree || workingCostmap.cells.at<short>(a) == workingCostmap.domFree){
				if( globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					workingFree++;
				}
			}
		}
	}
	return float(workingFree)/float(globalFree);
}

float getPrecisionObserved(Costmap &workingCostmap, Costmap &globalCostmap){

	// precision = correctly observed / observed free

	int correctlyObsFree = 0;
	int obsFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				obsFree++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					correctlyObsFree++;
				}
			}
		}
	}

	return float(correctlyObsFree)/float(obsFree);
}

float getPrecisionInferred(Costmap &workingCostmap, Costmap &globalCostmap){

	// precision = (correctly inferred free) / (inferred free)

	int correctlyInferredFree = 0;
	int inferredFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(workingCostmap.cells.at<short>(a) == workingCostmap.infFree || workingCostmap.cells.at<short>(a) == workingCostmap.domFree){
				inferredFree++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					correctlyInferredFree++;
				}
			}
		}
	}
	return float(correctlyInferredFree)/float(inferredFree);
}

float getPrecisionInferredAndObserved(Costmap &workingCostmap, Costmap &globalCostmap){

	// precision = (correctly inferred + observed free) / (inferred + observed free)

	int correctlyInferredFree = 0;
	int inferredFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(workingCostmap.cells.at<short>(a) == workingCostmap.infFree || workingCostmap.cells.at<short>(a) == workingCostmap.domFree || workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				inferredFree++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					correctlyInferredFree++;
				}
			}
		}
	}

	return float(correctlyInferredFree)/float(inferredFree);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Retired Bits /////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/*

	ofstream myfile;
	myfile.open ("/home/andy/git/coordination/Debug/InferenceSept14.csv", ofstream::out | ofstream::app);
	for(size_t i=0; i<timeLog.size(); i++){
		myfile << mapLog << ",";
		myfile << inferenceMethodLog[i] << ",";
		myfile << exploreMethodLog[i] << ",";
		myfile << timeLog[i] << ",";

		myfile << precisionInferredAndObservedLog[i] << ",";
		myfile << precisionInferredLog[i] << ",";
		myfile << precisionObservedLog[i] << ",";

		myfile << recallInferredLog[i] << ",";
		myfile << recallObservedLog[i] << "\n";
	}
	myfile.close();

	cerr << "wrote the file!" << endl;


Mat naiveCostMap = master.inference.makeNaiveMatForMiniMap(); // naive to remaining space
cout << "made naive" << endl;
Mat inferredGeometricCostMap = master.inference.makeGeometricInferenceMatForMiniMap(); // inferred remaining space
cout << "made geometric" << endl;
//Mat inferredStructuralCostMap = master.makeStructuralInferenceMatForMiniMap();
Mat inferredGlobalCostMap = master.inference.makeGlobalInferenceMat(world); // all remaining space
cout << "made global" << endl;

float observedEntropy = getObservedMapEntropy(inferredGlobalCostMap, naiveCostMap);
float inferredEntropy = getInferredMapEntropy(inferredGlobalCostMap, inferredGeometricCostMap);

namedWindow("main::naive Costmap", WINDOW_NORMAL);
imshow("main::naive Costmap", naiveCostMap);

namedWindow("main::inferred Geo Costmap", WINDOW_NORMAL);
imshow("main::inferred Geo Costmap", inferredGeometricCostMap);

namedWindow("main::inferred Global Costmap", WINDOW_NORMAL);
imshow("main::inferred Global Costmap", inferredGlobalCostMap);
waitKey(1);

// build miniMap
cout << "building miniMap" << endl;

miniMaster.importFrontiers(master.frontiers);
for(int i=0; i<numAgent; i++){ 		// import UAV locations
	miniMaster.cLocList.push_back(agents[i].cLoc);
}

cout << "main while loop1: " << master.frntsExist <<  endl;

miniMaster.createMiniGraph(inferredGlobalCostMap);
cout << "into display" << endl;
miniMaster.displayCoordMap();
cout << "out of display" << endl;
miniMaster.cState = miniMaster.findNearestNode(agents[0].cLoc);

cout << "main while loop2: " << master.frntsExist <<  endl;


// masterGraph planning /////////////////////////////////////////////////////////////////////////////////////
float maxPathLength = 100 - timeSteps;
int gNode = agents[0].gNode;
if(timeSteps % 1 == 0){
	gNode = miniMaster.masterGraphPathPlanning(maxPathLength, 10000, 10000);
	cout << "out of masterGraphPathPlanning" << endl;
	agents[0].gNode = gNode;
}
cout << "gNode: " << gNode << endl;
vector<int> gLoc;
gLoc.push_back(miniMaster.graf[gNode][0]);
gLoc.push_back(miniMaster.graf[gNode][1]);

cout << "main while loop3: " << master.frntsExist <<  endl;



// tsp path planner ///////////////////////////////////////////////////////////////////////////////////////////
cout << "into tsp" << endl;
vector<int> path = miniMaster.tspPathPlanner(2000,1);
for(size_t i=0; i<path.size(); i++){
	cout << path[i] << endl;
}
cout << "out of tsp" << endl;

vector<int> gLoc;
gLoc.push_back(miniMaster.graf[path[1]][0]);
gLoc.push_back(miniMaster.graf[path[1]][1]);
*/

/*
// mcts path planner //////////////////////////////////////////////////////////////////////////////////////////
vector<int> path;
miniMaster.mctsPathPlanner(path, 1000, 10000);

cout << "mctsPath: ";
for(size_t i=0; i<path.size(); i++){
	cout << path[i] << ", ";
}
cout << endl;

vector<int> gLoc;
gLoc.push_back(miniMaster.graf[path[1]][0]);
gLoc.push_back(miniMaster.graf[path[1]][1]);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


cout << "out of build miniMap" << endl;

*/
