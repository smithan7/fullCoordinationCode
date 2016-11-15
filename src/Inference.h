/*
 * Inference.h
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#ifndef INFERENCE_H_
#define INFERENCE_H_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "BuildingTemplate.h"
#include "RoomTemplate.h"
#include "Contour.h"
#include "World.h"
#include "Frontier.h"
#include "Graph.h"
#include "Pose.h"

using namespace std;
using namespace cv;

class Inference {
public:
	Inference();
	virtual ~Inference();

	// usfeul stuff
	int minContourToInfer;
	vector<Point> hullBreaches;

	// main inference functions
	void makeInference(string method, Costmap &costmap, World &world);
	void makeGlobalInference(World &world, Costmap &costmap);
	void makeGeometricInference(Costmap &costmap);
	void makeNaiveInference( Costmap &costmap);
	void makeStructuralInference(Costmap &costmap);
	void makeVisualInference(Costmap &costmap, Graph &graph);


	// visual inference
	void visualInferenceOnFreeSpace( Costmap &costmap, Graph &graph, Mat &visInfMat );
	void visualInferenceOnObstacleContours( Costmap &costmap, Mat &visInfMat );
	// calc the matching of two histograms, full rotation
	float calcVisualFit(Pose &po, Pose &pl, Costmap &costmap, int &iter);
	// calculate the reward of two aligned scans
	float visualReward(Costmap &costmap, float dist, Point &obsPt, Point &libPt);
	void addToVisualLibrary(Pose &pose);
	Pose getVisualPose(Point p, Costmap &costmap);
	void testPoseAgainstLibrary(Pose &oPose);
	vector<Pose> visualLibrary;


	// wall inflation
	int wallInflationDistance;
	int freeInflationDistance;
	void inflateWalls(Costmap &costmap);
	bool checkForInflation(Mat &costMat, int i, int j, int dist, int val);

	// frontier exits
	vector<int> getFrontierExits(vector<Point> &outerHull);

	// Inference tools
	void getImagePoints(Mat &image, vector<Point> &ptList, int value); // all points int the image with value
	void getMatWithValue(Costmap &costmap, Mat &mat, int value);  // mat with all points in costmap of value
	void resetInference(Costmap &costmap); // set all inferred spaces to unknown

	vector<Point> findHullBreaches(Mat &in, Costmap &costmap);
	Point findContourCenter( Mat &mat );
	void removeInaccessibleContours(Costmap &costmap);
	void removeInaccessibleContoursFromCostmap(Costmap &costmap);
	float evalPointsBitwiseAnd(Mat &gt, Mat &test);
	float evalPointsBitwiseAndContours(Mat &gt, Mat &test);
	Mat getObservedWallsOnContour( Costmap &costmap, BuildingTemplate &groundTruth);

	vector<BuildingTemplate> buildingMatches;
	void ucbBuildingRANSAC(Costmap &costmap, BuildingTemplate &groundTruth);
	void UCBstructuralBagOfWordsBuildingInference(Costmap &costmap);
	vector<int> findMatchingBuildingHistograms(vector<float> &histogram, int nMatches);
	void findMatchesByAutoCorrelation(vector<float> &histogram, vector<int> &matches, vector<int> &alignment, int nMatches);
	void placeBuildingInCostmap( Costmap &costmap, Mat &bestMap );
	Mat getObservedMap( Costmap &costmap );
	void geoInferenceRecycling( Costmap &costmap, Mat &structMat );
	void getBuildingTemplate2(Costmap &costmap, BuildingTemplate &groundTruth);

	vector< RoomTemplate > roomMatches;
	void closeDominatedContours( Costmap &costmap );
	void placeRoomInCostmap(Costmap& costmap, Mat &bestMap);
	void UCBstructuralBagOfWordsRoomInference(Costmap &costmap, vector<Contour> &rooms);
	void ucbRoomRANSAC(Costmap &costmap, RoomTemplate &groundTruth);
	vector<int> findMatchingRoomHistograms(vector<float> &histogram, int nMatches);
	void doorFinderByTravelGraph(Costmap &costmap, vector<Vec4i> &doors);
	void doorFinderAlongFrontiers(Costmap &costmap, vector<Vec4i> &doors);
	Graph inferenceGraph; // for finding rooms

	vector<Frontier> frontiers;
	vector<Point> findFrontiers(Costmap &costmap);
	void clusterFrontiers(vector<Point>  frntList, Costmap &costmap);

	void doorFinderByThinMat(Costmap &costmap, vector<Vec4i> &doors);
	void simulateObservation( Point pose, Mat &resultingView, Costmap &costmap, vector<float> &length, vector<Point> &endPts);
	void doorFinderByLengthHistogram(Costmap &costmap, vector<Contour> contours, vector<Vec4i> &doors);
	void doorFinderByStructure(Costmap &costmap, vector<Vec4i> &doors);
	void doorFinderByHullDefects(Costmap &costmap, vector<Vec4i> &doors);
	vector<Point> viewPerim;

	vector<Point> hullPts;
	void displayInferenceMat(Costmap &costmap, Mat &outerHullDrawing, Mat &obstacleHull, vector<Point> &outerHull, vector<int> frontierExits);

	// structural inference stuff
	void structuralBagOfWordsRoomInference(Costmap &costmap);
	BuildingTemplate getBuildingTemplate( Costmap &costmap );

	vector<Contour> getRoomContours(Costmap &costmap, vector<Vec4i> doors);
	bool checkVisibility(Costmap &costmap, Point a, Point b);

	vector<float> shiftHistogram( vector<float> hist, int skip);
	vector<float> compareSequenceByAutoCorrelation( vector<float> &hist1, vector<float> &hist2);
	float compareHistogramByAbsDist(vector<float> &hist1,vector<float> &hist2);
	float compareHistogramByChiSquaredDist(vector<float> &hist1,vector<float> &hist2);
	float compareHistogramByInvCorrelation(vector<float> &hist1,vector<float> &hist2);


	void roomFinder(Graph &graph);
	Graph infGraph; // for keeping the node locations to see what has changed
	vector<vector<int> > doorList; // list of door locations, draw in to seal off rooms

	// geometric inference stuff
	void getOuterHull(Costmap &costmap, Mat &outerHullDrawing, vector<Point> &outerHull);

	void extractInferenceContour();
	int getMatReward(Mat &in);
	void getLengthHistogram(vector<float> length, float meanLength, vector<int> &histogram, vector<float> &sequence);
	vector<vector<float> > roomHistogramList;
	vector<vector<Point> > roomPointList;
	vector<String> roomNameList;
	vector<Point> roomCenterList;
	vector<float> roomMeanLengthList;
	vector<vector<Point> > roomWallsList;

	vector<vector<float> > buildingHistogramList;
	vector<vector<float> > buildingSequenceList;
	vector<vector<Point> > buildingPointList;
	vector<String> buildingNameList;
	vector<Point> buildingCenterList;
	vector<float> buildingMeanLengthList;

	Mat geoInferenceMat, structInferenceMat;
	vector<Point> geoInfContour;

	float minMatchStrength;

	void visualBagOfWordsInference(vector<vector<Point> > &contours, Mat &obstaclesAndHull);
	void makeInferenceAndSetFrontierRewards(); // create outer hull and divide into contours
	void visualInference(); // perform visual inference on each contour
	void valueFrontiers(); // take the contours from inference and get the value of each

	void clusteringObstacles();

	/*
	//////////////////// Begin Retired Functions
	void mergeInferenceContours(Costmap &costmap);
	float evalBreadthSearch(Mat &gt, Mat &tst, int nTests);
	float breadthFirstSearchDist(Point in, Mat &mat);
	float evalPointsPoly(vector<Point> &gtCont, vector<Point> &testCont, float shift_x, float shift_y, int nTests);
	void structuralBagOfWordsBuildingInference(Costmap &costmap);
	void initMyRANSAC(Costmap &costmap, Mat &matchMat, Point matchCenter, Contour &contour, float &theta, float &cost);
	void myBuildingRANSAC(Costmap &costmap, Mat &matchMat, Mat &buildingHullWalls, Contour &contour, float &theta, Mat &bestMap, float cost);
	vector<float> getInferenceContourRewards(vector<int> frontierExits, vector<vector<Point> > contours);
	void setFrontierRewards(vector<float> rewards, vector<vector<Point> > inferenceContours);
	Mat createMiniMapInferImg();
	*/

};

#endif /* INFERENCE_H_ */
