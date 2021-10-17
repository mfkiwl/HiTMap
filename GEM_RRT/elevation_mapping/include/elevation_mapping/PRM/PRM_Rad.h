#pragma once

#include <string>
#include "elevation_mapping/PRM/Graph.h"

// OpenCV
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


namespace topological_mapping {

using namespace elevation_mapping;

typedef priority_queue<pdi, vector<pdi>, greater<pdi>> Queue;

class PRM_Rad
{
public:
	Graph network;
	Grid2D occupancy_grid;
	double radius_;
	int count;

  	cv::Mat image;	// for visualization
	cv::Mat obstacles;
	int interval_;
	int xmax_, ymax_;
	int numIterations_;
	int cellWidth_, cellHeight_;
	int x_numNodes, y_numNodes;
	float xcenter_, ycenter_;
	double memory_use;

	// constructor
	PRM_Rad();
	PRM_Rad(int interval, int xmax, int ymax, float xcenter, float ycenter, int cellWidth, int cellHeight, double radius);
	PRM_Rad(const PRM_Rad &prm);      // copy constructor
	~PRM_Rad();

	inline bool collisionCheck(GraphNode& node1, GraphNode& node2);
	inline void createLineEquation(double& A, double& B, double& C, GraphNode& node1, GraphNode& node2);
	inline bool checkDistanceMetric(double& A, double& B, double& C, GraphNode& node1, GraphNode& node2, Obstacle& square);
	inline double normDistance(double& A, double& B, double& C, double& y, double& x);
	
	GraphNode* getUniformNode(int& x_numNodes, int& y_numNodes, int iter);
	GraphNode* getRandomNode(int& x_numNodes, int& y_numNodes);

	Queue FindDistance(GraphNode& Point);
	
	inline double L2Distance(GraphNode& Node1, GraphNode& Node2);
	
	vector<pid> createConnectivityVector(Queue& distanceHeap, int neighbours);
	
	inline bool checkDuplicateNode(Queue& Heap);
	inline bool checkInBox(GraphNode& node1, GraphNode& node2, Obstacle& square);

	bool ConnectNeighbours(Queue& Heap, GraphNode& Point);
	void remapPRM(float xcenter, float ycenter);
	void incrementPRM(int x_index, int y_index);

	void changePRMCenter(float xcenter, float ycenter);
	void generatePRM();
	void remapPRM();
	void reset(int xmax, int ymax, float xcenter, float ycenter, int cellWidth, int cellHeight);
	void findFrontiers(int x, int y);
};

}