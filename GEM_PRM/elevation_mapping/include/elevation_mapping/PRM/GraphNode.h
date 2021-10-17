#pragma once

#include<iostream>
#include<utility>

using namespace std;

namespace topological_mapping {

class GraphNode
{
	int cellWidth, cellHeight; // is 10 cm * 10cm at this point.

public:
	// Node should contain the centroid coordinates
	double xlow, xhigh, ylow, yhigh;	// in meter
	double xcood, ycood;	// in meter
	double memory_use;
	bool isObstacle;
	bool isFrontier;
	bool isUnknown;
	bool isReal;

	// default constructor
	GraphNode();

	// copy constructor
	GraphNode(const GraphNode &graphNode); 

	// constructor with double values
	GraphNode(double x, double y, int w=10,int h=10);
	
	// to get values about given cell for debugging
	// get details about the current Node
	void getCellParam();
	// double getMemoryUse();
	
	// tochange into obstacle
	void setObstacle();
	void setFrontier();
	void releaseFrontier();
	void setKnown();
	void setFree();

	// check if a point is present in the cell;
	bool checkPresence(pair<double,double>&);
	
	// A bool to check if the node is an obstacle or not
	bool is_Obstacle();
	bool is_Unknown();

	pair<int, int> getWidthHeight(); // returns width and height of cell in cm
};

}