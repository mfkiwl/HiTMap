#include "elevation_mapping/PRM/GraphNode.h"
#include<iostream>

using std::pair;

namespace topological_mapping {

// default constructor
GraphNode::GraphNode():cellWidth(10), cellHeight(10), xcood(0.0), ycood(0.0),
isFrontier(false), isUnknown(true), isReal(false){
	xlow = xcood - ((double)cellWidth / 200);
	xhigh = xcood + ((double)cellWidth / 200);
	ylow = ycood - ((double)cellHeight / 200);
	yhigh = ycood + ((double)cellHeight / 200);
	memory_use = 0;
}


// xcood ycood constructor
GraphNode::GraphNode(double x, double y, int w, int h) : cellWidth(w), cellHeight(h), xcood(x),
ycood(y), isFrontier(false), isUnknown(true), isReal(false) {
	xlow = xcood - ((double)cellWidth / 200);
	xhigh = xcood + ((double)cellWidth / 200);
	ylow = ycood - ((double)cellHeight / 200);
	yhigh = ycood + ((double)cellHeight / 200);
	memory_use = 0;
}


// copy constructor
GraphNode::GraphNode(const GraphNode &graphNode){      
	xcood = graphNode.xcood;
	ycood = graphNode.ycood;
	xlow = graphNode.xlow;	
	xhigh = graphNode.xhigh;
	ylow = graphNode.ylow;
	yhigh = graphNode.yhigh;
	isObstacle = graphNode.isObstacle;	
	isFrontier = graphNode.isFrontier;
	isUnknown = graphNode.isUnknown;
	cellWidth = graphNode.cellWidth;
	cellHeight = graphNode.cellHeight;
	isReal = graphNode.isReal;
	memory_use = graphNode.memory_use;
}


void GraphNode::getCellParam() {
	std::cout << std::endl;
	std::cout<< " Given Node has these Parameters:\n";
	std::cout << "Cell Width = " << cellWidth << "\tCell Height = " << cellHeight << std::endl;
	std::cout << "Centroid x coordinate = " << xcood << "\t y coordinate = " << ycood<<std::endl;
	if (isObstacle) {
		std::cout << " This node is an obstacle" << std::endl;
	}
	else {
		std::cout << " This node is not an Obstacle" << std::endl;
	}
	if (isUnknown) {
		std::cout << " This node is within unknown places" << std::endl;
	}
	else {
		std::cout << " This node is known";
	}
}


// double GraphNode::getMemoryUse(){

// }


void GraphNode::setKnown() {
	isUnknown = true;
}


void GraphNode::setObstacle() {
	isObstacle = true;
	isUnknown = false;
}


void GraphNode::releaseFrontier(){
	isFrontier = false;
	isUnknown = false;
	isReal = false;
}


void GraphNode::setFree() {
	isObstacle = false;
	isUnknown = false;
}


void GraphNode::setFrontier() {
	isFrontier = true;
	isUnknown = false;
	isReal = true;
}


pair<int, int> GraphNode::getWidthHeight() {
	return (std::make_pair(cellWidth, cellHeight));
}


bool GraphNode::checkPresence(pair<double,double>& Coordinate) {
	double x = Coordinate.first;
	double y = Coordinate.second;
	
	// check if in bounds in both x and y
	if ((x >= xlow && x <= xhigh) && (y >= ylow && y <= yhigh)) {
		return true;
	}
	else {
		return false;
	}

}


// function to return true if obstacle and false if not
bool GraphNode::is_Obstacle() {
	if (isObstacle) {
		return true;
	}
	else {
		return false;
	}
}

// function to return true if unknown and false if not
bool GraphNode::is_Unknown() {
	if (isUnknown) {
		return true;
	}
	else {
		return false;
	}
}

}