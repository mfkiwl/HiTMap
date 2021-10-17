#pragma once

#include "elevation_mapping/PRM/GraphNode.h"

namespace topological_mapping {

class Obstacle
{
public:
	double xmin, xmax, ymin, ymax;
	
	Obstacle() :xmin(100000), xmax(-50000), ymin(10000), ymax(-50000) {};
	
	void UpdateCoordinates(GraphNode& CurrentNode) {
		// The function is to update the minimum and maximum values of an obstacle.
		xmin = (CurrentNode.xlow < xmin) ? CurrentNode.xlow : xmin;
		xmax = (CurrentNode.xhigh > xmax) ? CurrentNode.xhigh : xmax;
		ymin = (CurrentNode.ylow < ymin) ? CurrentNode.ylow : ymin;
		ymax = (CurrentNode.yhigh > ymax) ? CurrentNode.yhigh : ymax;
		// The four end points are (ymin,xmin),(ymin,xmax),(ymax,xmax),(ymax,xmin)
	}
};

}