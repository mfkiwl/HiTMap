#pragma once

#include<iostream>
#include<vector>
#include<algorithm>
#include<queue>
#include "elevation_mapping/PRM/GraphNode.h"
#include "elevation_mapping/PRM/Obstacle.h"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

using std::vector;
using std::pair;
using std::priority_queue;

namespace topological_mapping {

using namespace elevation_mapping;

class Grid2D
{
public:
	vector<vector<GraphNode>> DiscretizedGrid;
	vector<vector<GraphNode>> tmpMap;
	vector<Obstacle> ObstacleList;
	int Vehicle_size_x, Vehicle_size_y;	// in cm
	int X_num_nodes, Y_num_nodes;
	float xcenter_, ycenter_;	// absolute location of the center of the grid
	
	int dx[8] = {0,0,-1,1,-1,-1,1,1};
	int dy[8] = {1,-1,0,0,-1,1,-1,1};

	// default constructor
	Grid2D();

	// copy constructor
	Grid2D(const Grid2D &grid2D); 

	// constructor with x and y fov and mesh size
	Grid2D(int xmax, int ymax, float xcenter, float ycenter, int cellWidth = 10, int cellHeight = 10);
	
	// Set Vehicle_size parameter in cm
	void SetVehicleSize(int vehicleSizeX, int vehicleSizeY);

	// Consider creating an object called obstacle holding a sorted vector of coordinates.
	// One method to inflate cells the size of the robot
	void InflateObstacle(int i, int j);
	void InflateFrontier(int x_index, int y_index);
	
	// viewing function
	void ViewNodeDetails(int index1, int index2);

	// reset
	void reset(int xmax, int ymax, float xcenter, float ycenter, int cellWidth, int cellHeight);

	// find frontiers
	void findFrontiers(int x, int y);

	inline void dfs(int x, int y);

};

}