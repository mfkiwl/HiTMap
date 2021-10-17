#include "elevation_mapping/PRM/Grid2D.h"
#include<chrono>


namespace topological_mapping {

typedef pair<double,double> pd;
using namespace std::chrono;


// default constructor
Grid2D::Grid2D() {
	// The function should create a 2D vector of graph Nodes.
	// The size of the graph nodesis currently a 2x3m grid
	// The size of each cell is 10cm. Hence we will require a 30x20 grid (y*x)

	vector<vector<GraphNode>> temp(30);
	
	double x_cood = -0.05, y_cood = 0; // -0.05 as the first element will become 0.5 in the for loop
	
	for (int x_index = 0; x_index < 30; x_index++) {
		y_cood = -1.05;
		x_cood += 0.1; // convert coordinate to meters
		for (int y_index = 0; y_index < 20; y_index++) {
			// we have divided by 20,so that we have the coordinate of the centroid
			y_cood += 0.1;
			temp[x_index].push_back(GraphNode(x_cood, y_cood));
		}
	}

	DiscretizedGrid = temp;
	Vehicle_size_x = 20;
	Vehicle_size_y = 20;
	vector<Obstacle> temp1;
	ObstacleList = temp1;
	
}


Grid2D::Grid2D(int xmax, int ymax, float xcenter, float ycenter, int cellWidth, int cellHeight) {
	// xmax ymax xcenter ycenter in meters 
	// cellWidth and cellHeight in cm
	// check default constructor for more details

	xcenter_ = xcenter;
	ycenter_ = ycenter;
	double x_cood = -(double)cellWidth / 200 - ((double)xmax / 2);// + xcenter_; 
	double y_cood = -(double)cellHeight / 200;// + ycenter_;
	int num_x = floor((double)xmax / ((double)cellWidth / 100.0)) + 1;   // number of cells in x
	int num_y = floor((double)ymax / ((double)cellHeight / 100.0)) + 1;  // number of cells in y
	X_num_nodes = num_x;
	Y_num_nodes = num_y;

	cout << "X_num_nodes " << X_num_nodes << " Y_num_nodes " << Y_num_nodes << endl;

	vector<vector<GraphNode>> temp(num_x);
	for (int x_index = 0; x_index < num_x; x_index++) {
		y_cood = (-(double)cellHeight/ 200) - ((double)ymax / 2);// + ycenter_;  // to shift the axis from -ymax/2 to ymax/2
		x_cood += ((double)(cellWidth) / 100); // convert to meters 
		for (int y_index = 0; y_index < num_y; y_index++) {
			y_cood += ((double)cellHeight / 100);
			temp[x_index].push_back(GraphNode(x_cood, y_cood, cellWidth, cellHeight));
		}
	}

	DiscretizedGrid = temp;
	Vehicle_size_x = 40; // in cm
	Vehicle_size_y = 40;
	vector<Obstacle> temp1;
	ObstacleList = temp1;
}


Grid2D::Grid2D(const Grid2D &grid2D){
	DiscretizedGrid = grid2D.DiscretizedGrid;
	ObstacleList = grid2D.ObstacleList;
	Vehicle_size_x = grid2D.Vehicle_size_x;
	Vehicle_size_y = grid2D.Vehicle_size_y;
	X_num_nodes = grid2D.X_num_nodes;
	Y_num_nodes = grid2D.Y_num_nodes;
	xcenter_ = grid2D.xcenter_;
	ycenter_ = grid2D.ycenter_;
}


void Grid2D::ViewNodeDetails(int x_index, int y_index) {
	
	DiscretizedGrid[x_index-1][y_index-1].getCellParam();// convert to 0 based indexing
}


void Grid2D::InflateObstacle(int x_index, int y_index) {
	// we must make a circle of radius 20cm, i.e 2 grid
	int padding_x = ceil(Vehicle_size_x/DiscretizedGrid[x_index][y_index].getWidthHeight().first);
	int padding_y = ceil(Vehicle_size_y/DiscretizedGrid[x_index][y_index].getWidthHeight().second);
	Obstacle current;
	
	for (int x = x_index - padding_x; x <= x_index + padding_x; x++) {
		for (int y = y_index - padding_y; y <= y_index + padding_y; y++) {
			if ((x >= 0 && x < DiscretizedGrid.size()) && (y >= 0 && y < DiscretizedGrid[x_index].size())) {
				DiscretizedGrid[x][y].setObstacle();
				current.UpdateCoordinates(DiscretizedGrid[x][y]);
			}
		}
	}
	ObstacleList.push_back(current);
}


// set vehicle size param
void Grid2D::SetVehicleSize(int vehicleSizeX, int vehicleSizeY){
	Vehicle_size_x = vehicleSizeX;
	Vehicle_size_y = vehicleSizeY;
}

void Grid2D::reset(int xmax, int ymax, float xcenter, float ycenter, int cellWidth, int cellHeight){
	vector<Obstacle> tmpObstacle;
	ObstacleList.swap(tmpObstacle);

	xcenter_ = xcenter;
	ycenter_ = ycenter;
	double x_cood = -(double)cellWidth / 200 - ((double)xmax / 2);// + xcenter_; 
	double y_cood = -(double)cellHeight / 200;// + ycenter_;

	vector<vector<GraphNode>> temp(X_num_nodes);
	for (int x_index = 0; x_index < X_num_nodes; x_index++) {
		y_cood = (-(double)cellHeight/ 200) - ((double)ymax / 2);// + ycenter_;  // to shift the axis from -ymax/2 to ymax/2
		x_cood += ((double)(cellWidth) / 100); // convert to meters 
		for (int y_index = 0; y_index < Y_num_nodes; y_index++) {
			y_cood += ((double)cellHeight / 100);
			temp[x_index].push_back(GraphNode(x_cood, y_cood, cellWidth, cellHeight));
		}
	}

	DiscretizedGrid = temp;
	Vehicle_size_x = 40; // in cm
	Vehicle_size_y = 40;
	vector<Obstacle> temp1;
	ObstacleList = temp1;
}


// void Grid2D::InflateFrontier(int x_index, int y_index) {
// 	// we must make a circle of radius 20cm, i.e 2 grid
// 	int padding_x = 1;	
// 	int padding_y = 1;
	
// 	for (int x = x_index - padding_x; x <= x_index + padding_x; x++) {
// 		for (int y = y_index - padding_y; y <= y_index + padding_y; y++) {
// 			if ((x >= 0 && x < DiscretizedGrid.size()) && (y >= 0 && y < DiscretizedGrid[x_index].size())) {
// 				DiscretizedGrid[x][y].setFrontier();
// 			}
// 		}
// 	}
// }


// find frontiers
void Grid2D::findFrontiers(int x, int y){
	
	tmpMap = DiscretizedGrid;

	for (int x_index = 0; x_index < X_num_nodes; x_index++) {
		for (int y_index = 0; y_index < Y_num_nodes; y_index++) {
			tmpMap[x_index][y_index].isUnknown = true;
			if(DiscretizedGrid[x_index][y_index].isFrontier)
				DiscretizedGrid[x_index][y_index].releaseFrontier();
		}
	}

	dfs(x, y);
}


// find frontiers
inline void Grid2D::dfs(int x, int y){
	
	if(!DiscretizedGrid[x][y].isUnknown){// && !DiscretizedGrid[x][y].isObstacle){
		tmpMap[x][y].isUnknown = false;

		for(int j = 0; j < 4; j++){
			int xx = x + dx[j];
			int yy = y + dy[j];

			if(xx >= 0 && yy >= 0 && xx < X_num_nodes && yy < Y_num_nodes){
				if(!DiscretizedGrid[xx][yy].isUnknown && !DiscretizedGrid[xx][yy].isObstacle && tmpMap[xx][yy].isUnknown){
					dfs(xx, yy);
				}else if(DiscretizedGrid[xx][yy].isUnknown && tmpMap[xx][yy].isUnknown){
					DiscretizedGrid[x][y].setFrontier();
					// InflateFrontier(x, y);
				}else if(!DiscretizedGrid[xx][yy].isUnknown && !DiscretizedGrid[xx][yy].isObstacle && (xx == 0 || yy == 0 || xx == X_num_nodes - 1 || yy == Y_num_nodes - 1)){
					DiscretizedGrid[xx][yy].setFrontier();
				}
			}
		}
	}

}


}