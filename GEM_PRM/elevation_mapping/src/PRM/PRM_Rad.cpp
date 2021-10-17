#include<cstdlib>
#include<time.h>
#include<chrono>
#include<math.h>
#include "elevation_mapping/PRM/PRM_Rad.h"

namespace topological_mapping {

using namespace cv;
using namespace std::chrono;

PRM_Rad::PRM_Rad(int interval, int xmax, int ymax, float xcenter, float ycenter, int cellWidth, int cellHeight, double radius)
	:xmax_(xmax), ymax_(ymax), network(), radius_(radius), interval_(interval),
	cellWidth_(cellWidth), cellHeight_(cellHeight), xcenter_(xcenter), ycenter_(ycenter),
	occupancy_grid(xmax, ymax, xcenter, ycenter, cellWidth, cellHeight)
{
	x_numNodes = occupancy_grid.DiscretizedGrid.size();
	y_numNodes = occupancy_grid.DiscretizedGrid[0].size();
	count = 0;
	// auto compute sample points
	numIterations_ = ceil(x_numNodes * y_numNodes / pow(interval, 2));

	GraphNode* current = &occupancy_grid.DiscretizedGrid[occupancy_grid.X_num_nodes / 2][occupancy_grid.Y_num_nodes / 2];
	
	// add origin to the graph
	network.AddVertex(*current);

	cv::Mat grid(occupancy_grid.DiscretizedGrid[0].size(), occupancy_grid.DiscretizedGrid.size(), CV_8UC3, cv::Scalar(0,0,0));
	image = grid.clone();
	obstacles = grid.clone();
}

PRM_Rad::PRM_Rad(){}

PRM_Rad::PRM_Rad(const PRM_Rad &prm){      // copy constructor
	network = prm.network;
	xmax_ = prm.xmax_;
	ymax_ = prm.ymax_;
	xcenter_ = prm.xcenter_;
	ycenter_ = prm.ycenter_;
	cellWidth_ = prm.cellWidth_;
	cellHeight_ = prm.cellHeight_;
	interval_ = prm.interval_;
	numIterations_ = prm.numIterations_;
	x_numNodes = prm.x_numNodes;
	y_numNodes = prm.y_numNodes;
	radius_ = prm.radius_;
}

PRM_Rad::~PRM_Rad(){}


void PRM_Rad::findFrontiers(int x, int y){

	occupancy_grid.findFrontiers(x, y);
}


void PRM_Rad::changePRMCenter(float xcenter, float ycenter){
	xcenter_ = xcenter;
	ycenter_ = ycenter;
}


void PRM_Rad::remapPRM(float xcenter, float ycenter){

	for (int vertex = 0; vertex < network.Vertices.size(); vertex++) {
		network.Vertices[vertex].xcood += xcenter;
		network.Vertices[vertex].ycood += ycenter;
	}
}


void PRM_Rad::remapPRM(){

	for (int vertex = 0; vertex < network.Vertices.size(); vertex++) {
		network.Vertices[vertex].xcood += xcenter_;
		network.Vertices[vertex].ycood += ycenter_;
	}
}


void PRM_Rad::incrementPRM(int x_index, int y_index){

	// findFrontiers(occupancy_grid.X_num_nodes/2, occupancy_grid.Y_num_nodes/2);
	findFrontiers(x_index, y_index);

	for (int j = 0; j < occupancy_grid.DiscretizedGrid.size(); j++) {
		for (int i = 0; i < occupancy_grid.DiscretizedGrid[0].size(); i++) {
			// we will check if the coordinate is present in the current cell
			if (occupancy_grid.DiscretizedGrid[j][i].isObstacle && !occupancy_grid.DiscretizedGrid[j][i].isUnknown) {
				obstacles.at<cv::Vec3b>(i, j)[0] = 255;
				obstacles.at<cv::Vec3b>(i, j)[1] = 255;
				obstacles.at<cv::Vec3b>(i, j)[2] = 255;
			}else if(occupancy_grid.DiscretizedGrid[j][i].isUnknown){
				obstacles.at<cv::Vec3b>(i, j)[0] = 128;
				obstacles.at<cv::Vec3b>(i, j)[1] = 128;
				obstacles.at<cv::Vec3b>(i, j)[2] = 128;
			}else if(occupancy_grid.DiscretizedGrid[j][i].isFrontier){
				obstacles.at<cv::Vec3b>(i, j)[0] = 0;
				obstacles.at<cv::Vec3b>(i, j)[1] = 255;
				obstacles.at<cv::Vec3b>(i, j)[2] = 0;
			}else{
				obstacles.at<cv::Vec3b>(i, j)[0] = 0;
				obstacles.at<cv::Vec3b>(i, j)[1] = 0;
				obstacles.at<cv::Vec3b>(i, j)[2] = 0;
			}
		}
	}

	// if(interval_ == 1 || !(x_index % interval_) || !(y_index % interval_))
	// 	return;

	GraphNode* current = nullptr;

	if (!occupancy_grid.DiscretizedGrid[x_index][y_index].is_Obstacle() && !occupancy_grid.DiscretizedGrid[x_index][y_index].is_Unknown()) {
		current = &occupancy_grid.DiscretizedGrid[x_index][y_index];
		
		// for (int x = x_index - 1; x <= x_index + 1; x++) {
		// 	for (int y = y_index - 1; y <= y_index + 1; y++) {
		// 		if ((x >= 0 && x < occupancy_grid.DiscretizedGrid.size()) && (y >= 0 && y < occupancy_grid.DiscretizedGrid[x_index].size())) {
		// 			if(occupancy_grid.DiscretizedGrid[x][y].isFrontier)
		// 				current->setFrontier();
		// 		}
		// 	}
		// }
	}

	if (current != nullptr) {
		Queue distance_heap;
		distance_heap = FindDistance(*current);

		if(!checkDuplicateNode(distance_heap) && distance_heap.size() > 0){
			network.AddVertex(*current);
			ConnectNeighbours(distance_heap, *current);
		}
	}
}


void PRM_Rad::generatePRM(){

	Queue distance_heap;
	GraphNode* current = &occupancy_grid.DiscretizedGrid[occupancy_grid.X_num_nodes / 2][occupancy_grid.Y_num_nodes / 2];
	
	// add origin to the graph
	network.AddVertex(*current);
	srand(time(0));

	findFrontiers(occupancy_grid.X_num_nodes/2, occupancy_grid.Y_num_nodes/2);

	for (int x_index = 0; x_index < occupancy_grid.DiscretizedGrid.size(); x_index++) {
		for (int y_index = 0; y_index < occupancy_grid.DiscretizedGrid[0].size(); y_index++) {
			// we will check if the coordinate is present in the current cell
			if (occupancy_grid.DiscretizedGrid[x_index][y_index].isObstacle && !occupancy_grid.DiscretizedGrid[x_index][y_index].isUnknown) {
				image.at<cv::Vec3b>(y_index, x_index)[0] = 255;
				image.at<cv::Vec3b>(y_index, x_index)[1] = 255;
				image.at<cv::Vec3b>(y_index, x_index)[2] = 255;
				obstacles.at<cv::Vec3b>(y_index, x_index)[0] = 255;
				obstacles.at<cv::Vec3b>(y_index, x_index)[1] = 255;
				obstacles.at<cv::Vec3b>(y_index, x_index)[2] = 255;
			}else if(occupancy_grid.DiscretizedGrid[x_index][y_index].isUnknown){
				image.at<cv::Vec3b>(y_index, x_index)[0] = 128;
				image.at<cv::Vec3b>(y_index, x_index)[1] = 128;
				image.at<cv::Vec3b>(y_index, x_index)[2] = 128;
				obstacles.at<cv::Vec3b>(y_index, x_index)[0] = 128;
				obstacles.at<cv::Vec3b>(y_index, x_index)[1] = 128;
				obstacles.at<cv::Vec3b>(y_index, x_index)[2] = 128;
			}else if(occupancy_grid.DiscretizedGrid[x_index][y_index].isFrontier){
				obstacles.at<cv::Vec3b>(y_index, x_index)[0] = 0;
				obstacles.at<cv::Vec3b>(y_index, x_index)[1] = 255;
				obstacles.at<cv::Vec3b>(y_index, x_index)[2] = 0;
			}else{
				obstacles.at<cv::Vec3b>(y_index, x_index)[0] = 0;
				obstacles.at<cv::Vec3b>(y_index, x_index)[1] = 0;
				obstacles.at<cv::Vec3b>(y_index, x_index)[2] = 0;
			}
		}
	}

	int iteration = 0;

	// repeat numIterations times
	while (iteration < numIterations_) {
		
		// generate a random node
		current = getUniformNode(x_numNodes, y_numNodes, iteration);
		// current = getRandomNode(x_numNodes, y_numNodes);

		// if the current node is not an obstacle
		if (current != nullptr) {
			distance_heap = FindDistance(*current);

			network.AddVertex(*current);
			ConnectNeighbours(distance_heap, *current);
		}
		iteration++;
	}
	cout << "search iters " << count << endl;

}


bool PRM_Rad::ConnectNeighbours(Queue& Heap, GraphNode& Point) {
	double distance = Heap.top().first;
	int connectionIndex = Heap.top().second;
	int currentIndex = network.numVertices - 1;
	bool collision = false;
	bool connected = false;

	while (!Heap.empty()) {
		distance = Heap.top().first;
		
		// check if the distance is more than the radius
		if (distance > radius_) {
			// Queue empty;
			// swap(empty, Heap);
			return connected;
		}

		// image.at<cv::Vec3b>(floor((Point.ycood) * 100.0 / (double)cellHeight_ + (double)occupancy_grid.Y_num_nodes / 2.0), floor((Point.xcood) * 100.0 / (double)cellWidth_ + (double)occupancy_grid.X_num_nodes / 2.0))[0] = 255;
		// image.at<cv::Vec3b>(floor((Point.ycood) * 100.0 / (double)cellHeight_ + (double)occupancy_grid.Y_num_nodes / 2.0), floor((Point.xcood) * 100.0 / (double)cellWidth_ + (double)occupancy_grid.X_num_nodes / 2.0))[1] = 0;
		// image.at<cv::Vec3b>(floor((Point.ycood) * 100.0 / (double)cellHeight_ + (double)occupancy_grid.Y_num_nodes / 2.0), floor((Point.xcood) * 100.0 / (double)cellWidth_ + (double)occupancy_grid.X_num_nodes / 2.0))[2] = 0;

		connectionIndex = Heap.top().second;
		Heap.pop();

		collision = collisionCheck(Point, network.Vertices[connectionIndex]);
		count ++;

		if (!collision) {
			network.AddEdge(currentIndex, connectionIndex, distance);
			network.AddEdge(connectionIndex, currentIndex, distance);
			
			connected = true;
			// image.at<cv::Vec3b>(floor((Point.ycood) * 100.0 / (double)cellHeight_ + (double)occupancy_grid.Y_num_nodes / 2.0), floor((network.Vertices[connectionIndex].xcood)* 100.0 / (double)cellWidth_ + (double)occupancy_grid.X_num_nodes / 2.0))[0] = 0;
			// image.at<cv::Vec3b>(floor((Point.ycood) * 100.0 / (double)cellHeight_ + (double)occupancy_grid.Y_num_nodes / 2.0), floor((network.Vertices[connectionIndex].xcood)* 100.0 / (double)cellWidth_ + (double)occupancy_grid.X_num_nodes / 2.0))[1] = 255;
			// image.at<cv::Vec3b>(floor((Point.ycood) * 100.0 / (double)cellHeight_ + (double)occupancy_grid.Y_num_nodes / 2.0), floor((network.Vertices[connectionIndex].xcood)* 100.0 / (double)cellWidth_ + (double)occupancy_grid.X_num_nodes / 2.0))[2] = 0;

			// cv::Point p1((network.Vertices[connectionIndex].xcood) * 100 / cellWidth_ + occupancy_grid.X_num_nodes / 2, (network.Vertices[connectionIndex].ycood) * 100 / cellHeight_ + occupancy_grid.Y_num_nodes / 2);  
			// cv::Point p2((Point.xcood) * 100 / cellWidth_ + occupancy_grid.X_num_nodes / 2, (Point.ycood) * 100 / cellHeight_ + occupancy_grid.Y_num_nodes / 2);    
			// cv::line(image, p1, p2, Scalar(0, 0, 255), 1);
		}
	}
	return connected;
}


// function to check if obstacle is present in between two nodes
inline bool PRM_Rad::collisionCheck(GraphNode& node1, GraphNode& node2) {
	// inputs are two grid nodes. we need to use the Obstacle list in Grid2D object for this. 

	// step 1: create equation of line
	double A = 0, B = 0, C = 0;
	createLineEquation(A, B, C, node1, node2);
	bool collision = false;
	
	// step 2: check line collision
	for (int obstacle_number = 0; obstacle_number < occupancy_grid.ObstacleList.size(); obstacle_number++) {
		collision = checkDistanceMetric(A, B, C, node1, node2, occupancy_grid.ObstacleList[obstacle_number]);
		if (collision == true) {
			return true;
		}
	}

	return collision;
}


inline void PRM_Rad::createLineEquation(double& A, double& B, double& C, GraphNode& node1, GraphNode& node2) {
	// line equation is Ax - By + C = 0
	// where A is x2-x1, B is y2-y1, C is x2y1 - x1y2 
	A = node2.ycood - node1.ycood;
	B = node2.xcood - node1.xcood;
	C = (node2.xcood * node1.ycood) - (node1.xcood * node2.ycood);
}


inline bool PRM_Rad::checkDistanceMetric(double& A, double& B, double& C, GraphNode& node1, GraphNode& node2, Obstacle& square) {
	double D1 = 0, D2 = 0, D3 = 0, D4 = 0;
	D1 = normDistance(A, B, C, square.xmin, square.ymin);
	D2 = normDistance(A, B, C, square.xmin, square.ymax);
	D3 = normDistance(A, B, C, square.xmax, square.ymax);
	D4 = normDistance(A, B, C, square.xmax, square.ymin);

	if(!checkInBox(node1, node2, square))
		return false;

	if ((checkInBox(node1, node2, square) && (D1 >= 0 && D2 >= 0 && D3 >= 0 && D4 >= 0)) 
		|| (checkInBox(node1, node2, square) && (D1 <= 0 && D2 <= 0 && D3 <= 0 && D4 <= 0))) {
		return false;
	}
	else {
		return true;
	}
}


inline bool PRM_Rad::checkInBox(GraphNode& node1, GraphNode& node2, Obstacle& square){
	if(node1.xcood <= node2.xcood && node1.ycood <= node2.ycood && square.xmin <= node2.xcood && square.xmax >= node1.xcood 
		&& square.ymin <= node2.ycood && square.ymax >= node1.ycood)
		return true;
	
	if(node1.xcood <= node2.xcood && node1.ycood >= node2.ycood && square.xmin <= node2.xcood && square.xmax >= node1.xcood 
		&& square.ymin <= node1.ycood && square.ymax >= node2.ycood)
		return true;

	if(node1.xcood >= node2.xcood && node1.ycood >= node2.ycood && square.xmin <= node1.xcood && square.xmax >= node2.xcood 
		&& square.ymin <= node1.ycood && square.ymax >= node2.ycood)
		return true;

	if(node1.xcood >= node2.xcood && node1.ycood <= node2.ycood && square.xmin <= node1.xcood && square.xmax >= node2.xcood 
		&& square.ymin <= node2.ycood && square.ymax >= node1.ycood)
		return true;
}


double PRM_Rad::normDistance(double& A, double& B, double& C, double& x, double& y) {
	// return the norm distance without the denominator
	return ((A * x) - (B * y) + (C));
}


// Function to get uniform Node
GraphNode* PRM_Rad::getUniformNode(int& x_numNodes, int& y_numNodes, int iter) {

	GraphNode* node_pointer = nullptr;
	int x_index = (iter % (x_numNodes / interval_)) * interval_;
	int y_index = (iter / (x_numNodes / interval_)) * interval_;

	if (!occupancy_grid.DiscretizedGrid[x_index][y_index].is_Obstacle() && !occupancy_grid.DiscretizedGrid[x_index][y_index].is_Unknown()) {
		node_pointer = &occupancy_grid.DiscretizedGrid[x_index][y_index];
				
		// for (int x = x_index - 1; x <= x_index + 1; x++) {
		// 	for (int y = y_index - 1; y <= y_index + 1; y++) {
		// 		if ((x >= 0 && x < occupancy_grid.DiscretizedGrid.size()) && (y >= 0 && y < occupancy_grid.DiscretizedGrid[x_index].size())) {
		// 			if(occupancy_grid.DiscretizedGrid[x][y].isFrontier)
		// 				node_pointer->setFrontier();
		// 		}
		// 	}
		// }
	}
	return node_pointer;
}


// Function to pick random Node
GraphNode* PRM_Rad::getRandomNode(int& x_numNodes, int& y_numNodes) {
	// function creates a random node.
	// It checks if the node is an obstacle. 
	// If the node is an obstacle, nullptr pointer is returned.
	// If the node is not an obstacle, pointer to a node is returned.

	GraphNode* node_pointer = nullptr;
	int y_index = rand() % y_numNodes;
	int x_index = rand() % x_numNodes;
	if (!occupancy_grid.DiscretizedGrid[x_index][y_index].is_Obstacle()) {
		node_pointer = &occupancy_grid.DiscretizedGrid[x_index][y_index];
	}
	return node_pointer;
}


Queue PRM_Rad::FindDistance(GraphNode& Point) {
	// we will find the distance between each point in the Graph and the reference point and add to minHeap
	Queue Heap;
	double distance;
	for (int vertex = 0; vertex < network.Vertices.size(); vertex++) {
		distance = L2Distance(Point, network.Vertices[vertex]); // calculate L2 distance between ref point and current vertex
		Heap.push(make_pair(distance, vertex));
	}
	return Heap;
}


inline double PRM_Rad::L2Distance(GraphNode& Node1, GraphNode& Node2) {
	double distance = sqrt(pow((Node2.ycood - Node1.ycood), 2) + pow((Node2.xcood - Node1.xcood), 2));
	return distance;
}


inline bool PRM_Rad::checkDuplicateNode(Queue& Heap) {
	if (Heap.empty()) {
		return false;
	}
	if (Heap.top().first <= 0.000000005) {
		return true;
	}
	return false;
}

void PRM_Rad::reset(int xmax, int ymax, float xcenter, float ycenter, int cellWidth, int cellHeight){
	network.reset(xcenter, ycenter);
	GraphNode* current = &occupancy_grid.DiscretizedGrid[occupancy_grid.X_num_nodes / 2][occupancy_grid.Y_num_nodes / 2];
	
	// add origin to the graph
	network.AddVertex(*current);

	count = 0;
	xcenter_ = xcenter;
	ycenter_ = ycenter;
	occupancy_grid.reset(xmax, ymax, xcenter, ycenter, cellWidth, cellHeight);
	
	// init visualization
	cv::Mat grid(occupancy_grid.DiscretizedGrid[0].size(), occupancy_grid.DiscretizedGrid.size(), CV_8UC3, cv::Scalar(0,0,0));
	image = grid.clone();
	obstacles = grid.clone();
}

}