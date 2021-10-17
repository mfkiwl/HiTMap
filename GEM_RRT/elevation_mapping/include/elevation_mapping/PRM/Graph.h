#pragma once

#include "elevation_mapping/PRM/Grid2D.h"

namespace topological_mapping {

typedef pair<int, double> pid;
typedef pair<double,int> pdi;

class Graph
{
public:
	vector<GraphNode> Vertices;
	vector<vector<pid>> Adjacency_list;
	vector<vector<pid>> Reverse_list;
	float xcenter_, ycenter_;
	int numVertices;
	
	// Default Constructor
	Graph();
	// Copy constructor
	Graph(const Graph &graph); 

	// Add Edge to the Graph
	void AddEdge(int source, int dest, double weight);
	void AddVertex(GraphNode& Node);
	void RemoveLastVertex();
	void reset(float xcenter, float ycenter);
	void remapGraph();
	void recoverGraph();
};

}