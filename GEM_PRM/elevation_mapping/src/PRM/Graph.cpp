#include "elevation_mapping/PRM/Graph.h"

namespace topological_mapping {

Graph::Graph():Adjacency_list(), Reverse_list(), numVertices(0), Vertices(0) 
{
	// Initialize an empty adjacency list 
	// later maybe can use a heap
}


Graph::Graph(const Graph &graph){      // copy constructor
	Vertices = graph.Vertices;
	Adjacency_list = graph.Adjacency_list;
	Reverse_list = graph.Reverse_list;	
	numVertices = graph.numVertices;
	xcenter_ = graph.xcenter_;
	ycenter_ = graph.ycenter_;
}

void Graph::AddEdge(int source, int dest, double weight) 
{
	// add the edge as the form index, weight to the corresponding source and dest
	Adjacency_list[source].push_back(make_pair(dest, weight));
	if(dest < numVertices)
		Reverse_list[dest].push_back(make_pair(source, weight));
}


void Graph::AddVertex(GraphNode& Node) 
{
	Vertices.push_back(Node);
	Adjacency_list.push_back({});
	Reverse_list.push_back({});
	numVertices++;
}


void Graph::RemoveLastVertex() 
{
	Vertices.erase(Vertices.end() - 1);
	Adjacency_list.erase(Adjacency_list.end()-1);
	Reverse_list.erase(Reverse_list.end()-1);
	numVertices--;
}

void Graph::reset(float xcenter, float ycenter) 
{
	vector<GraphNode> tmp;
	vector<vector<pid>> tmpList, tmpReverse;
	xcenter_ = xcenter;
	ycenter_ = ycenter;
	Vertices.swap(tmp);
	Adjacency_list.swap(tmpList);
	Reverse_list.swap(tmpReverse);
	numVertices = 0;
}

void Graph::remapGraph()
{
	for (int vertex = 0; vertex < Vertices.size(); vertex++) {
		Vertices[vertex].xcood += xcenter_;
		Vertices[vertex].ycood += ycenter_;
	}
}

void Graph::recoverGraph()
{
	for (int vertex = 0; vertex < Vertices.size(); vertex++) {
		Vertices[vertex].xcood -= xcenter_;
		Vertices[vertex].ycood -= ycenter_;
	}
}
}