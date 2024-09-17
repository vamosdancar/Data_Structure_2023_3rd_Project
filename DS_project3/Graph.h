#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <iostream>
#include <cstring>
#include <string.h>
#include <fstream>
#include <map>
#include <set>
#include <math.h>
#include <vector>
#include <algorithm>
#include <deque>
#include <queue>
#include <stack>

using namespace std;

class Graph {	
protected:
	bool m_Type; // Type of the graph 
	int m_Size;  // Size of the graph 

public:
	// Constructor for Graph
	Graph(bool type, int size);

	// Virtual destructor for Graph
	virtual ~Graph();

	// Function to get the type 
	bool getType();

	// Function to get the size 
	int getSize();

	// Pure virtual function to get adjacent edges 
	virtual void getAdjacentEdges(int vertex, map<int, int>* m) = 0;

	// Pure virtual function to get direct adjacent edges 
	virtual void getAdjacentEdgesDirect(int vertex, map<int, int>* m) = 0;

	// Pure virtual function to insert an edge 
	virtual void insertEdge(int from, int to, int weight) = 0;

	// print the graph
	virtual bool printGraph(ofstream *fout) = 0;

	//to get the weight of an edge
    virtual int getWeight(int from, int to) = 0;

	//to get adjacent vertices of a given vertex
	vector<int> getAdjacentVertices(int vertex) {
		vector<int> adjVertices;
		map<int, int> adjEdges;
		getAdjacentEdges(vertex, &adjEdges); 

		// Add end points of each edge to the list of adjacent vertices
		for (auto& edge : adjEdges) {
			adjVertices.push_back(edge.first);
		}
		return adjVertices;
	}

	// Function to create an adjacency list 
    void makeAdjList(bool isDirected, vector<vector<pair<int, int>>>& adjList) {
        int V = getSize(); // Get the number of vertices
        adjList.resize(V + 1); // Resize the list 

        // Iterate through each vertex 
        for (int i = 1; i <= V; ++i) {
            map<int, int> adjEdges;
            if (isDirected) {
                getAdjacentEdgesDirect(i, &adjEdges); // Get direct edges 
            } else {
                getAdjacentEdges(i, &adjEdges); // Get edges 
            }

            // Add each adjacent edge to the adjacency list
            for (const auto& edge : adjEdges) {
                int adjVertex = edge.first;
                int weight = edge.second;
                adjList[i].push_back(make_pair(adjVertex, weight));
            }
        }
    }
};

#endif
