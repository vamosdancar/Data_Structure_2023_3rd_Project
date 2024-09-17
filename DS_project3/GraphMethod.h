#ifndef _GRAPHMETHOD_H_
#define _GRAPHMETHOD_H_

#include "ListGraph.h"
#include "MatrixGraph.h"

// Performs Breadth-First Search
bool BFS(Graph* graph, char option, int vertex);     

// Performs Depth-First Search
bool DFS(Graph* graph, char option,  int vertex);     

//  KwangWoon University project
bool KWANGWOON(Graph* graph, int vertex);  

// Performs Kruskal's algorithm
bool Kruskal(Graph* graph);

// Implements Dijkstra's algorithm 
bool Dijkstra(Graph* graph, char option, int vertex);    

// Implements the Bellman-Ford algorithm
bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex); 

// Implements Floyd-Warshall algorithm 
bool FLOYD(Graph* graph, char option);   

#endif
