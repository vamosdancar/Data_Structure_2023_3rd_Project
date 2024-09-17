#include <iostream>
#include <vector>
#include "GraphMethod.h"
#include <stack>
#include <queue>
#include <map>
#include <set>
#include <fstream>
#include <list>
#include <utility>
#include <climits>

using namespace std;

bool BFS(Graph* graph, char option, int startVertex) {
    if (graph == nullptr) return false; // Check if the graph is null

    map<int, bool> visited;
    queue<int> q;
    visited[startVertex] = true; // Mark the start vertex as visited
    q.push(startVertex); // Enqueue the start vertex

    // Start printing the result
    cout << "========== BFS ============" << endl;
    cout << (option == 'Y' ? "Directed" : "Undirected") << " Graph BFS result" << endl;

    while (!q.empty()) {
        int vertex = q.front(); // Get the front vertex
        q.pop(); // Dequeue the front vertex

        cout << vertex << " -> "; // Print the current vertex

        // Loop through all adjacent vertices
        for (int adjVertex : graph->getAdjacentVertices(vertex)) {
            if (!visited[adjVertex]) { //  not visited
                visited[adjVertex] = true; // Mark
                q.push(adjVertex); // Enqueue
            }
        }
    }

    cout << "\b\b  " << endl; // Remove the last arrow
    cout << "===========================" << endl; // End of BFS result

    return true;
}

bool DFS(Graph* graph, char option, int startVertex)
{
    if (graph == nullptr) return false; // Check if the graph is null

    map<int, bool> visited;
    stack<int> s;

    s.push(startVertex); // Push the start vertex 

    while (!s.empty()) {
        int vertex = s.top(); // Get the top vertex 
        s.pop(); // Pop the top vertex

        if (visited[vertex]) continue; // Skip if the vertex is already visited

        visited[vertex] = true; // Mark the vertex as visited
        cout << vertex << " "; // Print the current vertex

        
        // Inside the DFS function
        auto adjVertices = graph->getAdjacentVertices(vertex);
        sort(adjVertices.rbegin(), adjVertices.rend()); // Sort in descending order (processed in ascending order in stack)

        
        for (auto adjVertex : adjVertices) {
            if (!visited[adjVertex]) { // If the adjacent vertex 
                s.push(adjVertex); // Push the adjacent vertex
            }
        }
    }

    return true;
}


bool Kruskal(Graph* graph)
{
   if (!graph) {
        return false; // Check if the graph is null
    }

    int V = graph->getSize(); // Get the number of vertices 
    vector<int> parent(V + 1);
    int totalCost = 0;

    // Create a list of edges
    for (int i = 1; i <= V; i++) {
        for (int j = 1; j <= V; j++) {
            if (i != j) {
                int weight = graph->getWeight(i, j); // Get the weight 
            }
        }
    }
    // Initialize Union-Find
    for (int i = 1; i <= V; i++) {
        parent[i] = i; // Set each vertex 
    }
}


const int INF = std::numeric_limits<int>::max();

bool Dijkstra(Graph* graph, char option, int vertex)
{
    // Retrieve the adjacency list

    int V = graph->getSize(); // Get the number of vertices

    vector<vector<pair<int, int>>> adjList(V + 1);

    graph->makeAdjList(option, adjList); // Generate adjacency list

    vector<int> dist(V + 1, INF); // Initialize distances 
    vector<int> prev(V + 1, -1); // Initialize previous vertices
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq; // Min-heap priority queue

    dist[vertex] = 0; // Set the distance
    pq.push({0, vertex}); // Push the start vertex

    while (!pq.empty()) 
    {
        int currentDist = pq.top().first; // Current distance
        int currentNode = pq.top().second; // Current node
        pq.pop();

        if (dist[currentNode] < currentDist) continue; 

        
        for (auto& edge : adjList[currentNode]) {
            int nextNode = edge.first; // Next node
            int weight = edge.second; // Weight of the edge

            if (dist[nextNode] > currentDist + weight) {
                dist[nextNode] = currentDist + weight; // Update distance
                prev[nextNode] = currentNode; // Update previous vertex for path reconstruction
                pq.push({dist[nextNode], nextNode}); // Push next vertex onto the priority queue
            }
        }
    }
}


bool Bellmanford(Graph* graph, char option, int s_vertex, int e_vertex) 
{
    int V = graph->getSize(); // Get the number of vertices
    if (s_vertex < 1 || s_vertex > V || e_vertex < 1 || e_vertex > V) {
        return false; // Check if start and end vertices
    }
    vector<vector<pair<int, int>>> adjList(V + 1);
    graph->makeAdjList(option, adjList); // Generate adjacency list

    vector<int> prev(V + 1, -1); // Initialize previous vertices 

    // Edge relaxation
    for (int i = 0; i < V - 1; ++i) {
        for (int u = 1; u <= V; ++u) {
            for (const auto& edge : adjList[u]) {
                int v = edge.first; // Next vertex
                int weight = edge.second; // Weight of the edge

                // Relaxation logic goes here
            }
        }
    }

    // Detect negative cycles
    for (int u = 1; u <= V; ++u) {
        for (const auto& edge : adjList[u]) {
            int v = edge.first; // Next vertex
            int weight = edge.second; // Weight of the edge

            // Negative cycle
        }
    }
}

bool FLOYD(Graph* graph, char option)
{
    if (!graph) {
       return false; // Check the graph is null
    }
    int V = graph->getSize(); // Get the number of vertices in the graph
    vector<vector<int>> dist(V + 1, vector<int>(V + 1, INT_MAX));

    // Initialization: Fill the distance matrix with edge weights
    for (int i = 1; i <= V; ++i) {
        dist[i][i] = 0; // Distance
        for (int j = 1; j <= V; ++j) {
            int weight = graph->getWeight(i, j); // Get the weight of the edge
            if (weight != 0) {
                dist[i][j] = weight; // Set distance as edge weight if there is an edge
            }
        }
    }

    // Execute the Floyd-Warshall algorithm
    for (int k = 1; k <= V; ++k) {
        for (int i = 1; i <= V; ++i) {
            for (int j = 1; j <= V; ++j) {
                // Update the distance if a shorter path is found
                if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX && dist[i][j] > dist[i][k] + dist[k][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }

    // Check for negative cycles
    for (int i = 1; i <= V; ++i) {
        if (dist[i][i] < 0) {
            return false; // a negative cycle exists
        }
    }

    return true;
}


bool KWANGWOON(Graph* graph, int vertex) {

    if (graph->getSize() < 1 || graph->getAdjacentVertices(1).empty()) 
    {
        return false; // Check the graph is empty
    }

    vector<bool> visited(graph->getSize() + 1, false); // Initialize visited array
    int current = vertex; // Start from the given vertex
    visited[current] = true; // Mark the starting vertex as visited

}
