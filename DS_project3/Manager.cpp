#include "Manager.h"
#include "GraphMethod.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <climits>


Manager::Manager()  
{
    graph = nullptr;    
    fout.open("log.txt", ios::app);
    load = 0;   //Anything is not loaded
}

Manager::~Manager()
{
    if(load)    //if graph is loaded, delete graph
        delete graph;   
    if(fout.is_open())  //if fout is opened, close file
        fout.close();   //close log.txt File
}

void Manager::run(const char* command_txt)
{
    ifstream fin;   // Command File Input File Stream
    fin.open(command_txt, ios_base::in); // Open file in read mode
        
    if(!fin) 
    { 
        // If the command file cannot be read, print an error message
        fout << "command file open error" << endl;
        return; // Exit the function
    }

    string line;

    while(getline(fin, line)) 
    {
        stringstream ss(line);
        string command;
        ss >> command;

        if(command == "LOAD") 
        {
            string filename;
            ss >> filename;
            LOAD(filename.c_str()); // Call LOAD function with the filename
        }
        else if (command == "PRINT")
        {
            PRINT(); // Call PRINT function
        }
        else if (command == "BFS") 
        {
            char option;
            int vertex;
            ss >> option;
            if (ss.fail()) {
                printErrorCode(300); // Error: Option not provided
                continue;
            }

            if (option != 'Y' && option != 'N') {
                printErrorCode(300); // Error: Invalid option
                continue;
            }

            // Perform different actions based on option value
            if (option == 'N') {
                if (!(ss >> vertex)) {
                    printErrorCode(300); // Error: Vertex not provided
                    continue;
                }
            } else {
                if (ss.fail() || !(ss >> vertex)) {
                    printErrorCode(300); // Error: Vertex not provided
                    continue;
                }
            }

            mBFS(option, vertex); // Call mBFS function
        }
        else if (command == "DFS") 
        {
            char option;
            int vertex;
            ss >> option;
            if (ss.fail()) {
                printErrorCode(400); // Error: Option not provided
                continue;
            }

            if (option != 'Y' && option != 'N') {
                printErrorCode(400); // Error: Invalid option
                continue;
            }

            // Perform different actions based on option value
            if (option == 'N') {
                if (!(ss >> vertex)) {
                    printErrorCode(400); // Error: Vertex not provided
                    continue;
                }
            } else {
                if (ss.fail() || !(ss >> vertex)) {
                    printErrorCode(400); // Error: Vertex not provided
                    continue;
                }
            }

            mDFS(option, vertex); // Call mDFS function
        }
        else if (command == "KRUSKAL")
        {
            mKRUSKAL(); // Call mKRUSKAL function
        }
        else if (command == "BELLMANFORD") 
        {
            char option;
            int startVertex, endVertex;
            ss >> option >> startVertex >> endVertex;

            if (ss.fail()) {
                printErrorCode(800); 
                continue;
            }

            mBELLMANFORD(option, startVertex, endVertex); // Call mBELLMANFORD function
        }
        else if (command == "KWANGWOON")
        {
            mKwoonWoon(1); // Call mKwoonWoon function
        }
        else if (command == "DIJKSTRA") 
        {
            char option;
            int startVertex;

            ss >> option >> startVertex;

            if (ss.fail()) 
            {
             printErrorCode(700);
                continue;
            }

            mDIJKSTRA(option, startVertex); // Call mDIJKSTRA function
        }
        else if (command == "FLOYD") 
        {
            char option;
            ss >> option;

            if (ss.fail()) 
            {
                printErrorCode(900);
                continue;
            }
            
            mFLOYD(option); // Call mFLOYD function
        }
        else if (command == "EXIT") 
        {
            break;  // Exit the while loop
        }
    }

    fin.close(); // Close the file stream

    // Memory cleanup
    if (graph) {
        delete graph;
        graph = nullptr;
    }
    return;
}


bool Manager::LOAD(const char* filename)
{
    ifstream fin(filename); // Open file

    // Error handling if file opening fails
    if (!fin) {
        printErrorCode(100); // Print error code to log.txt
        return false; // Load failed
    }

    // Check if the file is empty
    fin.seekg(0, ios::end);
    if (fin.tellg() == 0) {
        printErrorCode(100); // Print error code to log.txt
        fin.close();
        return false; // File is empty
    }
    fin.seekg(0, ios::beg);

    // Delete existing graph
    if (load) {
        delete graph;
        graph = nullptr;
    }

    char graphType;
    fin >> graphType; // Read the first character for graph type
    fin.ignore(INT_MAX, '\n'); // Skip to the end of line

    int size;
    fin >> size; // Read graph size
    fin.ignore(INT_MAX, '\n'); // Skip to the end of line
    
    if (graphType == 'L') {
        graph = new ListGraph(true, size); // Create ListGraph object

        string line;
        int from = -1;  // Initialize the current 'from' vertex
        while (getline(fin, line)) {
            stringstream ss(line);
            int firstValue;
            ss >> firstValue;

            if (ss.fail()) {
                continue; // Ignore errors or empty lines
            }

            if (ss.eof()) {  // If there's only one integer, it's a new 'from' vertex
                from = firstValue;
            } else {  // If there are two integers, it's edge information
                int to = firstValue;
                int weight;
                ss >> weight;
                
                graph->insertEdge(from, to, weight); // Add edge to the graph
            }
        }
    }
    else if (graphType == 'M') {
        graph = new MatrixGraph(true, size); // Create MatrixGraph object

        for (int from = 1; from <= size; from++) {
            for (int to = 1; to <= size; to++) {
                int weight;
                fin >> weight;
                if (weight > 0) {
                    graph->insertEdge(from, to, weight); // Add edge to the graph
                }
            }
        }
    } else {
        printErrorCode(200); // Error for incorrect type
        return false;
    }

    load = 1; // Update graph load status

    // Additional logic to load graph data
    // Example: Read edge data from file, add edges to the graph object

    fin.close(); // Close file stream

    // Print success message
    fout << "======== LOAD ========" << endl;
    fout << "Success" << endl;
    fout << "======================" << endl;

    return true; // Load successful
}



bool Manager::PRINT()
{
    // Check if the graph exists
    if (!graph) {
        printErrorCode(200); // Print error code to log.txt
        return false;
    }

    // Call the appropriate print method based on graph type
    graph->printGraph(&fout); // Print the graph

    return true;
}



bool Manager::mBFS(char option, int vertex) {
    if (graph == nullptr) { // Check if the graph is loaded
        return false;
    }
    // Check if the vertex is within the valid range
    int V = graph->getSize();
    if (vertex < 1 || vertex > V) {
        printErrorCode(300); // Invalid vertex error
        return false;
    }

    std::map<int, bool> visited; // Map to keep track of visited vertices
    std::queue<int> q; // Queue for BFS
    std::vector<int> bfsResult; // Vector to store BFS result

    visited[vertex] = true; // Mark the starting vertex as visited
    q.push(vertex); // Enqueue the starting vertex

    while (!q.empty()) {
        int currentVertex = q.front(); // Get the front vertex from the queue
        q.pop(); // Dequeue the front vertex

        bfsResult.push_back(currentVertex); // Store the current vertex in the result vector

        // Get adjacent vertices and sort them
        map<int, int> adjEdges;
        if (option == 'Y') {
            graph->getAdjacentEdgesDirect(currentVertex, &adjEdges); // For directed graphs
        } else {
            graph->getAdjacentEdges(currentVertex, &adjEdges); // For undirected graphs
        }

        std::vector<int> sortedAdjVertices;
        for (const auto& edge : adjEdges) {
            sortedAdjVertices.push_back(edge.first); // Collect adjacent vertices
        }
        std::sort(sortedAdjVertices.begin(), sortedAdjVertices.end()); // Sort adjacent vertices

        // Enqueue sorted adjacent vertices
        for (int adjVertex : sortedAdjVertices) {
            if (!visited[adjVertex]) {
                visited[adjVertex] = true; // Mark as visited
                q.push(adjVertex); // Enqueue vertex
            }
        }
    }

    // Print BFS result
    fout << "========== BFS ============" << endl;
    fout << (option == 'Y' ? "Directed" : "Undirected") << " Graph BFS result" << endl;
    for (size_t i = 0; i < bfsResult.size(); ++i) {
        fout << bfsResult[i]; // Print each vertex
        if (i < bfsResult.size() - 1) {
            fout << " -> "; // Print arrow between vertices
        }
    }
    fout << endl << "===========================" << endl;

    return true;
}



bool Manager::mDFS(char option, int vertex) {
    if (graph == nullptr) { // Check if the graph is loaded
        return false;
    }
    // Check if the vertex is within the valid range
    int V = graph->getSize();
    if (vertex < 1 || vertex > V) {
        printErrorCode(400); // Invalid vertex error
        return false;
    }

    std::map<int, bool> visited; // Map to track visited vertices
    std::stack<int> s; // Stack for DFS
    std::vector<int> dfsResult; // Vector to store DFS result

    s.push(vertex); // Push the starting vertex onto the stack

    while (!s.empty()) {
        int currentVertex = s.top(); // Get the top vertex from the stack
        s.pop(); // Pop the top vertex

        if (visited[currentVertex]) {
            continue; // Skip if the vertex is already visited
        }

        visited[currentVertex] = true; // Mark the current vertex as visited
        dfsResult.push_back(currentVertex); // Store the current vertex in the result vector

        // Get and sort adjacent vertices
        map<int, int> adjEdges;
        if (option == 'Y') {
            graph->getAdjacentEdgesDirect(currentVertex, &adjEdges); // For directed graphs
        } else {
            graph->getAdjacentEdges(currentVertex, &adjEdges); // For undirected graphs
        }

        std::vector<int> sortedAdjVertices;
        for (const auto& adj : adjEdges) {
            sortedAdjVertices.push_back(adj.first); // Collect adjacent vertices
        }
        std::sort(sortedAdjVertices.begin(), sortedAdjVertices.end(), std::greater<int>()); // Sort in descending order

        // Push sorted adjacent vertices onto the stack
        for (int adjVertex : sortedAdjVertices) {
            if (!visited[adjVertex]) {
                s.push(adjVertex); // Push vertex onto the stack
            }
        }
    }

    // Print DFS result
    fout << "======== DFS ========" << endl;
    fout << (option == 'Y' ? "Directed" : "Undirected") << " Graph DFS result" << endl;
    for (size_t i = 0; i < dfsResult.size(); ++i) {
        fout << dfsResult[i]; // Print each vertex
        if (i < dfsResult.size() - 1) {
            fout << " -> "; // Print arrow between vertices
        }
    }
    fout << endl << "=====================" << endl;

    return true;
}


const int INF = std::numeric_limits<int>::max();

bool Manager::mDIJKSTRA(char option, int vertex) {
    // Retrieve adjacency list from the graph data structure
    int V = graph->getSize(); // Get the number of vertices in the graph
    vector<vector<pair<int, int>>> adjList(V + 1); // Create an adjacency list
    graph->makeAdjList(option, adjList); // Populate the adjacency list

    vector<int> dist(V + 1, INF); // Initialize distances with infinity
    vector<int> prev(V + 1, -1); // Initialize previous vertices for path reconstruction
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq; // Min-heap priority queue

    dist[vertex] = 0; // Set the distance of the start vertex to 0
    pq.push({0, vertex}); // Push the start vertex onto the priority queue

    while (!pq.empty()) {
        int currentDist = pq.top().first; // Current distance
        int currentNode = pq.top().second; // Current node
        pq.pop();

        if (dist[currentNode] < currentDist) continue; // Skip if a shorter path has been found

        // Relaxation step for each adjacent vertex
        for (auto& edge : adjList[currentNode]) {
            int nextNode = edge.first; // Next node
            int weight = edge.second; // Weight of the edge

            if (dist[nextNode] > currentDist + weight) {
                dist[nextNode] = currentDist + weight; // Update distance if shorter path found
                prev[nextNode] = currentNode; // Update previous vertex for path reconstruction
                pq.push({dist[nextNode], nextNode}); // Push next vertex onto the priority queue
            }
        }
    }

    // Print the result to log.txt
    fout << "====== Dijkstra =======" << endl;
    fout << (option == 'Y' ? "Directed" : "Undirected") << " Graph Dijkstra result" << endl;
    fout << "startvertex: " << vertex << endl;

    // Print paths and distances for each vertex
    for (int i = 1; i <= V; i++) {
        if (dist[i] == INF) {
            fout << "[" << i << "] x" << endl; // Unreachable vertex
            continue;
        }

        fout << "[" << i << "]"; // Vertex number
        vector<int> path;
        for (int v = i; v != -1; v = prev[v]) { // Construct path
            path.push_back(v);
        }
        reverse(path.begin(), path.end()); // Reverse to get correct order

        for (size_t j = 0; j < path.size(); j++) { // Print path
            fout << (j == 0 ? " " : " -> ") << path[j];
        }
        fout << " (" << dist[i] << ")" << endl; // Print total distance
    }

    fout << "======================" << endl;
    return true;
}






struct Edge {
    int from, to, weight;
    Edge(int f, int t, int w) : from(f), to(t), weight(w) {} // Edge constructor
};

// Quick sort partition function
int partition(vector<Edge>& edges, int low, int high) {
    int pivot = edges[high].weight; // Pivot element
    int i = low - 1;
    for (int j = low; j < high; j++) {
        if (edges[j].weight < pivot) { // Compare with pivot
            i++;
            swap(edges[i], edges[j]); // Swap elements
        }
    }
    swap(edges[i + 1], edges[high]); // Place pivot in correct position
    return i + 1; // Return pivot index
}

// Quick sort function
void quickSort(vector<Edge>& edges, int low, int high) {
    if (low < high) {
        int pi = partition(edges, low, high); // Partition the array
        quickSort(edges, low, pi - 1); // Sort the left part
        quickSort(edges, pi + 1, high); // Sort the right part
    }
}

// Union-Find find function
int find(int v, vector<int>& parent) {
    if (v != parent[v]) parent[v] = find(parent[v], parent); // Path compression
    return parent[v]; // Return root of the set
}

// Union-Find unite function
void unite(int a, int b, vector<int>& parent) {
    a = find(a, parent); // Find root of a
    b = find(b, parent); // Find root of b
    if (a != b) parent[b] = a; // Union the sets
}

bool Manager::mKRUSKAL() {
    if (!graph) {
        printErrorCode(600); // Error if the graph is not loaded
        return false;
    }

    int V = graph->getSize(); // Get the number of vertices in the graph
    vector<Edge> edges; // Vector to store all edges
    vector<int> parent(V + 1); // Union-Find parent array
    int totalCost = 0; // Total cost of the minimum spanning tree

    // Create a list of edges
    for (int i = 1; i <= V; i++) {
        for (int j = 1; j <= V; j++) {
            if (i != j) {
                int weight = graph->getWeight(i, j); // Get the weight of the edge
                if (weight > 0) {
                    edges.push_back(Edge(i, j, weight)); // Add edge to the vector
                }
            }
        }
    }

    // Sort edges using quicksort
    quickSort(edges, 0, edges.size() - 1);

    vector<Edge> selectedEdges; // Vector to store selected edges for MST

    // Initialize Union-Find
    for (int i = 1; i <= V; i++) {
        parent[i] = i; // Set each vertex as its own parent initially
    }

    // Execute Kruskal's algorithm
    for (const auto& edge : edges) {
        if (find(edge.from, parent) != find(edge.to, parent)) {
            unite(edge.from, edge.to, parent); // Union the sets
            selectedEdges.push_back(edge); // Store the selected edge
            totalCost += edge.weight; // Add the weight to the total cost
        }
    }

    // Group all edges by vertices
    map<int, vector<Edge>> groupedEdges;
    for (const auto& edge : edges) {
        groupedEdges[edge.from].push_back(edge);
        // Allow duplicates to add edges in the opposite direction
        groupedEdges[edge.to].push_back(Edge(edge.to, edge.from, edge.weight));
    }

    // Group selected edges by vertices
    map<int, vector<Edge>> groupedSelectedEdges;
    for (const auto& edge : selectedEdges) {
        groupedSelectedEdges[edge.from].push_back(edge);
        // For undirected graph, add edges in the opposite direction as well
        groupedSelectedEdges[edge.to].push_back(Edge(edge.to, edge.from, edge.weight));
    }

    // Sort edges within each group based on edge.to
    for (auto& group : groupedSelectedEdges) {
        sort(group.second.begin(), group.second.end(), [](const Edge& a, const Edge& b) {
            return a.to < b.to;
        });
    }

    // Print sorted selected edges for each vertex
    fout << "====== Kruskal =======" << endl;
    for (const auto& group : groupedSelectedEdges) {
        fout << "[" << group.first << "]"; // Vertex number
        for (const auto& edge : group.second) {
            fout << " " << edge.to << "(" << edge.weight << ")"; // Print edge and weight
        }
        fout << endl;
    }

    fout << "cost: " << totalCost << endl; // Print the total cost of MST
    fout << "======================" << endl; // End of output
    return true;
}


bool Manager::mBELLMANFORD(char option, int startVertex, int endVertex) {
    int V = graph->getSize(); // Get the number of vertices in the graph
    // Check if the start and end vertices are valid
    if (startVertex < 1 || startVertex > V || endVertex < 1 || endVertex > V) {
        printErrorCode(800); // Print error code 800
        return false;
    }
    vector<vector<pair<int, int>>> adjList(V + 1); // Create adjacency list
    graph->makeAdjList(option, adjList); // Populate the adjacency list

    vector<int> dist(V + 1, INF); // Initialize distances with infinity
    vector<int> prev(V + 1, -1); // Initialize previous vertices for path reconstruction
    dist[startVertex] = 0; // Set the distance of the start vertex to 0

    // Edge relaxation
    for (int i = 0; i < V - 1; ++i) {
        for (int u = 1; u <= V; ++u) {
            for (const auto& edge : adjList[u]) {
                int v = edge.first; // Next vertex
                int weight = edge.second; // Weight of the edge

                // Update distance and previous vertex if a shorter path is found
                if (dist[u] != INF && dist[v] > dist[u] + weight) {
                    dist[v] = dist[u] + weight;
                    prev[v] = u;
                }
            }
        }
    }

    // Detect negative cycles
    for (int u = 1; u <= V; ++u) {
        for (const auto& edge : adjList[u]) {
            int v = edge.first;
            int weight = edge.second;

            // If a shorter path is found, it indicates a negative cycle
            if (dist[u] != INF && dist[v] > dist[u] + weight) {
                printErrorCode(800); // Print error code 800
                return false;
            }
        }
    }

    // Print result
    fout << "====== Bellman-Ford =======" << endl;
    fout << (option == 'Y' ? "Directed" : "Undirected") << " Graph Bellman-Ford result" << endl;

    // Print the path and cost from startVertex to endVertex
    if (dist[endVertex] == INF) {
        fout << startVertex << " -> " << endVertex << " x" << endl; // Unreachable vertex
    } else {
        vector<int> path;
        for (int v = endVertex; v != -1; v = prev[v]) { // Construct path
            path.push_back(v);
        }
        reverse(path.begin(), path.end()); // Reverse to get correct order

        // Print path
        fout << path.front();
        for (size_t i = 1; i < path.size(); i++) {
            fout << " -> " << path[i];
        }
        fout << endl << "cost: " << dist[endVertex] << endl; // Print total cost
    }

    fout << "=========================" << endl; // End of output
    return true;
}


bool Manager::mFLOYD(char option) {
    if (!graph) {
        printErrorCode(900); // Error if the graph is not loaded
        return false;
    }
    int V = graph->getSize(); // Get the number of vertices in the graph
    vector<vector<int>> dist(V + 1, vector<int>(V + 1, INT_MAX)); // Initialize distance matrix

    // Initialization: Fill the distance matrix with edge weights
    for (int i = 1; i <= V; ++i) {
        dist[i][i] = 0; // Distance to self is zero
        for (int j = 1; j <= V; ++j) {
            int weight = graph->getWeight(i, j);
            if (weight != 0) {
                dist[i][j] = weight;
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
            printErrorCode(900); // Error code for negative cycle detection
            return false;
        }
    }

    // Print result
    fout << "======== FLOYD ========" << endl;
    fout << (option == 'Y' ? "Directed" : "Undirected") << " Graph FLOYD result" << endl;
    // Print the shortest distances between every pair of vertices
    for (int i = 1; i <= V; ++i) {
        fout << "[" << i << "]";
        for (int j = 1; j <= V; ++j) {
            if (dist[i][j] == INT_MAX) {
                fout << " x"; // Indicate unreachable vertex
            } else {
                fout << " " << dist[i][j]; // Print the distance
            }
        }
        fout << endl;
    }
    fout << "=========================" << endl; // End of output
    return true;
}


bool Manager::mKwoonWoon(int vertex) {
    // Check if the 'graph' object is of 'ListGraph' type and cast it
    auto listGraph = dynamic_cast<ListGraph*>(graph);
    if (!listGraph) {
        cout << "Graph is not a ListGraph" << endl;
        return false;
    }

    int graphSize = listGraph->getSize(); // Get the size of the ListGraph
    // Check if the vertex is valid and has adjacent vertices
    if (vertex < 1 || vertex > graphSize || listGraph->getAdjacentVertices(vertex).empty()) {
        printErrorCode(500); // Print error code if vertex is invalid
        return false;
    }

    vector<bool> visited(graphSize + 1, false); // Vector to track visited vertices
    int current = vertex; // Start from the specified vertex
    visited[current] = true; // Mark the starting vertex as visited

    fout << "======== KWANGWOON ========" << endl;
    fout << "startvertex: " << current << endl;
    fout << current; // Print starting vertex

    while (true) {
        const auto& adj = listGraph->getAdjacentVertices(current); // Get adjacent vertices of current vertex
        vector<int> candidates; // Vector to store unvisited adjacent vertices

        // Collect unvisited adjacent vertices
        for (const auto& v : adj) {
            if (!visited[v]) {
                candidates.push_back(v);
            }
        }

        // Break the loop if no candidates are available
        if (candidates.empty()) {
            break;
        }

        sort(candidates.begin(), candidates.end()); // Sort the candidates
        // Choose the next vertex based on the size of candidates
        current = (candidates.size() % 2 == 0) ? candidates.front() : candidates.back();
        visited[current] = true; // Mark the chosen vertex as visited
        fout << " -> " << current; // Print the chosen vertex
    }

    fout << endl << "============================" << endl; // End of output
    return true;
}

void Manager::printErrorCode(int n)
{
    fout << "========ERROR=======" << endl; // Start of error message
    fout << n << endl; // Print the error code
    fout << "====================" << endl << endl; // End of error message
}
