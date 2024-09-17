#include "ListGraph.h"
#include <iostream>
#include <utility>

// Constructor for ListGraph class
ListGraph::ListGraph(bool type, int size) : Graph(type, size) {
    m_List.clear();  
}

// Destructor for ListGraph class
ListGraph::~ListGraph() {
    
}

// Function to get adjacent edges of a given vertex
void ListGraph::getAdjacentEdges(int vertex, map<int, int>* m) {
    // Check for valid vertex number and non-null map pointer
    if (vertex < 1 || vertex > m_Size || m == nullptr) {
        return; // Exit if vertex number 
    }

    // Find edges starting from the given vertex
    for (const auto& edge : m_List[vertex]) {
        (*m)[edge.first] = edge.second; // Add adjacent vertex 
    }
}

// Function to get adjacent edges for directed graphs
void ListGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m) {
    getAdjacentEdges(vertex, m); // behaves same as getAdjacentEdges
}

// Function to insert 
void ListGraph::insertEdge(int from, int to, int weight) {
    // Check for valid vertex numbers 
    if (from >= 1 && from <= m_Size && to >= 1 && to <= m_Size && from != to) {
        m_List[from].emplace_back(to, weight); // Add edge 

        // add edge in the opposite direction as well
        if (!m_Type) {
            m_List[to].emplace_back(from, weight);
        }
    }
}

// Function to print the graph
bool ListGraph::printGraph(ofstream* fout) {
    *fout << "======== PRINT ========" << endl;

    // Set to track already printed edges
    set<pair<int, int>> printedEdges;

    // Iterate through all vertices
    for (int i = 1; i <= m_Size; i++) {
        *fout << "[" << i << "]";
        // Iterate through each edge of the vertex
        for (const auto& edge : m_List[i]) {
            int to = edge.first;
            int weight = edge.second;

            // Standardize edge to 
            pair<int, int> edgePair = make_pair(min(i, to), max(i, to));

            
            if (printedEdges.find(edgePair) == printedEdges.end()) {
                *fout << " -> (" << to << ", " << weight << ")";
                printedEdges.insert(edgePair); 
            }
        }
        *fout << endl;
    }

    *fout << "======================" << endl;
    return true;
}
