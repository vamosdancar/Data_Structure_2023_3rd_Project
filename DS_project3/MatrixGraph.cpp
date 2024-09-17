#include "MatrixGraph.h"
#include <iostream>
#include <vector>
#include <string>

// Constructor for MatrixGraph
MatrixGraph::MatrixGraph(bool type, int size) : Graph(type, size) {
    m_Mat = new int*[m_Size]; // Allocating memory for the matrix
    for (int i = 0; i < m_Size; i++) {
        m_Mat[i] = new int[m_Size](); // Initializing all values to 0
    }
}

// Destructor for MatrixGraph
MatrixGraph::~MatrixGraph() {
    for (int i = 0; i < m_Size; i++) {
        delete[] m_Mat[i]; // Deleting each array inside the matrix
    }
    delete[] m_Mat; // Deleting the matrix
}

// Function to get adjacent edges of a vertex
void MatrixGraph::getAdjacentEdges(int vertex, map<int, int>* m) {
    if (vertex < 1 || vertex > m_Size) {
        return; // Check for valid vertex number
    }

    // Find edges starting from the given vertex
    for (int i = 0; i < m_Size; i++) {
        if (m_Mat[vertex - 1][i] != 0) { // If an edge exists
            (*m)[i + 1] = m_Mat[vertex - 1][i]; // Add adjacent vertex and weight to the map
        }
    }
}

// Function to get direct adjacent edges of a vertex (used in directed graphs)
void MatrixGraph::getAdjacentEdgesDirect(int vertex, map<int, int>* m) {
    if (vertex < 1 || vertex > m_Size) {
        return; // Check for valid vertex number
    }

    for (int i = 0; i < m_Size; i++) {
        if (m_Mat[vertex - 1][i] != 0) { // If an edge exists
            (*m)[i + 1] = m_Mat[vertex - 1][i]; // Add adjacent vertex and weight to the map
        }
    }
}

// Function to insert an edge into the graph
void MatrixGraph::insertEdge(int from, int to, int weight) {
    if (from > 0 && from <= m_Size && to > 0 && to <= m_Size) {
        m_Mat[from - 1][to - 1] = weight; // Add edge only in one direction (for directed graphs)
    }
}

// Function to print the graph
bool MatrixGraph::printGraph(ofstream* fout) {
    *fout << "======== PRINT ========" << endl;
    *fout << "   ";
    for (int i = 1; i <= m_Size; i++) {
        *fout << "  [" <<  i << "]"; // Print column headers
    }
    *fout << endl;

    for (int i = 0; i < m_Size; i++) {
        *fout << "[" << (i + 1) << "]"; // Print row headers
        for (int j = 0; j < m_Size; j++) {
            *fout << " " << m_Mat[i][j]; // Print matrix values
            *fout << "   "; // Spacing for readability
        }
        *fout << endl;
    }
    *fout << "======================" << endl;
}
