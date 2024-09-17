#include "Graph.h"

Graph::Graph(bool type, int size) 
{
    m_Type = type; // Set graph type
    m_Size = size; // Set the size
}

// Destructor for the Graph class
Graph::~Graph() 
{
}

// Getter function to retrieve the type 
bool Graph::getType() { return m_Type; }  

// Getter function to retrieve the size of the graph
int Graph::getSize() { return m_Size; }
