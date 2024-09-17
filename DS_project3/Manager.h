#ifndef _MANAGER_H_
#define _MANAGER_H_

#include "GraphMethod.h"

// Manager class for graph operations
class Manager {    
private:
    Graph* graph;      // Graph object
    ofstream fout;     // Output file stream
    int load;          // Load  flag

public:
    Manager();          // Constructor
    ~Manager();         // Destructor

    // Executes commands 
    void run(const char * command_txt);
    
    // Loads graph data
    bool LOAD(const char* filename);  

    // Prints the current
    bool PRINT();  

    // Performs Breadth-First Search
    bool mBFS(char option, int vertex);  

    // Performs Depth-First Search
    bool mDFS(char option, int vertex);  

    // Implements Dijkstra's algorithm
    bool mDIJKSTRA(char option, int vertex);  

    // Implements Kruskal's algorithm 
    bool mKRUSKAL();  

    // Implements Bellman-Ford algorithm 
    bool mBELLMANFORD(char option, int s_vertex, int e_vertex);  

    // Implements Floyd-Warshall algorithm 
    bool mFLOYD(char option);

    // Custom method for exploring graph 
    bool mKwoonWoon(int vertex);

    void printErrorCode(int n); 
};

#endif
