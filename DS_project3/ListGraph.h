#ifndef _LIST_H_
#define _LIST_H_

#include "Graph.h"

class ListGraph : public Graph {	
private:
    map<int, vector<pair<int, int>>> m_List;

    map<int, vector<pair<int, int>>> adjList;

    // Array used for KwangWoon algorithm
    vector<int>* kw_graph;
    
public:	
    // Constructor
    ListGraph(bool type, int size);
    // Destructor
    ~ListGraph();


    //functions
    void getAdjacentEdges(int vertex, map<int, int>* m);	
    void getAdjacentEdgesDirect(int vertex, map<int, int>* m);
    void insertEdge(int from, int to, int weight);	
    bool printGraph(ofstream *fout) override;

    // Gets the weight of the edge between two vertices
    int getWeight(int from, int to) {
        if (from < 1 || from > m_Size || to < 1 || to > m_Size) {
            return 0; // Return 0 if vertex numbers are not valid
        }

        for (const auto& edge : m_List[from]) {
            if (edge.first == to) {
                return edge.second; // Return weight if edge is found in adjacency list
            }
        }

        return 0; // Return 0 if vertices are not connected
    }
};

#endif
