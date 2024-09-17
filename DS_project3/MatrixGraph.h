#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <map>
#include "Graph.h"

class MatrixGraph : public Graph {	
private:
    int** m_Mat;

    int** matrix;

public:
    // Constructor
    MatrixGraph(bool type, int size);
    // Destructor
    ~MatrixGraph();
    
    void getAdjacentEdges(int vertex, map<int, int>* m);	
    void getAdjacentEdgesDirect(int vertex, map<int, int>* m);
    void insertEdge(int from, int to, int weight);	
    bool printGraph(ofstream *fout) override;

    // Gets the weight of the edge between two vertices
    int getWeight(int from, int to) {
        if (from < 1 || from > m_Size || to < 1 || to > m_Size) {
            return 0; // Return 0 if vertex numbers are not valid
        }
        // Subtract 1 from vertex indices as array indices start from 0
        return m_Mat[from - 1][to - 1];
    }
};

#endif
