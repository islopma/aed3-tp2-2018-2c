#ifndef __GRAPH__
#define __GRAPH__

#include <vector>

using namespace std;

struct Node
{
    int id;

    Node(const int &id);
};

struct Edge
{
    int id;
    pair<Node, Node> nodes;

    Edge(const int id, const Node &first, const Node &second);
};

class Graph
{
private:
    vector<Edge> _edges;
    vector<vector<Node>> _adjacencyList;
public:
    Graph(const int &nodesNumber);
    void addNode();
    void addEdge(const Node &first, const Node &second);
    vector<Edge> getEdges() const;
    vector<vector<Node>> getAdjacencyList() const;
};

#endif