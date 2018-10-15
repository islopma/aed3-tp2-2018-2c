#ifndef __GRAPH__
#define __GRAPH__

#include <vector>

using namespace std;

struct Node
{
    int id;

    Node(const int &id);

    bool operator==(const Node& other) const{
        return id == other.id;
    }

    bool operator<(const Node& other) const{
        return id < other.id;
    }

};

struct Edge
{
    int id;
    pair<Node, Node> nodes;
    float weight;

    Edge(const int id, const Node &first, const Node &second, const float weight);

    bool operator==(const Edge& other) const{
        return (nodes == other.nodes) && (weight == other.weight);
    }

    bool operator<(const Edge& other) const{
        return (nodes < other.nodes);
    }
};

class Graph
{
private:
    vector<Edge> _edges;
    vector<vector<Node>> _adjacencyList;
    void addNode();
    void sortAdjacencyLists();


public:
    vector<vector<float>> getAdjacencyMatrix() const;
    Graph(const int &nodesNumber);
    void addEdge(const Node &first, const Node &second);
    void addEdge(const Node &first, const Node &second, const float weight);
    vector<Edge> getEdges() ;
    vector<vector<Node>> getAdjacencyList() ;
    Graph getMSTKruskal();
    bool operator==(Graph &other);
    Graph getPrimMST() const;
};

#endif