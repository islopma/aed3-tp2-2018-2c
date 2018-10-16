#ifndef __GRAPH__
#define __GRAPH__

#include <vector>

using namespace std;

class Graph;
class MSTStrategy;
class DisjoinSet;
class DisjoinSetDefault;
class DisjoinSetCompressed;

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

struct
{
    bool operator() (Edge& ledge, Edge& redge) {
        return ledge.weight <redge.weight;
    }
} cmpWeigth ;

class Graph
{
private:
    vector<Edge> _edges;
    vector<vector<Node>> _adjacencyList;
    void addNode();
    void sortAdjacencyLists();
    MSTStrategy *mstStrategy;

public:
    vector<vector<float>> getAdjacencyMatrix() const;
    Graph();
    Graph(const int &nodesNumber);
    void build(const int &nodesNumber);
    void addEdge(const Node &first, const Node &second);
    void addEdge(const Node &first, const Node &second, const float weight);
    void addEdge(Edge otherGraphEdge);
    vector<Edge> getEdges() ;
    vector<vector<Node>> getAdjacencyList() ;
    bool operator==(Graph &other);
    float getTotalWeigth();
    void setMSTStrategy(MSTStrategy *strategy);
    Graph getMSTKruskal();
    Graph getMST();
};

class MSTStrategy{
public:
    virtual Graph getMST(Graph *pGraph) = 0;
};

class KruskalDefaultMST : public MSTStrategy{
public:
    Graph getMST(Graph *pGraph);
    DisjoinSetDefault *getContatiner();
};

class KruskalCompressedMST : public MSTStrategy{
    Graph getMST(Graph *pGraph);
    DisjoinSetCompressed *getContainer();
};

class PrimMST : public MSTStrategy{
    Graph getMST(Graph *pGraph);
};


class DisjoinSet {
public:
    virtual int find(Node nodo) = 0;
    virtual void join(Node alreadyIn, Node newNode) = 0;
    virtual void create(Graph *graph) = 0;
    virtual ~DisjoinSet() { };

protected:
    std::vector<int> components;

};

class DisjoinSetDefault : public DisjoinSet {
public:
    int find(Node nodo);
    void join(Node alreadyIn, Node newNode);
    void create(Graph *graph);
};

class DisjoinSetCompressed : public DisjoinSet {
public:
    int find(Node nodo);
    void join(Node alreadyIn, Node newNode);
    void create(Graph *graph);

private:
    std::vector<int> heights;
};
#endif