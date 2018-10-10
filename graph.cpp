#include "graph.h"

Node::Node(const int &id)
: id(id) {}

Edge::Edge(const int id, const Node &first, const Node &second)
: id(id), nodes(make_pair(first, second)) {}

Graph::Graph(const int &nodesNumber)
{
    _edges = vector<Edge>();
    _adjacencyList = vector<vector<Node>>(nodesNumber, vector<Node>());
}

void Graph::addNode()
{
    _adjacencyList.push_back(vector<Node>());
}

void Graph::addEdge(const Node &first, const Node &second)
{
    _edges.push_back(Edge(_edges.size(), first, second));
    _adjacencyList[first.id].push_back(second.id);
    _adjacencyList[second.id].push_back(first.id);
}

vector<Edge> Graph::getEdges() const
{
    return _edges;
}

vector<vector<Node>> Graph::getAdjacencyList() const
{
    return _adjacencyList;
}