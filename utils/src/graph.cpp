#include "graph.h"
#include <algorithm>
#include <limits>

Node::Node(const int &id)
: id(id) {}

Edge::Edge(const int id, const Node &first, const Node &second, const float weight)
: id(id), nodes(make_pair(first, second)), weight(weight) {}

Graph::Graph() {
    _edges = vector<Edge>();
    _adjacencyList = vector<vector<Node>>();
}

Graph::Graph(const int &nodesNumber)
{
    _edges = vector<Edge>();
    _adjacencyList = vector<vector<Node>>(nodesNumber, vector<Node>());
}

void Graph::build(const int &nodesNumber) {
    _adjacencyList.resize(nodesNumber);
}

void Graph::addNode()
{
    _adjacencyList.push_back(vector<Node>());
}

void Graph::addEdge(const Node &first, const Node &second)
{
    addEdge(first, second, 0);
}

void Graph::addEdge(const Node &first, const Node &second, const float weight)
{
    _edges.push_back(Edge(_edges.size(), first, second, weight));
    _adjacencyList[first.id].push_back(second.id);
    _adjacencyList[second.id].push_back(first.id);
}

vector<Edge> Graph::getEdges()
{
    return _edges;
}

vector<vector<Node>> Graph::getAdjacencyList()
{
    return _adjacencyList;
}

bool Graph::operator==(Graph &other) {
    this->sortAdjacencyLists();
    other.sortAdjacencyLists();
    std::sort(this->_edges.begin(),this->_edges.end());
    std::sort(other.getEdges().begin(),other.getEdges().end());
    return  (this->_edges == other.getEdges() ) && (this->_adjacencyList == other.getAdjacencyList());
}

void Graph::sortAdjacencyLists() {
    for (auto &index : this->_adjacencyList) {
        std::sort(index.begin(), index.end() );
    }
}

vector<vector<float>> Graph::getAdjacencyMatrix() const
{
    auto nodesNumber = _adjacencyList.size();
    // initialize matrix in -1 which means no edge exists
    auto adjacencyMatrix = vector<vector<float>>(nodesNumber, vector<float>(nodesNumber, -1));
    for (auto const& edge : _edges)
    {
        auto first = edge.nodes.first.id;
        auto second = edge.nodes.second.id;
        adjacencyMatrix[first][second] = edge.weight;
        adjacencyMatrix[second][first] = edge.weight;
    }
    return adjacencyMatrix;
}

float Graph::getTotalWeigth() {
    float totalWeight = 0;
    for (auto const& edge : this->getEdges())
    {
        totalWeight += edge.weight;
    }
    return totalWeight;
}

void Graph::addEdge(Edge otherGraphEdge) {
    auto &first = otherGraphEdge.nodes.first;
    auto &second = otherGraphEdge.nodes.second;
    _edges.push_back(Edge(_edges.size(), first, second, otherGraphEdge.weight));
    _adjacencyList[first.id].push_back(second.id);
    _adjacencyList[second.id].push_back(first.id);
}

void Graph::setMSTStrategy(MSTStrategy *strategy) {
    this->mstStrategy = strategy;
}

Graph Graph::getMST() {
    return mstStrategy->getMST(this);
}

DisjoinSetCompressed *KruskalCompressedMST::getContainer() {
    return new DisjoinSetCompressed();
}

DisjoinSetDefault * KruskalDefaultMST::getContatiner() {
    return new DisjoinSetDefault();
}

Graph KruskalDefaultMST::getMST(Graph* graph) {
    Graph mst(graph->getAdjacencyList().size());
    DisjoinSet *disjoinSet = this->getContatiner();
    disjoinSet->create(graph);
    vector<Edge> edges = graph->getEdges();
    sort(edges.begin(),edges.end(), cmpWeigth);
    for(Edge edge : edges){
        if(disjoinSet->find(edge.nodes.first) !=  disjoinSet->find(edge.nodes.second)){
            mst.addEdge(edge);
            disjoinSet->join(disjoinSet->find(edge.nodes.first), disjoinSet->find(edge.nodes.second));
        }
    }
    return mst;
}

Graph KruskalCompressedMST::getMST(Graph* graph) {
    Graph mst(graph->getAdjacencyList().size());
    DisjoinSet *disjoinSet = this->getContainer();
    disjoinSet->create(graph);
    vector<Edge> edges = graph->getEdges();
    sort(edges.begin(),edges.end(), cmpWeigth);
    for(Edge edge : edges){
        if(disjoinSet->find(edge.nodes.first) !=  disjoinSet->find(edge.nodes.second)){
            mst.addEdge(edge);
            disjoinSet->join(disjoinSet->find(edge.nodes.first), disjoinSet->find(edge.nodes.second));
        }
    }
    return mst;
}

Graph PrimMST::getMST(Graph *graph)
{
    auto adjacencyList = graph->getAdjacencyList();
    auto nodesNumber = adjacencyList.size();
    auto adjacencyMatrix = graph->getAdjacencyMatrix();
    auto visited = vector<bool>(nodesNumber, false);
    auto minWeight = vector<float>(nodesNumber, numeric_limits<float>::infinity());
    auto parents = vector<int>(nodesNumber, -1);
    // select first node and update adjacent weights
    visited[0] = true;
    minWeight[0] = 0;
    for (auto const& node : adjacencyList[0])
    {
        minWeight[node.id] = adjacencyMatrix[0][node.id];
        parents[node.id] = 0;
    }

    auto mst = Graph(nodesNumber);
    // add n - 1 edges to complete MST
    for (size_t edgeId = 0; edgeId < nodesNumber - 1; ++edgeId)
    {
        int closestNode;
        auto bestWeight = numeric_limits<float>::infinity();
        // select closest node
        for (size_t nodeId = 1; nodeId < nodesNumber; ++nodeId)
        {
            if (!visited[nodeId] && minWeight[nodeId] < bestWeight)
            {
                closestNode = nodeId;
                bestWeight = minWeight[nodeId];
            }
        }
        // add edge and update weights
        mst.addEdge(Node(parents[closestNode]), Node(closestNode), bestWeight);
        visited[closestNode] = true;
        for (auto const& node : adjacencyList[closestNode])
        {
            if (!visited[node.id]
                && adjacencyMatrix[closestNode][node.id] < minWeight[node.id])
            {
                minWeight[node.id] = adjacencyMatrix[closestNode][node.id];
                parents[node.id] = closestNode;
            }
        }
    }

    return mst;
}

