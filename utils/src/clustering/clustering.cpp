#include "clustering.h"
#include <algorithm>
#include <cmath>

Clustering::Clustering(
    const vector<Point> points,
    MstMode mstMode,
    InconsistencyMode inconsistencyMode,
    int neighborhoodDepth,
    float inconsistencyParameter)
    : _points(points),
      _mstMode(mstMode),
      _inconsistencyMode(inconsistencyMode),
      _neighborhoodDepth(neighborhoodDepth),
      _inconsistencyParameter(inconsistencyParameter) {}

void Clustering::buildGraph()
{
    // build complete graph with the points
    auto pointsNumber = _points.size();
    _graph = Graph(pointsNumber);
    for (size_t firstIndex = 0; firstIndex < pointsNumber; ++firstIndex)
    {
        auto first = _points[firstIndex];
        for (size_t secondIndex = firstIndex + 1; secondIndex < pointsNumber; ++secondIndex)
        {
            auto second = _points[secondIndex];
            auto weight = first.distance(second);
            _graph.addEdge(Node(firstIndex), Node(secondIndex), weight);
        }
    }
}

vector<vector<pair<int, float>>> Clustering::getAdjacencyMatrix()
{
    auto adjacencyList = _graph.getAdjacencyList();
    auto nodesNumber = adjacencyList.size();
    // initialize matrix in (-1,-1) which means no edge exists
    auto adjacencyMatrix = vector<vector<pair<int, float>>>(nodesNumber, vector<pair<int, float>>(nodesNumber, pair<int, float>(-1, -1)));
    for (auto const& edge : _graph.getEdges())
    {
        auto first = edge.nodes.first.id;
        auto second = edge.nodes.second.id;
        adjacencyMatrix[first][second] = pair<int, float>(edge.id, edge.weight);
        adjacencyMatrix[second][first] = pair<int, float>(edge.id, edge.weight);
    }
    return adjacencyMatrix;
}

vector<Edge> Clustering::getNeighborhood(Edge edge, bool fromFirstNode)
{
    auto adjacencyList = _graph.getAdjacencyList();
    auto adjacencyMatrix = getAdjacencyMatrix();
    // this keeps record of edges added to the neighborhood with a true value
    // if their neighbors have also been added
    auto visitedEdges = map<int, bool>();
    // this is the final neighborhood of edges
    auto neighborhood = vector<Edge>();
    // add initial edge
    visitedEdges.insert(pair<int, bool>(edge.id, false));
    neighborhood.push_back(edge);
    auto depthLevel = 0;
    while (depthLevel < _neighborhoodDepth)
    {
        for (size_t neighborIndex = 0; neighborIndex < neighborhood.size(); ++neighborIndex)
        {
            auto neighbor = neighborhood[neighborIndex];
            // skip if already explored
            if (visitedEdges[neighbor.id]) continue;
            if (depthLevel > 0 || fromFirstNode)
            {
                // explore from first node
                auto firstNode = neighbor.nodes.first;
                for (auto const& adjacentNode : adjacencyList[firstNode.id])
                {
                    auto adjacentEdge = adjacencyMatrix[firstNode.id][adjacentNode.id];
                    // skip if already added as neighbor
                    if (visitedEdges.count(adjacentEdge.first)) continue;
                    // add newly found edge
                    visitedEdges.insert(pair<int, bool>(adjacentEdge.first, false));
                    neighborhood.push_back(Edge(adjacentEdge.first, firstNode, adjacentNode, adjacentEdge.second));
                }
            }
            if (depthLevel > 0 || !fromFirstNode)
            {
                // explore from second node
                auto secondNode = neighbor.nodes.second;
                for (auto const& adjacentNode : adjacencyList[secondNode.id])
                {
                    auto adjacentEdge = adjacencyMatrix[secondNode.id][adjacentNode.id];
                    // skip if already added as neighbor
                    if (visitedEdges.count(adjacentEdge.first)) continue;
                    // add newly found edge
                    visitedEdges.insert(pair<int, bool>(adjacentEdge.first, false));
                    neighborhood.push_back(Edge(adjacentEdge.first, secondNode, adjacentNode, adjacentEdge.second));
                }
            }
            visitedEdges[neighbor.id] = true;
        }
        ++depthLevel;
    }
    // remove initial edge
    neighborhood.erase(neighborhood.begin());
    return neighborhood;
}

pair<vector<Edge>, vector<Edge>> Clustering::getNeighborhoods(Edge edge)
{
    auto firstNeighborhood = getNeighborhood(edge, true);
    auto secondNeighborhood = getNeighborhood(edge, false);
    return pair<vector<Edge>, vector<Edge>>(firstNeighborhood, secondNeighborhood);
}

bool Clustering::isInconsistentEdge(Edge edge, pair<vector<Edge>, vector<Edge>> neighborhoods)
{
    auto firstAverage = getNeighborhoodAverage(neighborhoods.first);
    auto secondAverage = getNeighborhoodAverage(neighborhoods.second);
    bool isInconsistent;
    switch (_inconsistencyMode)
    {
        case InconsistencyMode::StandardDeviation:
        {
            auto firstStdDev = getNeighborhoodStdDev(neighborhoods.first, firstAverage);
            auto secondStdDev = getNeighborhoodStdDev(neighborhoods.second, secondAverage);
            auto firstMeasure = firstAverage + _inconsistencyParameter * firstStdDev;
            auto secondMeasure = secondAverage + _inconsistencyParameter * secondStdDev;
            isInconsistent = edge.weight > max(firstMeasure, secondMeasure);
            break;
        }
        case InconsistencyMode::Factor:
        {
            isInconsistent = edge.weight > _inconsistencyParameter * max(firstAverage, secondAverage);
            break;
        }
    }
    return isInconsistent;
}

float Clustering::getNeighborhoodAverage(vector<Edge> neighborhood)
{
    float sum = 0;
    for (auto const& neighbor : neighborhood)
    {
        sum += neighbor.weight;
    }
    return sum / neighborhood.size();
}

float Clustering::getNeighborhoodStdDev(vector<Edge> neighborhood, float average)
{
    float sum = 0;
    for (auto const& neighbor : neighborhood)
    {
        sum += pow(neighbor.weight - average, 2);
    }
    auto variance = sum / (neighborhood.size() - 1);
    return sqrt(variance);
}

vector<int> Clustering::getClustersByPoint()
{
    buildGraph();
    switch (_mstMode)
    {
        case MstMode::Prim:
            _graph.setMSTStrategy(new PrimMST());
            break;
        case MstMode::Kruskal:
            _graph.setMSTStrategy(new KruskalDefaultMST());
            break;
        case MstMode::KruskalCompressed:
            _graph.setMSTStrategy(new KruskalCompressedMST());
            break;
    }
    _graph = _graph.getMST();

    // remove inconsistent edges from MST
    auto edges = _graph.getEdges();
    for (int edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
    {
        auto edge = edges[edgeIndex];
        auto neighboorhoods = getNeighborhoods(edge);
        if (isInconsistentEdge(edge, neighboorhoods))
        {
            _graph.removeEdge(edge);
        }
    }

    // get clusters as connected components
    return vector<int>();
}