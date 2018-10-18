#ifndef __CLUSTERING__
#define __CLUSTERING__

#include "../graph.h"
#include "point.h"
#include <map>
#include <vector>

enum class MstMode
{
    Prim,
    Kruskal,
    KruskalCompressed
};

enum class InconsistencyMode
{
    StandardDeviation,
    Factor
};

class Clustering
{
private:
    vector<Point> _points;
    MstMode _mstMode;
    InconsistencyMode _inconsistencyMode;
    int _neighborhoodDepth;
    float _inconsistencyParameter;
    Graph _graph;

    void buildGraph();
    vector<vector<pair<int, float>>> getAdjacencyMatrix();
    vector<Edge> getNeighborhood(Edge edge, bool fromFirstNode);
    pair<vector<Edge>, vector<Edge>> getNeighborhoods(Edge edge);
    bool isInconsistentEdge(Edge edge, pair<vector<Edge>, vector<Edge>> neighborhoods);
    float getNeighborhoodAverage(vector<Edge> neighborhood);
    float getNeighborhoodStdDev(vector<Edge> neighborhood, float average);
public:
    Clustering(
        const vector<Point> points,
        MstMode mstMode,
        InconsistencyMode inconsistencyMode,
        int neighborhoodDepth,
        float inconsistencyParameter);
    vector<int> getClustersByPoint();
};
#endif