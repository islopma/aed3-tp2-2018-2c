#include "../graph.h"
#include "point.h"
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

Graph buildGraph(vector<Point> points)
{
    // build complete graph with the points
    auto pointsNumber = points.size();
    auto graph = Graph(pointsNumber);
    for (size_t firstIndex = 0; firstIndex < pointsNumber; ++firstIndex)
    {
        auto first = points[firstIndex];
        for (size_t secondIndex = firstIndex + 1; secondIndex < pointsNumber; ++secondIndex)
        {
            auto second = points[firstIndex];
            auto weight = first.distance(second);
            graph.addEdge(Node(firstIndex), Node(secondIndex), weight);
        }
    }
    return graph;
}

vector<string> tokenizeLine()
{
    string line;
    getline(cin, line);
    istringstream buf(line);
    istream_iterator<string> beg(buf), end;
    vector<string> tokens(beg, end);
    return tokens;
}

vector<Point> readPoints(int pointsNumber)
{
    auto points = vector<Point>();
    for (auto pointIndex = 0; pointIndex < pointsNumber; ++pointIndex)
    {
        auto tokens = tokenizeLine();
        points.push_back(Point(stof(tokens[0]), stof(tokens[1])));
    }
    return points;
}

int main(int argc, char const *argv[])
{
    auto tokens = tokenizeLine();
    auto pointsNumber = stoi(tokens[0]);
    if (argc < 2)
    {
        cerr << "No mode has been supplied. The program will exit" << endl;
        return -1;
    }

    auto points = readPoints(pointsNumber);
    auto graph = buildGraph(points);

    string mode = argv[1];
    if (mode == "Prim")
    {
        auto mst = graph.getPrimMST();
    }
    else if (mode == "Kruskal")
    {
        auto mst = graph.getMSTKruskal();
    }
    else if (mode == "KruskalPC")
    {
        //auto mst = new graph.getMSTKruskalPC();
    }
    else
    {
        cerr << "No valid mode has been chosen. The program will exit." << endl;
        return -1;
    }
}
