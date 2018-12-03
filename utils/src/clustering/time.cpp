#include "clustering.h"
#include "point.h"
#include <chrono>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

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
    if (argc < 5)
    {
        cerr << "Insufficient arguments have been supplied. The program will exit" << endl;
        return -1;
    }

    auto points = readPoints(pointsNumber);
    
    MstMode mstMode;
    string mstModeInput = argv[1];
    if (mstModeInput == "Prim")
    {
        mstMode = MstMode::Prim;
    }
    else if (mstModeInput == "Kruskal")
    {
        mstMode = MstMode::Kruskal;
    }
    else if (mstModeInput == "KruskalCompressed")
    {
        mstMode = MstMode::KruskalCompressed;
    }
    else
    {
        cerr << "No valid MST mode has been chosen. The program will exit." << endl;
        return -1;
    }

    InconsistencyMode inconsistencyMode;
    string inconsistencyModeInput = argv[2];
    if (inconsistencyModeInput == "StandardDeviation")
    {
        inconsistencyMode = InconsistencyMode::StandardDeviation;
    }
    else if (inconsistencyModeInput == "Factor")
    {
        inconsistencyMode = InconsistencyMode::Factor;
    }
    else
    {
        cerr << "No valid inconsistency mode has been chosen. The program will exit." << endl;
        return -1;
    }
    auto neighborhoodDepth = stoi(argv[3]);
    auto inconsistencyParameter = stof(argv[4]);

    auto start = chrono::high_resolution_clock::now();

    auto clustering = Clustering(points, mstMode, inconsistencyMode, neighborhoodDepth, inconsistencyParameter);
    auto clusters = clustering.getClustersByPoint();
    
    auto end = chrono::high_resolution_clock::now();
    cout << chrono::duration_cast<chrono::nanoseconds>(end-start).count() << endl;
}
