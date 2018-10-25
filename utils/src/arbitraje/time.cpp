#include "../graph.h"
#include <iostream>
#include <chrono>

using namespace std;

int main(int argc, char const *argv[])
{
    auto graph = Graph(0);
    graph.recibirParametrosArbitraje();
    if (argc < 2)
    {
        cerr << "Insufficient arguments have been supplied. The program will exit" << endl;
        return -1;
    }

    string output;
    chrono::high_resolution_clock::time_point start;
    chrono::high_resolution_clock::time_point end;
    string strategy = argv[1];
    if (strategy == "BellmanFord")
    {
        start = chrono::high_resolution_clock::now();
        graph.BellmanFord();
        end = chrono::high_resolution_clock::now();
    }
    else if (strategy == "FloydWarshall")
    {
        start = chrono::high_resolution_clock::now();
        graph.floydWarshall();
        end = chrono::high_resolution_clock::now();
    }
    else
    {
        cerr << "No valid strategy has been chosen. The program will exit." << endl;
        return -1;
    }

    cout << chrono::duration_cast<chrono::nanoseconds>(end-start).count() << endl;
}
