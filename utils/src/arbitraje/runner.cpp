#include "../graph.h"
#include <iostream>

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
    string strategy = argv[1];
    if (strategy == "BellmanFord")
    {
        output = graph.BellmanFord();
    }
    else if (strategy == "FloydWarshall")
    {
        output = graph.floydWarshall();
    }
    else
    {
        cerr << "No valid strategy has been chosen. The program will exit." << endl;
        return -1;
    }

    cout << output << endl;
}
