#include "gtest/gtest.h"
#include "../src/graph.h"

struct GraphTest: testing::Test{
    Graph *graph1;
    Graph *graph2;

    GraphTest(){
        // graph1
        graph1 = new Graph(4);
        graph1->addEdge(Node(0), Node(1), 3);
        graph1->addEdge(Node(0), Node(3), 5);
        graph1->addEdge(Node(1), Node(2), 8);
        graph1->addEdge(Node(1), Node(3), 1);
        graph1->addEdge(Node(2), Node(3), 2);
        // graph2
        graph2 = new Graph(9);
        graph2->addEdge(Node(0), Node(1), 4);
        graph2->addEdge(Node(0), Node(7), 8);
        graph2->addEdge(Node(1), Node(7), 11);
        graph2->addEdge(Node(1), Node(2), 8);
        graph2->addEdge(Node(2), Node(3), 7);
        graph2->addEdge(Node(2), Node(5), 4);
        graph2->addEdge(Node(2), Node(8), 2);
        graph2->addEdge(Node(3), Node(4), 9);
        graph2->addEdge(Node(3), Node(5), 14);
        graph2->addEdge(Node(4), Node(5), 10);
        graph2->addEdge(Node(5), Node(6), 2);
        graph2->addEdge(Node(6), Node(7), 1);
        graph2->addEdge(Node(6), Node(8), 6);
        graph2->addEdge(Node(7), Node(8), 7);
    }

    ~GraphTest(){
        delete graph1;
        delete graph2;
    }
};

TEST_F(GraphTest, whenCreateGraph_shouldHaveNodes)
{
    auto graph = Graph(10);
    ASSERT_EQ(0, graph.getEdges().size());
    ASSERT_EQ(10, graph.getAdjacencyList().size());
}

TEST_F(GraphTest, whenAddEdges_shouldHaveEdgesAndAdjacentNodes)
{
    auto graph = Graph(10);
    graph.addEdge(Node(0), Node(1));
    graph.addEdge(Node(1), Node(3));
    ASSERT_EQ(2, graph.getEdges().size());
    ASSERT_EQ(10, graph.getAdjacencyList().size());
    ASSERT_EQ(1, graph.getAdjacencyList()[0].size());
    ASSERT_EQ(2, graph.getAdjacencyList()[1].size());
}

TEST_F(GraphTest, whenGraph1PrimMST_shouldHaveTotalWeight6)
{
    auto mst = graph1->getPrimMST();
    float totalWeight = 0;
    for (auto const& edge : mst.getEdges())
    {
        totalWeight += edge.weight;
    }
    ASSERT_EQ(6, totalWeight);
}

TEST_F(GraphTest, whenGraph2PrimMST_shouldHaveTotalWeight37)
{
    auto mst = graph2->getPrimMST();
    float totalWeight = 0;
    for (auto const& edge : mst.getEdges())
    {
        totalWeight += edge.weight;
    }
    ASSERT_EQ(37, totalWeight);
}