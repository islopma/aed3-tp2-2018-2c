#include "gtest/gtest.h"
#include "graph.h"

TEST(Graph, whenCreateGraph_shouldHaveNodes)
{
    auto graph = Graph(10);
    ASSERT_EQ(0, graph.getEdges().size());
    ASSERT_EQ(10, graph.getAdjacencyList().size());
}

TEST(Graph, whenAddEdges_shouldHaveEdgesAndAdjacentNodes)
{
    auto graph = Graph(10);
    graph.addEdge(Node(0), Node(1));
    graph.addEdge(Node(1), Node(3));
    ASSERT_EQ(2, graph.getEdges().size());
    ASSERT_EQ(10, graph.getAdjacencyList().size());
    ASSERT_EQ(1, graph.getAdjacencyList()[0].size());
    ASSERT_EQ(2, graph.getAdjacencyList()[1].size());
}