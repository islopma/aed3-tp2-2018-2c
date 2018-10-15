#include "gtest/gtest.h"
#include "../src/graph.h"

struct GraphTest : testing::Test {
    GraphTest(){
    }
    ~GraphTest(){}

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

TEST_F(GraphTest, whenComparingEqGraphs_mustReturnTrue){
    Graph graph(3);
    graph.addEdge(Node(0),Node(1),5);
    graph.addEdge(Node(1),Node(2),5);

    Graph expectedGraph(3);
    expectedGraph.addEdge(Node(1),Node(2),5);
    expectedGraph.addEdge(Node(0),Node(1),5);

    ASSERT_TRUE(expectedGraph == graph);
}

TEST_F(GraphTest, whenComparingNotEqGraphs_mustReturnFalse){
    Graph graph(3);
    graph.addEdge(Node(0),Node(1),5);
    graph.addEdge(Node(1),Node(2),5);

    Graph expectedGraph(3);
    expectedGraph.addEdge(Node(0),Node(2),5);
    expectedGraph.addEdge(Node(1),Node(2),5);

    ASSERT_FALSE(expectedGraph == graph);
}