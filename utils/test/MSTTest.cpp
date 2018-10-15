//
// Created by Christian nahuel Rivera on 14/10/18.
//
#include "gtest/gtest.h"
#include "../src/graph.h"

struct MSTTest : testing::Test {
    MSTTest(){}
    ~MSTTest(){}
};

TEST_F(MSTTest,whenGetKruskalMSTOfATree_mustReturnTheSameGraph){
    Graph graph(2);
    graph.addEdge(Node(0),Node(1),5);
    graph.addEdge(Node(0),Node(1),200);

    Graph actualMST = graph.getMSTKruskal();

    Graph expectedMST(3);
    expectedMST.addEdge(Node(0),Node(1),5);
    ASSERT_EQ(actualMST.getTotalWeigth(),expectedMST.getTotalWeigth());
}

TEST_F(MSTTest,whenGetKruskalMSTOfACircuit_mustReturnAnMSTOfTheOriginalGraph){
    // A circuit of 3 nodes
    Graph graph(3);
    graph.addEdge(Node(0),Node(1),5);
    graph.addEdge(Node(0),Node(2),200);
    graph.addEdge(Node(1),Node(2),5);

    Graph actualMST = graph.getMSTKruskal();

    Graph expectedMST(3);
    expectedMST.addEdge(Node(0),Node(1),5);
    expectedMST.addEdge(Node(1),Node(2),5);
    ASSERT_EQ(expectedMST.getTotalWeigth() , actualMST.getTotalWeigth());
}

TEST_F(MSTTest, whenGraph1PrimMST_shouldHaveTotalWeight6)
{
    Graph graph1(4);
    graph1.addEdge(Node(0), Node(1), 3);
    graph1.addEdge(Node(0), Node(3), 5);
    graph1.addEdge(Node(1), Node(2), 8);
    graph1.addEdge(Node(1), Node(3), 1);
    graph1.addEdge(Node(2), Node(3), 2);

    auto mst = graph1.getPrimMST();

    float actualWeigth = mst.getTotalWeigth();
    ASSERT_EQ(6, actualWeigth);
}

TEST_F(MSTTest, whenGraph2PrimMST_shouldHaveTotalWeight37)
{
    Graph graph2(9);
    graph2.addEdge(Node(0), Node(1), 4);
    graph2.addEdge(Node(0), Node(7), 8);
    graph2.addEdge(Node(1), Node(7), 11);
    graph2.addEdge(Node(1), Node(2), 8);
    graph2.addEdge(Node(2), Node(3), 7);
    graph2.addEdge(Node(2), Node(5), 4);
    graph2.addEdge(Node(2), Node(8), 2);
    graph2.addEdge(Node(3), Node(4), 9);
    graph2.addEdge(Node(3), Node(5), 14);
    graph2.addEdge(Node(4), Node(5), 10);
    graph2.addEdge(Node(5), Node(6), 2);
    graph2.addEdge(Node(6), Node(7), 1);
    graph2.addEdge(Node(6), Node(8), 6);
    graph2.addEdge(Node(7), Node(8), 7);

    auto mst = graph2.getPrimMST();

    float actualWeigth = mst.getTotalWeigth();
    ASSERT_EQ(37, actualWeigth);
}