//
// Created by Christian nahuel Rivera on 14/10/18.
//
#include "gtest/gtest.h"
#include "../src/graph.h"

struct MSTTest : testing::Test {
    Graph *graph;

    MSTTest(){
        graph = new Graph();
        graph->setMSTStrategy(new PrimMST());
    }

    ~MSTTest(){
        delete graph;
    }
};

TEST_F(MSTTest,whenGetKruskalMSTOfATree_mustReturnTheSameGraph){
    graph->build(2);
    graph->addEdge(Node(0),Node(1),5);
    graph->addEdge(Node(0),Node(1),200);

    Graph actualMST = graph->getMST();

    Graph expectedMST(3);
    expectedMST.addEdge(Node(0),Node(1),5);
    ASSERT_EQ(actualMST.getTotalWeigth(),expectedMST.getTotalWeigth());
}

TEST_F(MSTTest,whenGetKruskalMSTOfACircuit_mustReturnAnMSTOfTheOriginalGraph){
    // A circuit of 3 nodes
    graph->build(3);
    graph->addEdge(Node(0),Node(1),5);
    graph->addEdge(Node(0),Node(2),200);
    graph->addEdge(Node(1),Node(2),5);

    Graph actualMST = graph->getMST();

    Graph expectedMST(3);
    expectedMST.addEdge(Node(0),Node(1),5);
    expectedMST.addEdge(Node(1),Node(2),5);
    ASSERT_EQ(expectedMST.getTotalWeigth() , actualMST.getTotalWeigth());
}

TEST_F(MSTTest, whenGraph1PrimMST_shouldHaveTotalWeight6)
{
    graph->build(4);
    graph->addEdge(Node(0), Node(1), 3);
    graph->addEdge(Node(0), Node(3), 5);
    graph->addEdge(Node(1), Node(2), 8);
    graph->addEdge(Node(1), Node(3), 1);
    graph->addEdge(Node(2), Node(3), 2);

    auto mst = graph->getMST();

    float actualWeigth = mst.getTotalWeigth();
    ASSERT_EQ(6, actualWeigth);
}

TEST_F(MSTTest, whenGraph2PrimMST_shouldHaveTotalWeight37)
{
    graph->build(9);
    graph->addEdge(Node(0), Node(1), 4);
    graph->addEdge(Node(0), Node(7), 8);
    graph->addEdge(Node(1), Node(7), 11);
    graph->addEdge(Node(1), Node(2), 8);
    graph->addEdge(Node(2), Node(3), 7);
    graph->addEdge(Node(2), Node(5), 4);
    graph->addEdge(Node(2), Node(8), 2);
    graph->addEdge(Node(3), Node(4), 9);
    graph->addEdge(Node(3), Node(5), 14);
    graph->addEdge(Node(4), Node(5), 10);
    graph->addEdge(Node(5), Node(6), 2);
    graph->addEdge(Node(6), Node(7), 1);
    graph->addEdge(Node(6), Node(8), 6);
    graph->addEdge(Node(7), Node(8), 7);

    auto mst = graph->getMST();

    float actualWeigth = mst.getTotalWeigth();
    ASSERT_EQ(37, actualWeigth);
}