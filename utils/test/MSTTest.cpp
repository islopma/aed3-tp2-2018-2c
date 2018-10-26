//
// Created by Christian nahuel Rivera on 14/10/18.
//
#include "gtest/gtest.h"
#include "../src/graph.h"

struct MSTTest : testing::Test {
    Graph *graph;
    Graph *graph1;
    Graph *graph2;

    MSTTest(){
        graph = new Graph();

        graph1 = new Graph();
        graph1->build(4);
        graph1->addEdge(Node(0), Node(1), 3);
        graph1->addEdge(Node(0), Node(3), 5);
        graph1->addEdge(Node(1), Node(2), 8);
        graph1->addEdge(Node(1), Node(3), 1);
        graph1->addEdge(Node(2), Node(3), 2);

        graph2 = new Graph();
        graph2->build(9);
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

    ~MSTTest(){
        delete graph;
        delete graph1;
        delete graph2;
    }
};

TEST_F(MSTTest,whenGetPrimMSTOfATree_mustReturnTheSameGraph){
    graph->build(2);
    graph->addEdge(Node(0),Node(1),5);

    graph->setMSTStrategy(new PrimMST());
    Graph actualMST = graph->getMST();

    Graph expectedMST(2);
    expectedMST.addEdge(Node(0),Node(1),5);
    ASSERT_EQ(actualMST.getTotalWeight(),expectedMST.getTotalWeight());
}

TEST_F(MSTTest,whenGetPrimMSTOfACircuit_mustReturnAnMSTOfTheOriginalGraph){
    // A circuit of 3 nodes
    graph->build(3);
    graph->addEdge(Node(0),Node(1),5);
    graph->addEdge(Node(0),Node(2),200);
    graph->addEdge(Node(1),Node(2),5);

    graph->setMSTStrategy(new PrimMST());
    Graph actualMST = graph->getMST();

    Graph expectedMST(3);
    expectedMST.addEdge(Node(0),Node(1),5);
    expectedMST.addEdge(Node(1),Node(2),5);
    ASSERT_EQ(expectedMST.getTotalWeight() , actualMST.getTotalWeight());
}

TEST_F(MSTTest,whenGetKruskalDefaultMSTOfATree_mustReturnTheSameGraph){
    graph->build(2);
    graph->addEdge(Node(0),Node(1),5);

    graph->setMSTStrategy(new KruskalDefaultMST());
    Graph actualMST = graph->getMST();

    Graph expectedMST(2);
    expectedMST.addEdge(Node(0),Node(1),5);
    ASSERT_EQ(actualMST.getTotalWeight(),expectedMST.getTotalWeight());
}

TEST_F(MSTTest,whenGetKruskalDefaultMSTOfACircuit_mustReturnAnMSTOfTheOriginalGraph){
    // A circuit of 3 nodes
    graph->build(3);
    graph->addEdge(Node(0),Node(1),5);
    graph->addEdge(Node(0),Node(2),200);
    graph->addEdge(Node(1),Node(2),5);

    graph->setMSTStrategy(new KruskalDefaultMST());
    Graph actualMST = graph->getMST();

    Graph expectedMST(3);
    expectedMST.addEdge(Node(0),Node(1),5);
    expectedMST.addEdge(Node(1),Node(2),5);
    ASSERT_EQ(expectedMST.getTotalWeight() , actualMST.getTotalWeight());
}

TEST_F(MSTTest,whenGetKruskalCompressedMSTOfATree_mustReturnTheSameGraph){
    graph->build(2);
    graph->addEdge(Node(0),Node(1),5);

    graph->setMSTStrategy(new KruskalCompressedMST());
    Graph actualMST = graph->getMST();

    Graph expectedMST(2);
    expectedMST.addEdge(Node(0),Node(1),5);
    ASSERT_EQ(actualMST.getTotalWeight(),expectedMST.getTotalWeight());
}

TEST_F(MSTTest,whenGetKruskalCompressedMSTOfACircuit_mustReturnAnMSTOfTheOriginalGraph){
    // A circuit of 3 nodes
    graph->build(3);
    graph->addEdge(Node(0),Node(1),5);
    graph->addEdge(Node(0),Node(2),200);
    graph->addEdge(Node(1),Node(2),5);

    graph->setMSTStrategy(new KruskalCompressedMST());
    Graph actualMST = graph->getMST();

    Graph expectedMST(3);
    expectedMST.addEdge(Node(0),Node(1),5);
    expectedMST.addEdge(Node(1),Node(2),5);
    ASSERT_EQ(expectedMST.getTotalWeight() , actualMST.getTotalWeight());
}

TEST_F(MSTTest, whenGraph1PrimMST_shouldHaveTotalWeight6)
{
    graph1->setMSTStrategy(new PrimMST());
    auto mst = graph1->getMST();

    float actualWeight = mst.getTotalWeight();
    ASSERT_EQ(6, actualWeight);
}

TEST_F(MSTTest, whenGraph2PrimMST_shouldHaveTotalWeight37)
{
    graph2->setMSTStrategy(new PrimMST());
    auto mst = graph2->getMST();

    float actualWeight = mst.getTotalWeight();
    ASSERT_EQ(37, actualWeight);
}

TEST_F(MSTTest, whenGraph1KruskalDefaultMST_shouldHaveTotalWeight6)
{
    graph1->setMSTStrategy(new KruskalDefaultMST());
    auto mst = graph1->getMST();

    float actualWeight = mst.getTotalWeight();
    ASSERT_EQ(6, actualWeight);
}

TEST_F(MSTTest, whenGraph2KruskalDefaultMST_shouldHaveTotalWeight37)
{
    graph2->setMSTStrategy(new KruskalDefaultMST());
    auto mst = graph2->getMST();

    float actualWeight = mst.getTotalWeight();
    ASSERT_EQ(37, actualWeight);
}

TEST_F(MSTTest, whenGraph1KruskalCompressedMST_shouldHaveTotalWeight6)
{
    graph1->setMSTStrategy(new KruskalCompressedMST());
    auto mst = graph1->getMST();

    float actualWeight = mst.getTotalWeight();
    ASSERT_EQ(6, actualWeight);
}

TEST_F(MSTTest, whenGraph2KruskalCompressedMST_shouldHaveTotalWeight37)
{
    graph2->setMSTStrategy(new KruskalCompressedMST());
    auto mst = graph2->getMST();

    float actualWeight = mst.getTotalWeight();
    ASSERT_EQ(37, actualWeight);
}