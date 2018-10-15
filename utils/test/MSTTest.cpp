//
// Created by Christian nahuel Rivera on 14/10/18.
//
#include "gtest/gtest.h"
#include "../src/graph.h"

struct MSTTest : testing::Test {
    MSTTest(){}
    ~MSTTest(){}
};

TEST_F(MSTTest,test_0){
    Graph graph(2);
    graph.addEdge(Node(0),Node(1),5);
    graph.addEdge(Node(0),Node(1),200);

    Graph actualMST = graph.getMSTKruskal();

    Graph expectedMST(3);
    expectedMST.addEdge(Node(0),Node(1),5);
    ASSERT_TRUE(actualMST == expectedMST);
}

TEST_F(MSTTest,test_1){
    // A circuit of 3 nodes
    Graph graph(3);
    graph.addEdge(Node(0),Node(1),5);
    graph.addEdge(Node(0),Node(2),200);
    graph.addEdge(Node(1),Node(2),5);

    Graph actualMST = graph.getMSTKruskal();

    Graph expectedMST(3);
    expectedMST.addEdge(Node(0),Node(1),5);
    expectedMST.addEdge(Node(1),Node(2),5);
    ASSERT_TRUE(actualMST == expectedMST);
}