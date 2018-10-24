#include "gtest/gtest.h"
#include "../src/graph.h"

struct Bellman : testing::Test {
    Graph *graph;

    Bellman(){
        graph = new Graph();
        
    }

    ~Bellman(){
        delete graph;
    }
};

TEST_F(Bellman, sinSolucion){
    graph->build(3);
    graph->addEdgeDir(Node(0),Node(1), 0.4);
    graph->addEdgeDir(Node(1),Node(0), 0.4);
    graph->addEdgeDir(Node(1),Node(2), 0.4);
    graph->addEdgeDir(Node(2),Node(1), 0.4);
    graph->addEdgeDir(Node(0),Node(2), 0.4);
    graph->addEdgeDir(Node(2),Node(0), 0.4);

    
   	EXPECT_EQ(graph->BellmanFord(), "NO");
}

TEST_F(Bellman, sinSolucion2){
    graph->build(3);
    graph->addEdgeDir(Node(0),Node(1), 0.4);
    graph->addEdgeDir(Node(1),Node(0), 0.2);
    graph->addEdgeDir(Node(1),Node(2), 0.4);
    graph->addEdgeDir(Node(2),Node(1), 0.2);
    graph->addEdgeDir(Node(0),Node(2), 0.05);
    graph->addEdgeDir(Node(2),Node(0), 0.2);


    EXPECT_EQ(graph->BellmanFord(), "NO");
}

TEST_F(Bellman, sinSolucion3){
    graph->build(3);
    graph->addEdgeDir(Node(0),Node(1), 0.4);
    graph->addEdgeDir(Node(1),Node(0), 0.2);
    graph->addEdgeDir(Node(1),Node(2), 0.4);
    graph->addEdgeDir(Node(2),Node(1), 0.2);
    graph->addEdgeDir(Node(0),Node(2), 0.09);
    graph->addEdgeDir(Node(2),Node(0), 1.99);

    EXPECT_EQ(graph->BellmanFord(), "NO");
}

TEST_F(Bellman, dosNodos){
    graph->build(2);
    graph->addEdgeDir(Node(0),Node(1), 1.3);
    graph->addEdgeDir(Node(1),Node(0), 1.3);
    
   	EXPECT_EQ(graph->BellmanFord(), "SI 1 0");
}

TEST_F(Bellman, dosNodos2){
    graph->build(2);
    graph->addEdgeDir(Node(0),Node(1), 1.35);
    graph->addEdgeDir(Node(1),Node(0), 0.80);

    EXPECT_EQ(graph->BellmanFord(), "SI 1 0");
}

TEST_F(Bellman, dosNodos3){
    graph->build(2);
    graph->addEdgeDir(Node(0),Node(1), 1.35);
    graph->addEdgeDir(Node(1),Node(0), 0.65);

    EXPECT_EQ(graph->BellmanFord(), "NO");
}

TEST_F(Bellman, tresNodos){
    graph->build(3);
    graph->addEdgeDir(Node(0),Node(1), 1.35);
    graph->addEdgeDir(Node(1),Node(2), 1.65);
    graph->addEdgeDir(Node(2),Node(0), 1.65);

    EXPECT_EQ(graph->BellmanFord(), "SI 2 1 0");
}

TEST_F(Bellman, tresNodos2){
    graph->build(3);
    graph->addEdgeDir(Node(0),Node(1), 1.35);
    graph->addEdgeDir(Node(1),Node(2), 0.50);
    graph->addEdgeDir(Node(2),Node(0), 1.65);

    EXPECT_EQ(graph->BellmanFord(), "SI 2 1 0");
}

TEST_F(Bellman, tresNodos3){
    graph->build(3);
    graph->addEdgeDir(Node(0),Node(1), 1.35);
    graph->addEdgeDir(Node(1),Node(0), 0.90);
    graph->addEdgeDir(Node(1),Node(2), 0.50);
    graph->addEdgeDir(Node(2),Node(0), 1.65);

    EXPECT_NE(graph->BellmanFord(), "NO");
}

TEST_F(Bellman, tresNodos4){
    graph->build(3);
    graph->addEdgeDir(Node(0),Node(1), 1.35);
    graph->addEdgeDir(Node(1),Node(2), 0.50);
    graph->addEdgeDir(Node(2),Node(0), 1.65);

    EXPECT_EQ(graph->BellmanFord(), "SI 2 1 0");
}

TEST_F(Bellman, cuatroNodos){
    graph->build(4);
    graph->addEdgeDir(Node(0),Node(1), 0.005);
    graph->addEdgeDir(Node(1),Node(2), 0.9);
    graph->addEdgeDir(Node(2),Node(1), 1.65);
    graph->addEdgeDir(Node(2),Node(3), 0.14);

    EXPECT_EQ(graph->BellmanFord(), "SI 2 1");
}

TEST_F(Bellman, cincoNodos){
    graph->build(5);
    graph->addEdgeDir(Node(0),Node(1), 1.333);
    graph->addEdgeDir(Node(1),Node(0), 0.75);
    graph->addEdgeDir(Node(1),Node(2), 1.1);
    graph->addEdgeDir(Node(0),Node(2), 6);
    graph->addEdgeDir(Node(0),Node(3), 7);
    graph->addEdgeDir(Node(3),Node(0), 0.142857);
    graph->addEdgeDir(Node(3),Node(2), 0.8);
    graph->addEdgeDir(Node(2),Node(4), 54);
    graph->addEdgeDir(Node(4),Node(3), 0.0225);


    EXPECT_NE(graph->BellmanFord(), "NO");
}

