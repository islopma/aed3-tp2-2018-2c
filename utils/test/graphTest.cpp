#include "gtest/gtest.h"
#include "../src/graph.h"


struct GraphTest: testing::Test{

    GraphTest(){
    }

    ~GraphTest(){
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

TEST_F(GraphTest, whenComparingEqGraphs_mustReturnTrue){
    Graph graph(3);
    graph.addEdge(Node(0),Node(1),5);
    graph.addEdge(Node(1),Node(2),5);

    Graph expectedGraph(3);
    expectedGraph.addEdge(Node(1),Node(2),5);
    expectedGraph.addEdge(Node(0),Node(1),5);

    ASSERT_TRUE(expectedGraph == graph);
}

TEST_F(GraphTest, whenComparingNotEqGraphs_mustReturnFalse) {
    Graph graph(3);
    graph.addEdge(Node(0), Node(1), 5);
    graph.addEdge(Node(1), Node(2), 5);

    Graph expectedGraph(3);
    expectedGraph.addEdge(Node(0), Node(2), 5);
    expectedGraph.addEdge(Node(1), Node(2), 5);

    ASSERT_FALSE(expectedGraph == graph);
}

TEST_F(GraphTest, whenAllVertexInTheSameComponent_mustReturnAllVertexInComponentOne){
    Graph graph(4);
    graph.addEdge(Node(0),Node(1), 5);
    graph.addEdge(Node(0),Node(2), 5);
    graph.addEdge(Node(1),Node(3), 5);
    graph.addEdge(Node(2),Node(3), 5);

    std::vector<int> actualComponent = graph.componenteConexaDeVertices();

    std::vector<int> expectedComponent(4,1);
    ASSERT_EQ( expectedComponent, actualComponent);
}

TEST_F(GraphTest, whenAllVertexInDifferentComponents_mustReturnAllVertexInDifferentComponent){
    Graph graph(4);

    std::vector<int> actualComponent = graph.componenteConexaDeVertices();

    int myints[] = {1,2,3,4};
    std::vector<int> expectedComponent (myints, myints + sizeof(myints) / sizeof(int) );
    ASSERT_EQ( expectedComponent, actualComponent);
}


TEST_F(GraphTest, whenGraphIsAForest_mustReturnAllVertexInDifferentComponent){
    Graph graph(8);
    graph.addEdge(Node(0),Node(2), 5);
    graph.addEdge(Node(0),Node(1), 5);

    graph.addEdge(Node(3),Node(3), 5);

    graph.addEdge(Node(4),Node(5), 5);
    graph.addEdge(Node(6),Node(5), 5);
    graph.addEdge(Node(6),Node(7), 5);

    std::vector<int> actualComponent = graph.componenteConexaDeVertices();

    ASSERT_EQ( actualComponent.at(0), actualComponent.at(1));
    ASSERT_EQ( actualComponent.at(0), actualComponent.at(2));

    GTEST_ASSERT_NE( actualComponent.at(3), actualComponent.at(0));

    ASSERT_EQ( actualComponent.at(4), actualComponent.at(5));
    ASSERT_EQ( actualComponent.at(4), actualComponent.at(6));
    ASSERT_EQ( actualComponent.at(4), actualComponent.at(7));

}


TEST_F(GraphTest, whenGraphIsAForest2_mustReturnAllVertexInDifferentComponent) {
    Graph graph(15);
    graph.addEdge(Node(0), Node(2), 5);
    graph.addEdge(Node(0), Node(1), 5);
    graph.addEdge(Node(0), Node(8), 5);
    graph.addEdge(Node(9), Node(8), 5);

    graph.addEdge(Node(3), Node(3), 5);

    graph.addEdge(Node(4), Node(5), 5);
    graph.addEdge(Node(6), Node(5), 5);
    graph.addEdge(Node(6), Node(7), 5);

    graph.addEdge(Node(10), Node(11), 5);
    graph.addEdge(Node(11), Node(12), 5);
    graph.addEdge(Node(11), Node(13), 5);
    graph.addEdge(Node(14), Node(10), 5);

    std::vector<int> actualComponent = graph.componenteConexaDeVertices();

    ASSERT_EQ(actualComponent.at(0), actualComponent.at(1));
    ASSERT_EQ(actualComponent.at(0), actualComponent.at(2));
    ASSERT_EQ(actualComponent.at(0), actualComponent.at(9));
    ASSERT_EQ(actualComponent.at(0), actualComponent.at(8));

    ASSERT_EQ(actualComponent.at(4), actualComponent.at(5));
    ASSERT_EQ(actualComponent.at(4), actualComponent.at(6));
    ASSERT_EQ(actualComponent.at(4), actualComponent.at(7));

    ASSERT_EQ(actualComponent.at(10), actualComponent.at(11));
    ASSERT_EQ(actualComponent.at(11), actualComponent.at(12));
    ASSERT_EQ(actualComponent.at(12), actualComponent.at(13));
    ASSERT_EQ(actualComponent.at(13), actualComponent.at(14));

    GTEST_ASSERT_NE(actualComponent.at(3), actualComponent.at(0));
    GTEST_ASSERT_NE(actualComponent.at(3), actualComponent.at(4));
    GTEST_ASSERT_NE(actualComponent.at(3), actualComponent.at(10));

    GTEST_ASSERT_NE(actualComponent.at(0), actualComponent.at(10));
    GTEST_ASSERT_NE(actualComponent.at(0), actualComponent.at(4));

    GTEST_ASSERT_NE(actualComponent.at(10), actualComponent.at(4));
}


//TESTS DEL SEGUNDO PROBLEMA

TEST_F(GraphTest, FloydPositivo3){          //hay res
    Graph grafo(3);
    grafo.addEdgeDirMinusLog(Node(0),Node(1),15);
    grafo.addEdgeDirMinusLog(Node(1),Node(0),0.3);
    grafo.addEdgeDirMinusLog(Node(0),Node(2),20);
    grafo.addEdgeDirMinusLog(Node(2),Node(0),0.05);
    grafo.addEdgeDirMinusLog(Node(1),Node(2),2);
    grafo.addEdgeDirMinusLog(Node(2),Node(1),0.5);
    bool res = grafo.getFloydCycle();

    EXPECT_EQ(res, true);

}


TEST_F(GraphTest, FloydPositivo4){          //hay res
    Graph grafo(4);
    grafo.addEdgeDirMinusLog(Node(0),Node(1),15);
    grafo.addEdgeDirMinusLog(Node(1),Node(0),0.3);
    grafo.addEdgeDirMinusLog(Node(0),Node(2),20);
    grafo.addEdgeDirMinusLog(Node(2),Node(0),0.05);
    grafo.addEdgeDirMinusLog(Node(1),Node(2),2);
    grafo.addEdgeDirMinusLog(Node(2),Node(1),0.5);
    grafo.addEdgeDirMinusLog(Node(3),Node(1),0.2);
    grafo.addEdgeDirMinusLog(Node(1),Node(3),5);
    grafo.addEdgeDirMinusLog(Node(3),Node(2),0.1);
    grafo.addEdgeDirMinusLog(Node(2),Node(3),10);
    grafo.addEdgeDirMinusLog(Node(3),Node(0),50);
    grafo.addEdgeDirMinusLog(Node(0),Node(3),0.02);

    bool res = grafo.getFloydCycle();

    EXPECT_EQ(res, true);

}

TEST_F(GraphTest, FloydotroPositivo3){          // hay res
    Graph grafo(3);
    grafo.addEdgeDirMinusLog(Node(0),Node(1),1);
    grafo.addEdgeDirMinusLog(Node(1),Node(0),1);
    grafo.addEdgeDirMinusLog(Node(0),Node(2),20);
    grafo.addEdgeDirMinusLog(Node(2),Node(0),0.05);
    grafo.addEdgeDirMinusLog(Node(1),Node(2),2);
    grafo.addEdgeDirMinusLog(Node(2),Node(1),0.5);
    bool res = grafo.getFloydCycle();

    EXPECT_EQ(res, true);

}

TEST_F(GraphTest, FloydNegativo3){          //no hay res, valen todas lo mismo
    Graph grafo(3);
    grafo.addEdgeDirMinusLog(Node(0),Node(1),1);
    grafo.addEdgeDirMinusLog(Node(1),Node(0),1);
    grafo.addEdgeDirMinusLog(Node(0),Node(2),1);
    grafo.addEdgeDirMinusLog(Node(2),Node(0),1);
    grafo.addEdgeDirMinusLog(Node(1),Node(2),1);
    grafo.addEdgeDirMinusLog(Node(2),Node(1),1);
    bool res = grafo.getFloydCycle();

    EXPECT_EQ(res, false);


}

TEST_F(GraphTest, FloydotroNegativo3){          //no hay res
    Graph grafo(3);
    grafo.addEdgeDirMinusLog(Node(0),Node(1),5);
    grafo.addEdgeDirMinusLog(Node(1),Node(0),0.2);
    grafo.addEdgeDirMinusLog(Node(0),Node(2),10);
    grafo.addEdgeDirMinusLog(Node(2),Node(0),0.1);
    grafo.addEdgeDirMinusLog(Node(1),Node(2),2);
    grafo.addEdgeDirMinusLog(Node(2),Node(1),0.5);
    bool res = grafo.getFloydCycle();

    EXPECT_EQ(res, false);


}
