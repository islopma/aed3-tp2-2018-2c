//
// Created by Christian nahuel Rivera on 14/10/18.
//

#include "gtest/gtest.h"
#include "../src/graph.h"

struct DisSetTest : testing::Test{
    DisjoinSet *disjoinSet;
    DisSetTest(){
        //  disjoinSet = new DisjoinSetDefault();
        disjoinSet = new DisjoinSetCompressed();
    }
    ~DisSetTest(){
        delete disjoinSet;
    }
};

TEST_F(DisSetTest,whenCreateADisjoinSet_mustAddEveryNodeIntoADifferentComponent){
    Graph graph(3);

    disjoinSet->create(&graph);

    ASSERT_EQ(0, disjoinSet->find(Node(0)));
    ASSERT_EQ(1, disjoinSet->find(Node(1)));
    ASSERT_EQ(2, disjoinSet->find(Node(2)));
}

TEST_F(DisSetTest, whenJoiningTwoComponents_mustPutThemTheSameComponentId){
    Graph graph(3);
    disjoinSet->create(&graph);

    disjoinSet->join(Node(0),Node(1));

    ASSERT_EQ(0, disjoinSet->find(Node(0)));
    ASSERT_EQ(0, disjoinSet->find(Node(1)));
}

TEST_F(DisSetTest, whenJoiningComponentsWithMoreThanOneElement_mustPutTheSameIdToAllElements){
    Graph graph(6);
    disjoinSet->create(&graph);
    disjoinSet->join(Node(0),Node(1));
    disjoinSet->join(Node(2),Node(3));
    disjoinSet->join(Node(4),Node(5));

    // En este momento tengo un disjoinSet con 3 componentes conexas: 0-1, 2-3 y 4-5
    disjoinSet->join(Node(0),Node(2));
    disjoinSet->join(Node(0),Node(4));

    // Junté todos los ejes entán en la misma componente conexa
    ASSERT_EQ(0, disjoinSet->find(Node(0)));
    ASSERT_EQ(0, disjoinSet->find(Node(1)));
    ASSERT_EQ(0, disjoinSet->find(Node(2)));
    ASSERT_EQ(0, disjoinSet->find(Node(3)));
    ASSERT_EQ(0, disjoinSet->find(Node(4)));
    ASSERT_EQ(0, disjoinSet->find(Node(5)));
}