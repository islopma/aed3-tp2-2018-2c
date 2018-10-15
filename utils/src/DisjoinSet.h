//
// Created by Christian nahuel Rivera on 14/10/18.
//

#ifndef TP2_MODELADO_CON_GRAFOS_DISJOINSET_H
#define TP2_MODELADO_CON_GRAFOS_DISJOINSET_H

#include "graph.h"

class DisjoinSet {
public:
    int find(Node nodo);
    void join(Node alreadyIn, Node newNode);

private:

};


#endif //TP2_MODELADO_CON_GRAFOS_DISJOINSET_H
