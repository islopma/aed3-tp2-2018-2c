//
// Created by Christian nahuel Rivera on 14/10/18.
//

#include "graph.h"

int DisjoinSet::find(Node nodo) {
    return this->components.at(nodo.id);
}

void DisjoinSet::join(Node alreadyIn, Node newNode) {
    int originalComponent = find(newNode);
    for (int &component : this->components) {
        if(component == originalComponent){
            component = find(alreadyIn);
        }
    }
}

DisjoinSet::DisjoinSet(Graph *graph) {
    std::vector<int> components;
    components.resize(graph->getAdjacencyList().size());
    for(int index = 0; index < components.size(); index++){
        components.at(index) = index;
    }
    this->components = components;
}
