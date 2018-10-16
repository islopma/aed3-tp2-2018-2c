//
// Created by Christian nahuel Rivera on 14/10/18.
//

#include "graph.h"

void DisjoinSetDefault::create(Graph *graph) {
    std::vector<int> components;
    components.resize(graph->getAdjacencyList().size());
    for(int index = 0; index < components.size(); index++){
        components.at(index) = index;
    }
    this->components = components;
}

int DisjoinSetDefault::find(Node nodo) {
    return this->components.at(nodo.id);
}

void DisjoinSetDefault::join(Node alreadyIn, Node newNode) {
    int originalComponent = find(newNode);
    for (int &component : this->components) {
        if(component == originalComponent){
            component = find(alreadyIn);
        }
    }
}

void DisjoinSetCompressed::create(Graph *graph) {
    std::vector<int> components;
    std::vector<int> heights;
    components.resize(graph->getAdjacencyList().size());
    heights.resize(graph->getAdjacencyList().size());
    for(int index = 0; index < components.size(); index++){
        components.at(index) = index;
        heights.at(index) = 1;
    }
    this->components = components;
    this->heights = heights;
}

int DisjoinSetCompressed::find(Node nodo) {
    // Si soy hijo, voy a buscar a mi padre y actualizo
    if( this->components.at(nodo.id) != nodo.id){
        this->components.at(nodo.id) = find(this->components.at(nodo.id));
    }
    return this->components.at(nodo.id);
}

void DisjoinSetCompressed::join(Node alreadyIn, Node newNode) {
    int lRepresentative = find(alreadyIn);
    int rRepresentative = find(newNode);

    if(heights.at(lRepresentative) < heights.at(rRepresentative)){
        this->components.at(lRepresentative) = rRepresentative;
    }else{
        this->components.at(rRepresentative) = lRepresentative;
    }
    if(heights.at(lRepresentative) == heights.at(rRepresentative)) {
        this->heights.at(lRepresentative) = this->heights.at(lRepresentative) + 1;
    }
}


