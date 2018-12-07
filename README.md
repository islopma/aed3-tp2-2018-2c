Instrucciones para compilar
===
1. Ejecutar CMake: cmake CMakeLists.txt
2. Compilar: make

Ejecutables
===
1. tp2_modelado_con_grafos -> corre tests con google-test
2. clustering -> recibe por standard input el problema a resolver y devuelve el resultado por standard output (es el que pide el enunciado).
    Hay que pasarle los argumentos obligatorios sobre el modo de resolver el problema:
        ./clustering {Prim|Kruskal|KruskalCompressed} {StandardDeviation|Factor} {neighborhoodDepth} {inconsistencyParameter}
        e.g. ./clustering Kruskal StandardDeviation 3 2.5
3. arbitraje -> recibe por standard input el problema a resolver y devuelve el resultado por standard output (es el que pide el enunciado).
    Hay que pasarle los argumentos obligatorios sobre el modo de resolver el problema:
        ./arbitraje {BellmanFord|FloydWarshall}
        e.g. ./arbitraje FloydWarshall
4. clustering_time -> recibe los mismos argumentos y entrada que clustering pero devuelve el tiempo en ns que demora en resolverlo
5. arbitraje_time -> recibe los mismos argumentos y entrada que arbitraje pero devuelve el tiempo en ns que demora en resolverlo