#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include "variables.hpp"

class Graph{
private:
    std::map<Point, std::vector<Point>> mGraph;
public:
    Graph();
    void addVertice(Point vertex);
    void addEdge(Point v1, Point v2);
    void printGraph();
    void removeVertex(Point vertex);
    std::vector<Point> getVertices();
    std::vector<Point> getEdge(Point vertex);

};