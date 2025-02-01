#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include "point.hpp"
#include "graph_for_task_planner_msg/msg/graph.hpp"
#include "graph_for_task_planner_msg/msg/point.hpp"
#include "graph_for_task_planner_msg/msg/edge.hpp"


class Graph{
private:
    std::map<Point, std::vector<Point>> mGraph;
public:
    Graph();
    Graph(const Graph& g);
    void addVertice(Point vertex);
    void addEdge(Point v1, Point v2);
    void printGraph();
    void removeVertex(Point vertex);
    std::vector<Point> getVertices();
    std::vector<Point> getEdge(Point vertex);

    void clear();
    // Convert Graph to ROS 2 message
    graph_for_task_planner_msg::msg::Graph toROSMsg() const;

};

#endif