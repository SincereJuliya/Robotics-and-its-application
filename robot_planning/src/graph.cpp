#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include "../include/robotPlanning/graph.hpp"

Graph::Graph() {}

Graph::Graph(const Graph &g)
{
    mGraph = g.mGraph;
}

void Graph::addVertice(Point vertex)
{
    if (!containsVertex(vertex))
    {
        mGraph[vertex] = {};
    }
}

void Graph::addEdge(Point v1, Point v2)
{
    if (!edgeExists(v1, v2))
    {
        mGraph[v1].push_back(v2);
        mGraph[v2].push_back(v1);
    }
}

bool Graph::edgeExists(const Point &v1, const Point &v2) const
{
    auto it = mGraph.find(v1);
    if (it != mGraph.end())
    {
        const std::vector<Point> &neighbors = it->second;
        return std::any_of(neighbors.begin(), neighbors.end(), [&v2](const Point &p)
                           { return p == v2; });
    }
    return false;
}

void Graph::printGraph()
{
    std::map<Point, std::vector<Point>>::const_iterator citer;
    for (citer = mGraph.cbegin(); citer != mGraph.cend(); ++citer)
    {
        std::cout << citer->first << " --> ";
        std::vector<Point>::iterator it;
        for (const auto &edge : citer->second)
        {
            std::cout << edge << " ";
        }
        std::cout << "\n";
    }
}

void Graph::removeVertex(Point vertex)
{
    if (mGraph.find(vertex) == mGraph.end())
    {
        std::cout << "Invalid vertex \n";
        return;
    }
    mGraph.erase(vertex);
    for (auto &element : mGraph)
    {
        auto &value = element.second;
        value.erase(std::remove(value.begin(), value.end(), vertex), value.end());
    }
}

std::vector<Point> Graph::getVertices() const
{
    std::vector<Point> vertices;

    for (auto &element : mGraph)
    {
        vertices.push_back(element.first);
    }

    return vertices;
}

std::vector<Point> Graph::getEdge(Point vertex)
{
    if (mGraph.find(vertex) == mGraph.end())
    {
        std::cout << "Invalid vertex \n";
        return {};
    }
    return mGraph[vertex];
}

void Graph::clear()
{
    mGraph.clear();
}

graph_for_task_planner_msg::msg::Graph Graph::toROSMsg() const
{
    graph_for_task_planner_msg::msg::Graph graph_msg;

    // Map for storing point indices
    std::map<Point, int> point_index;
    int index = 0;

    // Convert vertices to ROS message
    for (const auto &pair : mGraph)
    {
        graph_for_task_planner_msg::msg::Point ros_point;
        ros_point.x = pair.first.getX();
        ros_point.y = pair.first.getY();
        graph_msg.vertices.push_back(ros_point);
        point_index[pair.first] = index++; // Store index for later use in edges
    }

    // Convert edges to ROS message
    for (const auto &pair : mGraph)
    {
        // Iterate over the neighbors
        for (const Point &neighbor : pair.second)
        {          
            if (point_index.find(pair.first) != point_index.end() &&
                point_index.find(neighbor) != point_index.end())
            {
                // Create the edge
                graph_for_task_planner_msg::msg::Edge edge;

                // Start point
                graph_for_task_planner_msg::msg::Point start_point;
                start_point.x = pair.first.getX();
                start_point.y = pair.first.getY();
                edge.start_point = start_point;

                // End point (neighbor)
                graph_for_task_planner_msg::msg::Point end_point;
                end_point.x = neighbor.getX();
                end_point.y = neighbor.getY();
                edge.end_point = end_point;

                // Add edge to the message
                graph_msg.edges.push_back(edge);
            }
        }
    }

    return graph_msg;
}

Point Graph::findNearestPoint(const Point &goal) const
{
    if (mGraph.empty())
    {
        std::cerr << "Graph is empty. Cannot find the nearest point.\n";
        throw std::runtime_error("Graph is empty");
    }

    auto it = mGraph.begin();
    Point nearest_point = it->first;
    double min_distance = std::sqrt(std::pow(nearest_point.getX() - goal.getX(), 2) +
                                    std::pow(nearest_point.getY() - goal.getY(), 2));

    for (++it; it != mGraph.end(); ++it)
    {
        const Point &current_point = it->first;
        double distance = std::sqrt(std::pow(current_point.getX() - goal.getX(), 2) +
                                    std::pow(current_point.getY() - goal.getY(), 2));
        if (distance < min_distance && distance > 0)
        { 
            min_distance = distance;
            nearest_point = current_point;
        }
    }
    return nearest_point;
}

bool Graph::containsPoint(const Point &point) const
{
    return mGraph.find(point) != mGraph.end();
}

bool Graph::containsVertex(const Point &p) const
{
    return mGraph.find(p) != mGraph.end();
}

/* int main(){
    Graph mg;

    mg.addVertice(Point{1,1});
    mg.addVertice(Point{1,3});
    mg.addVertice(Point{1,2});
    mg.addVertice(Point{1,4});
    mg.addEdge(Point{1,1},Point{1,2});
    mg.addEdge(Point{1,2},Point{1,3});
    mg.addEdge(Point{1,1},Point{1,3});
    mg.addEdge(Point{1,1},Point{1,4});
    std::cout << "printing graph\n";
    mg.printGraph();

    mg.removeVertex({1,3});

    std::cout << "\nprinting graph after removal of an edge\n";
    mg.printGraph();
    mg.getEdge({1,1});
    mg.getVertices();
    return 0;
} */