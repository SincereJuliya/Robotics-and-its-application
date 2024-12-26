#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include "robotPlanning/graph.hpp"

Graph::Graph() {}

void Graph::addVertice(Point vertex){
    mGraph[vertex] = {};
}

void Graph::addEdge(Point v1, Point v2){
    mGraph[v1].push_back(v2);
    mGraph[v2].push_back(v1);
}

void Graph::printGraph(){
    std::map<Point, std::vector<Point>>::const_iterator citer;
    for(citer=mGraph.cbegin(); citer!=mGraph.cend();++citer){
        std::cout << citer->first << " --> ";
        std::vector<Point>::iterator it;
        for (const auto& edge : citer->second) {
            std::cout << edge << " ";
        }
        std::cout << "\n";
    }
}

void Graph::removeVertex(Point vertex){
    if(mGraph.find(vertex) == mGraph.end()){
        std::cout << "Invalid vertex \n" ;
        return;
    }
    mGraph.erase(vertex);
    for(auto& element: mGraph){
        auto& value = element.second;
        value.erase(std::remove(value.begin(), value.end(), vertex), value.end());
    }
}

std::vector<Point> Graph::getVertices(){
    std::cout << "vertices: " ;
    std::vector<Point> vertices;

    for(auto& element: mGraph){
        vertices.push_back(element.first);
        std::cout << element.first << "  ";
    }
    std::cout << "\n";

return vertices;
}

std::vector<Point> Graph::getEdge(Point vertex){
    if(mGraph.find(vertex) == mGraph.end()){
        std::cout << "Invalid vertex \n" ;
        return {};
}
    
std::cout << "edges of vertex (" << vertex.getX() <<  "," << vertex.getY() << "): " ;
    for(const auto& i: mGraph[vertex]){
        std::cout << i << " ";

    }
    std::cout << "\n";
    return mGraph[vertex]; 
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