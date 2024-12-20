#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

class Graph{

public:

    Graph() {}

    void addVertice(int vertex){
        mGraph[vertex] = {};
    }
    void addEdge(int v1, int v2){
        mGraph[v1].push_back(v2);
        mGraph[v2].push_back(v1);
    }
    void printGraph(){
        for( const auto& element: mGraph){
            std::cout << element.first << " : ";
            for( const auto& edge: element.second){
                std::cout << edge << " ";
            }
            std::cout << "\n";
        }
    }

    void removeVertex(int vertex){
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

    std::vector<int> getVertices(){
        std::cout << "vertices: " ;
        std::vector<int> vertices;

        for(auto& element: mGraph){
            vertices.push_back(element.first);
            std::cout << element.first << " ";
        }
        std::cout << "\n";

    return vertices;
    }

    std::vector<int> getEdge(int vertex){
        if(mGraph.find(vertex) == mGraph.end()){
            std::cout << "Invalid vertex \n" ;
            return {};
        }
        
        std::cout << "edges: " ;
        for(const auto& i: mGraph[vertex]){
            std::cout << i << " ";

        }
        std::cout << "\n";
        return mGraph[vertex]; 
    }

private:
    std::map<int, std::vector<int>> mGraph;

};

int main(){
    Graph mg;

    mg.addVertice(1);
    mg.addVertice(3);
    mg.addVertice(2);
    mg.addEdge(1,2);
    mg.addEdge(2,3);
    mg.addEdge(1,3);

    mg.printGraph();

    mg.removeVertex(3);
    mg.printGraph(); 
    mg.getEdge(1);
    mg.getVertices();
    return 0;
}