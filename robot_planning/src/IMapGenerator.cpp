#include "stdio.h"
#include <vector>
#include <map>
#include <iostream>
#include <graph.hpp>
#include <obstacles.hpp>

/*
    1. the communication between nodes: services, messages...
    2. how to organize map generator with 2 diff aproaches (cell + sample)
    3. discuss the algorithm (the paper)
*/
class IMapGenerator
{
    public:
        Graph G;

        virtual Graph toGenerateMap(int interactions)
        {           
            return G; 
        };
        
        std::vector<Obstacle> toGetObstacles()
        {
            return (new std::vector<Obstacle>());
        }
    
        bool isItInObstacle(Point p)
        {
            return false;
        };

        bool isItSafe()
        {
            return true;
        };

}