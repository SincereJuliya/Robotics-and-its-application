#include "stdio.h"
#include <vector>
#include <map>
#include <iostream>
#include "../include/robotPlanning/graph.hpp"
#include "../include/robotPlanning/obstacles.hpp"

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
            return std::vector<Obstacle>(); // Return a new vector by value, not a pointer
        }

    
        bool isItInObstacle(Point p)
        {
            return false;
        };

        bool isItSafe()
        {
            return true;
        };

};