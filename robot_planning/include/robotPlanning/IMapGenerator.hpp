#ifndef IMAPGENERATOR_HPP
#define IMAPGENERATOR_HPP

#include <vector>
#include <map>
#include "graph.hpp"         
#include "obstacles.hpp"    

class IMapGenerator
{
    public:
        Graph G;
        virtual Graph toGenerateMap(int interactions) = 0;
        std::vector<Obstacle> toGetObstacles();
        bool isItInObstacle(Point p);
        bool isItSafe();
};

#endif // IMAPGENERATOR_HPP
