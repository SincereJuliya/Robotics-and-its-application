/* #ifndef IMAPGENERATOR_HPP
#define IMAPGENERATOR_HPP

#include <vector>
#include <map>
#include "graph.hpp"         // Include necessary headers for Graph and Point
#include "obstacles.hpp"     // Include necessary headers for Obstacle

class IMapGenerator
{
    public:
        Graph G;

        // Pure virtual function to generate the map (to be implemented by derived classes)
        virtual Graph toGenerateMap(int interactions) = 0;
        
        // Returns a vector of obstacles (dynamically created; consider returning by value)
        std::vector<Obstacle> toGetObstacles();
    
        // Checks if the point is inside an obstacle
        bool isItInObstacle(Point p);

        // Checks if the area is safe
        bool isItSafe();
};

#endif // IMAPGENERATOR_HPP */

#ifndef IMAPGENERATOR_HPP
#define IMAPGENERATOR_HPP

#include "graph.hpp"
#include "point.hpp"
#include "obstacles.hpp"
#include <vector>

class IMapGenerator {
public:
    virtual ~IMapGenerator() = default;

    virtual void setGates(const std::vector<Point>& gates) = 0;
    virtual void setBorders(const std::vector<Point>& borders) = 0;
    virtual void setObstacles(const std::vector<Obstacle>& obstacles) = 0;
    virtual Graph generateGraph(const Point& init) = 0;
};

#endif // IMAPGENERATOR_HPP
