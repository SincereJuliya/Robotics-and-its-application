#ifndef __OBSTACLES_H__
#define __OBSTACLES_H__

#include "point.hpp"

/*
    should we add the number of the point?
    from the structure to class? 
    the smallest edge of a rectangle is a minDistance for the radius..
    */
enum obstacleType {CIRCLE, BOX};

class Obstacle{
private:
    obstacleType type;
    double radius;
    std::vector<Point> vertices;
public:
    Obstacle(double r, std::vector<Point> vs);
    Point getCentroid();
    void convertToSqaure();
    obstacleType getType();
    double getMinDist();
    bool isInsideObstacle(Point p);
    std::vector<double> getAbscissas();
};

#endif