#ifndef __OBSTACLES_H__
#define __OBSTACLES_H__

#include "variables.hpp"

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
};

#endif