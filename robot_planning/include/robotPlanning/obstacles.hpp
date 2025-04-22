#ifndef __OBSTACLES_H__
#define __OBSTACLES_H__

#include "point.hpp"

enum obstacleType {CIRCLE, BOX};

class Obstacle{
private:
    obstacleType type;
    double radius;
    std::vector<Point> vertices;
public:
    Obstacle(double r, std::vector<Point> vs);
    std::vector<Point> getVertices();
    Point getCentroid();
    void convertToSquare();
    obstacleType getType();
    double getMinDist();
    bool isInsideObstacle(Point p) const;
    std::vector<double> getAbscissas();
    bool belongsToObstacle(Point p1, Point p2, Point p3, Point p4, bool extr);
    std::vector<Point> isIntersecting(double x);
};

#endif