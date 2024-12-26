#ifndef __OBSTACLES_H__
#define __OBSTACLES_H__

#include "variables.hpp"

struct Point {
    double x;
    double y;
    
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const Point& other) const {
        if (x != other.x)
            return x < other.x;
        return y < other.y;
    }
};

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