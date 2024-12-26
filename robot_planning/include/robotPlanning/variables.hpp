#ifndef __VARIABLES_H__
#define __VARIABLES_H__

#include <vector>
#include <iostream>

typedef struct Point {
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

    std::string getInfo(const Point& o){
        return ("{" + std::to_string(o.x) + "," + std::to_string(o.y) + "}"); 
    }
} Point;

/* typedef struct Circle {
    Point center;
    double radius;
} Circle;

typedef struct Square {
    std::vector<Point> obstacle;
} Square; */


class Obstacle {
public:
    enum Shapes { Circle, Square };

    std::vector<Point> vertices;
    double radius;

    float isInside(Obstacle& o)
    {
        return 0;
    };

};

#endif
