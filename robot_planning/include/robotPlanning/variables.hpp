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
} Point;

typedef struct Circle {
    Point center;
    double radius;
} Circle;

typedef struct Square {
    std::vector<Point> obstacle;
} Square;

typedef struct Obstacle {
    std::vector<Point> vertices;
    double radius;
} Obstacle;

#endif
