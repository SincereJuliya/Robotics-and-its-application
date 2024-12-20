#ifndef __VARIABLES_H__
#define __VARIABLES_H__

#include <vector>
#include <iostream>

typedef struct Point {
    double x;
    double y;
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
