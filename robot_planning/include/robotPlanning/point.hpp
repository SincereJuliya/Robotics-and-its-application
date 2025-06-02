#ifndef __POINT_H_
#define __POINT_H_

#include <iostream>
#include <math.h>

class Point{
private:
    double x;
    double y;
public:
    Point();
    Point(double _x, double _y);
    double getX()const;
    double getY()const;
    void setX(double _x);
    void setY(double _y);
    double computeEuclideanDistance(const Point& p);
    bool operator <(const Point& p )const;
    bool operator==(const Point& p) const;
    bool operator!=(const Point& p) const;
    friend std::ostream& operator <<(std::ostream& os, const Point& p);
    std::string toString() const; 
    
};

std::ostream& operator <<(std::ostream& os, const Point& p);

#endif