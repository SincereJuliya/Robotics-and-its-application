
#include "../include/robotPlanning//point.hpp"

Point::Point(){
    x=0;
    y=0;
}
Point::Point(double _x, double _y){
    x=_x;
    y=_y;
}
double Point::getX()const{
    return x;
}
double Point::getY()const{
    return y;
}
void Point::setX(double _x){
    x=_x;
}
void Point::setY(double _y){
    y=_y;
}
double Point::computeEuclideanDistance(const Point& p){
    return (sqrt(pow((p.x-x),2)+pow((p.y-y),2)));
}
bool Point::operator <(const Point& p )const{
    if (x != p.x)
        return x < p.x;
    return y > p.y;
}
bool Point::operator==(const Point& p) const {
    return x == p.x && y == p.y;
}

std::ostream& operator <<(std::ostream& os, const Point& p){
    return os << "(" << p.getX() << "," << p.getY() << ")";
}