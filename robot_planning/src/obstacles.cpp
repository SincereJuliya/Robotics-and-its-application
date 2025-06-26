#include <math.h>
#include <vector>
#include <algorithm>
#include "../include/robotPlanning/obstacles.hpp"

Obstacle::Obstacle(double r, std::vector<Point> vs){
    radius = r;
    vertices = vs;
    if(r!=0){
        type = CIRCLE;
    }else{
        type = BOX;
    }
}

std::vector<Point> Obstacle::getVertices(){
    return vertices;
}

Point Obstacle::getCentroid(){
    if(type == BOX){
        double A=0; 
        double Cx=0;
        double Cy=0;
        vertices.push_back(vertices[0]);
        for(std::size_t i=0;i<vertices.size();i++){
            A += ((vertices[i].getX()*vertices[i+1].getY())-(vertices[i+1].getX()*vertices[i].getY()));
        }
        A/=2;
        for(std::size_t i=0;i<vertices.size();i++){
            Cx += ((vertices[i].getX()+vertices[i+1].getX())*(vertices[i].getX()*vertices[i+1].getY()-vertices[i+1].getX()*vertices[i].getY()));   
        }
        Cx /=(6*A);
        for(std::size_t i=0;i<vertices.size();i++){
            Cy += ((vertices[i].getY()+vertices[i+1].getY())*(vertices[i].getX()*vertices[i+1].getY()-vertices[i+1].getX()*vertices[i].getY()));
        }
        Cy /=(6*A);
        vertices.pop_back();
        return Point(Cx, Cy);
    }
    return vertices[0];
}

void Obstacle::convertToSquare(){
    if(type==BOX){
        return;
    }
    std::vector<Point> vec;
    vec.push_back(Point(vertices[0].getX()-radius, vertices[0].getY()+radius));
    vec.push_back(Point(vertices[0].getX()+radius, vertices[0].getY()+radius));
    vec.push_back(Point(vertices[0].getX()+radius, vertices[0].getY()-radius));
    vec.push_back(Point(vertices[0].getX()-radius, vertices[0].getY()-radius));
    vertices = vec;
    type = BOX;
}

obstacleType Obstacle::getType(){
    return type;
}

double Obstacle::getMinDist(){
    if(type==BOX){
        std::vector<double> edges;
        vertices.push_back(vertices[0]);
        for(std::size_t i=0;i<vertices.size()-1;i++){
            Point p1 = vertices[i];
            Point p2 = vertices[i+1];
            edges.push_back(sqrt(pow((p2.getX()-p1.getX()),2)+pow((p2.getY()-p1.getY()),2)));
        }
        vertices.pop_back();
        return *(std::min_element(edges.begin(), edges.end()));
    }
    return 2*radius;
}

bool Obstacle::isTooCloseToObstacle(const Point& p, double minDistance) const {
    double buffer = minDistance;
    //std::cout << "Checking if point " << p.toString() << " is too close to obstacle with type " << type << "\n";
    if (type == CIRCLE) {
        double dx = p.getX() - vertices[0].getX();  // center
        double dy = p.getY() - vertices[0].getY();
        double distSq = dx * dx + dy * dy;
        double safeRadius = radius + buffer;
        return distSq <= safeRadius * safeRadius;
        
    }

    if (type == BOX) {
        // Compute minimum distance from point to box edges or corners
        double px = p.getX(), py = p.getY();

        // Find box bounds (assumes 4 vertices, aligned or convex)
        double minX = vertices[0].getX(), maxX = vertices[0].getX();
        double minY = vertices[0].getY(), maxY = vertices[0].getY();
        for (const Point& v : vertices) {
            minX = std::min(minX, v.getX());
            maxX = std::max(maxX, v.getX());
            minY = std::min(minY, v.getY());
            maxY = std::max(maxY, v.getY());
        }

        // Expand the box by buffer (safe distance)
        minX -= buffer;
        maxX += buffer;
        minY -= buffer;
        maxY += buffer;

        // If point lies within the expanded box, it's too close
        return px >= minX && px <= maxX && py >= minY && py <= maxY;
    }

    return false;
}


bool Obstacle::isInsideObstacle(Point p) const{
    std::vector<Point> vertices_ = vertices;

    double buffer = 0.25;

    if(type==BOX){
        int cnt=0;
        vertices_.push_back(vertices_[0]);
        for(std::size_t i=0; i<vertices_.size()-1;i++){
            Point p1 = vertices_[i];
            Point p2 = vertices_[i+1];
            //with the following if check if the edges are parallel to any of the two axes and in case it was, check if the same or if it lays on the vertices_ 
            if((p1.getY()==p2.getY())&&(p1.getY()==p.getY())&&(p.getX()<p1.getX())!=(p.getX()<p2.getX())){
                    vertices_.pop_back();
                    return true;
                }
            if((p1.getX()==p2.getX())&&(p1.getX()==p.getX())&&(p.getY()<p1.getY())!=(p.getY()<p2.getY())){
                vertices_.pop_back();
                return true;
            }
            if(((p1.getX()==p.getX())&&(p1.getY()==p.getY()))||((p2.getX()==p.getX())&&(p2.getY()==p.getY()))){
                //need to add the check if in range of the obstacle edge                
                vertices_.pop_back();
                return true;
            }
            double err;
            //with the following check, check if the point lains on the line that connects two point and in case it was check if inside the range of x and y
            err = abs((((p2.getY()-p1.getY())/(p2.getX()-p1.getX()))*(p.getX()-p1.getX()))+p1.getY()- p.getY());
            if(err<buffer){ //still need to check if inside the range of the variables
                if((p.getY()<p1.getY())!=(p.getY()<p2.getY())&&(p.getX()<p1.getX())!=(p.getX()<p2.getX())){
                    vertices_.pop_back();
                    return true;
                }else {
                    vertices_.pop_back();
                    return false;
                }
            }
            //in case its not any of the previews cases
            if((p.getY()<p1.getY())!=(p.getY()<p2.getY())){
                double x0 = (p.getY()*(p1.getX()-p2.getX())-(p1.getX()-p2.getX())*p1.getY()+(p1.getY()-p2.getY())*p1.getX())/(p1.getY()-p2.getY()); 
                if(p.getX()<x0){ 
                    cnt++;
                }
            }
        }
        vertices_.pop_back();
        return ((cnt%2)==1);
    }
    return ((pow(p.getX()-vertices_[0].getX(),2)+pow(p.getY()-vertices_[0].getY(),2))<=(radius + buffer) * (radius + buffer));
    
}

bool Obstacle::belongsToObstacle(Point p1, Point p2, Point p3, Point p4, bool extr){
    int cnt = 0;
    for(std::size_t i=0; i<vertices.size();i++){
        if(p1==vertices[i] || p2==vertices[i] || p3==vertices[i] || p4==vertices[i]){
            cnt++;
        }
    }
    if((cnt==2 && !extr) || (cnt==1 && extr)){
        return true;
    }
    return false;
}

std::vector<double> Obstacle::getAbscissas(){
    std::vector<double> vec;
    
    for(size_t i=0; i<vertices.size(); i++){
        vec.push_back(vertices[i].getX());
    }
    return vec;
}

std::vector<Point> Obstacle::isIntersecting(double x){
    std::vector<Point> intersections;

        vertices.push_back(vertices[0]);
        for(std::size_t i=0; i<vertices.size()-1;i++){
            Point p1 = vertices[i];
            Point p2 = vertices[i+1];
            if(p1.getY() == p2.getY() && (x<p1.getX())!=(x<p2.getX())){
                intersections.push_back(Point{x, p1.getY()});
            }
            if(p1.getX() == p2.getX() && p1.getX() != x){
                continue;
            }
            double y = ((((p2.getY()-p1.getY())/(p2.getX()-p1.getX()))*(x-p1.getX()))+p1.getY());
            if((y<p1.getY())!=(y<p2.getY())&&(x<p1.getX())!=(x<p2.getX())){
                    intersections.push_back(Point{x, y});
            }
        }
        vertices.pop_back();
        return intersections;
}

double Obstacle::distanceTo(const Point& p) const {
    if (type == CIRCLE) {
        Point center = vertices[0];
        double dist = sqrt(pow(p.getX() - center.getX(), 2) + pow(p.getY() - center.getY(), 2));
        return std::max(0.0, dist - radius);
    } else if (type == BOX) {
        double minX = vertices[0].getX();
        double maxX = vertices[2].getX();
        double minY = vertices[0].getY();
        double maxY = vertices[2].getY();

        double dx = std::max({minX - p.getX(), 0.0, p.getX() - maxX});
        double dy = std::max({minY - p.getY(), 0.0, p.getY() - maxY});
        return sqrt(dx * dx + dy * dy);
    }
    return std::numeric_limits<double>::infinity();
}


/* int main(int argc, char** argv){    
    std::vector<Point> vec1 = {Point{-1.22492, -1.36989}, Point{-1.80755, -0.869648}, Point{-2.1845, -1.30868}, Point{-1.60187, -1.80892}};
    Obstacle o(0, vec1);
    std::vector<Point> vec2 = {Point{-1, 0.5}};
    Obstacle o2(0.5, vec2);
    std::vector<Point> vec3 = {Point(1, 1), Point(2,2), Point(3,1), Point(2,0)};
    Obstacle o3(0, vec3);

    std::cout << o.getMinDist() << "\n";
 
    o.getCentroid();
    o.getType();
    o2.convertToSqaure();
    //std::cout << o.isInsideObstacle(Point{-1.74056,-1.54777}) << "\n";
    //std::cout << o.isInsideObstacle(Point{-1,-2}) << "\n";
    //std::cout << o.isInsideObstacle(Point{-1.80755, -0.869648}) << "\n";
    //std::cout << o.isInsideObstacle(Point{-1.36161,-1.52907}) << "\n";
    //std::cout << o.isInsideObstacle(Point{-1,-1.105263158}) << "\n";

    std::cout << o.belongsToObstacle(Point{-1.80755, -0.869648}, Point{-1.60187, -1.80892}, Point(2,2), Point(3,1), false) << "\n";
    std::cout << o.belongsToObstacle(Point(2,2), Point(3,1), Point(2,2), Point(3,1),  true) << "\n"; 

    std::vector<Point> obs1 = {Point{-2, 0}, Point{0, 2}, Point{2, 0}, Point{0, -2}};
    Obstacle obs(0, obs1);

    std::vector<Point> intersections = obs.isIntersecting(-1);
    for(size_t i=0; i<intersections.size();i++){
        std::cout << intersections[i] << "\n";
    }

    return 0;
} */