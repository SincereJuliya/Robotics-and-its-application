#include <math.h>
#include <vector>

#include "robotPlanning/obstacles.hpp"

Obstacle::Obstacle(double r, std::vector<Point> vs){
    radius = r;
    vertices = vs;
    if(r!=0){
        type = CIRCLE;
        std::cout << "circle\n";
    }else{
        type = BOX;
        std::cout << "box\n";
    }
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

void Obstacle::convertToSqaure(){
    if(type==BOX){
        return;
    }
    std::vector<Point> vec;
    vec.push_back(Point(vertices[0].getX()-radius, vertices[0].getY()+radius));
    vec.push_back(Point(vertices[0].getX()+radius, vertices[0].getY()+radius));
    vec.push_back(Point(vertices[0].getX()+radius, vertices[0].getY()-radius));
    vec.push_back(Point(vertices[0].getX()-radius, vertices[0].getY()-radius));
    vertices = vec;
}

obstacleType Obstacle::getType(){
    return type;
}

double Obstacle::getMinDist(){
    if(type==BOX){
        
    }
    return 2*radius;
}

bool Obstacle::isInsideObstacle(Point p){
    if(type==BOX){
        int cnt=0;
        vertices.push_back(vertices[0]);
        for(std::size_t i=0; i<vertices.size();i++){
            Point p1 = vertices[i];
            Point p2 = vertices[i+1];
            if(((p1.getY()==p2.getY())&&(p1.getY()==p.getY()))||((p1.getX()==p2.getX())&&(p1.getX()==p.getX()))||((p1.getX()==p.getX())&&(p1.getY()==p.getY()))||((p2.getX()==p.getX())&&(p2.getY()==p.getY()))){
                return true;
            }
            if(p.getY()==(((p1.getY()-p2.getY())/(p1.getX()-p2.getX()))*p.getX()+p1.getY()*((p1.getY()-p2.getY())/(p1.getX()-p2.getX()))*p1.getX())){ //da testare
                return true;
            }
            if((p.getY()<p1.getY())!=(p.getY()<p2.getY())){
                double x0 = (p.getY()*(p1.getX()-p2.getX())-(p1.getX()-p2.getX())*p1.getY()+(p1.getY()-p2.getY())*p1.getX())/(p1.getY()-p2.getY()); 
                if(p.getX()<x0){ 
                    cnt++;
                }
            }
        }
        std::cout << cnt << "\n";
        vertices.pop_back();
        if((cnt%2)==1){
            std::cout << "is inside\n";
        }else{
            std::cout << "is outside\n";
        }
        return ((cnt%2)==1);
    }
    return ((pow(p.getX()-vertices[0].getX(),2)+pow(p.getY()-vertices[0].getY(),2))<pow(radius,2)); 
    
}

std::vector<double> Obstacle::getAbscissas(){
    std::vector<double> vec;
    for(size_t i; i<vertices.size(); i++){
        vec.push_back(vertices[i].getX());
    }
    return vec;
}
/* int main(int argc, char** argv){
    std::vector<Point> vec1 = {Point{-1.22492, -1.36989}, Point{-1.80755, -0.869648}, Point{-2.1845, -1.30868}, Point{-1.60187, -1.80892}};
    Obstacle o(0, vec1);
    std::vector<Point> vec2 = {Point{-1, -2}};
    Obstacle o2(2, vec2);

    o.getCentroid();
    o.getType();
    o2.convertToSqaure();
    std::cout << o.isInsideObstacle(Point{-1.74056,-1.54777}) << "\n";
    std::cout << o.isInsideObstacle(Point{-1,-2}) << "\n";
    std::cout << o.isInsideObstacle(Point{-1.80755, -0.869648}) << "\n";
    std::cout << o.isInsideObstacle(Point{-1.36, -1.53}) << "\n";

    return 0;
} */