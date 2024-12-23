#include "../include/robotPlanning/variables.hpp"
#include "../include/robotPlanning/obstacles.hpp"
#include <math.h>
#include <vector>

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
Obstacle::Point getCentroid(){
    if(type == BOX){
        double A=0; 
        double Cx=0;
        double Cy=0;
        vertices.push_back(vertices[0]);
        for(int i=0;i<vertices.size();i++){
            A += ((vertices[i].x*vertices[i+1].y)-(vertices[i+1].x*vertices[i].y));
        }
        A/=2;
        for(int i=0;i<vertices.size();i++){
            Cx += ((vertices[i].x+vertices[i+1].x)*(vertices[i].x*vertices[i+1].y-vertices[i+1].x*vertices[i].y));   
        }
        Cx /=(6*A);
        for(int i=0;i<vertices.size();i++){
            Cy += ((vertices[i].y+vertices[i+1].y)*(vertices[i].x*vertices[i+1].y-vertices[i+1].x*vertices[i].y));
        }
        Cy /=(6*A);
        vertices.pop_back();
        return Point{Cx, Cy};
    }
    return vertices[0];
}

Obstacle::void convertToSqaure(){
    if(type==BOX){
        return;
    }
    std::vector<Point> vec;
    vec.push_back(Point{vertices[0].x-radius, vertices[0].y+radius});
    vec.push_back(Point{vertices[0].x+radius, vertices[0].y+radius});
    vec.push_back(Point{vertices[0].x+radius, vertices[0].y-radius});
    vec.push_back(Point{vertices[0].x-radius, vertices[0].y-radius});
    vertices = vec;
}

Obstacle::obstacleType getType(){
    return type;
}

Obstacle::double getMinDist(){
    if(type==BOX){
        
    }
    return 2*radius;
}

Obstacle::bool isInsideObstacle(Point p){
    if(type==BOX){
        int cnt=0;
        vertices.push_back(vertices[0]);
        for(int i=0; i<vertices.size();i++){
            Point p1 = vertices[i];
            Point p2 = vertices[i+1];
            if(((p1.y==p2.y)&&(p1.y==p.y))||((p1.x==p2.x)&&(p1.x==p.x))||((p1.x==p.x)&&(p1.y==p.y))||((p2.x==p.x)&&(p2.y==p.y))){
                //add also check if point is on the line----------------------------
                return true;
            }
            if((p.y<p1.y)!=(p.y<p2.y)){
                double x0 = (p.y*(p1.x-p2.x)-(p1.x-p2.x)*p1.y+(p1.y-p2.y)*p1.x)/(p1.y-p2.y); 
                if(p.x<x0){ 
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
    return ((pow(p.x-vertices[0].x,2)+pow(p.y-vertices[0].y,2))<pow(radius,2)); 
    
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