#include "robotPlanning/variables.hpp"
#include "robotPlanning/obstacles.hpp"
#include "robotPlanning/graph.hpp"

class CellDecomposition{
private:
    Graph g;
    std::vector<Obstacle> obstacles;
    std::vector<double> abscissas;
public:
    CellDecomposition(std::vector<Obstacle> ob){
        //obstacles = ob;
        abscissas = getUniqueAbscissas(obstacles);
    }
    Point getAreaCenteralPoint(std::vector<Point> area){
        double A=0; 
        double Cx=0;
        double Cy=0;
        area.push_back(area[0]);
        for(std::size_t i=0;i<area.size();i++){
            A += ((area[i].x*area[i+1].y)-(area[i+1].x*area[i].y));
        }
        A/=2;
        for(std::size_t i=0;i<area.size();i++){
            Cx += ((area[i].x+area[i+1].x)*(area[i].x*area[i+1].y-area[i+1].x*area[i].y));   
        }
        Cx /=(6*A);
        for(std::size_t i=0;i<area.size();i++){
            Cy += ((area[i].y+area[i+1].y)*(area[i].x*area[i+1].y-area[i+1].x*area[i].y));
        }
        Cy /=(6*A);
        area.pop_back();
        return Point{Cx, Cy};
    }
    Point getLineCenteralPoint(Point p1, Point p2){
        return (Point{(p1.x-p2.x),p1.y});
    }
    std::vector<double> getUniqueAbscissas(std::vector<Obstacle> obs){
        std::vector<double> vec;
        std::vector<double> x;

        for(size_t i=0; i<obs.size(); i++){
            vec = obs[i].getAbscissas();
            for(size_t i=0; i<obs[i].getAbscissas().size(); i++){
                x.push_back(obs[i].getAbscissas()[i]);
                //x.push_back(vec[i]);
            }
        }

        sort( x.begin(), x.end() ); x.erase( unique( x.begin(), x.end() ), x.end() );
    }
    Graph cellDecomposition(std::vector<Obstacle> obstacles){
        for(size_t i=0; i<obstacles.size(); i++){

        }
    }
};

/*
------------------TODO------------------
- read data from topic
- try implementing cell-decomposition
*/

int main(int argc, char** argv){
    CellDecomposition();

    
    return 0;
}