#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/obstacles.hpp"
#include "../include/robotPlanning/graph.hpp"

class CellDecomposition{
private:
    Graph g;
    std::vector<Obstacle> obstacles;
    std::vector<Point> borders;
public:
    CellDecomposition(std::vector<Obstacle> ob, std::vector<Point> bord){
        obstacles = ob;
        borders = bord;
    }

    static bool sortPoints(const Point &a, const Point &b){
        return a<b;
    }

    Point getAreaCenteralPoint(std::vector<Point> area){
        double Cx=0;
        double Cy=0;

        for(std::size_t i=0;i<area.size();i++){
            Cx += area[i].getX();
            Cy += area[i].getY();   
        }
        Cx /=area.size();
        Cy /=area.size();

        return Point{Cx, Cy};
    }

    Point getLineCenteralPoint(Point p1, Point p2){
        return (Point{p1.getX(),p1.getY()-((p1.getY()-p2.getY())/2)});
    }

    void convertCirclesToSquares(){
        for(size_t i=0; i<obstacles.size(); i++){
            if(obstacles[i].getType()==CIRCLE){
                obstacles[i].convertToSqaure();
            }
        }
    }

    std::vector<double> getObstaclesUniqueAbscissas(){
        std::vector<double> vec;
        std::vector<double> x;

        for(size_t i=0; i<obstacles.size(); i++){
            if(obstacles[i].getType()==CIRCLE){
                obstacles[i].convertToSqaure();
            }
            vec = obstacles[i].getAbscissas();
            for(size_t i=0; i<vec.size(); i++){
                x.push_back(vec[i]);
            }
        }

        std::sort( x.begin(), x.end() ); 
        x.erase( std::unique( x.begin(), x.end() ), x.end() );
        return x;
    }

    std::vector<double> getObstalcesAndBordersUniqueAbscissas(){
        std::vector<double> x;
        x = getObstaclesUniqueAbscissas();

        for(size_t i=0; i<borders.size(); i++){
            x.push_back(borders[i].getX());
        }

        sort( x.begin(), x.end() ); 
        x.erase( unique( x.begin(), x.end() ), x.end() );
        return x;
    }
    
    std::vector<Point> getObstaclesPoints(){
        std::vector<Point> obstaclesVector;
        std::vector<Point> vertices;

        for(size_t i=0; i<obstacles.size(); i++){
            if(obstacles[i].getType()==CIRCLE){
                obstacles[i].convertToSqaure();
            }
            vertices = obstacles[i].getVertices();
            for(size_t j=0; j<vertices.size(); j++){
                obstaclesVector.push_back(vertices[j]);
            }
        }

        return obstaclesVector;
    }

    std::vector<Point> getSortedObstalcesAndBordersPoints(){
        std::vector<Point> unified;
        std::vector<Point> obstaclesVector;

        obstaclesVector = getObstaclesPoints();
        
        std::set_union(obstaclesVector.cbegin(), obstaclesVector.cend(), borders.cbegin(), borders.cend(), std::back_inserter(unified));
        std::sort(unified.begin(), unified.end());

        return unified;
        
    }

    std::vector<Point> getPointsAtGivenAbscissa(double x){
        std::vector<Point> vec;
        std::vector<Point> points;

        points = getSortedObstalcesAndBordersPoints();
        for(size_t i=0; i<points.size(); i++){
            if(points[i].getX() == x){
                vec.push_back(points[i]);
            }
        }
        return vec;
    }

    bool isInsideObstacles(Point p){
        for(size_t i=0; i<obstacles.size(); i++){
            if(obstacles[i].isInsideObstacle(p)){
                return true;
            }
        }
        return false;
    }

    std::vector<Point> getBorders(){
        return borders;
    }

    bool edgeBelongsToObstacle(Point p1, Point p2, Point p3, Point p4, bool extr){
        for(size_t i=0; i<obstacles.size(); i++){
            if(obstacles[i].belongsToObstacle(p1, p2, p3, p4, extr)){
                return true;
            }
        }
        return false;
    }

   bool isOnBorder(Point p){
        borders.push_back(borders[0]);
        for(std::size_t i=0; i<borders.size();i++){
            Point p1 = borders[i];
            Point p2 = borders[i+1];
            //with the following if check if the edges are parallel to any of the two axes and in case it was, check if the same or if it lays on the vertices 
            if(((p1.getY()==p2.getY())&&(p1.getY()==p.getY()))||((p1.getX()==p2.getX())&&(p1.getX()==p.getX()))||((p1.getX()==p.getX())&&(p1.getY()==p.getY()))||((p2.getX()==p.getX())&&(p2.getY()==p.getY()))){
                borders.pop_back();
                return true;
            }
            double err;
            //with the following check, check if the point laine on the line that connects two point and in case it was check if inside the range of x and y
            err = abs((((p2.getY()-p1.getY())/(p2.getX()-p1.getX()))*(p.getX()-p1.getX()))+p1.getY()- p.getY());
            if(err<0.0028){ //still need to check if inside the range of the variables
                if((p.getY()<p1.getY())!=(p.getY()<p2.getY())&&(p.getX()<p1.getX())!=(p.getX()<p2.getX())){
                    borders.pop_back();
                    return true;
                }else {
                    borders.pop_back();
                    return false;
                }
            }
        }
        borders.pop_back();
        return false;
    }

    std::vector<Point> getIntersectionWithBorders(double x){
            std::vector<Point> intersections;

            borders.push_back(borders[0]);
            for(std::size_t i=0; i<borders.size()-1;i++){
                Point p1 = borders[i];
                Point p2 = borders[i+1];
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
            borders.pop_back();
            return intersections;
    }

    std::vector<Point> getIntersectionWithObstacless(double x){
        std::vector<Point> intersections;
        std::vector<Point> obsIntersections;

        for(std::size_t i=0; i<obstacles.size();i++){
            obsIntersections = obstacles[i].isIntersecting(x);
            std::set_union(obsIntersections.cbegin(), obsIntersections.cend(), obsIntersections.cbegin(), obsIntersections.cend(), std::back_inserter(intersections));
        }
        return intersections;
    }
    
    std::vector<Point> nearestByYOffset(std::vector<Point> ps, Point p){
        std::vector<Point> pts;

        if (ps.size()==0) {
            pts.push_back(Point{0, 0});
            return pts;
        }
        
        double minOffset = std::abs(ps[0].getY() - p.getY());
        for (size_t i = 1; i < ps.size(); ++i) {
            double offset = std::abs(ps[i].getY() - p.getY());
            if (offset < minOffset) {
                minOffset = offset;
            }
        }
        for (size_t i = 0; i < ps.size(); ++i) {
            double offset = std::abs(ps[i].getY() - p.getY());
            if (offset == minOffset) {
                pts.push_back(ps[i]);
            }
        }
        return pts;
    }

    void cellDecomposition(){
        std::vector<Point> PointsL;
        std::vector<Point> PointsR;
        Point pArea;
        Point pLine;
        std::vector<Point> points; 
        std::map<Point, std::pair<Point, Point>> pointsMapR;
        std::map<Point, std::pair<Point, Point>> pointsMapL;
        bool extremes = true;
        std::vector<double> uniqueAbscissas = getObstalcesAndBordersUniqueAbscissas();
        points = getPointsAtGivenAbscissa(uniqueAbscissas[0]);
        if(points.size()==1){
            pointsMapL[points[0]] = std::pair<Point,Point>(points[0],points[0]);
            PointsL.push_back(points[0]);
        }else{
            pLine = getLineCenteralPoint(points[0], points[1]);
            PointsL.push_back(pLine);
            pointsMapL[pLine] = std::pair<Point,Point>(points[0], points[1]);
        }
        points.clear();
        for(size_t i=1; i<uniqueAbscissas.size(); i++){ 
            points = getPointsAtGivenAbscissa(uniqueAbscissas[i]);
            if(i!=(uniqueAbscissas.size()-1)){
                points.push_back(getIntersectionWithBorders(uniqueAbscissas[i])[0]);
                points.push_back(getIntersectionWithBorders(uniqueAbscissas[i])[1]);
            }
            std::vector<Point> intersWithObst = getIntersectionWithObstacless(uniqueAbscissas[i]);
            for(size_t z=0; z<intersWithObst.size(); z++){
                points.push_back(intersWithObst[z]);
            }
            sort( points.begin(), points.end() ); 
            points.erase( unique( points.begin(), points.end() ), points.end() );
            if(i!=(uniqueAbscissas.size()-1)){
                for(size_t j=0; j<points.size()-1; j++){
                    Point p1, p2;
                    p1 = points[j];
                    p2 = points[j+1];
                    pLine = getLineCenteralPoint(p1, p2);
                    if(isInsideObstacles(pLine)){
                        continue;
                    }else{
                        std::vector<Point> pts = nearestByYOffset(PointsL, pLine);
                        for(size_t h=0;h<pts.size();h++){
                            Point pl = pts[h];
                            pArea = getAreaCenteralPoint(std::vector<Point>{pointsMapL[pl].first, pointsMapL[pl].second, p1, p2});
                            g.addVertice(pArea);
                            if(i!=1){
                                g.addEdge(pl, pArea);
                            }
                            if(h==0){
                                pointsMapR[pLine] = std::pair<Point,Point>(p1,p2);
                                PointsR.push_back(pLine);
                                g.addVertice(pLine);    
                            }                    
                            g.addEdge(pArea, pLine);
                        }
                    }
                }
            }else{
                Point p1, p2;
                if(points.size()==1){
                    p1 = points[0];
                    p2 = points[0];
                }else{
                    p1 = points[0];
                    p2 = points[1];
                }
                pLine = getLineCenteralPoint(p1, p2);
                Point shifted_pLine = Point{(pLine.getX()-0.27), pLine.getY()};
                for(size_t j=0; j<PointsL.size(); j++){
                    Point pl = PointsL[j];
                    pArea = getAreaCenteralPoint(std::vector<Point>{pointsMapL[pl].first, pointsMapL[pl].second, p1, p2});
                    g.addVertice(pArea);
                    if(!isInsideObstacles(shifted_pLine)){   
                        if(j==0){
                            g.addVertice(shifted_pLine);
                        }
                        g.addEdge(pArea, shifted_pLine);
                    }
                }
            }
            std::cout << "\n";
            PointsL.clear();
            PointsL = PointsR;
            PointsR.clear();
            pointsMapL.clear();
            pointsMapL = pointsMapR;
            pointsMapR.clear();
            points.clear();
            extremes = false;
            if(i == (uniqueAbscissas.size()-2)){
                extremes = true;
            }
        }
    }

    void printGraph(){
        std::cout << "\n";
        g.printGraph();
    }
}; 

int main(){
    /* std::vector<Point> borders = {Point{-4, -3}, Point{-4, 3}, Point{4, 3}, Point{4, -3}};
    
    std::vector<Point> vec1 = {Point{-1.22492, -1.36989}, Point{-1.80755, -0.869648}, Point{-2.1845, -1.30868}, Point{-1.60187, -1.80892}};
    Obstacle o(0, vec1);
    std::vector<Point> vec2 = {Point{-1, 0.5}};
    Obstacle o2(0.5, vec2);
    std::vector<Point> vec3 = {Point(1, 1), Point(2,2), Point(3,1), Point(2,0)};
    Obstacle o3(0, vec3);

    std::vector<Obstacle> obs = {o,o2, o3};
    CellDecomposition c(obs, borders); 

    std::vector<Point> vec = c.getSortedObstalcesAndBordersPoints();
    std::cout << c.edgeBelongsToObstacle(Point{-1.80755, -0.869648}, Point{-1.60187, -1.80892}) << "\n";
    std::cout << c.edgeBelongsToObstacle(Point{-1.80755, -0.869648}, Point{-1, 0.5}) << "\n";
    std::cout << c.edgeBelongsToObstacle(Point(2,2), Point(3,1)) << "\n";
    
    for(int i =0; i< vec.size(); i++){
        std::cout <<  vec[i] << "\n";
    }*/
    std::vector<Point> borders = {Point{-5, 5}, Point{5, 5}, Point{5, -5}, Point{-5, -5}};    
    std::vector<Point> obs1 = {Point{-2, 0}, Point{0, 2}, Point{2, 0}, Point{0, -2}};
    std::vector<Point> obs2 = {Point{1, -2}, Point{3, -2}, Point{3, -4}, Point{1, -4}};
    std::vector<Point> obs3 = {Point{1, 4}, Point{3, 4}, Point{3, 2}, Point{1, 2}};
    Obstacle obs(0, obs1);
    Obstacle obs_2(0, obs2);
    Obstacle obs_3(0, obs3);
    std::vector<Obstacle> obstacles = {obs, obs_2, obs_3};

    CellDecomposition c(obstacles, borders);
    c.cellDecomposition();
    c.printGraph();

    return 0;
}