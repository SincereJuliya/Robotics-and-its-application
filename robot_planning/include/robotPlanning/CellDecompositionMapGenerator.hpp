// CellDecompositionMapGenerator.hpp
#include "IMapGenerator.hpp"
#include "point.hpp"
#include "graph.hpp"
#include "obstacles.hpp"
#include <vector>

class CellDecompositionMapGenerator : public IMapGenerator {
public:
    CellDecompositionMapGenerator();
    ~CellDecompositionMapGenerator() override = default;

    // IMapGenerator interface
    void setGates(const std::vector<Point>& gates) override;
    void setBorders(const std::vector<Point>& borders) override;
    void setObstacles(const std::vector<Obstacle>& obstacles) override;
    Graph generateGraph(const Point& init) override;


private:    
    Graph g_;
    std::vector<Obstacle> obstacles_;
    std::vector<Point> borders_;
    std::vector<Point> gates_;

    // Helper functions
    static bool sortPoints(const Point &a, const Point &b);
    Point getAreaCentralPoint(const std::vector<Point>& area);
    Point getLineCentralPoint(const Point& p1, const Point& p2);
    void convertCirclesToSquares();
    std::vector<double> getObstaclesUniqueAbscissas();
    std::vector<double> getObstaclesAndBordersUniqueAbscissas();
    std::vector<Point> getObstaclesPoints();
    std::vector<Point> getSortedObstaclesAndBordersPoints();
    std::vector<Point> getPointsAtGivenAbscissa(double x);
    bool isInsideObstacles(const Point &p);
    bool edgeBelongsToObstacle(const Point &p1, const Point &p2,
                               const Point &p3, const Point &p4, bool extr);
    void cellDecomposition();
    std::vector<Point> getIntersectionWithBorders(double x);
    std::vector<Point> getIntersectionWithObstacles(double x);
    std::vector<Point> nearestByYOffset(const std::vector<Point>& pts,
                                        const Point &p);
};
