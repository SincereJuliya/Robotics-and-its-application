// SampleBasedMapGenerator.hpp
#include "IMapGenerator.hpp"
#include "point.hpp"
#include "graph.hpp"
#include "obstacles.hpp"
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <random>

class SampleBasedMapGenerator : public IMapGenerator {
public:
    SampleBasedMapGenerator();
    ~SampleBasedMapGenerator() override = default;

    // IMapGenerator interface
    void setGates(const std::vector<Point>& gates) override;
    void setBorders(const std::vector<Point>& borders) override;
    void setObstacles(const std::vector<Obstacle>& obstacles) override;
    Graph generateGraph(const Point& init) override;

private:
    std::vector<Point> gates_;
    std::vector<Point> borders_;
    std::vector<Obstacle> obstacles_;
    Graph G_;

    float getSearchRadius() const;
    float getRandomPosition(float middle, float r) const;
    Point getRandomPoint(const Point& initP, float radius) const;

    bool isInsideBorder(const Point& p) const;
    bool isTooCloseToBorder(const Point& p, double minDistance) const;
    bool isInsideAnyObstacle(const Point& p) const;
    bool isReachedGate(const Point& p) const;

    float computeDistance(const Point& a, const Point& b) const;
    Point findNearest(const Graph& graph, const Point& p) const;


    bool collidesWithObstacleOrBorder(const Point& p1, const Point& p2) const;
    void connectVisibleEdges(Graph& graph, const Point& newP) const;

};
