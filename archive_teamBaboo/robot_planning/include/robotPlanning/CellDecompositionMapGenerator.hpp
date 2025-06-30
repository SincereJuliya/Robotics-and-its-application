/**
 * @file CellDecompositionMapGenerator.hpp
 * @brief Defines the CellDecompositionMapGenerator class for generating maps using cell decomposition.
 */

#pragma once

#include "IMapGenerator.hpp"
#include "point.hpp"
#include "graph.hpp"
#include "obstacles.hpp"
#include <vector>

/**
 * @class CellDecompositionMapGenerator
 * @brief Implements a map generator using cell decomposition.
 *
 * Generates a navigational graph based on borders, obstacles, and gates using a cell decomposition strategy.
 */
class CellDecompositionMapGenerator : public IMapGenerator {
public:
    /**
     * @brief Constructs a new CellDecompositionMapGenerator object.
     */
    CellDecompositionMapGenerator();

    /**
     * @brief Destroys the CellDecompositionMapGenerator object.
     */
    ~CellDecompositionMapGenerator() override = default;

    /// @name IMapGenerator Interface
    /// @{
    
    /**
     * @brief Sets the gate points used in map generation.
     * @param gates Vector of gate points.
     */
    void setGates(const std::vector<Point>& gates) override;

    /**
     * @brief Sets the borders (map limits) used in the environment.
     * @param borders Vector of border points.
     */
    void setBorders(const std::vector<Point>& borders) override;

    /**
     * @brief Sets the obstacles in the environment.
     * @param obstacles Vector of obstacle objects.
     */
    void setObstacles(const std::vector<Obstacle>& obstacles) override;

    /**
     * @brief Generates a graph using cell decomposition starting from an initial point.
     * @param init Initial point (robot's starting position).
     * @return Graph object representing the decomposed map.
     */
    Graph generateGraph(const Point& init) override;

    /// @}

private:
    Graph g_;                             ///< Graph to be built.
    std::vector<Obstacle> obstacles_;     ///< List of obstacles.
    std::vector<Point> borders_;          ///< Map borders.
    std::vector<Point> gates_;            ///< Gate positions.

    /// @name Helper Functions
    /// @{

    /**
     * @brief Sorts points in increasing order.
     * @param a First point.
     * @param b Second point.
     * @return True if a < b.
     */
    static bool sortPoints(const Point &a, const Point &b);

    /**
     * @brief Calculates the centroid of a given polygonal area.
     * @param area A vector of points forming the area.
     * @return Central point.
     */
    Point getAreaCentralPoint(const std::vector<Point>& area);

    /**
     * @brief Gets the midpoint of a line segment.
     * @param p1 First point.
     * @param p2 Second point.
     * @return Midpoint along the line segment.
     */
    Point getLineCentralPoint(const Point& p1, const Point& p2);

    /**
     * @brief Converts all circular obstacles to square-shaped equivalents.
     */
    void convertCirclesToSquares();

    /**
     * @brief Computes the Euclidean distance between two points.
     * @param a First point.
     * @param b Second point.
     * @return Distance between a and b.
     */
    double pointDistance(const Point& a, const Point& b) const;

    /**
     * @brief Retrieves unique x-coordinates from all obstacles.
     * @return Vector of unique x-values.
     */
    std::vector<double> getObstaclesUniqueAbscissas();

    /**
     * @brief Retrieves unique x-coordinates from both obstacles and borders.
     * @return Vector of unique x-values.
     */
    std::vector<double> getObstaclesAndBordersUniqueAbscissas();

    /**
     * @brief Collects all vertices from all obstacles.
     * @return Vector of points representing obstacle corners.
     */
    std::vector<Point> getObstaclesPoints();

    /**
     * @brief Merges and sorts all obstacle and border points.
     * @return Vector of sorted and unified points.
     */
    std::vector<Point> getSortedObstaclesAndBordersPoints();

    /**
     * @brief Gets all points at a specific x-coordinate.
     * @param x The x-value of interest.
     * @return Vector of points located at x.
     */
    std::vector<Point> getPointsAtGivenAbscissa(double x);

    /**
     * @brief Checks if a point lies inside any obstacle.
     * @param p The point to check.
     * @return True if point is inside an obstacle.
     */
    bool isInsideObstacles(const Point &p);

    /**
     * @brief Checks if an edge (composed of 4 points) belongs to an obstacle.
     * @param p1 First vertex.
     * @param p2 Second vertex.
     * @param p3 Third vertex.
     * @param p4 Fourth vertex.
     * @param extr Extended check flag.
     * @return True if the edge belongs to any obstacle.
     */
    bool edgeBelongsToObstacle(const Point &p1, const Point &p2,
                               const Point &p3, const Point &p4, bool extr);

    /**
     * @brief Performs the cell decomposition process and populates the graph.
     */
    void cellDecomposition();

    /**
     * @brief Gets all intersection points of a vertical line with map borders.
     * @param x X-coordinate of the vertical line.
     * @return Vector of intersection points.
     */
    std::vector<Point> getIntersectionWithBorders(double x);

    /**
     * @brief Gets all intersection points of a vertical line with obstacles.
     * @param x X-coordinate of the vertical line.
     * @return Vector of intersection points.
     */
    std::vector<Point> getIntersectionWithObstacles(double x);

    /**
     * @brief Finds the nearest points by vertical (Y-axis) offset from a reference point.
     * @param pts Candidate points.
     * @param p Reference point.
     * @return Vector of points closest by Y-offset.
     */
    std::vector<Point> nearestByYOffset(const std::vector<Point>& pts,
                                        const Point &p);

    /// @}
};
