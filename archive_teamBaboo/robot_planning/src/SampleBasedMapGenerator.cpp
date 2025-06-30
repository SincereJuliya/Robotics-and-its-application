
// SampleBasedMapGenerator.cpp
#include "../include/robotPlanning/SampleBasedMapGenerator.hpp"

const float minDistObs = 0.55f; // Minimum distance to obstacles
const float minDistBor = 0.6f;  // Minimum distance to obstacles

/**
 * @brief Constructor for the SampleBasedMapGenerator class.
 *
 * This constructor initializes the random number generator with the current
 * time as the seed. This ensures that the generated random numbers are
 * different each time the program is run.
 */
SampleBasedMapGenerator::SampleBasedMapGenerator()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

/**
 * @brief Retrieves the search radius used in the sample-based map generation.
 * 
 * This function returns a constant value representing the radius within which
 * the search is performed during the map generation process.
 * 
 * @return The search radius as a floating-point value.
 */
float SampleBasedMapGenerator::getSearchRadius() const
{
    return 5.0f;
}

void SampleBasedMapGenerator::setGates(const std::vector<Point> &gates)
{
    gates_ = gates;
}

void SampleBasedMapGenerator::setBorders(const std::vector<Point> &borders)
{
    borders_ = borders;
}

void SampleBasedMapGenerator::setObstacles(const std::vector<Obstacle> &obstacles)
{
    obstacles_ = obstacles;
}

/**
 * @brief Generates a random floating-point position within a specified range.
 *
 * This function generates a random position within the range 
 * [middle - r, middle + r] using a uniform distribution.
 *
 * @param middle The central value around which the random position is generated.
 * @param r The range radius. The random position will be within the interval 
 *          [middle - r, middle + r].
 * @return A random floating-point value within the specified range.
 */
float SampleBasedMapGenerator::getRandomPosition(float middle, float r) const
{
    /* return (middle - r) + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * r))); */
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(middle - r, middle + r);
    return dis(gen);
}

/**
 * @brief Generates a random point within a specified radius around an initial point, ensuring it satisfies certain constraints.
 *
 * This function generates a random point within a circular area defined by the given radius around the initial point.
 * The generated point must meet the following conditions:
 * 1. It must be inside the defined border and not too close to the border.
 * 2. It must not be inside or too close to any obstacle in the environment.
 *
 * @param initP The initial point around which the random point is generated.
 * @param radius The radius within which the random point is generated.
 * @return A valid random point that satisfies the constraints.
 */
Point SampleBasedMapGenerator::getRandomPoint(const Point &initP, float radius) const
{
    Point newP;

    while (true)
    {
        float x = getRandomPosition(initP.getX(), radius);
        float y = getRandomPosition(initP.getY(), radius);
        Point newP{x, y};

        // 1. Must be inside border
        if (!isInsideBorder(newP) || isTooCloseToBorder(newP, minDistBor))
            continue;

        // 2. Must not be inside or too close to any obstacle
        bool valid = true;
        for (const Obstacle &obs : obstacles_)
        {
            if (obs.isInsideObstacle(newP) || obs.isTooCloseToObstacle(newP, minDistObs))
            {
                valid = false;
                break;
            }
        }

        if (valid)
        {
            std::cout << "Generated point: (" << newP.getX() << ", " << newP.getY() << ")\n";
            return newP;
        }
    }

    return newP;
}

/**
 * @brief Determines if a given point is inside the polygonal border.
 *
 * This function uses the ray-casting algorithm to check whether a point lies
 * inside a polygon defined by the borders_ member variable. The polygon is
 * assumed to be closed, with the last point connecting back to the first.
 *
 * @param p The point to check.
 * @return True if the point is inside the border, false otherwise.
 */
bool SampleBasedMapGenerator::isInsideBorder(const Point &p) const
{
    int n = static_cast<int>(borders_.size());
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        const Point &pi = borders_[i];
        const Point &pj = borders_[j];
        if ((pi.getY() > p.getY()) != (pj.getY() > p.getY()) &&
            (p.getX() < (pj.getX() - pi.getX()) * (p.getY() - pi.getY()) / (pj.getY() - pi.getY()) + pi.getX()))
        {
            inside = !inside;
        }
    }
    return inside;
}

/**
 * @brief Checks if a given point is too close to the border of a defined area.
 *
 * This function determines whether a point lies within a specified margin
 * from the borders of a polygonal area. The borders are defined as a series
 * of connected points forming a closed loop.
 *
 * @param p The point to check.
 * @param margin The minimum allowable distance from the border.
 * @return true if the point is closer to the border than the specified margin,
 *         or if the borders are not initialized (less than 2 points).
 * @return false otherwise.
 */
bool SampleBasedMapGenerator::isTooCloseToBorder(const Point &p, double margin) const
{
    if (borders_.size() < 2)
        return true; // Not initialized, be conservative

    auto pointToSegmentDistance = [](const Point &p, const Point &a, const Point &b)
    {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();

        if (dx == 0 && dy == 0)
            return std::hypot(p.getX() - a.getX(), p.getY() - a.getY());

        double t = ((p.getX() - a.getX()) * dx + (p.getY() - a.getY()) * dy) / (dx * dx + dy * dy);
        t = std::max(0.0, std::min(1.0, t));

        double projX = a.getX() + t * dx;
        double projY = a.getY() + t * dy;

        return std::hypot(p.getX() - projX, p.getY() - projY);
    };

    for (size_t i = 0; i < borders_.size(); ++i)
    {
        const Point &a = borders_[i];
        const Point &b = borders_[(i + 1) % borders_.size()];
        double dist = pointToSegmentDistance(p, a, b);
        if (dist < margin)
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief Checks if a given point is inside or too close to any obstacle.
 * 
 * This function iterates through all obstacles and determines whether the 
 * specified point is either inside any obstacle or within a minimum distance 
 * from any obstacle.
 * 
 * @param p The point to be checked.
 * @return true If the point is inside or too close to any obstacle.
 * @return false If the point is neither inside nor too close to any obstacle.
 */
bool SampleBasedMapGenerator::isInsideAnyObstacle(const Point &p) const
{
    for (const auto &obs : obstacles_)
    {
        if (obs.isTooCloseToObstacle(p, minDistObs) || obs.isInsideObstacle(p))
            return true;
    }
    return false;
}

/**
 * @brief Checks if a given point has reached any of the gates.
 * 
 * This function iterates through all the gates and determines if the 
 * specified point is within or at a distance of zero from any gate.
 * 
 * @param p The point to check.
 * @return true If the point has reached any gate.
 * @return false If the point has not reached any gate.
 */
bool SampleBasedMapGenerator::isReachedGate(const Point &p) const
{
    for (const auto &gate : gates_)
    {
        if (computeDistance(gate, p) <= 0.0f)
            return true;
    }
    return false;
}

/**
 * @brief Computes the squared Euclidean distance between two points.
 * 
 * This function calculates the squared distance between two points `a` and `b`
 * in a 2D space. The squared distance is used to avoid the computational cost
 * of calculating the square root when the exact distance is not required.
 * 
 * @param a The first point.
 * @param b The second point.
 * @return The squared Euclidean distance between points `a` and `b`.
 */
float SampleBasedMapGenerator::computeDistance(const Point &a, const Point &b) const
{
    float dx = b.getX() - a.getX();
    float dy = b.getY() - a.getY();
    return dx * dx + dy * dy;
}

/**
 * @brief Finds the nearest point in the graph to the given point.
 * 
 * This function iterates through all the vertices in the provided graph
 * and computes the distance between each vertex and the given point. It
 * returns the vertex that is closest to the given point.
 * 
 * @param graph The graph containing the vertices to search.
 * @param p The point to which the nearest vertex is to be found.
 * @return The vertex in the graph that is closest to the given point.
 */
Point SampleBasedMapGenerator::findNearest(const Graph &graph, const Point &p) const
{
    const auto &verts = graph.getVertices();
    Point nearest = verts.front();
    float minD = computeDistance(nearest, p);
    for (const auto &v : verts)
    {
        float d = computeDistance(v, p);
        if (d < minD)
        {
            minD = d;
            nearest = v;
        }
    }
    return nearest;
}

/**
 * @brief Checks if the line segment between two points collides with any obstacle or border.
 *
 * This function interpolates points along the line segment between the given points `p1` and `p2`,
 * and checks if any of these points are too close to an obstacle, inside an obstacle, or otherwise
 * in collision with the environment.
 *
 * @param p1 The starting point of the line segment.
 * @param p2 The ending point of the line segment.
 * @return True if the line segment collides with an obstacle or border, false otherwise.
 *
 * @details
 * - The number of interpolated points is determined by the distance between `p1` and `p2`,
 *   with 3 samples per unit of distance.
 * - For each interpolated point, the function checks:
 *   - If the point is too close to any obstacle (using `isTooCloseToObstacle`).
 *   - If the point is inside any obstacle (using `isInsideObstacle`).
 * - If any of these checks return true, the function immediately returns true.
 * - If no collisions are detected after checking all interpolated points, the function returns false.
 */
bool SampleBasedMapGenerator::collidesWithObstacleOrBorder(const Point &p1, const Point &p2) const
{
    double distance = std::hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY());

    // 3 samples per one unit of distance
    const double samplesPerUnit = 3.0;
    int numSamples = std::max(1, static_cast<int>(distance * samplesPerUnit));

    for (int i = 0; i <= numSamples; ++i)
    {
        double t = static_cast<double>(i) / numSamples;
        Point interp(
            p1.getX() + t * (p2.getX() - p1.getX()),
            p1.getY() + t * (p2.getY() - p1.getY()));

        for (const auto &obs : obstacles_)
        {
            if (obs.isTooCloseToObstacle(interp, minDistObs) || obs.isInsideObstacle(interp))
            {
                return true;
            }
        }
    }

    return false; // No collision detected
}

/**
 * @brief Connects a new point to all visible vertices in the graph.
 *
 * This function iterates through all vertices in the graph and attempts to 
 * connect the given point to each vertex. A connection (edge) is added only 
 * if the following conditions are met:
 * - The vertex is not the same as the new point.
 * - An edge between the new point and the vertex does not already exist.
 * - The line segment between the new point and the vertex does not collide 
 *   with any obstacles or borders.
 *
 * @param graph The graph to which the new point and edges belong.
 * @param newP The new point to be connected to visible vertices in the graph.
 */
void SampleBasedMapGenerator::connectVisibleEdges(Graph &graph, const Point &newP) const
{
    const auto &verts = graph.getVertices();
    for (const auto &v : verts)
    {
        if (v == newP)
            continue;

        if (!graph.edgeExists(newP, v) && !collidesWithObstacleOrBorder(newP, v))
        {
            graph.addEdge(newP, v);
        }
    }
}

// attempt to generate a graph based on random sampling
/**
 * @brief Generates a graph using a sample-based map generation algorithm.
 * 
 * This function creates a graph by sampling random points within a defined 
 * search radius and connecting them with edges based on visibility and proximity 
 * constraints. The graph is initialized with a starting point and iteratively 
 * expanded by adding new vertices and edges.
 * 
 * @param init The initial point from which the graph generation starts.
 * @return Graph The generated graph containing vertices and edges.
 * 
 * @note The function requires that gates, borders, and obstacles are properly 
 *       initialized before calling. If any of these are missing, the function 
 *       will return an empty graph and log an error message.
 * 
 * @details
 * - The function clears any existing graph data before starting.
 * - A maximum of 90 iterations is performed to sample new points.
 * - Points are validated to ensure they are not inside obstacles, too close 
 *   to borders, or duplicates of existing vertices.
 * - New points are connected to the graph by adding edges to visible and 
 *   valid neighbors.
 * - The initial point is updated to the last added point after each iteration.
 */
Graph SampleBasedMapGenerator::generateGraph(const Point &init)
{
    G_.clear();

    if (gates_.empty() || borders_.empty() || obstacles_.empty())
    {
        std::cerr << "SampleBasedMapGenerator: missing data for graph generation.\n";
        return G_;
    }

    const int maxIterations = 90;

    // init!
    Point initP = {init.getX(), init.getY()};
    int count = 0;

    while (count < maxIterations)
    {
        Point newP = getRandomPoint(initP, getSearchRadius());
        if (isInsideAnyObstacle(newP) || isTooCloseToBorder(newP, minDistBor))
            continue;

        // Check for duplicates
        if (G_.containsVertex(newP))
            continue;

        ++count;

        if (G_.getVertices().empty())
        {
            G_.addEdge(initP, newP);
        }
        else
        {
            connectVisibleEdges(G_, newP);

            /*
            Point nearest = findNearest(G_, newP);
             if (!G_.edgeExists(nearest, newP)) {
                G_.addEdge(nearest, newP);
            } */
        }

        initP = newP; // Update initP to the last added point
        
    }

    return G_;
}
