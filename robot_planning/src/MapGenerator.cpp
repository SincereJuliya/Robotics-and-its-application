#include "stdio.h"
#include <vector>
#include <map>
#include <iostream>
#include <iostream>

struct Point
{
    int x;
    int y;
};

struct Graph
{
    std::map<Point, std::vector<Point>> adjVector;
};
/*
    1. the communication between nodes: services, messages...
    2. how to organize map generator with 2 diff aproaches (cell + sample)
    3. discuss the algorithm (the paper)

*/
class MapGenerator
{
    public:
        Graph G;

    private:
        Graph toGenerateMap(int interactions)
        {
            int count = 0;

            Point init{0, 0};

            while (count < interactions)
            {
                count ++;

                Point newP{getRandPosition(-10, 10), getRandPosition(-10, 10)};

                if (isItInObstacle(newP))
                {
                    continue;
                }

                Point nearestP = toFindNearest(G, newP);
                G.adjVector.at(nearestP).push_back(newP);

                if (isReachedGate(newP))
                {
                    return G;
                }
            }
            return G;
        };

        float getRandPosition(float min, float max)
        {
            float random = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
            return random;
        };

        // think if service or message from the Obstacle node
        bool isItInObstacle(Point p)
        {
            return false;
        };

        Point toFindNearest(Graph g, Point p)
        {
            return (Point{0, 0});
        };

        bool isReachedGate(Point p)
        {
            return true;
        };
}