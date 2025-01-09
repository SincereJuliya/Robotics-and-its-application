#include "stdio.h"
#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <IMapGenerator.cpp>

struct Point
{
    float x;
    float y;
};

struct Graph
{
    std::map<Point, std::vector<Point>> adjVector;
}; 

class SampleBasedMapGenerator: public IMapGenerator
{
    public:
        Graph G;

        /* to generate the map with sample based approach */
        virtual Graph toGenerateMap(int interactions) override
        {
            int count = 0;

            Point initP{0,0};

            while (count < interactions)
            {
                Point newP = getRandomPoint(initP, getSearchRadius());

                if (isItInObstacle(newP))
                {
                    continue;
                }
                count ++;

                Point nearestP = toFindNearest(G, newP);
                G.adjVector.at(nearestP).push_back(newP);

                if (isReachedGate(newP))
                {
                    return G;
                }
            }
            return G; 
        };

        /* TO DO to get the radius for searching */
        float getSearchRadius()
        {
            // check the nearest obstacle -> take the minDist()
            float radius = 0;
            return radius;
        }; 

        /* to get the new random point with checking */
        Point& getRandomPoint(Point initP, float radius)
        {
            bool flag = true;
            float x = initP.x; 
            float y = initP.y;

            while(flag) 
            {
                x = getRandomPosition(initP.x, radius);
                y = getRandomPosition(initP.y, radius);

                flag = isInsideArea((x - initP.x), (y - initP.y), radius) ? false : true;
            }

            return *(new Point{x,y});
        }; 

        /* to check if the new point is inside of the area (circle) */
        bool isInsideArea(float x, float y, float r)
        {
            return ( toComputeDistance(0, 0, x, y) <= r*r ) ? true : false;
        }

        /* to generate rand value */
        float getRandomPosition(float middle, float r)
        {
            float random = (middle - r) + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2*r)));
            return random;
        }; 

        /* to get the nearest point to connect the new point to the graph */
        Point toFindNearest(Graph g, Point p)
        {
            Point nearest;

            auto it = g.adjVector.begin();
            std::advance(it, 0);

            float minD = toComputeDistance((it->first).x, (it->first).y, p.x, p.y);
            ++it;

            while( &(it->first) != NULL )
            { 
                float currentD = sqrt(toComputeDistance((it->first).x, (it->first).y, p.x, p.y));

                if (currentD < minD)
                {
                    minD = currentD;
                    nearest = it->first;
                }

                ++it;
            }

            return nearest;
        };
        
        /* to compute the distance between two points */
        float toComputeDistance(float x0, float y0, float x, float y)
        {
            return ((x - x0)*(x - x0) + (y - y0)*(y - y0));
        };

        /* TO DO to get the info if we reached the gates */
        bool isReachedGate(Point p)
        {
            return true;
        }; 
}