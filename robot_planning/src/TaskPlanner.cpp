#include <iostream>
#include <vector>
#include <string>
#include <variables.hpp>
#include <stdio.h>
#include <unicode/unistr.h>

class TaskPlanner
{
        
    private:
        std::vector<Point> openList;
        std::vector<Point> closedList;

    public:

        std::vector<Point> generateSuccessors(const Point& q)
        {
            std::vector<Point> successors;
            successors.push_back(Point{q.x + 1, q.y});
            successors.push_back(Point{q.x - 1, q.y});
            successors.push_back(Point{q.x, q.y + 1});
            successors.push_back(Point{q.x, q.y - 1});
            return successors;
        }

        void run()
        {
            Point start{0,0};
            Point goal{5,5};

            openList.push_back(start);
            auto it = openList.begin();

            while(!openList.empty())
            {
                auto q = it++;
                openList.erase(q); // remove q from openList

                std::vector<Point> successors = generateSuccessors(*q);

                for (const auto &successor : successors)
                {
                    if (successor == goal)
                    {
                        std::cout << "Goal reached\n";
                        return;
                    }

                    float g = toComputeDistance(*q, successor);
                    float h = toComputeDistance(successor, goal);
                    float f = g + h;

                    if (isInOpenList(successor) && f < successor.f)
                    {
                        continue;
                    }

                    if (isInClosedList(successor) && f < successor.f)
                    {
                        continue;
                    }

                    openList.push_back(successor);
                }

                closedList.push_back(*q);
            }

        };

        float toComputeDistance(Point a, Point b)
        {
            return ((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
        };

        bool isInClosedList(const Point& point)
        {
            return std::find(closedList.begin(), closedList.end(), point) != closedList.end();
        }
}
/*
// A* Search Algorithm
1.  Initialize the open list
2.  Initialize the closed list
    put the starting node on the open 
    list (you can leave its f at zero)
3.  while the open list is not empty
    a) find the node with the least f on 
       the open list, call it "q"
    b) pop q off the open list
  
    c) generate q's 8 successors and set their 
       parents to q
   
    d) for each successor
        i) if successor is the goal, stop search
        
        ii) else, compute both g and h for successor
          successor.g = q.g + distance between 
                              successor and q
          successor.h = distance from goal to 
          successor (This can be done using many 
          ways, we will discuss three heuristics- 
          Manhattan, Diagonal and Euclidean 
          Heuristics)
          
          successor.f = successor.g + successor.h
        iii) if a node with the same position as 
            successor is in the OPEN list which has a 
           lower f than successor, skip this successor
        iV) if a node with the same position as 
            successor  is in the CLOSED list which has
            a lower f than successor, skip this successor
            otherwise, add  the node to the open list
     end (for loop)
  
    e) push q on the closed list
    end (while loop)
*/