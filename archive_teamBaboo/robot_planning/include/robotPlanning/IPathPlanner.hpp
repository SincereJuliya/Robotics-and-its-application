#ifndef IPATHPLANNER_HPP
#define IPATHPLANNER_HPP
#include "graph.hpp"
#include "point.hpp"
#include <vector>

#include "victim.hpp"

class IPathPlanner {
public:
    virtual ~IPathPlanner() = default;

    // Main interface method to compute a path
    virtual std::vector<Point> findPath(const std::vector<Victim>& victims) = 0;
};
#endif  // IPATHPLANNER_HPP