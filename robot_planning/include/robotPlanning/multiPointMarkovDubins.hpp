#ifndef __MULTIPOINTMARKOVDUBINS_H_
#define __MULTIPOINTMARKOVDUBINS_H_

#include <vector>
#include "point.hpp"
#include "obstacles.hpp"
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

#include <cassert>

#define VELOCITY 0.26

/// @brief Represents a state (x, y, heading)
struct arcVar {
  double x;   ///< x-coordinate
  double y;   ///< y-coordinate
  double th;  ///< orientation (theta)
};

/// @brief Represents a single Dubins arc (constant curvature segment)
struct dubinsCurve { 
  arcVar i;   ///< initial state
  double k;   ///< curvature
  double L;   ///< arc length
  arcVar f;   ///< final state
};

/// @brief Represents a full Dubins path composed of three arcs
struct tripleDubinsCurve {
  dubinsCurve a1; ///< first arc
  dubinsCurve a2; ///< second arc
  dubinsCurve a3; ///< third arc
  double L;       ///< total length (optional, can be recomputed)

  /// @brief Calculates total length of the path
  double totalLength() const {
    return a1.L + a2.L + a3.L;
  }

  /// @brief Integrates motion with constant curvature k over arc length s
  /// @return Final point after integration
  static Point integrate(double x0, double y0, double th0, double s, double k) {
      if (std::abs(k) < 1e-6) {
          // Straight line motion
          return Point(x0 + s * cos(th0), y0 + s * sin(th0));
      } else {
          // Circular arc motion
          double R = 1.0 / k;
          double dtheta = k * s;
          double cx = x0 - R * sin(th0);
          double cy = y0 + R * cos(th0);
          double x = cx + R * sin(th0 + dtheta);
          double y = cy - R * cos(th0 + dtheta);
          return Point(x, y);
      }
  }

  /// @brief Evaluates a point on the Dubins path at arc-length s
  Point evaluate(double s) const {
      if (s <= a1.L) {
          return integrate(a1.i.x, a1.i.y, a1.i.th, s, a1.k);
      } else if (s <= a1.L + a2.L) {
          return integrate(a2.i.x, a2.i.y, a2.i.th, s - a1.L, a2.k);
      } else if (s <= a1.L + a2.L + a3.L) {
          return integrate(a3.i.x, a3.i.y, a3.i.th, s - a1.L - a2.L, a3.k);
      } else {
          return Point(a3.f.x, a3.f.y); 
      }
  }
};

/// @brief Struct used to store parameters after scaling to a canonical Dubins problem
struct toStandard {
  double sc_th0;  ///< scaled initial heading
  double sc_thf;  ///< scaled final heading
  double sc_Kmax; ///< scaled max curvature
  double lamda;   ///< scaling factor
};

/// @brief Struct used to hold arc lengths after scaling back to original units
struct fromStandard {
  double s1, s2, s3; ///< arc lengths of the three segments
};

/// @brief Holds solution status and arc lengths
struct solution {
  bool ok;         ///< was a valid solution found?
  double s1, s2, s3; ///< arc lengths
};

/// @brief Wraps a computed Dubins curve and its type
struct curve {
  int curvType;             ///< LSL=0, RSR=1, LSR=2, RSL=3, RLR=4, LRL=5
  tripleDubinsCurve values; ///< corresponding path
};

// ----------------- Dubins Path Construction ----------------- //

/// @brief Constructs a Dubins arc segment from initial state, curvature and length
dubinsCurve dubinsArc(double x0, double y0, double th0, double Kmax, double L);

/// @brief Constructs a 3-part Dubins curve with specified segment lengths and curvatures
tripleDubinsCurve dubinsCur(double x0, double y0, double th0,
                            double s1, double s2, double s3,
                            double k0, double k1, double k2);

/// @brief Scales problem to canonical Dubins configuration (starting at origin)
toStandard scaleToStandard(double x0, double y0, double th0,
                           double xf, double yf, double thf,
                           double Kmax);

/// @brief Scales arc lengths back from standard to original configuration
fromStandard scaleFromStandard(double sc_s1, double sc_s2, double sc_s3, double lamda);

/// @brief Computes the final pose after traveling an arc of length L with curvature k
arcVar circLine(double L, double x0, double y0, double th0, double k);

// ----------------- Math Utilities ----------------- //

double mod2pi(double angle);     ///< Normalize angle to [0, 2pi)
double rangeSymm(double angle);  ///< Normalize angle to [-pi, pi)
double sinc(double t);           ///< Sinc function sin(t)/t

// ----------------- Geometry & Collision Checks ----------------- //
/// @brief Returns true if point is within `margin` of any border point
bool isTooCloseToBorder(const Point& p, double margin, const std::vector<Point>& mBorders);

/// @brief Computes shortest Dubins path avoiding obstacles and staying within borders
curve dubinsShortestPath(double x0, double y0, double th0,
                         double xf, double yf, double thf,
                         const std::vector<Obstacle>& obstacles,
                         double sampleStep, 
                         const std::vector<Point>& borders);

/// @brief Verifies feasibility of a Dubins path configuration
bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf);

// ----------------- Dubins Primitive Solvers ----------------- //

solution LSL(double th0, double thf, double Kmax);
solution RSR(double th0, double thf, double Kmax);
solution LSR(double th0, double thf, double Kmax);
solution RSL(double th0, double thf, double Kmax);
solution RLR(double th0, double thf, double Kmax);
solution LRL(double th0, double thf, double Kmax);

// ----------------- Visualization and Path Point extraction ----------------- //

void plotArc(dubinsCurve arc, std::vector<arcVar>& plt); ///< Append points along arc
std::vector<arcVar> plotDubins(tripleDubinsCurve dubCurv); ///< Get all points along full path

// ----------------- Debugging / Diagnostics ----------------- //

std::vector<std::pair<Point, Point>> getFailedSegments(); ///< // Segments Rejected due to Collisions
std::vector<std::pair<Point, Point>> getSlowSegments();   ///< Segments with low efficiency

// ----------------- Path Planning Interfaces ----------------- //

/// @brief Checks if a given Dubins path is free of obstacle collisions
bool isPathCollisionFree(const curve& dubinsCurve, const std::vector<Obstacle>& obstacles);

/// @brief Angle optimization for multipoint planning (Markov-based heuristics)
std::vector<double> optimizeAngles(std::vector<Point> points, double th0, double thf,
                                   int k, int m,
                                   const std::vector<Obstacle>& obstacles,
                                   const std::vector<Point>& borders);

/// @brief Main multipoint planner entry point
std::vector<arcVar> multiPointMarvkovDubinsPlan(std::vector<Point> points,
                                                double th0, double thf,
                                                int k, int m,
                                                std::vector<Obstacle>& obstacles,
                                                std::vector<Point>& borders,
                                                int maxTime);

#endif
