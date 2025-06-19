#ifndef __MULTIPOINTMARKOVDUBINS_H_
#define __MULTIPOINTMARKOVDUBINS_H_

#include <vector>
#include "point.hpp"
#include "obstacles.hpp"

struct arcVar{
  double x;
  double y;
  double th;
};

struct dubinsCurve{ 
  arcVar i;
  double k;
  double L;
  arcVar f;
};

struct tripleDubinsCurve{
  dubinsCurve a1;
  dubinsCurve a2;
  dubinsCurve a3;
  double L;
};

struct toStandard{
  double sc_th0;
  double sc_thf;
  double sc_Kmax;
  double lamda;
};

struct fromStandard{
  double s1;
  double s2;
  double s3;
};

struct solution{
  bool ok;
  double s1;
  double s2;
  double s3;
};

struct curve{
  int curvType; // index token with this order --> LSL, RSR, LSR, RSL, RLR, LRL 
  tripleDubinsCurve values;
};

dubinsCurve dubinsArc(double x0, double y0, double th0, double Kmax, double L);

tripleDubinsCurve dubinsCur(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2);

toStandard scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax);

fromStandard scaleFromStandard(double sc_s1, double sc_s2, double sc_s3, double lamda);

arcVar circLine(double L, double x0, double y0, double th0, double k);

double mod2pi(double angle);

double rangeSymm(double angle);

double sinc(double t);

curve dubinsShortestPath(double x0, double y0, double th0, double xf, double yf, double thf/* , double kmax */);

bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf);

solution LSL(double th0, double thf, double Kmax);

solution RSR(double th0, double thf, double Kmax);

solution LSR(double th0, double thf, double Kmax);

solution RSL(double th0, double thf, double Kmax);

solution RLR(double th0, double thf, double Kmax);

solution LRL(double th0, double thf, double Kmax);

void plotArc(dubinsCurve arc, std::vector<arcVar>& plt);

std::vector<arcVar> plotDubins(tripleDubinsCurve dubCurv);

bool isPathCollisionFree(const curve& dubinsCurve, const std::vector<Obstacle>& obstacles);

std::vector<double> optimizeAngles(std::vector<Point> points, double th0, double thf, int k, int m, const std::vector<Obstacle>& obstacles);

std::vector<arcVar> multiPointMarvkovDubinsPlan(std::vector<Point> points,double th0, double thf, std::vector<Obstacle> obstacles);

#endif