#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h> 
#include <cassert>
#include "../include/robotPlanning/multiPointMarkovDubins.hpp"

int ksigns [6][3] = {
  { 1, 0, 1},  //LSL
  {-1, 0, -1}, //RSR
  { 1, 0, -1}, //LSR
  {-1, 0, 1},  // RSL
  {-1, 1, -1}, //RLR
  {1, -1,  1}  // LRL
};

double Kmax = 1.0;

std::string curvNames[6] = {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"};

toStandard scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax){
  toStandard ret;

  double dx = xf - x0;
  double dy = yf - y0;
  double phi = atan2(dy, dx);

  ret.lamda = hypot(dx, dy)/2;
  ret.sc_th0 = mod2pi(th0 - phi);
  ret.sc_thf = mod2pi(thf - phi);
  ret.sc_Kmax = Kmax * ret.lamda;

  return ret;
}

fromStandard scaleFromStandard(double sc_s1, double sc_s2, double sc_s3, double lamda){
  fromStandard ret;

  ret.s1 = sc_s1 * lamda;
  ret.s2 = sc_s2 * lamda;
  ret.s3 = sc_s3 * lamda;
  return ret;
}

double sinc(double t){
  double s;
  if(abs(t) < 0.002){
    s = 1-pow(t, 2)*(1/6-pow(t, 2)/120);
  }else{
    s = sin(t)/t;
  }
  return s;
};

double mod2pi(double angle){
  double ret = angle;
  while(ret < 0){
    ret = ret +2*M_PI;
  }
  while(ret >= 2*M_PI){
    ret = ret -2*M_PI;
  }
  return ret;
};

double rangeSymm(double angle){
  double ret = angle;
  while(ret <= -M_PI){
    ret = ret +2*M_PI;
  }
  while(ret > M_PI){
    ret = ret -2*M_PI;
  }
  return ret;
};

arcVar circLine(double L, double x0, double y0, double th0, double k){
  arcVar arc;
  arc.x = x0 + L * sinc(k*L/2.0)*(cos(th0+k*L/2));
  arc.y = y0 + L * sinc(k*L/2.0)*(sin(th0+k*L/2));
  arc.th = mod2pi(th0+k*L);

  return arc;
};

bool check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf){
  double x0 = -1;
  double y0 = 0;
  double xf = 11;
  double yf = 0;

  double eq1 = x0 + s1 * sinc((1/2.) * k0 * s1) * cos(th0 + (1/2.) * k0 * s1) + s2 * sinc((1/2.) * k1 * s2) * cos(th0 + k0 * s1 + (1/2.) * k1 * s2) + s3 * sinc((1/2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - xf;
  double eq2 = y0 + s1 * sinc((1/2.) * k0 * s1) * sin(th0 + (1/2.) * k0 * s1) + s2 * sinc((1/2.) * k1 * s2) * sin(th0 + k0 * s1 + (1/2.) * k1 * s2) + s3 * sinc((1/2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - yf;
  double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

  bool Lpos = ((s1>0) || (s2>0 ) || (s3>0));
  bool ret =  ((sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos);
  return ret;
};

dubinsCurve dubinsArc(double x0, double y0, double th0, double k, double L){
  dubinsCurve c;
  c.i.x = x0;
  c.i.y = y0;
  c.i.th = th0;
  c.k = k;
  c.L = L;
  c.f = circLine(L, x0, y0, th0, k);
  return c;
};

tripleDubinsCurve dubinsCur(double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2){
  tripleDubinsCurve d;

  d.a1 = dubinsArc(x0, y0, th0, k0, s1);
  d.a2 = dubinsArc(d.a1.f.x, d.a1.f.y, d.a1.f.th, k1, s2);
  d.a3 = dubinsArc(d.a2.f.x, d.a2.f.y, d.a2.f.th, k2, s3);
  d.L = d.a1.L + d.a2.L + d.a3.L;

  return d;
};

solution LSL(double th0, double thf, double Kmax){
  solution ret;
  
  double  invK = 1 / Kmax;
  double C = cos(thf) - cos(th0);
  double S = 2 * Kmax + sin(th0) - sin(thf);
  double temp1 = atan2(C, S);
  ret.s1 = invK * mod2pi(temp1 - th0);
  double temp2 = 2 + 4 * pow(Kmax,2) - 2 * cos(th0 - thf) + 4 * Kmax * (sin(th0) - sin(thf));
  if (temp2 < 0){
    ret.ok = false; 
    ret.s1 = 0; 
    ret.s2 = 0; 
    ret.s3 = 0;
    return ret;
  }
  ret.s2 = invK * sqrt(temp2);
  ret.s3 = invK * mod2pi(thf - temp1);
  ret.ok = true;

  return ret;
}

solution RSR(double th0, double thf, double Kmax){
  solution ret;
  
  double  invK = 1 / Kmax;
  double C = cos(th0) - cos(thf);
  double S = 2 * Kmax - sin(th0) + sin(thf);
  double temp1 = atan2(C, S);
  ret.s1 = invK * mod2pi(th0 - temp1);
  double temp2 = 2 + 4 * pow(Kmax,2) - 2 * cos(th0 - thf) - 4 * Kmax * (sin(th0) - sin(thf));
  if (temp2 < 0){
    ret.ok = false; 
    ret.s1 = 0; 
    ret.s2 = 0; 
    ret.s3 = 0;
    return ret;
  }
  ret.s2 = invK * sqrt(temp2);
  ret.s3 = invK * mod2pi(temp1 - thf);
  ret.ok = true;

  return ret;
}

solution LSR(double th0, double thf, double Kmax){
  solution ret;
  
  double   invK = 1 / Kmax;
  double C = cos(th0) + cos(thf);
  double S = 2 * Kmax + sin(th0) + sin(thf);
  double temp1 = atan2(-C, S);
  double temp3 = 4 * pow(Kmax,2) - 2 + 2 * cos(th0 - thf) + 4 * Kmax * (sin(th0) + sin(thf));
  if (temp3 < 0){
    ret.ok = false; 
    ret.s1 = 0; 
    ret.s2 = 0; 
    ret.s3 = 0;
    return ret;
  }
  ret.s2 = invK * sqrt(temp3);
  double temp2 = -atan2(-2, ret.s2 * Kmax);
  ret.s1 = invK * mod2pi(temp1 + temp2 - th0);
  ret.s3 = invK * mod2pi(temp1 + temp2 - thf);
  ret.ok = true;
  
  return ret;
}

solution RSL(double th0, double thf, double Kmax){
  solution ret;

  double  invK = 1 / Kmax;
  double C = cos(th0) + cos(thf);
  double S = 2 * Kmax - sin(th0) - sin(thf);
  double temp1 = atan2(C, S);
  double temp3 = 4 * pow(Kmax,2) - 2 + 2 * cos(th0 - thf) - 4 * Kmax * (sin(th0) + sin(thf));
  if (temp3 < 0){
    ret.ok = false; 
    ret.s1 = 0; 
    ret.s2 = 0; 
    ret.s3 = 0;
    return ret;
  }
  ret.s2 = invK * sqrt(temp3);
  double temp2 = atan2(2, ret.s2 * Kmax);
  ret.s1 = invK * mod2pi(th0 - temp1 + temp2);
  ret.s3 = invK * mod2pi(thf - temp1 + temp2);
  ret.ok = true;

  return ret;
}

solution RLR(double th0, double thf, double Kmax){
  solution ret;

  double invK = 1 / Kmax;
  double C = cos(th0) - cos(thf);
  double S = 2 * Kmax - sin(th0) + sin(thf);
  double temp1 = atan2(C, S);
  double temp2 = 0.125 * (6 - 4 * pow(Kmax,2) + 2 * cos(th0 - thf) + 4 * Kmax * (sin(th0) - sin(thf)));
  if (abs(temp2) > 1){
    ret.ok = false; 
    ret.s1 = 0; 
    ret.s2 = 0; 
    ret.s3 = 0;
    return ret;
  }
  ret.s2 = invK * mod2pi(2 * M_PI - acos(temp2));
  ret.s1 = invK * mod2pi(th0 - temp1 + 0.5 * ret.s2 * Kmax);
  ret.s3 = invK * mod2pi(th0 - thf + Kmax * (ret.s2 - ret.s1));
  ret.ok = true;
  return ret;
}

solution LRL(double th0, double thf, double Kmax){
  solution ret;

  double invK = 1 / Kmax;
  double C = cos(thf) - cos(th0);
  double S = 2 * Kmax + sin(th0) - sin(thf);
  double temp1 = atan2(C, S);
  double temp2 = 0.125 * (6 - 4 * pow(Kmax,2) + 2 * cos(th0 - thf) - 4 * Kmax * (sin(th0) - sin(thf)));
  if (abs(temp2) > 1){
    ret.ok = false;
    ret.s1 = 0; 
    ret.s2 = 0; 
    ret.s3 = 0;
    return ret;
  }
  ret.s2 = invK * mod2pi(2 * M_PI - acos(temp2));
  ret.s1 = invK * mod2pi(temp1 - th0 + 0.5 * ret.s2 * Kmax);
  ret.s3 = invK * mod2pi(thf - th0 + Kmax * (ret.s2 - ret.s1));
  ret.ok = true;

  return ret;
}

curve dubinsShortestPath(double x0, double y0, double th0, double xf, double yf, double thf){
  toStandard var = scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax);
  fromStandard ret;
  double s1, s2, s3;
  curve res;
  tripleDubinsCurve dubinsRes;

  solution (* primitivesPtr[]) (double th0, double thf, double Kmax) = {LSL, RSR, LSR, RSL, RLR, LRL};
  int pidx = -1;
  solution sol;
  double L=0;
  double Lcur=0;

  sol = (*primitivesPtr[0])(var.sc_th0, var.sc_thf, var.sc_Kmax);
  L = sol.s1+sol.s2+sol.s3+1; //added 1 to make it longer than the real first value, to take it into consieration with the loop
  for(int i=0;i<6;i++){
    sol = (*primitivesPtr[i])(var.sc_th0, var.sc_thf, var.sc_Kmax);
    Lcur = sol.s1+sol.s2+sol.s3;
    if(sol.ok && Lcur < L){
      L = Lcur;
      s1 = sol.s1;
      s2 = sol.s2;
      s3 = sol.s3;
      pidx = i;
    }
    std::cout << curvNames[i] << " L = " << Lcur*var.lamda << ", " << "s1 = " << sol.s1*var.lamda << " s2 = " << sol.s2*var.lamda << " s3 = " << sol.s3*var.lamda<< "\n";
  }
  if(pidx>=0){
    ret = scaleFromStandard(s1, s2, s3, var.lamda);
    dubinsRes = dubinsCur(x0, y0, th0, ret.s1, ret.s2, ret.s3, ksigns[pidx][0]*Kmax, ksigns[pidx][1]*Kmax, ksigns[pidx][2]*Kmax);
  }

  res.curvType = pidx;
  res.values = dubinsRes;
  return res;
}

void plotArc(dubinsCurve arc, std::vector<arcVar>& plt){
  int nPts=arc.L/0.2;
  std::cout << "nPts: " << nPts << "\n";
  //int nPts=30; //to be changed to given distance and not fixed one
  double s;
  
  for(int i=0; i<nPts; i++){
      s = arc.L/nPts*i;
      plt.push_back(circLine(s, arc.i.x, arc.i.y, arc.i.th, arc.k));
  }
}

std::vector<arcVar> plotDubins(tripleDubinsCurve dubCurv){
  std::vector<arcVar> plot;
  
  plotArc(dubCurv.a1, plot);
  plotArc(dubCurv.a2, plot);
  plotArc(dubCurv.a3, plot);

  return plot;
}

std::vector<double> optimizeAngles(std::vector<Point> points, double th0, double thf, int k, int m){
  double h = 2*M_PI/k;  
  std::vector<double> ths;
  ths.push_back(th0);

  std::cout << "number of points: " << points.size() << "\n";
  for(std::vector<Point>::size_type i=0; i<points.size()-1; i++){ //for each point
    double minTh, minL, minThi;
    double thi=0;
    double lenght;

    thi = ths[i];
    minL = dubinsShortestPath(points[i].getX(),points[i].getY(),thi,points[i+1].getX(),points[i+1].getY(),0).values.L+1;
    if(i!=(points.size()-1)){
      std::cout << "starting first selection\n";
      for (int a=0; a<k;a++){ //find the angle with the shortest path among k slides
        double th = a*h;   
        lenght = dubinsShortestPath(points[i].getX(),points[i].getY(),thi,points[i+1].getX(),points[i+1].getY(),th).values.L;
        if(lenght<minL && lenght>0){
          minL = lenght;
          minTh = th;
        }
      }
      //refinement interations
      std::cout << "starting refinement step\n";
      for(int r=0; r<m; r++){
        h = 2*M_PI*((pow(3, r)/(pow(2*k, r))));
        minL = dubinsShortestPath(points[i].getX(),points[i].getY(),thi,points[i+1].getX(),points[i+1].getY(),minTh-(3/2)*h).values.L+1; 
        for(double j=minTh-(3/2)*h; j<=minTh+(3/2)*h; j+=h){ //refinement step (m-times)
          lenght = dubinsShortestPath(points[i].getX(),points[i].getY(),thi,points[i+1].getX(),points[i+1].getY(),j).values.L;
          if(lenght<minL && lenght>0){
            minL = lenght;
            minThi = j;
          }
        }
      } 
      ths.push_back(minThi);
    }else{
      lenght = dubinsShortestPath(points[i].getX(),points[i].getY(),thi,points[i+1].getX(),points[i+1].getY(),thf).values.L;
      ths.push_back(thf);
    }
  }
  return ths;
}

//need to check if the path is colliding with any object with at +/- distance (2/3 width of the robot)
std::vector<arcVar> multiPointMarvkovDubinsPlan(std::vector<Point> points,double th0, double thf){
  std::vector<arcVar> plan;
  std::vector<arcVar> planTmp;
  std::vector<double> optAngles;
  curve dubinsPortion;

  optAngles = optimizeAngles(points, th0, thf, 4, 4);
  for(std::vector<Point>::size_type  i=0; i<points.size()-2; i++){
    dubinsPortion = dubinsShortestPath(points[i].getX(), points[i].getY(), optAngles[i], points[i+1].getX(), points[i+1].getY(), optAngles[i+1]);
    planTmp = plotDubins(dubinsPortion.values);
    for(long unsigned int j =0; j<planTmp.size(); j++){
      plan.push_back(planTmp[j]);
    }
  }

  return plan;
}
/*   
int main(){
  std::vector<Point> pts = { Point{67.91, 124.28}, Point{872.23, 452.31}, Point{290.71, 117.93}};
  double th0 = 3.33;
  double thf = 6.12;
  std::vector<double> optAngles = optimizeAngles(pts, th0, thf, 2, 2);

  std::cout << "number of angles: " << optAngles.size() << "\n";
  std::cout << "number of points: " << pts.size() << "\n";

  std::vector<arcVar> plan = multiPointMarvkovDubinsPlan(pts, th0, thf);

  for (std::vector<arcVar>::size_type z=0; z < plan.size(); z++){
      std::cout << "pt: " << plan[z].x << ", " << plan[z].y << ", " << plan[z].th << "\n";  ;
  }
  
 
  return 0;
}*/