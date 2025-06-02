#ifndef ACEAFEF4_8AAA_41A0_8846_5AAD774D272F
#define ACEAFEF4_8AAA_41A0_8846_5AAD774D272F
#pragma once

#include <ostream>
#include <string> 
#include <vector>

enum ObstacleDefault_TYPE {
  CYLINDER,
  BOX
};

struct ObstacleDefault {
  double radius;
  double x, y;
  double dx, dy;
  double yaw;
  ObstacleDefault_TYPE type;
  std::string xml_file = "";
 
  friend std::ostream& operator<<(std::ostream& os, const ObstacleDefault& obs){
    os << "ObstacleDefault: ";
    if (obs.type == ObstacleDefault_TYPE::CYLINDER){
      os << "Cylinder: ";
      os << "x: " << obs.x << " y: " << obs.y << " radius: " << obs.radius;
    }
    else if (obs.type == ObstacleDefault_TYPE::BOX){
      os << "Box: ";
      os << "x: " << obs.x << " y: " << obs.y << " dx: " << obs.dx << " dy: " << obs.dy << " yaw: " << obs.yaw;
    }
    else{
      os << "Unknown type";
    }
    return os;
  }
};
// Juliya : i took from the default code bc my env didnt allow to include the original header file
struct Victim : public ObstacleDefault {
  double weight;  // add this
  Victim(double _x, double _y, double _r = 0.5) : ObstacleDefault() {
    this->x = _x;
    this->y = _y;
    this->radius = _r;
    this->yaw = 0.0;
    this->type = ObstacleDefault_TYPE::CYLINDER;
  }
};


#endif /* ACEAFEF4_8AAA_41A0_8846_5AAD774D272F */
