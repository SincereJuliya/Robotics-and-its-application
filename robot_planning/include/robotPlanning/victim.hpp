#ifndef ACEAFEF4_8AAA_41A0_8846_5AAD774D272F
#define ACEAFEF4_8AAA_41A0_8846_5AAD774D272F
#pragma once

#include <ostream>
#include <string> 
#include <vector>

enum VictimDefault_TYPE {
  VICTIM_CYLINDER,
  VICTIM_BOX
};

struct VictimDefault {
  double radius;
  double x, y;
  double dx, dy;
  double yaw;
  VictimDefault_TYPE type;
  std::string xml_file = "";
 
  friend std::ostream& operator<<(std::ostream& os, const VictimDefault& obs){
    os << "VictimDefault: ";
    if (obs.type == VictimDefault_TYPE::VICTIM_CYLINDER){
      os << "Cylinder: ";
      os << "x: " << obs.x << " y: " << obs.y << " radius: " << obs.radius;
    }
    else if (obs.type == VictimDefault_TYPE::VICTIM_BOX){
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
struct Victim : public VictimDefault {
  Victim(double _x, double _y, double _r = 0.5) : VictimDefault() {
    this->x = _x;
    this->y = _y;
    this->radius = _r;
    this->yaw = 0.0;
    this->type = VictimDefault_TYPE::VICTIM_CYLINDER;
  }
  
};

inline bool operator==(const Victim& lhs, const Victim& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}


#endif /* ACEAFEF4_8AAA_41A0_8846_5AAD774D272F */
