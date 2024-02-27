#include <cmath>
#include <vector>
#include <fstream>
#include <cmath>
#include <iostream>
#include <webots/Supervisor.hpp>
class ChassisPos{
  public:
    bool endPath = false;
    float x;
    float y;
    float theta;
    ChassisPos(float x, float y, float theta = 0);
      // void follow(std::vector<std::vector<double>> pathPoints, float lookahead, int timeout, bool forwards, bool async, float maxSpeed, Supervisor* robot);

    ChassisPos operator+(const ChassisPos& other) {
      return ChassisPos(this->x + other.x, this->y + other.y, this->theta);
    };   
    
    ChassisPos operator-(const ChassisPos& other) {
      return ChassisPos(this->x - other.x, this->y - other.y, this->theta);
    };
   
    float operator*(const ChassisPos& other) { 
      return this->x * other.x + this->y * other.y; 
    };
      
    ChassisPos operator/(const float& other) {
      return ChassisPos(this->x / other, this->y / other, this->theta);
    };
  
    ChassisPos lerp(ChassisPos other, float t) {
      return ChassisPos(
        this->x + (other.x - this->x) * t, 
        this->y + (other.y - this->y) * t, 
        this->theta
      );
    };

    float circleIntersect(ChassisPos p1, ChassisPos p2, ChassisPos pose, float lookaheadDist);

    float findCurvature(ChassisPos pos, float heading, ChassisPos lookahead);

    int findClosest(ChassisPos pos, std::vector<std::vector<double>> path);

    ChassisPos lookaheadPoint(ChassisPos lastLookahead, ChassisPos pose, std::vector<std::vector<double>> path, float lookaheadDist);

    void follow(std::vector<std::vector<double>> pathPoints, float lookahead, int timeout, bool forwards, bool async, float maxSpeed, webots::Supervisor* robot);
};