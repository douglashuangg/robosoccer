#include <cmath>
#include <vector>
#include <fstream>
#include <cmath>
#include <iostream>
#include "PurePursuit.h"

ChassisPos::ChassisPos(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

// pick the closest point from the path
int ChassisPos::findClosest(ChassisPos pos, std::vector<std::vector<double>> path){
    int closestPoint;
    float closestDistance = 1000000;
    float dist;

    for(int i = 0; i<path.size(); i++){
        // path[i][0] is x coordinate, and path[i][1] is y coordinate
        dist = std::hypot(path[i][0] - pos.x, path[i][1] - pos.y);
        if(dist < closestDistance){
            closestDistance = dist;
            closestPoint = i;
        }
    }

    return closestPoint;
}

// return positive or negative 1 depending on the sign of the input
int sgn(float x) {
    if (x < 0) return -1;
    else return 1;
}

float ChassisPos::findCurvature(ChassisPos pos, float heading, ChassisPos lookahead){
    std::cout << "heading" << heading << std::endl;
    float a = -std::tan(heading);
    float c = std::tan(heading)*pos.x - pos.y;
    float x = std::fabs(a*lookahead.x + lookahead.y + c)/std::sqrt(a*a + 1);
    float L = std::hypot(lookahead.x - pos.x, lookahead.y - pos.y);
    float side = sgn(std::sin(heading)*(lookahead.x - pos.x) - std::cos(heading)*(lookahead.y - pos.y));
    std::cout << "x: " << x << " L " << L << std::endl;
    return side * ((2 * x) / (L*L));
}


float ChassisPos::circleIntersect(ChassisPos p1, 
                                  ChassisPos p2, 
                                  ChassisPos pose, 
                                  float lookaheadDist) {
    // uses the quadratic formula to calculate intersection points
    ChassisPos d = p2 - p1;
    ChassisPos f = p1 - pose;
    float a = d * d;
    float b = 2 * (f * d);
    float c = (f * f) - lookaheadDist * lookaheadDist;
    float discriminant = b * b - 4 * a * c;

    // if a possible intersection was found
    if (discriminant >= 0) {
        discriminant = sqrt(discriminant);
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        // prioritize further down the path
        if (t2 >= 0 && t2 <= 1) return t2;
        else if (t1 >= 0 && t1 <= 1) return t1;
    }

    // no intersection found
    return -1;
}


ChassisPos ChassisPos::lookaheadPoint(ChassisPos lastLookahead, 
                                      ChassisPos pose, 
                                      std::vector<std::vector<double>> path,
                                      float lookaheadDist) 
{
    // std::cout << "LOOKAHEAD POINT" << path.size() << " " << lastLookahead.theta << std::endl;
    // it used to be lastLookAhead.theta for optimization I think, but is an issue if
    // a new path is constantly being generated
    for (int i = path.size() - 1; i > 0; i--) {
        const auto& previousPath = path.at(i - 1);
        const auto& currentPath = path.at(i);
        // std::cout << "Calculating lookahead point" << std::endl;

        ChassisPos lastPathChassisPos = ChassisPos(previousPath[0], previousPath[1]);
        // ChassisPos lastPathChassisPos = ChassisPos(pose.x, pose.y);
        ChassisPos currentPathChassisPos = ChassisPos(currentPath[0], currentPath[1]);
        // std::cout << "LAST CHASSIS POS: " << lastPathChassisPos.x << " " << lastPathChassisPos.y << std::endl;
        // std::cout << "CURRENT CHASSIS POS: " << currentPathChassisPos.x << " " << currentPathChassisPos.y << std::endl;
        float t = circleIntersect(lastPathChassisPos, currentPathChassisPos, pose, lookaheadDist);
        // std::cout << "LOOKAHEAD T: " << lastPathChassisPos.x << " " << lastPathChassisPos.y << std::endl;
        // std::cout << "LOOKAHEAD T: " << currentPathChassisPos.x << " " << currentPathChassisPos.y << std::endl;

        //std::cout << "LOOKAHEAD T: " << t << std::endl;
        if (t != -1) {
            std::cout << "valid" << std::endl;
            ChassisPos lookahead = lastPathChassisPos.lerp(currentPathChassisPos, t);
            lookahead.theta = i;
            return lookahead;
        }
    }

    // robot deviated from path, use last lookahead point
    return lastLookahead;
}