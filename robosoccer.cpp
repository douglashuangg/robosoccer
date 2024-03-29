// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/InertialUnit.hpp>
#include <PurePursuit.h>
#include <GridWorld.h>

#include <cmath>
#include <vector>
#include <fstream>
#include <cmath>
#include <iostream>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define BASE_WIDTH_IN 0.052

void ChassisPos::follow(std::vector<std::vector<double>> pathPoints, float lookahead, int timeout, bool forwards, bool async, float maxSpeed, Supervisor* robot){
    // call function to get the robots current position estimation
     // if (async) {
        // pros::Task task([&]() { follow(pathPoints, lookahead, timeout, forwards, false, maxSpeed); });
        // pros::delay(10); // delay to give the task time to start
        // return;
    // }
    
    //Supervisor *robot = new Supervisor();
    //int timeStep = (int)robot->getBasicTimeStep();
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    Node *robot_node = robot->getFromDef("EPUCK");
    InertialUnit *inertial_unit = (InertialUnit*)robot->getInertialUnit("inertial unit");
    Field *trans_field = robot_node->getField("translation");
    GridWorld gridWorld;
    // const double *ballPosition = ball->getPosition();
    inertial_unit->enable(10);
   
    const double *initial = trans_field->getSFVec3f();
   
    ChassisPos pose(initial[0], initial[1], 0);
    std::cout << initial[0] << " " << initial[1] << " " << std::endl;
    ChassisPos lastPose = pose;
    
    float curvature;
    int closestPoint = 0;

    ChassisPos lookAheadPose(0, 0, 0);
    // because our path is vectors, we need to get the x and y and then save it
    // into a ChassisPos class
    const auto& previousPath = pathPoints.at(0);
    ChassisPos lastLookahead = ChassisPos(previousPath[0], previousPath[1]);
    lastLookahead.theta = 0;
    Node *botNode = robot->getFromDef("EPUCK");
    Node *ball = robot->getFromDef("BALL");
    //gridWorld.ValueIteration();
    //std::pair<double, double> currentState = std::make_pair(0,0);
    // vector<vector<double>> optimalPath = gridWorld.GetOptimalPath(currentState);
    // need to test for how long this for loop will run
    vector<vector<double>> optimalPath;

    for(int i = 0; i < timeout/10 && !endPath; i++)
    {
         const double * inertial_meas = inertial_unit->getRollPitchYaw();
        std::cout << "INERTIAL" << inertial_meas[0] << inertial_meas[1] << inertial_meas[2] << std::endl;
        const double *values = trans_field->getSFVec3f();
        const double *botPosition = botNode->getPosition();
        const double *ballPosition = ball->getPosition();
        cout << "Ball " << ballPosition[0] << ballPosition[1] << endl;
        double ballX = round(ballPosition[0]*100)/100;
        double ballY = round(ballPosition[1]*100)/100;
        std::pair<double, double> currentBallState = std::make_pair(ballX, ballY);
        double currentX = round(botPosition[0]*100)/100;
        double currentY = round(botPosition[1]*100)/100;
        
        std::pair<double, double> currentState = std::make_pair(currentX, currentY);
        gridWorld.ValueIteration(currentBallState, currentState);
        optimalPath = gridWorld.ExtractPolicy(currentState);
        // }
          // for(int i = 0; i<sizeof(optimalPath); i++){
            // cout << optimalPath[i][0] << " " << optimalPath[i][1] << endl;
          // }
        //}
    // 
        //std::cout << "Rotation" << botRotation[3] << std::endl;
        // get the current robot position
        pose = ChassisPos(botPosition[0], botPosition[1], inertial_meas[2]);
        lastPose = pose;
       // std::cout << "Position" << values[0] << values[1] <<std::endl;
        
        // get the index of closest point on the path
        closestPoint = findClosest(pose, optimalPath);
        
        // end condition
        std::cout << closestPoint << optimalPath.size()-1 << std::endl;
        if (closestPoint == optimalPath.size() - 1) {
             cout << "FINISHED" << endl;
             //endPath = true;
             //break;
        }
        
        // check if in between threshold angle
  
        // find the lookahead point
        lookAheadPose = lookaheadPoint(lastLookahead, pose, optimalPath, lookahead);
        lastLookahead = lookAheadPose;
        std::cout << "lookahead " << lookAheadPose.x << " " << lookAheadPose.y << std::endl;
        // find the curvature of the arc between robot and the lookahead point
        std::cout << "pose theta" << pose.theta << std::endl;
        float heading = pose.theta;
        curvature = findCurvature(pose, heading, lookAheadPose);
        std::cout << "curvature " << curvature << std::endl;

        // setting some arbitrary velocity, don't know how fast it's supposed to be
        double targetVel = 3;
        double targetLeftVel;
        double targetRightVel;
        
        double ball_angle = atan2(ballY - currentY, ballX - currentX);
        // if(std::fabs(ball_angle - botRotation[3] > 0.524)){
          // std::cout << "corrected" << ball_angle << std::endl;
          // targetLeftVel = 0;
          // targetRightVel = 6.25;
        // } else {
         // calculate target left and right wheel velocities
        targetLeftVel = (targetVel * (2.0 + curvature * BASE_WIDTH_IN) / 2.0);
        targetRightVel = (targetVel * (2.0 - curvature * BASE_WIDTH_IN) / 2.0);
        std::cout << "Not corrected" << std::endl;
        float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / maxSpeed;
        if (ratio > 1) {
          targetLeftVel /= ratio;
          targetRightVel /= ratio;
        }
        std::cout << "CURRENT ROBOT" << currentX << currentY << std::endl;
        // }
        // std::cout << "VELOCITY: " << targetLeftVel << " " << targetRightVel << std::endl;

        // ratio the target velocities in terms of the max speed or use PID
        

        std::cout << "after ratio VELOCITY: " << targetLeftVel << " " << targetRightVel << std::endl;

        // from MP: param power millivolts from -12000 to 12000
        if(!endPath){
             //std::cout << "velocity" << targetLeftVel << targetRightVel << std::endl;
             leftMotor->setVelocity(std::fabs(targetLeftVel));
             rightMotor->setVelocity(std::fabs(targetRightVel));
             leftMotor->setPosition(INFINITY);
             rightMotor->setPosition(INFINITY);
        }
        
          // Get the time step of the simulation
      int timeStep = (int)robot->getBasicTimeStep();
    
      // Get the current simulation time
      int startTime = (int)robot->getTime();
    
      // Delay duration in milliseconds
      int delay = 1.3;
    
      while (robot->step(timeStep) != -1) {
        // Calculate the elapsed time
        int currentTime = (int)robot->getTime();
        int elapsedTime = currentTime - startTime;
    
        // Break the loop when the desired delay is reached
        if (elapsedTime >= delay)
          break;
      }
    }
    // commands to stop the robot, functions from drive.hpp in MP
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    delete robot;
}
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  std::vector<std::vector<double>> path =  {{0.01, 0.05}, {0.01, 0.1}, {0.01, 0.15}, {0.01, 0.2}, {0.01, 0.25}, {0.01, 0.3}, {0.01, 0.35}, {-0, 0.25}};
  ChassisPos follower(0, 0, 0);
  Supervisor *robot = new Supervisor();
  follower.follow(path, 0.08, 200, true, false, 6.25, robot);


  Node *ballNode = robot->getFromDef("pingpong");
  Node *botNode = robot->getFromDef("EPUCK");

  const double *ballPosition = ballNode->getPosition();

  std::cout << "Ball position: " << ballPosition[0] << " " << ballPosition[1] << std::endl;

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    // leftWheel->setVelocity(targetVelocity);
    // rightWheel->setVelocity(targetVelocity);
    const double *botPosition = botNode->getPosition();

    std::cout << "Robot position: " << botPosition[0] << " " << botPosition[1] << std::endl;

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
