#include "GridWorld.h"
#include <cstdint>
#include <map>
#include <cmath>
#include <algorithm>

constexpr double GRID_SIZE = 0.040;

double CalculateDistance(uint16_t *point1, uint16_t *point2) {
  double x = point2[0] - point1[0];
  double y = point2[1] - point1[1];
  return sqrt(x*x + y*y);
}


GridWorld::GridWorld(double noise, 
                     double width, 
                     double height, 
                     const std::vector<pair<double, double>> &blocked_states, 
                     double discount_factor) {
  _noise = noise;
  _width = width;
  _height = height;
  blocked_states_ = blocked_states;
  discount_factor_ = discount_factor;
  episode_rewards = {};
  goal_states = {};
  future_ball_state = {};
}

vector<pair<double, double>> GridWorld::GetStates() {
  vector<pair<double, double>> states = {};

  for (double i = -0.20; i < _width; i+=GRID_SIZE) {
    for (double j = -0.20; j < _height; j+=GRID_SIZE) {
      std::pair<double, double> myPair = std::make_pair(std::round(i*1000.0)/1000.0, std::round(j*1000.0)/1000.0);
      if (std::find(blocked_states_.begin(), blocked_states_.end(), myPair) == blocked_states_.end()) {
        states.push_back(std::make_pair(std::round(i*1000)/1000,  std::round(j*1000)/1000));
      }
    }
  }
  return states;
}

vector<Action> GridWorld::GetActions() {
  return {Action::UP, Action::DOWN, Action::LEFT, Action::RIGHT};
}        


vector<pair<double, double>> GridWorld::GetTransitions(pair<double, double> state, Action action) {
  vector<pair<double, double>> transitions;
  // if (is_terminal(state)) {
  //   return transitions;
  // }
  double x = state.first;
  double y = state.second;
  pair<double, double> future_state;
  switch(action) {
    case Action::UP:
      //if (state.second < _height - GRID_SIZE) {
      future_state = std::make_pair(x, std::round((y + GRID_SIZE)*1000.0)/1000.0);
      transitions.push_back(future_state);
      if(future_state == ball_state) {
          ball_square = true;
          future_ball_state = future_state;
      }
      //}
      break;
    case Action::DOWN:
        //if (state.second > 0) {
        future_state = std::make_pair(x,  std::round((y - GRID_SIZE)*1000.0)/1000.0);
        transitions.push_back(future_state);
        if(future_state == ball_state) {
            ball_square = true;
            future_ball_state = future_state;
        }
        //}
        break;
    case Action::LEFT:
        //if (state.first > 0) {
        future_state = std::make_pair(std::round((x - GRID_SIZE)*1000.0)/1000.0, y);
        transitions.push_back(future_state);
        if(future_state == ball_state) {
            ball_square = true;
            future_ball_state = future_state;
        }
        //}
        break;
    case Action::RIGHT:
        //if (state.first < _width - GRID_SIZE) {
        future_state = std::make_pair(std::round((x + GRID_SIZE)*1000.0)/1000.0, y);
        transitions.push_back(future_state);
        if(future_state == ball_state) {
            ball_square = true;
            future_ball_state = future_state;
        }
        //}
        break;
  }

  return transitions;
}

double GridWorld::GetReward(pair<double, double> state, Action action) {
  //double max_distance = 6;
  // if (is_terminal(state)) {
  //   return 0;
// }

  if (state == ball_state) {
    // cout << "reached" << '\n';
    return 2;
  // } else {
  //   if(future_ball_state){
  //       double distance = CalculateDistance(future_ball_state, [6,1]);
  //       return 10 - (distance/max_distance)*10;
  //   }
  //   // return 1;
  }
  
  return -0.04;
}

bool GridWorld::IsTerminal(pair<double, double> state) {
  // cout << "State: " << state.first << ", " << state.second << '\n';
  return state == ball_state;
}

double RoundToNearest(double number, double multiple) {
  return round(number / multiple) * multiple;
}

void GridWorld::ValueIteration(pair<double, double> curr_ball_state)
{
  ball_state = std::make_pair(RoundToNearest(curr_ball_state.first, GRID_SIZE), RoundToNearest(curr_ball_state.second, GRID_SIZE));
  cout << "Ball State: " << ball_state.first << ", " << ball_state.second << '\n';
  for(int i = 0; i < 30; i++){
    double delta = 0.0;
    map <pair<double, double>, double> new_state_values;
    // cout << "Iteration: " << i << '\n';
    for (pair<double, double> state : GetStates()) {
      // if (IsTerminal(state)) {
      //   continue;
      // }
      //double v = 0;
      std::map<std::pair<pair<double, double>, Action>, double> action_values;
      double max_action = -10000;
      for (Action action : GetActions()) {
        double new_value = 0;
        for (pair<double, double> transition : GetTransitions(state, action)) {

          double reward = GetReward(state, action);
          new_value += (reward + discount_factor_ * state_values[transition]);
        }
        action_values[make_pair(state, action)] += new_value;
        if(action_values[make_pair(state, action)] > max_action){
          max_action = action_values[make_pair(state, action)];
        }
      }
      delta = max(delta, abs(state_values[state] - max_action));
      
      new_state_values[state] = std::round(max_action*1000.0)/1000.0;
    }
    state_values = new_state_values;
  }
  // double count = 0;
  // cout << "SIZE" << state_values.size() << '\n';
  // for(const auto& state : state_values){
  //   if(count < 6){
  //     //cout << " " << state.first.first << ", " << state.first.second;
  //     cout << " : " << state.second << flush;
  //     count ++;
  //   } else {
  //     cout << " : " << state.second << '\n';
  //     count = 0;
  //   }
  // }
  // cout << "TEST" << state_values[std::make_pair(0.0, 0.1)] << '\n';
}

vector<vector<double>> GridWorld::ExtractPolicy(pair<double, double> currentState){
  cout << "THIS STATE " << currentState.first << ", " << currentState.second << '\n';
  vector<vector<double>> optimalPath;
  currentState = std::make_pair(RoundToNearest(currentState.first, GRID_SIZE), RoundToNearest(currentState.second, GRID_SIZE));
  // optimalPath.push_back(vector<double>{currentState.first, currentState.second});

  // cout << "Extracting Policy" << '\n';
  int count = 0;
  // while(!IsTerminal(currentState)){
  while(!IsTerminal(currentState)){
    //cout << "Terminal" << IsTerminal(currentState) << '\n';
    // for each state, find the action that maximizes the q-value
    // (i.e., the greedy action)
    double max_q = -INFINITY;
    pair<double, double> best_state;
    // Action best_action;
    for (Action action : GetActions()) {
      double q_value;
      pair<double, double> temp_transition;
      for (pair<double, double> transition : GetTransitions(currentState, action)){
        // cout << "value: " << state_values[std::make_pair(0.09 ,0)] << " : " << transition.first << ", " << transition.second << '\n';
        //cout << "maxq: " << max_q << '\n';
        q_value = state_values[transition];
        temp_transition = transition;
      }
      if (q_value > max_q) {
        max_q = q_value;
        // best_action = action;
        best_state = temp_transition;
      }
      
    }
    
    currentState = best_state;
    optimalPath.push_back(vector<double>{currentState.first, currentState.second});
    //policy[state] = best_action;
    cout << "Path: " << currentState.first << ", " << currentState.second << '\n';
    if(IsTerminal(best_state)){
        cout << "break" << '\n';
        break;
    }
    count ++;
    if(count > 30){
      break;
    }
  }
  return optimalPath;
}