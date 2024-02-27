#ifndef GRIDWORLD_H
#define GRIDWORLD_H

#include <iostream>
#include <vector>
#include <cstdint>
#include <map>

using namespace std;

enum class Action {
    UP,
    DOWN,
    LEFT,
    RIGHT,
};

class GridWorld {
    public:
        // GridWorld();
        GridWorld(double noise = 0.9, double width = 0.5, double height = 0.5, const std::vector<pair<double, double>> &blocked_states = {}, double discount_factor = 0.99);
        void ValueIteration(pair<double, double> curr_ball_state);
        vector<vector<double>> ExtractPolicy(pair<double, double> currentState);

    private:
        double _noise;
        double _width;
        double _height;
        std::vector<pair<double, double>> blocked_states_;
        float discount_factor_;
        bool ball_square = false;
        pair<double, double> future_ball_state;
        vector<double> episode_rewards;
        vector<vector<int>> goal_states;
        pair<double, double> ball_state;
        std::map<pair<double, double>, double> state_values;

        std::vector<pair<double, double>> GetStates();
        vector<Action> GetActions();
        vector<pair<double, double>> GetTransitions(pair<double, double> state, Action action);
        double GetReward(pair<double, double> state, Action action);
        bool IsTerminal(pair<double, double> state);
};

#endif