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

struct Point {
    double x;
    double y;
} typedef Point_t;

class GridWorld {
    public:
        GridWorld(double noise = 0.9, double width = 1.0, double height = 1.0, const std::vector<pair<double, double>> &blocked_states = {}, double discount_factor = 0.99);
        void ValueIteration(pair<double, double> curr_ball_state, pair<double, double> curr_state);
        vector<vector<double>> ExtractPolicy(pair<double, double> currentState);

    private:
        double _noise;
        double _width;
        double _height;
        std::vector<pair<double, double>> blocked_states_;
        float discount_factor_;
        bool ball_square = false;
        pair<double, double> ball_state;
        pair<double, double> future_ball_state;
        Point_t start_goal_post = {0.5, 0.5};
        Point_t end_goal_post = {0.5, 0.0};
        pair<double, double> current_state;
        // pair<double, double> start_goal_post = std::make_pair(0.5, 0.25);
        // pair<double, double> end_goal_post = std::make_pair(0.5, -0.25);
        vector<double> episode_rewards;
        vector<vector<int>> goal_states;
        std::map<pair<double, double>, double> state_values;

        std::vector<pair<double, double>> GetStates();
        vector<Action> GetActions();
        vector<pair<double, double>> GetTransitions(pair<double, double> state, Action action);
        double GetReward(pair<double, double> state, Action action);
        bool IsTerminal(pair<double, double> state);
        double CalculateDistance();
        double dot(Point_t a, Point_t b);
        double dist(Point_t a, Point_t b);
};

#endif