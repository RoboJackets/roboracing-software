#ifndef UCOSTSEARCH_H
#define UCOSTSEARCH_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

class UniformCostSearch {
    public:
        UniformCostSearch(cv::Mat obstacleGrid, cv::Mat distanceMap, cv::Point startPt, cv::Point goalPt);
        std::vector<cv::Point> search();
        void setStartPoint(cv::Point pt);
        void setGoalPoint(cv::Point pt);
        cv::Point getNearestFreePointBFS(cv::Point initPoint);

    private:
        struct State {
            cv::Point pt;
            std::vector<cv::Point> path;
            float cost;
            bool operator<(const State& n) const {
                return cost < n.cost;
            }
            bool operator>(const State& n) const {
                return cost > n.cost;
            }
        };
        cv::Mat obstacleGrid_;
        cv::Mat distanceGrid_;
        cv::Point startPoint_;
        cv::Point goalPoint_;
        State getStartState();
        bool isGoalState(State state);
        float pointToCost(cv::Point pt);
        bool isValidPoint(cv::Point pt);
        std::vector<State> getSuccessors(State state);

};
#endif
