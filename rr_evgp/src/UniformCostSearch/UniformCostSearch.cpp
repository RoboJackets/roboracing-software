#include "UniformCostSearch.h"
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <queue>


UniformCostSearch::UniformCostSearch(cv::Mat obstacleGrid, cv::Mat distanceMap, cv::Point startPt, cv::Point goalPt) {
    obstacleGrid_ = obstacleGrid;
    distanceGrid_ = obstacleGrid;
    startPoint_ = startPt;
    goalPoint_ = goalPt;
}

UniformCostSearch::State UniformCostSearch::getStartState() {
    UniformCostSearch::State s;
    s.pt = startPoint_;
    s.cost = 0;
    return s;
}

bool UniformCostSearch::isGoalState(UniformCostSearch::State s) {
    //#TODO
    if (s.pt == goalPoint_) {
        return true;
    }
    return false;
}


std::vector<cv::Point> UniformCostSearch::search() {
    std::priority_queue<UniformCostSearch::State> pq;
    std::vector<cv::Point> visited; //usually might use a set container, but this works fine here

    pq.push(this->getStartState());
    visited.push_back(this->getStartState().pt);

    //uniform cost search
    while (!pq.empty()) { //#TODO: check if all points have been visited
        UniformCostSearch::State e = pq.top();
        if (std::find(visited.begin(), visited.end(), e.pt) != visited.end()) { //pt not visited yet
            if (this->isGoalState(e)) {
                return e.path;
            }
            std::vector<UniformCostSearch::State> successors = this->getSuccessors(e);
            for (UniformCostSearch::State s : successors) {
                UniformCostSearch::State newState;
                newState.pt = s.pt;
                newState.path = e.path;
                newState.path.push_back(s.pt);
                newState.cost = e.cost + s.cost;
                pq.push(newState);
            }
            visited.push_back(e.pt);
            pq.pop(); //remove it now that we are done
        }
    }
    return std::vector<cv::Point>(); //no path found
}
