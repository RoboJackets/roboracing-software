#include <rr_evgp/UniformCostSearch.h>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <algorithm>
#include <vector>
#include <queue>
#include <functional>


UniformCostSearch::UniformCostSearch(cv::Mat obstacleGrid, cv::Mat distanceGrid, cv::Point startPt, cv::Point goalPt) {
    obstacleGrid_ = obstacleGrid;
    distanceGrid_ = distanceGrid;
    startPoint_ = startPt;
    goalPoint_ = goalPt;
}

UniformCostSearch::State UniformCostSearch::getStartState() {
    UniformCostSearch::State s;
    s.pt = startPoint_;
    s.cost = 0.0;
    return s;

}

bool UniformCostSearch::isGoalState(UniformCostSearch::State s) {
    //#TODO
    if (s.pt == goalPoint_) {
        return true;
    }
    return false;
}

float UniformCostSearch::pointToCost(cv::Point pt) {
    return distanceGrid_.at<float>(pt); //DistanceTransform outputs CV_32F
}

bool UniformCostSearch::isValidPoint(cv::Point pt) {
    cv::Rect rect(cv::Point(), distanceGrid_.size());
    return rect.contains(pt)&& obstacleGrid_.at<uchar>(pt) == 0; //Anything above 0 is obstacle
}

std::vector<UniformCostSearch::State> UniformCostSearch::getSuccessors(UniformCostSearch::State s) {
    //returns the 8 point grid around a center point
    std::vector<UniformCostSearch::State> successors;
    for (int i = -1; i < 2; i++) {
        for (int j = -1; j < 2; j++) {
            if (i != 0 || j != 0) { // not center position
                cv::Point pt(s.pt.x+i, s.pt.y+j);
                if (this->isValidPoint(pt)) {
                    UniformCostSearch::State newState;
                    newState.pt = pt;
                    newState.cost = this->pointToCost(pt);
                    successors.push_back(newState);
                }
            }
        }
    }
    return successors;
}

std::vector<cv::Point> UniformCostSearch::search() {
    std::priority_queue<UniformCostSearch::State, std::vector<UniformCostSearch::State>, std::greater<UniformCostSearch::State>> pq; //minimum priority queue
    //#std::vector<cv::Point> visited;
    cv::Mat visited(obstacleGrid_.size(), CV_8UC1, cv::Scalar(0));

    pq.push(this->getStartState());
    //uniform cost search
    while (!pq.empty()) { //#&& visited.size() < obstacleGrid_.size().area()) {
        UniformCostSearch::State e = pq.top();
        //std::cout << e.pt.x << "," << e.pt.y << std::endl;
        if (visited.at<uchar>(e.pt) == 0) {//#std::find(visited.begin(), visited.end(), e.pt) == visited.end()) { //pt not visited yet
            if (this->isGoalState(e)) {
                //std::cout << e.cost<< std::endl;
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
            //#visited.push_back(e.pt);
            visited.at<uchar>(e.pt) = 255;
        }
        pq.pop(); //remove it now that we are done
    }
    return std::vector<cv::Point>(); //no path found
}

void UniformCostSearch::setStartPoint(cv::Point pt) {
    startPoint_ = pt;
}

void UniformCostSearch::setGoalPoint(cv::Point pt) {
    goalPoint_ = pt;
}
