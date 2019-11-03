#include <ros/publisher.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <queue>

#include <distance_map_msgs/DistanceMap.h>

using namespace std;

/*
Overall #TODOs
To get test going:
update successor function to output states with costs
Convert point path to Path message of poses

Betterments:
Sync the costmap obstacles to the distance map.
make sure successor never outputs option in obstacle.
update isEndState
Move UCS to its own class
*/

ros::Publisher pub_path;

struct Point {
    int x;
    int y;
};

struct State {
    Point pt;
    vector<Point> path;
    float cost;
    bool operator<(const State& n) const {
        return cost < n.cost;
    }
    bool operator>(const State& n) const {
        return cost > n.cost;
    }
};

//access 1d array as 2d (row major order)
//note we handle x as column and y as row
float getItem(int x, int y, int yMax, float data[]) {
    return data[(y * yMax) + x]; //#TODO: check if out of bounds
}
float getItem(Point point, int yMax, float data[]) {
    return data[(point.y * yMax) + point.x]; //#TODO: check if out of bounds
}

float manhattanDistance(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

State start; //#TODO;

bool isEndState(State s) {
    //#TODO: want to get back nearby the the start but like behind it.
    //^maybe a way to see that the car left the range and then see when it comes in range again
    //float allowableDist = 1000.0;
    //return manhattanDistance(start.pt, s.pt) <= allowableDist; //want to get back nearby
    static int test = 0;
    test++;
    return test > 2000;
}
 int distanceMapWidth; //#TODO
 float distanceMap[];
// returns a list of States. Each state only has a pt and a cost
vector<State> getSuccessors(Point pt) {
    vector<State> successors;
    //go through x y coords around pt and add them.
    //Make sure they are in the map range and not an obstacle
    //#TODO
    //this is placeholder code for testing assuming we are not near edges
    for (int yi = 1; yi >= -1; yi--) {
        for (int xi = 1; xi >= -1; xi--) {
            State t;
            t.pt = {pt.x+xi, pt.y+yi};
            t.cost = getItem(t.pt, distanceMapWidth, distanceMap);
            successors.push_back(t);
        }
    }
    return successors;
}

vector<Point> uniformCostSearch() {
    //store data as tuple(coord, path of coords, cost)
    priority_queue<State> pq;
    set<Point> visited;

    pq.push(start);
    visited.insert(start);

    //uniform cost search
    while (!pq.empty()) {
        State e = pq.top();
        if (visited.count(e.pt) == 0) { //pt not visisted yet
            if (isEndState(e)) {
                return e.path; //#TODO: add the e.pt to the path?
            }
        }
        vector<State> successors = getSuccessors(e.pt);
        for (State s : successors) {
            State newState;
            newState.pt = s.pt;
            newState.path = e.path;
            newState.path.push_back(s.pt);
            newState.cost = e.cost + s.cost;
            pq.push(newState)
        }
        visited.insert(e.pt);
        pq.pop(); //remove it now that we are done

    }
    return vector<Point>();
}

nav_msgs::Path convertPath(vector<Point> pointPath) {
    nav_msgs::Path pathMsg;
    for (Point p : pointPath) { //#TODO stamps and orientation
        geometry_msgs::PoseStamped poseStamped;
        //poseStamped.header.stamp =
        //{x,y,z} in real map
        poseStamped.pose.position.x = p.x; //#TODO: convert to meters map
        poseStamped.pose.position.y = p.y;
        path.poses.push_back(poseStamped)
    }
    return pathMsg;
}


//void map_callback(const nav_msgs::Path& msg) {
void map_callback(const distance_map_msgs::DistanceMap& msg)
    //make 1d array accessible
    distanceMap = msg->data;

    vector<Point> pointPath = UniformCostSearch();
    nav_msgs::Path pathMsg = convertPath(pointPath);

    //#TODO: header and stuff
    pub_path.publish(pathMsg);


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "center_path_finder");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string distance_map_sub_node;
    std::string distance_map_info_sub_node;
    nhp.param("distance_map_sub", distance_map_sub_node,
              string("/basic_mapper/costmap/costmap"));
    nhp.param("distance_map_info_sub", distance_map_info_sub_node,
            string("/basic_mapper/costmap/info"));

    pub_path = nh.advertise<nav_msgs::Path>("/center_path", 1);
    auto distance_map_sub = nh.subscribe(distance_map_sub_node, 1, map_callback);
    auto distance_map_info_sub = nh.subscribe(subscription_node, 1, map_info_callback);


    ros::spin();
    return 0;
}
