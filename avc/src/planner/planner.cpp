#include "planner.h"

using namespace std;

planner::planner() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("steer_stddev", STEER_STDDEV);
    pnh.getParam("max_steer_angle", MAX_STEER_ANGLE);
    pnh.getParam("max_speed", MAX_SPEED);
    pnh.getParam("path_stage_time", PATH_STAGE_TIME);
    pnh.getParam("path_stages", PATH_STAGES);
    pnh.getParam("collision_radius", COLLISION_RADIUS);
    pnh.getParam("path_iterations", PATH_ITERATIONS);
    pnh.getParam("time_increment", TIME_INCREMENT);
    pnh.getParam("alternate_path_threshold", ALT_PATH_THRESHOLD);
    pnh.getParam("connected_path_distance", CONNECTED_PATH_DIST);

    map_sub = nh.subscribe("map", 1, &planner::mapCb, this);
    speed_pub = nh.advertise<rr_platform::speed>("speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("steering", 1);
    path_pub = nh.advertise<nav_msgs::Path>("path", 1);

    steering_gaussian = normal_distribution<double>(0, STEER_STDDEV);
    rand_gen = mt19937(std::random_device{}());
}

planner::pose planner::calculateStep(double velocity, double steer_angle, double timestep,
                                     planner::pose pStart) {
    if (abs(steer_angle) < 1e-6) {
        deltaX = velocity * timestep;
        deltaY = 0;
        deltaTheta = 0;
    } else {
        double turn_radius = constants::wheel_base / sin(abs(steer_angle));
        double temp_theta = velocity * timestep / turn_radius;
        deltaX = turn_radius * cos(M_PI / 2 - temp_theta);
        deltaY;
        if (steer_angle < 0) {
            deltaY = turn_radius - turn_radius * sin(M_PI / 2 - temp_theta);
        } else {
            deltaY = -(turn_radius - turn_radius * sin(M_PI / 2 - temp_theta));
        }
        deltaTheta = velocity / constants::wheel_base * sin(-steer_angle) * timestep;
    }
    pose p;
    p.x = pStart.x + (deltaX * cos(pStart.theta)
                    - deltaY * sin(pStart.theta));
    p.y = pStart.y + (deltaX * sin(pStart.theta)
                    + deltaY * cos(pStart.theta));
    p.theta = pStart.theta + deltaTheta;
    return p;
}

double planner::calculatePathCost(planner::sim_path path,
                                  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
    double cost = 0.0;
    for(int i = 0; i < path.poses.size(); i++) {
        cost += costAtPose(path.poses[i], kdtree) / pow(path.speeds[i], 2);
    }
    return cost;
}

planner::sim_path planner::calculatePath(vector<double> angles) {
    sim_path out;
    out.poses.resize(angles.size());
    out.speeds.resize(angles.size());
    out.speeds[0] = steeringToSpeed(angles[0]);
    out.poses[0] = calculateStep(out.speeds[0], angles[0], TIME_INCREMENT);
    for(int i = 1; i < angles.size(); i++) {
        out.speeds[i] = steeringToSpeed(angles[i]);
        out.poses[i] = calculateStep(out.speeds[i], angles[i], TIME_INCREMENT, out.poses[i-1]);
    }
    return out;
}

// eyeballed it. see https://www.desmos.com/calculator/suy8y1ylf0
double planner::steeringToSpeed(double angle) {
    return MAX_SPEED * cos(angle * 1.4706 / MAX_STEER_ANGLE);
}

double planner::costAtPose(pose step, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree) {
    pcl::PointXYZ searchPoint(step.x, step.y, 0);
    vector<int> pointIdxRadiusSearch(1);
    vector<float> pointRadiusSquaredDistance(1);
    int nResults = kdtree.nearestKSearch(searchPoint, 1, pointIdxRadiusSearch,
                                         pointRadiusSquaredDistance);

    double distSqr = pointRadiusSquaredDistance[0];
    double dist = sqrt(distSqr);

    if(nResults == 0) return 0; //is blind
    if(dist < COLLISION_RADIUS) return 1000.0; //collision
    return (1.0 / dist);
}

double planner::steeringSample() {
    double raw = steering_gaussian(rand_gen);
    if(raw >= -MAX_STEER_ANGLE && raw <= MAX_STEER_ANGLE) return raw;
    else return steeringSample();
}

geometry_msgs::PoseStamped planner::plannerPoseToPoseStamped(planner::pose pp) {
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = pp.x;
    ps.pose.position.y = pp.y;
    return ps;
}

//use this to sort WeightedSteeringVecs by first steer value in increasing order
bool planner::steeringVecCompare(const WeightedSteeringVec wsv1, const WeightedSteeringVec wsv2) {
    return wsv1.steers[0] < wsv2.steers[0];
}

//return n-dimensional euclidean distance
double distance(vector<double> vec1, vector<double> vec2) {
    double sum = 0;
    for(int i = 0; i < vec1.size(); i++) {
        sum += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
    }
    return sqrt(sum);
}

void planner::SteeringGroup::add(WeightedSteeringVec *wsv) {
    wsv->group = this;
    weightTotal += wsv->weight;
    weightedSteers.push_back(*wsv);
}
void planner::SteeringGroup::addAll(SteeringGroup *sg) {
    weightedSteers.insert(weightedSteers.end(), sg->weightedSteers.begin(),
                          sg->weightedSteers.end());
    for(int i = 0; i < sg->weightedSteers.size(); i++) {
        sg->weightedSteers[i].group = this;
    }
    weightTotal += sg->weightTotal;
}
bool planner::SteeringGroup::operator==(SteeringGroup other) {
    // shallow equality
    return this == &other;
}
vector<double> planner::SteeringGroup::weightedCenter() {
    vector<double> sums;
    for(int j = 0; j < weightedSteers[0].steers.size(); j++) sums.push_back(0);

    for(int i = 0; i < weightedSteers.size(); i++) {
        for(int j = 0; j < weightedSteers[i].steers.size(); j++) {
            sums[j] += weightedSteers[i].steers[j] * weightedSteers[i].weight / weightTotal;
        }
    }
    return sums;
}
double planner::SteeringGroup::averageWeight() {
    return (double)weightTotal / weightedSteers.size();
}

void planner::mapCb(const sensor_msgs::PointCloud2ConstPtr& map) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*map, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    if(cloud->empty()) {
        ROS_WARN("Point cloud is empty");
        rr_platform::speedPtr speedMSG(new rr_platform::speed);
        rr_platform::steeringPtr steerMSG(new rr_platform::steering);
        speedMSG->speed = 0.5; //proceed with caution; E-Kill if necessary
        steerMSG->angle = 0;
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    //build paths and evaluate their weights
    vector<WeightedSteeringVec> weightedSteerVecs(PATH_ITERATIONS);
    double angle;
    double bestWeight = 0;
    int increments = PATH_STAGE_TIME / TIME_INCREMENT;
    for(int i = 0; i < PATH_ITERATIONS; i++) {
        vector<double> steerPath(increments * PATH_STAGES, 0);
        WeightedSteeringVec wsv;
        wsv.group = nullptr;
        for(int s = 0; s < PATH_STAGES; s++) {
            angle = steeringSample();
            wsv.steers.push_back(angle);
            fill_n(steerPath.begin() + s*increments, increments, angle);
        }

        sim_path sp = calculatePath(steerPath);
        wsv.weight = 1.0 / calculatePathCost(sp, kdtree);
        weightedSteerVecs.push_back(wsv);

        if(wsv.weight > bestWeight) bestWeight = wsv.weight;
    }
    //ROS_INFO("best weight %f", bestWeight);

    // filter paths by weight and sort them by first steering value
    vector<WeightedSteeringVec> weightedSteerVecsFiltered;
    for(int i = 0; i < weightedSteerVecs.size(); i++) {
        if(weightedSteerVecs[i].weight > bestWeight * ALT_PATH_THRESHOLD) {
            weightedSteerVecsFiltered.push_back(weightedSteerVecs[i]);
        }
    }
    sort(weightedSteerVecsFiltered.begin(), weightedSteerVecsFiltered.end(), steeringVecCompare);

    // connect components based on distance
    for(int i = 0; i < weightedSteerVecsFiltered.size(); i++) {
        WeightedSteeringVec *thisSteerVec = &weightedSteerVecsFiltered[i];
        int j = i - 1;
        bool foundMatch = false;
        // search backwards along axis 0 for groups to connect to.
        // stops searching when j==0 or element j is more than connection radius away on axis 0
        while(j >= 0
           && (thisSteerVec->steers[0] - weightedSteerVecsFiltered[j].steers[0]) < CONNECTED_PATH_DIST)
        {
            WeightedSteeringVec *thatSteerVec = &weightedSteerVecsFiltered[j];
            if((thisSteerVec->group != thatSteerVec->group)
               && (distance(thisSteerVec->steers, thatSteerVec->steers) < CONNECTED_PATH_DIST))
            {
                if(foundMatch) {
                    //has already found a match for thisSteerVec. Merge groups
                    thisSteerVec->group->addAll(thatSteerVec->group);
                } else {
                    thatSteerVec->group->add(thisSteerVec);
                    foundMatch = true;
                }
            }
            j--;
        }

        if(!foundMatch) {
            SteeringGroup *sg = new SteeringGroup;
            sg->add(thisSteerVec);
            thisSteerVec->group = sg;
        }
    }

    // find the group with highest average weight and use its steering angles
    vector<SteeringGroup *> groups;
    SteeringGroup *bestGroup;
    double bestGroupAverageWeight = -1.0;
    for(int i = 0; i < weightedSteerVecsFiltered.size(); i++) {
        SteeringGroup* thisGroup = weightedSteerVecsFiltered[i].group;
        if(find(groups.begin(), groups.end(), thisGroup) == groups.end()) {
            //group has not been tested
            groups.push_back(thisGroup);
            double thisGroupAverageWeight = thisGroup->averageWeight();
            if(thisGroupAverageWeight > bestGroupAverageWeight) {
                bestGroupAverageWeight = thisGroupAverageWeight;
                bestGroup = thisGroup;
            }
        }
    }
    ROS_INFO("Planner found %d path categories", (int)groups.size());

    //construct a best path
    vector<double> bestGroupCenter = bestGroup->weightedCenter();
    vector<double> bestSteering(increments * PATH_STAGES, 0);
    for(int s = 0; s < PATH_STAGES; s++) {
        fill_n(bestSteering.begin() + s*increments, increments, bestGroupCenter[s]);
    }
    sim_path bestPath = calculatePath(bestSteering);

    desired_steer_angle = bestSteering[0];
    desired_velocity = bestPath.speeds[0];

    rr_platform::speedPtr speedMSG(new rr_platform::speed);
    rr_platform::steeringPtr steerMSG(new rr_platform::steering);
    speedMSG->speed = desired_velocity;
    steerMSG->angle = desired_steer_angle;
    speed_pub.publish(speedMSG);
    steer_pub.publish(steerMSG);

    nav_msgs::Path pathMsg;
    std::transform(bestPath.poses.begin(), bestPath.poses.end(),
                   back_inserter(pathMsg.poses), plannerPoseToPoseStamped);

    pathMsg.header.frame_id = "base_footprint";
    path_pub.publish(pathMsg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    planner plan;
    ros::spin();
    return 0;
}
