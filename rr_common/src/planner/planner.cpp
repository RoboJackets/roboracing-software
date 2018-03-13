#include "planner.h"
#include <algorithm>

using namespace std;

/*
 * advanceStep: calculate one distance-step of "simulated"
 * vehicle motion. Undocumented Ackermann steering trig is
 * grandfathered in.
 * Params:
 * steerAngle - steering angle in radians centered at 0
 * inOutPose - pose (x,y,theta) that is read and updated
 *      with the new pose
 */

void advanceStep(float steerAngle, Pose2D &inOutPose) {
    float deltaX, deltaY, deltaTheta;
    if (abs(steerAngle) < 1e-3) {
        deltaX = DISTANCE_INCREMENT;
        deltaY = 0;
        deltaTheta = 0;
    } else {
        float turnRadius = WHEEL_BASE / sin(abs(steerAngle));
        float tempTheta = DISTANCE_INCREMENT / turnRadius;
        deltaX = turnRadius * cos(M_PI / 2 - tempTheta);
        if (steerAngle < 0) {
            deltaY = turnRadius - turnRadius * sin(M_PI / 2 - tempTheta);
        } else {
            deltaY = -(turnRadius - turnRadius * sin(M_PI / 2 - tempTheta));
        }
        deltaTheta = DISTANCE_INCREMENT / WHEEL_BASE * sin(-steerAngle);
    }

    inOutPose.x += deltaX * cos(inOutPose.theta) - deltaY * sin(inOutPose.theta);
    inOutPose.y += deltaX * sin(inOutPose.theta) + deltaY * cos(inOutPose.theta);
    inOutPose.theta += deltaTheta;
}

/*
 * getCostAtPose: calculate the cost of being in a certain position
 * relative to obstacles. Cost = 1 / d where d is the distance to the
 * nearest detected obstacle.
 * Params:
 * pose - relative (x,y,theta) for a possible future pose. The robot's
 *      actual state is x=0, y=0, theta=0
 * kdtree - FLANN index constructed from the obstacles map
 */
float getCostAtPose(Pose2D &pose, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree) {
    pcl::PointXYZ searchPoint(pose.x, pose.y, 0);
    vector<int> pointIdxRadiusSearch(1);
    vector<float> pointRadiusSquaredDistance(1);
    int nResults = kdtree.nearestKSearch(searchPoint, 1, pointIdxRadiusSearch,
                                         pointRadiusSquaredDistance);

    float dist = sqrt(pointRadiusSquaredDistance[0]);
    if(nResults == 0) { //no obstacles in sight
        return 0;
    } else if(dist < COLLISION_RADIUS) { //collision
        return COLLISION_PENALTY;
    } else {
        return (1.0 / dist);
    }
}

/*
 * steeringToSpeed: Calculate a desired speed from a steering angle.
 * There is no physics here, I just eyeballed something intuitive. See
 * https://www.desmos.com/calculator/suy8y1ylf0
 * TODO utilize a more useful vehicle model
 * Params:
 * steering - proposed steering value to publish
 * maxSteering - the limit of the car's steering ability
 */
inline float steeringToSpeed(float steering, float maxSteering) {
    return MAX_SPEED * cos(steering * 1.4706 / maxSteering);
}

/*
 * aggregateCost: roll out a trajectory through the world using a
 * given control_vector and calculate the total cost of that path.
 * Cost = sum(pos_cost / speed for each position)
 * Params:
 * controlVector - vector of dimension N_PATH_SEGMENTS with a steering
 *      value for each stage/dimension
 * kdtree - FLANN index constructed from the obstacles map
 */
float aggregateCost(control_vector &controlVector, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree) {
    if(controlVector.size() != N_PATH_SEGMENTS) {
        ROS_ERROR("control vector of dimension %d does not match %d path segments",
                  (int)controlVector.size(), N_PATH_SEGMENTS);
    }

    Pose2D pose{0,0,0};
    float costSum = 0.0;

    for(int segmentIdx = 0; segmentIdx < N_PATH_SEGMENTS; segmentIdx++) {
        float steering = controlVector[segmentIdx];
        float speed = steeringToSpeed(steering, STEER_LIMITS[segmentIdx]);
        for(float dist = 0.0;
            dist < SEGMENT_DISTANCES[segmentIdx];
            dist += DISTANCE_INCREMENT)
        {
            advanceStep(steering, pose);
            costSum += getCostAtPose(pose, kdtree) / pow(speed, 2);
        }
    }
    return costSum;
}

/**
 * Member returning the current bestAngle averaged against the median.
 * Median is calculated based on previous angles, or PREV_STEERING_ANGLES
*/
float steeringAngleAverageAgainstMedian(float bestAngle) {
    PREV_STEERING_ANGLES_INDEX = (PREV_STEERING_ANGLES_INDEX + 1) % SMOOTHING_ARRAY_SIZE;
    std::vector<float> sortedSteeringAngles(PREV_STEERING_ANGLES);
    //sort array of previous vectors in ascending order from start to midpoint to find median
    std::nth_element (sortedSteeringAngles.begin(), sortedSteeringAngles.begin() + (SMOOTHING_ARRAY_SIZE / 2), sortedSteeringAngles.end());
    //append the average of the best curve and current median to PREV_STEERING_ANGLES
    PREV_STEERING_ANGLES[PREV_STEERING_ANGLES_INDEX] = (sortedSteeringAngles[SMOOTHING_ARRAY_SIZE / 2] + bestAngle) / 2;
    //return the average between the calculated steering angle and median value
    return (sortedSteeringAngles[SMOOTHING_ARRAY_SIZE / 2] + bestAngle) / 2; 
}

/*
 * steeringSample: get a random steering value from a normal
 * distribution. Each path stage has a potentially different bell
 * curve. If the value is out of bounds, try again
 * Params:
 * stage - the path stage or control vector dimension
 */
float steeringSample(int stage) {
    while(true) {
        float raw = steering_gaussians[stage](rand_gen);
        if(raw >= -STEER_LIMITS[stage] && raw <= STEER_LIMITS[stage])
            return raw;
    }
}

/*
 * getLocalMinima: The new "clustering" function. Given a set of
 * control vectors, find local minima and return their indices
 * in the input matrix.
 * Params:
 * controlVectors - list of lists of steering values of length
 *      N_PATH_SEGMENTS
 * costs - list of costs associated with each control_vector at the
 *      same indices
 * localMinimaIndices - output parameter which holds the indices
 *      of control_vectors with locally minimal costs
 */
void getLocalMinima(const vector<control_vector> &controlVectors,
                    const vector<float> &costs,
                    vector<int> &localMinimaIndices)
{
    int nSamples = controlVectors.size();

    // fill flann-format sample matrix
    float sampleArray[nSamples * N_PATH_SEGMENTS];
    for(int i = 0; i < nSamples; i++) {
        for(int j = 0; j < N_PATH_SEGMENTS; j++) {
            sampleArray[i * N_PATH_SEGMENTS + j] = controlVectors[i][j];
        }
    }
    flann::Matrix<float> samples(sampleArray, nSamples, N_PATH_SEGMENTS);

    // construct FLANN index (nearest neighbors preprocessing)
    flann::Index<flann::L1<float>> flannIndex(samples, flann::KDTreeSingleIndexParams());
    flannIndex.buildIndex();

    // Initialize group membership list
    vector<bool> sampleKnownStatus(nSamples);
    fill_n(sampleKnownStatus.begin(), nSamples, false);

    // initialize input/output parameters for radius searches
    flann::Matrix<int> indices(new int[nSamples], 1, nSamples);
    flann::Matrix<float> dists(new float[nSamples], 1, nSamples);
    flann::Matrix<float> query(new float[N_PATH_SEGMENTS], 1, N_PATH_SEGMENTS);
    flann::SearchParams searchParams(1);
    const float searchRadius = PATH_SIMILARITY_CUTOFF;

    for(int i = 0; i < nSamples; i++) {
        if(sampleKnownStatus[i]) continue;

        // perform radius search
        for(int d = 0; d < N_PATH_SEGMENTS; d++) {
            query[0][d] = samples[i][d];
        }
        int nNeighbors = flannIndex.radiusSearch(query, indices, dists,
                                                 searchRadius, searchParams);

        // Iterate through neighbors, tracking if lower costs exist.
        // Any neighbors in radius that have higher costs are not local minima.
        bool anyBetterInArea = false;
        for(int j = 0; j < nNeighbors; j++) {
            int foundIndex = indices[0][j];
            if(foundIndex != i) { //handle index of query later
                if(costs[foundIndex] < costs[i]) {
                    anyBetterInArea = true;
                } else {
                    sampleKnownStatus[foundIndex] = true; //known not minimum
                }
            }
        }

        sampleKnownStatus[i] = true;
        if(!anyBetterInArea) {
            // includes regions with only one point
            localMinimaIndices.push_back(i);
        }
    }
}


void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*map, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    if (cloud->empty()) {
        ROS_WARN("environment map pointcloud is empty");
        rr_platform::speedPtr speedMSG(new rr_platform::speed);
        rr_platform::steeringPtr steerMSG(new rr_platform::steering);
        speedMSG->speed = MAX_SPEED / 4; //proceed with caution; E-Kill if necessary
        steerMSG->angle = 0;
        steerMSG->header.stamp = ros::Time::now();
        speedMSG->header.stamp = ros::Time::now();
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    vector<control_vector> controlVectors(N_CONTROL_SAMPLES);
    vector<float> costs(N_CONTROL_SAMPLES);

    float bestCost = 1e10;
    for(int i = 0; i < N_CONTROL_SAMPLES; i++) {
        for(int stage = 0; stage < N_PATH_SEGMENTS; stage++) {
            controlVectors[i].push_back(steeringSample(stage));
        }
        float cost = aggregateCost(controlVectors[i], kdtree);
        if(cost < bestCost)
            bestCost = cost;
        costs[i] = cost;
    }

    vector<control_vector> goodControlVectors;
    vector<float> goodCosts;
    for(int i = 0; i < N_CONTROL_SAMPLES; i++) {
        if(costs[i] < bestCost * MAX_RELATIVE_COST) {
            goodControlVectors.push_back(controlVectors[i]);
            goodCosts.push_back(costs[i]);
        }
    }

    vector<int> localMinimaIndices;

    getLocalMinima(goodControlVectors, goodCosts, localMinimaIndices);

    // TODO implement smart path selection. For now, choose the straighest path
    int bestIndex = -1;
    float bestCurviness = 0;
    for(int i : localMinimaIndices) {
        float curviness = 0; // square of euclidean distance from zero turning
        for(auto x : goodControlVectors[i]) {
            curviness += x*x;
        }
        // cout << "i = " << i << ", curviness = " << curviness << endl;
        if(bestIndex == -1 || curviness < bestCurviness) {
            bestIndex = i;
            bestCurviness = curviness;
        }
    }

    /*ROS_INFO("Planner found %d local minima. Best cost is %.3f",
             (int)localMinimaIndices.size(), goodCosts[bestIndex]);*/

    rr_platform::speedPtr speedMSG(new rr_platform::speed);
    rr_platform::steeringPtr steerMSG(new rr_platform::steering);
    steerMSG->angle = steeringAngleAverageAgainstMedian(goodControlVectors[bestIndex][0]);
    speedMSG->speed = steeringToSpeed(steerMSG->angle, STEER_LIMITS[0]);
    steerMSG->header.stamp = ros::Time::now();
    speedMSG->header.stamp = ros::Time::now();
    speed_pub.publish(speedMSG);
    steer_pub.publish(steerMSG);

    if(path_pub.getNumSubscribers() > 0) {
        nav_msgs::Path pathMsg;
        Pose2D pose{0,0,0};
        control_vector &controlVector = goodControlVectors[bestIndex];
        for(int segmentIdx = 0; segmentIdx < N_PATH_SEGMENTS; segmentIdx++) {
            float steering = controlVector[segmentIdx];
            float speed = steeringToSpeed(steering, STEER_LIMITS[segmentIdx]);
            for(float dist = 0.0;
                dist < SEGMENT_DISTANCES[segmentIdx];
                dist += DISTANCE_INCREMENT)
            {
                advanceStep(steering, pose);
                geometry_msgs::PoseStamped ps;
                ps.pose.position.x = pose.x;
                ps.pose.position.y = pose.y;
                pathMsg.poses.push_back(ps);
            }
        }
        pathMsg.header.frame_id = "base_footprint";
        path_pub.publish(pathMsg);
    }
}

void parseFloatArrayStr(string &arrayAsString, vector<float> &floats) {
    char s[arrayAsString.size()];
    strcpy(s, arrayAsString.c_str());
    char* token = strtok(s, " ,");
    while(token != NULL) {
        floats.push_back(stof(string(token)));
        token = strtok(NULL, " ,");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");

    string obstacleCloudTopic;
    string segmentDistancesStr;
    string steerLimitsStr;
    string steerStdDevsStr;

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.param("N_PATH_SEGMENTS", N_PATH_SEGMENTS, 2);
    nhp.param("N_CONTROL_SAMPLES", N_CONTROL_SAMPLES, 200);
    nhp.param("SEGMENT_DISTANCES", segmentDistancesStr, string("3 5"));
    nhp.param("STEER_LIMITS", steerLimitsStr, string("0.4 0.4"));
    nhp.param("STEER_STDDEVS", steerStdDevsStr, string("0.2 0.2"));
    nhp.param("INPUT_CLOUD_TOPIC", obstacleCloudTopic, string("/map"));
    nhp.param("DISTANCE_INCREMENT", DISTANCE_INCREMENT, 0.2f);
    nhp.param("MAX_SPEED", MAX_SPEED, 0.5f);
    nhp.param("WHEEL_BASE", WHEEL_BASE, 0.37f); //TODO get from tf tree
    nhp.param("COLLISION_RADIUS", COLLISION_RADIUS, 0.3f);
    nhp.param("COLLISION_PENALTY", COLLISION_PENALTY, 1000.0f);
    nhp.param("PATH_SIMILARITY_CUTOFF", PATH_SIMILARITY_CUTOFF, 0.05f);
    nhp.param("MAX_RELATIVE_COST", MAX_RELATIVE_COST, 2.0f);
    nhp.param("SMOOTHING_ARRAY_SIZE", SMOOTHING_ARRAY_SIZE, 20);

    PREV_STEERING_ANGLES_INDEX = 0;
    PREV_STEERING_ANGLES.resize(SMOOTHING_ARRAY_SIZE);

    parseFloatArrayStr(segmentDistancesStr, SEGMENT_DISTANCES);
    parseFloatArrayStr(steerLimitsStr, STEER_LIMITS);
    parseFloatArrayStr(steerStdDevsStr, STEER_STDDEVS);

    for(int i = 0; i < N_PATH_SEGMENTS; i++) {
        steering_gaussians.push_back(normal_distribution<float>(0, STEER_STDDEVS[i]));
    }
    rand_gen = mt19937(std::random_device{}());

    auto map_sub = nh.subscribe(obstacleCloudTopic, 1, mapCallback);
    speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);
    path_pub = nh.advertise<nav_msgs::Path>("plan/path", 1);

    ROS_INFO("planner initialized");

    ros::spin();
    return 0;
}
