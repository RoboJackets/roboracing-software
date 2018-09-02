#include "planner.h"
#include <algorithm>
#include <cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PoseStamped.h>
#include "flann/flann.hpp"

namespace rr {
namespace planning {

Planner::Planner(const Params& params_ext) : params(params_ext) {
  prev_steering_angles_.resize(params.smoothing_array_size, 0);
  prev_steering_angles_index_ = 0;

  for(double d : params.steer_stddevs) {
    steering_gaussians_.emplace_back(0, d);
  }
  rand_gen_ = std::mt19937(std::random_device{}());
}

Pose Planner::StepKinematics(const Pose &pose, double steer_angle) {
  float deltaX, deltaY, deltaTheta;

  if (abs(steer_angle) < 1e-3) {
    deltaX = params.distance_increment;
    deltaY = 0;
    deltaTheta = 0;
  } else {
    float turnRadius = params.wheel_base / sin(abs(steer_angle));
    float tempTheta = params.distance_increment / turnRadius;
    deltaX = turnRadius * cos(M_PI / 2 - tempTheta);
    if (steer_angle < 0) {
        deltaY = turnRadius - turnRadius * sin(M_PI / 2 - tempTheta);
    } else {
        deltaY = -(turnRadius - turnRadius * sin(M_PI / 2 - tempTheta));
    }
    deltaTheta = params.distance_increment / params.wheel_base * sin(-steer_angle);
  }

  Pose out;
  out.x = pose.x + deltaX * cos(pose.theta) - deltaY * sin(pose.theta);
  out.y = pose.y + deltaX * sin(pose.theta) + deltaY * cos(pose.theta);
  out.theta = pose.theta + deltaTheta;
  return out;
}

std::tuple<bool, double> Plannner::GetCollisionDistance(const Pose &pose, 
                                                        const KdTreeMap &kd_tree_map) {
  double robotCenterX = (params.robot_collision.lengthFront - params.robot_collision.lengthBack) / 2.;
  double robotCenterY = (params.robot_collision.widthLeft - params.robot_collision.widthRight) / 2.;
  double searchX = pose.x + robotCenterX * cos(pose.theta);
  double searchY = pose.y + robotCenterY * sin(pose.theta);

  double halfLength = (params.robot_collision.lengthFront + params.robot_collision.lengthBack) / 2.;
  double halfWidth = (params.robot_collision.lengthFront + params.robot_collision.lengthBack) / 2.;
  double farthestPointOnRobot = sqrt(halfLength*halfLength + halfWidth*halfWidth);

  pcl::PointXYZ searchPoint(searchX, searchY, 0);
  vector<int> pointIdxs;
  vector<double> squaredDistances;
  int nResults = kd_tree_map.radiusSearch(searchPoint, OBSTACLE_SEARCH_RADIUS, pointIdxs,
                                          squaredDistances);
  // ROS_INFO_STREAM("nResults = " << nResults);

  if (nResults == 0) { //no obstacles in sight
    return 0;
  } else {
    const double cosTheta = cos(pose.theta);
    const double sinTheta = sin(pose.theta);

    // convert each point to robot reference frame and check collision and distances
    double minDist = 1000.0;
    for (int i : pointIdxs) {
      const auto& point = kdtree.getInputCloud()->at(i);
      const double tmpX = point.x - pose.x - robotCenterX;
      const double tmpY = point.y - pose.y - robotCenterY;
      const double x = cosTheta * tmpX - sinTheta * tmpY - robotCenterX;
      const double y = sinTheta * tmpX + cosTheta * tmpY - robotCenterY;
      // ROS_INFO("pose %.3f, %.3f, %.3f  before: %.3f, %.3f  after: %.3f, %.3f", pose.x, pose.y,
      //          pose.theta, point.x, point.y, x, y);

      // collisions
      if (abs(x) <= halfLength && abs(y) <= halfWidth) {
        return -1;
      }

      // find distance, in several cases
      double dist;
      if (abs(x) > halfLength) {
        // not alongside the robot
        if (abs(y) > halfWidth) {
          // closest to a corner
          double cornerX = halfLength * ((x < 0) ? -1 : 1);
          double cornerY = halfWidth * ((y < 0) ? -1 : 1);
          double dx = x - cornerX;
          double dy = y - cornerY;
          dist = sqrt(dx*dx + dy*dy);
        } else {
          // directly in front of or behind robot
          dist = abs(x) - halfLength;
        }
      } else {
        // directly to the side of the robot
        dist = abs(y) - halfWidth;
      }
      minDist = min(minDist, dist);
    }

    // ROS_INFO_STREAM("minDist = " << minDist);
    return minDist;
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
double Planner::SteeringToSpeed(double steer_angle);
    return params.max_speed * cos(steer_angle * 1.2 / maxSteering);
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

            float obsDist = distAtPose(pose, kdtree);
            if (obsDist < 0) {
                costSum += COLLISION_PENALTY;
            } else {
                double expectedDist = 2.0;
                costSum += 1.0 / (0.5 * obsDist / expectedDist + 2 * speed / MAX_SPEED);
            }
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

    // remove points that cause a collision at t=0
    for(auto iter = cloud->begin(); iter != cloud->end();) {
        auto& point = *iter;
        auto distance = std::sqrt((point.x*point.x) + (point.y*point.y));
        if(distance < COLLISION_RADIUS) {
            iter = cloud->erase(iter);
        } else {
            iter++;
        }
    }

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

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree(false);
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

    // ROS_INFO_STREAM("min cost: " << *min_element(costs.begin(), costs.end()));

    vector<control_vector> goodControlVectors;
    vector<float> goodCosts;
    for(int i = 0; i < N_CONTROL_SAMPLES; i++) {
        if(costs[i] <= bestCost * MAX_RELATIVE_COST) {
            goodControlVectors.push_back(controlVectors[i]);
            goodCosts.push_back(costs[i]);
        }
    }

    if (goodControlVectors.size() == 0) {
        ROS_WARN("Planner: no good control vectors found");
        return;
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

    ROS_INFO("Planner found %d local minima. Best cost is %.3f",
             (int)localMinimaIndices.size(), goodCosts[bestIndex]);

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



    PREV_STEERING_ANGLES_INDEX = 0;
    PREV_STEERING_ANGLES.resize(SMOOTHING_ARRAY_SIZE);

    parseFloatArrayStr(segmentDistancesStr, SEGMENT_DISTANCES);
    parseFloatArrayStr(steerLimitsStr, STEER_LIMITS);
    parseFloatArrayStr(steerStdDevsStr, STEER_STDDEVS);

    for(int i = 0; i < N_PATH_SEGMENTS; i++) {
        steering_gaussians.push_back(normal_distribution<float>(0, STEER_STDDEVS[i]));
    }
    rand_gen = mt19937(std::random_device{}());

}
