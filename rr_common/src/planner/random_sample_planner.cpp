#include "random_sample_planner.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <flann/flann.hpp>

double last_speed = 0;

namespace rr {

RandomSamplePlanner::RandomSamplePlanner(const Params& params_ext) : params(params_ext) {
  prev_steering_angles_.resize(params.smoothing_array_size, 0);
  prev_steering_angles_index_ = 0;

  for (double d : params.steer_stddevs) {
    steering_gaussians_.emplace_back(0, d);
  }
  rand_gen_ = std::mt19937(std::random_device{}());
}

Pose RandomSamplePlanner::StepKinematics(const Pose& pose, double steer_angle) {
  double deltaX, deltaY, deltaTheta;

  if (abs(steer_angle) < 1e-3) {
    deltaX = params.distance_increment;
    deltaY = 0;
    deltaTheta = 0;
  } else {
    double turnRadius = params.wheel_base / sin(abs(steer_angle));
    double tempTheta = params.distance_increment / turnRadius;
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



double RandomSamplePlanner::SampleSteering(int stage) {
  double angle;
  do {
    auto& gaussian = steering_gaussians_[stage];
    angle = gaussian(rand_gen_);
  } while (std::abs(angle) > params.steer_limits[stage]);

  return angle;
}

double RandomSamplePlanner::SteeringToSpeed(double steer_angle) {
  steer_angle = std::abs(steer_angle);

  double out;
  if (steer_angle < 1e-3) {
    out = params.max_speed;
  } else {
    double vRaw = std::sqrt(params.lateral_accel * params.wheel_base / std::sin(steer_angle));
    out = std::min(vRaw, params.max_speed);
  }
  return out;
}

std::vector<PathPoint> RandomSamplePlanner::RollOutPath(const std::vector<double>& control) {
  if(control.size() != params.n_path_segments) {
    std::cout << "[Planner] Warning: control vector of dimension " << control.size() 
              << " does not match " << params.n_path_segments << " path segments" << std::endl;
  }

  std::vector<PathPoint> path_points;
  path_points.emplace_back(Pose{0, 0, 0}, control[0], SteeringToSpeed(control[0]));

  for (int segment = 0; segment < params.n_path_segments; segment++) {
    double steer = control[segment];
    double speed = SteeringToSpeed(steer);
    const double& seg_dist = params.segment_distances[segment];
    for (double dist = 0.0; dist < seg_dist; dist += params.distance_increment) {
      const Pose& last_pose = path_points.back().pose;
      path_points.emplace_back(StepKinematics(last_pose, steer), steer, SteeringToSpeed(steer));
    }
  }

  return path_points;
}

std::tuple<bool, double> RandomSamplePlanner::GetCost(const std::vector<PathPoint>& path,
                                          const KdTreeMap& kd_tree_map) {
  double denominator = 0;
  bool is_collision;
  double dist;
  for(const auto& path_point : path) {
    std::tie(is_collision, dist) = GetCollisionDistance(path_point.pose, kd_tree_map);
    if (is_collision) {
      break;
    }
    denominator += params.k_dist * dist + params.k_speed * path_point.speed;
  }
  return std::make_tuple(is_collision, 1.0 / denominator);
}

std::vector<int> RandomSamplePlanner::GetLocalMinima(const std::vector<PlannedPath>& plans,
                                         const std::vector<bool>& mask) {
  // std::cout << "start GetLocalMinima" << std::endl;

  std::vector<int> local_minima_indices;
  int n_samples = count(mask.begin(), mask.end(), true);

  if (n_samples > 0) {
    // fill flann-format sample matrix
    double sampleArray[n_samples * params.n_path_segments];
    for(int i = 0; i < n_samples; i++) {
      for(int j = 0; j < params.n_path_segments; j++) {
        sampleArray[i * params.n_path_segments + j] = plans[i].get().control[j];
        // std::cout << "sampleArray[" << i * params.n_path_segments + j << "] = " 
        //           << plans[i].get().control[j] << std::endl;
      }
    }
    flann::Matrix<double> samples(sampleArray, n_samples, params.n_path_segments);

    // construct FLANN index (nearest neighbors preprocessing)
    // TODO determine if L1 (Manhattan) or L2 (Euclidean) distance is more useful here
    flann::Index<flann::L1<double>> flannIndex(samples, flann::KDTreeSingleIndexParams());
    flannIndex.buildIndex();

    // Initialize group membership list
    std::vector<bool> samples_known_status(n_samples, false);

    // initialize input/output parameters for radius searches
    flann::Matrix<int> indices(new int[n_samples], 1, n_samples);
    flann::Matrix<double> dists(new double[n_samples], 1, n_samples);
    flann::Matrix<double> query(new double[params.n_path_segments], 1, params.n_path_segments);
    flann::SearchParams searchParams(1);
    const double& searchRadius = params.path_similarity_cutoff;

    for (int i = 0; i < n_samples; i++) {
      if (samples_known_status[i]) {
        continue;
      }

      // perform radius search
      for(int d = 0; d < params.n_path_segments; d++) {
        query[0][d] = samples[i][d];
      }
      int n_neighbors = flannIndex.radiusSearch(query, indices, dists,
                                               searchRadius, searchParams);

      // Iterate through neighbors, tracking if lower costs exist.
      // Any neighbors in radius that have higher costs are not local minima.
      // i is the query, j is a neighbor
      bool any_better_in_area = false;
      for (int j = 0; j < n_neighbors; j++) {
        int found_index = indices[0][j];

        if (found_index == i) {  //handle index of query later
          continue;
        }

        if (plans[found_index].get().cost < plans[i].get().cost) {
          any_better_in_area = true;
        } else {
          samples_known_status[found_index] = true;  // known not minimum
        }
      }

      samples_known_status[i] = true;
      if(!any_better_in_area) {
        // includes regions with only one point
        local_minima_indices.push_back(i);
      }
    }
  }

  // std::cout << "end GetLocalMinima" << std::endl;
  return local_minima_indices;
}

double RandomSamplePlanner::FilterOutput(double this_steer) {
  // update circular buffer
  prev_steering_angles_[prev_steering_angles_index_] = this_steer;
  prev_steering_angles_index_ = (prev_steering_angles_index_ + 1) % params.smoothing_array_size;

  // sort array of previous vectors in ascending order from start to midpoint to find median
  auto angles = prev_steering_angles_;  // make a fresh copy
  int median_index = params.smoothing_array_size / 2;
  std::nth_element(angles.begin(), angles.begin() + median_index, angles.end());
  const double& median = angles[median_index];

  return median;
}

PlannedPath RandomSamplePlanner::Plan(const KdTreeMap& kd_tree_map) {

  // allocate planned paths
  std::vector<PlannedPath> plans;

  // fill planned paths and determine best cost
  double best_cost = 1e10;
  for (int i = 0; i < params.n_control_samples; i++) {
    plans.emplace_back();
    auto& plan = plans.back();

    for (int stage = 0; stage < params.n_path_segments; stage++) {
      plan.control.push_back(SampleSteering(stage));
    }

    plan.path = RollOutPath(plan.control);

    bool collision;
    std::tie(collision, plan.cost) = GetCost(plan.path, kd_tree_map);

    if (collision) {
      plans.pop_back();
      continue;
    }

    best_cost = std::min(best_cost, plan.cost);
  }

  std::vector<std::reference_wrapper<PlannedPath>> good_plans;

  // filter by cost (compared to best cost)
  for (auto& plan : plans) {
    if (plan.cost <= best_cost * params.max_relative_cost) {
      good_plans.push_back(std::ref(plan));
    }
  }

  if (good_plans.size() < 2) {
    std::cout << "[Planner] Warning: found " << good_plans.size() << " good paths" << std::endl;
  }

  PlannedPath fallback_plan;
  double backwards_steer = -params.steer_limits[0] * 0.3;
  fallback_plan.control = {backwards_steer, backwards_steer};
  fallback_plan.path = RollOutPath(fallback_plan.control);
  for (auto& path_point : fallback_plan.path) {
    path_point.speed = -0.3;
  }
  fallback_plan.cost = 0;

  if (good_plans.empty()) {
    std::cout << "[Planner] Using fallback plan" << std::endl;
    good_plans.push_back(std::ref(fallback_plan));
  }

  auto local_minima_indices = GetLocalMinima(good_plans);

  // TODO implement smarter path selection. For now, choose the straighest path
  int best_index = -1;
  double best_curviness = 0;
  for (int i : local_minima_indices) {
    double curviness = 0; // square of euclidean distance from zero turning
    for (auto x : good_plans[i].get().control) {
      curviness += x*x;
    }

    // cout << "i = " << i << ", curviness = " << curviness << endl;
    if(best_index == -1 || curviness < best_curviness) {
      best_index = i;
      best_curviness = curviness;
    }
  }

  PlannedPath& best_plan = good_plans[best_index];

  bool is_collision;
  double dist;
  double min_dist = 10000;
  for (const auto& path_point : best_plan.path) {
    std::tie(is_collision, dist) = GetCollisionDistance(path_point.pose, kd_tree_map);
    if (dist < min_dist) {
      min_dist = dist;
    }
  }

  if (min_dist < params.obs_dist_slow_thresh) {
    best_plan.path[0].speed *= params.obs_dist_slow_ratio;
  }

  double this_speed = best_plan.path[0].speed;

  if (last_speed > 0 && this_speed < 0) {
    best_plan.path[0].speed = last_speed;
  }

  last_speed = this_speed;

  std::cout << "Planner found " << local_minima_indices.size() << " local minima. "
            << "Best cost is " << best_plan.cost << std::endl;

  return best_plan;
}

}  // namespace rr
