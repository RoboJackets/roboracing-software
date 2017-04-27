#ifndef RR_COMMON_DENSITY_CLUSTER_H
#define RR_COMMON_DENSITY_CLUSTER_H

#include <cmath>
#include <iostream>
#include "opencv2/flann/miniflann.hpp"

#include "path_structs.h"

void cluster(const std::vector<path::WeightedSteeringVec> &weightedSteerVecsFiltered,
             std::vector<path::SteeringGroup> &outGroups, const float connectRadius,
             const int minConnections);

#endif //RR_COMMON_DENSITY_CLUSTER_H
