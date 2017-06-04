#ifndef RR_COMMON_DENSITY_CLUSTER_H
#define RR_COMMON_DENSITY_CLUSTER_H

#include <cmath>
#include <iostream>
#include "flann/flann.hpp"

#include "path_structs.h"

const int POINT_UNDISCOVERED = 0;
const int POINT_NOISE = -1;
const int POINT_DISCOVERED = -2;

void cluster(const std::vector<path::WeightedSteeringVec> &weightedSteerVecsFiltered,
             std::vector<path::SteeringGroup> &outGroups, const float connectRadius,
             const int minConnections);

#endif //RR_COMMON_DENSITY_CLUSTER_H
