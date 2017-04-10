#ifndef RR_COMMON_DENSITY_CLUSTER_H
#define RR_COMMON_DENSITY_CLUSTER_H

#include <cmath>

#include "path_structs.h"

void cluster(const std::vector<path::WeightedSteeringVec> &weightedSteerVecsFiltered,
             std::vector<path::SteeringGroup> &outGroups, const double connectRadius,
             const int minConnections);
double distance(std::vector<double> vec1, std::vector<double> vec2);

#endif //RR_COMMON_DENSITY_CLUSTER_H
