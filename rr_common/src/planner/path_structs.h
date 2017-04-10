#ifndef RR_COMMON_PATH_STRUCTS_H
#define RR_COMMON_PATH_STRUCTS_H

#include <vector>

namespace path
{
    struct pose {
        double x;
        double y;
        double theta;
    };

    struct path {
        std::vector<pose> poses;
        std::vector<double> speeds;
    };

    struct WeightedSteeringVec {
        std::vector<double> steers;
        double weight;
    };

    struct SteeringGroup {
        std::vector<WeightedSteeringVec> weightedSteers;
        double weightTotal;

        void add(const WeightedSteeringVec &wsv);
        void addAll(const SteeringGroup &sg);
        std::vector<double> weightedCenter();
        double averageWeight();
        bool operator==(SteeringGroup other);
    };
}

#endif //RR_COMMON_PATH_STRUCTS_H
