#ifndef RR_COMMON_PATH_STRUCTS_H
#define RR_COMMON_PATH_STRUCTS_H

#include <vector>

typedef std::vector<float> point_t;

namespace path
{
    struct pose {
        float x;
        float y;
        float theta;
    };

    struct path {
        std::vector<pose> poses;
        std::vector<float> speeds;
    };

    struct WeightedSteeringVec {
        point_t steers;
        float weight;
    };

    struct SteeringGroup {
        std::vector<WeightedSteeringVec> weightedSteers;
        float weightTotal;

        void add(const WeightedSteeringVec &wsv);
        void addAll(const SteeringGroup &sg);
        point_t weightedCenter();
        float averageWeight();
        bool operator==(SteeringGroup other);
    };
}

#endif //RR_COMMON_PATH_STRUCTS_H
