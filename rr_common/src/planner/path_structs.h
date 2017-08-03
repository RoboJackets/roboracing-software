#ifndef RR_COMMON_PATH_STRUCTS_H
#define RR_COMMON_PATH_STRUCTS_H

#include <vector>

typedef std::vector<float> control_vector;

namespace path
{
    struct Pose2D {
        float x;
        float y;
        float theta;
    };

    struct Path {
        std::vector<Pose2D> poses;
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
        point_t weightedCenter();
        float averageWeight();
        bool operator==(SteeringGroup other);
    };
}

#endif //RR_COMMON_PATH_STRUCTS_H
