#include "path_structs.h"

using namespace std;

// add a set of path instructions to a locally connected group
void path::SteeringGroup::add(const WeightedSteeringVec &wsv) {
    weightTotal += wsv.weight;
    weightedSteers.push_back(wsv);
}

// merge two connected-component groups of path instructions
void path::SteeringGroup::addAll(const SteeringGroup &sg) {
    weightedSteers.insert(weightedSteers.end(), sg.weightedSteers.begin(),
                          sg.weightedSteers.end());
    weightTotal += sg.weightTotal;
}

// makes sure two connected components are compared by reference equality
bool path::SteeringGroup::operator==(SteeringGroup other) {
    // shallow equality
    return this == &other;
}

// find the weighted average steering values in a locally connected set
vector<double> path::SteeringGroup::weightedCenter() {
    vector<double> sums;
    for(int j = 0; j < weightedSteers[0].steers.size(); j++) sums.push_back(0);

    for(int i = 0; i < weightedSteers.size(); i++) {
        for(int j = 0; j < weightedSteers[i].steers.size(); j++) {
            sums[j] += weightedSteers[i].steers[j] * weightedSteers[i].weight / weightTotal;
        }
    }
    return sums;
}

double path::SteeringGroup::averageWeight() {
    return (double)weightTotal / weightedSteers.size();
}

