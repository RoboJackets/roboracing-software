#include "density_cluster.h"

using namespace std;
using namespace path;

//return n-dimensional euclidean distance. Used for connecting steering set vectors
double distance(vector<double> vec1, vector<double> vec2) {
    double sum = 0;
    for(int i = 0; i < vec1.size(); i++) {
        sum += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
    }
    return sqrt(sum);
}



void cluster(const vector<WeightedSteeringVec> &weightedSteerVecsFiltered,
             vector<SteeringGroup> &outGroups, const double connectRadius,
             const int minConnections) {

    /* connect components based on distance. I refer to any random
     * path sample as a steerVec here. The variable groups holds the
     * sets of connected (connected = "close enough") paths
     */
    // outer loop: iterate through good enough paths
    for(const WeightedSteeringVec &newSteerVec : weightedSteerVecsFiltered) {
        bool foundMatch = false;
        int newGroupIndex;
        // inner loop 1: iterate through existing groups
        for(int comparisonGroupIndex = 0; comparisonGroupIndex < outGroups.size(); comparisonGroupIndex++) {
            SteeringGroup &comparisonGroup = outGroups[comparisonGroupIndex];
            const vector<WeightedSteeringVec> &comparisonSteers = comparisonGroup.weightedSteers;
            for(int j = 0; j < comparisonSteers.size(); j++) {
                // thatSteerVec belongs to thatGroup
                const WeightedSteeringVec &comparisonSteerVec = comparisonSteers[j];
                if (distance(newSteerVec.steers, comparisonSteerVec.steers) < connectRadius) {
                    if (foundMatch) {
                        // has already found a match for thisSteerVec. Merge groups
                        outGroups[newGroupIndex].addAll(comparisonGroup);
                        // delete the group at the current "other" group index
                        outGroups.erase(outGroups.begin() + comparisonGroupIndex);
                        comparisonGroupIndex--;
                    } else {
                        // no match has previously been found for this steering sample
                        comparisonGroup.add(newSteerVec);
                        newGroupIndex = comparisonGroupIndex;
                    }
                    foundMatch = true;
                    break; //end looking at this group
                }
            }
        }

        if(!foundMatch) {
            // no existing groups are close enough to the path sample
            path::SteeringGroup sg;
            sg.add(newSteerVec);
            outGroups.push_back(sg);
        }
    }
}


