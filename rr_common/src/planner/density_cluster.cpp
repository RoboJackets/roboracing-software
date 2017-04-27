#include "density_cluster.h"

using namespace std;
using namespace path;


void cluster(const vector<WeightedSteeringVec> &weightedSteerVecs,
             vector<SteeringGroup> &outGroups,
             const float connectRadius, const int minConnections)
{
    // measure number of samples and dimensionality (# of path segments)
    int nSamples = weightedSteerVecs.size();
    int nDims = weightedSteerVecs[0].steers.size();

    // fill opencv-format sample matrix
    // (weights still available in weightedSteerVecs)
    cv::Mat samples(nSamples, nDims, CV_32FC1);
    for(int i = 0; i < nSamples; i++) {
        for(int j = 0; j < nDims; j++) {
            samples.at<float>(i, j) = weightedSteerVecs[i].steers[j];
        }
    }

    // construct FLANN index (nearest neighbors preprocessing)
    cv::flann::KDTreeIndexParams params(1);
    cv::flann::Index flannIndex(samples, params);

    // Initialize group membership list. Index corresponds to indices
    // of weightedSteerVecs.
    // 0 for unvisited, -1 for noise, positive for belonging to a group
    vector<int> groupMembership(nSamples);
    fill_n(groupMembership.begin(), nSamples, 0);

    //initialize output arrays for radius searches
    vector<int> indices(nSamples);
    fill_n(indices.begin(), nSamples, 0);
    vector<int> indicesInner(nSamples);
    fill_n(indicesInner.begin(), nSamples, 0);
    vector<float> dists(nSamples);
    fill_n(dists.begin(), nSamples, 0);

    // set radius search params
    float radius = 0.1;
    int maxResults = 100;
    int clusterId = 0;
    cv::flann::SearchParams searchParams;

    // do a DBSCAN-like clustering. Roughly following Wikipedia pseudocode
    for(int i = 0; i < nSamples; i++) {
        if(groupMembership[i] != 0) {
            continue;
        }
        const point_t &query = weightedSteerVecs[i].steers;
        int nNeighbors = flannIndex.radiusSearch(
                query, indices, dists, radius, maxResults, searchParams);

        // printf("number of radius search results is %lu", indices.size());

        if(nNeighbors < minConnections) {
            // not enough neighbors. mark as noise
            groupMembership[i] = -1;
        } else {
            // new cluster
            clusterId++;
            // Cautions: don't use iterators because indices[] is appended to in the loop
            for(int j = 0; j < nNeighbors; j++) {
                // cout << "cluster " << clusterId << ", nNeighbors = " << nNeighbors << endl;
                int &index = indices[j];
                if(groupMembership[index] == 0) {
                    groupMembership[index] = clusterId;
                    const point_t &queryInner = weightedSteerVecs[index].steers;
                    int nNeighborsInner = flannIndex.radiusSearch(queryInner, indicesInner,
                                                  dists, radius, maxResults, searchParams);
                    if(nNeighborsInner >= minConnections) {
                        for(int ii = 0; ii < nNeighborsInner; ii++)
                            indices.push_back(indicesInner[ii]);
                        nNeighbors += nNeighborsInner;
                    }
                }
            }
        }
    }

    outGroups.resize(clusterId); //clusters are numbered 1..n
    for(int i = 0; i < nSamples; i++) {
        (outGroups[groupMembership[i] - 1]).add(weightedSteerVecs[i]);
    }
}


