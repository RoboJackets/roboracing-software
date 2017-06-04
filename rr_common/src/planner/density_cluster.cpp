#include "density_cluster.h"

using namespace std;
using namespace path;


//return n-dimensional euclidean distance. Used for connecting steering set vectors
static float distance(point_t vec1, point_t vec2) {
    float sum = 0;
    for(int i = 0; i < vec1.size(); i++) {
        sum += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
    }
    return sqrt(sum);
}


void cluster(const vector<WeightedSteeringVec> &weightedSteerVecs,
         vector<SteeringGroup> &outGroups,
         const float connectRadius, const int minConnections)
{
    // measure number of samples and dimensionality (# of path segments)
    int nSamples = weightedSteerVecs.size();
    int nDims = weightedSteerVecs[0].steers.size();

    // fill flann-format sample matrix
    // (weights still available in weightedSteerVecs)
    float sampleArray[nSamples * nDims];
    for(int i = 0; i < nSamples; i++) {
        for(int j = 0; j < nDims; j++) {
            sampleArray[i * nDims + j] = weightedSteerVecs[i].steers[j];
        }
    }
    flann::Matrix<float> samples(sampleArray, nSamples, nDims);

    // construct FLANN index (nearest neighbors preprocessing)
    flann::Index<flann::L2<float>> flannIndex(samples, flann::KDTreeIndexParams(1));
    flannIndex.buildIndex();

    // Initialize group membership list. Index corresponds to indices
    // of weightedSteerVecs.
    vector<int> groupMembership(nSamples);
    fill_n(groupMembership.begin(), nSamples, POINT_UNDISCOVERED);

    // set radius search params
    float radiusSqr = connectRadius * connectRadius;
    int clusterId = 0;
    flann::SearchParams searchParams;

    // initialize output arrays for radius searches
    flann::Matrix<int> indices(new int[nSamples], 1, nSamples);
    flann::Matrix<int> indicesInner(new int[nSamples], 1, nSamples);
    flann::Matrix<float> dists(new float[nSamples], 1, nSamples);
    flann::Matrix<float> query(new float[nDims], 1, nDims);

    vector<int> clusterCheckQueue;

    // do a DBSCAN-like clustering. Roughly following Wikipedia pseudocode
    for(int i = 0; i < nSamples; i++) {
        if(groupMembership[i] != POINT_UNDISCOVERED) {
            continue;
        }

        // copy data into query matrix
        for(int d = 0; d < nDims; d++) {
            query[0][d] = samples[i][d];
        }

        // do the initial radius search
        flannIndex.radiusSearch(query, indices, dists, radiusSqr, searchParams);

        // count the results. The indices list is terminated with -1
        int nNeighbors = 0;
        while(nNeighbors < nSamples && indices[0][nNeighbors] != -1) {
            nNeighbors++;
        }
        // cout << "number of radius search results is " << nNeighbors << endl;

        if(nNeighbors < minConnections) {
            // not enough neighbors. Mark as noise
            groupMembership[i] = POINT_NOISE;
            continue;
        }

        // we have a new cluster
        clusterId++;

        // transfer nearby points to cluster queue
        clusterCheckQueue.clear();
        for(int j = 0; j < nNeighbors; j++) {
            clusterCheckQueue.push_back(indices[0][j]);
        }

        /*
         * Iterate through the queue of points, doing a radius search from each.
         * Any undiscovered/unvisited results of these searches are added to the
         * queue. This will find all points in a cluster.
         */
        for(int j = 0; j < clusterCheckQueue.size(); j++) {
            int index = clusterCheckQueue[j];

            // discovered or undiscovered, as opposed to noise or a group ID
            bool exploreThis = groupMembership[index] == POINT_UNDISCOVERED
                            || groupMembership[index] == POINT_DISCOVERED;
            if(!exploreThis) {
                continue;
            }

            groupMembership[index] = clusterId;

            // copy data into query matrix
            for(int d = 0; d < nDims; d++) {
                query[0][d] = samples[index][d];
            }

            flannIndex.radiusSearch(query, indicesInner, dists, radiusSqr, searchParams);

            int nNeighborsInner = 0;
            while(nNeighborsInner < nSamples && indicesInner[0][nNeighborsInner] != -1) {
                nNeighborsInner++;
            }

            if(nNeighborsInner >= minConnections) {
                for(int ii = 0; ii < nNeighborsInner; ii++) {
                    int indexInner = indicesInner[0][ii];
                    if(groupMembership[indexInner] == POINT_UNDISCOVERED) {
                        // fprintf(stderr, "discovered index %d, queue size = %lu\n", indexInner, clusterCheckQueue.size());
                        groupMembership[indexInner] = POINT_DISCOVERED;
                        clusterCheckQueue.push_back(indexInner);
                    }
                }
            }

        }
    }

    outGroups.resize(clusterId); //clusters are numbered 1..n
    for(int i = 0; i < nSamples; i++) {
        if(groupMembership[i] > 0) {
            (outGroups[groupMembership[i] - 1]).add(weightedSteerVecs[i]);
        }
    }
}


