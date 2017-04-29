#include "flann/flann.hpp"
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
    float data[] = {
            1.0, 0.0,
            0.0, 1.0,
            0.5, 0.5,
            1.1, 0.5,
            0.1, 4,
            0.0, 0.0,
            4,5,
            6,7,
            8,4
    };
    int nSamples = 9;
    int nDims = 2;

    flann::Matrix<float> samples(data, nSamples, nDims);

    for(int i = 0; i < samples.rows; i++) {
        cout << samples[i][0] << ", " << samples[i][1] << endl;
    }

    flann::Index<flann::L2<float>> flannIndex(samples, flann::KDTreeIndexParams(1));
    flannIndex.buildIndex();

    flann::Matrix<int> indices(new int[nSamples], 1, nSamples);
    flann::Matrix<float> dists(new float[nSamples], 1, nSamples);
    flann::SearchParams searchParams;

    float queryArr[] = {0.4, 0.4};
    flann::Matrix<float> query(queryArr, 1, nDims);

    // note: list of return values has -1 as terminating value
    flannIndex.radiusSearch(query, indices, dists, 2.0, searchParams);

    for(int i = 0; i < nSamples; i++) {
        cout << indices[0][i] << endl;
    }
}