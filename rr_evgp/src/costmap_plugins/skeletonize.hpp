#include <opencv2/opencv.hpp>

namespace rr {

cv::Mat thinObstacles(const cv::Mat& grid_in) {
    std::vector<cv::Point> points;
    cv::findNonZero(grid_in, points);
    cv::Rect r = cv::boundingRect(points);

    // Zhang thinning "A Fast Parallel Algorithm for Thinning Digital Patterns"
    // ref https://rosettacode.org/wiki/Zhang-Suen_thinning_algorithm#C.2B.2B
    cv::Mat grid, grid_copy;
    grid_in(r).copyTo(grid);

    // eliminate border
    for (int r = 0; r < grid.rows; r++) {
        grid.at<uint8_t>(r, 0) = 0;
        grid.at<uint8_t>(r, grid.cols - 1) = 0;
    }
    for (int c = 0; c < grid.cols; c++) {
        grid.at<uint8_t>(0, c) = 0;
        grid.at<uint8_t>(grid.rows - 1, c) = 0;
    }

    grid.copyTo(grid_copy);

    /*
     * Count the number of rising edges around the center point
     *  9 2 3
     *  8 * 4
     *  7 6 5
     */
    auto clockwise_rising_edges = [](const cv::Mat& data, int r, int c) -> int {
        int sum = 0;
        sum += !data.at<uint8_t>(r - 1, c) && data.at<uint8_t>(r - 1, c + 1);  // 2 3
        sum += !data.at<uint8_t>(r - 1, c + 1) && data.at<uint8_t>(r, c + 1);  // 3 4
        sum += !data.at<uint8_t>(r, c + 1) && data.at<uint8_t>(r + 1, c + 1);  // 4 5
        sum += !data.at<uint8_t>(r + 1, c + 1) && data.at<uint8_t>(r + 1, c);  // 5 6
        sum += !data.at<uint8_t>(r + 1, c) && data.at<uint8_t>(r + 1, c - 1);  // 6 7
        sum += !data.at<uint8_t>(r + 1, c - 1) && data.at<uint8_t>(r, c - 1);  // 7 8
        sum += !data.at<uint8_t>(r, c - 1) && data.at<uint8_t>(r - 1, c - 1);  // 8 9
        sum += !data.at<uint8_t>(r - 1, c - 1) && data.at<uint8_t>(r - 1, c);  // 9 2
        return sum;
    };

    auto num_neighbors = [](const cv::Mat& data, int r, int c) -> int {
        int sum = 0;
        sum += (bool)data.at<uint8_t>(r - 1, c);
        sum += (bool)data.at<uint8_t>(r - 1, c + 1);
        sum += (bool)data.at<uint8_t>(r, c + 1);
        sum += (bool)data.at<uint8_t>(r + 1, c + 1);
        sum += (bool)data.at<uint8_t>(r + 1, c);
        sum += (bool)data.at<uint8_t>(r + 1, c - 1);
        sum += (bool)data.at<uint8_t>(r, c - 1);
        sum += (bool)data.at<uint8_t>(r - 1, c - 1);
        return sum;
    };

    int changed_pixels;
    do {
        changed_pixels = 0;

        // step 1
        grid.copyTo(grid_copy);
        for (int r = 1; r < grid.rows - 1; r++) {
            for (int c = 1; c < grid.cols - 1; c++) {
                if (grid.at<uint8_t>(r, c)) {
                    int n = num_neighbors(grid, r, c);
                    if (n >= 2 && n <= 6 && clockwise_rising_edges(grid, r, c) == 1 &&
                        !(grid.at<uint8_t>(r - 1, c) && grid.at<uint8_t>(r, c + 1) && grid.at<uint8_t>(r + 1, c)) &&
                        !(grid.at<uint8_t>(r, c + 1) && grid.at<uint8_t>(r + 1, c) && grid.at<uint8_t>(r, c - 1))) {
                        grid_copy.at<uint8_t>(r, c) = 0;
                        changed_pixels++;
                    }
                }
            }
        }

        // step 2
        grid_copy.copyTo(grid);
        for (int r = 1; r < grid.rows - 1; r++) {
            for (int c = 1; c < grid.cols - 1; c++) {
                if (grid_copy.at<uint8_t>(r, c)) {
                    int n = num_neighbors(grid_copy, r, c);
                    if (n >= 2 && n <= 6 && clockwise_rising_edges(grid_copy, r, c) == 1 &&
                        !(grid_copy.at<uint8_t>(r - 1, c) && grid_copy.at<uint8_t>(r, c + 1) &&
                          grid_copy.at<uint8_t>(r, c - 1)) &&
                        !(grid_copy.at<uint8_t>(r - 1, c) && grid_copy.at<uint8_t>(r + 1, c) &&
                          grid_copy.at<uint8_t>(r, c - 1))) {
                        grid.at<uint8_t>(r, c) = 0;
                        changed_pixels++;
                    }
                }
            }
        }
    } while (changed_pixels > 0);

    // remove staircases
    // shamelessly borrowed from https://github.com/yati-sagade/zhang-suen-thinning/blob/master/zhangsuen.cpp
    for (int iter = 0; iter < 2; iter++) {
        for (int i = 1; i < grid.rows - 1; i++) {
            for (int j = 1; j < grid.cols - 1; j++) {
                int c = grid.at<uint8_t>(i, j);
                if (!c) {
                    continue;
                }
                int e = grid.at<uint8_t>(i, j + 1), ne = grid.at<uint8_t>(i - 1, j + 1), n = grid.at<uint8_t>(i - 1, j),
                    nw = grid.at<uint8_t>(i - 1, j - 1), w = grid.at<uint8_t>(i, j - 1),
                    sw = grid.at<uint8_t>(i + 1, j - 1), s = grid.at<uint8_t>(i + 1, j),
                    se = grid.at<uint8_t>(i + 1, j + 1);

                if (iter == 0) {
                    // North biased staircase removal
                    if ((n && ((e && !ne && !sw && (!w || !s)) || (w && !nw && !se && (!e || !s))))) {
                        grid.at<uint8_t>(i, j) = 0;
                    }
                } else {
                    // South bias staircase removal
                    if ((s && ((e && !se && !nw && (!w || !n)) || (w && !sw && !ne && (!e || !n))))) {
                        grid.at<uint8_t>(i, j) = 0;
                    }
                }
            }
        }
    }

    cv::Mat grid_skel(grid_in.size(), CV_8U, cv::Scalar(0));
    grid.copyTo(grid_skel(r));
    return grid_skel;
}

cv::Mat removeSmallBranches(const cv::Mat& img, const int min_branch_length) {
    cv::Mat line_kernel(3, 3, CV_8U, cv::Scalar(1));
    line_kernel.at<uint8_t>(1, 1) = 10;

    cv::Mat filtered_img;
    cv::filter2D(img / 255, filtered_img, CV_16S, line_kernel);

    cv::Mat branches;
    cv::inRange(filtered_img, cv::Scalar(11), cv::Scalar(12), branches);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(branches, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    cv::Mat big_branches(img.rows, img.cols, CV_8U, cv::Scalar(0));
    for (size_t i = 0; i < contours.size(); i++) {
        if (cv::arcLength(contours[i], false) >= min_branch_length) {
            cv::drawContours(big_branches, contours, i, cv::Scalar(255), 1);
        }
    }

    big_branches.setTo(cv::Scalar(255), filtered_img >= 13);
    thinObstacles(big_branches);

    return big_branches;
}

}  // namespace rr
