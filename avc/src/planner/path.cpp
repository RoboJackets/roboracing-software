#include <vector>
#include <opencv2/opencv.hpp>

class path {
	std::vector<cv::Point> points;
public:
	path() {

	}
	path(std::vector<cv::Point> points) {
		this->points = points;
	}
	void pushBack(cv::Point p) {
		points.push_back(p);
	}
	path getPoints(path p) {
		return points;
	}

};
