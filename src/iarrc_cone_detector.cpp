#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
std::string img_file;

using namespace cv;
using namespace std;

struct loclength {
  float length; //real life how wide
  int size; //how many points wide
} ; //ok i dont like struct lol :(

struct pointloc {
    float x;
    float y;
};

ros::Publisher cone_publisher;
ros::Publisher point_publisher;

double cone_max, cone_min;

void ConeDetectorCB(const sensor_msgs::LaserScan::ConstPtr& msg) {
	// cout << *msg << endl;
	float angle = msg->angle_increment;
    float anglemin = msg->angle_min;
	vector<float> eps;

	int size = msg->ranges.size();
	float d;
	for (int i=0; i<size; i++){
        if (i==(size-1)){
            d = 0;
        } else {
            d = msg->ranges[i]*msg->ranges[i]+msg->ranges[i+1]*msg->ranges[i+1]-2.0*msg->ranges[i]*msg->ranges[i+1]*cos(angle); //finding length squared between point i and i+1 using cosine rule
            d = sqrt(d); //square root to get the length between point i and i+1
        }
        eps.push_back(d);
        //cout << "epsilon[" << i << "]: " << d << endl;
	}

    vector<loclength> l;
    int mid = 0;
    int objsize = 0;
    int loc = 0;
    float length = 0;
    loclength current;
    current.length = 0;
    current.size = 0;


    for (int i=0; i<size; i++){
        l.push_back(current);
        if (i==(objsize-1)){
            mid = (i+loc)/2;
            current.length = length;
            if (objsize>1){
                objsize++;
            } else {
                objsize = 0;
            }
            current.size = objsize;
            l[loc] = current;
        } else if (eps[i] > 0.1){ //detect jumps
            mid = (i+loc)/2;
            current.length = length;
            if (objsize>1){
                objsize++;
            } else {
                objsize = 0;
            }
            current.size = objsize;
            l[loc] = current;
            current.length = 0;
            current.size = 0;
            loc = i;
            length = 0;
            objsize = 1;
        } else if (eps[i] ==0){
            current.length = 0;
            current.size = 0;
            objsize = 0;
        } else { //add lengths
            length = length + eps[i];
            objsize++;
        }
    }

    int npoint = 0;
    float myangle = 0;
    float mylength = 0;
    pointloc mypointloc;
    mypointloc.x = 0;
    mypointloc.y = 0;
    vector<pointloc> pointlocs;

    int num_clusters = 0;

    for (int i=0; i<size; i++){
        if (l[i].size>0){

	        num_clusters++;

            npoint = (i+1)+(l[i].size)/2;
            myangle = anglemin + npoint*angle;
            mylength = msg->ranges[npoint];
            mypointloc.x = cos(myangle) * mylength;
            mypointloc.y = sin(myangle) * mylength;
            pointlocs.push_back(mypointloc);

            ROS_INFO_STREAM("Cluster at (" << mypointloc.x << ", " << mypointloc.y);
        }
    }

}

void help(std::ostream& ostr) {
    ostr << "Usage: iarrc_cone_detector _laser_topic:=<laser-topic> _img_file:=<file-name>" << std::endl;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "iarrc_cone_detector");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // FIXME: Not expected behavior
    if(argc >= 2) {
        help(std::cerr);
        exit(1);
    }

    std::string laser_topic;
    nhp.param(std::string("laser_topic"), laser_topic, std::string("/scan"));

    nhp.param(std::string("cone_max"), cone_max, 0.1);
    nhp.param(std::string("cone_min"), cone_min, 0.0);

    ROS_INFO("Laser topic:= %s", laser_topic.c_str());
    ROS_INFO("Laser file:= %s", img_file.c_str());

    // Subscribe to ROS topic with callback
    ros::Subscriber cone_detect_sub = nh.subscribe(laser_topic, 1, ConeDetectorCB);
    //cone_publisher = nh.advertise<sensor_msgs::LaserScan>("hist_cone_pub", 1); //EDIT LATER

    ROS_INFO("IARRC cone detection node ready.");
    ros::spin();
    ROS_INFO("Shutting down IARRC cone detection node.");
    return 0;
}
