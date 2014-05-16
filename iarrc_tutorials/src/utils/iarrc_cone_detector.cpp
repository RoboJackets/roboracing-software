#include <stdio.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
// OKAY THIS GAVE ME ERROR!?
#include <pcl_ros/point_cloud.h>
// THIS TOO???
#include <pcl/point_types.h>
std::string img_file;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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
PointCloud::Ptr newmsg (new PointCloud);


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

    newmsg->header.frame_id = "laser";
    newmsg->height = newmsg->width = 1;


    int npoint = 0;
    float myangle = 0;
    float mylength = 0;
    pointloc mypointloc;
    mypointloc.x = 0;
    mypointloc.y = 0;
    vector<pointloc> pointlocs;
    for (int i=0; i<size; i++){
        if (l[i].size>0){
            npoint = (i+1)+(l[i].size)/2;
            myangle = anglemin + npoint*angle;
            mylength = msg->ranges[npoint];
            mypointloc.x = cos(myangle) * mylength;
            mypointloc.y = sin(myangle) * mylength;
            pointlocs.push_back(mypointloc);
            newmsg->points.push_back (pcl::PointXYZ(mypointloc.x, mypointloc.y, 0.0));
        }
    }


     newmsg->header.stamp = ros::Time::now ();
     point_publisher.publish (newmsg);
     newmsg->points.clear();
}


/*int main(int argc, char** argv){
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "some_tf_frame";
    msg->height = msg->width = 1;
    msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));



    while (nh.ok()) {
     msg->header.stamp = ros::Time::now ();
     pub.publish (msg);
     ros::spinOnce ();
     loop_rate.sleep ();
   }
} */

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

    ROS_INFO("Laser topic:= %s", laser_topic.c_str());
    ROS_INFO("Laser file:= %s", img_file.c_str());

    // Subscribe to ROS topic with callback
    ros::Subscriber cone_detect_sub = nh.subscribe(laser_topic, 1, ConeDetectorCB);
    //cone_publisher = nh.advertise<sensor_msgs::LaserScan>("hist_cone_pub", 1); //EDIT LATER
    point_publisher = nh.advertise<PointCloud> ("point_cone_pub", 1);

    ROS_INFO("IARRC cone detection node ready.");
    ros::spin();
    ROS_INFO("Shutting down IARRC cone detection node.");
    return 0;
}
