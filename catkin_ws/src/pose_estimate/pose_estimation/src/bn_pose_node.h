#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// create folder
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
// Pcl downsampling
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>

#include "text_msgs/bn_pose_srv.h"
#include "text_msgs/object_pose.h"

using namespace ros;
using namespace std;
using namespace cv;
using namespace pcl;
using namespace message_filters;

class bn_pose_node{
  public:
    bn_pose_node();
    bool serviceCb(text_msgs::bn_pose_srv::Request &req, text_msgs::bn_pose_srv::Response &res);
    // void callback(const subt_msgs::BoundingBoxes);
    void getXYZ(float* , float* ,float );
  private:
    ServiceServer service;
    Publisher pub_pose;
    Publisher pub_pc_process;
    Publisher pub_pc;
    Publisher markerPub;
    ros::Subscriber ssd_result;
    PointCloud<PointXYZRGB>::Ptr input;
    PointCloud<PointXYZRGB>::Ptr process;
    PointCloud<PointXYZRGB>::Ptr output;
    string target, source;
    float fx, fy, cx, cy;
    float upper_bound, lower_bound;

    VoxelGrid<PointXYZRGB> downsample;

    visualization_msgs::MarkerArray markerArray;
};