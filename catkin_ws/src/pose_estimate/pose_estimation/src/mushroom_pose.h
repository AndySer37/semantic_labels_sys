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
// Pcl outlier removal
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
// Cluster
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>

#include "text_msgs/object_only.h"
#include "text_msgs/object_pose.h"

using namespace ros;
using namespace std;
using namespace cv;
using namespace pcl;
using namespace message_filters;

class object_pose_node{
  public:
    object_pose_node();
    bool serviceCb(text_msgs::object_only::Request &req, text_msgs::object_only::Response &res);
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
    float z_upper_bound, z_lower_bound;

    VoxelGrid<PointXYZRGB> downsample;
    StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;

    visualization_msgs::MarkerArray markerArray;
};