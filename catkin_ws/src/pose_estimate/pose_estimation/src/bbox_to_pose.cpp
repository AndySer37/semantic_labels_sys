#include <iostream>
#include <vector>
#include <time.h>
#include <string>
#include <math.h>
// Ros lib
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
// PCL lib
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
//opencv library
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Pose.h>
#include "subt_msgs/BoundingBox.h"
#include "subt_msgs/BoundingBoxes.h"
#include "subt_msgs/ArtifactPose.h"
#include "subt_msgs/ArtifactPoseArray.h"
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
using namespace std;
using namespace cv;

class bbox_to_pose{
private:
    ros::Subscriber sub_cloud;
    ros::Publisher pub_artifactpose;
public:
    bbox_to_pose(ros::NodeHandle&);
    ~bbox_to_pose(){}
    float fx;
    float fy;
    float cx;
    float cy;
    void cbprocess(const subt_msgs::BoundingBoxes&);
    void getXYZ(float* , float* ,float );
};
bbox_to_pose::bbox_to_pose(ros::NodeHandle& n){
    pub_artifactpose = n.advertise<subt_msgs::ArtifactPoseArray>("cloud_ground", 1);
    sub_cloud = n.subscribe("/ssd_prediction/BoundingBoxes", 1, &bbox_to_pose::cbprocess, this);
    sensor_msgs::CameraInfo::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",ros::Duration());
    fx = msg->P[0];
    fy = msg->P[5];
    cx = msg->P[2];
    cy = msg->P[6];
}   

void bbox_to_pose::getXYZ(float* a, float* b,float zc){

    float inv_fx = 1.0/fx;
    float inv_fy = 1.0/fy;
    *a = (*a - cx) * zc * inv_fx;
    *b = (*b - cy) * zc * inv_fy;
    return;
}

void bbox_to_pose::cbprocess(const subt_msgs::BoundingBoxes& bbox){
    cv_bridge::CvImagePtr img_ptr_depth = cv_bridge::toCvCopy(bbox.depth, sensor_msgs::image_encodings::TYPE_16UC1);
    subt_msgs::ArtifactPoseArray ArtifactPoseArray_out = subt_msgs::ArtifactPoseArray();
    for (int i = 0; i < bbox.bounding_boxes.size(); i++){
        float* x = new float((bbox.bounding_boxes[i].xmin + bbox.bounding_boxes[i].xmax) / 2);
        float* y = new float((bbox.bounding_boxes[i].ymin + bbox.bounding_boxes[i].ymax) / 2);
        float z = float(img_ptr_depth->image.at<unsigned short int>(*x, *y))/1000.; 
        getXYZ(y,x,z);
        std_msgs::Header header = std_msgs::Header();
        geometry_msgs::Pose p = geometry_msgs::Pose();
        p.position.x = *x;
        p.position.y = *y;
        p.position.z = z;
        subt_msgs::ArtifactPose artifact_msg = subt_msgs::ArtifactPose();
        artifact_msg.Class = bbox.bounding_boxes[i].Class;
        artifact_msg.probability = bbox.bounding_boxes[i].probability;
        artifact_msg.pose = p;
        ArtifactPoseArray_out.pose_array.push_back(artifact_msg);
    }
    pub_artifactpose.publish(ArtifactPoseArray_out);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "bbox_to_pose");

	ros::NodeHandle nh("~");
	bbox_to_pose bbox_to_pose(nh);
	
	ros::spin ();

    return 0;
}