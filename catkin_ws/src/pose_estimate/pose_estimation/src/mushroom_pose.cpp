#include "mushroom_pose.h"

void object_pose_node::getXYZ(float* a, float* b,float zc){

	float inv_fx = 1.0/fx;
	float inv_fy = 1.0/fy;
	*a = (*a - cx) * zc * inv_fx;
	*b = (*b - cy) * zc * inv_fy;
	return;
}
bool object_pose_node::serviceCb(text_msgs::object_only::Request &req, text_msgs::object_only::Response &res){
	res.count = 0;

	sensor_msgs::PointCloud2::ConstPtr ros_pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points",ros::Duration());
	pcl::fromROSMsg(*ros_pc, *input);
	static tf::TransformBroadcaster br;
	int count = 0;
	markerArray.markers.clear();

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*input, *input, indices);

	tf::StampedTransform transform;
	static tf::TransformListener listener;
	try{
		listener.waitForTransform(target, source, ros::Time(0), ros::Duration(0.5));
		listener.lookupTransform(target, source, ros::Time(0), transform);
	} catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	Eigen::Matrix4f eigen_tf = Eigen::Matrix4f::Identity();
	tf::Matrix3x3 rotation(transform.getRotation());
	for (int i = 0; i < 3; i++){
		for (int j = 1; j < 3; j++){
			eigen_tf(i, j) = rotation[i][j];
		}
	}
	eigen_tf(0, 3) = transform.getOrigin().getX();
	eigen_tf(1, 3) = transform.getOrigin().getY(); 
	eigen_tf(2, 3) = transform.getOrigin().getZ(); 

	pcl::transformPointCloud(*input, *input, eigen_tf);
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, z_lower_bound)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, z_upper_bound)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, y_lower_bound)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, y_upper_bound)));

	pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(input);
	condrem.filter(*process);

	// downsample the pc
	downsample.setInputCloud (process);
	downsample.filter (*process);
	// outlier removal
	sor.setInputCloud(process);
	sor.filter(*process);


	std::vector<pcl::PointIndices> cluster_indices;

	ec.setInputCloud (process);
	ec.extract (cluster_indices);
	for(int i=0; i<cluster_indices.size(); i++){ // In points number order
		pcl::PointCloud<pcl::PointXYZRGB> cluster(*process, cluster_indices[i].indices);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGB>);
		*p = cluster;
		*output += cluster;
		cout << "Cluster " << i << " with " << cluster_indices[i].indices.size() << " points\n";

		//// Visual bbox
		visualization_msgs::Marker marker;
		geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;
		marker.header.frame_id = "/camera_color_optical_frame";
		marker.header.stamp = ros::Time::now();
		marker.ns = "lines";
		marker.id = count;
		count ++;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.scale.x = 0.01;
		marker.scale.y = 0.01;
		marker.scale.z = 0.01;
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
		pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
		feature_extractor.setInputCloud (p);
		feature_extractor.compute ();
		pcl::PointXYZRGB minPt, maxPt, position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		feature_extractor.getOBB (minPt, maxPt, position_OBB, rotational_matrix_OBB);		
		Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
		Eigen::Vector3f a(minPt.x, minPt.y, minPt.z);
		a = rotational_matrix_OBB * a + position;
		p1.x = a(0);
		p1.y = a(1);
		p1.z = a(2);
		Eigen::Vector3f b(minPt.x, maxPt.y, minPt.z);
		b = rotational_matrix_OBB * b + position;
		p2.x = b(0);
		p2.y = b(1);
		p2.z = b(2);
		Eigen::Vector3f c(maxPt.x, maxPt.y, minPt.z);
		c = rotational_matrix_OBB * c + position;
		p3.x = c(0);
		p3.y = c(1);
		p3.z = c(2);
		Eigen::Vector3f d(maxPt.x, minPt.y, minPt.z);
		d = rotational_matrix_OBB * d + position;
		p4.x = d(0);
		p4.y = d(1);
		p4.z = d(2);
		Eigen::Vector3f e(minPt.x, minPt.y, maxPt.z);
		e = rotational_matrix_OBB * e + position;
		p5.x = e(0);
		p5.y = e(1);
		p5.z = e(2);
		Eigen::Vector3f f(minPt.x, maxPt.y, maxPt.z);
		f = rotational_matrix_OBB * f + position;
		p6.x = f(0);
		p6.y = f(1);
		p6.z = f(2);
		Eigen::Vector3f g(maxPt.x, maxPt.y, maxPt.z);
		g = rotational_matrix_OBB * g + position;
		p7.x = g(0);
		p7.y = g(1);
		p7.z = g(2);
		Eigen::Vector3f h(maxPt.x, minPt.y, maxPt.z);
		h = rotational_matrix_OBB * h + position;
		p8.x = h(0);
		p8.y = h(1);
		p8.z = h(2);

		marker.points.push_back(p1);
		marker.points.push_back(p2);
		marker.points.push_back(p3);
		marker.points.push_back(p4);
		marker.points.push_back(p1);
		marker.points.push_back(p5);
		marker.points.push_back(p6);
		marker.points.push_back(p7);
		marker.points.push_back(p8);
		marker.points.push_back(p5);
		marker.points.push_back(p6);
		marker.points.push_back(p2);
		marker.points.push_back(p3);
		marker.points.push_back(p7);
		marker.points.push_back(p8);
		marker.points.push_back(p4);	
		marker.lifetime = ros::Duration(10);
		markerArray.markers.push_back(marker); 
		cout << rotational_matrix_OBB << endl;

		//// Visual bbox end ////

		//// Publish tf and return pose
		tf::Vector3 tran = tf::Vector3(position(0), position(1), position(2));
		tf::Matrix3x3 rot = tf::Matrix3x3(rotational_matrix_OBB(0,0), rotational_matrix_OBB(0,1), rotational_matrix_OBB(0,2),
												rotational_matrix_OBB(1,0), rotational_matrix_OBB(1,1), rotational_matrix_OBB(1,2),
												rotational_matrix_OBB(2,0), rotational_matrix_OBB(2,1), rotational_matrix_OBB(2,2));
		if (rot[2][2] < 0){
			tf::Matrix3x3 temp_z = tf::Matrix3x3(-1, 0, 0,
												0, 1, 0,
												0, 0, -1);
			rot *= temp_z;
		}
		if (rot[1][1] < 0){
			tf::Matrix3x3 temp_y = tf::Matrix3x3(-1, 0, 0,
												0, -1, 0,
												0, 0, 1);
			rot *= temp_y;
		}

		tf::Transform object_tf = tf::Transform(rot, tran);
		text_msgs::object_pose ob_pose;
		// geometry_msgs::Pose pose;
		quaternionTFToMsg(object_tf.getRotation(), ob_pose.pose.orientation); 
		tf::Vector3 pose_trans =  object_tf.getOrigin();
		ob_pose.pose.position.x = pose_trans.getX();
		ob_pose.pose.position.y = pose_trans.getY();
		ob_pose.pose.position.z = pose_trans.getZ();
		res.ob_list.push_back(ob_pose);

		res.count += 1;
		/////////////////////////////////////
		br.sendTransform(tf::StampedTransform(object_tf, ros::Time::now(), target, "object" + std::to_string(count)));

	} 
	int num_cluster = cluster_indices.size();

	sensor_msgs::PointCloud2 object_cloud_msg;
	toROSMsg(*output, object_cloud_msg);
	object_cloud_msg.header.frame_id = "camera_color_optical_frame";
	pub_pc_process.publish(object_cloud_msg);

	markerPub.publish(markerArray);

	output->clear();
	input->clear();
	process->clear();

	return true;
}

object_pose_node::object_pose_node(){
	NodeHandle nh;
	service = nh.advertiseService("object_pose_node", &object_pose_node::serviceCb, this);
	sensor_msgs::CameraInfo::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",ros::Duration());
	fx = msg->P[0];
	fy = msg->P[5];
	cx = msg->P[2];
	cy = msg->P[6];

	target = "camera_color_optical_frame";  // base_link
	source = "camera_color_optical_frame";

	input.reset(new PointCloud<PointXYZRGB>()); 
	output.reset(new PointCloud<PointXYZRGB>());
	process.reset(new PointCloud<PointXYZRGB>());
	tree.reset(new pcl::search::KdTree<pcl::PointXYZRGB>);

	sor.setMeanK (10);
	sor.setStddevMulThresh (0.8);
	downsample.setLeafSize (0.015, 0.015, 0.015);

	ec.setClusterTolerance (0.05); 
	ec.setMinClusterSize (30);
	ec.setMaxClusterSize (20000);
	ec.setSearchMethod (tree);

	y_lower_bound = -0.05;
	y_upper_bound = 0.0;
	z_lower_bound = 0.3;
	z_upper_bound = 1.0;

	pub_pc_process = nh.advertise<sensor_msgs::PointCloud2> ("process_pc", 10);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2> ("pc", 10);

	markerPub = nh.advertise<visualization_msgs::MarkerArray>("bbox", 10);

	// pub_pose = nh.advertise<subt_msgs::ArtifactPoseArray> ("/artifact_pose", 10);
	// ssd_result = nh.subscribe<subt_msgs::BoundingBoxes>("/ssd_prediction/BoundingBoxes", 1, &object_pose_node::callback,this); 
}

int main(int argc, char** argv){
	init(argc, argv, "object_pose_node");
	object_pose_node object_pose_node;
	spin();
	return 0;
}