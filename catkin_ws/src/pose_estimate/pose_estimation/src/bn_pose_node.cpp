#include "bn_pose_node.h"

void bn_pose_node::getXYZ(float* a, float* b,float zc){

	float inv_fx = 1.0/fx;
	float inv_fy = 1.0/fy;
	*a = (*a - cx) * zc * inv_fx;
	*b = (*b - cy) * zc * inv_fy;
	return;
}
bool bn_pose_node::serviceCb(text_msgs::bn_pose_srv::Request &req, text_msgs::bn_pose_srv::Response &res){
	res.count = 0;

	static tf::TransformBroadcaster br;
	int count = 0;
	markerArray.markers.clear();

	// std::vector<int> indices;
	// pcl::removeNaNFromPointCloud(*input, *input, indices);

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

	cv_bridge::CvImagePtr img_ptr_depth = cv_bridge::toCvCopy(req.depth, sensor_msgs::image_encodings::TYPE_16UC1);
	cv_bridge::CvImagePtr img_ptr_img = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::RGB8);
	cv_bridge::CvImagePtr img_ptr_mask = cv_bridge::toCvCopy(req.mask, sensor_msgs::image_encodings::TYPE_8UC1);

	for(int cls = 0; cls < req.count; cls++){
		input->clear();
		// cout << req.list[cls] <<endl;
		for( int nrow = 0; nrow < img_ptr_depth->image.rows; nrow++){  
			for(int ncol = 0; ncol < img_ptr_depth->image.cols; ncol++){  
				if (img_ptr_mask->image.at<uint8_t>(nrow,ncol) == req.list[cls] && img_ptr_depth->image.at<unsigned short int>(nrow,ncol) > 0.01){

					//cout << img_ptr_depth->image.at<unsigned short int>(nrow,ncol) << endl;
					pcl::PointXYZRGB point;
					float* x = new float(nrow);
					float* y = new float(ncol);
				 	float z = float(img_ptr_depth->image.at<unsigned short int>(nrow,ncol))/1000.;

					getXYZ(y,x,z);
					point.x = z;
					point.y = -*y;
					point.z = -*x;    
					int i = img_ptr_mask->image.at<uint8_t>(nrow,ncol);   			
					Vec3b intensity =  img_ptr_img->image.at<Vec3b>(nrow, ncol); 
					point.r = int(intensity[0]);
					point.g = int(intensity[1]);
					point.b = int(intensity[2]);
					
					input->points.push_back(point);
					free(x);
					free(y);
				} 
			}  
		} 
		pcl::transformPointCloud(*input, *process, eigen_tf);

		// downsample the pc
		downsample.setInputCloud (process);
		downsample.filter (*process);
		// outlier removal
		sor.setInputCloud(process);
		sor.filter(*process);


		//// Visual bbox
		visualization_msgs::Marker marker;
		geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;
		marker.header.frame_id = "/camera_link";
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
		feature_extractor.setInputCloud (process);
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
		br.sendTransform(tf::StampedTransform(object_tf, ros::Time::now(), source, "object" + std::to_string(count)));
		*output += *process;
		
	}
	cout << output->points.size() << endl;

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

bn_pose_node::bn_pose_node(){
	NodeHandle nh;
	service = nh.advertiseService("bn_pose_node", &bn_pose_node::serviceCb, this);
	sensor_msgs::CameraInfo::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",ros::Duration());
	fx = msg->P[0];
	fy = msg->P[5];
	cx = msg->P[2];
	cy = msg->P[6];

	target = "base_link";  // base_link
	source = "camera_color_optical_frame";

	input.reset(new PointCloud<PointXYZRGB>()); 
	output.reset(new PointCloud<PointXYZRGB>());
	process.reset(new PointCloud<PointXYZRGB>());

	sor.setMeanK (10);
	sor.setStddevMulThresh (0.8);
	downsample.setLeafSize (0.01, 0.01, 0.01);

	pub_pc_process = nh.advertise<sensor_msgs::PointCloud2> ("process_pc", 10);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2> ("pc", 10);

	markerPub = nh.advertise<visualization_msgs::MarkerArray>("bbox", 10);

	// pub_pose = nh.advertise<subt_msgs::ArtifactPoseArray> ("/artifact_pose", 10);
	// ssd_result = nh.subscribe<subt_msgs::BoundingBoxes>("/ssd_prediction/BoundingBoxes", 1, &bn_pose_node::callback,this); 
}

int main(int argc, char** argv){
	init(argc, argv, "bn_pose_node");
	bn_pose_node bn_pose_node;
	spin();
	return 0;
}