

#include <string.h> 
#include <algorithm> 
#include <fstream>
#include <vector>
#include "joint_two_pointclouds.h"

Eigen::Matrix4f  getCalibrationResultFromLog(){
    std::ifstream  fin( "/home/zhaolei/.ros/A.txt" , std::ios::ate );  
	if( !fin )  
	{  
		std::cerr<<"Failed to load log file！";  
		exit(-1);  
	}  
	
	// 先倒回文件末尾两个字符  
	fin.seekg(-2, fin.cur);  
	// 假定反向读取4行记录  
	int lineCount = 4;  
	for(int i = 0; i < lineCount; i++)  
	{         
		// 查看前一个字符是否为回车符  
		while( fin.peek() != fin.widen('\n') )  
		{  
			fin.seekg(-1, fin.cur );  
		}  
		// 走到这里表示跳过一行了，所以继续跳直到够100行  
		fin.seekg(-1, fin.cur);  
	}  
	
	fin.seekg(2, fin.cur);  
	// 现在文件指针指向99行的末尾，可以读取了  
	
	Eigen::Matrix4f result; 
	for (int i = 0; i < 4; i++)//定义行循环  
    {  
        for (int j = 0; j < 4; j++)//定义列循环  
        {  
            fin >> result(i,j);//读取一个值（空格、制表符、换行隔开）就写入到矩阵中，行列不断循环进行  
            
		}  
    }  
	fin.clear();  
	fin.close();  
    //std::cout << "read calibration result:"<< result << std::endl;
    return result;


}


void JointTwoPointCloudsApp::PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header.frame_id = parent_frame_;
	in_publisher.publish(cloud_msg);
}

void JointTwoPointCloudsApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
	
	pcl::PointCloud<PointT>::Ptr in_parent_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr in_child_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

	pcl::PointCloud<PointT>::Ptr child_filtered_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr out_one_cloud (new pcl::PointCloud<PointT>);
    
	pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud);
	pcl::fromROSMsg(*in_child_cloud_msg, *in_child_cloud);

	parent_frame_ = in_parent_cloud_msg->header.frame_id;
	child_frame_ = in_child_cloud_msg->header.frame_id;

	calibration_result_=getCalibrationResultFromLog();
	//Eigen::Matrix3f rotation_matrix = final_guess_.block(0,0,3,3);
	//Eigen::Vector3f translation_vector = final_guess_.block(0,3,3,1);


        
	
	// Transforming unfiltered, input cloud using found transform.
	//ndt.align(*output_cloud, final_guess_);
	pcl::transformPointCloud (*in_child_cloud, *output_cloud,calibration_result_);
    out_one_cloud=in_parent_cloud;
	*out_one_cloud+=*output_cloud;
    PublishCloud(outone_cloud_publisher_, out_one_cloud);


}




void JointTwoPointCloudsApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
	//get params
	std::string points_parent_topic_str, points_child_topic_str;
	
	std::string calibrated_points_topic_str = "/points_calibrated";
	std::string outone_points_topic_str = "/points_outone";

	in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
	ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());

	in_private_handle.param<std::string>("points_child_src", points_child_topic_str, "points_raw");
	ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_child_topic_str.c_str());
    
	in_private_handle.param<std::string>("log_file_path", log_file_path, "/home/zhaolei/.ros/A.txt");
	
        
	ROS_INFO("[%s] LOG FILE PATH: %s", __APP_NAME__,log_file_path.c_str());

	getCalibrationResultFromLog();
	//generate subscribers and synchronizer
	cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                     points_parent_topic_str, 10);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());

	cloud_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                        points_child_topic_str, 10);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str.c_str());

	/*initialpose_subscriber_ = node_handle_.subscribe(initial_pose_topic_str, 10,
	                                                          &JointTwoPointCloudsApp::InitialPoseCallback, this);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, initial_pose_topic_str.c_str());*/

	calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(calibrated_points_topic_str, 1);
        
	ROS_INFO("[%s] Publishing Calibrated PointCloud to... %s",__APP_NAME__, calibrated_points_topic_str.c_str());
        
	outone_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(outone_points_topic_str, 1);

    ROS_INFO("[%s] Publishing outonePointCloud to... %s",__APP_NAME__, outone_points_topic_str.c_str());

	cloud_synchronizer_ =
			new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
			                                               *cloud_parent_subscriber_,
			                                               *cloud_child_subscriber_);
	cloud_synchronizer_->registerCallback(boost::bind(&JointTwoPointCloudsApp::PointsCallback, this, _1, _2));

}


void JointTwoPointCloudsApp::Run()
{
	ros::NodeHandle private_node_handle("~");

	InitializeRosIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END",__APP_NAME__);
}

JointTwoPointCloudsApp::JointTwoPointCloudsApp()
{
	//initialpose_quaternion_ = tf::Quaternion::getIdentity();
	
}
