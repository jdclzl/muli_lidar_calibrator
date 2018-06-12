#include <string>
#include <algorithm> 
#include <fstream>
#include <iostream>
#include <time.h>  
#include "multi_lidar_calibrator.h"
static int loopcount=0;
std::vector<Eigen::Matrix4f>   guesses_;
static bool   final_tf_ready=false;

std::string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );
    return tmp;
}



double SumMatrix(Eigen::Matrix4f matrix,int Column,int Line) //假设列数为Column 行数Line
{
    double sum=0; 
    for(int i=0;i<Line;i++){
		for(int j=0;j<Column;j++){

			sum+=matrix(i,j);
			return sum;

		}
   }
}
bool Matrixgreater ( Eigen::Matrix4f elem1, Eigen::Matrix4f elem2 ) {
 double elem1_sum=SumMatrix(elem1,4,4);
 double elem2_sum=SumMatrix(elem2,4,4);
 return   elem1_sum > elem2_sum;
}

void RosMultiLidarCalibratorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
	
	pcl::PointCloud<PointT>::Ptr in_parent_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr in_child_cloud(new pcl::PointCloud<PointT>);

	pcl::PointCloud<PointT>::Ptr child_filtered_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr out_one_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
	pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud);
	pcl::fromROSMsg(*in_child_cloud_msg, *in_child_cloud);

	parent_frame_ = in_parent_cloud_msg->header.frame_id;
	child_frame_ = in_child_cloud_msg->header.frame_id;

        
    
	DownsampleCloud(in_child_cloud, child_filtered_cloud, voxel_size_);

	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    
	ndt.setTransformationEpsilon(ndt_epsilon_);
	ndt.setStepSize(ndt_step_size_);
	ndt.setResolution(ndt_resolution_);

	ndt.setMaximumIterations(ndt_iterations_);

	ndt.setInputSource(child_filtered_cloud);
	ndt.setInputTarget(in_parent_cloud);
    if (loopcount<maxcount_){
	

	Eigen::Translation3f init_translation(initial_x_, initial_y_, initial_z_);
	Eigen::AngleAxisf init_rotation_x(initial_roll_, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(initial_pitch_, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(initial_yaw_, Eigen::Vector3f::UnitZ());

	Eigen::Matrix4f init_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	if(current_guess_ == Eigen::Matrix4f::Identity())
	{
		current_guess_ = init_guess_;
	}
    

	ndt.align(*output_cloud, current_guess_);
	std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged ()
	          << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability() << std::endl;

	std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;
	current_guess_ = ndt.getFinalTransformation();
    guesses_.push_back(current_guess_);
	Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
	Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);
	std::cout << "This transformation can be replicated using:" << std::endl;
	std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
	          << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
	          << " /" << child_frame_ << " 10" << std::endl;

        
	std::cout << "Corresponding transformation matrix:" << std::endl
	          << std::endl << current_guess_ << std::endl << std::endl;
	loopcount++;
	}else{

	//final_guess_=sum_guess_/maxcount_;
	if (!final_tf_ready){
		nth_element(guesses_.begin(),guesses_.begin()+maxcount_/2,guesses_.end(),Matrixgreater);		
	}
	final_tf_ready=true;
	final_guess_=guesses_[maxcount_/2-1];
	Eigen::Matrix3f rotation_matrix = final_guess_.block(0,0,3,3);
	Eigen::Vector3f translation_vector = final_guess_.block(0,3,3,1);
        
    
    	std::cout << "final tf is " << translation_vector.transpose()
	          << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
	          << " /" << child_frame_ << " 10" << std::endl;

		std::cout << "Corresponding transformation matrix:" << std::endl
	          << std::endl << final_guess_ << std::endl << std::endl;

    	std::ofstream outf("A.txt",std::ios::app);
        if(!outf){//检查是否打开成功
        std::cout << "Failed to create file...\n";
        exit(-1);
        }
		outf<<"--------------------------------------------------------"<<std::endl; 
		outf<<"calibration time :"<<getTime()<<std::endl<<std::endl;
        outf<< "calibration result(x y z yaw pitch roll):" << translation_vector.transpose()
	          << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
	          << " /" << child_frame_ << std::endl<< std::endl ;
		outf<< "Corresponding transformation matrix:" << std::endl
	          << std::endl << final_guess_ << std::endl;		
        outf.close();
		std::cout << "calibration complete!\n";
    exit(0);
	
	    
	
	}
	// timer end
	//auto end = std::chrono::system_clock::now();
	//auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	//std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

}


void RosMultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                                 pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                                                 double in_leaf_size)
{
	pcl::VoxelGrid<PointT> voxelized;
	voxelized.setInputCloud(in_cloud_ptr);
	voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
	voxelized.filter(*out_cloud_ptr);
}

void RosMultiLidarCalibratorApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
	//get params
	std::string points_parent_topic_str, points_child_topic_str;
	std::string initial_pose_topic_str = "/initialpose";
	std::string calibrated_points_topic_str = "/points_calibrated";
	std::string outone_points_topic_str = "/points_outone";

	in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
	ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());

	in_private_handle.param<std::string>("points_child_src", points_child_topic_str, "points_raw");
	ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_child_topic_str.c_str());

	in_private_handle.param<double>("voxel_size", voxel_size_, 0.1);
	ROS_INFO("[%s] ndt_epsilon: %.2f",__APP_NAME__, voxel_size_);

	in_private_handle.param<double>("ndt_epsilon", ndt_epsilon_, 0.01);
	ROS_INFO("[%s] voxel_size: %.2f",__APP_NAME__, ndt_epsilon_);

	in_private_handle.param<double>("ndt_step_size", ndt_step_size_, 0.1);
	ROS_INFO("[%s] ndt_step_size: %.2f",__APP_NAME__, ndt_step_size_);

	in_private_handle.param<double>("ndt_resolution", ndt_resolution_, 1.0);
	ROS_INFO("[%s] ndt_resolution: %.2f",__APP_NAME__, ndt_resolution_);

	in_private_handle.param<int>("ndt_iterations", ndt_iterations_, 400);
	ROS_INFO("[%s] ndt_iterations: %d",__APP_NAME__, ndt_iterations_);

        in_private_handle.param<int>("maxcount", maxcount_, 40);
	ROS_INFO("[%s] maxcount: %d",__APP_NAME__, maxcount_);

	in_private_handle.param<double>("x", initial_x_, 0.0);
	in_private_handle.param<double>("y", initial_y_, 0.0);
	in_private_handle.param<double>("z", initial_z_, 0.0);
	in_private_handle.param<double>("roll", initial_roll_, 0.0);
	in_private_handle.param<double>("pitch", initial_pitch_, 0.0);
	in_private_handle.param<double>("yaw", initial_yaw_, 0.0);
        
	ROS_INFO("[%s] Initialization Transform x: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f", __APP_NAME__,
	         initial_x_, initial_y_, initial_z_,
	         initial_roll_, initial_pitch_, initial_yaw_);

	//generate subscribers and synchronizer
	cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                     points_parent_topic_str, 10);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());

	cloud_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                        points_child_topic_str, 10);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str.c_str());

	/*initialpose_subscriber_ = node_handle_.subscribe(initial_pose_topic_str, 10,
	                                                          &RosMultiLidarCalibratorApp::InitialPoseCallback, this);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, initial_pose_topic_str.c_str());*/
	cloud_synchronizer_ =
			new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
			                                               *cloud_parent_subscriber_,
			                                               *cloud_child_subscriber_);
	cloud_synchronizer_->registerCallback(boost::bind(&RosMultiLidarCalibratorApp::PointsCallback, this, _1, _2));

}


void RosMultiLidarCalibratorApp::Run()
{
	ros::NodeHandle private_node_handle("~");

	InitializeRosIo(private_node_handle);
    
	ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END",__APP_NAME__);
}

RosMultiLidarCalibratorApp::RosMultiLidarCalibratorApp()
{
	//initialpose_quaternion_ = tf::Quaternion::getIdentity();
	current_guess_ = Eigen::Matrix4f::Identity();
}
