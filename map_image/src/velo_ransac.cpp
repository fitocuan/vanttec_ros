

#include "ros/ros.h"

#include <stdlib.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <pcl_ros/point_cloud.h>

#include <pcl/common/centroid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/PCLPointCloud2.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

using namespace cv;
using namespace std;
using namespace pcl;


class vpc{
public:
	vpc(){
		ros::NodeHandle n;
		pc_sub = n.subscribe("/velodyne_points", 10, &vpc::pcCallback, this);
		pub = n.advertise<sensor_msgs::PointCloud2>("output", 1);

	}

	void pcCallback(const sensor_msgs::PointCloud2 cloud_msg){
		cout<<"rec"<<endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(cloud_msg, *temp_cloud);


		pcl::NormalEstimation<PointXYZ, pcl::Normal> ne;
		pcl::SACSegmentationFromNormals<PointXYZ, pcl::Normal> seg;
		pcl::ExtractIndices<PointXYZ> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ> ());

		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);


		ne.setSearchMethod(tree);
		ne.setInputCloud(temp_cloud);
		ne.setKSearch(50);
		ne.compute(*cloud_normals);

		pcl::ModelCoefficients::Ptr coefficents (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE );
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight(0.01);
		seg.setMaxIterations(10000);
		seg.setDistanceThreshold(0.05);
		seg.setRadiusLimits(0,0.2);
		seg.setInputCloud(temp_cloud);
		seg.setInputNormals(cloud_normals);
		seg.segment(*inliers_sphere, *coefficients_sphere);

		

		extract.setInputCloud(temp_cloud);
		extract.setIndices(inliers_sphere);
		extract.setNegative(false);

		pcl::PointCloud<PointXYZ>::Ptr cloud_sphere (new pcl::PointCloud<PointXYZ> ());
		extract.filter(*cloud_sphere);


		if (!cloud_sphere->points.empty ()){
			std::cout << "PointCloud representing the cylindrical component: " << cloud_sphere->points.size () << " data points." << std::endl;
			cout<<*coefficients_sphere<<endl;
		}
		


		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*cloud_sphere,output);
		output.header = cloud_msg.header;

		pub.publish(output);

	}
	

protected:
	ros::Subscriber pc_sub;
	ros::Publisher pub;
};




int main(int argc, char **argv)
{

   ros::init(argc, argv, "velo_pc");

   vpc v;

   ros::Rate loop_rate(10);

    ros::spin();

   return 0;
}