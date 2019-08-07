

#include "ros/ros.h"

#include <stdlib.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/common/centroid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
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


#include <custom_msgs/ObjDetected.h>
#include <custom_msgs/ObjDetectedList.h>

using namespace cv;
using namespace std;
using namespace pcl;


struct object{
	pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
	pcl::PointXYZ cen;
	int counters;
	bool seen;
};

class vpc{
public:
	vpc(){
		ros::NodeHandle n;
		pc_sub = n.subscribe("/velodyne_points", 10, &vpc::pcCallback, this);
		pub = n.advertise<sensor_msgs::PointCloud2>("output", 1);
		pub_obj = n.advertise<custom_msgs::ObjDetectedList>("lidar_objs_det", 1);

	}

	void pcCallback(const sensor_msgs::PointCloud2 cloud_msg){
		cout<<"rec"<<endl;

/*
		pcl::PCLPointCloud2 pcl_pc;
		pcl_conversions::toPCL(cloud_msg, pcl_pc);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(pcl_pc,*temp_cloud);
*/

		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(cloud_msg, *temp_cloud);


		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (temp_cloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.06);
		ec.setMinClusterSize(200);
		ec.setMaxClusterSize(250000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(temp_cloud);
		ec.extract(cluster_indices);


		int j = 0;

		

		for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

			int r = rand() % 255;
			int g = rand() % 255;
			int b = rand() % 255;


			pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_output (new pcl::PointCloud<pcl::PointXYZRGB>);

			for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
				cloud_cluster->points.push_back(temp_cloud->points[*pit]);
				pcl::PointXYZRGB p;


				p.x = temp_cloud->points[*pit].x;
				p.y = temp_cloud->points[*pit].y;
				p.z = temp_cloud->points[*pit].z;

				p.r = r;
				p.g = g;
				p.b = b;

				color_output->points.push_back(p);
			}

			bool flag = false;



			pcl::PointXYZ centroid;
			pcl::computeCentroid(*color_output, centroid);

			float diff = 0.1;

			for(int ii = 0; ii<vec_obj.size();ii++){
				vec_obj[ii].seen = false;
			}


			for(int ii = 0; ii<vec_obj.size();ii++){
				if(abs(vec_obj[ii].cen.x - centroid.x) < diff && abs(vec_obj[ii].cen.y - centroid.y) < diff && abs(vec_obj[ii].cen.z - centroid.z) < diff){
					flag = true;
					vec_obj[ii].cen.x = centroid.x;
					vec_obj[ii].cen.y = centroid.y;
					vec_obj[ii].cen.z = centroid.z;

					for(int jj = 0; jj < color_output->points.size();jj++){
						color_output->points[jj].r = vec_obj[ii].point_cloud.points[0].r;
						color_output->points[jj].g = vec_obj[ii].point_cloud.points[0].g;
						color_output->points[jj].b = vec_obj[ii].point_cloud.points[0].b;
					}

					vec_obj[ii].seen = true;

					vec_obj[ii].point_cloud = *color_output;
					break;
				}
			}

			

			if(!flag){
				object obj;
				obj.cen = centroid;
				obj.point_cloud = *color_output;
				obj.counters = 0;
				obj.seen = false;
				vec_obj.push_back(obj);

				/*
				vec_cen.push_back(centroid);
				vec_pc.push_back(*color_output);
				counters.push_back(0);
				seen.push_back(false);
				*/
			}else{

				for(int ii = 0; ii<vec_obj.size();ii++){
					if(!vec_obj[ii].seen){
						vec_obj[ii].counters++;
					}else{
						vec_obj[ii].counters = 0;
					}
				}	
			}
			

			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);

		custom_msgs::ObjDetectedList obj_list;

		int c = 0;
		for(int i = 0; i<vec_obj.size(); i++){
			if(vec_obj[i].counters<15){

				*output_pcl += vec_obj[i].point_cloud;
				custom_msgs::ObjDetected obj_det;
				obj_det.X = roundf(vec_obj[i].cen.x * 100) / 100;
				obj_det.Y = roundf(vec_obj[i].cen.y * 100) / 100;
				obj_det.clase = "";
				obj_det.color = "";
				obj_list.objects.push_back(obj_det);
				c++;

			}else{
				vec_obj.erase(vec_obj.begin()+i);
			}
			
		}

		obj_list.len = c;

		cout<<"Obj: "<<vec_obj.size()<<endl;

		pub_obj.publish(obj_list);

		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*output_pcl,output);
		output.header = cloud_msg.header;

		pub.publish(output);

	}
	

protected:
	ros::Subscriber pc_sub;
	ros::Publisher pub;
	ros::Publisher pub_obj;
	std::vector<object> vec_obj;


};




int main(int argc, char **argv)
{

   ros::init(argc, argv, "velo_pc");

   vpc v;

   ros::Rate loop_rate(10);

    ros::spin();

   return 0;
}