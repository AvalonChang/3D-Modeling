#include <iostream>
#include <string>
#include <sstream>

//#include <std_msgs/Int64.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Eigen>

#include <pcl/features/normal_3d.h>
#include <cmath>
//#include <geometry_msgs/Vector3.h>

typedef pcl::PointXYZRGBA PointT;
struct Normal {
	long double x;
	long double y;
	long double z;
};

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	// set background to black (R = 0, G = 0, B = 0)
	viewer.setBackgroundColor(0.2, 0.2, 0.2);
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	// you can add something here, ex:  add text in viewer
}

void ShowCloud(pcl::PointCloud<PointT>::Ptr &Cloud_ptr);

pcl::PointCloud<PointT>::Ptr cloud_filter(pcl::PointCloud<PointT>::Ptr &cloud)
{
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

	//****************************************************//
	  // Create the filtering object - passthrough
	float Low, Height;

	pcl::PassThrough<PointT> passz;
	passz.setInputCloud(cloud);
	passz.setFilterFieldName("z");
	Low = -7;
	Height = -0.3;
	passz.setFilterLimits(Low, Height);
	passz.filter(*cloud_filtered);

	/*pcl::PassThrough<PointT> passy;
	passy.setInputCloud(cloud_filtered);
	passy.setFilterFieldName("y");
	Low = -0.5;
	Height = 0.22;
	passz.setFilterLimits(Low, Height);

	//pass.setFilterLimitsNegative (true);
	passy.filter(*cloud_filtered);*/

	pcl::PassThrough<PointT> passx;
	passx.setInputCloud(cloud_filtered);
	passx.setFilterFieldName("x");
	Low = -0.3;
	Height = 0.2;
	passx.setFilterLimits(Low, Height);

	//pass.setFilterLimitsNegative (true);
	passx.filter(*cloud_filtered);
	//****************************************************//

	//****************************************************//
	  // // segment ground
	  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	  // // Create the segmentation object
	  // pcl::SACSegmentation<PointT> seg;
	  // // Optional
	  // seg.setOptimizeCoefficients (true);
	  // // Mandatory
	  // seg.setModelType (pcl::SACMODEL_PLANE);  // plane
	  // seg.setMethodType (pcl::SAC_RANSAC);
	  // seg.setDistanceThreshold (0.010);

	  // seg.setInputCloud (cloud_filtered);
	  // seg.segment (*inliers, *coefficients);

	  // pcl::ExtractIndices<PointT> extract;
	  // extract.setInputCloud(cloud_filtered);
	  // extract.setIndices(inliers);
	  // extract.setNegative(true);
	  // extract.filter(*cloud_filtered);
	//****************************************************//

	//****************************************************//
	  // Create the filtering object - StatisticalOutlierRemoval filter
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud_filtered);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	//****************************************************//

	return cloud_filtered;
}

// FileName為不包含副檔名的檔案名稱
Normal plane_normal(pcl::PointCloud<PointT>::Ptr &cloud, std::string FileName = "")
{
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	cloud_filtered = cloud;

	// // segment ground
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);  // plane
	seg.setMethodType(pcl::SAC_RANSAC);
	double Threshold = 0.01;
	seg.setDistanceThreshold(Threshold);

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);

	///######################################
	///######################################
	///######################################
	extract.setNegative(true);
	extract.filter(*cloud_filtered);
	///######################################
	///######################################
	///######################################
	  //****************************************************//
		// Create the filtering object - StatisticalOutlierRemoval filter
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud_filtered);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
	ShowCloud(cloud_filtered);
	pcl::io::savePLYFileBinary(FileName + "_Extract.ply", *cloud_filtered);
	//****************************************************//

  ///######################################
  ///#### get normal of the plane
  ///######################################
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_filtered);

	normalEstimation.setRadiusSearch(0.05);

	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	normalEstimation.setSearchMethod(kdtree);

	normalEstimation.compute(*normals);

	long double mean_normal_x = 0;
	long double mean_normal_y = 0;
	long double mean_normal_z = 0;
	unsigned int normal_cnt = 0;

	for (unsigned int ix = 0; ix < normals->points.size(); ix++)
	{
		if (std::isnan(normals->points[ix].normal_x) ||
			std::isnan(normals->points[ix].normal_y) ||
			std::isnan(normals->points[ix].normal_z))
		{
			continue;
			// std::cout<<"no ix=" << ix << std::endl;
		}
		else
		{
			normal_cnt++;
			// std::cout<<"yes" << std::endl;
			mean_normal_x += normals->points[ix].normal_x;
			mean_normal_y += normals->points[ix].normal_y;
			mean_normal_z += normals->points[ix].normal_z;
		}
	}
	mean_normal_x /= normal_cnt;
	mean_normal_y /= normal_cnt;
	mean_normal_z /= normal_cnt;

	// std::cout << "x: " << mean_normal_x << std::endl;
	// std::cout << "y: " << mean_normal_y << std::endl;
	// std::cout << "z: " << mean_normal_z << std::endl;
	Normal normal_of_plane;
	normal_of_plane.x = mean_normal_x;
	normal_of_plane.y = mean_normal_y;
	normal_of_plane.z = mean_normal_z;

	///######################################
	///######################################
	///######################################

	return normal_of_plane;

}

float cal_angle(Normal norm1, Normal norm2)
{
	float angle;
	angle = std::acos(
		std::abs(norm1.x*norm2.x + norm1.y*norm2.y + norm1.z*norm2.z) /
		std::sqrt(norm1.x*norm1.x + norm1.y*norm1.y + norm1.z*norm1.z) /
		std::sqrt(norm2.x*norm2.x + norm2.y*norm2.y + norm2.z*norm2.z)
	);

	return angle;
}

void ShowCloud(pcl::PointCloud<PointT>::Ptr &Cloud_ptr)
{
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	// blocks until the cloud is actually rendered
	viewer.showCloud(Cloud_ptr);

	// use the following functions to get access to the underlying more advanced/powerful
	// PCLVisualizer

	// This will only get called once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	// This will get called once per visualization iteration
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped()) {

	}
}

int main(int argc, char **argv)
{
	pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);

	std::string FileName[2];
	FileName[0] = "C:\\Users\\newsmart\\Desktop\\NSTAlgo\\3D\\PCL\\stitch\\2\\2-11.ply";
	FileName[1] = "C:\\Users\\newsmart\\Desktop\\NSTAlgo\\3D\\PCL\\stitch\\2\\2-12.ply";
	Normal Norm[2];

	/*std::string FileName1 = "C:\\Users\\newsmart\\Desktop\\NSTAlgo\\3D\\PCL\\stitch\\2\\2-1.ply";
	std::size_t LastDot1  = FileName1.find_last_of('.');
	std::string FileName2 = "C:\\Users\\newsmart\\Desktop\\NSTAlgo\\3D\\PCL\\stitch\\2\\2-2.ply";
	std::size_t LastDot2  = FileName2.find_last_of('.');
	Normal norm1, norm2;*/

	for (int i = 0; i < 2; i++)
	{
		std::size_t LastDot = FileName[i].find_last_of('.');

		if (pcl::io::loadPLYFile<PointT>(FileName[i], *cloud_ptr) == -1) return -1;
		// Filter
		cloud_ptr = cloud_filter(cloud_ptr);
		pcl::io::savePLYFileBinary(FileName[i].substr(0, LastDot) + "_Filter.ply", *cloud_ptr);

		// Plane Normal
		Norm[i] = plane_normal(cloud_ptr, FileName[i].substr(0, LastDot));
		std::cout << "Norm " << i << "x: " << Norm[i].x << std::endl;
		std::cout << "Norm " << i << "y: " << Norm[i].y << std::endl;
		std::cout << "Norm " << i << "z: " << Norm[i].z << std::endl;
	}

	float angle_radians = cal_angle(Norm[0], Norm[1]);
	std::cout << "angle is (in radians):" << angle_radians << std::endl;
	if (angle_radians < 0.05) {
		// about 3 degrees
		//task_done = 1;

		//normal_final.x = norm2.x;
		//normal_final.y = norm2.y;
		//normal_final.z = norm2.z;
		return 1;
	}

	return 0;
}