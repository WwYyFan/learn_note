#include "gui/target_pose.h"
#include <stdio.h>     
#include <string>
#include<iostream>
#include<sstream>
#include <cstring>
#include <math.h>
#include <jsoncpp/json/json.h>
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <vtkAutoInit.h>         
#include <limits>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include "pcl/filters/project_inliers.h"
#include <pcl/features/normal_3d_omp.h>   //����

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>

#include <pcl/visualization/pcl_visualizer.h>   //���ӻ�
#include <pcl/visualization/cloud_viewer.h>

#include <cstdlib>
#include <fstream>
#include<string.h>
#include<vector>
#include <cstring>
#include <exception>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/region_growing.h>//��������

#include <pcl/features/feature.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
 #include <omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/keypoints/uniform_sampling.h>


typedef pcl::PointXYZRGB  PointTypeC;

int main()
{	
	clock_t start,end;
	start = clock();
    pcl::PointCloud<PointTypeC>::Ptr cloud(new pcl::PointCloud<PointTypeC>);

    //加载点云
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/efun/3dstruct/result/result.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	
    // //分割桌面
    // pcl::PassThrough<PointTypeC> pass;
	// pass.setInputCloud(cloud);
	// pass.setFilterFieldName("z");
	// pass.setFilterLimits(515, 530);
	// pass.setFilterLimitsNegative(false);
	// pass.filter(*cloud);
	

    //下采样
    pcl::UniformSampling<PointTypeC> filter;
    filter.setInputCloud(cloud);
    filter.setRadiusSearch(0.5f);
    pcl::PointCloud<int> keypointIndices;
    filter.compute(keypointIndices);
    pcl::PointCloud<PointTypeC>::Ptr filteredCloud(new pcl::PointCloud<PointTypeC>);
    pcl::copyPointCloud(*cloud, keypointIndices.points, *filteredCloud);
    cloud = filteredCloud;
	
    //降噪
    pcl::StatisticalOutlierRemoval<PointTypeC> sor;   
    sor.setInputCloud (cloud);                           
    sor.setMeanK (50);                               
    sor.setStddevMulThresh (1.0);                     
    sor.filter (*cloud);                    

    //欧式聚类分割
    pcl::search::KdTree<PointTypeC>::Ptr tree(new pcl::search::KdTree<PointTypeC>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointTypeC> ec;   
	ec.setClusterTolerance(1.0f);                     
	ec.setMinClusterSize(2000);               
	ec.setMaxClusterSize(2500000);             
	ec.setSearchMethod(tree);                   
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices); 

    if (cluster_indices.size() == 0)
	{
		std::cout << "未发现目标！" << std::endl;
		return 0;
	}
    else 
    {
        // std::cout << "total:" << cluster_indices.size() << std::endl;
    }

    //找到最高的物体聚类
	int topIndex = 0;
	double topZValue = 1000;
	pcl::PointCloud<PointTypeC>::Ptr topCluster(new pcl::PointCloud<PointTypeC>);
	
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<PointTypeC>::Ptr cloud_cluster(new pcl::PointCloud<PointTypeC>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); 
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		j++;
		// Eigen::Vector4f pcaCentroid;
		PointTypeC minp1, maxp1;
		// pcl::compute3DCentroid(*cloud_cluster, pcaCentroid);
		pcl::getMinMax3D(*cloud_cluster, minp1, maxp1);
        
		
        if (maxp1.z < topZValue)
		{
			topIndex = j;
			topZValue = maxp1.z;
			topCluster = cloud_cluster;
		}
        // std::cout << maxp1.z << std::endl;
        
		// pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		// viewer.showCloud(cloud_cluster);
		// while (!viewer.wasStopped())
		// {
		// }
	}

	// pcl::visualization::CloudViewer viewer5("Simple Cloud Viewer");
	// viewer5.showCloud(topCluster);
	// while (!viewer5.wasStopped())
	// {
	// }

	//基于区域生长分割
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointTypeC, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(topCluster);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);
	//若一个物体拍到了多个面，分割正面与斜面
	pcl::RegionGrowing<PointTypeC, pcl::Normal> reg;
	reg.setMinClusterSize(200);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(topCluster);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(0.1);
	std::vector <pcl::PointIndices> TopClustersIndices;
	reg.extract(TopClustersIndices);

	// std::cout << "region total:" << TopClustersIndices.size() << std::endl;

	//从分割出来的面中找到最高的面
	int maxSizeIndex = 0;
	double maxSizeValue = 1000;
	int topIndex2 = 0;
	double topZValue2 = 1000;
	pcl::PointCloud<PointTypeC>::Ptr maxSizeCluster(new pcl::PointCloud<PointTypeC>);

	for (std::vector<pcl::PointIndices>::const_iterator it = TopClustersIndices.begin(); it != TopClustersIndices.end(); ++it)
	{
		
		pcl::PointCloud<PointTypeC>::Ptr cloud_cluster(new pcl::PointCloud<PointTypeC>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(topCluster->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		
		PointTypeC minp1, maxp1;
		pcl::getMinMax3D(*cloud_cluster, minp1, maxp1);
		if (maxp1.z < topZValue2)
		{
			topIndex2 = j;
			topZValue2 = maxp1.z;
			maxSizeCluster = cloud_cluster;
		}
		// pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		// viewer.showCloud(cloud_cluster);
		// while (!viewer.wasStopped())
		// {
		// }
		// system("pause");
	}

	// std::cout << maxSizeCluster->size() << std::endl;

	//基于颜色区域生长
	pcl::RegionGrowingRGB<PointTypeC> reg1;
	reg1.setInputCloud(maxSizeCluster);
	// reg1.setIndices (indices);
	reg1.setSearchMethod(tree);
	reg1.setDistanceThreshold(1.0f);
	reg1.setPointColorThreshold(1);
	reg1.setRegionColorThreshold(3);
	reg1.setMinClusterSize(1200);
	std::vector <pcl::PointIndices> ColorClusters;
	reg1.extract(ColorClusters);

	std::cout << "color region total:" << ColorClusters.size() << std::endl;
	maxSizeIndex = 0;
	maxSizeValue = -100;

	pcl::PointCloud<PointTypeC>::Ptr maxSizeClusterColor(new pcl::PointCloud<PointTypeC>);
	std::vector<pcl::PointIndices>::const_iterator it = ColorClusters.begin();
	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)//���ñ�����Ƶ���������		
		maxSizeClusterColor->points.push_back(maxSizeCluster->points[*pit]); //*
	maxSizeClusterColor->width = maxSizeClusterColor->points.size();
	maxSizeClusterColor->height = 1;
	maxSizeClusterColor->is_dense = true;

	

	//PCA求姿态
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*maxSizeClusterColor,pcaCentroid);  //计算质心
	Eigen::Matrix3f covariance;	
	pcl::computeCovarianceMatrixNormalized(*maxSizeClusterColor,pcaCentroid,covariance); //计算协防差矩阵
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

	if (eigenVectorsPCA(2,0) < 0)
    {       
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0); 
    }
    else
    {
        eigenVectorsPCA.col(2) = -eigenVectorsPCA.col(0); 
    }
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(1);
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	
	
	// 转到参考坐标系，将点云主方向与参考坐标系的坐标轴进行对齐
	Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());
	transformation.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();										// R^(-1) = R^T
	transformation.block<3, 1>(0, 3) = -1.f * (transformation.block<3, 3>(0, 0) * pcaCentroid.head<3>());	// t^(-1) = -R^T * t

	pcl::PointCloud<PointTypeC> transformed_cloud;	// 变换后的点云
	pcl::transformPointCloud(*maxSizeClusterColor, transformed_cloud, transformation);

	float temp_area = 100000;
	Eigen::Matrix4f final_tm;
	Eigen::Affine3f transform_0 = Eigen::Affine3f::Identity();
	transform_0.rotate(Eigen::AngleAxisf(M_PI/180, Eigen::Vector3f::UnitZ())); 

	for(int i=0;i<90;i++)
	{
		pcl::transformPointCloud(*maxSizeClusterColor, transformed_cloud, transformation);
		PointTypeC min_pt, max_pt;						// 沿参考坐标系坐标轴的边界值
		pcl::getMinMax3D(transformed_cloud, min_pt, max_pt);

		float width  = fabs(max_pt.x - min_pt.x);
		float height = fabs(max_pt.y - min_pt.y);
		float area = width * height;

		if(area < temp_area)
		{
			temp_area = area;
			final_tm = transformation;
		}

		transformation = transform_0.matrix() * transformation;
	}

	Eigen::Matrix4f final_tm_inv = Eigen::Matrix4f::Identity();
	final_tm_inv = final_tm.inverse();

	PointTypeC min_pt, max_pt;	
	pcl::transformPointCloud(*maxSizeClusterColor, transformed_cloud, final_tm);
	pcl::getMinMax3D(transformed_cloud, min_pt, max_pt);
	Eigen::Vector3f center_temp,center;
	center_temp = 0.5*(min_pt.getVector3fMap() + max_pt.getVector3fMap());
	 
	Eigen::Affine3f tm_inv_aff(final_tm_inv);
	pcl::transformPoint(center_temp, center, tm_inv_aff);
	
	eigenVectorsPCA = final_tm_inv.block<3, 3>(0, 0);



	// PointTypeC min_pt, max_pt;						// 沿参考坐标系坐标轴的边界值
	// pcl::getMinMax3D(transformed_cloud, min_pt, max_pt);
	// const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());	// 形心


	// // 参考坐标系到主方向坐标系的变换关系
	// const Eigen::Quaternionf qfinal(eigenVectorsPCA);
	// const Eigen::Vector3f tfinal = eigenVectorsPCA * mean_diag + pcaCentroid.head<3>();

	// // 显示点云主方向
	// Eigen::Vector3f whd;		// 3个方向尺寸：宽高深
	// whd = max_pt.getVector3fMap() - min_pt.getVector3fMap();// getVector3fMap:返回Eigen::Map<Eigen::Vector3f> 
	// float scale = (whd(0) + whd(1) + whd(2)) / 3;	

	pcl::PointXYZ cp;			// 箭头由质心分别指向pirncipal_dir_X、pirncipal_dir_Y、pirncipal_dir_Z
	cp.x = center(0);
	cp.y = center(1);
	cp.z = center(2);

	pcl::PointXYZ principal_dir_X;
	principal_dir_X.x = 10 * eigenVectorsPCA(0, 0) + cp.x;
	principal_dir_X.y = 10 * eigenVectorsPCA(1, 0) + cp.y;
	principal_dir_X.z = 10 * eigenVectorsPCA(2, 0) + cp.z;

	pcl::PointXYZ principal_dir_Y;
	principal_dir_Y.x = 10 * eigenVectorsPCA(0, 1) + cp.x;
	principal_dir_Y.y = 10 * eigenVectorsPCA(1, 1) + cp.y;
	principal_dir_Y.z = 10 * eigenVectorsPCA(2, 1) + cp.z;

	pcl::PointXYZ principal_dir_Z;
	principal_dir_Z.x = 10 * eigenVectorsPCA(0, 2) + cp.x;
	principal_dir_Z.y = 10 * eigenVectorsPCA(1, 2) + cp.y;
	principal_dir_Z.z = 10 * eigenVectorsPCA(2, 2) + cp.z;

	// 显示结果
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud);
	viewer.addCoordinateSystem();
	viewer.addArrow(principal_dir_X, cp, 1.0, 0.0, 0.0, false, "arrow_x");		// 箭头附在起点上
	viewer.addArrow(principal_dir_Y, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	viewer.addArrow(principal_dir_Z, cp, 0.0, 0.0, 1.0, false, "arrow_z");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	
	end = clock();
	std::cout << "all time:" << ((double)(end-start)/CLOCKS_PER_SEC) * 1000 << std::endl;
    return 0;

}
