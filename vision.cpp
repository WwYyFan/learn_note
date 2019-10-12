#include "gui/vision.h"
pclVision::pclVision()
{

}

pclVision::~pclVision()
{

}

pcl::PointCloud<PointTypeC>::Ptr pclVision::obj2pcd(char* filename)
{
  char line[100];
  char mark[5];
  float x,y,z,r,g,b;

  pcl::PointCloud<PointTypeC>::Ptr cloud (new pcl::PointCloud<PointTypeC>());
  FILE* fp = fopen(filename, "r");
  // V.push_back(Point(0,0,0));

  if(!fp){
    cout<< "Couldn't open the file!"<<endl;
    exit(1);
  }
  int i = 0;
  while(fgets(line, 100, fp))
  {
	// if (i%2 != 1)
	// {
	// 	i++;
	// 	continue;
	// }
    if (line[0] == 'v') 
    {
        sscanf(line, "%s %f %f %f %f %f %f", mark, &x, &y, &z, &r, &g, &b);
        // cout<< x << y << z << r << g << b <<endl;
        PointTypeC pointsrgb;
        pointsrgb.x = x;
        pointsrgb.y = y;
        pointsrgb.z = z;
        pointsrgb.r = r;
        pointsrgb.g = g;
        pointsrgb.b = b;
        cloud->points.push_back(pointsrgb);
        // cout<< "fgfg"<<endl;
    }
	i++;
  }
  cout << "finished!" << endl;
  cloud->width = cloud->points.size ();
  cloud->height = 1;
  cloud->is_dense = true;
  fclose(fp);
  return cloud;
}

int pclVision::findMovePose(pcl::PointCloud<PointTypeC>::Ptr &transformed_cloud, pcl::PointCloud<PointTypeC>::Ptr &cloud_filtered)
{

    clock_t start, end, timespend,timespendall;

	start = clock();
	pcl::PointCloud<PointTypeC>::Ptr cloud(new pcl::PointCloud<PointTypeC>);
	pcl::PointCloud<PointTypeC>::Ptr rgbcloud(new pcl::PointCloud<PointTypeC>);

	rgbcloud = obj2pcd("/home/user/bin/4D_SLL/data/temp/caches/view00/debug-view00.obj");

	timespend = clock() - start;
	timespendall =timespend;
	cout << "load pcd time:" << (double)timespend / CLOCKS_PER_SEC << endl;

	cloud = rgbcloud;
	
	//将点云转到机械臂坐标系
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	
	Json::Value root;
	Json::Reader reader;
	Json::Value array;
	Json::Value value1;
	std::ifstream ifs("/home/user/bin/4D_SLL/data/temp/capture/analysis/DetectARMarker.json");//open file example.json
	
	if (!reader.parse(ifs, root))
	{
		// fail to parse
		 printf("Fail to parse");
		return 0;
	}
	else
	{
		// success
		array = root["mTransformAxisMatrix"];
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if (i != 3 && j == 3)
					// transform_1 (i,j)=array[i][j].asFloat() - originPoint[i];
					transform_1(i, j) = array[i][j].asFloat();
				else
					transform_1(i, j) = array[i][j].asFloat();
			}
		}
	}

	// Executing the transformation
	// pcl::PointCloud<PointTypeC>::Ptr transformed_cloud(new pcl::PointCloud<PointTypeC>());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);
	// pcl::io::savePCDFileASCII("K326-DD-REDUCE_NOISE.pcd",*transformed_cloud);
	cloud = transformed_cloud;

	timespend = clock() - start - timespendall;
	timespendall = timespendall + timespend;
	cout<<"transformation finished!  耗时："<<(double) timespend / CLOCKS_PER_SEC<<endl;

	//分割桌面，只留下物体
	/************************/
	// pcl::PointCloud<PointTypeC>::Ptr cloud_filtered(new pcl::PointCloud<PointTypeC>());
	double minplane = -0.330;
	double h = 0.2;
	pcl::PassThrough<PointTypeC> pass;
	pass.setInputCloud(transformed_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(minplane, minplane + h);
	pass.setFilterLimitsNegative(false);
	pass.filter(*cloud_filtered);
	cloud = cloud_filtered;

	timespend = clock() - start - timespendall;
	timespendall = timespendall + timespend;
	cout<<"PassThrough finished!  耗时："<<(double) timespend / CLOCKS_PER_SEC<<endl;

	//下采样，降低点云数量
	// ... and downsampling the point cloud
	//const float voxel_grid_size = 0.0005f;
	const float voxel_grid_size = 0.0005f;
	pcl::VoxelGrid<PointTypeC> vox_grid;
	vox_grid.setInputCloud(cloud_filtered);
	vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
	//vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
	pcl::PointCloud<PointTypeC>::Ptr tempCloud(new pcl::PointCloud<PointTypeC>);
	vox_grid.filter(*tempCloud);
	cloud = tempCloud;

	timespend = clock() - start - timespendall;
	timespendall = timespendall + timespend;
	cout<<"downsampling finished!  耗时："<<(double) timespend / CLOCKS_PER_SEC<<endl;

	//// RadiusOutlierRemoval fielter
	//// pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType> ());
	//start = clock();
	//pcl::RadiusOutlierRemoval<PointTypeC> outrem;
	//// build the filter
	//outrem.setInputCloud(cloud);
	//outrem.setRadiusSearch(0.003);
	//outrem.setMinNeighborsInRadius(25);
	//// apply filter
	//outrem.filter(*cloud_filtered);
	//cloud = cloud_filtered;
	//TRACE("RadiusOutlierRemoval finished!\n");
	//end = clock();
	//TRACE("RadiusOutlierRemoval pcd time %f\n", (double)(end - start) / CLOCKS_PER_SEC);

	//分割出一个一个的物体聚类
	// ����������ȡ����������kdtree������
	pcl::search::KdTree<PointTypeC>::Ptr tree(new pcl::search::KdTree<PointTypeC>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointTypeC> ec;   //ŷʽ�������
	ec.setClusterTolerance(0.001);                     // ���ý��������������뾶Ϊ2cm
	ec.setMinClusterSize(200);                 //����һ��������Ҫ�����ٵĵ���ĿΪ100
	ec.setMaxClusterSize(250000);               //����һ��������Ҫ��������ĿΪ25000
	ec.setSearchMethod(tree);                    //���õ��Ƶ���������
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);           //�ӵ�������ȡ���࣬������������������cluster_indices��
	std::cout << "按照点数提取聚类完毕！" << std::endl;
	std::cout << "cluster_indices :" <<cluster_indices.size() << std::endl;
	if (cluster_indices.size() == 0)
	{
		std::cout << "未发现目标！" << std::endl;
		return 0;
	}

	timespend = clock() - start - timespendall;
	timespendall = timespendall + timespend;
	cout<<"EuclideanClusterExtraction finished!  耗时："<<(double) timespend / CLOCKS_PER_SEC<<endl;

	//找到最高的物体聚类
	int topIndex = 0;
	double topZValue = -100;
	pcl::PointCloud<PointTypeC>::Ptr topCluster(new pcl::PointCloud<PointTypeC>);
	//�ҳ���������
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<PointTypeC>::Ptr cloud_cluster(new pcl::PointCloud<PointTypeC>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		j++;
		// Eigen::Vector4f pcaCentroid;
		PointTypeC minp1, maxp1;
		// pcl::compute3DCentroid(*cloud_cluster, pcaCentroid);
		pcl::getMinMax3D(*cloud_cluster, minp1, maxp1);
		//���zֵ���ľ��������ֵ
		if (maxp1.z > topZValue)
		{
			topIndex = j;
			topZValue = maxp1.z;
			topCluster = cloud_cluster;
		}
	}
	timespend = clock() - start - timespendall;
	timespendall = timespendall + timespend;
	cout<<"寻找最高聚类 finished!  耗时："<<(double) timespend / CLOCKS_PER_SEC<<endl;
	cout<<"剩余点："<<topCluster->points.size()<<endl;


	/*******�ָͬ��ĵ���,��ȡ�������ľ���**********/
	// pcl::search::Search<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
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

	//从分割出来的面中找到最高的面
	int maxSizeIndex = 0;
	double maxSizeValue = -100;
	int topIndex2 = 0;
	double topZValue2 = -100;
	pcl::PointCloud<PointTypeC>::Ptr maxSizeCluster(new pcl::PointCloud<PointTypeC>);

	for (std::vector<pcl::PointIndices>::const_iterator it = TopClustersIndices.begin(); it != TopClustersIndices.end(); ++it)
	{
		//���������еĵ��Ƶ����������ҷֿ����������ĵ���
		pcl::PointCloud<PointTypeC>::Ptr cloud_cluster(new pcl::PointCloud<PointTypeC>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(topCluster->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// if (cloud_cluster->points.size()  > maxSizeValue)
		// {
		// 	maxSizeValue = cloud_cluster->points.size();
		// 	maxSizeCluster = cloud_cluster;
		// }//变更为取最高聚类
		PointTypeC minp1, maxp1;
		// pcl::compute3DCentroid(*cloud_cluster, pcaCentroid);
		pcl::getMinMax3D(*cloud_cluster, minp1, maxp1);
		//���zֵ���ľ��������ֵ
		if (maxp1.z > topZValue2)
		{
			topIndex2 = j;
			topZValue2 = maxp1.z;
			maxSizeCluster = cloud_cluster;
		}
	}

	timespend = clock() - start - timespendall;
	timespendall = timespendall + timespend;
	cout<<"基于区域生长分割  耗时："<<(double) timespend / CLOCKS_PER_SEC<<endl;
	cout<<"剩余点："<<maxSizeCluster->points.size()<<endl;

	//基于颜色分割出靠在一起的两个面
	/***********���ڻҶȷָ���úܽ�������********************/
	//������ɫ���о���
	pcl::RegionGrowingRGB<PointTypeC> reg1;
	reg1.setInputCloud(maxSizeCluster);
	// reg1.setIndices (indices);
	reg1.setSearchMethod(tree);
	reg1.setDistanceThreshold(0.0008);
	reg1.setPointColorThreshold(2);
	reg1.setRegionColorThreshold(10);
	reg1.setMinClusterSize(380);
	std::vector <pcl::PointIndices> ColorClusters;
	reg1.extract(ColorClusters);
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg1.getColoredCloud ();
	maxSizeIndex = 0;
	maxSizeValue = -100;
	pcl::PointCloud<PointTypeC>::Ptr maxSizeClusterColor(new pcl::PointCloud<PointTypeC>);
	std::vector<pcl::PointIndices>::const_iterator it = ColorClusters.begin();
	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)//���ñ�����Ƶ���������		
		maxSizeClusterColor->points.push_back(maxSizeCluster->points[*pit]); //*
	maxSizeClusterColor->width = maxSizeClusterColor->points.size();
	maxSizeClusterColor->height = 1;
	maxSizeClusterColor->is_dense = true;
	
	timespend = clock() - start - timespendall;
	timespendall = timespendall + timespend;
	cout<<"颜色分割  耗时："<<(double) timespend / CLOCKS_PER_SEC<<endl;
	cout<<"剩余点："<<maxSizeClusterColor->points.size()<<endl;
	cout<< "in" <<endl;

	//移除噪点
	/***** RadiusOutlierRemoval ����******/
	pcl::RadiusOutlierRemoval<PointTypeC> outrem1;
	// build the filter
	outrem1.setInputCloud(maxSizeClusterColor);
	outrem1.setRadiusSearch(0.0030);
	outrem1.setMinNeighborsInRadius(2);
	// apply filter
	outrem1.filter(*cloud_filtered);
    
	cout << "in \n" << endl;
	timespend = clock() - start - timespendall;
	timespendall = timespendall + timespend;
	cout<<"RadiusOutlierRemoval finished!  耗时："<<(double) timespend / CLOCKS_PER_SEC<<endl;
	cout<<"剩余点："<<cloud_filtered->points.size()<<endl;
	cout << "out \n" << endl;
	
	//将点云转到坐标系原点，然后通过旋转点云，求出最小boundingbox，从而获得物体的抓取位姿
	pcl::PointCloud<PointTypeC>::Ptr cloud_cluster(new pcl::PointCloud<PointTypeC>);
	cloud_cluster = cloud_filtered;
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_cluster, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_cluster, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA  = eigen_solver.eigenvalues();
	// eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校准主方向
	// eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	// eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    //将朝上的设为z轴
	Eigen::Matrix3f eigenVectorsPCA_temp = Eigen::Matrix3f::Identity();
    if (eigenVectorsPCA(2,0) > 0)
    {       
        eigenVectorsPCA_temp.col(2) = eigenVectorsPCA.col(0); 
    }
    else
    {
        eigenVectorsPCA_temp.col(2) = -eigenVectorsPCA.col(0); //
    }
    eigenVectorsPCA_temp.col(1) = eigenVectorsPCA.col(1);
    eigenVectorsPCA_temp.col(0) = eigenVectorsPCA_temp.col(1).cross(eigenVectorsPCA_temp.col(2));
	eigenVectorsPCA = eigenVectorsPCA_temp;

	Eigen::Matrix4f tm 	   = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   // R^(-1) = R^T
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>()); // t^(-1) = -R^T * t

	Eigen::Affine3f transform_0 = Eigen::Affine3f::Identity();
	Eigen::Matrix4f transform_tm = tm;
	float theta = M_PI / 180; // The angle of rotation in radians around Z
	transform_0.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ())); //Χ��z����ת
	pcl::PointCloud<PointTypeC>::Ptr transformedCloud(new pcl::PointCloud<PointTypeC>);
	PointTypeC min_p1, max_p1;
	Eigen::Vector3f c1, c;
	float minArea = 10000;
	for (int i = 0; i < 90; i+=2)
	{
		pcl::transformPointCloud(*cloud_cluster, *transformedCloud, transform_tm);   //转换到参考坐标系
		pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);

		float templ = abs(min_p1.x - max_p1.x);
		float tempw = abs(min_p1.y - max_p1.y);
		float tempArea = templ * tempw;
		if (tempArea < minArea)
		{
			minArea = tempArea;
			tm = transform_tm;
			// std::cout << "minArea:\n" << minArea << std::endl;
		}

		transform_tm = transform_0.matrix()*transform_tm;

	}
	pcl::transformPointCloud(*cloud_cluster, *transformedCloud, tm);
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());
	tm_inv = tm.inverse();

	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);
	
	eigenVectorsPCA = tm_inv.block<3, 3>(0, 0);

	//Eigen::Vector3f originVector(-0.55206, 0.40047, -0.35026);
	//// Eigen::Vector3f oriDirVector(2.902,1.202,0); 
	//Eigen::Vector3f oriDirVector(3.1415926, 0, 0);
	Eigen::Vector3f tarVector(0, 0, 0);
	Eigen::Vector3f tarDirVector(0, 0, 0);	
	// tarVector = originVector + c;
	tarVector = c;

	Eigen::AngleAxisf rotate_vector;
	rotate_vector.fromRotationMatrix(eigenVectorsPCA);
	float ang = rotate_vector.angle();
	Eigen::Vector3f axi = rotate_vector.axis();

	printf("\ntarVector %d = < %0.3f, %0.3f, %0.3f >\n", j, tarVector(0), tarVector(1), tarVector(2));
	//TRACE("\ntarDirVector %d = < %0.3f, %0.3f, %0.3f >\n", j, tarDirVector(0), tarDirVector(1), tarDirVector(2));
	printf("\ntarDirVector %d = < %0.3f, %0.3f, %0.3f >\n", j, ang * axi(0), ang * axi(1), ang * axi(2));
	tarDirVector(0) = ang * axi(0);
	tarDirVector(1) = ang * axi(1);
	tarDirVector(2) = ang * axi(2);

	timespend = clock() - start - timespendall;
	timespendall = timespendall + timespend;
	cout<<"位姿计算 finished!  耗时："<<(double) timespend / CLOCKS_PER_SEC<<endl;

	end = clock() - start;
	printf("all time %f\n", (double)end / CLOCKS_PER_SEC);

	// pcl::PCDWriter writer;
	// stringstream ss;
	// ss << "/home/user/projects/4D_SLL/lib/Release/x64/data/temp/capture/model/" << i << ".pcd"
	// string filename;
	// ss >> filename;
	//// string filename1 = "/home/user/projects/4D_SLL/lib/Release/x64/data/temp/capture/model/cloud_filtered.pcd";
  	// writer.write(filename,*transformed_cloud);
  
	// pcl::io::savePCDFileASCII(filename1,*cloud_filtered);
	
	ofstream in;
	in.open("/home/user/bin/4D_SLL/data/temp/capture/model/points.txt",ios::trunc);
	in << c(0) << " " << c(1) << " " << c(2) << " " << "\n";
	in << (c(0) + 0.05*eigenVectorsPCA(0, 0)) << " " << (c(1) + 0.05*eigenVectorsPCA(1, 0)) << " " << (c(2) + 0.05*eigenVectorsPCA(2, 0)) << "\n";
	in << (c(0) + 0.05*eigenVectorsPCA(0, 1)) << " " << (c(1) + 0.05*eigenVectorsPCA(1, 1)) << " " << (c(2) + 0.05*eigenVectorsPCA(2, 1)) << "\n";
	in << (c(0) + 0.05*eigenVectorsPCA(0, 2)) << " " << (c(1) + 0.05*eigenVectorsPCA(1, 2)) << " " << (c(2) + 0.05*eigenVectorsPCA(2, 2)) << "\n";
	in.close();
	
	// //���ӻ�
	// pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	// //pcl::visualization::PointCloudColorHandlerCustom<PointTypeC> color_handler(tempCloud, 255, 255, 0);
	// pcl::visualization::PointCloudColorHandlerCustom<PointTypeC> model_color_handler(cloud_cluster, 255, 0, 0);
	// viewer.addPointCloud(tempCloud,  "scene_cloud");//���ӻ���������
	// viewer.addPointCloud(cloud_cluster, model_color_handler, "m_cloud");//���ӻ���������

	// pcl::PointXYZ  pointa(c(0), c(1), c(2));
	// pcl::PointXYZ  pointx(c(0) + 0.05*eigenVectorsPCA(0, 0), c(1) + 0.05*eigenVectorsPCA(1, 0), c(2) + 0.05*eigenVectorsPCA(2, 0));
	// pcl::PointXYZ  pointy(c(0) + 0.05*eigenVectorsPCA(0, 1), c(1) + 0.05*eigenVectorsPCA(1, 1), c(2) + 0.05*eigenVectorsPCA(2, 1));
	// pcl::PointXYZ  pointz(c(0) + 0.05*eigenVectorsPCA(0, 2), c(1) + 0.05*eigenVectorsPCA(1, 2), c(2) + 0.05*eigenVectorsPCA(2, 2));
	// // TRACE(" Az ~~~~~~~~ %f , %f , %f\n", eigenVectorsPCA(0, 0), eigenVectorsPCA(1, 0), eigenVectorsPCA(2, 0));
	// // TRACE(" Az ~~~~~~~~ %f , %f , %f\n", eigenVectorsPCA(0, 1), eigenVectorsPCA(1, 1), eigenVectorsPCA(2, 1));
	// // TRACE(" Az ~~~~~~~~ %f , %f , %f\n", eigenVectorsPCA(0, 2), eigenVectorsPCA(1, 2), eigenVectorsPCA(2, 2));


	// viewer.addLine<pcl::PointXYZ>(pointa, pointx, 255, 0, 0, "line1");
	// viewer.addLine<pcl::PointXYZ>(pointa, pointy, 0, 255, 0, "line2");
	// viewer.addLine<pcl::PointXYZ>(pointa, pointz, 0, 0, 255, "line3");
	// while (!viewer.wasStopped())
	// {
	// 	viewer.spinOnce(100);
	// // 	boost::this_thread::sleep (boost::posix_time::microseconds (10));
	// }
	
	//最后的抓取位姿
	grasppoint.x = tarVector(0);
	grasppoint.y = tarVector(1);
	grasppoint.z = tarVector(2);
	grasppoint.rx = tarDirVector(0);
	grasppoint.ry = tarDirVector(1);
	grasppoint.rz = tarDirVector(2);
	printf("%f,%f,%f,%f,%f,%f~~~~~~~~~~~\n", grasppoint.x, grasppoint.y, grasppoint.z, grasppoint.rx, grasppoint.ry, grasppoint.rz);
    return 1;
}



// void show()
// {
// 	/********识别***************/
// 	  // Point clouds
// 	pcl::PointCloud<PointTypeC>::Ptr object(new pcl::PointCloud<PointTypeC>);
// 	pcl::PointCloud<PointTypeC>::Ptr object_aligned(new pcl::PointCloud<PointTypeC>);
// 	pcl::PointCloud<PointTypeC>::Ptr scene(new pcl::PointCloud<PointTypeC>);

// 	FeatureCloudT::Ptr object_features (new FeatureCloudT);
// 	FeatureCloudT::Ptr scene_features (new FeatureCloudT);
// 	std::vector<pcl::PointCloud<PointTypeC> > object_templates;
// 	const float voxel_grid_size2 = 0.001f;
// 	pcl::VoxelGrid<PointTypeC> vox_grid2;
// 	vox_grid2.setInputCloud(cloud_filtered);
// 	vox_grid2.setLeafSize(voxel_grid_size2, voxel_grid_size2, voxel_grid_size2);
// 	vox_grid2.filter(*scene);

// 	// Load the object templates specified in the object_templates.txt file
// 	std::ifstream input_stream ("/home/user/ur_model/modeltest/model/object_templates.txt");
// 	object_templates.resize (0);
// 	std::string pcd_filename;
// 	while (input_stream.good ())
// 	{	
// 		std::getline (input_stream, pcd_filename);
// 		if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
// 		continue;
// 		if(pcl::io::loadPCDFile (pcd_filename, *object) < 0)
// 		{
// 		pcl::console::print_error ("Error loading object/scene file!\n");
// 		return (1);
// 		}
// 		std::cout << pcd_filename << std::endl;
// 		std::cout << object->size() << std::endl;
// 		object_templates.push_back (*object);
// 	}
// 	input_stream.close ();

// 	float lowestScore = 1000;
// 	int ObjectID = -1;//-1:没找到 0：长圆柱边 1：短圆柱边 2：长矩形 3：短圆柱顶圆 4：短矩形
   
// 	pcl::PointCloud<PointTypeC>::Ptr Final(new pcl::PointCloud<PointTypeC>);
// 	for(int i = 0; i < object_templates.size(); i++)
//     {
//       *object = object_templates[i];
//       float rate = (float)(object->size()) / (float)(scene->size());
//       if (rate < 0.7 || rate > 3) {//如果模板比场景聚类少太多或者太少，就略过 
//         continue;
//       }
// 	  //法向量
// 	  pcl::PointCloud <pcl::Normal>::Ptr scene_normals (new pcl::PointCloud <pcl::Normal>);
// 	  pcl::PointCloud <pcl::Normal>::Ptr object_normals (new pcl::PointCloud <pcl::Normal>);
// 	  pcl::NormalEstimationOMP<PointTypeC,pcl::Normal> nest;
// 	  nest.setRadiusSearch (0.02);//0.01
// 	  nest.setInputCloud (scene);
// 	  nest.compute (*scene_normals);
// 	  nest.setInputCloud (object);	
// 	  nest.compute (*object_normals);
//       // Estimate features
//       pcl::console::print_highlight ("Estimating features...\n");
//       FeatureEstimationT fest;
//       fest.setRadiusSearch (0.015);//0.025
//       fest.setInputCloud (object);
//       fest.setInputNormals (object_normals);
//       fest.compute (*object_features);
//       fest.setInputCloud (scene);
//       fest.setInputNormals (scene_normals);
//       fest.compute (*scene_features);
      
//       // Perform alignment
//       pcl::console::print_highlight ("Starting alignment...\n");
//       pcl::SampleConsensusPrerejective<PointTypeC,PointTypeC,FeatureT> align;
//     //   pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
//       align.setInputSource (object);
//       align.setSourceFeatures (object_features);
//       align.setInputTarget (scene);
//       align.setTargetFeatures (scene_features);
//       align.setMaximumIterations (50000); // Number of RANSAC iterations 50000
//       align.setNumberOfSamples (3); //3// Number of points to sample for generating/prerejecting a pose
//       align.setCorrespondenceRandomness (30); // Number of nearest features to use 10
//       align.setSimilarityThreshold (0.90f); // Polygonal edge length similarity threshold 0.92
//       align.setMaxCorrespondenceDistance (2.5f * 0.001); // Inlier threshold 2.5f * 0.015
//       align.setInlierFraction (0.36f); //0.25// Required inlier fraction for accepting a pose hypothesis 0.25 
//       {
//         pcl::ScopeTime t("Alignment");
//         align.align (*object_aligned);
//       }

//       if (align.hasConverged ())
//       {
//         // Print results
//         printf ("\n");
//         Eigen::Matrix4f transformation = align.getFinalTransformation ();
       
//         pcl::console::print_info ("Inliers: %i/%i/%i\n", align.getInliers ().size (), object->size (),scene->size());
//             // Show alignment
//         pcl::IterativeClosestPoint<PointTypeC, PointTypeC> icp;
//         // pcl::IterativeClosestPoint<PointNT, PointNT> icp;
//         icp.setInputSource(object_aligned);
//         icp.setInputTarget(scene);
//               // Set the maximum number of iterations (criterion 1)
//         icp.setMaximumIterations (200);
//         // Set the transformation epsilon (criterion 2)
//         icp.setTransformationEpsilon (1e-12);
//         // Set the euclidean distance difference epsilon (criterion 3)
//         icp.setEuclideanFitnessEpsilon (0.00001);
//         // PointCloudT::Ptr FinalTemp(new PointCloudT);
// 		pcl::PointCloud<PointTypeC>::Ptr FinalTemp(new pcl::PointCloud<PointTypeC>);
//         icp.align(*FinalTemp);
//         std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
//         std::cout << icp.getFinalTransformation() << std::endl;
      
//         if (icp.getFitnessScore() < lowestScore) 
//         {
//           Final = FinalTemp;
//           lowestScore = icp.getFitnessScore();
// 		  ObjectID = i;
//         }
//       }
//       else
//       {
//         pcl::console::print_error ("Alignment failed!\n");
//         // return (1);
//       }
//     }
// 	cout << "ObjectID " << ObjectID << endl;
// 	//0：长圆柱边 1：短圆柱边 2：长矩形 3：短圆柱顶圆 4：短矩形
// 	switch(ObjectID)
// 	{
// 		case -1:
// 			cout<<"识别失败！"<<endl;
// 			break;
// 		case 0:
// 			cout<<"0：长圆柱边"<<endl;
// 			break;
// 		case 1:
// 			cout<<"1：短圆柱边"<<endl;
// 			break;
// 		case 2:
// 			cout<<"2：长矩形"<<endl;
// 			break;
// 		case 3:
// 			cout<<"3：短圆柱顶圆"<<endl;
// 			break;
// 		case 4:
// 			cout<<"4：短矩形"<<endl;
// 			break;
// 		default:
// 			break;
// 	}

// }
