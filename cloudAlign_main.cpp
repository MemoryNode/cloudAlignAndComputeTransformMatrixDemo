#include<iostream>
#include<ctime>
#include<fstream>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/normal_space.h>
#include<pcl/features/normal_3d.h>
#include<pcl/common/transforms.h>
#include<Eigen/Dense>


int alignTest(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt, Eigen::Matrix4f& rt);

void saveRtMatrix(const std::string& filename, const Eigen::Matrix4f& rt);

int main()
{
	clock_t startTime = 0, endTime = 0;
	
	std::string cloudSrcPath = "data/cloud_002.pcd";
	std::string cloudTgtPath = "data/cloud_003.pcd";
	std::string rtMatrixFilePath = "data/RtMatrix.txt";
	std::string alignedCloudFilePath = "data/alignedCloud.pcd";

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTgt(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAligned(new pcl::PointCloud<pcl::PointXYZ>());

	startTime = clock();

	if (-1 == pcl::io::loadPCDFile(cloudSrcPath, *cloudSrc))
	{
		std::cout << "load pcd file failed." << std::endl;
		return -1;
	}
	if (-1 == pcl::io::loadPCDFile(cloudTgtPath, *cloudTgt))
	{
		std::cout << "load pcd file failed." << std::endl;
		return -1;
	}

	endTime = clock();
	std::cout << "load pcd file time cost: " << (endTime - startTime) << std::endl;

	Eigen::Matrix4f rt = Eigen::Matrix4f::Identity();

	clock_t sumTime = 0;

	const int N = 10;

	for (int idx = 0; idx < N; ++idx)
	{
		startTime = clock();

		alignTest(cloudSrc, cloudTgt, rt);

		endTime = clock();

		std::cout << "align time cost: " << (endTime - startTime) << std::endl;
		sumTime += (endTime - startTime);
	}
	
	std::cout << "align time cost average: " << (sumTime/N) << std::endl;

	saveRtMatrix(rtMatrixFilePath, rt);

	pcl::transformPointCloud(*cloudSrc, *cloudAligned, rt);

	pcl::io::savePCDFileBinary(alignedCloudFilePath, *cloudAligned);

	pcl::visualization::PCLVisualizer viewer("show cloud");

	int v1; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
	int v2;
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	viewer.setBackgroundColor(0.0, 0.05, 0.05, v1); //设着两个窗口的背景色
	viewer.setBackgroundColor(0.05, 0.05, 0.05, v2);
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(cloudSrc, 250, 0, 0); //设置源点云的颜色为红色
	viewer.addPointCloud(cloudSrc, sources_cloud_color, "sources_cloud_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(cloudTgt, 0, 250, 0);  //目标点云为绿色
	viewer.addPointCloud(cloudTgt, target_cloud_color, "target_cloud_v1", v1); //将点云添加到v1窗口

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");  //设置显示点的大小
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v1");

	viewer.addPointCloud(cloudAligned, sources_cloud_color, "aligend_cloud_v2", v2);
	viewer.addPointCloud(cloudTgt, target_cloud_color, "target_cloud_v2", v2);

	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	//getchar();
	return 0;
}

void saveRtMatrix(const std::string& filename, const Eigen::Matrix4f& rt)
{
	std::ofstream out(filename, std::ios::out);
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			out << rt(r, c) << ' ';
		}
		out << rt(r, 3) << std::endl;
	}
	out.close();
}

int alignTest(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt, Eigen::Matrix4f& rt)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTgt(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudAligned(new pcl::PointCloud<pcl::PointNormal>());

	const float leafSize = 0.5f;
	pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
	voxelSampler.setLeafSize(leafSize, leafSize, leafSize);

	voxelSampler.setInputCloud(src);
	voxelSampler.filter(*cloudSrc);

	voxelSampler.setInputCloud(tgt);
	voxelSampler.filter(*cloudTgt);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloudSrcWithNormal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudTgtWithNormal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(1.5f);

	ne.setInputCloud(cloudSrc);
	ne.compute(*normals);
	pcl::concatenateFields(*cloudSrc, *normals, *cloudSrcWithNormal);

	ne.setInputCloud(cloudTgt);
	ne.compute(*normals);
	pcl::concatenateFields(*cloudTgt, *normals, *cloudTgtWithNormal);
	
	pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> nss;
	const int binNum = 8;
	const float ratio = 0.5f;
	nss.setBins(binNum, binNum, binNum);
	nss.setKeepOrganized(false);
	nss.setSeed(200);

	nss.setInputCloud(cloudSrcWithNormal);
	nss.setNormals(cloudSrcWithNormal);
	nss.setSample(cloudSrcWithNormal->size()*ratio);
	nss.filter(*cloudSrcWithNormal);

	nss.setInputCloud(cloudTgtWithNormal);
	nss.setNormals(cloudTgtWithNormal);
	nss.setSample(cloudTgtWithNormal->size()*ratio);
	nss.filter(*cloudTgtWithNormal);

	pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(cloudSrcWithNormal);
	icp.setInputTarget(cloudTgtWithNormal);
	icp.setMaxCorrespondenceDistance(1.0f);
	icp.setTransformationEpsilon(0.0001f);
	icp.setEuclideanFitnessEpsilon(0.0001f);
	icp.setMaximumIterations(15);

	icp.align(*cloudAligned);
	
	rt = icp.getFinalTransformation();

	return 0;
}