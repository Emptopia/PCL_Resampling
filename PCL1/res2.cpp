//̰��ͶӰ���ǻ��㷨
//Դ�ԣ�https://blog.csdn.net/qq_33933704/article/details/78735797
//�޸ģ�56/57��	ͬres1.cpp
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
int main(int argc, char** argv)
{
	//���ļ�
	PointCloudT::Ptr cloud(new PointCloudT);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/bunny.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file1 \n");
		return (-1);
	}
	// ���Ʒ�����
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); //���㷨�ߣ�����洢��normals��
	//* normals ����ͬʱ������ķ������ͱ��������

	//�����ƺͷ��߷ŵ�һ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	//����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//��ʼ��GreedyProjectionTriangulation���󣬲����ò���
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//����������������ڴ洢���
	pcl::PolygonMesh triangles;

	//����GreedyProjectionTriangulation����Ĳ���
	//��һ������Ӱ��ܴ�
	gp3.setSearchRadius(0.015f); //�������ӵ�֮��������루���߳�������ȷ��k���ڵ���뾶��Ĭ��ֵ 0��
	gp3.setMu(2.5f); //��������ھ���ĳ��ӣ��Եõ�ÿ��������������뾶��Ĭ��ֵ 0��
	gp3.setMaximumNearestNeighbors(100); //��������������ڵ���������
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees��pi�����ƽ���
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees ÿ�����ǵ���С�Ƕ�
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees ÿ�����ǵ����Ƕ�
	gp3.setNormalConsistency(false); //���������һ�£�����Ϊtrue

	//���������������������
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);

	//ִ���ع������������triangles��
	gp3.reconstruct(triangles);

	//��������ͼ
	pcl::io::savePLYFile("result.ply", triangles);

	// Additional vertex information
	//std::vector<int> parts = gp3.getPartIDs();
	//std::vector<int> states = gp3.getPointStates();

	// ��ʾ���ͼ
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //���ñ���
	viewer->addPolygonMesh(triangles, "my"); //������ʾ������
	viewer->addCoordinateSystem(1.0); //��������ϵ
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}
