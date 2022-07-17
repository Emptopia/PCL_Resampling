//̰��ͶӰ���ǻ��㷨
//Դ�ԣ�https://blog.csdn.net/qq_36686437/article/details/114171571
//�޸ģ�
//bunny			39��0.015
//						40��2.5
//boolSurface	39��15.0
//						40��5.0
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>//̰��ͶӰ���ǻ��㷨�ඨ���ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;
int main()
{
	// --------------------------------���ص�������---------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("data/cylinderSurface1.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//----------------------------------���߹���-----------------------------------
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//-----------------------------����XYZ�ͷ������ֶ�-----------------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//-------------------------------��������������--------------------------------
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	//-------------------------------̰��ͶӰ���ǻ�--------------------------------
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//�������ǻ�����
	pcl::PolygonMesh triangles;          // �洢�������ǻ�������ģ��
	gp3.setSearchRadius(15.0);          // �������ӵ�֮��������루�������ε����߳���
	gp3.setMu(5.0);                      // ���ñ��������������ٽ������Զ���룬Ϊ����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100); // ������������������������
	gp3.setMaximumSurfaceAngle(M_PI / 4);// ����ĳ�㷨�߷���ƫ�������㷨�߷�������Ƕ�
	gp3.setMinimumAngle(M_PI / 18);      // �������ǻ���õ��������ڽǵ���С�Ƕ�
	gp3.setMaximumAngle(2 * M_PI / 3);   // �������ǻ���õ��������ڽǵ����Ƕ�
	gp3.setNormalConsistency(false);     // ���øò�����֤���߳���һ��

	// Get result
	gp3.setInputCloud(cloud_with_normals);// ��������������ߵĵ���
	gp3.setSearchMethod(tree2);           // ����������ʽ
	gp3.reconstruct(triangles);           // �ؽ���ȡ���ǻ�
	//cout << triangles;
	pcl::io::saveVTKFile("mesh.vtk", triangles);
	//--------------------------------���Ӷ�����Ϣ-------------------------------
	vector<int> parts = gp3.getPartIDs();
	vector<int> states = gp3.getPointStates();
	//---------------------------------������ӻ�--------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int v1(0), v2(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, color_in, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", v1);

	viewer->addPolygonMesh(triangles, "my", v2);

	viewer->addCoordinateSystem(0.2);
	//viewer->initCameraParameters ();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}

