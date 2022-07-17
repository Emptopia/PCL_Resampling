//���������ؽ�
//Դ�ԣ�https://blog.csdn.net/qq_36686437/article/details/107391813

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("data/cube_filtered.pcd", *cloud) == -1)
	{
		PCL_ERROR("Could not read pcd file!\n");

	}

	//----------------------------------���߹���------------------------------------
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	n.compute(*normals);
	//-------------------------------���ӷ��ߺ�����---------------------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//---------------------------------�����ؽ�-------------------------------------
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	pcl::Poisson<pcl::PointNormal> pn;
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
	pn.setDepth(6);              // ���ý����ڱ����ؽ�������������
	pn.setMinDepth(2);
	pn.setScale(1.25);           // ���������ؽ����������ֱ���������ı߽�������ֱ���ı�ֵ
	pn.setSolverDivide(3);       // ���ÿ��˹-���¶�������������������˹���̵���ȡ�
	pn.setIsoDivide(6);          // ���ÿ�ȱ�����ȡ��������ȡ�ȱ�������
	pn.setSamplesPerNode(10);    // ����ÿ���˲����ڵ������ٲ�������Ŀ
	pn.setConfidence(false);     // �������ű�־��Ϊtrueʱ��ʹ�÷�������������Ϊ���Ŷ���Ϣ��false����Ҫ�Է��߽��й�һ������
	pn.setManifold(false);       // �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	pn.setOutputPolygons(false); // �����Ƿ����Ϊ�����(���������ǻ��н�������Ľ��)��

	//--------------------------------�����ؽ����-----------------------------------
	pcl::PolygonMesh mesh;
	pn.performReconstruction(mesh);
	// pcl::io::savePLYFile("object_mesh.ply", mesh);
	 //------------------------------���ӻ��ؽ����----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("���������ؽ�");
	viewer->addPolygonMesh(mesh, "my");
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return (0);
}

