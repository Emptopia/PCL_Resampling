//̰��ͶӰ���ǻ��㷨
//Դ�ԣ�https://blog.csdn.net/qq_41843732/article/details/82115791
//�޸ģ�62/64��	ͬres1.cpp
//���з�ʽ��ĩβ
#include<iostream>
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�
#include <pcl/search/kdtree.h>//kdtree����������ඨ���ͷ�ļ�
#include <pcl/features/normal_3d.h>//������������������ඨ���ͷ�ļ�
#include <pcl/surface/gp3.h> //̰��ͶӰ���ǻ��㷨�ඨ���ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <tchar.h>
int _tmain(int argc, _TCHAR* argv[])
{
	//������һ���򵥵�plyת����pcd��ʽ���ƵĴ��룬���ڿ�ֱ����
/*	pcl::PLYReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudyjp(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read<pcl::PointXYZ>("F:/vs2013wenjian/612/testPCL2/testPCL2/blade.ply", *cloudyjp);
	pcl::io::savePCDFile("H:/out/blade.pcd", *cloudyjp);*/


	// ��������������ĺϷ���  
	if (argc != 2)  //���ֻ��һ������˵��û��ָ��Ŀ����ƣ����Ի���ʾ�÷�
	{
		PCL_ERROR("��������\n");
		getchar();
		return(-1);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // �������ƣ�ָ�룩
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) < 0) //* ����PCD��ʽ���ļ�������ļ������ڣ�����-1
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //�ļ�������ʱ�����ش�����ֹ����
		getchar();
		return (-1);
	}
	std::cerr << "���ƶ���   ���" << std::endl;

	//���ڱ�����ʹ�õ����ǻ��㷨�������Ϊ������ƣ�
	//������Ҫʹ��PCL�еķ��߹��Ʒ���Ԥ�ȹ��Ƴ�������ÿ����ķ��ߣ�����Ĵ������ɸ�Ԥ����
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��
	tree->setInputCloud(cloud);                        //��cloud����tree����
	n.setInputCloud(cloud);                            //Ϊ���߹��ƶ��������������
	n.setSearchMethod(tree);                          //������������
	n.setKSearch(20);                                 //����k������kֵΪ20
	n.compute(*normals);                              //���Ʒ��ߴ洢�����normals��
	std::cerr << "���߼���   ���" << std::endl;
	//����XYZ�����ֶκͷ����ֶ���Ҫ����ͬPointCloud�����У�
	//���Դ���һ���µ�PointNormal���͵ĵ������洢�����ֶκͷ������Ӻ�ĵ��ơ�
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);//�����ֶΣ�cloud_with_normals�洢�������

	//�����ǻ�������ر������ж���
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//��������������
	tree2->setInputCloud(cloud_with_normals);                //���õ��ƹ���������
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//�������ǻ�����
	pcl::PolygonMesh triangles;                               //�洢�������ǻ�������ģ��

	//�������ò�������ֵ��ʵ�ʽ������ǻ�
	gp3.setSearchRadius(0.025);//�������ӵ�֮��������루��Ϊ���������߳���Ϊ0.025
	//���ø���������ֵ�������С��ǰ���ֶԲ������õ�����
	gp3.setMu(2.5);//���ñ��������������ڽ������Զ����Ϊ2.5��Ϊ����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100); //������������������������Ϊ100
	gp3.setMaximumSurfaceAngle(M_PI / 2);   //����ĳ�㷨�߷���ƫ�������㷨�߷�������Ƕ�Ϊ45��
	gp3.setMinimumAngle(M_PI / 18);         //�������ǻ���õ��������ڽ���С�Ƕ�Ϊ10��
	gp3.setMaximumAngle(2 * M_PI / 3);        //�������ǻ���õ��������ڽ����Ƕ�Ϊ120��
	gp3.setNormalConsistency(false);       //���øò�����֤���߳���һ��
	gp3.setInputCloud(cloud_with_normals);//�����������Ϊ�������cloud_with_normals�������Լ�������Ϣ
	gp3.setSearchMethod(tree2);           //����������ʽΪtree2
	gp3.reconstruct(triangles);           //�ؽ���ȡ���ǻ�����ʼ�ؽ�
	std::cerr << "�ؽ�   ���" << std::endl;
	std::cerr << "��ʼ��ʾ ........" << std::endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0.5);  //���ô�����ɫ
	viewer->addPolygonMesh(triangles, "my");  //������Ҫ��ʾ���������
	//��������ģ����ʾģʽ   
	//viewer->setRepresentationToPointsForAllActors(); //����ģ���Ե���ʽ��ʾ  
	viewer->setRepresentationToWireframeForAllActors();  //����ģ�����߿�ͼģʽ��ʾ

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer2->setBackgroundColor(0, 0.5, 0);  //���ô�����ɫ������
	viewer2->addPolygonMesh(triangles, "my2");  //������Ҫ��ʾ���������
	viewer2->setRepresentationToSurfaceForAllActors(); //����ģ������Ƭ��ʽ��ʾ
	//viewer->addCoordinateSystem(0.1);  //��������ϵ,����Ϊ������ʾ�ߴ�
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	while (!viewer2->wasStopped())
	{
		viewer2->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}
//F:/vs2013wenjian/824/TEST824/x64/Debug/TEST824.exe H:/out/bunny.pcd
//�ڿ�ʼ-cmd�س�-����"exe�ļ��ĵ�ַ pcd��ʽ���Ƶĵ�ַ"�س��Ϳ���������������