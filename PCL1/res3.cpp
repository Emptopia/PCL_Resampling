//贪婪投影三角化算法
//源自：https://blog.csdn.net/qq_41843732/article/details/82115791
//修改：62/64行	同res1.cpp
//运行方式见末尾
#include<iostream>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/search/kdtree.h>//kdtree搜索对象的类定义的头文件
#include <pcl/features/normal_3d.h>//法向量特征估计相关类定义的头文件
#include <pcl/surface/gp3.h> //贪婪投影三角化算法类定义的头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <tchar.h>
int _tmain(int argc, _TCHAR* argv[])
{
	//以下是一个简单的ply转换到pcd格式点云的代码，后期可直接用
/*	pcl::PLYReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudyjp(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read<pcl::PointXYZ>("F:/vs2013wenjian/612/testPCL2/testPCL2/blade.ply", *cloudyjp);
	pcl::io::savePCDFile("H:/out/blade.pcd", *cloudyjp);*/


	// 检查程序输入命令的合法性  
	if (argc != 2)  //如果只有一个命令说明没有指定目标点云，所以会提示用法
	{
		PCL_ERROR("输入有误\n");
		getchar();
		return(-1);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) < 0) //* 读入PCD格式的文件，如果文件不存在，返回-1
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
		getchar();
		return (-1);
	}
	std::cerr << "点云读入   完成" << std::endl;

	//由于本例中使用的三角化算法输入必须为有向点云，
	//所以需要使用PCL中的法线估计方法预先估计出数据中每个点的法线，下面的代码就完成该预处理。
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree->setInputCloud(cloud);                        //用cloud构建tree对象
	n.setInputCloud(cloud);                            //为法线估计对象设置输入点云
	n.setSearchMethod(tree);                          //设置搜索方法
	n.setKSearch(20);                                 //设置k搜索的k值为20
	n.compute(*normals);                              //估计法线存储结果到normals中
	std::cerr << "法线计算   完成" << std::endl;
	//由于XYZ坐标字段和法线字段需要在相同PointCloud对象中，
	//所以创建一个新的PointNormal类型的点云来存储坐标字段和法线连接后的点云。
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云

	//对三角化对象相关变量进行定义
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
	tree2->setInputCloud(cloud_with_normals);                //利用点云构建搜索树
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
	pcl::PolygonMesh triangles;                               //存储最终三角化的网格模型

	//下面设置参数特征值和实际进行三角化
	gp3.setSearchRadius(0.025);//设置连接点之间的最大距离（即为三角形最大边长）为0.025
	//设置各参数特征值，详见本小节前部分对参数设置的描述
	gp3.setMu(2.5);//设置被样本点搜索其邻近点的最远距离为2.5，为了适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100); //设置样本点可搜索的邻域个数为100
	gp3.setMaximumSurfaceAngle(M_PI / 2);   //设置某点法线方向偏离样本点法线方向的最大角度为45度
	gp3.setMinimumAngle(M_PI / 18);         //设置三角化后得到三角形内角最小角度为10度
	gp3.setMaximumAngle(2 * M_PI / 3);        //设置三角化后得到三角形内角最大角度为120度
	gp3.setNormalConsistency(false);       //设置该参数保证法线朝向一致
	gp3.setInputCloud(cloud_with_normals);//设置输入点云为有向点云cloud_with_normals，点云以及法向信息
	gp3.setSearchMethod(tree2);           //设置搜索方式为tree2
	gp3.reconstruct(triangles);           //重建提取三角化，开始重建
	std::cerr << "重建   完成" << std::endl;
	std::cerr << "开始显示 ........" << std::endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0.5);  //设置窗口颜色
	viewer->addPolygonMesh(triangles, "my");  //设置所要显示的网格对象
	//设置网格模型显示模式   
	//viewer->setRepresentationToPointsForAllActors(); //网格模型以点形式显示  
	viewer->setRepresentationToWireframeForAllActors();  //网格模型以线框图模式显示

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer2->setBackgroundColor(0, 0.5, 0);  //设置窗口颜色红绿蓝
	viewer2->addPolygonMesh(triangles, "my2");  //设置所要显示的网格对象
	viewer2->setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示
	//viewer->addCoordinateSystem(0.1);  //设置坐标系,参数为坐标显示尺寸
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
//在开始-cmd回车-输入"exe文件的地址 pcd格式点云的地址"回车就可以运行上述代码