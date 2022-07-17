//a-shape 曲面重构(凹包算法扩展)
//https://blog.csdn.net/qq_36686437/article/details/119881696
//修改：24行
//buuny				0.003
//boolSurface		0.00007
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("data//bunny.pcd", *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> cavehull;
	cavehull.setInputCloud(cloud);
	cavehull.setAlpha(0.003);			//boolsurface.pcd	此处调整为0.0001时有模糊轮廓
	vector<pcl::Vertices> polygons;
	cavehull.reconstruct(*surface_hull, polygons);// 重建面要素到点云

	pcl::PolygonMesh mesh;
	cavehull.reconstruct(mesh);// 重建面要素到mesh
	pcl::io::saveOBJFile("object_mesh.obj", mesh);
	cerr << "Concave hull has: " << surface_hull->points.size()
		<< " data points." << endl;

	pcl::PCDWriter writer;
	writer.write("hull.pcd", *surface_hull, false);
	// 可视化
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("hull"));
	viewer->setWindowName("alshape曲面重构");
	viewer->addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "polyline");
	viewer->spin();

	return (0);
}

