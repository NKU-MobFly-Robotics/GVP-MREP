#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPLYFile("/home/charliedog/rosprojects/MURDER/map2.ply", *cloud);


    pcl::io::savePCDFileBinary("/home/charliedog/rosprojects/MURDER/map.pcd", *cloud);

    system("pause");

    return 0;
}