#include "Headers.h"
#include "Camera.h"
#include "Feature.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>


using namespace std;
using namespace pcl;
using namespace cooly;

void test_camera()
{
	PinholeCamera camera = PinholeCamera(625.0, 620.5, 323.3, 236.6, 640, 480);
	auto K = camera.getIntrinsicMatrix();
	std::cout << K << std::endl;
}

void test_normal()
{
	string bunny_file = "../data/bunny/bunny.ply";
	CloudXYZ::Ptr cloud(new CloudXYZ);
	io::loadPLYFile(bunny_file, *cloud);
    auto normals = computeNormals(cloud, 30);

    CloudNML::Ptr cloud_normals(new CloudNML);
    pcl::concatenateFields(*cloud, *normals, *cloud_normals);
    io::savePLYFile("../data/bunny/bunny_with_normals.ply", *cloud_normals);

}

int main(int argc, char** argv)
{
	//test_camera();

	test_normal();

	return 0;
}
