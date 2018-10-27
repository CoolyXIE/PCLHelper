#include "Headers.h"
#include "Camera.h"
#include "Filter.h"
#include <pcl/io/ply_io.h>

using namespace pcl;
using namespace std;

void test_camera()
{
	PinholeCamera camera = PinholeCamera(625.0, 620.5, 323.3, 236.6, 640, 480);
	auto K = camera.getIntrinsicMatrix();
	std::cout << K << std::endl;
}

void test_filter()
{
	string bodyPLYPath = "../test/data/humanbody.ply";

	CloudALL::Ptr cloud(new CloudALL);
	pcl::io::loadPLYFile(bodyPLYPath, *cloud);

	int n = 100;
	vector<int> indices;
	for (int i = 0; i < n; i++)
		indices.push_back(i);

	std::cout << "Before:\t" << cloud->points.size() << std::endl;
	auto outCloud = RemovePoints<PtALL>(*cloud, indices, false);
	std::cout << "After:\t" << outCloud.points.size() << std::endl;

}

int main(int argc, char** argv)
{
	test_camera();

	//test_filter();

	getchar();
	return 0;
}
