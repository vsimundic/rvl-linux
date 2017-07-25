#include <pcl/common/common.h>
#include "PCLPointCloud.h"

using namespace RVL;

PCLPointCloud::PCLPointCloud()
{
	vpPC = NULL;
}


PCLPointCloud::~PCLPointCloud()
{
	if (vpPC)
		delete[] vpPC;
}


void RVL::PCLPointCloud::Create(int w, int h)
{
	vpPC = new pcl::PointCloud<pcl::PointXYZRGBA>(w, h);
}
