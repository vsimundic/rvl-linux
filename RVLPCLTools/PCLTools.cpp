#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include "PCLTools.h"

using namespace RVL;

int RVL::PCLLoadPCD(
	char *FileName,
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC)
{
	return pcl::io::loadPCDFile<pcl::PointXYZRGBA>(FileName, *PC);
}

int RVL::PCLMeshToPolygonData(
	pcl::PolygonMesh &mesh, 
	vtkSmartPointer< vtkPolyData > &polygonData)
{
	return pcl::VTKUtils::mesh2vtk(mesh, polygonData);
}

void RVL::PCLSavePLY(
	char *FileName,
	pcl::PolygonMesh &mesh)
{
	pcl::io::savePLYFileBinary(FileName, mesh);
}
