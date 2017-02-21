#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include "RVLVTK.h"
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

//Petra:
void RVL::PCLICP(
	vtkSmartPointer<vtkPolyData> pdSource,
	vtkSmartPointer<vtkPolyData> pdDestination,
	float *T,
	int maxIterations,
	float maxCorrespondenceDist,
	int ICPvariant,
	double *fitnessScore)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_destination(new pcl::PointCloud<pcl::PointXYZ>);

	//creating source cloud
	vtkSmartPointer<vtkPoints> pdPoints = pdSource->GetPoints();
	cloud_source->width = pdPoints->GetNumberOfPoints();
	cloud_source->height = 1;
	cloud_source->is_dense = false;
	cloud_source->points.resize(cloud_source->width * cloud_source->height);
	double *point;

	for (int i = 0; i < pdPoints->GetNumberOfPoints(); i++)
	{
		point = pdPoints->GetPoint(i);
		cloud_source->points[i].x = point[0];			
		cloud_source->points[i].y = point[1];
		cloud_source->points[i].z = point[2];
	}

	//creating destination cloud
	pdPoints = pdDestination->GetPoints();
	cloud_destination->width = pdPoints->GetNumberOfPoints();
	cloud_destination->height = 1;
	cloud_destination->is_dense = false;
	cloud_destination->points.resize(cloud_destination->width * cloud_destination->height);
	

	for (int i = 0; i < pdPoints->GetNumberOfPoints(); i++)
	{
		point = pdPoints->GetPoint(i);
		cloud_destination->points[i].x = point[0];
		cloud_destination->points[i].y = point[1];
		cloud_destination->points[i].z = point[2];
	}

	Eigen::MatrixXf Ticp;
	if (ICPvariant == PCLICPVariants::Point_to_point)
	{

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaximumIterations(maxIterations);
		icp.setMaxCorrespondenceDistance(maxCorrespondenceDist);
		icp.setInputCloud(cloud_source);
		icp.setInputTarget(cloud_destination);
		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);
		*fitnessScore = icp.getFitnessScore();

		Ticp = icp.getFinalTransformation().transpose();
	}
	else if (ICPvariant == PCLICPVariants::Point_to_point_nonlinear)
	{
		pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp_nl;
		icp_nl.setMaximumIterations(maxIterations);
		icp_nl.setMaxCorrespondenceDistance(maxCorrespondenceDist);
		//icp_nl.setEuclideanFitnessEpsilon();
		//icp_nl.setTransformationEpsilon();
		icp_nl.setInputCloud(cloud_source);
		icp_nl.setInputTarget(cloud_destination);
		pcl::PointCloud<pcl::PointXYZ> Final;
		icp_nl.align(Final);
		*fitnessScore = icp_nl.getFitnessScore();

		Ticp = icp_nl.getFinalTransformation().transpose();

	}
	else if (ICPvariant == PCLICPVariants::GeneralizedICP)
	{
		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
		gicp.setMaximumIterations(maxIterations);
		gicp.setMaxCorrespondenceDistance(maxCorrespondenceDist);
		gicp.setInputCloud(cloud_source);
		gicp.setInputTarget(cloud_destination);
		pcl::PointCloud<pcl::PointXYZ> Final;
		gicp.align(Final);
		*fitnessScore = gicp.getFitnessScore();
		
		Ticp = gicp.getFinalTransformation().transpose();

	}
	else if (ICPvariant == PCLICPVariants::Point_to_plane)
	{
		/*pcl::IterativeClosestPointWithNormals<pcl::PointXYZ, pcl::PointXYZ> icp_plane;
		icp_plane.setMaximumIterations(maxIterations);
		icp_plane.setMaxCorrespondenceDistance(maxCorrespondenceDist);
		icp_plane.setInputCloud(cloud_source);
		icp_plane.setInputTarget(cloud_destination);
		pcl::PointCloud<pcl::PointXYZ> Final;
		icp_plane.align(Final);
		*fitnessScore = icp_plane.getFitnessScore();

		Ticp = icp_plane.getFinalTransformation().transpose();*/

	}
	memcpy(T, Ticp.data(), 16 * sizeof(float));
}