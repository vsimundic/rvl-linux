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
	double *fitnessScore,
	void *kdTreePtr)
{
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_destination(new pcl::PointCloud<pcl::PointXYZINormal>);

	//creating source cloud
	vtkSmartPointer<vtkPoints> pdPoints = pdSource->GetPoints();
	vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(pdSource->GetPointData()->GetNormals());
	cloud_source->width = pdPoints->GetNumberOfPoints();
	cloud_source->height = 1;
	cloud_source->is_dense = false;
	cloud_source->points.resize(cloud_source->width * cloud_source->height);
	double *point;
	float normal[3];
	bool hasNormals = false;
	if (normals.GetPointer()) //check if exists
		hasNormals = true;
	for (int i = 0; i < pdPoints->GetNumberOfPoints(); i++)
	{
		point = pdPoints->GetPoint(i);
		cloud_source->points[i].x = point[0];			
		cloud_source->points[i].y = point[1];
		cloud_source->points[i].z = point[2];
		if (hasNormals) //check if exists
		{
			normals->GetTupleValue(i, normal);
			cloud_source->points[i].normal_x = normal[0];
			cloud_source->points[i].normal_y = normal[1];
			cloud_source->points[i].normal_z = normal[2];
		}
	}

	//creating destination cloud
	pdPoints = pdDestination->GetPoints();
	normals = vtkFloatArray::SafeDownCast(pdDestination->GetPointData()->GetNormals());
	cloud_destination->width = pdPoints->GetNumberOfPoints();
	cloud_destination->height = 1;
	cloud_destination->is_dense = false;
	cloud_destination->points.resize(cloud_destination->width * cloud_destination->height);
	
	hasNormals = false;
	if (normals.GetPointer()) //check if exists
		hasNormals = true;
	for (int i = 0; i < pdPoints->GetNumberOfPoints(); i++)
	{
		point = pdPoints->GetPoint(i);
		cloud_destination->points[i].x = point[0];
		cloud_destination->points[i].y = point[1];
		cloud_destination->points[i].z = point[2];
		if (hasNormals) //check if exists
		{
			normals->GetTupleValue(i, normal);
			cloud_destination->points[i].normal_x = normal[0];
			cloud_destination->points[i].normal_y = normal[1];
			cloud_destination->points[i].normal_z = normal[2];
		}
	}

	Eigen::MatrixXf Ticp;
	if (ICPvariant == PCLICPVariants::Point_to_point)
	{

		pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
		icp.setMaximumIterations(maxIterations);
		icp.setMaxCorrespondenceDistance(maxCorrespondenceDist);
		icp.setInputCloud(cloud_source);
		if (kdTreePtr)
			icp.setSearchMethodTarget(*((pcl::search::KdTree<pcl::PointXYZINormal>::Ptr*)kdTreePtr), true);
		icp.setInputTarget(cloud_destination);
		pcl::PointCloud<pcl::PointXYZINormal> Final;
		icp.align(Final);
		*fitnessScore = icp.getFitnessScore();

		Ticp = icp.getFinalTransformation().transpose();
	}
	else if (ICPvariant == PCLICPVariants::Point_to_point_nonlinear)
	{
		pcl::IterativeClosestPointNonLinear<pcl::PointXYZINormal, pcl::PointXYZINormal> icp_nl;
		icp_nl.setMaximumIterations(maxIterations);
		icp_nl.setMaxCorrespondenceDistance(maxCorrespondenceDist);
		//icp_nl.setEuclideanFitnessEpsilon();
		//icp_nl.setTransformationEpsilon();
		icp_nl.setInputCloud(cloud_source);
		if (kdTreePtr)
			icp_nl.setSearchMethodTarget(*((pcl::search::KdTree<pcl::PointXYZINormal>::Ptr*)kdTreePtr), true);
		icp_nl.setInputTarget(cloud_destination);
		pcl::PointCloud<pcl::PointXYZINormal> Final;
		icp_nl.align(Final);
		*fitnessScore = icp_nl.getFitnessScore();

		Ticp = icp_nl.getFinalTransformation().transpose();

	}
	else if (ICPvariant == PCLICPVariants::GeneralizedICP)
	{
		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> gicp;
		gicp.setMaximumIterations(maxIterations);
		gicp.setMaxCorrespondenceDistance(maxCorrespondenceDist);
		gicp.setInputCloud(cloud_source);
		if (kdTreePtr)
			gicp.setSearchMethodTarget(*((pcl::search::KdTree<pcl::PointXYZINormal>::Ptr*)kdTreePtr), true);
		gicp.setInputTarget(cloud_destination);
		pcl::PointCloud<pcl::PointXYZINormal> Final;
		gicp.setCorrespondenceRandomness(10);
		gicp.setMaximumOptimizerIterations(10);
		gicp.setRotationEpsilon(0.5);
		gicp.setEuclideanFitnessEpsilon(1);
		gicp.align(Final);
		*fitnessScore = gicp.getFitnessScore();
		
		Ticp = gicp.getFinalTransformation().transpose();

	}
	else if (ICPvariant == PCLICPVariants::Point_to_plane)
	{
		pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp_plane;
		icp_plane.setMaximumIterations(maxIterations);
		icp_plane.setMaxCorrespondenceDistance(maxCorrespondenceDist);
		icp_plane.setInputCloud(cloud_source);
		if (kdTreePtr)
			icp_plane.setSearchMethodTarget(*((pcl::search::KdTree<pcl::PointXYZINormal>::Ptr*)kdTreePtr), true);
		icp_plane.setInputTarget(cloud_destination);
		pcl::PointCloud<pcl::PointXYZINormal> Final;
		icp_plane.align(Final);
		*fitnessScore = icp_plane.getFitnessScore();

		Ticp = icp_plane.getFinalTransformation().transpose();

	}
	memcpy(T, Ticp.data(), 16 * sizeof(float));
}