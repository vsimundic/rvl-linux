//#include "stdafx.h"
//#include "RVLCore2.h"

#include "PCLPointCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/normal_3d_omp.h>
#include "RVLCore2.h"
#include "PCLMeshBuilder.h"


using namespace RVL;

PCLMeshBuilder::PCLMeshBuilder()
{
	sigmaS = 5.0f;
	sigmaR = 0.05f;	
	normalEstR = 0.010f;
	flags = 0x00000000;
}


PCLMeshBuilder::~PCLMeshBuilder()
{

}

void PCLMeshBuilder::CreateMesh(
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
	pcl::PolygonMesh &mesh)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC_;

	int i;

	//for (i = 0; i < PC->points.size(); i++)
	//{
	//	PC->points[i].x *= 0.001f;
	//	PC->points[i].y *= 0.001f;
	//	PC->points[i].z *= 0.001f;
	//}

	// Bilateral filtering

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FPC(new pcl::PointCloud<pcl::PointXYZRGBA>());

	if (flags & RVLPCLMESHBUILDER_FLAG_BILATERAL_FILTER)
	{
		pcl::FastBilateralFilter<pcl::PointXYZRGBA> bilateralFilter;

		bilateralFilter.setSigmaS((float)sigmaS);
		bilateralFilter.setSigmaR((float)sigmaR);

		bilateralFilter.setInputCloud(PC);

		bilateralFilter.applyFilter(*FPC);

		PC_ = FPC;
	}
	else
		PC_ = PC;

	// Compute normals

	pcl::PointCloud<pcl::Normal>::Ptr N(new pcl::PointCloud<pcl::Normal>);

	pcl::search::OrganizedNeighbor<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBA>());

	pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> norm_est;

	norm_est.setRadiusSearch((float)normalEstR);
	norm_est.setInputCloud(PC_);
	norm_est.setSearchMethod(tree);
	norm_est.compute(*N);

	for (i = 0; i < N->points.size(); i++)
		if (!isfinite(N->points[i].normal_x))
		{
		N->points[i].normal_x = 0.0f;
		N->points[i].normal_y = 0.0f;
		N->points[i].normal_z = 0.0f;
		N->points[i].curvature = 0.0f;
		}

	//// Concatenate the Point and normal fields

	//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr OPC(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	//pcl::concatenateFields(*FPC, *N, *OPC);

	// Create OrganizedFastMesh

	pcl::OrganizedFastMesh<pcl::PointXYZRGBA> OFM;

	OFM.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGBA>::TRIANGLE_ADAPTIVE_CUT);

	OFM.setInputCloud(PC_);
	
	OFM.reconstruct(mesh);	// Conditions for adding a triangle are defined in function isShadowed in organized_fast_mesh.h.
	// This function is applied to endpoints of every triangle edge.
	// The meaning of most of the parameters of OrganizedFastMesh method can be understood from the code of this function.

	//pcl::OrganizedFastMesh<pcl::PointXYZRGBNormal> OFM;

	//OFM.setInputCloud(OPC);

	//pcl::PolygonMesh mesh;

	//OFM.reconstruct(mesh);	// Conditions for adding a triangle are defined in function isShadowed in organized_fast_mesh.h.
	//// This function is applied to endpoints of every triangle edge.
	//// The meaning of most of the parameters of OrganizedFastMesh method can be understood from the code of this function.

	// Add normals to mesh

	pcl::PCLPointCloud2 N2;
	pcl::toPCLPointCloud2(*N, N2);
	pcl::PCLPointCloud2 aux;
	pcl::concatenateFields(N2, mesh.cloud, aux);
	mesh.cloud = aux;
}

void PCLMeshBuilder::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("MeshBuilder.sigmaS", RVLPARAM_TYPE_DOUBLE, &sigmaS);
	pParamData = ParamList.AddParam("MeshBuilder.sigmaR", RVLPARAM_TYPE_DOUBLE, &sigmaR);
	pParamData = ParamList.AddParam("MeshBuilder.normalEstR", RVLPARAM_TYPE_DOUBLE, &normalEstR);
	pParamData = ParamList.AddParam("MeshBuilder.bilateralFilter", RVLPARAM_TYPE_FLAG, &flags);
	ParamList.AddID(pParamData, "yes", RVLPCLMESHBUILDER_FLAG_BILATERAL_FILTER);
}

