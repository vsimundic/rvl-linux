//#include "stdafx.h"
//#include "RVLCore2.h"

#include "PCLPointCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/normal_3d_omp.h>
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "PCLMeshBuilder.h"


using namespace RVL;

PCLMeshBuilder::PCLMeshBuilder()
{
	sigmaS = 5.0f;
	sigmaR = 0.05f;	
	normalEstR = 0.010f;
	flags = 0x00000000;

	vpBilateralFilter = new pcl::FastBilateralFilter<pcl::PointXYZRGBA>;
	vpNormalEstimator = new pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal>;
	vpOFM = new pcl::OrganizedFastMesh<pcl::PointXYZRGBA>;
}


PCLMeshBuilder::~PCLMeshBuilder()
{
	delete ((pcl::FastBilateralFilter<pcl::PointXYZRGBA> *)vpBilateralFilter);
	delete ((pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> *)vpNormalEstimator);
	delete ((pcl::OrganizedFastMesh<pcl::PointXYZRGBA> *)vpOFM);
}

void PCLMeshBuilder::CreateMesh(
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
	pcl::PolygonMesh &mesh)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC_;

	int i;

	// Bilateral filtering

	if (flags & RVLPCLMESHBUILDER_FLAG_BILATERAL_FILTER)
	{
		pcl::FastBilateralFilter<pcl::PointXYZRGBA> *pBilateralFilter = (pcl::FastBilateralFilter<pcl::PointXYZRGBA> *)vpBilateralFilter;

		pBilateralFilter->setSigmaS((float)sigmaS);
		pBilateralFilter->setSigmaR((float)sigmaR);

		pBilateralFilter->setInputCloud(PC);

		pBilateralFilter->applyFilter(FPC);

		PC_ = { boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>(FPC) };
	}
	else
		PC_ = PC;

	// Compute normals

	pcl::search::OrganizedNeighbor<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBA>());

	pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> *pNormalEstimator = (pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> *)vpNormalEstimator;

	pNormalEstimator->setRadiusSearch((float)normalEstR);
	pNormalEstimator->setInputCloud(PC_);
	pNormalEstimator->setSearchMethod(tree);
	pNormalEstimator->compute(N);

	for (i = 0; i < N.points.size(); i++)
		if (!isfinite(N.points[i].normal_x))
		{
		N.points[i].normal_x = 0.0f;
		N.points[i].normal_y = 0.0f;
		N.points[i].normal_z = 0.0f;
		N.points[i].curvature = 0.0f;
		}

	// Create OrganizedFastMesh

	pcl::OrganizedFastMesh<pcl::PointXYZRGBA> *pOFM = (pcl::OrganizedFastMesh<pcl::PointXYZRGBA> *)vpOFM;

	pOFM->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGBA>::TRIANGLE_ADAPTIVE_CUT);

	pOFM->setInputCloud(PC_);
	
	pOFM->reconstruct(mesh);	// Conditions for adding a triangle are defined in function isShadowed in organized_fast_mesh.h.
	// This function is applied to endpoints of every triangle edge.
	// The meaning of most of the parameters of OrganizedFastMesh method can be understood from the code of this function.

	// Add normals to mesh

	//pcl::toPCLPointCloud2(*N, N2);
	pcl::toPCLPointCloud2(N, N2);	
	pcl::concatenateFields(N2, mesh.cloud, aux);
	mesh.cloud = aux;
}

bool PCLMeshBuilder::CreateMesh(
	vtkSmartPointer<vtkPolyData> pPolygonData,
	pcl::PolygonMesh &mesh)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("Colors"));
	if (rgbPointData == NULL)
	{
		rgbPointData = rgbPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("RGB"));

		if (rgbPointData)
			rgbPointData->SetName("Colors");
		else
			return false;
	}

	pcl::VTKUtils::vtk2mesh(pPolygonData, mesh);

	vtkSmartPointer<vtkFloatArray> normalPointData = normalPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("Normals"));
	if (normalPointData == NULL)
	{
		normalPointData = normalPointData->SafeDownCast(pPolygonData->GetPointData()->GetNormals());

		if (normalPointData == NULL)
			return false;
	}

	int nPts = pPolygonData->GetNumberOfPoints();

	nPts = normalPointData->GetNumberOfTuples();

	N.resize(nPts);

	float N_[3];
	
	for (int i = 0; i < nPts; i++)
	{
		normalPointData->GetTupleValue(i, N_);

		N.points[i].normal_x = N_[0];
		N.points[i].normal_y = N_[1];
		N.points[i].normal_z = N_[2];
	}

	pcl::toPCLPointCloud2(N, N2);
	pcl::concatenateFields(N2, mesh.cloud, aux);
	mesh.cloud = aux;

	return true;
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

