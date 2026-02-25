#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Graph.h"
//#include <Eigen\Eigenvalues>
//#include <pcl/common/common.h>
//#include <pcl/PolygonMesh.h>
//#include "PCLTools.h"
//#include "PCLMeshBuilder.h"
//#include "RGBDCamera.h"
#include "Mesh.h"
//#include "Visualizer.h"
//#include "SurfelGraph.h"
//#include "PlanarSurfelDetector.h"
#include "RVLMeshNoiser.h"
#include <random>



using namespace RVL;


MeshNoiser::MeshNoiser()
{
	pointStdDev = 1;
	normalStdDev = 0.05;
}

MeshNoiser::~MeshNoiser()
{
}

void MeshNoiser::SetParam(float pointStdDev, float normalStdDev)
{
	this->pointStdDev = pointStdDev;
	this->normalStdDev = normalStdDev;
}

void MeshNoiser::AddWhiteNoise(Mesh *pMesh)
{
	int nPts = pMesh->pPolygonData->GetNumberOfPoints();

	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pMesh->pPolygonData->GetPoints()->GetData());

	vtkSmartPointer<vtkFloatArray> normalPointData = pointData->SafeDownCast(pMesh->pPolygonData->GetPointData()->GetArray("Normals"));

	int iPt,i;
	float P[3];
	float N[3];
	float fTmp;

	std::default_random_engine generator;
	std::normal_distribution<double> pointNoiseDistribution(0, pointStdDev);
	std::normal_distribution<double> normalNoiseDistribution(0, normalStdDev);

	float pointNoise;
	float normalNoise[3];


	for (iPt = 0; iPt < nPts; iPt++)
	{
		//load point and normal data
		pointData->GetTupleValue(iPt, P);
		normalPointData->GetTupleValue(iPt, N);
		RVLNORM3(N, fTmp);

		//add noise to points
		pointNoise = pointNoiseDistribution(generator);

		for (i = 0; i < 3; i++)
			P[i] = P[i] + pointNoise*N[i];

		//add noise to normal
		for (i = 0; i < 3; i++)
		{
			normalNoise[i] = normalNoiseDistribution(generator);
			N[i] += normalNoise[i];
		}
		RVLNORM3(N, fTmp);

		//save points and normal data
		pointData->SetTupleValue(iPt, P);
		normalPointData->SetTupleValue(iPt, N);
	}
}