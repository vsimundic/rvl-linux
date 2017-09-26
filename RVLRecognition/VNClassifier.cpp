//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "ObjectDetector.h"
#include <Eigen\Eigenvalues>
#include "VN.h"
#include "VNClassifier.h"

using namespace RVL;
using namespace RECOG;

VNClassifier::VNClassifier()
{
}


VNClassifier::~VNClassifier()
{
	Clear();
}

void VNClassifier::Create(char *cfgFileName)
{
	convexClustering.pMem = pMem;

	convexClustering.pSurfels = pSurfels;

	convexClustering.pSurfelDetector = pSurfelDetector;

	convexClustering.bDetectGroundPlane = false;

	concaveClustering.pMem = pMem;

	concaveClustering.pSurfels = pSurfels;

	concaveClustering.pSurfelDetector = pSurfelDetector;

	concaveClustering.clusterType = -1.0f;

	concaveClustering.bDetectGroundPlane = false;

	VN *pModel;

	pModel = new VN;

	VN_::CreateTorus(pModel, pMem0);

	models.push_back(pModel);

	pModel = new VN;

	VN_::CreateBottle(pModel, pMem0);

	models.push_back(pModel);

	pModel = new VN;

	VN_::CreateHammer(pModel, pMem0);

	models.push_back(pModel);

	pModel = new VN;

	VN_::CreateBowl(pModel, pMem0);

	models.push_back(pModel);

	pModel = new VN;

	VN_::CreateMug(pModel, pMem0);

	models.push_back(pModel);

	CreateParamList();

	paramList.LoadParams(cfgFileName);

	clusteringTolerance = 3.0f * convexClustering.kNoise * 2.0f / pSurfelDetector->kPlane;
}

void VNClassifier::CreateParamList()
{
	paramList.m_pMem = pMem0;

	RVLPARAM_DATA *pParamData;

	paramList.Init();

	pParamData = paramList.AddParam("VN.kMaxMatchCost", RVLPARAM_TYPE_FLOAT, &kMaxMatchCost);
	pParamData = paramList.AddParam("VN.maxnSClusters", RVLPARAM_TYPE_INT, &maxnSClusters);
}

void VNClassifier::Init(Mesh *pMesh)
{

}

void VNClassifier::Clear()
{
	int iModel;

	for (iModel = 0; iModel < models.size(); iModel++)
		delete[] models[iModel];

	models.clear();
}

void VNClassifier::Classify(
	Mesh *pMesh,
	float *&dS,
	bool *&bdS,
	Box<float> &SBoundingBox,
	int iModel)
{
	// Cluster surfels into convex surfaces.

	convexClustering.pMesh = pMesh;

	convexClustering.Clusters();

	// Cluster surfels into concave surfaces.

	concaveClustering.pMesh = pMesh;

	concaveClustering.Clusters();

	uchar SelectionColor[] = {0, 255, 0};

	pSurfels->NodeColors(SelectionColor);

	//Visualizer visualizer;

	//visualizer.Create();

	////concaveClustering.InitDisplay(&visualizer, &mesh, SelectionColor);

	////concaveClustering.Display();

	//convexClustering.InitDisplay(&visualizer, pMesh, SelectionColor);

	//convexClustering.Display();

	//visualizer.Run();

	//surfels.NodeColors(SelectionColor);

	//surfels.InitDisplay(&visualizer, &mesh, &surfelDetector);		

	//surfels.Display(&visualizer, &mesh);

	//visualizer.Run();

	printf("Matching VN model to scene...");

	InitBoundingBox<float>(&SBoundingBox, pSurfels->vertexArray.Element[0]->P);

	int iVertex;

	for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
		UpdateBoundingBox<float>(&SBoundingBox, pSurfels->vertexArray.Element[iVertex]->P);

	VN *pModel = models[iModel];

	dS = new float[pModel->featureArray.n];

	bdS = new bool[pModel->featureArray.n];

	pModel->Match4(pMesh, this, SBoundingBox, dS, bdS);

	printf("completed.\n");
}
