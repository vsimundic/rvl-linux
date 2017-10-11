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
	voxelSize = 0.01f;
	sampleVoxelDistance = 2;
	visualizationData.resolution = 0.01f;
	visualizationData.SDFSurfaceValue = 0.0f;
	modelDataBase = NULL; //Vidovic
	modelsInDataBase = NULL; //Vidovic
	classArray.Element = NULL;
	sceneObject.sampleArray.Element = NULL;
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

	alignment.modelDataBase = RVLCreateString(modelDataBase);

	printf("Loading CTI database...");

	alignment.LoadModelDataBase();

	printf("completed.\n");
}

void VNClassifier::CreateParamList()
{
	paramList.m_pMem = pMem0;

	RVLPARAM_DATA *pParamData;

	paramList.Init();

	pParamData = paramList.AddParam("VN.kMaxMatchCost", RVLPARAM_TYPE_FLOAT, &kMaxMatchCost);
	pParamData = paramList.AddParam("VN.maxnSClusters", RVLPARAM_TYPE_INT, &maxnSClusters);
	pParamData = paramList.AddParam("Recognition.mode", RVLPARAM_TYPE_ID, &mode);
	paramList.AddID(pParamData, "TRAINING", RVLRECOGNITION_MODE_TRAINING);
	paramList.AddID(pParamData, "RECOGNITION", RVLRECOGNITION_MODE_RECOGNITION);
	pParamData = paramList.AddParam("ModelDataBase", RVLPARAM_TYPE_STRING, &modelDataBase); //Vidovic
	pParamData = paramList.AddParam("ModelsInDataBase", RVLPARAM_TYPE_STRING, &modelsInDataBase); //Vidovic
	pParamData = paramList.AddParam("VN.voxelSize", RVLPARAM_TYPE_FLOAT, &voxelSize);
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

	RVL_DELETE_ARRAY(classArray.Element);
	RVL_DELETE_ARRAY(sceneObject.sampleArray.Element);
}

void VNClassifier::ComputeDescriptor(
	Mesh *pMesh,
	float *RIn,
	float *tIn,
	float *&dS,
	bool *&bdS,
	Box<float> &SBoundingBox,
	int iModel)
{
	// Detect surfels.

	pSurfels->Init(pMesh);

	pSurfelDetector->Init(pMesh, pSurfels, pMem);

	printf("Segmentation to surfels...");

	pSurfelDetector->Segment(pMesh, pSurfels);

	printf("completed.\n");

	int nSurfels = pSurfels->NodeArray.n;

	printf("No. of surfels = %d\n", nSurfels);

	pSurfels->DetectVertices(pMesh);

	// Create scene object.	

	float *R = sceneObject.R;

	if (RIn)
	{
		RVLCOPYMX3X3(RIn, R);
	}
	else
	{
		RVLUNITMX3(R);
	}

	float *t = sceneObject.t;

	if (tIn)
	{
		RVLCOPY3VECTOR(tIn, t);
	}
	else
	{
		RVLNULL3VECTOR(t);
	}

	sceneObject.vertexArray = new float[3 * pSurfels->vertexArray.n];

	float *P = sceneObject.vertexArray;

	int iVertex;
	SURFEL::Vertex *pVertex;

	for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++, P += 3)
	{
		pVertex = pSurfels->vertexArray.Element[iVertex];

		RVLTRANSF3(pVertex->P, R, t, P);
	}

	sceneObject.NArray = new float[3 * pSurfels->NodeArray.n];

	float *N = sceneObject.NArray;

	int iSurfel;
	Surfel *pSurfel;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++, N += 3)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		RVLMULMX3X3VECT(R, pSurfel->N, N);
	}

	int nSamplePts = 300;

	sceneObject.sampleArray.n = 2 * nSamplePts;

	RVL_DELETE_ARRAY(sceneObject.sampleArray.Element);

	sceneObject.sampleArray.Element = new VN_::Sample[2 * nSamplePts];

	SampleMesh(pMesh, R, t, sceneObject.sampleArray);

	Array<RECOG::VN_::Sample> sampleArray;
	Array3D<RECOG::VN_::Voxel> volume;
	float P0[3];
	Box<float> boundingBox;

	SampleMeshDistanceFunction(pMesh, pSurfels, voxelSize, sampleVoxelDistance, volume, P0, sampleArray, boundingBox);

	Array<int> iPtArray;

	iPtArray.n = sampleArray.n;

	RandomIndices(iPtArray);

	int iSample;
	float *P_;
	VN_::Sample *pSample, *pSample_;

	for (iSample = 0; iSample < nSamplePts; iSample++)
	{
		pSample = sampleArray.Element + iPtArray.Element[iSample];

		pSample_ = sceneObject.sampleArray.Element + nSamplePts + iSample;

		RVLTRANSF3(pSample->P, R, t, pSample_->P);

		pSample_->SDF = pSample->SDF;
	}

	//// Sample visualization

	//Visualizer visualizer;

	//visualizer.Create();

	//DisplaySampledMesh(&visualizer, volume, P0, voxelSize);

	//unsigned char color[] = { 0, 128, 255 };

	//visualizer.DisplayPointSet<float, VN_::Sample>(sampleArray, color, 6.0f);

	//visualizer.Run();

	delete[] sampleArray.Element;

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

	//concaveClustering.InitDisplay(&visualizer, pMesh, SelectionColor);

	//concaveClustering.Display();

	////convexClustering.InitDisplay(&visualizer, pMesh, SelectionColor);

	////convexClustering.Display();

	//visualizer.Run();

	//surfels.NodeColors(SelectionColor);

	//surfels.InitDisplay(&visualizer, &mesh, &surfelDetector);		

	//surfels.Display(&visualizer, &mesh);

	//visualizer.Run();

	printf("Matching VN model to scene...");

	P = sceneObject.vertexArray;

	InitBoundingBox<float>(&SBoundingBox, P);

	P += 3;

	for (iVertex = 1; iVertex < pSurfels->vertexArray.n; iVertex++, P += 3)
		UpdateBoundingBox<float>(&SBoundingBox, P);

	VN *pModel = models[iModel];

	dS = new float[pModel->featureArray.n];

	bdS = new bool[pModel->featureArray.n];

	pModel->Match4(pMesh, sceneObject, this, SBoundingBox, dS, bdS);

	delete[] sceneObject.vertexArray;
	delete[] sceneObject.NArray;
		 
	printf("completed.\n");
}

void VNClassifier::Learn(
	char *modelSequenceFileName,
	int iClass,
	Visualizer *pVisualizer)
{
	float resolution = 0.01f;

	Eigen::MatrixXf A(3, 66);

	A = alignment.ConvexTemplatenT();

	int iMetaModel = classArray.Element[iClass].iMetaModel;

	Mesh mesh;

	FileSequenceLoader modelsLoader;
	FileSequenceLoader dbLoader;

	char modelFilePath[200];
	char modelFileName[200];

	if (!modelDataBase)
		modelDataBase = "modelDB.dat";

	if (!modelsInDataBase)
		modelsInDataBase = "DBModels.txt";

	modelsLoader.Init(modelSequenceFileName);
	dbLoader.Init(modelsInDataBase);

	FILE *fp = fopen(modelDataBase, "a");

	bool saveDBSequenceFile = false;

	printf("Model DB creation started...\n");

	unsigned char color[] = { 0, 128, 255 };

	int currentModelID;
	float *dS;
	bool *bdS;
	Box<float> SBoundingBox;
	VN *pModel;
	float R[9];
	float t[3];
	char modelFilePath_[200];
	char modelFileName_[200];
	int modelID, modelID_;

	while (modelsLoader.GetNext(modelFilePath, modelFileName))
	{
		//if (ModelExistInDB(modelFileName, dbLoader))
		//	continue;

		printf("\nProcessing model %s!\n", modelFileName);

		saveDBSequenceFile = true;

		LoadMesh(vpMeshBuilder, modelFilePath, &mesh, false);

		currentModelID = dbLoader.GetLastModelID() + 1;

		pMem->Clear();

		dbLoader.ResetID();

		modelID = -1;

		while (dbLoader.GetNext(modelFilePath_, modelFileName_, &modelID_))
			if (strcmp(modelFileName, modelFileName_) == 0)
			{
				modelID = modelID_;

				break;
			}

		if (modelID < 0)
			printf("CTI of the considered model is not available!");
		else
		{
			alignment.ObjectAlignment(alignment.MCTISet.SegmentCTIs.Element[modelID],
				alignment.MCTISet.pCTI.Element,
				alignment.MCTISet.SegmentCTIs.Element[classArray.Element[iClass].iRefInstance],
				alignment.MCTISet.pCTI.Element, A, R, t, true);

			//RVLUNITMX3(R);
			//RVLNULL3VECTOR(t);

			ComputeDescriptor(&mesh, R, t, dS, bdS, SBoundingBox, iMetaModel);

			dbLoader.AddModel(currentModelID, modelFilePath, modelFileName);

			if (pVisualizer)
			{
				pModel = models[iMetaModel];

				ExpandBox<float>(&SBoundingBox, 10.0f * resolution);

				pVisualizer->renderer->RemoveAllViewProps();

				pModel->Display(pVisualizer, SBoundingBox, visualizationData.resolution, dS, bdS, visualizationData.SDFSurfaceValue);

				pVisualizer->DisplayPointSet<float, VN_::Sample>(sceneObject.sampleArray, color, 6.0f);

				pVisualizer->Run();
			}

			delete[] dS;
			delete[] bdS;
		}
	}

	printf("Model DB creation completed!\n");

	if (saveDBSequenceFile)
		SaveModelID(dbLoader, modelsInDataBase);

	fclose(fp);
}

void VN_::_3DNetDatabaseClasses(VNClassifier *pClassifier)
{
	pClassifier->classArray.n = 10;

	pClassifier->classArray.Element = new RECOG::ClassData[pClassifier->classArray.n];

	RECOG::ClassData *pClass;

	// class banana

	pClass = pClassifier->classArray.Element + 1;
	pClass->iMetaModel = RVLVN_METAMODEL_TORUS;
	pClass->iFirstInstance = 10;
	pClass->nInstances = 6;
	pClass->iRefInstance = 15;

	// class donut

	pClass = pClassifier->classArray.Element + 5;
	pClass->iMetaModel = RVLVN_METAMODEL_TORUS;
	pClass->iFirstInstance = 126;
	pClass->nInstances = 10;
	pClass->iRefInstance = 126;

	// class mug

	pClass = pClassifier->classArray.Element + 9;
	pClass->iMetaModel = RVLVN_METAMODEL_MUG;
	pClass->iFirstInstance = 196;
	pClass->nInstances = 61;
	pClass->iRefInstance = 199;
}
