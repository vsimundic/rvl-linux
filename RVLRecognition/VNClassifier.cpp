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
	refModel.d = NULL;
	refModel.bd = NULL;
	maxnSCClusters = 4;
	maxnSUClusters = 2;
	maxnSTClusters = 2;
	connectedComponentMaxDist = 0.050f;
	connectedComponentMinSize = 100;
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
	convexClustering.bOverlappingClusters = true;

	concaveClustering.pMem = pMem;

	concaveClustering.pSurfels = pSurfels;

	concaveClustering.pSurfelDetector = pSurfelDetector;

	concaveClustering.clusterType = -1.0f;

	concaveClustering.bDetectGroundPlane = false;
	concaveClustering.bOverlappingClusters = true;

	VN *pModel;

	pModel = new VN;

	VN_::CreateConvex(pModel, pMem0);

	models.push_back(pModel);

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

	pModel = new VN;

	VN_::CreateBanana(pModel, pMem0);

	models.push_back(pModel);

	CreateParamList();

	paramList.LoadParams(cfgFileName);

	clusteringTolerance = 3.0f * convexClustering.kNoise * 2.0f / pSurfelDetector->kPlane;

	alignment.pMem0 = pMem0;
	alignment.pMem = pMem;
	alignment.pSurfels = pSurfels;
	alignment.pSurfelDetector = pSurfelDetector;

	alignment.problem = RVLRECOGNITION_PROBLEM_CLASSIFICATION;

	alignment.Init(cfgFileName);

	pObjects = alignment.pObjects;

	alignment.modelDataBase = RVLCreateString(modelDataBase);

	printf("Loading CTI database...");

	alignment.LoadModelDataBase();

	printf("completed.\n");

	//alignment.bGroundPlaneRFDescriptors = true;
}

void VNClassifier::CreateParamList()
{
	paramList.m_pMem = pMem0;

	RVLPARAM_DATA *pParamData;

	paramList.Init();

	pParamData = paramList.AddParam("VN.kMaxMatchCost", RVLPARAM_TYPE_FLOAT, &kMaxMatchCost);
	pParamData = paramList.AddParam("VN.maxnSCClusters", RVLPARAM_TYPE_INT, &maxnSCClusters);
	pParamData = paramList.AddParam("VN.maxnSUClusters", RVLPARAM_TYPE_INT, &maxnSUClusters);
	pParamData = paramList.AddParam("VN.maxnSTClusters", RVLPARAM_TYPE_INT, &maxnSTClusters);
	pParamData = paramList.AddParam("Recognition.mode", RVLPARAM_TYPE_ID, &mode);
	paramList.AddID(pParamData, "TRAINING", RVLRECOGNITION_MODE_TRAINING);
	paramList.AddID(pParamData, "RECOGNITION", RVLRECOGNITION_MODE_RECOGNITION);
	pParamData = paramList.AddParam("ModelDataBase", RVLPARAM_TYPE_STRING, &modelDataBase); //Vidovic
	pParamData = paramList.AddParam("ModelsInDataBase", RVLPARAM_TYPE_STRING, &modelsInDataBase); //Vidovic
	pParamData = paramList.AddParam("VN.voxelSize", RVLPARAM_TYPE_FLOAT, &voxelSize);
	pParamData = paramList.AddParam("VN.connectedComponentMaxDist", RVLPARAM_TYPE_FLOAT, &connectedComponentMaxDist);
	pParamData = paramList.AddParam("VN.connectedComponentMinSize", RVLPARAM_TYPE_INT, &connectedComponentMinSize);
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
	RVL_DELETE_ARRAY(refModel.d);
	RVL_DELETE_ARRAY(refModel.bd);
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

	// Selection color for visualization.

	uchar SelectionColor[] = {0, 255, 0};

	//// Cluster visualization.

	//pSurfels->NodeColors(SelectionColor);

	//Visualizer visualizer;

	//visualizer.Create();

	//RVL_DELETE_ARRAY(convexClustering.clusterColor);

	//RandomColors(SelectionColor, convexClustering.clusterColor, convexClustering.clusters.n);

	//convexClustering.InitDisplay(&visualizer, pMesh, SelectionColor);

	//convexClustering.Display();

	////RVL_DELETE_ARRAY(concaveClustering.clusterColor);

	////RandomColors(SelectionColor, concaveClustering.clusterColor, concaveClustering.clusters.n);

	////concaveClustering.InitDisplay(&visualizer, &mesh, SelectionColor);

	////concaveClustering.Display();

	//visualizer.Run();

	// Surfel visualization.

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
	char *modelDescriptorFileName;
	int modelID, modelID_;
	FILE *fpDescriptor;

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

			modelDescriptorFileName = RVLCreateFileName(modelFilePath, ".ply", -1, ".vnd");

			fpDescriptor = fopen(modelDescriptorFileName, "w");

			delete[] modelDescriptorFileName;

			SaveDescriptor(fpDescriptor, dS, bdS, modelID_, iMetaModel);

			fclose(fpDescriptor);

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

void VNClassifier::Interpret(
	Mesh *pMesh,
	int iClass)
{
	// Detect surfels.

	pSurfels->Init(pMesh);

	pSurfelDetector->Init(pMesh, pSurfels, pMem);

	printf("Segmentation to surfels...");

	pSurfelDetector->Segment(pMesh, pSurfels);

	printf("completed.\n");

	int nSurfels = pSurfels->NodeArray.n;

	printf("No. of surfels = %d\n", nSurfels);

	// Relations between adjacent surfels.

#ifdef RVLSURFEL_IMAGE_ADJACENCY
	pSurfels->SurfelRelations(pMesh);
#endif

	// Detect ground plane.

	Array<int> groundPlaneSurfelArray;

	groundPlaneSurfelArray.Element = NULL;

	if (pSurfels->bGroundContactVertices)
	{
		groundPlaneSurfelArray.Element = new int[pSurfels->NodeArray.n];

		pSurfels->DetectDominantPlane(groundPlaneSurfelArray, NGnd, dGnd);

		bGnd = true;
	}

	// Detect vertices.

	printf("Detect vertices.\n");

	pSurfels->DetectVertices(pMesh);

	// Detect objects as connected surfel sets.

	pObjects->pMesh = pMesh;

	pObjects->CreateObjectsAsConnectedComponents(groundPlaneSurfelArray, connectedComponentMaxDist, connectedComponentMinSize);

	RVL_DELETE_ARRAY(groundPlaneSurfelArray.Element);

	// Sort objects.

	pObjects->nValidObjects = -1;
	pObjects->sortedObjectArray.n = -1;

	pObjects->SortObjects();

	// Assign vertices to objects.

	pObjects->GetVertices();

	// Detect objects in VOI

	if (pObjects->b3DNetVOI)
		pObjects->ObjectsInVOI();

	// Get foreground object.

	int iObject = pObjects->GetForegroundObject();

	// Create CTIs.

	alignment.CTISet.Init();

	if (iObject >= 0)
	{
		RVLCOPY3VECTOR(NGnd, alignment.NGnd);
		alignment.dGnd = dGnd;
		alignment.bGnd = true;

		SURFEL::Object *pObject = pObjects->objectArray.Element + iObject;

		alignment.CTIs(pObject->surfelList, pObject->iVertexArray, 0, iObject, &(alignment.CTISet), pMem);

		alignment.CTISet.CopyCTIsToArray();

		// Save model instances to a file.

		printf("Save model instances to a file.\n");

		char *PSGModelInstanceFileName = RVLCreateString(alignment.sceneFileName);

		sprintf(PSGModelInstanceFileName + strlen(PSGModelInstanceFileName) - 3, "cti");

		FILE *fp = fopen(PSGModelInstanceFileName, "w");

		delete[] PSGModelInstanceFileName;

		alignment.SaveModelInstances(fp); //Vidovic

		fclose(fp);

		// Align tbe scene object with a model.

		int iModel;
		float R[9], t[3];

		alignment.Classify(pMesh, classArray.Element[iClass].iRefInstance, classArray.Element[iClass].iRefInstance, iModel, R, t);

		FILE *fpR = fopen("R.txt", "w");

		PrintMatrix<float>(fpR, R, 3, 3);

		fclose(fpR);

		///

		Camera camera;

		camera.fu = 525;
		camera.fv = 525;
		camera.uc = 320;
		camera.vc = 240;

		Rect<float> ROI;

		pSurfels->GetDepthImageROI(pObject->iVertexArray, camera, ROI);

		Array2D<float> imagePtArray;

		SampleRect<float>(&ROI, 10.0f, 16, imagePtArray);

		int iMetaModel = classArray.Element[iClass].iMetaModel;

		VN *pModel = models[iMetaModel];

		Array2D<float> PtArray;

		PtArray.w = 3;
		PtArray.h = imagePtArray.h;

		PtArray.Element = new float[PtArray.w * PtArray.h];

		pModel->Project(refModel.d, R, t, camera, imagePtArray, PtArray);

		fp = fopen("P.txt", "w");

		PrintMatrix<float>(fp, PtArray.Element, PtArray.h, PtArray.w);

		fclose(fp);

		// Visualization

		unsigned char SelectionColor[] = {0, 255, 0};

		pSurfels->NodeColors(SelectionColor);

		Visualizer visualizer;

		visualizer.Create();

		pObjects->InitDisplay(&visualizer, pMesh, SelectionColor);
		pObjects->Display();

		unsigned char color[] = { 0, 128, 255 };

		Array<Point> PtArray_;

		PtArray_.Element = new Point[PtArray.h];

		PtArray_.n = 0;

		int iPt;
		float *P, *P_;

		for (iPt = 0; iPt < PtArray.h; iPt++)
		{
			P = PtArray.Element + PtArray.w * iPt;

			if (P[2] <= 2.0f)
			{
				P_ = PtArray_.Element[PtArray_.n++].P;

				RVLCOPY3VECTOR(P, P_);
			}
		}

		visualizer.DisplayPointSet<float, Point>(PtArray_, color, 6.0f);

		visualizer.Run();

		delete[] imagePtArray.Element;
		delete[] PtArray.Element;
	}
}

void VNClassifier::SaveDescriptor(
	FILE *fp,
	float *d,
	bool *bd,
	int iModel,
	int iMetaModel)
{
	VN *pModel = models[iMetaModel];

	fprintf(fp, "%d\t%d\t", iModel, iMetaModel);

	int i;

	for (i = 0; i < pModel->featureArray.n; i++)
		fprintf(fp, "%f\t", d[i]);

	for (i = 0; i < pModel->featureArray.n; i++)
		fprintf(fp, "%d\t", (int)(bd[i]));
}

void VNClassifier::LoadDescriptor(
	FILE *fp,
	float *d,
	bool *bd,
	int &iModel,
	int &iMetaModel)
{
	fscanf(fp, "%d\t%d\t", &iModel, &iMetaModel);

	VN *pModel = models[iMetaModel];

	int i;

	for (i = 0; i < pModel->featureArray.n; i++)
		fscanf(fp, "%f\t", d + i);

	int ibd;

	for (i = 0; i < pModel->featureArray.n; i++)
	{
		fscanf(fp, "%d\t", &ibd);

		bd[i] = (bool)ibd;
	}
}

void VN_::_3DNetDatabaseClasses(VNClassifier *pClassifier)
{
	pClassifier->classArray.n = 10;

	pClassifier->classArray.Element = new RECOG::ClassData[pClassifier->classArray.n];

	RECOG::ClassData *pClass;

	// class banana

	pClass = pClassifier->classArray.Element + 1;
	pClass->iMetaModel = RVLVN_METAMODEL_BANANA;
	pClass->iFirstInstance = 10;
	pClass->nInstances = 6;
	pClass->iRefInstance = 15;

	// class car

	pClass = pClassifier->classArray.Element + 4;
	pClass->iMetaModel = RVLVN_METAMODEL_HAMMER;
	pClass->iFirstInstance = 100;
	pClass->nInstances = 26;
	pClass->iRefInstance = 100;

	// class bottle

	pClass = pClassifier->classArray.Element + 2;
	pClass->iMetaModel = RVLVN_METAMODEL_BOTTLE;
	pClass->iFirstInstance = 16;
	pClass->nInstances = 69;
	pClass->iRefInstance = 16;

	// class bowl

	pClass = pClassifier->classArray.Element + 3;
	pClass->iMetaModel = RVLVN_METAMODEL_BOWL;
	pClass->iFirstInstance = 85;
	pClass->nInstances = 15;
	pClass->iRefInstance = 88;

	// class donut

	pClass = pClassifier->classArray.Element + 5;
	pClass->iMetaModel = RVLVN_METAMODEL_TORUS;
	pClass->iFirstInstance = 126;
	pClass->nInstances = 10;
	pClass->iRefInstance = 126;

	// class hammer

	pClass = pClassifier->classArray.Element + 6;
	pClass->iMetaModel = RVLVN_METAMODEL_HAMMER;
	pClass->iFirstInstance = 136;
	pClass->nInstances = 32;
	pClass->iRefInstance = 136;

	// class toilet paper

	pClass = pClassifier->classArray.Element + 8;
	pClass->iMetaModel = RVLVN_METAMODEL_TORUS;
	pClass->iFirstInstance = 190;
	pClass->nInstances = 6;
	pClass->iRefInstance = 190;

	// class mug

	pClass = pClassifier->classArray.Element + 9;
	pClass->iMetaModel = RVLVN_METAMODEL_MUG;
	pClass->iFirstInstance = 196;
	pClass->nInstances = 61;
	pClass->iRefInstance = 199;
}
