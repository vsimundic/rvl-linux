//#include "stdafx.h"

#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
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
#include <random> //VIDOVIC
#include <nanoflann.hpp>

//#define RVLPSGM_CTIMESH_DEBUG
//#define RVLPSGM_MATCHTGS_CREATE_SCENE_TG
//#define RVLPSGM_MATCHTGS_CREATE_SCENE_VG
#define RVLPSGM_MATCHCTI_MATCH_MATRIX //activate this flag regardless to version 170601 - Vidovic 20.07.2017
#define RVLPSGM_MATCH_HYPOTHESIS_LOG

using namespace RVL;
using namespace RECOG;

PSGM::PSGM()
{
	mode = RVLRECOGNITION_MODE_RECOGNITION;
	problem = RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION;
	bZeroRFDescriptor = false;
	bGTRFDescriptors = false;
	bGroundPlaneRFDescriptors = false;
	bMatchRANSAC = false;
	bWholeMeshCluster = false;
	bDetectGroundPlane = true;
	bOverlappingClusters = false;

	nDominantClusters = 1;
	kNoise = 1.2f;
	minInitialSurfelSize = 20;
#ifdef RVLVERSION_170601
	minVertexPerc = 50;
#else
	minVertexPerc = 100;
#endif
	kReferenceSurfelSize = 0.2f;
	kReferenceTangentSize = 0.3f;
	baseSeparationAngle = 22.5f;
	//edgeTangentAngle = 100.0f;	
	nModels = 35; //Vidovic
	nMSegments = 3; //Vidovic
	minClusterSize = 400;
	maxClusterSize = 66122;
	minSignificantClusterSize = 3200;
	minClusterBoundaryDiscontinuityPerc = 75;
	minClusterNormalDistributionStd = 0.1f;
	groundPlaneTolerance = 0.020f;
	gndCTIThr = 0.015f;
	clusterType = 1.0f;

	convexTemplate66.n = 66;
	convexTemplate66.Element = new RECOG::PSGM_::Plane[convexTemplate66.n];

	CreateTemplate66();

	convexTemplate = convexTemplate66;

	CreateTemplateBox();

	//Vidovic
	centroidID.n = 6;
	centroidID.Element = new QLIST::Index[centroidID.n];

	ConvexTemplateCentoidID();
	//END Vidovic

	vpObjectDetector = NULL;

	pSVertexGraph = NULL;

	clusters.Element = NULL;
	clusterMap = NULL;
	clusterMem = NULL;
	clusterSurfelMem = NULL;
	clusterVertexMem = NULL;
	//modelInstanceMem = NULL;
	sceneFileName = NULL;
	modelInstanceDB.Element = NULL; //Vidovic
	modelInstanceDB.n = 0; //Vidovic
	modelDataBase = NULL; //Vidovic
	modelsInDataBase = NULL; //Vidovic
	sceneMIMatch = NULL; //Vidovic	
	//matchMatrixMem = NULL;
	//matchMatrix.Element = NULL;
	sceneSegmentMatches.Element = NULL;
	sceneSegmentMatchesArray.Element = NULL;
	sceneSegmentMatchesArray.n = 0;
	bestSceneSegmentMatches.Element = NULL;
	bestSceneSegmentMatchesArray.Element = NULL;
	bestSceneSegmentMatchesArray.n = 0;
	CTIMatchMem = NULL;
	clusterColor = NULL;

	//nSamples = 20; //Vidovic
	stdNoise = 2; //Vidovic

	bNormalValidityTest = true; //Vidovic

	iScene = 0;

	//Vidovic
	pECCVGT = new ECCVGTLoader;

	scoreMatchMatrix.Element = NULL;
	scoreMatchMatrix.n = 0;

	scoreMatchMatrixICP.Element = NULL;
	scoreMatchMatrixICP.n = 0;

	nBestMatches = 100; //add loading from file
	//nBestMatches = 20; //add loading from file

	//Arrays allocation for Match function
	iValidSampleCandidate.Element = new QLIST::Index[convexTemplate.n];
	
	iValid.Element = new QLIST::Index[convexTemplate.n];
	
	iRansacCandidates.Element = new QLIST::Index[26]; //max 26 planes which satisfy condition
	
	iConsensus.Element = new QLIST::Index[convexTemplate.n];
	
	iConsensusTemp.Element = new QLIST::Index[convexTemplate.n];

	pCTImatchesArray.Element = NULL;
	pCTImatchesArray.n = 0;

	segmentGT.Element = NULL;

	e.Element = NULL;

	tBestMatch.Element = NULL;

	score.Element = NULL;

	pCTImatchesArray.Element = NULL;

	scoreMatchMatrix.Element = NULL;
	scoreMatchMatrix.n = 0;

	icpTMatrix = NULL;

	//fpTime = fopen("C:\\RVL\\MatchTime_WithoutRansac.txt", "w");
	//End Vidovic

	bGnd = false;
	bBoundingPlanes = false;

	MTGSet.nodeSimilarityThr = 0.0f;
	MTGSet.eLimit = 20.0f;

	TemplateMatrix(MTGSet.A);

	STGSet.nodeSimilarityThr = 0.003f;

	TemplateMatrix(STGSet.A);

	createMatchGT = false;

	//depthImg = new unsigned short(480*640); //Vidovic
	depth = cv::Mat(480, 640, CV_16UC1, cv::Scalar::all(0));

	iCorrectClass = 0;
}


PSGM::~PSGM()
{
	RVL_DELETE_ARRAY(clusters.Element);
	RVL_DELETE_ARRAY(clusterMap);
	RVL_DELETE_ARRAY(clusterMem);
	RVL_DELETE_ARRAY(clusterSurfelMem);
	RVL_DELETE_ARRAY(clusterVertexMem);
	RVL_DELETE_ARRAY(convexTemplate66.Element);
	RVL_DELETE_ARRAY(convexTemplateBox.Element);
	//RVL_DELETE_ARRAY(modelInstanceMem);
	RVL_DELETE_ARRAY(sceneFileName);
	RVL_DELETE_ARRAY(modelInstanceDB.Element); //Vidovic
	RVL_DELETE_ARRAY(modelDataBase); //Vidovic
	RVL_DELETE_ARRAY(modelsInDataBase); //Vidovic
	RVL_DELETE_ARRAY(sceneMIMatch); //Vidovic
	RVL_DELETE_ARRAY(centroidID.Element); //Vidovic
	RVL_DELETE_ARRAY(pCTImatchesArray.Element); //Vidovic
	RVL_DELETE_ARRAY(segmentGT.Element); //Vidovic
	//RVL_DELETE_ARRAY(matchMatrixMem);
	//RVL_DELETE_ARRAY(matchMatrix.Element);
	RVL_DELETE_ARRAY(sceneSegmentMatches.Element);
	RVL_DELETE_ARRAY(sceneSegmentMatchesArray.Element);
	RVL_DELETE_ARRAY(bestSceneSegmentMatches.Element);
	RVL_DELETE_ARRAY(bestSceneSegmentMatchesArray.Element);
	RVL_DELETE_ARRAY(CTIMatchMem);
	RVL_DELETE_ARRAY(clusterColor);

	//Vidovic
	int iSSegment;

	pECCVGT->~ECCVGTLoader();

	//Delete arrays used in Match() function
	RVL_DELETE_ARRAY(iValidSampleCandidate.Element);

	RVL_DELETE_ARRAY(iValid.Element);

	RVL_DELETE_ARRAY(iRansacCandidates.Element);

	RVL_DELETE_ARRAY(iConsensus.Element);

	RVL_DELETE_ARRAY(iConsensusTemp.Element);	

	for (int i = 0; i < MCTISet.pCTI.n; i++)
	{
		RVL_DELETE_ARRAY(e.Element[i].Element);
		RVL_DELETE_ARRAY(tBestMatch.Element[i].Element);
	}

	RVL_DELETE_ARRAY(e.Element);

	RVL_DELETE_ARRAY(tBestMatch.Element);

	RVL_DELETE_ARRAY(score.Element);

	RVL_DELETE_ARRAY(pCTImatchesArray.Element);

	//delete scoreMatchMatrix	
	for (iSSegment = 0; iSSegment < scoreMatchMatrix.n; iSSegment++)
	{
		RVL_DELETE_ARRAY(scoreMatchMatrix.Element[iSSegment].Element);
	}

	for (iSSegment = 0; iSSegment < scoreMatchMatrixICP.n; iSSegment++)
	{
		RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element[iSSegment].Element);
	}

	RVL_DELETE_ARRAY(scoreMatchMatrix.Element);

	RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element);

	if (icpTMatrix)
		delete[] icpTMatrix;

	//fclose(fpTime);

	//delete depthImg;
	//End Vidovic

	if (vpObjectDetector)
	{
		ObjectDetector *pObjectDetector = (ObjectDetector *)vpObjectDetector;

		delete pObjectDetector;
	}

	if (pSVertexGraph)
		delete pSVertexGraph;
}

void PSGM::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("Recognition.mode", RVLPARAM_TYPE_ID, &mode);
	ParamList.AddID(pParamData, "TRAINING", RVLRECOGNITION_MODE_TRAINING);
	ParamList.AddID(pParamData, "RECOGNITION", RVLRECOGNITION_MODE_RECOGNITION); //Vidovic
	ParamList.AddID(pParamData, "CREATE_CTIS", RVLRECOGNITION_MODE_PSGM_CREATE_CTIS);
	pParamData = ParamList.AddParam("Recognition.problem", RVLPARAM_TYPE_ID, &problem);
	ParamList.AddID(pParamData, "SHAPE_INSTANCE_DETECTION", RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION);
	ParamList.AddID(pParamData, "CLASSIFICATION", RVLRECOGNITION_PROBLEM_CLASSIFICATION);
	pParamData = ParamList.AddParam("PSGM.nDominantClusters", RVLPARAM_TYPE_INT, &nDominantClusters);
	pParamData = ParamList.AddParam("PSGM.kNoise", RVLPARAM_TYPE_FLOAT, &kNoise);
	pParamData = ParamList.AddParam("PSGM.minInitialSurfelSize", RVLPARAM_TYPE_INT, &minInitialSurfelSize);
	pParamData = ParamList.AddParam("PSGM.minVertexPerc", RVLPARAM_TYPE_INT, &minVertexPerc);
	pParamData = ParamList.AddParam("PSGM.kReferenceSurfelSize", RVLPARAM_TYPE_FLOAT, &kReferenceSurfelSize);
	pParamData = ParamList.AddParam("PSGM.kReferenceTangentSize", RVLPARAM_TYPE_FLOAT, &kReferenceTangentSize);
	pParamData = ParamList.AddParam("PSGM.baseSeparationAngle", RVLPARAM_TYPE_FLOAT, &baseSeparationAngle);
	pParamData = ParamList.AddParam("PSGM.wholeMeshCluster", RVLPARAM_TYPE_BOOL, &bWholeMeshCluster);
	//pParamData = ParamList.AddParam("PSGM.edgeTangentAngle", RVLPARAM_TYPE_FLOAT, &edgeTangentAngle);
	pParamData = ParamList.AddParam("ModelDataBase", RVLPARAM_TYPE_STRING, &modelDataBase); //Vidovic
	pParamData = ParamList.AddParam("ModelsInDataBase", RVLPARAM_TYPE_STRING, &modelsInDataBase); //Vidovic
	pParamData = ParamList.AddParam("PSGM.Match.RANSAC", RVLPARAM_TYPE_BOOL, &bMatchRANSAC); //Vidovic
	//pParamData = ParamList.AddParam("PSGM.RANSAC.nSamples", RVLPARAM_TYPE_INT, &nSamples); //Vidovic
	pParamData = ParamList.AddParam("PSGM.RANSAC.stdNoise", RVLPARAM_TYPE_INT, &stdNoise); //Vidovic
	pParamData = ParamList.AddParam("PSGM.normalValidityTest", RVLPARAM_TYPE_BOOL, &bNormalValidityTest); //Vidovic
	pParamData = ParamList.AddParam("PSGM.SceneMIMatch", RVLPARAM_TYPE_STRING, &sceneMIMatch); //Vidovic
	pParamData = ParamList.AddParam("PSGM.nModels", RVLPARAM_TYPE_INT, &nModels); //Vidovic
	pParamData = ParamList.AddParam("PSGM.nMSegments", RVLPARAM_TYPE_INT, &nMSegments); //Vidovic
	pParamData = ParamList.AddParam("PSGM.minClusterSize", RVLPARAM_TYPE_INT, &minClusterSize);
	pParamData = ParamList.AddParam("PSGM.maxClusterSize", RVLPARAM_TYPE_INT, &maxClusterSize);
	pParamData = ParamList.AddParam("PSGM.minSignificantClusterSize", RVLPARAM_TYPE_INT, &minSignificantClusterSize);
	pParamData = ParamList.AddParam("PSGM.minClusterBoundaryDiscontinuityPerc", RVLPARAM_TYPE_INT, &minClusterBoundaryDiscontinuityPerc);
	pParamData = ParamList.AddParam("PSGM.minClusterNormalDistributionStd", RVLPARAM_TYPE_FLOAT, &minClusterNormalDistributionStd);
	pParamData = ParamList.AddParam("PSGM.groundPlaneTolerance", RVLPARAM_TYPE_FLOAT, &groundPlaneTolerance);
	pParamData = ParamList.AddParam("PSGM.zeroRFDescriptor", RVLPARAM_TYPE_BOOL, &bZeroRFDescriptor);	
	pParamData = ParamList.AddParam("PSGM.GTRFDescriptors", RVLPARAM_TYPE_BOOL, &bGTRFDescriptors);
	pParamData = ParamList.AddParam("PSGM.groundPlaneRFDescriptors", RVLPARAM_TYPE_BOOL, &bGroundPlaneRFDescriptors);
	pParamData = ParamList.AddParam("PSGM.Visualization.hypothesisVisualizationMode", RVLPARAM_TYPE_ID, &(displayData.hypothesisVisualizationMode));
	ParamList.AddID(pParamData, "CTI", RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI);
	ParamList.AddID(pParamData, "PLY", RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY);
	pParamData = ParamList.AddParam("PSGM.symmetryMatchThr", RVLPARAM_TYPE_FLOAT, &symmetryMatchThr);
	pParamData = ParamList.AddParam("PSGM.debug1", RVLPARAM_TYPE_INT, &debug1);
	pParamData = ParamList.AddParam("PSGM.debug2", RVLPARAM_TYPE_INT, &debug2);
}

void PSGM::Init(char *cfgFileName)
{
	if (problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
	{
		ObjectDetector *pObjectDetector = new ObjectDetector;

		vpObjectDetector = pObjectDetector;

		pObjectDetector->pMem0 = pMem0;
		pObjectDetector->pMem = pMem;
		pObjectDetector->cfgFileName = RVLCreateString(cfgFileName);

		pObjectDetector->Init(this);

		pObjectDetector->vpMeshBuilder = vpMeshBuilder;
		pObjectDetector->LoadMesh = LoadMesh;

		pObjectDetector->bSegmentToObjects = true;
		pObjectDetector->bObjectAggregationLevel2 = false;
	}
}

void PSGM::Init(Mesh *pMesh_)
{
	pMesh = pMesh_;

	bGnd = false;
	bBoundingPlanes = false;


}

void PSGM::Interpret(
	Mesh *pMeshIn,
	int iScene)
{
	// Create ordered mesh.
	pMesh = pMeshIn;

	//pMesh->CreateOrderedMeshFromPolyData();

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

		pSurfels->DetectDominantPlane(groundPlaneSurfelArray);

		int i;

		for (i = 0; i < groundPlaneSurfelArray.n; i++)
			pSurfels->NodeArray.Element[groundPlaneSurfelArray.Element[i]].flags |= RVLSURFEL_FLAG_GND;

		bGnd = true;

		Surfel *pGndSurfel = pSurfels->NodeArray.Element + groundPlaneSurfelArray.Element[0];

		float *NGnd_ = pGndSurfel->N;

		RVLCOPY3VECTOR(NGnd_, NGnd);

		dGnd = pGndSurfel->d;
	}

	// Detect vertices.

	printf("Detect vertices.\n");

	pSurfels->DetectVertices(pMesh);

	if (problem == RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION)
	{
		/// Create clusters.

		if (bWholeMeshCluster)
		{
			// Create a single cluster from the whole mesh.

			WholeMeshCluster();
		}
		else
		{
			// Cluster surfels into convex surfaces.

			printf("Detect convex clusters.\n");

			Clusters();
		}

		///

		// Fit model.

		printf("Fit convex template.\n");

		int nClusters = RVLMIN(clusters.n, nDominantClusters);

		char *GTHFileName = NULL;
		FILE *fpGTH = NULL;

		if (bGTRFDescriptors)
		{
			char *GTHFileName = RVLCreateString(sceneFileName);

			sprintf(GTHFileName + strlen(GTHFileName) - 3, "gth");

			fpGTH = fopen(GTHFileName, "w");
		}

		int iCluster;
		RECOG::PSGM_::Cluster *pCluster;
		RECOG::PSGM_::ModelInstance *pModelInstance;
		float R[9];

		//Init CTISet Qlist
		CTISet.Init();//Vidovic

		for (iCluster = 0; iCluster < nClusters; iCluster++)
		{
			pCluster = clusters.Element[iCluster];

			if (bZeroRFDescriptor)
			{
				//QList<RECOG::PSGM_::ModelInstance> *pModelInstanceList = &(pCluster->modelInstanceList); //Vidovic

				//RVLQLIST_INIT(pModelInstanceList); //Vidovic

				//AddReferenceFrame(iCluster); //Vidovic

				pModelInstance = AddReferenceFrame();

				pModelInstance->iCluster = iCluster;
			}
			else if (bGTRFDescriptors)
			{
				//QList<RECOG::PSGM_::ModelInstance> *pModelInstanceList = &(pCluster->modelInstanceList); //Vidovic

				//RVLQLIST_INIT(pModelInstanceList); //Vidovic

				Array<GTInstance> *pGT = pECCVGT->GT.Element + iScene;

				int iGTInstance;
				GTInstance *pGTInstance;

				for (iGTInstance = 0; iGTInstance < pGT->n; iGTInstance++)
				{
					pGTInstance = pGT->Element + iGTInstance;

					RVLSCALEMX3X3(pGTInstance->R, 1000.0f, R);

					//AddReferenceFrame(iCluster, R, pGTInstance->t); //Vidovic

					pModelInstance = AddReferenceFrame(R, pGTInstance->t); //Vidovic

					pModelInstance->iCluster = iCluster;

					fprintf(fpGTH, "%d\t%d\n", iCluster, pGTInstance->iModel);
				}
			}
			else
				ReferenceFrames(iCluster);
		}

		//Vidovic
		bool bNormalValidityTest_ = bNormalValidityTest;

		if (mode == RVLRECOGNITION_MODE_TRAINING)
			bNormalValidityTest = false;

		pModelInstance = CTISet.CTI.pFirst;

		while (pModelInstance)
		{
			pCluster = clusters.Element[pModelInstance->iCluster];

			FitModel(pCluster->iVertexArray, pModelInstance);

			pModelInstance = pModelInstance->pNext;
		}

		bNormalValidityTest = bNormalValidityTest_;

		//Copy CTIs from Qlist to Array
		CTISet.CopyCTIsToArray();
		//END Vidovic

		// Create tangent graphs.

		if (mode == RVLRECOGNITION_MODE_TRAINING)
		{
			VertexGraph *pVertexGraph = new VertexGraph;

			pVertexGraph->idx = iScene;

			pVertexGraph->pMem = MTGSet.pMem;

			MTGSet.vertexGraphs.push_back(pVertexGraph);

			pVertexGraph->Create(pSurfels);

			//pVertexGraph->Clustering();

			TG *pTG = new TG;

			float R[9], t[3];

			RVLUNITMX3(R);
			RVLNULL3VECTOR(t);

			pTG->iObject = iScene;

			pTG->iVertexGraph = pVertexGraph->idx;

			Array<int> iVertexArray;

			iVertexArray.n = pVertexGraph->NodeArray.n;
			iVertexArray.Element = new int[iVertexArray.n];

			int i;

			for (i = 0; i < iVertexArray.n; i++)
				iVertexArray.Element[i] = i;

			pTG->A = MTGSet.A;

			pTG->Create(pVertexGraph, iVertexArray, R, t, &MTGSet, pSurfels, true);

			delete[] iVertexArray.Element;

			MTGSet.TGs.push_back(pTG);
		}

		//Vidovic
		//Match scene MI to model MI
		if (mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
#ifndef RVLVERSION_170601
			VertexGraph vertexGraph;

			vertexGraph.idx = iScene;

			vertexGraph.pMem = pMem;

			vertexGraph.Create(pSurfels);

			vertexGraph.Clustering();

			char *vertexGraphFileName = RVLCreateFileName(sceneFileName, ".ply", -1, ".vgr");

			FILE *fp = fopen(vertexGraphFileName, "w");

			delete[] vertexGraphFileName;

			vertexGraph.Save(fp);

			fclose(fp);
#endif

#ifdef RVLVERSION_170601
			Match();
#endif
		}

		if (bGTRFDescriptors)
		{
			if (fpGTH)
				fclose(fpGTH);

			RVL_DELETE_ARRAY(GTHFileName);
		}
	}	// if(problem == RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION)
	else if (problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
	{
		// Detect objects as connected surfel sets.

		pObjects->pMesh = pMesh;

		pObjects->CreateObjectsAsConnectedComponents(groundPlaneSurfelArray);

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

		CTISet.Init();

		if (iObject >= 0)
		{
			SURFEL::Object *pObject = pObjects->objectArray.Element + iObject;

			CTIs(pObject->surfelList, pObject->iVertexArray, 0, iObject, &CTISet, pMem);

			CTISet.CopyCTIsToArray();
		}

		// Save model instances to a file.

		printf("Save model instances to a file.\n");

		char *PSGModelInstanceFileName = RVLCreateString(sceneFileName);

		sprintf(PSGModelInstanceFileName + strlen(PSGModelInstanceFileName) - 3, "cti");

		FILE *fp = fopen(PSGModelInstanceFileName, "w");

		SaveModelInstances(fp); //Vidovic

		fclose(fp);

		delete[] PSGModelInstanceFileName;

		RVL_DELETE_ARRAY(groundPlaneSurfelArray.Element);

		// Classification

		Classify(pMesh);

		// Create vertex graph.

		//if (pSVertexGraph)
		//	delete pSVertexGraph;

		//pSVertexGraph = new VertexGraph;

		//pSVertexGraph->idx = iScene;

		//pSVertexGraph->pMem = pMem;

		//pSVertexGraph->Create(pSurfels);

		//// Vertex graph clustering.

		//pSVertexGraph->Clustering();

		//// Save vertex graph.

		//char *vertexGraphFileName = RVLCreateFileName(sceneFileName, ".ply", -1, ".vgr");

		//FILE *fp = fopen(vertexGraphFileName, "w");

		//delete[] vertexGraphFileName;

		//pSVertexGraph->Save(fp);

		//fclose(fp);

	}	// if(problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
}

//PETRA

//void PSGM::InterpreteCTIS(Mesh *pMesh)
//{
//
//	//Load matrix of primitives M
//	Eigen::Matrix<float, 9, 66> Mt;
//	Eigen::Matrix<float, 66, 9> M;
//
//	char *buffer = new char[594 * 8];
//	ifstream datafile("D:\\ARP3D\\Matlab_new\\M.bin", ios::in | ios::binary);
//	datafile.read(buffer, 594 * 8);
//	double *a = (double*)buffer;
//	for (int i = 0; i < 9; i++)
//	{
//		for (int j = 0; j < 66; j++)
//		{
//			Mt(i, j) = a[i * 66 + j];
//		}
//	}
//	M = Mt.transpose();
//	delete[] buffer;
//
//	//Load matrix QM
//	Eigen::MatrixXf QMt(4056, 9);
//	Eigen::MatrixXf QM(9, 4056);
//
//	buffer = new char[36504 * 8];
//	ifstream datafile2("D:\\ARP3D\\Matlab_new\\Q.bin", ios::in | ios::binary);
//	datafile2.read(buffer, 36504 * 8);
//	a = (double*)buffer;
//	for (int i = 0; i < 4056; i++)
//	{
//		for (int j = 0; j < 9; j++)
//		{
//			QMt(i, j) = a[i * 9 + j];
//		}
//	}
//	QM = QMt.transpose();
//	delete[] buffer;
//
//
//	//Load ModelDB CTIs
//	MCTIset.LoadSMCTI("D:\\ARP3D\\modelDB.dat", &convexTemplate); //needs to be in cfg file
//	RECOG::PSGM_::ModelInstance *pMCTI;
//	pMCTI = MCTIset.CTIArr.Element;
//	RECOG::PSGM_::ModelInstanceElement *pMIE;
//
//	//Number of model segments and number of models
//	int nSM = 3; //segments per model
//	int nSM_total = MCTIset.SegmentCTIs.n;
//	int nM = 35; //nSM_total / nSM
//
//	//Create matrix of model descriptors dM
//	Eigen::MatrixXf dM(66, MCTIset.SegmentCTIs.n);
//	for (int i = 0; i < MCTIset.SegmentCTIs.n; i++)
//	{
//		pMIE = pMCTI->modelInstance.Element;
//		for (int j = 0; j < 66; j++)
//		{
//			dM.block<1, 1>(j, i) << pMIE->d;
//			pMIE++;
//		}
//		pMCTI++;
//	}
//
//	//Load Scene CTIs
//	CTIset.LoadSMCTI("D:\\ARP3D\\ECCV_dataset\\CTIs4\\frame_20111220T111153.549117.cti", &convexTemplate);
//	RECOG::PSGM_::ModelInstance *pCTI;
//	pCTI = CTIset.CTIArr.Element;
//	RECOG::PSGM_::ModelInstanceElement *pSIE;
//
//
//	// Number of scene segments
//	int nSS = CTIset.SegmentCTIs.n;
//
//	// Matching loop:
//
//	int iMIE, iValid, rows;
//
//	//Translation vector (for visualisation purposes)
//	Eigen::MatrixXf t;
//
//	// Set all errors elements to -1 for future handling
//	for (int i = 0; i < nSS*nM*nSM; i++)
//	{
//		SMatch[i].Eseg = -1;
//	}
//
//	// Loop trough each Scene CTI 
//	// CTI-s are sorted in segments
//	Array<SortIndex<float>> iSortedSegmentCTI;
//	iSortedSegmentCTI.n = nM * nSM;
//	SortIndex<float> *sortedMatches_;
//
//	int iMatch = 0, brojac, iCTI = 0;
//
//	pMCTI = MCTIset.CTIArr.Element;
//	for (int iSS = 0; iSS < nSS; iSS++)
//	{
//		brojac = CTIset.SegmentCTIs.Element[iSS].n; //nCTI(iSS);
//		while (brojac != 0)
//		{
//			//MatchInPrimitiveSpace(QM, M, iCTI);
//			CTIMatch(dM, iCTI);
//			UpdateMatchMatrix(SMatch, iCTI);
//
//			iCTI++;
//			pCTI++;
//			brojac--;
//		}
//
//		// Sort CTIs in each Segment by cost e
//		int iValidMatches = 0;  // for valid matches
//		sortedMatches_ = sortedMatches + iSS * iSortedSegmentCTI.n;
//
//		for (int j = 0; j < iSortedSegmentCTI.n; j++, iMatch++)
//		{
//			if (SMatch[iMatch].Eseg != -1)
//			{
//				iValidMatches++;
//				sortedMatches_[j].idx = iMatch;
//				sortedMatches_[j].cost = SMatch[iMatch].Eseg;
//			}
//			else
//			{
//				sortedMatches_[j].cost = 1000000; //to be in the end of sorted list
//			}
//		}
//
//		iSortedSegmentCTI.Element = sortedMatches_;
//
//
//		BubbleSort<SortIndex<float>>(iSortedSegmentCTI);
//	}
//
//#ifdef Visualization
//	// Visualization loop:
//	printf("\n\n-------Visualization-------\n\n");
//	int scale = 1000;
//	int iSS_v;
//	int iMatch_v;
//	char k;
//	Eigen::VectorXf dS_v(66);
//	Eigen::VectorXi validS_v(66);
//	bool exit = false;
//
//	while (exit == false)
//	{
//		printf("\nPress q and enter to exit.\nPress c and enter to continue.\n\n");
//
//		scanf(" %c", &k);
//		if (k == 'q')//(GetAsyncKeyState(VK_ESCAPE))
//		{
//			exit = true;
//			break;
//		}
//		else if (k = 'c')
//		{
//			do
//			{
//				printf("iSS: \n");
//				scanf("%d", &iSS_v);
//
//				printf("iMatch_v: \n");
//				scanf("%d", &iMatch_v);
//			} while (iSS_v > nSS /*|| iMatch_v>iSortedSegmentCTI.n*/);
//
//			int idxCTI = sortedMatches[nM*nSM*iSS_v + iMatch_v].idx;
//
//			pCTI = CTIset.CTI.Element + SMatch[idxCTI].iCTIs;
//			pSIE = pCTI->modelInstance.Element;
//			for (iMIE = 0; iMIE < 66; iMIE++)
//			{
//				dS_v(iMIE) = scale * pSIE->d;
//				validS_v(iMIE) = pSIE->valid;
//				pSIE++;
//			}
//			Eigen::VectorXf dM(66);
//			Eigen::MatrixXf dMt(66, 1);
//
//			pMCTI = MCTIset.CTI.Element + SMatch[idxCTI].iCTIm;
//			pMIE = pMCTI->modelInstance.Element;
//			for (int k = 0; k < 66; k++)
//			{
//				dM(k) = pMIE->d;
//				pMIE++;
//			}
//			nT = ConvexTemplatenT();
//			VisualizeCTIMatch(nT.data(), dM.data(), SMatch[idxCTI].t.data(), dS_v.data(), validS_v.data());
//		}
//	}
//
//#endif
//
//}
//void PSGM::CTIMatch(
//	Eigen::MatrixXf dM,
//	int iCTI)
//{
//	Eigen::MatrixXf M(66, 3), dMv(66, dM.cols());
//	Eigen::VectorXi validS(66);
//	Eigen::VectorXf dS, dSv;
//	int iValid = 0;
//
//	RECOG::PSGM_::ModelInstance *pCTI;
//	pCTI = CTIset.CTIArr.Element + iCTI;
//	RECOG::PSGM_::ModelInstanceElement *pSIE;
//	pSIE = pCTI->modelInstance.Element;
//	RECOG::PSGM_::ModelInstance *pMCTI;
//	RECOG::PSGM_::ModelInstanceElement *pMIE;
//
//	int iSS = pCTI->iCluster;
//	int row = 0;
//	for (int iMIE = 0; iMIE < 66; iMIE++)
//	{
//		validS(iMIE) = pSIE->valid;
//		dS(iMIE) = pSIE->d;
//		if (validS(iMIE) == 1)
//		{
//			M.block<1, 3>(row, 0) << nT.block<3, 1>(0, iMIE);
//			dSv(iValid) = dS(iMIE);
//			dMv.block<1, 3>(row, 0) = dM.block<1, 3>(iMIE, 0);
//			row++;
//		}
//
//		pSIE++;
//	}
//
//	// QR decomposition of M
//	Eigen::ColPivHouseholderQR<Eigen::MatrixXf> qr(M);
//	Eigen::MatrixXf Rt_ = qr.matrixQR().triangularView<Eigen::Upper>();
//	Eigen::MatrixXf Qt_ = qr.matrixQ();
//	Eigen::MatrixXf Pt = qr.colsPermutation();
//	Eigen::MatrixXf Rt__;
//
//	for (int x = 0; x < 3; x++)
//	{
//		for (int y = 0; y < 3; y++)
//		{
//			if (Pt(y, x) == 1)
//			{
//				Rt__.block<3, 1>(0, y) = Rt_.block<3, 1>(0, x);
//			}
//		}
//	}
//
//	Eigen::MatrixXf Rt, Qt, ddv;
//	Eigen::MatrixXf Rtt = Rt__.transpose();
//
//	if (Rt__.block<1, 3>(2, 0)*Rtt.block<3, 1>(0, 2) < 1e-20)
//	{
//		Rt = Rt__.block<2, 3>(0, 0);
//		Qt = Qt_.block<9, 2>(0, 0);
//	}
//	else
//	{
//		Qt = Qt_.block<9, 3>(0, 0);;
//		Rt = Rt__;
//	}
//
//	int nM = dM.cols();
//	Eigen::MatrixXf ones(1, nM);
//	Eigen::MatrixXf t_;
//	for (int i = 0; i < nM; i++)
//	{
//		ones(0, i) = 1;
//	}
//
//
//	ddv = dSv*ones - dMv;
//
//	t_ = Qt.transpose()*ddv;
//	E = ddv - Qt*t_;
//
//}
//
//void PSGM::MatchInPrimitiveSpace(
//	Eigen::MatrixXf QM,
//	Eigen::MatrixXf M,
//	int iCTI
//	)
//{
//	//Parameters
//	float scale = 1000.0;
//	int nM = 35; //in future this needs to be loaded from modelDB
//	int nSM = 3; //in future this needs to be loaded from modelDB
//
//
//	RECOG::PSGM_::ModelInstance *pCTI;
//	pCTI = CTIset.CTIArr.Element + iCTI;
//	RECOG::PSGM_::ModelInstanceElement *pSIE;
//	pSIE = pCTI->modelInstance.Element;
//	RECOG::PSGM_::ModelInstance *pMCTI;
//	RECOG::PSGM_::ModelInstanceElement *pMIE;
//
//	int iSS = pCTI->iCluster;
//
//	// Visibility mask
//	Eigen::VectorXi validS(66);
//
//	// Desciptor
//	Eigen::VectorXf dS(66);
//
//	// Determine the number of rows (rows=iValid) of Mv
//	int rows = 0, iMIE;
//	for (iMIE = 0; iMIE < 66; iMIE++)
//	{
//		validS(iMIE) = pSIE->valid; // Filling visibility mask
//		dS(iMIE) = pSIE->d; // Filling descriptor
//		if (validS(iMIE) == 1)
//			rows++;
//		pSIE++;
//	}
//
//	// Search for valids and create dv and Mv
//	Eigen::MatrixXf Mv(rows, 9);
//	Eigen::MatrixXf dv(rows, 1);
//	Eigen::MatrixXf D(CTI.n, rows); //matrix of descriptors
//	int iValid = 0;
//	for (iMIE = 0; iMIE < 66; iMIE++)
//	{
//		if (validS(iMIE) == 1)
//		{
//			dv(iValid) = dS(iMIE);
//			Mv.block<1, 9>(iValid, 0) << M.block<1, 9>(iMIE, 0);
//			iValid++;
//		}
//		pSIE++;
//	}
//
//	// QR decomposition of M
//	Eigen::ColPivHouseholderQR<Eigen::MatrixXf> qr(Mv);
//	Eigen::MatrixXf R_ = qr.matrixQR().triangularView<Eigen::Upper>();
//	Eigen::MatrixXf Q_ = qr.matrixQ();
//	Eigen::MatrixXf P = qr.colsPermutation();
//	Eigen::MatrixXf Q(iValid, 9), R(9, 9), R_sorted(9, 9);
//
//	for (int x = 0; x < iValid; x++)
//	{
//		for (int y = 0; y < 9; y++)
//		{
//			Q(x, y) = Q_(x, y);
//		}
//	}
//
//	for (int x = 0; x < 9; x++)
//	{
//		for (int y = 0; y < 9; y++)
//		{
//			R(x, y) = R_(x, y);
//		}
//	}
//
//	int m = M.cols();
//	int ms = m - 3;
//	Eigen::MatrixXf q = Q.transpose() * dv;
//	Eigen::MatrixXf Rs(9, ms);
//	Eigen::MatrixXf Rt(9, (m - ms));
//
//	for (int x = 0; x < 9; x++)
//	{
//		for (int y = 0; y < 9; y++)
//		{
//			if (P(y, x) == 1)
//			{
//				R_sorted.block<9, 1>(0, y) = R.block<9, 1>(0, x);
//			}
//		}
//	}
//
//	for (int x = 0; x < 9; x++)
//	{
//		for (int y = 0; y < ms; y++)
//		{
//			Rs(x, y) = R_sorted(x, y);
//		}
//	}
//
//	for (int x = 0; x < 9; x++)
//	{
//		for (int y = 0; y < m - ms; y++)
//		{
//			Rt(x, y) = R_sorted(x, y + ms);
//		}
//	}
//
//	// QR decomposition of Rt
//	Eigen::ColPivHouseholderQR<Eigen::MatrixXf> qrRt(Rt);
//	Eigen::MatrixXf Pt = qrRt.colsPermutation();
//	Eigen::MatrixXf Rt_(3, 3);
//	Eigen::MatrixXf Rt_t(3, 3);
//	Eigen::MatrixXf RRt = qrRt.matrixQR().triangularView<Eigen::Upper>();
//	Eigen::MatrixXf Qt_ = qrRt.matrixQ();
//
//	for (int x = 0; x < 3; x++)
//	{
//		for (int y = 0; y < 3; y++)
//		{
//			if (Pt(y, x) == 1)
//			{
//				Rt_.block<3, 1>(0, y) = RRt.block<3, 1>(0, x);
//			}
//		}
//	}
//	Eigen::MatrixXf Qt = Qt_;
//	Eigen::MatrixXf RT = Rt_;
//	Rt_t = Rt_.transpose();
//
//	if (Rt_.block<1, 3>(2, 0)*Rt_t.block<3, 1>(0, 2) < 1e-20)
//	{
//		RT = Rt_.block<2, 3>(0, 0);
//		Qt = Qt_.block<9, 2>(0, 0);
//	}
//	else
//	{
//		Qt = Qt_.block<9, 3>(0, 0);;
//		RT = Rt_;
//	}
//
//
//	// Match CTI descriptor to model	
//	int Mn = QM.cols();
//	Eigen::MatrixXf s(ms, Mn);
//
//	for (int i = 0; i < ms; i++)
//	{
//		for (int j = 0; j < Mn; j++)
//		{
//			s(i, j) = QM(i, j);
//		}
//	}
//
//	Eigen::MatrixXf jed(1, Mn);
//	for (int i = 0; i < Mn; i++)
//	{
//		jed(0, i) = 1;
//	}
//
//	Eigen::MatrixXf es = q * jed - Rs*s;
//	Eigen::MatrixXf t_ = Qt.transpose() * es;
//	Eigen::MatrixXf e = es - Qt * t_;
//	E.resize(e.cols());
//
//	float *pt_ = t_.data();
//
//	t = RT.inverse()*t_;
//
//	// Calculate E
//	float sum;
//	int iM, iSM;
//	int var;
//
//	// Find E between each Scene segment and each Model segment
//	for (int j = 0; j < e.cols(); j++)
//	{
//		sum = 0;
//		for (int i = 0; i < e.rows(); i++)
//		{
//			sum += e(i, j)*e(i, j);
//		}
//		E(j) = sqrt(sum); // Least square
//	}
//
//}
//
//void PSGM::UpdateMatchMatrix(
//	RECOG::PSGM_::SegmentMatch *SMatch,
//	int iCTI
//	)
//{
//
//	RECOG::PSGM_::ModelInstance *pCTI;
//	pCTI = CTIset.CTIArr.Element + iCTI;
//	RECOG::PSGM_::ModelInstanceElement *pSIE;
//	pSIE = pCTI->modelInstance.Element;
//	RECOG::PSGM_::ModelInstance *pMCTI;
//	RECOG::PSGM_::ModelInstanceElement *pMIE;
//
//	int nM = 35; //from DB
//	int nSM = 3; //from DB
//	int iSM, iM, var, j;
//	int iSS = pCTI->iCluster;
//
//	for (j = 0; j < E.rows(); j++)
//	{
//		// Search for matching model parameters
//		pMCTI = MCTIset.CTIArr.Element + j;
//		pMIE = pMCTI->modelInstance.Element;
//		iM = pMCTI->iModel;
//		iSM = pMCTI->iCluster;
//		var = nM*nSM*iSS + nSM*iM + iSM;
//		if (SMatch[var].Eseg == -1 || E(j) < SMatch[var].Eseg)
//		{
//			SMatch[var].Eseg = E(j);
//			SMatch[var].iCTIm = j;
//			SMatch[var].iCTIs = iCTI;
//			SMatch[var].iSM = iSM;
//			SMatch[var].iSS = iSS;
//			SMatch[var].iM = iM;
//			SMatch[var].t = t.block<3, 1>(0, j);
//		}
//	}
//}
//
//
Eigen::MatrixXf PSGM::ConvexTemplatenT()
{
	Eigen::Matrix<float, 3, 13> nT;
	Eigen::Matrix<float, 3, 11> nT_;
	Eigen::Matrix<float, 3, 66> nT__;
	Eigen::Matrix<int, 8, 3> temp1;
	Eigen::Matrix<float, 3, 1> N;
	Eigen::Matrix<float, 1, 3> Nt;
	Eigen::Matrix<float, 1, 1> NtN;
	Eigen::Matrix<float, 3, 3> R;
	Eigen::Matrix<int, 1, 66> dT;

	float h, q, sh, ch, sq, cq;
	float pi = 3.1415;
	h = pi / 4;
	q = h / 2;
	sh = sin(h);
	ch = cos(h);
	sq = sin(q);
	cq = cos(q);

	//for (int i = 0; i < 3; i++)
	//{
	//	for (int j = 0; j < 13; j++)
	//	{
	//		nT[i][j] = 0;
	//	}
	//}
	//memset(nT->data(), 0, 13 * 3 * sizeof(float));
	nT.Zero();
	nT(0, 0) = 0;
	nT(1, 0) = 0;
	nT(2, 0) = 1;
	nT(0, 1) = 0;
	nT(1, 1) = -ch;
	nT(2, 1) = ch;
	nT(0, 2) = ch;
	nT(1, 2) = 0;
	nT(2, 2) = ch;
	nT(0, 11) = 0;
	nT(1, 11) = ch;
	nT(2, 11) = ch;
	nT(0, 12) = -ch;
	nT(1, 12) = 0;
	nT(2, 12) = ch;

	temp1 << 3, 0, 1, 4, 0, 2, 5, 1, 2, 6, 0, 11, 7, 0, 12, 8, 2, 11, 9, 1, 12, 10, 11, 12;

	for (int i = 0; i < temp1.rows(); i++)
	{
		int column1, column2, column3;
		column1 = temp1(i, 1);
		column2 = temp1(i, 2);
		column3 = temp1(i, 0);
		N << nT(0, column1) + nT(0, column2), nT(1, column1) + nT(1, column2), nT(2, column1) + nT(2, column2);
		Nt = N.transpose();
		NtN = Nt*N;
		nT.block<3, 1>(0, column3) << (N / (sqrt(NtN(0, 0)))); // there must be a better way to convert to float
	}

	R << 0, 0, -1, 1, 0, 0, 0, -1, 0;
	nT_ = nT.block < 3, 11 >(0, 0);
	nT__.block<3, 11>(0, 0) << nT_;
	nT__.block<3, 11>(0, 11) << R*nT_;
	nT__.block<3, 11>(0, 22) << R*R*nT_;
	nT__.block<3, 11>(0, 33) << -1 * nT_;
	nT__.block<3, 11>(0, 44) << -1 * R*nT_;
	nT__.block<3, 11>(0, 55) << -1 * R*R*nT_;

	int nF = nT__.cols();
	dT.Ones();
	return nT__;
}





//Generates vtkPolyData object (points and polys) that represenent a single CTI primitive, planeNormals is column wise (all_normals_x_coordinates, all_normals_y_coordinates, all_normals_z_coordinates)
vtkSmartPointer<vtkPolyData> GenerateCTIPrimitivePolydata_CW(float *planeNormals, float *planeDist, bool centered = false, int *mask = NULL)
{
	vtkSmartPointer<vtkPolyData> outPD;

	float *planeDistLocal = planeDist;
	//center the model
	if (!centered)
	{
		//make copy of original plane dist
		planeDistLocal = new float[66];
		memcpy(planeDistLocal, planeDist, 66 * sizeof(float));

		//Finding MIN and MAX for each normal dimension
		float maxN[3] = { -10, -10, -10 };
		int maxI[3] = { 0, 0, 0 };
		float minN[3] = { 10, 10, 10 };
		int minI[3] = { 0, 0, 0 };
		for (int i = 0; i < 66; i++)
		{
			if (planeNormals[i] > maxN[0])
			{
				maxN[0] = planeNormals[i];
				maxI[0] = i;
			}
			if (planeNormals[i] < minN[0])
			{
				minN[0] = planeNormals[i];
				minI[0] = i;
			}

			if (planeNormals[i + 66] > maxN[1])
			{
				maxN[1] = planeNormals[i + 66];
				maxI[1] = i;
			}
			if (planeNormals[i + 66] < minN[1])
			{
				minN[1] = planeNormals[i + 66];
				minI[1] = i;
			}

			if (planeNormals[i + 66 * 2] > maxN[2])
			{
				maxN[2] = planeNormals[i + 66 * 2];
				maxI[2] = i;
			}
			if (planeNormals[i + 66 * 2] < minN[2])
			{
				minN[2] = planeNormals[i + 66 * 2];
				minI[2] = i;
			}
		}
		//centering
		float newexampleTemp[66];
		float tempV[3];
		tempV[0] = 0.5 * (planeDistLocal[maxI[0]] - planeDistLocal[minI[0]]);
		tempV[1] = 0.5 * (planeDistLocal[maxI[1]] - planeDistLocal[minI[1]]);
		tempV[2] = 0.5 * (planeDistLocal[maxI[2]] - planeDistLocal[minI[2]]);
		for (int i = 0; i < 66; i++)
		{
			newexampleTemp[i] = planeNormals[i] * tempV[0] + planeNormals[i + 66] * tempV[1] + planeNormals[i + 66 * 2] * tempV[2];
			planeDistLocal[i] -= newexampleTemp[i];
		}
	}

	//Generiate primitive (convex hull)
	vtkSmartPointer<vtkHull> hullFilter = vtkSmartPointer<vtkHull>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> normalp = vtkSmartPointer<vtkFloatArray>::New();
	normalp->SetNumberOfComponents(3);
	for (int i = 0; i < 66; i++)
	{
		points->InsertPoint(i, planeNormals[i] * planeDistLocal[i], planeNormals[i + 66] * planeDistLocal[i], planeNormals[i + 66 * 2] * planeDistLocal[i]);
		normalp->InsertTuple3(i, planeNormals[i], planeNormals[i + 66], planeNormals[i + 66 * 2]);
	}
	vtkSmartPointer<vtkPlanes> planes = vtkSmartPointer<vtkPlanes>::New();
	planes->SetPoints(points);
	planes->SetNormals(normalp);
	hullFilter->SetPlanes(planes);
	vtkSmartPointer<vtkPolyData> hullPD = vtkSmartPointer<vtkPolyData>::New();
	hullFilter->GenerateHull(hullPD, -500, 500, -500, 500, -500, 500);
	vtkSmartPointer<vtkPolyData> interPD = hullPD;
	//If mask exists remove unwanted polygons
	if (mask)
	{
		double n[3];
		float cosfi;
		vtkSmartPointer<vtkCellArray> polys = hullPD->GetPolys();
		vtkSmartPointer<vtkCellArray> newpolys = vtkSmartPointer<vtkCellArray>::New();
		vtkIdType *polysPtsIds;
		vtkIdType npts;
		polys->InitTraversal();
		//run through all polygons and find planes with the same normal that shuld be in the output
		for (int i = 0; i < hullPD->GetNumberOfPolys(); i++)
		{
			polys->GetNextCell(npts, polysPtsIds);
			//calculate polygon normal
			vtkPolygon::ComputeNormal(hullPD->GetPoints(), npts, polysPtsIds, n);
			//find corresponding normal in normal list
			for (int k = 0; k < 66; k++)
			{
				cosfi = n[0] * planeNormals[k] + n[1] * planeNormals[k + 66] + n[2] * planeNormals[k + 66 * 2];
				if ((cosfi > 0.9999) && (mask[k] == 1))
				{
					newpolys->InsertNextCell(npts, polysPtsIds);
					break;
				}
			}

		}
		vtkSmartPointer<vtkPolyData> maskedPD = vtkSmartPointer<vtkPolyData>::New();
		maskedPD->SetPoints(hullPD->GetPoints());
		maskedPD->SetPolys(newpolys);

		interPD = maskedPD;
	}

	//clean polydata from unused poimts and degenerate polygons
	vtkSmartPointer<vtkCleanPolyData> cleanPD = vtkSmartPointer<vtkCleanPolyData>::New();
	cleanPD->SetInputData(interPD);
	cleanPD->Update();

	//make copy of the final polydata and send it back
	outPD = vtkSmartPointer<vtkPolyData>::New();
	outPD->DeepCopy(cleanPD->GetOutput());
	return outPD;
}

//Generates vtkPolyData object (points and polys) that represenent a single CTI primitive, planeNormals is row wise (normal_1_x_coordinate, normal_1_y_coordinate, normal_1_z_coordinate, normal_2_x_coordinate, ...)
vtkSmartPointer<vtkPolyData> GenerateCTIPrimitivePolydata_RW(float *planeNormals, float *planeDist, int nPlanes, bool centered = false, int *mask = NULL, float *t = NULL)
{
	vtkSmartPointer<vtkPolyData> outPD;

	float *planeDistLocal = planeDist;
	//center the model
	if (!centered)
	{
		//make copy of original plane dist
		planeDistLocal = new float[nPlanes];
		memcpy(planeDistLocal, planeDist, nPlanes * sizeof(float));

		//Finding MIN and MAX for each normal dimension
		float maxN[3] = { -10, -10, -10 };
		int maxI[3] = { 0, 0, 0 };
		float minN[3] = { 10, 10, 10 };
		int minI[3] = { 0, 0, 0 };
		for (int i = 0; i < nPlanes; i++)
		{
			if (planeNormals[i * 3] > maxN[0])
			{
				maxN[0] = planeNormals[i * 3];
				maxI[0] = i;
			}
			if (planeNormals[i * 3] < minN[0])
			{
				minN[0] = planeNormals[i * 3];
				minI[0] = i;
			}

			if (planeNormals[i * 3 + 1] > maxN[1])
			{
				maxN[1] = planeNormals[i * 3 + 1];
				maxI[1] = i;
			}
			if (planeNormals[i * 3 + 1] < minN[1])
			{
				minN[1] = planeNormals[i * 3 + 1];
				minI[1] = i;
			}

			if (planeNormals[i * 3 + 2] > maxN[2])
			{
				maxN[2] = planeNormals[i * 3 + 2];
				maxI[2] = i;
			}
			if (planeNormals[i * 3 + 2] < minN[2])
			{
				minN[2] = planeNormals[i * 3 + 2];
				minI[2] = i;
			}
		}
		//centering
		float newexampleTemp;
		float tempV[3];
		tempV[0] = 0.5 * (planeDistLocal[maxI[0]] - planeDistLocal[minI[0]]);
		tempV[1] = 0.5 * (planeDistLocal[maxI[1]] - planeDistLocal[minI[1]]);
		tempV[2] = 0.5 * (planeDistLocal[maxI[2]] - planeDistLocal[minI[2]]);
		for (int i = 0; i < nPlanes; i++)
		{
			newexampleTemp = planeNormals[i * 3] * tempV[0] + planeNormals[i * 3 + 1] * tempV[1] + planeNormals[i * 3 + 2] * tempV[2];
			planeDistLocal[i] -= newexampleTemp;
		}
		if (t)
			memcpy(t, tempV, 3 * sizeof(float));
	}

	//Generiate primitive (convex hull)
	vtkSmartPointer<vtkHull> hullFilter = vtkSmartPointer<vtkHull>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> normalp = vtkSmartPointer<vtkFloatArray>::New();
	normalp->SetNumberOfComponents(3);
	for (int i = 0; i < nPlanes; i++)
	{
		points->InsertPoint(i, planeNormals[i * 3] * planeDistLocal[i], planeNormals[i * 3 + 1] * planeDistLocal[i], planeNormals[i * 3 + 2] * planeDistLocal[i]);
		normalp->InsertTuple3(i, planeNormals[i * 3], planeNormals[i * 3 + 1], planeNormals[i * 3 + 2]);
	}
	vtkSmartPointer<vtkPlanes> planes = vtkSmartPointer<vtkPlanes>::New();
	planes->SetPoints(points);
	planes->SetNormals(normalp);
	hullFilter->SetPlanes(planes);
	vtkSmartPointer<vtkPolyData> hullPD = vtkSmartPointer<vtkPolyData>::New();
	hullFilter->GenerateHull(hullPD, -500, 500, -500, 500, -500, 500);
	vtkSmartPointer<vtkPolyData> interPD = hullPD;
	//If mask exists remove unwanted polygons
	if (mask)
	{
		double n[3];
		float cosfi;
		vtkSmartPointer<vtkCellArray> polys = hullPD->GetPolys();
		vtkSmartPointer<vtkCellArray> newpolys = vtkSmartPointer<vtkCellArray>::New();
		vtkIdType *polysPtsIds;
		vtkIdType npts;
		polys->InitTraversal();
		//run through all polygons and find planes with the same normal that shuld be in the output
		for (int i = 0; i < hullPD->GetNumberOfPolys(); i++)
		{
			polys->GetNextCell(npts, polysPtsIds);
			//calculate polygon normal
			vtkPolygon::ComputeNormal(hullPD->GetPoints(), npts, polysPtsIds, n);
			//find corresponding normal in normal list
			for (int k = 0; k < nPlanes; k++)
			{
				cosfi = n[0] * planeNormals[k * 3] + n[1] * planeNormals[k * 3 * 1] + n[2] * planeNormals[k * 3 + 2];
				if ((cosfi > 0.9999) && (mask[k] == 1))
				{
					newpolys->InsertNextCell(npts, polysPtsIds);
					break;
				}
			}

		}
		vtkSmartPointer<vtkPolyData> maskedPD = vtkSmartPointer<vtkPolyData>::New();
		maskedPD->SetPoints(hullPD->GetPoints());
		maskedPD->SetPolys(newpolys);

		//intermediate
		interPD = maskedPD;
	}

	//clean polydata from unused poimts and degenerate polygons
	vtkSmartPointer<vtkCleanPolyData> cleanPD = vtkSmartPointer<vtkCleanPolyData>::New();
	cleanPD->SetInputData(interPD);
	cleanPD->Update();

	//make copy of the final polydata and send it back
	outPD = vtkSmartPointer<vtkPolyData>::New();
	outPD->DeepCopy(cleanPD->GetOutput());

	return outPD;
}

void PSGM::VisualizeCTIMatchidx(int iSCTI, int iMCTI)
{
	Eigen::MatrixXf nT = ConvexTemplatenT();

	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;
	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;

	float *dS = new float[66];
	float *dM = new float[66];
	int *validS = new int[66];

	//Eigen::VectorXf dS(66);
	//Eigen::VectorXf dM(66);
	//Eigen::VectorXi validS(66);


	pSCTI = CTISet.pCTI.Element[iSCTI];
	pSIE = pSCTI->modelInstance.Element;
	pMCTI = MCTISet.pCTI.Element[iMCTI];
	pMIE = pMCTI->modelInstance.Element;

	for (int i = 0; i < 66; i++)
	{
		validS[i] = pSIE->valid; // visibility mask
		dS[i] = pSIE->d; // *1000; // Scene descriptor 
		dM[i] = pMIE->d / 1000; // Model descriptor
		pSIE++;
		pMIE++;
	}

	VisualizeCTIMatch(nT.data(), dM, dS, validS);
	//VisualizeCTIMatch(nT.data(), dM.data(), dS.data(), validS.data());

	delete[] dS;
	delete[] dM;
	delete[] validS;

}

void PSGM::VisualizeCTIMatch(float *nT, float *dM, float *dS, int *validS)
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//Generate model polydata
	vtkSmartPointer<vtkPolyData> modelPD = GenerateCTIPrimitivePolydata_RW(nT, dM, 66);
	/*if (tM) //if translation exists
	{
		float scale = 1;
		vtkSmartPointer<vtkTransform> modelT = vtkSmartPointer<vtkTransform>::New();
		modelT->Translate(tM[0], tM[1], tM[2]);
		vtkSmartPointer<vtkTransformPolyDataFilter> modelTFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		modelTFilter->SetTransform(modelT);
		modelTFilter->SetInputData(modelPD);
		modelTFilter->Update();
		vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		modelMapper->SetInputConnection(modelTFilter->GetOutputPort());
		vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
		modelActor->SetMapper(modelMapper);
		modelActor->GetProperty()->SetColor(0, 1, 0);
		renderer->AddActor(modelActor);
	}
	else*/
	{
		vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		modelMapper->SetInputData(modelPD);
		vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
		modelActor->SetMapper(modelMapper);
		modelActor->GetProperty()->SetColor(0, 1, 0);
		renderer->AddActor(modelActor);
	}

	/*
	vtkSmartPointer<vtkPolyDataMapper> modelMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper2->SetInputData(modelPD);
	vtkSmartPointer<vtkActor> modelActor2 = vtkSmartPointer<vtkActor>::New();
	modelActor2->SetMapper(modelMapper2);
	modelActor2->GetProperty()->SetColor(1, 0, 0);
	renderer->AddActor(modelActor2);
	*/
	//Generate scene polydata
	vtkSmartPointer<vtkPolyData> modelSPD = GenerateCTIPrimitivePolydata_RW(nT, dS, 66, false, validS);
	vtkSmartPointer<vtkPolyDataMapper> modelSMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelSMapper->SetInputData(modelSPD);
	vtkSmartPointer<vtkActor> modelSActor = vtkSmartPointer<vtkActor>::New();
	modelSActor->SetMapper(modelSMapper);
	modelSActor->GetProperty()->SetColor(0, 0, 1);
	renderer->AddActor(modelSActor);



	//Start VTK
	renderer->ResetCamera();
	renderer->TwoSidedLightingOff();
	window->Render();
	interactor->Start();
}
//
////END PETRA
//
//

#ifdef RVLVERSION_170601
void PSGM::Clusters()
{
	RVL_DELETE_ARRAY(clusterMap);

	clusterMap = new int[pSurfels->NodeArray.n];

	memset(clusterMap, 0xff, pSurfels->NodeArray.n * sizeof(int));

	RVL_DELETE_ARRAY(clusterMem);

	clusterMem = new RECOG::PSGM_::Cluster[pSurfels->NodeArray.n];

	clusters.n = 0;

	RVL_DELETE_ARRAY(clusterSurfelMem);

	clusterSurfelMem = new int[pSurfels->NodeArray.n];

	int *piSurfel = clusterSurfelMem;

	RVL_DELETE_ARRAY(clusterVertexMem);

	clusterVertexMem = new int[pSurfels->nVertexSurfelRelations];

	int *piVertex = clusterVertexMem;

	bool *bVertexVisited = new bool[pSurfels->vertexArray.n];
	bool *bVertexInCluster = new bool[pSurfels->vertexArray.n];

	bool *bSurfelVisited = new bool[pSurfels->NodeArray.n];

	QList<QLIST::Index> candidateList;
	QList<QLIST::Index> *pCandidateList = &candidateList;

	QLIST::Index *candidateMem = new QLIST::Index[pSurfels->NodeArray.n];

	Array<int> surfelBuff1, surfelBuff2;

	surfelBuff1.Element = new int[pSurfels->NodeArray.n];

	surfelBuff1.n = 0;

	int i;
	Surfel *pSurfel;

	for (i = 0; i < pSurfels->NodeArray.n; i++)
	{
		pSurfel = pSurfels->NodeArray.Element + i;

		if (!pSurfel->bEdge)
			surfelBuff1.Element[surfelBuff1.n++] = i;
	}

	surfelBuff2.Element = new int[surfelBuff1.n];

	Array<int> *pSurfelBuff = &surfelBuff1;
	Array<int> *pSurfelBuff_ = &surfelBuff2;

	int nValidClusters = 0;

	Array<int> *pTmp;

#ifdef RVLPSGM_NORMAL_HULL
	Array<RECOG::PSGM_::NormalHullElement> NHull;

	NHull.Element = new RECOG::PSGM_::NormalHullElement[pSurfels->NodeArray.n];
#else
	float meanN[3];
	float sumN[3];
	float wN;
#endif

	RECOG::PSGM_::Cluster *pCluster;
	int iCluster;
	int maxSurfelSize;
	int iLargestSurfel;
	int iFirstNewVertex;
	QLIST::Index *pCandidateIdx, *pBestCandidateIdx;
	QLIST::Index **ppCandidateIdx, **ppBestCandidateIdx;
	float dist, minDist;
	int nSurfelVertices;
	int nSurfelVerticesInCluster;
	int *piVertex_, *piVertex__;
	int iSurfel, iSurfel_;
	Surfel *pSurfel_;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *pVertexIdx;

	for (iCluster = 0; iCluster < pSurfels->NodeArray.n; iCluster++)
	{
		// pSurfel <- the largest surfel which is not assigned to a cluster.

		maxSurfelSize = minInitialSurfelSize - 1;

		iLargestSurfel = -1;

		pSurfelBuff_->n = 0;

		for (i = 0; i < pSurfelBuff->n; i++)
		{
			iSurfel = pSurfelBuff->Element[i];

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			if (!pSurfel->bEdge)
			{
				if (clusterMap[iSurfel] < 0)
				{
					pSurfelVertexList = pSurfels->surfelVertexList.Element + iSurfel;

					if (pSurfelVertexList->pFirst)
					{
						pSurfelBuff_->Element[pSurfelBuff_->n++] = iSurfel;

						if (pSurfel->size > maxSurfelSize)
						{
							maxSurfelSize = pSurfel->size;

							iLargestSurfel = iSurfel;
						}
					}
				}
			}
		}

		pTmp = pSurfelBuff;
		pSurfelBuff = pSurfelBuff_;
		pSurfelBuff_ = pTmp;

		if (iLargestSurfel < 0)
			break;

		//if (iLargestSurfel == 25)
		//	int debug = 0;

		//if (clusters.n == 15)
		//	int debug = 0;

		// Initialize a new cluster.

		pCluster = clusterMem + iCluster;

		pCluster->iSurfelArray.Element = piSurfel;
		pCluster->iVertexArray.Element = piVertex;

		pCluster->iSurfelArray.n = 0;
		pCluster->iVertexArray.n = 0;
		pCluster->size = 0;

		clusters.n++;

		clusterMap[iLargestSurfel] = iCluster;

		memset(bVertexVisited, 0, pSurfels->vertexArray.n * sizeof(bool));
		memset(bVertexInCluster, 0, pSurfels->vertexArray.n * sizeof(bool));
		memset(bSurfelVisited, 0, pSurfels->NodeArray.n * sizeof(bool));

		RVLQLIST_INIT(pCandidateList);

		QLIST::Index *pNewCandidate = candidateMem;

#ifdef RVLPSGM_NORMAL_HULL
		NHull.n = 0;
#else
		RVLNULL3VECTOR(sumN);
		wN = 0.0f;
#endif

		RVLQLIST_ADD_ENTRY(pCandidateList, pNewCandidate);

		pNewCandidate->Idx = iLargestSurfel;

		pNewCandidate++;

		bSurfelVisited[iLargestSurfel] = true;

		// Region growing.

		while (pCandidateList->pFirst)
		{
			// iSurfel <- the best candidate for expanding cluster.

			minDist = PI;

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = *ppCandidateIdx;

			while (pCandidateIdx)
			{
				iSurfel_ = pCandidateIdx->Idx;

				pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

#ifdef RVLPSGM_NORMAL_HULL
				dist = pSurfels->DistanceFromNormalHull(NHull, pSurfel_->N);
#else
				float e = RVLDOTPRODUCT3(meanN, pSurfel_->N);
				dist = (wN < 1e-10 ? 0.0f : acos(e));
#endif

				if (dist < minDist)
				{
					minDist = dist;

					iSurfel = iSurfel_;

					pBestCandidateIdx = pCandidateIdx;

					ppBestCandidateIdx = ppCandidateIdx;
				}

				ppCandidateIdx = &(pCandidateIdx->pNext);

				pCandidateIdx = *ppCandidateIdx;
			}

			// Remove iSurfel from candidateList.

			RVLQLIST_REMOVE_ENTRY(pCandidateList, pBestCandidateIdx, ppBestCandidateIdx);

			//if (iSurfel == 8)
			//	int debug = 0;

			// Add vertices of iSurfel, which are inside convex (or outside concave) surface into cluster.

			iFirstNewVertex = pCluster->iVertexArray.n;

			piVertex_ = piVertex;

			pSurfelVertexList = pSurfels->surfelVertexList.Element + iSurfel;

			nSurfelVertices = nSurfelVerticesInCluster = 0;

			pVertexIdx = pSurfelVertexList->pFirst;

			while (pVertexIdx)
			{
				if (bVertexVisited[pVertexIdx->Idx])
				{
					if (bVertexInCluster[pVertexIdx->Idx])
						nSurfelVerticesInCluster++;
				}
				else
				{
					if (Inside(pVertexIdx->Idx, pCluster, iSurfel))
					{
						*(piVertex++) = pVertexIdx->Idx;

						nSurfelVerticesInCluster++;
					}

				}

				nSurfelVertices++;

				pVertexIdx = pVertexIdx->pNext;
			}

			if (nSurfelVertices == 0)
				continue;

			if (100 * nSurfelVerticesInCluster / nSurfelVertices < minVertexPerc)
			{
				piVertex = piVertex_;

				continue;
			}

			pCluster->iVertexArray.n = piVertex - pCluster->iVertexArray.Element;

			pVertexIdx = pSurfelVertexList->pFirst;

			while (pVertexIdx)
			{
				bVertexVisited[pVertexIdx->Idx] = true;

				pVertexIdx = pVertexIdx->pNext;
			}

			for (piVertex__ = piVertex_; piVertex__ < piVertex; piVertex__++)
				bVertexInCluster[*piVertex__] = true;

			// Add iSurfel to cluster.

			//if (iLargestSurfel == 25 && iSurfel == 192)
			//	int debug = 0;

			clusterMap[iSurfel] = iCluster;

			*(piSurfel++) = iSurfel;

			pCluster->iSurfelArray.n++;

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			pCluster->size += pSurfel->size;

#ifdef RVLPSGM_NORMAL_HULL
			// Update normal hull.

			UpdateNormalHull(NHull, pSurfel->N);
#else
			// Update mean normal.

			UpdateMeanNormal(sumN, wN, pSurfel->N, (float)(pSurfel->size), meanN);
#endif

			// Remove candidates which are not consistent with new vertices added to the cluster.

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = candidateList.pFirst;

			while (pCandidateIdx)
			{
				pSurfel_ = pSurfels->NodeArray.Element + pCandidateIdx->Idx;

				//if (pCandidateIdx->Idx == 8)
				//	int debug = 0;

				if (BelowPlane(pCluster, pSurfel_, iFirstNewVertex))
					ppCandidateIdx = &(pCandidateIdx->pNext);
				else
					RVLQLIST_REMOVE_ENTRY(pCandidateList, pCandidateIdx, ppCandidateIdx)

					pCandidateIdx = pCandidateIdx->pNext;
			}

			// Add new candidates in candidateList.

			SURFEL::EdgePtr *pSurfelEdgePtr = pSurfel->EdgeList.pFirst;

			while (pSurfelEdgePtr)
			{
				iSurfel_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pSurfelEdgePtr);

				//if (iSurfel_ == 8)
				//	int debug = 0;

				if (clusterMap[iSurfel_] < 0)
				{
					if (!bSurfelVisited[iSurfel_])
					{
						//if (iSurfel_ == 8)
						//	int debug = 0;

						bSurfelVisited[iSurfel_] = true;

						pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

						if (pSurfel_->size > 1)
						{
							if (BelowPlane(pCluster, pSurfel_))
							{
								RVLQLIST_ADD_ENTRY(pCandidateList, pNewCandidate);

								pNewCandidate->Idx = iSurfel_;

								pNewCandidate++;
							}
						}
					}
				}

				pSurfelEdgePtr = pSurfelEdgePtr->pNext;
			}
		}	// region growing loop

		pCluster->orig = pCluster->size;

		if(pCluster->bValid = (pCluster->size >= minClusterSize))
			nValidClusters++;
	}	// for each cluster

	delete[] candidateMem;
	delete[] bVertexVisited;
	delete[] bVertexInCluster;
	delete[] bSurfelVisited;
#ifdef RVLPSGM_NORMAL_HULL
	delete[] NHull.Element;
#endif

	// Sort clusters.

	Array<SortIndex<int>> sortedClusterArray;

	sortedClusterArray.Element = new SortIndex<int>[nValidClusters];

	SortIndex<int> *pSortIndex = sortedClusterArray.Element;

	for (i = 0; i < clusters.n; i++)
	{
		pCluster = clusterMem + i;

		if (pCluster->bValid)
		{
			pSortIndex->cost = pCluster->orig;
			pSortIndex->idx = i;
			pSortIndex++;
		}
	}

	sortedClusterArray.n = nValidClusters;

	BubbleSort<SortIndex<int>>(sortedClusterArray, true);

	// Detect the ground plane and filter all clusters lying on the ground plane.

	if (bDetectGroundPlane)
	{
	Array<int> PtArray;

	PtArray.Element = new int[pMesh->NodeArray.n];

	bGnd = false;

	//int *piPt;
	int iiSurfel;
	//QLIST::Index2 *pPtIdx;
	//MESH::Distribution PtDistribution;
	//float *var;
	//int idx[3];
	//int iTmp;
	float eGnd;
	float *NGnd_;

	for (i = 0; i < nValidClusters; i++)
	{
		iCluster = sortedClusterArray.Element[i].idx;

		pCluster = clusterMem + iCluster;

		if (bGnd)
		{
			//ComputeClusterNormalDistribution(pCluster);

			//if (pCluster->normalDistributionStd1 < minClusterNormalDistributionStd && pCluster->normalDistributionStd2 < minClusterNormalDistributionStd)
			{
				//if (RVLDOTPRODUCT3(NGnd, pCluster->N) >= 0.95)
				{
					for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
					{
						iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

						pSurfel = pSurfels->NodeArray.Element + iSurfel;

						eGnd = RVLDOTPRODUCT3(NGnd, pSurfel->P) - dGnd;

						if (eGnd > groundPlaneTolerance)
							break;
					}

					if (iiSurfel >= pCluster->iSurfelArray.n)
						pCluster->bValid = false;
				}
			}
		}
		else
		{
			if (IsFlat(pCluster->iSurfelArray, NGnd, dGnd, PtArray))
			{
				pCluster->bValid = false;

				bGnd = true;
			}
		}
	}

	delete[] PtArray.Element;
	}

	//// Filter and sort clusters.

	//for (i = 0; i < clusters.n; i++)
	//{
	//	pCluster = clusterMem + i;

	//	if (pCluster->bValid)
	//		ComputeClusterBoundaryDiscontinuityPerc(i);
	//}

	//for (i = 0; i < clusters.n; i++)
	//{
	//	pCluster = clusterMem + i;

	//	if (pCluster->bValid)
	//	{
	//		if (pCluster->size <= maxClusterSize)
	//		{
	//			ComputeClusterNormalDistribution(pCluster);

	//			if (pCluster->size < minSignificantClusterSize)
	//			{
	//				if (pCluster->boundaryDiscontinuityPerc < minClusterBoundaryDiscontinuityPerc)
	//					if (pCluster->normalDistributionStd1 < minClusterNormalDistributionStd || pCluster->normalDistributionStd2 < minClusterNormalDistributionStd)
	//						pCluster->bValid = false;
	//			}
	//		}
	//		else
	//			pCluster->bValid = false;
	//	}
	//}

	RVL_DELETE_ARRAY(clusters.Element);

	clusters.Element = new RECOG::PSGM_::Cluster *[nValidClusters];

	int *clusterIndexMap = new int[clusters.n];

	memset(clusterIndexMap, 0xff, clusters.n * sizeof(int));

	int iCluster_ = 0;

	for (i = 0; i < sortedClusterArray.n; i++)
	{
		iCluster = sortedClusterArray.Element[i].idx;

		pCluster = clusterMem + iCluster;

		if (pCluster->bValid)
		{
			clusterIndexMap[iCluster] = iCluster_;

			clusters.Element[iCluster_] = pCluster;

			iCluster_++;
		}
	}

	clusters.n = iCluster_;

	//int maxClusterSize_ = 0;
	//int size;

	//for (i = 0; i < clusters.n; i++)
	//{
	//	size = clusterMem[i].size;

	//	if (size > maxClusterSize_)
	//		maxClusterSize_ = size;
	//}

	//int maxnBins = 100000;

	//int k = (maxClusterSize_ < maxnBins ? 1 : maxClusterSize_ / maxnBins + 1);

	//int *key = new int[clusters.n];

	//for (i = 0; i < clusters.n; i++)
	//	key[i] = clusterMem[i].size / k;

	//RVL::QuickSort(key, surfelBuff1.Element, clusters.n);

	//RVL_DELETE_ARRAY(clusters.Element);

	//clusters.Element = new RECOG::PSGM_::Cluster *[clusters.n];

	//for (i = 0; i < clusters.n; i++)
	//{
	//	iCluster = surfelBuff1.Element[clusters.n - i - 1];
	//	clusters.Element[i] = clusterMem + iCluster;
	//	surfelBuff2.Element[iCluster] = i;
	//}

	//delete[] key;

	// Update cluster map.	

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		iCluster = clusterMap[iSurfel];

		if (iCluster >= 0)
			clusterMap[iSurfel] = clusterIndexMap[iCluster];
	}

	delete[] clusterIndexMap;
	delete[] sortedClusterArray.Element;
	delete[] surfelBuff1.Element;
	delete[] surfelBuff2.Element;	
}
#endif

void PSGM::WholeMeshCluster()
{
	RVL_DELETE_ARRAY(clusterMap);

	clusterMap = new int[pSurfels->NodeArray.n];

	memset(clusterMap, 0, pSurfels->NodeArray.n * sizeof(int));

	RVL_DELETE_ARRAY(clusterMem);

	clusterMem = new RECOG::PSGM_::Cluster;

	RECOG::PSGM_::Cluster *pCluster = clusterMem;

	clusters.n = 0;

	RVL_DELETE_ARRAY(clusterSurfelMem);

	clusterSurfelMem = new int[pSurfels->NodeArray.n];

	pCluster->iSurfelArray.Element = clusterSurfelMem;

	RVL_DELETE_ARRAY(clusterVertexMem);

	clusterVertexMem = new int[pSurfels->nVertexSurfelRelations];

	pCluster->iVertexArray.Element = clusterVertexMem;

	int iSurfel;
	Surfel* pSurfel;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pCluster->iSurfelArray.Element[iSurfel] = iSurfel;

		pCluster->size += pSurfel->size;
	}

	pCluster->iSurfelArray.n = pSurfels->NodeArray.n;

	int iVertex;

	for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
		pCluster->iVertexArray.Element[iVertex] = iVertex;

	pCluster->iVertexArray.n = pSurfels->vertexArray.n;

	pCluster->bValid = true;

	RVL_DELETE_ARRAY(clusters.Element);

	clusters.Element = new RECOG::PSGM_::Cluster *;

	clusters.Element[0] = pCluster;

	clusters.n = 1;
}

void PSGM::CreateTemplate66()
{
	float h = 0.25f * PI;
	float q = 0.5f * h;
	float sh = sin(h);
	float ch = cos(h);
	float sq = sin(q);
	float cq = cos(q);

	float *NT = new float[3 * 13];

	float *N;

	N = NT;
	RVLSET3VECTOR(N, 0.0f, 0.0f, 1.0f);
	N = NT + 3;
	RVLSET3VECTOR(N, 0.0f, -ch, ch);
	N = NT + 2 * 3;
	RVLSET3VECTOR(N, ch, 0.0f, ch);
	N = NT + 11 * 3;
	RVLSET3VECTOR(N, 0.0f, ch, ch);
	N = NT + 12 * 3;
	RVLSET3VECTOR(N, -ch, 0.0f, ch);

	int templ[] = {
		3, 0, 1,
		4, 0, 2,
		5, 1, 2,
		6, 0, 11,
		7, 0, 12,
		8, 2, 11,
		9, 1, 12,
		10, 11, 12 };

	int i;
	float *N_, *N__;
	float fTmp;

	for (i = 0; i < 8; i++)
	{
		N = NT + 3 * templ[3 * i];
		N_ = NT + 3 * templ[3 * i + 1];
		N__ = NT + 3 * templ[3 * i + 2];
		RVLSUM3VECTORS(N_, N__, N);
		RVLNORM3(N, fTmp);
	}

	float R[] = {
		0.0f, 0.0f, -1.0f,
		1.0f, 0.0f, 0.0f,
		0.0f, -1.0f, 0.0f };

	float R_[9];

	RVLMXMUL3X3(R, R, R_);

	int j;
	int i_;
	RECOG::PSGM_::Plane *pPlane;

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 11; j++)
		{
			pPlane = convexTemplate66.Element + 11 * i + j;

			N = pPlane->N;

			N_ = NT + 3 * j;

			i_ = i % 3;

			if (i_ == 0)
			{
				RVLCOPY3VECTOR(N_, N);
			}
			else if (i_ == 1)
			{
				RVLMULMX3X3VECT(R, N_, N)
			}				
			else
			{
				RVLMULMX3X3VECT(R_, N_, N)
			}
				
			if (i >= 3)
			{ 
				RVLNEGVECT3(N, N);
			}
				
			pPlane->d = 1.0f;
		}
	}

	//// Only for debugging purpose!

	//FILE *fp = fopen("convex_template.txt", "w");

	//for (i = 0; i < convexTemplate.n; i++)
	//	fprintf(fp, "%f\t%f\t%f\n", convexTemplate.Element[i].N[0], convexTemplate.Element[i].N[1], convexTemplate.Element[i].N[2]);

	//fclose(fp);

	////

	delete[] NT;
}

void PSGM::CreateTemplateBox()
{
	convexTemplateBox.n = 6;
	convexTemplateBox.Element = new RECOG::PSGM_::Plane[convexTemplateBox.n];

	float *N;

	N = convexTemplateBox.Element[0].N;
	N[0] = 0.0f; N[1] = 0.0f; N[2] = 1.0f;
	convexTemplateBox.Element[0].d = 1.0;

	N = convexTemplateBox.Element[1].N;
	N[0] = -1.0f; N[1] = 0.0f; N[2] = 0.0f;
	convexTemplateBox.Element[1].d = 1.0;

	N = convexTemplateBox.Element[2].N;
	N[0] = 0.0f; N[1] = -1.0f; N[2] = 0.0f;
	convexTemplateBox.Element[2].d = 1.0;

	N = convexTemplateBox.Element[3].N;
	N[0] = 0.0f; N[1] = 0.0f; N[2] = -1.0f;
	convexTemplateBox.Element[3].d = 1.0;

	N = convexTemplateBox.Element[4].N;
	N[0] = 1.0f; N[1] = 0.0f; N[2] = 0.0f;
	convexTemplateBox.Element[4].d = 1.0;

	N = convexTemplateBox.Element[5].N;
	N[0] = 0.0f; N[1] = 1.0f; N[2] = 0.0f;
	convexTemplateBox.Element[5].d = 1.0;
}

void PSGM::TemplateMatrix(Array2D<float> &A)
{
	A.Element = new float[3 * convexTemplate.n];
	A.w = 3;
	A.h = convexTemplate.n;

	int i;
	float *a;
	float *N;

	for (i = 0; i < convexTemplate.n; i++)
	{
		a = A.Element + 3 * i;

		N = convexTemplate.Element[i].N;

		RVLCOPY3VECTOR(N, a);
	}
}

void PSGM::FitModel(
	Array<int> iVertexArray,
	RECOG::PSGM_::ModelInstance *pModelInstance,
	bool bMemAllocated,
	float *PGnd)
{
	if (!bMemAllocated)
		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RECOG::PSGM_::ModelInstanceElement, convexTemplate.n, pModelInstance->modelInstance.Element);

	pModelInstance->modelInstance.n = convexTemplate.n;

	float *R = pModelInstance->R;
	float *t = pModelInstance->t;

	bool bGnd_ = (bGnd && problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION && PGnd != NULL);

	int iModelInstanceElement;
	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;
	float d;
	SURFEL::Vertex *pVertex;
	int i;
	float *N, *P;
	float N_[3];
	//float dist;
	//float maxdDefinedNormal;
	int iVertex;
	float *PGnd_;
	float eGnd;

	for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
	{
		//if (iModelInstanceElement == 33)
		//	int debug = 0;

		pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

		pModelInstanceElement->valid = false;

		N = convexTemplate.Element[iModelInstanceElement].N;

		RVLMULMX3X3VECT(R, N, N_);		

		iVertex = iVertexArray.Element[0];

		pVertex = pSurfels->vertexArray.Element[iVertex];

		pModelInstanceElement->d = RVLDOTPRODUCT3(N_, pVertex->P);
		pModelInstanceElement->iVertex = 0;

		for (i = 0; i < iVertexArray.n; i++)
		{
			iVertex = iVertexArray.Element[i];

			pVertex = pSurfels->vertexArray.Element[iVertex];

			d = RVLDOTPRODUCT3(N_, pVertex->P);

			if (d > pModelInstanceElement->d)
			{
				pModelInstanceElement->d = d;
				pModelInstanceElement->iVertex = iVertex;
			}
		}

		//Vidovic
		if (bNormalValidityTest)
		{
			//if (pVertex->normalHull.n >= 3)
			//{
			//	dist = pSurfels->DistanceFromNormalHull(pVertex->normalHull, N_);

			//	if (dist <= 0.0f)
			//	{
			//		if (pModelInstanceElement->valid)
			//		{
			//			if (d > maxdDefinedNormal)
			//				maxdDefinedNormal = d;
			//		}
			//		else
			//		{
			//			maxdDefinedNormal = d;
			//			pModelInstanceElement->valid = true;
			//		}
			//	}
			//}
			pModelInstanceElement->valid = true;

			pVertex = pSurfels->vertexArray.Element[pModelInstanceElement->iVertex];

			P = pVertex->P;

			if (bGnd_)
			{
				for (i = 0; i < iVertexArray.n; i++)
				{
					PGnd_ = PGnd + 3 * i;

					eGnd = RVLDOTPRODUCT3(N_, PGnd_) - pModelInstanceElement->d;

					if (eGnd > gndCTIThr)
						break;
				}

				if (i < iVertexArray.n)
					pModelInstanceElement->valid = false;
			}
			else if (RVLDOTPRODUCT3(N_, P) >= 0.0f)
				pModelInstanceElement->valid = false;
		}
		else
			pModelInstanceElement->valid = true;
		//END Vidovic

		pModelInstanceElement->d -= RVLDOTPRODUCT3(N_, t);

		//Vidovic
		//if (bNormalValidityTest)
		//	pModelInstanceElement->e = (pModelInstanceElement->valid ? pModelInstanceElement->d - maxdDefinedNormal : 0.0f);
		//else
		pModelInstanceElement->e = 0.0f;
		//END Vidovic
	}	// for every model instance descriptor element

	//calculate segment centroid - Vidovic

	if (bBoundingPlanes)
	{
		//int minID, maxID;

		//for (i = 0; i < 3; i++)
		//{
		//	minID = centroidID.Element[i * 2].Idx;
		//	maxID = centroidID.Element[i * 2 + 1].Idx;

		//	pModelInstance->tc[i] = (pModelInstance->modelInstance.Element[maxID].d - pModelInstance->modelInstance.Element[minID].d) / 2; // PROVJERITI
		//}
		}

	//END calculate segment centroid - Vidovic
}

bool PSGM::ReferenceFrames(int iCluster)
{
	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	return ReferenceFrames(pCluster, iCluster);
}

bool PSGM::ReferenceFrames(
	RECOG::PSGM_::Cluster *pCluster,
	int iCluster)
{
	return ReferenceFrames(pCluster->iSurfelArray, clusterMap, iCluster);
}

bool PSGM::ReferenceFrames(
	Array<int> iSurfelArray,
	int *clusterMap,
	int iCluster)
{
	// Identify the largest surfel.

	int maxSize = 0;

	int i;
	Surfel *pSurfel;

	for (i = 0; i < iSurfelArray.n; i++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfelArray.Element[i];

		if (pSurfel->size > maxSize)
			maxSize = pSurfel->size;
	}

	if (maxSize == 0)
		return false;

	//if (iCluster == 3)
	//	int debug = 0;

	int sizeThr = (int)((float)maxSize * kReferenceSurfelSize);

	// Sort surfels in the cluster.

	Array<SortIndex<int>> iSortedSurfelArray;
	
	iSortedSurfelArray.Element = new SortIndex<int>[iSurfelArray.n];
	iSortedSurfelArray.n = 0;

	int iSurfel;

	for (i = 0; i < iSurfelArray.n; i++)
	{
		iSurfel = iSurfelArray.Element[i];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->size >= sizeThr)
		{
			iSortedSurfelArray.Element[iSortedSurfelArray.n].idx = iSurfel;
			iSortedSurfelArray.Element[iSortedSurfelArray.n].cost = pSurfel->size;
			iSortedSurfelArray.n++;
		}
	}
		
	BubbleSort<SortIndex<int>>(iSortedSurfelArray, true);

	/// Determine reference frames of model instances. 

	//QList<RECOG::PSGM_::ModelInstance> *pModelInstanceList = &(pCluster->modelInstanceList); //Vidovic

	//RVLQLIST_INIT(pModelInstanceList); //Vidovic

	float cs = COS45;

	Array<RECOG::PSGM_::Tangent> tangentArray;

	tangentArray.Element = new RECOG::PSGM_::Tangent[pSurfels->NodeArray.n];

	//Array<RECOG::PSGM_::NormalHullElement> normalHull;

	//normalHull.Element = new RECOG::PSGM_::NormalHullElement[pCluster->iSurfelArray.n];

	RECOG::PSGM_::TangentRegionGrowingData tangentRGData;

	tangentRGData.bParent = new bool[pSurfels->NodeArray.n];
	memset(tangentRGData.bParent, 0, pSurfels->NodeArray.n * sizeof(bool));
	tangentRGData.bBase = new bool[pSurfels->NodeArray.n];
	memset(tangentRGData.bBase, 0, pSurfels->NodeArray.n * sizeof(bool));
	tangentRGData.pRecognition = this;
	tangentRGData.cs = cs;
	tangentRGData.iCluster = iCluster;
	tangentRGData.clusterMap = clusterMap;
	tangentRGData.pTangentArray = &tangentArray;
	//tangentRGData.pNormalHull = &normalHull;
	float baseSeparationAngleRad = baseSeparationAngle * DEG2RAD;
	tangentRGData.baseSeparationAngle = baseSeparationAngleRad;

	int *iSurfelBuff = new int[iSurfelArray.n];

	float kReferenceTangentSize2 = kReferenceTangentSize *  kReferenceTangentSize;

	float csSeparationAngle = cos(baseSeparationAngleRad);

	Array<SortIndex<float>> iSortedTangentArray;
	
	iSortedTangentArray.Element = new SortIndex<float>[pSurfels->NodeArray.n];

	Array<QList<QLIST::Index>> iTangentAngleArray;

	iTangentAngleArray.n = (int)round(360.0f / baseSeparationAngle);
	iTangentAngleArray.Element = new QList<QLIST::Index>[iTangentAngleArray.n];
	QLIST::Index *iTangentAngleMem = new QLIST::Index[pSurfels->NodeArray.n];

	int *piSurfelFetch, *piSurfelPut, *piSurfel, *piSurfelBuffEnd;
	RECOG::PSGM_::ModelInstance *pModelInstance;
	int iTangent;
	float maxTangentLen;
	RECOG::PSGM_::Tangent *pTangent, *pTangent_;
	float *R, *Z, *X, *t, *X_;
	//float *P1, *P2;
	float Y[3];
	//float P[3];
	Eigen::Matrix3f M;
	Eigen::Vector3f B, t_;
	float p, q;
	//float d;
	float tangentLenThr;
	int iLargestTangent;
	float *X0;
	float Y0[3];
	int iAngle;
	QList<QLIST::Index> *pAngleBinList;	
	QLIST::Index *pTangentAngleEntry;
	int j;

	for (i = 0; i < iSortedSurfelArray.n; i++)
	{
		iSurfel = iSortedSurfelArray.Element[i].idx;

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (!tangentRGData.bBase[iSurfel])
		{
			Z = pSurfel->N;

			piSurfelPut = piSurfelFetch = iSurfelBuff;

			*(piSurfelPut++) = iSurfel;

			tangentRGData.bBase[iSurfel] = true;

			tangentRGData.bParent[iSurfel] = true;

			RVLCOPY3VECTOR(pSurfel->N, tangentRGData.planeA.N);
			tangentRGData.planeA.d = pSurfel->d;
			tangentArray.n = 0;
			//normalHull.n = 0;

			piSurfelBuffEnd = RegionGrowing<SurfelGraph, Surfel, SURFEL::Edge, SURFEL::EdgePtr, RECOG::PSGM_::TangentRegionGrowingData, RECOG::PSGM_::ValidTangent>
				(pSurfels, &tangentRGData, piSurfelFetch, piSurfelPut);

			for (piSurfel = iSurfelBuff; piSurfel < piSurfelBuffEnd; piSurfel++)
				tangentRGData.bParent[*piSurfel] = false;

			//if (piSurfelBuffEnd - iSurfelBuff > pCluster->iSurfelArray.n)
			//	int debug = 0;

			maxTangentLen = 0;
			
			for (iTangent = 0; iTangent < tangentArray.n; iTangent++)
			{
				pTangent = tangentArray.Element + iTangent;

				if (pTangent->len > maxTangentLen)
				{
					maxTangentLen = pTangent->len;

					iLargestTangent = iTangent;
				}					
			}

			if (maxTangentLen > 0.0f)
			{
				X0 = tangentArray.Element[iLargestTangent].V;

				RVLCROSSPRODUCT3(Z, X0, Y0);

				tangentLenThr = kReferenceTangentSize2 * maxTangentLen;

				for (iAngle = 0; iAngle < iTangentAngleArray.n; iAngle++)
				{
					pAngleBinList = iTangentAngleArray.Element + iAngle;

					RVLQLIST_INIT(pAngleBinList);
				}

				pTangentAngleEntry = iTangentAngleMem;

				iSortedTangentArray.n = 0;

				for (iTangent = 0; iTangent < tangentArray.n; iTangent++)
				{
					pTangent = tangentArray.Element + iTangent;

					if (pTangent->len >= tangentLenThr)
					{
						iSortedTangentArray.Element[iSortedTangentArray.n].idx = iTangent;
						iSortedTangentArray.Element[iSortedTangentArray.n].cost = pTangent->len;
						iSortedTangentArray.n++;

						X = pTangent->V;

						p = RVLDOTPRODUCT3(X0, X);
						q = RVLDOTPRODUCT3(Y0, X);

						iAngle = (int)round((atan2(q, p) + PI) / baseSeparationAngleRad) % iTangentAngleArray.n;

						pAngleBinList = iTangentAngleArray.Element + iAngle;

						RVLQLIST_ADD_ENTRY(pAngleBinList, pTangentAngleEntry);

						pTangentAngleEntry->Idx = iTangent;

						pTangentAngleEntry++;
					}
				}

				BubbleSort<SortIndex<float>>(iSortedTangentArray, true);

				for (iTangent = 0; iTangent < iSortedTangentArray.n; iTangent++)
				{
					pTangent = tangentArray.Element + iSortedTangentArray.Element[iTangent].idx;

					if (!pTangent->bMerged)
					{
						RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::ModelInstance, pModelInstance);

						//RVLQLIST_ADD_ENTRY(pModelInstanceList, pModelInstance); Vidovic

						CTISet.AddCTI(pModelInstance); //Vidovic

						pModelInstance->iCluster = iCluster; //Vidovic - ADDED iCluster data to scene MI

						pModelInstance->iModel = -1; //Vidovic - ADDED iModel data to scene MI

						R = pModelInstance->R;

						RVLCOPYTOCOL3(Z, 2, R);

						X = pTangent->V;

						RVLCOPYTOCOL3(X, 0, R);

						RVLCROSSPRODUCT3(Z, X, Y);

						RVLCOPYTOCOL3(Y, 1, R);

						// Computing origin of the reference frame.

						//P1 = pSurfels->vertexArray.Element[pTangent->iVertex[0]]->P;
						//P2 = pSurfels->vertexArray.Element[pTangent->iVertex[1]]->P;

						//RVLSUM3VECTORS(P1, P2, P);

						//RVLSCALE3VECTOR(P, 0.5f, P);

						//d = RVLDOTPRODUCT3(X, P);

						//M << pSurfel->N[0], pSurfel->N[1], pSurfel->N[2], pTangent->N[0], pTangent->N[1], pTangent->N[2], X[0], X[1], X[2];

						//B << pSurfel->d, pTangent->d, d;

						//t_ = M.colPivHouseholderQr().solve(B);

						t = pModelInstance->t;

						//RVLCOPY3VECTOR(t_, t);

						RVLNULL3VECTOR(t);

						p = RVLDOTPRODUCT3(X0, X);
						q = RVLDOTPRODUCT3(Y0, X);

						iAngle = (int)round((atan2(q, p) + PI) / baseSeparationAngleRad) % iTangentAngleArray.n;

						for (j = 0; j < 2; j++)
						{
							pAngleBinList = iTangentAngleArray.Element + iAngle;

							pTangentAngleEntry = pAngleBinList->pFirst;

							while (pTangentAngleEntry)
							{
								pTangent_ = tangentArray.Element + pTangentAngleEntry->Idx;

								X_ = pTangent_->V;

								if (RVLDOTPRODUCT3(X, X_) > csSeparationAngle)
									pTangent_->bMerged = true;

								pTangentAngleEntry = pTangentAngleEntry->pNext;
							}

							iAngle = (iAngle + iTangentAngleArray.n - 1) % iTangentAngleArray.n;
						}
					}	// if (pTangent->len >= kReferenceTangentSize2 * maxTangentLen)
				}	// for every tangent
			}	// if (maxTangentLen > 0.0f)
		}	// if (pSurfel->size >= kReferenceSurfelSize * maxSize)
	}	// for every surfel in the cluster

	delete[] tangentArray.Element;
	delete[] tangentRGData.bParent;
	delete[] tangentRGData.bBase;
	delete[] iSurfelBuff;
	//delete[] normalHull.Element;
	delete[] iSortedSurfelArray.Element;
	delete[] iSortedTangentArray.Element;
	delete[] iTangentAngleMem;

	return true;
}

RECOG::PSGM_::ModelInstance * PSGM::AddReferenceFrame(
	//int iCluster, //Vidovic
	float *RIn,
	float *tIn)
{
	//RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster]; //Vidovic

	//QList<RECOG::PSGM_::ModelInstance> *pModelInstanceList = &(pCluster->modelInstanceList); //Vidovic

	RECOG::PSGM_::ModelInstance *pModelInstance;

	RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::ModelInstance, pModelInstance);
	//RVLQLIST_ADD_ENTRY(pModelInstanceList, pModelInstance); //Vidovic

	CTISet.AddCTI(pModelInstance); //Vidovic

	float *R = pModelInstance->R;
	float *t = pModelInstance->t;

	if (RIn)
	{
		RVLCOPYMX3X3(RIn, R)
	}		
	else
	{
		RVLUNITMX3(R)
	}

	if (tIn)
	{
		RVLCOPY3VECTOR(tIn, t)
	}
	else
	{
		RVLNULL3VECTOR(t);
	}

	return pModelInstance;
}

int RVL::RECOG::PSGM_::ValidTangent(
	int iSurfel, 
	int iSurfel_, 
	SURFEL::Edge *pEdge, 
	SurfelGraph *pSurfels, 
	RECOG::PSGM_::TangentRegionGrowingData *pData)
{	
	PSGM *pRecognition = pData->pRecognition;

	//if (pRecognition->clusterMap[iSurfel] != pData->iCluster)
	//	return -1;

	if (pData->bParent[iSurfel])
		return -1;

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	float *N0 = pData->planeA.N;
	float d0 = pData->planeA.d;
	float *N = pSurfel->N;
	float *N_ = pSurfel_->N;
	float cs = RVLDOTPRODUCT3(N0, N);

	if (cs <= pData->cs)
	{
		RECOG::PSGM_::Tangent *pTangent = pData->pTangentArray->Element + pData->pTangentArray->n;

		pData->pTangentArray->n++;

		pTangent->bMerged = false;

		float *NT = pTangent->N;

		// N0'*NT = cs,     NT = (s*N + (1-s)*N_) / || s*N + (1-s)*N_ ||
		// N0'*(s*N + (1-s)*N_) = cs*sqrt(s*N + (1-s)*N_)'*(s*N + (1-s)*N_)
		// s*N0'*N + N0'*N_ - s*N0'*N_ = cs * sqrt(s^2*N'*N + 2*s*(1-s)*N'*N_ + (1-s)^2*N_'*N_)
		// N0'(N-N_)*s + N0'*N_ = cs * sqrt(s^2 + 2*s*(1-s)*N'*N_ + (1-s)^2)
		// N0'(N-N_)*s + N0'*N_ = cs * sqrt(2*(1 - N'*N_)*s^2 - 2*(1 - N'*N_)*s + 1)
		// a*s + b = cs * sqrt(c*s^2 - c*s + 1),     a = N0'(N-N_), b = N0'*N_, c = 2*(1 - N'*N_)
		// a^2*s^2 + 2*a*b*s + b^2 = cs^2 * (c*s^2 - c*s + 1)
		// p*s^2 + q*s + r = 0,     p = a^2-cs^2*c, q = 2*a*b+cs^2*c, r = b^2-cs^2
		// s = (-q +- sqrt(q^2 - 4*p*r))/(2*p),    0 <= s <= 1

		float VTmp[3];
		RVLDIF3VECTORS(N, N_, VTmp);
		float a = RVLDOTPRODUCT3(N0, VTmp);
		float b = RVLDOTPRODUCT3(N0, N_);
		float c = 2.0f * (1.0f - RVLDOTPRODUCT3(N, N_));
		float cs2 = pData->cs * pData->cs;
		float p = a*a - cs2 * c;
		float q = 2.0f * a * b + cs2 * c;
		float r = b * b - cs2;
		float f = -sqrt(q * q - 4.0f * p * r);
		float s = (-q + f) / (2 * p);
		
		if (s < 0.0f || s > 1.0f)
			s = (-q - f) / (2 * p);

		RVLSCALE3VECTOR(N, s, VTmp);
		float s_ = 1.0f - s;
		RVLSCALE3VECTOR(N_, s_, NT);
		RVLSUM3VECTORS(NT, VTmp, NT);
		float fTmp;
		RVLNORM3(NT, fTmp);

		//pRecognition->UpdateNormalHull(*(pData->pNormalHull), NT);

		QList<QLIST::Index> *pVertexList = pSurfels->surfelVertexList.Element + iSurfel;

		bool bMindT = false;

		int nTangentVertices = 0;
		
		pTangent->len = 0.0f;

		float *V = pTangent->V;

		RVLCROSSPRODUCT3(N0, NT, V);

		RVLNORM3(V, fTmp);

		Eigen::Matrix3f M;

		M << N0[0], N0[1], N0[2], NT[0], NT[1], NT[2], V[0], V[1], V[2];

		Eigen::Vector3f B, P_;
		SURFEL::Vertex *pVertex;
		int i;
		float mindT, dT, d_, len13, len23;
		float P1[3], P2[3], dP13[3], dP23[3], PProj[3], dP[3];
		float *P;

		QLIST::Index *pVertexIdx = pVertexList->pFirst;

		while (pVertexIdx)
		{
			pVertex = pSurfels->vertexArray.Element[pVertexIdx->Idx];

			for (i = 0; i < pVertex->iSurfelArray.n; i++)
			{
				if (pVertex->iSurfelArray.Element[i] == iSurfel_)
				{
					P = pVertex->P;

					dT = RVLDOTPRODUCT3(NT, P);

					d_ = RVLDOTPRODUCT3(V, P);

					B << d0, dT, d_;

					P_ = M.colPivHouseholderQr().solve(B);					
					
					if (nTangentVertices == 0)
					{
						nTangentVertices = 1;

						RVLCOPY3VECTOR(P_, P1);

						pTangent->iVertex[0] = pVertexIdx->Idx;
					}
					else if (nTangentVertices == 1)
					{
						nTangentVertices = 2;

						RVLCOPY3VECTOR(P_, P2);

						RVLDIF3VECTORS(P2, P1, dP);

						pTangent->len = RVLDOTPRODUCT3(dP, dP);

						pTangent->iVertex[1] = pVertexIdx->Idx;
					}
					else	// if (nTangentVertices == 2)
					{
						RVLCOPY3VECTOR(P_, PProj);

						RVLDIF3VECTORS(PProj, P1, dP13);

						len13 = RVLDOTPRODUCT3(dP13, dP13);

						RVLDIF3VECTORS(PProj, P2, dP23);

						len23 = RVLDOTPRODUCT3(dP23, dP23);

						if (len13 > pTangent->len || len23 > pTangent->len)
						{
							if (len13 > len23)
							{
								pTangent->iVertex[1] = pVertexIdx->Idx;

								pTangent->len = len13;
							}
							else
							{
								pTangent->iVertex[0] = pVertexIdx->Idx;

								pTangent->len = len23;
							}
						}
					}	// if (nTangentVertices == 2)

					if (bMindT)
					{
						if (dT < mindT)
							mindT = dT;
					}
					else
					{
						mindT = dT;

						bMindT = true;
					}
				}	// if (pVertex->iSurfelArray.Element[i] == iSurfel_) 			
			}	// for all surfels meeting in pVertex

			pVertexIdx = pVertexIdx->pNext;
		}	// for all vertices on the boundary of iSurfel

		pTangent->d = mindT;

		return -1;
	}	// if (RVLDOTPRODUCT3(N0, N) > pData->cs && RVLDOTPRODUCT3(N0, N_) <= pData->cs)
	else if (pData->clusterMap[iSurfel] == pData->iCluster)
	{
		pData->bParent[iSurfel] = true;

		//if (cs < pData->baseSeparationAngle)
			pData->bBase[iSurfel] = true;

		return 1;
	}
	else
		return -1;
}

bool PSGM::Inside(
	int iVertex, 
	RECOG::PSGM_::Cluster *pCluster,
	int iSurfel)
{
	float maxe = kNoise * 2.0f / pSurfelDetector->kPlane;

	SURFEL::Vertex *pVertex = pSurfels->vertexArray.Element[iVertex];

	int i;
	int iSurfel_;
	float e;
	Surfel *pSurfel_;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		iSurfel_ = pCluster->iSurfelArray.Element[i];

		if (iSurfel_ == iSurfel)
			continue;

		pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

		e = RVLDOTPRODUCT3(pSurfel_->N, pVertex->P) - pSurfel_->d;

		if (clusterType * e > maxe)
			return false;
	}

	return true;
}

bool PSGM::BelowPlane(
	RECOG::PSGM_::Cluster *pCluster,
	Surfel *pSurfel,
	int iFirstVertex)
{
	float maxe = kNoise * 2.0f / pSurfelDetector->kPlane;

	float e;
	int iVertex_;
	SURFEL::Vertex *pVertex;

	for (iVertex_ = 0; iVertex_ < pCluster->iVertexArray.n; iVertex_++)
	{
		pVertex = pSurfels->vertexArray.Element[pCluster->iVertexArray.Element[iVertex_]];

		e = RVLDOTPRODUCT3(pSurfel->N, pVertex->P) - pSurfel->d;

		if (clusterType * e > maxe)
			return false;
	}

	return true;
}

void PSGM::UpdateMeanNormal(
	float *sumN,
	float &wN,
	float *N,
	float w,
	float *meanN)
{
	float VTmp[3];
	RVLSCALE3VECTOR(N, w, VTmp);
	RVLSUM3VECTORS(sumN, VTmp, sumN);
	wN += w;
	RVLSCALE3VECTOR2(sumN, wN, meanN);
	float fTmp = sqrt(RVLDOTPRODUCT3(meanN, meanN));
	if (fTmp > 1e-10)
	{
		RVLSCALE3VECTOR2(meanN, fTmp, meanN);
	}
	else
		RVLSET3VECTOR(meanN, 0.0f, 0.0f, 1.0f);
}

void PSGM::SetSceneFileName(char *sceneFileName_)
{
	RVLCopyString(sceneFileName_, &sceneFileName);
}

void PSGM::SaveCTIs(
	FILE *fp,
	RECOG::CTISet *pCTISet,
	int iModel)
{
	int i;
	int iModelInstanceElement;
	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;

	RECOG::PSGM_::ModelInstance *pModelInstance = pCTISet->CTI.pFirst;

	while (pModelInstance)
	{
		fprintf(fp, "%d\t%d\t", iModel, pModelInstance->iCluster);

		for (i = 0; i < 9; i++)
			fprintf(fp, "%f\t", pModelInstance->R[i]);

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t", pModelInstance->t[i]);

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%f\t", pModelInstanceElement->d);
		}

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%d\t", (int)(pModelInstanceElement->valid));
		}

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%f\t", pModelInstanceElement->e);
		}

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t", pModelInstance->tc[i]);

		fprintf(fp, "\n");

		pModelInstance = pModelInstance->pNext;
	}
}

//Vidovic
void PSGM::SaveModelInstances(
	FILE *fp,
	int iModel)
{
	SaveCTIs(fp, &CTISet, iModel);
}


bool RECOG::ModelExistInDB(char *modelFileName, FileSequenceLoader dbLoader)
{
	char dbFileName[50];

	while (dbLoader.GetNextName(dbFileName))
		if (!strcmp(modelFileName, dbFileName))
			return 1;

	return 0;
}

void RECOG::SaveModelID(
	FileSequenceLoader dbLoader,
	char *modelsInDataBase)
{
	FILE *fp = fopen(modelsInDataBase, "w");

	char modelName[50], modelPath[200];
	int modelID;

	while (dbLoader.GetNext(modelPath, modelName, &modelID))
		fprintf(fp, "%d\t%s\n", modelID, modelName);

	fprintf(fp, "\nend");

	fclose(fp);
}

void PSGM::Learn(
	char *modelSequenceFileName,
	Visualizer *visualizer)
{
	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	if (problem == RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION)
	{
	FileSequenceLoader modelsLoader;
	FileSequenceLoader dbLoader;

	char modelFilePath[200];
	char modelFileName[200];

	Mesh mesh;

	//int iCluster;
	int nClusters, currentModelID;

	//RVL_DELETE_ARRAY(modelDataBase);
	//RVL_DELETE_ARRAY(modelsInDataBase);

	MTGSet.Clear();

	if (!modelDataBase)
		modelDataBase = "modelDB.dat";

	if (!modelsInDataBase)
		modelsInDataBase = "DBModels.txt";

	modelsLoader.Init(modelSequenceFileName);
	dbLoader.Init(modelsInDataBase);

	FILE *fp = fopen(modelDataBase, "a");

	bool saveDBSequenceFile = false;

	printf("Model DB creation started...\n");

	while (modelsLoader.GetNext(modelFilePath, modelFileName))
	{
		if (ModelExistInDB(modelFileName, dbLoader))
			continue;

		printf("\nProcessing model %s!\n", modelFileName);

		saveDBSequenceFile = true;

		//mesh.LoadPolyDataFromPLY(modelFilePath);
		LoadMesh(vpMeshBuilder, modelFilePath, &mesh, false);

		SetSceneFileName(modelFilePath);

		currentModelID = dbLoader.GetLastModelID() + 1;

		Interpret(&mesh, currentModelID);

		nClusters = RVLMIN(clusters.n, nDominantClusters);

		//Add vtkPolyData to vtkModelDB
		vtkModelDB.insert(std::make_pair(currentModelID, mesh.pPolygonData));

		SaveModelInstances(fp, currentModelID);

		dbLoader.AddModel(currentModelID, modelFilePath, modelFileName);

		if (visualizer)
		{
			pSurfels->NodeColors(SelectionColor);
			InitDisplay(visualizer, &mesh, SelectionColor);
			Display();
			visualizer->Run();

			visualizer->renderer->RemoveAllViewProps();
		}
	}

	printf("Model DB creation completed!\n");

	if (saveDBSequenceFile)
		SaveModelID(dbLoader, modelsInDataBase);

	fclose(fp);

	char *TGFileName = RVLCreateFileName(modelDataBase, ".dat", -1, ".tgr");

	MTGSet.Save(TGFileName);

	delete[] TGFileName;
}
	else if (problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
	{
		ObjectDetector *pObjectDetector = (ObjectDetector *)vpObjectDetector;

		pObjectDetector->bGroundTruthSegmentation = true;
		pObjectDetector->bGroundTruthSegmentationOnSurfelLevel = false;

		pObjectDetector->flags |= RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT;

		FILE *fp = fopen("TrainCTIs.txt", "w");

		fclose(fp);

		fp = fopen("TrainCTIs.txt", "a");

		FileSequenceLoader sceneSequence;
		sceneSequence.Init(modelSequenceFileName);

		int iScene = 0;

		char filePath[200];
		char fileName[200];

		while (sceneSequence.GetNext(filePath, fileName))
		{
			pMem->Clear();

			printf("Scene %s...\n", fileName);

			pObjectDetector->DetectObjects(filePath);

			CTISet.Init();

			CTIs(iScene, pObjects, &CTISet, pMem);

			SaveCTIs(fp, &CTISet, iScene);

			cv::imshow("Segmentation", pObjects->CreateSegmentationImage());

			//cv::waitKey();

			cv::waitKey(1);

			if (visualizer)
			{
				pSurfels->NodeColors(SelectionColor);
				pObjectDetector->pObjects->InitDisplay(visualizer, &(pObjectDetector->mesh), SelectionColor);
				pObjectDetector->pObjects->Display();
				visualizer->Run();

				visualizer->renderer->RemoveAllViewProps();
			}

			iScene++;
		}

		fclose(fp);
	}
}


void PSGM::LoadModelMeshDB(char *modelSequenceFileName, std::map<int, vtkSmartPointer<vtkPolyData>> *vtkModelDB, bool bDecimate, float decimatePercent)
{
	FileSequenceLoader modelsLoader;

	char modelFilePath[200];
	char modelFileName[200];

	Mesh mesh;

	int nClusters, currentModelID = 0;

	if (!modelsInDataBase)
		modelsInDataBase = "DBModels.txt";

	modelsLoader.Init(modelSequenceFileName);

	printf("Starting VTK Model DB creation.\n");
	while (modelsLoader.GetNext(modelFilePath, modelFileName))
	{
		printf("Loading VTK model %s to DB!\n", modelFileName);

		mesh.LoadPolyDataFromPLY(modelFilePath);
		vtkSmartPointer<vtkDecimatePro> decimate = vtkSmartPointer<vtkDecimatePro>::New();
		//mesh.pPolygonData->Print(std::cout);

		if (bDecimate) //subsampling the model to reduce number of points and fasten the process
		{
			decimate->SetInputData(mesh.pPolygonData);
			decimate->SetTargetReduction(decimatePercent);
			decimate->Update();
			//decimate->GetOutput()->Print(std::cout);
		}
		
		//Calculate normals
		vtkSmartPointer<vtkPolyDataNormals> normalsFilter = vtkSmartPointer<vtkPolyDataNormals>::New();
		normalsFilter->ComputePointNormalsOn();
		normalsFilter->SplittingOff();
		if (bDecimate)
			normalsFilter->SetInputConnection(decimate->GetOutputPort());
		else
			normalsFilter->SetInputData(mesh.pPolygonData);
		normalsFilter->Update();
		//normalsFilter->GetOutput()->Print(std::cout);
		/*vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
		cleanFilter->SetInputConnection(normalsFilter->GetOutputPort());
		cleanFilter->PointMergingOn();
		cleanFilter->SetAbsoluteTolerance(0.03);
		cleanFilter->ToleranceIsAbsoluteOn();
		cleanFilter->Update();*/
		//cleanFilter->GetOutput()->Print(std::cout);
		
		//Add vtkPolyData to vtkModelDB:
		normalsFilter->GetOutput()->GetPointData()->RemoveArray("RGB"); //Vidovic merging 20.07.2017
		vtkModelDB->insert(std::make_pair(currentModelID, normalsFilter->GetOutput()));
		currentModelID++;

		//vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
		//writer->SetFileName("test.ply");
		//writer->SetInputData(normalsFilter->GetOutput());
		////writer->SetFileTypeToASCII();
		////writer->SetColorModeToDefault();
		////writer->SetArrayName("RGB");
		//writer->Write();


	}

	printf("VTK Model DB creation completed!\n");
}


void PSGM::LoadModelDataBase()
{
	MCTISet.nT = convexTemplate.n;

	if (modelDataBase == NULL)
		return;

	MCTISet.Load(modelDataBase);

	//Alocate arrays for Match() function
	e.Element = new Array<float>[MCTISet.pCTI.n];
	e.n = MCTISet.pCTI.n;

	tBestMatch.Element = new Array<float>[MCTISet.pCTI.n];
	tBestMatch.n = MCTISet.pCTI.n;


	score.Element = new float[MCTISet.pCTI.n];
	score.n = MCTISet.pCTI.n;

	for (int i = 0; i < MCTISet.pCTI.n; i++)
	{
		e.Element[i].Element = new float[convexTemplate.n];
		e.Element[i].n = convexTemplate.n;

		tBestMatch.Element[i].Element = new float[3];
		tBestMatch.Element[i].n = 3;
	}

	//char *TGFileName = RVLCreateFileName(modelDataBase, ".dat", -1, ".tgr");

	//MTGSet.Load(TGFileName);

	//delete[] TGFileName;
}

void PSGM::LoadCTI(char *fileName)
{
	CTISet.nT = convexTemplate.n;

	CTISet.Load(fileName);
}

#ifdef NEVER
void PSGM::Match()
{
	printf("Scene to model match started...");

	ClearMatchMatrix(); //Vidovic

	float csMinSampleAngleDiff = cos(PI / 4);
	float minE, minETotal, E, score, SMI_minETotal;

	float cos45 = cos(PI / 4);

	float sigma = 4;
	float sigma25 = 2.5*2.5;

	float tBestMatch[3], SMI_tBestMatch[3];
	int bestMatchMIMID, bestMatchModelID, bestSRF, iSRF;
	int SMI_bestMatchMIMID, SMI_bestMatchModelID, SMI_bestSRF, SMI_iSRF;

	QList<RECOG::PSGM_::MatchInstance> *pSMIMatches = &SMImatches;

	//if (iScene == 0)
	RVLQLIST_INIT(pSMIMatches);

	RECOG::PSGM_::MatchInstance *pSMIMatch;

	float R_[9], t_[3];

	Eigen::Matrix3f A;
	Eigen::Vector3f B, t;

	int i, idx;

	float dISv, dIMvt;

	FILE *fp;

	//if (!sceneMIMatch)
	//	sceneMIMatch = "sceneMatch.txt";

	//fp = fopen(sceneMIMatch, "w");

	//find sample candidates
	QList<QLIST::Index> iSampleCandidateList;
	QList<QLIST::Index> *pISampleCandidateList = &iSampleCandidateList;

	RVLQLIST_INIT(pISampleCandidateList);

	QLIST::Index *pNewISampleCandidate;
	RECOG::PSGM_::Plane *pPlane = convexTemplate.Element;

	for (int idx = 0; idx < convexTemplate.n; idx++)
	{
		if (RVLABS(pPlane->N[2]) <= csMinSampleAngleDiff)
		{
			RVLMEM_ALLOC_STRUCT(pMem, QLIST::Index, pNewISampleCandidate);

			RVLQLIST_ADD_ENTRY(pISampleCandidateList, pNewISampleCandidate);

			pNewISampleCandidate->Idx = idx;
		}		

		pPlane++;
	}

	int iMIS, iMIM;
	int iSCluster, iSClusterMI;

	int MatchID = 0;

	int nClusters = RVLMIN(clusters.n, nDominantClusters);

	RECOG::PSGM_::ModelInstance *pSModelInstance;
	RECOG::PSGM_::ModelInstance *pMModelInstance;

	RECOG::PSGM_::ModelInstance *pSBestModelInstance;
	RECOG::PSGM_::ModelInstance *pMBestModelInstance;

	RECOG::PSGM_::ModelInstance *SMI_pSBestModelInstance;
	RECOG::PSGM_::ModelInstance *SMI_pMBestModelInstance;

	bool breakPrint;
	bool cluserMatch;

	//Arrays allocation
	Array<QLIST::Index> iValidSampleCandidate;
	iValidSampleCandidate.Element = new QLIST::Index[convexTemplate.n];

	Array<QLIST::Index> iValid;
	iValid.Element = new QLIST::Index[convexTemplate.n];

	Array<QLIST::Index> iRansacCandidates;
	iRansacCandidates.Element = new QLIST::Index[26]; //max 26 planes which satisfy condition

	Array<QLIST::Index> iConsensus;
	iConsensus.Element = new QLIST::Index[convexTemplate.n];

	Array<QLIST::Index> iConsensusTemp;
	iConsensusTemp.Element = new QLIST::Index[convexTemplate.n];

	iMIS = 0;

	bool segmentTP, TP_;

	float distanceThresh = 50;
	
	char *fileName = strrchr(sceneFileName, '\\') + 1;
	
	if (strcmp(fileName, "frame_20111220T114628.408278.ply") == 0)
		distanceThresh = 45;
	else if (strcmp(fileName, "frame_20111220T115430.348560.ply") == 0)
		distanceThresh = 20;
	else if (strcmp(fileName, "frame_20111220T115445.303284.ply") == 0)
		distanceThresh = 15;
	else if (strcmp(fileName, "frame_20111221T142636.413299.ply") == 0)
		distanceThresh = 25;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		segmentTP = false;

		breakPrint = false; //for DEBUG!

		cluserMatch = false;

		printf("%d/%d", iSCluster + 1, nClusters);

		pSModelInstance = clusters.Element[iSCluster]->modelInstanceList.pFirst;

		iSRF = -1;

		while (pSModelInstance)
		{
			SMI_minETotal = 66.0;

			iSRF++;

			pMModelInstance = modelInstanceDB.Element;

			for (iMIM = 0; iMIM < modelInstanceDB.n; iMIM++)
			{
				//minE = 66.0;
				minE = 412.5; // for sigma = 2.5^2

				//CTDMatchRANSAC
				float pPrior = 2.5 * stdNoise;

				int nValidSampleCandidates = 0;

				//find valid sample candidates
				iValidSampleCandidate.n = 0;

				QLIST::Index *piValidSampleCandidate = iValidSampleCandidate.Element;

				QLIST::Index *pISampleCandidate = pISampleCandidateList->pFirst;

				while (pISampleCandidate)
				{
					if (pSModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true && pMModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true)
					{
						piValidSampleCandidate->Idx = pISampleCandidate->Idx;

						piValidSampleCandidate->pNext = piValidSampleCandidate + 1;

						piValidSampleCandidate++;

						nValidSampleCandidates++;
					}

					pISampleCandidate = pISampleCandidate->pNext;
				}

				iValidSampleCandidate.n = nValidSampleCandidates;

				if (nValidSampleCandidates > 2)
				{
					int nValids = 0;

					//find iValid
					iValid.n = 0;

					QLIST::Index *piValid = iValid.Element;
					int iPlane;

					for (iPlane = 0; iPlane < convexTemplate.n; iPlane++)
					{
						if (pSModelInstance->modelInstance.Element[iPlane].valid == true && pMModelInstance->modelInstance.Element[iPlane].valid == true)
						{
							piValid->Idx = iPlane;

							piValid->pNext = piValid + 1;

							piValid++;

							nValids++;
						}
					}

					iValid.n = nValids;

					int iRansac;
					bool bValidSample;

					int iSample[2];
					int ID[2];
					float V[3];
					float N_[3] = { 0, 0, 1 };
					float N__[3] = { 0, -cos45, cos45 };
					float fTmp;
					int nValidSamplesSearch;
					int nRansacCandidates = 0;

					iConsensus.n = 0;

					QLIST::Index *piConsensus = iConsensus.Element;

					iRansacCandidates.n = 0;

					QLIST::Index *piRansacCandidates = iRansacCandidates.Element;

					//find RANSAC candidates
					RVLCROSSPRODUCT3(N_, N__, V);

					fTmp = sqrt(RVLDOTPRODUCT3(V, V));

					RVLSCALE3VECTOR2(V, fTmp, V);
					
					for (i = 0; i < iValidSampleCandidate.n; i++)
					{
						ID[0] = iValidSampleCandidate.Element[i].Idx;

						fTmp = RVLDOTPRODUCT3(V, convexTemplate.Element[ID[0]].N);

						if (RVLABS(fTmp) >= csMinSampleAngleDiff)
						{
							piRansacCandidates->Idx = ID[0];

							piRansacCandidates++;

							nRansacCandidates++;
						}
					}

					iRansacCandidates.n = nRansacCandidates;

					std::random_device rd;
					std::mt19937 eng(rd());
					std::uniform_int_distribution<> distribution(0, nRansacCandidates);

					nSamples = RVLMIN(13, nRansacCandidates);

					for (iRansac = 0; iRansac < nSamples; iRansac++)
					{
#ifdef NEVER
						bValidSample = false;

						nValidSamplesSearch = 0;

						while (!bValidSample)
						{
							nValidSamplesSearch++;

							iSample[0] = distribution(eng);
							iSample[1] = distribution(eng);

							ID[0] = iValidSampleCandidate.Element[iSample[0]].Idx;
							ID[1] = iValidSampleCandidate.Element[iSample[1]].Idx;

							RVLCROSSPRODUCT3(N_, convexTemplate.Element[ID[0]].N, V);

							fTmp = sqrt(RVLDOTPRODUCT3(V, V));

							//if (RVLABS(fTmp) < 1e-10)
							//return;

							RVLSCALE3VECTOR2(V, fTmp, V);

							if (RVLDOTPRODUCT3(V, convexTemplate.Element[ID[1]].N) >= csMinSampleAngleDiff)
								bValidSample = true;

							if (nValidSamplesSearch > 2000)
								break;
						}

						if (nValidSamplesSearch > 2000)
							break;
#endif

						ID[0] = 1; //second plane from convexTemplate

						iSample[1] = distribution(eng);

						ID[1] = iValidSampleCandidate.Element[iSample[1]].Idx;

						float dM[3];
						float dS[3];
						float N[9];

						dM[0] = pMModelInstance->modelInstance.Element[0].d;
						dM[1] = pMModelInstance->modelInstance.Element[ID[0]].d;
						dM[2] = pMModelInstance->modelInstance.Element[ID[1]].d;

						dS[0] = pSModelInstance->modelInstance.Element[0].d * 1000;
						dS[1] = pSModelInstance->modelInstance.Element[ID[0]].d * 1000;
						dS[2] = pSModelInstance->modelInstance.Element[ID[1]].d * 1000;

						RVLCOPYTOCOL3(convexTemplate.Element[0].N, 0, N);
						RVLCOPYTOCOL3(convexTemplate.Element[ID[0]].N, 1, N);
						RVLCOPYTOCOL3(convexTemplate.Element[ID[1]].N, 2, N);

						A << N[0], N[3], N[6], N[1], N[4], N[7], N[2], N[5], N[8]; // N'
						B << dS[0] - dM[0], dS[1] - dM[1], dS[2] - dM[2];
						t = A.colPivHouseholderQr().solve(B);

						iConsensusTemp.n = 0;

						QLIST::Index *piConsensusTemp = iConsensusTemp.Element;

						E = 0;

						for (i = 0; i < iValid.n; i++)
						{
							idx = iValid.Element[i].Idx;

							dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

							dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

							//fTmp = (dISv - dIMvt) / pPrior;
							fTmp = (dISv - dIMvt) / sigma;

							//if (fTmp*fTmp < 1)
							#ifdef RVLPSGM_MATCH_SATURATION
								if (fTmp*fTmp < sigma25)
								//if (fTmp*fTmp < 1)
								{
									E += fTmp*fTmp;

									piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
									piConsensusTemp->pNext = piConsensusTemp + 1;

									iConsensusTemp.n++;

									piConsensusTemp++;
								}
								else
									E += sigma25;
									//E += 1;
							#else
								E += fTmp*fTmp;

								//TREBA LI OVO?
								piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
								piConsensusTemp->pNext = piConsensusTemp + 1;

								iConsensusTemp.n++;

								piConsensusTemp++;
								
							#endif
						}

						if (E < minE)
						{
							minE = E;

							iConsensus.n = iConsensusTemp.n;

							piConsensusTemp = iConsensusTemp.Element;

							piConsensus = iConsensus.Element;

							for (i = 0; i < iConsensusTemp.n; i++)
							{
								piConsensus->Idx = piConsensusTemp->Idx;
								piConsensus->pNext = piConsensus + 1;

								piConsensus++;
								piConsensusTemp++;
							}
						}
					}

					if (iConsensus.n >= 3)
					{
						float dISc, dIMc, *nTc, *dISMc;

						nTc = new float[3 * iConsensus.n];
						dISMc = new float[iConsensus.n];

						piConsensus = iConsensus.Element;

						for (i = 0; i < iConsensus.n; i++)
						{
							idx = piConsensus->Idx;

							dISc = pSModelInstance->modelInstance.Element[idx].d * 1000;
							dIMc = pMModelInstance->modelInstance.Element[idx].d;

							dISMc[i] = dISc - dIMc;

							nTc[i] = convexTemplate.Element[idx].N[0];
							nTc[iConsensus.n + i] = convexTemplate.Element[idx].N[1];
							nTc[2 * iConsensus.n + i] = convexTemplate.Element[idx].N[2];

							piConsensus++;
						}

						int j, k;

						//nTc*nTc'
						for (i = 0; i < 3; i++)
							for (j = 0; j < 3; j++)
								if (i <= j)
								{
									A(i * 3 + j) = 0;

									for (k = 0; k < iConsensus.n; k++)
										A(i * 3 + j) += nTc[i * iConsensus.n + k] * nTc[j * iConsensus.n + k];
								}
								else
									A(i * 3 + j) = A(j * 3 + i);

						//nTc*(dISc-dIMc)
						for (i = 0; i < 3; i++)
						{
							B(i) = 0;
							for (j = 0; j < iConsensus.n; j++)
								B(i) += nTc[i * iConsensus.n + j] * dISMc[j];
						}

						t = A.colPivHouseholderQr().solve(B);

						delete[] nTc;
						delete[] dISMc;

						E = 0;

						for (i = 0; i < iValid.n; i++)
						{
							idx = iValid.Element[i].Idx;

							dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

							dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

							//fTmp = (dISv - dIMvt) / pPrior;
							fTmp = (dISv - dIMvt) / sigma;

							#ifdef RVLPSGM_MATCH_SATURATION
								//if (fTmp*fTmp < 1)
								if (fTmp*fTmp < sigma25)
									E += fTmp*fTmp;
								else
									//E += 1;
									E += sigma25;
							#else
								E += fTmp*fTmp;
							#endif
						}

						//score = E + 66 - iValid.n;
						score = E + sigma25*(66 - iValid.n);

						//save all SMI matches						
						for (i = 0; i < 3; i++)
							SMI_tBestMatch[i] = t(i);

						MSTransformation(pMModelInstance, pSModelInstance, SMI_tBestMatch, R_, t_);

						RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pSMIMatch);

						RVLQLIST_ADD_ENTRY(pSMIMatches, pSMIMatch);

						FillMatch(pSMIMatch, MatchID, iScene, iSCluster, iSRF, iMIS, pMModelInstance->iModel, pMModelInstance->iCluster, iMIM, R_, t_, SMI_tBestMatch, E, score, 0.0, 0.0, NAN, NAN, iValid.n);

						MatchID++;							

						//check MatchMatrix update
						if (matchMatrix.Element[pSMIMatch->iCluster].Element[pSMIMatch->iModel*nMSegments + pSMIMatch->iMCluster] == NULL)
						{
							UpdateMatchMatrix(pSMIMatch, pSMIMatch->score);
						}
						else
						{
							if (pSMIMatch->score < matchMatrix.Element[pSMIMatch->iCluster].Element[pSMIMatch->iModel*nMSegments + pSMIMatch->iMCluster]->score)
							{
								UpdateMatchMatrix(pSMIMatch, pSMIMatch->score);
							}
						}

						//create SegmentGT
						if (createSegmentGT)
						{
							TP_ = CompareMatchToGT(pSMIMatch, true, 0.0, distanceThresh);

							if (TP_ && !segmentTP)
							{
								segmentTP = true;
								segmentGT.Element[iScene*nDominantClusters + iSCluster].iScene = iScene;

								segmentGT.Element[iScene*nDominantClusters + iSCluster].iSSegment = iSCluster;
								segmentGT.Element[iScene*nDominantClusters + iSCluster].iModel = pSMIMatch->iModel;
								segmentGT.Element[iScene*nDominantClusters + iSCluster].iMSegment = pSMIMatch->iMCluster;
							}
						}
					}
					else
					{
						t << 0, 0, 0;
						//E = 66;
						E = 412.5;
					}
				}
				else
				{
					if (!breakPrint)
					{
						//printf("BREAK - nValidSampleCandidates = %d; iCluster: %d; iMIM: %d \n", nValidSampleCandidates, iSCluster, iMIM);
						breakPrint = true;
					}
				}

				pMModelInstance = pMModelInstance->pNext;

			}	//for all model MI

			iMIS++;

			pSModelInstance = pSModelInstance->pNext;

		}	// for all MI in cluster

		//Scene Segment doesn't have GT instance
		if (createSegmentGT)
		{
			if (!segmentTP)
			{
				segmentGT.Element[iScene*nDominantClusters + iSCluster].iScene = iScene;
				segmentGT.Element[iScene*nDominantClusters + iSCluster].iSSegment = iSCluster;
				segmentGT.Element[iScene*nDominantClusters + iSCluster].iModel = -1;
				segmentGT.Element[iScene*nDominantClusters + iSCluster].iMSegment = -1;
			}
		}

		if (clusters.n < 10)
			printf("\b");
		else
			printf("\b\b");

		if (iSCluster < 9)
		{
			printf("\b\b");
		}
		else
			printf("\b\b\b");

	}	// for all dominant clusters

	printf("completed.\n");

	int nSMI = iMIS;

	RVL_DELETE_ARRAY(iConsensusTemp.Element);

	RVL_DELETE_ARRAY(iValid.Element);

	RVL_DELETE_ARRAY(iConsensus.Element);

	RVL_DELETE_ARRAY(iRansacCandidates.Element);

	RVL_DELETE_ARRAY(iValidSampleCandidate.Element);

#ifdef PSGM_CALCULATE_PROBABILITY

	//calculate p(Mi|d) probability
	//float sigma = 1; //POSTAVITI KAO VANJSKI PARAMETAR?

	//float tConst = 1 / (2 * sigma);
	float tConst = 0.5;

	int nModels = 35;
	int nMSegments = 3;

	float maxCost = 412.5;

	int maxMSegments = nModels * nMSegments;

	int iSMI, iModel, iMSegment, j;

	float probability;

	int nMSCTI;

	Array<float> SMIminE;
	SMIminE.Element = new float[nSMI];

	for (i = 0; i < nSMI; i++)	
		SMIminE.Element[i] = maxCost;

	Array<Array<RECOG::PSGM_::MatchInstance*>> DBMBestMatches;
	DBMBestMatches.Element = new Array<RECOG::PSGM_::MatchInstance*>[nSMI];

	for (i = 0; i < nSMI; i++)
		DBMBestMatches.Element[i].Element = new RECOG::PSGM_::MatchInstance*[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			DBMBestMatches.Element[i].Element[j] = NULL;

	RECOG::PSGM_::MatchInstance *pDBMBestMatches;

	Array<Array<float>> DBMminE;
	DBMminE.Element = new Array<float>[nSMI];
		
	for (i = 0; i < nSMI; i++)
		DBMminE.Element[i].Element = new float[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			DBMminE.Element[i].Element[j] = maxCost;

	Array<Array<float>> SMIProbability1;

	SMIProbability1.Element = new Array<float>[nSMI];

	for (i = 0; i < nSMI; i++)
		SMIProbability1.Element[i].Element = new float[maxMSegments];

	Array<Array<float>> SMIProbability2;

	SMIProbability2.Element = new Array<float>[nSMI];

	for (i = 0; i < nSMI; i++)
		SMIProbability2.Element[i].Element = new float[maxMSegments];

	Array<Array<int>> nCTI;

	nCTI.Element = new Array<int>[nSMI];

	for (i = 0; i < nSMI; i++)
		nCTI.Element[i].Element = new int[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			nCTI.Element[i].Element[j] = 0.0;

	Array<Array<float>> Msum_;
	Msum_.Element = new Array<float>[nSMI];

	for (i = 0; i < nSMI; i++)
		Msum_.Element[i].Element = new float[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			Msum_.Element[i].Element[j] = 0.0;	

	Array<float> SMIsum;
	SMIsum.Element = new float[nSMI];

	Array<float> SMIsum2;
	SMIsum2.Element = new float[nSMI];

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 1 (DEBUG)!\n");

	//find min score for all SMI and for all DB model segments 
	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;
			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			if (pSMIMatch->E < SMIminE.Element[iSMI])
				SMIminE.Element[iSMI] = pSMIMatch->E;

			if (pSMIMatch->E < DBMminE.Element[iSMI].Element[iMSegment])
			{
				DBMminE.Element[iSMI].Element[iMSegment] = pSMIMatch->E;
				DBMBestMatches.Element[iSMI].Element[iMSegment] = pSMIMatch;
			}

			pSMIMatch = pSMIMatch->pNext;			
		}
		else
		{
			break;
		}		
	}

	for (i = 0; i < nSMI; i++)
		SMIsum.Element[i] = 0.0;

	printf("Probability calculation - STEP 2 (DEBUG)!\n");

	for (i = 0; i < nSMI; i++)
	{
		for (j = 0; j < maxMSegments; j++)
		{
			pSMIMatch = DBMBestMatches.Element[i].Element[j];

			if (pSMIMatch != NULL)
			{
				iSMI = pSMIMatch->iSMI;

				iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

				//SMIProbability1.Element[iSMI].Element[iMSegment] = exp(tConst*(SMIminE.Element[iSMI] * SMIminE.Element[iSMI] - pSMIMatch->E * pSMIMatch->E));
				SMIProbability1.Element[iSMI].Element[iMSegment] = exp(tConst*(SMIminE.Element[iSMI] - pSMIMatch->E));

				SMIsum.Element[iSMI] += SMIProbability1.Element[iSMI].Element[iMSegment];
			}
		}
	}

	printf("Probability calculation - STEP 3 (DEBUG)!\n");

	float tempSum = 0;

	for (i = 0; i < nSMI; i++)
	{
		for (j = 0; j < maxMSegments; j++)
		{
			pSMIMatch = DBMBestMatches.Element[i].Element[j];

			if (pSMIMatch != NULL)
			{
				iSMI = pSMIMatch->iSMI;

				iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

				probability = SMIProbability1.Element[iSMI].Element[iMSegment];

				pSMIMatch->probability1 = probability / SMIsum.Element[iSMI];

				tempSum += pSMIMatch->probability1;
			}
		}

		if (iSMI == 91)
			printf("\n\nTempSum P1: %f\n\n", tempSum);

		tempSum = 0;
	}

	//reset SMIminE
	//for (i = 0; i < nSMI; i++)
	//	SMIminE.Element[i] = 66.0;

	for (i = 0; i < nSMI; i++)
		SMIsum.Element[i] = 0.0;

	//pSMIMatch = SMImatches.pFirst;

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 4 (DEBUG)!\n");

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			nCTI.Element[iSMI].Element[iMSegment]++;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}


	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 5 (DEBUG)!\n");

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			probability = exp(tConst*(SMIminE.Element[iSMI] - pSMIMatch->E));

			nMSCTI = nCTI.Element[iSMI].Element[iMSegment];

			Msum_.Element[iSMI].Element[iMSegment] += probability / nMSCTI;

			SMIsum.Element[iSMI] += probability / nMSCTI;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}

	//calculate probability2
	printf("Probability calculation - STEP 6 (DEBUG)!\n");

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			if (SMIsum.Element[iSMI] > 0.0)
				pSMIMatch->probability2 = Msum_.Element[iSMI].Element[iMSegment] / SMIsum.Element[iSMI];
			else
				pSMIMatch->probability2 = 0.0;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}

	//find best matches for each scene segment
	QList<RECOG::PSGM_::MatchInstance> *pSSegmentMatches1 = &SSegmentMatches1;
	QList<RECOG::PSGM_::MatchInstance> *pSSegmentMatches2 = &SSegmentMatches2;

	RECOG::PSGM_::MatchInstance *pMatch;
	RECOG::PSGM_::MatchInstance *pNewMatch;

	bool newMatch = true;

	//if (iScene == 0)
	//{
	RVLQLIST_INIT(pSSegmentMatches1);
	RVLQLIST_INIT(pSSegmentMatches2);
	//}

	printf("Probability calculation - STEP 7 (DEBUG)!\n");

	//Probability1
	for (i = 0; i < nSMI; i++)
	{
		for (j = 0; j < maxMSegments; j++)
		{
			pSMIMatch = DBMBestMatches.Element[i].Element[j];

			if (pSMIMatch != NULL)
			{
				if (pSMIMatch->iScene == iScene)
				{
					iSMI = pSMIMatch->iSMI;

					iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

					pMatch = pSSegmentMatches1->pFirst;

					while (pMatch)
					{
						if (pMatch->iScene == pSMIMatch->iScene && pMatch->iCluster == pSMIMatch->iCluster && pMatch->iModel == pSMIMatch->iModel && pMatch->iMCluster == pSMIMatch->iMCluster)
						{
							//change pMatch in QList
							if (pSMIMatch->probability1 > pMatch->probability1)
							{
								pMatch->iCRF = pSMIMatch->iCRF;

								pMatch->iSMI = pSMIMatch->iSMI;

								pMatch->iMMI = pSMIMatch->iMMI;

								for (i = 0; i < 9; i++)
									pMatch->R[i] = pSMIMatch->R[i];

								for (i = 0; i < 3; i++)
								{
									pMatch->t[i] = pSMIMatch->t[i];
									pMatch->tMatch[i] = pSMIMatch->tMatch[i];
								}

								pMatch->E = pSMIMatch->E;

								pMatch->score = pSMIMatch->score;

								pMatch->probability1 = pSMIMatch->probability1;

								pMatch->probability2 = pSMIMatch->probability2;

								pMatch->angle = pSMIMatch->angle;

								pMatch->distance = pSMIMatch->distance;
							}

							newMatch = false;

							break;
						}

						pMatch = pMatch->pNext;
					}

					if (newMatch)
					{
						//Add new match to QList
						RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pNewMatch);

						RVLQLIST_ADD_ENTRY(pSSegmentMatches1, pNewMatch);

						pNewMatch->iScene = pSMIMatch->iScene;

						pNewMatch->iCluster = pSMIMatch->iCluster;

						pNewMatch->iCRF = pSMIMatch->iCRF;

						pNewMatch->iSMI = pSMIMatch->iSMI;

						pNewMatch->iModel = pSMIMatch->iModel;

						pNewMatch->iMCluster = pSMIMatch->iMCluster;

						pNewMatch->iMMI = pSMIMatch->iMMI;

						for (i = 0; i < 9; i++)
							pNewMatch->R[i] = pSMIMatch->R[i];

						for (i = 0; i < 3; i++)
						{
							pNewMatch->t[i] = pSMIMatch->t[i];
							pNewMatch->tMatch[i] = pSMIMatch->tMatch[i];
						}

						pNewMatch->E = pSMIMatch->E;

						pNewMatch->score = pSMIMatch->score;

						pNewMatch->probability1 = pSMIMatch->probability1;

						pNewMatch->probability2 = pSMIMatch->probability2;

						pNewMatch->angle = pSMIMatch->angle;

						pNewMatch->distance = pSMIMatch->distance;
					}

					newMatch = true;
				}
			}
		}
	}

	//Probability2
	newMatch = true;

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 8 (DEBUG)!\n");

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			pMatch = pSSegmentMatches2->pFirst;

			while (pMatch)
			{
				if (pMatch->iScene == pSMIMatch->iScene && pMatch->iCluster == pSMIMatch->iCluster && pMatch->iModel == pSMIMatch->iModel && pMatch->iMCluster == pSMIMatch->iMCluster)
				{
					//change pMatch in QList
					if ((pSMIMatch->probability2 > pMatch->probability2) || (pSMIMatch->probability2 == pMatch->probability2 && pSMIMatch->E < pMatch->E))
					{
						pMatch->iCRF = pSMIMatch->iCRF;

						pMatch->iSMI = pSMIMatch->iSMI;

						pMatch->iMMI = pSMIMatch->iMMI;

						for (i = 0; i < 9; i++)
							pMatch->R[i] = pSMIMatch->R[i];

						for (i = 0; i < 3; i++)
						{
							pMatch->t[i] = pSMIMatch->t[i];
							pMatch->tMatch[i] = pSMIMatch->tMatch[i];
						}

						pMatch->E = pSMIMatch->E;

						pMatch->score = pSMIMatch->score;

						pMatch->probability1 = pSMIMatch->probability1;

						pMatch->probability2 = pSMIMatch->probability2;

						pMatch->angle = pSMIMatch->angle;

						pMatch->distance = pSMIMatch->distance;
					}

					newMatch = false;

					break;
				}

				pMatch = pMatch->pNext;
			}

			if (newMatch)
			{
				//Add new match to QList
				RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pNewMatch);

				RVLQLIST_ADD_ENTRY(pSSegmentMatches2, pNewMatch);

				pNewMatch->iScene = pSMIMatch->iScene;

				pNewMatch->iCluster = pSMIMatch->iCluster;

				pNewMatch->iCRF = pSMIMatch->iCRF;

				pNewMatch->iSMI = pSMIMatch->iSMI;

				pNewMatch->iModel = pSMIMatch->iModel;

				pNewMatch->iMCluster = pSMIMatch->iMCluster;

				pNewMatch->iMMI = pSMIMatch->iMMI;

				for (i = 0; i < 9; i++)
					pNewMatch->R[i] = pSMIMatch->R[i];

				for (i = 0; i < 3; i++)
				{
					pNewMatch->t[i] = pSMIMatch->t[i];
					pNewMatch->tMatch[i] = pSMIMatch->tMatch[i];
				}

				pNewMatch->E = pSMIMatch->E;

				pNewMatch->score = pSMIMatch->score;

				pNewMatch->probability1 = pSMIMatch->probability1;

				pNewMatch->probability2 = pSMIMatch->probability2;

				pNewMatch->angle = pSMIMatch->angle;

				pNewMatch->distance = pSMIMatch->distance;
			}

			newMatch = true;

			//pCurrentSceneMatch = pSMIMatch;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}			

	RVL_DELETE_ARRAY(SMIminE.Element);

	RVL_DELETE_ARRAY(DBMBestMatches.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(DBMminE.Element[i].Element);

	RVL_DELETE_ARRAY(DBMminE.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(SMIProbability1.Element[i].Element);

	RVL_DELETE_ARRAY(SMIProbability1.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(SMIProbability2.Element[i].Element);

	RVL_DELETE_ARRAY(SMIProbability2.Element);

	//RVL_DELETE_ARRAY(Msum.Element);

	RVL_DELETE_ARRAY(SMIsum.Element);

	RVL_DELETE_ARRAY(SMIsum2.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(Msum_.Element[i].Element);

	RVL_DELETE_ARRAY(Msum_.Element);

#endif

	iScene++;

	//fclose(fp);

	//reset GT matches flag
	pECCVGT->ResetMatchFlag();

	SortMatchMatrix();

#ifdef RVLPSGM_SAVE_MATCHES
	printf("Saving matches to txt file...");
	SaveMatches();
	printf("completed!\n\n");
#endif
}
#endif

void PSGM::Match()
{
	printf("Scene to model match started...");

	matchID = 0;

	pCTImatches = &CTImatches;
	RVLQLIST_INIT(pCTImatches);

	float csMinSampleAngleDiff = cos(PI / 4);

	int iSRF;
	//int i;

	//find sample candidates
	QList<QLIST::Index> iSampleCandidateList;
	pISampleCandidateList = &iSampleCandidateList;

	RVLQLIST_INIT(pISampleCandidateList);

	QLIST::Index *pNewISampleCandidate;
	RECOG::PSGM_::Plane *pPlane = convexTemplate.Element;

	for (int idx = 0; idx < convexTemplate.n; idx++)
	{
		if (RVLABS(pPlane->N[2]) <= csMinSampleAngleDiff)
		{
			RVLMEM_ALLOC_STRUCT(pMem, QLIST::Index, pNewISampleCandidate);

			RVLQLIST_ADD_ENTRY(pISampleCandidateList, pNewISampleCandidate);

			pNewISampleCandidate->Idx = idx;
		}

		pPlane++;
	}

	int iMIS;
	int iSCluster;
	//int iSClusterMI;

	int nClusters = CTISet.maxSegmentIdx + 1;	

	RECOG::PSGM_::ModelInstance *pSModelInstance;
	RECOG::PSGM_::ModelInstance *pMModelInstance;

	int iSCTI, nCTI;

	int iMSegment;

	//int maxMSegments = (MCTISet.nModels + 1) * (MCTISet.maxSegmentIdx + 1);
	int maxMSegments = (MCTISet.nModels) * (MCTISet.maxSegmentIdx + 1);

	//delete scoreMatchMatrix	
	for (iSCluster = 0; iSCluster < scoreMatchMatrix.n; iSCluster++)
		RVL_DELETE_ARRAY(scoreMatchMatrix.Element[iSCluster].Element);

	RVL_DELETE_ARRAY(scoreMatchMatrix.Element);

	scoreMatchMatrix.Element = new Array<SortIndex<float>>[nClusters];
	scoreMatchMatrix.n = nClusters;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		scoreMatchMatrix.Element[iSCluster].Element = new SortIndex<float>[maxMSegments];
		scoreMatchMatrix.Element[iSCluster].n = maxMSegments;

		for (iMSegment = 0; iMSegment < maxMSegments; iMSegment++)
		{
			scoreMatchMatrix.Element[iSCluster].Element[iMSegment].cost = 10000; //MAX COST!!!

			scoreMatchMatrix.Element[iSCluster].Element[iMSegment].idx = -1;
		}
	}

	//used in Match(RECOG::PSGM_::ModelInstance *pSModelInstance, int startIdx, int endIdx);
	nTc = new float[3 * convexTemplate.n];
	dISMc = new float[convexTemplate.n];

#ifdef RVLPSGM_MATCHCTI_MATCH_MATRIX
	// maxnSClusterCTIs <- max no. of CTIs per scene cluster

	int maxnSClusterCTIs = 0;

	int nSClusterCTIs;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		nSClusterCTIs = CTISet.SegmentCTIs.Element[iSCluster].n;

		if (nSClusterCTIs > maxnSClusterCTIs)
			maxnSClusterCTIs = nSClusterCTIs;
	}

	// CTIInterval <- interval of CTI indices created from a particular model in MCTISet
	// maxnModelCTIs <- max no. of CTIs per model

	Pair<int, int> *CTIInterval = new Pair<int, int>[MCTISet.nModels];

	int maxnModelCTIs = 0;

	int iMCluster = 0;

	int iModel, iMCTI;
	int nModelCTIs;
	int nMClusterCTIs;

	for (iModel = 0; iModel < MCTISet.nModels; iModel++)
	{
		CTIInterval[iModel].a = MCTISet.SegmentCTIs.Element[iMCluster].Element[0];

		nModelCTIs = 0;

		nMClusterCTIs = MCTISet.SegmentCTIs.Element[iMCluster].n;

		while (true)
		{
			nModelCTIs += nMClusterCTIs;

			iMCluster++;

			if (iMCluster >= MCTISet.SegmentCTIs.n)
				break;

			nMClusterCTIs = MCTISet.SegmentCTIs.Element[iMCluster].n;

			if (nMClusterCTIs > 0)
			{
			iMCTI = MCTISet.SegmentCTIs.Element[iMCluster].Element[0];

			pMModelInstance = MCTISet.pCTI.Element[iMCTI];

				if (pMModelInstance->iModel != iModel)
					break;
			}
		}

		CTIInterval[iModel].b = (iMCluster < MCTISet.SegmentCTIs.n ? MCTISet.SegmentCTIs.Element[iMCluster].Element[0] : MCTISet.pCTI.n);

		if (nModelCTIs > maxnModelCTIs)
			maxnModelCTIs = nModelCTIs;
	}

	// Initialize hypothesis space.

	Space3DGrid<PSGM_::Hypothesis, float> HSpace;

	int HSpaceSize = 40;
	float HSpaceCellSize = 20.0f;
	int maxnMatches = maxnSClusterCTIs * maxnModelCTIs;

	HSpace.Create(HSpaceSize, HSpaceSize, HSpaceSize, HSpaceCellSize, maxnMatches);

	float volumeCorner[3];

	float halfVolumeSize = 0.5f * HSpaceCellSize * (float)HSpaceSize;

	RVLSET3VECTOR(volumeCorner, halfVolumeSize, halfVolumeSize, halfVolumeSize);

	Array<int> iMergingCandidates;

	iMergingCandidates.Element = new int[maxnMatches];

	//// Initialize match matrix.

	//RVL_DELETE_ARRAY(matchMatrixMem);

	//matchMatrixMem = new int[CTISet.pCTI.n * MCTISet.pCTI.n];

	//RVL_DELETE_ARRAY(matchMatrix.Element);

	//matchMatrix.Element = new Array<int>[nClusters * MCTISet.nModels];

	// Allocate memory for sceneSegmentMatches.

	RVL_DELETE_ARRAY(sceneSegmentMatches.Element);
	
	sceneSegmentMatches.Element = new Array<SortIndex<float>>[nClusters];

	sceneSegmentMatches.n = nClusters;

	int nMatches = CTISet.pCTI.n * MCTISet.pCTI.n;

	if (nMatches > sceneSegmentMatchesArray.n)
	{
		RVL_DELETE_ARRAY(sceneSegmentMatchesArray.Element);

		sceneSegmentMatchesArray.n = nMatches;

		sceneSegmentMatchesArray.Element = new SortIndex<float>[sceneSegmentMatchesArray.n];
	}

	RVL_DELETE_ARRAY(bestSceneSegmentMatches.Element);

	bestSceneSegmentMatches.Element = new Array<SortIndex<float>>[nClusters];

	bestSceneSegmentMatches.n = nClusters;

	int nBestMatchesPerCluster = 100;

	int nBestMatchesTotal = nBestMatchesPerCluster * nClusters;

	if (nBestMatchesTotal > bestSceneSegmentMatchesArray.n)
	{
		RVL_DELETE_ARRAY(bestSceneSegmentMatchesArray.Element);

		bestSceneSegmentMatchesArray.n = nBestMatchesTotal;

		bestSceneSegmentMatchesArray.Element = new SortIndex<float>[bestSceneSegmentMatchesArray.n];
	}

	// main loop

	PSGM_::MatchInstance **ppFirstMatch;
	float centroid[3];
	PSGM_::Cluster *pSCluster;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
		//iSCluster = 5;		// Only for debugging purpose!!!
	{
		printf("%d/%d\n", iSCluster + 1, nClusters);

		sceneSegmentMatches.Element[iSCluster].n = 0;

		sceneSegmentMatches.Element[iSCluster].Element = sceneSegmentMatchesArray.Element + matchID;

		nCTI = CTISet.SegmentCTIs.Element[iSCluster].n;

		pSCluster = clusters.Element[iSCluster];

		pSurfels->Centroid(pSCluster->iSurfelArray, centroid);

		RVLSCALE3VECTOR(centroid, 1000.0f, centroid);

		HSpace.SetVolume(centroid[0] - volumeCorner[0], centroid[1] - volumeCorner[1], centroid[2] - volumeCorner[2]);

		for (iModel = 0; iModel < MCTISet.nModels; iModel++)
		{
			ppFirstMatch = pCTImatches->ppNext;

			for (iSCTI = 0; iSCTI < nCTI; iSCTI++)
			{
				CTIIdx = CTISet.SegmentCTIs.Element[iSCluster].Element[iSCTI];

				pSModelInstance = CTISet.pCTI.Element[CTIIdx];

				Match(pSModelInstance, CTIInterval[iModel].a, CTIInterval[iModel].b);

				CalculateScore(RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SATURATED_SQUARE_DISTANCE, CTIInterval[iModel].a, CTIInterval[iModel].b);


			} // for all MI in cluster

			AddSegmentMatches(iSCluster, ppFirstMatch, HSpace, iMergingCandidates);
		}

		//// Only for debugging purpose!!!

		//FILE *fp = fopen("tmp.txt", "w");

		//for (int i = 0; i < sceneSegmentMatches.Element[iSCluster].n; i++)
		//	fprintf(fp, "%d\t%f\n", sceneSegmentMatches.Element[iSCluster].Element[i].idx, sceneSegmentMatches.Element[iSCluster].Element[i].cost);

		//fclose(fp);

		////

		bestSceneSegmentMatches.Element[iSCluster].Element = bestSceneSegmentMatchesArray.Element + nBestMatchesPerCluster * iSCluster;

		Min<SortIndex<float>, float>(sceneSegmentMatches.Element[iSCluster], nBestMatchesPerCluster, bestSceneSegmentMatches.Element[iSCluster]);

		BubbleSort<SortIndex<float>>(bestSceneSegmentMatches.Element[iSCluster]);
	}

	delete[] CTIInterval;
	delete[] iMergingCandidates.Element;

#else
	int startIdx = 0, endIdx = MCTISet.pCTI.n;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	//iSCluster = 5;		// Only for debugging purpose!!!
	{
		printf("%d/%d", iSCluster + 1, nClusters);
	
		nCTI = CTISet.SegmentCTIs.Element[iSCluster].n;

		for (iSCTI = 0; iSCTI < nCTI; iSCTI++)
		{
			CTIIdx = CTISet.SegmentCTIs.Element[iSCluster].Element[iSCTI];

			pSModelInstance = CTISet.pCTI.Element[CTIIdx];	

			Match(pSModelInstance, startIdx, endIdx);

			//CalculateScore(RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SATURATED_SQUARE_DISTANCE);
			CalculateScore(RVLPSGM_MATCH_SIMILARITY_MEASURE_SATURATED_SQUARE_DISTANCE_INVISIBILITY_PENAL);			

			UpdateScoreMatchMatrix(pSModelInstance);

		} // for all MI in cluster
		
		if (nClusters < 10)
			printf("\b");
		else
			printf("\b\b");

		if (iSCluster < 9)
		{
			printf("\b\b");
		}
		else
			printf("\b\b\b");

	}	// for all dominant clusters
#endif

	pCTImatchesArray.n = CTISet.pCTI.n * MCTISet.pCTI.n;

	RVL_DELETE_ARRAY(pCTImatchesArray.Element);

	pCTImatchesArray.Element = new RECOG::PSGM_::MatchInstance*[pCTImatchesArray.n];

	QLIST::CreatePtrArray<RECOG::PSGM_::MatchInstance>(pCTImatches, &pCTImatchesArray);

#ifndef RVLPSGM_MATCHCTI_MATCH_MATRIX
	SortScoreMatchMatrix();
#endif

	////for Visualization purposes:
	//RECOG::PSGM_::MatchInstance *pMatchx = pCTImatchesArray.Element[scoreMatchMatrix.Element[0].Element[0].idx];
	//VisualizeCTIMatchidx(pMatchx->iSCTI, pMatchx->iMCTI);
	////end of visualization

	//Transparency check
	////Transparency check

	//FilterHypothesesUsingTransparency(0.5, 0.01, true);

	printf("completed.\n");

	//int nSMI = iMIS;

	delete[] nTc;
	delete[] dISMc;

	iScene++;

#ifdef RVLPSGM_SAVE_MATCHES
	printf("Saving matches to txt file...");
	SaveMatches();
	printf("completed!\n\n");
#endif

	// Match TGs.

	//MatchTGs();
}

void PSGM::Match(
	RECOG::PSGM_::ModelInstance *pSModelInstance,
	int startIdx,
	int endIdx)
{
	float cos45 = cos(PI / 4);
	float csMinSampleAngleDiff = cos(PI / 4);
	float minE, minETotal, E, score, SMI_minETotal;

	float SMI_tBestMatch[3];

	Eigen::Matrix3f A;
	Eigen::Vector3f B, t;

	float sigma = 8; //!!!!
	float sigma25 = 2.5*2.5;

	int i, idx;

	float dISv, dIMvt;

	RECOG::PSGM_::ModelInstance *pMModelInstance;

	int iMCTI;

	bool TP_;

	float distanceThresh = 50;

	float R_[9], t_[3];

	int nSamples;

	for (iMCTI = startIdx; iMCTI < endIdx; iMCTI++)
	{
		pMModelInstance = MCTISet.pCTI.Element[iMCTI];

		//minE = 66.0;
		minE = 412.5; // for sigma = 2.5^2

		//CTDMatchRANSAC
		float pPrior = 2.5 * stdNoise;

		int nValidSampleCandidates = 0;

		//find valid sample candidates
		iValidSampleCandidate.n = 0;

		QLIST::Index *piValidSampleCandidate = iValidSampleCandidate.Element;

		QLIST::Index *pISampleCandidate = pISampleCandidateList->pFirst;

		while (pISampleCandidate)
		{
			if (pSModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true && pMModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true)
			{
				piValidSampleCandidate->Idx = pISampleCandidate->Idx;

				piValidSampleCandidate->pNext = piValidSampleCandidate + 1;

				piValidSampleCandidate++;

				nValidSampleCandidates++;
			}

			pISampleCandidate = pISampleCandidate->pNext;
		}

		iValidSampleCandidate.n = nValidSampleCandidates;

		if (nValidSampleCandidates > 2)
		{
			int nValids = 0;

			//find iValid
			iValid.n = 0;

			QLIST::Index *piValid = iValid.Element;
			int iPlane;

			for (iPlane = 0; iPlane < convexTemplate.n; iPlane++)
			{
				if (pSModelInstance->modelInstance.Element[iPlane].valid == true && pMModelInstance->modelInstance.Element[iPlane].valid == true)
				{
					piValid->Idx = iPlane;

					piValid->pNext = piValid + 1;

					piValid++;

					nValids++;
				}
			}

			iValid.n = nValids;

			int iRansac;
			bool bValidSample;

			int iSample[2];
			int ID[2];
			float V[3];
			float N_[3] = { 0, 0, 1 };
			float N__[3] = { 0, -cos45, cos45 };
			float fTmp;
			int nValidSamplesSearch;
			int nRansacCandidates = 0;

			iConsensus.n = 0;

			QLIST::Index *piConsensus = iConsensus.Element;

			if (bMatchRANSAC)
			{
				iRansacCandidates.n = 0;

				QLIST::Index *piRansacCandidates = iRansacCandidates.Element;

				//find RANSAC candidates
				RVLCROSSPRODUCT3(N_, N__, V);

				fTmp = sqrt(RVLDOTPRODUCT3(V, V));

				RVLSCALE3VECTOR2(V, fTmp, V);

				for (i = 0; i < iValidSampleCandidate.n; i++)
				{
					ID[0] = iValidSampleCandidate.Element[i].Idx;

					fTmp = RVLDOTPRODUCT3(V, convexTemplate.Element[ID[0]].N);

					if (RVLABS(fTmp) >= csMinSampleAngleDiff)
					{
						piRansacCandidates->Idx = ID[0];

						piRansacCandidates++;

						nRansacCandidates++;
					}
				}

				iRansacCandidates.n = nRansacCandidates;

				std::random_device rd;
				std::mt19937 eng(rd());
				std::uniform_int_distribution<> distribution(0, nRansacCandidates);

				nSamples = RVLMIN(13, nRansacCandidates);

				for (iRansac = 0; iRansac < nSamples; iRansac++)
				{
					ID[0] = 1; //second plane from convexTemplate

					iSample[1] = distribution(eng);

					ID[1] = iValidSampleCandidate.Element[iSample[1]].Idx;

					float dM[3];
					float dS[3];
					float N[9];

					dM[0] = pMModelInstance->modelInstance.Element[0].d;
					dM[1] = pMModelInstance->modelInstance.Element[ID[0]].d;
					dM[2] = pMModelInstance->modelInstance.Element[ID[1]].d;

					dS[0] = pSModelInstance->modelInstance.Element[0].d * 1000;
					dS[1] = pSModelInstance->modelInstance.Element[ID[0]].d * 1000;
					dS[2] = pSModelInstance->modelInstance.Element[ID[1]].d * 1000;

					RVLCOPYTOCOL3(convexTemplate.Element[0].N, 0, N);
					RVLCOPYTOCOL3(convexTemplate.Element[ID[0]].N, 1, N);
					RVLCOPYTOCOL3(convexTemplate.Element[ID[1]].N, 2, N);

					A << N[0], N[3], N[6], N[1], N[4], N[7], N[2], N[5], N[8]; // N'
					B << dS[0] - dM[0], dS[1] - dM[1], dS[2] - dM[2];
					t = A.colPivHouseholderQr().solve(B);

					iConsensusTemp.n = 0;

					QLIST::Index *piConsensusTemp = iConsensusTemp.Element;

					E = 0;

					for (i = 0; i < iValid.n; i++)
					{
						idx = iValid.Element[i].Idx;

						dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

						dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

						//fTmp = (dISv - dIMvt) / pPrior;
						fTmp = (dISv - dIMvt) / sigma;

#ifdef RVLPSGM_MATCH_SATURATION
						if (fTmp*fTmp < sigma25)
							//if (fTmp*fTmp < 1)
						{
							E += fTmp*fTmp;

							piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
							piConsensusTemp->pNext = piConsensusTemp + 1;

							iConsensusTemp.n++;

							piConsensusTemp++;
						}
						else
							E += sigma25;
						//E += 1;
#else
						E += fTmp*fTmp;

						//TREBA LI OVO?
						piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
						piConsensusTemp->pNext = piConsensusTemp + 1;

						iConsensusTemp.n++;

						piConsensusTemp++;
#endif
					}

					if (E < minE)
					{
						minE = E;

						iConsensus.n = iConsensusTemp.n;

						piConsensusTemp = iConsensusTemp.Element;

						piConsensus = iConsensus.Element;

						for (i = 0; i < iConsensusTemp.n; i++)
						{
							piConsensus->Idx = piConsensusTemp->Idx;
							piConsensus->pNext = piConsensus + 1;

							piConsensus++;
							piConsensusTemp++;
						}
					}
				}
			}
			else
				iConsensus.n = iValid.n;

			if (iConsensus.n >= 3)
			{
				float dISc, dIMc;

				piConsensus = iConsensus.Element;

				for (i = 0; i < iConsensus.n; i++)
				{
#ifdef RVLPSGM_RANSAC
					idx = piConsensus->Idx;
#else
					idx = iValid.Element[i].Idx;
#endif
					dISc = pSModelInstance->modelInstance.Element[idx].d * 1000;
					dIMc = pMModelInstance->modelInstance.Element[idx].d;

					dISMc[i] = dISc - dIMc;

					nTc[i] = convexTemplate.Element[idx].N[0];
					nTc[iConsensus.n + i] = convexTemplate.Element[idx].N[1];
					nTc[2 * iConsensus.n + i] = convexTemplate.Element[idx].N[2];

					piConsensus++;
				}

				int j, k;

				//nTc*nTc'
				for (i = 0; i < 3; i++)
					for (j = 0; j < 3; j++)
						if (i <= j)
						{
							A(i * 3 + j) = 0;

							for (k = 0; k < iConsensus.n; k++)
								A(i * 3 + j) += nTc[i * iConsensus.n + k] * nTc[j * iConsensus.n + k];
						}
						else
							A(i * 3 + j) = A(j * 3 + i);

				//nTc*(dISc-dIMc)
				for (i = 0; i < 3; i++)
				{
					B(i) = 0;
					for (j = 0; j < iConsensus.n; j++)
						B(i) += nTc[i * iConsensus.n + j] * dISMc[j];
				}				
				
				t = A.colPivHouseholderQr().solve(B);							

				E = 0;

				float eSum = 0;

				for (i = 0; i < iValid.n; i++)
				{
					idx = iValid.Element[i].Idx;

					dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

					dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

					e.Element[iMCTI].Element[idx] = dISv - dIMvt;

					eSum += e.Element[iMCTI].Element[idx];
				}

				RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pCTIMatch);
				RVLQLIST_ADD_ENTRY(pCTImatches, pCTIMatch);

				if (iMCTI == startIdx)
					pFirstSCTIMatch = pCTIMatch;

				for (i = 0; i < 3; i++)
				{
					tBestMatch.Element[iMCTI].Element[i] = t(i);
					pCTIMatch->tMatch[i] = t(i);
				}				

				pCTIMatch->ID = matchID++;
				pCTIMatch->iScene = iScene;

				pCTIMatch->iSCTI = CTIIdx;
				pCTIMatch->iMCTI = iMCTI;

				pCTIMatch->E = eSum;
				pCTIMatch->nValids = iValid.n;

				pCTIMatch->probability1 = NAN;
				pCTIMatch->probability2 = NAN;

				pCTIMatch->angleGT = NAN;
				pCTIMatch->distanceGT = NAN;

				MSTransformation(pMModelInstance, pSModelInstance, tBestMatch.Element[iMCTI].Element, pCTIMatch->R, pCTIMatch->t);
			}
			else
			{
				t << 0, 0, 0;
				//E = 66;
				E = 412.5;
			}
		}
	}	//for all model MI
}

void PSGM::MatchTGs()
{
	// Parameters.

	int nBestMatches = 10;

#ifdef RVLPSGM_MATCHTGS_CREATE_SCENE_TG
	// Initialize memory storage.

	CRVLMem mem;

	mem.Create(10000000);

	STGSet.pMem = &mem;

	// Create vertex graph.

	VertexGraph *pVertexGraph = new VertexGraph;

	pVertexGraph->pMem = &mem;

	pVertexGraph->idx = 0;

	STGSet.vertexGraphs.push_back(pVertexGraph);

	pVertexGraph->Create(pSurfels);
#else
#ifdef RVLPSGM_MATCHTGS_CREATE_SCENE_VG
	// Create vertex graph.

	VertexGraph *pVertexGraph = new VertexGraph;

	CRVLMem mem;

	mem.Create(10000000);

	pVertexGraph->pMem = &mem;

	pVertexGraph->idx = 0;

	pVertexGraph->Create(pSurfels);

	FILE *fpSVG = fopen("sceneVG.vgr", "w");

	pVertexGraph->Save(fpSVG);

	fclose(fpSVG);

	delete pVertexGraph;

	mem.Clear();
#endif
#endif

#ifdef RVLTG_MATCH_DEBUG
	FILE *fpDebug = fopen("TG_match_error.txt", "w");

	fclose(fpDebug);
#endif

	/// Compute Matching scores for the first nBestMatches best matches for every scene segment.

	Array<int> iVertexArray;

	iVertexArray.Element = new int[pSurfels->vertexArray.n];

	bool *bAlreadyInArray = new bool[pSurfels->vertexArray.n];

	memset(bAlreadyInArray, 0, pSurfels->vertexArray.n * sizeof(bool));

	int iSS, iSS_, i, j, iVertex;
	Box<float> MBoundingBox, SBoundingBox, boundingBoxIntersection;
	float RMS[9], tMS[3];
	float RMS_[9], tMS_[3];
	float RSM[9], tSM[3];
	PSGM_::Cluster *pSSegment;
	float boundingBoxOverlap;
	int nMatches;

	for (iSS = 0; iSS < scoreMatchMatrix.n; iSS++)
	//iSS = 5;
	{
#ifdef RVLTG_MATCH_DEBUG
		// Write matches to file.

		FILE *fp = fopen("TG_match_error.txt", "a");

		fprintf(fp, "%d\t%d\t0\t0\n", iSS, nBestMatches);

		fclose(fp);
#endif
		for (i = 0; i < nBestMatches; i++)
		{
			// Get match.

			int iMatch = scoreMatchMatrix.Element[iSS].Element[i].idx;

			if (iMatch < 0)
			{
#ifdef RVLTG_MATCH_DEBUG
				FILE *fp = fopen("TG_match_error.txt", "a");

				fprintf(fp, "-1\t0\t0\t%f\n", 0.0f);

				int i;

				for (i = 0; i < 3; i++)
					fprintf(fp, "%f\t%f\t%f\t%f\n", 0.0f, 0.0f, 0.0f, 0.0f);

				fclose(fp);
#endif

				continue;
			}

			RECOG::PSGM_::MatchInstance *pMatch = pCTImatchesArray.Element[iMatch];
			int iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
			int iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;

			RECOG::PSGM_::ModelInstance *pSCTI = CTISet.pCTI.Element[iSCTI];
			RECOG::PSGM_::ModelInstance *pMCTI = MCTISet.pCTI.Element[iMCTI];

			MSTransformation(pMCTI, pSCTI, pMatch->tMatch, RMS, tMS);

			RVLINVTRANSF3D(RMS, tMS, RSM, tSM);

			// Get model TG.

			RECOG::TG *pMTG = MTGSet.GetTG(pMCTI->iModel);

			if (pMTG)	// If there is a TG in MTGSet which corresponds to the model CTI
			{
				// Get vertex graph.

				VertexGraph *pVG = MTGSet.GetVertexGraph(pMTG);

				if (pVG)
				{
					// Determine model bounding box.

					if (pVG->BoundingBox(&MBoundingBox))
					{
						// Expand boundingBox.

						//float boundingBoxExtension = 20.0f;

						//boundingBox.minx -= boundingBoxExtension;
						//boundingBox.maxx += boundingBoxExtension;
						//boundingBox.miny -= boundingBoxExtension;
						//boundingBox.maxy += boundingBoxExtension;
						//boundingBox.minz -= boundingBoxExtension;
						//boundingBox.maxz += boundingBoxExtension;

						// iVertexArray <- scene vertices within boundingBox.

						//iVertexArray.n = 0;

						//SURFEL::Vertex *pVertex;
						//float PS[3], PM[3];

						//for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++)
						//{
						//	pVertex = pSurfels->vertexArray.Element[iVertex];

						//	RVLSCALE3VECTOR(pVertex->P, 1000.0f, PS);

						//	RVLTRANSF3(PS, RSM, tSM, PM);

						//	if (InBoundingBox<float>(&boundingBox, PM))
						//		iVertexArray.Element[iVertexArray.n++] = iVertex;
						//}

						// iVertexArray <- vertices of the clusters which overlap significantly with MBoundingBox

						iVertexArray.n = 0;

						for (iSS_ = 0; iSS_ < clusters.n; iSS_++)
						{
							pSSegment = clusters.Element[iSS_];

							if (pSurfels->BoundingBox(pSSegment->iVertexArray, RSM, tSM, 1000.0f, SBoundingBox))
							{
								if (BoxIntersection<float>(&MBoundingBox, &SBoundingBox, &boundingBoxIntersection))
								{
									boundingBoxOverlap = BoxVolume<float>(&boundingBoxIntersection) / BoxVolume<float>(&SBoundingBox);

									if (boundingBoxOverlap > 0.5f)
									{
										for (j = 0; j < pSSegment->iVertexArray.n; j++)
										{
											iVertex = pSSegment->iVertexArray.Element[j];

											if (!bAlreadyInArray[iVertex])
											{
												iVertexArray.Element[iVertexArray.n++] = iVertex;

												bAlreadyInArray[iVertex] = true;
											}
										}
									}
								}
							}
						}

						for (j = 0; j < iVertexArray.n; j++)
							bAlreadyInArray[iVertexArray.Element[j]] = false;
						
#ifdef RVLPSGM_MATCHTGS_CREATE_SCENE_TG
						// Create TG from the vertices in MBoundingBox.

						RECOG::TG *pSTG = new RECOG::TG;

						pSTG->A = STGSet.A;

						pSTG->iVertexGraph = pVertexGraph->idx;

						pSTG->iObject = pMTG->iObject;

						pSTG->Create(pVertexGraph, iVertexArray, RMS, tMS, &STGSet, pSurfels);

						STGSet.TGs.push_back(pSTG);
#endif

						// Match pMTG to vertices in iVertexArray.

						float score;
						Array<RECOG::TGCorrespondence> correspondences;

						pMTG->Match(pSurfels, iVertexArray, 1000.0f, &MTGSet, RMS, tMS, true, score, correspondences, RMS_, tMS_);

						// Free memory.

						RVL_DELETE_ARRAY(correspondences.Element);
					}	// If there is at least one point in the vertex graph
				}	// If pMTG has a vertex graph
			}	// If there is a TG in MTGSet which corresponds to the model CTI
		}	// for the first nBestMatches
	}	// for every scene segment

	delete[] iVertexArray.Element;
	delete[] bAlreadyInArray;

#ifdef RVLPSGM_MATCHTGS_CREATE_SCENE_TG
	STGSet.Save("sceneTG.tgr");

	STGSet.Clear();

	mem.Clear();
#endif
}

void PSGM::AddSegmentMatches(
	int iCluster,
	PSGM_::MatchInstance **ppFirstMatch,
	Space3DGrid<PSGM_::Hypothesis, float> &HSpace,
	Array<int> &iMergingCandidates)
{
	//float eqThr = 0.94;		// cos(20 deg)
	//float eqThr = COS45;
	float eqThr = 0.0f;

	PSGM_::MatchInstance *pMatch = *ppFirstMatch;

	PSGM_::Hypothesis Hypothesis;
	PSGM_::Hypothesis *pHypothesis_;
	Array<PSGM_::Hypothesis *> neighborArray;
	int i;
	float e;
	float *X, *Z, *X_, *Z_;

	while (pMatch)
	{
		//if (pMatch->ID == 12134)
		//	int debug = 0;

		X = pMatch->R;
		Z = pMatch->R + 6;

		HSpace.Neighbors(pMatch->t, neighborArray);

		iMergingCandidates.n = 0;

		for (i = 0; i < neighborArray.n; i++)
		{
			pHypothesis_ = neighborArray.Element[i];

			Z_ = pHypothesis_->R + 6;

			e = RVLDOTPRODUCT3(Z, Z_);

			if (e < eqThr)
				continue;

			X_ = pHypothesis_->R;

			e = RVLDOTPRODUCT3(X, X_);

			if (e < eqThr)
				continue;

			if (pMatch->score <= pHypothesis_->score)
				iMergingCandidates.Element[iMergingCandidates.n++] = i;
			else
				break;
		}

		if (i >= neighborArray.n)	// If there are no better hypotheses in the neighborhood of pMatch
		{
			Hypothesis.iMatch = pMatch->ID;
			RVLCOPY3VECTOR(pMatch->t, Hypothesis.P);
			RVLCOPYMX3X3(pMatch->R, Hypothesis.R);
			Hypothesis.score = pMatch->score;

			HSpace.AddData(Hypothesis);

			for (i = 0; i < iMergingCandidates.n; i++)
			{
				pHypothesis_ = neighborArray.Element[iMergingCandidates.Element[i]];

				HSpace.RemoveData(pHypothesis_);
			}
		}

		pMatch = pMatch->pNext;
	}

	Array<PSGM_::Hypothesis *> hypothesisArray;

	HSpace.GetData(hypothesisArray);

	SortIndex<float> *pMatchIdx = sceneSegmentMatches.Element[iCluster].Element + sceneSegmentMatches.Element[iCluster].n;

	PSGM_::Hypothesis **ppHypothesis = hypothesisArray.Element;

	for (i = 0; i < hypothesisArray.n; i++, pMatchIdx++, ppHypothesis++)
	{
		pHypothesis_ = *ppHypothesis;

		//if (pHypothesis_->iMatch == 174)
		//	int debug = 0;

		pMatchIdx->idx = pHypothesis_->iMatch;
		pMatchIdx->cost = pHypothesis_->score;
	}

	sceneSegmentMatches.Element[iCluster].n += hypothesisArray.n;

	HSpace.Clear();

#ifdef RVLPSGM_MATCH_HYPOTHESIS_LOG
	FILE *fp = fopen("hypotheses.txt", "w");

	for (i = 0; i < hypothesisArray.n; i++, pMatchIdx++, ppHypothesis++)
	{
		pHypothesis_ = hypothesisArray.Element[i];

		fprintf(fp, "%d\t", pHypothesis_->iMatch);

		for (int j = 0; j < 9; j++)
			fprintf(fp, "%f\t", pHypothesis_->R[j]);

		fprintf(fp, "%f\t%f\t%f\n", pHypothesis_->P[0], pHypothesis_->P[1], pHypothesis_->P[2]);
	}

	fclose(fp);
#endif
}

void PSGM::CalculateScore(
	int similarityMeasure,
	int iFirstCTI,
	int iEndCTI)
{
	float sigma = 8.0;

	float sigma25 = 2.5 * 2.5;

	int iMCTI, iValidPlane, idx;

	int nMCTI = MCTISet.pCTI.n;

	int iEndCTI_ = (iEndCTI >= 0 ? iEndCTI : nMCTI);

	float fTmp, eTmp, scoreTmp;

	float maxError;

	int medianIdx;

	RECOG::PSGM_::MatchInstance *pCTIMatch_ = pFirstSCTIMatch;

	switch (similarityMeasure)
	{
	case RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SQUARE_DISTANCE: //mean square error

		for (iMCTI = iFirstCTI; iMCTI < iEndCTI_; iMCTI++)
		{
			scoreTmp = 0;

			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				scoreTmp += eTmp * eTmp;
			}

			score.Element[iMCTI] = sqrt(scoreTmp / iValid.n);

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		break;
	case RVLPSGM_MATCH_SIMILARITY_MEASURE_MAX_ABS_DISTANCE: //maximum absolute error

		for (iMCTI = iFirstCTI; iMCTI < iEndCTI_; iMCTI++)
		{
			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				fTmp = RVLABS(eTmp);

				if (iValidPlane == 0)
					maxError = fTmp;
				else
					if (fTmp > maxError)
						maxError = fTmp;
			}

			score.Element[iMCTI] = maxError;

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		break;
	case RVLPSGM_MATCH_SIMILARITY_MEASURE_SATURATED_SQUARE_DISTANCE_INVISIBILITY_PENAL: //saturated square error

		for (iMCTI = iFirstCTI; iMCTI < iEndCTI_; iMCTI++)
		{
			scoreTmp = 0;

			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				fTmp = eTmp / sigma;

				if (fTmp * fTmp < sigma25)
				{
					scoreTmp += fTmp * fTmp;
				}
				else
					scoreTmp += sigma25;
			}

			score.Element[iMCTI] = scoreTmp + sigma25 * (66 - iValid.n);

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		break;
	case RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SATURATED_SQUARE_DISTANCE:
		for (iMCTI = iFirstCTI; iMCTI < iEndCTI_; iMCTI++)
		{
			scoreTmp = 0;

			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				fTmp = eTmp / sigma;

				if (fTmp * fTmp < sigma25)
				{
					scoreTmp += fTmp * fTmp;
				}
				else
					scoreTmp += sigma25;
			}

			score.Element[iMCTI] = scoreTmp / iValid.n;

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		break;
	default: //median of absolute error

		Array<SortIndex<float>> validErrors;

		validErrors.Element = new SortIndex<float>[iValid.n];
		validErrors.n = iValid.n;

		medianIdx = iValid.n / 2;

		for (iMCTI = iFirstCTI; iMCTI < iEndCTI_; iMCTI++)
		{
			for (iValidPlane = 0; iValidPlane < iValid.n; iValidPlane++)
			{
				idx = iValid.Element[iValidPlane].Idx;

				eTmp = e.Element[iMCTI].Element[idx];

				fTmp = RVLABS(eTmp);

				validErrors.Element[iValidPlane].cost = fTmp;
				validErrors.Element[iValidPlane].idx = iValidPlane;
			}

			BubbleSort(validErrors);

			if (iValid.n % 2 == 0)
				scoreTmp = (validErrors.Element[medianIdx - 1].cost + validErrors.Element[medianIdx].cost) / 2;
			else
				scoreTmp = validErrors.Element[medianIdx].cost;
			
			score.Element[iMCTI] = scoreTmp;

			pCTIMatch_->score = score.Element[iMCTI];
			pCTIMatch_ = pCTIMatch_->pNext;
		}

		RVL_DELETE_ARRAY(validErrors.Element);

		break;
	}
}

void PSGM::UpdateScoreMatchMatrix(RECOG::PSGM_::ModelInstance *pSModelInstance)
{
	int SSegmentIdx = pSModelInstance->iCluster;

	int MSegmentIdx, iMCTI;

	float scoreTmp, scoreTmp_;

	int idx;

	RECOG::PSGM_::MatchInstance *pCTIMatch_ = pFirstSCTIMatch;

	for (iMCTI = 0; iMCTI < MCTISet.pCTI.n; iMCTI++)
	{
		scoreTmp = score.Element[iMCTI];

		MSegmentIdx = MCTISet.pCTI.Element[iMCTI]->iModel * (MCTISet.maxSegmentIdx + 1) + MCTISet.pCTI.Element[iMCTI]->iCluster;

		scoreTmp_ = scoreMatchMatrix.Element[SSegmentIdx].Element[MSegmentIdx].cost;

		idx = scoreMatchMatrix.Element[SSegmentIdx].Element[MSegmentIdx].idx;

		if (scoreTmp < scoreTmp_ || idx == -1)
		{
			scoreMatchMatrix.Element[SSegmentIdx].Element[MSegmentIdx].cost = scoreTmp;

			scoreMatchMatrix.Element[SSegmentIdx].Element[MSegmentIdx].idx = pCTIMatch_->ID;
		}

		pCTIMatch_ = pCTIMatch_->pNext;
	}
}

void PSGM::SortScoreMatchMatrix(bool descending)
{
	int nSSegments = scoreMatchMatrix.n;

	int iSSegment;

	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
	{
		BubbleSort<SortIndex<float>>(scoreMatchMatrix.Element[iSSegment], descending);
	}
}

void PSGM::CreateScoreMatchMatrixICP()
{
	int nClusters = CTISet.maxSegmentIdx + 1;

	//int maxMSegments = (MCTISet.nModels + 1) * (MCTISet.maxSegmentIdx + 1);
	int maxMSegments = (MCTISet.nModels) * (MCTISet.maxSegmentIdx + 1);

	int iSCluster, iMSegment;

	//delete scoreMatchMatrix	
	for (iSCluster = 0; iSCluster < scoreMatchMatrixICP.n; iSCluster++)
		RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element[iSCluster].Element);

	RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element);

	scoreMatchMatrixICP.Element = new Array<SortIndex<float>>[nClusters];
	scoreMatchMatrixICP.n = nClusters;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		scoreMatchMatrixICP.Element[iSCluster].Element = new SortIndex<float>[maxMSegments];
		scoreMatchMatrixICP.Element[iSCluster].n = maxMSegments;

		for (iMSegment = 0; iMSegment < maxMSegments; iMSegment++)
		{
			scoreMatchMatrixICP.Element[iSCluster].Element[iMSegment].cost = 10000; //MAX COST!!!
			//scoreMatchMatrixICP.Element[iSCluster].Element[iMSegment].cost = -1; //MAX COST!!!

			scoreMatchMatrixICP.Element[iSCluster].Element[iMSegment].idx = -1;
		}
	}

	int iMatch;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		for (iMSegment = 0; iMSegment < nBestMatches; iMSegment++)
		{
			iMatch = scoreMatchMatrix.Element[iSCluster].Element[iMSegment].idx;

			if (iMatch != -1)
			{
				scoreMatchMatrixICP.Element[iSCluster].Element[iMSegment].cost = pCTImatchesArray.Element[iMatch]->cost_NN;

				scoreMatchMatrixICP.Element[iSCluster].Element[iMSegment].idx = iMatch;
			}
			else
				break;
		}
	}

	//sort ICP matrix
	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		BubbleSort<SortIndex<float>>(scoreMatchMatrixICP.Element[iSCluster], false); //ascending
		//BubbleSort<SortIndex<float>>(scoreMatchMatrixICP.Element[iSCluster], true); //descending
	}

}

//for multiple matches per model - tmp fnc
void PSGM::CreateScoreMatchMatrixICP_TMP()
{
	int iSCluster;
	int nClusters = CTISet.maxSegmentIdx + 1;

	//delete scoreMatchMatrix	
	for (iSCluster = 0; iSCluster < scoreMatchMatrixICP.n; iSCluster++)
		RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element[iSCluster].Element);

	RVL_DELETE_ARRAY(scoreMatchMatrixICP.Element);

	scoreMatchMatrixICP.Element = new Array<SortIndex<float>>[nClusters];
	scoreMatchMatrixICP.n = nClusters;

	int iMatch, iBestMatch;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		scoreMatchMatrixICP.Element[iSCluster].Element = new SortIndex<float>[nBestMatches];
		scoreMatchMatrixICP.Element[iSCluster].n = nBestMatches;

		for (iBestMatch = 0; iBestMatch < nBestMatches; iBestMatch++)
		{
			iMatch = bestSceneSegmentMatches.Element[iSCluster].Element[iBestMatch].idx;

			if (iMatch != -1)
			{
				scoreMatchMatrixICP.Element[iSCluster].Element[iBestMatch].cost = pCTImatchesArray.Element[iMatch]->cost_NN;

				scoreMatchMatrixICP.Element[iSCluster].Element[iBestMatch].idx = iMatch;
			}
			else
				break;
		}
	}

	//sort ICP matrix
	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		BubbleSort<SortIndex<float>>(scoreMatchMatrixICP.Element[iSCluster], false); //ascending
		//BubbleSort<SortIndex<float>>(scoreMatchMatrixICP.Element[iSCluster], true); //descending
	}

}

void PSGM::ComputeClusterNormalDistribution(
	RECOG::PSGM_::Cluster *pCluster)
{
	float R[9];

	float *X = R;
	float *Y = R + 3;
	float *Z = R + 6;

	float *meanN = Z;

	RVLNULL3VECTOR(meanN);

	float wTotal = 0.0f;

	int iiSurfel, iSurfel;
	Surfel *pSurfel;
	float *N;
	float wN[3];
	float w;
	float fTmp;
	float NP[3];
	float eig[2];
	int i1, i2, i3;

	for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
	{
		iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		N = pSurfel->N;

		w = (float)(pSurfel->size);

		RVLSCALE3VECTOR(N, w, wN);

		RVLSUM3VECTORS(meanN, wN, meanN);

		wTotal += w;
	}

	RVLSCALE3VECTOR2(meanN, wTotal, meanN);

	// Define projection reference frame.

	RVLORTHOGONAL3(Z, X, i1, i2, i3, fTmp);

	RVLCROSSPRODUCT3(Z, X, Y);

	// Project surfel normals onto the xy-plane of the projection reference frame and compute covariance matrix.

	float C[4];

	C[0] = C[1] = C[3] = 0.0f;

	for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
	{
		iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		N = pSurfel->N;

		w = (float)(pSurfel->size);

		RVLMULMX3X3VECT(R, N, NP);

		C[0] += (w * NP[0] * NP[0]);
		C[1] += (w * NP[0] * NP[1]);
		C[3] += (w * NP[1] * NP[1]);
	}

	C[0] /= wTotal;
	C[1] /= wTotal;
	C[2] = C[1];
	C[3] /= wTotal;

	// Compute eigenvalues of C.

	Eig2<float>(C, eig);

	// Compute normalDistributionStds.

	pCluster->normalDistributionStd1 = sqrt(eig[0]);
	pCluster->normalDistributionStd2 = sqrt(eig[1]);
	RVLCOPY3VECTOR(meanN, pCluster->N);
}

void PSGM::ComputeClusterBoundaryDiscontinuityPerc(int iCluster)
{
	RECOG::PSGM_::Cluster *pCluster = clusterMem + iCluster;

	int nContinuity = 0;
	int nDiscontinuity = 0;

	int iCluster_, iiSurfel, iSurfel, iSurfel_, iPt, iPt_, iBoundary, iPointEdge;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_;
	Point *pPt;
	MeshEdge *pEdge;
	RECOG::PSGM_::Cluster *pCluster_;

	for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
	{
		iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				pPt = pMesh->NodeArray.Element + iPt;

				if (pPt->bBoundary)
					nDiscontinuity++;
				else
				{
					pEdgePtr_ = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

					while (pEdgePtr_)
					{
						RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr_, pEdge, iPt_);

						iSurfel_ = pSurfels->surfelMap[iPt_];

						if (iSurfel_ >= 0 && iSurfel_ < pSurfels->NodeArray.n)
						{
							iCluster_ = clusterMap[iSurfel_];

							if (iCluster_ != iCluster && iCluster_ >= 0)
							{
								if (iCluster_ >= clusters.n)
									printf("iCluster_=%d\n", iCluster_);

								pCluster_ = clusterMem + iCluster_;

								if (pCluster_->bValid)
									break;
							}
						}

						pEdgePtr_ = pEdgePtr_->pNext;
					}

					if (pEdgePtr_)
						nContinuity++;
				}
			}
		}
	}

	if (nDiscontinuity == 0)
		pCluster->boundaryDiscontinuityPerc = 0;
	else
		pCluster->boundaryDiscontinuityPerc = 100 * nDiscontinuity / (nContinuity + nDiscontinuity);
}

void PSGM::WriteClusterNormalDistribution(FILE *fp)
{
	int iCluster;
	RECOG::PSGM_::Cluster *pCluster;

	for (iCluster = 0; iCluster < clusters.n; iCluster++)
	{
		pCluster = clusters.Element[iCluster];

		// Write eigenvalues to file.

		fprintf(fp, "%d\t%d\t%d\t%f\t%f\n", 
			iCluster, 
			pCluster->boundaryDiscontinuityPerc, 
			pCluster->size, 
			pCluster->normalDistributionStd1, 
			pCluster->normalDistributionStd2);
	}
}

void PSGM::MSTransformation(
	RECOG::PSGM_::ModelInstance *pMModelInstance,
	RECOG::PSGM_::ModelInstance *pSModelInstance,
	float *tBestMatch,
	float *R,
	float *t)
{
	float R_[9], t_[3];

	//RVLSCALE3VECTOR(pMModelInstance->t, 0, pMModelInstance->t);

	RVLINVTRANSF3D(pMModelInstance->R, pMModelInstance->t, R_, t_);

	RVLSUM3VECTORS(t_, tBestMatch, t_);

	RVLCOMPTRANSF3D(pSModelInstance->R, pSModelInstance->t, R_, t_, R, t);
}

//Compare single match to GT
bool PSGM::CompareMatchToGT(
	RECOG::PSGM_::MatchInstance *pMatch,
	//float score,
	//float scoreThresh,
	bool poseCheck,
	float angleThresh,
	float distanceThresh)
{
	RVL::GTInstance *pGT;

	int iGTS, iGTM, nGTModels;

	float R[9], R_[9], RGT[9], tGT[3], t[3];

	float V[3], theta, distance;

	iGTS = pMatch->iScene;

	int iMCTI, iMatchedModel;

	//TP
	pGT = pECCVGT->GT.Element[iGTS].Element;

	nGTModels = pECCVGT->GT.Element[iGTS].n;

	for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
	{
		iMCTI = pMatch->iMCTI;

		iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;

		if (iMatchedModel == pGT->iModel)
		{
			if (poseCheck)
			{
				RVLSCALEMX3X3(pGT->R, 1000, RGT);

				RVLMXMUL3X3T2(pMatch->R, RGT, R);

				RVLSCALE3VECTOR(pGT->t, 1000, tGT)

#ifdef RVLPSGM_MATCH_SEGMENT_CENTROID

				RVLDIFMX3X3(RGT, pMatch->R, R_);

				RVLMULMX3X3VECT(R_, modelInstanceDB.Element[pMatch->iMMI].tc, t);

				RVLSUM3VECTORS(t, tGT, t);

				RVLDIF3VECTORS(t, pMatch->t, t);
#else if
				RVLDIF3VECTORS(pMatch->t, tGT, t);
#endif

				GetAngleAxis(R, V, theta);

				GetDistance(t, distance);

				//if ((theta < angleThresh || (theta >(PI - angleThresh) && theta < (PI + angleThresh))) && distance < distanceThresh)
				if (distance < distanceThresh)
				{
					pGT->matched = true;
					return true;								
				}
			}
			else
			{
				pGT->matched = true;
				return true;							
			}
		}
	}

	//FP
	return false;
}

bool PSGM::CompareMatchToSegmentGT(
	RECOG::PSGM_::MatchInstance *pMatch)
{	
	int iSCTI = pMatch->iSCTI;

	int iSSegment = CTISet.pCTI.Element[iSCTI]->iCluster;

	int iScene = pMatch->iScene;

	int iSegmentGT = iScene * nDominantClusters + iSSegment;

	int nGTModels, iGTM;

	RVL::GTInstance *pGT;

	int iMCTI, iMatchedModel;

	pGT = pECCVGT->GT.Element[iScene].Element;
	nGTModels = pECCVGT->GT.Element[iScene].n;

	iMCTI = pMatch->iMCTI;

	iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;

	//if (pMatch->iModel == segmentGT.Element[iSegmentGT].iModel && pMatch->iMCluster == segmentGT.Element[iSegmentGT].iMSegment)
	if (iMatchedModel == segmentGT.Element[iSegmentGT].iModel)
	{
		//set GT matched flag
		for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
		{
			if (iMatchedModel == pGT->iModel)
				pGT->matched = true;
		}

		return true;
	}
	else
		return false;
}

bool PSGM::CompareMatchToSegmentGT(
	int iScene,
	int iSSegment,
	int iMatchedModel)
{
	int iSegmentGT = iScene * nDominantClusters + iSSegment;

	int nGTModels, iGTM;

	RVL::GTInstance *pGT;

	pGT = pECCVGT->GT.Element[iScene].Element;
	nGTModels = pECCVGT->GT.Element[iScene].n;

	if (iMatchedModel == segmentGT.Element[iSegmentGT].iModel)
	{
		//set GT matched flag
		for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
		{
			if (iMatchedModel == pGT->iModel)
				pGT->matched = true;
		}

		return true;
	}
	else
		return false;
}

void PSGM::CountTPandFN(
	int &TP,
	int &FN,
	bool printMatchInfo)
{
	int iGTS, iGTM, nGTModels;
	
	TP = 0;
	FN = 0;

	RVL::GTInstance *pGT;

	int nGTSecenes = pECCVGT->GT.n;

	pGT = pECCVGT->GT.Element[iScene-1].Element; //iScene-1 because iScene is incremented in Match()

	nGTModels = pECCVGT->GT.Element[iScene-1].n; //iScene-1 because iScene is incremented in Match()

	for (iGTM = 0; iGTM < nGTModels; iGTM++)
	{
		if (!pGT->matched)
		{
			FN++;

			if (printMatchInfo)
				printf("GT Model %d (ModelID: %d) NOT matched on scene %d!\n", iGTM, pGT->iModel, iScene - 1);
		}
		else
		{
			TP++;

			if(printMatchInfo)
				printf("GT Model %d (ModelID: %d) matched on scene %d!\n", iGTM, pGT->iModel, iScene - 1);
		}

		pGT++;
	}	
}

bool PSGM::PoseCheck(
	RVL::GTInstance *pGT,
	RECOG::PSGM_::MatchInstance *pMatch,
	float distanceThresh,
	float angleThresh,
	FILE *fpLog,
	FILE *fpnotFirstPoseErr,
	bool evaluateICP)
{
	float R[9], R_[9], RGT[9], tGT[3], t[3], RICP[9], tICP[3];
	float V[3], theta, distance;
	float zGT[3], z[3];
	float thetaZ;

	RVLSCALEMX3X3(pGT->R, 1000, RGT);
	RVLSCALE3VECTOR(pGT->t, 1000, tGT);	
	
	if (evaluateICP)
	{
		//CHECK BECAUSE ICP pMatch->RICP is changed to save final pose of the object
		//Commented on 14.06.2017 - Vidovic
		//float Rpom[9], tpom[3], tmp[3];
		//RVLCOMPTRANSF3DWITHINV(RGT, tGT, pMatch->RICP_, pMatch->tICP_, Rpom, tpom, tmp);
		//RVLCOMPTRANSF3D(Rpom, tpom, pMatch->R, pMatch->t, pMatch->RICP, pMatch->tICP);

		//RVLCOPYMX3X3(pMatch->RICP, R);
		//RVLCOPY3VECTOR(pMatch->tICP, t);
		float tmp[3];
		RVLCOMPTRANSF3DWITHINV(RGT, tGT, pMatch->RICP, pMatch->tICP, R, t, tmp);

	}
	else
	{
		float tmp[3];
		RVLCOMPTRANSF3DWITHINV(RGT, tGT, pMatch->R, pMatch->t, R, t, tmp);
	}

	

		/*
		#ifdef RVLPSGM_MATCH_SEGMENT_CENTROID

		RVLDIFMX3X3(RGT, pMatch->R, R_);

		RVLMULMX3X3VECT(R_, modelInstanceDB.Element[pMatch->iMMI].tc, t);

		RVLSUM3VECTORS(t, tGT, t);

		RVLDIF3VECTORS(t, pMatch->t, t);
		#else if
		RVLDIF3VECTORS(pMatch->t, tGT, t);
		#endif
		*/
		

	GetAngleAxis(R, V, theta);

	GetDistance(t, distance);

	zGT[0] = RGT[2];
	zGT[1] = RGT[5];
	zGT[2] = RGT[8];

	if (evaluateICP) //not anymore, because we do not have the apsolute pose, rather the relative
	{
		thetaZ = acos(R[9]);
	}
	else
	{
		z[0] = pMatch->R[2];
		z[1] = pMatch->R[5];
		z[2] = pMatch->R[8];
		thetaZ = RVLDOTPRODUCT3(zGT, z);
}

	//if ((theta < angleThresh || (theta >(PI - angleThresh) && theta < (PI + angleThresh))) && distance < distanceThresh)
	//if (distance < distanceThresh && theta < angleThresh)
	if (distance < distanceThresh && thetaZ > angleThresh)
	{
		return true;
	}
	else
	{
		if (fpLog)
		{
			//fprintf(fpLog, "%d\t%d\t%d\t%d\t%f\t%f\n", pMatch->iScene, MCTISet.pCTI.Element[pMatch->iMCTI]->iModel, MCTISet.pCTI.Element[pMatch->iMCTI]->iCluster, CTISet.pCTI.Element[pMatch->iSCTI]->iCluster, distance, theta);
			fprintf(fpLog, "%d\t%d\t%d\t%d\t%f\t%f\n", pMatch->iScene, MCTISet.pCTI.Element[pMatch->iMCTI]->iModel, MCTISet.pCTI.Element[pMatch->iMCTI]->iCluster, CTISet.pCTI.Element[pMatch->iSCTI]->iCluster, distance, acos(thetaZ) * 180 / PI);
		}

		return false;
	}
}

void PSGM::FindGTInstance(
	RVL::GTInstance **pGT,
	int iScene,
	int iModel)
{
	int nGTModels, iGTM;

	RVL::GTInstance *pGT_ = *pGT;

	pGT_ = pECCVGT->GT.Element[iScene].Element;

	nGTModels = pECCVGT->GT.Element[iScene].n;

	for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT_++)
		if (pGT_->iModel == iModel)
			break;

	*pGT = pGT_;
}

void PSGM::CalculatePR(int TP, int FP, int FN, float &precision, float &recall)
{
	if (TP + FP > 0)
		precision = (float)TP / (float)(TP + FP);
	else
		precision = 0.0;

	recall = (float)TP / (float)(TP + FN);

}

void PSGM::FindMinMaxInScoreMatchMatrix(float &min, float &max, Array<Array<SortIndex<float>>> &scoreMatchMatrix_)
{
	
	int nSSegments = scoreMatchMatrix_.n;

	int iSSegment;
	int iMSegment;
	float scoreTmp;
	int idx;

	//scoreMatchMatrix must be sorted
	min = scoreMatchMatrix_.Element[0].Element[0].cost;
	max = scoreMatchMatrix_.Element[0].Element[0].cost;

	for (iSSegment = 1; iSSegment < nSSegments; iSSegment++)
		if (scoreMatchMatrix_.Element[iSSegment].Element[0].cost < min)
			min = scoreMatchMatrix_.Element[iSSegment].Element[0].cost;

	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
		for (iMSegment = 0; iMSegment < nMSegments; iMSegment++)
		{
			scoreTmp = scoreMatchMatrix_.Element[iSSegment].Element[iMSegment].cost;

			idx = scoreMatchMatrix_.Element[iSSegment].Element[iMSegment].idx;

			if (scoreTmp > max && idx != -1)
				max = scoreTmp;
		}
}

void PSGM::EvaluateMatchesByScore(
	FILE *fp,
	FILE *fpLog,
	FILE *fpPoseError,
	FILE *fpnotFirstInfo,
	FILE *fpnotFirstPoseErr,
	int nBestSegments,
	bool evaluateICP)
{
#ifdef RVLPSGM_EVALUATION_PRINT_INFO
	cout << "Matches evaluation..." << "\n";;
#endif

	float precision, recall;

	int graphID = 0;

	float minScore, maxScore;

	int iSSegment;
	//int nSSegments = scoreMatchMatrix.n;
	int nSSegments = CTISet.SegmentCTIs.n;

	int iMSegment;
	//int nMSegments = (MCTISet.nModels + 1) * (MCTISet.maxSegmentIdx + 1);
	int nMSegments = (MCTISet.nModels) * (MCTISet.maxSegmentIdx + 1);

	float scoreTmp;

	int idx;

	float cos30 = sqrt(3) / 2;

	//scoreMatchMatrix is sorted
	/*
	minScore = scoreMatchMatrix.Element[0].Element[0].cost;
	maxScore = scoreMatchMatrix.Element[0].Element[0].cost;

	for (iSSegment = 1; iSSegment < nSSegments; iSSegment++)
		if (scoreMatchMatrix.Element[iSSegment].Element[0].cost < minScore)
			minScore = scoreMatchMatrix.Element[iSSegment].Element[0].cost;
	
	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
		for (iMSegment = 0; iMSegment < nMSegments; iMSegment++)
		{
			scoreTmp = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].cost;

			idx = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].idx;

			if (scoreTmp > maxScore && idx != -1)
				maxScore = scoreTmp;
		}
		*/

	if (evaluateICP)
	{
		CreateScoreMatchMatrixICP();
		FindMinMaxInScoreMatchMatrix(minScore, maxScore, scoreMatchMatrixICP);
	}
	else
		FindMinMaxInScoreMatchMatrix(minScore, maxScore, scoreMatchMatrix);

	int iScore, nScoreSteps = 200;

	float scoreThresh;

	float scoreStep = (maxScore - minScore) / nScoreSteps;

	bool TPMatch, poseMatch;

	int TP_ = 0, FP_ = 0, FN_ = 0;

	int iMatchedModel, iCTI;

	int iMatch;

	int *firstTP = new int[nDominantClusters];
	float *firstTPScore = new float[nDominantClusters];
	int *firstTPiModel = new int[nDominantClusters];

	int iBestMatches;

	int iMCTI, iSCTI;

	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstance *pMCTI;

	RVL::GTInstance *pGT = NULL;

	float R_[9], t_[3];

	if (nBestSegments == 0)
	{
		scoreThresh = minScore;

		for (iScore = 0; iScore < nScoreSteps; iScore++)
		{
			scoreThresh = minScore + iScore * scoreStep;

			for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
			{
				firstTP[iSSegment] = -1;

				for (iMSegment = 0; iMSegment < nMSegments; iMSegment++)
				{
					if (evaluateICP)
						iMatch = scoreMatchMatrixICP.Element[iSSegment].Element[iMSegment].idx;
					else
					iMatch = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].idx;

					if (iMatch != -1)
					{
						if (evaluateICP)
							scoreTmp = scoreMatchMatrixICP.Element[iSSegment].Element[iMSegment].cost;
						else
						scoreTmp = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].cost;

						if (scoreTmp <= scoreThresh)
						{
							//Compare to segment GT
							iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
							iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;

							int iSegmentGT = (iScene - 1) * nDominantClusters + iSSegment;

							//eliminate FP from segments without GT
							if (!segmentGT.Element[iSegmentGT].valid)
								TPMatch = false;
							else
								TPMatch = CompareMatchToSegmentGT((iScene - 1), iSSegment, iMatchedModel);

							if (!TPMatch)
							{
								FP_++;
							}
							else
							{
								if (firstTP[iSSegment] == -1)
								{
									firstTP[iSSegment] = iMSegment;
									firstTPScore[iSSegment] = scoreTmp;
									firstTPiModel[iSSegment] = iMatchedModel;
								}

								//check pose of TP segment matches
								if (iScore == nScoreSteps - 1)
								{
									iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;

									pSCTI = CTISet.pCTI.Element[iSCTI];
									pMCTI = MCTISet.pCTI.Element[iMCTI];

									MSTransformation(pMCTI, pSCTI, pCTImatchesArray.Element[iMatch]->tMatch, pCTImatchesArray.Element[iMatch]->R, pCTImatchesArray.Element[iMatch]->t);

									FindGTInstance(&pGT, pCTImatchesArray.Element[iMatch]->iScene, iMatchedModel);

									//poseMatch = PoseCheck(pGT, pCTImatchesArray.Element[iMatch], 50.0, PI / 6, fpPoseError);
									poseMatch = PoseCheck(pGT, pCTImatchesArray.Element[iMatch], 50.0, cos30, fpPoseError, fpnotFirstPoseErr, evaluateICP);
								}

							}
						}
					}
				}
			}


#ifdef RVLPSGM_EVALUATION_PRINT_INFO
			CountTPandFN(TP_, FN_, true);
#else
			CountTPandFN(TP_, FN_, false);
#endif

			CalculatePR(TP_, FP_, FN_, precision, recall);

			pECCVGT->ResetMatchFlag();

			PrintMatchInfo(fp, fpLog, TP_, FP_, FN_, precision, recall, nSSegments, firstTP, firstTPiModel, firstTPScore, scoreThresh, minScore, maxScore, scoreStep, nBestSegments, -1.0, graphID);

			TP_ = 0; FP_ = 0; FN_ = 0;

			graphID++;
			
		}

	}
	else
	{
		for (iBestMatches = 0; iBestMatches < nBestSegments; iBestMatches++)
		{
			//for (scoreThresh = minScore; scoreThresh <= maxScore; scoreThresh += scoreStep)
			//{
				for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
				{
					firstTP[iSSegment] = -1;

					for (iMSegment = 0; iMSegment <= iBestMatches; iMSegment++)
					{
					if (evaluateICP)
						iMatch = scoreMatchMatrixICP.Element[iSSegment].Element[iMSegment].idx;
					else
						iMatch = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].idx;

						if (iMatch != -1)
						{
						if (evaluateICP)
							scoreTmp = scoreMatchMatrixICP.Element[iSSegment].Element[iMSegment].cost;
						else
							scoreTmp = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].cost;

						//save score for best match
						//if(iMSegment == 0)
						//bestNesto = scoreTmp

							//if (scoreTmp <= scoreThresh)
							//{
								//Compare to segment GT
								iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
								iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;

								int iSegmentGT = (iScene - 1) * nDominantClusters + iSSegment;

								//eliminate FP from segments without GT
								if (!segmentGT.Element[iSegmentGT].valid)
									TPMatch = false;
								else
						{
									TPMatch = CompareMatchToSegmentGT((iScene - 1), iSSegment, iMatchedModel);
						}

								if (!TPMatch)
								{
									FP_++;
								}
								else
						{
									if (firstTP[iSSegment] == -1)
									{
								//if (iSSegment == 2)
									//printf("iMatch: %d, iMatchedModel: %d", iMatch, iMatchedModel);

										firstTP[iSSegment] = iMSegment;
										firstTPScore[iSSegment] = scoreTmp;
										firstTPiModel[iSSegment] = iMatchedModel;

								//if iMSegment != 0
								//u file zapisati bestNesto i scoreTmp
							}

							//check pose of TP segment matches
							if (iBestMatches == nBestSegments - 1)
							{
								iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;

								pSCTI = CTISet.pCTI.Element[iSCTI];
								pMCTI = MCTISet.pCTI.Element[iMCTI];
								
								MSTransformation(pMCTI, pSCTI, pCTImatchesArray.Element[iMatch]->tMatch, pCTImatchesArray.Element[iMatch]->R, pCTImatchesArray.Element[iMatch]->t);					

								FindGTInstance(&pGT, pCTImatchesArray.Element[iMatch]->iScene, iMatchedModel);

								//poseMatch = PoseCheck(pGT, pCTImatchesArray.Element[iMatch], 50.0, PI / 6, fpPoseError);
								if (iMSegment != 0) // pose check for matches that are not on the first place 
									int klkl = 0;
									//zapis u file1
									//fprintf(fpnotFirstInfo, "%d, %d, %d", iScene, iSSegment, iMSegment);
									//poseMatch = PoseCheck(pGT, pCTImatchesArray.Element[iMatch], 50.0, cos30, fpPoseError, fpnotFirstPoseErr, evaluateICP);
							}
									}
							//}
						}
					}
				}

#ifdef RVLPSGM_EVALUATION_PRINT_INFO
				if (iBestMatches == nBestSegments - 1)
				CountTPandFN(TP_, FN_, true);
				else
					CountTPandFN(TP_, FN_, false);
#else
				CountTPandFN(TP_, FN_, false);
#endif

				CalculatePR(TP_, FP_, FN_, precision, recall);

			//pECCVGT->ResetMatchFlag();

				if (iBestMatches == nBestSegments - 1)
				PrintMatchInfo(fp, fpLog, TP_, FP_, FN_, precision, recall, nSSegments, firstTP, firstTPiModel, firstTPScore, -1.0, -1.0, -1.0, -1.0, nBestSegments, iBestMatches, graphID);

				TP_ = 0; FP_ = 0; FN_ = 0;

				graphID++;

			//}
		}

	}

#ifdef RVLPSGM_EVALUATION_PRINT_INFO
	cout << "---------------------------------------------------\n";
#endif

	delete[] firstTP;
	delete[] firstTPScore;
	delete[] firstTPiModel;
}

void PSGM::PrintMatchInfo(
	FILE *fp,
	FILE *fpLog,
	int TP_,
	int FP_,
	int FN_,
	float precision,
	float recall,
	int nSSegments,
	int *firstTP,
	int *firstTPiModel,
	float *firstTPScore,
	float scoreThresh,
	float minScore,
	float maxScore,
	float scoreStep,
	int nBestSegments,
	int iBestMatches,
	int graphID)
{

	int iSSegment;

#ifdef RVLPSGM_EVALUATION_PRINT_INFO
	printf("---------------------------------------------------\n");
	printf("Scene: %d\n", iScene - 1);
	printf("TP: %d\n", TP_);
	printf("FP: %d\n", FP_);
	printf("FN: %d\n", FN_);

	if (nBestSegments)
		printf("nBestMatches: %d\n", iBestMatches);
	else
	{
		printf("ScoreThresh: %f\n", scoreThresh);
		printf("Min score: %f\n", minScore);
		printf("Max score: %f\n", maxScore);
		printf("Score step: %f\n", scoreStep);
	}

	printf("Precision: %f\n", precision);
	printf("Recall: %f\n", recall);

	printf("...................................................\n");
	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
		if (firstTP[iSSegment] != -1)
			printf("First TP for segment %d is on %d place; Matched with iModel: %d (score = %f)\n", iSSegment, firstTP[iSSegment], firstTPiModel[iSSegment], firstTPScore[iSSegment]);
	printf("---------------------------------------------------\n\n");
#endif

	//print to log file
	fprintf(fpLog, "---------------------------------------------------\n");
	fprintf(fpLog, "Scene: %d\n", iScene - 1);
	fprintf(fpLog, "TP: %d\n", TP_);
	fprintf(fpLog, "FP: %d\n", FP_);
	fprintf(fpLog, "FN: %d\n", FN_);

	if (nBestSegments)
		fprintf(fpLog, "nBestMatches: %d\n", iBestMatches);
	else
	{
		fprintf(fpLog, "ScoreThresh: %f\n", scoreThresh);
		fprintf(fpLog, "Min score: %f\n", minScore);
		fprintf(fpLog, "Max score: %f\n", maxScore);
		fprintf(fpLog, "Score step: %f\n", scoreStep);
	}

	fprintf(fpLog, "Precision: %f\n", precision);
	fprintf(fpLog, "Recall: %f\n", recall);

	fprintf(fpLog, "...................................................\n");
	for (iSSegment = 0; iSSegment < nSSegments; iSSegment++)
		if (firstTP[iSSegment] != -1)
			fprintf(fpLog, "First TP for segment %d is on %d place; Matched with iModel: %d (score = %f)\n", iSSegment, firstTP[iSSegment], firstTPiModel[iSSegment], firstTPScore[iSSegment]);
	fprintf(fpLog, "---------------------------------------------------\n\n");

	fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\n", graphID, iScene - 1, TP_, FP_, FN_, nBestSegments, scoreThresh, -1.0, precision, recall);

}

void PSGM::SaveMatches()
{

	char *matchFileName = RVLCreateString(sceneFileName);

	sprintf(matchFileName + strlen(matchFileName) - 3, "smf");
	
	FILE *fp = fopen(matchFileName, "w");

	RECOG::PSGM_::MatchInstance *pMatch = CTImatches.pFirst;

	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstance *pMCTI;

	int i;

	while (pMatch)
	{
		fprintf(fp, "%d\t%d\t%d\t", pMatch->iScene, pMatch->iSCTI, pMatch->iMCTI);

		pSCTI = CTISet.pCTI.Element[pMatch->iSCTI];
		pMCTI = MCTISet.pCTI.Element[pMatch->iMCTI];

		MSTransformation(pMCTI, pSCTI, pMatch->tMatch, pMatch->R, pMatch->t);

		for (i = 0; i < 9; i++)
			fprintf(fp, "%f\t", pMatch->R[i]);

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t", pMatch->tMatch[i]);

		fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%d\n", pMatch->score, pMatch->E, pMatch->probability2, pMatch->angleGT, pMatch->distanceGT, pMatch->nValids);

		pMatch = pMatch->pNext;
	}

	fclose(fp);
}

void PSGM::SaveSegmentGT(FILE*fp, int iScene)
{
	int iSSegment;

	int nClusters = CTISet.SegmentCTIs.n;

	for (iSSegment = 0; iSSegment < nClusters; iSSegment++)
		fprintf(fp, "%d\t%d\t%d\t%d\t%d\n", segmentGT.Element[iScene*nDominantClusters + iSSegment].iScene, segmentGT.Element[iScene*nDominantClusters + iSSegment].iSSegment, segmentGT.Element[iScene*nDominantClusters + iSSegment].iModel, segmentGT.Element[iScene*nDominantClusters + iSSegment].iMSegment, segmentGT.Element[iScene*nDominantClusters + iSSegment].valid);
}

void PSGM::LoadSegmentGT(FILE*fp, int iScene)
{
	char line[3000] = { 0 };

	int nLines, iLine;

	nLines = 0;

	int iSSegmentTmp;

	//count number of lines in gt file
	while (!feof(fp))
	{
		line[0] = '\0';

		fgets(line, 3000, fp);

		if (line[0] == '\0' || line[0] == '\n')
			continue;

		nLines++;
	}

	rewind(fp);

	//load segment GT for current scene
	for (iLine = 0; iLine < nLines; iLine++)
	{
		fscanf(fp, "%d\t", &iSSegmentTmp);

		segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].iScene = iScene;

		segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].iSSegment = iSSegmentTmp;

		fscanf(fp, "%d\t%d\t%d\n", &segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].iModel, &segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].iMSegment, &segmentGT.Element[iScene*nDominantClusters + iSSegmentTmp].valid);
	}
}

void PSGM::LoadCompleteSegmentGT(FileSequenceLoader sceneSequence)
{
	RVL_DELETE_ARRAY(segmentGT.Element);

	segmentGT.Element = new RVL::SegmentGTInstance[nDominantClusters * sceneSequence.nFileNames];
	segmentGT.n = nDominantClusters * sceneSequence.nFileNames;

	char filePath[200];

	char *segmentGTFilePath;

	FILE *fp;

	int iSceneTmp = 0;

	while (sceneSequence.GetNextPath(filePath))
	{
		//create segment GT file path
		segmentGTFilePath = RVLCreateFileName(filePath, ".ply", -1, ".sgt", pMem);

		fp = fopen(segmentGTFilePath, "r");

		if (fp)
		{
			LoadSegmentGT(fp, iSceneTmp);

			fclose(fp);
		}
		else
		{
			printf("*****************WARNING****************\n", filePath);
			printf("Segment GT file is missing for scene:\n%s\n", filePath);
			printf("****************************************\n", filePath);
		}

		iSceneTmp++;
	}
}

void PSGM::ConvexTemplateCentoidID()
{
	int iPlane, i;

	float minx, maxx, miny, maxy, minz, maxz;
	

	for (i = 0; i < 6; i++)
		centroidID.Element[i].Idx = 0;
	
	minx = convexTemplate.Element[0].N[0];
	maxx = convexTemplate.Element[0].N[0];

	miny = convexTemplate.Element[0].N[1];
	maxy = convexTemplate.Element[0].N[1];

	minz = convexTemplate.Element[0].N[2];
	maxz = convexTemplate.Element[0].N[2];		

	for (iPlane = 1; iPlane < convexTemplate.n; iPlane++)
	{
		//find minx
		if (convexTemplate.Element[iPlane].N[0] < minx)
		{
			minx = convexTemplate.Element[iPlane].N[0];
			centroidID.Element[0].Idx = iPlane;
		}

		//find maxx
		if (convexTemplate.Element[iPlane].N[0] > maxx)
		{
			maxx = convexTemplate.Element[iPlane].N[0];
			centroidID.Element[1].Idx = iPlane;
		}

		//find miny
		if (convexTemplate.Element[iPlane].N[1] < miny)
		{
			miny = convexTemplate.Element[iPlane].N[1];
			centroidID.Element[2].Idx = iPlane;
		}

		//find maxy
		if (convexTemplate.Element[iPlane].N[1] > maxy)
		{
			maxy = convexTemplate.Element[iPlane].N[1];
			centroidID.Element[3].Idx = iPlane;
		}

		//find minz
		if (convexTemplate.Element[iPlane].N[2] < minz)
		{
			minz = convexTemplate.Element[iPlane].N[2];
			centroidID.Element[4].Idx = iPlane;
		}

		//find maxz
		if (convexTemplate.Element[iPlane].N[2] > maxz)
		{
			maxz = convexTemplate.Element[iPlane].N[2];
			centroidID.Element[5].Idx = iPlane;
		}
	}

	bBoundingPlanes = true;
}
//END Vidovic

void PSGM::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	unsigned char *selectionColor)
{
	pVisualizer->normalLength = 10.0;

	pVisualizer->SetMesh(pMesh);

	displayData.pMesh = pMesh;
	displayData.pSurfels = pSurfels;
	displayData.pRecognition = this;
	displayData.pVisualizer = pVisualizer;	
	RVLCOPY3VECTOR(selectionColor, displayData.selectionColor);
	displayData.iSelectedCluster = -1;
		pSurfels->DisplayData.keyPressUserFunction = &RECOG::PSGM_::keyPressUserFunction;
		pSurfels->DisplayData.mouseRButtonDownUserFunction = &RECOG::PSGM_::mouseRButtonDownUserFunction;
		pSurfels->DisplayData.vpUserFunctionData = &displayData;

	pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);
	
	//Is this the best place for this????????
	this->pMesh = pMesh;
}

void PSGM::Display()
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	if (problem == RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION)
	{
		DisplayClusters();

		pSurfels->DisplayEdgeFeatures();

		displayData.bClusters = true;
	}
	else if (problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
	{
		displayData.bClusters = false;

		pSurfels->Display(pVisualizer, pMesh);

		//pSVertexGraph->Display(pVisualizer);
	}

	//pSurfels->Display(pVisualizer, pMesh);

	//DisplayVertices();

	//if (!displayData.bVertices)
	//	displayData.vertices->VisibilityOff();

	//DisplayReferenceFrames();
}

void PSGM::DisplayClusters()
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	int iCluster;
	unsigned char color[3];

	for (iCluster = 0; iCluster < clusters.n; iCluster++)
	{
		RandomColor(color);

		PaintCluster(iCluster, color);
	}
}

void PSGM::DisplayModelInstance(Visualizer *pVisualizer)
{
	//Create polygonal mesh

	//// Setup four points
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(0.0, 0.0, 0.0);
	points->InsertNextPoint(1.0, 0.0, 0.0);
	points->InsertNextPoint(1.5, 0.5, 0.0);
	points->InsertNextPoint(1.0, 1.0, 0.0);
	points->InsertNextPoint(0.0, 1.0, 0.0);

	// Define some colors
	unsigned char red[3] = { 255, 0, 0 };
	unsigned char green[3] = { 0, 255, 0 };
	unsigned char blue[3] = { 0, 0, 255 };
	unsigned char white[3] = { 255, 255, 255 };
	unsigned char black[3] = { 0, 0, 0 };

	// Setup the colors array
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	// Add the three colors we have created to the array
	colors->InsertNextTupleValue(red);
	colors->InsertNextTupleValue(green);
	colors->InsertNextTupleValue(blue);
	colors->InsertNextTupleValue(white);
	colors->InsertNextTupleValue(black);

	// Create the polygon
	vtkSmartPointer<vtkPolygon> polygon =
		vtkSmartPointer<vtkPolygon>::New();
	polygon->GetPointIds()->SetNumberOfIds(5); //make a quad
	polygon->GetPointIds()->SetId(0, 0);
	polygon->GetPointIds()->SetId(1, 1);
	polygon->GetPointIds()->SetId(2, 2);
	polygon->GetPointIds()->SetId(3, 3);
	polygon->GetPointIds()->SetId(4, 4);
	//polygon->GetPointIds()->SetId(5, 5);

	// Add the polygon to a list of polygons
	vtkSmartPointer<vtkCellArray> polygons =
		vtkSmartPointer<vtkCellArray>::New();
	polygons->InsertNextCell(polygon);

	// Create a polydata object and add everything to it
	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	polydata->SetPolys(polygons);
	polydata->GetPointData()->SetScalars(colors);

	//Mapper
	pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
	//map->SetInputData(pMesh->pPolygonData);
	pVisualizer->map->SetInputData(polydata);
	//map->SetInputConnection(polyDataNormals->GetOutputPort());
	pVisualizer->map->InterpolateScalarsBeforeMappingOff();

	//Actor
	pVisualizer->actor = vtkSmartPointer<vtkActor>::New();
	pVisualizer->actor->SetMapper(pVisualizer->map);

	//Insert actor
	pVisualizer->renderer->AddActor(pVisualizer->actor);
}

void PSGM::DisplayCTIs(
	Visualizer *pVisualizer,
	RECOG::CTISet *pCTISet,
	Array<int> *pCTIArray)
{
	Array<int> CTIArray;
	int i;

	if (pCTIArray)
		CTIArray = *pCTIArray;
	else
	{
		CTIArray.Element = new int[pCTISet->pCTI.n];
		CTIArray.n = pCTISet->pCTI.n;

		for (i = 0; i < pCTISet->pCTI.n; i++)
			CTIArray.Element[i] = i;
	}

	RECOG::PSGM_::ModelInstance *pCTI;

	for (i = 0; i < CTIArray.n; i++)
	{
		pCTI = pCTISet->pCTI.Element[CTIArray.Element[i]];

		DisplayCTI(pVisualizer, pCTI);
	}
}

void PSGM::DisplayCTI(
	Visualizer *pVisualizer,
	RECOG::PSGM_::ModelInstance *pCTI)
{
	float *N = new float[3 * convexTemplate.n];

	float *N_ = N;

	float *d = new float[convexTemplate.n];

	int i;
	float *N__;

	for (i = 0; i < convexTemplate.n; i++, N_ += 3)
	{
		N__ = convexTemplate.Element[i].N;

		RVLCOPY3VECTOR(N__, N_);

		d[i] = pCTI->modelInstance.Element[i].d;
	}
	
	float tCTIc_CTI[3];

	vtkSmartPointer<vtkPolyData> modelPD = GenerateCTIPrimitivePolydata_RW(N, d, convexTemplate.n, false, NULL, tCTIc_CTI);

	float *RCTI_S = pCTI->R;
	float *tCTI_S = pCTI->t;
	
	float tCTIc_S[3];

	RVLTRANSF3(tCTIc_CTI, RCTI_S, tCTI_S, tCTIc_S);

	double T[16];

	RVLHTRANSFMX(RCTI_S, tCTIc_S, T);

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(T);

	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(modelPD);

	transformFilter->SetTransform(transform);
	transformFilter->Update();

	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper->SetInputConnection(transformFilter->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
	modelActor->SetMapper(modelMapper);
	modelActor->GetProperty()->SetColor(0, 1, 0);
	pVisualizer->renderer->AddActor(modelActor);

	delete[] N;
	delete[] d;
}

void PSGM::PaintCluster(
	int iCluster,
	unsigned char *color)
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	int i;
	int iSurfel;
	Surfel *pSurfel;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		iSurfel = pCluster->iSurfelArray.Element[i];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (!pSurfel->bEdge)
		pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);
	}
}

void PSGM::ResetClusterColor(int iCluster)
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	int i;
	int iSurfel;
	Surfel *pSurfel;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		iSurfel = pCluster->iSurfelArray.Element[i];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (!pSurfel->bEdge)
			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, clusterColor + 3 * clusterMap[iSurfel]);
	}
}

void PSGM::PaintClusterVertices(
	int iCluster,
	unsigned char *color)
{
	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	pSurfels->PaintVertices(&(pCluster->iVertexArray), color);
}

void PSGM::DisplayReferenceFrames()
{
	Visualizer *pVisualizer = displayData.pVisualizer;

	double axesLength = 10.0;

	// Create the polydata where we will store all the geometric data
	referenceFramesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	// Create lines.
	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	// Create colors.
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	//int nClusters = RVLMIN(clusters.n, nDominantClusters); //Vidovic

	//RECOG::PSGM_::Cluster *pCluster; //Vidovic
	//int iCluster; //Vidovic
	RECOG::PSGM_::ModelInstance *pModelInstance;

	//Vidovic
	/*
	for (iCluster = 0; iCluster < nClusters; iCluster++)
	{
		pCluster = clusters.Element[iCluster];

		pModelInstance = pCluster->modelInstanceList.pFirst;

		while (pModelInstance)
		{
			pVisualizer->AddReferenceFrame(pts, lines, colors, pModelInstance->R, pModelInstance->t, 10.0);

			//vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

			//axes->SetTotalLength(axesLength, axesLength, axesLength);

			//vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

			//vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
			//double T[16];
			//RVLCREATE3DTRANSF(pModelInstance->R, pModelInstance->t, T);

			//transform->SetMatrix(T);

			//axes->SetUserTransform(transform);

			//vtkMatrix4x4 *T_ = axes->GetMatrix();

			//pVisualizer->renderer->AddActor(axes);

			pModelInstance = pModelInstance->pNext;
		}
	}*/

	pModelInstance = CTISet.CTI.pFirst;

	while (pModelInstance)
	{
		pVisualizer->AddReferenceFrame(pts, lines, colors, pModelInstance->R, pModelInstance->t, 10.0);

		//vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

		//axes->SetTotalLength(axesLength, axesLength, axesLength);

		//vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

		//vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		//double T[16];
		//RVLCREATE3DTRANSF(pModelInstance->R, pModelInstance->t, T);

		//transform->SetMatrix(T);

		//axes->SetUserTransform(transform);

		//vtkMatrix4x4 *T_ = axes->GetMatrix();

		//pVisualizer->renderer->AddActor(axes);

		pModelInstance = pModelInstance->pNext;
	}
	//END Vidovic

	// Add the points to the polydata container
	referenceFramesPolyData->SetPoints(pts);

	// Add the lines to the polydata container
	referenceFramesPolyData->SetLines(lines);

	// Color the lines.
	referenceFramesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(referenceFramesPolyData);

	displayData.referenceFrames = vtkSmartPointer<vtkActor>::New();
	displayData.referenceFrames->SetMapper(mapper);

	pVisualizer->renderer->AddActor(displayData.referenceFrames);
}

bool RVL::RECOG::PSGM_::keyPressUserFunction(
	Mesh *pMesh, 
	SurfelGraph *pSurfels, 
	std::string &key, 
	void *vpData
	)
{
	RECOG::PSGM_::DisplayData *pData = (RECOG::PSGM_::DisplayData *)vpData;

	PSGM *pRecognition = pData->pRecognition;
	Visualizer *pVisualizer = pData->pVisualizer;

	if (key == "a")
	{
		pData->bClusters = !pData->bClusters;

		if (pData->bClusters)
			pRecognition->DisplayClusters();
		else
		{
			pSurfels->Display(pVisualizer, pMesh);

			//if (pSurfels->DisplayData.bVertices)
			//{
			//	if (pData->iSelectedCluster >= 0)
			//	{
			//		unsigned char color[3];

			//		RVLSET3VECTOR(color, 255, 0, 0);

			//		pRecognition->PaintClusterVertices(pData->iSelectedCluster, color);

			//		pSurfels->UpdateVertexDisplayLines();
			//	}
			//}

			pData->iSelectedCluster = -1;
		}
			
		return true;
	}

	//sometimes it doesn't exit properly, as if "c" is constantly pressed
	if (key == "c") // choose which hypothesis is shown (0-6) 
	{
		std::string line;
		int iHypothesesRank = -1;
		unsigned char SelectionColor[3];

		SelectionColor[0] = 0;
		SelectionColor[1] = 255;
		SelectionColor[2] = 0;
		do
		{
			std::cout << "Enter hypothesis rank, 0-6: ";
			std::getline(std::cin, line);
			sscanf(line.data(), "%d", &iHypothesesRank);
		} while (iHypothesesRank < 0 || iHypothesesRank > 25);

		//delete visualized ICP matches from the scene:
		pVisualizer->renderer->RemoveAllViewProps();
		//pRecognition->InitDisplay(pVisualizer, pMesh, SelectionColor);
		pVisualizer->SetMesh(pMesh);
		pRecognition->Display();


#ifdef RVLPSGM_ICP
		for (int i = 0; i < pRecognition->scoreMatchMatrixICP.n; i++)
		{
			if (pRecognition->scoreMatchMatrixICP.Element[i].Element[iHypothesesRank].idx != -1)
			{
				//visualize new ICP matches on the scene
				pRecognition->AddOneModelToVisualizer(pVisualizer, pRecognition->scoreMatchMatrixICP.Element[i].Element[iHypothesesRank].idx, iHypothesesRank, true, true);
			}
		}
		////Filko - for transparency testing
		//for (int i = 0; i < pRecognition->scoreMatchMatrixICP.n; i++)
		//{
		//	for (int j = 0; j < RVLMIN(7, pRecognition->scoreMatchMatrixICP.Element[i].n); j++)
		//	{

		//		if (pRecognition->scoreMatchMatrixICP.Element[i].Element[j].idx != -1)
		//		{
		//			//visualize new ICP matches on the scene
		//			pRecognition->AddOneModelToVisualizer(pVisualizer, pRecognition->scoreMatchMatrixICP.Element[i].Element[j].idx, j, true);
		//			break;
		//		}
		//	}
		//}
#else
#ifdef RVLPSGM_MATCHCTI_MATCH_MATRIX
		for (int i = 0; i < pRecognition->bestSceneSegmentMatches.n; i++)
		{
			if (pRecognition->bestSceneSegmentMatches.Element[i].Element[iHypothesesRank].idx != -1)
			{
				//visualize new ICP matches on the scene
				pRecognition->AddOneModelToVisualizer(pVisualizer, pRecognition->bestSceneSegmentMatches.Element[i].Element[iHypothesesRank].idx, iHypothesesRank, true);
			}
		}
#else

		for (int i = 0; i < pRecognition->scoreMatchMatrix.n; i++)
		{
			if (pRecognition->scoreMatchMatrix.Element[i].Element[iHypothesesRank].idx != -1)
			{
				//visualize new matches on the scene
				pRecognition->AddOneModelToVisualizer(pVisualizer, pRecognition->scoreMatchMatrix.Element[i].Element[iHypothesesRank].idx, iHypothesesRank, false);

				//printf("CTI %d\n", pRecognition->pCTImatchesArray.Element[pRecognition->scoreMatchMatrix.Element[i].Element[iHypothesesRank].idx]->iSCTI);
			}
		}
#endif
#endif
		
		return true;
	}

	if (key == "m")
	{
		pRecognition->AddGTModelsToVisualizer(pVisualizer);
	}

	//single match visualization (choose segment and hypothesis to show)
	if (key == "d") 
	{
		char cSelection;
		bool bICPPose;
		Array<Array<SortIndex<float>>> *scoreMatchMatrix;

		do
		{
			printf("Enter 'c' for CTI match or 'i' for ICP match dispaly: ");
			scanf("%c", &cSelection);
		} while (cSelection != 'c' && cSelection != 'i');

		if (cSelection == 'c')
		{
#ifdef RVLPSGM_MATCHCTI_MATCH_MATRIX
			scoreMatchMatrix = &pRecognition->bestSceneSegmentMatches;
#else
			scoreMatchMatrix = &pRecognition->scoreMatchMatrix;
#endif
			bICPPose = false;
		}
		else
		{
			scoreMatchMatrix = &pRecognition->scoreMatchMatrixICP;
			bICPPose = true;
		}

		int iSegment = -1, iHypothesesRank = -1;
		unsigned char SelectionColor[3];

		//SelectionColor[0] = rand() % 256;
		//SelectionColor[1] = rand() % 256;
		//SelectionColor[2] = rand() % 256;

		do
		{
			printf("Enter segment identifier: ");
			scanf("%d", &iSegment);
		} while (iSegment < 0 || iSegment > scoreMatchMatrix->n);

		do
		{
			printf("Enter hypothesis rank: ");
			scanf("%d", &iHypothesesRank);
		} while (iHypothesesRank < 0 || iHypothesesRank > scoreMatchMatrix->Element[iSegment].n);


		if (scoreMatchMatrix->Element[iSegment].Element[iHypothesesRank].idx != -1)
		{
			pRecognition->AddOneModelToVisualizer(pVisualizer, scoreMatchMatrix->Element[iSegment].Element[iHypothesesRank].idx, iHypothesesRank, true, false, bICPPose);
		}

	}

	//print CTI/ICP matches info
	if (key == "i")
	{
		char cSelection;

		do
		{
			printf("Enter 'c' for CTI match or 'i' for ICP match dispaly: ");
			scanf("%c", &cSelection);
		} while (cSelection != 'c' && cSelection != 'i');

		if (cSelection == 'c')
			pRecognition->PrintCTIMatches();
		else
			pRecognition->PrintICPMatches();
	}

	//delete visualized matches from the scene
	if (key == "Delete")
	{
		pVisualizer->renderer->RemoveAllViewProps();
		pVisualizer->SetMesh(pMesh);
		pRecognition->Display();
	}

	//visualize consensus hypotheses
	if (key == "Insert")
	{
		pRecognition->VisualizeConsensusHypotheses(pVisualizer);
	}

	//create matchGT
	if (key == "F2")
	{
		pRecognition->createMatchGT = !pRecognition->createMatchGT;
		char *matchGTFileName;

		if (pRecognition->createMatchGT)
		{
			cout << "GT match visualization started!" << endl;
			matchGTFileName = RVLCreateFileName(pRecognition->sceneFileName, ".ply", -1, ".mgt", pRecognition->pMem);
			pRecognition->fpMatchGT = fopen(matchGTFileName, "w");
			pRecognition->matchGTiS = 0;
			pRecognition->matchGTiRank = -1;
		}
		else
		{
			fclose(pRecognition->fpMatchGT);
			cout << "GT match visualization stoped!" << endl;
		}
	}

	if (pRecognition->createMatchGT)
	{
		RECOG::PSGM_::MatchInstance *pMatch;

		//visualize next match
		if (key == "BackSpace")
		{
			int iSegment, iSegmentGT, iMatchedModel;

			do
			{
				if (pRecognition->matchGTiRank < pRecognition->scoreMatchMatrixICP.Element[pRecognition->matchGTiS].n - 1)
					pRecognition->matchGTiRank++;
				else
					if (pRecognition->matchGTiS < pRecognition->scoreMatchMatrixICP.n - 1)
					{
						pRecognition->matchGTiS++;
						pRecognition->matchGTiRank = 0;
					}
					else
					{
						pRecognition->createMatchGT = false;
						fclose(pRecognition->fpMatchGT);
						cout << "GT match visualization finished!" << endl;
					}

				pMatch = pRecognition->GetMatch(pRecognition->scoreMatchMatrixICP.Element[pRecognition->matchGTiS].Element[pRecognition->matchGTiRank].idx);
				iSegment = pRecognition->GetSCTI(pMatch)->iCluster;
				iSegmentGT = pMatch->iScene * pRecognition->nDominantClusters + iSegment;
				iMatchedModel = pRecognition->GetMCTI(pMatch)->iModel;

			} while ((iMatchedModel != pRecognition->segmentGT.Element[iSegmentGT].iModel || !pRecognition->segmentGT.Element[iSegmentGT].valid) && pRecognition->createMatchGT); //TP model

			if (pRecognition->createMatchGT)
			{
				cout << "GT match visualization:" << endl;
				cout << "Segment: " << iSegment << endl;
				cout << "Model: " << iMatchedModel << endl;
				cout << "Rank: " << pRecognition->matchGTiRank << endl;
				pRecognition->VisualizeGTMatch(pVisualizer);
			}
		}

		//print current match to file
		if (key == "Return")
		{
			cout << "***GT match printed to file!***" << endl;
			pMatch = pRecognition->GetMatch(pRecognition->scoreMatchMatrixICP.Element[pRecognition->matchGTiS].Element[pRecognition->matchGTiRank].idx);
			int CTIRank = pRecognition->FindCTIMatchRank(pMatch->ID, pRecognition->matchGTiS);
			fprintf(pRecognition->fpMatchGT, "%d\t%d\t%d\t%d\t%d\t%d\n", pMatch->iScene, pRecognition->GetSCTI(pMatch)->iCluster, pRecognition->GetMCTI(pMatch)->iModel, pMatch->ID, CTIRank, pRecognition->matchGTiRank);
		}
	}

	return false;
}

bool RVL::RECOG::PSGM_::mouseRButtonDownUserFunction(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSelectedPt,
	int iSelectedSurfel,
	void *vpData)
{
	RECOG::PSGM_::DisplayData *pData = (RECOG::PSGM_::DisplayData *)vpData;

	if (!pData->bClusters)
		return false;

	PSGM *pRecognition = pData->pRecognition;
	Visualizer *pVisualizer = pData->pVisualizer;

	unsigned char color[3];

	if (pData->iSelectedCluster >= 0)
	{
		if (pRecognition->clusterColor)
			pRecognition->ResetClusterColor(pData->iSelectedCluster);
		else
		{
			RandomColor(color);

			pRecognition->PaintCluster(pData->iSelectedCluster, color);
		}

		if (pSurfels->DisplayData.bVertices)
		{
			RVLSET3VECTOR(color, 255, 0, 0);

			pRecognition->PaintClusterVertices(pData->iSelectedCluster, color);
		}
	}

	int iCluster = pRecognition->clusterMap[iSelectedSurfel];

	if (iCluster >= 0)
	{
		printf("Selected segment: %d\n", iCluster);

		pRecognition->PaintCluster(iCluster, pData->selectionColor);

		if (pSurfels->DisplayData.bVertices)
		{
			RVLSET3VECTOR(color, 255, 255, 0);

			pRecognition->PaintClusterVertices(iCluster, color);

			pSurfels->UpdateVertexDisplayLines();
		}

		pData->iSelectedCluster = iCluster;

		//FILE *fp = fopen("C:\\RVL\\Debug\\cluster_vertices.txt", "w");

		//RECOG::PSGM_::Cluster *pCluster = pRecognition->clusters.Element[iCluster];

		//int i, iVertex;
		//SURFEL::Vertex *pVertex;

		//for (i = 0; i < pCluster->iVertexArray.n; i++)
		//{
		//	iVertex = pCluster->iVertexArray.Element[i];

		//	pVertex = pSurfels->vertexArray.Element[iVertex];

		//	fprintf(fp, "%d\t%lf\t%lf\t%lf\n", iVertex, pVertex->P[0], pVertex->P[1], pVertex->P[2]);
		//}

		//fclose(fp);

		return true;
	}
	else
		return false;
}

void PSGM::CalculatePose(int iMatch)
{
	//int iMatch = scoreMatchMatrix.Element[iSSegment].Element[iMSegment].idx;
	int iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
	int iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;

	RVL::RECOG::PSGM_::ModelInstance *pSCTI = CTISet.pCTI.Element[iSCTI];
	RVL::RECOG::PSGM_::ModelInstance *pMCTI = MCTISet.pCTI.Element[iMCTI];

	MSTransformation(pMCTI, pSCTI, pCTImatchesArray.Element[iMatch]->tMatch, pCTImatchesArray.Element[iMatch]->R, pCTImatchesArray.Element[iMatch]->t);
}



void PSGM::AddModelsToVisualizer(Visualizer *pVisualizer, bool align, RVL::PSGM::ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr)
{
	printf("Visualization:\n\n");
	for (int i = 0; i < scoreMatchMatrixICP.n; i++)
	{
		if (scoreMatchMatrixICP.Element[i].Element[0].idx != -1)
			//AddOneModelToVisualizer(pVisualizer, scoreMatchMatrixICP.Element[i].Element[0].idx, 0, align); //without double ICP
			AddOneModelToVisualizerICP(pVisualizer, scoreMatchMatrixICP.Element[i].Element[0].idx, align, ICPFunction, ICPvariant, kdTreePtr); //with icp
	}
}

//OLD visualizer - for ICP in scene c.s.
//void PSGM::AddOneModelToVisualizer(Visualizer *pVisualizer, int iMatch, int iRank, bool align)
//{
//	//Setting indices:
//	int iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
//	int iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
//	int iCluster, iModel;
//
//	//Getting scene and model pointers:
//	RECOG::PSGM_::ModelInstance *pMCTI;
//	RECOG::PSGM_::ModelInstanceElement *pMIE;
//	RECOG::PSGM_::ModelInstance *pSCTI;
//	RECOG::PSGM_::ModelInstanceElement *pSIE;
//	pMCTI = MCTISet.pCTI.Element[iMCTI];
//	pMIE = pMCTI->modelInstance.Element;
//	pSCTI = CTISet.pCTI.Element[iSCTI];
//	pSIE = pSCTI->modelInstance.Element;
//
//	iCluster = pSCTI->iCluster;
//	iModel = pMCTI->iModel;
//
//	//For a chosen hypothesis, prints which scene segment is matched to which model
//#ifdef RVLPSGM_ICP
//	printf("SSegment: %d\tMatchedModel: %d (score: %f)\n", iCluster, iModel, scoreMatchMatrixICP.Element[iCluster].Element[iRank].cost);
//#else
//	printf("SSegment: %d\tMatchedModel: %d (score: %f)\n", iCluster, iModel, scoreMatchMatrix.Element[iCluster].Element[iRank].cost);
//#endif
//
//	//Setting descriptors:
//	float *dM = new float[66];
//	float *dS = new float[66];
//	int *validS = new int[66];
//
//	for (int i = 0; i < 66; i++)
//	{
//		dS[i] = pSIE->d; // Filling scene descriptor
//		validS[i] = pSIE->valid;
//		pSIE++;
//
//		dM[i] = pMIE->d / 1000.0; // Filling model descriptor
//		pMIE++;
//	}
//
//	//Getting match pointer and calculating pose:
//	RECOG::PSGM_::MatchInstance *pMatch = pCTImatchesArray.Element[iMatch];
//	CalculatePose(iMatch);
//	Eigen::MatrixXf nT = ConvexTemplatenT();
//
//	//Generate model polydata
//	float t[3];
//	vtkSmartPointer<vtkPolyData> modelPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dM, 66, false, NULL, t);
//
//	//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
//	float *R_M_S = pCTImatchesArray.Element[iMatch]->R;
//	float *t_M_S_mm = pCTImatchesArray.Element[iMatch]->t;
//	float t_M_S[3];
//	RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);
//	float *R_CTIM_M = pMCTI->R;
//	float *t_CTIM_M = pMCTI->t;
//	float R_CTIM_S[9], t_CTIM_S[3];
//
//	RVLCOMPTRANSF3D(R_M_S, t_M_S, R_CTIM_M, t_CTIM_M, R_CTIM_S, t_CTIM_S);
//	float t_CCTIM_S[3];
//	RVLTRANSF3(t, R_CTIM_S, t_CTIM_S, t_CCTIM_S);
//	double T_CCTIM_S[16];
//	RVLHTRANSFMX(pSCTI->R, t_CCTIM_S, T_CCTIM_S);
//
//	//Generate scene CTI polydata
//	vtkSmartPointer<vtkPolyData> modelSPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dS, 66, false, validS, t);
//
//	//Getting transform from centered (scene) CTI polygon data to scene (T_CCTIS_S)
//	float t_CCTIS_S[3];
//	RVLTRANSF3(t, pSCTI->R, pSCTI->t, t_CCTIS_S)
//		double T_CCTIS_S[16];
//	RVLHTRANSFMX(pSCTI->R, t_CCTIS_S, T_CCTIS_S);
//
//
//	//PLY Model transformation
//	double T_M_S[16];
//	RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);
//	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
//
//#ifdef RVLPSGM_ICP
//	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
//		transform->SetMatrix(T_M_S); //when transforming PLY models to scene
//#endif
//
//	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
//		transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene
//
//#ifdef RVLPSGM_ICP
//	//Scaling PLY model to meters
//	vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
//	transformScale->Scale(0.001, 0.001, 0.001);
//	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//	transformFilterScale->SetInputData(vtkModelDB.at(iModel));
//	transformFilterScale->SetTransform(transformScale);
//	transformFilterScale->Update();
//#endif
//
//	//Transforming PLY model or CTI convex hull model to scene
//	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
//		transformFilter->SetInputData(modelPD); //model CTI convex hull
//#ifdef RVLPSGM_ICP
//	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
//		transformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model
//#endif
//
//	transformFilter->SetTransform(transform);
//	transformFilter->Update();
//
//	//Transforming scene CTI convex hull
//	vtkSmartPointer<vtkTransform> transform2 = vtkSmartPointer<vtkTransform>::New();
//	transform2->SetMatrix(T_CCTIS_S);
//	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//	transformFilter2->SetInputData(modelSPD);
//	transformFilter2->SetTransform(transform2);
//	transformFilter2->Update();
//	
//#ifdef RVLPSGM_ICP
//	//Aligning point clouds (if required) 
//	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterICP;
//	if (align)
//		{
//		//Sampling filter for model polydata
//		//vtkSmartPointer<vtkTriangleFilter> modelSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
//		//modelSamplerTriangleFilter->SetInputConnection(transformFilter->GetOutputPort());
//		//modelSamplerTriangleFilter->Update();
//		//vtkSmartPointer<vtkPolyDataPointSampler> modelSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
//		//modelSampler->SetInputConnection(modelSamplerTriangleFilter->GetOutputPort());
//		//modelSampler->SetDistance(0.005);
//		//modelSampler->Update();
//		//vtkSmartPointer<vtkPolyData> modelSamplerPD = modelSampler->GetOutput();
//
//		//Sampling filter for scene polydata
//		//vtkSmartPointer<vtkTriangleFilter> sceneSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New(); //Creates triangles from polygons (Samples don't work with polygons)
//		//sceneSamplerTriangleFilter->SetInputConnection(transformFilter2->GetOutputPort());
//		//sceneSamplerTriangleFilter->Update();
//		//vtkSmartPointer<vtkPolyDataPointSampler> sceneSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
//		//sceneSampler->SetInputConnection(sceneSamplerTriangleFilter->GetOutputPort());
//		//sceneSampler->SetDistance(0.005);
//		//sceneSampler->Update();
//		//vtkSmartPointer<vtkPolyData> sceneSamplerPD = sceneSampler->GetOutput();
//
//		/*vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
//		cleanFilter->SetInputConnection(transformFilter->GetOutputPort());
//		cleanFilter->PointMergingOn();
//		cleanFilter->SetAbsoluteTolerance(0.005);
//		cleanFilter->ToleranceIsAbsoluteOn();
//		cleanFilter->Update();*/
//		//vtkSmartPointer<vtkPolyData> scenePD = GetSceneModelPC(iCluster); //Generate scene model pointcloud
//		
//		//Aligning pointcluds (using PCL ICP)
//		double icpT[16];
//
//		vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilter->GetOutput()); //Generate visible parts of the models pointcloud
//		for (int k = 0; k < 16; k++)
//			{
//			icpT[k] = icpTMatrix[iCluster*nBestMatches * 16 + iRank * 16 + k];
//			}
//
//		//Transforming model polydata to ICP pose
//		vtkSmartPointer<vtkTransform> transformICP = vtkSmartPointer<vtkTransform>::New();
//		transformICP->SetMatrix(icpT);
//		transformFilterICP = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//		transformFilterICP->SetInputData(visiblePD);
//		//transformFilterICP->SetInputConnection(transformFilter->GetOutputPort());
//		transformFilterICP->SetTransform(transformICP);
//		transformFilterICP->Update();
//		}		
//#endif
//		
//
//	//Mapper and actor for model
//	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//#ifdef RVLPSGM_ICP
//	if (align)
//		modelMapper->SetInputConnection(transformFilterICP->GetOutputPort());
//	else
//#endif
//		modelMapper->SetInputConnection(transformFilter->GetOutputPort());
//	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
//	modelActor->SetMapper(modelMapper);
//	modelActor->GetProperty()->SetColor(1, 0, 0);
//	modelActor->GetProperty()->SetPointSize(3);
//	pVisualizer->renderer->AddActor(modelActor);
//
//	//Mapper and actor for scene
//	vtkSmartPointer<vtkPolyDataMapper> modelMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
//	modelMapper2->SetInputConnection(transformFilter2->GetOutputPort());
//	vtkSmartPointer<vtkActor> modelActor2 = vtkSmartPointer<vtkActor>::New();
//	modelActor2->SetMapper(modelMapper2);
//	modelActor2->GetProperty()->SetColor(0, 0, 1);
//	pVisualizer->renderer->AddActor(modelActor2);
//
//	delete[] dM;
//	delete[] dS;
//	delete[] validS;
//}

void PSGM::AddOneModelToVisualizer(Visualizer *pVisualizer, int iMatch, int iRank, bool align, bool useTG, bool bICPPose)
{
	//Setting indices:
	int iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
	int iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
	int iCluster, iModel;

	//Getting scene and model pointers:
	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;
	pMCTI = MCTISet.pCTI.Element[iMCTI];
	pMIE = pMCTI->modelInstance.Element;
	pSCTI = CTISet.pCTI.Element[iSCTI];
	pSIE = pSCTI->modelInstance.Element;

	iCluster = pSCTI->iCluster;
	iModel = pMCTI->iModel;

	//For a chosen hypothesis, prints which scene segment is matched to which model
	if (iRank != -1)
	{
#ifdef RVLPSGM_ICP
		printf("SSegment: %d\tMatchedModel: %d (score: %f) - ID: %d\n", iCluster, iModel, scoreMatchMatrixICP.Element[iCluster].Element[iRank].cost, scoreMatchMatrixICP.Element[iCluster].Element[iRank].idx);
#else
#ifdef RVLPSGM_MATCHCTI_MATCH_MATRIX
		printf("SSegment: %d\tMatchedModel: %d (score: %f) - ID: %d\n", iCluster, iModel, bestSceneSegmentMatches.Element[iCluster].Element[iRank].cost, bestSceneSegmentMatches.Element[iCluster].Element[iRank].idx);
#else
		printf("SSegment: %d\tMatchedModel: %d (score: %f) - ID: %d\n", iCluster, iModel, scoreMatchMatrix.Element[iCluster].Element[iRank].cost, scoreMatchMatrix.Element[iCluster].Element[iRank].idx);
#endif
#endif
	}

	//Setting descriptors:
	float *dM = new float[66];
	float *dS = new float[66];
	int *validS = new int[66];

	for (int i = 0; i < 66; i++)
	{
		dS[i] = pSIE->d; // Filling scene descriptor
		validS[i] = pSIE->valid;
		pSIE++;

		if (useTG)
		{
			dM[i] = MTGSet.TGs.at(MCTISet.pCTI.Element[pCTImatchesArray.Element[iMatch]->iMCTI]->iModel)->descriptor.Element[i].pFirst->ptr->d / 1000.0;
			pMIE++;
		}
		else
		{
			dM[i] = pMIE->d / 1000.0; // Filling model descriptor
			pMIE++;
		}
	}

	//Getting match pointer and calculating pose:
	RECOG::PSGM_::MatchInstance *pMatch = pCTImatchesArray.Element[iMatch];
	CalculatePose(iMatch);
	Eigen::MatrixXf nT = ConvexTemplatenT();

	//Generate model polydata
	float tMc[3];
	vtkSmartPointer<vtkPolyData> modelPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dM, 66, false, NULL, tMc);

	//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
	float *R_M_S = pCTImatchesArray.Element[iMatch]->R;
	float *t_M_S_mm = pCTImatchesArray.Element[iMatch]->t;
	float t_M_S[3];
	RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);
	float *R_CTIM_M = pMCTI->R;
	float *t_CTIM_M = pMCTI->t;
	float R_CTIM_S[9], t_CTIM_S[3];

	RVLCOMPTRANSF3D(R_M_S, t_M_S, R_CTIM_M, t_CTIM_M, R_CTIM_S, t_CTIM_S);
	float t_CCTIM_S[3];
	RVLTRANSF3(tMc, R_CTIM_S, t_CTIM_S, t_CCTIM_S);
	double T_CCTIM_S[16];
	RVLHTRANSFMX(pSCTI->R, t_CCTIM_S, T_CCTIM_S);
	
	//Generate scene CTI polydata
	float t[3];
	vtkSmartPointer<vtkPolyData> modelSPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dS, 66, false, validS, t);

	//Getting transform from centered (scene) CTI polygon data to scene (T_CCTIS_S)
	float t_CCTIS_S[3];
	RVLTRANSF3(t, pSCTI->R, pSCTI->t, t_CCTIS_S)
		double T_CCTIS_S[16];
	RVLHTRANSFMX(pSCTI->R, t_CCTIS_S, T_CCTIS_S);


	//PLY Model transformation
	//double T_M_S[16];
	//RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);
	//vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

	//Get T matrix from match; T_ICP matrix is saved as T_S_M * T_ICP (ICP is done in model c.s.)
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	double T_M_S[16], tICPm[3], tm[3];
	//for (int i = 0; i < 16; i++)
	//	T_M_S[i] = pMatch->T_ICP[i];

	if (bICPPose)
	{
		RVLSCALE3VECTOR2(pMatch->tICP, 1000, tICPm);
		RVLHTRANSFMX(pMatch->RICP, tICPm, T_M_S);
	}
	else
	{
		RVLSCALE3VECTOR2(pMatch->t, 1000, tm);
		RVLHTRANSFMX(pMatch->R, tm, T_M_S);
	}

#ifdef RVLPSGM_ICP
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
	{
		//transform->SetMatrix((double*)pMatch->T_ICP);//
		transform->SetMatrix(T_M_S); //when transforming PLY models to scene
	}
#endif

	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
	{
		if (useTG)
		{
			float t_Mc_S[3];

			RVLTRANSF3(tMc, R_M_S, t_M_S, t_Mc_S);

			double T_Mc_S[16];

			RVLHTRANSFMX(R_M_S, t_Mc_S, T_Mc_S);

			transform->SetMatrix(T_Mc_S);
		}
		else
			transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene
	}

#ifdef RVLPSGM_ICP
	//Scaling PLY model to meters
	vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
	transformScale->Scale(0.001, 0.001, 0.001);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iModel));
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();
#endif

	//Transforming PLY model or CTI convex hull model to scene
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
		transformFilter->SetInputData(modelPD); //model CTI convex hull
#ifdef RVLPSGM_ICP
	vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilterScale->GetOutput(), T_M_S); //Generate visible parts of the models pointcloud

	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
		transformFilter->SetInputData(visiblePD); //PLY model
#endif

	transformFilter->SetTransform(transform);
	transformFilter->Update();

	//Transforming scene CTI convex hull
	vtkSmartPointer<vtkTransform> transform2 = vtkSmartPointer<vtkTransform>::New();
	transform2->SetMatrix(T_CCTIS_S);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter2->SetInputData(modelSPD);
	transformFilter2->SetTransform(transform2);
	transformFilter2->Update();

//#ifdef RVLPSGM_ICP
//	//Aligning point clouds (if required) 
//	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterICP;
//	if (align)
//	{
//		//Aligning pointcluds (using PCL ICP)
//		double icpT[16];
//
//		vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilter->GetOutput()); //Generate visible parts of the models pointcloud
//		for (int k = 0; k < 16; k++)
//		{
//			icpT[k] = icpTMatrix[iCluster*nBestMatches * 16 + iRank * 16 + k];
//		}
//
//		//Transforming model polydata to ICP pose
//		vtkSmartPointer<vtkTransform> transformICP = vtkSmartPointer<vtkTransform>::New();
//		transformICP->SetMatrix(icpT);
//		transformFilterICP = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//		transformFilterICP->SetInputData(visiblePD);
//		//transformFilterICP->SetInputConnection(transformFilter->GetOutputPort());
//		transformFilterICP->SetTransform(transformICP);
//		transformFilterICP->Update();
//	}
//#endif


	//Mapper and actor for model
	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//#ifdef RVLPSGM_ICP
//	if (align)
//		modelMapper->SetInputConnection(transformFilterICP->GetOutputPort());
//	else
//#endif
		modelMapper->SetInputConnection(transformFilter->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
	modelActor->SetMapper(modelMapper);
	modelActor->GetProperty()->SetColor(1, 0, 0);
	modelActor->GetProperty()->SetPointSize(3);
	pVisualizer->renderer->AddActor(modelActor);

	//Mapper and actor for scene
	vtkSmartPointer<vtkPolyDataMapper> modelMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper2->SetInputConnection(transformFilter2->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor2 = vtkSmartPointer<vtkActor>::New();
	modelActor2->SetMapper(modelMapper2);
	modelActor2->GetProperty()->SetColor(0, 0, 1);
	pVisualizer->renderer->AddActor(modelActor2);

	delete[] dM;
	delete[] dS;
	delete[] validS;
}

//Vidovic
void PSGM::AddGTModelsToVisualizer(Visualizer *pVisualizer)
{
	//get GT transformation
	vtkSmartPointer<vtkTransform> GTtransform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkTransform> GTTransformScale = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkTransformPolyDataFilter> GTTransformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	vtkSmartPointer<vtkTransformPolyDataFilter> GTtransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	vtkSmartPointer<vtkPolyData> GTVisiblePD = vtkSmartPointer<vtkPolyData>::New();

	//model are in mm
	GTTransformScale->Scale(0.001, 0.001, 0.001);

	int iSegmentGT, iSSegment, iGTM, iGTS, nGTModels;
	double TGT[16], RGT[9];
	RVL::GTInstance *pGT;

	iGTS = iScene - 1;

	pGT = pECCVGT->GT.Element[iGTS].Element;
	nGTModels = pECCVGT->GT.Element[iGTS].n;

	for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
	{
		RVLSCALEMX3X3(pGT->R, 1000, RGT);
		RVLHTRANSFMX(RGT, pGT->t, TGT);

		GTtransform->SetMatrix(TGT); //transform model to GT pose

		GTTransformFilterScale->SetInputData(vtkModelDB.at(pGT->iModel));
		GTTransformFilterScale->SetTransform(GTTransformScale);
		GTTransformFilterScale->Update();

		GTVisiblePD = GetVisiblePart(GTTransformFilterScale->GetOutput(), TGT); //Generate visible parts of the models pointcloud

		GTtransformFilter->SetInputData(GTVisiblePD);
		GTtransformFilter->SetTransform(GTtransform);
		GTtransformFilter->Update();

		//Model mapper for GT model
		vtkSmartPointer<vtkPolyDataMapper> GTmodelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		vtkSmartPointer<vtkActor> GTmodelActor = vtkSmartPointer<vtkActor>::New();

		vtkSmartPointer<vtkPolyData> finPD = vtkSmartPointer<vtkPolyData>::New();
		finPD->DeepCopy(GTtransformFilter->GetOutput());
		
		GTmodelMapper->SetInputData(finPD);
		GTmodelActor->SetMapper(GTmodelMapper);
		GTmodelActor->GetProperty()->SetColor(0, 1, 0);
		GTmodelActor->GetProperty()->SetPointSize(3);
		pVisualizer->renderer->AddActor(GTmodelActor);
	}
}
//END Vidovic

//With ICP calculation
void PSGM::AddOneModelToVisualizerICP(Visualizer *pVisualizer, int iMatch, bool align, RVL::PSGM::ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr)
{
	//Setting indices:
	int iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
	int iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
	int iCluster, iModel;
	
	//Getting scene and model pointers:
	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;
	pMCTI = MCTISet.pCTI.Element[iMCTI];
	pMIE = pMCTI->modelInstance.Element;
	pSCTI = CTISet.pCTI.Element[iSCTI];
	pSIE = pSCTI->modelInstance.Element;

	iCluster = pSCTI->iCluster;
	iModel = pMCTI->iModel;
	
	//Setting descriptors:
	float *dM = new float[66];
	float *dS = new float[66];
	int *validS = new int[66];

	for (int i = 0; i < 66; i++)
	{
		dS[i] = pSIE->d; // Filling scene descriptor
		validS[i] = pSIE->valid;
		pSIE++;

		dM[i] = pMIE->d / 1000.0; // Filling model descriptor
		pMIE++;
	}

	//Getting match pointer and calculating pose:
	RECOG::PSGM_::MatchInstance *pMatch = pCTImatchesArray.Element[iMatch];
	CalculatePose(iMatch);
	Eigen::MatrixXf nT = ConvexTemplatenT();

	//Generate model polydata
	float t[3];
	vtkSmartPointer<vtkPolyData> modelPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dM, 66, false, NULL, t);
	
	//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
	float *R_M_S = pCTImatchesArray.Element[iMatch]->R;
	float *t_M_S_mm = pCTImatchesArray.Element[iMatch]->t;
	float t_M_S[3];
	RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);
	float *R_CTIM_M = pMCTI->R;
	float *t_CTIM_M = pMCTI->t;
	float R_CTIM_S[9], t_CTIM_S[3];

	RVLCOMPTRANSF3D(R_M_S, t_M_S, R_CTIM_M, t_CTIM_M, R_CTIM_S, t_CTIM_S);
	float t_CCTIM_S[3];
	RVLTRANSF3(t, R_CTIM_S, t_CTIM_S, t_CCTIM_S);
	double T_CCTIM_S[16];
	RVLHTRANSFMX(pSCTI->R, t_CCTIM_S, T_CCTIM_S);

	//Generate scene CTI polydata
	vtkSmartPointer<vtkPolyData> modelSPD = GenerateCTIPrimitivePolydata_RW(nT.data(), dS, 66, false, validS, t);
	
	//Getting transform from centered (scene) CTI polygon data to scene (T_CCTIS_S)
	float t_CCTIS_S[3];
	RVLTRANSF3(t, pSCTI->R, pSCTI->t, t_CCTIS_S)
	double T_CCTIS_S[16];
	RVLHTRANSFMX(pSCTI->R, t_CCTIS_S, T_CCTIS_S);


	//PLY Model transformation
	double T_M_S[16];
	RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
		transform->SetMatrix(T_M_S); //when transforming PLY models to scene

	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
		transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene

	//Scaling PLY model to meters
	vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
	transformScale->Scale(0.001, 0.001, 0.001);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iModel));
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();

	//Transforming PLY model or CTI convex hull model to scene
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI)
		transformFilter->SetInputData(modelPD); //model CTI convex hull
	if (displayData.hypothesisVisualizationMode == RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY)
		transformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model

	transformFilter->SetTransform(transform);
	transformFilter->Update();

	//Transforming scene CTI convex hull
	vtkSmartPointer<vtkTransform> transform2 = vtkSmartPointer<vtkTransform>::New();
	transform2->SetMatrix(T_CCTIS_S);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter2->SetInputData(modelSPD);
	transformFilter2->SetTransform(transform2);
	transformFilter2->Update();

	//Aligning point clouds (if required) 
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterICP;
	if (align) 
	{
		//Sampling filter for model polydata
		//vtkSmartPointer<vtkTriangleFilter> modelSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
		//modelSamplerTriangleFilter->SetInputConnection(transformFilter->GetOutputPort());
		//modelSamplerTriangleFilter->Update();
		//vtkSmartPointer<vtkPolyDataPointSampler> modelSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
		//modelSampler->SetInputConnection(modelSamplerTriangleFilter->GetOutputPort());
		//modelSampler->SetDistance(0.005);
		//modelSampler->Update();
		//vtkSmartPointer<vtkPolyData> modelSamplerPD = modelSampler->GetOutput();

		//Sampling filter for scene polydata
		//vtkSmartPointer<vtkTriangleFilter> sceneSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New(); //Creates triangles from polygons (Samples don't work with polygons)
		//sceneSamplerTriangleFilter->SetInputConnection(transformFilter2->GetOutputPort());
		//sceneSamplerTriangleFilter->Update();
		//vtkSmartPointer<vtkPolyDataPointSampler> sceneSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
		//sceneSampler->SetInputConnection(sceneSamplerTriangleFilter->GetOutputPort());
		//sceneSampler->SetDistance(0.005);
		//sceneSampler->Update();
		//vtkSmartPointer<vtkPolyData> sceneSamplerPD = sceneSampler->GetOutput();

		/*vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
		cleanFilter->SetInputConnection(transformFilter->GetOutputPort());
		cleanFilter->PointMergingOn();
		cleanFilter->SetAbsoluteTolerance(0.005);
		cleanFilter->ToleranceIsAbsoluteOn();
		cleanFilter->Update();*/


		//vtkSmartPointer<vtkPolyData> scenePD = GetSceneModelPC(iCluster); //Generate scene model pointcloud
		//Aligning pointcluds (using PCL ICP)
		float icpT[16];
		double icpTd[16];
		double fitnessScore;

		vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilter->GetOutput()); //Generate visible scene model pointcloud
		ICPFunction(visiblePD, /*this->pMesh->pPolygonData*/this->segmentN_PD.at(iCluster), icpT, 30, 0.01, ICPvariant, &fitnessScore, kdTreePtr);
		for (int i = 0; i < 16; i++)
		{
			icpTd[i] = icpT[i];
		}

		//Transforming model polydata to ICP pose
		vtkSmartPointer<vtkTransform> transformICP = vtkSmartPointer<vtkTransform>::New();
		transformICP->SetMatrix(icpTd);
		transformFilterICP = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		transformFilterICP->SetInputData(visiblePD);
		//transformFilterICP->SetInputConnection(transformFilter->GetOutputPort());
		transformFilterICP->SetTransform(transformICP);
		transformFilterICP->Update();
	}

	//Mapper and actor for model
	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	if (align)
		modelMapper->SetInputConnection(transformFilterICP->GetOutputPort());
	else 
		modelMapper->SetInputConnection(transformFilter->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
	modelActor->SetMapper(modelMapper);
	modelActor->GetProperty()->SetColor(1, 0, 0);
	modelActor->GetProperty()->SetPointSize(3);
	pVisualizer->renderer->AddActor(modelActor);
	
	//Mapper and actor for scene
	vtkSmartPointer<vtkPolyDataMapper> modelMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper2->SetInputConnection(transformFilter2->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor2 = vtkSmartPointer<vtkActor>::New();
	modelActor2->SetMapper(modelMapper2);
	modelActor2->GetProperty()->SetColor(0, 0, 1);
	pVisualizer->renderer->AddActor(modelActor2);

	delete[] dM;
	delete[] dS;
	delete[] validS;
}

vtkSmartPointer<vtkPolyData> PSGM::GetSceneModelPC(int iCluster)
{

	RECOG::PSGM_::Cluster *pCluster;
	Surfel *pSurfel;
	RVL::QLIST::Index2 *pt;
	//RECOG::PSGM_::ModelInstance *pModelInstance;
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
	normals->SetNumberOfComponents(3);
	//points->SetDataTypeToDouble();
	vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
	int ptIdx = 0;
	pCluster = clusters.Element[iCluster];
	for (int i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		pSurfel = &this->pSurfels->NodeArray.Element[pCluster->iSurfelArray.Element[i]];
		pt = pSurfel->PtList.pFirst;

		while (pt)
		{
			points->InsertNextPoint(this->pMesh->NodeArray.Element[pt->Idx].P);
			normals->InsertNextTuple(this->pMesh->NodeArray.Element[pt->Idx].N);
			verts->InsertNextCell(1);
			verts->InsertCellPoint(ptIdx);
			ptIdx++;
			pt = pt->pNext;
		}
	}

	vtkSmartPointer<vtkPolyData> PD = vtkSmartPointer<vtkPolyData>::New();
	PD->SetPoints(points);
	PD->GetPointData()->SetNormals(normals);
	PD->SetVerts(verts);
	return PD;
}

void PSGM::CalculateICPCost(RVL::PSGM::ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr)
{
	int iMatch;
	int iMCTI, iSCTI, iCluster, iModel;
	
	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;
	RECOG::PSGM_::MatchInstance *pMatch;

	for (int i = 0; i < scoreMatchMatrix.n; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			iMatch = scoreMatchMatrix.Element[i].Element[j].idx;
			
			//Setting indices:
			iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
			iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
	
			//Getting scene and model pointers:
			pMCTI = MCTISet.pCTI.Element[iMCTI];
			pMIE = pMCTI->modelInstance.Element;
			pSCTI = CTISet.pCTI.Element[iSCTI];
			pSIE = pSCTI->modelInstance.Element;

			iCluster = pSCTI->iCluster;
			iModel = pMCTI->iModel;

			//Getting match pointer and calculating pose:
			pMatch = pCTImatchesArray.Element[iMatch];
			CalculatePose(iMatch);

			//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
			float *R_M_S = pMatch->R;
			float *t_M_S_mm = pMatch->t;
			float t_M_S[3];
			RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);
			
			//PLY Model transformation
			double T_M_S[16];
			RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);
			vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
			transform->SetMatrix(T_M_S); //when transforming PLY models to scene
			//transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene

			//Scaling PLY model to meters
			vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
			transformScale->Scale(0.001, 0.001, 0.001);
			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilterScale->SetInputData(vtkModelDB.at(iModel));
			transformFilterScale->SetTransform(transformScale);
			transformFilterScale->Update();

			//Transforming PLY model or CTI convex hull model to scene
			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//transformFilter->SetInputData(modelPD); //model CTI convex hull
			transformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model
			transformFilter->SetTransform(transform);
			transformFilter->Update();

			
			//Sampling filter for model polydata
			//vtkSmartPointer<vtkTriangleFilter> modelSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
			//modelSamplerTriangleFilter->SetInputConnection(transformFilter->GetOutputPort());
			//modelSamplerTriangleFilter->Update();
			//vtkSmartPointer<vtkPolyDataPointSampler> modelSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
			//modelSampler->SetInputConnection(modelSamplerTriangleFilter->GetOutputPort());
			//modelSampler->SetDistance(0.005);
			//modelSampler->Update();
			//vtkSmartPointer<vtkPolyData> modelSamplerPD = modelSampler->GetOutput();

			//Sampling filter for scene polydata
			//vtkSmartPointer<vtkTriangleFilter> sceneSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New(); //Creates triangles from polygons (Samples don't work with polygons)
			//sceneSamplerTriangleFilter->SetInputConnection(transformFilter2->GetOutputPort());
			//sceneSamplerTriangleFilter->Update();
			//vtkSmartPointer<vtkPolyDataPointSampler> sceneSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
			//sceneSampler->SetInputConnection(sceneSamplerTriangleFilter->GetOutputPort());
			//sceneSampler->SetDistance(0.005);
			//sceneSampler->Update();
			//vtkSmartPointer<vtkPolyData> sceneSamplerPD = sceneSampler->GetOutput();


			vtkSmartPointer<vtkPolyData> scenePD = GetSceneModelPC(iCluster); //Generate scene model pointcloud
			//Aligning pointcluds (using PCL ICP)
			float icpT[16];
			double fitnessScore;
			ICPFunction(transformFilter->GetOutput(), scenePD, icpT, 20, 0.02, ICPvariant, &fitnessScore, kdTreePtr);

			pMatch->cost_ICP = fitnessScore;
			
			//Commented on 14.06.2017 - Vidovic
			//save only final pose in pMatch
			/*memcpy(pMatch->T_ICP, icpT, 16 * sizeof(float));
			
			pMatch->RICP_[0] = pMatch->T_ICP[0];
			pMatch->RICP_[1] = pMatch->T_ICP[1];
			pMatch->RICP_[2] = pMatch->T_ICP[2];
			pMatch->RICP_[3] = pMatch->T_ICP[4];
			pMatch->RICP_[4] = pMatch->T_ICP[5];
			pMatch->RICP_[5] = pMatch->T_ICP[6];
			pMatch->RICP_[6] = pMatch->T_ICP[8];
			pMatch->RICP_[7] = pMatch->T_ICP[9];
			pMatch->RICP_[8] = pMatch->T_ICP[10];

			pMatch->tICP_[0] = pMatch->T_ICP[3];
			pMatch->tICP_[1] = pMatch->T_ICP[7];
			pMatch->tICP_[2] = pMatch->T_ICP[11];

			RVLCOMPTRANSF3D(pMatch->R, pMatch->t, pMatch->RICP_, pMatch->tICP_, pMatch->RICP, pMatch->tICP);*/	

			//Added on 14.06.2017 - Vidovic
			float RICP_[9], tICP_[3];

			RICP_[0] = icpT[0];
			RICP_[1] = icpT[1];
			RICP_[2] = icpT[2];
			RICP_[3] = icpT[4];
			RICP_[4] = icpT[5];
			RICP_[5] = icpT[6];
			RICP_[6] = icpT[8];
			RICP_[7] = icpT[9];
			RICP_[8] = icpT[10];
					   
			tICP_[0] = icpT[3];
			tICP_[1] = icpT[7];
			tICP_[2] = icpT[11];

			RVLCOMPTRANSF3D(pMatch->R, pMatch->t, RICP_, tICP_, pMatch->RICP, pMatch->tICP);

		}
	}
}

vtkSmartPointer<vtkPolyData> PSGM::GetVisiblePart(vtkSmartPointer<vtkPolyData> PD)
{
	vtkSmartPointer<vtkPoints> visiblePoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> visibleNormals = vtkSmartPointer<vtkFloatArray>::New();
	visibleNormals->SetNumberOfComponents(3);
	//points->SetDataTypeToDouble();
	vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyData> visiblePD = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPoints> pdPoints = PD->GetPoints();
	vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(PD->GetPointData()->GetNormals());
	double *point;
	float normal[3];
	int ptIdx = 0;

	if (!normals.GetPointer()) //check if mode does not have normals 
	{
		printf("Invalid input model. Doesn't have normals.\n");
		return NULL;
	}

	for (int i = 0; i < pdPoints->GetNumberOfPoints(); i++)
	{
		point = pdPoints->GetPoint(i);
		normals->GetTupleValue(i, normal);
		if ((point[0] * normal[0] + point[1] * normal[1] + point[2] * normal[2]) < 0) //cheks the scalar product, must be negative
		{
			visiblePoints->InsertNextPoint(point);
			visibleNormals->InsertNextTuple(normal);
			verts->InsertNextCell(1);
			verts->InsertCellPoint(ptIdx);
			ptIdx++;
		}
	}
	visiblePD->SetPoints(visiblePoints);
	visiblePD->GetPointData()->SetNormals(visibleNormals);
	visiblePD->SetVerts(verts);
	return visiblePD;
}

//Vidovic
vtkSmartPointer<vtkPolyData> PSGM::GetVisiblePart(vtkSmartPointer<vtkPolyData> PD, double *T_M_S)
{
	vtkSmartPointer<vtkPoints> pdPoints = PD->GetPoints();
	vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(PD->GetPointData()->GetNormals());

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(T_M_S); //transformation from model c.s. to scene c.s.

	//Transforming PLY model or CTI convex hull model to scene
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(PD); //PLY model
	transformFilter->SetTransform(transform);
	transformFilter->Update();

	vtkSmartPointer<vtkPoints> visiblePoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> visibleNormals = vtkSmartPointer<vtkFloatArray>::New();
	visibleNormals->SetNumberOfComponents(3);
	//points->SetDataTypeToDouble();
	vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyData> visiblePD = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPoints> transformedPoints = transformFilter->GetOutput()->GetPoints();
	vtkSmartPointer<vtkFloatArray> transformedNormals = vtkFloatArray::SafeDownCast(transformFilter->GetOutput()->GetPointData()->GetNormals());
	double *transformedPoint;
	float transformedNormal[3], normal[3];
	int ptIdx = 0;

	if (!normals.GetPointer()) //check if mode does not have normals 
	{
		printf("Invalid input model. Doesn't have normals.\n");
		return NULL;
	}

	for (int i = 0; i < transformedPoints->GetNumberOfPoints(); i++)
	{
		transformedPoint = transformedPoints->GetPoint(i);
		transformedNormals->GetTupleValue(i, transformedNormal);
		if ((transformedPoint[0] * transformedNormal[0] + transformedPoint[1] * transformedNormal[1] + transformedPoint[2] * transformedNormal[2]) < 0) //cheks the scalar product, must be negative
		{
			normals->GetTupleValue(i, normal);

			visiblePoints->InsertNextPoint(pdPoints->GetPoint(i));
			visibleNormals->InsertNextTuple(normal);
			verts->InsertNextCell(1);
			verts->InsertCellPoint(ptIdx);
			ptIdx++;
		}
	}
	visiblePD->SetPoints(visiblePoints);
	visiblePD->GetPointData()->SetNormals(visibleNormals);
	visiblePD->SetVerts(verts);
	return visiblePD;
}
//END Vidovic

void PSGM::CalculateNNCost(Visualizer *pVisualizer, RVL::PSGM::ICPfunction ICPFunction, int ICPvariant)
{
	int iMatch;
	int iMCTI, iSCTI, iCluster, iModel;

	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;

	RECOG::PSGM_::MatchInstance *pMatch;

	if (icpTMatrix)
		delete[] icpTMatrix;

	icpTMatrix = new double[scoreMatchMatrix.n * nBestMatches * 16]; //nSSegments * nBestMatches * 16 elements of matrix T

	for (int i = 0; i < scoreMatchMatrix.n; i++)
	{
		for (int j = 0; j < nBestMatches; j++)
		{
			iMatch = scoreMatchMatrix.Element[i].Element[j].idx;

			if (iMatch == 39597)
				int debug = 0;

			if (iMatch != -1)
			{
			//Setting indices:
			iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
			iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;

			//Getting scene and model pointers:
			pMCTI = MCTISet.pCTI.Element[iMCTI];
			pMIE = pMCTI->modelInstance.Element;
			pSCTI = CTISet.pCTI.Element[iSCTI];
			pSIE = pSCTI->modelInstance.Element;

			iCluster = pSCTI->iCluster;
			iModel = pMCTI->iModel;

			//Getting match pointer and calculating pose:
			pMatch = pCTImatchesArray.Element[iMatch];
			CalculatePose(iMatch);

			//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
			float *R_M_S = pMatch->R;
			float *t_M_S_mm = pMatch->t;
			float t_M_S[3];
			RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);

			//PLY Model transformation
			double T_M_S[16], eyeT[16];
			RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);

			//Without transforming model points to scene c.s.
			eyeT[0] = 1; eyeT[1] = 0; eyeT[2] = 0; eyeT[3] = 0;
			eyeT[4] = 0; eyeT[5] = 1; eyeT[6] = 0; eyeT[7] = 0;
			eyeT[8] = 0; eyeT[9] = 0; eyeT[10] = 1; eyeT[11] = 0;
			eyeT[12] = 0; eyeT[13] = 0; eyeT[14] = 0; eyeT[15] = 1;

			vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
			transform->SetMatrix(T_M_S); //when transforming PLY models to scene
			//transform->SetMatrix(eyeT); //when transforming PLY models to scene
			//transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene

			//Scaling PLY model to meters
			vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
			transformScale->Scale(0.001, 0.001, 0.001);
			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilterScale->SetInputData(vtkModelDB.at(iModel));
			transformFilterScale->SetTransform(transformScale);
			transformFilterScale->Update();

			//Transforming PLY model or CTI convex hull model to scene
			//vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			////transformFilter->SetInputData(modelPD); //model CTI convex hull
			//transformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model
			//transformFilter->SetTransform(transform);
			//transformFilter->Update();


			//Sampling filter for model polydata
			//vtkSmartPointer<vtkTriangleFilter> modelSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
			//modelSamplerTriangleFilter->SetInputConnection(transformFilter->GetOutputPort());
			//modelSamplerTriangleFilter->Update();
			//vtkSmartPointer<vtkPolyDataPointSampler> modelSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
			//modelSampler->SetInputConnection(modelSamplerTriangleFilter->GetOutputPort());
			//modelSampler->SetDistance(0.005);
			//modelSampler->Update();
			//vtkSmartPointer<vtkPolyData> modelSamplerPD = modelSampler->GetOutput();

			//Sampling filter for scene polydata
			//vtkSmartPointer<vtkTriangleFilter> sceneSamplerTriangleFilter = vtkSmartPointer<vtkTriangleFilter>::New(); //Creates triangles from polygons (Samples don't work with polygons)
			//sceneSamplerTriangleFilter->SetInputConnection(transformFilter2->GetOutputPort());
			//sceneSamplerTriangleFilter->Update();
			//vtkSmartPointer<vtkPolyDataPointSampler> sceneSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
			//sceneSampler->SetInputConnection(sceneSamplerTriangleFilter->GetOutputPort());
			//sceneSampler->SetDistance(0.005);
			//sceneSampler->Update();
			//vtkSmartPointer<vtkPolyData> sceneSamplerPD = sceneSampler->GetOutput();


			//vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilter->GetOutput()); //Generate visible scene model pointcloud - points are in scene c.s.
			vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilterScale->GetOutput(), T_M_S); //Generate visible scene model pointcloud - points are in model c.s.

			//Aligning pointcluds (using PCL ICP)
			float icpT[16];
			double icpTd[16];
			double fitnessScore;
			
			icpT[0] = 1; icpT[1] = 0; icpT[2] = 0; icpT[3] = 0;
			icpT[4] = 0; icpT[5] = 1; icpT[6] = 0; icpT[7] = 0;
			icpT[8] = 0; icpT[9] = 0; icpT[10] = 1; icpT[11] = 0;
			icpT[12] = 0; icpT[13] = 0; icpT[14] = 0; icpT[15] = 1;

			//Transforming scene points to model c.s.
			float R_S_M[9], t_S_M[3];
			double T_S_M[16];
			
			RVLINVTRANSF3D(R_M_S, t_M_S, R_S_M, t_S_M);
			RVLHTRANSFMX(R_S_M, t_S_M, T_S_M);

			vtkSmartPointer<vtkTransform> S_M_transform = vtkSmartPointer<vtkTransform>::New();
			S_M_transform->SetMatrix(T_S_M); //when transforming PLY models to scene

			vtkSmartPointer<vtkTransformPolyDataFilter> sceneTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//sceneTransformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model
			sceneTransformFilter->SetInputData(this->segmentN_PD.at(iCluster)); 
			sceneTransformFilter->SetTransform(S_M_transform);
			sceneTransformFilter->Update();
			
			//ICPFunction(visiblePD, /*this->pMesh->pPolygonData*/this->segmentN_PD.at(iCluster), icpT, 10, 0.01, ICPvariant, &fitnessScore, NULL); //ICP in scene c.s.

			ICPFunction(visiblePD, /*this->pMesh->pPolygonData*/sceneTransformFilter->GetOutput(), icpT, 10, 0.01, ICPvariant, &fitnessScore, NULL); //ICP in model c.s.

			//VTK ICP
			//vtkSmartPointer<vtkIterativeClosestPointTransform> vtkicp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
			//vtkicp->SetSource(visiblePD); //Ulazni objekt (poèetna poza objekta)
			//vtkicp->SetTarget(this->segmentN_PD.at(iCluster)); //Konaèni objekt (željena poza objekta)
			//vtkicp->GetLandmarkTransform()->SetModeToRigidBody(); //Potrebni naèin rada je transformacija za kruta tijela
			//vtkicp->SetMaximumNumberOfIterations(10); //Željeni broj iteracija
			//vtkicp->SetMaximumNumberOfLandmarks(1000); //Koliko parova toèaka da se koristi prilikom minimiziranja cost funkcije
			//vtkicp->Update(); //Provedi algoritam
			//vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
			//vtkicp->GetMatrix(m);
			//std::cout << "Matrica:" << *m << std::endl;
			//END VTK ICP

			//TEST Vidovic
			float R_ICP[9], t_ICP[3];
			//float R_ICP_inv[9], t_ICP_inv[3];
			float R_ICP_S[9], t_ICP_S[3];
			float icpT2[16];

			R_ICP[0] = icpT[0]; R_ICP[1] = icpT[1]; R_ICP[2] = icpT[2];
			R_ICP[3] = icpT[4]; R_ICP[4] = icpT[5]; R_ICP[5] = icpT[6];
			R_ICP[6] = icpT[8]; R_ICP[7] = icpT[9]; R_ICP[8] = icpT[10];

			t_ICP[0] = icpT[3]; t_ICP[1] = icpT[7]; t_ICP[2] = icpT[11];

			//RVLINVTRANSF3D(R_ICP, t_ICP, R_ICP_inv, t_ICP_inv);
			//RVLCOMPTRANSF3D(R_M_S, t_M_S, R_ICP_inv, t_ICP_inv, R_ICP_S, t_ICP_S);
			RVLCOMPTRANSF3D(R_M_S, t_M_S, R_ICP, t_ICP, R_ICP_S, t_ICP_S);
			RVLHTRANSFMX(R_ICP_S, t_ICP_S, icpT2);

			double icpT2d[16];
			//END test Vidovic

			if (iCluster == 2 && j == 1)
				int debug = 0;

				for (int k = 0; k < 16; k++)
			{
				//icpT[k] = vtkicp->GetMatrix()->GetElement(k / 4, k % 4); //VTK ICP
				
				//if (iCluster == 2 && j == 1){
				//	icpT[0] = 1; icpT[1] = 0; icpT[2] = 0; icpT[3] = 0;
				//	icpT[4] = 0; icpT[5] = 1; icpT[6] = 0; icpT[7] = 0;
				//	icpT[8] = 0; icpT[9] = 0; icpT[10] = 1; icpT[11] = 0;
				//	icpT[12] = 0; icpT[13] = 0; icpT[14] = 0; icpT[15] = 1;
				//}

					icpTd[k] = icpT[k];			
					//save icpT to PSGM class
				icpTMatrix[i*nBestMatches * 16 + j * 16 + k] = icpT2[k];
				icpT2d[k] = (double)icpT2[k];
			}

			//Transforming model polydata to ICP pose
			vtkSmartPointer<vtkTransform> transformICP = vtkSmartPointer<vtkTransform>::New();
			transformICP->SetMatrix(icpTd);

			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterICP = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//transformFilterICP->SetInputData(visiblePD); //VIDOVIC
			transformFilterICP->SetInputData(transformFilterScale->GetOutput()); //VIDOVIC transformFilter->GetOutput()
			//transformFilterICP->SetTransform(vtkicp); //VTK ICP
			transformFilterICP->SetTransform(transformICP);	//PCL ICP
			transformFilterICP->Update();

			//calculate new visible part of models (using ICP pose) - VIDOVIC
			//visiblePD = GetVisiblePart(transformFilterICP->GetOutput()); ////Generate visible scene model pointcloud - points are in scene c.s.
			//visiblePD = GetVisiblePart(transformFilterICP->GetOutput(), T_M_S); ////Generate visible scene model pointcloud - points are in model c.s.
			visiblePD = GetVisiblePart(transformFilterICP->GetOutput(), icpT2d); ////Generate visible scene model pointcloud - points are in model c.s.

			//for visualisation only - Vidovic
			//transformFilterICP->SetTransform(transform);
			//transformFilterICP->Update();

				//if visualization is needed right here:

				//Mapper and actor for model
				/*vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
				modelMapper->SetInputConnection(transformFilterICP->GetOutputPort());
				vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
				modelActor->SetMapper(modelMapper);
				modelActor->GetProperty()->SetColor(0, 1, 0);
				modelActor->GetProperty()->SetPointSize(3);

				if (iCluster == 2 && j == 1)
				{
					pVisualizer->renderer->AddActor(modelActor);
				}*/


			vtkSmartPointer<vtkTransformPolyDataFilter> sceneSegmentTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			sceneSegmentTransformFilter->SetInputData(GetSceneModelPC(iCluster));
			sceneSegmentTransformFilter->SetTransform(S_M_transform);
			sceneSegmentTransformFilter->Update();


			//pMatch->cost_NN = NNCost(iCluster, transformFilterICP->GetOutput(), RVLPSGM_ICP_SIMILARITY_MEASURE_SATURATED_SCORE); //VIDOVIC
			//pMatch->cost_NN = NNCost(iCluster, sceneTransformFilter->GetOutput(), visiblePD, RVLPSGM_ICP_SIMILARITY_MEASURE_SATURATED_SCORE); //VIDOVIC - cost for points in the neighbourhood of segment
			pMatch->cost_NN = NNCost(iCluster, sceneTransformFilter->GetOutput(), visiblePD, RVLPSGM_ICP_SIMILARITY_MEASURE_COSTNN); //VIDOVIC - cost for points in the neighbourhood of segment
			//pMatch->cost_NN = NNCost(iCluster, sceneSegmentTransformFilter->GetOutput(), visiblePD, RVLPSGM_ICP_SIMILARITY_MEASURE_COSTNN); //VIDOVIC - cost for points in the neighbourhood of segment
			//pMatch->cost_NN = NNCost(iCluster, sceneSegmentTransformFilter->GetOutput(), visiblePD, RVLPSGM_ICP_SIMILARITY_MEASURE_SATURATED_SCORE); //VIDOVIC - cost for segment points

			//penalize the distance from the gound plane - Vidovic
			float tConst = 30.0;
			float tDistance;

			tDistance = groundPlaneDistance(iModel, icpT2d);

			//if (iCluster == 2 && (iModel == 12 || iModel == 29))
			//	printf("\nDistance of model %d from the ground plane is: %f\n (cost: %f; cost penalty: %f; total: %f;)", iModel, tDistance, pMatch->cost_NN, tConst * RVLABS(groundPlaneDistance(iModel, icpT2d)) * pMatch->cost_NN, pMatch->cost_NN + tConst * RVLABS(groundPlaneDistance(iModel, icpT2d)) * pMatch->cost_NN);


			//pMatch->cost_NN += tConst * groundPlaneDistance(iModel, icpT2d);
			pMatch->gndDistance = groundPlaneDistance(iModel, icpT2d);

#ifdef RVLPSGM_GROUND_PLANE_DISTANCE_PENALIZATION
			pMatch->cost_NN += tConst * RVLABS(pMatch->gndDistance) * pMatch->cost_NN;
#endif

				//memcpy(pMatch->T_ICP, icpT2, 16 * sizeof(float));

				/*pMatch->RICP_[0] = pMatch->T_ICP[0];
				pMatch->RICP_[1] = pMatch->T_ICP[1];
				pMatch->RICP_[2] = pMatch->T_ICP[2];
				pMatch->RICP_[3] = pMatch->T_ICP[4];
				pMatch->RICP_[4] = pMatch->T_ICP[5];
				pMatch->RICP_[5] = pMatch->T_ICP[6];
				pMatch->RICP_[6] = pMatch->T_ICP[8];
				pMatch->RICP_[7] = pMatch->T_ICP[9];
				pMatch->RICP_[8] = pMatch->T_ICP[10];

				pMatch->tICP_[0] = pMatch->T_ICP[3] * 1000;
				pMatch->tICP_[1] = pMatch->T_ICP[7] * 1000;
				pMatch->tICP_[2] = pMatch->T_ICP[11] * 1000;*/

				pMatch->RICP[0] = icpT2[0];
				pMatch->RICP[1] = icpT2[1];
				pMatch->RICP[2] = icpT2[2];
				pMatch->RICP[3] = icpT2[4];
				pMatch->RICP[4] = icpT2[5];
				pMatch->RICP[5] = icpT2[6];
				pMatch->RICP[6] = icpT2[8];
				pMatch->RICP[7] = icpT2[9];
				pMatch->RICP[8] = icpT2[10];

				pMatch->tICP[0] = icpT2[3] * 1000;
				pMatch->tICP[1] = icpT2[7] * 1000;
				pMatch->tICP[2] = icpT2[11] * 1000;

				
			}
			else break;

		}
	}

}

float PSGM::NNCost(int iCluster, vtkSmartPointer<vtkPolyData> sourcePD, vtkSmartPointer<vtkPolyData> targetPD, int similarityMeasure)
{
	//vtkSmartPointer<vtkPolyData> sourcePD = this->segmentN_PD.at(iCluster);//GetSceneModelPC(iCluster); //Generate scene model pointcloud
	NanoFlannPointCloud<float> targetPC;
	vtkSmartPointer<vtkPoints> pdPoints = targetPD->GetPoints();
	targetPC.pts.resize(pdPoints->GetNumberOfPoints());
	double *point;
	int i;

	for (i = 0; i < pdPoints->GetNumberOfPoints(); i++)
	{
		point = pdPoints->GetPoint(i);
		targetPC.pts.at(i).x = point[0];
		targetPC.pts.at(i).y = point[1];
		targetPC.pts.at(i).z = point[2];
	}

	nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, NanoFlannPointCloud<float> >, NanoFlannPointCloud<float>, 3> index(3 /*dim*/, targetPC, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	index.buildIndex();

	vtkSmartPointer<vtkPoints> sourcePoints = sourcePD->GetPoints();
	float pointF[3];

	std::vector<size_t>   ret_index(1);
	std::vector<float> out_dist_sqr(1);
	float costNN = 0;
	float score = 0, distance;
	int br = 0;

	switch (similarityMeasure)
	{
	case RVLPSGM_ICP_SIMILARITY_MEASURE_SATURATED_SCORE:

		for (i = 0; i < sourcePoints->GetNumberOfPoints(); i++)
		{
			point = sourcePoints->GetPoint(i);
			pointF[0] = point[0];
			pointF[1] = point[1];
			pointF[2] = point[2];

			index.knnSearch(pointF, 1, &ret_index[0], &out_dist_sqr[0]);
			distance = out_dist_sqr.at(0) / 0.0001; //0.0001 = 0.01*0.01

			if (distance < 1)
				score += 1 - distance;
		}

		//if (iCluster == 3)
		//	printf("Broj toèaka: %d, Score: %f\n", sourcePoints->GetNumberOfPoints(), score);

		return score / sourcePoints->GetNumberOfPoints();

	default:

		for (i = 0; i < sourcePoints->GetNumberOfPoints(); i++)
		{
			point = sourcePoints->GetPoint(i);
			pointF[0] = point[0];
			pointF[1] = point[1];
			pointF[2] = point[2];

			index.knnSearch(pointF, 1, &ret_index[0], &out_dist_sqr[0]);
			if (sqrt(out_dist_sqr.at(0)) > 0.01)
			{
				costNN += 0.01;
				br++;
			}
			else costNN += sqrt(out_dist_sqr.at(0));
		}

		float meanCost = costNN / i;
		return costNN;
	}
}


void PSGM::RVLPSGInstanceMesh(Eigen::MatrixXf nI, float *dI)
{
#ifdef RVLPSGM_CTIMESH_DEBUG
	FILE *fp = fopen("CTIMeshDebug.txt", "w");
#endif

	//transform nI and dI to Eigen:
	Eigen::MatrixXf nIE = nI;
	Eigen::MatrixXf dIE(1, 66);

	//if nI was an array
	//Eigen::MatrixXf nIE(3, 66);
	//int br = 0;
	//for (int i = 0; i < 3; i++)
	//{
	//	for (int j = 0; j < 66; j++)
	//	{
	//		nIE(i, j) = nI[br];
	//		br++;
	//	}
	//}
	for (int i = 0; i < 66; i++)
	{
		dIE(0, i) = dI[i];
	}

	float noise = 1e-6;
	int nF = nIE.cols();
	float halfCubeSize;

	float max, min;
	max = dIE.maxCoeff();
	min = dIE.minCoeff();
	if (min<0 && min*-1 > max)
		max = -1 * min;
	halfCubeSize = max*1.1;

	P.resize(3, 8);
	P << 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1;
	P = halfCubeSize*P;

	Eigen::MatrixXi Premoved = Eigen::MatrixXi::Zero(1, P.cols());//list of removed vertices
	Eigen::MatrixXi nP = Eigen::MatrixXi::Ones(nF + 6, 1);
	nP = 4 * nP;

	F = Eigen::MatrixXi::Zero(nF + 6, 66 * 66);
	//F.block<6, 4>(0, 0) << 1, 3, 4, 2, 3, 7, 8, 4, 2, 4, 8, 6, 5, 6, 8, 7, 1, 2, 6, 5, 1, 5, 7, 3;
	F.block<6, 4>(0, 0) << 0, 2, 3, 1, 2, 6, 7, 3, 1, 3, 7, 5, 4, 5, 7, 6, 0, 1, 5, 4, 0, 4, 6, 2;


	Eigen::MatrixXi E = Eigen::MatrixXi::Ones(nF + 6, nF + 6);
	E *= -1;
	Eigen::MatrixXi Fn = Eigen::MatrixXi::Zero(nF + 6, 66 * 66);

	int iP1, iP2;
	int l;
	int br2;
	int NextCirc, PrevCirc;
	for (int i_ = 0; i_ < 6; i_++) //for every face
	{
		int i, j;
		i = i_;
		for (int k = 0; k < 4; k++)
		{
			NextCirc = (k + 1) % 4;
			iP1 = F(i, k);
			iP2 = F(i, NextCirc);

			for (int j_ = i_ + 1; j_ < 6; j_++)
			{
				j = j_;
				l = -1;
				for (int iF = 0; iF < nP(j); iF++) //find(F(j,:)==iP1)
				{
					if (F(j, iF) == iP1)
					{
						l = iF;
						break;
					}
				}
				if (l >= 0)
				{
					PrevCirc = (l + 3) % 4;
					if (F(j, PrevCirc) == iP2)
					{
						E(i, j) = iP1;
						E(j, i) = iP2;
						Fn(i, k) = j;
						Fn(j, PrevCirc) = i;
					}
				}
			}
		}
	}

#ifdef RVLPSGM_CTIMESH_DEBUG
	PrintCTIMeshFaces(fp, F, Fn, 6, nP);

	fprintf(fp, "\n\n\n");

	fclose(fp);
#endif

	Eigen::MatrixXi iNewVertices;
	Eigen::MatrixXi iNeighbors;
	Eigen::MatrixXf N;
	Eigen::MatrixXf dCut, dCutSorted;
	float d, d_;
	int nCut;
	Eigen::MatrixXf Temp;
	Eigen::MatrixXf Tempnext;
	Eigen::MatrixXf Temp2;
	for (int i = 0; i < nF; i++) //for every face
	{
		printf("%d\n", i);

#ifdef RVLPSGM_CTIMESH_DEBUG
		fprintf(fp, "i = %d\n\n\n", i);
#endif
		iNewVertices.resize(0, 0);
		iNeighbors.resize(0, 0);
		N = nIE.block<3, 1>(0, i); //normal of the i-th face
		d = dIE(0, i); //distance of the i-th face
		dCut = Eigen::MatrixXf::Zero(P.cols(), 1);
		dCutSorted = Eigen::MatrixXf::Zero(P.cols(), 1);
		nCut = 0;


		for (int k = 0; k < 8; k++)
		{
			Temp = N.transpose()*P.block<3, 1>(0, k);
			d_ = Temp(0, 0) - d;
			if (d_ > 0)
			{
				nCut += 1;
				dCut(nCut, 0) = d_;
			}
		}

		//Bubble sort:
		float temp;
		for (int idCut = 0; idCut < nCut; idCut++)
		{
			for (int jdCut = 0; jdCut < nCut; jdCut++)
			{
				if (dCut(jdCut, 0)>dCut(jdCut + 1, 0))
				{
					temp = dCut(jdCut, 0);
					dCut(jdCut, 0) = dCut(jdCut + 1, 0);
					dCut(jdCut + 1, 0) = temp;
				}
			}
		}

		float dCorr = 0;
		for (int k = 0; k < nCut; k++)
		{
			if (dCut(k, 0) - dCorr < noise)
				dCorr = dCut(k, 0);

		}
		d += dCorr;

		Eigen::MatrixXi F_;//j-th face
		Eigen::MatrixXi Fn_;//neighbors of F_
		Eigen::MatrixXf P_; //position vector of vector iP
		Eigen::MatrixXf Pnext;//position vector of vertex iPNext
		int nP_; //number of vertices od F_
		int iP; // k-th vertex of F_
		int iPNext; //next vertex
		Eigen::MatrixXf dP;
		int L; //neighbor of F_ on the opposite side of edge iP-iPNext

		int iPolygon, iVertex;
		int iPNew, iPNewVertex, iFNewVertex;
		float s;


		for (int j = 0; j <= i + 5; j++) //for every previously considered face
		{
			F_ = F.block(j, 0, 1, F.cols());
			Fn_ = Fn.block(j, 0, 1, Fn.cols());
			int ff = F(j, 0);
			int k = 0;
			nP_ = nP(j, 0);

			for (int k_ = 0; k_ < nP_; k_++)
			{
				int p = P.cols();
				iP = F_(0, k_);
				P_ = P.block(0, iP, P.rows(), 1);


				if (k_ == nP_ - 1) NextCirc = 0;
				else NextCirc = k_ % (nP_)+1;

				iPNext = F_(0, NextCirc);
				Pnext = P.block(0, iPNext, P.rows(), 1);

				dP.resize(P.rows(), 1);
				dP = Pnext - P_;

				L = Fn_(0, k_);

				Temp = N.transpose()*P_;
				Tempnext = N.transpose()*Pnext;

				if (i == 1 && j == 2)
					int debug = 1;


				if (Temp(0, 0) > d) //if iP is over new plane
				{
					if (Premoved(0, iP) == 0)
						Premoved(0, iP) = 1; //vertex iP is removed

					iPolygon = j;
					iVertex = k;

					//Remove Vertex from Polygon:
					for (int iF = 0; iF < (nP(iPolygon) - 1 - iVertex); iF++)
					{
						F(iPolygon, iVertex + iF) = F(iPolygon, iVertex + iF + 1);
						Fn(iPolygon, iVertex + iF) = Fn(iPolygon, iVertex + iF + 1);
					}
					nP(iPolygon) = nP(iPolygon) - 1;
#ifdef RVLPSGM_CTIMESH_DEBUG
					fp = fopen("CTIMeshDebug.txt", "a");

					fprintf(fp, "j = %d, k_ = %d\n\n", j, k_);

					PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

					fprintf(fp, "\n\n\n");

					fclose(fp);
#endif

					if (Tempnext(0, 0) > d)
					{
						E(L, j) = -1;
						E(j, L) = -1;
						k -= 1;
					}
					else
					{
						if (E(j, L) == iP) //Vertex is not updated
						{
							//Add new vertex
							Temp = N.transpose()*P_;
							Temp2 = N.transpose()*dP;
							s = (d - Temp(0, 0)) / Temp2(0, 0);
							Eigen::MatrixXf Ptemp = P;
							P.resize(P.rows(), P.cols() + 1);
							P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
							Eigen::Vector3f col = P_ + s*dP;
							P.col(P.cols() - 1) = col;
							iPNew = P.cols() - 1;

							Eigen::MatrixXi Premovedtemp = Premoved;
							Premoved.resize(1, Premoved.cols() + 1);
							Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
							Premoved(0, Premoved.cols() - 1) = 0;
							E(j, L) = iPNew;
						}
						else
							iPNew = E(j, L);

						iPolygon = j;
						iVertex = k;
						iPNewVertex = iPNew;
						iFNewVertex = L;
						//Add new Vertex to Polygon:
						Eigen::MatrixXi Ftemp = F;
						Eigen::MatrixXi Fntemp = Fn;
						for (int iF = 0; iF < (nP(iPolygon) - iVertex); iF++)
						{
							F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon, iVertex + iF);
							Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
						}
						F(iPolygon, iVertex) = iPNewVertex;
						Fn(iPolygon, iVertex) = iFNewVertex;
						nP(iPolygon) = nP(iPolygon) + 1;
#ifdef RVLPSGM_CTIMESH_DEBUG
						fp = fopen("CTIMeshDebug.txt", "a");

						fprintf(fp, "j = %d, k_ = %d\n\n", j, k_);

						PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

						fprintf(fp, "\n\n\n");

						fclose(fp);
#endif

						Eigen::MatrixXi iNewVerticestemp = iNewVertices;
						iNewVertices.resize(1, iNewVertices.cols() + 1);
						iNewVertices.block(0, 0, iNewVerticestemp.rows(), iNewVerticestemp.cols()) = iNewVerticestemp;
						iNewVertices(0, iNewVertices.cols() - 1) = iPNew;


						Eigen::MatrixXi iNeighborstemp = iNeighbors;
						iNeighbors.resize(1, iNeighbors.cols() + 1);
						iNeighbors.block(0, 0, iNeighborstemp.rows(), iNeighborstemp.cols()) = iNeighborstemp;
						iNeighbors(0, iNeighbors.cols() - 1) = j;

						E((i + 6), j) = iPNew;
					}
				}

				else if (Tempnext(0, 0) > d)
				{
					if (E(L, j) == iPNext) //Vertex is not updated
					{
						Temp = N.transpose()*P_;
						Temp2 = N.transpose()*dP;
						s = (d - Temp(0, 0)) / Temp2(0, 0);


						Eigen::MatrixXf Ptemp = P;
						P.resize(P.rows(), P.cols() + 1);
						P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
						Eigen::Vector3f col = P_ + s*dP;
						P.col(P.cols() - 1) = col;
						iPNew = P.cols() - 1;

						Eigen::MatrixXi Premovedtemp = Premoved;
						Premoved.resize(1, Premoved.cols() + 1);
						Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
						Premoved(0, Premoved.cols() - 1) = 0;

						E(L, j) = iPNew;
					}
					else
						iPNew = E(L, j);

					iPolygon = j;
					iVertex = k + 1;
					iPNewVertex = iPNew;
					iFNewVertex = i + 6;
					//Add new Vertex to Polygon:
					Eigen::MatrixXi Ftemp = F;
					Eigen::MatrixXi Fntemp = Fn;
					for (int iF = 0; iF < (nP(iPolygon) - iVertex); iF++)
					{
						F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon, iVertex + iF);
						Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
					}
					F(iPolygon, iVertex) = iPNewVertex;
					Fn(iPolygon, iVertex) = iFNewVertex;
					nP(iPolygon) = nP(iPolygon) + 1;
#ifdef RVLPSGM_CTIMESH_DEBUG
					fp = fopen("CTIMeshDebug.txt", "a");

					fprintf(fp, "j = %d, k_ = %d\n\n", j, k_);

					PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

					fprintf(fp, "\n\n\n");

					fclose(fp);
#endif

					E(j, i + 6) = iPNew;
					k = k + 1;
				}
				k = k + 1;
			}

#ifdef RVLPSGM_CTIMESH_DEBUG
			fp = fopen("CTIMeshDebug.txt", "a");

			fprintf(fp, "j = %d\n\n", j);

			PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

			fprintf(fp, "\n\n\n");

			fclose(fp);
#endif
		}	 //for every previously considered face
		int iNeighbor;
		nP(i + 6) = iNewVertices.cols(); //rows

		int m;
		if (nP(i + 6) > 0)
		{
			Eigen::MatrixXi F_;
			Eigen::MatrixXi Fn_;
			m = 0;
			while (1)
			{
				Eigen::MatrixXi F_temp = F_;
				F_.resize(1, F_.cols() + 1);
				F_.block(0, 0, F_temp.rows(), F_temp.cols()) = F_temp;
				F_(0, F_.cols() - 1) = iNewVertices(m);

				int iN = iNeighbors(0, m);
				iNeighbor = iNeighbors(0, m);


				Eigen::MatrixXi Fn_temp = Fn_;
				Fn_.resize(1, Fn_.cols() + 1);
				Fn_.block(0, 0, Fn_temp.rows(), Fn_temp.cols()) = Fn_temp;
				Fn_(0, Fn_.cols() - 1) = iNeighbor;


				iPNext = E(iNeighbor, (i + 6));
				int iNV;
				for (iNV = 0; iNV < iNewVertices.cols(); iNV++)
				{
					int a = iPNext;
					int b = iNewVertices(iNV);
					if (iNewVertices(iNV) == iPNext)
					{
						m = iNV;
						break;
					}
				}
				if (m == 0)
					break;
			}

			for (int iF = 0; iF < F_.cols(); iF++)
			{
				F((i + 6), iF) = F_(0, iF);
				Fn((i + 6), iF) = Fn_(0, iF);
			}

		}

#ifdef RVLPSGM_CTIMESH_DEBUG
		fp = fopen("CTIMeshDebug.txt", "a");

		PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

		fprintf(fp, "\n\n\n");

		fclose(fp);
#endif
	}	 //for every face
	int mF = F.cols();

	for (int i = 0; i < F.rows(); i++)
	{
		for (int iF = 0; iF < (mF-nP(i)-1); iF++)
			F(i, nP(i) + 1 + iF) = 0;
	}

	Eigen::MatrixXi Ftemp = F;
	F.resize(Ftemp.rows() - 7, Ftemp.cols());
	F = Ftemp.block(6, 0, Ftemp.rows() - 7, Ftemp.cols());
	Edges = E;
	printf ("Finished.");
#ifdef RVLPSGM_CTIMESH_DEBUG
	fclose(fp);
#endif
}


void PSGM::PrintCTIMeshFaces(FILE *fp, Eigen::MatrixXi F, Eigen::MatrixXi Fn, int n, Eigen::MatrixXi nP)
{
	fprintf(fp, "F:\n");

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < nP(i); j++)
			fprintf(fp, "%d\t", F(i, j));

		fprintf(fp, "\n");
	}

	fprintf(fp, "\n");

	fprintf(fp, "Fn:\n");

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < nP(i); j++)
			fprintf(fp, "%d\t", Fn(i, j));

		fprintf(fp, "\n");
	}

	fprintf(fp, "\n");
}

//with Eigen
//void PSGM::RVLPSGInstanceMesh(Eigen::MatrixXf nI, float *dI)
//{
//	//transform nI and dI to Eigen:
//	Eigen::MatrixXf nIE = nI;
//	Eigen::MatrixXf dIE(1, 66);
//
//	//if nI was an array
//	//Eigen::MatrixXf nIE(3, 66);
//	//int br = 0;
//	//for (int i = 0; i < 3; i++)
//	//{
//	//	for (int j = 0; j < 66; j++)
//	//	{
//	//		nIE(i, j) = nI[br];
//	//		br++;
//	//	}
//	//}
//	for (int i = 0; i < 66; i++)
//	{
//		dIE(0, i) = dI[i];
//	}
//
//	float noise = 1e-6;
//	int nF = nIE.cols();
//	float halfCubeSize;
//
//	float max, min;
//	max = dIE.maxCoeff();
//	min = dIE.minCoeff();
//	if (min<0 && min*-1 > max)
//		max = min;
//	halfCubeSize = max*1.1;
//
//	P.resize(3, 8);
//	P << 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1;
//	P = halfCubeSize*P;
//
//	Eigen::MatrixXf Premoved = Eigen::MatrixXf::Zero(1, P.cols());//list of removed vertices
//	Eigen::MatrixXf nP = Eigen::MatrixXf::Ones(nF + 6, 1);
//	nP = 4 * nP;
//	
//	F = Eigen::MatrixXi::Zero(nF + 6, 66*66);
//	//F.block<6, 4>(0, 0) << 1, 3, 4, 2, 3, 7, 8, 4, 2, 4, 8, 6, 5, 6, 8, 7, 1, 2, 6, 5, 1, 5, 7, 3;
//	F.block<6, 4>(0, 0) << 0, 2, 3, 1, 2, 6, 7, 3, 1, 3, 7, 5, 4, 5, 7, 6, 0, 1, 5, 4, 0, 4, 6, 2;
//
//
//	Eigen::MatrixXi E = Eigen::MatrixXi::Ones(nF + 6, nF+6);
//	E *= -1;
//	Eigen::MatrixXi Fn = Eigen::MatrixXi::Zero(nF + 6, 66*66);
//
//	int iP1, iP2;
//	int l;
//	int br2;
//	int NextCirc, PrevCirc;
//	for (int i_ = 0; i_ < 6; i_++) //for every face
//	{
//		int i, j;
//		i = i_;
//		for (int k = 0; k < 4; k++)
//		{
//			if (k == 3) NextCirc = 0;
//			else NextCirc = k % 3 +1;
//			iP1 = F(i, k);
//			iP2 = F(i, NextCirc);
//
//			for (int j_ = i_ + 1; j_ < 6; j_++)
//			{
//				j = j_;
//				l = -1;
//				for (int iF = 0; iF < F.cols(); iF++) //find(F(j,:)==iP1)
//				{
//					if (F(j,iF)==iP1)
//					{
//						l = iF;
//					}		
//				}
//				if (l>=0)
//				{
//					PrevCirc = (l + 3) % 4;
//					if (F(j, PrevCirc)== iP2)
//					{
//						E(i,j) = iP1;
//						E(j,i) = iP2;
//						Fn(i,k) = j;
//						Fn(j, PrevCirc) = i;
//					}
//				}
//			}
//		}
//	}
//
//	Eigen::MatrixXi iNewVertices;
//	Eigen::MatrixXi iNeighbors;
//	Eigen::MatrixXf N;
//	Eigen::MatrixXf dCut, dCutSorted;
//	float d, d_;
//	int nCut;
//	Eigen::MatrixXf Temp;
//	Eigen::MatrixXf Tempnext;
//	Eigen::MatrixXf Temp2;
//	for (int i = 0; i < nF; i++) //for every face
//	{
//		printf("%d\n", i);
//
//		N = nIE.block<3, 1>(0, i); //normal of the i-th face
//		d = dIE(0, i); //distance of the i-th face
//		dCut = Eigen::MatrixXf::Zero(P.cols(), 1);
//		dCutSorted = Eigen::MatrixXf::Zero(P.cols(), 1);
//		nCut = 0;
//
//
//		for (int k = 0; k < 8; k++)
//		{
//			Temp = N.transpose()*P.block<3, 1>(0, k);
//			d_ = Temp(0,0) - d;
//			if (d_ > 0)
//			{
//				nCut += 1;
//				dCut(nCut,0) = d_;
//			}
//		}
//
//		//Bubble sort:
//		float temp;
//		for (int idCut = 0; idCut < nCut; idCut++)
//		{
//			for (int jdCut = 0; jdCut < nCut; jdCut++)
//			{
//				if (dCut(jdCut,0)>dCut(jdCut + 1, 0))
//				{
//					temp = dCut(jdCut, 0);
//					dCut(jdCut, 0) = dCut(jdCut + 1, 0);
//					dCut(jdCut + 1, 0) = temp;
//				}
//			}
//		}
//
//		float dCorr = 0;
//		for (int k = 0; k < nCut; k++)
//		{
//			if (dCut(k,0) - dCorr < noise)
//				dCorr=dCut(k,0);
//
//		}
//		d += dCorr;
//
//		Eigen::MatrixXi F_;//j-th face
//		Eigen::MatrixXi Fn_;//neighbors of F_
//		Eigen::MatrixXf P_; //position vector of vector iP
//		Eigen::MatrixXf Pnext;//position vector of vertex iPNext
//		int nP_; //number of vertices od F_
//		int iP; // k-th vertex of F_
//		int iPNext; //next vertex
//		Eigen::MatrixXf dP;
//		int L; //neighbor of F_ on the opposite side of edge iP-iPNext
//
//		int iPolygon, iVertex;
//		int iPNew, iPNewVertex, iFNewVertex;
//		float s;
//
//
//		for (int j = 0; j <= i + 5; j++) //for every previously considered face
//		{
//			F_ = F.block(j, 0, 1, F.cols());
//			Fn_ = Fn.block(j, 0, 1, Fn.cols());
//			int ff = F(j, 0);
//			int k = 0;
//			nP_ = nP(j, 0);
//
//			for (int k_ = 0; k_ < nP_; k_++)
//			{
//				int p = P.cols();
//				iP = F_(0, k_);
//				P_ = P.block(0, iP, P.rows(), 1);
//				
//
//				if (k_ == nP_ - 1) NextCirc = 0;
//				else NextCirc  = k_ % (nP_) + 1;
//				
//				iPNext = F_(0, NextCirc);
//				Pnext = P.block(0, iPNext, P.rows(), 1);
//
//				dP.resize(P.rows(), 1);
//				dP = Pnext - P_;
//
//				L = Fn_(0,k_);
//
//				Temp = N.transpose()*P_;
//				Tempnext = N.transpose()*Pnext;
//
//				if (i == 1 && j == 2) 
//					int debug = 1;
//
//
//				if (Temp(0,0) > d) //if iP is over new plane
//				{
//					if (Premoved(0,iP) == 0)
//						Premoved(0,iP) = 1; //vertex iP is removed
//
//					iPolygon = j;
//					iVertex = k;
//
//					//Remove Vertex from Polygon:
//					for (int iF = 0; iF < (nP(iPolygon) - 1 - iVertex); iF++)
//					{
//						F(iPolygon, iVertex + iF) = F(iPolygon, iVertex + iF + 1);
//						Fn(iPolygon, iVertex + iF) = Fn(iPolygon, iVertex + iF + 1);
//					}
//					nP(iPolygon) = nP(iPolygon) - 1;
//										
//					if (Tempnext(0,0) > d)
//					{
//						E(L,j) = -1;
//						E(j,L) = -1;
//						k -= 1;
//					}
//					else
//					{
//						if (E(j,L) == iP) //Vertex is not updated
//						{
//							//Add new vertex
//							Temp = N.transpose()*P_;
//							Temp2 = N.transpose()*dP;
//							s = (d - Temp(0,0))/Temp2(0,0);
//							Eigen::MatrixXf Ptemp = P;
//							P.resize(P.rows(), P.cols() + 1);
//							P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
//							Eigen::Vector3f col = P_ + s*dP;
//							P.col(P.cols()-1) = col;
//							iPNew = P.cols()-1;
//
//							Eigen::MatrixXf Premovedtemp = Premoved;
//							Premoved.resize(1, Premoved.cols() + 1);
//							Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
//							Premoved(0, Premoved.cols()-1) = 0;
//							E(j,L) = iPNew;
//						}
//						else
//							iPNew = E(j,L);
//
//						iPolygon = j;
//						iVertex = k;
//						iPNewVertex = iPNew;
//						iFNewVertex = L;
//						//Add new Vertex to Polygon:
//						Eigen::MatrixXi Ftemp = F;
//						Eigen::MatrixXi Fntemp = Fn;
//						for (int iF = 0; iF < (nP(iPolygon) - iVertex ); iF++)
//						{
//							F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon,  iVertex + iF);
//							Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
//						}
//						F(iPolygon, iVertex) = iPNewVertex;
//						Fn(iPolygon, iVertex) = iFNewVertex;
//						nP(iPolygon) = nP(iPolygon) + 1;
//
//						Eigen::MatrixXi iNewVerticestemp = iNewVertices;
//						iNewVertices.resize(1, iNewVertices.cols() + 1);
//						iNewVertices.block(0, 0, iNewVerticestemp.rows(), iNewVerticestemp.cols()) = iNewVerticestemp;
//						iNewVertices(0, iNewVertices.cols() - 1)= iPNew;
//
//
//						Eigen::MatrixXi iNeighborstemp = iNeighbors;
//						iNeighbors.resize(1, iNeighbors.cols() + 1);
//						iNeighbors.block(0, 0, iNeighborstemp.rows(), iNeighborstemp.cols()) = iNeighborstemp;
//						iNeighbors(0, iNeighbors.cols() - 1) = j;
//
//						E((i + 6), j) = iPNew;
//					}
//				}
//
//				else if (Tempnext(0,0) > d)
//				{
//					if (E(L,j)== iPNext) //Vertex is not updated
//					{
//						Temp = N.transpose()*P_;
//						Temp2 = N.transpose()*dP;
//						s = (d - Temp(0, 0)) / Temp2(0, 0);
//
//
//						Eigen::MatrixXf Ptemp = P;
//						P.resize(P.rows(), P.cols() + 1);
//						P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
//						Eigen::Vector3f col = P_ + s*dP;
//						P.col(P.cols() - 1) = col;
//						iPNew = P.cols() - 1;
//
//						Eigen::MatrixXf Premovedtemp = Premoved;
//						Premoved.resize(1, Premoved.cols() + 1);
//						Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
//						Premoved(0, Premoved.cols() - 1) = 0;
//
//						E(L,j) = iPNew;
//					}
//					else
//						iPNew = E(L,j);
//
//					iPolygon = j;
//					iVertex = k+1;
//					iPNewVertex = iPNew;
//					iFNewVertex = i+6;
//					//Add new Vertex to Polygon:
//					Eigen::MatrixXi Ftemp = F;
//					Eigen::MatrixXi Fntemp = Fn;
//					for (int iF = 0; iF < (nP(iPolygon) - iVertex); iF++)
//					{
//						F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon, iVertex + iF);
//						Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
//					}
//					F(iPolygon, iVertex) = iPNewVertex;
//					Fn(iPolygon, iVertex) = iFNewVertex;
//					nP(iPolygon) = nP(iPolygon) + 1;
//
//					E(j, i+6) = iPNew;
//					k = k + 1;
//				}
//				k = k + 1;
//			}
//			
//		}
//		int iNeighbor;
//		nP(i + 6) = iNewVertices.cols(); //rows
//
//		int m;
//		if (nP(i + 6) > 0)
//		{
//			Eigen::MatrixXi F_; 
//			Eigen::MatrixXi Fn_;
//			m = 0;
//			while (1)
//			{
//				Eigen::MatrixXi F_temp = F_;
//				F_.resize(1, F_.rows() + 1);
//				F_.block(0, 0, F_temp.rows(), F_temp.cols()) = F_temp;
//				F_(0, F_.rows() - 1) = iNewVertices(m);
//				
//				int iN = iNeighbors(0, m);
//				iNeighbor = iNeighbors(0, m);
//
//
//				Eigen::MatrixXi Fn_temp = Fn_;
//				Fn_.resize(1, Fn_.rows() + 1);
//				Fn_.block(0, 0, Fn_temp.rows(), Fn_temp.cols()) = Fn_temp;
//				Fn_(0, Fn_.rows() - 1) = iNeighbor;
//
//
//				iPNext = E(iNeighbor, (i + 6));
//				int iNV;
//				for (iNV = 0; iNV < iNewVertices.cols(); iNV++)
//				{
//					int a = iPNext;
//					int b = iNewVertices(iNV);
//					if (iNewVertices(iNV) == iPNext)
//					{
//						m = iNV;
//						break;
//					}
//				}
//				if (m == 0 )
//					break;
//			}
//
//			for (int iF = 0; iF < F_.cols(); iF++)
//			{
//				F((i + 6), iF) = F_(0, iF);
//				Fn((i + 6), iF) = Fn_(0, iF);
//			}
//
//		}
//		
//	}
//	int mF = F.cols();
//
//	for (int i = 0; i < F.rows(); i++)
//	{
//		for (int iF = 0; iF < mF; iF++)
//			F(i, nP(i) + 1 + iF) = 0;
//	}
//
//	Eigen::MatrixXi Ffinal = F.block((F.rows() - 6), F.cols(), 6, 0);
//	Edges = E;
//}

//without Eigen; under construction
//void PSGM::RVLPSGInstanceMesh(float *nI, float *dI)
//{
//	float noise = 1e-6;
//	int nF = 66; // sizeof(nI) / sizeof(float); //total number of faces
//	float halfCubeSize;
//
//	float max;
//	if (dI[0] < 0)
//		dI[0] *= -1;
//	max = dI[0];
//	for (int i = 0; i < 56; i += 11)
//	{
//		if (dI[i] < 0)
//			dI[i] *= -1;
//		if (dI[i]>max)
//			max = dI[i];
//	}
//	halfCubeSize = max*1.1;
//
//	P = new float[3 * 8];
//	int PRows = 3, PCols = 8;
//
//	P[0] = P[1] = P[4] = P[5] = P[8] = P[10] = P[12] = P[14] = P[16] = P[17] = P[18] = P[19] = halfCubeSize;
//	P[2] = P[3] = P[6] = P[7] = P[9] = P[11] = P[13] = P[15] = P[20] = P[21] = P[22] = P[23] = -1 * halfCubeSize;
//
//	int Premoved[8]; //length=PCols; //list of removed vertices
//
//	for (int i = 0; i < 8; i++)
//		Premoved[i] = 0;
//
//	int *nP = new int(nF + 6);
//	memset(nP, 4, (nF + 6)*sizeof(float));
//	//for (int i = 0; i < nF + 6; i++)
//	//	nP[i] = 4;
//
//	F = new float((nF + 6) * 4); //initial cube faces
//	memset(F, 0, ((nF + 6) * 4)*sizeof(float));
//	//for (int i = 0; i < (nF + 6) * 4; i++)
//	//	F[i] = 0;
//
//	F[0] = 1;
//	F[1] = 3;
//	F[2] = 4;
//	F[3] = 2;
//	F[4] = 3;
//	F[5] = 7;
//	F[6] = 8;
//	F[7] = 4;
//	F[8] = 2;
//	F[9] = 4;
//	F[10] = 8;
//	F[11] = 6;
//	F[12] = 5;
//	F[13] = 6;
//	F[14] = 8;
//	F[15] = 7;
//	F[16] = 1;
//	F[17] = 2;
//	F[18] = 6;
//	F[19] = 5;
//	F[20] = 1;
//	F[21] = 5;
//	F[22] = 7;
//	F[23] = 3;
//
//	E = new float((nF + 6)*(nF + 6)); //inital Edge matrix
//	memset(E, 0, ((nF + 6)*(nF + 6))*sizeof(float));
//	//for (int i = 0; i < (nF + 6)*(nF + 6); i++)
//	//	E[i] = 0;
//
//	float *Fn = new float((nF + 6) * 4);
//	memset(Fn, 0, ((nF + 6)*(nF + 6))*sizeof(float));
//	//for (int i = 0; i < (nF + 6)*4; i++)
//	//	Fn[i] = 0;
//
//	float iP1, iP2;
//	float l[4];
//	int br;
//	int NextCirc, PrevCirc;
//	for (int i_ = 0; i_ < 6; i_++) //for every face
//	{
//		int i, j;
//		i = i_;
//		for (int k = 0; k < 4; k++)
//		{
//			NextCirc = k % 4 + 1;
//			iP1 = F[i*(nF + 6) + k];
//			iP2 = F[i*(nF + 6) + NextCirc];
//
//			for (int j_ = i_ + 1; j < 6; j++)
//			{
//				br = 0;
//				j = j_;
//				for (int iF = 0; iF < 4; iF++) //find(F(j,:)==iP1)
//				{
//					if (F[j*(nF + 6) * 4 + iF] == iP1)
//					{
//						br++;
//						l[br] = j*(nF + 6) * 4 + iF;
//					}
//				}
//				if (br > 0)
//				{
//					PrevCirc = ((1 + 4 - 2) % 4) + 1;
//					if (F[j, PrevCirc] == iP2)
//					{
//						E[i*(nF + 6) + j] = iP1;
//						E[j*(nF + 6) + i] = iP2;
//						Fn[i*(nF + 6) + k] = j;
//						Fn[j*(nF + 6) + PrevCirc] = i;
//					}
//				}
//			}
//		}
//	}
//
//	int *iNewVertices;
//	int *iNeighbors;
//
//	int nCut;
//	float dCut[8];
//	//float dCutSorted[8];
//
//	float N, d, d_;
//	for (int i = 0; i < nF; i++) //for every face
//	{
//		N = nI[i]; //normal of the i-th face
//		d = dI[i]; //distance of the i-th face
//		memset(dCut, 0, 8 * sizeof(float));
//		//memset(dCutSorted, 0, 4 * sizeof(float));
//		nCut = 0;
//
//		for (int k = 0; k < 8; k++)
//		{
//			d_ = (N*P[k] + N *P[8 + k] + N*P[16 + k]) - d;
//			if (d_ > 0)
//			{
//				nCut += 1;
//				dCut[nCut] = d_;
//			}
//		}
//
//
//
//		//Bubble sort:
//		float temp;
//		for (int idCut = 0; idCut < nCut; idCut++)
//		{
//			for (int jdCut = 0; jdCut < nCut; jdCut++)
//			{
//				if (dCut[jdCut]>dCut[jdCut + 1])
//				{
//					temp = dCut[jdCut];
//					dCut[jdCut] = dCut[jdCut + 1];
//					dCut[jdCut + 1] = temp;
//				}
//			}
//		}
//
//		float dCorr = 0;
//		for (int k = 0; k < nCut; k++)
//		{
//			if (dCut[k] - dCorr < noise)
//				dCorr = dCut[k];
//
//		}
//		d = d + dCorr;
//
//		float F_[4]; //j-th face
//		float Fn_[4]; // neighbors if F_
//		int nP_; //number of vertices od F_
//		int iP; // k-th vertex of F_
//		float P_[3]; //position vector of vector iP
//		int iPNext; //next vertex
//		float Pnext[3]; //position vector of vertex iPNext
//		int dP[3];
//		int L; //neighbor of F_ on the opposite side of edge iP-iPNext
//
//		int iPolygon, iVertex;
//		int iPNew, iPNewVertex, iFNewVertex;
//		float s;
//
//		for (int j = 0; j < i + 5; j++) //for every previously considered face
//		{
//			for (int iF = 0; iF < 4; iF++)
//			{
//				F_[iF] = F[j * 4 + iF];
//				Fn_[iF] = Fn[j * 4 + iF];
//			}
//
//			int k = 0;
//			nP_ = nP[j];
//
//			for (int k_ = 0; k_ < nP_; k_++)
//			{
//				iP = F_[k_];
//				for (int iiP = 0; iiP < 3; iiP++)
//					P_[iiP] = P[8 * iiP + iP];
//
//				NextCirc = k_ % nP_ + 1;
//				iPNext = F_[NextCirc];
//				for (int iiP = 0; iiP < 3; iiP++)
//					Pnext[iiP] = P[8 * iiP + iPNext];
//				for (int idP = 0; idP < 3; idP++)
//					dP[idP] = Pnext[idP] - P_[idP];
//				L = Fn_[k_];
//
//				float Nsum = N*(P_[0] + P_[1] + P_[2]);
//				if (Nsum > d) //if iP is over new plane
//				{
//					if (Premoved[iP] == 0)
//						Premoved[iP] == 1; //vertex iP is removed
//
//					iPolygon = j;
//					iVertex = k;
//
//					//Remove Vertex from Polygon:
//					for (int iF = 0; iF < (nP[iPolygon] - 1 - iVertex); iF++)
//					{
//						F[iPolygon * 4 + iVertex + iF] = F[iPolygon * 4 + iVertex + iF + 1];
//						Fn[iPolygon * 4 + iVertex + iF] = Fn[iPolygon * 4 + iVertex + iF + 1];
//					}
//					nP[iPolygon] = nP[iPolygon] - 1;
//
//					float Nsum = N*(Pnext[0] + Pnext[1] + Pnext[2]);
//					if (Nsum > d)
//					{
//						E[L*(nF + 6) + j] = 0;
//						E[j*(nF + 6) + L] = 0;
//						k -= 1;
//					}
//					else
//					{
//						if (E[j*(nF + 6) + L] == iP) //Vertex is not updated
//						{
//							//Add new vertex
//							s = (d - (N*(P_[0] + P_[1] + P_[2]))) / (N*(dP[0] + dP[1] + dP[2]));
//							P[0] = P_[0] + s*dP[0];
//							P[1] = P_[1] + s*dP[1];
//							P[2] = P_[2] + s*dP[2];
//							iPNew = 3;
//							//Premoved = [Premoved, 0];
//							E[j*(nF + 6) + L] = iPNew;
//						}
//						else
//							iPNew = E[j*(nF + 6) + L];
//
//						iPolygon = j;
//						iVertex = k;
//						iPNewVertex = iPNew;
//						iFNewVertex = 1;
//						//Add new Vertex to Polygon:
//						for (int iF = 0; iF < (nP[iPolygon] + 1 - iVertex + 1); iF++)
//						{
//							F[iPolygon * 4 + iVertex + iF + 1] = F[iPolygon * 4 + iVertex + iF];
//							Fn[iPolygon * 4 + iVertex + iF + 1] = Fn[iPolygon * 4 + iVertex + iF];
//						}
//						F[iPolygon * 4 + iVertex] = iPNewVertex;
//						Fn[iPolygon * 4 + iVertex] = iFNewVertex;
//						nP[iPolygon] = nP[iPolygon] + 1;
//						//iNewVertices = [iNewVertices; iPNew];
//						//iNeighbors = [iNeighbors; j];
//						E[(i + 6)*(nF + 6) + j] = iPNew;
//					}
//				}
//				else if ((N*(Pnext[0] + Pnext[1] + Pnext[2]) > d))
//				{
//					if (E[L*(nF + 6) + j] == iPNext) //Vertex is not updated
//					{
//						s = (d - (N*(P_[0] + P_[1] + P_[2]))) / (N*(dP[0] + dP[1] + dP[2]));
//						P[0] = P_[0] + s*dP[0];
//						P[1] = P_[1] + s*dP[1];
//						P[2] = P_[2] + s*dP[2];
//						iPNew = 3;
//						//Premoved = [Premoved, 0];
//						E[L*(nF + 6) + j] = iPNew;
//					}
//					else
//						iPNew = E[L*(nF + 6) + j];
//
//					iPolygon = j;
//					iVertex = k + 1;
//					iPNewVertex = iPNew;
//					iFNewVertex = i + 6;
//					//Add new Vertex to Polygon:
//					for (int iF = 0; iF < (nP[iPolygon] + 1 - iVertex + 1); iF++)
//					{
//						F[iPolygon * 4 + iVertex + iF + 1] = F[iPolygon * 4 + iVertex + iF];
//						Fn[iPolygon * 4 + iVertex + iF + 1] = Fn[iPolygon * 4 + iVertex + iF];
//					}
//					F[iPolygon * 4 + iVertex] = iPNewVertex;
//					Fn[iPolygon * 4 + iVertex] = iFNewVertex;
//					nP[iPolygon] = nP[iPolygon] + 1;
//					//iNewVertices = [iNewVertices; iPNew];
//					//iNeighbors = [iNeighbors; j];
//					E[j*(nF + 6) + i + 6] = iPNew;
//					k = k + 1;
//				}
//				k = k + 1;
//			}
//
//		}
//		int iNewVerticesSize; //change
//		int iNeighbor;
//		nP[i + 6] = iNewVerticesSize;
//
//		int m = 1, br = 0;
//		if (nP[i + 6] > 0)
//		{
//			float *F_ = new float(1000 * sizeof(float));
//			float *Fn_ = new float(1000 * sizeof(float));
//
//			while (1)
//			{
//				F_[br] = iNewVertices[m];
//				iNeighbor = iNeighbors[m];
//				Fn_[br] = iNeighbor;
//				iPNext = E[iNeighbor*(nF + 6) + (i + 6)];
//				for (int iNV = 0; iNV < iNewVerticesSize; iNV++)
//				{
//					if (iNewVertices[iNV] == iPNext)
//						m = iNV;
//				}
//				if (m == 1)
//					break;
//				br++;
//			}
//			for (int iF = 0; iF < br; iF++)
//			{
//				F[(i + 6) * 4 + iF] = F_[iF];
//				Fn[(i + 6) * 4 + iF] = F_[iF];
//			}
//
//		}
//	}
//	int mF = 4;
//	int sizeF1, sizeF2; //dodati
//	for (int i = 0; i < sizeF2; i++)
//	{
//		for (int iF = 0; iF < mF; iF++)
//			F[i * 4 + nP[i] + 1 + iF] = 0;
//	}
//
//	float *Ffinal;
//	int br = 0;
//	for (int i = 7; i < sizeF2 - 7; i++)
//	{
//		for (int j = 0; j < 4; j++)
//		{
//			Ffinal[br] = F[i * 4 + j];
//			br++;
//		}
//	}
//}

//Vidovic
//for multiple matches per model
void PSGM::ICP(RVL::PSGM::ICPfunction ICPFunction, int ICPvariant)
{
	int iMatch;
	int iMCTI, iSCTI, iCluster, iModel;

	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	RECOG::PSGM_::ModelInstance *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pSIE;

	RECOG::PSGM_::MatchInstance *pMatch;

	//if (icpTMatrix)
	//	delete[] icpTMatrix;

	//icpTMatrix = new double[scoreMatchMatrix.n * nBestMatches * 16]; //nSSegments * nBestMatches * 16 elements of matrix T

	for (int i = 0; i < bestSceneSegmentMatches.n; i++)
	{
		cout << "Segment: " << i << ":\n";

		for (int j = 0; j < bestSceneSegmentMatches.Element[i].n; j++)
		{
			iMatch = bestSceneSegmentMatches.Element[i].Element[j].idx;

			if (iMatch != -1)
			{
				//Setting indices:
				iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
				iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;

				//Getting scene and model pointers:
				pMCTI = MCTISet.pCTI.Element[iMCTI];
				pMIE = pMCTI->modelInstance.Element;
				pSCTI = CTISet.pCTI.Element[iSCTI];
				pSIE = pSCTI->modelInstance.Element;

				iCluster = pSCTI->iCluster;
				iModel = pMCTI->iModel;

				//cout << "Match: " << j << " ModelID:" << iModel << "\n";

				//Getting match pointer and calculating pose:
				pMatch = pCTImatchesArray.Element[iMatch];
				//CalculatePose(iMatch);

				//Getting transform from centered (model) CTI polygon data to scene (T_CCTIM_S)
				float *R_M_S = pMatch->R;
				float *t_M_S_mm = pMatch->t;
				float t_M_S[3];
				RVLSCALE3VECTOR2(t_M_S_mm, 1000.0f, t_M_S);

				//PLY Model transformation
				double T_M_S[16], eyeT[16];
				RVLHTRANSFMX(R_M_S, t_M_S, T_M_S);

				//Without transforming model points to scene c.s.
				eyeT[0] = 1; eyeT[1] = 0; eyeT[2] = 0; eyeT[3] = 0;
				eyeT[4] = 0; eyeT[5] = 1; eyeT[6] = 0; eyeT[7] = 0;
				eyeT[8] = 0; eyeT[9] = 0; eyeT[10] = 1; eyeT[11] = 0;
				eyeT[12] = 0; eyeT[13] = 0; eyeT[14] = 0; eyeT[15] = 1;

				vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
				transform->SetMatrix(T_M_S); //when transforming PLY models to scene
				//transform->SetMatrix(eyeT); //when transforming PLY models to scene
				//transform->SetMatrix(T_CCTIM_S); //when transforming CTI convex hull to scene

				//Scaling PLY model to meters
				vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
				transformScale->Scale(0.001, 0.001, 0.001);
				vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
				transformFilterScale->SetInputData(vtkModelDB.at(iModel));
				transformFilterScale->SetTransform(transformScale);
				transformFilterScale->Update();

				//vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilter->GetOutput()); //Generate visible scene model pointcloud - points are in scene c.s.
				vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilterScale->GetOutput(), T_M_S); //Generate visible scene model pointcloud - points are in model c.s.

				//Aligning pointcluds (using PCL ICP)
				float icpT[16];
				double icpTd[16];
				double fitnessScore;

				icpT[0] = 1; icpT[1] = 0; icpT[2] = 0; icpT[3] = 0;
				icpT[4] = 0; icpT[5] = 1; icpT[6] = 0; icpT[7] = 0;
				icpT[8] = 0; icpT[9] = 0; icpT[10] = 1; icpT[11] = 0;
				icpT[12] = 0; icpT[13] = 0; icpT[14] = 0; icpT[15] = 1;

				//Transforming scene points to model c.s.
				float R_S_M[9], t_S_M[3];
				double T_S_M[16];

				RVLINVTRANSF3D(R_M_S, t_M_S, R_S_M, t_S_M);
				RVLHTRANSFMX(R_S_M, t_S_M, T_S_M);

				vtkSmartPointer<vtkTransform> S_M_transform = vtkSmartPointer<vtkTransform>::New();
				S_M_transform->SetMatrix(T_S_M); //when transforming PLY models to scene

				vtkSmartPointer<vtkTransformPolyDataFilter> sceneTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
				//sceneTransformFilter->SetInputConnection(transformFilterScale->GetOutputPort()); //PLY model
				sceneTransformFilter->SetInputData(this->segmentN_PD.at(iCluster));
				sceneTransformFilter->SetTransform(S_M_transform);
				sceneTransformFilter->Update();

				//ICPFunction(visiblePD, /*this->pMesh->pPolygonData*/this->segmentN_PD.at(iCluster), icpT, 10, 0.01, ICPvariant, &fitnessScore, NULL); //ICP in scene c.s.

				ICPFunction(visiblePD, /*this->pMesh->pPolygonData*/sceneTransformFilter->GetOutput(), icpT, 10, 0.01, ICPvariant, &fitnessScore, NULL); //ICP in model c.s.

				//VTK ICP
				//vtkSmartPointer<vtkIterativeClosestPointTransform> vtkicp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
				//vtkicp->SetSource(visiblePD); //Ulazni objekt (poèetna poza objekta)
				//vtkicp->SetTarget(this->segmentN_PD.at(iCluster)); //Konaèni objekt (željena poza objekta)
				//vtkicp->GetLandmarkTransform()->SetModeToRigidBody(); //Potrebni naèin rada je transformacija za kruta tijela
				//vtkicp->SetMaximumNumberOfIterations(10); //Željeni broj iteracija
				//vtkicp->SetMaximumNumberOfLandmarks(1000); //Koliko parova toèaka da se koristi prilikom minimiziranja cost funkcije
				//vtkicp->Update(); //Provedi algoritam
				//vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();
				//vtkicp->GetMatrix(m);
				//std::cout << "Matrica:" << *m << std::endl;
				//END VTK ICP

				//TEST Vidovic
				float R_ICP[9], t_ICP[3];
				//float R_ICP_inv[9], t_ICP_inv[3];
				float R_ICP_S[9], t_ICP_S[3];
				float icpT2[16];

				R_ICP[0] = icpT[0]; R_ICP[1] = icpT[1]; R_ICP[2] = icpT[2];
				R_ICP[3] = icpT[4]; R_ICP[4] = icpT[5]; R_ICP[5] = icpT[6];
				R_ICP[6] = icpT[8]; R_ICP[7] = icpT[9]; R_ICP[8] = icpT[10];

				t_ICP[0] = icpT[3]; t_ICP[1] = icpT[7]; t_ICP[2] = icpT[11];

				//RVLINVTRANSF3D(R_ICP, t_ICP, R_ICP_inv, t_ICP_inv);
				//RVLCOMPTRANSF3D(R_M_S, t_M_S, R_ICP_inv, t_ICP_inv, R_ICP_S, t_ICP_S);
				RVLCOMPTRANSF3D(R_M_S, t_M_S, R_ICP, t_ICP, R_ICP_S, t_ICP_S);
				RVLHTRANSFMX(R_ICP_S, t_ICP_S, icpT2);

				double icpT2d[16];
				//END test Vidovic

				if (iCluster == 2 && j == 1)
					int debug = 0;

				for (int k = 0; k < 16; k++)
				{
					//icpT[k] = vtkicp->GetMatrix()->GetElement(k / 4, k % 4); //VTK ICP

					//if (iCluster == 2 && j == 1){
					//	icpT[0] = 1; icpT[1] = 0; icpT[2] = 0; icpT[3] = 0;
					//	icpT[4] = 0; icpT[5] = 1; icpT[6] = 0; icpT[7] = 0;
					//	icpT[8] = 0; icpT[9] = 0; icpT[10] = 1; icpT[11] = 0;
					//	icpT[12] = 0; icpT[13] = 0; icpT[14] = 0; icpT[15] = 1;
					//}

					icpTd[k] = icpT[k];
					//save icpT to PSGM class
					//icpTMatrix[i*nBestMatches * 16 + j * 16 + k] = icpT2[k];
					icpT2d[k] = (double)icpT2[k];
				}

				//Transforming model polydata to ICP pose
				vtkSmartPointer<vtkTransform> transformICP = vtkSmartPointer<vtkTransform>::New();
				transformICP->SetMatrix(icpTd);

				vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterICP = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
				//transformFilterICP->SetInputData(visiblePD); //VIDOVIC
				transformFilterICP->SetInputData(transformFilterScale->GetOutput()); //VIDOVIC transformFilter->GetOutput()
				//transformFilterICP->SetTransform(vtkicp); //VTK ICP
				transformFilterICP->SetTransform(transformICP);	//PCL ICP
				transformFilterICP->Update();

				//calculate new visible part of models (using ICP pose) - VIDOVIC
				//visiblePD = GetVisiblePart(transformFilterICP->GetOutput()); ////Generate visible scene model pointcloud - points are in scene c.s.
				//visiblePD = GetVisiblePart(transformFilterICP->GetOutput(), T_M_S); ////Generate visible scene model pointcloud - points are in model c.s.
				visiblePD = GetVisiblePart(transformFilterICP->GetOutput(), icpT2d); ////Generate visible scene model pointcloud - points are in model c.s.

				vtkSmartPointer<vtkTransformPolyDataFilter> sceneSegmentTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
				sceneSegmentTransformFilter->SetInputData(GetSceneModelPC(iCluster));
				sceneSegmentTransformFilter->SetTransform(S_M_transform);
				sceneSegmentTransformFilter->Update();


				//pMatch->cost_NN = NNCost(iCluster, transformFilterICP->GetOutput(), RVLPSGM_ICP_SIMILARITY_MEASURE_SATURATED_SCORE); //VIDOVIC
				//pMatch->cost_NN = NNCost(iCluster, sceneTransformFilter->GetOutput(), visiblePD, RVLPSGM_ICP_SIMILARITY_MEASURE_SATURATED_SCORE); //VIDOVIC - cost for points in the neighbourhood of segment
				pMatch->cost_NN = NNCost(iCluster, sceneTransformFilter->GetOutput(), visiblePD, RVLPSGM_ICP_SIMILARITY_MEASURE_COSTNN); //VIDOVIC - cost for points in the neighbourhood of segment
				//pMatch->cost_NN = NNCost(iCluster, sceneSegmentTransformFilter->GetOutput(), visiblePD, RVLPSGM_ICP_SIMILARITY_MEASURE_COSTNN); //VIDOVIC - cost for points in the neighbourhood of segment
				//pMatch->cost_NN = NNCost(iCluster, sceneSegmentTransformFilter->GetOutput(), visiblePD, RVLPSGM_ICP_SIMILARITY_MEASURE_SATURATED_SCORE); //VIDOVIC - cost for segment points

				//penalize the distance from the gound plane - Vidovic
				float tConst = 30.0;
				float tDistance;

				tDistance = groundPlaneDistance(iModel, icpT2d);
				pMatch->gndDistance = groundPlaneDistance(iModel, icpT2d);

	#ifdef RVLPSGM_GROUND_PLANE_DISTANCE_PENALIZATION
				pMatch->cost_NN += tConst * RVLABS(pMatch->gndDistance) * pMatch->cost_NN;
	#endif

				pMatch->RICP[0] = icpT2[0];
				pMatch->RICP[1] = icpT2[1];
				pMatch->RICP[2] = icpT2[2];
				pMatch->RICP[3] = icpT2[4];
				pMatch->RICP[4] = icpT2[5];
				pMatch->RICP[5] = icpT2[6];
				pMatch->RICP[6] = icpT2[8];
				pMatch->RICP[7] = icpT2[9];
				pMatch->RICP[8] = icpT2[10];

				pMatch->tICP[0] = icpT2[3] * 1000;
				pMatch->tICP[1] = icpT2[7] * 1000;
				pMatch->tICP[2] = icpT2[11] * 1000;
			}
			else continue;
		}
	}
}

float PSGM::groundPlaneDistance(int iModel, double *MSTransform)
{
	VertexGraph *pVertexGraph;
	pVertexGraph = MTGSet.vertexGraphs.at(iModel);
	int iVertex;

	double R[9], t[3];
	float P_[3], P[3];
	float minDistance, distance;

	for (iVertex = 0; iVertex < pVertexGraph->NodeArray.n; iVertex++)
	{
		RVLSCALE3VECTOR2(pVertexGraph->NodeArray.Element[iVertex].P, 1000, P_);
		RVLHTRANSFMXDECOMP(MSTransform, R, t);
		RVLTRANSF3(P_, R, t, P);

		if (iVertex == 0)
			minDistance = RVLDOTPRODUCT3(P, NGnd) - dGnd;
		else
		{
			distance = RVLDOTPRODUCT3(P, NGnd) - dGnd;

			if (distance < minDistance)
				minDistance = distance;
		}			
	}

	return minDistance;
}

//calculate RMSE for all TP hypothesis on the scene
void PSGM::RMSE(FILE *fp, bool allTPHypotheses)
{
	int iMatch, iGTS, iSegment, iGTM, iMCTI, iMatchedModel, iSegmentGT, iSSegment, iHypothesis;
	bool TPHypothesis = false;
	int nGTModels;
	RVL::GTInstance *pGT;
	RECOG::PSGM_::MatchInstance *pMatch;
	float RMSE_ = 0.0;
	double TGT[16], T[16], tICP[3], RGT[9], tGT[3];

	if (scoreMatchMatrixICP.Element == NULL)
		CreateScoreMatchMatrixICP();

	iMatch = scoreMatchMatrixICP.Element[0].Element[0].idx;

	//iGTS = pCTImatchesArray.Element[iMatch]->iScene;
	iGTS = iScene - 1;
	nGTModels = pECCVGT->GT.Element[iGTS].n;

	printf("RMSE for TP hypothesis:\n");

	if (!allTPHypotheses)
	{
		pGT = pECCVGT->GT.Element[iGTS].Element;

		for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
		{
			for (iSegment = 0; iSegment < scoreMatchMatrix.n; iSegment++)
			{
				iMatch = scoreMatchMatrixICP.Element[iSegment].Element[0].idx;

				if (iMatch != -1)
				{
					pMatch = pCTImatchesArray.Element[iMatch];

					iMCTI = pMatch->iMCTI;
					iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;
					iSSegment = CTISet.pCTI.Element[pMatch->iSCTI]->iCluster;

					iSegmentGT = iGTS * nDominantClusters + iSSegment;

					if (iMatchedModel == segmentGT.Element[iSegmentGT].iModel && iMatchedModel == pGT->iModel)
					{
						RVLSCALEMX3X3(pGT->R, 1000, RGT);
						RVLSCALE3VECTOR(pGT->t, 1000, tGT);
						RVLHTRANSFMX(RGT, tGT, TGT);
						RVLHTRANSFMX(pMatch->RICP, pMatch->tICP, T);

						RMSE_ = RMSE(iMatchedModel, TGT, T);
						printf("Segment %d matched with model %d. RMSE for TP is: %f\n", iSegment, iMatchedModel, RMSE_);
						fprintf(fp, "%d\t%d\t%d\t%f\n", iGTS, iSSegment, iMatchedModel, RMSE_);
					}
				}
			}
		}
		printf("---------------------------------------------------\n");
	}//only for 0-th palced hypotheses
	else
	{
		pGT = pECCVGT->GT.Element[iGTS].Element;

		for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
		{
			for (iSegment = 0; iSegment < scoreMatchMatrix.n; iSegment++)
			{
				TPHypothesis = false;

				for (iHypothesis = 0; iHypothesis < nBestMatches; iHypothesis++)
				{
					iMatch = scoreMatchMatrixICP.Element[iSegment].Element[iHypothesis].idx;

					if (iMatch != -1)
					{
						pMatch = pCTImatchesArray.Element[iMatch];

						iMCTI = pMatch->iMCTI;
						iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;
						iSSegment = CTISet.pCTI.Element[pMatch->iSCTI]->iCluster;

						iSegmentGT = iGTS * nDominantClusters + iSSegment;

						if (segmentGT.Element[iSegmentGT].iModel == pGT->iModel)
						{
							RVLSCALEMX3X3(pGT->R, 1000, RGT);
							RVLSCALE3VECTOR(pGT->t, 1000, tGT);
							RVLHTRANSFMX(RGT, tGT, TGT);
							RVLHTRANSFMX(pMatch->RICP, pMatch->tICP, T);

							RMSE_ = RMSE(iMatchedModel, TGT, T);

							if (iMatchedModel == segmentGT.Element[iSegmentGT].iModel)
							{
								printf("Segment %d matched with model %d on %d. place (TP). RMSE is: %f\n", iSegment, iMatchedModel, iHypothesis, RMSE_);
								fprintf(fp, "%d\t%d\t%d\t%f\t%d\n", iGTS, iSSegment, iMatchedModel, RMSE_, 1);
								TPHypothesis = true;

								//printing distance from the ground plane - TEST
								//fprintf(fp, "%d\t%d\t%d\t%d\t%f\t%d\n", iGTS, iSSegment, iHypothesis, iMatchedModel, pMatch->gndDistance, 1);
							}
							else
							{
								printf("Segment %d matched with model %d on %d. place (FP). RMSE is: %f\n", iSegment, iMatchedModel, iHypothesis, RMSE_);
								fprintf(fp, "%d\t%d\t%d\t%f\t%d\n", iGTS, iSSegment, iMatchedModel, RMSE_, 0);

								//printing distance from the ground plane - TEST
								//fprintf(fp, "%d\t%d\t%d\t%d\t%f\t%d\n", iGTS, iSSegment, iHypothesis, iMatchedModel, pMatch->gndDistance, 0);
							}
						}

						if (TPHypothesis)
							break;
					}
				}
			}
		}
		printf("---------------------------------------------------\n");

	}//for all TP hypothesis
}

//calculate RMSE for single model
float PSGM::RMSE(int iModel, double *TGT, double *T)
{
	vtkSmartPointer<vtkTransform> GTtransform = vtkSmartPointer<vtkTransform>::New();
	GTtransform->SetMatrix(TGT); //tranform model to the GT pose

	vtkSmartPointer<vtkTransformPolyDataFilter> GTtransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	GTtransformFilter->SetInputData(vtkRMSEModelDB.at(iModel));
	GTtransformFilter->SetTransform(GTtransform);
	GTtransformFilter->Update();

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(T); //transform model to the hypothesis pose

	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(vtkRMSEModelDB.at(iModel));
	transformFilter->SetTransform(transform);
	transformFilter->Update();

	NanoFlannPointCloud<float> GTPC;
	vtkSmartPointer<vtkPoints> GTPoints = GTtransformFilter->GetOutput()->GetPoints();
	GTPC.pts.resize(GTPoints->GetNumberOfPoints());
	double *point;
	int iPoint;

	for (iPoint = 0; iPoint < GTPoints->GetNumberOfPoints(); iPoint++)
	{
		point = GTPoints->GetPoint(iPoint);
		GTPC.pts.at(iPoint).x = point[0];
		GTPC.pts.at(iPoint).y = point[1];
		GTPC.pts.at(iPoint).z = point[2];
	}

	nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, NanoFlannPointCloud<float> >, NanoFlannPointCloud<float>, 3> index(3 /*dim*/, GTPC, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	index.buildIndex();

	vtkSmartPointer<vtkPoints> hypothesisPoints = transformFilter->GetOutput()->GetPoints();
	float pointF[3];

	std::vector<size_t>   ret_index(1);
	std::vector<float> out_dist_sqr(1);
	float RMSE_ = 0;

	for (iPoint = 0; iPoint < hypothesisPoints->GetNumberOfPoints(); iPoint++)
	{
		point = hypothesisPoints->GetPoint(iPoint);
		pointF[0] = point[0];
		pointF[1] = point[1];
		pointF[2] = point[2];

		index.knnSearch(pointF, 1, &ret_index[0], &out_dist_sqr[0]);

		RMSE_ += out_dist_sqr.at(0);
	}

	RMSE_ = sqrt((RMSE_ / hypothesisPoints->GetNumberOfPoints()));

	return RMSE_;
}

//END Vidovic

//with Eigen
//void PSGM::RVLPSGInstanceMesh(Eigen::MatrixXf nI, float *dI)
//{
//	//transform nI and dI to Eigen:
//	Eigen::MatrixXf nIE = nI;
//	Eigen::MatrixXf dIE(1, 66);
//
//	//if nI was an array
//	//Eigen::MatrixXf nIE(3, 66);
//	//int br = 0;
//	//for (int i = 0; i < 3; i++)
//	//{
//	//	for (int j = 0; j < 66; j++)
//	//	{
//	//		nIE(i, j) = nI[br];
//	//		br++;
//	//	}
//	//}
//	for (int i = 0; i < 66; i++)
//	{
//		dIE(0, i) = dI[i];
//	}
//
//	float noise = 1e-6;
//	int nF = nIE.cols();
//	float halfCubeSize;
//
//	float max, min;
//	max = dIE.maxCoeff();
//	min = dIE.minCoeff();
//	if (min<0 && min*-1 > max)
//		max = min;
//	halfCubeSize = max*1.1;
//
//	P.resize(3, 8);
//	P << 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1;
//	P = halfCubeSize*P;
//
//	Eigen::MatrixXf Premoved = Eigen::MatrixXf::Zero(1, P.cols());//list of removed vertices
//	Eigen::MatrixXf nP = Eigen::MatrixXf::Ones(nF + 6, 1);
//	nP = 4 * nP;
//	
//	F = Eigen::MatrixXi::Zero(nF + 6, 66*66);
//	//F.block<6, 4>(0, 0) << 1, 3, 4, 2, 3, 7, 8, 4, 2, 4, 8, 6, 5, 6, 8, 7, 1, 2, 6, 5, 1, 5, 7, 3;
//	F.block<6, 4>(0, 0) << 0, 2, 3, 1, 2, 6, 7, 3, 1, 3, 7, 5, 4, 5, 7, 6, 0, 1, 5, 4, 0, 4, 6, 2;
//
//
//	Eigen::MatrixXi E = Eigen::MatrixXi::Ones(nF + 6, nF+6);
//	E *= -1;
//	Eigen::MatrixXi Fn = Eigen::MatrixXi::Zero(nF + 6, 66*66);
//
//	int iP1, iP2;
//	int l;
//	int br2;
//	int NextCirc, PrevCirc;
//	for (int i_ = 0; i_ < 6; i_++) //for every face
//	{
//		int i, j;
//		i = i_;
//		for (int k = 0; k < 4; k++)
//		{
//			if (k == 3) NextCirc = 0;
//			else NextCirc = k % 3 +1;
//			iP1 = F(i, k);
//			iP2 = F(i, NextCirc);
//
//			for (int j_ = i_ + 1; j_ < 6; j_++)
//			{
//				j = j_;
//				l = -1;
//				for (int iF = 0; iF < F.cols(); iF++) //find(F(j,:)==iP1)
//				{
//					if (F(j,iF)==iP1)
//					{
//						l = iF;
//					}		
//				}
//				if (l>=0)
//				{
//					PrevCirc = (l + 3) % 4;
//					if (F(j, PrevCirc)== iP2)
//					{
//						E(i,j) = iP1;
//						E(j,i) = iP2;
//						Fn(i,k) = j;
//						Fn(j, PrevCirc) = i;
//					}
//				}
//			}
//		}
//	}
//
//	Eigen::MatrixXi iNewVertices;
//	Eigen::MatrixXi iNeighbors;
//	Eigen::MatrixXf N;
//	Eigen::MatrixXf dCut, dCutSorted;
//	float d, d_;
//	int nCut;
//	Eigen::MatrixXf Temp;
//	Eigen::MatrixXf Tempnext;
//	Eigen::MatrixXf Temp2;
//	for (int i = 0; i < nF; i++) //for every face
//	{
//		printf("%d\n", i);
//
//		N = nIE.block<3, 1>(0, i); //normal of the i-th face
//		d = dIE(0, i); //distance of the i-th face
//		dCut = Eigen::MatrixXf::Zero(P.cols(), 1);
//		dCutSorted = Eigen::MatrixXf::Zero(P.cols(), 1);
//		nCut = 0;
//
//
//		for (int k = 0; k < 8; k++)
//		{
//			Temp = N.transpose()*P.block<3, 1>(0, k);
//			d_ = Temp(0,0) - d;
//			if (d_ > 0)
//			{
//				nCut += 1;
//				dCut(nCut,0) = d_;
//			}
//		}
//
//		//Bubble sort:
//		float temp;
//		for (int idCut = 0; idCut < nCut; idCut++)
//		{
//			for (int jdCut = 0; jdCut < nCut; jdCut++)
//			{
//				if (dCut(jdCut,0)>dCut(jdCut + 1, 0))
//				{
//					temp = dCut(jdCut, 0);
//					dCut(jdCut, 0) = dCut(jdCut + 1, 0);
//					dCut(jdCut + 1, 0) = temp;
//				}
//			}
//		}
//
//		float dCorr = 0;
//		for (int k = 0; k < nCut; k++)
//		{
//			if (dCut(k,0) - dCorr < noise)
//				dCorr=dCut(k,0);
//
//		}
//		d += dCorr;
//
//		Eigen::MatrixXi F_;//j-th face
//		Eigen::MatrixXi Fn_;//neighbors of F_
//		Eigen::MatrixXf P_; //position vector of vector iP
//		Eigen::MatrixXf Pnext;//position vector of vertex iPNext
//		int nP_; //number of vertices od F_
//		int iP; // k-th vertex of F_
//		int iPNext; //next vertex
//		Eigen::MatrixXf dP;
//		int L; //neighbor of F_ on the opposite side of edge iP-iPNext
//
//		int iPolygon, iVertex;
//		int iPNew, iPNewVertex, iFNewVertex;
//		float s;
//
//
//		for (int j = 0; j <= i + 5; j++) //for every previously considered face
//		{
//			F_ = F.block(j, 0, 1, F.cols());
//			Fn_ = Fn.block(j, 0, 1, Fn.cols());
//			int ff = F(j, 0);
//			int k = 0;
//			nP_ = nP(j, 0);
//
//			for (int k_ = 0; k_ < nP_; k_++)
//			{
//				int p = P.cols();
//				iP = F_(0, k_);
//				P_ = P.block(0, iP, P.rows(), 1);
//				
//
//				if (k_ == nP_ - 1) NextCirc = 0;
//				else NextCirc  = k_ % (nP_) + 1;
//				
//				iPNext = F_(0, NextCirc);
//				Pnext = P.block(0, iPNext, P.rows(), 1);
//
//				dP.resize(P.rows(), 1);
//				dP = Pnext - P_;
//
//				L = Fn_(0,k_);
//
//				Temp = N.transpose()*P_;
//				Tempnext = N.transpose()*Pnext;
//
//				if (i == 1 && j == 2) 
//					int debug = 1;
//
//
//				if (Temp(0,0) > d) //if iP is over new plane
//				{
//					if (Premoved(0,iP) == 0)
//						Premoved(0,iP) = 1; //vertex iP is removed
//
//					iPolygon = j;
//					iVertex = k;
//
//					//Remove Vertex from Polygon:
//					for (int iF = 0; iF < (nP(iPolygon) - 1 - iVertex); iF++)
//					{
//						F(iPolygon, iVertex + iF) = F(iPolygon, iVertex + iF + 1);
//						Fn(iPolygon, iVertex + iF) = Fn(iPolygon, iVertex + iF + 1);
//					}
//					nP(iPolygon) = nP(iPolygon) - 1;
//										
//					if (Tempnext(0,0) > d)
//					{
//						E(L,j) = -1;
//						E(j,L) = -1;
//						k -= 1;
//					}
//					else
//					{
//						if (E(j,L) == iP) //Vertex is not updated
//						{
//							//Add new vertex
//							Temp = N.transpose()*P_;
//							Temp2 = N.transpose()*dP;
//							s = (d - Temp(0,0))/Temp2(0,0);
//							Eigen::MatrixXf Ptemp = P;
//							P.resize(P.rows(), P.cols() + 1);
//							P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
//							Eigen::Vector3f col = P_ + s*dP;
//							P.col(P.cols()-1) = col;
//							iPNew = P.cols()-1;
//
//							Eigen::MatrixXf Premovedtemp = Premoved;
//							Premoved.resize(1, Premoved.cols() + 1);
//							Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
//							Premoved(0, Premoved.cols()-1) = 0;
//							E(j,L) = iPNew;
//						}
//						else
//							iPNew = E(j,L);
//
//						iPolygon = j;
//						iVertex = k;
//						iPNewVertex = iPNew;
//						iFNewVertex = L;
//						//Add new Vertex to Polygon:
//						Eigen::MatrixXi Ftemp = F;
//						Eigen::MatrixXi Fntemp = Fn;
//						for (int iF = 0; iF < (nP(iPolygon) - iVertex ); iF++)
//						{
//							F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon,  iVertex + iF);
//							Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
//						}
//						F(iPolygon, iVertex) = iPNewVertex;
//						Fn(iPolygon, iVertex) = iFNewVertex;
//						nP(iPolygon) = nP(iPolygon) + 1;
//
//						Eigen::MatrixXi iNewVerticestemp = iNewVertices;
//						iNewVertices.resize(1, iNewVertices.cols() + 1);
//						iNewVertices.block(0, 0, iNewVerticestemp.rows(), iNewVerticestemp.cols()) = iNewVerticestemp;
//						iNewVertices(0, iNewVertices.cols() - 1)= iPNew;
//
//
//						Eigen::MatrixXi iNeighborstemp = iNeighbors;
//						iNeighbors.resize(1, iNeighbors.cols() + 1);
//						iNeighbors.block(0, 0, iNeighborstemp.rows(), iNeighborstemp.cols()) = iNeighborstemp;
//						iNeighbors(0, iNeighbors.cols() - 1) = j;
//
//						E((i + 6), j) = iPNew;
//					}
//				}
//
//				else if (Tempnext(0,0) > d)
//				{
//					if (E(L,j)== iPNext) //Vertex is not updated
//					{
//						Temp = N.transpose()*P_;
//						Temp2 = N.transpose()*dP;
//						s = (d - Temp(0, 0)) / Temp2(0, 0);
//
//
//						Eigen::MatrixXf Ptemp = P;
//						P.resize(P.rows(), P.cols() + 1);
//						P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
//						Eigen::Vector3f col = P_ + s*dP;
//						P.col(P.cols() - 1) = col;
//						iPNew = P.cols() - 1;
//
//						Eigen::MatrixXf Premovedtemp = Premoved;
//						Premoved.resize(1, Premoved.cols() + 1);
//						Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
//						Premoved(0, Premoved.cols() - 1) = 0;
//
//						E(L,j) = iPNew;
//					}
//					else
//						iPNew = E(L,j);
//
//					iPolygon = j;
//					iVertex = k+1;
//					iPNewVertex = iPNew;
//					iFNewVertex = i+6;
//					//Add new Vertex to Polygon:
//					Eigen::MatrixXi Ftemp = F;
//					Eigen::MatrixXi Fntemp = Fn;
//					for (int iF = 0; iF < (nP(iPolygon) - iVertex); iF++)
//					{
//						F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon, iVertex + iF);
//						Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
//					}
//					F(iPolygon, iVertex) = iPNewVertex;
//					Fn(iPolygon, iVertex) = iFNewVertex;
//					nP(iPolygon) = nP(iPolygon) + 1;
//
//					E(j, i+6) = iPNew;
//					k = k + 1;
//				}
//				k = k + 1;
//			}
//			
//		}
//		int iNeighbor;
//		nP(i + 6) = iNewVertices.cols(); //rows
//
//		int m;
//		if (nP(i + 6) > 0)
//		{
//			Eigen::MatrixXi F_; 
//			Eigen::MatrixXi Fn_;
//			m = 0;
//			while (1)
//			{
//				Eigen::MatrixXi F_temp = F_;
//				F_.resize(1, F_.rows() + 1);
//				F_.block(0, 0, F_temp.rows(), F_temp.cols()) = F_temp;
//				F_(0, F_.rows() - 1) = iNewVertices(m);
//				
//				int iN = iNeighbors(0, m);
//				iNeighbor = iNeighbors(0, m);
//
//
//				Eigen::MatrixXi Fn_temp = Fn_;
//				Fn_.resize(1, Fn_.rows() + 1);
//				Fn_.block(0, 0, Fn_temp.rows(), Fn_temp.cols()) = Fn_temp;
//				Fn_(0, Fn_.rows() - 1) = iNeighbor;
//
//
//				iPNext = E(iNeighbor, (i + 6));
//				int iNV;
//				for (iNV = 0; iNV < iNewVertices.cols(); iNV++)
//				{
//					int a = iPNext;
//					int b = iNewVertices(iNV);
//					if (iNewVertices(iNV) == iPNext)
//					{
//						m = iNV;
//						break;
//					}
//				}
//				if (m == 0 )
//					break;
//			}
//
//			for (int iF = 0; iF < F_.cols(); iF++)
//			{
//				F((i + 6), iF) = F_(0, iF);
//				Fn((i + 6), iF) = Fn_(0, iF);
//			}
//
//		}
//		
//	}
//	int mF = F.cols();
//
//	for (int i = 0; i < F.rows(); i++)
//	{
//		for (int iF = 0; iF < mF; iF++)
//			F(i, nP(i) + 1 + iF) = 0;
//	}
//
//	Eigen::MatrixXi Ffinal = F.block((F.rows() - 6), F.cols(), 6, 0);
//	Edges = E;
//}

//without Eigen; under construction
//void PSGM::RVLPSGInstanceMesh(float *nI, float *dI)
//{
//	float noise = 1e-6;
//	int nF = 66; // sizeof(nI) / sizeof(float); //total number of faces
//	float halfCubeSize;
//
//	float max;
//	if (dI[0] < 0)
//		dI[0] *= -1;
//	max = dI[0];
//	for (int i = 0; i < 56; i += 11)
//	{
//		if (dI[i] < 0)
//			dI[i] *= -1;
//		if (dI[i]>max)
//			max = dI[i];
//	}
//	halfCubeSize = max*1.1;
//
//	P = new float[3 * 8];
//	int PRows = 3, PCols = 8;
//
//	P[0] = P[1] = P[4] = P[5] = P[8] = P[10] = P[12] = P[14] = P[16] = P[17] = P[18] = P[19] = halfCubeSize;
//	P[2] = P[3] = P[6] = P[7] = P[9] = P[11] = P[13] = P[15] = P[20] = P[21] = P[22] = P[23] = -1 * halfCubeSize;
//
//	int Premoved[8]; //length=PCols; //list of removed vertices
//
//	for (int i = 0; i < 8; i++)
//		Premoved[i] = 0;
//
//	int *nP = new int(nF + 6);
//	memset(nP, 4, (nF + 6)*sizeof(float));
//	//for (int i = 0; i < nF + 6; i++)
//	//	nP[i] = 4;
//
//	F = new float((nF + 6) * 4); //initial cube faces
//	memset(F, 0, ((nF + 6) * 4)*sizeof(float));
//	//for (int i = 0; i < (nF + 6) * 4; i++)
//	//	F[i] = 0;
//
//	F[0] = 1;
//	F[1] = 3;
//	F[2] = 4;
//	F[3] = 2;
//	F[4] = 3;
//	F[5] = 7;
//	F[6] = 8;
//	F[7] = 4;
//	F[8] = 2;
//	F[9] = 4;
//	F[10] = 8;
//	F[11] = 6;
//	F[12] = 5;
//	F[13] = 6;
//	F[14] = 8;
//	F[15] = 7;
//	F[16] = 1;
//	F[17] = 2;
//	F[18] = 6;
//	F[19] = 5;
//	F[20] = 1;
//	F[21] = 5;
//	F[22] = 7;
//	F[23] = 3;
//
//	E = new float((nF + 6)*(nF + 6)); //inital Edge matrix
//	memset(E, 0, ((nF + 6)*(nF + 6))*sizeof(float));
//	//for (int i = 0; i < (nF + 6)*(nF + 6); i++)
//	//	E[i] = 0;
//
//	float *Fn = new float((nF + 6) * 4);
//	memset(Fn, 0, ((nF + 6)*(nF + 6))*sizeof(float));
//	//for (int i = 0; i < (nF + 6)*4; i++)
//	//	Fn[i] = 0;
//
//	float iP1, iP2;
//	float l[4];
//	int br;
//	int NextCirc, PrevCirc;
//	for (int i_ = 0; i_ < 6; i_++) //for every face
//	{
//		int i, j;
//		i = i_;
//		for (int k = 0; k < 4; k++)
//		{
//			NextCirc = k % 4 + 1;
//			iP1 = F[i*(nF + 6) + k];
//			iP2 = F[i*(nF + 6) + NextCirc];
//
//			for (int j_ = i_ + 1; j < 6; j++)
//			{
//				br = 0;
//				j = j_;
//				for (int iF = 0; iF < 4; iF++) //find(F(j,:)==iP1)
//				{
//					if (F[j*(nF + 6) * 4 + iF] == iP1)
//					{
//						br++;
//						l[br] = j*(nF + 6) * 4 + iF;
//					}
//				}
//				if (br > 0)
//				{
//					PrevCirc = ((1 + 4 - 2) % 4) + 1;
//					if (F[j, PrevCirc] == iP2)
//					{
//						E[i*(nF + 6) + j] = iP1;
//						E[j*(nF + 6) + i] = iP2;
//						Fn[i*(nF + 6) + k] = j;
//						Fn[j*(nF + 6) + PrevCirc] = i;
//					}
//				}
//			}
//		}
//	}
//
//	int *iNewVertices;
//	int *iNeighbors;
//
//	int nCut;
//	float dCut[8];
//	//float dCutSorted[8];
//
//	float N, d, d_;
//	for (int i = 0; i < nF; i++) //for every face
//	{
//		N = nI[i]; //normal of the i-th face
//		d = dI[i]; //distance of the i-th face
//		memset(dCut, 0, 8 * sizeof(float));
//		//memset(dCutSorted, 0, 4 * sizeof(float));
//		nCut = 0;
//
//		for (int k = 0; k < 8; k++)
//		{
//			d_ = (N*P[k] + N *P[8 + k] + N*P[16 + k]) - d;
//			if (d_ > 0)
//			{
//				nCut += 1;
//				dCut[nCut] = d_;
//			}
//		}
//
//
//
//		//Bubble sort:
//		float temp;
//		for (int idCut = 0; idCut < nCut; idCut++)
//		{
//			for (int jdCut = 0; jdCut < nCut; jdCut++)
//			{
//				if (dCut[jdCut]>dCut[jdCut + 1])
//				{
//					temp = dCut[jdCut];
//					dCut[jdCut] = dCut[jdCut + 1];
//					dCut[jdCut + 1] = temp;
//				}
//			}
//		}
//
//		float dCorr = 0;
//		for (int k = 0; k < nCut; k++)
//		{
//			if (dCut[k] - dCorr < noise)
//				dCorr = dCut[k];
//
//		}
//		d = d + dCorr;
//
//		float F_[4]; //j-th face
//		float Fn_[4]; // neighbors if F_
//		int nP_; //number of vertices od F_
//		int iP; // k-th vertex of F_
//		float P_[3]; //position vector of vector iP
//		int iPNext; //next vertex
//		float Pnext[3]; //position vector of vertex iPNext
//		int dP[3];
//		int L; //neighbor of F_ on the opposite side of edge iP-iPNext
//
//		int iPolygon, iVertex;
//		int iPNew, iPNewVertex, iFNewVertex;
//		float s;
//
//		for (int j = 0; j < i + 5; j++) //for every previously considered face
//		{
//			for (int iF = 0; iF < 4; iF++)
//			{
//				F_[iF] = F[j * 4 + iF];
//				Fn_[iF] = Fn[j * 4 + iF];
//			}
//
//			int k = 0;
//			nP_ = nP[j];
//
//			for (int k_ = 0; k_ < nP_; k_++)
//			{
//				iP = F_[k_];
//				for (int iiP = 0; iiP < 3; iiP++)
//					P_[iiP] = P[8 * iiP + iP];
//
//				NextCirc = k_ % nP_ + 1;
//				iPNext = F_[NextCirc];
//				for (int iiP = 0; iiP < 3; iiP++)
//					Pnext[iiP] = P[8 * iiP + iPNext];
//				for (int idP = 0; idP < 3; idP++)
//					dP[idP] = Pnext[idP] - P_[idP];
//				L = Fn_[k_];
//
//				float Nsum = N*(P_[0] + P_[1] + P_[2]);
//				if (Nsum > d) //if iP is over new plane
//				{
//					if (Premoved[iP] == 0)
//						Premoved[iP] == 1; //vertex iP is removed
//
//					iPolygon = j;
//					iVertex = k;
//
//					//Remove Vertex from Polygon:
//					for (int iF = 0; iF < (nP[iPolygon] - 1 - iVertex); iF++)
//					{
//						F[iPolygon * 4 + iVertex + iF] = F[iPolygon * 4 + iVertex + iF + 1];
//						Fn[iPolygon * 4 + iVertex + iF] = Fn[iPolygon * 4 + iVertex + iF + 1];
//					}
//					nP[iPolygon] = nP[iPolygon] - 1;
//
//					float Nsum = N*(Pnext[0] + Pnext[1] + Pnext[2]);
//					if (Nsum > d)
//					{
//						E[L*(nF + 6) + j] = 0;
//						E[j*(nF + 6) + L] = 0;
//						k -= 1;
//					}
//					else
//					{
//						if (E[j*(nF + 6) + L] == iP) //Vertex is not updated
//						{
//							//Add new vertex
//							s = (d - (N*(P_[0] + P_[1] + P_[2]))) / (N*(dP[0] + dP[1] + dP[2]));
//							P[0] = P_[0] + s*dP[0];
//							P[1] = P_[1] + s*dP[1];
//							P[2] = P_[2] + s*dP[2];
//							iPNew = 3;
//							//Premoved = [Premoved, 0];
//							E[j*(nF + 6) + L] = iPNew;
//						}
//						else
//							iPNew = E[j*(nF + 6) + L];
//
//						iPolygon = j;
//						iVertex = k;
//						iPNewVertex = iPNew;
//						iFNewVertex = 1;
//						//Add new Vertex to Polygon:
//						for (int iF = 0; iF < (nP[iPolygon] + 1 - iVertex + 1); iF++)
//						{
//							F[iPolygon * 4 + iVertex + iF + 1] = F[iPolygon * 4 + iVertex + iF];
//							Fn[iPolygon * 4 + iVertex + iF + 1] = Fn[iPolygon * 4 + iVertex + iF];
//						}
//						F[iPolygon * 4 + iVertex] = iPNewVertex;
//						Fn[iPolygon * 4 + iVertex] = iFNewVertex;
//						nP[iPolygon] = nP[iPolygon] + 1;
//						//iNewVertices = [iNewVertices; iPNew];
//						//iNeighbors = [iNeighbors; j];
//						E[(i + 6)*(nF + 6) + j] = iPNew;
//					}
//				}
//				else if ((N*(Pnext[0] + Pnext[1] + Pnext[2]) > d))
//				{
//					if (E[L*(nF + 6) + j] == iPNext) //Vertex is not updated
//					{
//						s = (d - (N*(P_[0] + P_[1] + P_[2]))) / (N*(dP[0] + dP[1] + dP[2]));
//						P[0] = P_[0] + s*dP[0];
//						P[1] = P_[1] + s*dP[1];
//						P[2] = P_[2] + s*dP[2];
//						iPNew = 3;
//						//Premoved = [Premoved, 0];
//						E[L*(nF + 6) + j] = iPNew;
//					}
//					else
//						iPNew = E[L*(nF + 6) + j];
//
//					iPolygon = j;
//					iVertex = k + 1;
//					iPNewVertex = iPNew;
//					iFNewVertex = i + 6;
//					//Add new Vertex to Polygon:
//					for (int iF = 0; iF < (nP[iPolygon] + 1 - iVertex + 1); iF++)
//					{
//						F[iPolygon * 4 + iVertex + iF + 1] = F[iPolygon * 4 + iVertex + iF];
//						Fn[iPolygon * 4 + iVertex + iF + 1] = Fn[iPolygon * 4 + iVertex + iF];
//					}
//					F[iPolygon * 4 + iVertex] = iPNewVertex;
//					Fn[iPolygon * 4 + iVertex] = iFNewVertex;
//					nP[iPolygon] = nP[iPolygon] + 1;
//					//iNewVertices = [iNewVertices; iPNew];
//					//iNeighbors = [iNeighbors; j];
//					E[j*(nF + 6) + i + 6] = iPNew;
//					k = k + 1;
//				}
//				k = k + 1;
//			}
//
//		}
//		int iNewVerticesSize; //change
//		int iNeighbor;
//		nP[i + 6] = iNewVerticesSize;
//
//		int m = 1, br = 0;
//		if (nP[i + 6] > 0)
//		{
//			float *F_ = new float(1000 * sizeof(float));
//			float *Fn_ = new float(1000 * sizeof(float));
//
//			while (1)
//			{
//				F_[br] = iNewVertices[m];
//				iNeighbor = iNeighbors[m];
//				Fn_[br] = iNeighbor;
//				iPNext = E[iNeighbor*(nF + 6) + (i + 6)];
//				for (int iNV = 0; iNV < iNewVerticesSize; iNV++)
//				{
//					if (iNewVertices[iNV] == iPNext)
//						m = iNV;
//				}
//				if (m == 1)
//					break;
//				br++;
//			}
//			for (int iF = 0; iF < br; iF++)
//			{
//				F[(i + 6) * 4 + iF] = F_[iF];
//				Fn[(i + 6) * 4 + iF] = F_[iF];
//			}
//
//		}
//	}
//	int mF = 4;
//	int sizeF1, sizeF2; //dodati
//	for (int i = 0; i < sizeF2; i++)
//	{
//		for (int iF = 0; iF < mF; iF++)
//			F[i * 4 + nP[i] + 1 + iF] = 0;
//	}
//
//	float *Ffinal;
//	int br = 0;
//	for (int i = 7; i < sizeF2 - 7; i++)
//	{
//		for (int j = 0; j < 4; j++)
//		{
//			Ffinal[br] = F[i * 4 + j];
//			br++;
//		}
//	}
//}

bool PSGM::IsFlat(
	Array<int> surfelArray,
	float *N,
	float &d,
	Array<int> PtArray)
{
	MESH::Distribution PtDistribution;

	int *piPt = PtArray.Element;

	int iSurfel, iiSurfel;
	Surfel *pSurfel;
	QLIST::Index2 *pPtIdx;

	for (iiSurfel = 0; iiSurfel < surfelArray.n; iiSurfel++)
	{
		iSurfel = surfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		pPtIdx = pSurfel->PtList.pFirst;

		while (pPtIdx)
		{
			*(piPt++) = pPtIdx->Idx;

			pPtIdx = pPtIdx->pNext;
		}
	}

	PtArray.n = piPt - PtArray.Element;

	if (PtArray.n == 0)
		return false;

	pMesh->ComputeDistributionDouble(PtArray, PtDistribution);

	float *var = PtDistribution.var;

	int idx[3];
	int iTmp;
	float *N_;

	RVLSORT3ASCEND(var, idx, iTmp);

	if (var[idx[0]] / var[idx[1]] <= 0.0005)
	{
		N_ = PtDistribution.R + 3 * idx[0];

		if (N_[2] > 0.0f)
		{
			RVLNEGVECT3(N_, N);
		}
		else
		{
			RVLCOPY3VECTOR(N_, N);
		}

		d = RVLDOTPRODUCT3(N, PtDistribution.t);

		return true;
	}
	else
		return false;
}

void PSGM::DetectGroundPlane(SURFEL::ObjectGraph *pObjects)
{
	if (pObjects->sortedObjectArray.n < 0)
		pObjects->SortObjects();

	Array<int> PtArray;

	PtArray.Element = new int[pMesh->NodeArray.n];

	Array<int> iSurfelArray;

	iSurfelArray.Element = new int[pSurfels->NodeArray.n];

	bGnd = false;

	iGndObject = -1;

	int iObject;
	SURFEL::Object *pObject;

	for (iObject = 0; iObject < pObjects->objectArray.n; iObject++)
	{
		pObject = pObjects->objectArray.Element + iObject;

		QLIST::CopyToArray(&(pObject->surfelList), &iSurfelArray);

		if (IsFlat(iSurfelArray, NGnd, dGnd, PtArray))
		{
			iGndObject = iObject;

			bGnd = true;

			break;
		}			
	}

	delete[] PtArray.Element;
	delete[] iSurfelArray.Element;
}

bool PSGM::GravityReferenceFrames(
	QList<QLIST::Index> surfelList,
	RECOG::CTISet *pCTISet,
	CRVLMem *pMem_)
{
	if (!bGnd)
		return false;

	int n = 0;

	QLIST::Index *piSurfel = surfelList.pFirst;

	while (piSurfel)
	{
		n++;

		piSurfel = piSurfel->pNext;
	}

	if (n == 0)
		return false;

	float csSeparationAngle = cos(baseSeparationAngle * DEG2RAD);

	Array<SortIndex<float>> sortedSurfelArray;

	sortedSurfelArray.Element = new SortIndex<float>[n];

	float *UMem = new float[3 * n];
	float *VMem = new float[3 * n];

	float *U = UMem;
	float *V = VMem;

	SortIndex<float> *piSurfel_ = sortedSurfelArray.Element;

	sortedSurfelArray.n = 0;

	float ZGC[3];

	RVLCOPY3VECTOR(NGnd, ZGC);

	float ZSkew[9];

	RVLSKEW(ZGC, ZSkew);

	float J[6];

	float *Jx = J;
	float *Jy = J + 3;

	piSurfel = surfelList.pFirst;

	int iSurfel;
	float *N, *R, *X, *Y;
	Surfel *pSurfel;
	float lenV, fTmp;
	float A[9], B[9], CV[9];
	float stdx, stdy, varv, kx, ky;

	while (piSurfel)
	{
		iSurfel = piSurfel->Idx;

		//printf("Surfel %d\n", iSurfel);

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->flags & RVLSURFEL_FLAG_RF)
		{
			/// Computation of varv according to ARP3D.TR11.

			N = pSurfel->N;

			fTmp = RVLDOTPRODUCT3(N, NGnd);

			if (RVLABS(fTmp) <= csSeparationAngle)
			{
			R = pSurfel->R;

			X = R;

			Y = R + 3;

			stdx = 1.0f / pSurfel->r1;

			stdy = 1.0f / pSurfel->r2;

			// V <- NGnd x N
			RVLCROSSPRODUCT3(ZGC, N, V);

			// V <- V / || V ||
			// lenV <- || V ||
			RVLNORM3(V, lenV);

			kx = stdx / lenV;
			ky = stdy / lenV;

			// A <- V * V'
			RVLVECTCOV3(V, A);
			RVLCOMPLETESIMMX3(A);

			// B <- (I - A) * [NGnd]x
			RVLMXMUL3X3(A, ZSkew, B);
			RVLDIFMX3X3(ZSkew, B, B);

			// J <- (B * [X Y])'
			RVLMULMX3X3VECT(B, X, Jx);
			RVLMULMX3X3VECT(B, Y, Jy);

			// J <- stdx * J / lenV
			RVLSCALE3VECTOR(Jx, kx, Jx);
			RVLSCALE3VECTOR(Jy, ky, Jy);

			// CV <- J * J'
			RVLVECTCOV3(Jx, A);
			RVLVECTCOV3(Jy, B);
			RVLSUMMX3X3UT(A, B, CV);

			// U <- NGnd x V / || NGnd x V ||
			RVLCROSSPRODUCT3(ZGC, V, U);
			RVLNORM3(U, fTmp);

			// varv <- U' * CV * U
			varv = RVLCOV3DTRANSFTO1D(CV, U);

				piSurfel_->idx = sortedSurfelArray.n;
				piSurfel_->cost = varv;

				sortedSurfelArray.n++;

				U += 3;
				V += 3;

				piSurfel_++;
			}
		}	// if (pSurfel->flags & RVLSURFEL_FLAG_RF)

		piSurfel = piSurfel->pNext;
	}	// for every surfel in surfelList

	if (sortedSurfelArray.n == 0)
	{
		delete[] sortedSurfelArray.Element;
		delete[] UMem;
		delete[] VMem;

		return false;
	}

	if (sortedSurfelArray.n > 1)
		BubbleSort<SortIndex<float>>(sortedSurfelArray);

	RECOG::PSGM_::ModelInstance **ppFirstModelInstance = pCTISet->CTI.ppNext;

	int i;
	RECOG::PSGM_::ModelInstance *pModelInstance;
	float *t;

	for (i = 0; i < sortedSurfelArray.n; i++)
	{
		X = VMem + 3 * sortedSurfelArray.Element[i].idx;

		pModelInstance = *ppFirstModelInstance;

		while (pModelInstance)
		{
			if (RVLMULROWCOL3(X, pModelInstance->R, 0, 0) > csSeparationAngle)
				break;

			pModelInstance = pModelInstance->pNext;
		}

		if (pModelInstance)
			continue;

		RVLMEM_ALLOC_STRUCT(pMem_, RECOG::PSGM_::ModelInstance, pModelInstance);

		pCTISet->AddCTI(pModelInstance); //Vidovic

		R = pModelInstance->R;

		RVLCOPYTOCOL3(ZGC, 2, R);
		RVLCOPYTOCOL3(X, 0, R);

		Y = UMem + 3 * sortedSurfelArray.Element[i].idx;

		RVLCOPYTOCOL3(Y, 1, R);

		t = pModelInstance->t;

		RVLNULL3VECTOR(t);

		pModelInstance->varX = sortedSurfelArray.Element[i].cost;
	}

	delete[] sortedSurfelArray.Element;
	delete[] UMem;
	delete[] VMem;

	return true;
}

int PSGM::CTIs(
	QList<QLIST::Index> surfelList,
	Array<int> iVertexArray,
	int iModel,
	int iCluster,
	RECOG::CTISet *pCTISet,
	CRVLMem *pMem)
{
	// Generate CTI reference frames.

	RECOG::PSGM_::ModelInstance **ppCTI = pCTISet->CTI.ppNext;

	if (bGroundPlaneRFDescriptors)
	{
		float RGC[9];
		float varX;

		if (!GravityReferenceFrames(surfelList, pCTISet, pMem))
			return 0;
	}
	else
	{
		Array<int> iSurfelArray;

		iSurfelArray.Element = new int[pSurfels->NodeArray.n];

		QLIST::CopyToArray(&surfelList, &iSurfelArray);

		ReferenceFrames(iSurfelArray, pObjects->objectMap, iCluster);

		delete[] iSurfelArray.Element;
	}

	// Project vertices onto the ground plane.

	float *PGnd = NULL;

	if (bGnd && problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
	{
		PGnd = new float[3 * iVertexArray.n];

		float s;
		int i;
		float *PGnd_;
		int iVertex;
		SURFEL::Vertex *pVertex;
		float *P;

		for (i = 0; i < iVertexArray.n; i++)
		{
			PGnd_ = PGnd + 3 * i;

			iVertex = iVertexArray.Element[i];

			pVertex = pSurfels->vertexArray.Element[iVertex];

			P = pVertex->P;

			s = dGnd / RVLDOTPRODUCT3(NGnd, P);

			RVLSCALE3VECTOR(P, s, PGnd_);
		}
	}

	// Create CTI descriptors.

	int nCTIs = 0;

	RECOG::PSGM_::ModelInstance *pCTI = *ppCTI;

	while (pCTI)
	{
		pCTI->iCluster = iCluster;
		pCTI->iModel = iModel;

		FitModel(iVertexArray, pCTI, false, PGnd);

		nCTIs++;

		pCTI = pCTI->pNext;
	}

	RVL_DELETE_ARRAY(PGnd);

	return nCTIs;
}

void PSGM::CTIs(
	int iModel,
	SURFEL::ObjectGraph *pObjects,
	RECOG::CTISet *pCTISet,
	CRVLMem *pMem_)
{
	pCTISet->Init();

	if (!bGnd)
		DetectGroundPlane(pObjects);

	if (!bGnd)
		return;

	RVL_DELETE_ARRAY(pCTISet->SegmentCTIs.Element);

	pCTISet->SegmentCTIs.Element = new Array<int>[pObjects->objectArray.n];
	pCTISet->SegmentCTIs.n = pObjects->objectArray.n;

	int iObject;
	SURFEL::Object *pObject;

	for (iObject = 0; iObject < pObjects->objectArray.n; iObject++)
	{
		if (iObject != iGndObject)
		{
			//printf("Object %d:\n", iObject);

			pObject = pObjects->objectArray.Element + iObject;

			if (pObject->iVertexArray.n >= 3)
				pCTISet->SegmentCTIs.Element[iObject].n = CTIs(pObject->surfelList, pObject->iVertexArray, iModel, iObject, pCTISet, pMem_);
			else
				pCTISet->SegmentCTIs.Element[iObject].n = 0;			
		}
		else
			pCTISet->SegmentCTIs.Element[iObject].n = 0;
	}

	RVL_DELETE_ARRAY(pCTISet->segmentCTIIdxMem);

	pCTISet->segmentCTIIdxMem = new int[pCTISet->pCTI.n];

	int *iSegmentCTIIdx = pCTISet->segmentCTIIdxMem;

	RVL_DELETE_ARRAY(pCTISet->pCTI.Element);

	pCTISet->pCTI.Element = new RECOG::PSGM_::ModelInstance*[pCTISet->pCTI.n];

	QLIST::CreatePtrArray<RECOG::PSGM_::ModelInstance>(&(pCTISet->CTI), &(pCTISet->pCTI));

	for (iObject = 0; iObject < pObjects->objectArray.n; iObject++)
	{
		if (pCTISet->SegmentCTIs.Element[iObject].n > 0)
		{
			pCTISet->SegmentCTIs.Element[iObject].Element = iSegmentCTIIdx;

			iSegmentCTIIdx += pCTISet->SegmentCTIs.Element[iObject].n;

			pCTISet->SegmentCTIs.Element[iObject].n = 0;
		}
		else
			pCTISet->SegmentCTIs.Element[iObject].Element = NULL;
	}

	int iCTI;

	for (iCTI = 0; iCTI < pCTISet->pCTI.n; iCTI++)
	{
		iObject = pCTISet->pCTI.Element[iCTI]->iCluster;

		pCTISet->SegmentCTIs.Element[iObject].Element[pCTISet->SegmentCTIs.Element[iObject].n++] = iCTI;
	}
}

float PSGM::Symmetry(
	SURFEL::ObjectGraph *pObjects,
	int iObject1,
	int iObject2,
	RECOG::CTISet *pCTIs,
	Array<RECOG::PSGM_::SymmetryMatch> &symmetryMatch)
{
	bool bDebug = (iObject1 == debug1 && iObject2 == debug2 || iObject1 == debug2 && iObject2 == debug1);

	if (bDebug)
		int debug = 0;

	//bool bDebug = false;

	int iObject[2];

	iObject[0] = iObject1;
	iObject[1] = iObject2;

	SURFEL::Object *pObject[2];
	
	pObject[0] = pObjects->objectArray.Element + iObject[0];
	pObject[1] = pObjects->objectArray.Element + iObject[1];

	// Select the better RF.

	float *RGC = NULL;

	float varX = 0.0f;

	int iObject1_ = -1;

	RECOG::PSGM_::ModelInstance *pCTI;
	int i;

	for (i = 0; i < 2; i++)
	{
		if (pCTIs->SegmentCTIs.Element[iObject[i]].n > 0)
		{
			pCTI = pCTIs->pCTI.Element[pCTIs->SegmentCTIs.Element[iObject[i]].Element[0]];

			if (RGC == NULL || pCTI->varX < varX)
			{
				RGC = pCTI->R;

				varX = pCTI->varX;

				iObject1_ = i;
			}
		}
	}

	if (RGC == NULL)
		return 0.0f;

	int iObject2_ = 1 - iObject1_;

	// Select symmetry planes from convexTemplate.

	Array<int> iSymmetryPlanes;

	iSymmetryPlanes.Element = new int[convexTemplate.n];

	iSymmetryPlanes.n = 0;

	float *N;

	for (i = 0; i < convexTemplate.n; i++)
	{
		N = convexTemplate.Element[i].N;

		if (RVLABS(N[2]) < 1e-10)
			iSymmetryPlanes.Element[iSymmetryPlanes.n++] = i;
	}
	
	// 

	Array<int> iVertexArray[2];

	iVertexArray[0] = pObject[iObject1_]->iVertexArray;
	iVertexArray[1] = pObject[iObject2_]->iVertexArray;

	int nVertices2 = iVertexArray[1].n;

	pCTI = pCTIs->pCTI.Element[pCTIs->SegmentCTIs.Element[iObject[iObject1_]].Element[0]];

	//RECOG::PSGM_::ModelInstanceElement *pCTIElement1 = CTI1.modelInstance.Element;

	// Determine convex hull.

	//int *hull = new int[convexTemplate.n];

	//int hull_;

	//for (i = 0; i < convexTemplate.n; i++, pCTIElement1++, pCTIElement2++)
	//{
	//	hull_ = -1;

	//	if (pCTIElement1->valid)
	//		hull_ = 0;
	//	
	//	if (pCTIElement2->valid)
	//	{
	//		if (hull_ > 0)
	//		{
	//			if (pCTIElement2->d > pCTIElement1->d)
	//				hull_ = 1;
	//		}
	//		else
	//			hull_ = 1;
	//	}

	//	hull[i] = hull_;
	//}

	// Transform convex template to gravity RF.

	Array<RECOG::PSGM_::Plane> convexTemplateC;

	convexTemplateC.Element = new RECOG::PSGM_::Plane[convexTemplate.n];

	float *NC;

	for (i = 0; i < convexTemplate.n; i++)
	{
		N = convexTemplate.Element[i].N;

		NC = convexTemplateC.Element[i].N;

		RVLMULMX3X3VECT(RGC, N, NC);
	}

	/// Identify the symmetry plane.

	//BYTE *bVisible = new BYTE[nSymmetryPlanes];
	
	Array<RECOG::PSGM_::SymmetryMatch> symmetryMatch_;

	symmetryMatch_.Element = new RECOG::PSGM_::SymmetryMatch[convexTemplate.n];

	Array<SortIndex<float>> sortedSymmatryMatchIdx;

	sortedSymmatryMatchIdx.Element = new SortIndex<float>[convexTemplate.n];

	float *P2RMem = new float[3 * iVertexArray[1].n];
	
	float maxSymmetryScore = 0.0f;

	int iSymmetryPlane;
	float *NSymmetryPlaneG, *P;
	float NSymmetryPlaneC[3], NR[3];
	float k, d, absk;
	int iVertex;
	float *P2R;
	int j, jBest;
	float dMax;
	float sumw, halfSumw, t_;
	float fTmp;
	float symmetryScore;
	RECOG::PSGM_::SymmetryMatch *pSymmetryMatch;
	float e;
	int iBestSymmetryPlane;
	float dBestSymmetryPlane;
	RECOG::PSGM_::ModelInstanceElement *pCTIElement1;

	for (iSymmetryPlane = 0; iSymmetryPlane < iSymmetryPlanes.n; iSymmetryPlane++)
	//iSymmetryPlane = 15;
	{
		NSymmetryPlaneG = convexTemplate.Element[iSymmetryPlanes.Element[iSymmetryPlane]].N;

		RVLMULMX3X3VECT(RGC, NSymmetryPlaneG, NSymmetryPlaneC);

		// Compute mirror images of all vertices of iObject2.

		P2R = P2RMem;

		for (i = 0; i < nVertices2; i++, P2R += 3)
		{
			iVertex = iVertexArray[1].Element[i];

			P = pSurfels->vertexArray.Element[iVertex]->P;

			d = 2.0f * RVLDOTPRODUCT3(P, NSymmetryPlaneC);

			RVLSCALE3VECTOR(NSymmetryPlaneC, d, P2R);

			RVLDIF3VECTORS(P, P2R, P2R);
		}

		//memset(bVisible, 0, nSymmetryPlanes * sizeof(BYTE));
		
		// For every element of CTI of iObject1 identify the corresponding tangent to mirror images of the vertices of iObject2.

		symmetryMatch_.n = 0;

		sortedSymmatryMatchIdx.n = 0;

		sumw = 0.0f;

		pCTIElement1 = pCTI->modelInstance.Element;

		for (i = 0; i < convexTemplate.n; i++, pCTIElement1++)
		{
			if (!pCTIElement1->valid)
				continue;

			NC = convexTemplateC.Element[i].N;			

			//if (i < nSymmetryPlanes)
			//{
			//	k = RVLDOTPRODUCT3(NSymmetryPlaneG, N);

			//	bVisible[i] = (k > 0.0f ? 1 : -1);
			//}
			//else
			//{
			//	if (bVisible[i - nSymmetryPlanes] > 0)
			//		continue;
			//}

			k = RVLDOTPRODUCT3(NSymmetryPlaneC, NC);

			if (k > -1e-6)
			{
				fTmp = 2.0f * k;

				RVLSCALE3VECTOR(NSymmetryPlaneC, fTmp, NR);

				RVLDIF3VECTORS(NC, NR, NR);

				P2R = P2RMem;

				dMax = RVLDOTPRODUCT3(NC, P2R);

				jBest = 0;

				for (j = 1; j < nVertices2; j++, P2R += 3)
				{
					d = RVLDOTPRODUCT3(NC, P2R);

					if (d > dMax)
					{
						dMax = d;

						jBest = j;
					}
				}

				//iVertex = iVertexArray[1].Element[jBest];

				//P = pSurfels->vertexArray.Element[iVertex]->P;

				//if (RVLDOTPRODUCT3(P, NR) < 0.0f)
				{
					absk = RVLABS(k);

					pSymmetryMatch = symmetryMatch_.Element + symmetryMatch_.n;

					pSymmetryMatch->d = dMax;
					pSymmetryMatch->w = k;
					pSymmetryMatch->iCTIElement = i;
					pSymmetryMatch->b = (absk > 1e-6);	// normal of the CTI element is not parallel to the symmetry plane

					if (pSymmetryMatch->b)
					{
						sortedSymmatryMatchIdx.Element[sortedSymmatryMatchIdx.n].cost = (pCTIElement1->d - dMax) / k;
						sortedSymmatryMatchIdx.Element[sortedSymmatryMatchIdx.n].idx = symmetryMatch_.n;
						sortedSymmatryMatchIdx.n++;
						sumw += absk;
					}

					symmetryMatch_.n++;
				}
			}	// if (NSymmetryPlaneC' * NC > -1e-6)
		}	// for every element of convexTemplate

		// Compute optimal symmetry plane offset.

		if (sortedSymmatryMatchIdx.n > 0)
		{
			BubbleSort<SortIndex<float>>(sortedSymmatryMatchIdx);

			halfSumw = 0.5f * sumw;

			sumw = 0.0f;

			for (i = 0; i < sortedSymmatryMatchIdx.n && sumw < halfSumw; i++)
			{
				k = symmetryMatch_.Element[sortedSymmatryMatchIdx.Element[i].idx].w;

				sumw += RVLABS(k);
			}
				
			t_ = sortedSymmatryMatchIdx.Element[i].cost;
		}

		// Compute symmetry score.		

		symmetryScore = 0.0f;

		for (i = 0; i < symmetryMatch_.n; i++)
		{
			pSymmetryMatch = symmetryMatch_.Element + i;

			e = (pSymmetryMatch->d + pSymmetryMatch->w * t_ - pCTI->modelInstance.Element[pSymmetryMatch->iCTIElement].d) / symmetryMatchThr;

			e *= e;

			if (e < 1.0f)
			{
				e = 1.0f - e;

				symmetryScore += e;

				pSymmetryMatch->w = e;
				pSymmetryMatch->b = true;
			}
			else
				pSymmetryMatch->b = false;
		}

		if (symmetryScore > maxSymmetryScore)
		{
			maxSymmetryScore = symmetryScore;

			iBestSymmetryPlane = iSymmetryPlane;

			dBestSymmetryPlane = -0.5f * t_;

			symmetryMatch.n = 0;

			for (i = 0; i < symmetryMatch_.n; i++)
				if (symmetryMatch_.Element[i].b)
					symmetryMatch.Element[symmetryMatch.n++] = symmetryMatch_.Element[i];
		}
	}	// for each symmetry plane

	///

	// Save the results to a file. 

	if (bDebug)
	{
		FILE *fp = fopen("symmetry.txt", "w");

		PrintMatrix<float>(fp, RGC, 3, 3);

		NSymmetryPlaneG = convexTemplate.Element[iSymmetryPlanes.Element[iBestSymmetryPlane]].N;

		//NSymmetryPlaneG = convexTemplate.Element[22].N;

		RVLMULMX3X3VECT(RGC, NSymmetryPlaneG, NSymmetryPlaneC);

		PrintMatrix<float>(fp, NSymmetryPlaneC, 1, 3);

		fprintf(fp, "%f\t%d\t%d\t\n", dBestSymmetryPlane, iVertexArray[0].n, iVertexArray[1].n);

		SURFEL::Vertex *pVertex;

		for (j = 0; j < 2; j++)
		{
			for (i = 0; i < iVertexArray[j].n; i++)
			{
				pVertex = pSurfels->vertexArray.Element[iVertexArray[j].Element[i]];

				fprintf(fp, "%f\t%f\t%f\t\n", pVertex->P[0], pVertex->P[1], pVertex->P[2]);
			}
		}

		fclose(fp);
	}

	// Free memory.

	//delete[] bVisible;
	delete[] P2RMem;
	delete[] symmetryMatch_.Element;
	delete[] iSymmetryPlanes.Element;
	delete[] sortedSymmatryMatchIdx.Element;

	// Return the symmetry score.

	return maxSymmetryScore;
}

#ifdef NEVER
void PSGM::RVLPSGInstanceMesh(Eigen::MatrixXf nI, float *dI)
{
#ifdef RVLPSGM_CTIMESH_DEBUG
	FILE *fp = fopen("CTIMeshDebug.txt", "w");
#endif

	//transform nI and dI to Eigen:
	Eigen::MatrixXf nIE = nI;
	Eigen::MatrixXf dIE(1, 66);

	//if nI was an array
	//Eigen::MatrixXf nIE(3, 66);
	//int br = 0;
	//for (int i = 0; i < 3; i++)
	//{
	//	for (int j = 0; j < 66; j++)
	//	{
	//		nIE(i, j) = nI[br];
	//		br++;
	//	}
	//}
	for (int i = 0; i < 66; i++)
	{
		dIE(0, i) = dI[i];
	}

	float noise = 1e-6;
	int nF = nIE.cols();
	float halfCubeSize;

	float max, min;
	max = dIE.maxCoeff();
	min = dIE.minCoeff();
	if (min<0 && min*-1 > max)
		max = min;
	halfCubeSize = max*1.1;

	P.resize(3, 8);
	P << 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, -1, -1;
	P = halfCubeSize*P;

	Eigen::MatrixXi Premoved = Eigen::MatrixXi::Zero(1, P.cols());//list of removed vertices
	Eigen::MatrixXi nP = Eigen::MatrixXi::Ones(nF + 6, 1);
	nP = 4 * nP;

	F = Eigen::MatrixXi::Zero(nF + 6, 66 * 66);
	//F.block<6, 4>(0, 0) << 1, 3, 4, 2, 3, 7, 8, 4, 2, 4, 8, 6, 5, 6, 8, 7, 1, 2, 6, 5, 1, 5, 7, 3;
	F.block<6, 4>(0, 0) << 0, 2, 3, 1, 2, 6, 7, 3, 1, 3, 7, 5, 4, 5, 7, 6, 0, 1, 5, 4, 0, 4, 6, 2;


	Eigen::MatrixXi E = Eigen::MatrixXi::Ones(nF + 6, nF + 6);
	E *= -1;
	Eigen::MatrixXi Fn = Eigen::MatrixXi::Zero(nF + 6, 66 * 66);

	int iP1, iP2;
	int l;
	int br2;
	int NextCirc, PrevCirc;
	for (int i_ = 0; i_ < 6; i_++) //for every face
	{
		int i, j;
		i = i_;
		for (int k = 0; k < 4; k++)
		{
			NextCirc = (k + 1) % 4;
			iP1 = F(i, k);
			iP2 = F(i, NextCirc);

			for (int j_ = i_ + 1; j_ < 6; j_++)
			{
				j = j_;
				l = -1;
				for (int iF = 0; iF < nP(j); iF++) //find(F(j,:)==iP1)
				{
					if (F(j, iF) == iP1)
					{
						l = iF;
						break;
					}
				}
				if (l >= 0)
				{
					PrevCirc = (l + 3) % 4;
					if (F(j, PrevCirc) == iP2)
					{
						E(i, j) = iP1;
						E(j, i) = iP2;
						Fn(i, k) = j;
						Fn(j, PrevCirc) = i;
					}
				}
			}
		}
	}

#ifdef RVLPSGM_CTIMESH_DEBUG
	PrintCTIMeshFaces(fp, F, Fn, 6, nP);

	fprintf(fp, "\n\n\n");

	fclose(fp);
#endif

	Eigen::MatrixXi iNewVertices;
	Eigen::MatrixXi iNeighbors;
	Eigen::MatrixXf N;
	Eigen::MatrixXf dCut, dCutSorted;
	float d, d_;
	int nCut;
	Eigen::MatrixXf Temp;
	Eigen::MatrixXf Tempnext;
	Eigen::MatrixXf Temp2;
	for (int i = 0; i < nF; i++) //for every face
	{
		printf("%d\n", i);

#ifdef RVLPSGM_CTIMESH_DEBUG
		fprintf(fp, "i = %d\n\n\n", i);
#endif
		iNewVertices.resize(0, 0);
		iNeighbors.resize(0, 0);
		N = nIE.block<3, 1>(0, i); //normal of the i-th face
		d = dIE(0, i); //distance of the i-th face
		dCut = Eigen::MatrixXf::Zero(P.cols(), 1);
		dCutSorted = Eigen::MatrixXf::Zero(P.cols(), 1);
		nCut = 0;


		for (int k = 0; k < 8; k++)
		{
			Temp = N.transpose()*P.block<3, 1>(0, k);
			d_ = Temp(0, 0) - d;
			if (d_ > 0)
			{
				nCut += 1;
				dCut(nCut, 0) = d_;
			}
		}

		//Bubble sort:
		float temp;
		for (int idCut = 0; idCut < nCut; idCut++)
		{
			for (int jdCut = 0; jdCut < nCut; jdCut++)
			{
				if (dCut(jdCut, 0)>dCut(jdCut + 1, 0))
				{
					temp = dCut(jdCut, 0);
					dCut(jdCut, 0) = dCut(jdCut + 1, 0);
					dCut(jdCut + 1, 0) = temp;
				}
			}
		}

		float dCorr = 0;
		for (int k = 0; k < nCut; k++)
		{
			if (dCut(k, 0) - dCorr < noise)
				dCorr = dCut(k, 0);

		}
		d += dCorr;

		Eigen::MatrixXi F_;//j-th face
		Eigen::MatrixXi Fn_;//neighbors of F_
		Eigen::MatrixXf P_; //position vector of vector iP
		Eigen::MatrixXf Pnext;//position vector of vertex iPNext
		int nP_; //number of vertices od F_
		int iP; // k-th vertex of F_
		int iPNext; //next vertex
		Eigen::MatrixXf dP;
		int L; //neighbor of F_ on the opposite side of edge iP-iPNext

		int iPolygon, iVertex;
		int iPNew, iPNewVertex, iFNewVertex;
		float s;


		for (int j = 0; j <= i + 5; j++) //for every previously considered face
		{
			F_ = F.block(j, 0, 1, F.cols());
			Fn_ = Fn.block(j, 0, 1, Fn.cols());
			int ff = F(j, 0);
			int k = 0;
			nP_ = nP(j, 0);

			for (int k_ = 0; k_ < nP_; k_++)
			{
				int p = P.cols();
				iP = F_(0, k_);
				P_ = P.block(0, iP, P.rows(), 1);


				if (k_ == nP_ - 1) NextCirc = 0;
				else NextCirc = k_ % (nP_)+1;

				iPNext = F_(0, NextCirc);
				Pnext = P.block(0, iPNext, P.rows(), 1);

				dP.resize(P.rows(), 1);
				dP = Pnext - P_;

				L = Fn_(0, k_);

				Temp = N.transpose()*P_;
				Tempnext = N.transpose()*Pnext;

				if (i == 1 && j == 2)
					int debug = 1;


				if (Temp(0, 0) > d) //if iP is over new plane
				{
					if (Premoved(0, iP) == 0)
						Premoved(0, iP) = 1; //vertex iP is removed

					iPolygon = j;
					iVertex = k;

					//Remove Vertex from Polygon:
					for (int iF = 0; iF < (nP(iPolygon) - 1 - iVertex); iF++)
					{
						F(iPolygon, iVertex + iF) = F(iPolygon, iVertex + iF + 1);
						Fn(iPolygon, iVertex + iF) = Fn(iPolygon, iVertex + iF + 1);
					}
					nP(iPolygon) = nP(iPolygon) - 1;
#ifdef RVLPSGM_CTIMESH_DEBUG
					fp = fopen("CTIMeshDebug.txt", "a");

					fprintf(fp, "j = %d, k_ = %d\n\n", j, k_);

					PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

					fprintf(fp, "\n\n\n");

					fclose(fp);
#endif

					if (Tempnext(0, 0) > d)
					{
						E(L, j) = -1;
						E(j, L) = -1;
						k -= 1;
					}
					else
					{
						if (E(j, L) == iP) //Vertex is not updated
						{
							//Add new vertex
							Temp = N.transpose()*P_;
							Temp2 = N.transpose()*dP;
							s = (d - Temp(0, 0)) / Temp2(0, 0);
							Eigen::MatrixXf Ptemp = P;
							P.resize(P.rows(), P.cols() + 1);
							P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
							Eigen::Vector3f col = P_ + s*dP;
							P.col(P.cols() - 1) = col;
							iPNew = P.cols() - 1;

							Eigen::MatrixXi Premovedtemp = Premoved;
							Premoved.resize(1, Premoved.cols() + 1);
							Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
							Premoved(0, Premoved.cols() - 1) = 0;
							E(j, L) = iPNew;
						}
						else
							iPNew = E(j, L);

						iPolygon = j;
						iVertex = k;
						iPNewVertex = iPNew;
						iFNewVertex = L;
						//Add new Vertex to Polygon:
						Eigen::MatrixXi Ftemp = F;
						Eigen::MatrixXi Fntemp = Fn;
						for (int iF = 0; iF < (nP(iPolygon) - iVertex); iF++)
						{
							F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon, iVertex + iF);
							Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
						}
						F(iPolygon, iVertex) = iPNewVertex;
						Fn(iPolygon, iVertex) = iFNewVertex;
						nP(iPolygon) = nP(iPolygon) + 1;
#ifdef RVLPSGM_CTIMESH_DEBUG
						fp = fopen("CTIMeshDebug.txt", "a");

						fprintf(fp, "j = %d, k_ = %d\n\n", j, k_);

						PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

						fprintf(fp, "\n\n\n");

						fclose(fp);
#endif

						Eigen::MatrixXi iNewVerticestemp = iNewVertices;
						iNewVertices.resize(1, iNewVertices.cols() + 1);
						iNewVertices.block(0, 0, iNewVerticestemp.rows(), iNewVerticestemp.cols()) = iNewVerticestemp;
						iNewVertices(0, iNewVertices.cols() - 1) = iPNew;


						Eigen::MatrixXi iNeighborstemp = iNeighbors;
						iNeighbors.resize(1, iNeighbors.cols() + 1);
						iNeighbors.block(0, 0, iNeighborstemp.rows(), iNeighborstemp.cols()) = iNeighborstemp;
						iNeighbors(0, iNeighbors.cols() - 1) = j;

						E((i + 6), j) = iPNew;
					}
				}

				else if (Tempnext(0, 0) > d)
				{
					if (E(L, j) == iPNext) //Vertex is not updated
					{
						Temp = N.transpose()*P_;
						Temp2 = N.transpose()*dP;
						s = (d - Temp(0, 0)) / Temp2(0, 0);


						Eigen::MatrixXf Ptemp = P;
						P.resize(P.rows(), P.cols() + 1);
						P.block(0, 0, Ptemp.rows(), Ptemp.cols()) = Ptemp;
						Eigen::Vector3f col = P_ + s*dP;
						P.col(P.cols() - 1) = col;
						iPNew = P.cols() - 1;

						Eigen::MatrixXi Premovedtemp = Premoved;
						Premoved.resize(1, Premoved.cols() + 1);
						Premoved.block(0, 0, Premovedtemp.rows(), Premovedtemp.cols()) = Premovedtemp;
						Premoved(0, Premoved.cols() - 1) = 0;

						E(L, j) = iPNew;
					}
					else
						iPNew = E(L, j);

					iPolygon = j;
					iVertex = k + 1;
					iPNewVertex = iPNew;
					iFNewVertex = i + 6;
					//Add new Vertex to Polygon:
					Eigen::MatrixXi Ftemp = F;
					Eigen::MatrixXi Fntemp = Fn;
					for (int iF = 0; iF < (nP(iPolygon) - iVertex); iF++)
					{
						F(iPolygon, iVertex + iF + 1) = Ftemp(iPolygon, iVertex + iF);
						Fn(iPolygon, iVertex + iF + 1) = Fntemp(iPolygon, iVertex + iF);
					}
					F(iPolygon, iVertex) = iPNewVertex;
					Fn(iPolygon, iVertex) = iFNewVertex;
					nP(iPolygon) = nP(iPolygon) + 1;
#ifdef RVLPSGM_CTIMESH_DEBUG
					fp = fopen("CTIMeshDebug.txt", "a");

					fprintf(fp, "j = %d, k_ = %d\n\n", j, k_);

					PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

					fprintf(fp, "\n\n\n");

					fclose(fp);
#endif

					E(j, i + 6) = iPNew;
					k = k + 1;
				}
				k = k + 1;
			}

#ifdef RVLPSGM_CTIMESH_DEBUG
			fp = fopen("CTIMeshDebug.txt", "a");

			fprintf(fp, "j = %d\n\n", j);

			PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

			fprintf(fp, "\n\n\n");

			fclose(fp);
#endif
		}	 //for every previously considered face
		int iNeighbor;
		nP(i + 6) = iNewVertices.cols(); //rows

		int m;
		if (nP(i + 6) > 0)
		{
			Eigen::MatrixXi F_;
			Eigen::MatrixXi Fn_;
			m = 0;
			while (1)
			{
				Eigen::MatrixXi F_temp = F_;
				F_.resize(1, F_.cols() + 1);
				F_.block(0, 0, F_temp.rows(), F_temp.cols()) = F_temp;
				F_(0, F_.cols() - 1) = iNewVertices(m);

				int iN = iNeighbors(0, m);
				iNeighbor = iNeighbors(0, m);


				Eigen::MatrixXi Fn_temp = Fn_;
				Fn_.resize(1, Fn_.cols() + 1);
				Fn_.block(0, 0, Fn_temp.rows(), Fn_temp.cols()) = Fn_temp;
				Fn_(0, Fn_.cols() - 1) = iNeighbor;


				iPNext = E(iNeighbor, (i + 6));
				int iNV;
				for (iNV = 0; iNV < iNewVertices.cols(); iNV++)
				{
					int a = iPNext;
					int b = iNewVertices(iNV);
					if (iNewVertices(iNV) == iPNext)
					{
						m = iNV;
						break;
					}
				}
				if (m == 0)
					break;
			}

			for (int iF = 0; iF < F_.cols(); iF++)
			{
				F((i + 6), iF) = F_(0, iF);
				Fn((i + 6), iF) = Fn_(0, iF);
			}

		}

#ifdef RVLPSGM_CTIMESH_DEBUG
		fp = fopen("CTIMeshDebug.txt", "a");

		PrintCTIMeshFaces(fp, F, Fn, i + 7, nP);

		fprintf(fp, "\n\n\n");

		fclose(fp);
#endif
	}	 //for every face
	int mF = F.cols();

	for (int i = 0; i < F.rows(); i++)
	{
		for (int iF = 0; iF < mF; iF++)
			F(i, nP(i) + 1 + iF) = 0;
	}

	Eigen::MatrixXi Ffinal = F.block((F.rows() - 6), F.cols(), 6, 0);
	Edges = E;

#ifdef RVLPSGM_CTIMESH_DEBUG
	fclose(fp);
#endif
}

void PSGM::PrintCTIMeshFaces(FILE *fp, Eigen::MatrixXi F, Eigen::MatrixXi Fn, int n, Eigen::MatrixXi nP)
{
	fprintf(fp, "F:\n");

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < nP(i); j++)
			fprintf(fp, "%d\t", F(i, j));

		fprintf(fp, "\n");
	}

	fprintf(fp, "\n");

	fprintf(fp, "Fn:\n");

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < nP(i); j++)
			fprintf(fp, "%d\t", Fn(i, j));

		fprintf(fp, "\n");
	}

	fprintf(fp, "\n");
}

#endif

void PSGM::BoundingBoxSize(
	RECOG::PSGM_::ModelInstance *pBoundingBox,
	float *size)
{
	float a;
	a = pBoundingBox->modelInstance.Element[4].d + pBoundingBox->modelInstance.Element[1].d;
	size[0] = RVLABS(a);
	a = pBoundingBox->modelInstance.Element[5].d + pBoundingBox->modelInstance.Element[2].d;
	size[1] = RVLABS(a);	
	a = pBoundingBox->modelInstance.Element[0].d + pBoundingBox->modelInstance.Element[3].d;
	size[2] = RVLABS(a);
}

//Return a consensus of hypotheses where no one is in collision
void PSGM::GetHypothesesCollisionConsensus(std::vector<int> *noCollisionHypotheses, Array<Array<SortIndex<float>>> *scoreMatchMatrix, float thr)
{
	//std::vector<int> chosenHypotheses;
	std::set<int> finishedSeg;
	int nSegments; //Vidovic
	//get sorted list of hypothesis
	////helper stuff////
	struct Hyp{
		int idSeg;
		int idMatch;
		float score;
		Hyp(int ids, int idm, float s) : idSeg(ids), idMatch(idm), score(s) {}
		bool operator < (const Hyp& other) const
		{
			return (score < other.score);
		}
	};
	////////////////////
	//fill the list
	std::vector<Hyp> hypotheses;

	/*
#ifdef RVLPSGM_ICP
	nSegments = scoreMatchMatrixICP.n;

	for (int i = 0; i < scoreMatchMatrixICP.n; i++)
	{
		for (int j = 0; j < RVLMIN(1, scoreMatchMatrixICP.Element[i].n); j++)
			if (scoreMatchMatrixICP.Element[i].Element[j].idx >= 0)
				hypotheses.push_back(Hyp(i, scoreMatchMatrixICP.Element[i].Element[j].idx, scoreMatchMatrixICP.Element[i].Element[j].cost));
	}
#else
	nSegments = scoreMatchMatrix.n;

	for (int i = 0; i < scoreMatchMatrix.n; i++)
	{
		for (int j = 0; j < RVLMIN(nBestMatches, scoreMatchMatrix.Element[i].n); j++)
			if (scoreMatchMatrix.Element[i].Element[j].idx >= 0)
				hypotheses.push_back(Hyp(i, scoreMatchMatrix.Element[i].Element[j].idx, scoreMatchMatrix.Element[i].Element[j].cost));
	}
#endif*/
	nSegments = scoreMatchMatrix->n;
	int j;

	for (int i = 0; i < scoreMatchMatrix->n; i++)
	{
		for (j = 0; j < RVLMIN(1, scoreMatchMatrix->Element[i].n); j++)
			//if (scoreMatchMatrix->Element[i].Element[j].idx >= 0)
			if (!(std::find(transparentHypotheses.begin(), transparentHypotheses.end(), scoreMatchMatrix->Element[i].Element[j].idx) != transparentHypotheses.end()))
				hypotheses.push_back(Hyp(i, scoreMatchMatrix->Element[i].Element[j].idx, scoreMatchMatrix->Element[i].Element[j].cost));
	}

	//Sort it
	std::sort(hypotheses.begin(), hypotheses.end());

	//Add first hypothesis
	//chosenHypotheses.push_back(hypotheses.at(0).idMatch);
	noCollisionHypotheses->push_back(hypotheses.at(0).idMatch); //Vidovic test
	finishedSeg.insert(hypotheses.at(0).idSeg);
	int currentMatch;
	int currentSeg;
	bool collision;
	for (int i = 0; i < hypotheses.size(); i++)
	{
		currentSeg = hypotheses.at(i).idSeg;
		//Check is that segment is already finished
		if (finishedSeg.count(currentSeg))
			continue;
		currentMatch = hypotheses.at(i).idMatch;
		//Check collision with chosen hypotheses
		collision = false;
		//for (int h = 0; h < chosenHypotheses.size(); h++)
		for (int h = 0; h < noCollisionHypotheses->size(); h++)
		{
			//collision = CheckHypothesesCollision(currentMatch, chosenHypotheses.at(h), thr);
			collision = CheckHypothesesCollision(currentMatch, noCollisionHypotheses->at(h), thr);
			if (collision)
				break;
		}
		if (collision)
			continue;
		//else add it to the list of the chosen
		//chosenHypotheses.push_back(currentMatch);
		noCollisionHypotheses->push_back(currentMatch); //Vidovic test
		finishedSeg.insert(currentSeg);
		//check if all segments are finished
		if (finishedSeg.size() == nSegments)
			break;

	}

	//memcpy(&noCollisionHypotheses, &chosenHypotheses, chosenHypotheses.size() * sizeof(std::vector<int>)); //Vidovic

	//return chosenHypotheses;
}

//Check if two objects are in collision. Return true if they are.
bool PSGM::CheckHypothesesCollision(int firstHyp, int secondHyp, float thr)
{
	bool inCollision = false;
	//First hypothesis data
	RECOG::PSGM_::MatchInstance* firstMatch = pCTImatchesArray.Element[firstHyp];
	VertexGraph * firstVG = MTGSet.vertexGraphs.at(MCTISet.pCTI.Element[firstMatch->iMCTI]->iModel);
	TG* firstTG = MTGSet.TGs.at(MCTISet.pCTI.Element[firstMatch->iMCTI]->iModel);
	//Second hypothesis data
	RECOG::PSGM_::MatchInstance* secondMatch = pCTImatchesArray.Element[secondHyp];
	VertexGraph * secondVG = MTGSet.vertexGraphs.at(MCTISet.pCTI.Element[secondMatch->iMCTI]->iModel);
	TG* secondTG = MTGSet.TGs.at(MCTISet.pCTI.Element[secondMatch->iMCTI]->iModel);
	float R12[9], t12[3], R21[9], t21[3], V3Tmp[3];

#ifdef RVLPSGM_ICP
	RVLCOMPTRANSF3DWITHINV(secondMatch->RICP, secondMatch->tICP, firstMatch->RICP, firstMatch->tICP, R12, t12, V3Tmp);
#else
	RVLCOMPTRANSF3DWITHINV(secondMatch->R, secondMatch->t, firstMatch->R, firstMatch->t, R12, t12, V3Tmp);
#endif

	RVLINVTRANSF3D(R12, t12, R21, t21);
	SURFEL::Vertex * rvlvertex;
	TGNode* plane;
	float tP[3];
	float dist;
	float* N;

	//Check second to first
	//for each plane
	float* minDistPlane = new float[firstTG->A.h + secondTG->A.h];
	//memset(minDistPlane, 1, (firstTG->A.h + secondTG->A.h)*sizeof(float)); //some big value
	for (int j = 0; j < firstTG->A.h; j++)
	{
		plane = firstTG->descriptor.Element[j].pFirst->ptr;
		//For each point
		minDistPlane[j] = 1000000;
		for (int i = 0; i < secondVG->NodeArray.n; i++)
		{
			//Transform vertex to first model space
			rvlvertex = &secondVG->NodeArray.Element[i];
			RVLTRANSF3(rvlvertex->P, R21, t21, tP);
			//distance
			N = &firstTG->A.Element[firstTG->A.w * plane->i];
			dist = RVLDOTPRODUCT3(N, tP) - plane->d;
			if (dist < minDistPlane[j])
				minDistPlane[j] = dist;
		}

	}
	////find max
	//float maxVal = -1000000;
	//for (int i = 0; i < firstTG->A.h; i++)
	//{
	//	if (minDistPlane[i] > maxVal)
	//		maxVal = minDistPlane[i];
	//}
	//delete[] minDistPlane;
	////Check
	//if (maxVal < -thr)
	//	return true;	
	

	//Check first to second
	//for each plane
	//minDistPlane = new float[secondTG->A.h];
	//memset(minDistPlane, 1, secondTG->A.h*sizeof(float)); //some big value
	for (int j = 0; j < secondTG->A.h; j++)
	{
		plane = secondTG->descriptor.Element[j].pFirst->ptr;
		//For each point
		minDistPlane[firstTG->A.h + j] = 1000000;
		for (int i = 0; i < firstVG->NodeArray.n; i++)
		{
			//Transform vertex to second model space
			rvlvertex = &firstVG->NodeArray.Element[i];
			RVLTRANSF3(rvlvertex->P, R12, t12, tP);
			//distance
			N = &secondTG->A.Element[secondTG->A.w * plane->i];
			dist = RVLDOTPRODUCT3(N, tP) - plane->d;
			if (dist < minDistPlane[firstTG->A.h + j])
				minDistPlane[firstTG->A.h + j] = dist;
		}

	}
	//find max
	float maxVal = -1000000;
	for (int i = 0; i < firstTG->A.h + secondTG->A.h; i++)
	{
		if (minDistPlane[i] > maxVal)
			maxVal = minDistPlane[i];
	}
	delete[] minDistPlane;
	//Check
	if (maxVal < -thr)
		return true;

	return inCollision;
}

float PSGM::GetObjectTransparencyRatio(vtkSmartPointer<vtkPolyData> object, unsigned short *depthImg, float depthThr, int width, int height, float c_fu, float c_fv, float c_uc, float c_vc)
{
	//get vtk data
	vtkSmartPointer<vtkPoints> points = object->GetPoints();
	vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(object->GetPointData()->GetNormals());

	float sumW_T = 0;
	float sumW_A = 0;
	int noPts = points->GetNumberOfPoints();
	double point[3];
	float pointN[3];
	float norm;
	float normal[3];
	int u;
	int v;
	float w;
	//vizualization
	/*cv::Mat depth(480, 640, CV_16UC1, cv::Scalar::all(0));
	memcpy(depth.data, depthImg, 640 * 480 * sizeof(short));
	cv::Mat depthShow(480, 640, CV_8UC1);
	double minVal, maxVal;*/

	for (int i = 0; i < noPts; i++)
	{
		//Get point
		points->GetPoint(i, point);
		pointN[0] = point[0];
		pointN[1] = point[1];
		pointN[2] = point[2];
		//Get u, v;
		u = c_fu * point[0] / point[2] + c_uc;
		v = c_fv * point[1] / point[2] + c_vc;
		//Check if the point is within scene (image)
		if ((u < 0) || (u >= width) || (v < 0) || (v >= height))
			continue;
		//vizualization
		//depth.at<unsigned short>(v, u) = (point[2] * 1000) - depthImg[v * width + u];

		//Get normal
		normals->GetTupleValue(i, normal);
		RVLNORM3(pointN, norm);
		w = abs(RVLDOTPRODUCT3(normal, pointN));

		//if (w < 0.2)
		//	w = 0.0;
		//check transparency
		if (depthImg[v * width + u] - (point[2] * 1000) > depthThr)
		{
			//new vizualization
			/*if (w > 0)
				depth.at<unsigned short>(v, u) = 0;*/

			sumW_T += w;
		}
		sumW_A += w;
		
	}
	//std::cout << "Transparency: " << sumW_T / sumW_A << endl;
	////vizualization
	//cv::minMaxLoc(depth, &minVal, &maxVal);
	//depth.convertTo(depthShow, CV_8U, -255.0f / maxVal, 255.0f);
	//cv::imshow("Transparency test", depthShow);
	//cv::waitKey();

	

	return sumW_T / sumW_A;
}

//Requires scoreMatchMatrixICP???
void PSGM::FilterHypothesesUsingTransparency(float tranThr, float depthThr, bool verbose)
{
	transparentHypotheses.clear(); //Vidovic

	RECOG::PSGM_::MatchInstance *pMatch; //Vidovic

	CreateDilatedDepthImage();

	float tranRatio;
	for (int i = 0; i < scoreMatchMatrixICP.n; i++)
	{
		for (int j = 0; j < RVLMIN(nBestMatches, scoreMatchMatrixICP.Element[i].n); j++) //constant 7 changed to nBestMatches - Vidovic
		{
			if (scoreMatchMatrixICP.Element[i].Element[j].idx >= 0)
			{
				vtkSmartPointer<vtkPolyData> object = GetPoseCorrectedVisibleModel(scoreMatchMatrixICP.Element[i].Element[j].idx);
				//tranRatio = GetObjectTransparencyRatio(object, this->depthImg, depthThr, 640, 480, 525, 525, 320, 240); //HARDCODED FOR ECCV DATASET CAMERA
				tranRatio = GetObjectTransparencyRatio(object, (unsigned short *)depth.data, depthThr, 640, 480, 525, 525, 320, 240); //HARDCODED FOR ECCV DATASET CAMERA

				//save transparency ratio to match instance - Vidovic
				pMatch = GetMatch(scoreMatchMatrixICP.Element[i].Element[j].idx);
				pMatch->transparencyRatio = tranRatio;

				//transparency penalization - Vidovic
				pMatch->cost_NN += RVLABS(pMatch->transparencyRatio) * pMatch->cost_NN;

				if (tranRatio > tranThr)
				{
					if (verbose)
					{
						std::cout << "Hypothesis " << scoreMatchMatrixICP.Element[i].Element[j].idx << " Model: " << MCTISet.pCTI.Element[pCTImatchesArray.Element[scoreMatchMatrixICP.Element[i].Element[j].idx]->iMCTI]->iModel << " for segment " << i << " has been invalidated by transparency ratio of: " << tranRatio << "(rank: " << j << ")" << std::endl;
					}
					transparentHypotheses.push_back(scoreMatchMatrixICP.Element[i].Element[j].idx);
					scoreMatchMatrixICP.Element[i].Element[j].idx = -1;
				}
				//else //Commented to get transparency ratio for all matches - Vidovic
				//	break; //Break transparency check for this segment
			}
		}
	}
}

//Uses T_ICP transformation matrix
vtkSmartPointer<vtkPolyData> PSGM::GetPoseCorrectedVisibleModel(int iMatch)
{
	//Get model instance
	RECOG::PSGM_::ModelInstance *pMCTI;
	pMCTI = MCTISet.pCTI.Element[pCTImatchesArray.Element[iMatch]->iMCTI];
	int iModel = pMCTI->iModel;
	
	//Gat match instance (and calculate pose??? Is this necessary???)
	RECOG::PSGM_::MatchInstance *pMatch = pCTImatchesArray.Element[iMatch];
	CalculatePose(iMatch);
	
	//Set transform
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	double T_M_S[16], tICPm[3];

	//Commented on 14.06.2017 - Vidovic
	//for (int i = 0; i < 16; i++)
	//	T_M_S[i] = pMatch->T_ICP[i];	//FROM ICP???

	//Added on 14.06.2017. - Vidovic
	RVLSCALE3VECTOR2(pMatch->tICP, 1000, tICPm);
	RVLHTRANSFMX(pMatch->RICP, tICPm, T_M_S);

	transform->SetMatrix(T_M_S);

	//Scaling PLY model to meters
	vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
	transformScale->Scale(0.001, 0.001, 0.001);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iModel));	//Get model from DB
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();

	//Generate visible parts of the models pointcloud
	vtkSmartPointer<vtkPolyData> visiblePD = GetVisiblePart(transformFilterScale->GetOutput(), T_M_S); 

	//Transform it
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(visiblePD); //PLY model
	transformFilter->SetTransform(transform);
	transformFilter->Update();

	//copy data to final object (this is not needed, it could be in visiblePD)
	vtkSmartPointer<vtkPolyData> finPD = vtkSmartPointer<vtkPolyData>::New();
	finPD->DeepCopy(transformFilter->GetOutput());

	return finPD;
}

#ifdef NEVER	// Old version
void PSGM::ObjectAlignment()
{
	FILE *fpTranspose;
	fpTranspose = fopen("D:\\ARP3D\\mug_transpose.txt", "w");

	//apple-> 0, 10; banana -> 10, 6; bottle -> 16, 69; bowl -> 85, 15; car -> 100, 26 
	//donut -> 126, 10; hammer -> 136, 32; tetra_pak -> 168, 22; toilet_paper -> 190, 6 mug ->196, 61
	int iRefObject = 0;
	int nObjectsInClass = 3;

	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	

	Eigen::MatrixXf M(4, 66), P, D(66, 1), T(4, 4), T0p(4, 4), Tiq(4, 4), A(3,66), d(1, 66), S, E;

	double sum;
	double min;
	int p, q;
	Eigen::VectorXf t(3);
	double s;
	
	A = ConvexTemplatenT(); //normals
	
	int m_l = MCTISet.SegmentCTIs.Element[iRefObject].n; //number of reference model CTI-s

	//int n = MCTISet.nModels; // number of models in database
	int n = nObjectsInClass;
	int iPrevClusters = m_l;

	for (int i = 1; i < n; i++) //for all non-reference models
	{
		int m_i = MCTISet.SegmentCTIs.Element[iRefObject + i].n; //number of current model CTI-s
		D.resize(66, m_i);
		for (int k = 0; k < m_i; k++) //for all CTI-s in current model
		{
			//pMCTI = MCTISet.pCTI.Element[iPrevClusters+k];
			pMCTI = MCTISet.pCTI.Element[MCTISet.SegmentCTIs.Element[iRefObject + i].Element[k]];
			pMIE = pMCTI->modelInstance.Element;

			for (int di = 0; di < 66; di++)
			{
				D.block<1, 1>(di, k) << pMIE->d;
				pMIE++;
			}
			//if (k!=m_i-1) D.conservativeResize(D.rows(), D.cols() + 1);
			//D.conservativeResize(D.rows(), D.cols() + 1);
		}

		min = 1000;

		for (int j = 0; j < m_l; j++) //for all CTI-s in reference model
		{
			pMCTI = MCTISet.pCTI.Element[MCTISet.SegmentCTIs.Element[iRefObject].Element[j]];
			pMIE = pMCTI->modelInstance.Element;

			for (int di = 0; di < 66; di++)
			{
				d(0, di) = pMIE->d;
				pMIE++;
			}
			M.block<3, 66>(0, 0) << A;
			M.block<1, 66>(3, 0) << d;
			Eigen::MatrixXf Mt = M.transpose();
			P = (Mt.transpose()*Mt).inverse()*Mt.transpose();
			S = P*D;
			E = D - Mt*S;

			for (int je = 0; je < E.cols(); je++)
			{
				sum = 0;
				for (int ie = 0; ie < E.rows(); ie++)
				{
					sum += E(ie, je)*E(ie, je);
				}
				if (sum < min)
				{
					min = sum;
					p = j;
					q = je;
					t = S.block<3, 1>(0, q);
					//s = *(float*)(&S.data()[3 * S.cols() + q]);
					s = S(3, q);
				}
			}
			
		}
		//I = Eigen::Matrix<float, 3, 3>::Identity();			
		
		T.block<3, 3>(0, 0) << s, 0, 0, 0, s, 0, 0, 0, s;
		T.block<3, 1>(0, 3) << t;
		T.block<1, 3>(3, 0) << 0, 0, 0;
		T.block<1, 1>(3, 3) << 1;

		
		//memcpy(Tiq.block<3, 3>(0, 0).data(), MCTISet.pCTI.Element[iPrevClusters + q]->R, 9 * sizeof(float));
		//memcpy(Tiq.block<3, 1>(0, 3).data(), MCTISet.pCTI.Element[iPrevClusters + q]->t, 3 * sizeof(float));


		//Tiq(0, 0) = MCTISet.pCTI.Element[iPrevClusters + q]->R[0];
		//Tiq(0, 1) = MCTISet.pCTI.Element[iPrevClusters + q]->R[1];
		//Tiq(0, 2) = MCTISet.pCTI.Element[iPrevClusters + q]->R[2];
		//Tiq(0, 3) = MCTISet.pCTI.Element[iPrevClusters + q]->t[0];
		//Tiq(1, 0) = MCTISet.pCTI.Element[iPrevClusters + q]->R[3];
		//Tiq(1, 1) = MCTISet.pCTI.Element[iPrevClusters + q]->R[4];
		//Tiq(1, 2) = MCTISet.pCTI.Element[iPrevClusters + q]->R[5];
		//Tiq(1, 3) = MCTISet.pCTI.Element[iPrevClusters + q]->t[1];
		//Tiq(2, 0) = MCTISet.pCTI.Element[iPrevClusters + q]->R[6];
		//Tiq(2, 1) = MCTISet.pCTI.Element[iPrevClusters + q]->R[7];
		//Tiq(2, 2) = MCTISet.pCTI.Element[iPrevClusters + q]->R[8];
		//Tiq(2, 3) = MCTISet.pCTI.Element[iPrevClusters + q]->t[2];
		int iCTI = MCTISet.SegmentCTIs.Element[iRefObject + i].Element[q];
		Tiq(0, 0) = MCTISet.pCTI.Element[iCTI]->R[0];
		Tiq(0, 1) = MCTISet.pCTI.Element[iCTI]->R[1];
		Tiq(0, 2) = MCTISet.pCTI.Element[iCTI]->R[2];
		Tiq(0, 3) = MCTISet.pCTI.Element[iCTI]->t[0];
		Tiq(1, 0) = MCTISet.pCTI.Element[iCTI]->R[3];
		Tiq(1, 1) = MCTISet.pCTI.Element[iCTI]->R[4];
		Tiq(1, 2) = MCTISet.pCTI.Element[iCTI]->R[5];
		Tiq(1, 3) = MCTISet.pCTI.Element[iCTI]->t[1];
		Tiq(2, 0) = MCTISet.pCTI.Element[iCTI]->R[6];
		Tiq(2, 1) = MCTISet.pCTI.Element[iCTI]->R[7];
		Tiq(2, 2) = MCTISet.pCTI.Element[iCTI]->R[8];
		Tiq(2, 3) = MCTISet.pCTI.Element[iCTI]->t[2];
		Tiq.block<1, 3>(3, 0) << 0, 0, 0;
		Tiq(3, 3) = 1;

		float t1 = Tiq(0, 0);
		float t2 = Tiq(0, 1);
		float t3 = Tiq(0, 2);
		float t4 = Tiq(0, 3);
		float t5 = Tiq(1, 0);
		float t6 = Tiq(1, 1);
		float t7 = Tiq(1, 2);
		float t8 = Tiq(1, 3);
		float t9 = Tiq(2, 0);
		float t10 = Tiq(2, 1);
		float t11 = Tiq(2, 2);
		float t12 = Tiq(2, 3);

		//memcpy(T0p.block<3, 3>(0, 0).data(), MCTISet.pCTI.Element[p]->R, 9 * sizeof(float));
		//memcpy(T0p.block<3, 1>(0, 3).data(), MCTISet.pCTI.Element[p]->t, 3 * sizeof(float));
		iCTI = MCTISet.SegmentCTIs.Element[iRefObject].Element[p];
		T0p(0, 0) = MCTISet.pCTI.Element[iCTI]->R[0];
		T0p(0, 1) = MCTISet.pCTI.Element[iCTI]->R[1];
		T0p(0, 2) = MCTISet.pCTI.Element[iCTI]->R[2];
		T0p(0, 3) = MCTISet.pCTI.Element[iCTI]->t[0];
		T0p(1, 0) = MCTISet.pCTI.Element[iCTI]->R[3];
		T0p(1, 1) = MCTISet.pCTI.Element[iCTI]->R[4];
		T0p(1, 2) = MCTISet.pCTI.Element[iCTI]->R[5];
		T0p(1, 3) = MCTISet.pCTI.Element[iCTI]->t[1];
		T0p(2, 0) = MCTISet.pCTI.Element[iCTI]->R[6];
		T0p(2, 1) = MCTISet.pCTI.Element[iCTI]->R[7];
		T0p(2, 2) = MCTISet.pCTI.Element[iCTI]->R[8];
		T0p(2, 3) = MCTISet.pCTI.Element[iCTI]->t[2];
		T0p.block<1, 3>(3, 0) << 0, 0, 0;
		T0p(3, 3) = 1;

		t1 = T0p(0, 0);
		t2 = T0p(0, 1);
		t3 = T0p(0, 2);
		t4 = T0p(0, 3);
		t5 = T0p(1, 0);
		t6 = T0p(1, 1);
		t7 = T0p(1, 2);
		t8 = T0p(1, 3);
		t9 = T0p(2, 0);
		t10 = T0p(2, 1);
		t11 = T0p(2, 2);
		t12 = T0p(2, 3);

		T0i = Tiq*T*T0p.transpose();

		t1 = T0i(0, 0);
		t2 = T0i(0, 1);
		t3 = T0i(0, 2);
		t4 = T0i(0, 3);
		t5 = T0i(1, 0);
		t6 = T0i(1, 1);
		t7 = T0i(1, 2);
		t8 = T0i(1, 3);
		t9 = T0i(2, 0);
		t10 = T0i(2, 1);
		t11 = T0i(2, 2);
		t12 = T0i(2, 3);

		fprintf(fpTranspose, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", T0i(0, 0), T0i(0, 1), T0i(0, 2), T0i(0, 3), T0i(1, 0), T0i(1, 1), T0i(1, 2), T0i(1, 3), T0i(2, 0), T0i(2, 1), T0i(2, 2), T0i(2, 3), T0i(3, 0), T0i(3, 1), T0i(3, 2), T0i(3, 3));

		

		VisualizeAlignedModels(0, i);

		iPrevClusters += m_i;

	}
	fclose(fpTranspose);
}
#endif

void PSGM::VisualizeAlignedModels(int iRefModel, int iModel)
{
	//Set transform
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	double T_M_S[16];
	
	T_M_S[0]= T0i(0, 0);
	T_M_S[1]= T0i(0, 1);
	T_M_S[2]= T0i(0, 2);
	T_M_S[3]= T0i(0, 3);
	T_M_S[4]= T0i(1, 0);
	T_M_S[5]= T0i(1, 1);
	T_M_S[6]= T0i(1, 2);
	T_M_S[7]= T0i(1, 3);
	T_M_S[8]= T0i(2, 0);
	T_M_S[9] = T0i(2, 1);
	T_M_S[10] = T0i(2, 2);
	T_M_S[11] = T0i(2, 3);
	T_M_S[12] = T0i(3, 0);
	T_M_S[13] = T0i(3, 1);
	T_M_S[14] = T0i(3, 2);
	T_M_S[15] = T0i(3, 3);

	transform->SetMatrix(T_M_S);

	//Scaling Ref. PLY model to meters (if needed)
	vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
	//transformScale->Scale(0.001,0.001,0.001);
	transformScale->Scale(1,1,1);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iRefModel));	//Get model from DB
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();

	//Transform it
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(transformFilterScale->GetOutput()); //PLY model
	transformFilter->SetTransform(transform);
	transformFilter->Update();

	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//Generate Ref. model polydata
	vtkSmartPointer<vtkPolyData> modelPD = transformFilter->GetOutput();
	//vtkSmartPointer<vtkPolyData> modelPD = transformFilterScale->GetOutput();
	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper->SetInputData(modelPD);
	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
	modelActor->SetMapper(modelMapper);
	modelActor->GetProperty()->SetColor(0, 1, 0);
	renderer->AddActor(modelActor);


	//Generate scene polydata
	//Scaling PLY model to meters (if needed)
	transformScale = vtkSmartPointer<vtkTransform>::New();
	//transformScale->Scale(0.001,0.001,0.001);
	transformScale->Scale(1,1,1);
	transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iModel));	//Get model from DB
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();


	vtkSmartPointer<vtkPolyData> modelSPD = transformFilterScale->GetOutput();
	vtkSmartPointer<vtkPolyDataMapper> modelSMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelSMapper->SetInputData(modelSPD);
	vtkSmartPointer<vtkActor> modelSActor = vtkSmartPointer<vtkActor>::New();
	modelSActor->SetMapper(modelSMapper);
	modelSActor->GetProperty()->SetColor(0, 0, 1);
	renderer->AddActor(modelSActor);

	//Start VTK
	renderer->ResetCamera();
	renderer->TwoSidedLightingOff();
	window->Render();
	interactor->Start();

}

void PSGM::Classify(Mesh *pMesh)
{
	//apple-> 0, 10; banana -> 10, 6; bottle -> 16, 69; bowl -> 85, 15; car -> 100, 26 
	//donut -> 126, 10; hammer -> 136, 32; tetra_pak -> 168, 22; toilet_paper -> 190, 6 mug ->196, 61
	//int iRefObject = 196;
	//int nObjectsInClass = 61;

	int imodelfirst = 190;
	int imodellast = 195; 
	int firstCTIinClass = 15529; 
	RECOG::PSGM_::ModelInstance *pMCTI, *pSCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE, *pSIE;
	RECOG::PSGM_::MatchInstance *pMatch;

	Eigen::MatrixXf M(4, 66), P, D(66, 1), T(4, 4), T0p(4, 4), Tiq(4, 4), A(3, 66), S, E;


	double sum;
	double min;
	int p, q;
	Eigen::VectorXf t(3), d(66);
	double s, s_;

	A = ConvexTemplatenT(); //normals

	int m_l = CTISet.pCTI.n; //number of reference (scene) model CTI-s

	int nModels = MCTISet.nModels; // number of models in database
	//int n = nObjectsInClass;

	//number of all CTI-s of all models in database:
	int nCTIs = 0;
	for (int i = imodelfirst; i < imodellast+1; i++) //mugs
//	for (int i = 85; i < 100; i++) //bowls
	//for (int i = 0; i < 10; i++) //apples
	//for (int i = 0; i < nModels; i++)
	{
		nCTIs += MCTISet.SegmentCTIs.Element[i].n;
	}

	RVL_DELETE_ARRAY(CTIMatchMem);
	CTIMatchMem = new RECOG::PSGM_::MatchInstance[m_l * nCTIs];

	D.resize(66, nCTIs);
	for (int k = 0; k < nCTIs; k++) //for all CTI-s of all models in database
	{
		pMCTI = MCTISet.pCTI.Element[k + firstCTIinClass];
//		pMCTI = MCTISet.pCTI.Element[k];
		pMIE = pMCTI->modelInstance.Element;

		for (int di = 0; di < 66; di++)
		{
			D(di, k) = pMIE->d;
			pMIE++;
		}
	}

	min = 1000;

	for (int j = 0; j < m_l; j++) //for all CTI-s in reference (scene) model
	{
		pSCTI = CTISet.pCTI.Element[j];
		pSIE = pSCTI->modelInstance.Element;


		// Determine the number of rows (rows=iValid) of M
		int rows = 0;
		Eigen::VectorXi validS(66);
		for (int iSIE = 0; iSIE < 66; iSIE++)
		{
			validS(iSIE) = pSIE->valid;
			d(iSIE) = pSIE->d; // Filling descriptor
			if (pSIE->valid == 1)
				rows++;
			pSIE++;
		}
		
		M.block<3, 66>(0, 0) << A;
		Eigen::MatrixXf Mt = M.transpose();

		// Search for valids and create dv and Mv
		Eigen::MatrixXf Mv(rows, 4), Dv(rows, D.cols());
		Eigen::VectorXf dv(rows);
	
		int iValid = 0;
		for (int iSIE = 0; iSIE < 66; iSIE++)
		{
			if (validS(iSIE) == 1)
			{
				dv(iValid) = d(iSIE);
				Mv.block<1, 3>(iValid, 0) << Mt.block<1, 3>(iSIE, 0);
				Dv.block(iValid, 0, 1, D.cols()) << D.block(iSIE, 0, 1, D.cols());
				iValid++;
			}
			pSIE++;
		}
		
		for (int i = 0; i < rows; i++)
		{
			Mv(i, 3) = dv(i);
		}

		P = (Mv.transpose()*Mv).inverse()*Mv.transpose();
		S = P*Dv;
		E = Dv - Mv*S;

		for (int je = 0; je < E.cols(); je++)
		{
			sum = 0;
			for (int ie = 0; ie < E.rows(); ie++)
			{
				sum += E(ie, je)*E(ie, je);
}
			s_ = S(3, je);
			sum /= (s_*s_); //in scene-space

			//all matches
			p = j;
			q = je;
			t = S.block<3, 1>(0, q);
			s = s_;

			pMatch = CTIMatchMem + j*E.cols() + je;

			pMCTI = MCTISet.pCTI.Element[q + firstCTIinClass];
			pMatch->iSCTI = p;
			pMatch->iMCTI = q+firstCTIinClass;
			pMatch->iClass = pMCTI->iModel;
			pMatch->score = sum;
			pMatch->s = s;
			pMatch->t_class[0] = t(0);
			pMatch->t_class[1] = t(1);
			pMatch->t_class[2] = t(2);
			pMatch->ID = j*E.cols() + je;



			////the best match:
			//if (sum < min)
			//{
			//	min = sum;
			//	p = j;
			//	q = je;
			//	t = S.block<3, 1>(0, q);
			//	s = s_;
			//	IDbestMatch = j*E.cols() + je;
			//	
			//}

		}
	}

	int iMatch = 0, brojac, iCTI = 0;
	float minScore;
	int IDbestMatch=0;
	pMatch = CTIMatchMem;
	minScore = pMatch[0].score;

	for (int iArr = 0; iArr < m_l*E.cols(); iArr++)
	{
		if (pMatch[iArr].score < minScore)
		{
			minScore = pMatch[iArr].score;
			IDbestMatch = pMatch[iArr].ID;
		}

	}
	
	//SORT:
	//Array<SortIndex<float>> ArrSum;
	//ArrSum.n = m_l*E.cols();
	//ArrSum.Element = new SortIndex<float>[ArrSum.n];

	//pMatch = CTIMatchMem;

	

	//for (int iArr = 0; iArr < ArrSum.n; iArr++)
	//{
	//	ArrSum.Element[iArr].idx = pMatch[iArr].ID;
	//	ArrSum.Element[iArr].cost = pMatch[iArr].score;

	//}

	//BubbleSort<SortIndex<float>>(ArrSum);

	//s = pMatch[ArrSum.Element[0].idx].s;

	s = pMatch[IDbestMatch].s;

	T.block<3, 3>(0, 0) << s, 0, 0, 0, s, 0, 0, 0, s;
	//T.block<3, 1>(0, 3) << pMatch[ArrSum.Element[0].idx].t_class[0], pMatch[ArrSum.Element[0].idx].t_class[1], pMatch[ArrSum.Element[0].idx].t_class[2];
	T.block<3, 1>(0, 3) << pMatch[IDbestMatch].t_class[0], pMatch[IDbestMatch].t_class[1], pMatch[IDbestMatch].t_class[2];
	T.block<1, 3>(3, 0) << 0, 0, 0;
	T.block<1, 1>(3, 3) << 1;

	//int iCTI = MCTISet.SegmentCTIs.Element[iRefObject + i].Element[q];
	iCTI = pMatch[IDbestMatch].iMCTI;
	Tiq(0, 0) = MCTISet.pCTI.Element[iCTI]->R[0];
	Tiq(0, 1) = MCTISet.pCTI.Element[iCTI]->R[1];
	Tiq(0, 2) = MCTISet.pCTI.Element[iCTI]->R[2];
	Tiq(0, 3) = MCTISet.pCTI.Element[iCTI]->t[0];
	Tiq(1, 0) = MCTISet.pCTI.Element[iCTI]->R[3];
	Tiq(1, 1) = MCTISet.pCTI.Element[iCTI]->R[4];
	Tiq(1, 2) = MCTISet.pCTI.Element[iCTI]->R[5];
	Tiq(1, 3) = MCTISet.pCTI.Element[iCTI]->t[1];
	Tiq(2, 0) = MCTISet.pCTI.Element[iCTI]->R[6];
	Tiq(2, 1) = MCTISet.pCTI.Element[iCTI]->R[7];
	Tiq(2, 2) = MCTISet.pCTI.Element[iCTI]->R[8];
	Tiq(2, 3) = MCTISet.pCTI.Element[iCTI]->t[2];
	Tiq.block<1, 3>(3, 0) << 0, 0, 0;
	Tiq(3, 3) = 1;

	float t1 = Tiq(0, 0);
	float t2 = Tiq(0, 1);
	float t3 = Tiq(0, 2);
	float t4 = Tiq(0, 3);
	float t5 = Tiq(1, 0);
	float t6 = Tiq(1, 1);
	float t7 = Tiq(1, 2);
	float t8 = Tiq(1, 3);
	float t9 = Tiq(2, 0);
	float t10 = Tiq(2, 1);
	float t11 = Tiq(2, 2);
	float t12 = Tiq(2, 3);

	//iCTI = CTISet.SegmentCTIs.Element[iRefObject].Element[p];
	iCTI = pMatch[IDbestMatch].iSCTI;
	T0p(0, 0) = CTISet.pCTI.Element[iCTI]->R[0];
	T0p(0, 1) = CTISet.pCTI.Element[iCTI]->R[1];
	T0p(0, 2) = CTISet.pCTI.Element[iCTI]->R[2];
	T0p(0, 3) = CTISet.pCTI.Element[iCTI]->t[0];
	T0p(1, 0) = CTISet.pCTI.Element[iCTI]->R[3];
	T0p(1, 1) = CTISet.pCTI.Element[iCTI]->R[4];
	T0p(1, 2) = CTISet.pCTI.Element[iCTI]->R[5];
	T0p(1, 3) = CTISet.pCTI.Element[iCTI]->t[1];
	T0p(2, 0) = CTISet.pCTI.Element[iCTI]->R[6];
	T0p(2, 1) = CTISet.pCTI.Element[iCTI]->R[7];
	T0p(2, 2) = CTISet.pCTI.Element[iCTI]->R[8];
	T0p(2, 3) = CTISet.pCTI.Element[iCTI]->t[2];
	T0p.block<1, 3>(3, 0) << 0, 0, 0;
	T0p(3, 3) = 1;

	t1 = T0p(0, 0);
	t2 = T0p(0, 1);
	t3 = T0p(0, 2);
	t4 = T0p(0, 3);
	t5 = T0p(1, 0);
	t6 = T0p(1, 1);
	t7 = T0p(1, 2);
	t8 = T0p(1, 3);
	t9 = T0p(2, 0);
	t10 = T0p(2, 1);
	t11 = T0p(2, 2);
	t12 = T0p(2, 3);

	T0i = (Tiq*T*T0p.inverse()).inverse();
	//T0i = (Tiq*T*T0p.inverse()).inverse();

	int iModel = pMatch[IDbestMatch].iClass;
//	delete[] ArrSum.Element;

	////q += 15879;
	//pMCTI = MCTISet.pCTI.Element[q];
	//int iClass = pMCTI->iModel;
	//if (iClass < 10)
	//{
	//	
	//	//iCorrectClass++;
	//	printf("Class - apple.\n");// %d\n", iCorrectClass);
	//}
	//
	//if (iClass >= 10 && iClass<16) 	
	//{
	//	//iCorrectClass++;
	//	printf("Class - banana.\n");// %d\n", iCorrectClass);
	//}
	//
	//if (iClass >= 16 && iClass < 85) 
	//{
	//	//iCorrectClass++;
	//	printf("Class - bottle.\n");// %d\n", iCorrectClass);
	//}
	//
	//if (iClass >= 85 && iClass<100) 
	//{
	//	//iCorrectClass++;
	//	printf("Class - bowl.\n");// %d\n", iCorrectClass);
	//}
	//if (iClass >= 100 && iClass<126) 	
	//{
	//	//iCorrectClass++;
	//	printf("Class - car.\n");// %d\n", iCorrectClass);
	//}
	//if (iClass >= 126 && iClass<136) 
	//{
	//	//iCorrectClass++;
	//	printf("Class - donnut.\n"); // %d\n", iCorrectClass);
	//}
	//if (iClass >= 136 && iClass<168)
	//{
	//	//iCorrectClass++;
	//	printf("Class - hammer.\n");// %d\n", iCorrectClass);
	//}
	//if (iClass >= 168 && iClass<190)
	//{
	//	//iCorrectClass++;
	//	printf("Class - tetra_pak.\n");// %d\n", iCorrectClass);
	//}
	//if (iClass >= 190 && iClass<196) 
	//{
	//	//iCorrectClass++;
	//	printf("Class - toilet_paper.\n");//  %d\n", iCorrectClass);
	//}
	//if (iClass >= 196)
	//{
	//	//iCorrectClass++;
	//	printf("Class - mug.\n");// %d\n", iCorrectClass);
	//}

	//FILE *fpClass;
	//fpClass = fopen("D:\\ARP3D\\matchClass.dat", "w");

	//fprintf(fpClass, "%d\t%d\t%.3f\t%.3f\t%.3f\t%.3f", q, p, s, t(0), t(1), t(2));

	//fclose(fpClass);
	
	FILE *fpClassTranspose;
	fpClassTranspose = fopen("D:\\ARP3D\\3DNet_dataset\\Cat10_ModelDatabase\\Cat10_ModelDatabase\\transposeToToiletPapers.txt", "a");
	fprintf(fpClassTranspose, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", T0i(0, 0), T0i(0, 1), T0i(0, 2), T0i(0, 3), T0i(1, 0), T0i(1, 1), T0i(1, 2), T0i(1, 3), T0i(2, 0), T0i(2, 1), T0i(2, 2), T0i(2, 3), T0i(3, 0), T0i(3, 1), T0i(3, 2), T0i(3, 3));
	fclose(fpClassTranspose);



	
	//VisualizeObjectClass(iModel, pMesh);

}

void PSGM::VisualizeObjectClass(int iModel, Mesh *pMesh)
{
	//Set transform
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	double T_M_S[16];

	T_M_S[0] = T0i(0, 0);
	T_M_S[1] = T0i(0, 1);
	T_M_S[2] = T0i(0, 2);
	T_M_S[3] = T0i(0, 3);
	T_M_S[4] = T0i(1, 0);
	T_M_S[5] = T0i(1, 1);
	T_M_S[6] = T0i(1, 2);
	T_M_S[7] = T0i(1, 3);
	T_M_S[8] = T0i(2, 0);
	T_M_S[9] = T0i(2, 1);
	T_M_S[10] = T0i(2, 2);
	T_M_S[11] = T0i(2, 3);
	T_M_S[12] = T0i(3, 0);
	T_M_S[13] = T0i(3, 1);
	T_M_S[14] = T0i(3, 2);
	T_M_S[15] = T0i(3, 3);

	transform->SetMatrix(T_M_S);

	//Scaling model PLY model to meters (if needed)
	vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
	//transformScale->Scale(0.001,0.001,0.001);
	transformScale->Scale(1, 1, 1);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iModel));	//Get model from DB
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();

	//Transform it
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(transformFilterScale->GetOutput()); //PLY model
	transformFilter->SetTransform(transform);
	transformFilter->Update();

	vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexGlyphFilter->AddInputData(transformFilter->GetOutput());


	//// Initialize VTK.
	//vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	//vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	//vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	//window->AddRenderer(renderer);
	//window->SetSize(800, 600);
	//interactor->SetRenderWindow(window);
	//vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	//interactor->SetInteractorStyle(style);
	//renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//Generate model polydata
	//vtkSmartPointer<vtkPolyData> modelPD = transformFilter->GetOutput();
	//vtkSmartPointer<vtkPolyData> modelPD = transformFilterScale->GetOutput();
	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//modelMapper->SetInputData(modelPD);
	modelMapper->SetInputConnection(vertexGlyphFilter->GetOutputPort());
	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
	modelActor->SetMapper(modelMapper);
	modelActor->GetProperty()->SetColor(1, 0, 0);
	
	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	Visualizer visualizer;

	visualizer.Create();

	this->pSurfels->NodeColors(SelectionColor);
	this->InitDisplay(&visualizer, pMesh, SelectionColor);
	this->Display();
	this->displayData.pVisualizer->renderer->AddActor(modelActor);
	this->displayData.pVisualizer->Run();


	
	//Show current scene
	//surfels.NodeColors(SelectionColor);
	//recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
	//recognition.Display();
	//visualizer.Run();


	////Start VTK
	//renderer->ResetCamera();
	//renderer->TwoSidedLightingOff();
	//window->Render();
	//interactor->Start();

}

//Vidovic
void PSGM::GetTransparencyAndCollisionConsensus(Visualizer *pVisualizer)
{
	int i, iMatch;

	consensusHypotheses.clear();

	for (i = 0; i < noCollisionHypotheses.size(); i++)
	{
		iMatch = noCollisionHypotheses.at(i);

		if (std::find(transparentHypotheses.begin(), transparentHypotheses.end(), iMatch) != transparentHypotheses.end())
			continue;
		else
		{
			consensusHypotheses.push_back(iMatch);

			if (pVisualizer)
				AddOneModelToVisualizer(pVisualizer, noCollisionHypotheses.at(i), -1, false, true);

			cout << "Segment: " << GetSCTI(GetMatch(iMatch))->iCluster << " matched with model: " << GetMCTI(GetMatch(iMatch))->iModel << " with score " << GetMatch(iMatch)->cost_NN << "\n";
		}			
	}
}

void PSGM::EvaluateConsensusMatches(float &precision, float &recall, bool verbose)
{
	if (verbose)
		cout << "Consensus matches evaluation for scene "<< iScene - 1 << "..." << "\n";

	int iMatch, iHypothesis, iMCTI, iSCTI, iMatchedModel, iSSegment, iSegmentGT;
	int TP_ = 0, FP_ = 0, FN_ = 0;
	bool TPMatch;
	RECOG::PSGM_::MatchInstance *pMatch ;

	pECCVGT->ResetMatchFlag();

	for (iHypothesis = 0; iHypothesis < consensusHypotheses.size(); iHypothesis++)
	{
		iMatch = consensusHypotheses.at(iHypothesis);
		pMatch = pCTImatchesArray.Element[iMatch];

		//Compare to segment GT
		iMCTI = pCTImatchesArray.Element[iMatch]->iMCTI;
		iMatchedModel = MCTISet.pCTI.Element[iMCTI]->iModel;
		iSCTI = pCTImatchesArray.Element[iMatch]->iSCTI;
		iSSegment = CTISet.pCTI.Element[iSCTI]->iCluster;

		iSegmentGT = pMatch->iScene * nDominantClusters + iSSegment;

		//eliminate FP from segments without GT
		if (!segmentGT.Element[iSegmentGT].valid)
			TPMatch = false;
		else	
			TPMatch = CompareMatchToSegmentGT(pMatch->iScene, iSSegment, iMatchedModel); //check if match is TP and update match flag
	
		if (!TPMatch)
		{
			FP_++;
		}
	}

	CountTPandFN(TP_, FN_, verbose);

	CalculatePR(TP_, FP_, FN_, precision, recall);

	//precision = (float) (TP_ / (TP_ + FP_));
	//recall = (float) (TP_ / (TP_ + FN_));

	if (verbose)
	{
		cout << "Precision: " << precision << "\n" << "Recall: " << recall << "\n";
		cout << "TP: " << TP_ << "\tFP: " << FP_ << "\tFN: " << FN_ << "\n";
		cout << "---------------------------------------------------\n";
	}
}

RECOG::PSGM_::MatchInstance* PSGM::GetMatch(int matchID)
{
	return pCTImatchesArray.Element[matchID];
}

RECOG::PSGM_::ModelInstance* PSGM::GetMCTI(int iMCTI)
{
	return MCTISet.pCTI.Element[iMCTI];
}

RECOG::PSGM_::ModelInstance* PSGM::GetMCTI(RECOG::PSGM_::MatchInstance *pMatch)
{
	return MCTISet.pCTI.Element[pMatch->iMCTI];
}

RECOG::PSGM_::ModelInstance* PSGM::GetSCTI(int iSCTI)
{
	return CTISet.pCTI.Element[iSCTI];
}

RECOG::PSGM_::ModelInstance* PSGM::GetSCTI(RECOG::PSGM_::MatchInstance *pMatch)
{
	return CTISet.pCTI.Element[pMatch->iSCTI];
}

void PSGM::PrintCTIMatches()
{
	cout << "\nCTI matches per segment:\n";
	for (int i = 0; i < bestSceneSegmentMatches.n; i++)
	{
		cout << "Segment: " << i << ":\n";

		for (int j = 0; j < bestSceneSegmentMatches.Element[i].n; j++)
			cout << "Match: " << j << " ModelID:" << GetMCTI(GetMatch(bestSceneSegmentMatches.Element[i].Element[j].idx))->iModel << "\tCTI score: " << GetMatch(bestSceneSegmentMatches.Element[i].Element[j].idx)->score << " (matchID: " << bestSceneSegmentMatches.Element[i].Element[j].idx << ")" << "\n";

		cout << "---------------------------------------------------\n";
	}

}

void PSGM::PrintICPMatches()
{
	cout << "\nICP matches per segment:\n";
	for (int i = 0; i < scoreMatchMatrixICP.n; i++)
	{
		cout << "Segment: " << i << ":\n";

		for (int j = 0; j < scoreMatchMatrixICP.Element[i].n; j++)
			if (scoreMatchMatrixICP.Element[i].Element[j].idx != -1)
				cout << "Match: " << j << " ModelID:" << GetMCTI(GetMatch(scoreMatchMatrixICP.Element[i].Element[j].idx))->iModel << "\tICP cost: " << GetMatch(scoreMatchMatrixICP.Element[i].Element[j].idx)->cost_NN << " gndDistance: " << GetMatch(scoreMatchMatrixICP.Element[i].Element[j].idx)->gndDistance << " transparency ratio:" << GetMatch(scoreMatchMatrixICP.Element[i].Element[j].idx)->transparencyRatio << " (matchID: " << scoreMatchMatrixICP.Element[i].Element[j].idx << ")" << "\n";

		cout << "---------------------------------------------------\n";
	}
}

void PSGM::VisualizeConsensusHypotheses(Visualizer *pVisualizer)
{
	int iHypothesis;

	for (iHypothesis = 0; iHypothesis < consensusHypotheses.size(); iHypothesis++)
		AddOneModelToVisualizer(pVisualizer, noCollisionHypotheses.at(iHypothesis), -1, false, false, true);
		
}

void PSGM::VisualizeGTMatch(Visualizer *pVisualizer)
{
	int iSegment, CTIRank, matchID, iSegmentGT, iMatchedModel;
	char cSelection;

	RECOG::PSGM_::MatchInstance *pMatch;
	matchID = scoreMatchMatrixICP.Element[matchGTiS].Element[matchGTiRank].idx;
	pMatch = GetMatch(matchID);

	//FILE *fp;
	//fp = fopen("C:\\RVL\\matchGT.txt", "w");
	iSegment = GetSCTI(pMatch)->iCluster;

	iSegmentGT = pMatch->iScene * nDominantClusters + iSegment;
	iMatchedModel = GetMCTI(pMatch)->iModel;

	if (iMatchedModel == segmentGT.Element[iSegmentGT].iModel) //TP model
	{
		//delete visualized matches from the scene:
		pVisualizer->renderer->RemoveAllViewProps();
		pVisualizer->SetMesh(pMesh);
		Display();

		//visualize new ICP matches on the scene
		AddOneModelToVisualizer(pVisualizer, matchID, matchGTiRank, true);
	}







	//nSegments = scoreMatchMatrixICP.n;

	//for (iSegment = 0; iSegment < nSegments; iSegment++)
	//{
	//	for (iRank = 0; iRank < nBestMatches; iRank++)
	//	{
	//		//delete visualized matches from the scene:
	//		pVisualizer->renderer->RemoveAllViewProps();
	//		pVisualizer->SetMesh(pMesh);
	//		Display();

	//		matchID = scoreMatchMatrixICP.Element[iSegment].Element[iRank].idx;

	//		//visualize new ICP matches on the scene
	//		AddOneModelToVisualizer(pVisualizer, matchID, iRank, true);

	//		CTIRank = FindCTIMatchRank(matchID, iSegment);

	//		//debug
	//		//cv::waitKey();

	//		//do
	//		//{
	//		//	printf("Enter one of the options: \n");
	//		//	printf("c:\tdisplay CTI match,\n");
	//		//	printf("i:\tdisplay ICP match,\n");
	//		//	printf("n:\tnext match,\n");
	//		//	printf("x:\tprint info and proceed to next segment,\n");
	//		//	scanf("%c", &cSelection);

	//		//	if (cSelection == 'c')
	//		//	{
	//		//		//delete visualized matches from the scene:
	//		//		pVisualizer->renderer->RemoveAllViewProps();
	//		//		pVisualizer->SetMesh(pMesh);
	//		//		Display();

	//		//		AddOneModelToVisualizer(pVisualizer, matchID, CTIRank, false);
	//		//	}
	//		//	else if (cSelection == 'p')
	//		//	{
	//		//		//delete visualized matches from the scene:
	//		//		pVisualizer->renderer->RemoveAllViewProps();
	//		//		pVisualizer->SetMesh(pMesh);
	//		//		Display();

	//		//		AddOneModelToVisualizer(pVisualizer, scoreMatchMatrixICP.Element[iSegment].Element[iRank].idx, iRank, true);
	//		//	}

	//		//} while (cSelection != 'n' && cSelection != 'x' );

	//		//if (cSelection == 'x')
	//		//{
	//		//	if (fp)
	//		//	{
	//		//		pMatch = GetMatch(matchID);
	//		//		fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\n", pMatch->iScene, iSegment, GetMCTI(pMatch)->iModel, matchID, CTIRank, iRank);
	//		//	}
	//		//	else
	//		//		printf("MatchGT info - fp ERROR!\n");

	//		//	break; //proceed to the next segment
	//		//}
	//	}
	//}
}

int PSGM::FindCTIMatchRank(int matchID, int iSegment)
{
	int iRank;

	for (iRank = 0; iRank < nBestMatches; iRank++)
		if (bestSceneSegmentMatches.Element[iSegment].Element[iRank].idx == matchID)
			return iRank;
}


// vidovic branch

int PSGM::FindICPMatchRank(int matchID, int iSegment)
{
	int iRank;

	for (iRank = 0; iRank < nBestMatches; iRank++)
		if (scoreMatchMatrixICP.Element[iSegment].Element[iRank].idx == matchID)
			return iRank;
}

void PSGM::createVersionTestFile()
{
	char *versionTestFileName, *matchGTFileName;
	FILE *fpTF;

	matchGTFileName = RVLCreateFileName(sceneFileName, ".ply", -1, ".mgt", pMem);
	fpMatchGT = fopen(matchGTFileName, "r");

	versionTestFileName = RVLCreateFileName(sceneFileName, ".ply", -1, ".tf", pMem);
	fpTF = fopen(versionTestFileName, "w");

	//int iScene, iSegment, iModel, matchID, CTIrank, ICPrank;
	RECOG::PSGM_::MGT MGTinstance;

	if (fpMatchGT)
	{
		while (!feof(fpMatchGT))
		{
			fscanf(fpMatchGT, "%d\t%d\t%d\t%d\t%d\t%d", &MGTinstance.iScene, &MGTinstance.iSegment, &MGTinstance.iModel, &MGTinstance.matchID, &MGTinstance.CTIrank, &MGTinstance.ICPrank);
			fprintf(fpTF, "%d\t%d\t%d\t%d\t%d\t%d\t%.4f\t%.4f\t%.4f\t%.4f\n", MGTinstance.iScene, MGTinstance.iSegment, MGTinstance.iModel, MGTinstance.matchID, MGTinstance.CTIrank, MGTinstance.ICPrank, pCTImatchesArray.Element[MGTinstance.matchID]->score, pCTImatchesArray.Element[MGTinstance.matchID]->cost_NN, pCTImatchesArray.Element[MGTinstance.matchID]->gndDistance, pCTImatchesArray.Element[MGTinstance.matchID]->transparencyRatio);
		}
	}

	fclose(fpMatchGT);
	fclose(fpTF);
}

void PSGM::checkVersionTestFile(bool verbose)
{
	char *versionTestFileName;
	FILE *fpTF;

	versionTestFileName = RVLCreateFileName(sceneFileName, ".ply", -1, ".tf", pMem);
	fpTF = fopen(versionTestFileName, "r");

	RECOG::PSGM_::MGT MGTinstance;
	bool diff = false;
	int nDiff = 0;

	printf("***********************************************************************\n");
	printf("Version Test File checking started...");

	if (fpTF)
	{
		while (!feof(fpTF))
		{
			fscanf(fpTF, "%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f", &MGTinstance.iScene, &MGTinstance.iSegment, &MGTinstance.iModel, &MGTinstance.matchID, &MGTinstance.CTIrank, &MGTinstance.ICPrank, &MGTinstance.CTIscore, &MGTinstance.ICPcost, &MGTinstance.gndDistance, &MGTinstance.transparencyRatio);

			if (checkVersionTestFile(MGTinstance, verbose))
			{
				if (!verbose)
					printf("\nDifference between current and old version found in match %d!!", MGTinstance.matchID);

				diff = true;
				nDiff++;
			}
		}
	

		if (diff)
			printf("\nVersion Test File checking completed with %d differences!!\n", nDiff);
		else
			printf(" completed with no differences!!\n");
	}
	else
		printf("\nVersion Test File %s is missing!!\n", versionTestFileName);


	printf("***********************************************************************\n");
}

bool PSGM::checkVersionTestFile(RECOG::PSGM_::MGT MGTinstance, bool verbose)
{
	RECOG::PSGM_::MatchInstance *pMatch;
	pMatch = pCTImatchesArray.Element[MGTinstance.matchID];

	bool diff = false;
	int CTIrank, ICPrank;

	if (MGTinstance.matchID == 200107 || MGTinstance.matchID == 200294)
		int debug = 0;

	if (MGTinstance.iSegment != GetSCTI(pMatch)->iCluster)
	{
		diff = true;

		if (verbose)
			printf("\nVersion Test File difference for match %d. (TF segment is %d, while current is %d)!!", MGTinstance.matchID, MGTinstance.iSegment, GetSCTI(pMatch)->iCluster);
	}

	if (MGTinstance.iModel != GetMCTI(pMatch)->iModel)
	{
		diff = true;

		if (verbose)
			printf("\nVersion Test File difference for match %d. (TF model is %d, while current is %d)!!", MGTinstance.matchID, MGTinstance.iModel, GetMCTI(pMatch)->iModel);
	}

	CTIrank = FindCTIMatchRank(MGTinstance.matchID, GetSCTI(pMatch)->iCluster);

	if (MGTinstance.CTIrank != CTIrank)
	{
		diff = true;

		if (verbose)
			printf("\nVersion Test File difference for match %d. (TF CTI rank is %d, while current is %d)!!", MGTinstance.matchID, MGTinstance.CTIrank, CTIrank);
	}

	ICPrank = FindICPMatchRank(MGTinstance.matchID, GetSCTI(pMatch)->iCluster);

	if (MGTinstance.ICPrank != ICPrank)
	{
		diff = true;

		if (verbose)
			printf("\nVersion Test File difference for match %d. (TF ICP rank is %d, while current is %d)!!", MGTinstance.matchID, MGTinstance.ICPrank, ICPrank);
	}

	if (RVLABS((MGTinstance.CTIscore - pMatch->score)) > 0.00009)
	{
		diff = true;

		if (verbose)
			printf("\nVersion Test File difference for match %d. (TF CTI score is %.4f, while current is %f)!!", MGTinstance.matchID, MGTinstance.CTIscore, pMatch->score);
	}

	if (RVLABS((MGTinstance.ICPcost - pMatch->cost_NN)) > 0.00009)
	{
		diff = true;

		if (verbose)
			printf("\nVersion Test File difference for match %d. (TF ICP cost is %.4f, while current is %f)!!", MGTinstance.matchID, MGTinstance.ICPcost, pMatch->cost_NN);
	}

	if (RVLABS((MGTinstance.gndDistance - pMatch->gndDistance)) > 0.00009)
	{
		diff = true;

		if (verbose)
			printf("\nVersion Test File difference for match %d. (TF ground distance is %.4f, while current is %f)!!", MGTinstance.matchID, MGTinstance.gndDistance, pMatch->gndDistance);
	}

	if (RVLABS((MGTinstance.transparencyRatio - pMatch->transparencyRatio)) > 0.00009)
	{
		diff = true;

		if (verbose)
			printf("\nVersion Test File difference for match %d. (TF transparency ratio is %.4f, while current is %f)!!", MGTinstance.matchID, MGTinstance.transparencyRatio, pMatch->transparencyRatio);
	}

	return diff;
}
//END Vidovic

void PSGM::CreateDilatedDepthImage()
{
	//Generate scene depth
	double point[3];
	int u, v;

	depth.setTo(cv::Scalar(0));
	//cv::Mat depth(480, 640, CV_16UC1, cv::Scalar::all(0));
	for (int i = 0; i < pMesh->pPolygonData->GetNumberOfPoints(); i++)
	{
		pMesh->pPolygonData->GetPoint(i, point);
		if ((point[0] == 0) && (point[1] == 0) && (point[2] == 0))
			continue;
		v = floor(float(i) / 640);
		u = i - v * 640;
		depth.at<uint16_t>(v, u) = (uint16_t)(point[2] * 1000); //in milimeters
	}
	//Postprocessing
	for (int y = 0; y < depth.rows; y++)
	{
		for (int x = 0; x < depth.cols; x++)
		{
			if (depth.at<uint16_t>(y, x) == 0)
				depth.at<uint16_t>(y, x) = 10000; //in milimeters
		}
	}
	cv::Mat elementE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(17, 17));
	cv::erode(depth, depth, elementE);
	//Set PSGM depth
	//depthImg = (unsigned short*)depth.data; //Vidovic 21.07.2017. depth.data is deleted after this function finish

	//Vidovic
	//unsigned short *depthTMP;
	//depthTMP = (unsigned short*)depth.data;
	//int a = sizeof(unsigned short);
	//int s = sizeof(depthImg);
	//memcpy(&depthImg, &depthTMP, depth.rows * depth.cols * sizeof(unsigned short));
	////END Vidovic
}

//END Vidovic

// cupec_branch3

#ifdef NEVER		// Even older version
void PSGM::ObjectAlignment()
{
	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;
	

	Eigen::MatrixXf M(4, 66), P, D(66, 1), T(4, 4), T0p(4, 4), Tiq(4, 4), A, d(1, 66), S, E, I;

	float sum;
	float min;
	int p, q;
	Eigen::VectorXf t(3);
	float s;
	
	A = ConvexTemplatenT(); //normals
	
	int m_l = MCTISet.SegmentCTIs.Element[0].n; //number of reference model CTI-s
	int n = MCTISet.nModels; // number of models in database
	int iPrevClusters = m_l;

	for (int i = 1; i < n-1; i++) //for all non-reference models
	{
		int m_i = MCTISet.SegmentCTIs.Element[i].n; //number of current model CTI-s
		D.resize(66, 1);
		for (int k = 0; k < m_i; k++) //for all CTI-s in current model
		{
			pMCTI = MCTISet.pCTI.Element[iPrevClusters+k];
			pMIE = pMCTI->modelInstance.Element;

			for (int di = 0; di < 66; di++)
			{
				D.block<1, 1>(di, k) << pMIE->d;
				pMIE++;
			}
			D.conservativeResize(D.rows(), D.cols() + 1);
		}

		min = 1000;

		for (int j = 0; j < m_l; j++) //for all CTI-s in reference model
		{
			pMCTI = MCTISet.pCTI.Element[j];
			pMIE = pMCTI->modelInstance.Element;

			for (int di = 0; di < 66; di++)
			{
				d(0, di) = pMIE->d;
				pMIE++;
			}
			M.block<3, 66>(0, 0) << A;
			M.block<1, 66>(3, 0) << d;
			Eigen::MatrixXf Mt = M.transpose();
			P = (Mt.transpose()*Mt).inverse()*Mt.transpose();
			S = P*D;
			E = D - Mt*S;

			for (int je = 0; je < E.cols(); je++)
			{
				sum = 0;
				for (int ie = 0; ie < E.rows(); ie++)
				{
					sum += E(ie, je)*E(ie, je);
				}
				if (sum < min)
				{
					min = sum;
					p = j;
					q = je;
					t = S.block<3, 1>(0, q);
					//s = *(float*)(&S.data()[3 * S.cols() + q]);
					s = S(3, q);
				}
			}
			
		}
		//I = Eigen::Matrix<float, 3, 3>::Identity();			
		
		T.block<3, 3>(0, 0) << s, 0, 0, 0, s, 0, 0, 0, s;
		T.block<3, 1>(0, 3) << t;
		T.block<1, 3>(3, 0) << 0, 0, 0;
		T.block<1, 1>(3, 3) << 1;

		
		//memcpy(Tiq.block<3, 3>(0, 0).data(), MCTISet.pCTI.Element[iPrevClusters + q]->R, 9 * sizeof(float));
		//memcpy(Tiq.block<3, 1>(0, 3).data(), MCTISet.pCTI.Element[iPrevClusters + q]->t, 3 * sizeof(float));


		Tiq(0, 0) = MCTISet.pCTI.Element[iPrevClusters + q]->R[0];
		Tiq(0, 1) = MCTISet.pCTI.Element[iPrevClusters + q]->R[1];
		Tiq(0, 2) = MCTISet.pCTI.Element[iPrevClusters + q]->R[2];
		Tiq(0, 3) = MCTISet.pCTI.Element[iPrevClusters + q]->t[0];
		Tiq(1, 0) = MCTISet.pCTI.Element[iPrevClusters + q]->R[3];
		Tiq(1, 1) = MCTISet.pCTI.Element[iPrevClusters + q]->R[4];
		Tiq(1, 2) = MCTISet.pCTI.Element[iPrevClusters + q]->R[5];
		Tiq(1, 3) = MCTISet.pCTI.Element[iPrevClusters + q]->t[1];
		Tiq(2, 0) = MCTISet.pCTI.Element[iPrevClusters + q]->R[6];
		Tiq(2, 1) = MCTISet.pCTI.Element[iPrevClusters + q]->R[7];
		Tiq(2, 2) = MCTISet.pCTI.Element[iPrevClusters + q]->R[8];
		Tiq(2, 3) = MCTISet.pCTI.Element[iPrevClusters + q]->t[2];
		Tiq.block<1, 3>(3, 0) << 0, 0, 0;
		Tiq(3, 3) = 1;

		float t1 = Tiq(0, 0);
		float t2 = Tiq(0, 1);
		float t3 = Tiq(0, 2);
		float t4 = Tiq(0, 3);
		float t5 = Tiq(1, 0);
		float t6 = Tiq(1, 1);
		float t7 = Tiq(1, 2);
		float t8 = Tiq(1, 3);
		float t9 = Tiq(2, 0);
		float t10 = Tiq(2, 1);
		float t11 = Tiq(2, 2);
		float t12 = Tiq(2, 3);

		//memcpy(T0p.block<3, 3>(0, 0).data(), MCTISet.pCTI.Element[p]->R, 9 * sizeof(float));
		//memcpy(T0p.block<3, 1>(0, 3).data(), MCTISet.pCTI.Element[p]->t, 3 * sizeof(float));

		T0p(0, 0) = MCTISet.pCTI.Element[p]->R[0];
		T0p(0, 1) = MCTISet.pCTI.Element[p]->R[1];
		T0p(0, 2) = MCTISet.pCTI.Element[p]->R[2];
		T0p(0, 3) = MCTISet.pCTI.Element[p]->t[0];
		T0p(1, 0) = MCTISet.pCTI.Element[p]->R[3];
		T0p(1, 1) = MCTISet.pCTI.Element[p]->R[4];
		T0p(1, 2) = MCTISet.pCTI.Element[p]->R[5];
		T0p(1, 3) = MCTISet.pCTI.Element[p]->t[1];
		T0p(2, 0) = MCTISet.pCTI.Element[p]->R[6];
		T0p(2, 1) = MCTISet.pCTI.Element[p]->R[7];
		T0p(2, 2) = MCTISet.pCTI.Element[p]->R[8];
		T0p(2, 3) = MCTISet.pCTI.Element[p]->t[2];
		T0p.block<1, 3>(3, 0) << 0, 0, 0;
		T0p(3, 3) = 1;

		t1 = T0p(0, 0);
		t2 = T0p(0, 1);
		t3 = T0p(0, 2);
		t4 = T0p(0, 3);
		t5 = T0p(1, 0);
		t6 = T0p(1, 1);
		t7 = T0p(1, 2);
		t8 = T0p(1, 3);
		t9 = T0p(2, 0);
		t10 = T0p(2, 1);
		t11 = T0p(2, 2);
		t12 = T0p(2, 3);


		T0i = Tiq*T*T0p.transpose();

		t1 = T0i(0, 0);
		t2 = T0i(0, 1);
		t3 = T0i(0, 2);
		t4 = T0i(0, 3);
		t5 = T0i(1, 0);
		t6 = T0i(1, 1);
		t7 = T0i(1, 2);
		t8 = T0i(1, 3);
		t9 = T0i(2, 0);
		t10 = T0i(2, 1);
		t11 = T0i(2, 2);
		t12 = T0i(2, 3);

		VisualizeAlignedModels(0, i);

		iPrevClusters += m_i;

	}
}

void PSGM::VisualizeAlignedModels(int iRefModel, int iModel)
{
	//Set transform
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	double T_M_S[16];
	
	T_M_S[0]= T0i(0, 0);
	T_M_S[1]= T0i(0, 1);
	T_M_S[2]= T0i(0, 2);
	T_M_S[3]= T0i(0, 3);
	T_M_S[4]= T0i(1, 0);
	T_M_S[5]= T0i(1, 1);
	T_M_S[6]= T0i(1, 2);
	T_M_S[7]= T0i(1, 3);
	T_M_S[8]= T0i(2, 0);
	T_M_S[9] = T0i(2, 1);
	T_M_S[10] = T0i(2, 2);
	T_M_S[11] = T0i(2, 3);
	T_M_S[12] = T0i(3, 0);
	T_M_S[13] = T0i(3, 1);
	T_M_S[14] = T0i(3, 2);
	T_M_S[15] = T0i(3, 3);

	transform->SetMatrix(T_M_S);

	//Scaling Ref. PLY model to meters (if needed)
	vtkSmartPointer<vtkTransform> transformScale = vtkSmartPointer<vtkTransform>::New();
	//transformScale->Scale(0.001,0.001,0.001);
	transformScale->Scale(1,1,1);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iRefModel));	//Get model from DB
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();

	//Transform it
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(transformFilterScale->GetOutput()); //PLY model
	transformFilter->SetTransform(transform);
	transformFilter->Update();

	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//Generate Ref. model polydata
	vtkSmartPointer<vtkPolyData> modelPD = transformFilter->GetOutput();
	//vtkSmartPointer<vtkPolyData> modelPD = transformFilterScale->GetOutput();
	vtkSmartPointer<vtkPolyDataMapper> modelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelMapper->SetInputData(modelPD);
	vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
	modelActor->SetMapper(modelMapper);
	modelActor->GetProperty()->SetColor(0, 1, 0);
	renderer->AddActor(modelActor);


	//Generate scene polydata
	//Scaling PLY model to meters (if needed)
	transformScale = vtkSmartPointer<vtkTransform>::New();
	//transformScale->Scale(0.001,0.001,0.001);
	transformScale->Scale(1,1,1);
	transformFilterScale = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilterScale->SetInputData(vtkModelDB.at(iModel));	//Get model from DB
	transformFilterScale->SetTransform(transformScale);
	transformFilterScale->Update();


	vtkSmartPointer<vtkPolyData> modelSPD = transformFilterScale->GetOutput();
	vtkSmartPointer<vtkPolyDataMapper> modelSMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	modelSMapper->SetInputData(modelSPD);
	vtkSmartPointer<vtkActor> modelSActor = vtkSmartPointer<vtkActor>::New();
	modelSActor->SetMapper(modelSMapper);
	modelSActor->GetProperty()->SetColor(0, 0, 1);
	renderer->AddActor(modelSActor);

	//Start VTK
	renderer->ResetCamera();
	renderer->TwoSidedLightingOff();
	window->Render();
	interactor->Start();

}
#endif 

//Returns XYZ oriented bounding box segment neighbourhood
std::vector<std::vector<int>> PSGM::GetSegmentBBNeighbourhood(float dist, bool verbose)
{
	//number of segments
	int noSegments = this->clusters.n;
	//create list
	std::vector<std::vector<int>> neighbourhood(noSegments);

	//find bounding box for all segments
	float* bbs = new float[noSegments * 2 * 3]; //XYZ, min, max
	memset(bbs, 0, noSegments * 2 * 3 * sizeof(float));
	Array<SURFEL::Vertex *> *vertexArray = &this->pSurfels->vertexArray;
	//QList<QLIST::Index> *pSurfelVertexList;
	//QLIST::Index *qlistelement;
	float* P;
	for (int i = 0; i < noSegments; i++)
	{
		//set the starting bounding box to be equal to first vertex of first surfel
		P = &vertexArray->Element[this->clusters.Element[i]->iVertexArray.Element[0]]->P[0];
		bbs[i * 6] = P[0];
		bbs[i * 6 + 1] = P[0];
		bbs[i * 6 + 2] = P[1];
		bbs[i * 6 + 3] = P[1];
		bbs[i * 6 + 4] = P[2];
		bbs[i * 6 + 5] = P[2];
		//Running through all segment vertices
		for (int v = 1; v < this->clusters.Element[i]->iVertexArray.n; v++)
		{
			P = &vertexArray->Element[this->clusters.Element[i]->iVertexArray.Element[v]]->P[0];

			//set min/max per dimension
			//X
			if (bbs[i * 6] > P[0])
				bbs[i * 6] = P[0];
			else if (bbs[i * 6 + 1] < P[0])
				bbs[i * 6 + 1] = P[0];
			//Y
			if (bbs[i * 6 + 2] > P[1])
				bbs[i * 6 + 2] = P[1];
			else if (bbs[i * 6 + 3] < P[1])
				bbs[i * 6 + 3] < P[1];
			//Z
			//Y
			if (bbs[i * 6 + 4] > P[2])
				bbs[i * 6 + 4] = P[2];
			else if (bbs[i * 6 + 5] < P[2])
				bbs[i * 6 + 5] < P[2];
		}
	}

	//Find neighbourhoods for all segments
	float distance = 0.0;
	float distanceTemp;
	int intersection = 0;
	for (int i = 0; i < noSegments; i++)
	{
		//Check all segments
		for (int j = i + 1; j < noSegments; j++)
		{
			////Check all dimensions
			distance = 0.0;
			intersection = 0;
			for (int k = 0; k < 3; k++)
			{
				//Check intersection per dimension
				if ((bbs[i * 6 + k * 2] < bbs[j * 6 + k * 2 + 1]) && bbs[i * 6 + k * 2] > bbs[j * 6 + k * 2])	//min_i < max_j & min_i > min_j
					intersection++;
				else if ((bbs[j * 6 + k * 2] < bbs[i * 6 + k * 2 + 1]) && (bbs[j * 6 + k * 2] > bbs[i * 6 + k * 2])) //min_j < max_i & min_j > min_i
					intersection++;
				else if ((bbs[j * 6 + k * 2] > bbs[i * 6 + k * 2]) && (bbs[j * 6 + k * 2 + 1] < bbs[i * 6 + k * 2 + 1])) //min_j > min_i & max_j < max_i
					intersection++;
				else if ((bbs[i * 6 + k * 2] > bbs[j * 6 + k * 2]) && (bbs[i * 6 + k * 2 + 1] < bbs[j * 6 + k * 2 + 1])) //min_i > min_j & max_i < max_j
					intersection++;
				
				//Calculate distance per dimension
				if (bbs[j * 6 + k * 2 + 1] < bbs[i * 6 + k * 2]) //max_j < min_i
				{
					distanceTemp = bbs[j * 6 + k * 2 + 1] - bbs[i * 6 + k * 2];
					distance += distanceTemp * distanceTemp;
				}
				else if (bbs[j * 6 + k * 2] > bbs[i * 6 + k * 2 + 1])	//min_j > max_i
				{
					distanceTemp = bbs[j * 6 + k * 2] - bbs[i * 6 + k * 2 + 1];
					distance += distanceTemp * distanceTemp;
				}
			}
			distance = sqrtf(distance);
			
			//Check distance threshold and check if intersection
			if ((distance < dist) || (intersection == 3))
			{
				//Add to neighbourhood
				neighbourhood.at(i).push_back(j);
				neighbourhood.at(j).push_back(i);

				if (verbose)
					std::cout << "Segments " << i << " and " << j << " are in the neighbourhood with distance of: " << distance << ", Intersection: " << intersection << std::endl;
			}
		}
	}

	//deref
	delete[] bbs;

	//return
	return neighbourhood;
}

//FOR DEBUGGING PURPOSES ONLY!!!! Saves data to a file
void PSGM::CheckHypothesesToSegmentEnvelopmentAndCollision_DEBUG(int hyp, int segment, float d1, float d2)
{
	std::fstream dat("CheckHypothesesToSegmentEnvelopment_DEBUG.txt", std::fstream::out);
	dat << "Hypothesis: " << hyp << std::endl;
	dat << "Segment: " << segment << std::endl;
	
	//Hypothesis data
	RECOG::PSGM_::MatchInstance* hypothesis = pCTImatchesArray.Element[hyp];
	VertexGraph * hypVG = MTGSet.vertexGraphs.at(MCTISet.pCTI.Element[hypothesis->iMCTI]->iModel);
	TG* hypTG = MTGSet.TGs.at(MCTISet.pCTI.Element[hypothesis->iMCTI]->iModel);
	TGNode* plane;
	float* N;
	float* minSegDist = new float[hypTG->A.h];
	float *RMS = hypothesis->RICP_;
	float *tMS = hypothesis->tICP_;

	dat << "Model CTI: " << MCTISet.pCTI.Element[hypothesis->iMCTI]->iModel << std::endl;

	//Segment data
	Array<SURFEL::Vertex *> *vertexArray = &this->pSurfels->vertexArray;
	SURFEL::Vertex * rvlvertex;
	float* minModDist = new float[hypTG->A.h];
	RECOG::PSGM_::ModelInstance *pSCTI = CTISet.pCTI.Element[CTISet.SegmentCTIs.Element[segment].Element[0]];
	RECOG::PSGM_::ModelInstanceElement *pSIE = pSCTI->modelInstance.Element;

	float RMSCTI[9];
	float tMSCTI[3];
	float V3Tmp[3];

	RVLCOMPTRANSF3DWITHINV(pSCTI->R, pSCTI->t, RMS, tMS, RMSCTI, tMSCTI, V3Tmp);

	//Transformation vars?
	float tPs[3];
	float tPm[3];
	float sPs[3];
	float sPm[3];
	float st[3] = { tMSCTI[0] / 1000, tMSCTI[1] / 1000, tMSCTI[2] / 1000 };

	//Calculate distances segment vertices to model convex hull
	int totalPts = this->clusters.Element[segment]->iVertexArray.n;
	float distance;
	float segMinDist = 1000000;
	for (int j = 0; j < hypTG->A.h; j++)
	{
		plane = hypTG->descriptor.Element[j].pFirst->ptr;
		minSegDist[j] = 1000000;
		//For each point
		for (int i = 0; i < totalPts; i++)
		{
			rvlvertex = vertexArray->Element[this->clusters.Element[segment]->iVertexArray.Element[i]];

			////Transform vertex to hyp model space
			sPs[0] = 1000 * rvlvertex->P[0];
			sPs[1] = 1000 * rvlvertex->P[1];
			sPs[2] = 1000 * rvlvertex->P[2];
			RVLINVTRANSF3(sPs, RMS, tMS, tPs, V3Tmp);

			dat << tPs[0] << " " << tPs[1] << " " << tPs[2];

			N = &hypTG->A.Element[hypTG->A.w * plane->i];

			//sDistances[i * hypTG->A.h + j] = RVLDOTPRODUCT3(N, tPs) - plane->d;
			distance = (RVLDOTPRODUCT3(N, tPs) - plane->d);
			if (distance > d1)
				dat << " " << 0;
			else
				dat << " " << 1;

			if (distance > -d2)
				dat << " " << 0 << std::endl;
			else
				dat << " " << 1 << std::endl;
		}
	}
}

//
std::map<int, std::vector<int>> PSGM::GetSceneConsistancy(float nDist, float d1, float d2, bool verbose)
{
	//CheckHypothesesToSegmentEnvelopmentAndCollision_DEBUG(scoreMatchMatrixICP.Element[0].Element[0].idx, 0, d1, d2);

	std::map<int, std::vector<int>> constL;

	//Get segment neighbourhood
	std::vector<std::vector<int>> neighbourhood = GetSegmentBBNeighbourhood(nDist);

	//For every segment's hypothesis check if they envelop its source segment and neighbouring segments??? No longer true???
	int resLab = 0;
	for (int i = 0; i < scoreMatchMatrixICP.n; i++)	//Per segment
	{
		for (int j = 0; j < RVLMIN(7, scoreMatchMatrixICP.Element[i].n); j++)	//Per hypothesis
		{
			if (scoreMatchMatrixICP.Element[i].Element[j].idx >= 0) //If hypothesis is valid
			{
				//Check for source segment
				resLab = CheckHypothesesToSegmentEnvelopmentAndCollision(scoreMatchMatrixICP.Element[i].Element[j].idx, i, d1, d2);
				if (resLab == 2)
				{
					//It must be first entry for this hypothesis
					//make new list
					std::vector<int> l;
					l.push_back(i);
					constL.insert(std::make_pair(scoreMatchMatrixICP.Element[i].Element[j].idx, l));
				}
				else if (resLab == 0)
				{
					if (verbose)
						std::cout << "Hypothesis " << scoreMatchMatrixICP.Element[i].Element[j].idx << " invalidated!" << std::endl;
					scoreMatchMatrixICP.Element[i].Element[j].idx = -1;
					continue;
				}

				//Check for neighbourhood segments
				for (int k = 0; k < neighbourhood.at(i).size(); k++)
				{
					resLab = CheckHypothesesToSegmentEnvelopmentAndCollision(scoreMatchMatrixICP.Element[i].Element[j].idx, neighbourhood.at(i).at(k), d1, d2);
					if (resLab == 2)
					{
						if (constL.count(scoreMatchMatrixICP.Element[i].Element[j].idx))
							constL.at(scoreMatchMatrixICP.Element[i].Element[j].idx).push_back(neighbourhood.at(i).at(k)); //Add segment to hypothesis list
						else
						{
							std::vector<int> l;
							l.push_back(neighbourhood.at(i).at(k));
							constL.insert(std::make_pair(scoreMatchMatrixICP.Element[i].Element[j].idx, l));
						}
					}
					else if (resLab == 0)
					{
						if (verbose)
							std::cout << "Hypothesis " << scoreMatchMatrixICP.Element[i].Element[j].idx << " invalidated!" << std::endl;
						scoreMatchMatrixICP.Element[i].Element[j].idx = -1;
						constL.erase(scoreMatchMatrixICP.Element[i].Element[j].idx);
						break;
					}
				}
			}
		}
	}

	if (verbose)
	{
		std::map<int, std::vector<int>>::iterator it;
		for (it = constL.begin(); it != constL.end(); it++)
		{
			for (int i = 0; i < it->second.size(); i++)
				std::cout << "Created pair hypothesis/segment: " << it->first << "/" << it->second.at(i) << std::endl;
		}
	}

	return constL;
}

//
int PSGM::CheckHypothesesToSegmentEnvelopmentAndCollision(int hyp, int segment, float d1, float d2)
{
	int retVal = 0;

	//Hypothesis data
	RECOG::PSGM_::MatchInstance* hypothesis = pCTImatchesArray.Element[hyp];
	VertexGraph * hypVG = MTGSet.vertexGraphs.at(MCTISet.pCTI.Element[hypothesis->iMCTI]->iModel);
	TG* hypTG = MTGSet.TGs.at(MCTISet.pCTI.Element[hypothesis->iMCTI]->iModel);
	TGNode* plane;
	float* N;
	float* minSegDist = new float[hypTG->A.h];
	float *RMS = hypothesis->RICP_;
	float *tMS = hypothesis->tICP_;

	//Segment data
	Array<SURFEL::Vertex *> *vertexArray = &this->pSurfels->vertexArray;	
	SURFEL::Vertex * rvlvertex;
	float* minModDist = new float[hypTG->A.h];
	RECOG::PSGM_::ModelInstance *pSCTI = CTISet.pCTI.Element[CTISet.SegmentCTIs.Element[segment].Element[0]];
	RECOG::PSGM_::ModelInstanceElement *pSIE = pSCTI->modelInstance.Element;

	float RMSCTI[9];
	float tMSCTI[3];
	float V3Tmp[3];

	RVLCOMPTRANSF3DWITHINV(pSCTI->R, pSCTI->t, RMS, tMS, RMSCTI, tMSCTI, V3Tmp);

	//Transformation vars?
	float tPs[3];
	float tPm[3];
	float sPs[3];
	float sPm[3];
	float st[3] = { tMSCTI[0] / 1000, tMSCTI[1] / 1000, tMSCTI[2] / 1000 };

	//Calculate distances segment vertices to model convex hull
	int totalPts = this->clusters.Element[segment]->iVertexArray.n;
	float distance;
	float segMinDist = 1000000;
	bool passedfirst = true;
	for (int j = 0; j < hypTG->A.h; j++)
	{
		plane = hypTG->descriptor.Element[j].pFirst->ptr;
		minSegDist[j] = 1000000;
		//For each point
		for (int i = 0; i < totalPts; i++)
		{
			rvlvertex = vertexArray->Element[this->clusters.Element[segment]->iVertexArray.Element[i]];

			////Transform vertex to hyp model space
			sPs[0] = 1000 * rvlvertex->P[0];
			sPs[1] = 1000 * rvlvertex->P[1];
			sPs[2] = 1000 * rvlvertex->P[2];
			RVLINVTRANSF3(sPs, RMS, tMS, tPs, V3Tmp);

			N = &hypTG->A.Element[hypTG->A.w * plane->i];

			//sDistances[i * hypTG->A.h + j] = RVLDOTPRODUCT3(N, tPs) - plane->d;
			distance = (RVLDOTPRODUCT3(N, tPs) - plane->d);
			if ((distance > d1))
				passedfirst = false;

			if (minSegDist[j] > distance)
				minSegDist[j] = distance;
		}
	}

	if (passedfirst)
		return 2;

	if (!passedfirst)
	{
		//Calcuate distnces model vertices to segment convex hull
		totalPts = hypVG->NodeArray.n;
		for (int j = 0; j < hypTG->A.h; j++)
		{
			plane = hypTG->descriptor.Element[j].pFirst->ptr;
			minModDist[j] = 1000000;
			//For each point
			for (int i = 0; i < totalPts; i++)
			{
				rvlvertex = &hypVG->NodeArray.Element[i];

				////Transform vertex to segment space
				sPm[0] = rvlvertex->P[0] / 1000;
				sPm[1] = rvlvertex->P[1] / 1000;
				sPm[2] = rvlvertex->P[2] / 1000;
				RVLTRANSF3(sPm, RMSCTI, st, tPm, V3Tmp);

				N = &hypTG->A.Element[hypTG->A.w * plane->i];	//Same normal at same index as the model CTI?

				//mDistances[i * hypTG->A.h + j] = RVLDOTPRODUCT3(N, tPm) - pMIE[0].d;
				distance = RVLDOTPRODUCT3(N, tPm) - pSIE[j].d;
				if (minModDist[j] > distance)
					minModDist[j] = distance;
			}
		}

		//find max?
		for (int i = 0; i < hypTG->A.h; i++)
		{
			if ((minSegDist[i] > -d2) || (minModDist[i] > -d2/1000.0))
				return 1;
		}
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////
//
// CUPEC
//
///////////////////////////////////////////////////////////////////////////

#ifndef RVLVERSION_170601
void PSGM::Clusters()
{
	RVL_DELETE_ARRAY(clusterMap);

	clusterMap = new int[pSurfels->NodeArray.n];

	memset(clusterMap, 0xff, pSurfels->NodeArray.n * sizeof(int));

	RVL_DELETE_ARRAY(clusterMem);

	clusterMem = new RECOG::PSGM_::Cluster[pSurfels->NodeArray.n];

	clusters.n = 0;

	RVL_DELETE_ARRAY(clusterSurfelMem);

	clusterSurfelMem = new int[pSurfels->NodeArray.n];

	int *piSurfel = clusterSurfelMem;

	RVL_DELETE_ARRAY(clusterVertexMem);

	clusterVertexMem = new int[pSurfels->nVertexSurfelRelations];

	int *piVertex = clusterVertexMem;

	bool *bVertexVisited = new bool[pSurfels->vertexArray.n];
	bool *bVertexInCluster = new bool[pSurfels->vertexArray.n];

	bool *bSurfelVisited = new bool[pSurfels->NodeArray.n];

	QList<QLIST::Index> candidateList;
	QList<QLIST::Index> *pCandidateList = &candidateList;

	QLIST::Index *candidateMem = new QLIST::Index[pSurfels->NodeArray.n];

	Array<int> surfelBuff1, surfelBuff2;

	surfelBuff1.Element = new int[pSurfels->NodeArray.n];

	surfelBuff1.n = 0;

	int i;
	Surfel *pSurfel;

	for (i = 0; i < pSurfels->NodeArray.n; i++)
	{
		pSurfel = pSurfels->NodeArray.Element + i;

		if (!pSurfel->bEdge)
			surfelBuff1.Element[surfelBuff1.n++] = i;
	}

	surfelBuff2.Element = new int[surfelBuff1.n];

	Array<int> *pSurfelBuff = &surfelBuff1;
	Array<int> *pSurfelBuff_ = &surfelBuff2;

	int nValidClusters = 0;

	Array<int> *pTmp;

#ifdef RVLPSGM_NORMAL_HULL
	Array<RECOG::PSGM_::NormalHullElement> NHull;

	NHull.Element = new RECOG::PSGM_::NormalHullElement[pSurfels->NodeArray.n];
#else
	float meanN[3];
	float sumN[3];
	float wN;
#endif

	RECOG::PSGM_::Cluster *pCluster;
	int iCluster;
	int maxSurfelSize;
	int iLargestSurfel;
	int iFirstNewVertex;
	QLIST::Index *pCandidateIdx, *pBestCandidateIdx;
	QLIST::Index **ppCandidateIdx, **ppBestCandidateIdx;
	float dist, minDist;
	int nSurfelVertices;
	int nSurfelVerticesInCluster;
	int *piVertex_, *piVertex__;
	int iSurfel, iSurfel_;
	Surfel *pSurfel_;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *pVertexIdx;

	for (iCluster = 0; iCluster < pSurfels->NodeArray.n; iCluster++)
	{
		// pSurfel <- the largest surfel which is not assigned to a cluster.

		maxSurfelSize = minInitialSurfelSize - 1;

		iLargestSurfel = -1;

		pSurfelBuff_->n = 0;

		for (i = 0; i < pSurfelBuff->n; i++)
		{
			iSurfel = pSurfelBuff->Element[i];

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			if (!pSurfel->bEdge)
			{
				if (clusterMap[iSurfel] < 0)
				{
					pSurfelVertexList = pSurfels->surfelVertexList.Element + iSurfel;

					if (pSurfelVertexList->pFirst)
					{
						pSurfelBuff_->Element[pSurfelBuff_->n++] = iSurfel;

						if (pSurfel->size > maxSurfelSize)
						{
							maxSurfelSize = pSurfel->size;

							iLargestSurfel = iSurfel;
						}
					}
				}
			}
		}

		pTmp = pSurfelBuff;
		pSurfelBuff = pSurfelBuff_;
		pSurfelBuff_ = pTmp;

		if (iLargestSurfel < 0)
			break;

		//if (iLargestSurfel == 25)
		//	int debug = 0;

		//if (clusters.n == 15)
		//	int debug = 0;

		// Initialize a new cluster.

		pCluster = clusterMem + iCluster;

		if (bOverlappingClusters)
		{
			piSurfel = clusterSurfelMem;
			piVertex = clusterVertexMem;
		}

		pCluster->iSurfelArray.Element = piSurfel;
		pCluster->iVertexArray.Element = piVertex;

		pCluster->iSurfelArray.n = 0;
		pCluster->iVertexArray.n = 0;
		pCluster->size = 0;

		clusters.n++;

		clusterMap[iLargestSurfel] = iCluster;

		memset(bVertexVisited, 0, pSurfels->vertexArray.n * sizeof(bool));
		memset(bVertexInCluster, 0, pSurfels->vertexArray.n * sizeof(bool));
		memset(bSurfelVisited, 0, pSurfels->NodeArray.n * sizeof(bool));

		RVLQLIST_INIT(pCandidateList);

		QLIST::Index *pNewCandidate = candidateMem;

#ifdef RVLPSGM_NORMAL_HULL
		NHull.n = 0;
#else
		RVLNULL3VECTOR(sumN);
		wN = 0.0f;
#endif

		RVLQLIST_ADD_ENTRY(pCandidateList, pNewCandidate);

		pNewCandidate->Idx = iLargestSurfel;

		pNewCandidate++;

		bSurfelVisited[iLargestSurfel] = true;

		// Region growing.

		while (pCandidateList->pFirst)
		{
			// iSurfel <- the best candidate for expanding cluster.

			minDist = PI;

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = *ppCandidateIdx;

			while (pCandidateIdx)
			{
				iSurfel_ = pCandidateIdx->Idx;

				pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

#ifdef RVLPSGM_NORMAL_HULL
				dist = pSurfels->DistanceFromNormalHull(NHull, pSurfel_->N);
#else
				float e = RVLDOTPRODUCT3(meanN, pSurfel_->N);
				dist = (wN < 1e-10 ? 0.0f : acos(e));
#endif

				if (dist < minDist)
				{
					minDist = dist;

					iSurfel = iSurfel_;

					pBestCandidateIdx = pCandidateIdx;

					ppBestCandidateIdx = ppCandidateIdx;
				}

				ppCandidateIdx = &(pCandidateIdx->pNext);

				pCandidateIdx = *ppCandidateIdx;
			}

			// Remove iSurfel from candidateList.

			RVLQLIST_REMOVE_ENTRY(pCandidateList, pBestCandidateIdx, ppBestCandidateIdx);

			//if (iSurfel == 8)
			//	int debug = 0;

			// Add vertices of iSurfel, which are inside convex (or outside concave) surface into cluster.

			iFirstNewVertex = pCluster->iVertexArray.n;

			piVertex_ = piVertex;

			pSurfelVertexList = pSurfels->surfelVertexList.Element + iSurfel;

			nSurfelVertices = nSurfelVerticesInCluster = 0;

			pVertexIdx = pSurfelVertexList->pFirst;

			while (pVertexIdx)
			{
				if (bVertexVisited[pVertexIdx->Idx])
				{
					if (bVertexInCluster[pVertexIdx->Idx])
						nSurfelVerticesInCluster++;
				}
				else
				{
					if (Inside(pVertexIdx->Idx, pCluster, iSurfel))
					{
						*(piVertex++) = pVertexIdx->Idx;

						nSurfelVerticesInCluster++;
					}

				}

				nSurfelVertices++;

				pVertexIdx = pVertexIdx->pNext;
			}

			if (nSurfelVertices == 0)
				continue;

			if (100 * nSurfelVerticesInCluster / nSurfelVertices < minVertexPerc)
			{
				piVertex = piVertex_;

				continue;
			}

			pCluster->iVertexArray.n = piVertex - pCluster->iVertexArray.Element;

			pVertexIdx = pSurfelVertexList->pFirst;

			while (pVertexIdx)
			{
				bVertexVisited[pVertexIdx->Idx] = true;

				pVertexIdx = pVertexIdx->pNext;
			}

			for (piVertex__ = piVertex_; piVertex__ < piVertex; piVertex__++)
				bVertexInCluster[*piVertex__] = true;

			// Add iSurfel to cluster.

			//if (iLargestSurfel == 25 && iSurfel == 192)
			//	int debug = 0;

			clusterMap[iSurfel] = iCluster;

			*(piSurfel++) = iSurfel;

			pCluster->iSurfelArray.n++;

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			pCluster->size += pSurfel->size;

#ifdef RVLPSGM_NORMAL_HULL
			// Update normal hull.

			UpdateNormalHull(NHull, pSurfel->N);
#else
			// Update mean normal.

			UpdateMeanNormal(sumN, wN, pSurfel->N, (float)(pSurfel->size), meanN);
#endif

			// Remove candidates which are not consistent with new vertices added to the cluster.

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = candidateList.pFirst;

			while (pCandidateIdx)
			{
				pSurfel_ = pSurfels->NodeArray.Element + pCandidateIdx->Idx;

				//if (pCandidateIdx->Idx == 8)
				//	int debug = 0;

				if (BelowPlane(pCluster, pSurfel_, iFirstNewVertex))
					ppCandidateIdx = &(pCandidateIdx->pNext);
				else
					RVLQLIST_REMOVE_ENTRY(pCandidateList, pCandidateIdx, ppCandidateIdx)

					pCandidateIdx = pCandidateIdx->pNext;
			}

			// Add new candidates in candidateList.

			SURFEL::EdgePtr *pSurfelEdgePtr = pSurfel->EdgeList.pFirst;

			while (pSurfelEdgePtr)
			{
				iSurfel_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pSurfelEdgePtr);

				//if (iSurfel_ == 8)
				//	int debug = 0;

				if (bOverlappingClusters || clusterMap[iSurfel_] < 0)
				{
					if (!bSurfelVisited[iSurfel_])
					{
						//if (iSurfel_ == 8)
						//	int debug = 0;

						bSurfelVisited[iSurfel_] = true;

						pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

						if (pSurfel_->size > 1)
						{
							if (BelowPlane(pCluster, pSurfel_))
							{
								RVLQLIST_ADD_ENTRY(pCandidateList, pNewCandidate);

								pNewCandidate->Idx = iSurfel_;

								pNewCandidate++;
							}
						}
					}
				}

				pSurfelEdgePtr = pSurfelEdgePtr->pNext;
			}
		}	// region growing loop		

		if (pCluster->bValid = (pCluster->size >= minClusterSize))
			nValidClusters++;

		if (bOverlappingClusters)
		{
			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, pCluster->iSurfelArray.n, pCluster->iSurfelArray.Element);

			memcpy(pCluster->iSurfelArray.Element, clusterSurfelMem, pCluster->iSurfelArray.n * sizeof(int));

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, pCluster->iVertexArray.n, pCluster->iVertexArray.Element);

			memcpy(pCluster->iVertexArray.Element, clusterVertexMem, pCluster->iVertexArray.n * sizeof(int));
		}
		else
			pCluster->orig = pCluster->size;
	}	// for each cluster

	delete[] candidateMem;
	delete[] bVertexVisited;
	delete[] bVertexInCluster;
	delete[] bSurfelVisited;
#ifdef RVLPSGM_NORMAL_HULL
	delete[] NHull.Element;
#endif

	// Sort clusters.

	Array<SortIndex<int>> sortedClusterArray;

	sortedClusterArray.Element = new SortIndex<int>[nValidClusters];

	SortIndex<int> *pSortIndex = sortedClusterArray.Element;

	for (i = 0; i < clusters.n; i++)
	{
		pCluster = clusterMem + i;

		if (pCluster->bValid)
		{
			pSortIndex->cost = pCluster->size;
			pSortIndex->idx = i;
			pSortIndex++;
		}
	}

	sortedClusterArray.n = nValidClusters;

	BubbleSort<SortIndex<int>>(sortedClusterArray, true);

	int j;

	if (bOverlappingClusters)
	{
		memset(clusterMap, 0xff, pSurfels->NodeArray.n * sizeof(int));

		for (i = 0; i < nValidClusters; i++)
		{
			iCluster = sortedClusterArray.Element[i].idx;

			pCluster = clusterMem + iCluster;

			pCluster->orig = 0;

			for (j = 0; j < pCluster->iSurfelArray.n; j++)
			{
				iSurfel = pCluster->iSurfelArray.Element[j];

				pSurfel = pSurfels->NodeArray.Element + iSurfel;

				if (clusterMap[iSurfel] < 0)
				{
					clusterMap[iSurfel] = iCluster;

					pCluster->orig += pSurfel->size;
				}
			}

			sortedClusterArray.Element[i].cost = pCluster->orig;
		}

		BubbleSort<SortIndex<int>>(sortedClusterArray, true);
	}

	// Detect the ground plane and filter all clusters lying on the ground plane.

	if (bDetectGroundPlane)
	{
		Array<int> PtArray;

		PtArray.Element = new int[pMesh->NodeArray.n];

		bGnd = false;

		//int *piPt;
		int iiSurfel;
		//QLIST::Index2 *pPtIdx;
		//MESH::Distribution PtDistribution;
		//float *var;
		//int idx[3];
		//int iTmp;
		float eGnd;
		float *NGnd_;

		for (i = 0; i < nValidClusters; i++)
		{
			iCluster = sortedClusterArray.Element[i].idx;

			pCluster = clusterMem + iCluster;

			if (bGnd)
			{
				//ComputeClusterNormalDistribution(pCluster);

				//if (pCluster->normalDistributionStd1 < minClusterNormalDistributionStd && pCluster->normalDistributionStd2 < minClusterNormalDistributionStd)
				{
					//if (RVLDOTPRODUCT3(NGnd, pCluster->N) >= 0.95)
					{
						for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
						{
							iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

							pSurfel = pSurfels->NodeArray.Element + iSurfel;

							eGnd = RVLDOTPRODUCT3(NGnd, pSurfel->P) - dGnd;

							if (eGnd > groundPlaneTolerance)
								break;
						}

						if (iiSurfel >= pCluster->iSurfelArray.n)
							pCluster->bValid = false;
					}
				}
			}
			else
			{
				if (IsFlat(pCluster->iSurfelArray, NGnd, dGnd, PtArray))
				{
					pCluster->bValid = false;

					bGnd = true;
				}
			}
		}

		delete[] PtArray.Element;
	}

	//// Filter and sort clusters.

	//for (i = 0; i < clusters.n; i++)
	//{
	//	pCluster = clusterMem + i;

	//	if (pCluster->bValid)
	//		ComputeClusterBoundaryDiscontinuityPerc(i);
	//}

	//for (i = 0; i < clusters.n; i++)
	//{
	//	pCluster = clusterMem + i;

	//	if (pCluster->bValid)
	//	{
	//		if (pCluster->size <= maxClusterSize)
	//		{
	//			ComputeClusterNormalDistribution(pCluster);

	//			if (pCluster->size < minSignificantClusterSize)
	//			{
	//				if (pCluster->boundaryDiscontinuityPerc < minClusterBoundaryDiscontinuityPerc)
	//					if (pCluster->normalDistributionStd1 < minClusterNormalDistributionStd || pCluster->normalDistributionStd2 < minClusterNormalDistributionStd)
	//						pCluster->bValid = false;
	//			}
	//		}
	//		else
	//			pCluster->bValid = false;
	//	}
	//}

	RVL_DELETE_ARRAY(clusters.Element);

	clusters.Element = new RECOG::PSGM_::Cluster *[nValidClusters];

	int *clusterIndexMap = new int[clusters.n];

	memset(clusterIndexMap, 0xff, clusters.n * sizeof(int));

	int iCluster_ = 0;

	for (i = 0; i < sortedClusterArray.n; i++)
	{
		iCluster = sortedClusterArray.Element[i].idx;

		pCluster = clusterMem + iCluster;

		if (pCluster->bValid)
		{
			clusterIndexMap[iCluster] = iCluster_;

			clusters.Element[iCluster_] = pCluster;

			iCluster_++;
		}
	}

	clusters.n = iCluster_;

	//int maxClusterSize_ = 0;
	//int size;

	//for (i = 0; i < clusters.n; i++)
	//{
	//	size = clusterMem[i].size;

	//	if (size > maxClusterSize_)
	//		maxClusterSize_ = size;
	//}

	//int maxnBins = 100000;

	//int k = (maxClusterSize_ < maxnBins ? 1 : maxClusterSize_ / maxnBins + 1);

	//int *key = new int[clusters.n];

	//for (i = 0; i < clusters.n; i++)
	//	key[i] = clusterMem[i].size / k;

	//RVL::QuickSort(key, surfelBuff1.Element, clusters.n);

	//RVL_DELETE_ARRAY(clusters.Element);

	//clusters.Element = new RECOG::PSGM_::Cluster *[clusters.n];

	//for (i = 0; i < clusters.n; i++)
	//{
	//	iCluster = surfelBuff1.Element[clusters.n - i - 1];
	//	clusters.Element[i] = clusterMem + iCluster;
	//	surfelBuff2.Element[iCluster] = i;
	//}

	//delete[] key;

	// Update cluster map.	

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		iCluster = clusterMap[iSurfel];

		if (iCluster >= 0)
			clusterMap[iSurfel] = clusterIndexMap[iCluster];
	}

	delete[] clusterIndexMap;
	delete[] sortedClusterArray.Element;
	delete[] surfelBuff1.Element;
	delete[] surfelBuff2.Element;
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// END CUPEC
//
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
//
// VIDOVIC
//
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
//
// END VIDOVIC
//
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
//
// PETRA
//
///////////////////////////////////////////////////////////////////////////

void PSGM::ObjectAlignment(
	Array<int> iSCTIArray,
	RECOG::PSGM_::ModelInstance **SCTIArray,
	Array<int> iMCTIArray,
	RECOG::PSGM_::ModelInstance **MCTIArray,
	Eigen::MatrixXf A,
	float *R,
	float *t,
	bool bTSM)
{
	Eigen::MatrixXf M(4, 66), P, D(66, 1), T(4, 4), T0p(4, 4), Tiq(4, 4), d(1, 66), S, E;

	D.resize(66, iSCTIArray.n);

	RECOG::PSGM_::ModelInstance *pMCTI;
	RECOG::PSGM_::ModelInstanceElement *pMIE;

	for (int k = 0; k < iSCTIArray.n; k++) //for all CTI-s in current model
	{
		pMCTI = SCTIArray[iSCTIArray.Element[k]];
		pMIE = pMCTI->modelInstance.Element;

		for (int di = 0; di < 66; di++)
		{
			D.block<1, 1>(di, k) << pMIE->d;
			pMIE++;
		}
		//if (k!=m_i-1) D.conservativeResize(D.rows(), D.cols() + 1);
		//D.conservativeResize(D.rows(), D.cols() + 1);
	}

	float min = 1000;

	float sum, s;
	int p, q;
	Eigen::VectorXf t_(3);

	for (int j = 0; j < iMCTIArray.n; j++) //for all CTI-s in reference model
	{
		pMCTI = MCTIArray[iMCTIArray.Element[j]];
		pMIE = pMCTI->modelInstance.Element;

		for (int di = 0; di < 66; di++)
		{
			d(0, di) = pMIE->d;
			pMIE++;
		}
		M.block<3, 66>(0, 0) << A;
		M.block<1, 66>(3, 0) << d;
		Eigen::MatrixXf Mt = M.transpose();
		P = (Mt.transpose()*Mt).inverse()*Mt.transpose();
		S = P*D;
		E = D - Mt*S;

		for (int je = 0; je < E.cols(); je++)
		{
			sum = 0;
			for (int ie = 0; ie < E.rows(); ie++)
			{
				sum += E(ie, je)*E(ie, je);
			}
			if (sum < min)
			{
				min = sum;
				p = j;
				q = je;
				t_ = S.block<3, 1>(0, q);
				//s = *(float*)(&S.data()[3 * S.cols() + q]);
				s = S(3, q);
			}
		}

	}
	//I = Eigen::Matrix<float, 3, 3>::Identity();			

	T.block<3, 3>(0, 0) << s, 0, 0, 0, s, 0, 0, 0, s;
	T.block<3, 1>(0, 3) << t_;
	T.block<1, 3>(3, 0) << 0, 0, 0;
	T.block<1, 1>(3, 3) << 1;


	//memcpy(Tiq.block<3, 3>(0, 0).data(), MCTISet.pCTI.Element[iPrevClusters + q]->R, 9 * sizeof(float));
	//memcpy(Tiq.block<3, 1>(0, 3).data(), MCTISet.pCTI.Element[iPrevClusters + q]->t, 3 * sizeof(float));


	//Tiq(0, 0) = MCTISet.pCTI.Element[iPrevClusters + q]->R[0];
	//Tiq(0, 1) = MCTISet.pCTI.Element[iPrevClusters + q]->R[1];
	//Tiq(0, 2) = MCTISet.pCTI.Element[iPrevClusters + q]->R[2];
	//Tiq(0, 3) = MCTISet.pCTI.Element[iPrevClusters + q]->t[0];
	//Tiq(1, 0) = MCTISet.pCTI.Element[iPrevClusters + q]->R[3];
	//Tiq(1, 1) = MCTISet.pCTI.Element[iPrevClusters + q]->R[4];
	//Tiq(1, 2) = MCTISet.pCTI.Element[iPrevClusters + q]->R[5];
	//Tiq(1, 3) = MCTISet.pCTI.Element[iPrevClusters + q]->t[1];
	//Tiq(2, 0) = MCTISet.pCTI.Element[iPrevClusters + q]->R[6];
	//Tiq(2, 1) = MCTISet.pCTI.Element[iPrevClusters + q]->R[7];
	//Tiq(2, 2) = MCTISet.pCTI.Element[iPrevClusters + q]->R[8];
	//Tiq(2, 3) = MCTISet.pCTI.Element[iPrevClusters + q]->t[2];
	pMCTI = SCTIArray[iSCTIArray.Element[q]];
	Tiq(0, 0) = pMCTI->R[0];
	Tiq(0, 1) = pMCTI->R[1];
	Tiq(0, 2) = pMCTI->R[2];
	Tiq(0, 3) = pMCTI->t[0];
	Tiq(1, 0) = pMCTI->R[3];
	Tiq(1, 1) = pMCTI->R[4];
	Tiq(1, 2) = pMCTI->R[5];
	Tiq(1, 3) = pMCTI->t[1];
	Tiq(2, 0) = pMCTI->R[6];
	Tiq(2, 1) = pMCTI->R[7];
	Tiq(2, 2) = pMCTI->R[8];
	Tiq(2, 3) = pMCTI->t[2];
	Tiq.block<1, 3>(3, 0) << 0, 0, 0;
	Tiq(3, 3) = 1;

	float t1 = Tiq(0, 0);
	float t2 = Tiq(0, 1);
	float t3 = Tiq(0, 2);
	float t4 = Tiq(0, 3);
	float t5 = Tiq(1, 0);
	float t6 = Tiq(1, 1);
	float t7 = Tiq(1, 2);
	float t8 = Tiq(1, 3);
	float t9 = Tiq(2, 0);
	float t10 = Tiq(2, 1);
	float t11 = Tiq(2, 2);
	float t12 = Tiq(2, 3);

	//memcpy(T0p.block<3, 3>(0, 0).data(), MCTISet.pCTI.Element[p]->R, 9 * sizeof(float));
	//memcpy(T0p.block<3, 1>(0, 3).data(), MCTISet.pCTI.Element[p]->t, 3 * sizeof(float));
	pMCTI = MCTIArray[iMCTIArray.Element[p]];
	T0p(0, 0) = pMCTI->R[0];
	T0p(0, 1) = pMCTI->R[1];
	T0p(0, 2) = pMCTI->R[2];
	T0p(0, 3) = pMCTI->t[0];
	T0p(1, 0) = pMCTI->R[3];
	T0p(1, 1) = pMCTI->R[4];
	T0p(1, 2) = pMCTI->R[5];
	T0p(1, 3) = pMCTI->t[1];
	T0p(2, 0) = pMCTI->R[6];
	T0p(2, 1) = pMCTI->R[7];
	T0p(2, 2) = pMCTI->R[8];
	T0p(2, 3) = pMCTI->t[2];
	T0p.block<1, 3>(3, 0) << 0, 0, 0;
	T0p(3, 3) = 1;

	t1 = T0p(0, 0);
	t2 = T0p(0, 1);
	t3 = T0p(0, 2);
	t4 = T0p(0, 3);
	t5 = T0p(1, 0);
	t6 = T0p(1, 1);
	t7 = T0p(1, 2);
	t8 = T0p(1, 3);
	t9 = T0p(2, 0);
	t10 = T0p(2, 1);
	t11 = T0p(2, 2);
	t12 = T0p(2, 3);

	T0i = Tiq*T*T0p.inverse();

	if (bTSM)
		T = T0i.inverse();
	else
		T = T0i;

	RVLMXEL(R,3,0,0) = T(0, 0);
	RVLMXEL(R,3,0,1) = T(0, 1);
	RVLMXEL(R,3,0,2) = T(0, 2);
	t[0] = T(0, 3);
	RVLMXEL(R,3,1,0) = T(1, 0);
	RVLMXEL(R,3,1,1) = T(1, 1);
	RVLMXEL(R,3,1,2) = T(1, 2);
	t[1] = T(1, 3);
	RVLMXEL(R,3,2,0) = T(2, 0);
	RVLMXEL(R,3,2,1) = T(2, 1);
	RVLMXEL(R,3,2,2) = T(2, 2);
	t[2] = T(2, 3);
}

void PSGM::ObjectAlignment()
{
	FILE *fpTranspose;
	fpTranspose = fopen("D:\\ARP3D\\mug_transpose.txt", "w");

	//apple-> 0, 10; banana -> 10, 6; bottle -> 16, 69; bowl -> 85, 15; car -> 100, 26 
	//donut -> 126, 10; hammer -> 136, 32; tetra_pak -> 168, 22; toilet_paper -> 190, 6 mug ->196, 61
	int iRefObject = 0;
	int nObjectsInClass = 3;

	Eigen::MatrixXf A(3, 66);

	A = ConvexTemplatenT(); //normals

	//int n = MCTISet.nModels; // number of models in database
	int n = nObjectsInClass;

	RECOG::PSGM_::ModelInstance **MCTIArray = MCTISet.pCTI.Element;

	Array<int> iMCTIArray = MCTISet.SegmentCTIs.Element[iRefObject];

	Array<int> iSCTIArray;
	float R[9];
	float t[3];
		
	for (int i = 1; i < n; i++) //for all non-reference models
	{
		iSCTIArray = MCTISet.SegmentCTIs.Element[iRefObject + i];

		ObjectAlignment(iSCTIArray, MCTIArray, iMCTIArray, MCTIArray, A, R, t);

		fprintf(fpTranspose, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", 
			RVLMXEL(R, 3, 0, 0),
			RVLMXEL(R, 3, 0, 1),
			RVLMXEL(R, 3, 0, 2),
			t[0],
			RVLMXEL(R, 3, 1, 0),
			RVLMXEL(R, 3, 1, 1),
			RVLMXEL(R, 3, 1, 2),
			t[1],
			RVLMXEL(R, 3, 0, 0),
			RVLMXEL(R, 3, 2, 0),
			RVLMXEL(R, 3, 2, 1),
			t[2]);

		VisualizeAlignedModels(0, i);
	}
	fclose(fpTranspose);
}


///////////////////////////////////////////////////////////////////////////
//
// END PETRA
//
///////////////////////////////////////////////////////////////////////////
