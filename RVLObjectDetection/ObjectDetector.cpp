#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "ObjectGraph.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "PSGM.h"
#include "ObjectDetector.h"

using namespace RVL;

ObjectDetector::ObjectDetector()
{
	SVMClassifierParamsFileName = NULL;
	cfgFileName = NULL;

	flags = 0x00000000;

	convexityThr = 0.010f;
	convexityRatioThr1 = 0.77f;
	convexityRatioThr2 = 0.75f;

	bSegmentToObjects = false;
	bObjectAggregationLevel2 = false;
	bCTIBasedObjectAggregation = false;
	bConcaveObjectAggregation = false;

	pSurfels = NULL;
	pSurfelDetector = NULL;
	pObjects = NULL;
	vpMeshBuilder = NULL;
}


ObjectDetector::~ObjectDetector()
{
	if (pSurfels)
		delete pSurfels;

	if (pSurfelDetector)
		delete pSurfelDetector;

	if (pObjects)
		delete pObjects;

	RVL_DELETE_ARRAY(cfgFileName);
	RVL_DELETE_ARRAY(SVMClassifierParamsFileName);
}


void ObjectDetector::Init()
{
	CreateParamList();

	if (cfgFileName)
		ParamList.LoadParams(cfgFileName);

	if (flags & RVLOBJECTDETECTION_FLAG_SAVE_SSF)
		flags |= RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT;

	pSurfels = new SurfelGraph;

	pSurfels->pMem = pMem;

	pSurfels->CreateParamList(pMem0);

	pSurfels->ParamList.LoadParams(cfgFileName);

	pSurfelDetector = new PlanarSurfelDetector;

	pSurfelDetector->CreateParamList(pMem0);

	pSurfelDetector->ParamList.LoadParams(cfgFileName);

	pObjects = new SURFEL::ObjectGraph;

	pObjects->CreateParamList(pMem0);

	pObjects->ParamList.LoadParams(cfgFileName);

	if (pObjects->relationClassifier == RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_SVM)
	{
		std::cout << "Initializing SVM Classifier!" << std::endl;
		pObjects->InitSVMClassifier(SVMClassifierParamsFileName);
	}

	pObjects->objectAggregationLevel2Criterion = OBJECT_DETECTION::Symmetry;
	pObjects->vpObjectAggregationLevel2CriterionData = this;

	pPSGM = new PSGM;

	pPSGM->CreateParamList(pMem0);

	pPSGM->ParamList.LoadParams(cfgFileName);

	pPSGM->pMem = pMem;

	pPSGM->pSurfels = pSurfels;

	pPSGM->pSurfelDetector = pSurfelDetector;
}

void ObjectDetector::CreateParamList()
{
	ParamList.m_pMem = pMem0;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("Save PLY", RVLPARAM_TYPE_FLAG, &flags);
	ParamList.AddID(pParamData, "yes", RVLOBJECTDETECTION_FLAG_SAVE_PLY);
	pParamData = ParamList.AddParam("Save SSF", RVLPARAM_TYPE_FLAG, &flags);
	ParamList.AddID(pParamData, "yes", RVLOBJECTDETECTION_FLAG_SAVE_SSF);
	pParamData = ParamList.AddParam("ObjectDetector.Segmentation GT", RVLPARAM_TYPE_FLAG, &flags);
	ParamList.AddID(pParamData, "yes", RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT);
	pParamData = ParamList.AddParam("ObjectDetector.SegmentToObjects", RVLPARAM_TYPE_BOOL, &bSegmentToObjects);
	pParamData = ParamList.AddParam("ObjectDetector.ObjectAggregationLevel2", RVLPARAM_TYPE_BOOL, &bObjectAggregationLevel2);
	pParamData = ParamList.AddParam("ObjectDetector.SVMClassifierParamsFileName", RVLPARAM_TYPE_STRING, SVMClassifierParamsFileName);
	pParamData = ParamList.AddParam("ObjectDetector.CTIBasedObjectAggregation", RVLPARAM_TYPE_BOOL, &bCTIBasedObjectAggregation);
	pParamData = ParamList.AddParam("ObjectDetector.convexityThr", RVLPARAM_TYPE_FLOAT, &convexityThr);
	pParamData = ParamList.AddParam("ObjectDetector.convexityRatioThr1", RVLPARAM_TYPE_FLOAT, &convexityRatioThr1);
	pParamData = ParamList.AddParam("ObjectDetector.convexityRatioThr2", RVLPARAM_TYPE_FLOAT, &convexityRatioThr2);
	pParamData = ParamList.AddParam("ObjectGraph.concaveObjectAggregation", RVLPARAM_TYPE_BOOL, &bConcaveObjectAggregation);
}

void ObjectDetector::DetectObjects(char *MeshFilePathName)
{
	// Segmentation to surfels.

	bSurfelsFromSSF = false;

	char *fileExtension = RVLGETFILEEXTENSION(MeshFilePathName);	

	if (strcmp(fileExtension, "ssf") == 0)
	{
		// Read surfels from a ssf-file.

		std::string ssfFileName(MeshFilePathName);
		ssfFileName.erase(ssfFileName.find_last_of("."));
		ssfFileName += ".ssf";

		std::cout << "Loading and creating ObjectGraph from " << ssfFileName.data() << "." << std::endl;
		pObjects->CreateFromSSF(ssfFileName);

		std::cout << "Compute relation cost!" << std::endl;
		pObjects->ComputeRelationCosts();

		bSurfelsFromSSF = true;
	}
	else
	{
		// Read mesh from file.

		printf("Creating mesh from %s:\n", MeshFilePathName);

		if (LoadMesh(vpMeshBuilder, MeshFilePathName, &mesh, (flags & RVLOBJECTDETECTION_FLAG_SAVE_PLY) != 0))
			printf("Mesh created.\n");
		else
			printf("ERROR: Mesh can't be created!\n");

		// Segment mesh to surfels.				

		pSurfels->Init(&mesh);

		pSurfelDetector->Init(&mesh, pSurfels, pMem);

		printf("Segmentation to surfels... ");

		double StartTime = pSurfelDetector->pTimer->GetTime();

		pSurfelDetector->Segment(&mesh, pSurfels);

		double ExecTime = pSurfelDetector->pTimer->GetTime() - StartTime;

		printf("completed.\n");
		printf("No. of surfels = %d\n", pSurfels->NodeArray.n);
		printf("Total segmentation time = %lf s\n", ExecTime);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
		if (flags & RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT)
			pSurfels->AssignGroundTruthSegmentation(MeshFilePathName, pSurfelDetector->minSurfelSize);

		// Group surfels into objects.

		if (bSegmentToObjects || (flags & RVLOBJECTDETECTION_FLAG_SAVE_SSF))
		{
			printf("Computing relations between adjacent surfels...");

			pSurfels->ImageAdjacency(&mesh);

			Surfel *pSurfel = pSurfels->NodeArray.Element;

			for (int i = 0; i < pSurfels->NodeArray.n; pSurfel++, i++)
			{
				if (pSurfel->size <= 1)
					continue;

				//if (pSurfel->bEdge)
				//	continue;

				pSurfels->DetermineImgAdjDescriptors(pSurfel, &mesh);
			}

			pObjects->Create(pSurfels);

			pObjects->ComputeRelationCosts();

			printf("completed.\n");

			pObjects->Debug();

			// Detect vertices.

			pSurfels->DetectVertices(&mesh);

			// Assign mesh to PSGM.

			pPSGM->pMesh = &mesh;
		}

		if (flags & RVLOBJECTDETECTION_FLAG_SAVE_SSF)
		{
			std::string ssfFileName(MeshFilePathName);
			ssfFileName.erase(ssfFileName.find_last_of("."));
			ssfFileName += ".ssf";

			std::cout << "Saving SSF!" << std::endl;
			pSurfels->GenerateSSF(ssfFileName, pSurfelDetector->minSurfelSize, false);
			std::cout << "Saved!" << std::endl;
		}
#endif
	}	// If fileExtension != "ssf"

#ifdef RVLSURFEL_IMAGE_ADJACENCY
	if (bSegmentToObjects)
	{
		printf("Aggregating surfels into objects... ");

		pObjects->WERSegmentation();

		printf("completed.\n");

		if (!bSurfelsFromSSF && bObjectAggregationLevel2)
		{
			printf("Aggregating objects (LEVEL 2)... ");

			//pSurfels->DetectVertices(&mesh);

			//Generate color histograms for surfels
			/*std::string imgFileName(MeshFileName);
			imgFileName.erase(imgFileName.find_last_of("."));
			imgFileName += ".png";
			cv::Mat img = cv::imread(imgFileName);
			cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
			int binsize[3] = { 8, 8, 0 };
			surfels.CalculateSurfelsColorHistograms(img, RVLColorDescriptor::ColorSpaceList::HSV, false, binsize, true);
			objects.CalculateObjectsColorHistogram();
			TestCHMatching(&objects);*/
			////Filko
			//objects.DetermineObjectConvexityData(0.005, 0.5);
			//ObjectAggregationLevel2(&objects, &surfels, &mesh, MeshFileName);
			cv::imshow("Colored object image", pObjects->CreateSegmentationImage());
			cv::waitKey(1);
			/*VisualizeObjectGraphVertexPointCloud(&objects, 100);*/
			//if (bCTIBasedObjectAggregation)
			if (pObjects->objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_SYMMETRY)
				pPSGM->InitSymmetry(pObjects);
			pObjects->DetermineObjectConvexityData(convexityThr, 0.15, bConcaveObjectAggregation);
			pObjects->ObjectAggregationLevel2_ViaObjectPairConvexity(convexityThr, convexityRatioThr1, convexityRatioThr2, 300);
			pPSGM->FreeSymmetry();
			cv::imshow("New Colored object image", pObjects->CreateSegmentationImage());
			cv::waitKey(1);
			////
			//Evaluation
			/*int E[2];
			int N = 0;
			objects.CalculateOverAndUnderSegmentation(E, N, false, "", false);
			std::cout << "Oversegmenation error: " << 100.0f * (1 - E[0] / (float)N) << "%" << std::endl;
			std::cout << "Undersegmenation error: " << 100.0f * E[1] / (float)N << "%" << std::endl;*/

			printf("completed.\n");
		}
	}
#endif
}

void ObjectDetector::Evaluate(
	FILE *fp,
	char *fileName)
{
#ifdef RVLSURFEL_IMAGE_ADJACENCY
	if (bSegmentToObjects)
	{
		int E[2];
		int N = 0;

		if (bSurfelsFromSSF)
			pObjects->CalculateOverAndUnderSegmentation_SSF(E, N, true, false);
		else
			pObjects->CalculateOverAndUnderSegmentation(E, N, true, std::string(fileName), false);

		std::cout << "Oversegmenation error: " << 100.0f * (1 - E[0] / (float)N) << "%" << std::endl;
		std::cout << "Undersegmenation error: " << 100.0f * E[1] / (float)N << "%" << std::endl;

		if (fp)
			fprintf(fp, "%s\t%d\t%d\t%d\n", fileName, E[0], E[1], N);
	}
#endif
}

void ObjectDetector::CTIs()
{
	pPSGM->CTIs(pObjects, &(pPSGM->CTISet));

	FILE *fp = fopen("CTIs.txt", "w");

	pPSGM->SaveCTIs(fp, &(pPSGM->CTISet));

	fclose(fp);
}

void OBJECT_DETECTION::Symmetry(
	SURFEL::ObjectGraph *pObjects,
	int iObject1,
	int iObject2,
	void *vpData)
{
	ObjectDetector *pObjectDetector = (ObjectDetector *)vpData;

	pObjectDetector->pPSGM->Symmetry(pObjects, iObject1, iObject2);
}