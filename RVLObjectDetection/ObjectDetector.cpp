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
	pPSGM = NULL;
}


ObjectDetector::~ObjectDetector()
{
	if (pSurfels)
		delete pSurfels;

	if (pSurfelDetector)
		delete pSurfelDetector;

	if (pObjects)
		delete pObjects;

	if (pPSGM)
		delete pPSGM;

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

void SmoothMesh(vtkSmartPointer<vtkPolyData> pd, int noIter)
{
	//Bilateral filtring
	vtkSmartPointer<vtkPolyData> norPD = pd;
	norPD->BuildLinks();
	vtkSmartPointer<vtkFloatArray> norNormalData = vtkFloatArray::SafeDownCast(norPD->GetPointData()->GetNormals());
	vtkSmartPointer<vtkPoints> norPts = norPD->GetPoints();
	int noPts = norPts->GetNumberOfPoints();
	float sumW = 0.0;
	float sumPts[3];
	float sumNorm[3];
	int noCells = 0;
	vtkSmartPointer<vtkIdList> ptCells = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkIdList> ptCellPts = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkPoints> pointsCpy = vtkSmartPointer<vtkPoints>::New();	//points copy where new coordinates will go
	pointsCpy->SetDataTypeToFloat();
	pointsCpy->SetNumberOfPoints(640 * 480);
	//pointsCpy->DeepCopy(points);
	vtkSmartPointer<vtkFloatArray> normalsCpy = vtkSmartPointer<vtkFloatArray>::New();	//point normals copy
	normalsCpy->SetNumberOfComponents(3);
	normalsCpy->SetNumberOfTuples(640 * 480);
	int ptID = 0;
	float tempDist2 = 0.0;
	float tempL1 = 0.0;
	float tempW = 0.0;
	float currNor[3];
	float tempNor[3];
	float newPt[3];
	float newNor[3];
	double currPt[3];
	double pt1[3];
	for (int it = 0; it < noIter; it++)
	{
		for (int i = 0; i < noPts; i++)
		{
			norPts->GetPoint(i, currPt);
			if ((currPt[0] <= -1.0) && (currPt[1] <= -1.0) && (currPt[2] <= -1.0))
			{
				pointsCpy->SetPoint(i, -1.0, -1.0, -1.0);
				continue;
			}
			norNormalData->GetTupleValue(i, currNor);
			norPD->GetPointCells(i, ptCells);
			sumPts[0] = 0.0; sumPts[1] = 0.0; sumPts[2] = 0.0;
			sumNorm[0] = 0.0; sumNorm[1] = 0.0; sumNorm[2] = 0.0;
			sumW = 0.0;
			noCells = ptCells->GetNumberOfIds();
			for (int j = 0; j < noCells; j++)
			{
				ptCellPts->Reset();
				norPD->GetCellPoints(ptCells->GetId(j), ptCellPts);
				for (int k = 0; k < 3; k++)
				{
					ptID = ptCellPts->GetId(k);
					if (ptID != i)
					{
						norPts->GetPoint(ptID, pt1);
						norNormalData->GetTupleValue(ptID, tempNor);
						//calculate distance
						tempDist2 = sqrt(vtkMath::Distance2BetweenPoints(currPt, pt1));
						//calculate L1 normals norm
						tempL1 = abs(currNor[0] - tempNor[0]) + abs(currNor[1] - tempNor[1]) + abs(currNor[2] - tempNor[2]);
						//calculate weight
						tempW = exp(tempDist2) * exp(tempL1);
						//calculate sum weight and sum point coordinates and sum normals
						sumW += tempW;
						sumPts[0] += tempW * pt1[0];
						sumPts[1] += tempW * pt1[1];
						sumPts[2] += tempW * pt1[2];
						sumNorm[0] += tempW * tempNor[0];
						sumNorm[1] += tempW * tempNor[1];
						sumNorm[2] += tempW * tempNor[2];
					}
				}
			}
			//calculate new point coordinates and new normals
			if (sumW == 0.0)
			{
				pointsCpy->SetPoint(i, currPt);
				normalsCpy->SetTuple(i, currNor);
			}
			else
			{
				newPt[0] = sumPts[0] / sumW;
				newPt[1] = sumPts[1] / sumW;
				newPt[2] = sumPts[2] / sumW;
				newNor[0] = sumNorm[0] / sumW;
				newNor[1] = sumNorm[1] / sumW;
				newNor[2] = sumNorm[2] / sumW;
				pointsCpy->SetPoint(i, newPt);
				normalsCpy->SetTuple(i, newNor);
			}
		}
		norPD->SetPoints(pointsCpy);
		//norPD->Print(cout);
		norPD->GetPointData()->SetNormals(normalsCpy);
		std::cout << "Iteration " << it << " finished!" << std::endl;
	}
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

		//SmoothMesh(mesh.pPolygonData, 10);

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
			pObjects->GetVertices();	
			pPSGM->Init(&mesh);
			//pPSGM->CTIs(pObjects, &CTIs);
			pPSGM->convexTemplate = pPSGM->convexTemplateBox;
			pPSGM->CTIs(pObjects, &boundingBoxes);
			//pPSGM->convexTemplate = pPSGM->convexTemplate66;
			pObjects->pMesh = &mesh;
			pObjects->DetermineObjectConvexityData(convexityThr, 0.15, bConcaveObjectAggregation, true);
			pObjects->vpObjectAggregationLevel2CriterionData = this;
			pObjects->ExtFuncCheckIfWithinVolume = &RVL::ObjectDetector::CheckIfWithinCTIBoundingBox;
			pObjects->ObjectAggregationLevel2_ViaObjectPairConvexity(convexityThr, convexityRatioThr1, convexityRatioThr2, pObjects->minObjectSize, true);
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

void ObjectDetector::BoundingBox(
	int iObject1,
	int iObject2,
	RECOG::PSGM_::ModelInstance *pBoundingBox)
{
	if (!pPSGM->bGnd)
		return;

	if (boundingBoxes.SegmentCTIs.n <= iObject1 && boundingBoxes.SegmentCTIs.n <= iObject2)
		return;

	int iObject[2];

	iObject[0] = iObject1;
	iObject[1] = iObject2;

	SURFEL::Object *pObject[2];

	pObject[0] = pObjects->objectArray.Element + iObject1;
	pObject[1] = pObjects->objectArray.Element + iObject2;

	float *R_ = NULL;

	float varX = 0.0;

	int i, j;
	int iCTI;
	RECOG::PSGM_::ModelInstance *pCTI;

	for (i = 0; i < 2; i++)
		if (boundingBoxes.SegmentCTIs.Element[iObject[i]].n > 0)
		{
			iCTI = boundingBoxes.SegmentCTIs.Element[iObject[i]].Element[0];

			pCTI = boundingBoxes.pCTI.Element[iCTI];

			if (R_ == NULL || pCTI->varX < varX)
			{
				varX = pCTI->varX;
				R_ = pCTI->R;
			}
		}

	if (R_ == NULL)
		return;

	float *R = pBoundingBox->R;

	RVLCOPYMX3X3(R_, R);

	float *t = pBoundingBox->t;

	RVLNULL3VECTOR(t);

	Array<int> iVertexArray;

	iVertexArray.n = pObject[0]->iVertexArray.n + pObject[1]->iVertexArray.n;
	iVertexArray.Element = new int[iVertexArray.n];

	int *piVertex = iVertexArray.Element;

	for (i = 0; i < 2; i++)
		for (j = 0; j < pObject[i]->iVertexArray.n; j++)
			*(piVertex++) = pObject[i]->iVertexArray.Element[j];

	Array<RECOG::PSGM_::Plane> convexTemplateTmp = pPSGM->convexTemplate;

	pPSGM->convexTemplate = pPSGM->convexTemplateBox;

	pPSGM->FitModel(iVertexArray, pBoundingBox, true);

	pPSGM->convexTemplate = convexTemplateTmp;

	delete[] iVertexArray.Element;
}

void OBJECT_DETECTION::Symmetry(
	SURFEL::ObjectGraph *pObjects,
	int iObject1,
	int iObject2,
	void *vpData)
{
	ObjectDetector *pObjectDetector = (ObjectDetector *)vpData;

	Array<RECOG::PSGM_::SymmetryMatch> symmetryMatch;

	symmetryMatch.Element = new RECOG::PSGM_::SymmetryMatch[pObjectDetector->pPSGM->convexTemplate.n];

	float symmetryScore = pObjectDetector->pPSGM->Symmetry(pObjects, iObject1, iObject2, &(pObjectDetector->CTIs), symmetryMatch);

	if (pObjectDetector->pObjects->fpSymmetry)
	{
		Array<int> iCTIArray = pObjectDetector->CTIs.SegmentCTIs.Element[iObject1];

		if (iCTIArray.n > 0)
		{
			fprintf(pObjectDetector->pObjects->fpSymmetry, "%d\t%d\t%f\t", iObject1, iObject2, symmetryScore);

			bool *b = new bool[pObjectDetector->pPSGM->convexTemplate.n];

			memset(b, 0, pObjectDetector->pPSGM->convexTemplate.n * sizeof(bool));

			int i;

			for (i = 0; i < symmetryMatch.n; i++)
				b[symmetryMatch.Element[i].iCTIElement] = true;

			int iCTI = iCTIArray.Element[0];

			RECOG::PSGM_::ModelInstance *pCTI = pObjectDetector->CTIs.pCTI.Element[iCTI];

			for (i = 0; i < pObjectDetector->pPSGM->convexTemplate.n; i++)
				fprintf(pObjectDetector->pObjects->fpSymmetry, "%f\t", pCTI->modelInstance.Element[i].d);

			for (i = 0; i < pObjectDetector->pPSGM->convexTemplate.n; i++)
				fprintf(pObjectDetector->pObjects->fpSymmetry, "%d\t", (int)b[i]);

			fprintf(pObjectDetector->pObjects->fpSymmetry, "\n");

			delete[] b;
		}
	}	

	delete[] symmetryMatch.Element;
}

bool ObjectDetector::CheckIfWithinCTIBoundingBox(void * odObj, int iObject1, int iObject2, float dimThr)
{
	ObjectDetector * od = (ObjectDetector*)odObj;
	//Generate CTI bounding box
	RECOG::PSGM_::ModelInstance boundingBox;
	boundingBox.modelInstance.Element = new RECOG::PSGM_::ModelInstanceElement[6];
	od->BoundingBox(od->pObjects->iObjectAssignedToNode[iObject1], od->pObjects->iObjectAssignedToNode[iObject2], &boundingBox);

	//Determine bounding box dimensions
	float dims[3];
	dims[0] = abs(boundingBox.modelInstance.Element[4].d + boundingBox.modelInstance.Element[1].d); //"X"
	dims[1] = abs(boundingBox.modelInstance.Element[5].d + boundingBox.modelInstance.Element[2].d); //"Y"
	dims[2] = abs(boundingBox.modelInstance.Element[0].d + boundingBox.modelInstance.Element[3].d); //"Z"

	if ((dims[0] < dimThr) && (dims[1] < dimThr) && (dims[2] < dimThr))
		return true;
	else
		return false;

	delete[] boundingBox.modelInstance.Element;
}

