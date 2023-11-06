#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "ReconstructionEval.h"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "ObjectGraph.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "ObjectDetector.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
#include "NeighborhoodTool.h"
#include "VN.h"
#include "VNClass.h"
#include "VNInstance.h"
#include "VNClassifier.h"
#include <tuple>
#include <vector>
#include <set>
#include <sstream>

using namespace RVL;
using namespace OBJECT_DETECTION;

ObjectDetector::ObjectDetector()
{
	SVMClassifierParamsFileName = NULL;
	cfgFileName = NULL;

	flags = 0x00000000;
	objectAggregationLevel1Method = RVLOBJECTDETECTION_METHOD_FUZZY_WER;

	camera.fu = 525.0f;
	camera.fv = 525.0f;
	camera.uc = 320.0f;
	camera.vc = 240.0f;
	camera.w = 640;
	camera.h = 480;

	convexityThr = 0.010f;
	convexityRatioThr1 = 0.77f;
	convexityRatioThr2 = 0.75f;
	connectedComponentMaxDist = 0.100f;

	nMultilateralFilterIterations = 10;
	joinSmallObjectsToLargestNeighborSizeThr = 1000;
	joinSmallObjectsToLargestNeighborDistThr = 0.020f;
	appearanceCost = 0.0002f;

	bSegmentToObjects = false;
	bObjectAggregationLevel2 = false;
	bCTIBasedObjectAggregation = false;
	bTrainingHMI = false;
	bDisplay = false;
	bMultilateralFilter = false;
	bJoinSmallObjectsToLargestNeighbor = false;
	bGroundTruthSegmentation = false;
	bGroundTruthSegmentationOnSurfelLevel = false;
	bGroundTruthBoundingBoxes = false;
	bOwnsSurfelDetectionTool = false;
	bOwnsPSGM = false;
	bOwnConvexConcaveClustering = false;
	bOwnsVNClassifier = false;
	visualizationData.bVisualizeConvexClusters = false;
	visualizationData.bVisualizeConcaveClusters = false;
	visualizationData.bVisualizeSurfels = false;
	visualizationData.bVisualizeAllClusters = false;
	visualizationData.bVisualizeConvexHulls = false;
	bHypothesisLog = false;

	pSurfels = NULL;
	pSurfelDetector = NULL;
	pObjects = NULL;
	vpMeshBuilder = NULL;
	pPSGM = NULL;
	pConvexClustering = pConcaveClustering = NULL;
	objectMap.Element = NULL;
	SClusters.Element = NULL;
	clusterMap = NULL;
	vpVNClassifier = NULL;
	resultsFolder = NULL;
	visualizationData.clusterVisualizationData.clusterColor = NULL;

	debug1 = debug2 = -1;
}


ObjectDetector::~ObjectDetector()
{
	if (bOwnsSurfelDetectionTool)
	{
		if (pSurfels)
			delete pSurfels;

		if (pSurfelDetector)
			delete pSurfelDetector;
	}

	if (pObjects)
		delete pObjects;

	if (bOwnsPSGM)
	{
		if (pPSGM)
		{
			pPSGM->vpObjectDetector = NULL;

			delete pPSGM;
		}
	}

	if (bOwnConvexConcaveClustering)
	{
		if (pConvexClustering)
			delete pConvexClustering;

		if (pConcaveClustering)
			delete pConcaveClustering;
	}

	if (bOwnsVNClassifier)
	{
		if (vpVNClassifier)
		{
			VNClassifier *pVNClassifier = (VNClassifier *)vpVNClassifier;

			delete pVNClassifier;
		}
	}

	RVL_DELETE_ARRAY(objectMap.Element);
	RVL_DELETE_ARRAY(SClusters.Element);
	RVL_DELETE_ARRAY(clusterMap);

	RVL_DELETE_ARRAY(cfgFileName);

	RVL_DELETE_ARRAY(SVMClassifierParamsFileName);

	RVL_DELETE_ARRAY(visualizationData.clusterVisualizationData.clusterColor);
}


void ObjectDetector::Init(
	PSGM *pPSGM_,
	void *vpVNClassifier_)
{
	CreateParamList();

	if (cfgFileName)
		ParamList.LoadParams(cfgFileName);

	if (flags & RVLOBJECTDETECTION_FLAG_SAVE_SSF)
		flags |= RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT;

	pSurfels = NULL;

	pSurfelDetector = NULL;

	if (pPSGM_)
	{
		pPSGM = pPSGM_;

		pSurfels = pPSGM->pSurfels;

		pSurfelDetector = pPSGM->pSurfelDetector;

		bOwnsSurfelDetectionTool = false;
		bOwnsPSGM = false;
	}
	else
	{
		pPSGM = new PSGM;

		pPSGM->CreateParamList(pMem0);

		pPSGM->ParamList.LoadParams(cfgFileName);

		pPSGM->pMem = pMem;

		pPSGM->vpObjectDetector = this;

		pPSGM->pSurfelDetector = pSurfelDetector;

		pPSGM->bTangentRFDescriptors = false;

		bOwnsSurfelDetectionTool = true;
		bOwnsPSGM = true;
	}

	pObjects = new SURFEL::ObjectGraph;

	pObjects->CreateParamList(pMem0);

	pObjects->ParamList.LoadParams(cfgFileName);

	if (pObjects->relationClassifier == RVLPCSEGMENT_OBJECT_RELATION_CLASSIFIER_SVM)
	{
		std::cout << "Initializing SVM Classifier!" << std::endl;
		pObjects->InitSVMClassifier(SVMClassifierParamsFileName);
	}

	pObjects->pMem = pMem;
	pObjects->pSurfels = pSurfels;
	pObjects->objectAggregationLevel2Criterion = OBJECT_DETECTION::Symmetry;
	pObjects->vpObjectAggregationLevel2CriterionData = this;

	pPSGM->pObjects = pObjects;

	VNClassifier *pVNClassifier;

	if (pObjects->objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_VN)
	{
		if (vpVNClassifier_)
		{
			vpVNClassifier = vpVNClassifier_;

			pVNClassifier = (VNClassifier *)vpVNClassifier;

			pSurfels = pVNClassifier->pSurfels;

			pSurfelDetector = pVNClassifier->pSurfelDetector;

			bOwnsSurfelDetectionTool = false;
			bOwnsVNClassifier = false;
		}
		else
		{
			pVNClassifier = new VNClassifier;

			vpVNClassifier = pVNClassifier;

			pVNClassifier->pMem0 = pMem0;
			pVNClassifier->pMem = pMem;

			pVNClassifier->resultsFolder = resultsFolder;

			bOwnsVNClassifier = true;
		}

		bOwnConvexConcaveClustering = false;

		pConvexClustering = &(pVNClassifier->convexClustering);
		pConcaveClustering = &(pVNClassifier->concaveClustering);

		pVNClassifier->camera = camera;
		pVNClassifier->pObjects = pObjects;
	}
	else
	{
		if (vpVNClassifier)
		{
			pVNClassifier = (VNClassifier *)vpVNClassifier;

			delete pVNClassifier;
		}

		vpVNClassifier = NULL;

		bOwnsVNClassifier = false;		

		if (objectAggregationLevel1Method == RVLOBJECTDETECTION_METHOD_CONVEX_AND_CONCAVE)
		{
			// Create pConvexClustering and pConcaveClustering.
		}
	}

	if (objectAggregationLevel1Method == RVLOBJECTDETECTION_METHOD_CONVEX_AND_CONCAVE)
	{
		if (pConvexClustering)
		{
			pConvexClustering->CreateParamList(pMem0);
			pConvexClustering->ParamList.LoadParams(cfgFileName);
		}

		if (pConcaveClustering)
		{
			pConcaveClustering->CreateParamList(pMem0);
			pConcaveClustering->ParamList.LoadParams(cfgFileName);
		}
	}

	if (pSurfels == NULL)
	{
		pSurfels = new SurfelGraph;

		pSurfels->pMem = pMem;

		pSurfels->CreateParamList(pMem0);

		pSurfels->ParamList.LoadParams(cfgFileName);

		pSurfelDetector = new PlanarSurfelDetector;

		pSurfelDetector->CreateParamList(pMem0);

		pSurfelDetector->ParamList.LoadParams(cfgFileName);

		bOwnsSurfelDetectionTool = true;
	}

	pPSGM->pSurfels = pSurfels;

	pPSGM->pSurfelDetector = pSurfelDetector;

	if (bOwnsVNClassifier)
	{
		pVNClassifier->pSurfels = pSurfels;

		pVNClassifier->pSurfelDetector = pSurfelDetector;

		pVNClassifier->Create(cfgFileName);
	}
}

void ObjectDetector::CreateParamList()
{
	ParamList.m_pMem = pMem0;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("ObjectDetector.objectAggregationLevel1.method", RVLPARAM_TYPE_ID, &objectAggregationLevel1Method);
	ParamList.AddID(pParamData, "FUZZY_WER", RVLOBJECTDETECTION_METHOD_FUZZY_WER);
	ParamList.AddID(pParamData, "CONVEX_AND_CONCAVE", RVLOBJECTDETECTION_METHOD_CONVEX_AND_CONCAVE);
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
	pParamData = ParamList.AddParam("ObjectDetector.connectedComponentMaxDist", RVLPARAM_TYPE_FLOAT, &connectedComponentMaxDist);
	pParamData = ParamList.AddParam("ObjectDetector.multilateralFilterIterations", RVLPARAM_TYPE_INT, &nMultilateralFilterIterations);
	pParamData = ParamList.AddParam("ObjectDetector.joinSmallObjectsToLargestNeighborSizeThr", RVLPARAM_TYPE_INT, &joinSmallObjectsToLargestNeighborSizeThr);
	pParamData = ParamList.AddParam("ObjectDetector.joinSmallObjectsToLargestNeighborDistThr", RVLPARAM_TYPE_FLOAT, &joinSmallObjectsToLargestNeighborDistThr);
	pParamData = ParamList.AddParam("ObjectDetector.appearanceCost", RVLPARAM_TYPE_FLOAT, &appearanceCost);
	pParamData = ParamList.AddParam("ObjectDetector.multilateralFilter", RVLPARAM_TYPE_BOOL, &bMultilateralFilter);
	pParamData = ParamList.AddParam("ObjectDetector.joinSmallObjectsToLargestNeighbor", RVLPARAM_TYPE_BOOL, &bJoinSmallObjectsToLargestNeighbor);
	pParamData = ParamList.AddParam("ObjectDetector.GroundTruthSegmentation", RVLPARAM_TYPE_BOOL, &bGroundTruthSegmentation);
	pParamData = ParamList.AddParam("ObjectDetector.TrainingHMI", RVLPARAM_TYPE_BOOL, &bTrainingHMI);
	pParamData = ParamList.AddParam("ObjectDetector.Display", RVLPARAM_TYPE_BOOL, &bDisplay);
	pParamData = ParamList.AddParam("ObjectDetector.hypothesisLog", RVLPARAM_TYPE_BOOL, &bHypothesisLog);
	pParamData = ParamList.AddParam("ObjectDetector.visualizeConvexClusters", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeConvexClusters));
	pParamData = ParamList.AddParam("ObjectDetector.visualizeConcaveClusters", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeConcaveClusters));
	pParamData = ParamList.AddParam("ObjectDetector.visualizeSurfels", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeSurfels));
	pParamData = ParamList.AddParam("ObjectDetector.visualizeAllClusters", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeAllClusters));
	pParamData = ParamList.AddParam("ObjectDetector.visulaizeConvexHulls", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeConvexHulls));
	pParamData = ParamList.AddParam("ObjectDetector.debug1", RVLPARAM_TYPE_INT, &debug1);
	pParamData = ParamList.AddParam("ObjectDetector.debug2", RVLPARAM_TYPE_INT, &debug2);
}

//Dirk Holz and Sven Behnke: "Approximate Triangulation and Region Growing for Efficient Segmentation and Smoothing of Range Images"
//NOT DEBUGGED
//NOT UPDATED
vtkSmartPointer<vtkPolyData> MultilateralSmoothMesh(vtkSmartPointer<vtkPolyData> inputPD, int noIter)
{

	vtkSmartPointer<vtkPolyData> outputPD = vtkSmartPointer<vtkPolyData>::New();
	outputPD->DeepCopy(inputPD);
	outputPD->BuildLinks();

	// get a copy of pPolygonData points(destination points for first iteration)
	vtkSmartPointer<vtkPoints> pointsSource = outputPD->GetPoints();
	vtkSmartPointer<vtkPoints> pointsDestination = vtkSmartPointer<vtkPoints>::New();
	pointsDestination->DeepCopy(outputPD->GetPoints());
	vtkSmartPointer<vtkPoints> pointsTemp;

	vtkSmartPointer<vtkFloatArray> normalsSource = vtkFloatArray::SafeDownCast(outputPD->GetPointData()->GetNormals());
	vtkSmartPointer<vtkFloatArray> normalsDestination = vtkSmartPointer<vtkFloatArray>::New();	//point normals copy
	normalsDestination->DeepCopy(vtkFloatArray::SafeDownCast(outputPD->GetPointData()->GetNormals()));
	vtkSmartPointer<vtkFloatArray> normalsTemp;

	int noPts = outputPD->GetNumberOfPoints();
	float sumW = 0.0;
	float sumPts[3];
	float sumNorm[3];
	int noCells = 0;
	vtkSmartPointer<vtkIdList> ptCells = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkIdList> ptCellPts = vtkSmartPointer<vtkIdList>::New();
	vtkSmartPointer<vtkPoints> pointsCpy = vtkSmartPointer<vtkPoints>::New();	//points copy where new coordinates will go

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
			pointsSource->GetPoint(i, currPt);
			if ((currPt[0] <= -1.0) && (currPt[1] <= -1.0) && (currPt[2] <= -1.0))
			{
				pointsCpy->SetPoint(i, -1.0, -1.0, -1.0);
				continue;
			}
			normalsSource->GetTupleValue(i, currNor);
			outputPD->GetPointCells(i, ptCells);
			sumPts[0] = 0.0; sumPts[1] = 0.0; sumPts[2] = 0.0;
			sumNorm[0] = 0.0; sumNorm[1] = 0.0; sumNorm[2] = 0.0;
			sumW = 0.0;
			noCells = ptCells->GetNumberOfIds();
			for (int j = 0; j < noCells; j++)
			{
				ptCellPts->Reset();
				outputPD->GetCellPoints(ptCells->GetId(j), ptCellPts);
				for (int k = 0; k < 3; k++)
				{
					ptID = ptCellPts->GetId(k);
					if (ptID != i)
					{
						pointsSource->GetPoint(ptID, pt1);
						normalsSource->GetTupleValue(ptID, tempNor);
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
				pointsDestination->SetPoint(i, currPt);
				normalsDestination->SetTuple(i, currNor);
			}
			else
			{
				newPt[0] = sumPts[0] / sumW;
				newPt[1] = sumPts[1] / sumW;
				newPt[2] = sumPts[2] / sumW;
				newNor[0] = sumNorm[0] / sumW;
				newNor[1] = sumNorm[1] / sumW;
				newNor[2] = sumNorm[2] / sumW;
				pointsDestination->SetPoint(i, newPt);
				normalsDestination->SetTuple(i, newNor);
			}
		}
		
		//Swap source and distination
		pointsTemp = pointsSource;
		pointsSource = pointsDestination;
		pointsDestination = pointsTemp;

		normalsTemp = normalsSource;
		normalsSource = normalsDestination;
		normalsDestination = normalsTemp;

		std::cout << "Iteration " << it << " finished!" << std::endl;
	}

	outputPD->SetPoints(pointsSource);
	//norPD->Print(cout);
	outputPD->GetPointData()->SetNormals(normalsSource);

	return outputPD;
}

//Dirk Holz and Sven Behnke: "Approximate Triangulation and Region Growing for Efficient Segmentation and Smoothing of Range Images"
//NOT DEBUGGED
void MultilateralSmoothMesh(Mesh *pMesh, int noIter, bool Boundary1DFiltering, bool onlyFirstNeigh_1DFiltering = false, bool verbose = false)
{
	//get a copy of pPolygonData points (destination points for first iteration)
	vtkSmartPointer<vtkPoints> pointsSource = pMesh->pPolygonData->GetPoints();
	vtkSmartPointer<vtkPoints> pointsDestination = vtkSmartPointer<vtkPoints>::New();
	pointsDestination->DeepCopy(pMesh->pPolygonData->GetPoints());
	vtkSmartPointer<vtkPoints> pointsTemp;

	vtkSmartPointer<vtkFloatArray> normalsSource = vtkFloatArray::SafeDownCast(pMesh->pPolygonData->GetPointData()->GetNormals());
	vtkSmartPointer<vtkFloatArray> normalsDestination = vtkSmartPointer<vtkFloatArray>::New();	//point normals copy
	normalsDestination->DeepCopy(vtkFloatArray::SafeDownCast(pMesh->pPolygonData->GetPointData()->GetNormals()));
	vtkSmartPointer<vtkFloatArray> normalsTemp;

	Point* currPoint;
	double currPointD[3];
	Point* currEdgePoint;
	double currEdgePointD[3];
	Point* nextEdgePoint;
	double nextEdgePointD[3];
	MeshEdge* pEdge;
	MeshEdgePtr* pEdgePtr;
	MeshEdge* pEdge2;
	MeshEdgePtr* pEdgePtr2;
	int othersideIdx;
	int otherothersideIdx;
	int noPointEdges = 0;
	int neighbourPoints[20]; //assumption: there is maximum 20 edges for any point
	bool boundaryEdges[20];

	float tempDist2 = 0.0;
	float tempL1 = 0.0;
	float tempW = 0.0;
	float currNor[3];
	float otherNor[3];
	float newPt[3];
	float newNor[3];
	double currPt[3];
	double otherPt[3];
	float sumW = 0.0;
	float sumPts[3];
	float sumNorm[3];
	bool first = true;
	bool found = false;
	for (int iter = 0; iter < noIter; iter++)
	{
		for (int idx = 0; idx < pMesh->NodeArray.n; idx++)
		{
			//current point and edge list
			currPoint = pMesh->NodeArray.Element + idx;
			if (!currPoint->bValid)
				continue;
			pointsSource->GetPoint(idx, currPointD);
			normalsSource->GetTupleValue(idx, currNor);
			pEdgePtr = currPoint->EdgeList.pFirst;
			noPointEdges = 0;
			if (Boundary1DFiltering && currPoint->bBoundary)//If the point is on the boundary and we use 1D boundary filtering
			{
			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(idx, pEdgePtr, pEdge, othersideIdx);	//get the other side index and point
				nextEdgePoint = pMesh->NodeArray.Element + othersideIdx;
					if ((noPointEdges == 0) || (pEdgePtr->pNext == NULL))	//boundary is only first and last pointer?
					{
						neighbourPoints[noPointEdges] = othersideIdx;
						boundaryEdges[noPointEdges] = true;
						noPointEdges++;
						if (!onlyFirstNeigh_1DFiltering)	//if we use first and second neighbours
						{
							//find neighbours boundary edges
							pEdgePtr2 = nextEdgePoint->EdgeList.pFirst;
							first = true;
							while (pEdgePtr2)
							{
								RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(othersideIdx, pEdgePtr2, pEdge2, otherothersideIdx);	//get the other side index
								if ((first || (pEdgePtr2->pNext == NULL)) && (otherothersideIdx != idx)) //boundary is only first and last pointer? //Also it must not be the poiter to current point
								{
									//Check if it is not already on the list
									found = false;
									for (int i = 0; i < noPointEdges; i++)
									{
										if (neighbourPoints[i] == otherothersideIdx)	//If it is on the list
										{
											found = true;
											break;
										}
									}
									if (!found)	//If it is not already in, add it
									{
										neighbourPoints[noPointEdges] = otherothersideIdx;
										boundaryEdges[noPointEdges] = true;
										noPointEdges++;
									}
								}
								first = false;	//It is no longer the first pointer
								pEdgePtr2 = pEdgePtr2->pNext;
							}
						}
					}

					pEdgePtr = pEdgePtr->pNext;
				}
			}
			else   //Standard multilateral filter takes all neighbours into considiration regardless of boundary
			{
			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(idx, pEdgePtr, pEdge, othersideIdx);	//get the other side index and point
				nextEdgePoint = pMesh->NodeArray.Element + othersideIdx;
				neighbourPoints[noPointEdges] = othersideIdx;	//set other side index as in the neighbourhood
					if ((noPointEdges == 0) || (pEdgePtr->pNext == NULL))	//Check if that edge is boundary //first and last are boundary edges
					boundaryEdges[noPointEdges] = true;
				else
					boundaryEdges[noPointEdges] = false;

				noPointEdges++;
				pEdgePtr = pEdgePtr->pNext;
			}
			}

			sumPts[0] = 0.0; sumPts[1] = 0.0; sumPts[2] = 0.0;
			sumNorm[0] = 0.0; sumNorm[1] = 0.0; sumNorm[2] = 0.0;
			sumW = 0.0;

			if (Boundary1DFiltering && onlyFirstNeigh_1DFiltering && currPoint->bBoundary)	//If we take only first neighbours into considiration for 1D boundary filtering we also use the current (central point)
			{
				sumW += 1.0;
				sumPts[0] += currPointD[0];
				sumPts[1] += currPointD[1];
				sumPts[2] += currPointD[2];
				sumNorm[0] += currNor[0];
				sumNorm[1] += currNor[1];
				sumNorm[2] += currNor[2];
			}

			for (int id_curr = 0; id_curr < noPointEdges; id_curr++)	//for each neighbour on the list
			{
				/*if (boundaryEdgeFiltering && currPoint->bBoundary && !boundaryEdges[id_curr])
					continue;*/
				pointsSource->GetPoint(neighbourPoints[id_curr], otherPt);
				normalsSource->GetTupleValue(neighbourPoints[id_curr], otherNor);
				//calculate distance
				tempDist2 = sqrt(vtkMath::Distance2BetweenPoints(currPointD, otherPt));
				//calculate L1 normals norm
				tempL1 = abs(currNor[0] - otherNor[0]) + abs(currNor[1] - otherNor[1]) + abs(currNor[2] - otherNor[2]);
				//calculate weight
				tempW = exp(tempDist2) * exp(tempL1);
				//calculate sum weight and sum point coordinates and sum normals
				sumW += tempW;
				sumPts[0] += tempW * otherPt[0];
				sumPts[1] += tempW * otherPt[1];
				sumPts[2] += tempW * otherPt[2];
				sumNorm[0] += tempW * otherNor[0];
				sumNorm[1] += tempW * otherNor[1];
				sumNorm[2] += tempW * otherNor[2];
			}
			
			//calculate new point coordinates and new normals
			if (sumW == 0.0)
			{
				pointsDestination->SetPoint(idx, currPt);
				normalsDestination->SetTuple(idx, currNor);
			}
			else
			{
				newPt[0] = sumPts[0] / sumW;
				newPt[1] = sumPts[1] / sumW;
				newPt[2] = sumPts[2] / sumW;
				newNor[0] = sumNorm[0] / sumW;
				newNor[1] = sumNorm[1] / sumW;
				newNor[2] = sumNorm[2] / sumW;
				pointsDestination->SetPoint(idx, newPt);
				normalsDestination->SetTuple(idx, newNor);
			}

		}
		//Swap source and distination
		pointsTemp = pointsSource;
		pointsSource = pointsDestination;
		pointsDestination = pointsTemp;

		normalsTemp = normalsSource;
		normalsSource = normalsDestination;
		normalsDestination = normalsTemp;
		if (verbose)
			std::cout << "Multilateral smoothing: Finished " << iter << " iteration!" << std::endl;
	}

	//setting final points (? is this needed ?)
	pMesh->pPolygonData->SetPoints(pointsSource);
	pMesh->pPolygonData->GetPointData()->SetNormals(normalsSource);

	//Updating mesh point and normal data;
	if (verbose)
		std::cout << "Updating mesh data!" << std::endl;
	for (int idx = 0; idx < pMesh->NodeArray.n; idx++)
	{
		currPoint = pMesh->NodeArray.Element + idx;
		pointsSource->GetPoint(idx, currPointD);
		normalsSource->GetTupleValue(idx, currPoint->N);
		currPoint->P[0] = currPointD[0];
		currPoint->P[1] = currPointD[1];
		currPoint->P[2] = currPointD[2];
	}
	if (verbose)
		std::cout << "Mesh data updated!" << std::endl;
}

void LaplaceSmooting(Mesh *pMesh, int noIter, bool useCotan)
{
	//get a copy of pPolygonData points (destination points for first iteration)
	vtkSmartPointer<vtkPoints> pointsSource = pMesh->pPolygonData->GetPoints();
	vtkSmartPointer<vtkPoints> pointsDestination = vtkSmartPointer<vtkPoints>::New();
	pointsDestination->DeepCopy(pMesh->pPolygonData->GetPoints());
	vtkSmartPointer<vtkPoints> pointsTemp;
	//Calculate cotangent weights for each point to point edge
	/*std::vector<std::vector<float>> neighboorhoodCoTangentW;
	neighboorhoodCoTangentW.resize(pMesh->NodeArray.n);*/
	//std::vector<std::vector<bool>> neighboorhoodBoundary;
	//neighboorhoodBoundary.resize(pMesh->NodeArray.n);

	Point* currPoint;
	double currPointD[3];
	Point* currEdgePoint;
	double currEdgePointD[3];
	Point* nextEdgePoint;
	double nextEdgePointD[3];
	Point* prevEdgePoint;
	double prevEdgePointD[3];
	double destCurrPointD[3];
	MeshEdge* pEdge;
	MeshEdgePtr* pEdgePtr;
	int othersideIdx;
	int noPointEdges = 0;
	int neighbourPoints[20]; //assumption: there is maximum 20 edges for any point
	bool boundaryEdges[20];
	float neighboorhoodCoTangentW[20];
	//Running for all points
	float v1[3], v2[3], v3[3], v4[3];
	int id_curr, id_prev, id_next;
	float cotan1 = 0.0f;
	float cotan2 = 0.0f;
	float v1v2Dot;
	float v1v2Cross[3];
	float v3v4Dot;
	float v3v4Cross[3];
	float cog[3];
	float sum;
	float norm;
	for (int iter = 0; iter < noIter; iter++)
	{
		for (int idx = 0; idx < pMesh->NodeArray.n; idx++)
		{
			//current point and edge list
			currPoint = pMesh->NodeArray.Element + idx;
			/*if ((currPoint->P[0] == 0.0) && (currPoint->P[1] == 0.0) && (currPoint->P[2] == 0.0))
				continue;*/
			if (!currPoint->bValid)
				continue;
			pointsSource->GetPoint(idx, currPointD);
			pEdgePtr = currPoint->EdgeList.pFirst;
			noPointEdges = 0;
			while (pEdgePtr)
			{
				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(idx, pEdgePtr, pEdge, othersideIdx);	//get the other side index and point
				nextEdgePoint = pMesh->NodeArray.Element + othersideIdx;
				neighbourPoints[noPointEdges] = othersideIdx;	//set other side index as in the neighbourhood
				if ((noPointEdges == 0) || (pEdgePtr->pNext == NULL))	//Check if that edge is boundary
					boundaryEdges[noPointEdges] = true;
				else
					boundaryEdges[noPointEdges] = false;

				noPointEdges++;
				pEdgePtr = pEdgePtr->pNext;
			}

			if (useCotan)
			{
			//calculating cotangent weight
			for (id_curr = 0; id_curr < noPointEdges; id_curr++)
			{
				id_next = (id_curr + 1) >= noPointEdges ? 0 : id_curr + 1;
				id_prev = (id_curr - 1) >= 0 ? (id_curr - 1) : noPointEdges - 1;

				/*currEdgePoint = pMesh->NodeArray.Element + neighbourPoints[id_curr];
				nextEdgePoint = pMesh->NodeArray.Element + neighbourPoints[id_next];
				prevEdgePoint = pMesh->NodeArray.Element + neighbourPoints[id_prev];
				v1[0] = currPoint->P[0] - prevEdgePoint->P[0];
				v1[1] = currPoint->P[1] - prevEdgePoint->P[1];
				v1[2] = currPoint->P[2] - prevEdgePoint->P[2];
				v2[0] = currEdgePoint->P[0] - prevEdgePoint->P[0];
				v2[1] = currEdgePoint->P[1] - prevEdgePoint->P[1];
				v2[2] = currEdgePoint->P[2] - prevEdgePoint->P[2];
				v3[0] = currPoint->P[0] - nextEdgePoint->P[0];
				v3[1] = currPoint->P[1] - nextEdgePoint->P[1];
				v3[2] = currPoint->P[2] - nextEdgePoint->P[2];
				v4[0] = currEdgePoint->P[0] - nextEdgePoint->P[0];
				v4[1] = currEdgePoint->P[1] - nextEdgePoint->P[1];
				v4[2] = currEdgePoint->P[2] - nextEdgePoint->P[2];*/
				pointsSource->GetPoint(neighbourPoints[id_curr], currEdgePointD);
				pointsSource->GetPoint(neighbourPoints[id_next], nextEdgePointD);
				pointsSource->GetPoint(neighbourPoints[id_prev], prevEdgePointD);
				v1[0] = currPointD[0] - prevEdgePointD[0];
				v1[1] = currPointD[1] - prevEdgePointD[1];
				v1[2] = currPointD[2] - prevEdgePointD[2];
				v2[0] = currEdgePointD[0] - prevEdgePointD[0];
				v2[1] = currEdgePointD[1] - prevEdgePointD[1];
				v2[2] = currEdgePointD[2] - prevEdgePointD[2];
				v3[0] = currPointD[0] - nextEdgePointD[0];
				v3[1] = currPointD[1] - nextEdgePointD[1];
				v3[2] = currPointD[2] - nextEdgePointD[2];
				v4[0] = currEdgePointD[0] - nextEdgePointD[0];
				v4[1] = currEdgePointD[1] - nextEdgePointD[1];
				v4[2] = currEdgePointD[2] - nextEdgePointD[2];

				/*const Vec3 v1 = c_pos - geom.vertex(id_prev);
				const Vec3 v2 = geom.vertex(id_curr) - geom.vertex(id_prev);
				const Vec3 v3 = c_pos - geom.vertex(id_next);
				const Vec3 v4 = geom.vertex(id_curr) - geom.vertex(id_next);*/

				// wij = (cot(alpha) + cot(beta)),
				// for boundary edge, there is only one such edge
				// If the mesh is not a water-tight closed volume
				// we must check for edges lying on the sides of wholes
				cotan1 = 0.0;
				cotan2 = 0.0;
				if (!boundaryEdges[id_curr])
				{
					// general case: not a boundary
					v1v2Dot = RVLDOTPRODUCT3(v1, v2);
					RVLCROSSPRODUCT3(v1, v2, v1v2Cross);
					norm = RVLDOTPRODUCT3(v1v2Cross, v1v2Cross);
					cotan1 = v1v2Dot / sqrt(norm);
					v3v4Dot = RVLDOTPRODUCT3(v3, v4);
					RVLCROSSPRODUCT3(v3, v4, v3v4Cross);
					norm = RVLDOTPRODUCT3(v3v4Cross, v3v4Cross);
					cotan2 = v3v4Dot / sqrt(norm);
					/*cotan1 = (v1.dot(v2)) / (v1.cross(v2)).norm();
					cotan2 = (v3.dot(v4)) / (v3.cross(v4)).norm();*/
				}
				else // boundary edge, only have one such angle
				{
					if (id_next == id_prev)
					{
						// two angles are the same, e.g. corner of a square
						v1v2Dot = RVLDOTPRODUCT3(v1, v2);
						RVLCROSSPRODUCT3(v1, v2, v1v2Cross);
						norm = RVLDOTPRODUCT3(v1v2Cross, v1v2Cross);
						cotan1 = v1v2Dot / sqrt(norm);
						//cotan1 = (v1.dot(v2)) / (v1.cross(v2)).norm();
					}
					else
					{
						// find the angle not on the boundary
						if (!boundaryEdges[id_next])
						{
							v3v4Dot = RVLDOTPRODUCT3(v3, v4);
							RVLCROSSPRODUCT3(v3, v4, v3v4Cross);
							norm = RVLDOTPRODUCT3(v3v4Cross, v3v4Cross);
							cotan2 = v3v4Dot / sqrt(norm);
							//cotan2 = (v3.dot(v4)) / (v3.cross(v4)).norm();
						}
						else
						{
							v1v2Dot = RVLDOTPRODUCT3(v1, v2);
							RVLCROSSPRODUCT3(v1, v2, v1v2Cross);
							norm = RVLDOTPRODUCT3(v1v2Cross, v1v2Cross);
							cotan1 = v1v2Dot / sqrt(norm);
							//cotan1 = (v1.dot(v2)) / (v1.cross(v2)).norm();
						}
					}
				}

				//neighboorhoodCoTangentW.at(i).push_back(cotan1 + cotan2);
				neighboorhoodCoTangentW[id_curr] = cotan1 + cotan2;
			}
			}
			else
			{
				for (id_curr = 0; id_curr < noPointEdges; id_curr++)
					neighboorhoodCoTangentW[id_curr] = 1.0;
			}

			//Calculating new point position
			cog[0] = 0.0;
			cog[1] = 0.0;
			cog[2] = 0.0;
			sum = 0.0;
			for (int id_curr = 0; id_curr < noPointEdges; id_curr++)
			{
				//float w = _cotan_weights[i][n];
				//cog += src_vertices[neigh] * w;
				//sum += w;
				pointsSource->GetPoint(neighbourPoints[id_curr], currEdgePointD);
				cog[0] += currEdgePointD[0] * neighboorhoodCoTangentW[id_curr];
				cog[1] += currEdgePointD[1] * neighboorhoodCoTangentW[id_curr];
				cog[2] += currEdgePointD[2] * neighboorhoodCoTangentW[id_curr];
				sum += neighboorhoodCoTangentW[id_curr];
			}
			//float t = smooth_factors[i];
			//dst_vertices[i] = (cog / sum) * t + src_vertices[i] * (1.f - t);
			destCurrPointD[0] = (cog[0] / sum) * 1 + currPointD[0] * 1;
			destCurrPointD[1] = (cog[1] / sum) * 1 + currPointD[1] * 1;
			destCurrPointD[2] = (cog[2] / sum) * 1 + currPointD[2] * 1;
			pointsDestination->SetPoint(idx, destCurrPointD);
		}
		//Swap source and distination
		pointsTemp = pointsSource;
		pointsSource = pointsDestination;
		pointsDestination = pointsTemp;
		std::cout << "Laplace smoothing: Finished " << iter << " iteration!" << std::endl;
	}

	//setting final points (? is this needed ?)
	pMesh->pPolygonData->SetPoints(pointsSource);

	//Recalculating normals
	vtkSmartPointer<vtkPolyDataNormals> normalsFilter = vtkSmartPointer<vtkPolyDataNormals>::New();
	normalsFilter->SetInputData(pMesh->pPolygonData);
	normalsFilter->ComputeCellNormalsOff();
	normalsFilter->ComputePointNormalsOn();
	normalsFilter->SplittingOff();
	normalsFilter->Update();

	//Updating mesh point and normal data;
	pMesh->pPolygonData = normalsFilter->GetOutput();
	vtkSmartPointer<vtkPoints> pdPoints = normalsFilter->GetOutput()->GetPoints();
	vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(normalsFilter->GetOutput()->GetPointData()->GetNormals());
	int noPoints = pdPoints->GetNumberOfPoints();
	//float normal[3];
	for (int i = 0; i < noPoints; i++)
	{
		currPoint = pMesh->NodeArray.Element + i;
		pdPoints->GetPoint(i, currPointD);
		normals->GetTupleValue(i, currPoint->N);
		currPoint->P[0] = currPointD[0];
		currPoint->P[1] = currPointD[1];
		currPoint->P[2] = currPointD[2];
	}

	//// Initialize VTK.
	//vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();;
	//vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	//vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	//window->AddRenderer(renderer);
	//window->SetSize(800, 600);
	//interactor->SetRenderWindow(window);
	//vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	//interactor->SetInteractorStyle(style);
	//renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//mapper->SetInputData(pMesh->pPolygonData);
	//vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
	//act->SetMapper(mapper);
	//renderer->AddActor(act);

	////Start VTK
	//renderer->ResetCamera();
	//window->Render();
	//interactor->Start();
}

void ObjectDetector::DetectObjects(
	char *MeshFilePathName,
	Array2D<short int> *pDepthImage,
	IplImage *pRGBImage)
{
	// Segmentation to surfels.

	bSurfelsFromSSF = false;

	char *fileExtension = (MeshFilePathName ? RVLGETFILEEXTENSION(MeshFilePathName) : (char *)"");

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
		if (MeshFilePathName)
		{
			// Read mesh from file.

			printf("Creating mesh from %s:\n", MeshFilePathName);

			if (LoadMesh(vpMeshBuilder, MeshFilePathName, &mesh, (flags & RVLOBJECTDETECTION_FLAG_SAVE_PLY) != 0, NULL, NULL))
				printf("Mesh created.\n");
			else
				printf("ERROR: Mesh can't be created!\n");
		}
		else if (pDepthImage)
		{
			printf("Creating mesh from depth image.\n");

			CreateMesh(vpMeshBuilder, pDepthImage, pRGBImage, &mesh, ((flags & RVLOBJECTDETECTION_FLAG_SAVE_PLY) != 0), "scene.ply");

			printf("Mesh created.\n");
		}
		else
			printf("ERROR: No mesh or depth image specified!\n");

		//SmoothMesh(&mesh, 30);
		//LaplaceSmooting(&mesh, 30);
		if (bMultilateralFilter)
			MultilateralSmoothMesh(&mesh, nMultilateralFilterIterations, false, false, true);		

		// Segment mesh to surfels.				

		pSurfels->Init(&mesh);

		pSurfelDetector->Init(&mesh, pSurfels, pMem);

		printf("Segmentation to surfels... ");

		double StartTime, ExecTime;

		if (pSurfelDetector->pTimer)
			StartTime = pSurfelDetector->pTimer->GetTime();

		pSurfelDetector->Segment(&mesh, pSurfels);

		if (pSurfelDetector->pTimer)
			ExecTime = pSurfelDetector->pTimer->GetTime() - StartTime;

		printf("completed.\n");
		printf("No. of surfels = %d\n", pSurfels->NodeArray.n);

		if (pSurfelDetector->pTimer)
			printf("Total segmentation time = %lf s\n", ExecTime);

#ifdef RVLSURFEL_GT_OBJECT_HISTOGRAM
		if (flags & RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT)
			pSurfels->AssignGroundTruthSegmentation(MeshFilePathName, pSurfelDetector->minSurfelSize);
#endif

		// Group surfels into objects.

		if (bSegmentToObjects || (flags & RVLOBJECTDETECTION_FLAG_SAVE_SSF))
		{
			printf("Computing relations between adjacent surfels...");

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			pSurfels->SurfelRelations(&mesh);
#endif

#ifdef RVLSURFEL_GT_OBJECT_HISTOGRAM
			if (bGroundTruthSegmentation && bGroundTruthSegmentationOnSurfelLevel)
				pObjects->CreateFromGroundTruth(pSurfels);
			else
#endif
			{
				pObjects->Create(pSurfels);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
				pObjects->ComputeRelationCosts();
#endif
			}
			
			printf("completed.\n");

			//pObjects->Debug();

			// Detect vertices.

			pSurfels->DetectVertices(&mesh);

			pSurfels->DetectOcclusionVertices(&mesh, camera);
		}

#ifdef RVLSURFEL_IMAGE_ADJACENCY
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

	if (bSegmentToObjects)
	{
		if (bGroundTruthSegmentation && bGroundTruthSegmentationOnSurfelLevel)
		{			
			pObjects->sortedObjectArray.n = -1;
			pObjects->nValidObjects = -1;
			pObjects->GetVertices();
			pPSGM->Init(&mesh);
			GroundTruthGroundPlane();
			if (bGroundTruthBoundingBoxes)
			{
				pPSGM->convexTemplate = pPSGM->convexTemplateBox;
				pPSGM->CTIs(-1, pObjects, &boundingBoxes, pMem);
				SaveBoundingBoxSizes(MeshFilePathName);
			}
		}
		else
		{
			printf("Aggregating surfels into objects... ");

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectAggregationLevel1Method == RVLOBJECTDETECTION_METHOD_FUZZY_WER)
				pObjects->WERSegmentation();
			else 
#endif
				if (objectAggregationLevel1Method == RVLOBJECTDETECTION_METHOD_CONVEX_AND_CONCAVE)
			{
				// Cluster surfels into convex and concave surfaces.

				RVL_DELETE_ARRAY(SClusters.Element);

				RVL_DELETE_ARRAY(clusterMap);

				clusterMap = new int[pSurfels->NodeArray.n];

				int nSCClusters, nSUClusters;

				RECOG::PSGM_::ConvexAndConcaveClusters(&mesh, pSurfels, pSurfelDetector, pConvexClustering, pConcaveClustering,
					SClusters, nSCClusters, nSUClusters, pObjects->minObjectSize, clusterMap, -1, -1, true, 0,
					visualizationData.bVisualizeConvexClusters, visualizationData.bVisualizeConcaveClusters, 
					visualizationData.bVisualizeSurfels, visualizationData.bVisualizeAllClusters);

				RECOG::CreateObjectGraphFromSceneClusters(SClusters, pObjects);

				//printf("No. of clusters = %d\n", SClusters.n);

				//for (int i = 0; i < SClusters.n; i++)
				//{
				//	RECOG::PSGM_::Cluster *pCluster = (RECOG::PSGM_::Cluster *)(SClusters.Element[i].vpCluster);
				//	
				//	printf("Cluster %d: size=%d orig=%d\n", i, pCluster->size, pCluster->orig);
				//}
			}

			pObjects->GetVertices();

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
				if (bDisplay)
				{
					cv::imshow("Level1", pObjects->CreateSegmentationImage());
					cv::waitKey(1);
				}
				/*VisualizeObjectGraphVertexPointCloud(&objects, 100);*/
				//if (bCTIBasedObjectAggregation)
				pPSGM->Init(&mesh);
				//pPSGM->CTIs(pObjects, &CTIs);
				pPSGM->convexTemplate = pPSGM->convexTemplateBox;
				pPSGM->CTIs(-1, pObjects, &boundingBoxes, pMem);
				//SaveBoundingBoxSizes(MeshFilePathName);
				//pPSGM->convexTemplate = pPSGM->convexTemplate66;
				if (pObjects->objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_CONVEXITY)
				{
					pObjects->pMesh = &mesh;
					pObjects->DetermineObjectConvexityData(convexityThr, 0.15, false);
					pObjects->vpObjectAggregationLevel2CriterionData = this;
					pObjects->ExtFuncCheckIfWithinVolume = &RVL::ObjectDetector::CheckIfWithinCTIBoundingBox;
					pObjects->ObjectAggregationLevel2_ViaObjectPairConvexity(convexityThr, convexityRatioThr1, convexityRatioThr2, pObjects->minObjectSize, false);
				}
				else if (pObjects->objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_VN)
					//ObjectAggregationLevel2VN3();
					ObjectAggregationLevel2VN4(3);
				if (bDisplay && !bJoinSmallObjectsToLargestNeighbor)
					cv::imshow("Level2", pObjects->CreateSegmentationImage());

				////
				//Evaluation
				/*int E[2];
				int N = 0;
				objects.CalculateOverAndUnderSegmentation(E, N, false, "", false);
				std::cout << "Oversegmenation error: " << 100.0f * (1 - E[0] / (float)N) << "%" << std::endl;
				std::cout << "Undersegmenation error: " << 100.0f * E[1] / (float)N << "%" << std::endl;*/

				printf("completed.\n");
			}	// if (!bSurfelsFromSSF && bObjectAggregationLevel2)

			if (bJoinSmallObjectsToLargestNeighbor)
			{
				int mergedIt = 0;
				while (pObjects->MergeSmallObjects(joinSmallObjectsToLargestNeighborSizeThr, joinSmallObjectsToLargestNeighborDistThr))
				{
					//std::cout << "Merged iteration: " << mergedIt << std::endl;
					mergedIt++;
				}
				if (bDisplay)
					cv::imshow("level2 + merge small objects", pObjects->CreateSegmentationImage());
				//cv::waitKey(1);
			}

#ifdef RVLSURFEL_GT_OBJECT_HISTOGRAM
			if (bGroundTruthSegmentation && !bGroundTruthSegmentationOnSurfelLevel)
			{
				pObjects->GroupAccordingToGroundTruth();

				GroundTruthGroundPlane();

				pObjects->sortedObjectArray.n = -1;
				pObjects->nValidObjects = -1;

				pObjects->GetVertices();
			}
#endif
		}	// if (!bGroundTruthSegmentation)
	}
}

//Hypothesis sort function
bool HypSortFunc(void* first, void* second) 
{ 
	return (((RECOG::VN_::Hypothesis*)first)->cost < ((RECOG::VN_::Hypothesis*)second)->cost);
}

//
bool LatSorrFunc(void* first, void* second)
{
	return (std::get<0>(*((std::tuple<float, float*, int, int, bool>*)first)) < std::get<0>(*((std::tuple<float, float*, int, int, bool>*)second)));
}

void ObjectDetector::ObjectAggregationLevel2VN2()
{
	//Disabled - 05.09.2018. Filko
	//VNClassifier *pVNClassifier = (VNClassifier *)vpVNClassifier;

	//pVNClassifier->descriptorMem.Clear();

	//int iClass = pPSGM->debug1;

	//RECOG::ClassData *pClass = pVNClassifier->classArray.Element + iClass;

	//// Prepare data for scene fitting.

	//Camera camera;

	//camera.fu = 525;
	//camera.fv = 525;
	//camera.uc = 320;
	//camera.vc = 240;
	//camera.w = mesh.width;
	//camera.h = mesh.height;

	//SURFEL::SceneSamples sceneSamples;

	//sceneSamples.PGnd = NULL;

	//Array<OrientedPoint> PtArray;

	//PtArray.Element = new OrientedPoint[mesh.NodeArray.n];

	//Array<Point> PtArray_;

	//PtArray_.Element = new Point[mesh.NodeArray.n];

	//RECOG::VN_::FitData fitData;

	//RECOG::VN_::Init(fitData, pClass->M, pVNClassifier->fitParams.beta, pVNClassifier->fitParams.lambda, false, 0);

	//RECOG::VN_::FitData fitDataRot;

	//RECOG::VN_::Init(fitDataRot, pClass->M, pVNClassifier->fitParams.beta, pVNClassifier->fitParams.lambda, true, pSurfels->vertexArray.n);

	//// Generating a list of valid objects and creating VN descriptors.

	//Array<int> iSurfelArray;

	//iSurfelArray.Element = new int[pSurfels->NodeArray.n];

	//FILE *fp = fopen((std::string(resultsFolder) + "\\scenedescriptors.txt").data(), "w");

	//FILE *fpInstances = fopen((std::string(resultsFolder) + "\\instances.txt").data(), "w");

	//float *q = new float[pClass->M.h];

	//float *d = new float[pClass->M.w];

	//float t[3];

	//RVLNULL3VECTOR(t);

	//hypotheses.clear();

	//int i, j;
	//GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	//QLIST::Index *piElement;
	//std::vector<int> validObjects;
	////std::vector<std::pair<float*, uchar*>> validObjects_BDs; //For conviniance
	//int iObject, iObject_;
	//SURFEL::Object *pObject_;
	//float E;
	//int iR;
	//bool bAllDoFsVisible;
	//int nb;
	//float *a;
	//RECOG::VN_::Hypothesis *pHypothesis;
	//int surfelListSize;
	//float EHull;
	//VN *pModel;
	//bool bGnd;
	//float *R_;
	//float zMin;
	//int nOutliers;
	//float sceneSupport, score;

	/////
	//int maxNoHypPerObj = 50;
	////preallocate buffer
	//std::vector<std::tuple<float, float*, int, int, bool>> allbuffer(1056, std::tuple<float, float*, int, int, bool>(-1.0, NULL, 0, 0, false));
	//for (int i = 0; i < 1056; i++)
	//	std::get<1>(allbuffer[i]) = new float[pClass->M.h];
	/////

	//for (iObject = 0; iObject < pObjects->NodeArray.n; iObject++)
	//{
	//	pObject = pObjects->NodeArray.Element + iObject;

	//	if (pObject->size < pObjects->minObjectSize)
	//		continue;

	//	piElement = pObject->elementList.pFirst;

	//	//check if object

	//	if (!piElement)
	//		continue;
	//	
	//	// Add object to the list of valid objects.

	//	validObjects.push_back(iObject);

	//	// Compute block descriptor.

	//	iObject_ = pObjects->iObjectAssignedToNode[iObject];

	//	pObject_ = pObjects->objectArray.Element + iObject_;

	//	QLIST::CopyToArray(&(pObject_->surfelList), &iSurfelArray);

	//	pVNClassifier->ComputeDescriptors(iSurfelArray, pObject_->iVertexArray, &(pObject_->d), &(pObject_->bd));

	//	// Project block descriptor to the latent subspace.
	//	pVNClassifier->ProjectToLatentSubspace(pObject_->d, pObject_->bd, pClass, &fitData, &allbuffer);
	//	//sort
	//	std::sort(allbuffer.begin(), allbuffer.end(), LatSorrFunc);

	//	/// Create hypothesis for first N ?
	//	for (int i = 0; i < maxNoHypPerObj; i++)
	//	{
	//		E = std::get<0>(allbuffer[i]);
	//		q = std::get<1>(allbuffer[i]);
	//		iR = std::get<2>(allbuffer[i]);
	//		nb = std::get<3>(allbuffer[i]);
	//		bAllDoFsVisible = std::get<4>(allbuffer[i]);

	//		if (bAllDoFsVisible && nb >= 16)
	//		{
	//			RVLMEM_ALLOC_STRUCT(pMem, RECOG::VN_::Hypothesis, pHypothesis);

	//			pHypothesis->d = pObject_->d;
	//			pHypothesis->bd = pObject_->bd;
	//			pHypothesis->cost = E;
	//			pHypothesis->iClass = iClass;
	//			pHypothesis->estStage = 1;	//estimation stage
	//			pHypothesis->objects.push_back(iObject); //Add object to hypothesis
	//			// Copy latent vector.
	//			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pClass->M.h, pHypothesis->q);
	//			memcpy(pHypothesis->q, q, pClass->M.h * sizeof(float));

	//			surfelListSize = QLIST::Size<QLIST::Index>(pObject_->surfelList);

	//			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, surfelListSize, pHypothesis->iSurfelArray.Element);

	//			QLIST::CopyToArray(&(pObject_->surfelList), &(pHypothesis->iSurfelArray));

	//			pHypothesis->iVertexArray = pObject_->iVertexArray;

	//			R_ = pVNClassifier->SO3Samples.Element + 9 * iR;

	//			RVLCOPYMX3X3(R_, pHypothesis->R);	// This line is not needed if fitting of the class model to scene samples is performed.				

	//			// Add hypothesis to the hypothesis list.

	//			hypotheses.push_back(pHypothesis);

	//			// Only for debugging purpose!

	//			pVNClassifier->SaveLatentVector(fpInstances, q, 0);
	//		}
	//	}
	//}

	//fclose(fp);
	//fclose(fpInstances);

	////
	//std::vector<unsigned long long> rels;
	//unsigned long long currRel;
	//bool found;
	//int noIterations = 100;
	//float k1Stage = 0.5;
	//float k2Stage = 0.5;
	//for (int iter = 0; iter < noIterations; iter++)
	//{
	//	///Sort hypothesis
	//	std::sort(hypotheses.begin(), hypotheses.end(), HypSortFunc);

	//	//Get lowest cost hypothesis
	//	pHypothesis = (RECOG::VN_::Hypothesis*)hypotheses[0];
	//
	//	//How many estimation stages are there?????????
	//	if (pHypothesis->estStage < 2)
	//	{
	//		//apply evaluation
	//		// Fit class model to scene samples.
	//		RVLCOPYMX3X3(pHypothesis->R, R_);

	//		pSurfels->SampleSurfelSet(&mesh, pHypothesis->iSurfelArray, pHypothesis->iVertexArray, camera, sceneSamples);

	//		PtArray.n = sceneSamples.imagePtArray.h;

	//		bGnd = pVNClassifier->fitParams.bGnd;

	//		pVNClassifier->fitParams.bInit = false;
	//		pVNClassifier->fitParams.bGnd = pVNClassifier->fitParams.bGnd2;

	//		pModel = pVNClassifier->models[pClass->iMetaModel];

	//		if (pPSGM->debug2 == 0)
	//			pClass->nHull = 0;

	//		pModel->FitRotLMCC(pSurfels, pHypothesis->iVertexArray, sceneSamples, pClass, NULL, NULL, R_, pVNClassifier->fitParams,
	//			pHypothesis->q, pHypothesis->R, EHull, false, &camera);

	//		pVNClassifier->fitParams.bInit = true;
	//		pVNClassifier->fitParams.bGnd = bGnd;

	//		// Hypothesis evaluation.

	//		RVLMULMXTVECT(pClass->M.Element, pHypothesis->q, pClass->M.h, pClass->M.w, d, i, j, a);

	//		pModel->Project(d, pHypothesis->R, t, camera, sceneSamples.imagePtArray, PtArray);

	//		sceneSupport = RECOG::VN_::SceneFittingScore2(sceneSamples, PtArray, pVNClassifier->fitParams.maxe, 2.0f, pVNClassifier->fitParams.cosSurfaceRayAngleThr, nOutliers, true);

	//		score = pVNClassifier->fitParams.kSceneSupport * sceneSupport - pVNClassifier->fitParams.kOutliers * (float)nOutliers;

	//		//Weighted sum of scores
	//		pHypothesis->cost = k1Stage * pHypothesis->cost + k2Stage * score;

	//		SURFEL::DeleteSceneSamples(sceneSamples);

	//		//Increment stage
	//		pHypothesis->estStage++;
	//	}
	//	else if (pHypothesis->estStage == 2) //check if hypothesis estimation is complete	
	//	{
	//		//If estimation is finished, try to create new hypothesis by merging other (near) objects with current hypothesis
	//		//run through objects
	//		for (int iObject = 0; iObject < validObjects.size(); iObject++)
	//		{
	//			//Check if object is in hypothesis already
	//			if (std::find(pHypothesis->objects.begin(), pHypothesis->objects.end(), validObjects.at(iObject)) != pHypothesis->objects.end())
	//				continue;

	//			//Check if hypothesis with those combination of objects exists already
	//			//Construct bitwise code
	//			currRel = 0;
	//			for (int o = 0; o < pHypothesis->objects.size(); o++)
	//				currRel = currRel | (1 << pHypothesis->objects[i]);
	//			currRel = currRel | (1 << iObject);
	//			//check
	//			found = false;
	//			for (int r = 0; r < rels.size(); r++)
	//			{
	//				if (currRel == rels[r])
	//				{
	//					found = true;
	//					break;
	//				}
	//			}
	//			if (found)
	//				continue;

	//			//Check if within bounding box
	//			if (!CheckIfWithinCTIBoundingBox(this, pHypothesis->objects.at(0), validObjects.at(iObject), 0.30))	//Hardcoded BB threshold
	//				continue;

	//			//Add to relations
	//			rels.push_back(currRel);

	//			//Get object
	//			iObject_ = pObjects->iObjectAssignedToNode[validObjects.at(iObject)];
	//			pObject_ = pObjects->objectArray.Element + iObject_;

	//			//Merge block descriptors
	//			float* new_d = new float[66];
	//			uchar* new_bd = new uchar[66];
	//			pVNClassifier->MergeDescriptors(pHypothesis->d, pHypothesis->bd, pObject_->d, pObject_->bd, 1, new_d, new_bd);

	//			//Create (again) multiple new hypothesis
	//			// Project block descriptor to the latent subspace.
	//			pVNClassifier->ProjectToLatentSubspace(new_d, new_bd, pClass, &fitData, &allbuffer);
	//			//sort
	//			std::sort(allbuffer.begin(), allbuffer.end(), LatSorrFunc);

	//			/// Create hypothesis for first N ?
	//			for (int i = 0; i < maxNoHypPerObj; i++)
	//			{
	//				E = std::get<0>(allbuffer[i]);
	//				q = std::get<1>(allbuffer[i]);
	//				iR = std::get<2>(allbuffer[i]);
	//				nb = std::get<3>(allbuffer[i]);
	//				bAllDoFsVisible = std::get<4>(allbuffer[i]);

	//				if (bAllDoFsVisible && nb >= 16)
	//				{
	//					RECOG::VN_::Hypothesis *pHypothesisNew;
	//					RVLMEM_ALLOC_STRUCT(pMem, RECOG::VN_::Hypothesis, pHypothesis);

	//					pHypothesisNew->d = new_d;
	//					pHypothesisNew->bd = new_bd;
	//					pHypothesisNew->cost = E;
	//					pHypothesisNew->iClass = iClass;
	//					pHypothesisNew->estStage = 1;	//estimation stage
	//					pHypothesisNew->objects = pHypothesis->objects;
	//					pHypothesisNew->objects.push_back(validObjects.at(iObject)); //Add object to hypothesis

	//					// Copy latent vector.
	//					RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pClass->M.h, pHypothesisNew->q);
	//					memcpy(pHypothesisNew->q, q, pClass->M.h * sizeof(float));

	//					surfelListSize = QLIST::Size<QLIST::Index>(pObject_->surfelList) + pHypothesisNew->iSurfelArray.n;

	//					RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, surfelListSize, pHypothesisNew->iSurfelArray.Element);
	//					//Copy object surfels and then orig hypothesis surfels
	//					QLIST::CopyToArray(&(pObject_->surfelList), &(pHypothesisNew->iSurfelArray));
	//					memcpy(pHypothesisNew->iSurfelArray.Element + QLIST::Size<QLIST::Index>(pObject_->surfelList), pHypothesis->iSurfelArray.Element, pHypothesis->iSurfelArray.n);
	//					//Copy vertex arrays
	//					RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, pHypothesis->iVertexArray.n + pObject_->iVertexArray.n, pHypothesisNew->iVertexArray.Element);
	//					memcpy(pHypothesisNew->iVertexArray.Element, pHypothesis->iVertexArray.Element, pHypothesis->iVertexArray.n);//copy original hypothesis veretex array first
	//					memcpy(pHypothesisNew->iVertexArray.Element + pHypothesis->iVertexArray.n, pObject_->iVertexArray.Element, pObject_->iVertexArray.n);//then copy object veretex array
	//					pHypothesisNew->iVertexArray.n = pHypothesis->iVertexArray.n + pObject_->iVertexArray.n;

	//					R_ = pVNClassifier->SO3Samples.Element + 9 * iR;

	//					RVLCOPYMX3X3(R_, pHypothesisNew->R);	// This line is not needed if fitting of the class model to scene samples is performed.				

	//					//Add new hypothesis
	//					hypotheses.push_back(pHypothesisNew);
	//				}
	//			}

	//			
	//		}
	//		pHypothesis->estStage++;
	//	}
	//	else   //if estimation stage is more than 3 than estimation and expansion of this hypothesis is finished
	//		continue;
	//}

	////Chech scene consistency
	//
	//RECOG::VN_::Delete(fitData);
	//RECOG::VN_::Delete(fitDataRot);
	//delete[] iSurfelArray.Element;
	//delete[] q;
	//delete[] d;
	//delete[] PtArray.Element;
	//delete[] PtArray_.Element;

	////deallocate buffer
	//for (int i = 0; i < 1056; i++)
	//	delete [] std::get<1>(allbuffer[i]);
}


void ObjectDetector::ObjectAggregationLevel2VN()
{
	VNClassifier *pVNClassifier = (VNClassifier *)vpVNClassifier;

	pVNClassifier->descriptorMem.Clear();

	int iClass = pPSGM->debug1;

	RECOG::ClassData *pClass = pVNClassifier->classArray.Element + iClass;

	// Prepare data for scene fitting.

	Camera camera;

	camera.fu = 525;
	camera.fv = 525;
	camera.uc = 320;
	camera.vc = 240;
	camera.w = mesh.width;
	camera.h = mesh.height;

	SURFEL::SceneSamples sceneSamples;

	sceneSamples.PGnd = NULL;

	Array<OrientedPoint> PtArray;

	PtArray.Element = new OrientedPoint[mesh.NodeArray.n];

	Array<Point> PtArray_;

	PtArray_.Element = new Point[mesh.NodeArray.n];

	RECOG::VN_::FitData fitData;

	RECOG::VN_::Init(fitData, pClass->M, pVNClassifier->fitParams.beta, pVNClassifier->fitParams.lambda0, pVNClassifier->fitParams.lambda1, false, 0);

	RECOG::VN_::FitData fitDataRot;

	RECOG::VN_::Init(fitDataRot, pClass->M, pVNClassifier->fitParams.beta, pVNClassifier->fitParams.lambda0, pVNClassifier->fitParams.lambda1, true, pSurfels->vertexArray.n);

	// Generating a list of valid objects and creating VN descriptors.

	Array<int> iSurfelArray;

	iSurfelArray.Element = new int[pSurfels->NodeArray.n];

	FILE *fp = fopen((std::string(resultsFolder) + "\\scenedescriptors.txt").data(), "w");

	FILE *fpInstances = fopen((std::string(resultsFolder) + "\\instances.txt").data(), "w");

	float *q = new float[pClass->M.h];

	float *d = new float[pClass->M.w];

	float t[3];

	RVLNULL3VECTOR(t);

	hypotheses.clear();

	int i, j;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	std::vector<int> validObjects;
	//std::vector<std::pair<float*, uchar*>> validObjects_BDs; //For conviniance
	int iObject, iObject_;
	SURFEL::Object *pObject_;
	float E;
	int iR;
	bool bAllDoFsVisible;
	int nb, nSamples;
	float *a;
	RECOG::VN_::Hypothesis *pHypothesis;
	int surfelListSize;
	float EHull;
	VN *pModel;
	bool bGnd;
	float *R_;
	float zMin;
	int nOutliers;
	float sceneSupport, score, chamferDist;

	for (iObject = 0; iObject < pObjects->NodeArray.n; iObject++)
	{
		pObject = pObjects->NodeArray.Element + iObject;

		if (pObject->size < pObjects->minObjectSize)
			continue;

		piElement = pObject->elementList.pFirst;

		//check if object

		if (!piElement)
			continue;
		
		// Add object to the list of valid objects.

		validObjects.push_back(iObject);

		// Compute block descriptor.

		iObject_ = pObjects->iObjectAssignedToNode[iObject];

		pObject_ = pObjects->objectArray.Element + iObject_;

		QLIST::CopyToArray(&(pObject_->surfelList), &iSurfelArray);

		pVNClassifier->ComputeDescriptors(iSurfelArray, pObject_->iVertexArray, &(pObject_->d), &(pObject_->bd));

		////Add object block descriptor to list
		//validObjects_BDs.push_back(std::make_pair(pObject_->d, pObject_->bd));

		// Project block descriptor to the latent subspace.

		E = pVNClassifier->ProjectToLatentSubspace(pObject_->d, pObject_->bd, pClass, &fitData, q, iR, nb, bAllDoFsVisible);

		// Only for debugging purpose!

		//if (iObject_ == 1)
		//{
		//	printf("iR=%d\n", iR);

		//	for (int i = 0; i < pClass->M.h; i++)
		//		printf("%f\t", q[i]);

		//	printf("\n");

		//	pVNClassifier->SaveDescriptors(fp, pObject_->d, pObject_->bd, iObject_, RVLVN_CLUSTER_TYPE_CONVEX);
		//}

		/// Create hypothesis.

		if (bAllDoFsVisible && nb >= 16)
		{
			RVLMEM_ALLOC_STRUCT(pMem, RECOG::VN_::Hypothesis, pHypothesis);

			pHypothesis->d = pObject_->d;
			pHypothesis->bd = pObject_->bd;
			pHypothesis->cost = E;
			pHypothesis->iClass = iClass;
			pHypothesis->estStage = 1;	//estimation stage
			pHypothesis->objects.push_back(iObject); //Add object to hypothesis

			surfelListSize = QLIST::Size<QLIST::Index>(pObject_->surfelList);

			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, surfelListSize, pHypothesis->iSurfelArray.Element);

			QLIST::CopyToArray(&(pObject_->surfelList), &(pHypothesis->iSurfelArray));

			pHypothesis->iVertexArray = pObject_->iVertexArray;

			R_ = pVNClassifier->SO3Samples.Element + 9 * iR;

			//printf("iR=%d\n", iR);

			RVLCOPYMX3X3(R_, pHypothesis->R);	// This line is not needed if fitting of the class model to scene samples is performed.

			// Fit class model to scene vertices.

			//pSurfels->CenterVertices(pHypothesis->iVertexArray, fitDataRot.P, fitDataRot.Pc);

			//memcpy(fitDataRot.q_ + 3, q, pClass->M.h * sizeof(float));

			////pSurfels->VertexCentroid(pHypothesis->iVertexArray, fitDataRot.Pc);

			//pVNClassifier->FitRotLS(pHypothesis->iVertexArray, 1.0f, pVNClassifier->sampledUnitSphere.Element, pClass, R_, 
			//	pVNClassifier->fitParams.nFitRotLSIterations, true, true, &fitDataRot, q, pHypothesis->R, E);

			// Fit class model to scene samples.

			pSurfels->SampleSurfelSet(&mesh, pHypothesis->iSurfelArray, pHypothesis->iVertexArray, camera, sceneSamples);

			PtArray.n = sceneSamples.imagePtArray.h;

			bGnd = pVNClassifier->fitParams.bGnd;

			pVNClassifier->fitParams.bInit = false;
			pVNClassifier->fitParams.bGnd = pVNClassifier->fitParams.bGnd2;

			pModel = pVNClassifier->models[pClass->iMetaModel];

			if (pPSGM->debug2 == 0)
				pClass->nHull = 0;

			pModel->FitRotLMCC(pSurfels, pHypothesis->iVertexArray, sceneSamples, pClass, NULL, NULL, R_, pVNClassifier->fitParams, 
				q, pHypothesis->R, EHull, false, &camera);

			pVNClassifier->fitParams.bInit = true;
			pVNClassifier->fitParams.bGnd = bGnd;

			// Copy latent vector.
			
			RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pClass->M.h, pHypothesis->q);

			memcpy(pHypothesis->q, q, pClass->M.h * sizeof(float));

			// Hypothesis evaluation.

			RVLMULMXTVECT(pClass->M.Element, q, pClass->M.h, pClass->M.w, d, i, j, a);

			pModel->Project(d, pHypothesis->R, t, camera, sceneSamples.imagePtArray, PtArray);

			sceneSupport = RECOG::VN_::SceneFittingScore2(sceneSamples, PtArray, pVNClassifier->fitParams.maxe, 2.0f, 
				pVNClassifier->fitParams.cosSurfaceRayAngleThr, nSamples, chamferDist, nOutliers, true);

			score = pVNClassifier->fitParams.kSceneSupport * sceneSupport - pVNClassifier->fitParams.kOutliers * (float)nOutliers;

			SURFEL::DeleteSceneSamples(sceneSamples);

			// Add hypothesis to the hypothesis list.

			hypotheses.push_back(pHypothesis);

			// Only for debugging purpose!

			pVNClassifier->SaveLatentVector(fpInstances, q, 0);
		}

		//if (iObject_ == 1)
		//{
		//	FILE *fpVertices = fopen((std::string(resultsFolder) + "\\vertices.txt").data(), "w");

		//	float *P;

		//	for (int i = 0; i < pObject_->iVertexArray.n; i++)
		//	{
		//		P = pSurfels->vertexArray.Element[pObject_->iVertexArray.Element[i]]->P;

		//		fprintf(fpVertices, "%f\t%f\t%f\n", P[0], P[1], P[2]);
		//	}

		//	fclose(fpVertices);
		//}

		///
	}

	fclose(fp);
	fclose(fpInstances);

	//Chech scene consistency
	
	RECOG::VN_::Delete(fitData);
	RECOG::VN_::Delete(fitDataRot);
	delete[] iSurfelArray.Element;
	delete[] q;
	delete[] d;
	delete[] PtArray.Element;
	delete[] PtArray_.Element;
}

void ObjectDetector::ObjectAggregationLevel2VN3()
{
	VNClassifier *pVNClassifier = (VNClassifier *)vpVNClassifier;

	pVNClassifier->descriptorMem.Clear();

	// Prepare data for scene fitting.

	pVNClassifier->Init(&mesh);

	FILE *fp = fopen((std::string(resultsFolder) + "\\scenedescriptors.txt").data(), "w");

	FILE *fpInstances = fopen((std::string(resultsFolder) + "\\instances.txt").data(), "w");

	//float *q = new float[pClass->M.h];

	//float *d = new float[pClass->M.w];

	float t[3];

	RVLNULL3VECTOR(t);

	Visualizer visualizer;

	if (visualizationData.bVisualizeConvexHulls)
	{
		visualizer.Create();

		visualizer.SetMesh(&mesh);
	}

	int i, j;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	std::vector<int> validObjects;
	int iObject, iObject_;
	SURFEL::Object *pObject_;
	float E;
	int iR;
	bool bAllDoFsVisible;
	int nb, nSamples;
	float *a;
	RECOG::VN_::Hypothesis *pHypothesis;
	int surfelListSize;
	float EHull;
	VN *pModel;
	bool bGnd;
	float *R_;
	float zMin;
	int nOutliers;
	float sceneSupport, score, chamferDist;
	float *P, *P_;

	// validObjects <- array of objects larger than pObjects->minObjectSize

	for (iObject = 0; iObject < pObjects->NodeArray.n; iObject++)
	{
		pObject = pObjects->NodeArray.Element + iObject;

		if (pObject->size < pObjects->minObjectSize)
			continue;

		piElement = pObject->elementList.pFirst;

		//check if object

		if (!piElement)
			continue;

		// Add object to the list of valid objects.

		validObjects.push_back(iObject);
	}

	// convexObjects <- array of convex objects

	Array<int> iConvexObjects;

	iConvexObjects.Element = new int[validObjects.size()];

	iConvexObjects.n = 0;

	for (i = 0; i < validObjects.size(); i++)
	{
		iObject = validObjects[i];

		iObject_ = pObjects->iObjectAssignedToNode[iObject];

		if (!(pObjects->objectArray.Element[iObject_].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE))
			iConvexObjects.Element[iConvexObjects.n++] = iObject_;
	}

	// Surfel cells.

	pObjects->SurfelCells(validObjects);

	// Hypothesis generation.

	Array<int> iSegmentArray;

	iSegmentArray.Element = new int[validObjects.size()];

	hypotheses.clear();

	pVNClassifier->bGnd = pPSGM->bGnd;

	if (pVNClassifier->bGnd)
	{
		RVLCOPY3VECTOR(pPSGM->NGnd, pVNClassifier->NGnd);

		pVNClassifier->dGnd = pPSGM->dGnd;
	}

	QList<RECOG::VN_::Hypothesis> hypothesisList;
	QList<RECOG::VN_::Hypothesis> *pHypothesisList = &hypothesisList;

	FILE *fpHypothesisLog;

	if (bHypothesisLog)
		fpHypothesisLog = fopen((std::string(resultsFolder) + "\\hypotheses.log").data(), "w");

	int nSegmentCombinations = 0;

	int k, l, iVertex;
	float J[3];

	for (i = 0; i < validObjects.size(); i++)
	{
		iObject = validObjects[i];

		iObject_ = pObjects->iObjectAssignedToNode[iObject];

		iSegmentArray.Element[0] = iObject_;

		for (j = -1; j < i; j++)
		{
			CRVLTimer timer;

			timer.Start();

			if (j >= 0)
			{
				iObject = validObjects[j];

				iSegmentArray.Element[1] = pObjects->iObjectAssignedToNode[iObject];

				iSegmentArray.n = 2;
			}
			else
				iSegmentArray.n = 1;

			if (debug1 >= 0)
			{
				if (debug2 >= 0)
				{
					if (iSegmentArray.n < 2)
						continue;

					if (iSegmentArray.Element[0] != debug1 && iSegmentArray.Element[1] != debug1)
						continue;

					if (iSegmentArray.Element[0] != debug2 && iSegmentArray.Element[1] != debug2)
						continue;
				}
				else
				{
					if (iSegmentArray.n != 1)
						continue;

					if (iSegmentArray.Element[0] != debug1)
						continue;
				}
			}

			// Classify pObject_.	

			//QLIST::CopyToArray(&(pObject_->surfelList), &iSurfelArray);

			//RVLMEM_ALLOC_STRUCT(pMem, RECOG::VN_::Hypothesis, pHypothesis);

			//RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pVNClassifier->sampledUnitSphere.h, pHypothesis->d);

			//RVLMEM_ALLOC_STRUCT_ARRAY(pMem, uchar, pVNClassifier->sampledUnitSphere.h, pHypothesis->bd);		

			//pHypothesis->iClass = -1;

			//pHypothesis->cost = pVNClassifier->FitCTI(&mesh, iSurfelArray, pObject_->iVertexArray, camera, 10, pHypothesis->R, pHypothesis->d, 
			//	pHypothesis->bd);

			if (!pVNClassifier->Classify(&mesh, pObjects, &iConvexObjects, iSegmentArray, pVNClassifier->fitData, pHypothesisList, pVNClassifier->D,
				pVNClassifier->bD, pVNClassifier->f, pVNClassifier->R, J, pMem))
				continue;
				//int debug = 0;

			nSegmentCombinations++;

			timer.Stop();

			double tCTIClassification = timer.GetTime();

			// Alignment visualization in Matlab using RVLVNFitCTIDebug.m. (Only for debugging purpose!)

			//float *NCH = new float[3 * pVNClassifier->convexHull.iVisibleFaces.n];

			//float *NSrc, *NTgt;
			//MESH::Face *pFace;

			//for (int i = 0; i < pVNClassifier->convexHull.iVisibleFaces.n; i++)
			//{
			//	pFace = pVNClassifier->convexHull.faces.Element[pVNClassifier->convexHull.iVisibleFaces.Element[i]];

			//	NSrc = pFace->N;

			//	NTgt = NCH + 3 * i;

			//	RVLCOPY3VECTOR(NSrc, NTgt);

			//	RVLSCALE3VECTOR(NTgt, pFace->Area, NTgt);
			//}

			//FILE *fpNCH = fopen(".\\NCH.txt", "w");

			//PrintMatrix<float>(fpNCH, NCH, pVNClassifier->convexHull.iVisibleFaces.n, 3);

			//fclose(fpNCH);

			//delete[] NCH;

			//FILE *fpR = fopen(".\\R.txt", "w");

			//RECOG::VN_::Hypothesis *pHypothesis = pHypothesisList->pFirst;

			//while (pHypothesis)
			//{
			//	PrintMatrix<float>(fpR, pHypothesis->R, 3, 3);

			//	pHypothesis = pHypothesis->pNext;
			//}

			//fclose(fpR);

			///

			if (visualizationData.bVisualizeConvexHulls)
			{
				//convexHull.iVisibleFaces.Element[2] = convexHull.iVisibleFaces.Element[convexHull.iVisibleFaces.n - 1];
				//convexHull.iVisibleFaces.n--;

				pVNClassifier->convexHull.CreateVTKPolyData(NULL, NULL, true);

				visualizer.AddMesh(pVNClassifier->convexHull.pPolygonData);
			}

			//pVNClassifier->ComputeDescriptors(pHypothesis->d, D, bD);

			// Only for debugging purpose!

			//if (iObject_ == 1)
			//{
			//	printf("iR=%d\n", iR);

			//	for (int i = 0; i < pClass->M.h; i++)
			//		printf("%f\t", q[i]);

			//	printf("\n");

			//	pVNClassifier->SaveDescriptors(fp, pObject_->d, pObject_->bd, iObject_, RVLVN_CLUSTER_TYPE_CONVEX);
			//}

			//pVNClassifier->SaveDescriptors(fp, D, bD, nDescriptors, iObject_, RVLVN_CLUSTER_TYPE_CONVEX);

			/// Create hypothesis.

			// Add hypothesis to the hypothesis list.

			if (bHypothesisLog)
			{
				fprintf(fpHypothesisLog, "Object %d", iSegmentArray.Element[0]);

				for (k = 1; k < iSegmentArray.n; k++)
					fprintf(fpHypothesisLog, "+%d", iSegmentArray.Element[k]);

				fprintf(fpHypothesisLog, "\n");
			}

			pHypothesis = hypothesisList.pFirst;

			while (pHypothesis)
			{
				if (iSegmentArray.n == 1)
					pHypothesis->cost += appearanceCost;

				hypotheses.push_back(pHypothesis);

				if (bHypothesisLog)
					fprintf(fpHypothesisLog, "Class %d E=%f \n", pHypothesis->iClass, pHypothesis->cost);

				pHypothesis = pHypothesis->pNext;
			}

			if (bHypothesisLog)
				fprintf(fpHypothesisLog, "\n");

			// Only for debugging purpose!

			//pVNClassifier->SaveLatentVector(fpInstances, q, 0);

			///

			//if (iObject_ == 1)
			//{
			//	FILE *fpVertices = fopen((std::string(resultsFolder) + "\\vertices.txt").data(), "w");

			//	float *P;

			//	for (int i = 0; i < pObject_->iVertexArray.n; i++)
			//	{
			//		P = pSurfels->vertexArray.Element[pObject_->iVertexArray.Element[i]]->P;

			//		fprintf(fpVertices, "%f\t%f\t%f\n", P[0], P[1], P[2]);
			//	}

			//	fclose(fpVertices);
			//}

			///
		}	// for every object with index lower than iObject_.
	}	// for every object.

	fclose(fp);
	fclose(fpInstances);

	// Interpretation.

	RVL_DELETE_ARRAY(pVNClassifier->hypotheses.Element);

	pVNClassifier->hypotheses.n = hypotheses.size();

	pVNClassifier->hypotheses.Element = new RECOG::VN_::Hypothesis *[pVNClassifier->hypotheses.n];

	for (i = 0; i < pVNClassifier->hypotheses.n; i++)
		pVNClassifier->hypotheses.Element[i] = (RECOG::VN_::Hypothesis *)(hypotheses[i]);
	
	pVNClassifier->Interpretation();

	RVL_DELETE_ARRAY(objectMap.Element);

	// Save hypotheses to a file.

	if (bHypothesisLog)
	{
		fprintf(fpHypothesisLog, "\n");

		int iHypothesis;

		for (i = 0; i < pVNClassifier->sortedHypotheses.n; i++)
		{
			iHypothesis = pVNClassifier->sortedHypotheses.Element[i].idx;

			pHypothesis = (RECOG::VN_::Hypothesis *)(hypotheses[iHypothesis]);

			fprintf(fpHypothesisLog, "%03d: Segment", iHypothesis);
			
			for (j = 0; j < pHypothesis->iSegmentArray.n; j++)
				fprintf(fpHypothesisLog, " %d", pHypothesis->iSegmentArray.Element[j]);

			fprintf(fpHypothesisLog, " Class %d size=%d Cost: shape=%f convexity=%f outliers=%d total=%f\n", 
				pHypothesis->iClass, pHypothesis->support, pHypothesis->shapeCost, pHypothesis->convexityError, pHypothesis->nOutliers, 
				pHypothesis->cost);
		}

		fprintf(fpHypothesisLog, "\n\n");

		for (i = 0; i < pVNClassifier->interpretation.n; i++)
		{
			iHypothesis = pVNClassifier->interpretation.Element[i];

			pHypothesis = pVNClassifier->hypotheses.Element[iHypothesis];

			fprintf(fpHypothesisLog, "%03d: Segment", iHypothesis);

			for (j = 0; j < pHypothesis->iSegmentArray.n; j++)
				fprintf(fpHypothesisLog, " %d", pHypothesis->iSegmentArray.Element[j]);

			fprintf(fpHypothesisLog, " Class %d size=%d Cost: shape=%f convexity=%f outliers=%d total=%f\n",
				pHypothesis->iClass, pHypothesis->support, pHypothesis->shapeCost, pHypothesis->convexityError, pHypothesis->nOutliers,
				pHypothesis->cost);
		}

		fclose(fpHypothesisLog);
	}

	// Pixel-level image segmentation.

	pVNClassifier->ImageSegmentation(&mesh, objectMap);

	hypotheses.clear();

	for (i = 0; i < pVNClassifier->interpretation.n; i++)
		hypotheses.push_back(pVNClassifier->hypotheses.Element[pVNClassifier->interpretation.Element[i]]);

	// Display convex hulls.

	if (visualizationData.bVisualizeConvexHulls)
		visualizer.Run();

	// Free memory.

	delete[] iSegmentArray.Element;
}

void ObjectDetector::ObjectAggregationLevel2VN4(int maxNeighbourhoodSize)
{
	//OVDJE

	//VNClassifier *pVNClassifier = (VNClassifier *)vpVNClassifier;

	//pVNClassifier->descriptorMem.Clear();

	////get object bounding sphere neighbourhood
	//pVNClassifier->maxBoundingSphereRadius = 0.2f;
	//std::vector<std::vector<int>> bsNeighbourhood = pObjects->GetObjectBSNeighbourhood(pVNClassifier->maxBoundingSphereRadius);

	////Generate hypotheses based on nighbourhoods
	//std::vector<uint64_t> hypothesesBV;
	//uint64_t bvCode;
	//uint64_t tempULL = 1;
	//int nSize = 0;
	//int komb;
	//bool found = false;
	//std::vector<int> objs;
	//for (int i = 0; i < bsNeighbourhood.size(); i++)
	//{
	//	//Check if valid object/neighbourhood
	//	if (bsNeighbourhood[i].size() == 0)
	//		continue;

	//	//get neighbourhood size, truncated to maxNeighbourhoodSize
	//	nSize = bsNeighbourhood[i].size();// /*<= maxNeighbourhoodSize ? bsNeighbourhood[i].size() : maxNeighbourhoodSize;*/
	//	komb = 1 << nSize;
	//	for (int per = 1; per < komb; per = per + 2)	//First on the list is alwaes added //Only odd numbers since first bit must be set
	//	{
	//		bvCode = 0;
	//		objs.clear();
	//		for (int b = 0; b < nSize; b++)
	//		{
	//			//tempULL = 1;
	//			if (per & (tempULL << b))
	//			{
	//				bvCode = bvCode | (tempULL << bsNeighbourhood[i][b]);
	//				objs.push_back(bsNeighbourhood[i][b]);
	//			}
	//		}

	//		//Check if maximum number of objects in hypothesis is exceded
	//		if (objs.size() > maxNeighbourhoodSize)
	//			continue;

	//		//Check if that code exists.
	//		found = false;
	//		for (int r = 0; r < hypothesesBV.size(); r++)
	//		{
	//			if (bvCode == hypothesesBV[r])
	//			{
	//				found = true;
	//				break;
	//			}
	//		}
	//		if (found)
	//			continue;

	//		//Check if the combination is withinn bounding sphere
	//		if (!pObjects->CheckBoundingSphere(objs, pVNClassifier->maxBoundingSphereRadius))
	//			continue;

	//		//Add to hypothesis bit values
	//		hypothesesBV.push_back(bvCode);
	//	}
	//}

	//// Prepare data for scene fitting.

	//pVNClassifier->Init(&mesh);

	//FILE *fp = fopen((std::string(resultsFolder) + "\\scenedescriptors.txt").data(), "w");

	//FILE *fpInstances = fopen((std::string(resultsFolder) + "\\instances.txt").data(), "w");

	////float *q = new float[pClass->M.h];

	////float *d = new float[pClass->M.w];

	//float t[3];

	//RVLNULL3VECTOR(t);

	//Visualizer visualizer;

	//if (visualizationData.bVisualizeConvexHulls)
	//{
	//	visualizer.Create();

	//	visualizer.SetMesh(&mesh);
	//}

	//int i, j;
	//GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	//QLIST::Index *piElement;
	//std::vector<int> validObjects;
	//int iObject, iObject_;
	//SURFEL::Object *pObject_;
	//float E;
	//int iR;
	//bool bAllDoFsVisible;
	//int nb, nSamples;
	//float *a;
	//RECOG::VN_::Hypothesis *pHypothesis;
	//int surfelListSize;
	//float EHull;
	//VN *pModel;
	//bool bGnd;
	//float *R_;
	//float zMin;
	//int nOutliers;
	//float sceneSupport, score, chamferDist;
	//float *P, *P_;

	//// validObjects <- array of objects larger than pObjects->minObjectSize

	//for (iObject = 0; iObject < pObjects->NodeArray.n; iObject++)
	//{
	//	pObject = pObjects->NodeArray.Element + iObject;

	//	if (pObject->size < pObjects->minObjectSize)
	//		continue;

	//	piElement = pObject->elementList.pFirst;

	//	//check if object

	//	if (!piElement)
	//		continue;

	//	// Add object to the list of valid objects.

	//	validObjects.push_back(iObject);
	//}

	//// convexObjects <- array of convex objects

	//Array<int> iConvexObjects;

	//iConvexObjects.Element = new int[validObjects.size()];

	//iConvexObjects.n = 0;

	//for (i = 0; i < validObjects.size(); i++)
	//{
	//	iObject = validObjects[i];

	//	iObject_ = pObjects->iObjectAssignedToNode[iObject];

	//	if (!(pObjects->objectArray.Element[iObject_].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE))
	//		iConvexObjects.Element[iConvexObjects.n++] = iObject_;
	//}

	//// Surfel cells.

	//pObjects->SurfelCells(validObjects);

	//// Hypothesis generation.

	//Array<int> iSegmentArray;

	//iSegmentArray.Element = new int[validObjects.size()];

	//hypotheses.clear();

	//pVNClassifier->bGnd = pPSGM->bGnd;

	//if (pVNClassifier->bGnd)
	//{
	//	RVLCOPY3VECTOR(pPSGM->NGnd, pVNClassifier->NGnd);

	//	pVNClassifier->dGnd = pPSGM->dGnd;
	//}

	//QList<RECOG::VN_::Hypothesis> hypothesisList;
	//QList<RECOG::VN_::Hypothesis> *pHypothesisList = &hypothesisList;

	//FILE *fpHypothesisLog;

	//if (bHypothesisLog)
	//fpHypothesisLog = fopen((std::string(resultsFolder) + "\\hypotheses.log").data(), "w");

	//int nSegmentCombinations = 0;

	//int k, l, iVertex;

	//for (int i = 0; i < hypothesesBV.size(); i++)
	//{
	//	CRVLTimer timer;
	//	timer.Start();

	//	//Convert binary code to segment list
	//	bvCode = hypothesesBV[i];
	//	iSegmentArray.n = 0;
	//	for (int b = 0; b < 64; b++)	//64 is number of bits in long long?
	//	{
	//		tempULL = 1;
	//		if (bvCode & (tempULL << b))
	//		{
	//			iSegmentArray.Element[iSegmentArray.n] = b;// pObjects->iObjectAssignedToNode[b];
	//			iSegmentArray.n++;
	//		}
	//	}

	//	if (debug1 >= 0)
	//	{
	//		if (debug2 >= 0)
	//		{
	//			if (iSegmentArray.n < 2)
	//				continue;

	//			if (iSegmentArray.Element[0] != debug1 && iSegmentArray.Element[1] != debug1)
	//				continue;

	//			if (iSegmentArray.Element[0] != debug2 && iSegmentArray.Element[1] != debug2)
	//				continue;
	//		}
	//		else
	//		{
	//			if (iSegmentArray.n != 1)
	//				continue;

	//			if (iSegmentArray.Element[0] != debug1)
	//				continue;
	//		}
	//	}

	//	// Classify pObject_.	

	//	//QLIST::CopyToArray(&(pObject_->surfelList), &iSurfelArray);

	//	//RVLMEM_ALLOC_STRUCT(pMem, RECOG::VN_::Hypothesis, pHypothesis);

	//	//RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pVNClassifier->sampledUnitSphere.h, pHypothesis->d);

	//	//RVLMEM_ALLOC_STRUCT_ARRAY(pMem, uchar, pVNClassifier->sampledUnitSphere.h, pHypothesis->bd);		

	//	//pHypothesis->iClass = -1;

	//	//pHypothesis->cost = pVNClassifier->FitCTI(&mesh, iSurfelArray, pObject_->iVertexArray, camera, 10, pHypothesis->R, pHypothesis->d, 
	//	//	pHypothesis->bd);

	//	if (!pVNClassifier->Classify(&mesh, pObjects, &iConvexObjects, iSegmentArray, pVNClassifier->fitData, pHypothesisList, pVNClassifier->D,
	//		pVNClassifier->bD, pVNClassifier->f, pVNClassifier->R, pMem))
	//		continue;

	//	nSegmentCombinations++;

	//	timer.Stop();

	//	double tCTIClassification = timer.GetTime();

	//	// Alignment visualization in Matlab using RVLVNFitCTIDebug.m. (Only for debugging purpose!)

	//	float *NCH = new float[3 * pVNClassifier->convexHull.iVisibleFaces.n];

	//	float *NSrc, *NTgt;
	//	MESH::Face *pFace;

	//	for (int i = 0; i < pVNClassifier->convexHull.iVisibleFaces.n; i++)
	//	{
	//		pFace = pVNClassifier->convexHull.faces.Element[pVNClassifier->convexHull.iVisibleFaces.Element[i]];

	//		NSrc = pFace->N;

	//		NTgt = NCH + 3 * i;

	//		RVLCOPY3VECTOR(NSrc, NTgt);

	//		RVLSCALE3VECTOR(NTgt, pFace->Area, NTgt);
	//	}

	//	FILE *fpNCH = fopen(".\\NCH.txt", "w");

	//	PrintMatrix<float>(fpNCH, NCH, pVNClassifier->convexHull.iVisibleFaces.n, 3);

	//	fclose(fpNCH);

	//	delete[] NCH;

	//	FILE *fpR = fopen(".\\R.txt", "w");

	//	RECOG::VN_::Hypothesis *pHypothesis = pHypothesisList->pFirst;

	//	while (pHypothesis)
	//	{
	//		PrintMatrix<float>(fpR, pHypothesis->R, 3, 3);

	//		pHypothesis = pHypothesis->pNext;
	//	}

	//	fclose(fpR);

	//	///

	//	if (visualizationData.bVisualizeConvexHulls)
	//	{
	//		//convexHull.iVisibleFaces.Element[2] = convexHull.iVisibleFaces.Element[convexHull.iVisibleFaces.n - 1];
	//		//convexHull.iVisibleFaces.n--;

	//		pVNClassifier->convexHull.CreateVTKPolyData(NULL, NULL, true);

	//		visualizer.AddMesh(pVNClassifier->convexHull.pPolygonData);
	//	}

	//	//pVNClassifier->ComputeDescriptors(pHypothesis->d, D, bD);

	//	// Only for debugging purpose!

	//	//if (iObject_ == 1)
	//	//{
	//	//	printf("iR=%d\n", iR);

	//	//	for (int i = 0; i < pClass->M.h; i++)
	//	//		printf("%f\t", q[i]);

	//	//	printf("\n");

	//	//	pVNClassifier->SaveDescriptors(fp, pObject_->d, pObject_->bd, iObject_, RVLVN_CLUSTER_TYPE_CONVEX);
	//	//}

	//	//pVNClassifier->SaveDescriptors(fp, D, bD, nDescriptors, iObject_, RVLVN_CLUSTER_TYPE_CONVEX);

	//	/// Create hypothesis.

	//	// Add hypothesis to the hypothesis list.

	//	if (bHypothesisLog)
	//	{
	//		fprintf(fpHypothesisLog, "Object %d", iSegmentArray.Element[0]);

	//		for (k = 1; k < iSegmentArray.n; k++)
	//			fprintf(fpHypothesisLog, "+%d", iSegmentArray.Element[k]);

	//		fprintf(fpHypothesisLog, "\n");
	//	}

	//	pHypothesis = hypothesisList.pFirst;

	//	while (pHypothesis)
	//	{
	//		if (iSegmentArray.n == 1)
	//			pHypothesis->cost += appearanceCost;

	//		hypotheses.push_back(pHypothesis);

	//		if (bHypothesisLog)
	//			fprintf(fpHypothesisLog, "Class %d E=%f \n", pHypothesis->iClass, pHypothesis->cost);

	//		pHypothesis = pHypothesis->pNext;
	//	}

	//	if (bHypothesisLog)
	//		fprintf(fpHypothesisLog, "\n");

	//	// Only for debugging purpose!

	//	//pVNClassifier->SaveLatentVector(fpInstances, q, 0);

	//	///

	//	//if (iObject_ == 1)
	//	//{
	//	//	FILE *fpVertices = fopen((std::string(resultsFolder) + "\\vertices.txt").data(), "w");

	//	//	float *P;

	//	//	for (int i = 0; i < pObject_->iVertexArray.n; i++)
	//	//	{
	//	//		P = pSurfels->vertexArray.Element[pObject_->iVertexArray.Element[i]]->P;

	//	//		fprintf(fpVertices, "%f\t%f\t%f\n", P[0], P[1], P[2]);
	//	//	}

	//	//	fclose(fpVertices);
	//	//}

	//	///
	//	//}	// for every object with index lower than iObject_.
	//}	// for every hypothesis.

	//fclose(fp);
	//fclose(fpInstances);

	//// Interpretation.

	//RVL_DELETE_ARRAY(pVNClassifier->hypotheses.Element);

	//pVNClassifier->hypotheses.n = hypotheses.size();

	//pVNClassifier->hypotheses.Element = new RECOG::VN_::Hypothesis *[pVNClassifier->hypotheses.n];

	//for (i = 0; i < pVNClassifier->hypotheses.n; i++)
	//	pVNClassifier->hypotheses.Element[i] = (RECOG::VN_::Hypothesis *)(hypotheses[i]);

	//pVNClassifier->Interpretation();

	//RVL_DELETE_ARRAY(objectMap.Element);

	//// Save hypotheses to a file.

	//if (bHypothesisLog)
	//{
	//	fprintf(fpHypothesisLog, "\n");

	//	int iHypothesis;

	//	for (i = 0; i < pVNClassifier->sortedHypotheses.n; i++)
	//	{
	//		iHypothesis = pVNClassifier->sortedHypotheses.Element[i].idx;

	//		pHypothesis = (RECOG::VN_::Hypothesis *)(hypotheses[iHypothesis]);

	//		fprintf(fpHypothesisLog, "%03d: Segment", iHypothesis);

	//		for (j = 0; j < pHypothesis->iSegmentArray.n; j++)
	//			fprintf(fpHypothesisLog, " %d", pHypothesis->iSegmentArray.Element[j]);

	//		fprintf(fpHypothesisLog, " Class %d size=%d Cost: shape=%f convexity=%f outliers=%f total=%f\n",
	//			pHypothesis->iClass, pHypothesis->support, pHypothesis->shapeCost, pHypothesis->convexityCost, pHypothesis->outlierCost,
	//			pHypothesis->cost);
	//	}

	//	fprintf(fpHypothesisLog, "\n\n");

	//	for (i = 0; i < pVNClassifier->interpretation.n; i++)
	//	{
	//		iHypothesis = pVNClassifier->interpretation.Element[i];

	//		pHypothesis = pVNClassifier->hypotheses.Element[iHypothesis];

	//		fprintf(fpHypothesisLog, "%03d: Segment", iHypothesis);

	//		for (j = 0; j < pHypothesis->iSegmentArray.n; j++)
	//			fprintf(fpHypothesisLog, " %d", pHypothesis->iSegmentArray.Element[j]);

	//		fprintf(fpHypothesisLog, " Class %d size=%d Cost: shape=%f convexity=%f outliers=%f total=%f\n",
	//			pHypothesis->iClass, pHypothesis->support, pHypothesis->shapeCost, pHypothesis->convexityCost, pHypothesis->outlierCost,
	//			pHypothesis->cost);
	//	}

	//	fclose(fpHypothesisLog);
	//}

	//// Pixel-level image segmentation.

	//pVNClassifier->ImageSegmentation(&mesh, objectMap);

	//hypotheses.clear();

	//for (i = 0; i < pVNClassifier->interpretation.n; i++)
	//	hypotheses.push_back(pVNClassifier->hypotheses.Element[pVNClassifier->interpretation.Element[i]]);

	//// Display convex hulls.

	//if (visualizationData.bVisualizeConvexHulls)
	//visualizer.Run();

	//// Free memory.

	//delete[] iSegmentArray.Element;




	VNClassifier *pVNClassifier = (VNClassifier *)vpVNClassifier;

	pVNClassifier->descriptorMem.Clear();

	//get object bounding sphere neighbourhood
	//pVNClassifier->maxBoundingSphereRadius = 0.2f;
	std::vector<std::vector<int>> bsNeighbourhood = pObjects->GetObjectBSNeighbourhood(pVNClassifier->maxBoundingSphereRadius);

	//Generate hypotheses based on nighbourhoods
	std::vector<uint64_t> hypothesesBV;
	uint64_t bvCode;
	uint64_t tempULL = 1;
	int nSize = 0;
	int komb;
	bool found = false;
	std::vector<int> objs;
	for (int i = 0; i < bsNeighbourhood.size(); i++)
	{
		//Check if valid object/neighbourhood
		if (bsNeighbourhood[i].size() == 0)
			continue;

		//get neighbourhood size, truncated to maxNeighbourhoodSize
		nSize = bsNeighbourhood[i].size();// /*<= maxNeighbourhoodSize ? bsNeighbourhood[i].size() : maxNeighbourhoodSize;*/
		komb = 1 << nSize;
		for (int per = 1; per < komb; per = per + 2)	//First on the list is alwaes added //Only odd numbers since first bit must be set
		{
			bvCode = 0;
			objs.clear();
			for (int b = 0; b < nSize; b++)
			{
				//tempULL = 1;
				if (per & (tempULL << b))
				{
					bvCode = bvCode | (tempULL << bsNeighbourhood[i][b]);
					objs.push_back(bsNeighbourhood[i][b]);
				}
			}

			//Check if maximum number of objects in hypothesis is exceded
			if (objs.size() > maxNeighbourhoodSize)
				continue;

			//Check if that code exists.
			found = false;
			for (int r = 0; r < hypothesesBV.size(); r++)
			{
				if (bvCode == hypothesesBV[r])
				{
					found = true;
					break;
				}
			}
			if (found)
				continue;

			//Check if the combination is withinn bounding sphere
			if (!pObjects->CheckBoundingSphere(objs, pVNClassifier->maxBoundingSphereRadius))
				continue;

			//Add to hypothesis bit values
			hypothesesBV.push_back(bvCode);
		}
	}

	// Prepare data for scene fitting.

	pVNClassifier->Init(&mesh);

	FILE *fp = fopen((std::string(resultsFolder) + "\\scenedescriptors.txt").data(), "w");

	FILE *fpInstances = fopen((std::string(resultsFolder) + "\\instances.txt").data(), "w");

	//float *q = new float[pClass->M.h];

	//float *d = new float[pClass->M.w];

	float t[3];

	RVLNULL3VECTOR(t);

	Visualizer visualizer;

	if (visualizationData.bVisualizeConvexHulls)
	{
		visualizer.Create();

		visualizer.SetMesh(&mesh);
	}

	int i, j;
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;
	std::vector<int> validObjects;
	int iObject, iObject_;
	SURFEL::Object *pObject_;
	float E;
	int iR;
	bool bAllDoFsVisible;
	int nb, nSamples;
	float *a;
	RECOG::VN_::Hypothesis *pHypothesis;
	int surfelListSize;
	float EHull;
	VN *pModel;
	bool bGnd;
	float *R_;
	float zMin;
	int nOutliers;
	float sceneSupport, score, chamferDist;
	float *P, *P_;

	// validObjects <- array of objects larger than pObjects->minObjectSize

	for (iObject = 0; iObject < pObjects->NodeArray.n; iObject++)
	{
		pObject = pObjects->NodeArray.Element + iObject;

		if (pObject->size < pObjects->minObjectSize)
			continue;

		piElement = pObject->elementList.pFirst;

		//check if object

		if (!piElement)
			continue;

		// Add object to the list of valid objects.

		validObjects.push_back(iObject);
	}

	// convexObjects <- array of convex objects

	Array<int> iConvexObjects;

	iConvexObjects.Element = new int[validObjects.size()];

	iConvexObjects.n = 0;

	for (i = 0; i < validObjects.size(); i++)
	{
		iObject = validObjects[i];

		iObject_ = pObjects->iObjectAssignedToNode[iObject];

		if (!(pObjects->objectArray.Element[iObject_].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE))
			iConvexObjects.Element[iConvexObjects.n++] = iObject_;
	}

	// Surfel cells.

	pObjects->SurfelCells(validObjects);

	// Hypothesis generation.

	Array<int> iSegmentArray;

	iSegmentArray.Element = new int[validObjects.size()];

	hypotheses.clear();

	pVNClassifier->bGnd = pPSGM->bGnd;

	if (pVNClassifier->bGnd)
	{
		RVLCOPY3VECTOR(pPSGM->NGnd, pVNClassifier->NGnd);

		pVNClassifier->dGnd = pPSGM->dGnd;
	}

	QList<RECOG::VN_::Hypothesis> hypothesisList;
	QList<RECOG::VN_::Hypothesis> *pHypothesisList = &hypothesisList;

	FILE *fpHypothesisLog;

	if (bHypothesisLog)
	fpHypothesisLog = fopen((std::string(resultsFolder) + "\\hypotheses.log").data(), "w");

	int nSegmentCombinations = 0;

	int k, l, iVertex;
	float J[3];

	/*for (i = 0; i < validObjects.size(); i++)
	{
		iObject = validObjects[i];

		iObject_ = pObjects->iObjectAssignedToNode[iObject];

		iSegmentArray.Element[0] = iObject_;

		for (j = -1; j < i; j++)
		{
			CRVLTimer timer;

			timer.Start();

			if (j >= 0)
			{
				iObject = validObjects[j];

				iSegmentArray.Element[1] = pObjects->iObjectAssignedToNode[iObject];

				iSegmentArray.n = 2;
			}
			else
				iSegmentArray.n = 1;

			if (debug1 >= 0)
			{
				if (debug2 >= 0)
				{
					if (iSegmentArray.n < 2)
						continue;

					if (iSegmentArray.Element[0] != debug1 && iSegmentArray.Element[1] != debug1)
						continue;

					if (iSegmentArray.Element[0] != debug2 && iSegmentArray.Element[1] != debug2)
						continue;
				}
				else
				{
					if (iSegmentArray.n != 1)
						continue;

					if (iSegmentArray.Element[0] != debug1)
						continue;
				}
			}*/
	for (int i = 0; i < hypothesesBV.size(); i++)
	{
		CRVLTimer timer;
		timer.Start();

		//Convert binary code to segment list
		bvCode = hypothesesBV[i];
		iSegmentArray.n = 0;
		for (int b = 0; b < 64; b++)	//64 is number of bits in long long?
		{
			tempULL = 1;
			if (bvCode & (tempULL << b))
			{
				iSegmentArray.Element[iSegmentArray.n] = b;// pObjects->iObjectAssignedToNode[b];
				iSegmentArray.n++;
			}
		}

		if (debug1 >= 0)
		{
			if (debug2 >= 0)
			{
				if (iSegmentArray.n < 2)
					continue;

				if (iSegmentArray.Element[0] != debug1 && iSegmentArray.Element[1] != debug1)
					continue;

				if (iSegmentArray.Element[0] != debug2 && iSegmentArray.Element[1] != debug2)
					continue;
			}
			else
			{
				if (iSegmentArray.n != 1)
					continue;

				if (iSegmentArray.Element[0] != debug1)
					continue;
			}
		}

			// Classify pObject_.	

			//QLIST::CopyToArray(&(pObject_->surfelList), &iSurfelArray);

			//RVLMEM_ALLOC_STRUCT(pMem, RECOG::VN_::Hypothesis, pHypothesis);

			//RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pVNClassifier->sampledUnitSphere.h, pHypothesis->d);

			//RVLMEM_ALLOC_STRUCT_ARRAY(pMem, uchar, pVNClassifier->sampledUnitSphere.h, pHypothesis->bd);		

			//pHypothesis->iClass = -1;

			//pHypothesis->cost = pVNClassifier->FitCTI(&mesh, iSurfelArray, pObject_->iVertexArray, camera, 10, pHypothesis->R, pHypothesis->d, 
			//	pHypothesis->bd);

			if (!pVNClassifier->Classify(&mesh, pObjects, &iConvexObjects, iSegmentArray, pVNClassifier->fitData, pHypothesisList, pVNClassifier->D,
				pVNClassifier->bD, pVNClassifier->f, pVNClassifier->R, J, pMem))
				continue;
			//int debug = 0;

			nSegmentCombinations++;

			timer.Stop();

			double tCTIClassification = timer.GetTime();

			// Alignment visualization in Matlab using RVLVNFitCTIDebug.m. (Only for debugging purpose!)

			//float *NCH = new float[3 * pVNClassifier->convexHull.iVisibleFaces.n];

			//float *NSrc, *NTgt;
			//MESH::Face *pFace;

			//for (int i = 0; i < pVNClassifier->convexHull.iVisibleFaces.n; i++)
			//{
			//	pFace = pVNClassifier->convexHull.faces.Element[pVNClassifier->convexHull.iVisibleFaces.Element[i]];

			//	NSrc = pFace->N;

			//	NTgt = NCH + 3 * i;

			//	RVLCOPY3VECTOR(NSrc, NTgt);

			//	RVLSCALE3VECTOR(NTgt, pFace->Area, NTgt);
			//}

			//FILE *fpNCH = fopen(".\\NCH.txt", "w");

			//PrintMatrix<float>(fpNCH, NCH, pVNClassifier->convexHull.iVisibleFaces.n, 3);

			//fclose(fpNCH);

			//delete[] NCH;

			//FILE *fpR = fopen(".\\R.txt", "w");

			//RECOG::VN_::Hypothesis *pHypothesis = pHypothesisList->pFirst;

			//while (pHypothesis)
			//{
			//	PrintMatrix<float>(fpR, pHypothesis->R, 3, 3);

			//	pHypothesis = pHypothesis->pNext;
			//}

			//fclose(fpR);

			///

			if (visualizationData.bVisualizeConvexHulls)
			{
				//convexHull.iVisibleFaces.Element[2] = convexHull.iVisibleFaces.Element[convexHull.iVisibleFaces.n - 1];
				//convexHull.iVisibleFaces.n--;

				pVNClassifier->convexHull.CreateVTKPolyData(NULL, NULL, true);

				visualizer.AddMesh(pVNClassifier->convexHull.pPolygonData);
			}

			//pVNClassifier->ComputeDescriptors(pHypothesis->d, D, bD);

			// Only for debugging purpose!

			//if (iObject_ == 1)
			//{
			//	printf("iR=%d\n", iR);

			//	for (int i = 0; i < pClass->M.h; i++)
			//		printf("%f\t", q[i]);

			//	printf("\n");

			//	pVNClassifier->SaveDescriptors(fp, pObject_->d, pObject_->bd, iObject_, RVLVN_CLUSTER_TYPE_CONVEX);
			//}

			//pVNClassifier->SaveDescriptors(fp, D, bD, nDescriptors, iObject_, RVLVN_CLUSTER_TYPE_CONVEX);

			/// Create hypothesis.

			// Add hypothesis to the hypothesis list.

			if (bHypothesisLog)
			{
				fprintf(fpHypothesisLog, "Object %d", iSegmentArray.Element[0]);

				for (k = 1; k < iSegmentArray.n; k++)
					fprintf(fpHypothesisLog, "+%d", iSegmentArray.Element[k]);

				fprintf(fpHypothesisLog, "\n");
			}

			pHypothesis = hypothesisList.pFirst;

			while (pHypothesis)
			{
				if (iSegmentArray.n == 1)
					pHypothesis->cost += appearanceCost;

				hypotheses.push_back(pHypothesis);

				if (bHypothesisLog)
					fprintf(fpHypothesisLog, "Class %d E=%f \n", pHypothesis->iClass, pHypothesis->cost);

				pHypothesis = pHypothesis->pNext;
			}

			if (bHypothesisLog)
				fprintf(fpHypothesisLog, "\n");

			// Only for debugging purpose!

			//pVNClassifier->SaveLatentVector(fpInstances, q, 0);

			///

			//if (iObject_ == 1)
			//{
			//	FILE *fpVertices = fopen((std::string(resultsFolder) + "\\vertices.txt").data(), "w");

			//	float *P;

			//	for (int i = 0; i < pObject_->iVertexArray.n; i++)
			//	{
			//		P = pSurfels->vertexArray.Element[pObject_->iVertexArray.Element[i]]->P;

			//		fprintf(fpVertices, "%f\t%f\t%f\n", P[0], P[1], P[2]);
			//	}

			//	fclose(fpVertices);
			//}

			///
		//}	// for every object with index lower than iObject_.
	}	// for every object.

	fclose(fp);
	fclose(fpInstances);

	// Interpretation.

	RVL_DELETE_ARRAY(pVNClassifier->hypotheses.Element);

	pVNClassifier->hypotheses.n = hypotheses.size();

	pVNClassifier->hypotheses.Element = new RECOG::VN_::Hypothesis *[pVNClassifier->hypotheses.n];

	for (i = 0; i < pVNClassifier->hypotheses.n; i++)
		pVNClassifier->hypotheses.Element[i] = (RECOG::VN_::Hypothesis *)(hypotheses[i]);

	pVNClassifier->Interpretation();

	RVL_DELETE_ARRAY(objectMap.Element);

	// Save hypotheses to a file.

	if (bHypothesisLog)
	{
		fprintf(fpHypothesisLog, "\n");

		int iHypothesis;

		for (i = 0; i < pVNClassifier->sortedHypotheses.n; i++)
		{
			iHypothesis = pVNClassifier->sortedHypotheses.Element[i].idx;

			pHypothesis = (RECOG::VN_::Hypothesis *)(hypotheses[iHypothesis]);

			fprintf(fpHypothesisLog, "%03d: Segment", iHypothesis);

			for (j = 0; j < pHypothesis->iSegmentArray.n; j++)
				fprintf(fpHypothesisLog, " %d", pHypothesis->iSegmentArray.Element[j]);

			fprintf(fpHypothesisLog, " Class %d size=%d Cost: shape=%f convexity=%f outliers=%d total=%f\n",
				pHypothesis->iClass, pHypothesis->support, pHypothesis->shapeCost, pHypothesis->convexityError, pHypothesis->nOutliers,
				pHypothesis->cost);
		}

		fprintf(fpHypothesisLog, "\n\n");

		for (i = 0; i < pVNClassifier->interpretation.n; i++)
		{
			iHypothesis = pVNClassifier->interpretation.Element[i];

			pHypothesis = pVNClassifier->hypotheses.Element[iHypothesis];

			fprintf(fpHypothesisLog, "%03d: Segment", iHypothesis);

			for (j = 0; j < pHypothesis->iSegmentArray.n; j++)
				fprintf(fpHypothesisLog, " %d", pHypothesis->iSegmentArray.Element[j]);

			fprintf(fpHypothesisLog, " Class %d size=%d Cost: shape=%f convexity=%f outliers=%d total=%f\n",
				pHypothesis->iClass, pHypothesis->support, pHypothesis->shapeCost, pHypothesis->convexityError, pHypothesis->nOutliers,
				pHypothesis->cost);
		}

		fclose(fpHypothesisLog);
	}

	// Pixel-level image segmentation.

	pVNClassifier->ImageSegmentation(&mesh, objectMap);

	hypotheses.clear();

	for (i = 0; i < pVNClassifier->interpretation.n; i++)
		hypotheses.push_back(pVNClassifier->hypotheses.Element[pVNClassifier->interpretation.Element[i]]);

	// Display convex hulls.

	if (visualizationData.bVisualizeConvexHulls)
	visualizer.Run();

	// Free memory.

	delete[] iSegmentArray.Element;
}


void ObjectDetector::Evaluate(
	FILE *fp,
	char *fileName,
	char *selectedGTObjectsFileName)
{
	if (!(flags & RVLOBJECTDETECTION_FLAG_SEGMENTATION_GT))
		return;

#ifdef RVLSURFEL_IMAGE_ADJACENCY
	if (bSegmentToObjects)
	{	
		int E[2];
		int N = 0;

		if (bSurfelsFromSSF)
			pObjects->CalculateOverAndUnderSegmentation_SSF(E, N, true, false);
		else
		{			
			if (selectedGTObjectsFileName)
			{
				std::vector<SURFEL::ObjectCoverage> selectedGTObjectCoverage;

				pObjects->CalculateOverAndUnderSegmentation(E, N, true, std::string(fileName), false, std::string(selectedGTObjectsFileName), &selectedGTObjectCoverage);
				/*std::string imageName = fileName;
				imageName.erase(imageName.find_last_of("."));
				std::string depthImgFileName = imageName + "d.png";
				std::string labelImgFileName = imageName + "a.png";
				std::string segLabelImgFileName = imageName + "LCCPLabels.png";
				pObjects->CalculateOverAndUnderSegmentation_Img(E, N, segLabelImgFileName, labelImgFileName, depthImgFileName, true, false, std::string(selectedGTObjectsFileName), &selectedGTObjectCoverage);*/

				FILE *fpSelectedGTObjectCoverage = fopen("selected_GT_object_coverage.txt", "a");

				for (int i = 0; i < selectedGTObjectCoverage.size(); i++)
					fprintf(fpSelectedGTObjectCoverage, "%d\t%d\t%f\n", selectedGTObjectCoverage.at(i).iObject,
					selectedGTObjectCoverage.at(i).type, selectedGTObjectCoverage.at(i).coverage);

				fclose(fpSelectedGTObjectCoverage);
			}
			else
				pObjects->CalculateOverAndUnderSegmentation(E, N, true, std::string(fileName), false);
		}
			
		std::cout << "Oversegmenation error: " << 100.0f * (1 - E[0] / (float)N) << "%" << std::endl;
		std::cout << "Undersegmenation error: " << 100.0f * E[1] / (float)N << "%" << std::endl;

		if (fp)
			fprintf(fp, "%s\t%d\t%d\t%d\n", fileName, E[0], E[1], N);
	}
#endif
}

void ObjectDetector::Evaluate3DReconstruction(
	ReconstructionEval *pReconstructionEval,
	int &TP,
	int &FP,
	int &FN,
	Array3D<uchar> *pGrid)
{
	//printf("Evaluation of 3D reconstruction:\n");

	VNClassifier *pVNClassifier = (VNClassifier *)vpVNClassifier;

	Array3D<uchar> grid = pReconstructionEval->grid;
	float resolution = pReconstructionEval->resolution;
	float *RST = pReconstructionEval->cameraPose.R;
	float *tST = pReconstructionEval->cameraPose.t;

	int nCells = grid.a * grid.b * grid.c;

	if (pGrid->Element == NULL)
	{
		*pGrid = grid;
		pGrid->Element = new uchar[nCells];
	}

	memset(pGrid->Element, 0, nCells * sizeof(uchar));

	float tMS[3];
	RVLNULL3VECTOR(tMS);

	int nCTI = pVNClassifier->sampledUnitSphere.h;

	float *N_ = pVNClassifier->sampledUnitSphere.Element;

	int i, j, k, l, iCell;
	float *a;
	int iHypothesis;
	RECOG::VN_::Hypothesis *pHypothesis;
	RECOG::ClassData *pClass;
	float *d, *N;
	float PT[3], PM[3], V3Tmp[3], RMT[9], tMT[3];

	for (iHypothesis = 0; iHypothesis < hypotheses.size(); iHypothesis++)
	{
		//printf("Hypothesis %d\/%d\n", iHypothesis, hypotheses.size());

		pHypothesis = (RECOG::VN_::Hypothesis *)(hypotheses[iHypothesis]);

		if (pHypothesis->iClass >= 0)
		{
			pClass = pVNClassifier->classArray.Element + pHypothesis->iClass;

			d = new float[pClass->M.w];

			RVLMULMXTVECT(pClass->M.Element, pHypothesis->q, pClass->M.h, pClass->M.w, d, i, j, a);
		}
		else
		{
			d = new float[nCTI];

			memcpy(d, pHypothesis->d, nCTI * sizeof(float));
		}

		RVLCOMPTRANSF3D(RST, tST, pHypothesis->R, tMS, RMT, tMT);

		iCell = 0;

		for (k = 0; k < grid.c; k++)
			for (j = 0; j < grid.b; j++)
				for (i = 0; i < grid.a; i++, iCell++)
				{
					PT[0] = resolution * (float(i));
					PT[1] = resolution * (float(j));
					PT[2] = resolution * (float(k));

					RVLINVTRANSF3(PT, RMT, tMT, PM, V3Tmp);

					for (l = 0; l < nCTI; l++)
					{
						N = N_ + 3 * l;

						if (RVLDOTPRODUCT3(N, PM) - d[l] > 0.0f)
							break;
					}

					if (l >= nCTI)
						pGrid->Element[iCell] = 1;
				}

		delete[] d;
	}

	TP = FP = FN = 0;

	for (iCell = 0; iCell < nCells; iCell++)
		if (grid.Element[iCell] == 1)
		{
			if (pGrid->Element[iCell] == 1)
				TP++;
			else
				FN++;
		}
		else if (pGrid->Element[iCell] == 1)
			FP++;
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

void ObjectDetector::GroundTruthGroundPlane()
{
	int iLargestBackgroundSurfel = -1;

	float largestBackgroundSurfelSize = 0.0f;

	pPSGM->iGndObject = -1;

	int iSurfel;
	Surfel *pSurfel;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->ObjectID == 0)
		{
			if (pPSGM->iGndObject < 0)
				pPSGM->iGndObject = iSurfel;

			if (pSurfel->size > largestBackgroundSurfelSize)
			{
				largestBackgroundSurfelSize = pSurfel->size;
				iLargestBackgroundSurfel = iSurfel;			
			}
		}
	}

	if (iLargestBackgroundSurfel < 0)
		return;

	pSurfel = pSurfels->NodeArray.Element + iLargestBackgroundSurfel;

	pPSGM->bGnd = true;

	RVLCOPY3VECTOR(pSurfel->N, pPSGM->NGnd);
	pPSGM->dGnd = pSurfel->d;	
}

void ObjectDetector::SaveBoundingBoxSizes(char *imageFileName)
{
	FILE *fpBoundingBoxes = fopen("bounding_boxes.txt", "a");

	int i;
	RECOG::PSGM_::ModelInstance *pBoundingBox;
	float size[3];

	for (i = 0; i < boundingBoxes.pCTI.n; i++)
	{
		pBoundingBox = boundingBoxes.pCTI.Element[i];

		pPSGM->BoundingBoxSize(pBoundingBox, size);

		fprintf(fpBoundingBoxes, "%s\t%d\t%f\t%f\t%f\n", imageFileName, pBoundingBox->iCluster, size[0], size[1], size[2]);
	}		

	fclose(fpBoundingBoxes);
}

void ObjectDetector::TrainingHMI(char *meshFileName)
{
	char *RGBFileName = RVLCreateFileName(meshFileName, ".ply", -1, ".png");

	cv::Mat RGB = cv::imread(RGBFileName);

	cv::imshow(cv::String(RGBFileName), RGB);

	TrainingHMIData data;

	data.pObjectDetector = this;
	data.RGB = RGB;
	data.imageName = RGBFileName;
	data.pObject = data.pObject2 = NULL;

	cvSetMouseCallback(RGBFileName, TrainingHMIMouseCallback, &data);

	QList<GRAPH::HierarchyNode> *pHierarchy = &(pObjects->hierarchy);

	uchar color[] = { 0, 255, 255 };

	int key;
	GRAPH::HierarchyNode *pNode, *pNode_;
	cv::Mat RGB_;

	do
	{		
		key = cv::waitKey();

		switch (key){
		case 'm':
			if (data.pObject != NULL && data.pObject2 != NULL && data.pObject != data.pObject2)
			{
				RVLMEM_ALLOC_STRUCT(pMem, GRAPH::HierarchyNode, pNode);

				RVLQLIST_ADD_ENTRY(pHierarchy, pNode);

				data.pObject->pParent = pNode;

				pNode->pChild[0] = data.pObject;

				data.pObject2->pParent = pNode;

				pNode->pChild[1] = data.pObject2;

				pNode->pParent = NULL;

				data.pObject = pNode;

				data.pObject2 = NULL;

				RGB.copyTo(RGB_);

				DisplaySelectedObject(data.pObject, color, RGB_);

				cv::imshow(RGBFileName, RGB_);
			}

			break;
		case 's':
			pNode = data.pObject->pChild[0];

			if (pNode)
			{
				pNode_ = data.pObject->pChild[1];

				pNode->pParent = pNode_->pParent = NULL;

				GRAPH::HierarchyNode *pNewSelectionNode = GetObject(data.iPix);

				if (pNode == pNewSelectionNode)
				{
					data.pObject = pNode;
					data.pObject2 = pNode_;
				}
				else
				{
					data.pObject = pNode_;
					data.pObject2 = pNode;
				}
			}
			else
				data.pObject2 = NULL;

			RGB.copyTo(RGB_);

			DisplaySelectedObject(data.pObject, color, RGB_);
			DisplaySelectedObject(data.pObject2, color, RGB_);

			cv::imshow(RGBFileName, RGB_);
		}
	} while (key != 27);
	
	delete[] RGBFileName;
}

void ObjectDetector::Display(Visualizer *pVisualizer)
{
	if (pObjects->objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_VN)
	{
		VNClassifier *pVNClassifier = (VNClassifier *)vpVNClassifier;

		float R[9];

		RVLUNITMX3(R);

		float t[3];

		RVLNULL3VECTOR(t);

		unsigned char pointColor[] = { 0, 128, 255 };

		SURFEL::SceneSamples sceneSamples;

		sceneSamples.PGnd = NULL;

		Array<OrientedPoint> PtArray;

		pPSGM->convexTemplate = pPSGM->convexTemplate66;

		FILE *fpDebug = fopen("C:\\RVL\\Debug\\d.txt", "w");

		int i, j;
		float *a;
		int iHypothesis;
		RECOG::VN_::Hypothesis *pHypothesis;
		VN *pModel;
		RECOG::ClassData *pClass;
		float *d;

		for (iHypothesis = 0; iHypothesis < hypotheses.size(); iHypothesis++)
		{
			pHypothesis = (RECOG::VN_::Hypothesis *)(hypotheses[iHypothesis]);

			if (pHypothesis->iClass >= 0)
			{
				pClass = pVNClassifier->classArray.Element + pHypothesis->iClass;

				d = new float[pClass->M.w];

				RVLMULMXTVECT(pClass->M.Element, pHypothesis->q, pClass->M.h, pClass->M.w, d, i, j, a);
			}
			else
			{
				d = new float[pVNClassifier->sampledUnitSphere.h];

				memcpy(d, pHypothesis->d, pVNClassifier->sampledUnitSphere.h * sizeof(float));
			}
			
			for (i = 0; i < 66; i++)
				fprintf(fpDebug, "%f ", d[i]);

			fprintf(fpDebug, "\n");

			// Display projected VN model.

			//pModel = pVNClassifier->models[pClass->iMetaModel];

			//pSurfels->SampleSurfelSet(&mesh, pHypothesis->iSurfelArray, pHypothesis->iVertexArray, camera, sceneSamples);	

			//PtArray.n = sceneSamples.imagePtArray.h;

			//PtArray.Element = new OrientedPoint[PtArray.n];

			//pModel->Project(d, pHypothesis->R, t, camera, sceneSamples.imagePtArray, PtArray);

			//Array<Point> PtArray_;

			//MESH::CreatePointArrayFromOrientedPointArray(PtArray, PtArray_, 2.0f);

			//pVisualizer->DisplayPointSet<float, Point>(PtArray_, pointColor, 6.0f);

			//delete[] PtArray.Element;
			//delete[] PtArray_.Element;
			//SURFEL::DeleteSceneSamples(sceneSamples);

			// Display CTI.

			double *color = NULL;

#ifdef RVLOBJECTDETECTION_DISPLAY_ECMR19
			double color_[3] = {1.0, 1.0, 1.0};

			color = color_;
#endif

			pPSGM->DisplayCTI(pVisualizer, pVNClassifier->sampledUnitSphere.Element, pVNClassifier->sampledUnitSphere.h, 
				d, pHypothesis->R, t, NULL, color);

			// Display projected convex hull (+ concavity).

#ifndef RVLOBJECTDETECTION_DISPLAY_ECMR19
			pHypothesis->pVN->Display(pVisualizer, &mesh, pSurfels, pHypothesis->iSurfelArray, pHypothesis->iVertexArray,
				pHypothesis->d, R, pointColor, camera);
#endif

			delete[] d;
		}

		fclose(fpDebug);
	}
}

void ObjectDetector::DisplayObjectMap()
{
	if (pObjects->objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_VN)
	{
		VNClassifier *pVNClassifier = (VNClassifier *)vpVNClassifier;

		uchar *color = new uchar[3 * pVNClassifier->interpretation.n];

		uchar *color_ = color;

		int iHypothesis;

		for (iHypothesis = 0; iHypothesis < pVNClassifier->interpretation.n; iHypothesis++, color_ += 3)
		{
			color_[0] = rand() % 180;
			color_[1] = rand() % 256;
			color_[2] = 255;
		}

		cv::Mat HSVObjectMap(camera.h, camera.w, CV_8UC3);

		int nPix = camera.h * camera.w;

		uchar *pix = HSVObjectMap.data;

		int iPix;

		for (iPix = 0; iPix < nPix; iPix++, pix += 3)
		{
			if (objectMap.Element[iPix] >= 0)
			{
				color_ = color + 3 * objectMap.Element[iPix];

				RVLCOPY3VECTOR(color_, pix);
			}
			else
				RVLSET3VECTOR(pix, 0, 0, 0);
		}

		delete[] color;

		cv::Mat BGRObjectMap(camera.h, camera.w, CV_8UC3);

		cv::cvtColor(HSVObjectMap, BGRObjectMap, cv::COLOR_HSV2BGR);

		cv::imshow("Object Map", BGRObjectMap);

		cv::waitKey(10);
	}
}

void ObjectDetector::SaveObjectMapLabelImg(std::string filename)
{
	if (pObjects->objectAggregationLevel2Method == RVLPCSEGMENT_OBJECT_AGGREGATION_LEVEL2_METHOD_VN)
	{
		cv::Mat ObjectMapImg(camera.h, camera.w, CV_8UC1, cv::Scalar::all(0));

		int nPix = camera.h * camera.w;

		uchar *pix = ObjectMapImg.data;

		int iPix;

		for (iPix = 0; iPix < nPix; iPix++, pix++)
		{
			if (objectMap.Element[iPix] >= 0)
				*pix = objectMap.Element[iPix] + 1;	//Labels start from 1
		}

		//Save image (preferable as png)
		cv::imwrite(filename, ObjectMapImg);
	}
}

void ObjectDetector::DisplaySelectedObject(
	GRAPH::HierarchyNode *pObject,
	uchar *color,
	cv::Mat RGB)
{
	if (pObject == NULL)
		return;

	int w = RGB.cols;
	int h = RGB.rows;

	bool *bSelectedObject = new bool[pSurfels->NodeArray.n];

	memset(bSelectedObject, 0, pSurfels->NodeArray.n * sizeof(bool));

	GRAPH::HierarchyNode **nodeBuff = new GRAPH::HierarchyNode *[pSurfels->NodeArray.n];

	GRAPH::HierarchyNode **pNodeFetch, **pNodePut;

	pNodePut = pNodeFetch = nodeBuff;

	*(pNodePut++) = pObject;

	GRAPH::HierarchyNode *pNode;

	while (pNodeFetch < pNodePut)
	{
		pNode = *(pNodeFetch++);

		if (pNode->pChild[0])
		{
			*(pNodePut++) = pNode->pChild[0];
			*(pNodePut++) = pNode->pChild[1];
		}
		else
			bSelectedObject[pNode->iElement] = true;
	}
	
	int i;
	int iSurfel;
	Surfel *pSurfel;
	QLIST::Index2 *pPtIdx;
	uchar *pix;
	int u, v, du, dv, u0, v0, du_;

	for (pNodeFetch = nodeBuff; pNodeFetch < pNodePut; pNodeFetch++)
	{
		pNode = *pNodeFetch;

		if (pNode->pChild[0] == NULL)
		{
			pSurfel = pSurfels->NodeArray.Element + pNode->iElement;

			pPtIdx = pSurfel->PtList.pFirst;

			while (pPtIdx)
			{
				u0 = pPtIdx->Idx % w;
				v0 = pPtIdx->Idx / w;

				du = 1;
				dv = 0;

				for (i = 0; i < 4; i++)
				{
					u = u0 + du;
					v = v0 + dv;

					if (u >= 0 && u < w && v >= 0 && v < h)
					{
						iSurfel = pSurfels->surfelMap[u + v * w];

						if (iSurfel < 0)
							break;

						if (!bSelectedObject[iSurfel])
							break;
					}

					du_ = du;
					du = dv;
					dv = -du_;
				}

				if (i < 4)
				{
					pix = RGB.data + 3 * u0 + v0 * RGB.step;

					RVLCOPY3VECTOR(color, pix);
				}

				pPtIdx = pPtIdx->pNext;
			}	// for every surfel pixel
		}	// if pNode is a surfel node
	}	// for (pNodeFetch = nodeBuff; pNodeFetch < pNodePut; pNodeFetch++) 				

	delete[] bSelectedObject;
	delete[] nodeBuff;
}

void ObjectDetector::DisplayClustersInteractive(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	uchar *selectionColor)
{
	RVL_DELETE_ARRAY(visualizationData.clusterVisualizationData.clusterColor);

	RECOG::DisplayClustersInteractive(pVisualizer, SClusters, pMesh, pSurfels, pSurfelDetector, clusterMap, selectionColor, 
		visualizationData.clusterVisualizationData.clusterColor, &(visualizationData.clusterVisualizationData));

	pSurfels->DisplayVertices();
}

float ObjectDetector::CalculateIntersectionOverUnion(std::string GTfilename, Array<int> iSegmentArray)
{
	float IOU = 0;

	//load GT files
	cv::Mat GTLabImg = cv::imread(GTfilename, cv::ImreadModes::IMREAD_ANYDEPTH);

	//Find maximum label
	double dMaxLab;
	double dMinLab;
	cv::minMaxLoc(GTLabImg, &dMinLab, &dMaxLab);
	int maxGTLab = (int)dMaxLab;
	
	//Determine no of point for each GT object
	int *GTObjNoPix = new int[maxGTLab];
	memset(GTObjNoPix, 0, maxGTLab*sizeof(int));
	int x, y;
	int GTLabel;
	for (y = 0; y < GTLabImg.rows; y++)
	{
		for (x = 0; x < GTLabImg.cols; x++)
		{
			GTLabel = (int)GTLabImg.at<unsigned char>(y, x);
			if (GTLabel == 0)
				continue;
			GTObjNoPix[GTLabel - 1]++;	//-1 because idx starts at 0
		}
	}
	
	//Run through surfels
	int *overlap = new int[maxGTLab];
	memset(overlap, 0, maxGTLab*sizeof(int));
	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	Surfel *pCurrSurfel;
	QLIST::Index *piElement;
	RVL::QLIST::Index2 *pt;
	int noSegmentPix = 0;;
	for (int i = 0; i < iSegmentArray.n; i++)
	{
		pObject = pObjects->NodeArray.Element + iSegmentArray.Element[i];
		piElement = pObject->elementList.pFirst;

		while (piElement)
		{
			pCurrSurfel = pSurfels->NodeArray.Element + piElement->Idx;
			//check 
			if (!((pCurrSurfel->size <= 1) || pCurrSurfel->bEdge))
			{
				pt = pCurrSurfel->PtList.pFirst;
				//for all surfel's points
				for (int k = 0; k < pCurrSurfel->size; k++)
				{
					y = floor(pt->Idx / 640.0);		//hardcoded image width!!!
					x = floor(pt->Idx - 640.0 * y);	//hardcoded image width!!!
					if ((x > 639) || (y > 479))
						continue;

					GTLabel = (int)GTLabImg.at<unsigned char>(y, x);
					if (GTLabel == 0)
						continue;
					overlap[GTLabel - 1]++;	//-1 because idx starts at 0
					noSegmentPix++;

					pt = pt->pNext;
				}
			}
			piElement = piElement->pNext;
		}
	}

	//Find max overlap
	int maxIdx = 0;
	for (int i = 1; i < maxGTLab; i++)
	{
		if (overlap[i] > overlap[maxIdx])
			maxIdx = i;
	}

	//Compute IOU = overlap[maxIdx] / (ukupni_broj_piksela_objekta_idx + ukupni_broj_piksela_segmenta - overlap[maxIdx])
	IOU = overlap[maxIdx] / (float)(GTObjNoPix[maxIdx] + noSegmentPix - overlap[maxIdx]);

	delete[] GTObjNoPix;
	delete[] overlap;

	return IOU;
}

void OBJECT_DETECTION::TrainingHMIMouseCallback(int event, int x, int y, int flags, void* vpData)
{
	TrainingHMIData *pData = (TrainingHMIData *)vpData;

	ObjectDetector *pObjectDetector = pData->pObjectDetector;

	int w = pData->RGB.cols;

	pData->iPix = x + y * w;

	uchar color[] = { 0, 255, 255 };

	cv::Mat RGB;

	switch (event){
	case CV_EVENT_LBUTTONDOWN:
		pData->RGB.copyTo(RGB);

		pData->pObject = pObjectDetector->GetObject(pData->iPix);

		pData->pObject2 = NULL;

		if (pData->pObject)
			pObjectDetector->DisplaySelectedObject(pData->pObject, color, RGB);

		cv::imshow(pData->imageName, RGB);

		break;
	case CV_EVENT_RBUTTONDOWN:
		if (pData->pObject)
		{
			pData->RGB.copyTo(RGB);

			pObjectDetector->DisplaySelectedObject(pData->pObject, color, RGB);

			pData->pObject2 = pObjectDetector->GetObject(pData->iPix);

			if (pData->pObject2)
				pObjectDetector->DisplaySelectedObject(pData->pObject2, color, RGB);

			cv::imshow(pData->imageName, RGB);
		}
	}	// switch (event)
}

GRAPH::HierarchyNode * ObjectDetector::GetObject(int iPix)
{
	int iSurfel = pSurfels->surfelMap[iPix];

	if (iSurfel < 0)
		return NULL;
	
	GRAPH::HierarchyNode *pNode = pObjects->hierarchy.pFirst;

	while (pNode)
	{
		if (pNode->iElement == iSurfel)
			break;

		pNode = pNode->pNext;
	}

	if (pNode == NULL)
		return NULL;

	while (true)
	{
		if (pNode->pParent == NULL)
			return pNode;

		pNode = pNode->pParent;
	}

	return NULL;
}