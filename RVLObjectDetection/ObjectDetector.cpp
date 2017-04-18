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

	nMultilateralFilterIterations = 10;
	joinSmallObjectsToLargestNeighborSizeThr = 1000;
	joinSmallObjectsToLargestNeighborDistThr = 0.020f;

	bSegmentToObjects = false;
	bObjectAggregationLevel2 = false;
	bCTIBasedObjectAggregation = false;
	bMultilateralFilter = false;
	bJoinSmallObjectsToLargestNeighbor = false;

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
	pParamData = ParamList.AddParam("ObjectDetector.multilateralFilterIterations", RVLPARAM_TYPE_INT, &nMultilateralFilterIterations);
	pParamData = ParamList.AddParam("ObjectDetector.joinSmallObjectsToLargestNeighborSizeThr", RVLPARAM_TYPE_INT, &joinSmallObjectsToLargestNeighborSizeThr);
	pParamData = ParamList.AddParam("ObjectDetector.joinSmallObjectsToLargestNeighborDistThr", RVLPARAM_TYPE_FLOAT, &joinSmallObjectsToLargestNeighborDistThr);
	pParamData = ParamList.AddParam("ObjectDetector.multilateralFilter", RVLPARAM_TYPE_BOOL, &bMultilateralFilter);
	pParamData = ParamList.AddParam("ObjectDetector.joinSmallObjectsToLargestNeighbor", RVLPARAM_TYPE_BOOL, &bJoinSmallObjectsToLargestNeighbor);
}

//Dirk Holz and Sven Behnke: "Approximate Triangulation and Region Growing for Efficient Segmentation and Smoothing of Range Images"
//NOT DEBUGGED
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

		//SmoothMesh(&mesh, 30);
		//LaplaceSmooting(&mesh, 30);
		if (bMultilateralFilter)
			MultilateralSmoothMesh(&mesh, nMultilateralFilterIterations, false, false, true);
		

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
			cv::imshow("Level1", pObjects->CreateSegmentationImage());
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
			pObjects->DetermineObjectConvexityData(convexityThr, 0.15, false);
			pObjects->vpObjectAggregationLevel2CriterionData = this;
			pObjects->ExtFuncCheckIfWithinVolume = &RVL::ObjectDetector::CheckIfWithinCTIBoundingBox;
			pObjects->ObjectAggregationLevel2_ViaObjectPairConvexity(convexityThr, convexityRatioThr1, convexityRatioThr2, pObjects->minObjectSize, false);
			cv::imshow("Level2", pObjects->CreateSegmentationImage());
			if (bJoinSmallObjectsToLargestNeighbor)
			{
				pObjects->MergeSmallObjects(joinSmallObjectsToLargestNeighborSizeThr, joinSmallObjectsToLargestNeighborDistThr);
			cv::imshow("level2 + merge small objects", pObjects->CreateSegmentationImage());
			cv::waitKey(1);
			}

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

