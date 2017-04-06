// RVLObjectDetectionDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Util.h"
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
#include "PSGM.h"
#include "ObjectDetector.h"
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"
#include "vtkOBBTree.h"
#include "vtkLine.h"

using namespace RVL;

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	char **pSequenceFileName,
	char **pSegmentationResultsFileName,
	bool &b3DVisualization,
	bool &b2DVisualization)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("MeshFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("SequenceFileName", RVLPARAM_TYPE_STRING, pSequenceFileName);
	pParamData = pParamList->AddParam("SegmentationResultsFileName", RVLPARAM_TYPE_STRING, pSegmentationResultsFileName);
	pParamData = pParamList->AddParam("Visualization.3D", RVLPARAM_TYPE_BOOL, &b3DVisualization);
	pParamData = pParamList->AddParam("Visualization.2D", RVLPARAM_TYPE_BOOL, &b2DVisualization);
}

//void VisualizeSurfelNormals(Visualizer *vis, SurfelGraph* pSurfels)
//{
//	Surfel *pSurfel;
//	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//	points->SetDataTypeToDouble();
//	vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
//	normals->SetNumberOfComponents(3);
//	normals->SetName("Normals");
//	for (int i = 0; i < pSurfels->NodeArray.n; i++)
//	{
//		pSurfel = pSurfels->NodeArray.Element + i;
//		if ((pSurfel->size < 2) || pSurfel->bEdge)
//			continue;
//
//		points->InsertNextPoint(pSurfel->P);
//		normals->InsertNextTuple(pSurfel->N);
//	}
//	vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
//	pd->SetPoints(points);
//	pd->GetPointData()->AddArray(normals);
//	pd->GetPointData()->SetActiveNormals("Normals");
//
//	vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();
//
//	vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
//	glyph3D->SetSourceConnection(arrowSource->GetOutputPort());
//	glyph3D->SetVectorModeToUseNormal();
//	glyph3D->SetInputData(pd);
//	glyph3D->SetScaleFactor(0.03);
//	glyph3D->Update();
//
//	vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();
//	mapper->SetInputConnection(glyph3D->GetOutputPort());
//	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
//	actor->SetMapper(mapper);
//	vis->renderer->AddActor(actor);
//}

void MeshSmoothTest(vtkSmartPointer<vtkPolyData> pd)
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();;
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);

	//vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
	//smoothFilter->SetInputData(pd);
	//smoothFilter->SetNumberOfIterations(30);
	//smoothFilter->SetRelaxationFactor(1.0);
	////smoothFilter->FeatureEdgeSmoothingOn();
	////smoothFilter->SetFeatureAngle(45);
	////smoothFilter->SetEdgeAngle(60);
	//smoothFilter->BoundarySmoothingOn();
	//smoothFilter->Update();

	vtkSmartPointer<vtkWindowedSincPolyDataFilter> smoother = vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
	smoother->SetInputData(pd);
	smoother->SetNumberOfIterations(15);
	smoother->BoundarySmoothingOn();
	//smoother->FeatureEdgeSmoothingOn();
	//smoother->SetFeatureAngle(60.0);
	smoother->SetPassBand(0.1);
	smoother->NonManifoldSmoothingOn();
	smoother->NormalizeCoordinatesOn();
	smoother->Update();


	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(smoother->GetOutputPort());
	vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
	act->SetMapper(mapper);
	renderer->AddActor(act);

	//Start VTK
	renderer->ResetCamera();
	window->Render();
	interactor->Start();
}

void BilateralMeshTest(vtkSmartPointer<vtkPolyData> pd, int noIter = 1)
{
	// Initialize VTK.
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();;
	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	interactor->SetRenderWindow(window);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactor->SetInteractorStyle(style);
	renderer->SetBackground(0.5294, 0.8078, 0.9803);
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

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(norPD);
	vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
	act->SetMapper(mapper);
	renderer->AddActor(act);

	//Start VTK
	renderer->ResetCamera();
	window->Render();
	interactor->Start();
}

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

//vtkSmartPointer<vtkPolyData> LaplaceSmooting(vtkSmartPointer<vtkPolyData> inputPD)
//{
//	//generate a copy of input polydata
//	vtkSmartPointer<vtkPolyData> outputPD = vtkSmartPointer<vtkPolyData>::New();
//	outputPD->DeepCopy(inputPD);
//	//outputPD->BuildLinks();
//	//find boundary edges using vtkFeatureEdges filter
//	vtkSmartPointer<vtkFeatureEdges> boundaryEdges = vtkSmartPointer<vtkFeatureEdges>::New();
//	boundaryEdges->SetInputData(outputPD);
//	boundaryEdges->BoundaryEdgesOn();
//	boundaryEdges->FeatureEdgesOff();
//	boundaryEdges->ManifoldEdgesOff();
//	boundaryEdges->NonManifoldEdgesOff();
//	boundaryEdges->Update();
//	//running through generated lines
//	vtkSmartPointer<vtkIdList> llist = vtkSmartPointer<vtkIdList>::New();
//	int p1Idx;
//	int p2Idx;
//	size_t hashVal = 0;
//	std::set<size_t> edges;
//	vtkSmartPointer<vtkCellArray> lines = boundaryEdges->GetOutput()->GetLines();
//	lines->InitTraversal();
//	//over all lines (boundary edges)
//	for (int i = 0; i < lines->GetNumberOfCells(); i++)
//	{
//		llist->Reset();
//		lines->GetNextCell(llist);
//		//get sides
//		p1Idx = llist->GetId(0);
//		p2Idx = llist->GetId(1);
//		hashVal = 0;
//		//first the smaller value will be entered into hash then the larger
//		if (p1Idx < p2Idx)
//		{
//			hash_combine(hashVal, p1Idx);
//			hash_combine(hashVal, p2Idx);
//		}
//		else
//		{
//			hash_combine(hashVal, p2Idx);
//			hash_combine(hashVal, p1Idx);
//		}
//		//add to a list of edges
//		edges.insert(hashVal);
//	}
//
//	//Compile a list of points and its neighbours
//	std::vector<std::vector<int>> neighboorhoods;
//	int noPts = outputPD->GetNumberOfPoints();
//	neighboorhoods.resize(noPts);
//	vtkSmartPointer<vtkIdList> cellIdList =	vtkSmartPointer<vtkIdList>::New();
//	vtkSmartPointer<vtkIdList> pointIdList = vtkSmartPointer<vtkIdList>::New();
//	//First extract point edges
//	vtkSmartPointer<vtkExtractEdges> extractEdges =	vtkSmartPointer<vtkExtractEdges>::New();
//	extractEdges->SetInputData(outputPD);
//	extractEdges->Update();
//	vtkSmartPointer<vtkPolyData> extractEdgesPD = extractEdges->GetOutput();
//	for (int i = 0; i < noPts; i++)
//	{
//		cellIdList->Reset();
//		extractEdgesPD->GetPointCells(i, cellIdList);
//		for (int id = 0; id < cellIdList->GetNumberOfIds(); id++)
//		{
//			pointIdList->Reset();
//			extractEdgesPD->GetCellPoints(cellIdList->GetId(id), pointIdList);
//			if (pointIdList->GetId(0) != i)
//				neighboorhoods.at(i).push_back(pointIdList->GetId(0));
//			else
//				neighboorhoods.at(i).push_back(pointIdList->GetId(1));
//		}
//	}
//
//	//Calculate cotangent weights for each point to point edge
//	std::vector<std::vector<float>> neighboorhoods;
//
//	//Return output polydata
//	return outputPD;
//}

void LaplaceSmooting(Mesh *pMesh, int noIter)
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
	for (int iter = 0; iter < noIter; iter++)
	{
		for (int idx = 0; idx < pMesh->NodeArray.n; idx++)
		{
			//current point and edge list
			currPoint = pMesh->NodeArray.Element + idx;
			if ((currPoint->P[0] == 0.0) && (currPoint->P[1] == 0.0) && (currPoint->P[2] == 0.0))
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

				noPointEdges++;
				pEdgePtr = pEdgePtr->pNext;
			}

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
					cotan1 = v1v2Dot / sqrt(RVLDOTPRODUCT3(v1v2Cross, v1v2Cross));
					v3v4Dot = RVLDOTPRODUCT3(v3, v4);
					RVLCROSSPRODUCT3(v3, v4, v3v4Cross);
					cotan2 = v3v4Dot / sqrt(RVLDOTPRODUCT3(v1v2Cross, v3v4Cross));
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
						cotan1 = v1v2Dot / sqrt(RVLDOTPRODUCT3(v1v2Cross, v1v2Cross));
						//cotan1 = (v1.dot(v2)) / (v1.cross(v2)).norm();
					}
					else
					{
						// find the angle not on the boundary
						if (!boundaryEdges[id_next])
						{
							v3v4Dot = RVLDOTPRODUCT3(v3, v4);
							RVLCROSSPRODUCT3(v3, v4, v3v4Cross);
							cotan2 = v3v4Dot / sqrt(RVLDOTPRODUCT3(v1v2Cross, v3v4Cross));
							//cotan2 = (v3.dot(v4)) / (v3.cross(v4)).norm();
						}
						else
						{
							v1v2Dot = RVLDOTPRODUCT3(v1, v2);
							RVLCROSSPRODUCT3(v1, v2, v1v2Cross);
							cotan1 = v1v2Dot / sqrt(RVLDOTPRODUCT3(v1v2Cross, v1v2Cross));
							//cotan1 = (v1.dot(v2)) / (v1.cross(v2)).norm();
						}
					}
				}

				//neighboorhoodCoTangentW.at(i).push_back(cotan1 + cotan2);
				neighboorhoodCoTangentW[id_curr] = cotan1 + cotan2;
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
			destCurrPointD[0] = cog[0] / sum + currPointD[0];
			destCurrPointD[1] = cog[1] / sum + currPointD[1];
			destCurrPointD[2] = cog[2] / sum + currPointD[2];
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
}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	CRVLMem mem;	// cycle memory

	mem.Create(1000000000);

	// Read parameters from a configuration file.

	char cfgFileName[] = "RVLObjectDetectionDemo.cfg";

	char *MeshFileName = NULL;
	char *SequenceFileName = NULL;
	char *SegmentationResultsFileName = NULL;
	bool b3DVisualization, b2DVisualization;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList, &mem0, &MeshFileName, &SequenceFileName, &SegmentationResultsFileName, b3DVisualization, b2DVisualization);

	ParamList.LoadParams(cfgFileName);

	// Create mesh builder.

	PCLMeshBuilder meshBuilder;

	meshBuilder.CreateParamList(&mem0);

	meshBuilder.ParamList.LoadParams(cfgFileName);

	int w = 640;
	int h = 480;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));

	meshBuilder.PC = PC;

	// Initialize object detector.

	ObjectDetector objectDetector;

	objectDetector.pMem0 = &mem0;
	objectDetector.pMem = &mem;
	objectDetector.cfgFileName = RVLCreateString(cfgFileName);

	objectDetector.Init();

	objectDetector.vpMeshBuilder = &meshBuilder;
	objectDetector.LoadMesh = LoadMesh;

	objectDetector.pSurfelDetector->pTimer = new CRVLTimer;

	// Object detection.

	bool bSequence = (SequenceFileName != NULL) ? true : false;

	FILE *fp = (SegmentationResultsFileName ? fopen(SegmentationResultsFileName, "w") : NULL);

	if (fp)
		fprintf(fp, "Image\tE0\tE1\tN\n");

	if (bSequence)
	{
		//Run sequence
		FileSequenceLoader sceneSequence;
		sceneSequence.Init(SequenceFileName);

		char filePath[200];
		char fileName[200];

		while (sceneSequence.GetNext(filePath, fileName))
		{
			mem.Clear();

			printf("Scene %s...\n", fileName);

			objectDetector.DetectObjects(filePath);

			printf("Scene %s...finished!\n\n", fileName);

			objectDetector.Evaluate(fp, filePath);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectDetector.bSegmentToObjects)
			{
				std::string segmentationImageFileName(filePath);
				segmentationImageFileName.erase(segmentationImageFileName.find_last_of("."));
				segmentationImageFileName += "OGLabels.png";
				objectDetector.pObjects->SaveSegmentationLabelImg(segmentationImageFileName);

				//cv::imshow("Segmentation", objectDetector.pObjects->CreateSegmentationImage());
				cv::waitKey(1);
			}
#endif
		}
		system("pause");
	}
	else
	{

		objectDetector.DetectObjects(MeshFileName);

		objectDetector.Evaluate(fp, MeshFileName);

		//Smooth test
		//MeshSmoothTest(objectDetector.mesh.pPolygonData);
		//BilateralMeshTest(objectDetector.mesh.pPolygonData, 10);
		//LaplaceSmooting(&objectDetector.mesh, 5);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
		if (objectDetector.bSurfelsFromSSF)
		{
			if (objectDetector.bSegmentToObjects)
			{
				//Visualization
				cv::imshow("Colored surfel image", objectDetector.pSurfels->GenColoredSurfelImgFromSSF(objectDetector.pObjects->ssf));
				cv::imshow("Colored segmentation image", objectDetector.pObjects->CreateSegmentationImageFromSSF());
				cv::waitKey(1);
			}
		}
		else
#endif
		{
			// Display segmentation.

			unsigned char SelectionColor[3];

			SelectionColor[0] = 0;
			SelectionColor[1] = 255;
			SelectionColor[2] = 0;

			objectDetector.pSurfels->NodeColors(SelectionColor);

			Visualizer visualizer;

			visualizer.b2D = b2DVisualization;
			visualizer.b3D = b3DVisualization;

			visualizer.Create();
			objectDetector.pSurfels->InitDisplay(&visualizer, &(objectDetector.mesh), objectDetector.pSurfelDetector);
			//VisualizeSurfelNormals(&visualizer, objectDetector.pSurfels);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectDetector.bSegmentToObjects)
			{
				objectDetector.pObjects->InitDisplay(&visualizer, &(objectDetector.mesh), SelectionColor);
				objectDetector.pObjects->Display();
			}
			else
#endif
				objectDetector.pSurfels->Display(&visualizer, &(objectDetector.mesh));

#ifdef RVLSURFEL_IMAGE_ADJACENCY
			if (objectDetector.bSegmentToObjects)
			{
				std::string segmentationImageFileName(MeshFileName);
				segmentationImageFileName.erase(segmentationImageFileName.find_last_of("."));
				segmentationImageFileName += "OGLabels.png";
				objectDetector.pObjects->SaveSegmentationLabelImg(segmentationImageFileName);
			}
#endif
			// DEMO: common bounding box of objects 9 and 19.

			//RECOG::PSGM_::ModelInstance boundingBox;

			//boundingBox.modelInstance.Element = new RECOG::PSGM_::ModelInstanceElement[66];

			//objectDetector.BoundingBox(9, 19, &boundingBox);

			//objectDetector.pPSGM->convexTemplate = objectDetector.pPSGM->convexTemplateBox;

			//objectDetector.pPSGM->DisplayCTI(&visualizer, &boundingBox);

			//objectDetector.pPSGM->convexTemplate = objectDetector.pPSGM->convexTemplate66;

			//delete[] boundingBox.modelInstance.Element;

			// END DEMO

			/*objectDetector.pPSGM->convexTemplate = objectDetector.pPSGM->convexTemplateBox;

			objectDetector.pPSGM->DisplayCTIs(&visualizer, &(objectDetector.boundingBoxes));

			objectDetector.pPSGM->convexTemplate = objectDetector.pPSGM->convexTemplate66;*/

			//detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
			visualizer.Run();
		}

	}
		
	// Memory deallocation.

	if (fp)
		fclose(fp);

	RVL_DELETE_ARRAY(MeshFileName);
	RVL_DELETE_ARRAY(SequenceFileName);
	RVL_DELETE_ARRAY(SegmentationResultsFileName);

	return 0;
}

