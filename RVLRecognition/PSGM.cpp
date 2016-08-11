//#include "stdafx.h"

#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "RVLCore2.h"
#include "Graph.h"
#include <Eigen\Eigenvalues>
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "PCLMeshBuilder.h"
#include "RGBDCamera.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "PSGM.h"

using namespace RVL;

PSGM::PSGM()
{
	maxnClusters = 1;
	kNoise = 1.2f;

	clusters.Element = NULL;
	surfelVertexList.Element = NULL;
	surfelVertexMem = NULL;
	clusterMap = NULL;
	clusterSurfelMem = NULL;
	clusterVertexMem = NULL;
	vertexArray.Element = NULL;
}


PSGM::~PSGM()
{
	RVL_DELETE_ARRAY(clusters.Element);
	RVL_DELETE_ARRAY(surfelVertexList.Element);
	RVL_DELETE_ARRAY(surfelVertexMem);
	RVL_DELETE_ARRAY(clusterMap);
	RVL_DELETE_ARRAY(clusterSurfelMem);
	RVL_DELETE_ARRAY(clusterVertexMem);
	RVL_DELETE_ARRAY(vertexArray.Element);
}

void PSGM::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("PSGM.maxnClusters", RVLPARAM_TYPE_INT, &maxnClusters);
	pParamData = ParamList.AddParam("PSGM.kNoise", RVLPARAM_TYPE_FLOAT, &kNoise);
}

void PSGM::Interpret(
	Mesh *pMesh)
{
	// Create ordered mesh.

	pMesh->CreateOrderedMeshFromPolyData();

	// Detect surfels.

	pSurfels->Init(pMesh);

	pSurfelDetector->Init(pMesh, pSurfels, pMem);

	printf("Segmentation to surfels...");

	pSurfelDetector->Segment(pMesh, pSurfels);

	printf("completed.\n");

	int nSurfels = pSurfels->NodeArray.n;

	printf("No. of surfels = %d\n", nSurfels);

	// Detect vertices.

	QList<RECOG::PSGM_::Vertex> *pVertexList = &vertexList;

	RVLQLIST_INIT(pVertexList);

	int nVertexSurfelRelations = 0;

	int nVertices = 0;

	RVL_DELETE_ARRAY(surfelVertexList.Element);

	surfelVertexList.Element = new QList<QLIST::Index>[pSurfels->NodeArray.n];
	surfelVertexList.n = pSurfels->NodeArray.n;

	int iSurfel, iSurfel_, iPrevSurfel;
	int iBoundary;
	int iPointEdge;
	int iPt, iPt_;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_;
	MeshEdge *pEdge;
	QList<MeshEdgePtr> *pEdgeList;
	RECOG::PSGM_::Vertex *pVertex;
	Point *pPt;
	QList<QLIST::Index> *pSurfelVertexList;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		RVLQLIST_INIT(pSurfelVertexList);

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->size == 0)
			continue;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				pEdgePtr_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr_);

				pPt = pMesh->NodeArray.Element + iPt;

				pEdgeList = &(pPt->EdgeList);

				iPrevSurfel = -1;

				//int debug = 0;

				while (true)
				{					
					RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr_);

					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr_, pEdge, iPt_);

					iSurfel_ = pSurfels->surfelMap[iPt_];

					if (iSurfel_ == iSurfel)
						break;

					//debug++;

					//if (debug >= 20)
					//	debug = 0;

					if (iSurfel_ >= 0 && iSurfel_ < pMesh->NodeArray.n)
					{
						if (iPrevSurfel >= 0 && iPrevSurfel != iSurfel_)
						{
							if (iSurfel < iSurfel_ && iSurfel < iPrevSurfel)
							{
								RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::Vertex, pVertex);

								RVLCOPY3VECTOR(pPt->P, pVertex->P);

								RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, 3, pVertex->iSurfelArray.Element);

								pVertex->iSurfelArray.Element[0] = iSurfel;
								pVertex->iSurfelArray.Element[1] = iSurfel_;
								pVertex->iSurfelArray.Element[2] = iPrevSurfel;
								pVertex->iSurfelArray.n = 3;

								RVLQLIST_ADD_ENTRY(pVertexList, pVertex);

								nVertices++;

								nVertexSurfelRelations += 3;
							}
						}

						iPrevSurfel = iSurfel_;
					}	// if (iSurfel_ >= 0 && iSurfel_ < pMesh->NodeArray.n)					
				}	// for each neighboring point of the point iPt
			}	// for each point-edge on the boundary contour
		}	// for each boundary contour
	}	// for each surfel

	RVL_DELETE_ARRAY(vertexArray.Element);

	vertexArray.Element = new RECOG::PSGM_::Vertex *[nVertices];
	vertexArray.n = nVertices;

	QLIST::CreatePtrArray<RECOG::PSGM_::Vertex>(&vertexList, &vertexArray);

	// Assign vertices to surfels.

	RVL_DELETE_ARRAY(surfelVertexMem);

	surfelVertexMem = new QLIST::Index[nVertexSurfelRelations];

	QLIST::Index *pVertexIdx = surfelVertexMem;

	int iVertex = 0;

	pVertex = vertexList.pFirst;

	while (pVertex)
	{
		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			pSurfelVertexList = surfelVertexList.Element + pVertex->iSurfelArray.Element[iSurfel];

			RVLQLIST_ADD_ENTRY(pSurfelVertexList, pVertexIdx);

			pVertexIdx->Idx = iVertex;

			pVertexIdx++;
		}

		iVertex++;

		pVertex = pVertex->pNext;
	}

	nVertices = iVertex;

	///// Cluster surfels into convex surfaces.

	maxnClusters = pSurfels->NodeArray.n;

	RVL_DELETE_ARRAY(clusterMap);

	clusterMap = new int[pSurfels->NodeArray.n];

	memset(clusterMap, 0xff, pSurfels->NodeArray.n * sizeof(int));

	RVL_DELETE_ARRAY(clusters.Element);

	clusters.Element = new RECOG::PSGM_::Cluster[maxnClusters];
	clusters.n = 0;

	RVL_DELETE_ARRAY(clusterSurfelMem);

	clusterSurfelMem = new int[pSurfels->NodeArray.n];

	int *piSurfel = clusterSurfelMem;

	RVL_DELETE_ARRAY(clusterVertexMem);

	clusterVertexMem = new int[nVertexSurfelRelations];

	int *piVertex = clusterVertexMem;

	bool *bVertexVisited = new bool[vertexArray.n];	

	bool *bSurfelVisited = new bool[pSurfels->NodeArray.n];

	QList<QLIST::Index> candidateList;
	QList<QLIST::Index> *pCandidateList = &candidateList;

	QLIST::Index *candidateMem = new QLIST::Index[pSurfels->NodeArray.n];

	Array<RECOG::PSGM_::NormalHullElement> NHull;

	NHull.Element = new RECOG::PSGM_::NormalHullElement[pSurfels->NodeArray.n];

	RECOG::PSGM_::Cluster *pCluster;
	int iCluster;
	Surfel *pSurfel_;
	int maxSurfelSize;
	int iLargestSurfel;
	int iFirstNewVertex;
	QLIST::Index *pCandidateIdx, *pBestCandidateIdx;
	QLIST::Index **ppCandidateIdx, **ppBestCandidateIdx;
	float dist, minDist;

	for (iCluster = 0; iCluster < maxnClusters; iCluster++)
	{
		// pSurfel <- the largest surfel which is not assigned to a cluster.

		maxSurfelSize = 0;

		iLargestSurfel = -1;

		for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
		{
			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			if (pSurfel->size > maxSurfelSize)
			{
				if (clusterMap[iSurfel] < 0)
				{
					maxSurfelSize = pSurfel->size;

					iLargestSurfel = iSurfel;
				}
			}
		}

		if (iLargestSurfel < 0)
			break;

		// Initialize a new cluster.

		pCluster = clusters.Element + iCluster;

		pCluster->iSurfelArray.Element = piSurfel;
		pCluster->iVertexArray.Element = piVertex;

		pCluster->iSurfelArray.n = 0;
		pCluster->iVertexArray.n = 0;

		clusters.n++;

		memset(bVertexVisited, 0, vertexArray.n * sizeof(bool));
		memset(bSurfelVisited, 0, pSurfels->NodeArray.n * sizeof(bool));

		RVLQLIST_INIT(pCandidateList);

		QLIST::Index *pNewCandidate = candidateMem;

		NHull.n = 0;

		RVLQLIST_ADD_ENTRY(pCandidateList, pNewCandidate);

		pNewCandidate->Idx = iLargestSurfel;

		pNewCandidate++;

		bSurfelVisited[iLargestSurfel] = true;

		// Region growing.

		while (pCandidateList->pFirst)
		{
			// iSurfel <- the best candidate for expanding cluster.

			minDist = 2.0f;

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = *ppCandidateIdx;

			while (pCandidateIdx)
			{
				iSurfel_ = pCandidateIdx->Idx;

				pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

				dist = DistanceFromNormalHull(NHull, pSurfel_->N);

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

			// Add iSurfel to cluster.

			if (iSurfel == 56)
				int debug = 0;

			clusterMap[iSurfel] = iCluster;

			*(piSurfel++) = iSurfel;

			pCluster->iSurfelArray.n++;

			// Update normal hull.

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			UpdateNormalHull(NHull, pSurfel->N);

			// Add vertices of iSurfel, which are inside convex (or outside concave) surface into cluster.

			iFirstNewVertex = pCluster->iVertexArray.n;

			pSurfelVertexList = surfelVertexList.Element + iSurfel;

			pVertexIdx = pSurfelVertexList->pFirst;

			while (pVertexIdx)
			{
				if (!bVertexVisited[pVertexIdx->Idx])
				{
					bVertexVisited[pVertexIdx->Idx] = true;

					if (Inside(pVertexIdx->Idx, pCluster, iSurfel))
						*(piVertex++) = pVertexIdx->Idx;
				}

				pVertexIdx = pVertexIdx->pNext;
			}

			pCluster->iVertexArray.n = piVertex - pCluster->iVertexArray.Element;

			// Remove candidates which are not consistent with new vertices added to the cluster.

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = candidateList.pFirst;

			while (pCandidateIdx)
			{
				pSurfel_ = pSurfels->NodeArray.Element + pCandidateIdx->Idx;

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

				if (clusterMap[iSurfel_] < 0)
				{
					if (!bSurfelVisited[iSurfel_])
					{
						bSurfelVisited[iSurfel_] = true;

						pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

						if (pSurfel_->size > 0)
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
	}	// for each cluster

	delete[] candidateMem;
	delete[] bVertexVisited;
	delete[] bSurfelVisited;
	delete[] NHull.Element;
}

bool PSGM::Inside(
	int iVertex, 
	RECOG::PSGM_::Cluster *pCluster,
	int iSurfel)
{
	float maxe = kNoise * 2.0f / pSurfelDetector->kPlane;

	RECOG::PSGM_::Vertex *pVertex = vertexArray.Element[iVertex];

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

		if (e > maxe)
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
	RECOG::PSGM_::Vertex *pVertex;

	for (iVertex_ = 0; iVertex_ < pCluster->iVertexArray.n; iVertex_++)
	{
		pVertex = vertexArray.Element[pCluster->iVertexArray.Element[iVertex_]];

		e = RVLDOTPRODUCT3(pSurfel->N, pVertex->P) - pSurfel->d;

		if (e > maxe)
			return false;
	}

	return true;
}

void PSGM::UpdateNormalHull(
	Array<RECOG::PSGM_::NormalHullElement> &NHull,
	float *N)
{
	float *N_;
	float *Nh_;
	float fTmp;

	if (NHull.n == 0)
	{
		N_ = NHull.Element[0].N;

		RVLCOPY3VECTOR(N, N_);

		NHull.n = 1;

		return;
	}
	else if (NHull.n == 1)
	{
		N_ = NHull.Element[0].N;
		Nh_ = NHull.Element[0].Nh;

		RVLCROSSPRODUCT3(N, N_, Nh_);

		fTmp = sqrt(RVLDOTPRODUCT3(Nh_, Nh_));

		if (RVLABS(fTmp) < 1e-10)
			return;

		RVLSCALE3VECTOR2(Nh_, fTmp, Nh_);

		N_ = NHull.Element[1].N;
		float *Nh__ = NHull.Element[1].Nh;

		RVLCOPY3VECTOR(N, N_);

		RVLNEGVECT3(Nh_, Nh__);

		NHull.n = 2;

		return;
	}

	RECOG::PSGM_::NormalHullElement *pHullElement = NHull.Element + NHull.n - 1;

	N_ = pHullElement->N;
	Nh_ = pHullElement->Nh;

	bool bPrevIn = (RVLDOTPRODUCT3(Nh_, N) <= 0.0f);

	int iStart = -1;

	int iEnd;
	int i;	
	bool bIn;

	for (i = 0; i < NHull.n; i++)
	{
		pHullElement = NHull.Element + i;

		N_ = pHullElement->N;
		Nh_ = pHullElement->Nh;

		bIn = (RVLDOTPRODUCT3(Nh_, N) <= 0.0f);

		if (bIn)
		{
			if (!bPrevIn)
				iEnd = i;
		}
		else if (bPrevIn)
			iStart = i;

		bPrevIn = bIn;
	}

	if (iStart < 0)
		return;

	pHullElement = NHull.Element + iStart;

	N_ = pHullElement->N;
	Nh_ = pHullElement->Nh;

	float Nh[3];

	RVLCROSSPRODUCT3(N, N_, Nh);

	fTmp = sqrt(RVLDOTPRODUCT3(Nh, Nh));

	if (RVLABS(fTmp) < 1e-10)
		return;

	RVLSCALE3VECTOR2(Nh, fTmp, Nh_);

	pHullElement = NHull.Element + iEnd;

	N_ = pHullElement->N;

	RVLCROSSPRODUCT3(N_, N, Nh);

	fTmp = sqrt(RVLDOTPRODUCT3(Nh, Nh));

	if (RVLABS(fTmp) < 1e-10)
		return;

	if (iEnd == (iStart + 1) % NHull.n)	// Size of NHull should be increased.
	{
		if (iEnd > 0)
		{
			memmove(NHull.Element + iEnd + 1, NHull.Element + iEnd, (NHull.n - iEnd) * sizeof(RECOG::PSGM_::NormalHullElement));

			iEnd++;
		}

		NHull.n++;
	}
	else if (iEnd > (iStart + 2) % NHull.n)	// Size of NHull should be decreased.
	{
		if (iEnd > iStart)
		{
			memmove(NHull.Element + iStart + 2, NHull.Element + iEnd, (NHull.n - iEnd - 1) * sizeof(RECOG::PSGM_::NormalHullElement));

			NHull.n -= (iEnd - iStart - 2);
		}
		else 
		{
			if (iEnd > 0)
			{
				memmove(NHull.Element, NHull.Element + iEnd, (iStart - iEnd) * sizeof(RECOG::PSGM_::NormalHullElement));

				iStart -= iEnd;
			}

			NHull.n = iStart + 2;			
		}
	}

	pHullElement = NHull.Element + (iStart + 1) % NHull.n;

	N_ = pHullElement->N;
	Nh_ = pHullElement->Nh;

	RVLCOPY3VECTOR(N, N_);
	RVLSCALE3VECTOR2(Nh, fTmp, Nh_);
}

float PSGM::DistanceFromNormalHull(
	Array<RECOG::PSGM_::NormalHullElement> &NHull,
	float *N)
{
	if (NHull.n == 0)
		return 0.0f;
	if (NHull.n == 1)
	{
		float *N_ = NHull.Element[0].N;

		float e = RVLDOTPRODUCT3(N_, N);

		return (e < 0.0f ? 1.0f : sqrt(1.0f - e * e));
	}		

	float maxDist = 0.0f;

	int i;
	float dist;
	float *Nh_;

	for (i = 0; i < NHull.n; i++)
	{
		Nh_ = NHull.Element[i].Nh;

		dist = RVLDOTPRODUCT3(Nh_, N);

		if (dist > maxDist)
			maxDist = dist;
	}

	return maxDist;
}

void PSGM::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh)
{
	pVisualizer->SetMesh(pMesh);

	displayData.pMesh = pMesh;
	displayData.pSurfels = pSurfels;
	displayData.pRecognition = this;
	displayData.pVisualizer = pVisualizer;

	pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);
}

void PSGM::Display()
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	//DisplayClusters();

	pSurfels->Display(pVisualizer, pMesh);

	DisplayVertices();
}

void PSGM::DisplayClusters()
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	RECOG::PSGM_::Cluster *pCluster;
	int iCluster;
	Surfel *pSurfel;
	int i;
	unsigned char color[3];

	for (iCluster = 0; iCluster < clusters.n; iCluster++)
	{
		pCluster = clusters.Element + iCluster;

		color[0] = (unsigned char)(rand() % 256);
		color[1] = (unsigned char)(rand() % 256);
		color[2] = (unsigned char)(rand() % 256);

		for (i = 0; i < pCluster->iSurfelArray.n; i++)
		{
			pSurfel = pSurfels->NodeArray.Element + pCluster->iSurfelArray.Element[i];

			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);
		}
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

void PSGM::DisplayVertices()
{
	double lineLength = 10.0;

	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> linesPolyData =
		vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iLine = 0;

	double P0[3], P[3], V[3];
	Surfel *pSurfel;
	int iSurfel;

	RECOG::PSGM_::Vertex *pVertex = vertexList.pFirst;

	while (pVertex)
	{
		RVLCOPY3VECTOR(pVertex->P, P0);

		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			pSurfel = pSurfels->NodeArray.Element + pVertex->iSurfelArray.Element[iSurfel];

			pts->InsertNextPoint(P0);

			RVLSCALE3VECTOR(pSurfel->N, lineLength, V);

			RVLSUM3VECTORS(P0, V, P);

			pts->InsertNextPoint(P);

			iLine++;
		}

		pVertex = pVertex->pNext;
	}

	// Add the points to the polydata container
	linesPolyData->SetPoints(pts);

	// Create lines.

	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	unsigned char red[3] = { 255, 0, 0 };

	int nLines = iLine;

	vtkSmartPointer<vtkLine> *line = new vtkSmartPointer<vtkLine>[nLines];

	iLine = 0;

	pVertex = vertexList.pFirst;

	while (pVertex)
	{
		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			line[iLine] = vtkSmartPointer<vtkLine>::New();

			line[iLine]->GetPointIds()->SetId(0, 2 * iLine);
			line[iLine]->GetPointIds()->SetId(1, 2 * iLine + 1);

			lines->InsertNextCell(line[iLine]);

			colors->InsertNextTupleValue(red);

			iLine++;
		}

		pVertex = pVertex->pNext;
	}

	// Add the lines to the polydata container
	linesPolyData->SetLines(lines);

	// Color the lines.
	// SetScalars() automatically associates the values in the data array passed as parameter
	// to the elements in the same indices of the cell data array on which it is called.
	// This means the first component (red) of the colors array
	// is matched with the first component of the cell array (line 0)
	// and the second component (green) of the colors array
	// is matched with the second component of the cell array (line 1)
	linesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(linesPolyData);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	pVisualizer->renderer->AddActor(actor);

	delete[] line;
}