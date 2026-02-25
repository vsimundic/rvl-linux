// #include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkLine.h>
#include <vtkPolyLine.h>
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
// #include "RFRecognition.h" //VIDOVIC
// #include <Eigen\Eigenvalues>

// #define RVLSURFELGRAPH_VERTEX_DETECTION_VERSION_0
#ifdef RVLVERSION_171125
#define RVLSURFELGRAPH_VERTEX_DETECTION_VERSION_1
#define RVLSURFELGRAPH_IMAGE_ADJACENCY_NEW
#else
#define RVLSURFELGRAPH_VERTEX_DETECTION_VERSION_2
#endif
#define RVLSURFELGRAPH_DISPLAY_VERTICES

// #define RVLSURFELGRAPH_VERTEX_DETECTION_DEBUG

#define RVLSURFELGRAPH_IMAGE_ADJACENCY_ADD_CONNECTION(pSurfel, pOtherSurfel, iOtherSurfel, surfelIdx, dist, nImageAdjacencyRelations, pMem, desc, bVisited, bNeighbor) \
	{                                                                                                                                                                  \
		if (surfelIdx[iOtherSurfel] < 0)                                                                                                                               \
		{                                                                                                                                                              \
			bNeighbor = true;                                                                                                                                          \
			nImageAdjacencyRelations++;                                                                                                                                \
			RVLMEM_ALLOC_STRUCT(pMem, SurfelAdjecencyDescriptors, desc);                                                                                               \
			desc->minDist = dist;                                                                                                                                      \
			desc->avgDist = dist;                                                                                                                                      \
			desc->cupyDescriptor[0] = 0.0;                                                                                                                             \
			desc->cupyDescriptor[1] = 0.0;                                                                                                                             \
			desc->cupyDescriptor[2] = 0.0;                                                                                                                             \
			desc->cupyDescriptor[3] = 0.0;                                                                                                                             \
			desc->commonBoundaryLength = 1;                                                                                                                            \
			surfelIdx[iOtherSurfel] = pSurfel->imgAdjacency.size();                                                                                                    \
			pSurfel->imgAdjacency.push_back(pOtherSurfel);                                                                                                             \
			pSurfel->imgAdjacencyDescriptors.push_back(desc);                                                                                                          \
			pOtherSurfel->imgAdjacency.push_back(pSurfel);                                                                                                             \
			pOtherSurfel->imgAdjacencyDescriptors.push_back(desc);                                                                                                     \
		}                                                                                                                                                              \
		else                                                                                                                                                           \
		{                                                                                                                                                              \
			desc = pSurfel->imgAdjacencyDescriptors.at(surfelIdx[iOtherSurfel]);                                                                                       \
			if (dist < desc->minDist)                                                                                                                                  \
				desc->minDist = dist;                                                                                                                                  \
			desc->avgDist += dist;                                                                                                                                     \
			desc->commonBoundaryLength++;                                                                                                                              \
		}                                                                                                                                                              \
		bVisited[iOtherSurfel] = true;                                                                                                                                 \
	}

using namespace RVL;
using namespace SURFEL;

SurfelGraph::SurfelGraph()
{
	sizeUnit = 1e-6;
	imageAdjacencyThr = 6;
	TIVertexToleranceAngle = 22.5f; // deg
	edgeDepth = 20;
	occlusionVertexMinZ = 0.5f;			 // m
	occlusionVertexMaxZ = 4.0f;			 // m
	occlusionVertexResolutionZ = 0.004f; // m
	occlusionVertexWinSize = 9;
	occlusionVertexMeanShiftWinSize = 5;
	occlusionVertexMinClusterSize = 3;
	occlusionVertexMinDepthStep = 5;

	bGroundContactVertices = false;

	PtMem = NULL;
	surfelBndMem = NULL;
	surfelBndMem2 = NULL;
	BndMem = NULL;
	neighborEdge = NULL;
	surfelMap = NULL;
	edgeMap = NULL;
	nodeColor = NULL;
	NodeArray.Element = NULL;
	edgeMarkMap = NULL;
	EdgeArray.Element = NULL;
	surfelVertexList.Element = NULL;
	surfelVertexMem = NULL;
	vertexArray.Element = NULL;
	vertexEdgeArray.Element = NULL;
	vertexDisplayLineArray.Element = NULL;
	vertexDisplayLineArrayMem = NULL;
	bVertexAssigned = NULL;
	iVertexMem = NULL;
	momentsMem = NULL;
	surfelRefPtMem = NULL;
	planarSurfaceSurfelMem = NULL;
	polygonDataMem = NULL;
	polyEdges.n = 0;
	polyEdges.Element = NULL;
	polygonVerticesS.Element = NULL;
	polygonVerticesS.n = 0;
	triangleMem = NULL;

	bContactEdgeVertices = false;

	DisplayData.mode = RVLSURFEL_DISPLAY_MODE_SURFELS;
	DisplayData.mouseRButtonDownUserFunction = NULL;
	DisplayData.keyPressUserFunction = NULL;
	DisplayData.vpUserFunctionData = NULL;
	DisplayData.edgeFeatureIdxArray = NULL;
	DisplayData.edgeFeatureDepth = 0.01f;
	DisplayData.normalLen = 10.0f;
	DisplayData.bCallbackFunctionsDefined = false;
	DisplayData.bEdges = false;
	DisplayData.bPolygons = false;
	RVLSET3VECTOR(DisplayData.ForegroundColor, 0, 255, 0);
	RVLSET3VECTOR(DisplayData.BackgroundColor, 255, 0, 0);
	RVLSET3VECTOR(DisplayData.ConvexColor, 0, 255, 0);
	RVLSET3VECTOR(DisplayData.ConcaveColor, 255, 0, 255);
}

SurfelGraph::~SurfelGraph()
{
	Clear();
}

void SurfelGraph::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("SurfelGraph.visualization.edgeFeatureDepth", RVLPARAM_TYPE_FLOAT, &(DisplayData.edgeFeatureDepth));
	pParamData = ParamList.AddParam("SurfelGraph.visualization.normalLen", RVLPARAM_TYPE_FLOAT, &(DisplayData.normalLen));
	pParamData = ParamList.AddParam("SurfelGraph.edgeDepth", RVLPARAM_TYPE_INT, &edgeDepth);
	pParamData = ParamList.AddParam("SurfelGraph.groundContactVertices", RVLPARAM_TYPE_BOOL, &bGroundContactVertices);
	pParamData = ParamList.AddParam("SurfelGraph.occlusionVertex.minZ", RVLPARAM_TYPE_FLOAT, &occlusionVertexMinZ);
	pParamData = ParamList.AddParam("SurfelGraph.occlusionVertex.maxZ", RVLPARAM_TYPE_FLOAT, &occlusionVertexMaxZ);
	pParamData = ParamList.AddParam("SurfelGraph.occlusionVertex.resolutionZ", RVLPARAM_TYPE_FLOAT, &occlusionVertexResolutionZ);
	pParamData = ParamList.AddParam("SurfelGraph.occlusionVertex.winSize", RVLPARAM_TYPE_INT, &occlusionVertexWinSize);
	pParamData = ParamList.AddParam("SurfelGraph.occlusionVertex.meanShiftWinSize", RVLPARAM_TYPE_INT, &occlusionVertexMeanShiftWinSize);
	pParamData = ParamList.AddParam("SurfelGraph.occlusionVertex.minClusterSize", RVLPARAM_TYPE_INT, &occlusionVertexMinClusterSize);
	pParamData = ParamList.AddParam("SurfelGraph.occlusionVertex.minDepthStep", RVLPARAM_TYPE_INT, &occlusionVertexMinDepthStep);
}

void SurfelGraph::InitGetNeighborsBoundaryAndSize()
{
	neighborEdge = new SURFEL::Edge *[NodeArray.n];

	memset(neighborEdge, 0, NodeArray.n * sizeof(SURFEL::Edge *));

	memset(edgeMarkMap, 0, nMeshEdges * sizeof(unsigned char));
}

void SurfelGraph::FreeGetNeighborsBoundaryAndSize()
{
	RVL_DELETE_ARRAY(neighborEdge);
}

void SURFEL::ComputeParameters(
	Surfel *pSurfel,
	MESH::Distribution &distribution,
	Point *pPt)
{
	float *var = distribution.var;

	int idx[3];
	int iTmp;

	RVLSORT3ASCEND(var, idx, iTmp);

	float *N = pSurfel->N;
	float *N_ = distribution.R + 3 * idx[0];

	RVLCOPY3VECTOR(N_, N);

	if (RVLDOTPRODUCT3(pPt->N, N) < 0.0)
	{
		RVLNEGVECT3(N, N);
	}

	float *X = pSurfel->R;
	float *Y = pSurfel->R + 3;
	float *Z = pSurfel->R + 6;

	float *X_ = distribution.R + 3 * idx[2];

	RVLCOPY3VECTOR(X_, X);
	RVLCOPY3VECTOR(N, Z);

	RVLCROSSPRODUCT3(Z, X, Y);

	float *P = pSurfel->P;
	float *P_ = distribution.t;

	RVLCOPY3VECTOR(P_, P);

	pSurfel->d = RVLDOTPRODUCT3(N, P);

	int *RGB = pSurfel->RGB;
	int *RGB_ = distribution.RGB;

	RVLCOPY3VECTOR(RGB_, RGB);

	float *P0 = pSurfel->P0;

	RVLCOPY3VECTOR(pPt->P, P0);

	pSurfel->r0 = pSurfel->d / RVLDOTPRODUCT3(N, P0);
	pSurfel->r1 = 2.0f * sqrt(distribution.var[idx[2]]);
	pSurfel->r2 = 2.0f * sqrt(distribution.var[idx[1]]);

	pSurfel->flags |= RVLSURFEL_FLAG_RF;
}

void SURFEL::CreateFromPoint(
	Surfel *pSurfel,
	Point *pPt)
{
	float *P = pSurfel->P;
	float *P_ = pPt->P;

	RVLCOPY3VECTOR(P_, P);

	float *P0 = pSurfel->P0;

	RVLCOPY3VECTOR(P_, P0);

	float *N = pSurfel->N;
	float *N_ = pPt->N;

	RVLCOPY3VECTOR(N_, N);

	pSurfel->d = RVLDOTPRODUCT3(N, P);

	RVLCOPY3VECTOR(pPt->RGB, pSurfel->RGB);

	pSurfel->r0 = pSurfel->d / RVLDOTPRODUCT3(N, P0);
}

// Create point pPoint from the surfel pSurfel such that its position is identical to the position of the surfel centroid,
// its normal i identical to the surfel normal and its color is identical to the surfel color

void SURFEL::GetPoint(
	Surfel *pSurfel,
	Point *pPoint)
{
	RVLCOPY3VECTOR(pSurfel->P, pPoint->P);
	RVLCOPY3VECTOR(pSurfel->N, pPoint->N);
	RVLCOPY3VECTOR(pSurfel->RGB, pPoint->RGB);
}

void SurfelGraph::Init(Mesh *pMesh)
{
	Clear();

	nMeshVertices = pMesh->NodeArray.n;
	nMeshEdges = pMesh->EdgeArray.n;

	PtMem = new QLIST::Index2[nMeshVertices];
	surfelBndMem = new MeshEdgePtr *[2 * nMeshEdges];
	surfelBndMem2 = new Array<MeshEdgePtr *>[nMeshEdges];
	BndMem = new MeshEdgePtr *[pMesh->nBoundaryPts];
	surfelMap = new int[nMeshVertices];
	edgeMap = new int[nMeshVertices];
	// surfelBndMap = new QLIST::Index2[nPoints];
	NodeArray.Element = new Surfel[2 * nMeshVertices];
	edgeMarkMap = new unsigned char[nMeshEdges];

	QList<SURFEL::Vertex> *pVertexList = &vertexList;
	RVLQLIST_INIT(pVertexList);
	vertexArray.n = 0;
}

#ifdef RVLSURFEL_IMAGE_ADJACENCY
void SurfelGraph::SurfelRelations(Mesh *pMesh)
{
	ImageAdjacency(pMesh);

	Surfel *pSurfel = NodeArray.Element;

	for (int i = 0; i < NodeArray.n; pSurfel++, i++)
	{
		if (pSurfel->size <= 1)
			continue;

		// if (pSurfel->bEdge)
		//	continue;

		DetermineImgAdjDescriptors(pSurfel, pMesh);
	}
}

void SurfelGraph::ImageAdjacency(Mesh *pMesh)
{
	bool *bVisited = new bool[NodeArray.n];

	memset(bVisited, 0, NodeArray.n * sizeof(bool));

	int *surfelIdx = new int[NodeArray.n];

	memset(surfelIdx, 0xff, NodeArray.n * sizeof(int));

	nImageAdjacencyRelations = 0;

	int iSurfel;
	Surfel *pSurfel;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->size <= 1)
			continue;

		// if (pSurfel->bEdge)
		//	continue;

		ImageAdjacency(pMesh, iSurfel, surfelIdx, bVisited);
	}

	delete[] bVisited;
	delete[] surfelIdx;
}

void SurfelGraph::ImageAdjacency(
	Mesh *pMesh,
	int iSurfel,
	int *surfelIdx,
	bool *bVisited)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	if (pSurfel->BoundaryArray.n == 0)
		return;

	// find largest boundary (most probable outer boundary)
	int boundary = 0;
	int boundarySize = 0;
	if (pSurfel->BoundaryArray.n > 1)
	{
		for (int b = 0; b < pSurfel->BoundaryArray.n; b++)
		{
			if (pSurfel->BoundaryArray.Element[b].n > boundarySize)
			{
				boundarySize = pSurfel->BoundaryArray.Element[b].n;
				boundary = b;
			}
		}
	}
	else
		boundarySize = pSurfel->BoundaryArray.Element[boundary].n;

	int i;
	int iOtherSurfel;

	for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
	{
		iOtherSurfel = pSurfel->imgAdjacency.at(i) - NodeArray.Element;

		surfelIdx[iOtherSurfel] = i;
	}

	float *N = pSurfel->N;
	float d = pSurfel->d;

	// run through edges
	Array<MeshEdgePtr *> BoundaryArray = pSurfel->BoundaryArray.Element[boundary];
	MeshEdgePtr *pCurrEdge;
	Surfel *pOtherSurfel;
	SurfelAdjecencyDescriptors *desc;
	// int iBoundary;
	int iPointEdge;
	int iPt, iPt2, x, y;
	float dist;
	Point *pPt, *pPt2;
	float *P, *P2, *N2;
	float P_[3], P2_[3];
	float dP[3];
	MeshEdgePtr *pEdgePtr;
	bool bNeighbor;
	float fTmp;
	// float V[3];
#ifdef RVLSURFELGRAPH_IMAGE_ADJACENCY_NEW
	float minDist;
	int iNeighbor;
#endif

	for (iPointEdge = 0; iPointEdge < BoundaryArray.n; iPointEdge++)
	{
		pCurrEdge = BoundaryArray.Element[iPointEdge];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);

		pPt = pMesh->NodeArray.Element + iPt;

		P = pPt->P;

		if (pSurfel->bEdge)
		{
			pEdgePtr = pPt->EdgeList.pFirst;

			while (pEdgePtr)
			{
				iPt2 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

				pPt2 = pMesh->NodeArray.Element + iPt2;

				if (pPt2->bBoundary)
				{
					iOtherSurfel = edgeMap[iPt2];

					if (iOtherSurfel >= 0 && iOtherSurfel < NodeArray.n && iOtherSurfel != iSurfel)
					{
						pOtherSurfel = NodeArray.Element + iOtherSurfel; // surfel owner of the pixel

						// if (iSurfel == 719 && iOtherSurfel == 846 || iSurfel == 846 && iOtherSurfel == 719)
						//	int debug = 0;

						P2 = pPt2->P;

						RVLDIF3VECTORS(P2, P, dP);

						dist = sqrt(RVLDOTPRODUCT3(dP, dP));

						RVLSURFELGRAPH_IMAGE_ADJACENCY_ADD_CONNECTION(pSurfel, pOtherSurfel, iOtherSurfel, surfelIdx, dist, nImageAdjacencyRelations, pMem, desc, bVisited, bNeighbor);
					}
				}

				pEdgePtr = pEdgePtr->pNext;
			}
		}
		else
		{
			P = pPt->P;

			RVL_PROJECT_3DPOINT_TO_PLANE(P, N, d, P_);

			bNeighbor = false;

#ifdef RVLSURFELGRAPH_IMAGE_ADJACENCY_NEW
			pEdgePtr = pPt->EdgeList.pFirst;

			while (pEdgePtr)
			{
				iPt2 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

				iOtherSurfel = surfelMap[iPt2];

				if (iOtherSurfel >= 0 && iOtherSurfel < NodeArray.n)
				{
					pOtherSurfel = NodeArray.Element + iOtherSurfel; // surfel owner of the pixel

					if ((pOtherSurfel->size < 640 * 480) && (pOtherSurfel->size > 1) && (iOtherSurfel != iSurfel))
					{
						pPt2 = pMesh->NodeArray.Element + iPt2;

						P2 = pPt2->P;

						N2 = pOtherSurfel->N;

						RVL_PROJECT_3DPOINT_TO_PLANE(P2, N2, pOtherSurfel->d, P2_);

						RVLDIF3VECTORS(P2_, P_, dP);

						dist = sqrt(RVLDOTPRODUCT3(dP, dP));

						RVLSURFELGRAPH_IMAGE_ADJACENCY_ADD_CONNECTION(pSurfel, pOtherSurfel, iOtherSurfel, surfelIdx, dist, nImageAdjacencyRelations, pMem, desc, bVisited, bNeighbor);
					}
				}

				pEdgePtr = pEdgePtr->pNext;
			}

			if (!bNeighbor && pMesh->bOrganizedPC)
			{
				minDist = 0.0f;
				iNeighbor = -1;
#endif

				y = floor(iPt / 640.0);
				x = floor(iPt - 640.0 * y);

				// Running through point neighbourhood
				for (int yy = y - imageAdjacencyThr; yy < y + imageAdjacencyThr; yy++)
				{
					if ((yy < 0) || (yy >= 480))
						continue;
					for (int xx = x - imageAdjacencyThr; xx < x + imageAdjacencyThr; xx++)
					{
						if ((xx < 0) || (xx >= 640))
							continue;
						iPt2 = yy * 640 + xx;

						iOtherSurfel = surfelMap[iPt2];

						if (iOtherSurfel < 0 || iOtherSurfel >= NodeArray.n)
							continue;

						// if (iSurfel == 114 && iOtherSurfel == 116 || iSurfel == 116 && iOtherSurfel == 114)
						//	int debug = 0;

						pOtherSurfel = NodeArray.Element + iOtherSurfel; // surfel owner of the pixel

						if ((pOtherSurfel->size < 640 * 480) && (pOtherSurfel->size > 1) && (iOtherSurfel != iSurfel))
						{
							pPt2 = pMesh->NodeArray.Element + iPt2;

							P2 = pPt2->P;

							N2 = pOtherSurfel->N;

							RVL_PROJECT_3DPOINT_TO_PLANE(P2, N2, pOtherSurfel->d, P2_);

							RVLDIF3VECTORS(P2_, P_, dP);

							dist = sqrt(RVLDOTPRODUCT3(dP, dP));

#ifdef RVLSURFELGRAPH_IMAGE_ADJACENCY_NEW
							if (iNeighbor < 0 || dist < minDist)
							{
								minDist = dist;
								iNeighbor = iOtherSurfel;
							}
#else
						RVLSURFELGRAPH_IMAGE_ADJACENCY_ADD_CONNECTION(pSurfel, pOtherSurfel, iOtherSurfel, surfelIdx, dist, nImageAdjacencyRelations, pMem, desc, bVisited, bNeighbor);
#endif
						}
					}
				} // Running through point neighbourhood

#ifdef RVLSURFELGRAPH_IMAGE_ADJACENCY_NEW
				if (iNeighbor >= 0)
				{
					pOtherSurfel = NodeArray.Element + iNeighbor;

					RVLSURFELGRAPH_IMAGE_ADJACENCY_ADD_CONNECTION(pSurfel, pOtherSurfel, iNeighbor, surfelIdx, minDist, nImageAdjacencyRelations, pMem, desc, bVisited, bNeighbor);
				}
			} // if (!bNeighbor)
#endif
			// Identify adjacent edge features.

			if (pPt->bBoundary)
			{
				iOtherSurfel = edgeMap[iPt];

				if (iOtherSurfel >= 0 && iOtherSurfel < NodeArray.n)
				{
					pOtherSurfel = NodeArray.Element + iOtherSurfel; // surfel owner of the pixel

					RVLSURFELGRAPH_IMAGE_ADJACENCY_ADD_CONNECTION(pSurfel, pOtherSurfel, iOtherSurfel, surfelIdx, 0.0f, nImageAdjacencyRelations, pMem, desc, bVisited, bNeighbor);
				}
			}
		} // if (!pSurfel->bEdge)

		// Reset bVisited and compute the common boundary length of all neighboring surfels.

		for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
		{
			pOtherSurfel = pSurfel->imgAdjacency.at(i);

			iOtherSurfel = pOtherSurfel - NodeArray.Element;

			if (bVisited[iOtherSurfel])
			{
				bVisited[iOtherSurfel] = false;

				desc = pSurfel->imgAdjacencyDescriptors.at(i);
			}
		}
	} // for every boundary point

	for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
	{
		desc = pSurfel->imgAdjacencyDescriptors.at(i);

		// desc->minDist = sqrt(desc->minDist);

		pOtherSurfel = pSurfel->imgAdjacency.at(i);

		iOtherSurfel = pOtherSurfel - NodeArray.Element;

		surfelIdx[iOtherSurfel] = -1;
	}
}

void SurfelGraph::DetermineImgAdjDescriptors(
	Surfel *pSurfel,
	Mesh *mesh)
{
	if (pSurfel->BoundaryArray.n == 0)
		return;

	// Calculate Cupy adjacency descriptor
	// find largest boundary (most probable outer boundary)
	int boundary = 0;

	if (!pSurfel->bEdge)
	{
		int boundarySize = 0;
		if (pSurfel->BoundaryArray.n > 1)
		{
			for (int b = 0; b < pSurfel->BoundaryArray.n; b++)
			{
				if (pSurfel->BoundaryArray.Element[b].n > boundarySize)
				{
					boundarySize = pSurfel->BoundaryArray.Element[b].n;
					boundary = b;
				}
			}
		}
	}

	float *N = pSurfel->N;

	// run through neighbours
	// Array<MeshEdgePtr *> BoundaryArray;
	// MeshEdgePtr *pCurrEdge;
	Surfel *pOtherSurfel;
	SurfelAdjecencyDescriptors *desc;
	// int iBoundary, iPointEdge;
	// int iPt;
	float a[4];
	float dN[3], dP[3];
	float V[3];
	float dOffset;
	float *N_;
	// float *N_, *P, *P_;
	// float A, dA, x, x_, y, y_;
	float fTmp;

	for (int i = 0; i < pSurfel->imgAdjacency.size(); i++)
	{
		// Get other surfel
		pOtherSurfel = pSurfel->imgAdjacency.at(i);

#ifdef RVLSURFELGRAPH_DEBUG_RELATION_DESCRIPTOR
		bDebug = false;

		if (bDebug)
			int debug = 0;

		// if (pSurfel - NodeArray.Element == 1 && pOtherSurfel - NodeArray.Element == 921)
		if (pSurfel - NodeArray.Element == 986)
			int debug = 0;
#endif

		if (pOtherSurfel->BoundaryArray.n == 0)
			continue;

		desc = pSurfel->imgAdjacencyDescriptors.at(i);
		if ((desc->cupyDescriptor[0] + desc->cupyDescriptor[1] + desc->cupyDescriptor[2] + desc->cupyDescriptor[3]) != 0.0)
			continue; // this adjacancy descriptor has already been set, probably by other surfel

		N_ = pOtherSurfel->N;

		RVLDIF3VECTORS(N_, N, dN);

		fTmp = RVLDOTPRODUCT3(dN, dN);

		if (RVLABS(fTmp) < 1e-10)
		{
			desc->avgDist /= (double)(desc->commonBoundaryLength);
			desc->cupyDescriptor[0] = acos(RVLDOTPRODUCT3(N, N_));
			desc->cupyDescriptor[1] = 1.0f;
			desc->cupyDescriptor[2] = 1.0f;
			desc->cupyDescriptor[3] = desc->minDist;
		}
		else
		{
			fTmp = sqrt(fTmp);

			RVLNORM3(dN, fTmp);

			dOffset = (pOtherSurfel->d - pSurfel->d) / fTmp;

#ifdef RVLSURFELGRAPH_DEBUG_RELATION_DESCRIPTOR
			FILE *fpDebug;

			if (bDebug)
			{
				fpDebug = fopen("C:\\RVL\\Debug\\planes.txt", "w");

				fprintf(fpDebug, "%f\t%f\t%f\t%f\n", pSurfel->N[0], pSurfel->N[1], pSurfel->N[2], pSurfel->d);
				fprintf(fpDebug, "%f\t%f\t%f\t%f\n", pOtherSurfel->N[0], pOtherSurfel->N[1], pOtherSurfel->N[2], pOtherSurfel->d);

				fclose(fpDebug);
			}
#endif

			// get other boundary
			int boundaryOther = 0;

			if (!pOtherSurfel->bEdge)
			{
				int boundarySizeOther = 0;
				if (pOtherSurfel->BoundaryArray.n > 1)
				{
					for (int b = 0; b < pOtherSurfel->BoundaryArray.n; b++)
					{
						if (pOtherSurfel->BoundaryArray.Element[b].n > boundarySizeOther)
						{
							boundarySizeOther = pOtherSurfel->BoundaryArray.Element[b].n;
							boundaryOther = b;
						}
					}
				}
			}

			memset(a, 0, 4 * sizeof(float));

#ifdef RVLSURFELGRAPH_DEBUG_RELATION_DESCRIPTOR
			if (bDebug)
				fpDebug = fopen("C:\\RVL\\Debug\\surfel1.txt", "w");
#endif
			SurfelAreaDistribution(mesh, pSurfel, boundary, dN, dOffset, a);

#ifdef RVLSURFELGRAPH_DEBUG_RELATION_DESCRIPTOR
			if (bDebug)
				fpDebug = fopen("C:\\RVL\\Debug\\surfel2.txt", "w");
#endif
			RVLNEGVECT3(dN, dN);
			dOffset = -dOffset;

			SurfelAreaDistribution(mesh, pOtherSurfel, boundaryOther, dN, dOffset, a + 2);

			int p, q;
			double tempm = 0;
			// argmax_l(max_i(a_i_l))
			for (int pp = 0; pp < 4; pp++)
			{
				if (a[pp] > tempm)
				{
					p = pp;
					tempm = a[pp];
				}
			}
			p = (p % 2 == 0) ? 1 : 2;
			// argmax_i(a_i_p)
			q = (a[p - 1] > a[2 + p - 1]) ? 1 : 2;

			// get and update descriptor
			desc->avgDist /= (double)(desc->commonBoundaryLength);
			desc->cupyDescriptor[0] = (3 - 2 * p) * acos(RVLDOTPRODUCT3(N, N_)); //(3 - 2*p)*acos(n_i*n_j)
			desc->cupyDescriptor[1] = a[(q - 1) * 2 + (p - 1)];
			desc->cupyDescriptor[2] = a[((3 - q) - 1) * 2 + (p - 1)];
			desc->cupyDescriptor[3] = (pSurfel->bEdge || pOtherSurfel->bEdge ? desc->minDist : desc->avgDist);
			// desc->cupyDescriptor[3] = desc->minDist;
		} // if (RVLABS(fTmp) >= 1e-10)
	}
}

void SurfelGraph::SurfelAreaDistribution(
	Mesh *mesh,
	Surfel *pSurfel,
	int iBoundary,
	float *dN,
	float dOffset,
	float *a)
{
	float edgeDepth_ = (float)edgeDepth * 0.001f;

	Array<MeshEdgePtr *> boundary;
	int iPointEdge;
	float *P;
	int iPt;
	MeshEdgePtr *pCurrEdge;
	float V[3];

	if (pSurfel->bEdge)
	{
		a[0] = 0.0;

		boundary = pSurfel->BoundaryArray.Element[0];

		float r, s, k;

		for (iPointEdge = 0; iPointEdge < boundary.n; iPointEdge++)
		{
			pCurrEdge = boundary.Element[iPointEdge];

			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);

			P = mesh->NodeArray.Element[iPt].P;

			r = edgeDepth_ / sqrt(RVLDOTPRODUCT3(P, P));

			RVLSCALE3VECTOR(P, r, V);

			k = RVLDOTPRODUCT3(dN, V) / edgeDepth_;

			if (RVLABS(k) >= 1e-3)
			{
				s = (dOffset - RVLDOTPRODUCT3(dN, P)) / k;

				if (k >= 0.0f)
				{
					if (s > 0.0f)
						a[0] += (s >= 1.0f ? 1.0f : s);
				}
				else
				{
					if (s < 1.0f)
						a[0] += (s <= 0.0f ? 1.0f : 1.0f - s);
				}
			}
			else
			{
				s = dOffset - RVLDOTPRODUCT3(dN, P);

				if (s <= 0.0f)
					a[0] += 1.0f;
			}
		}

		a[0] /= (float)(boundary.n);
	}
	else
	{
		float *N = pSurfel->N;

		RVLCROSSPRODUCT3(N, dN, V);

		float A = 0.0f;

		boundary = pSurfel->BoundaryArray.Element[iBoundary];

		pCurrEdge = boundary.Element[boundary.n - 1];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);

		float *P_ = mesh->NodeArray.Element[iPt].P;

		float x_ = RVLDOTPRODUCT3(P_, V);

		float y_ = RVLDOTPRODUCT3(dN, P_) - dOffset;

		float dP[3];
		float x, y, dA;

		for (iPointEdge = 0; iPointEdge < boundary.n; iPointEdge++)
		{
			pCurrEdge = boundary.Element[iPointEdge];

			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);

			// if ((tempN[0] * mesh->NodeArray.Element[iPt].P[0] + tempN[1] * mesh->NodeArray.Element[iPt].P[1] + tempN[2] * mesh->NodeArray.Element[iPt].P[2] - dOffset) <= 0.0)
			//	a[0]++;

			P = mesh->NodeArray.Element[iPt].P;

			RVLDIF3VECTORS(P, P_, dP);

			x = RVLDOTPRODUCT3(P, V);

			y = RVLDOTPRODUCT3(dN, P) - dOffset;

#ifdef RVLSURFELGRAPH_DEBUG_RELATION_DESCRIPTOR
			if (bDebug)
				fprintf(fpDebug, "%f\t%f\t%f\t%f\t%f\n", P[0], P[1], P[2], x, y);
#endif

			dA = 0.5f * (y + y_) * (x - x_);

			P_ = P;

			x_ = x;

			y_ = y;

			A += dA;

			if (y <= 0.0f)
				a[0] += dA;
		}

#ifdef RVLSURFELGRAPH_DEBUG_RELATION_DESCRIPTOR
		if (bDebug)
			fclose(fpDebug);
#endif

		// if (boundarySize > 0)
		//	a[0] /= (double)boundarySize;
		// else
		//	a[0] = 0.0f;

		a[0] = (A > 0.0f ? a[0] / A : 0.0f);
	}

	a[1] = 1.0 - a[0];
}

void SurfelGraph::SplitAndMergeError(
	Surfel *pCurrSurfel,
	Surfel *pOtherSurfel,
	int nGTObjects,
	int &splitError,
	int &mergeError)
{
	if (pCurrSurfel - NodeArray.Element == 3 && pOtherSurfel - NodeArray.Element == 141)
		int debug = 0;

	int iGTObject, GTObjectSize, GTObjectSize_, commonGTObjectSize;
	int iRefSurfel, mergeCost, splitCost, maxOtherGTObjectSize;
	Surfel *pOtherSurfel_;

	int nTotal = 0;
	int maxCommonGTObjectSize = 0;
	int maxGTObjectSize = 0;

	for (iGTObject = 0; iGTObject < nGTObjects; iGTObject++)
	{
		GTObjectSize = pCurrSurfel->GTObjHist.at(iGTObject);
		if (GTObjectSize > maxGTObjectSize)
		{
			maxGTObjectSize = GTObjectSize;
			iRefSurfel = 0;
		}
		GTObjectSize_ = pOtherSurfel->GTObjHist.at(iGTObject);
		if (GTObjectSize_ > maxGTObjectSize)
		{
			maxGTObjectSize = GTObjectSize_;
			iRefSurfel = 1;
		}
		commonGTObjectSize = GTObjectSize + GTObjectSize_;
		nTotal += commonGTObjectSize;
		if (commonGTObjectSize > maxCommonGTObjectSize)
			maxCommonGTObjectSize = commonGTObjectSize;
	}

	mergeError = nTotal - maxCommonGTObjectSize;

	pOtherSurfel_ = (iRefSurfel == 0 ? pOtherSurfel : pCurrSurfel);

	if (pCurrSurfel->ObjectID == pOtherSurfel->ObjectID)
	{
		maxOtherGTObjectSize = 0;

		for (iGTObject = 0; iGTObject < nGTObjects; iGTObject++)
		{
			if (iGTObject != pOtherSurfel_->ObjectID)
			{
				GTObjectSize_ = pOtherSurfel_->GTObjHist.at(iGTObject);

				if (GTObjectSize_ > maxOtherGTObjectSize)
					maxOtherGTObjectSize = GTObjectSize_;
			}
		}
	}
	else
		maxOtherGTObjectSize = pOtherSurfel_->GTObjHist.at(pOtherSurfel_->ObjectID);

	splitError = nTotal - maxGTObjectSize - maxOtherGTObjectSize;
}

// Generate scene segmenation file
void SurfelGraph::GenerateSSF(
	std::string filename,
	int minSurfelSize,
	bool checkbackground)
{
	std::stringstream ss;
	// SceneSegFile object
	SceneSegFile::SceneSegFile *ssf = new SceneSegFile::SceneSegFile("Scene");
	std::shared_ptr<SceneSegFile::SegFileElement> surfel;

	Surfel *pCurrSurfel = NodeArray.Element;
	Surfel *pOtherSurfel;
	int nGTObjects;
	int splitError, mergeError, falseClassificationCost;
	QLIST::Index *qlistelementVertex;
	SURFEL::Vertex *rvlvertex;
	QList<QLIST::Index> *pSurfelVertexList;
	// for surfel
	for (int i = 0; i < NodeArray.n; pCurrSurfel++, i++)
	{
		// if ((pCurrSurfel->ObjectID == -1) || (checkbackground && ((pCurrSurfel->ObjectID == 255) || (pCurrSurfel->ObjectID == 0))) || (pCurrSurfel->size == 1) || (pCurrSurfel->size == 0) || pCurrSurfel->bEdge || pCurrSurfel->size < minSurfelSize)
		if ((checkbackground && ((pCurrSurfel->ObjectID == 255) || (pCurrSurfel->ObjectID == 0))) || (pCurrSurfel->size <= 1) || pCurrSurfel->bEdge)
			continue;

		// Create element
		ss.clear();
		ss.str("");
		ss << "Surfel_" << i;
		surfel = std::make_shared<SceneSegFile::SegFileElement>(i, ss.str());
		ssf->AddElement(surfel);

		// Add surfel features
		// Centroid
		surfel->features.AddFeature(SceneSegFile::FeaturesList::Centroid, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::Centroid), "float");
		surfel->features.CopyFeatureData<float>(SceneSegFile::FeaturesList::Centroid, pCurrSurfel->P, 3);
		// GT object ID
		surfel->features.AddFeature(SceneSegFile::FeaturesList::GTObjectID, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::GTObjectID), "int");
		surfel->features.CopyFeatureData<int>(SceneSegFile::FeaturesList::GTObjectID, &pCurrSurfel->ObjectID, 1);
		// Pixel affiliation
		int *pixelIndices = new int[pCurrSurfel->size];
		RVL::QLIST::Index2 *pt;
		pt = pCurrSurfel->PtList.pFirst;
		for (int i = 0; i < pCurrSurfel->size; i++)
		{
			pixelIndices[i] = pt->Idx;
			pt = pt->pNext;
		}
		surfel->features.AddFeature(SceneSegFile::FeaturesList::PixelAffiliation, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::PixelAffiliation), "int");
		surfel->features.SetFeatureData<int>(SceneSegFile::FeaturesList::PixelAffiliation, pixelIndices, pCurrSurfel->size);
		// GTObjHistogram
		nGTObjects = pCurrSurfel->GTObjHist.size();
		int *GTObjHist = new int[nGTObjects];
		for (int i = 0; i < nGTObjects; i++)
			GTObjHist[i] = pCurrSurfel->GTObjHist[i];
		surfel->features.AddFeature(SceneSegFile::FeaturesList::GTObjHistogram, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::GTObjHistogram), "int");
		surfel->features.SetFeatureData<int>(SceneSegFile::FeaturesList::GTObjHistogram, GTObjHist, pCurrSurfel->GTObjHist.size());
		// Vertices
		if (this->surfelVertexList.Element)
		{
			// getting current surfel vertex list
			pSurfelVertexList = this->surfelVertexList.Element + (pCurrSurfel - this->NodeArray.Element);
			// running through added surfel vertices
			qlistelementVertex = pSurfelVertexList->pFirst;
			int noVertices = 0;
			// find out how many vertices there are
			while (qlistelementVertex)
			{
				noVertices++;
				// Next
				qlistelementVertex = qlistelementVertex->pNext;
			}
			if (noVertices > 0)
			{
				// add vertices
				float *vertices = new float[noVertices * 3];
				int iVert = 0;
				qlistelementVertex = pSurfelVertexList->pFirst;
				while (qlistelementVertex)
				{
					rvlvertex = this->vertexArray.Element[qlistelementVertex->Idx];

					vertices[iVert * 3] = rvlvertex->P[0];
					vertices[iVert * 3 + 1] = rvlvertex->P[1];
					vertices[iVert * 3 + 2] = rvlvertex->P[2];
					iVert++;
					// Next
					qlistelementVertex = qlistelementVertex->pNext;
				}
				surfel->features.AddFeature(SceneSegFile::FeaturesList::Vertices3D, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::Vertices3D), "float");
				surfel->features.SetFeatureData<float>(SceneSegFile::FeaturesList::Vertices3D, vertices, noVertices * 3);
			}
		}

		// Add feature groups
		// Adjacency group
		surfel->AddFeatureGroup(SceneSegFile::FeatureGroupsList::AdjacencyFeatureGroup, SceneSegFile::FeatureGroupsDictionary::dictionary.at(SceneSegFile::FeatureGroupsList::AdjacencyFeatureGroup));
		std::shared_ptr<SceneSegFile::FeatureGroup> adjFeatureGroup = surfel->featureGroups.at(SceneSegFile::FeatureGroupsList::AdjacencyFeatureGroup);

		// Adjacency group's set
		for (int i = 0; i < pCurrSurfel->imgAdjacency.size(); i++)
		{
			pOtherSurfel = pCurrSurfel->imgAdjacency.at(i);

			if (pOtherSurfel->size <= 1 || pOtherSurfel->bEdge)
				continue;

			// feature group's feature set
			adjFeatureGroup->AddFeatureSet(pOtherSurfel - NodeArray.Element, SceneSegFile::FeatureSetsDictionary::dictionary.at(SceneSegFile::FeatureSetsList::AdjacencyNode), true);
			std::shared_ptr<SceneSegFile::FeatureSet> adjFeatureSet = adjFeatureGroup->featureSets.at(pOtherSurfel - NodeArray.Element);
			// Add features
			// Same GT object
			adjFeatureSet->AddFeature(SceneSegFile::FeaturesList::SameGTObject, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::SameGTObject), "bool");
			bool sameGTObj = pCurrSurfel->ObjectID == pOtherSurfel->ObjectID ? true : false;
			adjFeatureSet->SetFeatureData<bool>(SceneSegFile::FeaturesList::SameGTObject, &sameGTObj);
			// Cupy feature vector
			adjFeatureSet->AddFeature(SceneSegFile::FeaturesList::CupysFeature, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::CupysFeature), "double");
			adjFeatureSet->CopyFeatureData<double>(SceneSegFile::FeaturesList::CupysFeature, pCurrSurfel->imgAdjacencyDescriptors.at(i)->cupyDescriptor, 4);
			// CommonBoundaryLenght
			adjFeatureSet->AddFeature(SceneSegFile::FeaturesList::CommonBoundaryLength, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::CommonBoundaryLength), "int");
			adjFeatureSet->CopyFeatureData<int>(SceneSegFile::FeaturesList::CommonBoundaryLength, &pCurrSurfel->imgAdjacencyDescriptors.at(i)->commonBoundaryLength, 1);
			// Cost of false classification
			SplitAndMergeError(pCurrSurfel, pOtherSurfel, nGTObjects, splitError, mergeError);
			falseClassificationCost = (sameGTObj ? splitError - mergeError : mergeError - splitError);
			adjFeatureSet->AddFeature(SceneSegFile::FeaturesList::FalseSegmentationCost, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::FalseSegmentationCost), "int");
			adjFeatureSet->CopyFeatureData<int>(SceneSegFile::FeaturesList::FalseSegmentationCost, &falseClassificationCost, 1);
		}
	}

	ssf->Save(filename);
	delete ssf;
}
#endif

#ifdef RVLSURFEL_GT_OBJECT_HISTOGRAM
// Returns surfels Label ID with most object support
void SurfelGraph::SetPrimaryGTObj(Surfel *pSurfel, cv::Mat labGTImg, int noObj)
{
	int objIdx = -1; // default value
	// Generate histogram of object (pixel) support
	int *objHist = new int[noObj];
	memset(objHist, 0, noObj * sizeof(int));
	RVL::QLIST::Index2 *pt;
	int x = 0, y = 0;
	pt = pSurfel->PtList.pFirst;
	for (int i = 0; i < pSurfel->size; i++)
	{
		y = floor(pt->Idx / 640.0);
		x = floor(pt->Idx - 640.0 * y);
		objHist[labGTImg.at<cv::Vec3b>(y, x)[0]]++;
		pt = pt->pNext;
	}
	// find max support and set surfel GTObjHist
	int max = 0;
	for (int i = 0; i < noObj; i++)
	{
		if (objHist[i] > max)
		{
			max = objHist[i];
			objIdx = i;
		}
		pSurfel->GTObjHist.push_back(objHist[i]);
	}
	delete[] objHist;
	pSurfel->ObjectID = objIdx;
}

void SurfelGraph::AssignGroundTruthSegmentation(
	char *meshFileName,
	int minSurfelSize)
{
	// Segmentation analysis
	// filenames
	std::string labelImgFileName(meshFileName);
	labelImgFileName.erase(labelImgFileName.find_last_of("."));
	// std::string depthImgFileName = labelImgFileName + "d.png";
	// std::string ssfFileName = labelImgFileName + ".ssf";
	labelImgFileName += "a.png";
	////TEST SSF LOAD
	// SceneSegFile::SceneSegFile* ssf = new SceneSegFile::SceneSegFile("test");
	// ssf->Load(ssfFileName);
	//
	// Load label image
	cv::Mat GTlabImg = cv::imread(labelImgFileName);
	// cv::Mat GTdepthImg = cv::imread(depthImgFileName, cv::ImreadModes::IMREAD_ANYDEPTH);
	////Preprocess GT label image (such as labeling background)
	// PreprocessGTLab(GTlabImg, GTdepthImg);
	// Get label min/max value
	double minLab, maxLab;
	cv::minMaxLoc(GTlabImg, &minLab, &maxLab);

	// Detect primary GT object for ALL surfels
	Surfel *pCurrSurfel = NodeArray.Element;
	std::cout << "Detecting primary GT object!" << std::endl;
	for (int i = 0; i < NodeArray.n; pCurrSurfel++, i++)
	{
		pCurrSurfel->ObjectID = -1;
		// if ((pCurrSurfel->size == 1) || (pCurrSurfel->size == 0) || pCurrSurfel->bEdge || pCurrSurfel->size < minSurfelSize)
		if ((pCurrSurfel->size <= 1) || pCurrSurfel->bEdge)
			continue;
		/*pCurrSurfel->ObjectID = DetPrimaryGTObj(pCurrSurfel, GTlabImg, 256);*/ // 256 objects because background has label of 255
		SetPrimaryGTObj(pCurrSurfel, GTlabImg, maxLab + 1);						 // maxLab + 1 because the last GT object label has to be maxLab and not maxLab - 1
	}
}
#endif

cv::Mat SurfelGraph::GenColoredSurfelImg()
{
	int w = 640;
	int h = 480;

	cv::Mat coloredSegLab(h, w, CV_8UC3, cv::Scalar::all(0));

	int nPixels = w * h;

	uchar noSurfelColor[] = {0, 0, 0};

	int iPix;
	uchar *labSegColor;
	int iSurfel;
	int x, y;

	for (iPix = 0; iPix < nPixels; iPix++)
	{
		iSurfel = surfelMap[iPix];

		labSegColor = (iSurfel >= 0 ? nodeColor + 3 * iSurfel : noSurfelColor);

		x = iPix % w;
		y = iPix / w;

		coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
		coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
		coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
	}

	return coloredSegLab;
}

// Generate a colored opencv image based on surfel data from SSF
cv::Mat SurfelGraph::GenColoredSurfelImgFromSSF(std::shared_ptr<SceneSegFile::SceneSegFile> ssf)
{
	std::shared_ptr<SceneSegFile::SegFileElement> currSSFElement;
	std::shared_ptr<SceneSegFile::FeatureTypeInt> pixAff;

	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));

	unsigned char labSegColor[3];
	int x = 0, y = 0;

	for (int i = 0; i < ssf->elements.size(); i++)
	{
		currSSFElement = ssf->elements.at(i);

		pixAff = std::dynamic_pointer_cast<SceneSegFile::FeatureTypeInt>(currSSFElement->features.features.at(SceneSegFile::FeaturesList::PixelAffiliation));

		// Generate surfel color
		labSegColor[0] = rand() % 255;
		labSegColor[1] = rand() % 255;
		labSegColor[2] = rand() % 255;

		// Set pixel colors
		for (int k = 0; k < pixAff->size; k++)
		{
			y = floor(pixAff->data[k] / 640.0);
			x = floor(pixAff->data[k] - 640.0 * y);
			coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
			coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
			coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
		}
	}
	// return image
	return coloredSegLab;
}

void SurfelGraph::Clear()
{
	RVL_DELETE_ARRAY(PtMem);
	RVL_DELETE_ARRAY(surfelBndMem);
	RVL_DELETE_ARRAY(surfelBndMem2);
	RVL_DELETE_ARRAY(BndMem);
	RVL_DELETE_ARRAY(surfelMap);
	RVL_DELETE_ARRAY(edgeMap);
	// RVL_DELETE_ARRAY(surfelBndMap);
	RVL_DELETE_ARRAY(nodeColor);
	if (NodeMem)
		RVL_DELETE_ARRAY(NodeMem)
	else
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(edgeMarkMap);
	RVL_DELETE_ARRAY(neighborEdge);
	RVL_DELETE_ARRAY(EdgeArray.Element);
	RVL_DELETE_ARRAY(vertexArray.Element);
	RVL_DELETE_ARRAY(vertexEdgeArray.Element);
	RVL_DELETE_ARRAY(surfelVertexList.Element);
	RVL_DELETE_ARRAY(surfelVertexMem);
	RVL_DELETE_ARRAY(vertexDisplayLineArray.Element);
	RVL_DELETE_ARRAY(vertexDisplayLineArrayMem);
	RVL_DELETE_ARRAY(bVertexAssigned);
	RVL_DELETE_ARRAY(iVertexMem);
	RVL_DELETE_ARRAY(momentsMem);
	RVL_DELETE_ARRAY(surfelRefPtMem);
	RVL_DELETE_ARRAY(planarSurfaceSurfelMem);
	RVL_DELETE_ARRAY(DisplayData.edgeFeatureIdxArray);
	RVL_DELETE_ARRAY(polygonDataMem);
	RVL_DELETE_ARRAY(polyEdges.Element);
	RVL_DELETE_ARRAY(polygonVerticesS.Element);
	RVL_DELETE_ARRAY(triangleMem);
}

#ifdef RVLSURFELGRAPH_VERTEX_DETECTION_VERSION_1

void SurfelGraph::DetectVertices(
	Mesh *pMesh)
{
	QList<Vertex> *pVertexList = &vertexList;

	RVLQLIST_INIT(pVertexList);

	nVertexSurfelRelations = 0;

	int nVertices = 0;

	RVL_DELETE_ARRAY(surfelVertexList.Element);

	surfelVertexList.Element = new QList<QLIST::Index>[NodeArray.n];
	surfelVertexList.n = NodeArray.n;

	bool *bVisited = new bool[pMesh->NodeArray.n];

	memset(bVisited, 0, pMesh->NodeArray.n * sizeof(bool));

	float q = TIVertexToleranceAngle * DEG2RAD;
	float cq = cos(q);
	float sq = sin(q);

	// float csEdgeTangentAngle = cos(edgeTangentAngle * DEG2RAD);
	// float snEdgeTangentAngle = sqrt(1.0f - csEdgeTangentAngle * csEdgeTangentAngle);

	// bool *bVisited = new bool[NodeArray.n];

	// memset(bVisited, 0, NodeArray.n * sizeof(bool));

	int iSurfel, iSurfel_;
	// int iSurfel1, iSurfel2;
	// int iPrevSurfel;
	int iBoundary;
	int iPointEdge;
	int iPt, iPt_, iPt__;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_, *pLastEdgePtr;
	// MeshEdge *pEdge;
	QList<MeshEdgePtr> *pEdgeList;
	Vertex *pVertex, *pVertex_;
	Point *pPt, *pPt_;
	// Point *pPt_;
	QList<QLIST::Index> *pSurfelVertexList;
	float *N, *P_, *N1, *N2, *N3;
	// float *N1, *N2;
	// float N2_[3], VTmp[3];
	// float fTmp;
	int nPlanarFeatures, nEdgeFeatures;
	int nFeatures, iFeature, iFeature_, iFeature__;
	int iF[3], iP[3];
	int iEdgeFeature, iEdgeFeature_, iEdgeFeature__;
	bool bSmallestIndex;
	Surfel *pSurfel_, *pEdgeFeature, *pFeature, *pF;
	Surfel *pFeature_[3];
	int iiF;
	bool bCycleCompleted;
	int i, j;
	float *P;
	float fnFeatures;
	BYTE bConvex[3];
	float VTmp[3], N3_[3];
	float fTmp, c13, c23;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		RVLQLIST_INIT(pSurfelVertexList);

		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		if (pSurfel->size <= 1)
			continue;

		N = pSurfel->N;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				// if (iPt == 296509)
				//	int debug = 0;

				if (bVisited[iPt])
					continue;

				pPt = pMesh->NodeArray.Element + iPt;

				iFeature = (pPt->bBoundary ? edgeMap[iPt] : iSurfel);

				if (iFeature == -1)
					continue;

				bVisited[iPt] = true;

				// if (iPt / 640 == 304364)
				//	int debug = 0;

				pFeature = (iFeature >= 0 ? NodeArray.Element + iFeature : NULL);

				iFeature__ = iFeature;

				bCycleCompleted = false;

				pEdgeList = &(pPt->EdgeList);

				pEdgePtr_ = pEdgeList->pFirst;

				while (true) // for every neighbor of iPt
				{
					iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

					if (!bVisited[iPt_])
					{
						pPt_ = pMesh->NodeArray.Element + iPt_;

						iFeature_ = (pPt_->bBoundary ? edgeMap[iPt_] : surfelMap[iPt_]);

						if (iFeature_ != iFeature)
						{
							if (iFeature__ != iFeature && iFeature__ != iFeature_)
							{
								// if (iFeature_ == 161 || iFeature__ == 161)
								//	int debug = 0;

								iF[0] = iFeature;
								iF[1] = iFeature__;
								iF[2] = iFeature_;

								//// Determine convex/concave edges.

								pFeature_[0] = pFeature_[1] = pFeature_[2] = NULL;

								iiF = 0;

								for (i = 0; i < 3; i++)
									if (iF[i] >= 0)
										pFeature_[iiF++] = NodeArray.Element + iF[i];

								RVLNULL3VECTOR(bConvex);

								if (pFeature_[0])
								{
									for (i = 0; i < pFeature_[0]->imgAdjacency.size(); i++)
									{
										pF = pFeature_[0]->imgAdjacency.at(i);

										if (pF == pFeature_[1])
											bConvex[0] = (pFeature_[0]->imgAdjacencyDescriptors.at(i)->cupyDescriptor[0] >= 0);
										else if (pF == pFeature_[2])
											bConvex[2] = (pFeature_[0]->imgAdjacencyDescriptors.at(i)->cupyDescriptor[0] >= 0);
									}

									if (pFeature_[1])
									{
										for (i = 0; i < pFeature_[1]->imgAdjacency.size(); i++)
										{
											pF = pFeature_[1]->imgAdjacency.at(i);

											if (pF == pFeature_[2])
												bConvex[1] = (pFeature_[1]->imgAdjacencyDescriptors.at(i)->cupyDescriptor[0] >= 0);
										}
									}

									// Create vertex.

									RVLMEM_ALLOC_STRUCT(pMem, Vertex, pVertex);

									// Compute vertex position.

									P = pVertex->P;

									nFeatures = 0;

									iP[0] = iPt;
									iP[1] = iPt__;
									iP[2] = iPt_;

									RVLNULL3VECTOR(P);

									for (i = 0; i < 3; i++)
									{
										P_ = pMesh->NodeArray.Element[iP[i]].P;

										RVLSUM3VECTORS(P, P_, P);

										if (iF[i] >= 0)
											nFeatures++;
									}

									RVLSCALE3VECTOR2(P, 3.0f, P);

									// Update nVertexSurfelRelations.

									nVertexSurfelRelations += nFeatures;

									// Classify vertex.

									pVertex->type = (nFeatures >= 2 ? bConvex[0] + bConvex[1] + bConvex[2] : 4);

									// Is the vertex on an occluded edge?

									pVertex->bForeground = true;

									for (i = 0; i < 3; i++)
									{
										if (pMesh->NodeArray.Element[iP[i]].flags & RVLMESH_POINT_FLAG_BACKGROUND)
										{
											pVertex->bForeground = false;

											break;
										}
									}

									// Fill iSurfelArray

									pVertex->iSurfelArray.n = nFeatures;

									RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, nFeatures, pVertex->iSurfelArray.Element);

									j = 0;

									for (i = 0; i < 3; i++)
										if (iF[i] >= 0)
											pVertex->iSurfelArray.Element[j++] = iF[i];

									// Determine normal hull.

									pVertex->normalHull.n = 0;

									RVLMEM_ALLOC_STRUCT_ARRAY(pMem, NormalHullElement, (pVertex->type == 1 ? 4 : nFeatures), pVertex->normalHull.Element);

									// if (pVertex->type == 1)
									//{
									//	for (i = 0; i < 3; i++)
									//		if (bConvex[i])
									//			break;

									//	N1 = pFeature_[i]->N;
									//	N2 = pFeature_[(i + 1) % 3]->N;

									//	UpdateNormalHull(pVertex->normalHull, N1);
									//	UpdateNormalHull(pVertex->normalHull, N2);

									//	RVLCROSSPRODUCT3(N1, N2, VTmp);
									//	RVLNORM3(VTmp, fTmp);
									//	RVLSCALE3VECTOR(VTmp, sq, VTmp);
									//	RVLSCALE3VECTOR(N1, cq, N3_);
									//	RVLSUM3VECTORS(VTmp, N3_, N3_);
									//	UpdateNormalHull(pVertex->normalHull, N3_);
									//	RVLSCALE3VECTOR(N2, cq, N3_);
									//	RVLSUM3VECTORS(VTmp, N3_, N3_);
									//	UpdateNormalHull(pVertex->normalHull, N3_);
									//}
									// else
									{
										for (i = 0; i < 3; i++)
											if (iF[i] >= 0)
												UpdateNormalHull(pVertex->normalHull, NodeArray.Element[iF[i]].N);
									}

									// Add vertex to the vertex list.

									RVLQLIST_ADD_ENTRY(pVertexList, pVertex);

									nVertices++;

									if (bContactEdgeVertices)
									{
										// If pVertex->type == 1, then add one more vertex.

										if (pVertex->type == 1 && nFeatures == 3)
										{
											for (i = 0; i < 3; i++)
												if (bConvex[i])
													break;

											N1 = pFeature_[i]->N;
											N2 = pFeature_[(i + 1) % 3]->N;
											N3 = pFeature_[(i + 2) % 3]->N;

											c13 = RVLDOTPRODUCT3(N1, N3);
											c23 = RVLDOTPRODUCT3(N2, N3);

											if (RVLABS(c13) <= 0.87 && RVLABS(c23) <= 0.87)
											{
												RVLMEM_ALLOC_STRUCT(pMem, Vertex, pVertex_);

												P_ = pVertex_->P;

												RVLCOPY3VECTOR(P, P_, P_);

												nVertexSurfelRelations += nFeatures;

												pVertex_->type = 3;

												pVertex_->iSurfelArray.n = nFeatures;

												RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, nFeatures, pVertex_->iSurfelArray.Element);

												memcpy(pVertex_->iSurfelArray.Element, pVertex->iSurfelArray.Element, nFeatures * sizeof(int));

												pVertex_->normalHull.n = 0;

												RVLMEM_ALLOC_STRUCT_ARRAY(pMem, NormalHullElement, nFeatures, pVertex_->normalHull.Element);

												UpdateNormalHull(pVertex_->normalHull, N1);
												UpdateNormalHull(pVertex_->normalHull, N2);
												RVLNEGVECT3(N3, N3_);
												UpdateNormalHull(pVertex_->normalHull, N3_);

												RVLQLIST_ADD_ENTRY(pVertexList, pVertex_);

												nVertices++;
											}
										}
									} // if (bContactEdgeVertices)
								}

							} // if (iFeature__ != iFeature && iFeature__ != iFeature_)

							iFeature__ = iFeature_;
						} // if (iFeature_ != iFeature)
					}	  // if (!bVisited[iPt_])
					else
						iFeature__ = iFeature;

					if (pEdgePtr_ == pEdgeList->pFirst && bCycleCompleted)
						break;

					pEdgePtr_ = pEdgePtr_->pNext;

					if (pEdgePtr_ == NULL)
					{
						if (pPt->bBoundary)
							break;
						else
						{
							pEdgePtr_ = pEdgeList->pFirst;

							bCycleCompleted = true;
						}
					}

					iPt__ = iPt_;
				} // for every neighbor of iPt
			}	  // for each point-edge on the boundary contour
		}		  // for each boundary contour
	}			  // for each surfel

	delete[] bVisited;

	RVL_DELETE_ARRAY(vertexArray.Element);

	vertexArray.Element = new Vertex *[nVertices];
	vertexArray.n = nVertices;

	QLIST::CreatePtrArray<Vertex>(&vertexList, &vertexArray);

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
}

#endif

#ifdef RVLSURFELGRAPH_VERTEX_DETECTION_VERSION_2

void SurfelGraph::DetectVertices(
	Mesh *pMesh)
{
#ifdef RVLSURFELGRAPH_VERTEX_DETECTION_DEBUG
	FILE *fp = fopen("vertex_detection.txt", "w");
	fclose(fp);
#endif

	QList<Vertex> *pVertexList = &vertexList;

	RVLQLIST_INIT(pVertexList);

	nVertexSurfelRelations = 0;

	int nVertices = 0;

	RVL_DELETE_ARRAY(surfelVertexList.Element);

	surfelVertexList.Element = new QList<QLIST::Index>[NodeArray.n];
	surfelVertexList.n = NodeArray.n;

	Array<Vertex *> boundaryVertexArray;

	boundaryVertexArray.Element = new Vertex *[2 * pMesh->EdgeArray.n];

	bool *bNewVertex = new bool[2 * pMesh->EdgeArray.n];

	Pair<int, int> *iNeighborSurfels = new Pair<int, int>[2 * pMesh->EdgeArray.n];

	float q = TIVertexToleranceAngle * DEG2RAD;
	float cq = cos(q);
	float sq = sin(q);

	QList<SURFEL::VertexEdge> *pVertexEdgeList = &vertexEdgeList;

	RVLQLIST_INIT(pVertexEdgeList);

	vertexEdgeArray.n = 0;

	bool *bBelongsToRefVertex = new bool[NodeArray.n];

	memset(bBelongsToRefVertex, 0, NodeArray.n * sizeof(bool));

	Vertex **edgeConnectorVertexMap = new Vertex *[2 * pMesh->EdgeArray.n];

	memset(edgeConnectorVertexMap, 0, 2 * pMesh->EdgeArray.n * sizeof(Vertex *));

	// float csEdgeTangentAngle = cos(edgeTangentAngle * DEG2RAD);
	// float snEdgeTangentAngle = sqrt(1.0f - csEdgeTangentAngle * csEdgeTangentAngle);

	// bool *bVisited = new bool[NodeArray.n];

	// memset(bVisited, 0, NodeArray.n * sizeof(bool));

	int iSurfel, iSurfel_;
	// int iSurfel1, iSurfel2;
	// int iPrevSurfel;
	int iBoundary;
	int iPointEdge;
	int iPt, iPt_, iPt__;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_, *pLastEdgePtr;
	// MeshEdge *pEdge;
	QList<MeshEdgePtr> *pEdgeList;
	Vertex *pVertex, *pVertex_;
	Point *pPt, *pPt_;
	// Point *pPt_;
	QList<QLIST::Index> *pSurfelVertexList;
	float *N, *P_, *N1, *N2, *N3;
	// float *N1, *N2;
	// float N2_[3], VTmp[3];
	// float fTmp;
	int nPlanarFeatures, nEdgeFeatures;
	int nFeatures, iFeature, iFeature_, iFeature__;
	int iF[3], iP[3];
	int iEdgeFeature, iEdgeFeature_, iEdgeFeature__;
	bool bSmallestIndex;
	Surfel *pSurfel_, *pEdgeFeature, *pFeature, *pF;
	Surfel *pFeature_[3];
	int iiF;
	int k;
	bool bFirst;
	bool bVertex;
	VertexEdge *pEdge;
	bool bNewVertex_;
	MeshEdgePtr *pEdgePtr__, *pEdgePtrOpp_;
	int nCommonSurfels;
	GRAPH::EdgePtr2<VertexEdge> *pVertexEdgePtr;
	int i, j;
	float *P;
	float fnFeatures;
	BYTE bConvex[3];
	float VTmp[3], N3_[3];
	float fTmp, c13, c23;
	QList<GRAPH::EdgePtr2<VertexEdge>> *pVertexEdgeList_;
	float NOpp[3];

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		// if (iSurfel == 47)
		//	int debug = 0;

		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		RVLQLIST_INIT(pSurfelVertexList);

		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->size <= 1)
			continue;

		N = pSurfel->N;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			boundaryVertexArray.n = 0;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				// if (iPt == 296509)
				//	int debug = 0;

				// if(iPointEdge == 169)
				//	int debug = 0;

				pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				pPt = pMesh->NodeArray.Element + iPt;

				iFeature = iSurfel;

				if (iFeature == -1)
					continue;

				// if (iPt / 640 == 304364)
				//	int debug = 0;

				pFeature = (iFeature >= 0 ? NodeArray.Element + iFeature : NULL);

				iFeature__ = iFeature;

				pEdgeList = &(pPt->EdgeList);

				pEdgePtr_ = pEdgePtr;

				pEdgePtr__ = NULL;

				bFirst = true;

				while (true) // for every neighbor of iPt
				{
					iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

					pPt_ = pMesh->NodeArray.Element + iPt_;

					iFeature_ = surfelMap[iPt_];

					if (iFeature_ == iFeature && !bFirst)
						break;

					pEdgePtrOpp_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr_);

					pVertex = edgeConnectorVertexMap[pEdgePtrOpp_ - pMesh->EdgePtrMem];

					bVertex = false;

					if (pVertex)
					{
						boundaryVertexArray.Element[boundaryVertexArray.n] = pVertex;

						bNewVertex[boundaryVertexArray.n] = false;

						iNeighborSurfels[boundaryVertexArray.n].a = iFeature__;
						iNeighborSurfels[boundaryVertexArray.n].b = iFeature_;

						boundaryVertexArray.n++;

						bVertex = true;
					}
					else if (pEdgePtr__)
					{
						pVertex = edgeConnectorVertexMap[pEdgePtr__ - pMesh->EdgePtrMem];

						if (pVertex)
						{
							boundaryVertexArray.Element[boundaryVertexArray.n] = pVertex;

							bNewVertex[boundaryVertexArray.n] = false;

							iNeighborSurfels[boundaryVertexArray.n].a = iFeature__;
							iNeighborSurfels[boundaryVertexArray.n].b = iFeature_;

							boundaryVertexArray.n++;

							bVertex = true;
						}
					}

					if (!bVertex)
					{
						bFirst = false;
						{
							if (iFeature__ != iFeature && iFeature__ != iFeature_)
							{
								// if (iFeature_ == 161 || iFeature__ == 161)
								//	int debug = 0;

								iF[0] = iFeature;
								iF[1] = iFeature__;
								iF[2] = iFeature_;

								//// Determine convex/concave edges.

								pFeature_[0] = pFeature_[1] = pFeature_[2] = NULL;

								iiF = 0;

								for (i = 0; i < 3; i++)
									if (iF[i] >= 0)
										pFeature_[iiF++] = NodeArray.Element + iF[i];

								RVLNULL3VECTOR(bConvex);

								if (pFeature_[0])
								{
#ifdef RVLSURFEL_IMAGE_ADJACENCY
									for (i = 0; i < pFeature_[0]->imgAdjacency.size(); i++)
									{
										pF = pFeature_[0]->imgAdjacency.at(i);

										if (pF == pFeature_[1])
											bConvex[0] = (pFeature_[0]->imgAdjacencyDescriptors.at(i)->cupyDescriptor[0] >= 0);
										else if (pF == pFeature_[2])
											bConvex[2] = (pFeature_[0]->imgAdjacencyDescriptors.at(i)->cupyDescriptor[0] >= 0);
									}

									if (pFeature_[1])
									{
										for (i = 0; i < pFeature_[1]->imgAdjacency.size(); i++)
										{
											pF = pFeature_[1]->imgAdjacency.at(i);

											if (pF == pFeature_[2])
												bConvex[1] = (pFeature_[1]->imgAdjacencyDescriptors.at(i)->cupyDescriptor[0] >= 0);
										}
									}
#endif

									// Create vertex.

									RVLMEM_ALLOC_STRUCT(pMem, Vertex, pVertex);

									pVertex->idx = nVertices;

									// Compute vertex position.

									P = pVertex->P;

									nFeatures = 0;

									iP[0] = iPt;
									iP[1] = iPt__;
									iP[2] = iPt_;

									RVLNULL3VECTOR(P);

									for (i = 0; i < 3; i++)
									{
										P_ = pMesh->NodeArray.Element[iP[i]].P;

										RVLSUM3VECTORS(P, P_, P);

										if (iF[i] >= 0)
											nFeatures++;
									}

									RVLSCALE3VECTOR2(P, 3.0f, P);

									// Update nVertexSurfelRelations.

									nVertexSurfelRelations += nFeatures;

									// Classify vertex.

#ifdef RVLSURFEL_IMAGE_ADJACENCY
									pVertex->type = (nFeatures >= 2 ? bConvex[0] + bConvex[1] + bConvex[2] : 4);
#else
									pVertex->type = 0;
#endif

									// Reset cluster ID.

									pVertex->iCluster = -1;

									// Fill iSurfelArray

									pVertex->iSurfelArray.n = nFeatures;

									RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, nFeatures, pVertex->iSurfelArray.Element);

									j = 0;

									for (i = 0; i < 3; i++)
										if (iF[i] >= 0)
											pVertex->iSurfelArray.Element[j++] = iF[i];

									// Determine normal hull.

									pVertex->normalHull.n = 0;

									RVLMEM_ALLOC_STRUCT_ARRAY(pMem, NormalHullElement, (pVertex->type == 1 ? 4 : nFeatures), pVertex->normalHull.Element);

									// if (pVertex->type == 1)
									//{
									//	for (i = 0; i < 3; i++)
									//		if (bConvex[i])
									//			break;

									//	N1 = pFeature_[i]->N;
									//	N2 = pFeature_[(i + 1) % 3]->N;

									//	UpdateNormalHull(pVertex->normalHull, N1);
									//	UpdateNormalHull(pVertex->normalHull, N2);

									//	RVLCROSSPRODUCT3(N1, N2, VTmp);
									//	RVLNORM3(VTmp, fTmp);
									//	RVLSCALE3VECTOR(VTmp, sq, VTmp);
									//	RVLSCALE3VECTOR(N1, cq, N3_);
									//	RVLSUM3VECTORS(VTmp, N3_, N3_);
									//	UpdateNormalHull(pVertex->normalHull, N3_);
									//	RVLSCALE3VECTOR(N2, cq, N3_);
									//	RVLSUM3VECTORS(VTmp, N3_, N3_);
									//	UpdateNormalHull(pVertex->normalHull, N3_);
									//}
									// else
									{
										for (i = 0; i < 3; i++)
											if (iF[i] >= 0)
											{
												pSurfel_ = NodeArray.Element + iF[i];

												if (bGroundContactVertices)
												{
													if (pSurfel_->flags & RVLSURFEL_FLAG_GND)
													{
														RVLNEGVECT3(pSurfel_->N, NOpp);

														UpdateNormalHull(pVertex->normalHull, NOpp);
													}
													else
														UpdateNormalHull(pVertex->normalHull, pSurfel_->N);
												}
												else
													UpdateNormalHull(pVertex->normalHull, pSurfel_->N);
											}
									}

									// Initialize edge list.

									pVertexEdgeList_ = &(pVertex->EdgeList);

									RVLQLIST_INIT(pVertexEdgeList_);

									// Add vertex to the vertex list.

									RVLQLIST_ADD_ENTRY(pVertexList, pVertex);

									nVertices++;

									// Assign pVertex to edge connectors.

									edgeConnectorVertexMap[pEdgePtrOpp_ - pMesh->EdgePtrMem] =
										edgeConnectorVertexMap[pEdgePtr__ - pMesh->EdgePtrMem] = pVertex;

									boundaryVertexArray.Element[boundaryVertexArray.n] = pVertex;

									bNewVertex[boundaryVertexArray.n] = true;

									iNeighborSurfels[boundaryVertexArray.n].a = iFeature__;
									iNeighborSurfels[boundaryVertexArray.n].b = iFeature_;

									boundaryVertexArray.n++;

#ifdef RVLSURFEL_IMAGE_ADJACENCY
									if (bContactEdgeVertices)
									{
										// If pVertex->type == 1, then add one more vertex.

										if (pVertex->type == 1 && nFeatures == 3)
										{
											for (i = 0; i < 3; i++)
												if (bConvex[i])
													break;

											N1 = pFeature_[i]->N;
											N2 = pFeature_[(i + 1) % 3]->N;
											N3 = pFeature_[(i + 2) % 3]->N;

											c13 = RVLDOTPRODUCT3(N1, N3);
											c23 = RVLDOTPRODUCT3(N2, N3);

											if (RVLABS(c13) <= 0.87 && RVLABS(c23) <= 0.87)
											{
												RVLMEM_ALLOC_STRUCT(pMem, Vertex, pVertex_);

												P_ = pVertex_->P;

												RVLCOPY3VECTOR(P, P_, P_);

												nVertexSurfelRelations += nFeatures;

												pVertex_->type = 3;

												pVertex_->iSurfelArray.n = nFeatures;

												RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, nFeatures, pVertex_->iSurfelArray.Element);

												memcpy(pVertex_->iSurfelArray.Element, pVertex->iSurfelArray.Element, nFeatures * sizeof(int));

												pVertex_->normalHull.n = 0;

												RVLMEM_ALLOC_STRUCT_ARRAY(pMem, NormalHullElement, nFeatures, pVertex_->normalHull.Element);

												UpdateNormalHull(pVertex_->normalHull, N1);
												UpdateNormalHull(pVertex_->normalHull, N2);
												RVLNEGVECT3(N3, N3_);
												UpdateNormalHull(pVertex_->normalHull, N3_);

												RVLQLIST_ADD_ENTRY(pVertexList, pVertex_);

												nVertices++;
											}
										}
									} // if (bContactEdgeVertices)
#endif

#ifdef RVLSURFELGRAPH_VERTEX_DETECTION_DEBUG
									FILE *fp = fopen("vertex_detection.txt", "a");

									fprintf(fp, "%d\t%d\t%d\t%d\n", pVertex->idx, pVertex->iSurfelArray.Element[0], pVertex->iSurfelArray.Element[1], pVertex->iSurfelArray.Element[2]);

									fclose(fp);
#endif
								} // if (pFeature_[0]) then create vertex.
							}	  // if (iFeature__ != iFeature && iFeature__ != iFeature_)
						}		  // if (iFeature_ != iFeature)
					}			  // if (!bVertex)

					iFeature__ = iFeature_;

					pEdgePtr__ = pEdgePtr_;

					RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr_);

					iPt__ = iPt_;
				} // for every neighbor of iPt
			}	  // for each point-edge on the boundary contour

			/// Connect vertices by edges.

			pVertex_ = boundaryVertexArray.Element[boundaryVertexArray.n - 1];

			// bNewVertex_ = bNewVertex[boundaryVertexArray.n - 1];

			for (i = 0; i < boundaryVertexArray.n; i++)
			{
				pVertex = boundaryVertexArray.Element[i];

				// if (pVertex->idx == 153 && pVertex_->idx == 581 || pVertex_->idx == 153 && pVertex->idx == 581)
				//	int debug = 0;

				// if (bNewVertex_ || bNewVertex[i])
				{
					// Check if pVertex and pVertex_ are connected.

					pVertexEdgePtr = pVertex->EdgeList.pFirst;

					while (pVertexEdgePtr)
					{
						if (RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pVertexEdgePtr) == pVertex_->idx)
						{
							pEdge = pVertexEdgePtr->pEdge;

							if (pEdge->iSurfel[0] == iFeature && pEdge->iSurfel[1] == iNeighborSurfels[i].a ||
								pEdge->iSurfel[1] == iFeature && pEdge->iSurfel[0] == iNeighborSurfels[i].a)
								break;
						}

						pVertexEdgePtr = pVertexEdgePtr->pNext;
					}

					if (pVertexEdgePtr == NULL)
					{
						// Check if pVertex and pVertex_ have two common surfels.

						// nCommonSurfels = 0;

						// for (j = 0; j < pVertex->iSurfelArray.n; j++)
						//	for (k = 0; k < pVertex_->iSurfelArray.n; k++)
						//		if (pVertex->iSurfelArray.Element[j] == pVertex_->iSurfelArray.Element[k])
						//		{
						//			nCommonSurfels++;

						//			break;
						//		}

						// if (nCommonSurfels >= 2)
						if (iNeighborSurfels[i].a == iNeighborSurfels[(i + boundaryVertexArray.n - 1) % boundaryVertexArray.n].b)
						{
							// Connect pVertex and pVertex_.

							pEdge = ConnectNodes<Vertex, VertexEdge, GRAPH::EdgePtr2<VertexEdge>>(pVertex, pVertex_, pVertex->idx, pVertex_->idx, pMem);

							RVLQLIST_ADD_ENTRY(pVertexEdgeList, pEdge);

							pEdge->iSurfel[0] = iFeature;
							pEdge->iSurfel[1] = iNeighborSurfels[i].a;
							pEdge->idx = vertexEdgeArray.n;

							// Only for debugging purpose!!!

							// if (pEdge->iSurfel[0] >= 0 && pEdge->iSurfel[1] >= 0)
							//{
							//	bool debug[2];

							//	debug[0] = debug[1] = false;

							//	for (j = 0; j < pVertex->iSurfelArray.n; j++)
							//		for (k = 0; k < 2; k++)
							//			if (pVertex->iSurfelArray.Element[j] == pEdge->iSurfel[k])
							//				debug[k] = true;

							//	if (!(debug[0] && debug[1]))
							//		int debug_ = 0;

							//	debug[0] = debug[1] = false;

							//	for (j = 0; j < pVertex_->iSurfelArray.n; j++)
							//		for (k = 0; k < 2; k++)
							//			if (pVertex_->iSurfelArray.Element[j] == pEdge->iSurfel[k])
							//				debug[k] = true;

							//	if (!(debug[0] && debug[1]))
							//		int debug_ = 0;
							//}

							// Increment vertex edge counter.

							vertexEdgeArray.n++;
						}
					}
				}

				pVertex_ = pVertex;

				bNewVertex_ = bNewVertex[i];
			}

			///
		} // for each boundary contour
	}	  // for each surfel

	delete[] edgeConnectorVertexMap;
	delete[] bBelongsToRefVertex;
	delete[] boundaryVertexArray.Element;
	delete[] bNewVertex;
	delete[] iNeighborSurfels;

	RVL_DELETE_ARRAY(vertexArray.Element);

	vertexArray.Element = new Vertex *[nVertices];
	vertexArray.n = nVertices;

	QLIST::CreatePtrArray<Vertex>(&vertexList, &vertexArray);

	RVL_DELETE_ARRAY(vertexEdgeArray.Element);

	vertexEdgeArray.Element = new SURFEL::VertexEdge *[vertexEdgeArray.n];

	QLIST::CreatePtrArray<SURFEL::VertexEdge>(&vertexEdgeList, &vertexEdgeArray);

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
} // SurfelGraph::DetectVertices()

#endif

#ifdef RVLSURFELGRAPH_VERTEX_DETECTION_VERSION_0

void SurfelGraph::DetectVertices(
	Mesh *pMesh)
{
	QList<Vertex> *pVertexList = &vertexList;

	RVLQLIST_INIT(pVertexList);

	nVertexSurfelRelations = 0;

	int nVertices = 0;

	RVL_DELETE_ARRAY(surfelVertexList.Element);

	surfelVertexList.Element = new QList<QLIST::Index>[NodeArray.n];
	surfelVertexList.n = NodeArray.n;

	// float csEdgeTangentAngle = cos(edgeTangentAngle * DEG2RAD);
	// float snEdgeTangentAngle = sqrt(1.0f - csEdgeTangentAngle * csEdgeTangentAngle);

	bool *bVisited = new bool[NodeArray.n];

	memset(bVisited, 0, NodeArray.n * sizeof(bool));

	int iSurfel, iSurfel_;
	// int iSurfel1, iSurfel2;
	// int iPrevSurfel;
	int iBoundary;
	int iPointEdge;
	int iPt, iPt_, iPt__;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_, *pLastEdgePtr;
	// MeshEdge *pEdge;
	QList<MeshEdgePtr> *pEdgeList;
	Vertex *pVertex;
	Point *pPt;
	// Point *pPt_;
	QList<QLIST::Index> *pSurfelVertexList;
	float *N;
	// float *N1, *N2;
	// float N2_[3], VTmp[3];
	// float fTmp;
	int nPlanarFeatures, nEdgeFeatures, nFeatures, iFeature;
	int iEdgeFeature, iEdgeFeature_, iEdgeFeature__;
	bool bSmallestIndex;
	Surfel *pSurfel_, *pEdgeFeature;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		RVLQLIST_INIT(pSurfelVertexList);

		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		if (pSurfel->size <= 1)
			continue;

		N = pSurfel->N;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				// Determine if iPt is the point with the smallest index in its immediate neighborhood.

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				pPt = pMesh->NodeArray.Element + iPt;

				if (pPt->bBoundary)
					iEdgeFeature = edgeMap[iPt];

				bVisited[iSurfel] = true;

				nPlanarFeatures = 1;

				pEdgeList = &(pPt->EdgeList);

				pEdgePtr_ = pEdgeList->pFirst;

				while (pEdgePtr_)
				{
					iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

					iSurfel_ = surfelMap[iPt_];

					if (iSurfel_ >= 0 && iSurfel_ < NodeArray.n)
					{
						if (!bVisited[iSurfel_])
						{
							if (iSurfel_ < iSurfel)
								break;

							nPlanarFeatures++;

							bVisited[iSurfel_] = true;
						}
					}

					if (pEdgePtr_->pNext == NULL)
						pLastEdgePtr = pEdgePtr_;

					pEdgePtr_ = pEdgePtr_->pNext;
				}

				bSmallestIndex = (pEdgePtr_ == NULL);

				// Reset bVisited.

				bVisited[iSurfel] = false;

				pEdgePtr_ = pEdgeList->pFirst;

				while (pEdgePtr_)
				{
					iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

					iSurfel_ = surfelMap[iPt_];

					if (iSurfel_ >= 0 && iSurfel_ < NodeArray.n)
						bVisited[iSurfel_] = false;

					pEdgePtr_ = pEdgePtr_->pNext;
				}

				nEdgeFeatures = 0;

				if (bSmallestIndex) // If iPt is the point with the smallest index in its immediate neighborhood
				{
					if (pPt->bBoundary)
					{
						if (iEdgeFeature >= 0)
							nEdgeFeatures++;

						pEdgePtr_ = pEdgeList->pFirst;

						iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

						iEdgeFeature_ = edgeMap[iPt_];

						if (iEdgeFeature_ >= 0 && iEdgeFeature_ != iEdgeFeature)
							nEdgeFeatures++;

						iPt__ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pLastEdgePtr);

						iEdgeFeature__ = edgeMap[iPt__];

						if (iEdgeFeature__ >= 0 && iEdgeFeature__ != iEdgeFeature && iEdgeFeature__ != iEdgeFeature_)
							nEdgeFeatures++;

						if (nPlanarFeatures == 1 && nEdgeFeatures >= 2)
						{
							if ((iEdgeFeature_ >= 0 && iEdgeFeature > iEdgeFeature_) || (iEdgeFeature__ >= 0 && iEdgeFeature > iEdgeFeature__))
								bSmallestIndex = false;
						}
					}
				} // if (pEdgePtr_ == NULL)

				nFeatures = nPlanarFeatures + nEdgeFeatures;

				if (bSmallestIndex && nFeatures >= 3) // If iPt is the point with the smallest index in its immediate neighborhood
													  // and at least three features meet in iPt, then this point is a vertex.
				{
					// if (iPt == 221223)
					//	int debug = 0;

					// Create vertex.

					RVLMEM_ALLOC_STRUCT(pMem, Vertex, pVertex);

					pVertex->bEdge = pPt->bBoundary;

					RVLCOPY3VECTOR(pPt->P, pVertex->P);

					RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, nFeatures, pVertex->iSurfelArray.Element);

					RVLMEM_ALLOC_STRUCT_ARRAY(pMem, NormalHullElement, nFeatures, pVertex->normalHull.Element);

					nVertexSurfelRelations += nFeatures;

					pVertex->normalHull.n = 0;

					iFeature = 0;

					bVisited[iSurfel] = true;

					pVertex->iSurfelArray.Element[iFeature++] = iSurfel;

					UpdateNormalHull(pVertex->normalHull, pSurfel->N);

					pEdgePtr_ = pEdgeList->pFirst;

					while (pEdgePtr_)
					{
						iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

						iSurfel_ = surfelMap[iPt_];

						if (iSurfel_ >= 0 && iSurfel_ < NodeArray.n)
						{
							if (!bVisited[iSurfel_])
							{
								bVisited[iSurfel_] = true;

								pVertex->iSurfelArray.Element[iFeature++] = iSurfel_;

								pSurfel_ = NodeArray.Element + iSurfel_;

								UpdateNormalHull(pVertex->normalHull, pSurfel_->N);
							}
						}

						pEdgePtr_ = pEdgePtr_->pNext;
					}

					if (pPt->bBoundary)
					{
						if (iEdgeFeature >= 0)
						{
							pVertex->iSurfelArray.Element[iFeature++] = iEdgeFeature;

							pEdgeFeature = NodeArray.Element + iEdgeFeature;

							UpdateNormalHull(pVertex->normalHull, pEdgeFeature->N);
						}

						pEdgePtr_ = pEdgeList->pFirst;

						iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

						iEdgeFeature_ = edgeMap[iPt_];

						if (iEdgeFeature_ >= 0 && iEdgeFeature_ != iEdgeFeature)
						{
							pVertex->iSurfelArray.Element[iFeature++] = iEdgeFeature_;

							pEdgeFeature = NodeArray.Element + iEdgeFeature_;

							UpdateNormalHull(pVertex->normalHull, pEdgeFeature->N);
						}

						iPt__ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pLastEdgePtr);

						iEdgeFeature__ = edgeMap[iPt__];

						if (iEdgeFeature__ >= 0 && iEdgeFeature__ != iEdgeFeature && iEdgeFeature__ != iEdgeFeature_)
						{
							pVertex->iSurfelArray.Element[iFeature++] = iEdgeFeature__;

							pEdgeFeature = NodeArray.Element + iEdgeFeature__;

							UpdateNormalHull(pVertex->normalHull, pEdgeFeature->N);
						}
					}

					pVertex->iSurfelArray.n = nFeatures;

					// if (pVertex->normalHull.n < 3)
					//	int debug = 0;

					// if (iFeature != nFeatures)
					//	int debug = 0;

					// Reset bVisited.

					bVisited[iSurfel] = false;

					pEdgePtr_ = pEdgeList->pFirst;

					while (pEdgePtr_)
					{
						iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

						iSurfel_ = surfelMap[iPt_];

						if (iSurfel_ >= 0 && iSurfel_ < NodeArray.n)
							bVisited[iSurfel_] = false;

						pEdgePtr_ = pEdgePtr_->pNext;
					}

					// Add vertex to the vertex list.

					RVLQLIST_ADD_ENTRY(pVertexList, pVertex);

					nVertices++;
				} // if (bSmallestIndex && nFeatures >= 3)

#ifdef NEVER
				// Old version.

				pEdgePtr_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr_);

				pPt = pMesh->NodeArray.Element + iPt;

				pEdgeList = &(pPt->EdgeList);

				iPrevSurfel = -1;

				while (true) // for each neighboring point of the point iPt
				{
					if (pPt->bBoundary && pEdgePtr_->pNext == NULL && iPrevSurfel < pMesh->NodeArray.n)
						iSurfel_ = pMesh->NodeArray.n;
					else
					{
						RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr_);

						RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr_, pEdge, iPt_);

						iSurfel_ = pSurfels->surfelMap[iPt_];

						if (iSurfel_ == iSurfel)
							break;
					}

					// debug++;

					// if (debug >= 20)
					//	debug = 0;

					if (iSurfel_ >= 0 && iSurfel_ <= pMesh->NodeArray.n)
					{
						if (iPrevSurfel >= 0 && iPrevSurfel != iSurfel_)
						{
							if (iSurfel < iSurfel_ && iSurfel < iPrevSurfel)
							{
								RVLMEM_ALLOC_STRUCT(pMem, Vertex, pVertex);

								RVLCOPY3VECTOR(pPt->P, pVertex->P);

								RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, 3, pVertex->iSurfelArray.Element);

								if (iSurfel_ > iPrevSurfel)
								{
									iSurfel1 = iPrevSurfel;
									iSurfel2 = iSurfel_;
								}
								else
								{
									iSurfel1 = iSurfel_;
									iSurfel2 = iPrevSurfel;
								}

								pVertex->iSurfelArray.Element[0] = iSurfel;
								pVertex->iSurfelArray.Element[1] = iSurfel1;
								pVertex->iSurfelArray.Element[2] = iSurfel2;
								pVertex->iSurfelArray.n = 3;

								N1 = pSurfels->NodeArray.Element[iSurfel1].N;

								if (iSurfel2 == pMesh->NodeArray.n)
								{
									RVLSUM3VECTORS(N, N1, VTmp);

									// fTmp = csEdgeTangentAngle / sqrt(RVLDOTPRODUCT3(VTmp, VTmp));

									RVLSCALE3VECTOR(VTmp, fTmp, VTmp);

									RVLCROSSPRODUCT3(N, N1, )

									N2 = N2_;
								}
								else
									N2 = pSurfels->NodeArray.Element[iSurfel2].N;

								RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RECOG::PSGM_::NormalHullElement, 3, pVertex->normalHull.Element);
								pVertex->normalHull.n = 0;
								UpdateNormalHull(pVertex->normalHull, N);
								UpdateNormalHull(pVertex->normalHull, N1);
								UpdateNormalHull(pVertex->normalHull, N2);

								RVLQLIST_ADD_ENTRY(pVertexList, pVertex);

								nVertices++;

								nVertexSurfelRelations += 3;
							}
						}
					} // if (iSurfel_ >= 0 && iSurfel_ <= pMesh->NodeArray.n)

					iPrevSurfel = iSurfel_;
				} // for each neighboring point of the point iPt
#endif
			} // for each point-edge on the boundary contour
		}	  // for each boundary contour
	}		  // for each surfel

	delete[] bVisited;

	RVL_DELETE_ARRAY(vertexArray.Element);

	vertexArray.Element = new Vertex *[nVertices];
	vertexArray.n = nVertices;

	QLIST::CreatePtrArray<Vertex>(&vertexList, &vertexArray);

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
}
#endif

// Nh(i) = norm(N(i+1) x N(i))

void SurfelGraph::UpdateNormalHull(
	Array<NormalHullElement> &NHull,
	float *N)
{
	SURFEL::UpdateNormalHull(NHull, N);
}

float SurfelGraph::DistanceFromNormalHull(
	Array<SURFEL::NormalHullElement> &NHull,
	float *N)
{
	return SURFEL::DistanceFromNormalHull(NHull, N);
}

bool SurfelGraph::ComputeTangent(
	Array<int> iVertexArray,
	float *N,
	float o,
	bool bNormalTest,
	bool bVisibilityTest,
	float maxDistFromNHull,
	float maxedmax,
	float &d,
	bool &bd,
	int &iVertex)
{
	bool bdmax = false;

	bool bdmaxValid = false;

	int j;
	int iVertex_;
	SURFEL::Vertex *pVertex;
	float *P;
	float d_;
	float distFromNHull;
	float dSmaxValid;

	for (j = 0; j < iVertexArray.n; j++)
	{
		iVertex_ = iVertexArray.Element[j];

		// fprintf(fpDebug4, "iVertex_=%d\n", iVertex_);

		pVertex = vertexArray.Element[iVertex_];

		P = pVertex->P;

		d_ = RVLDOTPRODUCT3(N, P);

		if (bdmax)
		{
			if (o * d_ > o * d)
			{
				d = d_;

				iVertex = iVertex_;
			}
		}
		else
		{
			d = d_;

			iVertex = iVertex_;

			bdmax = true;
		}

		if (bNormalTest)
		{
			if (d_ < 0.0f || !bVisibilityTest)
			{
				distFromNHull = DistanceFromNormalHull(pVertex->normalHull, N);

				if (distFromNHull <= maxDistFromNHull)
				{
					if (bdmaxValid)
					{
						if (o * d_ > o * dSmaxValid)
							dSmaxValid = d_;
					}
					else
					{
						dSmaxValid = d_;

						bdmaxValid = true;
					}
				}
			}
		}
	} // for each vertex

	if (bdmaxValid)
		bd = (o * (d - dSmaxValid) <= maxedmax);
	else
		bd = false;

	return bdmax;
}

bool SurfelGraph::ComputeTangent(
	Array<int> iVertexArray,
	float *PArray,
	float *N,
	float o,
	bool bNormalTest,
	bool bVisibilityTest,
	float maxDistFromNHull,
	float maxedmax,
	float &d,
	bool &bd,
	int &iVertex)
{
	bool bdmax = false;

	bool bdmaxValid = false;

	int j;
	int iVertex_;
	SURFEL::Vertex *pVertex;
	float *P;
	float d_;
	float distFromNHull;
	float dSmaxValid;

	for (j = 0; j < iVertexArray.n; j++)
	{
		iVertex_ = iVertexArray.Element[j];

		// fprintf(fpDebug4, "iVertex_=%d\n", iVertex_);

		pVertex = vertexArray.Element[iVertex_];

		P = PArray + 3 * iVertexArray.Element[j];

		d_ = RVLDOTPRODUCT3(N, P);

		if (bdmax)
		{
			if (o * d_ > o * d)
			{
				d = d_;

				iVertex = iVertex_;
			}
		}
		else
		{
			d = d_;

			iVertex = iVertex_;

			bdmax = true;
		}

		if (bNormalTest)
		{
			if (d_ < 0.0f || !bVisibilityTest)
			{
				distFromNHull = DistanceFromNormalHull(pVertex->normalHull, N);

				if (distFromNHull <= maxDistFromNHull)
				{
					if (bdmaxValid)
					{
						if (o * d_ > o * dSmaxValid)
							dSmaxValid = d_;
					}
					else
					{
						dSmaxValid = d_;

						bdmaxValid = true;
					}
				}
			}
		}
	} // for each vertex

	if (bdmaxValid)
		bd = (o * (d - dSmaxValid) <= maxedmax);
	else
		bd = false;

	return bdmax;
}

bool SurfelGraph::ComputeTangent(
	Array<int> iVertexArray,
	float *PArray,
	Array<SURFEL::NormalHullElement> *NHullArray,
	float *N,
	float o,
	bool bNormalTest,
	bool bVisibilityTest,
	float maxDistFromNHull,
	float maxedmax,
	float &d,
	bool &bd,
	int &iVertex)
{
	bool bdmax = false;

	bool bdmaxValid = false;

	int j;
	int iVertex_;
	float *P;
	float d_;
	float distFromNHull;
	float dSmaxValid;

	for (j = 0; j < iVertexArray.n; j++)
	{
		iVertex_ = iVertexArray.Element[j];

		// fprintf(fpDebug4, "iVertex_=%d\n", iVertex_);

		P = PArray + 3 * iVertex_;

		d_ = RVLDOTPRODUCT3(N, P);

		if (bdmax)
		{
			if (o * d_ > o * d)
			{
				d = d_;

				iVertex = iVertex_;
			}
		}
		else
		{
			d = d_;

			iVertex = iVertex_;

			bdmax = true;
		}

		if (bNormalTest)
		{
			if (d_ < 0.0f || !bVisibilityTest)
			{
				distFromNHull = DistanceFromNormalHull(NHullArray[iVertex_], N);

				if (distFromNHull <= maxDistFromNHull)
				{
					if (bdmaxValid)
					{
						if (o * d_ > o * dSmaxValid)
							dSmaxValid = d_;
					}
					else
					{
						dSmaxValid = d_;

						bdmaxValid = true;
					}
				}
			}
		}
	} // for each vertex

	if (bdmaxValid)
		bd = (o * (d - dSmaxValid) <= maxedmax);
	else
		bd = false;

	return bdmax;
}

float SurfelGraph::Distance(
	Surfel *pSurfel,
	float *P,
	bool bUncertainty)
{
	if (bUncertainty)
	{
		// PF <- transformation of P into the surfel RF

		float P_[3];

		float *P0 = pSurfel->P;

		RVLDIF3VECTORS(P, P0, P_);

		float *R = pSurfel->R;

		float PF[3];

		RVLMULMX3X3VECT(R, P_, PF);

		// r <- Mahalanobis distance of PF w.r.t. the ellipsoid approximation of surfel

		float r = PF[0] / pSurfel->r1;
		r *= r;
		float fTmp = PF[1] / pSurfel->r2;
		fTmp *= fTmp;
		r += fTmp;

		// Distance computation.

		if (r < 1.0f)
			return PF[2];
		else
			return PF[2] / sqrt(r);
	}
	else
	{
		float *N = pSurfel->N;

		return (RVLDOTPRODUCT3(N, P) - pSurfel->d);
	}
}

int SurfelGraph::GetNoVertices(QList<QLIST::Index> surfelList)
{
	int nVertices = 0;

	int iSurfel;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *pVertexIdx;

	QLIST::Index *piSurfel = surfelList.pFirst;

	while (piSurfel)
	{
		iSurfel = piSurfel->Idx;

		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		pVertexIdx = pSurfelVertexList->pFirst;

		while (pVertexIdx)
		{
			if (!bVertexAssigned[pVertexIdx->Idx])
			{
				nVertices++;

				bVertexAssigned[pVertexIdx->Idx] = true;
			}

			pVertexIdx = pVertexIdx->pNext;
		}

		piSurfel = piSurfel->pNext;
	}

	piSurfel = surfelList.pFirst;

	while (piSurfel)
	{
		iSurfel = piSurfel->Idx;

		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		pVertexIdx = pSurfelVertexList->pFirst;

		while (pVertexIdx)
		{
			bVertexAssigned[pVertexIdx->Idx] = false;

			pVertexIdx = pVertexIdx->pNext;
		}

		piSurfel = piSurfel->pNext;
	}

	return nVertices;
}

void SurfelGraph::GetVertices(
	QList<QLIST::Index> surfelList,
	Array<int> *piVertexArray,
	int *&piVertexIdxMem)
{
	piVertexArray->Element = piVertexIdxMem;

	int iSurfel;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *pVertexIdx;

	QLIST::Index *piSurfel = surfelList.pFirst;

	while (piSurfel)
	{
		iSurfel = piSurfel->Idx;

		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		pVertexIdx = pSurfelVertexList->pFirst;

		while (pVertexIdx)
		{
			if (!bVertexAssigned[pVertexIdx->Idx])
			{
				*(piVertexIdxMem++) = pVertexIdx->Idx;

				bVertexAssigned[pVertexIdx->Idx] = true;
			}

			pVertexIdx = pVertexIdx->pNext;
		}

		piSurfel = piSurfel->pNext;
	}

	piVertexArray->n = piVertexIdxMem - piVertexArray->Element;

	int i;

	for (i = 0; i < piVertexArray->n; i++)
		bVertexAssigned[piVertexArray->Element[i]] = false;
}

bool SurfelGraph::BoundingBox(
	Array<int> iVertexArray,
	float *R,
	float *t,
	float scale,
	Box<float> &boundingBox)
{
	if (iVertexArray.n == 0)
		return false;

	SURFEL::Vertex *pVertex = vertexArray.Element[iVertexArray.Element[0]];

	float P[3], P_[3];

	RVLSCALE3VECTOR(pVertex->P, scale, P_);

	RVLTRANSF3(P_, R, t, P);

	InitBoundingBox<float>(&boundingBox, P);

	int i;

	for (i = 1; i < iVertexArray.n; i++)
	{
		pVertex = vertexArray.Element[iVertexArray.Element[i]];

		RVLSCALE3VECTOR(pVertex->P, scale, P_);

		RVLTRANSF3(P_, R, t, P);

		UpdateBoundingBox<float>(&boundingBox, P);
	}

	return true;
}

void SurfelGraph::Centroid(
	Array<int> iSurfelArray,
	float *centroid)
{
	RVLNULL3VECTOR(centroid);

	float wTotal = 0.0f;

	int i;
	Surfel *pSurfel;
	float w;
	float V3Tmp[3];

	for (i = 0; i < iSurfelArray.n; i++)
	{
		pSurfel = NodeArray.Element + iSurfelArray.Element[i];

		w = (float)(pSurfel->size);

		RVLSCALE3VECTOR(pSurfel->P, w, V3Tmp);

		RVLSUM3VECTORS(centroid, V3Tmp, centroid);

		wTotal += w;
	}

	if (wTotal > 0.0f)
		RVLSCALE3VECTOR2(centroid, wTotal, centroid);
}

void SurfelGraph::VertexCentroid(
	Array<int> iVertexArray,
	float *centroid)
{
	RVLNULL3VECTOR(centroid);

	int i;
	float *P;

	for (i = 0; i < iVertexArray.n; i++)
	{
		P = vertexArray.Element[iVertexArray.Element[i]]->P;

		RVLSUM3VECTORS(centroid, P, centroid);
	}

	float s = (float)(iVertexArray.n);

	RVLSCALE3VECTOR2(centroid, s, centroid);
}

void SurfelGraph::CenterVertices(
	Array<int> iVertexArray,
	float *P,
	float *centroid)
{
	VertexCentroid(iVertexArray, centroid);

	float *P_, *P__;
	int i;
	int iVertex;

	for (i = 0; i < iVertexArray.n; i++)
	{
		iVertex = iVertexArray.Element[i];

		P_ = P + 3 * iVertex;

		P__ = vertexArray.Element[iVertex]->P;

		RVLDIF3VECTORS(P__, centroid, P_);
	}
}

void SurfelGraph::NodeColors(unsigned char *SelectionColor)
{
	RVL_DELETE_ARRAY(nodeColor);

	RandomColors(SelectionColor, nodeColor, NodeArray.n);
}

void SurfelGraph::DisplayHardEdges(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iSurfel,
	unsigned char *Color)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;

	int iPt, iPt_;
	MeshEdgePtr *pEdgePtr;
	MeshEdge *pEdge;
	int iSurfel_;
	// Surfel *pSurfel_;
	bool bEdge;
	Point *pPt;

	while (pPtIdx)
	{
		iPt = pPtIdx->Idx;

		pPt = pMesh->NodeArray.Element + iPt;

		bEdge = false;

		pEdgePtr = pPt->EdgeList.pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

			iSurfel_ = surfelMap[iPt_];

			if (iSurfel_ != iSurfel)
			{
				bEdge = true;

				break;
			}

			pEdgePtr = pEdgePtr->pNext;
		}

		if (bEdge)
			pVisualizer->PaintPoint(iPt, pMesh->pPolygonData, Color);
		// else
		//	int debug = 0;

		pPtIdx = pPtIdx->pNext;
	}
}

void SurfelGraph::Display(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iSelectedSurfel,
	unsigned char *SelectionColor,
	int *ColorScale,
	unsigned char *ColorOffset,
	bool bDisplayVertices)
{
	Figure *pFig;

	if (pVisualizer->b2D)
	{
		pFig = pVisualizer->OpenFigure("Segmentation");

		memset(pFig->pImage->imageData, 0, pFig->pImage->imageSize * sizeof(char));
	}
	else
		pFig = NULL;

	unsigned char MarkColor[3];

	MarkColor[0] = 0;
	MarkColor[1] = 255;
	MarkColor[2] = 0;

	int iSurfel;
	Surfel *pSurfel;
	unsigned char color[3];
	unsigned char *color_;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		color_ = nodeColor + 3 * iSurfel;

		if (iSurfel == iSelectedSurfel)
			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, SelectionColor, pFig, pMesh->mapNodesToPolyData);
		else
		{
			RVLCOPY3VECTOR(color_, color);

			if (DisplayData.mode == RVLSURFEL_DISPLAY_MODE_FOREGROUND_BACKGROUND || DisplayData.mode == RVLSURFEL_DISPLAY_MODE_CONVEX_CONCAVE)
			{
				int colorScale_[] = {25, 25, 25};

				RVLSCALECOLOR2(color, colorScale_, color);
			}
			else if (ColorScale)
				RVLSCALECOLOR2(color, ColorScale, color);

			if (ColorOffset)
			{
				RVLSUM3VECTORS(color, ColorOffset, color);
			}

			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color, pFig, pMesh->mapNodesToPolyData);
		}

		// DisplayHardEdges(pVisualizer, pMesh, iSurfel, MarkColor);
	}

	if (DisplayData.mode == RVLSURFEL_DISPLAY_MODE_FOREGROUND_BACKGROUND)
		DisplayForegroundAndBackgroundEdges(pVisualizer, pMesh);
#ifdef RVLSURFEL_IMAGE_ADJACENCY
	else if (DisplayData.mode == RVLSURFEL_DISPLAY_MODE_CONVEX_CONCAVE)
		DisplayConvexAndConcaveEdges(pVisualizer, pMesh);
#endif

	if (DisplayData.bEdges)
		DisplayEdgeFeatures();

	DisplayData.bEdges = false;

	if (bDisplayVertices)
		DisplayVertices();
}

// VTK Render window right mouse button press callback
void SURFEL::MouseRButtonDown(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor *>(caller);
	SURFEL::DisplayCallbackData *pData = (SURFEL::DisplayCallbackData *)clientdata;

	Mesh *pMesh = pData->pMesh;
	SurfelGraph *pSurfels = pData->pSurfels;

	vtkSmartPointer<vtkPolyData> pd = pMesh->pPolygonData;

	vtkSmartPointer<vtkFloatArray> pointData;
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;
	vtkSmartPointer<vtkFloatArray> normalPointData;
	int noPts = 0;
	// FetchVTKPointData(pd, pointData, rgbPointData, normalPointData, noPts);

	pData->pVisualizer->pointPicker->Pick(interactor->GetEventPosition()[0], interactor->GetEventPosition()[1], 0,
										  interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	vtkIdType selectedPoint = pData->pVisualizer->pointPicker->GetPointId();

	if (selectedPoint >= 0)
	{
		if (pData->bPolygons)
		{
			printf("Selected: vertex %d", selectedPoint);
			for (int i = 0; i < pData->selectedActors.size(); i++)
				pData->pVisualizer->renderer->RemoveViewProp(pData->selectedActors[i]);
			Array<Point> selectedVertex;
			selectedVertex.n = 1;
			Point selectedVertexMem[2];
			selectedVertex.Element = selectedVertexMem;
			float *PSrc = pSurfels->polygonVerticesS.Element[selectedPoint].Element;
			float *PTgt = selectedVertex.Element[0].P;
			RVLCOPY3VECTOR(PSrc, PTgt);
			pData->selectedActors.push_back(pData->pVisualizer->DisplayPointSet<float, Point>(selectedVertex, pData->SelectionColor, 6.0f));
			int iPolyEdge;
			SURFEL::PolyEdge *pPolyEdge;
			for (iPolyEdge = 0; iPolyEdge < pSurfels->polyEdges.n; iPolyEdge++)
			{
				pPolyEdge = pSurfels->polyEdges.Element + iPolyEdge;
				if (pPolyEdge->iVertex[0] == selectedPoint)
					break;
			}
			if (iPolyEdge < pSurfels->polyEdges.n)
			{
				printf(", edge %d, polygon %d, surfel %d\n", iPolyEdge, pPolyEdge->iPolygon, pPolyEdge->iSurfel);
				PSrc = pSurfels->polygonVerticesS.Element[pPolyEdge->iVertex[1]].Element;
				PTgt = selectedVertex.Element[1].P;
				RVLCOPY3VECTOR(PSrc, PTgt);
				pData->selectedActors.push_back(pData->pVisualizer->DisplayLine(selectedVertexMem, pData->SelectionColor, 2.0f));
			}
			else
				printf("\n");
			interactor->GetRenderWindow()->Render();
		}
		else
		{
			int iSurfel = pSurfels->surfelMap[selectedPoint];

		bool bSelection = false;

		if (pData->mouseRButtonDownUserFunction)
		{
			if (iSurfel >= 0)
			{
				printf("Selected surfel: %d\n", iSurfel);
					bSelection |= pData->mouseRButtonDownUserFunction(pMesh, pSurfels, (int)selectedPoint, iSurfel, pData->vpUserFunctionData);
			}
		}

		if (!bSelection)
		{
			if (pData->iSelectedSurfel >= 0 && (pData->iSelection == 1 && pData->iSelectedSurfel != iSurfel))
					pData->pVisualizer->PaintPointSet(&(pSurfels->NodeArray.Element[pData->iSelectedSurfel].PtList), pMesh->pPolygonData,
													  pSurfels->GetColor(pData->iSelectedSurfel));

			if (pData->iSelectedSurfel2 >= 0 && ((pData->iSelection == 1 || (pData->iSelection == 2 && pData->iSelectedSurfel2 != iSurfel))))
					pData->pVisualizer->PaintPointSet(&(pSurfels->NodeArray.Element[pData->iSelectedSurfel2].PtList), pMesh->pPolygonData,
													  pSurfels->GetColor(pData->iSelectedSurfel2));

				// pSurfels->DisplaySurfelBoundary(pData->pVisualizer, pMesh, iSurfel, pData->SelectionColor);

			if (pData->iSelection == 1)
			{
				if (iSurfel >= 0)
						pData->pVisualizer->PaintPointSet(&(pSurfels->NodeArray.Element[iSurfel].PtList), pMesh->pPolygonData, pData->SelectionColor);

				pData->iSelectedSurfel = iSurfel;

				pData->iSelectedSurfel2 = -1;
			}
			else // if (pData->iSelection == 2)
			{
				unsigned char SelectionColor2[3];

				RVLSCALECOLOR(pData->SelectionColor, 75, SelectionColor2);

					pData->pVisualizer->PaintPointSet(&(pSurfels->NodeArray.Element[iSurfel].PtList), pMesh->pPolygonData, SelectionColor2);

				pData->iSelectedSurfel2 = iSurfel;

				pData->iSelection = 1;
			}

			bSelection = true;
		}

		if (bSelection)
		{
			pd->Modified();

			if (pData->bEdges)
				pData->edgeFeaturesPolyData->Modified();

				pSurfels->PrintData(pData->pVisualizer, pMesh, selectedPoint, iSurfel);

			interactor->GetRenderWindow()->Render();
		}
	}
}
}

// VTK Render window key press callback
void SURFEL::KeyPressCallback(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor *>(caller);
	SURFEL::DisplayCallbackData *pData = (SURFEL::DisplayCallbackData *)clientdata;
	SurfelGraph *pSurfels = pData->pSurfels;

	if (!pData->bFirstKey)
	{
		pData->bFirstKey = true;

		return;
	}

	pData->bFirstKey = false;

	Mesh *pMesh = pData->pMesh;

	vtkSmartPointer<vtkPolyData> pd = pMesh->pPolygonData;

	vtkSmartPointer<vtkFloatArray> pointData;
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;
	vtkSmartPointer<vtkFloatArray> normalPointData;
	int noPts = 0;

	std::string keySym = "";
	keySym = interactor->GetKeySym();

	PlanarSurfelDetector *pDetector = (PlanarSurfelDetector *)(pData->vpDetector);

	bool bUpdateDisplay = false;
	bool bDisplayBoundary = false;
	bool bDefineBoundary = false;
	bool bDisplaySoftEdges = false;

	if (!DefinePlaneInteractive(pData->pSurfels, keySym, pData->iSelectedSurfel))
	{
		if (keySym == "2")
		{
			pData->iSelection = 2;
		}
		else if (keySym == "b")
		{
			if (pData->mode != RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR)
			{
				int colorScale[3];

				RVLSET3VECTOR(colorScale, 0, 0, 75);

				unsigned char colorOffset[3];

				RVLSET3VECTOR(colorOffset, 0, 0, 64);

				pData->pSurfels->Display(pData->pVisualizer, pMesh, -1, NULL, colorScale, colorOffset);

				bDefineBoundary = true;

				bUpdateDisplay = true;
			}
		}
		else if (keySym == "c")
		{
			bDisplayBoundary = true;

			bUpdateDisplay = true;
		}
		else if (keySym == "g")
		{
			bDisplaySoftEdges = !bDisplaySoftEdges;

			if (bDisplaySoftEdges)
				pDetector->DisplaySoftEdges(pData->pVisualizer, pMesh, pData->pSurfels, pData->SelectionColor);
			else
				pData->pSurfels->Display(pData->pVisualizer, pMesh);

			bUpdateDisplay = true;
		}
		else if (keySym == "n")
		{
			if (pData->pVisualizer->bNormals)
			{
				pData->pVisualizer->bNormalsVisible = !pData->pVisualizer->bNormalsVisible;

				if (pData->pVisualizer->bNormalsVisible)
					pData->pVisualizer->normals->VisibilityOn();
				else
					pData->pVisualizer->normals->VisibilityOff();

				bUpdateDisplay = true;
			}
		}
		else if (keySym == "p")
		{
			if (pData->iSelectedSurfel >= 0)
			{
				pDetector->DefinePolygon(pMesh, pData->pSurfels, pData->iSelectedSurfel);

				pData->pSurfels->Display(pData->pVisualizer, pMesh, pData->iSelectedSurfel, pData->SelectionColor);

				bUpdateDisplay = true;
			}
		}
		else if (keySym == "s")
		{
			if (pData->mode == RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR)
			{
				pData->pSurfels->Display(pData->pVisualizer, pMesh, pData->iSelectedSurfel, pData->SelectionColor);

				bUpdateDisplay = true;
			}

			pData->mode = RVLSURFEL_DISPLAY_MODE_SURFELS;
		}
		else if (keySym == "v")
		{
			pData->bVertices = !pData->bVertices;

			if (pData->bVertices)
				pData->vertices->VisibilityOn();
			else
				pData->vertices->VisibilityOff();

			bUpdateDisplay = true;
		}
		else if (keySym == "F1")
		{
			std::cout << "Enter surfel index: ";

			std::string line;

			std::getline(std::cin, line);

			int iSelectedSurfel;

			sscanf(line.data(), "%d", &iSelectedSurfel);

			if (iSelectedSurfel >= 0 && iSelectedSurfel < pData->pSurfels->NodeArray.n)
			{
				if (pData->bPolygons)
				{
					for (int i = 0; i < pData->selectedActors.size(); i++)
						pData->pVisualizer->renderer->RemoveViewProp(pData->selectedActors[i]);
					Surfel *pSurfel = pSurfels->NodeArray.Element + iSelectedSurfel;
					Pair<int, int> vertexInterval;
					Vector3<float> *pVertex;
					RVLVISUALIZER_LINES_INIT(visPts, visLines, pSurfels->polygonVerticesS.n);
					Point *pVisPt = visPts.Element;
					Pair<int, int> *pVisLine = visLines.Element;
					int iVisPt = 0;
					int iVisPt0;
					for (int iPoly = 0; iPoly < pSurfel->polygonVertexIntervals.n; iPoly++)
					{
						vertexInterval = pSurfel->polygonVertexIntervals.Element[iPoly];
						iVisPt0 = iVisPt;
						for (int iVertex = vertexInterval.a; iVertex <= vertexInterval.b; iVertex++)
						{
							pVertex = pSurfels->polygonVerticesS.Element + iVertex;
							pVisPt = visPts.Element + iVisPt;
							RVLCOPY3VECTOR(pVertex->Element, pVisPt->P);
							pVisLine->a = iVisPt;
							pVisLine->b = (iVertex < vertexInterval.b ? iVisPt + 1 : iVisPt0);
							iVisPt++;
							pVisLine++;
						}
					}
					visPts.n = iVisPt;
					visLines.n = pVisLine - visLines.Element;
					pData->selectedActors.push_back(pData->pVisualizer->DisplayPointSet<float, Point>(visPts, pData->SelectionColor, 6.0f));
					pData->selectedActors.push_back(pData->pVisualizer->DisplayLines(visPts, visLines, pData->SelectionColor, 2.0f));
					RVLVISUALIZER_LINES_FREE(visPts, visLines);
					interactor->GetRenderWindow()->Render();
				}
				else
				{
				if (pData->iSelectedSurfel >= 0 && pData->iSelectedSurfel < pData->pSurfels->NodeArray.n)
					pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[pData->iSelectedSurfel].PtList), pMesh->pPolygonData,
													  pData->pSurfels->GetColor(pData->iSelectedSurfel));

				pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSelectedSurfel].PtList), pMesh->pPolygonData, pData->SelectionColor);

				pData->iSelectedSurfel = iSelectedSurfel;

				bUpdateDisplay = true;
			}
		}
		}
#ifdef RVLMESH_BOUNDARY_DEBUG
		else if (keySym == "plus")
		{
			if (pData->mode == RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR)
			{
				if (pData->iSelectedSurfel >= 0 && pData->iSelectedSurfel2 >= 0)
				{
					pMesh->debugState++;

					bDefineBoundary = true;

					bUpdateDisplay = true;
				}
			}
			else
			{
				if (pData->iSelectedSurfel >= 0)
				{
					pMesh->debugState++;

					bDisplayBoundary = true;

					bUpdateDisplay = true;
				}
			}
		}
#endif
	}

	if (pData->keyPressUserFunction)
		bUpdateDisplay |= pData->keyPressUserFunction(pMesh, pData->pSurfels, keySym, pData->vpUserFunctionData);

	if (bDefineBoundary)
	{
		QList<QLIST::Index> G;

		int iSurfel = pData->iSelectedSurfel;
		int iSurfel_ = pData->iSelectedSurfel2;

		pDetector->DefineBoundaryTest(pMesh, pData->pSurfels, iSurfel, iSurfel_, G);

		unsigned char white[3];

		RVLSET3VECTOR(white, 255, 255, 255);

		unsigned char green[3];

		RVLSET3VECTOR(green, 0, 255, 0);

		unsigned char black[3];

		RVLSET3VECTOR(black, 0, 0, 0);

		pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel].PtList), pMesh->pPolygonData, white);

		// pData->pVisualizer->PaintPointSet(&G, pMesh->pPolygonData, green);

		pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel_].PtList), pMesh->pPolygonData, black);

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
		unsigned char red[3];

		RVLSET3VECTOR(red, 255, 0, 0);

		pData->pVisualizer->PaintPointSet(&(pDetector->debugPtArray), pMesh->pPolygonData, red);
#endif

		// #ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
		//		pData->pVisualizer->PaintPointSet(&(pDetector->debugPtArray), pMesh->pPolygonData, green);
		// #endif

		pData->mode = RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR;
	}

	if (bDisplayBoundary)
	{
		if (pData->iSelectedSurfel >= 0)
		{
			pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[pData->iSelectedSurfel].PtList), pMesh->pPolygonData,
											  pData->pSurfels->GetColor(pData->iSelectedSurfel));

			FILE *fpPts = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugPoints.txt", "w");
			FILE *fpEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "w");

			pData->pSurfels->Save(pData->iSelectedSurfel, pMesh, fpPts, fpEdges);

			fclose(fpPts);
			fclose(fpEdges);

			pData->pSurfels->DisplaySurfelBoundary(pData->pVisualizer, pMesh, pData->iSelectedSurfel, pData->SelectionColor);
		}
	}

	if (bUpdateDisplay)
	{
		pd->Modified();

		if (pData->bEdges)
			pData->edgeFeaturesPolyData->Modified();

		pData->pSurfels->PrintData(pData->pVisualizer, pMesh, -1, pData->iSelectedSurfel);

		interactor->GetRenderWindow()->Render();
	}
}

void SurfelGraph::PrintData(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iVertex,
	int iSurfel)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	char str[2000], str2[500];

	if (iVertex >= 0)
	{
		Point *pPt = pMesh->NodeArray.Element + iVertex;

		sprintf(str, "Point %d\nP=(%f, %f, %f)\nN=(%f, %f, %f)\nRGB=(%d, %d, %d)",
				iVertex, pPt->P[0], pPt->P[1], pPt->P[2], pPt->N[0], pPt->N[1], pPt->N[2], pPt->RGB[0], pPt->RGB[1], pPt->RGB[2]);

		// if (RVLDOTPRODUCT3(pPt->P, pPt->P) > 1e-10)
		//{
		//	FILE *fp = fopen("selectedPts.txt", "a");

		//	fprintf(fp, "%f\t%f\t%f\n", pPt->P[0], pPt->P[1], pPt->P[2]);

		//	fclose(fp);
		//}
	}
	else
		str[0] = 0;

	if (iSurfel >= 0)
	{
		sprintf(str2, "\nSurfel %d\nP=(%f, %f, %f)\nN=(%f, %f, %f)\nRGB=(%d, %d, %d)\nsize=%d",
				iSurfel,
				pSurfel->P[0], pSurfel->P[1], pSurfel->P[2],
				pSurfel->N[0], pSurfel->N[1], pSurfel->N[2],
				pSurfel->RGB[0], pSurfel->RGB[1], pSurfel->RGB[2],
				pSurfel->size);

		strcat(str, str2);
	}

	//// Print indices of the adjacent surfels

	// strcat(str, "\nNeighbors:\n");

	// VertexEdgePtr *pEdgePtr = pSurfel->EdgeList.pFirst;

	// Surfel *pSurfel_ = pSurfel;

	// Surfel *pSurfel__;
	// MeshEdge *pEdge;
	// int iSurfel__;
	// float eZ_, eZ__, eXY;
	// float N_[3], N__[3], Z[3];
	// float V3Tmp[3];
	// int RGB_[3], RGB__[3], dRGB[3], eRGB;
	// float fTmp;

	// RVLCONVTOINT3(pSurfel_->RGB, RGB_);

	// while (pEdgePtr)	// for each neighbor of iNode
	//{
	//	RVLSEGMENTATION_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iSurfel__);

	//	pSurfel__ = surfelArray.Element + iSurfel__;

	//	RVLCONVTOINT3(pSurfel__->RGB, RGB__);

	//	RVLDIF3VECTORS(RGB__, RGB_, dRGB);

	//	eRGB = RVLDOTPRODUCT3(dRGB, dRGB);

	//	RVLDIF3VECTORS(pSurfel__->P, pSurfel_->P, Z);

	//	RVLNORM3(Z, fTmp);

	//	eZ_ = RVLDOTPRODUCT3(Z, pSurfel_->N);

	//	eZ__ = RVLDOTPRODUCT3(Z, pSurfel__->N);

	//	RVLSCALE3VECTOR(Z, eZ_, V3Tmp);
	//	RVLDIF3VECTORS(pSurfel_->N, V3Tmp, N_);
	//	RVLNORM3(N_, fTmp);
	//	RVLSCALE3VECTOR(Z, eZ__, V3Tmp);
	//	RVLDIF3VECTORS(pSurfel__->N, V3Tmp, N__);
	//	RVLNORM3(N__, fTmp);
	//	RVLDIF3VECTORS(N__, N_, V3Tmp);

	//	eXY = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

	//	sprintf(str2, "%d: e=(%f, %f, %f, %f)\n", iSurfel__, eZ_, eZ__, sqrt(eXY), sqrt((float)eRGB));

	//	strcat(str, str2);

	//	pEdgePtr = pEdgePtr->pNext;
	//}	// for each neighbor of iNode

	// Put the text on the screen

	pVisualizer->text->SetText(2, str);
}

unsigned char *SurfelGraph::GetColor(int iSurfel)
{
	return nodeColor + 3 * iSurfel;
}

void SurfelGraph::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	void *vpDetector,
	bool bCallbackFunctions)
{
	DisplayData.pMesh = pMesh;
	DisplayData.pSurfels = this;
	DisplayData.pVisualizer = pVisualizer;
	DisplayData.vpDetector = vpDetector;
	RVLSET3VECTOR(DisplayData.SelectionColor, 0, 255, 0);
	DisplayData.mode = RVLSURFEL_DISPLAY_MODE_SURFELS;
	// DisplayData.mode = RVLSURFEL_DISPLAY_MODE_FOREGROUND_BACKGROUND;
	// DisplayData.mode = RVLSURFEL_DISPLAY_MODE_CONVEX_CONCAVE;
	DisplayData.iSelectedSurfel = DisplayData.iSelectedSurfel2 = -1;
	DisplayData.iSelection = 1;
	DisplayData.bVertices = false;
	DisplayData.bFirstKey = true;
	DisplayData.selectedActors.clear();

	if (pVisualizer->b3D)
	{
		if (!DisplayData.bPolygons)
		pVisualizer->SetMesh(pMesh);
		if (bCallbackFunctions)
		{
			if (!DisplayData.bCallbackFunctionsDefined)
			{
				pVisualizer->SetMouseRButtonDownCallback(SURFEL::MouseRButtonDown, &DisplayData);
				pVisualizer->SetKeyPressCallback(SURFEL::KeyPressCallback, &DisplayData);

				DisplayData.bCallbackFunctionsDefined = true;
			}
		}
	}

	if (!pMesh->bOrganizedPC)
		pVisualizer->b2D = false;

	if (pVisualizer->b2D)
	{
		Figure *pFig = pVisualizer->OpenFigure("Segmentation");

		pFig->pImage = cvCreateImage(cvSize(pMesh->width, pMesh->height), IPL_DEPTH_8U, 3);
	}
}

void SurfelGraph::DisplaySurfelBoundary(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iSurfel,
	unsigned char *Color)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	// QList<QLIST::Index2> *pSurfelPtList = &(pSurfel->PtList);

	// pSurfel->BoundaryArray.Element = new Array <MeshEdgePtr *>[nMeshVertices];

	// MeshEdgePtr **boundaryMem = new MeshEdgePtr *[pMesh->EdgeArray.n];

	// MeshEdgePtr **pBoundaryMem = boundaryMem;

	// pMesh->Boundary(pSurfelPtList, surfelMap, pSurfel->BoundaryArray, pBoundaryMem, edgeMarkMap);

	Array<int> boundaryPtArray;

	boundaryPtArray.Element = new int[nMeshVertices];
	boundaryPtArray.n = 0;

	int iBoundary, iPointEdge;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr;

	for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
	{
		pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

		for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
		{
			pEdgePtr = pBoundary->Element[iPointEdge];

			boundaryPtArray.Element[boundaryPtArray.n++] = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);
		}
	}

	pVisualizer->PaintPointSet(&boundaryPtArray, pMesh->pPolygonData, Color);

	// delete[] pSurfel->BoundaryArray.Element;
	// delete[] boundaryMem;
	delete[] boundaryPtArray.Element;

	// QList<QLIST::Index> Boundary;

	// QLIST::Index *BoundaryMem = new QLIST::Index[pMesh->NodeArray.n];

	// pMesh->Boundary(pSurfelPtList, surfelMap, &Boundary, BoundaryMem);

	// pVisualizer->PaintPointSet(&Boundary, pMesh->pPolygonData, Color);

	// delete[] BoundaryMem;
}

void SurfelGraph::DisplayPolygons(
	Visualizer *pVisualizer,
	uchar *color,
	bool bDisplayEdgeGraph,
	bool bMultiColor)
{
	// Colors.

	uchar darkGreen[] = {0, 128, 0};
	uchar *vertexColor, *lineColor;
	if (bMultiColor)
	{
		vertexColor = new uchar[3 * polyEdges.n];
		lineColor = new uchar[3 * polyEdges.n];
	}
	else
		vertexColor = lineColor = color;

	// Display poligons.

	Array<Point> vertices;
	vertices.n = 0;
	// vertices.Element = new Point[polygonVertices.size()];
	vertices.Element = new Point[polyEdges.n];
	memset(vertices.Element, 0, polyEdges.n * sizeof(Point));
	Point *pPt;
	Array<Pair<int, int>> lines;
	lines.n = 0;
	// lines.Element = new Pair<int, int>[polygonVertices.size()];
	lines.Element = new Pair<int, int>[polyEdges.n];
	Pair<int, int> *pLine;
	int iSurfel;
	Surfel *pSurfel;
	int iPolygon, iVertex;
	Pair<int, int> *pPolygonVertexInterval;
	int iPolyEdge;
	SURFEL::PolyEdge *pPolyEdge;
	float *PS;
	uchar *surfelColor;
	uchar *color_;
	for (iPolyEdge = 0; iPolyEdge < polyEdges.n; iPolyEdge++)
	{
		pPolyEdge = polyEdges.Element + iPolyEdge;
		iVertex = pPolyEdge->iVertex[0];
		pPt = vertices.Element + iVertex;
		if (iVertex > vertices.n)
			vertices.n = iVertex;
		PS = polygonVerticesS.Element[iVertex].Element;
		RVLCOPY3VECTOR(PS, pPt->P);
		pLine = lines.Element + lines.n;
		pLine->a = iVertex;
		pLine->b = pPolyEdge->iVertex[1];
		if (bMultiColor)
		{
			surfelColor = color + 3 * pPolyEdge->iSurfel;
			color_ = vertexColor + 3 * iVertex;
			RVLCOPY3VECTOR(surfelColor, color_);
			color_ = lineColor + 3 * lines.n;
			RVLCOPY3VECTOR(surfelColor, color_);
		}
		lines.n++;
	}
	vertices.n++;

	// for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	//{
	//	pSurfel = NodeArray.Element + iSurfel;
	//	for (iPolygon = 0; iPolygon < pSurfel->polygonVertexIntervals.n; iPolygon++)
	//	{
	//		pPolygonVertexInterval = pSurfel->polygonVertexIntervals.Element + iPolygon;
	//		for (iVertex = pPolygonVertexInterval->a; iVertex <= pPolygonVertexInterval->b; iVertex++)
	//		{
	//			P2D = polygonVertices[iVertex].P;
	//			RVLCOPY2VECTOR(P2D, PF);

	//			pPt = vertices.Element + (vertices.n++);
	//			RVLTRANSF3(PF, pSurfel->R, pSurfel->P, pPt->P);
	//			pLine = lines.Element + (lines.n++);
	//			pLine->a = iVertex;
	//			pLine->b = (iVertex < pPolygonVertexInterval->b ? iVertex + 1 : pPolygonVertexInterval->a);
	//		}
	//	}
	//	//break;
	//}
	vtkSmartPointer<vtkActor> verticesActor = pVisualizer->DisplayPointSet<float, Point>(vertices, vertexColor, 3.0f, bMultiColor);
	pVisualizer->DisplayLines(vertices, lines, lineColor, 1.0f, bMultiColor);
	delete[] vertices.Element;
	delete[] lines.Element;
	if (bMultiColor)
	{
		delete[] vertexColor;
		delete[] lineColor;
	}

	// Display surfel graph.

	SURFEL::PolyEdgeGraphEdge *pPolygonEdgeGraphEdge;
	// if (surfelGraph2.EdgeArray.n > 0)
	//{
	//	vertices.n = 0;
	//	vertices.Element = new Point[2 * surfelGraph2.EdgeArray.n];
	//	lines.n = 0;
	//	lines.Element = new Pair<int, int>[surfelGraph2.EdgeArray.n];
	//	int iSurfelGraphEdge;
	//	GRAPH::Edge* pSurfelGraphEdge;
	//	for (iSurfelGraphEdge = 0; iSurfelGraphEdge < surfelGraph2.EdgeArray.n; iSurfelGraphEdge++)
	//	{
	//		pSurfelGraphEdge = surfelGraph2.EdgeArray.Element + iSurfelGraphEdge;
	//		pPolygonEdgeGraphEdge = polygonEdgeGraphEdges.data() + pSurfelGraphEdge->idx;
	//		pLine = lines.Element + (lines.n++);
	//		pLine->a = vertices.n;
	//		pPt = vertices.Element + (vertices.n++);
	//		RVLCOPY3VECTOR(pPolygonEdgeGraphEdge->P[0], pPt->P);
	//		pLine->b = vertices.n;
	//		pPt = vertices.Element + (vertices.n++);
	//		RVLCOPY3VECTOR(pPolygonEdgeGraphEdge->P[1], pPt->P);
	//	}
	//	pVisualizer->DisplayLines(vertices, lines, darkGreen);

	//	delete[] vertices.Element;
	//	delete[] lines.Element;
	//}

	// Display polygon edge graph.

	if (bDisplayEdgeGraph && polygonEdgeGraphEdges.size() > 0)
	{
		vertices.n = 0;
		vertices.Element = new Point[2 * polygonEdgeGraphEdges.size()];
		lines.n = 0;
		lines.Element = new Pair<int, int>[polygonEdgeGraphEdges.size()];
		int iPolygonEdgeGraphEdge;
		for (iPolygonEdgeGraphEdge = 0; iPolygonEdgeGraphEdge < polygonEdgeGraphEdges.size(); iPolygonEdgeGraphEdge++)
		{
			pPolygonEdgeGraphEdge = polygonEdgeGraphEdges.data() + iPolygonEdgeGraphEdge;
			pLine = lines.Element + (lines.n++);
			pLine->a = vertices.n;
			pPt = vertices.Element + (vertices.n++);
			RVLCOPY3VECTOR(pPolygonEdgeGraphEdge->P[0], pPt->P);
			pLine->b = vertices.n;
			pPt = vertices.Element + (vertices.n++);
			RVLCOPY3VECTOR(pPolygonEdgeGraphEdge->P[1], pPt->P);
		}
		pVisualizer->DisplayLines(vertices, lines, darkGreen);

		delete[] vertices.Element;
		delete[] lines.Element;
	}

	// Restrict the point picker to vertices.

	pVisualizer->pointPicker->InitializePickList();
	pVisualizer->pointPicker->AddPickList(verticesActor);
	pVisualizer->pointPicker->PickFromListOn();
}

void SurfelGraph::DisplayEdgeFeatures()
{
	Visualizer *pVisualizer = DisplayData.pVisualizer;

	vtkSmartPointer<vtkPoints> pts;
	vtkSmartPointer<vtkCellArray> polyLines;

	if (pVisualizer->b3D)
	{
		// Create the polydata where we will store all the geometric data
		DisplayData.edgeFeaturesPolyData = vtkSmartPointer<vtkPolyData>::New();

		// Create a vtkPoints container and store the points in it
		pts = vtkSmartPointer<vtkPoints>::New();

		//// Create a cell array to store the lines in and add the lines to it
		polyLines = vtkSmartPointer<vtkCellArray>::New();

#ifdef NEVER
		// Create colors.
		vtkSmartPointer<vtkUnsignedCharArray> colors =
			vtkSmartPointer<vtkUnsignedCharArray>::New();

		colors->SetNumberOfComponents(3);

		unsigned char red[3] = {255, 0, 0};

		colors->InsertNextTupleValue(red);
#endif
		///

		DisplayData.bEdges = true;
	}

	// Determine the total number of edge features.

	int nEdgeFeatures = 0;

	int iFeature;

	for (iFeature = 0; iFeature < NodeArray.n; iFeature++)
		if (NodeArray.Element[iFeature].bEdge)
			nEdgeFeatures++;

	// Allocate polyline pointers.

	vtkSmartPointer<vtkPolyLine> *polyLine;

	if (pVisualizer->b3D)
		polyLine = new vtkSmartPointer<vtkPolyLine>[nEdgeFeatures];

	//

	int iEdgeFeature = 0;

	RVL_DELETE_ARRAY(DisplayData.edgeFeatureIdxArray);

	DisplayData.edgeFeatureIdxArray = new int[NodeArray.n];

	memset(DisplayData.edgeFeatureIdxArray, 0xff, NodeArray.n * sizeof(int));

	int i;
	Surfel *pFeature;
	float *N, *V, *P1;
	float P2[3], P3[3], P4[3], U[3], VTmp[3];
	double P[3];
	float fTmp;

	for (iFeature = 0; iFeature < NodeArray.n; iFeature++)
	{
		pFeature = NodeArray.Element + iFeature;

		// if (iFeature != 52)
		//	continue;

		if (!pFeature->bEdge)
			continue;

		// N <- edge feature normal

		N = pFeature->N;

		// V <- unit vector in edge direction.

		V = pFeature->V;

		// P1 <- the first endpoint of the edge feature

		P1 = pFeature->P;

		// P2 <- P1 + pFeature->physicalSize * V

		RVLSCALE3VECTOR(V, pFeature->physicalSize, VTmp);

		RVLSUM3VECTORS(P1, VTmp, P2);

		// U <- unit(V x N)

		RVLCROSSPRODUCT3(V, N, U);

		RVLNORM3(U, fTmp);

		// P3 <- P1 + DisplayData.edgeFeatureDepth * P1 / (P1' * U)

		fTmp = DisplayData.edgeFeatureDepth / RVLDOTPRODUCT3(P1, U);

		RVLSCALE3VECTOR(P1, fTmp, P3);

		RVLSUM3VECTORS(P1, P3, P3);

		// P4 <- P4 + DisplayData.edgeFeatureDepth * P4 / (P4' * U)

		fTmp = DisplayData.edgeFeatureDepth / RVLDOTPRODUCT3(P2, U);

		RVLSCALE3VECTOR(P2, fTmp, P4);

		RVLSUM3VECTORS(P2, P4, P4);

		// Add P1, P2, P3 and P4 to pts

		RVLCOPY3VECTOR(P1, P);

		if (pVisualizer->b3D)
			pts->InsertNextPoint(P);

		RVLCOPY3VECTOR(P2, P);

		if (pVisualizer->b3D)
			pts->InsertNextPoint(P);

		RVLCOPY3VECTOR(P4, P);

		if (pVisualizer->b3D)
			pts->InsertNextPoint(P);

		RVLCOPY3VECTOR(P3, P);

		if (pVisualizer->b3D)
			pts->InsertNextPoint(P);

		// Create rectangle P1-P2-P3-P4.

		if (pVisualizer->b3D)
		{
			polyLine[iEdgeFeature] = vtkSmartPointer<vtkPolyLine>::New();

			polyLine[iEdgeFeature]->GetPointIds()->SetNumberOfIds(5);

			for (i = 0; i < 4; i++)
				polyLine[iEdgeFeature]->GetPointIds()->SetId(i, 4 * iEdgeFeature + i);

			polyLine[iEdgeFeature]->GetPointIds()->SetId(4, 4 * iEdgeFeature);

			// Add polyline to polyLines.

			polyLines->InsertNextCell(polyLine[iEdgeFeature]);

			// Assign color to polyline.

			// colors->InsertNextTupleValue(red);

			DisplayData.edgeFeatureIdxArray[iFeature] = iEdgeFeature;
		}

		iEdgeFeature++;
	}

	if (pVisualizer->b3D)
	{
		// Add the points to the polydata container
		DisplayData.edgeFeaturesPolyData->SetPoints(pts);

		// Add the lines to the polydata container
		DisplayData.edgeFeaturesPolyData->SetLines(polyLines);

		// Create two colors - one for each line
		unsigned char color[3] = {255, 255, 255};

		// Create a vtkUnsignedCharArray container and store the colors in it
		vtkSmartPointer<vtkUnsignedCharArray> colors =
			vtkSmartPointer<vtkUnsignedCharArray>::New();
		colors->SetNumberOfComponents(3);

		for (iEdgeFeature = 0; iEdgeFeature < nEdgeFeatures; iEdgeFeature++)
			colors->InsertNextTypedTuple(color);

		// Color the lines.
		DisplayData.edgeFeaturesPolyData->GetCellData()->SetScalars(colors);

		// Setup the visualization pipeline
		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

		mapper->SetInputData(DisplayData.edgeFeaturesPolyData);

		DisplayData.edgeFeatures = vtkSmartPointer<vtkActor>::New();
		DisplayData.edgeFeatures->SetMapper(mapper);

		pVisualizer->renderer->AddActor(DisplayData.edgeFeatures);
	}
}

void SurfelGraph::DisplayForegroundAndBackgroundEdges(
	Visualizer *pVisualizer,
	Mesh *pMesh)
{
	QLIST::Entry<Array<MeshEdgePtr *>> *pBoundary = BoundaryList.pFirst;

	int i;
	int iPt;
	Point *pPt;
	MeshEdgePtr *pEdgePtr;
	BYTE edgeClass;

	while (pBoundary)
	{
		for (i = 0; i < pBoundary->data.n; i++)
		{
			pEdgePtr = pBoundary->data.Element[i];

			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

			pPt = pMesh->NodeArray.Element + iPt;

			edgeClass = (pPt->flags & RVLMESH_POINT_FLAG_EDGE_CLASS);

			if (edgeClass == RVLMESH_POINT_FLAG_FOREGROUND)
				pVisualizer->PaintPoint(iPt, pMesh->pPolygonData, DisplayData.ForegroundColor);
			else if (edgeClass == RVLMESH_POINT_FLAG_BACKGROUND)
				pVisualizer->PaintPoint(iPt, pMesh->pPolygonData, DisplayData.BackgroundColor);
		}

		pBoundary = pBoundary->pNext;
	}
}

void SurfelGraph::DisplayVertexGraph(Visualizer *pVisualizer)
{
	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> linesPolyData =
		vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iVertex;
	SURFEL::Vertex *pVertex;
	double P[3];

	for (iVertex = 0; iVertex < vertexArray.n; iVertex++)
	{
		pVertex = vertexArray.Element[iVertex];

		RVLCOPY3VECTOR(pVertex->P, P);

		pts->InsertNextPoint(P);
	}

	// Add the points to the polydata container
	linesPolyData->SetPoints(pts);

	// Create lines.

	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	unsigned char red[3] = {255, 0, 0};

	vtkSmartPointer<vtkLine> *line = new vtkSmartPointer<vtkLine>[vertexEdgeArray.n];

	int iEdge;
	SURFEL::VertexEdge *pEdge;

	for (iEdge = 0; iEdge < vertexEdgeArray.n; iEdge++)
	{
		pEdge = vertexEdgeArray.Element[iEdge];

		line[iEdge] = vtkSmartPointer<vtkLine>::New();

		line[iEdge]->GetPointIds()->SetId(0, pEdge->iVertex[0]);
		line[iEdge]->GetPointIds()->SetId(1, pEdge->iVertex[1]);

		lines->InsertNextCell(line[iEdge]);

		colors->InsertNextTypedTuple(red);
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

	// vtkSmartPointer<vtkActor> actor =
	//	vtkSmartPointer<vtkActor>::New();
	// actor->SetMapper(mapper);
	vtkSmartPointer<vtkActor> edges = vtkSmartPointer<vtkActor>::New();
	edges->SetMapper(mapper);

	pVisualizer->renderer->AddActor(edges);

	delete[] line;
}

#ifdef RVLSURFEL_IMAGE_ADJACENCY
void SurfelGraph::DisplayConvexAndConcaveEdges(
	Visualizer *pVisualizer,
	Mesh *pMesh)
{
	Figure *pFig = pVisualizer->OpenFigure("Segmentation");

	int iSurfel, iSurfel_, iSurfel__;
	int i;
	int iPt, iPt_;
	Point *pPt;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_;
	QList<MeshEdgePtr> *pEdgeList;
	Surfel *pSurfel, *pSurfel_;
	int iBoundary, iPointEdge;
	Array<MeshEdgePtr *> *pBoundary;
	SurfelAdjecencyDescriptors *pDescriptor;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		if (pSurfel->size <= 1)
			continue;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				pPt = pMesh->NodeArray.Element + iPt;

				iSurfel__ = -1;

				pEdgeList = &(pPt->EdgeList);

				pEdgePtr_ = pEdgeList->pFirst;

				while (pEdgePtr_) // for every neighbor of iPt
				{
					iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

					iSurfel_ = surfelMap[iPt_];

					if (iSurfel_ != iSurfel)
					{
						if (iSurfel__ >= 0)
						{
							if (iSurfel_ != iSurfel__)
								break;
						}
						else
							iSurfel__ = iSurfel_;
					}

					pEdgePtr_ = pEdgePtr_->pNext;
				}

				if (pEdgePtr_ == NULL)
				{
					if (pPt->bBoundary)
					{
						iSurfel_ = edgeMap[iPt];

						iSurfel__ = (iSurfel__ >= 0 ? -1 : iSurfel_);
					}

					if (iSurfel__ >= 0)
					{
						for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
						{
							pSurfel_ = pSurfel->imgAdjacency.at(i);

							iSurfel_ = pSurfel_ - NodeArray.Element;

							if (iSurfel_ == iSurfel__)
							{
								pDescriptor = pSurfel->imgAdjacencyDescriptors.at(i);

								if (pDescriptor->cupyDescriptor[0] > 0)
									pVisualizer->PaintPoint(iPt, pMesh->pPolygonData, DisplayData.ConvexColor, pFig);
								else if (pDescriptor->cupyDescriptor[0] < 0)
									pVisualizer->PaintPoint(iPt, pMesh->pPolygonData, DisplayData.ConcaveColor, pFig);

								break;
							}
						}
					}
				}
			}
		}
	}
}
#endif

void SurfelGraph::DisplayVertices()
{
	double lineLength = DisplayData.normalLen;

	Mesh *pMesh = DisplayData.pMesh;
	Visualizer *pVisualizer = DisplayData.pVisualizer;

	int iVertex;
	Vertex *pVertex;

	// Display occlusion vertices.

	Array<Point> truePtArray;

	truePtArray.Element = new Point[vertexArray.n];

	Array<Point> occlusionPtArray;

	occlusionPtArray.Element = new Point[vertexArray.n];

	truePtArray.n = 0;
	occlusionPtArray.n = 0;

	Point *pPt;

	for (iVertex = 0; iVertex < vertexArray.n; iVertex++)
	{
		pVertex = vertexArray.Element[iVertex];

		if (pVertex->type & RVLSURFELVERTEX_TYPE_OCCLUSION)
		// if ((pVertex->type & RVLSURFELVERTEX_TYPE_CONVEX_CONCAVE) != 3)
		// if (pVertex->iSurfelArray.n < 3)
		{
			pPt = occlusionPtArray.Element + occlusionPtArray.n;

			RVLCOPY3VECTOR(pVertex->P, pPt->P);

			occlusionPtArray.n++;
		}
		else
		{
			pPt = truePtArray.Element + truePtArray.n;

			RVLCOPY3VECTOR(pVertex->P, pPt->P);

			truePtArray.n++;
		}
	}

	uchar trueVertexColor[] = {0, 128, 0};

	pVisualizer->DisplayPointSet<float, Point>(truePtArray, trueVertexColor, 12);

	delete[] truePtArray.Element;

	uchar occlusionVertexColor[] = {128, 0, 0};

	pVisualizer->DisplayPointSet<float, Point>(occlusionPtArray, occlusionVertexColor, 12);

	delete[] occlusionPtArray.Element;

	/// Display the normal hulls of vertices.

	// Create the polydata where we will store all the geometric data
	linesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iLine = 0;

	double P0[3], P[3], V[3];
	int i;

	pVertex = vertexList.pFirst;

	while (pVertex)
	{
		RVLCOPY3VECTOR(pVertex->P, P0);

#ifdef RVLSURFEL_DISPLAY_VERTEX_NORMAL_HULL
		for (i = 0; i < pVertex->normalHull.n; i++)
		{
			pts->InsertNextPoint(P0);

			RVLSCALE3VECTOR(pVertex->normalHull.Element[i].N, lineLength, V);
#else
		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			pSurfel = NodeArray.Element + pVertex->iSurfelArray.Element[iSurfel];

			pts->InsertNextPoint(P0);

			RVLSCALE3VECTOR(pSurfel->N, lineLength, V);
#endif
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

	unsigned char color[5][3] = {{255, 0, 255}, {255, 0, 0}, {255, 128, 0}, {0, 255, 0}, {128, 128, 128}};

	int nLines = iLine;

	vtkSmartPointer<vtkLine> *line = new vtkSmartPointer<vtkLine>[nLines];

	RVL_DELETE_ARRAY(vertexDisplayLineArray.Element);

	vertexDisplayLineArray.Element = new Array<int>[vertexArray.n];

	RVL_DELETE_ARRAY(vertexDisplayLineArrayMem);

	vertexDisplayLineArrayMem = new int[nLines];

	int *pVertexDisplayLineIdx = vertexDisplayLineArrayMem;

	iLine = 0;

	for (iVertex = 0; iVertex < vertexArray.n; iVertex++)
	{
		pVertex = vertexArray.Element[iVertex];

#ifdef RVLSURFEL_DISPLAY_VERTEX_NORMAL_HULL
		vertexDisplayLineArray.Element[iVertex].n = pVertex->normalHull.n;
#else
		vertexDisplayLineArray.Element[iVertex].n = pVertex->iSurfelArray.n;
#endif

		vertexDisplayLineArray.Element[iVertex].Element = pVertexDisplayLineIdx;

#ifdef RVLSURFEL_DISPLAY_VERTEX_NORMAL_HULL
		for (i = 0; i < pVertex->normalHull.n; i++)
#else
		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
#endif
		{
			line[iLine] = vtkSmartPointer<vtkLine>::New();

			line[iLine]->GetPointIds()->SetId(0, 2 * iLine);
			line[iLine]->GetPointIds()->SetId(1, 2 * iLine + 1);

			lines->InsertNextCell(line[iLine]);

			if (pVertex->iSurfelArray.n >= 3)
				colors->InsertNextTypedTuple(color[pVertex->type]);
			else
				colors->InsertNextTypedTuple(color[4]);

			*(pVertexDisplayLineIdx++) = iLine;

			iLine++;
		}
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

	// vtkSmartPointer<vtkActor> actor =
	//	vtkSmartPointer<vtkActor>::New();
	// actor->SetMapper(mapper);
	DisplayData.vertices = vtkSmartPointer<vtkActor>::New();
	DisplayData.vertices->SetMapper(mapper);

	pVisualizer->renderer->AddActor(DisplayData.vertices);

	delete[] line;
}

void SurfelGraph::UpdateVertexDisplayLines()
{
	linesPolyData->Modified();
}

void SurfelGraph::PaintVertices(
	Array<int> *pVertexArray,
	unsigned char *color)
{
	int iVertex;
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(linesPolyData->GetCellData()->GetScalars());

	int i, j;

	for (i = 0; i < pVertexArray->n; i++)
	{
		iVertex = pVertexArray->Element[i];

		for (j = 0; j < vertexDisplayLineArray.Element[iVertex].n; j++)
			rgbPointData->SetTypedTuple(vertexDisplayLineArray.Element[iVertex].Element[j], color);
	}
}

void SurfelGraph::PaintSurfels(
	Mesh *pMesh,
	Visualizer *pVisualizer,
	Array<int> iSurfelArray,
	unsigned char *clusterColor,
	unsigned char *edgeColorIn,
	int *clusterMap)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = NULL;

	if (DisplayData.edgeFeaturesPolyData)
		vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(DisplayData.edgeFeaturesPolyData->GetCellData()->GetScalars());

	unsigned char defaultEdgeColor[] = {255, 255, 255};

	unsigned char *edgeColor = (edgeColorIn ? edgeColorIn : defaultEdgeColor);

	int i;
	int iSurfel;
	Surfel *pSurfel;
	unsigned char *color;

	for (i = 0; i < iSurfelArray.n; i++)
	{
		iSurfel = iSurfelArray.Element[i];

		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
		{
			if (rgbPointData)
				rgbPointData->SetTypedTuple(DisplayData.edgeFeatureIdxArray[iSurfel], edgeColor);
		}
		else
		{
			color = (clusterMap ? clusterColor + 3 * clusterMap[iSurfel] : clusterColor);

			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);
		}
	}
}

void SurfelGraph::DisplayRGB(cv::Mat RGB)
{
	int nPix = RGB.rows * RGB.cols;
	uchar *color;
	int iSurfel;
	Surfel *pSurfel;
	uchar *pix;
	for (int iPix = 0; iPix < nPix; iPix++)
	{
		iSurfel = surfelMap[iPix];
		if (iSurfel < 0 || iSurfel >= NodeArray.n)
			continue;
		pSurfel = NodeArray.Element + iSurfel;
		if (pSurfel->bEdge)
			continue;
		color = nodeColor + 3 * iSurfel;
		pix = RGB.data + 3 * iPix;
		RVLCOPY3VECTOR(color, pix);
	}
}

void SurfelGraph::DisplaySphericalNormalHistogram(
	Array<int> surfels,
	Visualizer *pVisualizer)
{
	Array<Pair<Vector3<float>, float>> normals;
	normals.Element = new Pair<Vector3<float>, float>[surfels.n];
	normals.n = surfels.n;
	for (int i = 0; i < surfels.n; i++)
	{
		int iSurfel = surfels.Element[i];
		Surfel *pSurfel = NodeArray.Element + iSurfel;
		RVLCOPY3VECTOR(pSurfel->N, normals.Element[i].a.Element);
		normals.Element[i].b = (float)(pSurfel->size);
	}
	pVisualizer->DisplaySphericalHistogram(normals);

	delete[] normals.Element;
}

void SurfelGraph::SetDisplayPolygonsOn()
{
	DisplayData.bPolygons = true;
}

void SurfelGraph::Save(
	int iSurfel,
	Mesh *pMesh,
	FILE *fpPoints,
	FILE *fpEdges)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	int iPt, iPt_;
	Point *pPt;
	MeshEdge *pEdge;
	MeshEdgePtr *pEdgePtr;

	QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;

	while (pPtIdx)
	{
		iPt = pPtIdx->Idx;

		pPt = pMesh->NodeArray.Element + iPt;

		fprintf(fpPoints, "%d\t%f\t%f\t%f\t%d\n", iPt, pPt->P[0], pPt->P[1], pPt->P[2], 1);

		pEdgePtr = pPt->EdgeList.pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

			if (surfelMap[iPt_] == iSurfel)
				if (iPt < iPt_)
					fprintf(fpEdges, "%d\t%d\t%d\t1\n", pEdge->idx, iPt, iPt_);

			pEdgePtr = pEdgePtr->pNext;
		}

		pPtIdx = pPtIdx->pNext;
	}
}

void SurfelGraph::SaveSurfel(
	FILE *fp,
	int iSurfel)
{
	fwrite(&iSurfel, sizeof(int), 1, fp);

	Surfel *pSurfel = NodeArray.Element + iSurfel;

	fwrite(pSurfel->N, sizeof(float), 3, fp);
	fwrite(&(pSurfel->d), sizeof(float), 1, fp);
	fwrite(pSurfel->P, sizeof(float), 3, fp);
	fwrite(pSurfel->RGB, sizeof(int), 3, fp);
}

void SurfelGraph::LoadSurfel(
	FILE *fp,
	int iSurfel)
{
	fread(&iSurfel, sizeof(int), 1, fp);

	Surfel *pSurfel = NodeArray.Element + iSurfel;

	fread(pSurfel->N, sizeof(float), 3, fp);
	fread(&(pSurfel->d), sizeof(float), 1, fp);
	fread(pSurfel->P, sizeof(float), 3, fp);
	fread(pSurfel->RGB, sizeof(int), 3, fp);
}

void SurfelGraph::Save(
	FILE *fp,
	char *meshFileName,
	void *vpDetector)
{
	char header[] = "RVL::SurfelGraph 000";

	int headerLength = strlen(header);

	sprintf(header + headerLength - 3, "%03d", RVLSURFEL_VERSION_0);

	fwrite(header, sizeof(char), headerLength + 1, fp);

	fwrite(meshFileName, sizeof(char), strlen(meshFileName) + 1, fp);

	PlanarSurfelDetector *pDetector = (PlanarSurfelDetector *)vpDetector;

	pDetector->Save(fp);

	fwrite(&nMeshVertices, sizeof(int), 1, fp);
	fwrite(surfelMap, sizeof(int), nMeshVertices, fp);

	fwrite(&(NodeArray.n), sizeof(int), 1, fp);

	int iSurfel;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
		if (NodeArray.Element[iSurfel].size > 0)
			SaveSurfel(fp, iSurfel);
}

void SurfelGraph::CalculateSurfelsColorHistograms(
	Mesh *pMesh,
	int colorspace,
	bool oneDimensional,
	const int *bindata,
	bool noBins,
	const int *chFilterThrreshold,
	bool useFilter)
{
	// Calculate color histograms for all surfels in surfel graph
	Surfel *pCurrSurfel = NodeArray.Element;

	// uint8_t colorPts[640 * 480 * 3];            //Max number of points
	cv::Mat colorpix(1, 640 * 480, CV_8UC3); // Max number of points // OpenCV class because of color conversion

	for (int s = 0; s < NodeArray.n; pCurrSurfel++, s++)
	{
		// check
		if ((pCurrSurfel->size <= 1) || pCurrSurfel->bEdge)
			continue;

		// create descriptor
		pCurrSurfel->colordescriptor = std::make_shared<RVLColorDescriptor>(RVLColorDescriptor(colorspace, oneDimensional, bindata, noBins, chFilterThrreshold));

		// run through all surfel pixels and fill the array
		RVL::QLIST::Index2 *pt;

		Point *point;
		pt = pCurrSurfel->PtList.pFirst;

		for (int i = 0; i < pCurrSurfel->size; i++)
		{
			point = pMesh->NodeArray.Element + pt->Idx;
			memcpy(&colorpix.data[i * 3], &point->RGB, 3); // Hardcoded for 3-channel color data
			pt = pt->pNext;
		}

		// If we use RGB color space than the data can go in direcly, but if we use HSV or Lab than conversion must take place
		if (colorspace == RVLColorDescriptor::ColorSpaceList::HSV)
			cv::cvtColor(colorpix, colorpix, CV_RGB2HSV);

		else if (colorspace == RVLColorDescriptor::ColorSpaceList::Lab)
			cv::cvtColor(colorpix, colorpix, CV_RGB2Lab);

		// Calculate histogram
		pCurrSurfel->colordescriptor->InsertArrayIntoHistogram(colorpix.data, pCurrSurfel->size, useFilter);

		// pCurrSurfel->colordescriptor->DisplayColorHistogram(true);
	}
}

#ifdef RVLSURFEL_COLOR_HISTOGRAM
void SurfelGraph::CalculateSurfelsColorHistograms(cv::Mat img, int colorspace, bool oneDimensional, const int *bindata, bool noBins)
{
	// Calculate color histograms for all surfels in surfel graph
	Surfel *pCurrSurfel = NodeArray.Element;
	uint8_t colorPts[640 * 480 * 3];
	for (int s = 0; s < NodeArray.n; pCurrSurfel++, s++)
	{
		// check
		if ((pCurrSurfel->size <= 1) || pCurrSurfel->bEdge)
			continue;
		// create descriptor
		pCurrSurfel->colordescriptor = new RVLColorDescriptor(colorspace, oneDimensional, bindata, noBins);
		// run through all surfel pixels and filling array
		RVL::QLIST::Index2 *pt;
		pt = pCurrSurfel->PtList.pFirst;
		for (int i = 0; i < pCurrSurfel->size; i++)
		{
			memcpy(&colorPts[i * 3], &img.data[pt->Idx * 3], 3); // Hardcoded for 3-channel images
			pt = pt->pNext;
		}
		// Calculate histogram
		pCurrSurfel->colordescriptor->InsertArrayIntoHistogram(colorPts, pCurrSurfel->size);
	}
}
#endif

bool SurfelGraph::FitPlane(
	Mesh *pMesh,
	Array<int> iSurfelArray,
	MESH::Distribution &PtDistribution,
	Array<int> &PtArray)
{
	GetPoints(iSurfelArray, PtArray);

	if (PtArray.n == 0)
		return false;

	pMesh->ComputeDistributionDouble(PtArray, PtDistribution);

	return true;
}

void SurfelGraph::DetectDominantPlane(
	Mesh *pMesh,
	Array<int> &dominantPlaneSurfelArray,
	float *N,
	float &d)
{
	// Detect largest surfel.

	int largestSurfelSize = 0;

	int iSurfel, iLargestSurfel;
	Surfel *pSurfel;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		if (pSurfel->size > largestSurfelSize)
		{
			largestSurfelSize = pSurfel->size;

			iLargestSurfel = iSurfel;
		}
	}

	pSurfel = NodeArray.Element + iLargestSurfel;

	float *NGnd = pSurfel->N;
	float dGnd = pSurfel->d;

	// Add neighboring surfels recursively by region growing.

	PlaneDetectionRGData RGData;

	RGData.csqThr = cos(10.0f * DEG2RAD);

	RGData.bVisited = new bool[NodeArray.n];

	memset(RGData.bVisited, 0, NodeArray.n * sizeof(bool));

	RGData.bVisited[iLargestSurfel] = true;

	RVLCOPY3VECTOR(pSurfel->N, RGData.NRef);

	dominantPlaneSurfelArray.Element = new int[NodeArray.n];

	int *piSurfelPut = dominantPlaneSurfelArray.Element;

	int *piSurfelFetch = piSurfelPut;

	*(piSurfelPut++) = iLargestSurfel;

	int *piSurfelBuffEnd = RegionGrowing<SurfelGraph, Surfel, Edge, EdgePtr, PlaneDetectionRGData, PlaneDetectionRG>(this, &RGData, piSurfelFetch, piSurfelPut);

	dominantPlaneSurfelArray.n = piSurfelBuffEnd - dominantPlaneSurfelArray.Element;

	delete[] RGData.bVisited;

	// Set GND flag of all surfels belonging to the dominant plane.

	int i;

	for (i = 0; i < dominantPlaneSurfelArray.n; i++)
		NodeArray.Element[dominantPlaneSurfelArray.Element[i]].flags |= RVLSURFEL_FLAG_GND;

	Array<int> PtArray;

	PtArray.Element = new int[pMesh->NodeArray.n];

	MESH::Distribution PtDistribution;

	FitPlane(pMesh, dominantPlaneSurfelArray, PtDistribution, PtArray);

	delete[] PtArray.Element;

	float minVar = PtDistribution.var[0];
	int iMinVar = 0;

	for (i = 1; i < 3; i++)
		if (PtDistribution.var[i] < minVar)
		{
			minVar = PtDistribution.var[i];
			iMinVar = i;
		}

	float *N_ = PtDistribution.R + 3 * iMinVar;

	if (N_[2] > 0.0f)
	{
		RVLNEGVECT3(N_, N);
	}
	else
	{
		RVLCOPY3VECTOR(N_, N);
	}

	d = RVLDOTPRODUCT3(N, PtDistribution.t);

	// Surfel *pGndSurfel = NodeArray.Element + dominantPlaneSurfelArray.Element[0];

	// RVLCOPY3VECTOR(pGndSurfel->N, N);

	// d = pGndSurfel->d;
}

void SurfelGraph::ComputeDistribution(
	Mesh *pMesh,
	Array<int> iSurfelArray,
	GaussianDistribution3D<float> *pDistribution,
	int *PtMemIn)
{
	int *PtMem = (PtMemIn ? PtMemIn : new int[pMesh->NodeArray.n]);

	Array<int> PtArray;

	PtArray.Element = PtMem;

	GetPoints(iSurfelArray, PtArray);

	// For debugging purposes!!!

	// FILE *fp = fopen("P.txt", "w");

	// float *P;

	// for (int i = 0; i < PtArray.n; i++)
	//{
	//	P = pMesh->NodeArray.Element[PtArray.Element[i]].P;

	//	fprintf(fp, "%f\t%f\t%f\n", P[0], P[1], P[2]);
	//}

	// fclose(fp);

	/////

	if (PtArray.n > 0)
		pMesh->ComputeDistribution(PtArray, pDistribution);

	if (PtMemIn == NULL)
		delete[] PtMem;
}

void SurfelGraph::GetDepthImageROI(
	Array<int> iVertexArray,
	Camera camera,
	Rect<float> &ROI)
{
	float *P = vertexArray.Element[iVertexArray.Element[0]]->P;

	float m[2];

	m[0] = camera.fu * P[0] / P[2] + camera.uc;
	m[1] = camera.fv * P[1] / P[2] + camera.vc;

	InitRect<float>(&ROI, m);

	int i;

	for (i = 1; i < iVertexArray.n; i++)
	{
		P = vertexArray.Element[iVertexArray.Element[i]]->P;

		m[0] = camera.fu * P[0] / P[2] + camera.uc;
		m[1] = camera.fv * P[1] / P[2] + camera.vc;

		UpdateRect<float>(&ROI, m);
	}
}

void SurfelGraph::DetectOcclusionVertices(
	Mesh *pMesh,
	Camera camera)
{
	int halfWinSize = (occlusionVertexWinSize - 1) / 2;

	Point *ptArray = pMesh->NodeArray.Element;

	int maxiZ = (int)ceil(occlusionVertexMaxZ / occlusionVertexResolutionZ);
	int miniZ = (int)floor(occlusionVertexMinZ / occlusionVertexResolutionZ);

	float scaleZ = 1.0f / (float)occlusionVertexResolutionZ;

	int histSize = maxiZ - miniZ + 1;

	int *histMem = new int[histSize];

	memset(histMem, 0, histSize * sizeof(int));

	Array<int> clusters;

	clusters.Element = new int[histSize];

	int *assignment = new int[histSize];

	int *wAgg = new int[histSize];

	int *move = new int[histSize];

	int *w = new int[histSize];

	Rect<int> img;

	img.minx = 0;
	img.maxx = camera.w - 1;
	img.miny = 0;
	img.maxy = camera.h - 1;

	int iZForgroundCluster = 0;

	int halfMeanShiftWinSize = (occlusionVertexMeanShiftWinSize - 1) / 2;

	int miniZ__ = miniZ + halfMeanShiftWinSize;
	int maxiZ__ = maxiZ - halfMeanShiftWinSize;

	int i, u, v, u0, v0, iVertex, iZ, miniZ_, maxiZ_, iForegroundCluster, iCluster, iZCluster;
	float *P;
	Rect<int> win;
	SURFEL::Vertex *pVertex;
	Array<int> histZ;
	int nDepthValues, iSide, du, uBound;
	bool bFirstCluster;
	int wCluster, wGap, i0;

	for (iVertex = 0; iVertex < vertexArray.n; iVertex++)
	{
		pVertex = vertexArray.Element[iVertex];

		P = pVertex->P;

		// float PDebug[3];
		// RVLSET3VECTOR(PDebug, -0.017, -0.015, 0.616);
		// RVLDIF3VECTORS(PDebug, P, PDebug);
		// if (sqrt(RVLDOTPRODUCT3(PDebug, PDebug)) < 0.002)
		//	int debug = 0;

		u0 = (int)floor(camera.fu * P[0] / P[2] + camera.uc + 0.5f);
		v0 = (int)floor(camera.fv * P[1] / P[2] + camera.vc + 0.5f);

		win.minx = u0 - halfWinSize;
		win.maxx = u0 + halfWinSize;
		win.miny = v0 - halfWinSize;
		win.maxy = v0 + halfWinSize;

		if (!IsContainedInRect(win, img))
		{
			pVertex->type |= RVLSURFELVERTEX_TYPE_OCCLUSION;

			continue;
		}

		// Create histogram.

		miniZ_ = histSize;
		maxiZ_ = -1;

		for (v = win.miny; v <= win.maxy; v++)
		{
			uBound = 0;

			du = -1;

			for (iSide = 0; iSide < 2; iSide++)
			{
				u = u0 + iSide;

				nDepthValues = 0;

				while (nDepthValues < halfWinSize)
				{
					iZ = (int)(scaleZ * ptArray[u + v * camera.w].P[2]);

					if (iZ >= miniZ__ && iZ <= maxiZ__)
					{
						i = iZ - miniZ;

						if (i < miniZ_)
							miniZ_ = i;

						if (i > maxiZ_)
							maxiZ_ = i;

						histMem[i]++;

						nDepthValues++;
					}

					if (u == uBound)
						break;

					u += du;
				}

				if (nDepthValues < halfWinSize)
					break;

				uBound = camera.w - 1;

				du = 1;
			}

			if (nDepthValues < halfWinSize)
				break;
		}

		if (nDepthValues < halfWinSize)
		{
			pVertex->type |= RVLSURFELVERTEX_TYPE_OCCLUSION;

			continue;
		}

		histZ.Element = histMem + miniZ_ - halfMeanShiftWinSize;
		histZ.n = maxiZ_ - miniZ_ + occlusionVertexMeanShiftWinSize;

		/// Vertex classification based on Z-histogram.

		if (histZ.n > occlusionVertexMeanShiftWinSize)
		{
			//// Histogram clustering by Mean Shift.

			// MeanShift1D(histZ, occlusionVertexMeanShiftWinSize, occlusionVertexMinClusterSize, clusters, assignment, wAgg, move, w);

			//// Vertex classification.

			// if (clusters.n > 1)
			//{
			//	iForegroundCluster = -1;

			//	for (iCluster = 0; iCluster < clusters.n; iCluster++)
			//	{
			//		iZCluster = clusters.Element[iCluster];

			//		if (iForegroundCluster < 0 || iZCluster < iZForgroundCluster)
			//		{
			//			iForegroundCluster = iCluster;

			//			iZForgroundCluster = iZCluster;
			//		}
			//	}

			//	i = (int)(scaleZ * ptArray[u0 + v0 * camera.w].P[2]) - miniZ - miniZ_ + halfMeanShiftWinSize;

			//	if (assignment[i] != iZForgroundCluster)
			//		pVertex->type |= RVLSURFELVERTEX_TYPE_OCCLUSION;
			//}

			// Vertex classification based on histogram clusters.

			i0 = (int)(scaleZ * P[2]) - miniZ - miniZ_ + halfMeanShiftWinSize;

			if (i0 > 0)
			{
				bFirstCluster = false;
				wGap = 0;
				wCluster = 0;

				for (i = 0; i <= i0; i++)
				{
					if (bFirstCluster)
					{
						if (histZ.Element[i] == 0)
						{
							wGap++;

							if (wGap >= occlusionVertexMinDepthStep)
								break;
						}
						else
							wGap = 0;
					}
					else
					{
						if (histZ.Element[i] > 0)
						{
							wCluster += histZ.Element[i];

							if (wCluster >= occlusionVertexMinClusterSize)
								bFirstCluster = true;
						}
					}
				}

				if (i < i0)
					pVertex->type |= RVLSURFELVERTEX_TYPE_OCCLUSION;
			}
			else
				pVertex->type |= RVLSURFELVERTEX_TYPE_OCCLUSION;
		}

		///

		memset(histZ.Element, 0, histZ.n * sizeof(int));
	}

	delete[] histMem;
	delete[] clusters.Element;
	delete[] assignment;
	delete[] wAgg;
	delete[] move;
	delete[] w;
}

void SurfelGraph::HideOcclustionFaces(
	Mesh *pMesh,
	Array<int> iVertexArray)
{
	int *iVisibleFacesBuff = new int[pMesh->iVisibleFaces.n];

	memcpy(iVisibleFacesBuff, pMesh->iVisibleFaces.Element, pMesh->iVisibleFaces.n * sizeof(int));

	int nVisibleFaces = 0;

	int i, j, iFace;
	MESH::Face *pFace;
	int iMeshPt, iVertex;
	MeshEdgePtr *pEdgePtr, *pEdgePtr0;
	bool bValid;

	for (i = 0; i < pMesh->iVisibleFaces.n; i++)
	{
		iFace = iVisibleFacesBuff[i];

		pFace = pMesh->faces.Element[iFace];

		bValid = false;

		pEdgePtr0 = pEdgePtr = pFace->pFirstEdgePtr;

		do
		{
			iMeshPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

			iVertex = iVertexArray.Element[iMeshPt];

			if (!(vertexArray.Element[iVertex]->type & RVLSURFELVERTEX_TYPE_OCCLUSION))
			{
				bValid = true;

				break;
			}

			pEdgePtr = pEdgePtr->pNext;

			if (pEdgePtr == NULL)
				pEdgePtr = pMesh->NodeArray.Element[iMeshPt].EdgeList.pFirst;

			pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);
		} while (pEdgePtr != pEdgePtr0);

		if (bValid)
			pMesh->iVisibleFaces.Element[nVisibleFaces++] = iFace;
		else
			pFace->flags &= ~RVLMESH_FACE_FLAG_VISIBLE;
	}

	delete[] iVisibleFacesBuff;

	pMesh->iVisibleFaces.n = nVisibleFaces;
}

void SurfelGraph::TransformVertices(
	Array<int> iVertexArray,
	float scale,
	float *R,
	float *t,
	float *PArray)
{
	float sR[9];
	float st[3];

	// sR <- scale * R

	RVLSCALEMX3X3(R, scale, sR);
	RVLSCALE3VECTOR2(t, scale, st);

	// Transform vertices to TG RF.

	int j, iVertex;
	SURFEL::Vertex *pVertex;
	float *P;
	float V3Tmp[3];

	for (j = 0; j < iVertexArray.n; j++)
	{
		iVertex = iVertexArray.Element[j];

		pVertex = vertexArray.Element[iVertex];

		// if (pVertex->normalHull.n < 3)
		//	continue;

		P = PArray + 3 * j;

		RVLINVTRANSF3(pVertex->P, sR, st, P, V3Tmp);
	}
}

void SurfelGraph::ProjectVerticesOntoGroundPlane(
	Array<int> iVertexArray,
	float *NGnd,
	float dGnd,
	float *PGnd)
{
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

		pVertex = vertexArray.Element[iVertex];

		P = pVertex->P;

		s = dGnd / RVLDOTPRODUCT3(NGnd, P);

		RVLSCALE3VECTOR(P, s, PGnd_);
	}
}

bool SurfelGraph::Coplanar(
	float *N,
	float d,
	Array<int> iVertexArray,
	float tolerance)
{
	int i;
	float *P;
	float e;

	for (i = 0; i < iVertexArray.n; i++)
	{
		P = vertexArray.Element[iVertexArray.Element[i]]->P;

		e = RVLDOTPRODUCT3(N, P) - d;

		if (RVLABS(e) > tolerance)
			return false;
	}

	return true;
}

bool SurfelGraph::Coplanar(
	float *N,
	float d,
	QList<QLIST::Index> *pVertexList,
	float tolerance)
{
	float *P;
	float e;

	QLIST::Index *pVertexIdx = pVertexList->pFirst;

	while (pVertexIdx)
	{
		P = vertexArray.Element[pVertexIdx->Idx]->P;

		e = RVLDOTPRODUCT3(N, P) - d;

		if (RVLABS(e) > tolerance)
			return false;

		pVertexIdx = pVertexIdx->pNext;
	}

	return true;
}

bool SurfelGraph::Coplanar(
	Moments<double> moments1,
	Array<int> iVertexArray1,
	Moments<double> moments2,
	QList<QLIST::Index> *pVertexList2,
	float tolerance)
{
	Moments<double> moments;

	SumMoments<double>(moments1, moments2, moments);

	MESH::Distribution distribution;

	MESH::ComputeDistributionDouble(moments, distribution);

	float N[3];
	float d;

	MESH::ComputePlaneParameters(distribution, N, d);

	return (Coplanar(N, d, iVertexArray1, tolerance) && Coplanar(N, d, pVertexList2, tolerance));
}

bool SurfelGraph::Below(int iSurfel, int iSurfel_)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	float *N = pSurfel->N;

	QList<QLIST::Index> *pSurfelVertexList = surfelVertexList.Element + iSurfel_;

	QLIST::Index *pVertexIdx = pSurfelVertexList->pFirst;

	float e;
	int iVertex;
	SURFEL::Vertex *pVertex;
	Surfel *pSurfel_;

	while (pVertexIdx)
	{
		pVertex = vertexArray.Element[pVertexIdx->Idx];

		e = RVLDOTPRODUCT3(N, pVertex->P) - pSurfel->d;

		if (e > 0.0f)
			return false;

		pVertexIdx = pVertexIdx->pNext;
	}

	return true;
}

void SurfelGraph::EdgePointNormals(Mesh *pMesh)
{
	int iSurfel;
	Surfel *pSurfel;
	QLIST::Index2 *pPtIdx;
	Point *pPt;
	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfel = NodeArray.Element + iSurfel;
		if (!pSurfel->bEdge)
			continue;
		pPtIdx = pSurfel->PtList.pFirst;
		while (pPtIdx)
		{
			pPt = pMesh->NodeArray.Element + pPtIdx->Idx;
			RVLCOPY3VECTOR(pSurfel->N, pPt->N);
			pPtIdx = pPtIdx->pNext;
		}
	}
}

void SurfelGraph::RepresentativeSurfelSamples(
	Mesh *pMesh,
	int minSurfelSize,
	int minEdgeSize)
{
	RVL_DELETE_ARRAY(surfelRefPtMem);
	surfelRefPtMem = new float[3 * 4 * NodeArray.n];
	float *surfelRefPts = surfelRefPtMem;
	int iPt, iSurfel;
	Surfel *pSurfel;
	Point *pPt;
	float V[3], V3Tmp[3];
	float *P_;
	int nSamples;
	int i;
	float fTmp;
	int iBoundary;
	int maxBoudarySize;
	int iLargestBoundary;
	Array<MeshEdgePtr *> boundary;
	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++, surfelRefPts += 12)
	{
		// if (iSurfel == 1391)
		//	int debug = 0;
		pSurfel = NodeArray.Element + iSurfel;
		pSurfel->representativePts = surfelRefPts;
		nSamples = 0;
		if (pSurfel->bEdge)
		{
			if (pSurfel->size < minEdgeSize)
				continue;
			P_ = surfelRefPts;
			RVLCOPY3VECTOR(pSurfel->P, P_);
			iPt = pMesh->FurthestPoint(pSurfel->P, pSurfel->PtList);
			pPt = pMesh->NodeArray.Element + iPt;
			P_ += 3;
			RVLCOPY3VECTOR(pPt->P, P_);
			iPt = pMesh->FurthestPoint(pPt->P, pSurfel->PtList);
			pPt = pMesh->NodeArray.Element + iPt;
			P_ += 3;
			RVLCOPY3VECTOR(pPt->P, P_);
			nSamples = 3;
		}
		else
		{
			if (pSurfel->size < minSurfelSize)
				continue;
			P_ = surfelRefPts;
			RVLCOPY3VECTOR(pSurfel->P, P_);
			maxBoudarySize = 0;
			for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
				if (pSurfel->BoundaryArray.Element[iBoundary].n > maxBoudarySize)
				{
					maxBoudarySize = pSurfel->BoundaryArray.Element[iBoundary].n;
					iLargestBoundary = iBoundary;
				}
			boundary = pSurfel->BoundaryArray.Element[iLargestBoundary];
			iPt = pMesh->FurthestPoint(pSurfel->P, boundary);
			pPt = pMesh->NodeArray.Element + iPt;
			P_ += 3;
			RVLCOPY3VECTOR(pPt->P, P_);
			iPt = pMesh->FurthestPoint(pPt->P, boundary);
			pPt = pMesh->NodeArray.Element + iPt;
			RVLDIF3VECTORS(pPt->P, P_, V);
			P_ += 3;
			RVLCOPY3VECTOR(pPt->P, P_);
			RVLNORM3(V, fTmp);
			iPt = pMesh->FurthestPoint(pPt->P, boundary, V);
			pPt = pMesh->NodeArray.Element + iPt;
			P_ += 3;
			RVLCOPY3VECTOR(pPt->P, P_);
			nSamples = 4;
		}
		P_ = surfelRefPts;
		for (i = 0; i < nSamples; i++, P_ += 3)
		{
			fTmp = RVLDOTPRODUCT3(pSurfel->N, P_) - pSurfel->d;
			RVLSCALE3VECTOR(pSurfel->N, fTmp, V3Tmp);
			RVLDIF3VECTORS(P_, V3Tmp, P_);
		}
	}
}

void SurfelGraph::RepresentativeComplexSurfelSamples(
	Mesh *pMesh,
	int minSurfelSize,
	int minEdgeSize,
	SurfelGraph *pElements)
{
	RVL_DELETE_ARRAY(surfelRefPtMem);
	surfelRefPtMem = new float[3 * 4 * NodeArray.n];
	float *surfelRefPts = surfelRefPtMem;
	int iPt, iSurfel;
	Surfel *pSurfel;
	float V[3], V3Tmp[3];
	float *P, *P_, *P__;
	int i;
	float fTmp;
	Array<Vector3<float>> elementRefPts;
	elementRefPts.Element = new Vector3<float>[pElements->NodeArray.n * 3];
	QLIST::Index *pMemberSurfelIdx;
	Surfel *pMemberSurfel;
	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++, surfelRefPts += 12)
	{
		pSurfel = NodeArray.Element + iSurfel;
		pSurfel->representativePts = surfelRefPts;
		if (pSurfel->size < minSurfelSize)
			continue;
		P_ = surfelRefPts;
		RVLCOPY3VECTOR(pSurfel->P, P_);
		elementRefPts.n = 0;
		pMemberSurfelIdx = pSurfel->children.pFirst;
		while (pMemberSurfelIdx)
		{
			pMemberSurfel = pElements->NodeArray.Element + pMemberSurfelIdx->Idx;
			P__ = pMemberSurfel->representativePts + 3;
			for (i = 1; i < 4; i++, P__ += 3)
			{
				P = elementRefPts.Element[elementRefPts.n++].Element;
				RVLCOPY3VECTOR(P__, P);
			}
			pMemberSurfelIdx = pMemberSurfelIdx->pNext;
		}
		iPt = FurthestPoint(pSurfel->P, elementRefPts);
		P = elementRefPts.Element[iPt].Element;
		P_ += 3;
		RVLCOPY3VECTOR(P, P_);
		iPt = FurthestPoint(P, elementRefPts);
		P = elementRefPts.Element[iPt].Element;
		RVLDIF3VECTORS(P, P_, V);
		P_ += 3;
		RVLCOPY3VECTOR(P, P_);
		RVLNORM3(V, fTmp);
		iPt = FurthestPoint(P, elementRefPts, V);
		P = elementRefPts.Element[iPt].Element;
		P_ += 3;
		RVLCOPY3VECTOR(P, P_);
		P_ = surfelRefPts;
		for (i = 0; i < 4; i++, P_ += 3)
		{
			fTmp = RVLDOTPRODUCT3(pSurfel->N, P_) - pSurfel->d;
			RVLSCALE3VECTOR(pSurfel->N, fTmp, V3Tmp);
			RVLDIF3VECTORS(P_, V3Tmp, P_);
		}
	}
	delete[] elementRefPts.Element;
}

int SURFEL::PlaneDetectionRG(
	int iSurfel,
	int iSurfel_,
	Edge *pEdge,
	SurfelGraph *pSurfels,
	SURFEL::PlaneDetectionRGData *pData)
{
	if (pData->bVisited[iSurfel])
		return 0;

	pData->bVisited[iSurfel] = true;

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;

	if (pSurfel->bEdge)
		return 0;

	if (pSurfel->size <= 1)
		return 0;

	return (RVLDOTPRODUCT3(pSurfel->N, pData->NRef) >= pData->csqThr ? 1 : 0);
}

void RVL::SampleMesh(
	Mesh *pMesh,
	float *R,
	float *t,
	Array<MESH::Sample> &sampleArray)
{
	int nSamplePts = sampleArray.n;

	Array<int> iPtArray;

	iPtArray.n = pMesh->NodeArray.n;

	RandomIndices(iPtArray);

	int i;
	float *P_;
	MESH::Sample *pSample;

	for (i = 0; i < nSamplePts; i++)
	{
		pSample = sampleArray.Element + i;

		P_ = pMesh->NodeArray.Element[iPtArray.Element[i]].P;

		RVLTRANSF3(P_, R, t, pSample->P);

		pSample->SDF = 0.0f;
	}
}

void RVL::SampleMeshDistanceFunction(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	float voxelSize,
	int sampleVoxelDistance,
	Array3D<Voxel> &volume,
	float *P0,
	Array<MESH::Sample> &sampleArray,
	Box<float> &boundingBox)
{
	int border = 2 * sampleVoxelDistance + 1;

	Array<int> zeroDistanceVoxelArray;
	QLIST::Index *PtMem;

	MESH::TSDF(pMesh->pPolygonData, voxelSize, border, volume, P0, boundingBox, zeroDistanceVoxelArray, PtMem);

	// sampleArray <- array of sample points at distance approximatelly equal to sampleVoxelDistance.
	// Field SDF of every sample represents the distance function value.

	int nVoxels = volume.a * volume.b * volume.c;

	// int maxVoxelDistance = volume.a + volume.b + volume.c;

	int maxVoxelDistance = sampleVoxelDistance + 1;

	int iVoxel;
	Voxel *pVoxel;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
	{
		pVoxel = volume.Element + iVoxel;

		if (pVoxel->voxelDistance > 0)
			pVoxel->voxelDistance = maxVoxelDistance;
	}

	sampleArray.Element = new MESH::Sample[nVoxels];

	sampleArray.n = 0;

	int *RGBuff = new int[nVoxels];

	int *pPut = RGBuff + zeroDistanceVoxelArray.n;

	int *pFetch = RGBuff;

	memcpy(RGBuff, zeroDistanceVoxelArray.Element, zeroDistanceVoxelArray.n * sizeof(int));

	float maxeSDF = voxelSize * (sampleVoxelDistance + 2);

	float maxDist = maxeSDF * maxeSDF;

	int dijk[][3] = {
		{-1, 0, 0},
		{1, 0, 0},
		{0, -1, 0},
		{0, 1, 0},
		{0, 0, -1},
		{0, 0, 1}};

	int i, j, k, l;
	int i_, j_, k_;
	int voxelDistance;
	int i__, j__, k__;
	int p, q, r;
	float dist, minDist;
	int iVoxel_, iVoxel__;
	float *P_;
	float dP[3];
	int iClosestPt;
	MESH::Sample *pSample;
	float SDF, eSDF, mineSDF;
	Surfel *pFeature;
	int iFeature;
	QLIST::Index *pPtIdx;
	float *P;

	while (pPut > pFetch)
	{
		iVoxel = (*pFetch++);

		pVoxel = volume.Element + iVoxel;

		voxelDistance = pVoxel->voxelDistance + 1;

		RVL3DARRAY_INDICES(volume, iVoxel, i, j, k);

		for (l = 0; l < 6; l++)
		{
			i_ = i + dijk[l][0];
			j_ = j + dijk[l][1];
			k_ = k + dijk[l][2];

			if (i_ >= 0 && i_ < volume.a && j_ >= 0 && j_ < volume.b && k_ >= 0 && k_ < volume.c)
			{
				iVoxel_ = RVL3DARRAY_INDEX(volume, i_, j_, k_);

				pVoxel = volume.Element + iVoxel_;

				if (pVoxel->voxelDistance > voxelDistance)
				{
					pVoxel->voxelDistance = voxelDistance;

					*(pPut++) = iVoxel_;

					if (voxelDistance == sampleVoxelDistance)
					{
						pSample = sampleArray.Element + sampleArray.n;

						P = pSample->P;

						P[0] = (float)i_ * voxelSize;
						P[1] = (float)j_ * voxelSize;
						P[2] = (float)k_ * voxelSize;

						RVLSUM3VECTORS(P, P0, P);

						minDist = maxDist;

						iClosestPt = -1;

						for (k__ = k_ - sampleVoxelDistance; k__ <= k_ + sampleVoxelDistance; k__++)
							for (j__ = j_ - sampleVoxelDistance; j__ <= j_ + sampleVoxelDistance; j__++)
								for (i__ = i_ - sampleVoxelDistance; i__ <= i_ + sampleVoxelDistance; i__++)
								{
									if (i__ == i_ && j__ == j_ && k__ == k_)
										continue;

									iVoxel__ = RVL3DARRAY_INDEX(volume, i__, j__, k__);

									pPtIdx = volume.Element[iVoxel__].PtList.pFirst;

									while (pPtIdx)
									{
										P_ = pMesh->NodeArray.Element[pPtIdx->Idx].P;

										RVLDIF3VECTORS(P_, P, dP);

										dist = RVLDOTPRODUCT3(dP, dP);

										if (dist < minDist)
										{
											minDist = dist;

											iClosestPt = pPtIdx->Idx;

											p = i__;
											q = j__;
											r = k__;
										}

										pPtIdx = pPtIdx->pNext;
									}
								}

						pSample->iFeature = -1;

						mineSDF = maxeSDF;

						if (iClosestPt >= 0)
						{
							pSample->SDF = sqrt(minDist);

							for (k__ = r - 1; k__ <= r + 1; k__++)
								for (j__ = q - 1; j__ <= q + 1; j__++)
									for (i__ = p - 1; i__ <= p + 1; i__++)
									{
										iVoxel__ = RVL3DARRAY_INDEX(volume, i__, j__, k__);

										pPtIdx = volume.Element[iVoxel__].PtList.pFirst;

										while (pPtIdx)
										{
											iFeature = pSurfels->surfelMap[pPtIdx->Idx];

											if (iFeature >= 0)
											{
												pFeature = pSurfels->NodeArray.Element + iFeature;

												if (pFeature->size > 1)
												{
													SDF = RVLDOTPRODUCT3(pFeature->N, P) - pFeature->d;

													eSDF = pSample->SDF - SDF;

													eSDF = RVLABS(eSDF);

													if (eSDF < mineSDF)
													{
														mineSDF = eSDF;

														pSample->iFeature = iFeature;
													}
												}
											}

											pPtIdx = pPtIdx->pNext;
										}
									}

							sampleArray.n++;
						} // if (iClosestPt >= 0)
					}	  // if (voxelDistance == sampleVoxelDistance)
				}		  // if (pVoxel->voxelDistance > voxelDistance)
			}			  // if (i_ >= 0 && i_ < volume.a && j_ >= 0 && j_ < volume.b && k_ >= 0 && k_ < volume.c)
		}				  // for (l = 0; l < 6; l++)
	}					  // while (pPut > pFetch)

	delete[] PtMem;
	delete[] RGBuff;
	delete[] zeroDistanceVoxelArray.Element;
}

void SurfelGraph::SampleMeshDistanceFunction(
	Mesh *pMesh,
	float *R,
	float *t,
	int nSamplePts,
	float voxelSize,
	int sampleVoxelDistance,
	Array<MESH::Sample> &sampleArray)
{
	sampleArray.n = 2 * nSamplePts;

	RVL_DELETE_ARRAY(sampleArray.Element);

	sampleArray.Element = new MESH::Sample[2 * nSamplePts];

	SampleMesh(pMesh, R, t, sampleArray);

	Array<MESH::Sample> sampleArray_;
	Array3D<Voxel> volume;
	float P0[3];
	Box<float> boundingBox;

	RVL::SampleMeshDistanceFunction(pMesh, this, voxelSize, sampleVoxelDistance, volume, P0, sampleArray_, boundingBox);

	delete[] volume.Element;

	Array<int> iPtArray;

	iPtArray.n = RVLMIN(nSamplePts, sampleArray_.n);

	RandomIndices(iPtArray);

	int iSample;
	MESH::Sample *pSample, *pSample_;

	for (iSample = 0; iSample < iPtArray.n; iSample++)
	{
		pSample = sampleArray_.Element + iPtArray.Element[iSample];

		pSample_ = sampleArray.Element + nSamplePts + iSample;

		RVLTRANSF3(pSample->P, R, t, pSample_->P);

		pSample_->SDF = pSample->SDF;
	}

	//// Sample visualization

	// Visualizer visualizer;

	// visualizer.Create();

	// DisplaySampledMesh(&visualizer, volume, P0, voxelSize);

	// unsigned char color[] = { 0, 128, 255 };

	// visualizer.DisplayPointSet<float, MESH::Sample>(sampleArray_, color, 6.0f);

	// visualizer.Run();

	delete[] sampleArray_.Element;
}

void SurfelGraph::SampleSurfelSet(
	Mesh *pMesh,
	Array<int> iSurfelArray,
	Array<int> iVertexArray,
	Camera camera,
	SURFEL::SceneSamples &sceneSamples,
	bool bExternalSamples,
	bool bCenter)
{
	// Parameters.

	int imageNeighborhood = 7;
	float ROIBorderSize = 10.0f;
	int nSamplesPerMaxROISize = 32;

	//

	Rect<float> ROI;

	GetDepthImageROI(iVertexArray, camera, ROI);

	Rect<float> cameraWin;

	cameraWin.minx = 0.0f;
	cameraWin.maxx = (float)(camera.w - 1);
	cameraWin.miny = 0.0f;
	cameraWin.maxy = (float)(camera.h - 1);

	SampleRect<float>(&ROI, ROIBorderSize, cameraWin, nSamplesPerMaxROISize, sceneSamples.imagePtArray, &(sceneSamples.PtIdxArray),
					  &(sceneSamples.w), &(sceneSamples.h));

	int halfImageNeighborhood = (imageNeighborhood - 1) / 2;

	Rect<int> cropWin;
	cropWin.minx = 0;
	cropWin.maxx = camera.w - 1;
	cropWin.miny = 0;
	cropWin.maxy = camera.h - 1;

	bool *bBelongsToObject = new bool[NodeArray.n];

	memset(bBelongsToObject, 0, NodeArray.n * sizeof(bool));

	int i;

	for (i = 0; i < iSurfelArray.n; i++)
		bBelongsToObject[iSurfelArray.Element[i]] = true;

	sceneSamples.PtArray.Element = new float *[sceneSamples.imagePtArray.h];

	sceneSamples.PtArray.n = sceneSamples.imagePtArray.h;

	sceneSamples.g = new float[sceneSamples.imagePtArray.h];

	sceneSamples.SDF = new float[sceneSamples.imagePtArray.h];

	sceneSamples.PMem = new float[3 * sceneSamples.imagePtArray.h];

	sceneSamples.status = new uchar[sceneSamples.imagePtArray.h];

	float *P__ = sceneSamples.PMem;

	float *P, *P0;
	float *m;
	int u, v;
	int iPt;
	int iSurfel;
	int u_, v_;
	float dist;
	Point *pPt;
	bool bPtWithDepth;
	Rect<int> neighborhood;
	bool bExternalSample;

	for (i = 0; i < sceneSamples.imagePtArray.h; i++, P__ += 3)
	{
		m = sceneSamples.imagePtArray.Element + sceneSamples.imagePtArray.w * i;

		u = (int)round(m[0]);
		v = (int)round(m[1]);

		iPt = u + v * pMesh->width;

		iSurfel = surfelMap[iPt];

		pPt = pMesh->NodeArray.Element + iPt;

		if (iSurfel < 0 || iSurfel >= NodeArray.n)
		{
			bPtWithDepth = false;

			if (pPt->bValid)
				// if (RVLDOTPRODUCT3(pPt->N, pPt->N) > 0.5f)
				bPtWithDepth = true;

			bExternalSample = false;

			if (bExternalSamples)
			{
				if (ExternalSample(pMesh, camera, cropWin, m, halfImageNeighborhood, bBelongsToObject, dist, P__))
				{
					sceneSamples.status[i] = 1;

					sceneSamples.g[i] = dist;

					sceneSamples.PtArray.Element[i] = P__;

					bExternalSample = true;
				}
			}

			if (!bExternalSample)
			{
				if (bPtWithDepth)
				{
					sceneSamples.status[i] = 2;

					P0 = pMesh->NodeArray.Element[iPt].P;

					RVLCOPY3VECTOR(P0, P__);

					sceneSamples.g[i] = 0.0f;
				}
				else
				{
					sceneSamples.status[i] = 0;

					RVLNULL3VECTOR(P__);
				}

				sceneSamples.PtArray.Element[i] = P__;
			}

			continue;
		} // if (iSurfel < 0 || iSurfel >= NodeArray.n)

		P0 = pMesh->NodeArray.Element[iPt].P;

		RVLCOPY3VECTOR(P0, P__);

		sceneSamples.PtArray.Element[i] = P__;

		if (bBelongsToObject[iSurfel])
		{
			sceneSamples.status[i] = 1;

			sceneSamples.g[i] = 0.0f;
		}
		else
		{
			sceneSamples.status[i] = 2;

			iPt = u + v * pMesh->width;

			iSurfel = surfelMap[iPt];

			if (NodeArray.Element[iSurfel].bEdge)
			{
				RVLNEIGHBORHOOD(u, v, 1, cropWin.minx, cropWin.maxx, cropWin.miny, cropWin.maxy, neighborhood.minx, neighborhood.maxx, neighborhood.miny, neighborhood.maxy);

				for (v_ = neighborhood.miny; v_ <= neighborhood.maxy; v_++)
					for (u_ = neighborhood.minx; u_ <= neighborhood.maxx; u_++)
					{
						iPt = u_ + v_ * pMesh->width;

						iSurfel = surfelMap[iPt];

						if (iSurfel >= 0 && iSurfel < NodeArray.n)
						{
							if (bBelongsToObject[iSurfel])
							{
								sceneSamples.status[i] = 1;

								P0 = pMesh->NodeArray.Element[iPt].P;

								RVLCOPY3VECTOR(P0, P__);

								sceneSamples.PtArray.Element[i] = P__;

								break;
							}
						}
					}
			}

			if (bExternalSamples)
			{
				if (ExternalSample(pMesh, camera, cropWin, m, halfImageNeighborhood, bBelongsToObject, dist, P__))
				{
					sceneSamples.status[i] = 1;

					sceneSamples.g[i] = dist;

					sceneSamples.PtArray.Element[i] = P__;
				}
				else
				{
					sceneSamples.status[i] = 2;

					sceneSamples.g[i] = 0.0f;

					P0 = pMesh->NodeArray.Element[iPt].P;

					RVLCOPY3VECTOR(P0, P__);

					sceneSamples.PtArray.Element[i] = P__;
				}
			}
			else
				sceneSamples.g[i] = 0.0f;
		}
	}

	delete[] bBelongsToObject;

	// Compute centroid of scene points.

	RVLNULL3VECTOR(sceneSamples.Pc);

	int nObjectPts = 0;

	for (i = 0; i < sceneSamples.PtArray.n; i++)
	{
		if (sceneSamples.status[i] == 1)
		{
			P = sceneSamples.PtArray.Element[i];

			RVLSUM3VECTORS(sceneSamples.Pc, P, sceneSamples.Pc);

			nObjectPts++;
		}
	}

	float fTmp = (float)nObjectPts;

	RVLSCALE3VECTOR2(sceneSamples.Pc, fTmp, sceneSamples.Pc);

	// Center PtArray in Pc.

	if (bCenter)
	{
		for (i = 0; i < sceneSamples.PtArray.n; i++)
		{
			P = sceneSamples.PtArray.Element[i];

			RVLDIF3VECTORS(P, sceneSamples.Pc, P);
		}
	}

	sceneSamples.bCentered = bCenter;

	sceneSamples.PGnd = NULL;
}

bool SurfelGraph::ExternalSample(
	Mesh *pMesh,
	Camera camera,
	Rect<int> cropWin,
	float *m,
	int halfImageNeighborhood,
	bool *bBelongsToObject,
	float &distOut,
	float *POut)
{
	float Ray[3];

	RVLSET3VECTOR(Ray, ((m[0] - camera.uc) / camera.fu), ((m[1] - camera.vc) / camera.fv), 1.0f);

	float fTmp = RVLDOTPRODUCT3(Ray, Ray);

	int u = (int)round(m[0]);
	int v = (int)round(m[1]);

	Rect<int> neighborhood;

	neighborhood.minx = u - halfImageNeighborhood;
	neighborhood.maxx = u + halfImageNeighborhood;
	neighborhood.miny = v - halfImageNeighborhood;
	neighborhood.maxy = v + halfImageNeighborhood;

	CropRect<int>(neighborhood, cropWin);

	bool bFirst = true;

	int u_, v_, iPt, iSurfel;
	float s, sClosest, dist, minDist;
	float P_[3], dP[3];
	float *P;

	for (v_ = neighborhood.miny; v_ <= neighborhood.maxy; v_++)
		for (u_ = neighborhood.minx; u_ <= neighborhood.maxx; u_++)
		{
			iPt = u_ + v_ * pMesh->width;

			iSurfel = surfelMap[iPt];

			if (iSurfel < 0 || iSurfel >= NodeArray.n)
				continue;

			if (!bBelongsToObject[iSurfel])
				continue;

			P = pMesh->NodeArray.Element[iPt].P;

			s = RVLDOTPRODUCT3(Ray, P) / fTmp;

			RVLSCALE3VECTOR(Ray, s, P_);

			RVLDIF3VECTORS(P, P_, dP);

			dist = RVLDOTPRODUCT3(dP, dP);

			if (bFirst)
			{
				minDist = dist;

				sClosest = s;

				bFirst = false;
			}
			else if (dist < minDist)
			{
				minDist = dist;

				sClosest = s;
			}
		}

	if (bFirst)
		return false;

	RVLSCALE3VECTOR(Ray, sClosest, POut);

	distOut = sqrt(minDist);
}

void SurfelGraph::GetPoints(
	Array<int> iSurfelArray,
	Array<int> &PtArray)
{
	int *piPt = PtArray.Element;

	int iSurfel, iiSurfel;
	Surfel *pSurfel;
	QLIST::Index2 *pPtIdx;

	for (iiSurfel = 0; iiSurfel < iSurfelArray.n; iiSurfel++)
	{
		iSurfel = iSurfelArray.Element[iiSurfel];

		pSurfel = NodeArray.Element + iSurfel;

		// if (pSurfel->bEdge)
		//	continue;

		pPtIdx = pSurfel->PtList.pFirst;

		while (pPtIdx)
		{
			*(piPt++) = pPtIdx->Idx;

			pPtIdx = pPtIdx->pNext;
		}
	}

	PtArray.n = piPt - PtArray.Element;
}

void SurfelGraph::GetVertices(Array2D<float> &vertices)
{
	int iVertex;
	float *P, *P_;

	for (iVertex = 0; iVertex < vertexArray.n; iVertex++)
	{
		P = vertexArray.Element[iVertex]->P;

		P_ = vertices.Element + 3 * iVertex;

		RVLCOPY3VECTOR(P, P_);
	}

	vertices.h = vertexArray.n;
}

void SurfelGraph::GetVertices(
	Array<int> iVertexArray,
	Array2D<float> &vertices)
{
	int i;
	float *P, *P_;

	for (i = 0; i < iVertexArray.n; i++)
	{
		P = vertexArray.Element[iVertexArray.Element[i]]->P;

		P_ = vertices.Element + 3 * i;

		RVLCOPY3VECTOR(P, P_);
	}

	vertices.h = iVertexArray.n;
}

void SurfelGraph::GetVertices(
	int iSurfel,
	Array2D<float> &vertices)
{
	QList<QLIST::Index> *pSurfelVertexList = surfelVertexList.Element + iSurfel;

	vertices.h = 0;

	float *P, *P_;

	QLIST::Index *pVertexIdx = pSurfelVertexList->pFirst;

	while (pVertexIdx)
	{
		P = vertexArray.Element[pVertexIdx->Idx]->P;

		P_ = vertices.Element + 3 * vertices.h;

		RVLCOPY3VECTOR(P, P_);

		vertices.h++;

		pVertexIdx = pVertexIdx->pNext;
	}
}

int SurfelGraph::SupportSize(Array<int> iSurfelArray)
{
	int support = 0;

	int i;

	for (i = 0; i < iSurfelArray.n; i++)
		support += NodeArray.Element[iSurfelArray.Element[i]].size;

	return support;
}

bool SurfelGraph::InVOI(
	Array<int> iVertexArray,
	float *RSG,
	float *tSG,
	float r)
{
	float r2 = r * r;

	int i;
	SURFEL::Vertex *pVertex;
	float P[3];

	for (i = 0; i < iVertexArray.n; i++)
	{
		pVertex = vertexArray.Element[iVertexArray.Element[i]];

		RVLTRANSF3(pVertex->P, RSG, tSG, P);

		if (P[0] * P[0] + P[1] * P[1] > r2)
			break;
	}

	return (i >= iVertexArray.n);
}

void SurfelGraph::SurfelsInVOI(
	float *RSG,
	float *tSG,
	float r)
{
	float r2 = r * r;

	int iSurfel;
	Surfel *pSurfel;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *pVertexIdx;
	SURFEL::Vertex *pVertex;
	float P[3];

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		pVertexIdx = pSurfelVertexList->pFirst;

		while (pVertexIdx)
		{
			pVertex = vertexArray.Element[pVertexIdx->Idx];

			RVLTRANSF3(pVertex->P, RSG, tSG, P);

			if (P[0] * P[0] + P[1] * P[1] > r2)
				break;

			pVertexIdx = pVertexIdx->pNext;
		}

		if (pVertexIdx == NULL)
		{
			pSurfel = NodeArray.Element + iSurfel;

			pSurfel->flags |= RVLSURFEL_FLAG_VOI;
		}
	}
}

// Intersection of line segment (x11, y11) - (x12, y12) with line segment (x21, y21) - (x22, y22).
// If the intersection exists bIntersection is true and the intersection is (x, y).
// Auxiliary variables: dx1, dy1, dx2, dy2, det, s

#define RVL2DLINE_SEGMENT_INTERSECTION2(x11, y11, x21, y21, dx1, dy1, dx2, dy2, bIntersection, x, y, det, s1, s2) \
	{                                                                                                             \
		det = dx2 * dy1 - dx1 * dy2;                                                                              \
		if (det < 1e-6 && det > -1e-6)                                                                            \
			bIntersection = false;                                                                                \
		else                                                                                                      \
		{                                                                                                         \
			bIntersection = true;                                                                                 \
			s1 = (-dy2 * (x21 - x11) + dx2 * (y21 - y11)) / det;                                                  \
			s2 = (-dy1 * (x21 - x11) + dx1 * (y21 - y11)) / det;                                                  \
			x = x11 + s1 * dx1;                                                                                   \
			y = y11 + s1 * dy1;                                                                                   \
		}                                                                                                         \
	}

#define RVL2DLINE_SEGMENT_INTERSECTION(x11, y11, x12, y12, x21, y21, x22, y22, bIntersection, x, y, dx1, dy1, dx2, dy2, det, s1, s2) \
	{                                                                                                                                \
		dx1 = x12 - x11;                                                                                                             \
		dy1 = y12 - y11;                                                                                                             \
		dx2 = x22 - x21;                                                                                                             \
		dy2 = y22 - y21;                                                                                                             \
		RVL2DLINE_SEGMENT_INTERSECTION2(x11, y11, x21, y21, dx1, dy1, dx2, dy2, bIntersection, x, y, det, s1, s2);                   \
	}

void SurfelGraph::CreatePolygonGraph(float distThr)
{
	// Parameters.

	int maxnEdgesInGridCell = 100;

	// Constants.

	float cellSize = 2.0f * distThr;
	float dist2Thr = distThr * distThr;

	// polyEdges <- polygon edges

	int iSurfel;
	Surfel *pSurfel;
	RVL_DELETE_ARRAY(polyEdges.Element);
	polyEdges.Element = new SURFEL::PolyEdge[polygonVertices.size()];
	polyEdges.n = 0;
	SURFEL::PolyEdge *pPolyEdge = polyEdges.Element;
	int iEdge;
	int iPoly;
	Pair<int, int> *pPolygonVertexInterval;
	int iVertex;
	int i;
	float dP[3];
	float *P1, *P2;
	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfel = NodeArray.Element + iSurfel;

		for (iPoly = 0; iPoly < pSurfel->polygonVertexIntervals.n; iPoly++)
		{
			pPolygonVertexInterval = pSurfel->polygonVertexIntervals.Element + iPoly;
			iEdge = 0;
			for (iVertex = pPolygonVertexInterval->a; iVertex <= pPolygonVertexInterval->b; iVertex++, iEdge++, pPolyEdge++)
			{
				pPolyEdge->iSurfel = iSurfel;
				pPolyEdge->iPolygon = iPoly;
				pPolyEdge->iEdge = iEdge;
				pPolyEdge->iVertex[0] = iVertex;
				pPolyEdge->iVertex[1] = iVertex + 1;
				if (pPolyEdge->iVertex[1] > pPolygonVertexInterval->b)
					pPolyEdge->iVertex[1] = pPolygonVertexInterval->a;
				P1 = polygonVerticesS.Element[pPolyEdge->iVertex[0]].Element;
				P2 = polygonVerticesS.Element[pPolyEdge->iVertex[1]].Element;
				RVLDIF3VECTORS(P2, P1, dP);
				pPolyEdge->len = sqrt(RVLDOTPRODUCT3(dP, dP));
				RVLSCALE3VECTOR2(dP, pPolyEdge->len, pPolyEdge->V);
			}
		}
	}
	polyEdges.n = pPolyEdge - polyEdges.Element;

	// Scene bounding box.

	Box<float> sceneBBox;
	float *PS = polygonVerticesS.Element[0].Element;
	InitBoundingBox<float>(&sceneBBox, PS);
	for (iVertex = 1; iVertex < polygonVerticesS.n; iVertex++)
	{
		PS = polygonVerticesS.Element[iVertex].Element;
		UpdateBoundingBox<float>(&sceneBBox, PS);
	}

	// 3D grid.

	struct PolyEdgeIdx
	{
		int idx;
		int iCell;
		float *P;
		PolyEdgeIdx *pNext;
		PolyEdgeIdx **pPtrToThis;
	};

	Space3DGrid<PolyEdgeIdx, float> grid;
	int gridSize[3];
	gridSize[0] = (int)ceil((sceneBBox.maxx - sceneBBox.minx) / cellSize);
	gridSize[1] = (int)ceil((sceneBBox.maxy - sceneBBox.miny) / cellSize);
	gridSize[2] = (int)ceil((sceneBBox.maxz - sceneBBox.minz) / cellSize);
	int nGridCells = gridSize[0] * gridSize[1] * gridSize[2];
	grid.Create(gridSize[0], gridSize[1], gridSize[2], cellSize);
	grid.SetVolume(sceneBBox.minx, sceneBBox.miny, sceneBBox.minz);
	grid.SetMultipleInstancesOn();

	// Assign grid cells to polygon edges.

	float s;
	int cellIdx[3];
	int nextCellIdx[3];
	int iCell;
	// float signV[3];
	float iSignV[3];
	int iAxis;
	float d;
	float PS0[3];
	RVLSET3VECTOR(PS0, sceneBBox.minx, sceneBBox.miny, sceneBBox.minz);
	bool bAxis[3];
	float mins;
	int iNextCellSurfaceAxis;
	std::vector<int> edgeCells;
	int *firstEdgeCell = new int[polyEdges.n + 1];
	SURFEL::PolyEdge *pPolyEdge_;
	for (iEdge = 0; iEdge < polyEdges.n; iEdge++)
	{
		// if (iEdge == 15)
		//	int debug = 0;
		pPolyEdge = polyEdges.Element + iEdge;
		firstEdgeCell[iEdge] = edgeCells.size();
		PS = polygonVerticesS.Element[pPolyEdge->iVertex[0]].Element;
		grid.Cell(PS, cellIdx[0], cellIdx[1], cellIdx[2]);
		iCell = grid.Cell(cellIdx[0], cellIdx[1], cellIdx[2]);
		edgeCells.push_back(iCell);
		RVLCOPY3VECTOR(cellIdx, nextCellIdx);
		for (iAxis = 0; iAxis < 3; iAxis++)
		{
			if (pPolyEdge->V[iAxis] >= 0.0f)
			{
				nextCellIdx[iAxis]++;
				// signV[iAxis] = 1.0f;
				iSignV[iAxis] = 1;
				bAxis[iAxis] = (pPolyEdge->V[iAxis] >= 1e-6);
			}
			else
			{
				// signV[iAxis] = -1.0f;
				iSignV[iAxis] = -1;
				bAxis[iAxis] = (pPolyEdge->V[iAxis] <= -1e-6);
			}
		}
		do
		{
			mins = pPolyEdge->len;
			iNextCellSurfaceAxis = -1;
			for (iAxis = 0; iAxis < 3; iAxis++)
			{
				if (!bAxis[iAxis])
					continue;
				d = (float)nextCellIdx[iAxis] * cellSize + PS0[iAxis];
				s = (d - PS[iAxis]) / pPolyEdge->V[iAxis];
				if (s < mins)
				{
					mins = s;
					iNextCellSurfaceAxis = iAxis;
				}
			}
			if (iNextCellSurfaceAxis >= 0)
			{
				cellIdx[iNextCellSurfaceAxis] += iSignV[iNextCellSurfaceAxis];
				nextCellIdx[iNextCellSurfaceAxis] += iSignV[iNextCellSurfaceAxis];
				iCell = grid.Cell(cellIdx[0], cellIdx[1], cellIdx[2]);
				edgeCells.push_back(iCell);
			}
		} while (iNextCellSurfaceAxis >= 0);
	}
	firstEdgeCell[polyEdges.n] = edgeCells.size();

	// Fill 3D grid with edge indices.

	grid.SetDataMem(edgeCells.size());
	iEdge = 0;
	PolyEdgeIdx edgeIdx;
	int iNextEdge = 1;
	for (i = 0; i < edgeCells.size(); i++)
	{
		iCell = edgeCells[i];
		if (i >= firstEdgeCell[iNextEdge])
		{
			iEdge = iNextEdge;
			iNextEdge++;
		}
		edgeIdx.idx = iEdge;
		// if (iEdge >= polyEdges.n)
		//	int debug = 0;
		grid.AddData(iCell, edgeIdx);
	}

	// Only for debugging purpose!!!

	// std::vector<Pair<int, float>> sortCells;
	// sortCells.reserve(nGridCells);
	// Array<PolyEdgeIdx *> cellData;
	// cellData.Element = new PolyEdgeIdx *[edgeCells.size()];
	// Pair<int, float> cellDataSize;
	// for (iCell = 0; iCell < nGridCells; iCell++)
	//{
	//	cellData.n = 0;
	//	grid.GetData(iCell, cellData);
	//	if (cellData.n > 0)
	//	{
	//		cellDataSize.a = iCell;
	//		cellDataSize.b = (float)(cellData.n);
	//		sortCells.push_back(cellDataSize);
	//	}
	// }
	// std::sort(sortCells.begin(), sortCells.end(), IdxCostPairComparisonDesc);
	// delete[] cellData.Element;
	// printf("Grid cells of highest density:\n");
	// for (i = 0; i < 10; i++)
	//	printf("%d\n", (int)round(sortCells[i].b));

	// Remove short edges from high-density grid cells.

	Array<PolyEdgeIdx *> cellData;
	cellData.Element = new PolyEdgeIdx *[edgeCells.size()];
	std::vector<Pair<int, float>> sortedEdges;
	sortedEdges.reserve(polyEdges.n);
	Pair<int, float> edgeLen;
	float minLen;
	PolyEdgeIdx *pPolyEdgeIdx;
	for (iCell = 0; iCell < nGridCells; iCell++)
	{
		cellData.n = 0;
		sortedEdges.clear();
		grid.GetData(iCell, cellData);
		if (cellData.n > maxnEdgesInGridCell)
		{
			for (i = 0; i < cellData.n; i++)
			{
				iEdge = cellData.Element[i]->idx;
				pPolyEdge = polyEdges.Element + iEdge;
				edgeLen.a = iEdge;
				edgeLen.b = pPolyEdge->len;
				sortedEdges.push_back(edgeLen);
			}
			std::sort(sortedEdges.begin(), sortedEdges.end(), IdxCostPairComparisonDesc);
			minLen = sortedEdges[maxnEdgesInGridCell - 1].b;
			for (i = 0; i < cellData.n; i++)
			{
				pPolyEdgeIdx = cellData.Element[i];
				iEdge = pPolyEdgeIdx->idx;
				pPolyEdge = polyEdges.Element + iEdge;
				if (pPolyEdge->len < minLen)
					grid.RemoveData(pPolyEdgeIdx);
			}
		}
	}
	delete[] cellData.Element;

	// Edge neighborhood relations.

	bool *bNeighbor = new bool[polyEdges.n];
	memset(bNeighbor, 0, polyEdges.n * sizeof(bool));
	Array<PolyEdgeIdx *> localNeighbors;
	Array<int> neighbors;
	neighbors.Element = new int[polyEdges.n];
	neighbors.n = 0;
	int j;
	int iEdge_;
	iEdge = 0;
	iNextEdge = 1;
	pPolyEdge = polyEdges.Element;
	bNeighbor[0] = true;
	polygonEdgeGraphEdges.clear();
	float BA[3], DC[3], AC[3], BC[3], DCSqrMag, inPlaneA[3], inPlaneB[3], inPlaneBA[3];
	float fTmp;
	float V3Tmp[3];
	float *P11, *P12, *P21, *P22;
	float P1_[3], P2_[3];
	float *P;
	float dist2;
	SURFEL::PolyEdgeGraphEdge polyEdgeGraphEdge;
	int nEdgeRelations = 0;
	float s_;
	for (i = 0; i < edgeCells.size(); i++)
	{
		iCell = edgeCells[i];
		if (i >= firstEdgeCell[iNextEdge])
		{
			// if (iEdge == 148)
			//	int debug = 0;
			for (j = 0; j < neighbors.n; j++)
			{
				iEdge_ = neighbors.Element[j];
				pPolyEdge_ = polyEdges.Element + iEdge_;
				float *P11 = polygonVerticesS.Element[pPolyEdge->iVertex[0]].Element;
				float *P12 = polygonVerticesS.Element[pPolyEdge->iVertex[1]].Element;
				float *P21 = polygonVerticesS.Element[pPolyEdge_->iVertex[0]].Element;
				float *P22 = polygonVerticesS.Element[pPolyEdge_->iVertex[1]].Element;
				RVL3DLINE_SEGMENTS_CLOSEST_POINTS(P11, P12, P21, P22, P1_, P2_, s, s_, BA, DC, AC, BC, DCSqrMag, inPlaneA, inPlaneB, inPlaneBA, fTmp, V3Tmp);
				RVLDIF3VECTORS(P2_, P1_, V3Tmp);
				dist2 = RVLDOTPRODUCT3(V3Tmp, V3Tmp);
				if (dist2 <= dist2Thr)
				{
					polyEdgeGraphEdge.iVertex[0] = iEdge;
					polyEdgeGraphEdge.iVertex[1] = iEdge_;
					P = polyEdgeGraphEdge.P[0];
					RVLCOPY3VECTOR(P1_, P);
					P = polyEdgeGraphEdge.P[1];
					RVLCOPY3VECTOR(P2_, P);
					polyEdgeGraphEdge.dist2 = dist2;
					polyEdgeGraphEdge.idx = polygonEdgeGraphEdges.size();
					polygonEdgeGraphEdges.push_back(polyEdgeGraphEdge);
				}
				// if (iEdge_ >= polyEdges.n)
				//	int debug = 0;
				bNeighbor[iEdge_] = false;
				nEdgeRelations++;
				if (nEdgeRelations % 1000000 == 0)
					printf(".");
			}
			neighbors.n = 0;
			// if (iEdge >= polyEdges.n)
			//	int debug = 0;
			bNeighbor[iEdge] = false;
			iEdge = iNextEdge;
			pPolyEdge = polyEdges.Element + iEdge;
			iNextEdge++;
			// if (iEdge >= polyEdges.n)
			//	int debug = 0;
			bNeighbor[iEdge] = true;
			// if (iEdge == 148)
			//	int debug = 0;
		}
		grid.Neighbors(iCell, localNeighbors);
		for (j = 0; j < localNeighbors.n; j++)
		{
			iEdge_ = localNeighbors.Element[j]->idx;
			// if (iEdge_ >= polyEdges.n)
			//	int debug = 0;
			if (bNeighbor[iEdge_])
				continue;
			if (iEdge_ <= iEdge)
				continue;
			pPolyEdge_ = polyEdges.Element + iEdge_;
			if (pPolyEdge_->iSurfel == pPolyEdge->iSurfel)
				continue;
			pPolyEdge = polyEdges.Element + iEdge;
			bNeighbor[iEdge_] = true;
			neighbors.Element[neighbors.n++] = iEdge_;
		}
	}
	if (nEdgeRelations >= 1000000)
		printf("\n");

	// Create polygon edge graph.

	polyEdgeGraph.NodeArray.n = polyEdges.n;
	RVL_DELETE_ARRAY(polyEdgeGraph.NodeMem);
	polyEdgeGraph.NodeMem = new GRAPH::Node_<GRAPH::EdgePtr<SURFEL::PolyEdgeGraphEdge>>[polyEdgeGraph.NodeArray.n];
	polyEdgeGraph.NodeArray.Element = polyEdgeGraph.NodeMem;
	GRAPH::Node_<GRAPH::EdgePtr<SURFEL::PolyEdgeGraphEdge>> *pPolyEdgeGraphNode = polyEdgeGraph.NodeArray.Element;
	QList<GRAPH::EdgePtr<SURFEL::PolyEdgeGraphEdge>> *pEdgeList;
	for (iEdge = 0; iEdge < polyEdges.n; iEdge++, pPolyEdgeGraphNode++)
	{
		pPolyEdgeGraphNode->idx = iEdge;
		pEdgeList = &(pPolyEdgeGraphNode->EdgeList);
		RVLQLIST_INIT(pEdgeList);
	}
	int iEdgeGraphEdge;
	SURFEL::PolyEdgeGraphEdge *pPolyEdgeGraphEdge = polygonEdgeGraphEdges.data();
	RVL_DELETE_ARRAY(polyEdgeGraph.EdgePtrMem);
	polyEdgeGraph.EdgePtrMem = new GRAPH::EdgePtr<SURFEL::PolyEdgeGraphEdge>[2 * polygonEdgeGraphEdges.size()];
	GRAPH::EdgePtr<SURFEL::PolyEdgeGraphEdge> *pPolyEdgeGraphEdgePtr = polyEdgeGraph.EdgePtrMem;
	for (iEdgeGraphEdge = 0; iEdgeGraphEdge < polygonEdgeGraphEdges.size(); iEdgeGraphEdge++, pPolyEdgeGraphEdge++, pPolyEdgeGraphEdgePtr += 2)
		ConnectNodes<GRAPH::Node_<GRAPH::EdgePtr<SURFEL::PolyEdgeGraphEdge>>, SURFEL::PolyEdgeGraphEdge, GRAPH::EdgePtr<SURFEL::PolyEdgeGraphEdge>>(polyEdgeGraph.NodeArray, pPolyEdgeGraphEdge, pPolyEdgeGraphEdgePtr);

	// Create surfel graph.

	SURFEL::PolyEdgeGraphEdge **surfelGraphEdgePtr = new SURFEL::PolyEdgeGraphEdge *[NodeArray.n];
	memset(surfelGraphEdgePtr, 0, NodeArray.n * sizeof(SURFEL::PolyEdgeGraphEdge *));
	Array<int> neighborSurfels;
	neighborSurfels.Element = new int[NodeArray.n];
	RVL_DELETE_ARRAY(surfelGraph2.NodeMem);
	surfelGraph2.NodeMem = new GRAPH::Node[NodeArray.n];
	GRAPH::Node *pSurfelGraphNode = surfelGraph2.NodeMem;
	surfelGraph2.NodeArray.Element = surfelGraph2.NodeMem;
	surfelGraph2.NodeArray.n = NodeArray.n;
	QList<GRAPH::EdgePtr<GRAPH::Edge>> *pSurfelGraphEdgeList;
	RVL_DELETE_ARRAY(surfelGraph2.EdgeMem);
	surfelGraph2.EdgeMem = new GRAPH::Edge[polygonEdgeGraphEdges.size()];
	surfelGraph2.EdgeArray.Element = surfelGraph2.EdgeMem;
	GRAPH::Edge *pSurfelGraphEdge = surfelGraph2.EdgeMem;
	int iSurfel_;
	SURFEL::PolyEdgeGraphEdge *pShortestPolyEdgeGraphEdge;
	pPolyEdgeGraphNode = polyEdgeGraph.NodeArray.Element;
	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++, pSurfelGraphNode++)
	{
		pSurfel = NodeArray.Element + iSurfel;
		pSurfelGraphNode->idx = iSurfel;
		pSurfelGraphEdgeList = &(pSurfelGraphNode->EdgeList);
		RVLQLIST_INIT(pSurfelGraphEdgeList);
		neighborSurfels.n = 0;
		for (iPoly = 0; iPoly < pSurfel->polygonVertexIntervals.n; iPoly++)
		{
			pPolygonVertexInterval = pSurfel->polygonVertexIntervals.Element + iPoly;
			for (iEdge = pPolygonVertexInterval->a; iEdge <= pPolygonVertexInterval->b; iEdge++, pPolyEdgeGraphNode++)
			{
				pPolyEdgeGraphEdgePtr = pPolyEdgeGraphNode->EdgeList.pFirst;
				while (pPolyEdgeGraphEdgePtr)
				{
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iEdge, pPolyEdgeGraphEdgePtr, pPolyEdgeGraphEdge, iEdge_);
					pPolyEdge_ = polyEdges.Element + iEdge_;
					iSurfel_ = pPolyEdge_->iSurfel;
					if (iSurfel_ > iSurfel)
					{
						pShortestPolyEdgeGraphEdge = surfelGraphEdgePtr[iSurfel_];
						if (pShortestPolyEdgeGraphEdge == NULL)
						{
							neighborSurfels.Element[neighborSurfels.n++] = iSurfel_;
							surfelGraphEdgePtr[iSurfel_] = pPolyEdgeGraphEdge;
						}
						else if (pPolyEdgeGraphEdge->dist2 < pShortestPolyEdgeGraphEdge->dist2)
							surfelGraphEdgePtr[iSurfel_] = pPolyEdgeGraphEdge;
					}
					pPolyEdgeGraphEdgePtr = pPolyEdgeGraphEdgePtr->pNext;
				}
			}
		}
		for (i = 0; i < neighborSurfels.n; i++)
		{
			iSurfel_ = neighborSurfels.Element[i];
			pShortestPolyEdgeGraphEdge = surfelGraphEdgePtr[iSurfel_];
			if (pShortestPolyEdgeGraphEdge)
			{
				pSurfelGraphEdge->idx = pShortestPolyEdgeGraphEdge->idx;
				pSurfelGraphEdge->iVertex[0] = iSurfel;
				pSurfelGraphEdge->iVertex[1] = iSurfel_;
				pSurfelGraphEdge++;
				surfelGraphEdgePtr[iSurfel_] = NULL;
			}
		}
	}
	delete[] surfelGraphEdgePtr;
	delete[] neighborSurfels.Element;
	RVL_DELETE_ARRAY(surfelGraph2.EdgePtrMem);
	surfelGraph2.EdgeArray.n = pSurfelGraphEdge - surfelGraph2.EdgeArray.Element;
	surfelGraph2.EdgePtrMem = new GRAPH::EdgePtr<GRAPH::Edge>[2 * surfelGraph2.EdgeArray.n];
	pSurfelGraphEdge = surfelGraph2.EdgeArray.Element;
	GRAPH::EdgePtr<GRAPH::Edge> *pSurfelGraphEdgePtr = surfelGraph2.EdgePtrMem;
	for (i = 0; i < surfelGraph2.EdgeArray.n; i++, pSurfelGraphEdge++, pSurfelGraphEdgePtr += 2)
		ConnectNodes<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>>(surfelGraph2.NodeArray, pSurfelGraphEdge, pSurfelGraphEdgePtr);

	//

	delete[] firstEdgeCell;
	delete[] bNeighbor;
	delete[] neighbors.Element;
}

#ifdef RVLVTK
void RVL::DisplaySampledMesh(
	Visualizer *pVisualizer,
	Array3D<Voxel> volume,
	float *P0,
	float voxelSize)
{
	Array3D<float> f;

	f.a = volume.a;
	f.b = volume.b;
	f.c = volume.c;

	int nVoxels = volume.a * volume.b * volume.c;

	f.Element = new float[nVoxels];

	int iVoxel;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
		f.Element[iVoxel] = (volume.Element[iVoxel].voxelDistance > 0 ? 1.0f : -1.0f);

	vtkSmartPointer<vtkPolyData> polyData = DisplayIsoSurface(f, P0, voxelSize, 0.0f);

	// Create a mapper and actor.
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	pVisualizer->renderer->AddActor(actor);

	delete[] f.Element;
}
#endif

void SURFEL::DeleteSceneSamples(SURFEL::SceneSamples &sceneSamples)
{
	RVL_DELETE_ARRAY(sceneSamples.imagePtArray.Element);
	sceneSamples.imagePtArray.Element = NULL;
	RVL_DELETE_ARRAY(sceneSamples.PtIdxArray);
	sceneSamples.PtIdxArray = NULL;
	RVL_DELETE_ARRAY(sceneSamples.PtArray.Element);
	sceneSamples.PtArray.Element = NULL;
	RVL_DELETE_ARRAY(sceneSamples.PGnd);
	sceneSamples.PGnd = NULL;
	RVL_DELETE_ARRAY(sceneSamples.g);
	sceneSamples.g = NULL;
	RVL_DELETE_ARRAY(sceneSamples.SDF);
	sceneSamples.SDF = NULL;
	RVL_DELETE_ARRAY(sceneSamples.status);
	sceneSamples.status = NULL;
}

bool SURFEL::DefinePlaneInteractive(SurfelGraph *pSurfels,
									std::string keySym,
									int iSelectedSurfel)
{
	bool bNewPlane = false;

	Surfel *pSurfel;
	bool bRF;
	float axis[3];

	if (keySym == "0")
	{
		bNewPlane = true;
		bRF = true;
		RVLSET3VECTOR(axis, 1.0f, 0.0f, 0.0f);
	}
	else if (keySym == "1")
	{
		bNewPlane = true;
		bRF = true;
		RVLSET3VECTOR(axis, 0.0f, 1.0f, 0.0f);
	}
	else if (keySym == "2")
	{
		bNewPlane = true;
		bRF = true;
		RVLSET3VECTOR(axis, 0.0f, 0.0f, 1.0f);
	}
	else if (keySym == "3")
	{
		bNewPlane = true;
		bRF = true;
		RVLSET3VECTOR(axis, -1.0f, 0.0f, 0.0f);
	}
	else if (keySym == "4")
	{
		bNewPlane = true;
		bRF = true;
		RVLSET3VECTOR(axis, 0.0f, -1.0f, 0.0f);
	}
	else if (keySym == "5")
	{
		bNewPlane = true;
		bRF = true;
		RVLSET3VECTOR(axis, 0.0f, 0.0f, -1.0f);
	}
	else if (keySym == "6")
	{
		bNewPlane = true;
		bRF = false;
	}

	if (bNewPlane)
	{
		FILE *fp = fopen("C:\\RVL\\ExpRez\\planes.txt", "a");

		pSurfel = pSurfels->NodeArray.Element + iSelectedSurfel;

		if (bRF)
		{
			QList<QLIST::Index> *pVertexList = pSurfels->surfelVertexList.Element + iSelectedSurfel;

			QLIST::Index *pVertexIdx = pVertexList->pFirst;

			if (pVertexIdx)
			{
				float *P = pSurfels->vertexArray.Element[pVertexIdx->Idx]->P;

				float maxd = RVLDOTPRODUCT3(axis, P);

				float d;

				pVertexIdx = pVertexIdx->pNext;

				while (pVertexIdx)
				{
					P = pSurfels->vertexArray.Element[pVertexIdx->Idx]->P;

					d = RVLDOTPRODUCT3(axis, P);

					if (d > maxd)
						maxd = d;

					pVertexIdx = pVertexIdx->pNext;
				}

				fprintf(fp, "%f\t%f\t%f\t%f\n", axis[0], axis[1], axis[2], maxd);
			}
		}
		else
			fprintf(fp, "%f\t%f\t%f\t%f\n", pSurfel->N[0], pSurfel->N[1], pSurfel->N[2], pSurfel->d);

		fclose(fp);
	}

	return bNewPlane;
}

void SURFEL::UpdateNormalHull(
	Array<NormalHullElement> &NHull,
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

		if (fTmp < 1e-10)
			return;

		RVLSCALE3VECTOR2(Nh_, fTmp, Nh_);

		NHull.Element[0].snq = NHull.Element[1].snq = fTmp;

		N_ = NHull.Element[1].N;
		float *Nh__ = NHull.Element[1].Nh;

		RVLCOPY3VECTOR(N, N_);

		RVLNEGVECT3(Nh_, Nh__);

		NHull.n = 2;

		return;
	}

	NormalHullElement *pHullElement = NHull.Element + NHull.n - 1;

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

	if (fTmp < 1e-10)
		return;

	RVLSCALE3VECTOR2(Nh, fTmp, Nh_);

	pHullElement->snq = fTmp;

	pHullElement = NHull.Element + iEnd;

	N_ = pHullElement->N;

	RVLCROSSPRODUCT3(N_, N, Nh);

	fTmp = sqrt(RVLDOTPRODUCT3(Nh, Nh));

	if (fTmp < 1e-10)
		return;

	if (iEnd == (iStart + 1) % NHull.n) // Size of NHull should be increased.
	{
		if (iEnd > 0)
		{
			memmove(NHull.Element + iEnd + 1, NHull.Element + iEnd, (NHull.n - iEnd) * sizeof(NormalHullElement));

			iEnd++;
		}

		NHull.n++;
	}
	else if (iEnd > (iStart + 2) % NHull.n) // Size of NHull should be decreased.
	{
		if (iEnd > iStart)
		{
			memmove(NHull.Element + iStart + 2, NHull.Element + iEnd, (NHull.n - iEnd - 1) * sizeof(NormalHullElement));

			NHull.n -= (iEnd - iStart - 2);
		}
		else
		{
			if (iEnd > 0)
			{
				memmove(NHull.Element, NHull.Element + iEnd, (iStart - iEnd) * sizeof(NormalHullElement));

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
	pHullElement->snq = fTmp;
}

float SURFEL::DistanceFromNormalHull(
	Array<SURFEL::NormalHullElement> &NHull,
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
