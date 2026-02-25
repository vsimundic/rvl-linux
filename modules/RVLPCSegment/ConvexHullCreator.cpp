#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include "Util.h"
#include "Graph.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
#include "Mesh.h"
#include "Visualizer.h"
#include "AccuSphere.h"
#include "ConvexHullCreator.h"

using namespace RVL;

ConvexHullCreator::ConvexHullCreator()
{
	accuSphere.Create(3);
	maxnEnclosingPlanes = accuSphere.resolution * accuSphere.resolution * 6;
	planes.Element = NULL;
	fAccuSphereCHRange = 0.5f * (float)(accuSphere.resolution);
	accuSphereCHOffset = 0.5f - fAccuSphereCHRange;
	planeData.Element = NULL;
}


ConvexHullCreator::~ConvexHullCreator()
{
	Clear();
}

void ConvexHullCreator::Create(
	CRVLMem* pMemIn,
	int maxnPlanesIn)
{
	maxnPlanes = maxnPlanesIn + maxnEnclosingPlanes;
	planes.Element = new Plane[maxnPlanes];
	planeData.w = 3;
	planeData.Element = new float[planeData.w * maxnPlanes];
	pMem = pMemIn;
}

void ConvexHullCreator::Clear()
{
	RVL_DELETE_ARRAY(planes.Element);
	RVL_DELETE_ARRAY(planeData.Element);
}

void ConvexHullCreator::ConvexHull(
	Array<Plane> planesIn,
	Array<Vector3<float>> vertices,
	Mesh* pMesh)
{
	// Bounding box.

	int iVertex;
	Vector3<float>* pVertex;
	Box<float> bbox;
	pVertex = vertices.Element;
	InitBoundingBox<float>(&bbox, pVertex->Element);
	for (iVertex = 1; iVertex < vertices.n; iVertex++)
	{
		pVertex = vertices.Element + iVertex;
		UpdateBoundingBox<float>(&bbox, pVertex->Element);
	}

	// PC <- the center of the bbox.

	float PC[3];
	BoxCenter<float>(&bbox, PC);

	// Allocate memory for plane data.

	if (planesIn.n + maxnEnclosingPlanes > maxnPlanes)
	{
		maxnPlanes = planesIn.n + maxnEnclosingPlanes;
		RVL_DELETE_ARRAY(planes.Element);
		planes.Element = new Plane[maxnPlanes];
		RVL_DELETE_ARRAY(planeData.Element);
		planeData.Element = new float[planeData.w * maxnPlanes];
	}

	// Translate the planes from planesIn with respect to the center of the bbox and store them in planes.
	// Only the translated planes with d > 0 are store in planes.

	int iPlane;
	Plane* pPlaneIn;
	Plane * pPlane = planes.Element;
	float d;
	for (iPlane = 0; iPlane < planesIn.n; iPlane++)
	{
		pPlaneIn = planesIn.Element + iPlane;
		d = pPlaneIn->d - RVLDOTPRODUCT3(pPlaneIn->N, PC);
		if (d > 1e-7)
		{
			RVLCOPY3VECTOR(pPlaneIn->N, pPlane->N);
			pPlane->d = d;
			pPlane++;
		}
	}
	planes.n = pPlane - planes.Element;

	// Fill accuSphere with plane normals.

	accuSphere.Init(planes.n);
	pPlane = planes.Element;
	for (iPlane = 0; iPlane < planes.n; iPlane++, pPlane++)
		accuSphere.Vote(pPlane->N, 1.0f, iPlane);

	// Enclosing planes.

	int i, j;
	int iAxis0, iAccu;
	AccuSphereBin* pAccuBin;
	int iFarthestVertex;
	float maxd = 0.0f;	
	float fTmp;
	for (iAccu = 0; iAccu < 6; iAccu++)
	{
		for (i = 0; i < accuSphere.resolution; i++)
			for (j = 0; j < accuSphere.resolution; j++)
			{
				//if (iiConvex == 1 && iAccu == 2 && i == 1 && j == 1)
				//	int debug = 0;
				pAccuBin = accuSphere.GetBin(iAccu, i, j);
				if (pAccuBin->w == 0.0f)
				{
					RVLNULL3VECTOR(pPlane->N);
					iAxis0 = iAccu % 3;
					pPlane->N[iAxis0] = (iAccu < 3 ? fAccuSphereCHRange : -fAccuSphereCHRange);
					pPlane->N[(iAxis0 + 1) % 3] = (float)i + accuSphereCHOffset;
					pPlane->N[(iAxis0 + 2) % 3] = (float)j + accuSphereCHOffset;
					RVLNORM3(pPlane->N, fTmp);
					iFarthestVertex = -1;
					for(iVertex = 0; iVertex < vertices.n; iVertex++)
					{
						pVertex = vertices.Element + iVertex;
						d = RVLDOTPRODUCT3(pPlane->N, pVertex->Element);
						if (iFarthestVertex < 0 || d > maxd)
						{
							maxd = d;
							iFarthestVertex = iVertex;
						}
					}
					pPlane->d = maxd - RVLDOTPRODUCT3(pPlane->N, PC);
					pPlane++;
				}
			}
	}
	planes.n = pPlane - planes.Element;

	// convexHullH <- convex hull of planes as points
	
	planeData.h = planes.n;
	float* A = planeData.Element;
	Mesh convexHullH;
	for (iPlane = 0; iPlane < planes.n; iPlane++, A += 3)
	{
		pPlane = planes.Element + iPlane;
		RVLSCALE3VECTOR2(pPlane->N, pPlane->d, A);
	}
	RVLMEM_ALLOC_LOCAL_INIT(pMem)
	convexHullH.ConvexHull(planeData, pMem, false, false);
	RVLMEM_ALLOC_LOCAL_FREE(pMem);

	/// Compute pMesh as the dual polyhedron of convexHullH.

	// Compute the vertices of pMesh from the triangles of convexHullH.

	MeshEdgePtr* pEdgePtr;
	Point* pCHHVertex[3];
	pMesh->NodeMem = new Point[convexHullH.faces.n];
	pMesh->NodeArray.Element = pMesh->NodeMem;
	pMesh->EdgeArray.n = 0;
	Point* pCHVertex = pMesh->NodeArray.Element;
	int iCHHFace;
	MESH::Face* pCHHFace;
	int iCHHVertex;
	int triangleMem[3];
	Array<int> triangleVertices;
	triangleVertices.Element = triangleMem;
	double V1[3], V2[3];
	double P[3];
	double lfTmp;
	for (iCHHFace = 0; iCHHFace < convexHullH.faces.n; iCHHFace++)
	{
		pCHHFace = convexHullH.faces.Element[iCHHFace];
		pCHHFace->idx = iCHHFace;
		triangleVertices.n = 0;
		pEdgePtr = pCHHFace->pFirstEdgePtr;
		do
		{
			iCHHVertex = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);
			if (triangleVertices.n < 3)
				triangleVertices.Element[triangleVertices.n++] = iCHHVertex;
			pMesh->EdgeArray.n++;
			pEdgePtr = pEdgePtr->pNext;
			if (pEdgePtr == NULL)
				pEdgePtr = convexHullH.NodeArray.Element[iCHHVertex].EdgeList.pFirst;
			pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);
		} while (pEdgePtr != pCHHFace->pFirstEdgePtr);
		for (i = 0; i < 3; i++)
			pCHHVertex[i] = convexHullH.NodeArray.Element + triangleVertices.Element[i];
		RVLDIF3VECTORS(pCHHVertex[1]->P, pCHHVertex[0]->P, V1);
		RVLDIF3VECTORS(pCHHVertex[2]->P, pCHHVertex[1]->P, V2);
		RVLCROSSPRODUCT3(V1, V2, P);
		lfTmp = RVLDOTPRODUCT3(P, pCHHVertex[1]->P);
		RVLSCALE3VECTOR2(P, lfTmp, P);
		RVLSUM3VECTORS(P, PC, pCHVertex->P);
		pCHVertex++;
	}
	pMesh->NodeArray.n = pCHVertex - pMesh->NodeArray.Element;
	pMesh->EdgeArray.n /= 2;

	// Faces.

	RVL_DELETE_ARRAY(pMesh->faceMem);
	pMesh->faceMem = new MESH::Face[planes.n];
	RVL_DELETE_ARRAY(pMesh->faces.Element);
	pMesh->faces.Element = new MESH::Face *[planes.n];
	pMesh->faces.n = planes.n;
	MESH::Face* pFace;
	for (iPlane = 0; iPlane < planes.n; iPlane++)
	{
		pPlane = planes.Element + iPlane;
		pFace = pMesh->faceMem + iPlane;
		pFace->idx = iPlane;
		RVLCOPY3VECTOR(pPlane->N, pFace->N);
		pFace->d = pPlane->d + RVLDOTPRODUCT3(pPlane->N, PC);
		pMesh->faces.Element[iPlane] = pFace;
	}

	// Compute edges of pMesh from the edges of convexHullH.

	QList<MeshEdgePtr>* pEdgeList;
	int iCHVertex;
	for (iCHVertex = 0; iCHVertex < pMesh->NodeArray.n; iCHVertex++)
	{
		pCHVertex = pMesh->NodeArray.Element + iCHVertex;
		pEdgeList = &(pCHVertex->EdgeList);
		RVLQLIST_INIT(pEdgeList);
	}
	pMesh->EdgeMem = new MeshEdge[pMesh->EdgeArray.n];
	pMesh->EdgeArray.Element = pMesh->EdgeMem;
	MeshEdge* pCHEdge = pMesh->EdgeMem;
	pMesh->EdgePtrMem = new MeshEdgePtr[2 * pMesh->EdgeArray.n];
	MeshEdgePtr* pEdgePtr_ = pMesh->EdgePtrMem;
	int iCHHVertex_, iCHHVertex__;
	Point* pCHHVertex_;
	MeshEdge* pCHHEdge;
	float debug = 0.0f;
	for (i = 0; i < convexHullH.iValidVertices.n; i++)
	{
		iCHHVertex_ = convexHullH.iValidVertices.Element[i];
		pCHHVertex_ = convexHullH.NodeArray.Element + iCHHVertex_;
		pEdgePtr = pCHHVertex_->EdgeList.pFirst;
		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iCHHVertex_, pEdgePtr, pCHHEdge, iCHHVertex__);
			if (iCHHVertex_ < iCHHVertex__)
			{
				pCHEdge->idx = pCHEdge - pMesh->EdgeMem;
				pCHEdge->iVertex[0] = pCHHEdge->pFace[0]->idx;
				pCHEdge->iVertex[1] = pCHHEdge->pFace[1]->idx;
				pCHEdge->pFace[0] = pMesh->faces.Element[pCHHEdge->iVertex[0]];
				pCHEdge->pFace[1] = pMesh->faces.Element[pCHHEdge->iVertex[1]];
				ConnectNodes<Point, MeshEdge, MeshEdgePtr>(pMesh->NodeArray, pCHEdge, pEdgePtr_);
				// Only for debugging purpose!!!
				//for(int i = 0; i < 2; i++)
				//	for (int j = 0; j < 2; j++)
				//	{
				//		pCHVertex = pMesh->NodeArray.Element + pCHEdge->iVertex[i];
				//		float debug_ = RVLDOTPRODUCT3(pCHEdge->pFace[j]->N, pCHVertex->P) - pCHEdge->pFace[j]->d;
				//		debug_ = RVLABS(debug_);
				//		if (debug_ > debug)
				//			debug = debug_;
				//		if (debug_ > 0.0035f)
				//			int debug__ = 0;
				//	}
				//
				pCHEdge++;
				pEdgePtr_ += 2;
			}
			pEdgePtr = pEdgePtr->pNext;
		}
	}
	pMesh->EdgeArray.n = pCHEdge - pMesh->EdgeArray.Element;

	// Only for debugging purpose!!!

	//float e;
	//float maxe = 0.0f;
	//int iEdgeDebug, iDebug, jDebug;
	//for (int iEdge = 0; iEdge < pMesh->EdgeArray.n; iEdge++)
	//{
	//	pCHEdge = pMesh->EdgeArray.Element + iEdge;
	//	for (i = 0; i < 2; i++)
	//	{
	//		pCHVertex = pMesh->NodeArray.Element + pCHEdge->iVertex[i];
	//		for (j = 0; j < 2; j++)
	//		{
	//			pFace = pCHEdge->pFace[j];
	//			e = RVLDOTPRODUCT3(pFace->N, pCHVertex->P) - pFace->d;
	//			e = RVLABS(e);
	//			if (e > maxe)
	//			{
	//				maxe = e;
	//				iEdgeDebug = iEdge;
	//				iDebug = i;
	//				jDebug = j;
	//			}
	//		}
	//	}
	//}
	//int debug = 0;

	//
}
