//#include "stdafx.h"
//#include <pcl/io/pcd_io.h>
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include <Eigen\Eigenvalues>
//#include <pcl/common/common.h>
//#include <pcl/PolygonMesh.h>
//#include <pcl/surface/vtk_smoothing/vtk_utils.h>
//#include "PCLTools.h"
//#include "PCLMeshBuilder.h"
//#include "RGBDCamera.h"
#include "Mesh.h"

using namespace RVL;

Mesh::Mesh()
{
	normalEstimationRadius = 10.0f;
#ifdef RVLMESH_BOUNDARY_DEBUG
	debugState = 1;
#endif
}


Mesh::~Mesh()
{
}

// Loads an ordered mesh from a PLY file. 
// The definition of the ordered mesh is given in ARP3D.TR3.

void Mesh::LoadPolyDataFromPLY(char *PLYFileName)
{
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(PLYFileName);
	reader->Update();
	pPolygonData = reader->GetOutput();
}

//VIDOVIC
void Mesh::SavePolyDataToPLY(char *PLYFileName)
{
	char *sceneNoisedMeshFileName = new char[100];

	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();

	//create noised mesh filename
	strcpy(sceneNoisedMeshFileName, PLYFileName);
	strcpy(sceneNoisedMeshFileName + strlen(sceneNoisedMeshFileName) - strlen(".ply"), "_noised.ply");

	writer->SetFileName(sceneNoisedMeshFileName);
	
#if VTK_MAJOR_VERSION <= 5
	writer->SetInput(pPolygonData);
#else
	writer->SetInputData(pPolygonData);
#endif

	writer->Write();

	delete[] sceneNoisedMeshFileName;
}
//END VIDOVIC

bool Mesh::CreateOrderedMeshFromPolyData()
{
	int noPts = pPolygonData->GetNumberOfPoints();
	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pPolygonData->GetPoints()->GetData());
	if (pointData == NULL)
		return false;

	vtkSmartPointer<vtkFloatArray> normalPointData = normalPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("Normals"));
	if (normalPointData == NULL)
	{
		normalPointData = normalPointData->SafeDownCast(pPolygonData->GetPointData()->GetNormals());

		if (normalPointData == NULL)
			return false;
	}

	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("RGB"));
	if (rgbPointData == NULL)
	{
		rgbPointData = rgbPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("Colors"));

		if (rgbPointData)
			rgbPointData->SetName("RGB");
		else
			return false;
	}
		
	// Get vertices

	Clear();

	NodeMem = new Point[noPts]; // VIDOVIC

	//NodeArray.Element = new Point[noPts]; //VIDOVIC
	NodeArray.Element = NodeMem; //VIDOVIC

	NodeArray.n = noPts;

	Point *pPt = NodeArray.Element;

	QList<MeshEdgePtr> *pEdgeList;
	
	int iPt;
	unsigned char RGB[3];
	float fTmp;

	for (iPt = 0; iPt < noPts; iPt++, pPt++)
	{
		rgbPointData->GetTupleValue(iPt, RGB);
		RVLCONVTOINT3(RGB, pPt->RGB);
		pointData->GetTupleValue(iPt, pPt->P);
		normalPointData->GetTupleValue(iPt, pPt->N);
		RVLNORM3(pPt->N, fTmp);
	}

	// maxnPolygonVertices <- max. no. of vertices per polygon

	int nPolys = pPolygonData->GetPolys()->GetNumberOfCells();

	vtkSmartPointer<vtkCellArray> Polys = pPolygonData->GetPolys();

	int maxnPolygonVertices = 0;

	Polys->InitTraversal();

	vtkSmartPointer<vtkIdList> l = vtkSmartPointer<vtkIdList>::New();

	int nPts;
	int iPoly;

	for (iPoly = 0; iPoly < nPolys; iPoly++)
	{
		if (Polys->GetNextCell(l))
		{
			nPts = l->GetNumberOfIds();

			if (nPts > maxnPolygonVertices)
				maxnPolygonVertices = nPts;
		}
	}

	/// Get edges and polygons

	int maxnEdges = maxnPolygonVertices * nPolys;

	EdgeMem = new MeshEdge[maxnEdges]; // VIDOVIC

	//EdgeArray.Element = new MeshEdge[maxnEdges]; //VIDOVIC
	EdgeArray.Element = EdgeMem; //VIDOVIC

	MeshEdge *pEdge = EdgeArray.Element;

	Array<QList<QLIST::Index>> VertexEdgeListArray;	// Each element of this array is a list of edge indices corresponding to a mesh vertex. 
	
	VertexEdgeListArray.Element = new QList<QLIST::Index>[noPts];
	VertexEdgeListArray.n = noPts;

	QList<QLIST::Index> *pVertexEdgeList;

	RVLQLIST_ARRAY_INIT(VertexEdgeListArray, pVertexEdgeList);

	QLIST::Index *VertexEdgeMem = new QLIST::Index[2 * maxnEdges];

	QLIST::Index *pVertexEdgeIdx = VertexEdgeMem;

	int polyDataSize = maxnPolygonVertices + 1;

	int *PolyArray = new int[polyDataSize * nPolys];	// A matrix nPolys x polyDataSize. Each row represents one polygon.
														// The first element of each row represents the number of polygon vertices nPts.
														// The next nPts elements of a row are indices of the polygon vertices.
														// The remaining elements of the row are undefined.

	int *EdgePolyAssignmentArray = new int[4 * maxnEdges];	// A matrix EdgeArray.n x 4. Each row represents one edge.
																// The first two elements of i-th row are the indices of the polygons sharing the edge i.
																// The first of these two elements represents the index of the polygon on the left side of the edge,
																// while the second one represents the index of the polygon on the right side of the edge.
																// The other two elements of i-th row are the indices of the edge in the polygon edge list.
																// The third element is the index of the edge in the list of the polygon on the left side of the edge,
																// while the fourth one is the index of the edge in the list of the polygon on the right side of the edge.

	memset(EdgePolyAssignmentArray, 0xff, 4 * maxnEdges * sizeof(int));
	
	int *PolyEdgeAssignmentArray = new int[maxnEdges];	// A matrix nPolys x maxnPolygonVertices. Each row contains the indices of the edges of the corresponding polygon.
														// Only the first nPts elements of each row are defined, where nPts is the number of the polygon vertices.

	int *PolyEdge = PolyEdgeAssignmentArray;

	int edgeSide;

	int *EdgePoly;
	int *EdgePolyIdx;

	int iEdge = 0;

	Polys->InitTraversal();

	int iPt1, iPt2;
	int iEdge_;
	int *PolyVertex;
	int *PolyData;
	bool bAlreadyExists;
	MeshEdge *pEdge_;
	QLIST::Index *pVertexEdgeIdx_;

	for (iPoly = 0; iPoly < nPolys; iPoly++)
	{
		if (Polys->GetNextCell(l))
		{
			PolyData = PolyArray + polyDataSize * iPoly;

			nPts = l->GetNumberOfIds();

			PolyData[0] = nPts;

			PolyVertex = PolyData + 1;

			PolyEdge = PolyEdgeAssignmentArray + maxnPolygonVertices * iPoly;

			iPt1 = l->GetId(nPts - 1);

			for (iPt = 0; iPt < nPts; iPt++)
			{
				iPt2 = l->GetId(iPt);

				PolyVertex[iPt] = iPt2;

				// bAlreadyExists <- edge connecting the vertices iPt1 and iPt2 already exists in the list of VertexEdgeListArray[iPt1].

				bAlreadyExists = false;

				pVertexEdgeList = VertexEdgeListArray.Element + iPt1;

				pVertexEdgeIdx_ = pVertexEdgeList->pFirst;

				while (pVertexEdgeIdx_)
				{
					iEdge_ = pVertexEdgeIdx_->Idx;

					pEdge_ = EdgeArray.Element + iEdge_;

					if (bAlreadyExists = (pEdge_->iVertex[0] == iPt2))
						break;

					pVertexEdgeIdx_ = pVertexEdgeIdx_->pNext;
				}

				// 

				if (bAlreadyExists)
					edgeSide = 1; // Polygon l is on the right side of edge iEdge_.
				else
				{
					// Create a new edge and add it to EdgeArray. 
					// Add the index of the new edge to VertexEdgeListArray[iPt1] and VertexEdgeListArray[iPt2].

					pEdge->iVertex[0] = iPt1;
					pEdge->iVertex[1] = iPt2;
					pEdge->idx = iEdge;
					
					RVLQLIST_ADD_ENTRY(pVertexEdgeList, pVertexEdgeIdx);
					pVertexEdgeIdx->Idx = iEdge;
					pVertexEdgeIdx++;

					pVertexEdgeList = VertexEdgeListArray.Element + iPt2;
					RVLQLIST_ADD_ENTRY(pVertexEdgeList, pVertexEdgeIdx);
					pVertexEdgeIdx->Idx = iEdge;
					pVertexEdgeIdx++;

					edgeSide = 0; // Polygon l is on the left side of edge iEdge_.

					iEdge_ = iEdge;
					pEdge_ = pEdge;

					pEdge++;
					iEdge++;
				}
				
				// Store the edge index to PolyEdge.

				PolyEdge[iPt] = iEdge_;

				// EdgePoly <- four element vector corresponding to the edge iEdge_

				EdgePoly = EdgePolyAssignmentArray + 4 * iEdge_;
				EdgePolyIdx = EdgePoly + 2;

				// Store the index of the polygon l on the side edgeSide of the edge iEdge_ in the corresponding field of EdgePoly.

				EdgePoly[edgeSide] = iPoly;

				// Store the index of the edge in the polygon edge list of the polygon l in the corresponding field of EdgePoly. 

				EdgePolyIdx[edgeSide] = iPt;

				iPt1 = iPt2;
			}	// for each Poly vertex
		}
	}	// for each Poly

	EdgeArray.n = pEdge - EdgeArray.Element;

	// Remove invalid points.

	Array<int> iPtBuff1;

	iPtBuff1.Element = new int[noPts];

	Array<int> iPtBuff2;

	iPtBuff2.Element = new int[noPts];

	Array<int> *piPtBuffPut = &iPtBuff1;
	Array<int> *piPtBuffFetch = &iPtBuff2;

	for (iPt = 0; iPt < noPts; iPt++)
	{
		piPtBuffFetch->Element[iPt] = iPt;

		pPt = NodeArray.Element + iPt;

		pPt->bValid = true;
	}
		
	piPtBuffFetch->n = noPts;

	piPtBuffPut->n = 0;

	bool *bVisited = new bool[noPts];

	memset(bVisited, 0, noPts * sizeof(bool));

	int i, j, k;
	int nEdges;
	int nBoundaryEdges;
	Array<int> *piPtBuffTmp;
	int iPt_;
	Point *pPt_;
	int *EdgePoly_;

	while (piPtBuffFetch->n > 0)
	{
		for (i = 0; i < piPtBuffFetch->n; i++)
		{
			iPt = piPtBuffFetch->Element[i];

			//if (iPt == 45075)
			//	int debug = 0;

			pPt = NodeArray.Element + iPt;

			pVertexEdgeList = VertexEdgeListArray.Element + iPt;

			nEdges = nBoundaryEdges = 0;

			pVertexEdgeIdx = pVertexEdgeList->pFirst;

			while (pVertexEdgeIdx)
			{
				iEdge = pVertexEdgeIdx->Idx;

				EdgePoly = EdgePolyAssignmentArray + 4 * iEdge;

				if (EdgePoly[0] >= 0 || EdgePoly[1] >= 0)
				{
					if (EdgePoly[0] < 0 || EdgePoly[1] < 0)
						nBoundaryEdges++;

					nEdges++;
				}

				pVertexEdgeIdx = pVertexEdgeIdx->pNext;
			}

			// If there is less than two edges connected to vertex pPt, then don't generate the edge list for that vertex.

			if (nEdges < 2)
			{
				pPt->bValid = false;

				continue;
			}	

			// If there are more than two boundary edges connected to vertex pPt, then set position and normal of pPt to null vectors.
			// In that case, the edge list is empty.
			// The vertices with normal set to null vector should be rejected from any further processing, which effectively removes such vertices from the mesh.
			// By removing all vertices with more than two boundary edges from the mesh, the 2. property of the organized mesh is preserved 
			// (See the definition of the organized mesh in ARP3D.TR30).

			if (nBoundaryEdges <= 2)
				pPt->bBoundary = (nBoundaryEdges > 0);
			else
			{
				pPt->bValid = false;

				RVLNULL3VECTOR(pPt->P);
				RVLNULL3VECTOR(pPt->N);

				pVertexEdgeIdx = pVertexEdgeList->pFirst;

				while (pVertexEdgeIdx)
				{
					iEdge = pVertexEdgeIdx->Idx;

					pEdge = EdgeArray.Element + iEdge;

					EdgePoly = EdgePolyAssignmentArray + 4 * iEdge;

					for (j = 0; j < 2; j++)
					{
						iPoly = EdgePoly[j];

						if (iPoly >= 0)
						{
							//if (iPoly == 43984)
							//	int debug = 0;

							PolyData = PolyArray + polyDataSize * iPoly;

							nPts = PolyData[0];

							PolyEdge = PolyEdgeAssignmentArray + maxnPolygonVertices * iPoly;

							for (k = 0; k < nPts; k++)
							{
								iEdge_ = PolyEdge[k];

								EdgePoly_ = EdgePolyAssignmentArray + 4 * iEdge_;

								if (EdgePoly_[0] == iPoly)
									EdgePoly_[0] = -1;
								else
									EdgePoly_[1] = -1;
							}
						}
					}

					iPt_ = (pEdge->iVertex[0] == iPt ? pEdge->iVertex[1] : pEdge->iVertex[0]);

					if (!bVisited[iPt_])
					{
						pPt_ = NodeArray.Element + iPt_;

						if (pPt_->bValid)
						{
							bVisited[iPt_] = true;

							piPtBuffPut->Element[piPtBuffPut->n++] = iPt_;
						}
					}

					pVertexEdgeIdx = pVertexEdgeIdx->pNext;
				}	// for every edge ending in pPt
			}	// if (!pPt->bValid)
		}	// for every point in piPtBuffFetch

		for (i = 0; i < piPtBuffPut->n; i++)
			bVisited[piPtBuffPut->Element[i]] = false;

		piPtBuffTmp = piPtBuffFetch;
		piPtBuffFetch = piPtBuffPut;
		piPtBuffPut = piPtBuffTmp;

		piPtBuffPut->n = 0;
	}	// while (piPtBuffFetch->n > 0)

	delete[] iPtBuff1.Element;
	delete[] iPtBuff2.Element;
	delete[] bVisited;

	// Arrange edge lists of vertices according to the 2. property of Organized mesh (See the definition of organized mesh in ARP3D.TR3).

	EdgePtrMem = new MeshEdgePtr[2 * EdgeArray.n];

	MeshEdgePtr *pMeshEdgePtr = EdgePtrMem;

	//int watchdog;

	nBoundaryPts = 0;

	int iEdge0;

	for (iPt = 0; iPt < noPts; iPt++)
	{
		pPt = NodeArray.Element + iPt;

		pEdgeList = &(pPt->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		if (!pPt->bValid)
			continue;

		pVertexEdgeList = VertexEdgeListArray.Element + iPt;

		// Count edges connected to vertex pPt, i.e. the size of the edge list of pPt.
		// Count the boundary edges (See the definition of boundary edges in ARP3D.TR3).
		// If there are boundary edges connected to pPt, then iEdge0 <- the first edge connected to pPt in the CCW direction.

		nEdges = nBoundaryEdges = 0;		// only for debugging purpose!!!

		pVertexEdgeIdx = pVertexEdgeList->pFirst;

		while (pVertexEdgeIdx)
		{
			iEdge = pVertexEdgeIdx->Idx;

			pEdge = EdgeArray.Element + iEdge;

			EdgePoly = EdgePolyAssignmentArray + 4 * iEdge;

			if (EdgePoly[0] >= 0 || EdgePoly[1] >= 0)
			{
				if (EdgePoly[0] < 0)
				{
					nBoundaryEdges++;		// only for debugging purpose!!!

					if (pEdge->iVertex[1] == iPt)
						iEdge0 = iEdge;
				}
				else if (EdgePoly[1] < 0)
				{
					nBoundaryEdges++;		// only for debugging purpose!!!

					if (pEdge->iVertex[0] == iPt)
						iEdge0 = iEdge;
				}

				nEdges++;		// only for debugging purpose!!!
			}

			pVertexEdgeIdx = pVertexEdgeIdx->pNext;
		}

		if (nEdges < 2)
			int debug = 0;

		if (nBoundaryEdges > 2)
			int debug = 0;

		// If no boundary edge is connected to vertex pPt, then the first edge in its edge list can be any edge connected to it.

		if (pPt->bBoundary)
			nBoundaryPts++;
		else
			iEdge0 = pVertexEdgeList->pFirst->Idx;

		//watchdog = 0;

		iEdge = iEdge0;
		
		do
		{
			pEdge = EdgeArray.Element + iEdge;

			// Connect pEdge to the edge list of vertex pPt using the connector pMeshEdgePtr.

			RVLQLIST_ADD_ENTRY(pEdgeList, pMeshEdgePtr);

			pMeshEdgePtr->pEdge = pEdge;

			EdgePoly = EdgePolyAssignmentArray + 4 * iEdge;
			EdgePolyIdx = EdgePoly + 2;

			edgeSide = (pEdge->iVertex[0] == iPt ? 0 : 1);

			pEdge->pVertexEdgePtr[edgeSide] = pMeshEdgePtr;

			pMeshEdgePtr++;

			// iPoly <- index of the polygon in CCW direction from the edge pEdge w.r.t. vertex pPt.

			iPoly = EdgePoly[edgeSide];	

			// if there is no polygon in CCW direction from the edge pEdge w.r.t. vertex pPt, then the edge list is completed.

			if (iPoly < 0)
				break;

			// iEdge <- the preceding edge of iEdge in the edge list of the polygon iPoly.

			PolyData = PolyArray + polyDataSize * iPoly;

			nPts = PolyData[0];

			PolyEdge = PolyEdgeAssignmentArray + maxnPolygonVertices * iPoly;

			iEdge = PolyEdge[(EdgePolyIdx[edgeSide] + nPts - 1) % nPts];

			//// debug

			//MeshEdge *pEdge_ = EdgeArray.Element + iEdge;

			//int edgeSide_ = (pEdge_->iVertex[0] == iPt ? 0 : 1);

			//Point Pt1 = NodeArray.Element[iPt];
			//Point Pt2 = NodeArray.Element[pEdge->iVertex[1 - edgeSide]];
			//Point Pt3 = NodeArray.Element[pEdge_->iVertex[1 - edgeSide_]];

			//float V1[3], V2[3];

			//RVLDIF3VECTORS(Pt2.P, Pt1.P, V1);
			//RVLDIF3VECTORS(Pt3.P, Pt1.P, V2);

			//float Z[3];

			//RVLCROSSPRODUCT3(V1, V2, Z);

			//float a = RVLDOTPRODUCT3(Pt1.P, Z);

			//if (a > 0.0)
			//	int debug = 0;

			///////

			//watchdog++;

			//if (watchdog > 8)
			//	break;
		} while (iEdge != iEdge0);
	}

	/////

	//MeshEdge* pEdgeArrayEnd = EdgeArray.Element + EdgeArray.n;

	//int i;

	//for (pEdge = EdgeArray.Element; pEdge < pEdgeArrayEnd; pEdge++)
	//{
	//	for (i = 0; i < 2; i++)
	//	{
	//		iPt = pEdge->iVertex[i];

	//		pPt = NodeArray.Element + iPt;

	//		pEdgeList = &(pPt->EdgeList);

	//		RVLQLIST_ADD_ENTRY(pEdgeList, pMeshEdgePtr);

	//		pMeshEdgePtr->pEdge = pEdge;

	//		pMeshEdgePtr++;
	//	}
	//}

	delete[] VertexEdgeListArray.Element;
	delete[] VertexEdgeMem;
	delete[] PolyArray;
	delete[] EdgePolyAssignmentArray;
	delete[] PolyEdgeAssignmentArray;

	return true;
}

// Given a point index array, computes distribution of points and their average color.
// Input: PtArray - point index array.
// Output: distribution - see definition of MESH::Distribution

void Mesh::ComputeDistribution(
	Array<int> &PtArray,
	MESH::Distribution &distribution)
{
	Moments<float> moments;

	InitMoments<float>(moments);

	int RGB[3];

	RVLNULL3VECTOR(RGB);

	int i;
	Point *pPt;

	for (i = 0; i < PtArray.n; i++)
	{
		pPt = NodeArray.Element + PtArray.Element[i];

		UpdateMoments<float>(moments, pPt->P);

		RVLSUM3VECTORS(RGB, pPt->RGB, RGB);
	}

	float C[9];

	GetCovMatrix3<float>(&moments, C, distribution.t);

	RVLSCALE3VECTOR2(RGB, moments.n, distribution.RGB);

	//Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver(Eigen::Map<Eigen::Matrix3f>(C));
	Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver;

	eigenSolver.compute(Eigen::Map<Eigen::Matrix3f>(C));

	Eigen::Map<Eigen::Matrix3f>(distribution.R) = eigenSolver.pseudoEigenvectors();

	//Eigen::Map<Eigen::Vector3f>(distribution.var) = eigenSolver.eigenvalues();

	Eigen::Vector3cf var_ = eigenSolver.eigenvalues();

	distribution.var[0] = var_[0].real();
	distribution.var[1] = var_[1].real();
	distribution.var[2] = var_[2].real();
}

// Input:  list of point indices pInPtList,
//         array map such that all points i in pInPtList have the same value map[i],
//         ptr. pPtIdx to the first element of pInPtList which is searched; the elements preceding this pointer are not searched.
// Output: idx. iPt of the found boundary point,
//         connector pEdgePtr to the first boundary edge in CCW direction connected to the point iPt,
//         ptr. pPtIdx to the element of the list pInPtList corresponding to the point iPt.

bool Mesh::FindBoundaryEdge(
	QList<QLIST::Index> *pInPtList,
	QLIST::Index *&pPtIdx,
	int *map,
	int &iPt,
	MeshEdgePtr *&pEdgePtr)
{
	int idx = map[pPtIdx->Idx];

	while (pPtIdx)
	{
		iPt = pPtIdx->Idx;

		if (IsBoundaryPoint(iPt, map, idx, pEdgePtr))
			return true;

		pPtIdx = pPtIdx->pNext;
	}

	return false;
}

// Given a region defined by a vertex list pInPtList and an array map (map[i] is equal for all vertices i of the mesh belonging to the region), 
// the function returns an index array pOutPtArray of region boundary points.
// The function requires a pre-allocated array QLIST::Index *pMem.

void Mesh::Boundary(
	QList<QLIST::Index2> *pInPtList,
	int *map,
	QList<QLIST::Index> *pOutPtArray,
	QLIST::Index *pMem)
{
	RVLQLIST_INIT(pOutPtArray);

	QLIST::Index2 *pPtIdx = pInPtList->pFirst;

	int idx = map[pPtIdx->Idx];

	MeshEdgePtr *pEdgePtr;

	while (pPtIdx)
	{
		if (IsBoundaryPoint(pPtIdx->Idx, map, idx, pEdgePtr))
		{
			RVLQLIST_ADD_ENTRY(pOutPtArray, pMem);

			pMem->Idx = pPtIdx->Idx;

			pMem++;
		}

		pPtIdx = pPtIdx->pNext;
	}

//	// Find a boundary point
//
//#ifdef RVLMESH_BOUNDARY_DEBUG
//	FILE *fpDebug = fopen("C:\\RVL\\Debug\\meshdebug.txt", "w");
//#endif
//
//	QLIST::Index *pPtIdx = pInPtList->pFirst;
//
//	int iPt;
//	MeshEdgePtr *pEdgePtr;
//
//	while (FindBoundaryEdge(pInPtList, pPtIdx, map, iPt, pEdgePtr))
//	{
//#ifdef RVLMESH_BOUNDARY_DEBUG
//		fprintf(fpDebug, "P %d (%d) E %d\n", iPt, map[iPt], pEdgePtr - EdgePtrMem);
//#endif
//
//		RVLQLIST_INIT(pOutPtArray);
//
//		// Follow boundary
//
//#ifdef RVLMESH_BOUNDARY_DEBUG
//		int debugState_ = 0;
//#endif	
//
//		MeshEdge *pEdge = pEdgePtr->pEdge;
//
//		int side = (pEdge->iVertex[0] == iPt ? 0 : 1);
//
//		MeshEdgePtr *pEdgePtr0 = pEdgePtr;
//
//		int iNeighborPt;
//		QList<MeshEdgePtr> *pEdgeList;
//		Point *pPt;
//
//		do
//		{
//			RVLQLIST_ADD_ENTRY(pOutPtArray, pMem);
//
//			pMem->Idx = iPt;
//
//			pMem++;
//
//#ifdef RVLMESH_BOUNDARY_DEBUG
//			debugState_++;
//
//			if (debugState_ >= debugState)
//				break;
//#endif
//
//			RVLMESH_GET_POINT(pEdge, 1 - side, iPt, pEdgePtr);
//			RVLMESH_GET_NEXT_IN_REGION(this, iPt, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge);
//
//#ifdef RVLMESH_BOUNDARY_DEBUG
//			fprintf(fpDebug, "\nP %d (%d) E %d\n", iPt, map[iPt], pEdgePtr - EdgePtrMem);
//#endif
//		} while (pEdgePtr != pEdgePtr0);
//
//#ifdef RVLMESH_BOUNDARY_DEBUG
//		fclose(fpDebug);
//#endif
//	}
}

// Input:  pInPtList - list of indices of region points
//         map - map
// Output: BoundaryArray - array of boundary contours; each boundary contour is represented by an array of ptrs. to edge connectors;
//                         memory for BoundaryArray must be allocated befor calling the function
//         pBoundaryMem - ptr. to the next free allocated memory for storing ptrs. to edge connectors 
//         edgeMarkMap - each element corresponds to a mesh edge; all elements corresponding to the edges of all region boundaries must be 
//                       set to 0 before calling the function

void Mesh::Boundary(
	QList<QLIST::Index2> *pInPtList,
	int *map,
	Array<Array<MeshEdgePtr *>> &BoundaryArray,
	MeshEdgePtr **&pBoundaryMem,
	unsigned char *edgeMarkMap)
{
	QLIST::Index2 *pPtIdx = pInPtList->pFirst;

	int idx = map[pPtIdx->Idx];

	BoundaryArray.n = 0;

	MeshEdge *pEdge;
	MeshEdgePtr *pEdgePtr, *pEdgePtr0, *pEdgePtr0_;
	int iPt, iPt_;
	int side;
	int iNeighborPt;
	QList<MeshEdgePtr> *pEdgeList;
	Point *pPt;
	Array<MeshEdgePtr *> *pBoundary;

	while (pPtIdx)
	{
		iPt = pPtIdx->Idx;

		// Identify the first boundary point.

		//if (iPt == 45740)
		//	int debug = 0;

		if (IsBoundaryPoint(iPt, map, idx, pEdgePtr))
		{
			pEdgePtr0_ = pEdgePtr;

			iPt_ = iPt;

			do  // for each boundary passing through the vertex iPt
			{
				pEdge = pEdgePtr->pEdge;

				side = RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdge, iPt_);

				if ((edgeMarkMap[pEdgePtr->pEdge->idx] & (1 << side)) == 0)
				{
					// Open new boundary.

					pBoundary = BoundaryArray.Element + BoundaryArray.n;

					BoundaryArray.n++;

					pBoundary->Element = pBoundaryMem;

					/// Follow boundary.

					pEdgePtr0 = pEdgePtr;

					do
					{
						edgeMarkMap[pEdgePtr->pEdge->idx] |= (1 << side);

						*(pBoundaryMem++) = pEdgePtr;	// Add connector pEdgePtr to boundary.

						RVLMESH_GET_POINT(pEdge, 1 - side, iPt_, pEdgePtr);

						RVLMESH_GET_NEXT_IN_REGION(this, iPt_, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge);

					} while (pEdgePtr != pEdgePtr0);

					pBoundary->n = pBoundaryMem - pBoundary->Element;

					///
				}
			} while (GetNextBoundaryEdge(iPt, map, pEdgePtr) && pEdgePtr != pEdgePtr0_);	 // for each boundary passing through the vertex iPt
		}

		pPtIdx = pPtIdx->pNext;
	}	// for each surfel point
}

//bool Mesh::Load(
//	char *FileName,
//	PCLMeshBuilder *pMeshBuilder,
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
//	pcl::PolygonMesh &PCLMesh,
//	bool bSavePLY)
//{
//	char *fileExtension = RVLGETFILEEXTENSION(FileName);
//
//	if (strcmp(fileExtension, "ply") == 0)
//		LoadPolyDataFromPLY(FileName);
//	else
//	{
//		if (strcmp(fileExtension, "pcd") == 0)
//			PCLLoadPCD(FileName, PC);
//		else if (strcmp(fileExtension, "bmp") == 0)
//		{
//			char *depthFileName = RVLCreateString(FileName);
//
//			sprintf(RVLGETFILEEXTENSION(depthFileName), "txt");
//
//			Array2D<short int> depthImage;
//
//			depthImage.Element = NULL;
//			depthImage.w = depthImage.h = 0;
//
//			unsigned int format;
//
//			ImportDisparityImage(depthFileName, depthImage, format);
//
//			IplImage *RGBImage = cvLoadImage(FileName);
//
//			RGBDCamera camera;
//
//			printf("Creating point cloud from RGB-D image.\n");
//
//			camera.GetPointCloud(&depthImage, RGBImage, PC);
//
//			delete[] depthFileName;
//			delete[] depthImage.Element;
//
//			cvReleaseImage(&RGBImage);
//		}
//		else
//		{
//			printf("ERROR: Unknown file format!\n");
//
//			return false;
//		}
//
//		printf("Creating organized PCL mesh from point cloud...");
//
//		pMeshBuilder->CreateMesh(PC, PCLMesh);
//
//		printf("completed.\n");
//
//		if (bSavePLY)
//		{
//			char *PLYFileName = RVLCreateString(FileName);
//
//			sprintf(RVLGETFILEEXTENSION(PLYFileName), "ply");
//
//			printf("Saving mesh to %s...", PLYFileName);
//
//			PCLSavePLY(PLYFileName, PCLMesh);
//
//			printf("completed.\n");
//
//			delete[] PLYFileName;
//		}
//
//		PCLMeshToPolygonData(PCLMesh, pPolygonData);
//	}
//
//	printf("Creating ordered mesh from PCL mesh...");
//
//	CreateOrderedMeshFromPolyData();
//
//	printf("completed.\n");
//
//	return true;
//}

void Mesh::BoundingBox(Box<float> *pBox)
{
	if (NodeArray.n == 0)
		return;

	float *P = NodeArray.Element[0].P;

	pBox->minx = pBox->maxx = P[0];
	pBox->miny = pBox->maxy = P[1];
	pBox->minz = pBox->maxz = P[2];

	int iPt;

	for (iPt = 1; iPt < NodeArray.n; iPt++)
	{
		P = NodeArray.Element[iPt].P;

		if (P[0] < pBox->minx)
			pBox->minx = P[0];
		else if (P[0] > pBox->maxx)
			pBox->maxx = P[0];

		if (P[1] < pBox->miny)
			pBox->miny = P[1];
		else if (P[1] > pBox->maxy)
			pBox->maxy = P[1];

		if (P[2] < pBox->minz)
			pBox->minz = P[2];
		else if (P[2] > pBox->maxz)
			pBox->maxz = P[2];
	}
}
