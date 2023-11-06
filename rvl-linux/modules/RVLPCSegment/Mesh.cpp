// #include "stdafx.h"
// #include <pcl/io/pcd_io.h>
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
// #include <pcl/common/common.h>
// #include <pcl/PolygonMesh.h>
// #include <pcl/surface/vtk_smoothing/vtk_utils.h>
// #include "PCLTools.h"
// #include "PCLMeshBuilder.h"
// #include "RGBDCamera.h"
#include "Mesh.h"
#include "Visualizer.h"

/////

using namespace RVL;

Mesh::Mesh()
{
	flags = 0x00000000;
	mapNodesToPolyData = NULL;
	normalEstimationRadius = 10.0f;
	bOrganizedPC = false;
	bLabels = false;
#ifdef RVLMESH_BOUNDARY_DEBUG
	debugState = 1;
#endif
	faces.Element = NULL;
	faceMem = NULL;
	iValidVertices.Element = NULL;
	iVisibleFaces.Element = NULL;
	name = NULL;
}

Mesh::~Mesh()
{
	RVL_DELETE_ARRAY(mapNodesToPolyData);
	RVL_DELETE_ARRAY(faces.Element);
	RVL_DELETE_ARRAY(iValidVertices.Element);
	RVL_DELETE_ARRAY(iVisibleFaces.Element);
	RVL_DELETE_ARRAY(name);
}

void Mesh::LoadFromPLY(
	char *PLYFileName,
	float maxMeshTriangleEdgeLen,
	bool bOrganizedPCIn,
	Camera *pCamera)
{
	LoadPolyDataFromPLY(PLYFileName);
	if (bOrganizedPCIn)
	{
		bOrganizedPC = true;
		width = pCamera->w;
		height = pCamera->h;
	}
	else
		bOrganizedPC = false;
	CreateOrderedMeshFromPolyData(NULL, NULL, maxMeshTriangleEdgeLen);
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

// VIDOVIC
void Mesh::SavePolyDataToPLY(char *PLYFileName)
{
	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();

	writer->SetFileName(PLYFileName);

	writer->SetColorMode(VTK_COLOR_MODE_DEFAULT);
	writer->SetArrayName("Colors");

#if VTK_MAJOR_VERSION <= 5
	writer->SetInput(pPolygonData);
#else
	writer->SetInputData(pPolygonData);
#endif

	writer->Write();
}

void Mesh::SaveNoisedPolyDataToPLY(char *PLYFileName)
{
	char *sceneNoisedMeshFileName = new char[100];

	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();

	// create noised mesh filename
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
// END VIDOVIC

bool Mesh::CreateOrderedMeshFromPolyData(
	int *pixMap,
	int pixMapSize,
	float maxPolyEdgeLen)
{
	int noPts = pPolygonData->GetNumberOfPoints();
	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pPolygonData->GetPoints()->GetData());
	if (pointData == NULL)
		return false;

	vtkSmartPointer<vtkFloatArray> normalPointData = normalPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("Normals"));
	if (normalPointData == NULL)
	{
		normalPointData = normalPointData->SafeDownCast(pPolygonData->GetPointData()->GetNormals());

		if (normalPointData != NULL)
			flags |= RVLMESH_FLAG_NORMALS;
		else
		{
			vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
			normals->SetInputData(pPolygonData);

			normals->ComputePointNormalsOn();
			// normals->ComputeCellNormalsOff();
			normals->SetFlipNormals(1);
			normals->Update();

			// pPolygonData = normals->GetOutput();
			// normalPointData = vtkFloatArray::SafeDownCast(pPolygonData->GetPointData()->GetNormals());
			normalPointData = vtkFloatArray::SafeDownCast(normals->GetOutput()->GetPointData()->GetNormals());

			flags |= RVLMESH_FLAG_NORMALS;

			// vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();

			// writer->SetFileName("D:\\tmp.ply");

			// writer->SetInputData(normals->GetOutput());

			// writer->Write();
		}
	}
	else
		flags |= RVLMESH_FLAG_NORMALS;

	// vtkIdType numberOfPointArrays = pPolygonData->GetPointData()->GetNumberOfArrays();

	// for (vtkIdType i = 0; i < numberOfPointArrays; i++)
	//{
	//	int dataTypeID = pPolygonData->GetPointData()->GetArray(i)->GetDataType();
	//	std::cout << "Array " << i << ": " << pPolygonData->GetPointData()->GetArrayName(i)
	//		<< " (type: " << dataTypeID << ")" << std::endl;
	// }

	int iPt;
	unsigned char RGB[4];

	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("RGB"));
	if (rgbPointData == NULL)
	{
		rgbPointData = rgbPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("RGBA"));

		if (rgbPointData)
		{
			vtkSmartPointer<vtkUnsignedCharArray> rgbPointDataTgt = vtkSmartPointer<vtkUnsignedCharArray>::New();
			rgbPointDataTgt->SetNumberOfComponents(3);
			rgbPointDataTgt->SetName("RGB");

			for (iPt = 0; iPt < noPts; iPt++)
			{
				rgbPointData->GetTypedTuple(iPt, RGB);
				rgbPointDataTgt->InsertNextTypedTuple(RGB);
			}

			pPolygonData->GetPointData()->SetScalars(rgbPointDataTgt);

			flags |= RVLMESH_FLAG_COLOR;
		}
		else
		{
			rgbPointData = rgbPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("Colors"));

			if (rgbPointData)
			{
				rgbPointData->SetName("RGB");

				flags |= RVLMESH_FLAG_COLOR;
			}
			else
			{
				// vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = vtkSmartPointer<vtkUnsignedCharArray>::New();
				rgbPointData = vtkSmartPointer<vtkUnsignedCharArray>::New();
				rgbPointData->SetNumberOfComponents(3);
				rgbPointData->SetName("RGB");

				unsigned char color[] = {255, 255, 255};

				for (iPt = 0; iPt < noPts; iPt++)
					rgbPointData->InsertNextTypedTuple(color);

				pPolygonData->GetPointData()->SetScalars(rgbPointData);

				flags |= RVLMESH_FLAG_COLOR;
			}
		}
	}
	else
		flags |= RVLMESH_FLAG_COLOR;

	// Get vertices

	Clear();

	if (pixMapSize > 0)
		NodeMem = new Point[pixMapSize];
	else
		NodeMem = new Point[noPts]; // VIDOVIC

	// NodeArray.Element = new Point[noPts]; //VIDOVIC
	NodeArray.Element = NodeMem; // VIDOVIC

	NodeArray.n = (pixMapSize > 0 ? pixMapSize : noPts);

	Point *pPt;

	if (pixMap)
		memset(NodeArray.Element, 0, pixMapSize * sizeof(Point));

	QList<MeshEdgePtr> *pEdgeList;
	float fTmp;

	for (iPt = 0; iPt < noPts; iPt++)
	{
		pPt = NodeArray.Element + (pixMap ? pixMap[iPt] : iPt);

		if (flags & RVLMESH_FLAG_COLOR)
		{
			rgbPointData->GetTypedTuple(iPt, RGB);
			// RVLCONVTOINT3(RGB, pPt->RGB);
			RVLCOPY3VECTOR(RGB, pPt->RGB);
		}
		else
			RVLSET3VECTOR(pPt->RGB, 255, 255, 255);
		pointData->GetTypedTuple(iPt, pPt->P);
		if (flags & RVLMESH_FLAG_NORMALS)
		{
			normalPointData->GetTypedTuple(iPt, pPt->N);
			fTmp = sqrt(RVLDOTPRODUCT3(pPt->N, pPt->N));
			if (fTmp > 0.0f)
				RVLSCALE3VECTOR2(pPt->N, fTmp, pPt->N);
		}
		pPt->flags = 0x00;
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

	// EdgeArray.Element = new MeshEdge[maxnEdges]; //VIDOVIC
	EdgeArray.Element = EdgeMem; // VIDOVIC

	MeshEdge *pEdge = EdgeArray.Element;

	Array<QList<QLIST::Index>> VertexEdgeListArray; // Each element of this array is a list of edge indices corresponding to a mesh vertex.

	VertexEdgeListArray.Element = new QList<QLIST::Index>[NodeArray.n];
	VertexEdgeListArray.n = NodeArray.n;

	QList<QLIST::Index> *pVertexEdgeList;

	RVLQLIST_ARRAY_INIT(VertexEdgeListArray, pVertexEdgeList);

	QLIST::Index *VertexEdgeMem = new QLIST::Index[2 * maxnEdges];

	QLIST::Index *pVertexEdgeIdx = VertexEdgeMem;

	int polyDataSize = maxnPolygonVertices + 1;

	int *PolyArray = new int[polyDataSize * nPolys]; // A matrix nPolys x polyDataSize. Each row represents one polygon.
													 // The first element of each row represents the number of polygon vertices nPts.
													 // The next nPts elements of a row are indices of the polygon vertices.
													 // The remaining elements of the row are undefined.

	int *EdgePolyAssignmentArray = new int[4 * maxnEdges]; // A matrix EdgeArray.n x 4. Each row represents one edge.
														   // The first two elements of i-th row are the indices of the polygons sharing the edge i.
														   // The first of these two elements represents the index of the polygon on the left side of the edge,
														   // while the second one represents the index of the polygon on the right side of the edge.
														   // The other two elements of i-th row are the indices of the edge in the polygon edge list.
														   // The third element is the index of the edge in the list of the polygon on the left side of the edge,
														   // while the fourth one is the index of the edge in the list of the polygon on the right side of the edge.

	memset(EdgePolyAssignmentArray, 0xff, 4 * maxnEdges * sizeof(int));

	int *PolyEdgeAssignmentArray = new int[maxnEdges]; // A matrix nPolys x maxnPolygonVertices. Each row contains the indices of the edges of the corresponding polygon.
													   // Only the first nPts elements of each row are defined, where nPts is the number of the polygon vertices.

	int *PolyEdge = PolyEdgeAssignmentArray;

	int edgeSide;

	int *EdgePoly;
	int *EdgePolyIdx;

	int iEdge = 0;

	double maxPolyEdgeLen2 = maxPolyEdgeLen * maxPolyEdgeLen;

	Polys->InitTraversal();

	int iPt1, iPt2, iPt1_, iPt2_;
	int iEdge_;
	int *PolyVertex;
	int *PolyData;
	bool bAlreadyExists;
	MeshEdge *pEdge_;
	QLIST::Index *pVertexEdgeIdx_;
	iPoly = 0;
	int iPoly_;
	Point *pPt1, *pPt2;
	float dP[3];
	for (iPoly_ = 0; iPoly_ < nPolys; iPoly_++)
	{
		if (Polys->GetNextCell(l))
		{
			PolyData = PolyArray + polyDataSize * iPoly;

			nPts = l->GetNumberOfIds();

			PolyData[0] = nPts;

			if (maxPolyEdgeLen > 0.0f)
			{
				iPt1 = l->GetId(nPts - 1);
				pPt1 = NodeArray.Element + iPt1;
				for (iPt = 0; iPt < nPts; iPt++)
				{
					iPt2 = l->GetId(iPt);
					pPt2 = NodeArray.Element + iPt2;
					RVLDIF3VECTORS(pPt2->P, pPt1->P, dP);
					if (RVLDOTPRODUCT3(dP, dP) > maxPolyEdgeLen2)
						break;
					pPt1 = pPt2;
				}
				if (iPt < nPts)
					continue;
			}

			PolyVertex = PolyData + 1;

			PolyEdge = PolyEdgeAssignmentArray + maxnPolygonVertices * iPoly;

			iPt1 = l->GetId(nPts - 1);

			iPt1_ = (pixMap ? pixMap[iPt1] : iPt1);

			for (iPt = 0; iPt < nPts; iPt++)
			{
				iPt2 = l->GetId(iPt);

				iPt2_ = (pixMap ? pixMap[iPt2] : iPt2);

				PolyVertex[iPt] = iPt2_;

				// bAlreadyExists <- edge connecting the vertices iPt1 and iPt2 already exists in the list of VertexEdgeListArray[iPt1].

				bAlreadyExists = false;

				pVertexEdgeList = VertexEdgeListArray.Element + iPt1_;

				pVertexEdgeIdx_ = pVertexEdgeList->pFirst;

				while (pVertexEdgeIdx_)
				{
					iEdge_ = pVertexEdgeIdx_->Idx;

					pEdge_ = EdgeArray.Element + iEdge_;

					if (bAlreadyExists = (pEdge_->iVertex[0] == iPt2_))
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

					pEdge->iVertex[0] = iPt1_;
					pEdge->iVertex[1] = iPt2_;
					pEdge->idx = iEdge;

					RVLQLIST_ADD_ENTRY(pVertexEdgeList, pVertexEdgeIdx);
					pVertexEdgeIdx->Idx = iEdge;
					pVertexEdgeIdx++;

					pVertexEdgeList = VertexEdgeListArray.Element + iPt2_;
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

				iPt1_ = iPt2_;
			} // for each Poly vertex
			iPoly++;
		}
	} // for each Poly

	EdgeArray.n = pEdge - EdgeArray.Element;

	// Remove invalid points.

	Array<int> iPtBuff1;

	iPtBuff1.Element = new int[noPts];

	Array<int> iPtBuff2;

	iPtBuff2.Element = new int[noPts];

	Array<int> *piPtBuffPut = &iPtBuff1;
	Array<int> *piPtBuffFetch = &iPtBuff2;

	int iPt_;

	for (iPt = 0; iPt < noPts; iPt++)
	{
		iPt_ = (pixMap ? pixMap[iPt] : iPt);

		piPtBuffFetch->Element[iPt] = iPt_;

		pPt = NodeArray.Element + iPt_;

		pPt->bValid = true;
	}

	piPtBuffFetch->n = noPts;

	piPtBuffPut->n = 0;

	bool *bVisited = new bool[NodeArray.n];

	memset(bVisited, 0, NodeArray.n * sizeof(bool));

	int i, j, k;
	int nEdges;
	int nBoundaryEdges;
	Array<int> *piPtBuffTmp;
	Point *pPt_;
	int *EdgePoly_;

	while (piPtBuffFetch->n > 0)
	{
		for (i = 0; i < piPtBuffFetch->n; i++)
		{
			iPt = piPtBuffFetch->Element[i];

			// if (iPt == 45075)
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
							// if (iPoly == 43984)
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
				} // for every edge ending in pPt
			}	  // if (!pPt->bValid)
		}		  // for every point in piPtBuffFetch

		for (i = 0; i < piPtBuffPut->n; i++)
			bVisited[piPtBuffPut->Element[i]] = false;

		piPtBuffTmp = piPtBuffFetch;
		piPtBuffFetch = piPtBuffPut;
		piPtBuffPut = piPtBuffTmp;

		piPtBuffPut->n = 0;
	} // while (piPtBuffFetch->n > 0)

	delete[] iPtBuff1.Element;
	delete[] iPtBuff2.Element;
	delete[] bVisited;

	// Arrange edge lists of vertices according to the 2. property of Organized mesh (See the definition of organized mesh in ARP3D.TR3).

	EdgePtrMem = new MeshEdgePtr[2 * EdgeArray.n];

	MeshEdgePtr *pMeshEdgePtr = EdgePtrMem;

	// int watchdog;

	nBoundaryPts = 0;

	int iEdge0;

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		// if (iPt == 5929)
		//	int debug = 0;

		pPt = NodeArray.Element + iPt;

		pEdgeList = &(pPt->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		if (!pPt->bValid)
			continue;

		pVertexEdgeList = VertexEdgeListArray.Element + iPt;

		// Count edges connected to vertex pPt, i.e. the size of the edge list of pPt.
		// Count the boundary edges (See the definition of boundary edges in ARP3D.TR3).
		// If there are boundary edges connected to pPt, then iEdge0 <- the first edge connected to pPt in the CCW direction.

		nEdges = nBoundaryEdges = 0; // only for debugging purpose!!!

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
					nBoundaryEdges++; // only for debugging purpose!!!

					if (pEdge->iVertex[1] == iPt)
						iEdge0 = iEdge;
				}
				else if (EdgePoly[1] < 0)
				{
					nBoundaryEdges++; // only for debugging purpose!!!

					if (pEdge->iVertex[0] == iPt)
						iEdge0 = iEdge;
				}

				nEdges++; // only for debugging purpose!!!
			}

			pVertexEdgeIdx = pVertexEdgeIdx->pNext;
		}

		// if (nEdges < 2)
		//	int debug = 0;

		// if (nBoundaryEdges > 2)
		//	int debug = 0;

		// If no boundary edge is connected to vertex pPt, then the first edge in its edge list can be any edge connected to it.

		if (pPt->bBoundary)
			nBoundaryPts++;
		else
			iEdge0 = pVertexEdgeList->pFirst->Idx;

		// watchdog = 0;

		// if (iPt == 88031)
		//	int debug = 0;

		iEdge = iEdge0;

		do
		{
			pEdge = EdgeArray.Element + iEdge;

			// if (iEdge == 265106)
			//	int debug = 0;

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

			// MeshEdge *pEdge_ = EdgeArray.Element + iEdge;

			// int edgeSide_ = (pEdge_->iVertex[0] == iPt ? 0 : 1);

			// Point Pt1 = NodeArray.Element[iPt];
			// Point Pt2 = NodeArray.Element[pEdge->iVertex[1 - edgeSide]];
			// Point Pt3 = NodeArray.Element[pEdge_->iVertex[1 - edgeSide_]];

			// float V1[3], V2[3];

			// RVLDIF3VECTORS(Pt2.P, Pt1.P, V1);
			// RVLDIF3VECTORS(Pt3.P, Pt1.P, V2);

			// float Z[3];

			// RVLCROSSPRODUCT3(V1, V2, Z);

			// float a = RVLDOTPRODUCT3(Pt1.P, Z);

			// if (a > 0.0)
			//	int debug = 0;

			///////

			// watchdog++;

			// if (watchdog > 8)
			//	break;
		} while (iEdge != iEdge0);
	}

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		pPt = NodeArray.Element + iPt;

		if (pPt->N[0] != pPt->N[0] || pPt->N[1] != pPt->N[1] || pPt->N[2] != pPt->N[2])
		{
			pPt->bValid = false;

			continue;
		}

		if (RVLDOTPRODUCT3(pPt->N, pPt->N) < 0.5f)
		{
			pPt->bValid = false;

			continue;
		}
	}

	/////

	// MeshEdge* pEdgeArrayEnd = EdgeArray.Element + EdgeArray.n;

	// int i;

	// for (pEdge = EdgeArray.Element; pEdge < pEdgeArrayEnd; pEdge++)
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

void Mesh::ComputeMoments(Array<int> &PtArray,
						  Moments<float> &moments)
{
	InitMoments<float>(moments);

	int i;
	Point *pPt;

	for (i = 0; i < PtArray.n; i++)
	{
		pPt = NodeArray.Element + PtArray.Element[i];

		UpdateMoments<float>(moments, pPt->P);
	}
}

void Mesh::ComputeMoments(Array<int> &PtArray,
						  Moments<double> &moments)
{
	InitMoments<double>(moments);

	int i;
	Point *pPt;
	double P[3];

	for (i = 0; i < PtArray.n; i++)
	{
		pPt = NodeArray.Element + PtArray.Element[i];
		RVLCOPY3VECTOR(pPt->P, P);
		UpdateMoments<double>(moments, P);
	}
}

void Mesh::ComputeDistribution(
	Array<int> &PtArray,
	GaussianDistribution3D<float> *pDistribution)
{
	Moments<float> moments;
	ComputeMoments(PtArray, moments);
	GetCovMatrix3<float>(&moments, pDistribution->C, pDistribution->P);
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

	// Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver(Eigen::Map<Eigen::Matrix3f>(C));
	Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver;

	eigenSolver.compute(Eigen::Map<Eigen::Matrix3f>(C));

	Eigen::Map<Eigen::Matrix3f>(distribution.R) = eigenSolver.pseudoEigenvectors();

	// Eigen::Map<Eigen::Vector3f>(distribution.var) = eigenSolver.eigenvalues();

	Eigen::Vector3cf var_ = eigenSolver.eigenvalues();

	distribution.var[0] = var_[0].real();
	distribution.var[1] = var_[1].real();
	distribution.var[2] = var_[2].real();
}

void Mesh::ComputeDistributionDouble(
	Array<int> &PtArray,
	MESH::Distribution &distribution)
{
	Moments<double> moments;

	InitMoments<double>(moments);

	int RGB[3];

	RVLNULL3VECTOR(RGB);

	int i;
	Point *pPt;
	double P[3];

	for (i = 0; i < PtArray.n; i++)
	{
		pPt = NodeArray.Element + PtArray.Element[i];

		RVLCOPY3VECTOR(pPt->P, P);

		UpdateMoments<double>(moments, P);

		RVLSUM3VECTORS(RGB, pPt->RGB, RGB);
	}

	RVLSCALE3VECTOR2(RGB, moments.n, distribution.RGB);

	MESH::ComputeDistributionDouble(moments, distribution);
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
	// #ifdef RVLMESH_BOUNDARY_DEBUG
	//	FILE *fpDebug = fopen("C:\\RVL\\Debug\\meshdebug.txt", "w");
	// #endif
	//
	//	QLIST::Index *pPtIdx = pInPtList->pFirst;
	//
	//	int iPt;
	//	MeshEdgePtr *pEdgePtr;
	//
	//	while (FindBoundaryEdge(pInPtList, pPtIdx, map, iPt, pEdgePtr))
	//	{
	// #ifdef RVLMESH_BOUNDARY_DEBUG
	//		fprintf(fpDebug, "P %d (%d) E %d\n", iPt, map[iPt], pEdgePtr - EdgePtrMem);
	// #endif
	//
	//		RVLQLIST_INIT(pOutPtArray);
	//
	//		// Follow boundary
	//
	// #ifdef RVLMESH_BOUNDARY_DEBUG
	//		int debugState_ = 0;
	// #endif
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
	// #ifdef RVLMESH_BOUNDARY_DEBUG
	//			debugState_++;
	//
	//			if (debugState_ >= debugState)
	//				break;
	// #endif
	//
	//			RVLMESH_GET_POINT(pEdge, 1 - side, iPt, pEdgePtr);
	//			RVLMESH_GET_NEXT_IN_REGION(this, iPt, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge);
	//
	// #ifdef RVLMESH_BOUNDARY_DEBUG
	//			fprintf(fpDebug, "\nP %d (%d) E %d\n", iPt, map[iPt], pEdgePtr - EdgePtrMem);
	// #endif
	//		} while (pEdgePtr != pEdgePtr0);
	//
	// #ifdef RVLMESH_BOUNDARY_DEBUG
	//		fclose(fpDebug);
	// #endif
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

		// if (iPt == 45740)
		//	int debug = 0;

		if (IsBoundaryPoint(iPt, map, idx, pEdgePtr))
		{
			pEdgePtr0_ = pEdgePtr;

			iPt_ = iPt;

			do // for each boundary passing through the vertex iPt
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

						*(pBoundaryMem++) = pEdgePtr; // Add connector pEdgePtr to boundary.

						RVLMESH_GET_POINT(pEdge, 1 - side, iPt_, pEdgePtr);

						RVLMESH_GET_NEXT_IN_REGION(this, iPt_, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge);

					} while (pEdgePtr != pEdgePtr0);

					pBoundary->n = pBoundaryMem - pBoundary->Element;

					///
				}
			} while (GetNextBoundaryEdge(iPt, map, pEdgePtr) && pEdgePtr != pEdgePtr0_); // for each boundary passing through the vertex iPt
		}

		pPtIdx = pPtIdx->pNext;
	} // for each surfel point
}

// bool Mesh::Load(
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
// }

void MESH::BoundingBox(
	vtkSmartPointer<vtkPolyData> pPolygonData,
	Box<float> *pBox)
{
	BoundingBox<float>(pPolygonData, pBox);
}

void Mesh::BoundingBox(Box<float> *pBox)
{
	if (NodeArray.n == 0)
		return;

	float *P = NodeArray.Element[0].P;

	InitBoundingBox<float>(pBox, P);

	int iPt;

	for (iPt = 1; iPt < NodeArray.n; iPt++)
	{
		P = NodeArray.Element[iPt].P;

		UpdateBoundingBox<float>(pBox, P);
	}
}

bool Mesh::ConvexHull2(
	Array2D<float> points,
	CRVLMem *pMem,
	bool bClearMem,
	bool bDepthImage)
{
	if (bClearMem)
		pMem->Clear();

	// Add points to NodeArray.

	RVL_DELETE_ARRAY(NodeMem);
	NodeArray.n = points.h;
	NodeMem = new Point[NodeArray.n];
	NodeArray.Element = NodeMem;

	bool *bInHull = new bool[NodeArray.n];

	float *PSrc = points.Element;

	int iPt;
	float *P;
	Point *pPt;
	QList<MeshEdgePtr> *pEdgeList;

	for (iPt = 0; iPt < points.h; iPt++, PSrc += 3)
	{
		pPt = NodeArray.Element + iPt;

		P = pPt->P;

		RVLCOPY3VECTOR(PSrc, P);

		pEdgeList = &(pPt->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		pPt->bValid = false;

		bInHull[iPt] = false;
	}

	// Create empty face list.

	QList<MESH::Face> faceList;

	QList<MESH::Face> *pFaceList = &faceList;

	RVLQLIST_INIT(pFaceList);

	/// Create the initial convex hull from four non-coplanar points.

	// pPt0 <- the first point in NodeArray

	Point *pPt0 = NodeArray.Element;

	float *P0 = pPt0->P;

	int iPt_[4];

	// pPt0 <- the most distant point from pPt0

	float maxDist = 1e-6;

	int iPt0 = -1;

	float *P1;
	float V01[3];
	float fTmp;
	Point *pPt1;

	for (iPt = 1; iPt < NodeArray.n; iPt++)
	{
		pPt1 = NodeArray.Element + iPt;

		P1 = pPt1->P;

		RVLDIF3VECTORS(P1, P0, V01);

		fTmp = RVLDOTPRODUCT3(V01, V01);

		if (fTmp > maxDist)
		{
			maxDist = fTmp;

			iPt0 = iPt;
		}
	}

	if (iPt0 < 0)
	{
		delete[] bInHull;

		return false;
	}

	bInHull[iPt0] = true;

	iPt_[0] = iPt0;

	pPt0 = NodeArray.Element + iPt0;

	P0 = pPt0->P;

	// pPt1 <- the most distant point from pPt0

	maxDist = 1e-6;

	int iPt1 = -1;

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		pPt1 = NodeArray.Element + iPt;

		P1 = pPt1->P;

		RVLDIF3VECTORS(P1, P0, V01);

		fTmp = RVLDOTPRODUCT3(V01, V01);

		if (fTmp > maxDist)
		{
			maxDist = fTmp;

			iPt1 = iPt;
		}
	}

	bInHull[iPt1] = true;

	iPt_[1] = iPt1;

	pPt1 = NodeArray.Element + iPt1;

	P1 = pPt1->P;

	RVLDIF3VECTORS(P1, P0, V01);

	MeshEdgePtr *pEdgePtr;

	// pPt2 <- the most distant point from the line defined by pPt0 and pPt1

	maxDist = 1e-6;

	int iPt2 = -1;

	Point *pPt2;
	float *P2;
	float V02[3], N[3];

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		pPt2 = NodeArray.Element + iPt;

		P2 = pPt2->P;

		RVLDIF3VECTORS(P2, P0, V02);

		fTmp = RVLDOTPRODUCT3(V02, V02);

		if (fTmp > 1e-6)
		{
			RVLDIF3VECTORS(P2, P1, V02);

			RVLCROSSPRODUCT3(V01, V02, N);

			fTmp = RVLDOTPRODUCT3(N, N);

			if (fTmp > maxDist)
			{
				maxDist = fTmp;

				iPt2 = iPt;
			}
		}
	}

	if (iPt2 < 0)
	{
		delete[] bInHull;

		return false;
	}

	bInHull[iPt2] = true;

	iPt_[2] = iPt2;

	pPt2 = NodeArray.Element + iPt2;

	P2 = pPt2->P;

	RVLDIF3VECTORS(P2, P1, V02);

	RVLCROSSPRODUCT3(V01, V02, N);

	fTmp = RVLDOTPRODUCT3(N, N);

	// pPt3 <- the most distant point from the plane defined by pPt0, pPt1 and pPt2

	float Area = sqrt(fTmp);

	RVLSCALE3VECTOR2(N, Area, N);

	float d = RVLDOTPRODUCT3(N, P0);

	maxDist = 1e-6;

	int iPt3 = -1;

	Point *pPt3;
	float *P3;

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		pPt3 = NodeArray.Element + iPt;

		P3 = pPt3->P;

		fTmp = RVLDOTPRODUCT3(N, P3) - d;

		fTmp = RVLABS(fTmp);

		if (fTmp > maxDist)
		{
			maxDist = fTmp;

			iPt3 = iPt;
		}
	}

	faces.n = 0;

	if (iPt3 < 0)
	{
		delete[] bInHull;

		return false;
	}

	bInHull[iPt3] = true;

	iPt_[3] = iPt3;

	pPt3 = NodeArray.Element + iPt3;

	P3 = pPt3->P;

	fTmp = RVLDOTPRODUCT3(N, P3) - d;

	// Create the initial tetrahedron.

	if (fTmp > 0.0f)
	{
		iPt = iPt_[2];
		iPt_[2] = iPt_[1];
		iPt_[1] = iPt;
		RVLNEGVECT3(N, N);
		d = -d;
	}

	int iFace = 0; // This variable is not necessary for this algorihtm, but it facilitates debugging.

	MESH::Face *pFace;

	RVLMEM_ALLOC_STRUCT(pMem, MESH::Face, pFace);

	RVLQLIST_ADD_ENTRY(pFaceList, pFace);

	pFace->idx = iFace++; // Field idx is not necessary for this algorihtm, but it facilitates debugging.

	pFace->flags = 0x00;

	RVLCOPY3VECTOR(N, pFace->N);

	pFace->d = d;

	pFace->Area = 0.5f * Area;

	MeshEdge *pEdge;

	pEdge = ConnectNodes<Point, MeshEdge, MeshEdgePtr>(iPt_[0], iPt_[1], NodeArray, pMem);
	pEdge->pFace[0] = pFace;
	pFace->pFirstEdgePtr = pEdge->pVertexEdgePtr[0]; // Field pFirstEdgePtr is not necessary for the algorithm,
													 // but it facilitate usage of the result.
	pEdge = ConnectNodes<Point, MeshEdge, MeshEdgePtr>(iPt_[1], iPt_[2], NodeArray, pMem);
	pEdge->pFace[0] = pFace;
	pEdge = ConnectNodes<Point, MeshEdge, MeshEdgePtr>(iPt_[2], iPt_[0], NodeArray, pMem);
	pEdge->pFace[0] = pFace;

	iPt = iPt_[3];

	pEdgeList = &(pPt3->EdgeList);

	P = P3;

	iPt2 = iPt_[0];

	pPt2 = pPt0;

	QList<MeshEdgePtr> *pEdgeList2 = &(pPt2->EdgeList);

	pEdgePtr = pEdgeList2->pFirst->pNext;

	MeshEdgePtr **ppEdgePtr = &(pEdgePtr->pNext);

	MeshEdgePtr **ppEdgePtrNew = &(pEdgeList->pFirst);

	int i;
	MeshEdgePtr *pEdgePtr2, *pEdgePtr3;

	for (i = 0; i < 3; i++)
	{
		RVLMEM_ALLOC_STRUCT(pMem, MeshEdge, pEdge);

		pEdge->iVertex[0] = iPt2;
		pEdge->iVertex[1] = iPt;

		RVLMEM_ALLOC_STRUCT(pMem, MeshEdgePtr, pEdgePtr2);

		RVLQLIST_INSERT_ENTRY2(ppEdgePtr, pEdgePtr2);

		pEdge->pVertexEdgePtr[0] = pEdgePtr2;
		pEdgePtr2->pEdge = pEdge;

		RVLMEM_ALLOC_STRUCT(pMem, MeshEdgePtr, pEdgePtr3);

		RVLQLIST_INSERT_ENTRY2(ppEdgePtrNew, pEdgePtr3);

		pEdge->pVertexEdgePtr[1] = pEdgePtr3;
		pEdgePtr3->pEdge = pEdge;

		RVLMEM_ALLOC_STRUCT(pMem, MESH::Face, pFace);

		RVLQLIST_ADD_ENTRY(pFaceList, pFace);

		pFace->idx = iFace++;

		pFace->flags = 0x00;

		P1 = NodeArray.Element[RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr)].P;
		P2 = pPt2->P;

		RVLDIF3VECTORS(P1, P2, V01);
		RVLDIF3VECTORS(P, P2, V02);
		RVLCROSSPRODUCT3(V01, V02, pFace->N);
		RVLNORM3(pFace->N, pFace->Area);
		pFace->Area *= 0.5f;
		pFace->d = RVLDOTPRODUCT3(pFace->N, P);

		pEdge->pFace[1] = pFace;

		pFace->pFirstEdgePtr = pEdge->pVertexEdgePtr[1];

		pEdgePtr->pEdge->pFace[RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdgePtr->pEdge, iPt2)] = pFace;

		pEdgePtr = pEdgePtr2;

		RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList2, pEdgePtr);

		iPt2 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

		pPt2 = NodeArray.Element + iPt2;

		pEdgeList2 = &(pPt2->EdgeList);

		pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

		ppEdgePtr = &(pEdgePtr->pNext);
	}

	pEdgePtr = pEdgeList->pFirst;

	while (pEdgePtr->pNext)
	{
		pEdgePtr->pNext->pEdge->pFace[0] = pEdgePtr->pEdge->pFace[1];

		pEdgePtr = pEdgePtr->pNext;
	}

	pEdgeList->pFirst->pEdge->pFace[0] = pEdgePtr->pEdge->pFace[1];

	/// Main loop.

	float z, d1, d2, d3;
	bool bUpdateHull;
	uchar flag[2];
	MeshEdgePtr *pEdgePtr0;

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	// for (iPt = 0; iPt < 28; iPt++)
	{
		// if (iPt == 27)
		//	int debug = 0;

		if (bInHull[iPt])
			continue;

		pPt = NodeArray.Element + iPt;

		P = pPt->P;

		pEdgeList = &(pPt->EdgeList);

		bUpdateHull = false;

		pFace = faceList.pFirst;

		while (pFace)
		{
			// if (pFace->idx == 92 || pFace->idx == 88 || pFace->idx == 84)
			//	int debug = 0;

			if (!(pFace->flags & RVLMESH_FACE_FLAG_REJECTED))
			{
				z = RVLDOTPRODUCT3(pFace->N, P) - pFace->d;

				// if (z > 1e-5)
				if (z > 0.0f)
				{
					pFace->flags |= RVLMESH_FACE_FLAG_REJECTED;

					// if (pFace->idx == 14)
					//	int debug = 0;

					bUpdateHull = true;
				}
			}

			pFace = pFace->pNext;
		}

		if (!bUpdateHull)
			continue;

		for (iPt2 = 0; iPt2 < NodeArray.n; iPt2++)
		{
			pPt2 = NodeArray.Element + iPt2;

			pEdgeList2 = &(pPt2->EdgeList);

			pEdgePtr = pEdgeList2->pFirst;

			while (pEdgePtr)
			{
				pEdge = pEdgePtr->pEdge;

				flag[0] = (pEdge->pFace[0]->flags & RVLMESH_FACE_FLAG_REJECTED);
				flag[1] = (pEdge->pFace[1]->flags & RVLMESH_FACE_FLAG_REJECTED);

				if (flag[0] ^ flag[1])
				{
					if (flag[RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdge, iPt2)])
						break;
				}

				pEdgePtr = pEdgePtr->pNext;
			}

			if (pEdgePtr)
				break;
		} // for (iPt2 = 0; iPt2 < NodeArray.n; iPt2++)

		if (iPt2 >= NodeArray.n)
			continue;

		pEdgePtr0 = pEdgePtr;

		ppEdgePtr = &(pEdgePtr->pNext);

		ppEdgePtrNew = &(pEdgeList->pFirst);

		while (true)
		{
			RVLMEM_ALLOC_STRUCT(pMem, MeshEdge, pEdge);

			pEdge->iVertex[0] = iPt2;
			pEdge->iVertex[1] = iPt;

			RVLMEM_ALLOC_STRUCT(pMem, MeshEdgePtr, pEdgePtr2);

			RVLQLIST_INSERT_ENTRY2(ppEdgePtr, pEdgePtr2);

			pEdge->pVertexEdgePtr[0] = pEdgePtr2;
			pEdgePtr2->pEdge = pEdge;

			RVLMEM_ALLOC_STRUCT(pMem, MeshEdgePtr, pEdgePtr3);

			RVLQLIST_INSERT_ENTRY2(ppEdgePtrNew, pEdgePtr3);

			pEdge->pVertexEdgePtr[1] = pEdgePtr3;
			pEdgePtr3->pEdge = pEdge;

			RVLMEM_ALLOC_STRUCT(pMem, MESH::Face, pFace);

			RVLQLIST_ADD_ENTRY(pFaceList, pFace);

			pFace->idx = iFace++;

			pFace->flags = 0x00;

			P1 = NodeArray.Element[RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr)].P;
			P2 = pPt2->P;

			RVLDIF3VECTORS(P1, P2, V01);
			RVLDIF3VECTORS(P, P2, V02);
			RVLCROSSPRODUCT3(V01, V02, pFace->N);
			RVLNORM3(pFace->N, pFace->Area);
			pFace->Area *= 0.5f;
			d1 = RVLDOTPRODUCT3(pFace->N, P1);
			d2 = RVLDOTPRODUCT3(pFace->N, P2);
			d3 = RVLDOTPRODUCT3(pFace->N, P);
			pFace->d = RVLMAX(d1, d2);
			pFace->d = RVLMAX(pFace->d, d3);

			pEdge->pFace[1] = pFace;

			pFace->pFirstEdgePtr = pEdge->pVertexEdgePtr[1];

			pEdgePtr->pEdge->pFace[RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdgePtr->pEdge, iPt2)] = pFace;

			ppEdgePtr = &(pEdgePtr2->pNext);

			if (*ppEdgePtr == NULL)
				ppEdgePtr = &(pEdgeList2->pFirst);

			pEdgePtr = *ppEdgePtr;

			while (pEdgePtr->pEdge->pFace[0]->flags & pEdgePtr->pEdge->pFace[1]->flags & RVLMESH_FACE_FLAG_REJECTED)
			{
				RVLQLIST_REMOVE_ENTRY(pEdgeList2, pEdgePtr, ppEdgePtr);

				if (*ppEdgePtr == NULL)
					ppEdgePtr = &(pEdgeList2->pFirst);

				RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList2, pEdgePtr);
			}

			iPt2 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

			pPt2 = NodeArray.Element + iPt2;

			pEdgeList2 = &(pPt2->EdgeList);

			pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

			if (pEdgePtr == pEdgePtr0)
				break;

			ppEdgePtr = &(pEdgePtr->pNext);
		}

		pEdgePtr = pEdgeList->pFirst;

		while (pEdgePtr->pNext)
		{
			pEdgePtr->pNext->pEdge->pFace[0] = pEdgePtr->pEdge->pFace[1];

			pEdgePtr = pEdgePtr->pNext;
		}

		pEdgeList->pFirst->pEdge->pFace[0] = pEdgePtr->pEdge->pFace[1];

		// Only for debugging purpose!

		// for (iPt2 = 0; iPt2 <= iPt; iPt2++)
		//{
		//	pPt2 = NodeArray.Element + iPt2;

		//	P2 = pPt2->P;

		//	pFace = faceList.pFirst;

		//	while (pFace)
		//	{
		//		if (!(pFace->flags & RVLMESH_FACE_FLAG_REJECTED))
		//		{
		//			z = RVLDOTPRODUCT3(pFace->N, P2) - pFace->d;

		//			if (z > 1e-6)
		//				int debug = 0;
		//		}

		//		pFace = pFace->pNext;
		//	}
		//}

		//
	} // for every point

	///

	// Free memory.

	delete[] bInHull;

	// Copy faces from faceList to faces.

	RVL_DELETE_ARRAY(faces.Element);

	faces.Element = new MESH::Face *[iFace];

	pFace = faceList.pFirst;

	while (pFace)
	{
		if (!(pFace->flags & RVLMESH_FACE_FLAG_REJECTED))
			faces.Element[faces.n++] = pFace;

		pFace = pFace->pNext;
	}

	// Create array of valid vertices.

	RVL_DELETE_ARRAY(iValidVertices.Element);

	iValidVertices.Element = new int[NodeArray.n];

	iValidVertices.n = 0;

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		pPt = NodeArray.Element + iPt;

		pEdgePtr = pPt->EdgeList.pFirst;

		if (!pEdgePtr)
			continue;

		if (pEdgePtr->pEdge->pFace[0]->flags & RVLMESH_FACE_FLAG_REJECTED)
			continue;

		iValidVertices.Element[iValidVertices.n++] = iPt;

		pPt->bValid = true;
	}

	// Create array of visible faces.

	iVisibleFaces.Element = new int[faces.n];

	iVisibleFaces.n = 0;

	for (i = 0; i < faces.n; i++)
		if (faces.Element[i]->d < 0.0f || !bDepthImage)
		{
			iVisibleFaces.Element[iVisibleFaces.n++] = i;

			faces.Element[i]->flags |= RVLMESH_FACE_FLAG_VISIBLE;
		}

	//

	return true;
}

bool Mesh::ConvexHull(
	Array2D<float> points,
	CRVLMem *pMem,
	bool bClearMem,
	bool bDepthImage)
{
	if (bClearMem)
		pMem->Clear();

	// Add points to NodeArray and compute their bounding box.

	Box<float> box;

	InitBoundingBox<float>(&box, points.Element);

	RVL_DELETE_ARRAY(NodeMem);
	NodeArray.n = points.h;
	NodeMem = new Point[NodeArray.n];
	NodeArray.Element = NodeMem;

	bool *bInHull = new bool[NodeArray.n];

	float *PSrc = points.Element;

	int iPt;
	float *P;
	Point *pPt;
	QList<MeshEdgePtr> *pEdgeList;

	for (iPt = 0; iPt < points.h; iPt++, PSrc += 3)
	{
		pPt = NodeArray.Element + iPt;

		P = pPt->P;

		// RVLCOPY3VECTOR(PSrc, P);

		pEdgeList = &(pPt->EdgeList);

		RVLQLIST_INIT(pEdgeList);

		pPt->bValid = false;

		bInHull[iPt] = false;

		UpdateBoundingBox<float>(&box, PSrc);
	}

	// Create integer points.

	int *iPArray = new int[3 * points.h];

	float Pc[3];

	RVLSET3VECTOR(Pc, (0.5f * (box.minx + box.maxx)), (0.5f * (box.miny + box.maxy)), (0.5f * (box.minz + box.maxz)));

	float Lx = box.maxx - box.minx;
	float Ly = box.maxy - box.miny;
	float Lz = box.maxz - box.minz;

	float maxL = RVLMAX(Lx, Ly);
	if (Lz > maxL)
		maxL = Lz;

	float s = 1000.0f / maxL;

	int *iP = iPArray;

	PSrc = points.Element;

	for (iPt = 0; iPt < points.h; iPt++, PSrc += 3, iP += 3)
	{
		iP[0] = (int)round(s * (PSrc[0] - Pc[0]));
		iP[1] = (int)round(s * (PSrc[1] - Pc[1]));
		iP[2] = (int)round(s * (PSrc[2] - Pc[2]));

		pPt = NodeArray.Element + iPt;

		P = pPt->P;

		RVLCOPY3VECTOR(iP, P);
		RVLSCALE3VECTOR2(P, s, P);
		RVLSUM3VECTORS(P, Pc, P);
	}

	// Create empty face list.

	QList<MESH::Face> faceList;

	QList<MESH::Face> *pFaceList = &faceList;

	RVLQLIST_INIT(pFaceList);

	/// Create the initial convex hull from four non-coplanar points.

	faces.n = 0;

	// iP0 <- the first point in NodeArray

	int *iP0 = iPArray;

	int iPt_[4];

	int iPt0 = 0;

	bInHull[iPt0] = true;

	iPt_[0] = iPt0;

	// iP1 <- a point different from pPt0

	int V01[3];
	int dist = 0;

	for (iPt = 1; iPt < points.h; iPt++)
	{
		iP = iPArray + 3 * iPt;

		RVLDIF3VECTORS(iP, iP0, V01);

		dist = RVLDOTPRODUCT3(V01, V01);

		if (dist > 0)
			break;
	}

	if (dist == 0)
	{
		delete[] bInHull;
		delete[] iPArray;

		return false;
	}

	int iPt1 = iPt;

	bInHull[iPt1] = true;

	iPt_[1] = iPt1;

	int *iP1 = iP;

	MeshEdgePtr *pEdgePtr;

	// iP2 <- a point which is not colinear with the points iP0 and iP1

	int V02[3], N0[3];

	for (iPt = 1; iPt < points.h; iPt++)
	{
		iP = iPArray + 3 * iPt;

		RVLDIF3VECTORS(iP, iP0, V02);

		RVLCROSSPRODUCT3(V01, V02, N0);

		if (N0[0] != 0)
			break;

		if (N0[1] != 0)
			break;

		if (N0[2] != 0)
			break;
	}

	if (iPt >= points.h)
	{
		delete[] bInHull;
		delete[] iPArray;

		return false;
	}

	int iPt2 = iPt;

	bInHull[iPt2] = true;

	iPt_[2] = iPt2;

	int *iP2 = iP;

	// pPt3 <- a point which is not coplanar with the points iP0, iP1 and iP2

	int d0 = RVLDOTPRODUCT3(N0, iP0);

	for (iPt = 1; iPt < points.h; iPt++)
	{
		iP = iPArray + 3 * iPt;

		dist = RVLDOTPRODUCT3(N0, iP) - d0;

		if (dist != 0)
			break;
	}

	if (iPt >= points.h)
	{
		delete[] bInHull;
		delete[] iPArray;

		return false;
	}

	int iPt3 = iPt;

	bInHull[iPt3] = true;

	iPt_[3] = iPt3;

	int *iP3 = iP;

	// Create the initial tetrahedron.

	if (dist > 0)
	{
		iPt = iPt_[2];
		iPt_[2] = iPt_[1];
		iPt_[1] = iPt;
		RVLNEGVECT3(N0, N0);
		d0 = -d0;
	}

	int iFace = 0; // This variable is not necessary for this algorihtm, but it facilitates debugging.

	MESH::Face *pFace;

	RVLMEM_ALLOC_STRUCT(pMem, MESH::Face, pFace);

	RVLQLIST_ADD_ENTRY(pFaceList, pFace);

	pFace->idx = iFace++; // Field idx is not necessary for this algorihtm, but it facilitates debugging.

	pFace->flags = 0x00;

	MESH::IntPlane *piPlane;

	RVLMEM_ALLOC_STRUCT(pMem, MESH::IntPlane, piPlane);

	pFace->piPlane = piPlane;

	RVLCOPY3VECTOR(N0, piPlane->N);

	piPlane->d = d0;

	MeshEdge *pEdge;

	pEdge = ConnectNodes<Point, MeshEdge, MeshEdgePtr>(iPt_[0], iPt_[1], NodeArray, pMem);
	pEdge->pFace[0] = pFace;
	pFace->pFirstEdgePtr = pEdge->pVertexEdgePtr[0]; // Field pFirstEdgePtr is not necessary for the algorithm,
	// but it facilitate usage of the result.
	pEdge = ConnectNodes<Point, MeshEdge, MeshEdgePtr>(iPt_[1], iPt_[2], NodeArray, pMem);
	pEdge->pFace[0] = pFace;
	pEdge = ConnectNodes<Point, MeshEdge, MeshEdgePtr>(iPt_[2], iPt_[0], NodeArray, pMem);
	pEdge->pFace[0] = pFace;

	iPt = iPt_[3];

	pPt = NodeArray.Element;

	pEdgeList = &(pPt[iPt3].EdgeList);

	iP = iP3;

	iPt2 = iPt_[0];

	QList<MeshEdgePtr> *pEdgeList2 = &(pPt[iPt2].EdgeList);

	pEdgePtr = pEdgeList2->pFirst->pNext;

	MeshEdgePtr **ppEdgePtr = &(pEdgePtr->pNext);

	MeshEdgePtr **ppEdgePtrNew = &(pEdgeList->pFirst);

	int i;
	MeshEdgePtr *pEdgePtr2, *pEdgePtr3;

	for (i = 0; i < 3; i++)
	{
		RVLMEM_ALLOC_STRUCT(pMem, MeshEdge, pEdge);

		pEdge->iVertex[0] = iPt2;
		pEdge->iVertex[1] = iPt;

		RVLMEM_ALLOC_STRUCT(pMem, MeshEdgePtr, pEdgePtr2);

		RVLQLIST_INSERT_ENTRY2(ppEdgePtr, pEdgePtr2);

		pEdge->pVertexEdgePtr[0] = pEdgePtr2;
		pEdgePtr2->pEdge = pEdge;

		RVLMEM_ALLOC_STRUCT(pMem, MeshEdgePtr, pEdgePtr3);

		RVLQLIST_INSERT_ENTRY2(ppEdgePtrNew, pEdgePtr3);

		pEdge->pVertexEdgePtr[1] = pEdgePtr3;
		pEdgePtr3->pEdge = pEdge;

		RVLMEM_ALLOC_STRUCT(pMem, MESH::Face, pFace);

		RVLQLIST_ADD_ENTRY(pFaceList, pFace);

		pFace->idx = iFace++;

		pFace->flags = 0x00;

		RVLMEM_ALLOC_STRUCT(pMem, MESH::IntPlane, piPlane);

		pFace->piPlane = piPlane;

		iP1 = iPArray + 3 * RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);
		iP2 = iPArray + 3 * iPt2;

		RVLDIF3VECTORS(iP1, iP2, V01);
		RVLDIF3VECTORS(iP, iP2, V02);
		RVLCROSSPRODUCT3(V01, V02, piPlane->N);
		piPlane->d = RVLDOTPRODUCT3(piPlane->N, iP);

		pEdge->pFace[1] = pFace;

		pFace->pFirstEdgePtr = pEdge->pVertexEdgePtr[1];

		pEdgePtr->pEdge->pFace[RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdgePtr->pEdge, iPt2)] = pFace;

		pEdgePtr = pEdgePtr2;

		RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList2, pEdgePtr);

		iPt2 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

		pEdgeList2 = &(pPt[iPt2].EdgeList);

		pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

		ppEdgePtr = &(pEdgePtr->pNext);
	}

	pEdgePtr = pEdgeList->pFirst;

	while (pEdgePtr->pNext)
	{
		pEdgePtr->pNext->pEdge->pFace[0] = pEdgePtr->pEdge->pFace[1];

		pEdgePtr = pEdgePtr->pNext;
	}

	pEdgeList->pFirst->pEdge->pFace[0] = pEdgePtr->pEdge->pFace[1];

	/// Main loop.

	int z;
	bool bUpdateHull;
	uchar flag[2];
	MeshEdgePtr *pEdgePtr0;

	for (iPt = 0; iPt < points.h; iPt++)
	// for (iPt = 0; iPt < 28; iPt++)
	{
		// if (iPt == 27)
		//	int debug = 0;

		if (bInHull[iPt])
			continue;

		iP = iPArray + 3 * iPt;

		pEdgeList = &(pPt[iPt].EdgeList);

		bUpdateHull = false;

		pFace = faceList.pFirst;

		while (pFace)
		{
			// if (pFace->idx == 92 || pFace->idx == 88 || pFace->idx == 84)
			//	int debug = 0;

			if (!(pFace->flags & RVLMESH_FACE_FLAG_REJECTED))
			{
				z = RVLDOTPRODUCT3(pFace->piPlane->N, iP) - pFace->piPlane->d;

				// if (z > 1e-5)
				if (z > 0)
				{
					pFace->flags |= RVLMESH_FACE_FLAG_REJECTED;

					// if (pFace->idx == 14)
					//	int debug = 0;

					bUpdateHull = true;
				}
			}

			pFace = pFace->pNext;
		}

		if (!bUpdateHull)
			continue;

		for (iPt2 = 0; iPt2 < points.h; iPt2++)
		{
			pEdgeList2 = &(pPt[iPt2].EdgeList);

			pEdgePtr = pEdgeList2->pFirst;

			while (pEdgePtr)
			{
				pEdge = pEdgePtr->pEdge;

				flag[0] = (pEdge->pFace[0]->flags & RVLMESH_FACE_FLAG_REJECTED);
				flag[1] = (pEdge->pFace[1]->flags & RVLMESH_FACE_FLAG_REJECTED);

				if (flag[0] ^ flag[1])
				{
					if (flag[RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdge, iPt2)])
						break;
				}

				pEdgePtr = pEdgePtr->pNext;
			}

			if (pEdgePtr)
				break;
		} // for (iPt2 = 0; iPt2 < NodeArray.n; iPt2++)

		if (iPt2 >= points.h)
			continue;

		pEdgePtr0 = pEdgePtr;

		ppEdgePtr = &(pEdgePtr->pNext);

		ppEdgePtrNew = &(pEdgeList->pFirst);

		while (true)
		{
			RVLMEM_ALLOC_STRUCT(pMem, MeshEdge, pEdge);

			pEdge->iVertex[0] = iPt2;
			pEdge->iVertex[1] = iPt;

			RVLMEM_ALLOC_STRUCT(pMem, MeshEdgePtr, pEdgePtr2);

			RVLQLIST_INSERT_ENTRY2(ppEdgePtr, pEdgePtr2);

			pEdge->pVertexEdgePtr[0] = pEdgePtr2;
			pEdgePtr2->pEdge = pEdge;

			RVLMEM_ALLOC_STRUCT(pMem, MeshEdgePtr, pEdgePtr3);

			RVLQLIST_INSERT_ENTRY2(ppEdgePtrNew, pEdgePtr3);

			pEdge->pVertexEdgePtr[1] = pEdgePtr3;
			pEdgePtr3->pEdge = pEdge;

			RVLMEM_ALLOC_STRUCT(pMem, MESH::Face, pFace);

			RVLQLIST_ADD_ENTRY(pFaceList, pFace);

			pFace->idx = iFace++;

			pFace->flags = 0x00;

			RVLMEM_ALLOC_STRUCT(pMem, MESH::IntPlane, piPlane);

			pFace->piPlane = piPlane;

			iP1 = iPArray + 3 * RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);
			iP2 = iPArray + 3 * iPt2;

			RVLDIF3VECTORS(iP1, iP2, V01);
			RVLDIF3VECTORS(iP, iP2, V02);
			RVLCROSSPRODUCT3(V01, V02, piPlane->N);
			piPlane->d = RVLDOTPRODUCT3(piPlane->N, iP);

			pEdge->pFace[1] = pFace;

			pFace->pFirstEdgePtr = pEdge->pVertexEdgePtr[1];

			pEdgePtr->pEdge->pFace[RVLPCSEGMENT_GRAPH_GET_EDGE_SIDE(pEdgePtr->pEdge, iPt2)] = pFace;

			ppEdgePtr = &(pEdgePtr2->pNext);

			if (*ppEdgePtr == NULL)
				ppEdgePtr = &(pEdgeList2->pFirst);

			pEdgePtr = *ppEdgePtr;

			while (pEdgePtr->pEdge->pFace[0]->flags & pEdgePtr->pEdge->pFace[1]->flags & RVLMESH_FACE_FLAG_REJECTED)
			{
				RVLQLIST_REMOVE_ENTRY(pEdgeList2, pEdgePtr, ppEdgePtr);

				if (*ppEdgePtr == NULL)
					ppEdgePtr = &(pEdgeList2->pFirst);

				RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList2, pEdgePtr);
			}

			iPt2 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

			pEdgeList2 = &(pPt[iPt2].EdgeList);

			pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

			if (pEdgePtr == pEdgePtr0)
				break;

			ppEdgePtr = &(pEdgePtr->pNext);
		}

		pEdgePtr = pEdgeList->pFirst;

		while (pEdgePtr->pNext)
		{
			pEdgePtr->pNext->pEdge->pFace[0] = pEdgePtr->pEdge->pFace[1];

			pEdgePtr = pEdgePtr->pNext;
		}

		pEdgeList->pFirst->pEdge->pFace[0] = pEdgePtr->pEdge->pFace[1];

		// Only for debugging purpose!

		// for (iPt2 = 0; iPt2 <= iPt; iPt2++)
		//{
		//	pPt2 = NodeArray.Element + iPt2;

		//	P2 = pPt2->P;

		//	pFace = faceList.pFirst;

		//	while (pFace)
		//	{
		//		if (!(pFace->flags & RVLMESH_FACE_FLAG_REJECTED))
		//		{
		//			z = RVLDOTPRODUCT3(pFace->N, P2) - pFace->d;

		//			if (z > 1e-6)
		//				int debug = 0;
		//		}

		//		pFace = pFace->pNext;
		//	}
		//}

		//
	} // for every point

	///

	// Free memory.

	delete[] bInHull;

	// Copy faces from faceList to faces and compute face parameters.

	RVL_DELETE_ARRAY(faces.Element);

	faces.Element = new MESH::Face *[iFace];

	float s2 = s * s;

	pFace = faceList.pFirst;

	while (pFace)
	{
		if (!(pFace->flags & RVLMESH_FACE_FLAG_REJECTED))
		{
			RVLCOPY3VECTOR(pFace->piPlane->N, pFace->N);
			RVLNORM3(pFace->N, pFace->Area);
			pFace->d = (float)(pFace->piPlane->d);
			pFace->d = pFace->d / (s * pFace->Area) + RVLDOTPRODUCT3(pFace->N, Pc);
			pFace->Area /= (2.0f * s2);

			faces.Element[faces.n++] = pFace;
		}

		pFace = pFace->pNext;
	}

	delete[] iPArray;

	// Create array of valid vertices.

	RVL_DELETE_ARRAY(iValidVertices.Element);

	iValidVertices.Element = new int[NodeArray.n];

	iValidVertices.n = 0;

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		pPt = NodeArray.Element + iPt;

		pEdgePtr = pPt->EdgeList.pFirst;

		if (!pEdgePtr)
			continue;

		if (pEdgePtr->pEdge->pFace[0]->flags & RVLMESH_FACE_FLAG_REJECTED)
			continue;

		iValidVertices.Element[iValidVertices.n++] = iPt;

		pPt->bValid = true;
	}

	// Create array of visible faces.

	iVisibleFaces.Element = new int[faces.n];

	iVisibleFaces.n = 0;

	for (i = 0; i < faces.n; i++)
		if (faces.Element[i]->d < 0.0f || !bDepthImage)
		{
			iVisibleFaces.Element[iVisibleFaces.n++] = i;

			faces.Element[i]->flags |= RVLMESH_FACE_FLAG_VISIBLE;
		}

	//

	return true;
}

void Mesh::VisibleSurface(
	Array2D<float> &N,
	float *&d)
{
	N.w = 3;
	N.h = iVisibleFaces.n;

	int i;
	MeshEdge *pEdge;
	MeshEdgePtr *pEdgePtr;

	for (i = 0; i < iValidVertices.n; i++)
	{
		pEdgePtr = NodeArray.Element[iValidVertices.Element[i]].EdgeList.pFirst;

		while (pEdgePtr)
		{
			pEdge = pEdgePtr->pEdge;

			if (pEdgePtr == pEdge->pVertexEdgePtr[0])
			{
				if ((pEdge->pFace[0]->flags ^ pEdge->pFace[1]->flags) & RVLMESH_FACE_FLAG_VISIBLE)
					N.h++;
			}

			pEdgePtr = pEdgePtr->pNext;
		}
	}

	N.Element = new float[3 * N.h];

	d = new float[N.h];

	float *N_ = N.Element;
	MESH::Face *pFace;
	int iFace;

	for (iFace = 0; iFace < iVisibleFaces.n; iFace++, N_ += 3)
	{
		pFace = faces.Element[iVisibleFaces.Element[iFace]];

		RVLCOPY3VECTOR(pFace->N, N_);

		d[iFace] = pFace->d;
	}

	// N.h = iVisibleFaces.n;

	// return;

	int side;
	float *P1, *P2;
	float dP[3];
	float fTmp;

	for (i = 0; i < iValidVertices.n; i++)
	{
		pEdgePtr = NodeArray.Element[iValidVertices.Element[i]].EdgeList.pFirst;

		while (pEdgePtr)
		{
			pEdge = pEdgePtr->pEdge;

			if (pEdgePtr == pEdge->pVertexEdgePtr[0])
			{
				if ((pEdge->pFace[0]->flags ^ pEdge->pFace[1]->flags) & RVLMESH_FACE_FLAG_VISIBLE)
				{
					side = (pEdge->pFace[0]->flags & RVLMESH_FACE_FLAG_VISIBLE ? 0 : 1);

					P1 = NodeArray.Element[pEdge->iVertex[side]].P;
					P2 = NodeArray.Element[pEdge->iVertex[1 - side]].P;

					RVLDIF3VECTORS(P2, P1, dP);

					RVLCROSSPRODUCT3(P1, dP, N_);

					fTmp = RVLDOTPRODUCT3(N_, N_);

					if (RVLABS(fTmp) >= 1e-10)
					{
						fTmp = sqrt(fTmp);

						RVLSCALE3VECTOR2(N_, fTmp, N_);

						d[iFace] = RVLDOTPRODUCT3(N_, P1);

						// Only for debugging purpose!

						// for (int j = 0; j < NodeArray.n; j++)
						//{
						//	float e = RVLDOTPRODUCT3(N_, NodeArray.Element[j].P) - d[iFace];

						//	if (e < -1e-10)
						//		int debug = 0;
						//}

						//

						iFace++;

						N_ += 3;
					}
				}
			}

			pEdgePtr = pEdgePtr->pNext;
		}
	}
}

void Mesh::VoxelGridFilter(
	float voxelSize,
	Array<OrientedPoint> &outputPts,
	Array3D<int> &grid,
	Box<double> &boundingBox,
	Array<int> &voxels,
	float scale,
	Array<int> *pSubset,
	bool bHollow)
{
	vtkSmartPointer<vtkPoints> points = pPolygonData->GetPoints();
	vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(pPolygonData->GetPointData()->GetNormals());

	int iPt;
	Array<int> *pPoints;

	if (pSubset)
		pPoints = pSubset;
	else
	{
		pPoints = new Array<int>;

		pPoints->Element = new int[points->GetNumberOfPoints()];
		pPoints->n = points->GetNumberOfPoints();

		for (iPt = 0; iPt < pPoints->n; iPt++)
			pPoints->Element[iPt] = iPt;
	}

	double *P = points->GetPoint(pPoints->Element[0]);
	;

	InitBoundingBox<double>(&boundingBox, P);

	for (iPt = 1; iPt < pPoints->n; iPt++)
	{
		P = points->GetPoint(pPoints->Element[iPt]);

		UpdateBoundingBox<double>(&boundingBox, P);
	}

	double boundingBoxSizeX, boundingBoxSizeY, boundingBoxSizeZ;

	BoxSize<double>(&boundingBox, boundingBoxSizeX, boundingBoxSizeY, boundingBoxSizeZ);

	grid.a = (int)ceil(boundingBoxSizeX / voxelSize) + 1;
	grid.b = (int)ceil(boundingBoxSizeY / voxelSize) + 1;
	grid.c = (int)ceil(boundingBoxSizeZ / voxelSize) + 1;

	int nVoxels = grid.a * grid.b * grid.c;

	grid.Element = new int[nVoxels];

	memset(grid.Element, 0xff, nVoxels * sizeof(int));

	voxels.Element = new int[points->GetNumberOfPoints()];
	voxels.n = 0;

	int iVoxel, i, j, k, iPt_, iiPt;
	double *P_;
	double dP[3], dP_[3], Pc[3];
	float N[3];

	for (iiPt = 0; iiPt < pPoints->n; iiPt++)
	{
		iPt = pPoints->Element[iiPt];

		normals->GetTypedTuple(iPt, N);

		if (N[0] != N[0])
			continue;

		P = points->GetPoint(iPt);

		i = (int)round((P[0] - boundingBox.minx) / voxelSize);
		j = (int)round((P[1] - boundingBox.miny) / voxelSize);
		k = (int)round((P[2] - boundingBox.minz) / voxelSize);

		iVoxel = RVL3DARRAY_INDEX(grid, i, j, k);

		iPt_ = grid.Element[iVoxel];

		if (iPt_ >= 0)
		{
			Pc[0] = boundingBox.minx + (double)i * voxelSize;
			Pc[1] = boundingBox.miny + (double)j * voxelSize;
			Pc[2] = boundingBox.minz + (double)k * voxelSize;

			RVLDIF3VECTORS(P, Pc, dP);

			P_ = points->GetPoint(iPt_);

			RVLDIF3VECTORS(P_, Pc, dP_);

			if (RVLDOTPRODUCT3(dP, dP) < RVLDOTPRODUCT3(dP_, dP_))
				grid.Element[iVoxel] = iPt;
		}
		else
		{
			grid.Element[iVoxel] = iPt;

			voxels.Element[voxels.n++] = iVoxel;
		}
	}

	outputPts.n = (bHollow ? 2 * voxels.n : voxels.n);

	outputPts.Element = new OrientedPoint[outputPts.n];

	OrientedPoint *pPt, *pPt_;
	float fTmp;

	for (i = 0; i < voxels.n; i++)
	{
		iPt = grid.Element[voxels.Element[i]];

		P = points->GetPoint(iPt);

		pPt = outputPts.Element + i;

		RVLSCALE3VECTOR(P, scale, pPt->P);

		normals->GetTypedTuple(iPt, pPt->N);

		RVLNORM3(pPt->N, fTmp);

		if (bHollow)
		{
			pPt_ = outputPts.Element + voxels.n + i;

			RVLCOPY3VECTOR(pPt->P, pPt_->P);

			RVLNEGVECT3(pPt->N, pPt_->N);
		}
	}

	if (pSubset == NULL)
	{
		delete[] pPoints->Element;
		delete pPoints;
	}
}

void Mesh::VoxelGridFilter(
	float voxelSize,
	float normalProxThr,
	Array<OrientedPoint> &outputPts,
	Array3D<QList<QLIST::Index>> &grid,
	Box<double> &boundingBox,
	Array<int> &voxels,
	float scale,
	Array<int> *pSubset)
{
	vtkSmartPointer<vtkPoints> points = pPolygonData->GetPoints();
	vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(pPolygonData->GetPointData()->GetNormals());

	int iPt;
	Array<int> *pPoints;

	if (pSubset)
		pPoints = pSubset;
	else
	{
		pPoints = new Array<int>;

		pPoints->Element = new int[points->GetNumberOfPoints()];
		pPoints->n = points->GetNumberOfPoints();

		for (iPt = 0; iPt < pPoints->n; iPt++)
			pPoints->Element[iPt] = iPt;
	}

	double *P = points->GetPoint(pPoints->Element[0]);
	;

	InitBoundingBox<double>(&boundingBox, P);

	for (iPt = 1; iPt < pPoints->n; iPt++)
	{
		P = points->GetPoint(pPoints->Element[iPt]);

		UpdateBoundingBox<double>(&boundingBox, P);
	}

	double boundingBoxSizeX, boundingBoxSizeY, boundingBoxSizeZ;

	BoxSize<double>(&boundingBox, boundingBoxSizeX, boundingBoxSizeY, boundingBoxSizeZ);

	grid.a = (int)ceil(boundingBoxSizeX / voxelSize) + 1;
	grid.b = (int)ceil(boundingBoxSizeY / voxelSize) + 1;
	grid.c = (int)ceil(boundingBoxSizeZ / voxelSize) + 1;

	int nVoxels = grid.a * grid.b * grid.c;

	grid.Element = new QList<QLIST::Index>[nVoxels];

	int iVoxel;
	QList<QLIST::Index> *pPtIdxList;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
	{
		pPtIdxList = grid.Element + iVoxel;

		RVLQLIST_INIT(pPtIdxList);
	}

	QLIST::Index *ptIdxArray = new QLIST::Index[points->GetNumberOfPoints()];

	QLIST::Index *pNewPtIdx = ptIdxArray;

	voxels.Element = new int[points->GetNumberOfPoints()];
	voxels.n = 0;

	int i, j, k, iPt_, iiPt;
	float eN;
	double *P_;
	double dP[3], dP_[3], Pc[3];
	float N[3], N_[3];
	QLIST::Index *pPtIdx;

	for (iiPt = 0; iiPt < pPoints->n; iiPt++)
	{
		iPt = pPoints->Element[iiPt];

		normals->GetTypedTuple(iPt, N);

		if (N[0] != N[0])
			continue;

		P = points->GetPoint(iPt);

		i = (int)round((P[0] - boundingBox.minx) / voxelSize);
		j = (int)round((P[1] - boundingBox.miny) / voxelSize);
		k = (int)round((P[2] - boundingBox.minz) / voxelSize);

		iVoxel = RVL3DARRAY_INDEX(grid, i, j, k);

		pPtIdxList = grid.Element + iVoxel;

		pPtIdx = pPtIdxList->pFirst;

		if (pPtIdx)
		{
			Pc[0] = boundingBox.minx + (double)i * voxelSize;
			Pc[1] = boundingBox.miny + (double)j * voxelSize;
			Pc[2] = boundingBox.minz + (double)k * voxelSize;

			while (pPtIdx)
			{
				iPt_ = pPtIdx->Idx;

				normals->GetTypedTuple(iPt_, N_);

				eN = RVLDOTPRODUCT3(N, N_);

				if (eN >= normalProxThr)
				{
					RVLDIF3VECTORS(P, Pc, dP);

					P_ = points->GetPoint(iPt_);

					RVLDIF3VECTORS(P_, Pc, dP_);

					if (RVLDOTPRODUCT3(dP, dP) < RVLDOTPRODUCT3(dP_, dP_))
					{
						pPtIdx->Idx = iPt;

						break;
					}
				}

				pPtIdx = pPtIdx->pNext;
			}
		}
		else
			voxels.Element[voxels.n++] = iVoxel;

		if (pPtIdx == NULL)
		{
			pNewPtIdx->Idx = iPt;

			pPtIdxList = grid.Element + iVoxel;

			RVLQLIST_ADD_ENTRY(pPtIdxList, pNewPtIdx);

			pNewPtIdx++;
		}
	}

	outputPts.n = pNewPtIdx - ptIdxArray;

	outputPts.Element = new OrientedPoint[outputPts.n];

	OrientedPoint *pPt, *pPt_;
	float fTmp;

	for (iPt = 0; iPt < outputPts.n; iPt++)
	{
		iPt_ = ptIdxArray[iPt].Idx;

		P = points->GetPoint(iPt_);

		pPt = outputPts.Element + iPt;

		RVLSCALE3VECTOR(P, scale, pPt->P);

		normals->GetTypedTuple(iPt_, pPt->N);

		RVLNORM3(pPt->N, fTmp);
	}

	delete[] ptIdxArray;
	if (pSubset == NULL)
	{
		delete[] pPoints->Element;
		delete pPoints;
	}
}

void Mesh::DistanceTransform(
	Array3D<int> grid,
	Box<double> boundingBox,
	Array<int> voxels)
{
}

#ifdef NEVER

void Mesh::CreateVTKPolyData(bool bVisibleFacesOnly)
{
	pPolygonData = vtkSmartPointer<vtkPolyData>::New();

	// Create points from NodeArray.

	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	int iPt;
	float *P;

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		P = NodeArray.Element[iPt].P;

		points->InsertNextPoint(P);
	}

	pPolygonData->SetPoints(points);

	// Create faces from faceList.

	bool *bVisible = NULL;

	int i;

	if (bVisibleFacesOnly)
	{
		bVisible = new bool[faces.n];

		memset(bVisible, 0, faces.n * sizeof(bool));

		for (i = 0; i < iVisibleFaces.n; i++)
			bVisible[iVisibleFaces.Element[i]] = true;
	}

	vtkSmartPointer<vtkCellArray> triangles =
		vtkSmartPointer<vtkCellArray>::New();

	MESH::Face *pFace;
	int iPt_;
	vtkSmartPointer<vtkTriangle> triangle;
	MeshEdgePtr *pEdgePtr, *pEdgePtr0, *pEdgePtr_;
	QList<MeshEdgePtr> *pEdgeList;

	for (i = 0; i < faces.n; i++)
	{
		if (bVisibleFacesOnly)
			if (!bVisible[i])
				continue;

		pFace = faces.Element[i];

		triangle = vtkSmartPointer<vtkTriangle>::New();

		iPt_ = 0;

		pEdgePtr = pFace->pFirstEdgePtr;

		pEdgePtr0 = pEdgePtr;

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

		do
		{
			triangle->GetPointIds()->SetId(iPt_++, iPt);

			pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

			pEdgeList = &(NodeArray.Element[iPt].EdgeList);

			pEdgePtr_ = pEdgeList->pFirst;

			while (pEdgePtr_->pNext != pEdgePtr && pEdgePtr_->pNext != NULL)
				pEdgePtr_ = pEdgePtr_->pNext;

			pEdgePtr = pEdgePtr_;
		} while (pEdgePtr != pEdgePtr0);

		triangles->InsertNextCell(triangle);
	}

	pPolygonData->SetPolys(triangles);

	RVL_DELETE_ARRAY(bVisible);
}

#endif

void Mesh::CreateVTKPolyData(
	float *R,
	float *t,
	bool bVisibleFacesOnly)
{
	pPolygonData = vtkSmartPointer<vtkPolyData>::New();

	// Create points from NodeArray.

	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	float R_[9];

	if (R)
	{
		RVLCOPYMX3X3(R, R_);
	}
	else
	{
		RVLUNITMX3(R_);
	}

	float t_[3];

	if (t)
	{
		RVLCOPY3VECTOR(t, t_);
	}
	else
		RVLNULL3VECTOR(t_);

	int iPt;
	float *P;
	float P_[3];

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		P = NodeArray.Element[iPt].P;

		RVLTRANSF3(P, R_, t_, P_);

		points->InsertNextPoint(P_);
	}

	pPolygonData->SetPoints(points);

	// Create faces from faceList.

	bool *bVisible = NULL;

	int i;

	if (bVisibleFacesOnly)
	{
		bVisible = new bool[faces.n];

		memset(bVisible, 0, faces.n * sizeof(bool));

		for (i = 0; i < iVisibleFaces.n; i++)
			bVisible[iVisibleFaces.Element[i]] = true;
	}

	vtkSmartPointer<vtkCellArray> polygons =
		vtkSmartPointer<vtkCellArray>::New();

	MESH::Face *pFace;
	int iPt_, nPts;
	vtkSmartPointer<vtkPolygon> polygon;
	MeshEdgePtr *pEdgePtr, *pEdgePtr0, *pEdgePtr_;
	QList<MeshEdgePtr> *pEdgeList;

	for (i = 0; i < faces.n; i++)
	{
		if (bVisibleFacesOnly)
			if (!bVisible[i])
				continue;

		pFace = faces.Element[i];

		pEdgePtr = pFace->pFirstEdgePtr;

		pEdgePtr0 = pEdgePtr;

		nPts = 0;

		do
		{
			nPts++;

			pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

			pEdgeList = &(NodeArray.Element[iPt].EdgeList);

			pEdgePtr_ = pEdgeList->pFirst;

			while (pEdgePtr_->pNext != pEdgePtr && pEdgePtr_->pNext != NULL)
				pEdgePtr_ = pEdgePtr_->pNext;

			pEdgePtr = pEdgePtr_;
		} while (pEdgePtr != pEdgePtr0);

		polygon = vtkSmartPointer<vtkPolygon>::New();

		polygon->GetPointIds()->SetNumberOfIds(nPts);

		iPt_ = 0;

		pEdgePtr = pFace->pFirstEdgePtr;

		pEdgePtr0 = pEdgePtr;

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

		do
		{
			polygon->GetPointIds()->SetId(iPt_++, iPt);

			pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

			pEdgeList = &(NodeArray.Element[iPt].EdgeList);

			pEdgePtr_ = pEdgeList->pFirst;

			while (pEdgePtr_->pNext != pEdgePtr && pEdgePtr_->pNext != NULL)
				pEdgePtr_ = pEdgePtr_->pNext;

			pEdgePtr = pEdgePtr_;
		} while (pEdgePtr != pEdgePtr0);

		polygons->InsertNextCell(polygon);
	}

	pPolygonData->SetPolys(polygons);

	RVL_DELETE_ARRAY(bVisible);
}

void Mesh::Transform(
	float *R,
	float *t)
{
	int iPt;
	float *PSrc;
	float PTgt[3];

	for (iPt = 0; iPt < NodeArray.n; iPt++)
	{
		PSrc = NodeArray.Element[iPt].P;

		RVLTRANSF3(PSrc, R, t, PTgt);

		RVLCOPY3VECTOR(PTgt, PSrc);
	}
}

int Mesh::FurthestPoint(
	float *P,
	QList<QLIST::Index2> ptList)
{
	float maxDist = 0.0f;
	float dist;
	Point *pPt;
	float dP[3];
	QLIST::Index2 *pPtIdx = ptList.pFirst;
	int iFurthestPt;
	while (pPtIdx)
	{
		pPt = NodeArray.Element + pPtIdx->Idx;
		RVLDIF3VECTORS(pPt->P, P, dP);
		dist = RVLDOTPRODUCT3(dP, dP);
		if (dist > maxDist)
		{
			maxDist = dist;
			iFurthestPt = pPtIdx->Idx;
		}
		pPtIdx = pPtIdx->pNext;
	}
	return iFurthestPt;
}

int Mesh::FurthestPoint(
	float *P,
	Array<MeshEdgePtr *> ptArray,
	float *V)
{
	float maxDist = 0.0f;
	float dist;
	Point *pPt;
	float dP[3], P_[3];
	int iPt;
	int iFurthestPt;
	float fTmp;
	for (int i = 0; i < ptArray.n; i++)
	{
		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(ptArray.Element[i]);
		pPt = NodeArray.Element + iPt;
		RVLDIF3VECTORS(pPt->P, P, dP);
		if (V)
		{
			fTmp = RVLDOTPRODUCT3(V, dP);
			RVLSCALE3VECTOR(V, fTmp, P_);
			RVLDIF3VECTORS(pPt->P, P_, dP);
		}
		dist = RVLDOTPRODUCT3(dP, dP);
		if (dist > maxDist)
		{
			maxDist = dist;
			iFurthestPt = iPt;
		}
	}
	return iFurthestPt;
}

void Mesh::GetDepth(Array2D<short> &depthImg)
{
	depthImg.w = width;
	depthImg.h = height;
	int nPix = depthImg.w * depthImg.h;
	depthImg.Element = new short[nPix];
	short depth;
	for (int iPix = 0; iPix < nPix; iPix++)
		depthImg.Element[iPix] = (short)round(NodeArray.Element[iPix].P[2] * 1000.0f);
}

void Mesh::LoadLabels(char *labelFileName)
{
	FILE *fp = fopen(labelFileName, "r");

	if (fp == NULL)
		return;

	int i;

	for (i = 0; i < NodeArray.n; i++)
		fscanf(fp, "%d\n", &(NodeArray.Element[i].label));

	fclose(fp);

	bLabels = true;
}

void Mesh::SaveConvexHullVertices(FILE *fp)
{
	int i;
	float *P;

	for (i = 0; i < iValidVertices.n; i++)
	{
		P = NodeArray.Element[iValidVertices.Element[i]].P;

		fprintf(fp, "%f\t%f\t%f\n", P[0], P[1], P[2]);
	}
}

void Mesh::GetBGR(cv::Mat &BGR)
{
	BGR.create(height, width, CV_8UC3);
	int nPix = width * height;
	uchar *tgtPix = BGR.data;
	uchar *srcPix;
	for (int iPix = 0; iPix < nPix; iPix++, tgtPix += 3)
	{
		srcPix = NodeArray.Element[iPix].RGB;
		RVLSET3VECTOR(tgtPix, srcPix[2], srcPix[1], srcPix[0]);
	}
}

void RVL::MESH::TSDF(
	vtkSmartPointer<vtkPolyData> pPolygonData,
	float voxelSize,
	int border,
	Array3D<Voxel> &volume,
	float *P0,
	Box<float> &boundingBox,
	Array<int> &zeroDistanceVoxelArray,
	QLIST::Index *&PtMem)
{
	int nPts = pPolygonData->GetNumberOfPoints();

	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pPolygonData->GetPoints()->GetData());
	if (pointData == NULL)
		return;

	// volume <- empty 3D voxel array with voxel size specified by voxelSize.
	// It is larger than the bounding box of pMesh for sampleVoxelDistance + 1 on each side.

	MESH::BoundingBox(pPolygonData, &boundingBox);

	int nx = (int)ceil(0.5f * (boundingBox.maxx - boundingBox.minx) / voxelSize) + border;
	int ny = (int)ceil(0.5f * (boundingBox.maxy - boundingBox.miny) / voxelSize) + border;
	int nz = (int)ceil(0.5f * (boundingBox.maxz - boundingBox.minz) / voxelSize) + border;

	float a = voxelSize * (float)nx;
	float b = voxelSize * (float)ny;
	float c = voxelSize * (float)nz;

	float center[3];

	BoxCenter<float>(&boundingBox, center);

	Box<float> box;

	box.minx = center[0] - a;
	box.miny = center[1] - b;
	box.minz = center[2] - c;
	box.maxx = center[0] + a;
	box.maxy = center[1] + b;
	box.maxz = center[2] + c;

	float halfVoxelSize = 0.5f * voxelSize;

	P0[0] = box.minx + halfVoxelSize;
	P0[1] = box.miny + halfVoxelSize;
	P0[2] = box.minz + halfVoxelSize;

	volume.a = 2 * nx;
	volume.b = 2 * ny;
	volume.c = 2 * nz;

	int nVoxels = volume.a * volume.b * volume.c;

	volume.Element = new Voxel[nVoxels];

	// Assign mesh points to volume voxels.
	// Set voxelDistance field of all voxels to -1.

	int i;
	QList<QLIST::Index> *pPtList;
	Voxel *pVoxel;

	for (i = 0; i < nVoxels; i++)
	{
		pVoxel = volume.Element + i;

		pPtList = &(pVoxel->PtList);

		RVLQLIST_INIT(pPtList);

		pVoxel->voxelDistance = -1;
	}

	PtMem = new QLIST::Index[nPts];

	QLIST::Index *pPtIdx = PtMem;

	float P[3];
	int iPt;
	int j, k;

	for (iPt = 0; iPt < nPts; iPt++)
	{
		// P = pMesh->NodeArray.Element[iPt].P;
		pointData->GetTypedTuple(iPt, P);

		i = (int)floor((P[0] - box.minx) / voxelSize);
		j = (int)floor((P[1] - box.miny) / voxelSize);
		k = (int)floor((P[2] - box.minz) / voxelSize);

		pVoxel = RVL3DARRAY_ELEMENT(volume, i, j, k);

		pPtList = &(pVoxel->PtList);

		pPtIdx->Idx = iPt;

		RVLQLIST_ADD_ENTRY(pPtList, pPtIdx);

		pPtIdx++;
	}

	// Assign distance function value to all voxels outside pMesh.

	int *RGBuff = new int[nVoxels];

	int *pPut = RGBuff;
	int *pFetch = RGBuff;

	zeroDistanceVoxelArray.Element = new int[nVoxels];

	zeroDistanceVoxelArray.n = 0;

	*(pPut++) = 0;

	int dijk[][3] = {
		{-1, 0, 0},
		{1, 0, 0},
		{0, -1, 0},
		{0, 1, 0},
		{0, 0, -1},
		{0, 0, 1}};

	int iVoxel, iVoxel_;
	int i_, j_, k_, l;

	while (pPut > pFetch)
	{
		iVoxel = (*pFetch++);

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

				if (pVoxel->voxelDistance >= 0)
					continue;

				if (pVoxel->PtList.pFirst)
				{
					pVoxel->voxelDistance = 0;

					zeroDistanceVoxelArray.Element[zeroDistanceVoxelArray.n++] = iVoxel_;
				}
				else
				{
					pVoxel->voxelDistance = 1;

					*(pPut++) = iVoxel_;
				}
			}
		}
	}

	delete[] RGBuff;
}

void RVL::MESH::TSDF2(
	Tetrahedrons &tetrahedrons,
	float resolution,
	float distanceValuePerVoxel,
	Array3D<float> &TSDF,
	float *P0,
	Box<float> &boundingBox,
	float &voxelSize,
	bool bRoom)
{
	TriangleTDF(tetrahedrons.triangles, tetrahedrons.vertices, resolution, distanceValuePerVoxel, TSDF, P0, boundingBox, voxelSize);

	//// volume <- empty 3D voxel array with voxel size specified by voxelSize.
	//// It is larger than the bounding box of pMesh for sampleVoxelDistance + 1 on each side.

	// InitBoundingBox<float>(&boundingBox, tetrahedrons.vertices.Element[0].P);

	// int iPt;

	// for (iPt = 1; iPt < tetrahedrons.vertices.n; iPt++)
	//	UpdateBoundingBox<float>(&boundingBox, tetrahedrons.vertices.Element[iPt].P);

	// float boundingBoxSizeX, boundingBoxSizeY, boundingBoxSizeZ;

	// BoxSize<float>(&boundingBox, boundingBoxSizeX, boundingBoxSizeY, boundingBoxSizeZ);

	////float boundingBoxSize = BoxSize<float>(&boundingBox);

	////voxelSize = resolution * boundingBoxSize;

	// voxelSize = pow(resolution * resolution * resolution * boundingBoxSizeX * boundingBoxSizeY * boundingBoxSizeZ, 1.0f / 3.0f);

	// float fTriangleBorder = 1.0f / distanceValuePerVoxel;

	// float kDistanceValue = distanceValuePerVoxel / voxelSize;
	// float kDistanceValue2 = kDistanceValue * kDistanceValue;

	// int triangleBorder = ceil(fTriangleBorder) + 1;

	// fTriangleBorder *= voxelSize;

	// int nx = (int)floor(0.5f * boundingBoxSizeX / voxelSize) + triangleBorder;
	// int ny = (int)floor(0.5f * boundingBoxSizeY / voxelSize) + triangleBorder;
	// int nz = (int)floor(0.5f * boundingBoxSizeZ / voxelSize) + triangleBorder;

	// float a = voxelSize * (float)nx;
	// float b = voxelSize * (float)ny;
	// float c = voxelSize * (float)nz;

	// float center[3];

	// BoxCenter<float>(&boundingBox, center);

	// Box<float> box;

	// box.minx = center[0] - a;
	// box.miny = center[1] - b;
	// box.minz = center[2] - c;
	// box.maxx = center[0] + a;
	// box.maxy = center[1] + b;
	// box.maxz = center[2] + c;

	// P0[0] = box.minx;
	// P0[1] = box.miny;
	// P0[2] = box.minz;

	// TSDF.a = 2 * nx;
	// TSDF.b = 2 * ny;
	// TSDF.c = 2 * nz;

	int nVoxels = TSDF.a * TSDF.b * TSDF.c;

	// TSDF.Element = new float[nVoxels];

	int iVoxel;

	// for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
	//	TSDF.Element[iVoxel] = 1.0f;

	////// Compute TSDF in the vicinity of the object surface.

	// Array<int> iPositiveY;
	// int iPositiveYMem[2];
	// iPositiveY.Element = iPositiveYMem;

	// int iCommonPt[3] = {1, 0, 2};

	// int iTriangle;
	float P[4][3];
	int i, j;
	float X[3][3];
	// float Y[3][3];
	// float Z[3];
	float d;
	// float dy[3];
	float fTmp;
	// Box<float> triangleBox;
	// Box<int> triangleVoxelBox;
	int ix, iy, iz;
	float P_[3];
	// float x, y, xyDist2, z, zDist, dist2, dist_, dist2_;
	// float positiveY[2];
	// float V3Tmp[3];
	float *P__;
	// float L[3];
	// MESH::Triangle *pTriangle;

	// for (iTriangle = 0; iTriangle < tetrahedrons.triangles.n; iTriangle++)
	//{
	//	pTriangle = tetrahedrons.triangles.Element + iTriangle;

	//	if (!pTriangle->bSurface)
	//		continue;

	//	TriangleTDF(pTriangle, tetrahedrons.vertices, voxelSize, distanceValuePerVoxel, box, TSDF);

	//// P[i], i = 0,1,2 <- i-th vertex of pTriangle

	// for (i = 0; i < 3; i++)
	//{
	//	P__ = tetrahedrons.vertices.Element[pTriangle->iVertex[i]].P;

	//	RVLCOPY3VECTOR(P__, P[i]);
	//}

	//// X[i] <- unit(P[i+1] - P[i]), L[i] <- length of the line (P[i+1], P[i]), i = 0,1,2

	// for (i = 0; i < 3; i++)
	//{
	//	j = (i + 1) % 3;

	//	RVLDIF3VECTORS(P[j], P[i], X[i]);
	//	RVLNORM3(X[i], L[i]);
	//}

	//// Z <- -unit(X[0] x X[2])

	// RVLCROSSPRODUCT3(X[0], X[2], Z);
	// RVLNEGVECT3(Z, Z);
	// RVLNORM3(Z, fTmp);

	//// d <- Z' * P[0]

	// d = RVLDOTPRODUCT3(Z, P[0]);

	//// Y[i] <- X[i] x Z, dy[i] <- Y[i]' * P[i], i = 0,1,2

	// for (i = 0; i < 3; i++)
	//{
	//	RVLCROSSPRODUCT3(X[i], Z, Y[i]);
	//	RVLNORM3(Y[i], fTmp);
	//	dy[i] = RVLDOTPRODUCT3(Y[i], P[i]);
	// }

	//// triangleBox <- bounding box of pTriangle expanded for fTriangleBorder

	// InitBoundingBox<float>(&triangleBox, P[0]);
	// UpdateBoundingBox<float>(&triangleBox, P[1]);
	// UpdateBoundingBox<float>(&triangleBox, P[2]);

	// ExpandBox<float>(&triangleBox, fTriangleBorder);

	//// triangleVoxelBox <- box of voxels containing triangleBox
	//
	// triangleVoxelBox.minx = floor((triangleBox.minx - P0[0]) / voxelSize);
	// triangleVoxelBox.maxx = ceil((triangleBox.maxx - P0[0]) / voxelSize);
	// triangleVoxelBox.miny = floor((triangleBox.miny - P0[1]) / voxelSize);
	// triangleVoxelBox.maxy = ceil((triangleBox.maxy - P0[1]) / voxelSize);
	// triangleVoxelBox.minz = floor((triangleBox.minz - P0[2]) / voxelSize);
	// triangleVoxelBox.maxz = ceil((triangleBox.maxz - P0[2]) / voxelSize);

	///// Compute distance value for every voxel in the vicinity of pTriangle.

	// for (iz = triangleVoxelBox.minz; iz <= triangleVoxelBox.maxz; iz++)
	//	for (iy = triangleVoxelBox.miny; iy <= triangleVoxelBox.maxy; iy++)
	//		for (ix = triangleVoxelBox.minx; ix <= triangleVoxelBox.maxx; ix++)
	//		{
	//			//if (ix == 3 && iy == 3 && iz == 104)
	//			//	int debug = 0;

	//			// iVoxel <- index of the voxel with indices (ix, iy, iz)

	//			iVoxel = RVL3DARRAY_INDEX(TSDF, ix, iy, iz);

	//			if (iVoxel >= nVoxels)
	//				int debug = 0;

	//			// P_ <- coordinates of the voxel with indices (ix, iy, iz)

	//			P_[0] = (float)ix * voxelSize + P0[0];
	//			P_[1] = (float)iy * voxelSize + P0[1];
	//			P_[2] = (float)iz * voxelSize + P0[2];

	//			// z <- Z' * P_ - d

	//			z = (RVLDOTPRODUCT3(Z, P_) - d);

	//			// zDist <- kDistanceValue * |z|

	//			zDist = kDistanceValue * RVLABS(z);

	//			// If zDist < 1.0f, then compute the distance value of voxel (ix, iy, iz) and store it in TSDF.Element[iVoxel].

	//			if (zDist < 1.0f)
	//			{
	//				iPositiveY.n = 0;

	//				for (i = 0; i < 3; i++)
	//				{
	//					y = RVLDOTPRODUCT3(Y[i], P_) - dy[i];

	//					if (y > 0.0f)
	//					{
	//						iPositiveY.Element[iPositiveY.n] = i;
	//						positiveY[iPositiveY.n] = y;
	//						iPositiveY.n++;
	//					}
	//				}

	//				if (iPositiveY.n == 0)
	//					xyDist2 = 0.0f;
	//				else
	//				{
	//					RVLSCALE3VECTOR(Z, z, V3Tmp);
	//					RVLDIF3VECTORS(P_, V3Tmp, V3Tmp);

	//					if (iPositiveY.n == 1)
	//					{
	//						iPt = iPositiveY.Element[0];
	//						P__ = P[iPt];
	//						RVLDIF3VECTORS(V3Tmp, P__, V3Tmp);

	//						x = RVLDOTPRODUCT3(X[iPt], V3Tmp);

	//						y = positiveY[0];

	//						if (x < 0.0f)
	//							xyDist2 = kDistanceValue2 * (x * x + y * y);
	//						else if (x < L[iPt])
	//							xyDist2 = kDistanceValue2 * y * y;
	//						else
	//						{
	//							x -= L[iPt];
	//							xyDist2 = kDistanceValue2 * (x * x + y * y);
	//						}
	//					}
	//					else
	//					{
	//						iPt = iCommonPt[iPositiveY.Element[0] + iPositiveY.Element[1] - 1];
	//						P__ = P[iPt];
	//						RVLDIF3VECTORS(V3Tmp, P__, V3Tmp);
	//						xyDist2 = kDistanceValue2 * RVLDOTPRODUCT3(V3Tmp, V3Tmp);
	//					}
	//				}

	//				dist2 = xyDist2 + zDist * zDist;

	//				//dist2 = zDist * zDist;

	//				if (dist2 < 1.0f)
	//				{
	//					dist_ = TSDF.Element[iVoxel];

	//					dist2_ = dist_ * dist_;

	//					if (dist2 < dist2_)
	//						TSDF.Element[iVoxel] = sqrt(dist2);
	//				}
	//			}	// if (zDist < 1.0f)
	//		}	// for every voxel
	///// END: Compute distance function value for every voxel in the vicinity of pTriangle.
	//}	// for every triangle

	//// END: Compute TSDF in the vicinity of the object surface.

	if (bRoom)
		for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
			TSDF.Element[iVoxel] = -TSDF.Element[iVoxel];

	// Compute distance signs of all voxels.

	int k, l;
	int iTetrahedron;
	Box<float> tetrahedronBox;
	Box<int> tetrahedronVoxelBox;
	MESH::Tetrahedron *pTetrahedron;
	float ZTe[4][3];
	float dTe[4];

	for (iTetrahedron = 0; iTetrahedron < tetrahedrons.tetrahedrons.n; iTetrahedron++)
	{
		pTetrahedron = tetrahedrons.tetrahedrons.Element + iTetrahedron;

		if (bRoom)
		{
			if (pTetrahedron->bValid)
				continue;
		}
		else
		{
			if (!(pTetrahedron->bValid))
				continue;
		}

		// P[i], i = 0,1,2,3 <- i-th vertex of pTetrahedron

		for (i = 0; i < 4; i++)
		{
			P__ = tetrahedrons.vertices.Element[pTetrahedron->iVertex[i]].P;

			RVLCOPY3VECTOR(P__, P[i]);
		}

		// ZTe[i], dTe[i] <- normal and offset of the i-th side of pTetrahedron, i = 0,1,2,3

		for (i = 0; i < 4; i++)
		{
			j = (i + 1) % 4;
			k = (i + 2) % 4;
			l = (i + 3) % 4;

			RVLDIF3VECTORS(P[j], P[i], X[0]);
			RVLDIF3VECTORS(P[k], P[i], X[1]);
			RVLDIF3VECTORS(P[l], P[i], X[2]);
			RVLCROSSPRODUCT3(X[0], X[1], ZTe[i]);
			RVLNORM3(ZTe[i], fTmp);
			if (RVLDOTPRODUCT3(ZTe[i], X[2]) > 0.0f)
			{
				RVLNEGVECT3(ZTe[i], ZTe[i]);
			}
			dTe[i] = RVLDOTPRODUCT3(ZTe[i], P[i]);
		}

		// tetrahedronBox <- bounding box of pTetrahedron

		InitBoundingBox<float>(&tetrahedronBox, P[0]);
		UpdateBoundingBox<float>(&tetrahedronBox, P[1]);
		UpdateBoundingBox<float>(&tetrahedronBox, P[2]);
		UpdateBoundingBox<float>(&tetrahedronBox, P[3]);

		// tetrahedronVoxelBox <- box of voxels containing tetrahedronBox

		tetrahedronVoxelBox.minx = floor((tetrahedronBox.minx - P0[0]) / voxelSize);
		tetrahedronVoxelBox.maxx = ceil((tetrahedronBox.maxx - P0[0]) / voxelSize);
		tetrahedronVoxelBox.miny = floor((tetrahedronBox.miny - P0[1]) / voxelSize);
		tetrahedronVoxelBox.maxy = ceil((tetrahedronBox.maxy - P0[1]) / voxelSize);
		tetrahedronVoxelBox.minz = floor((tetrahedronBox.minz - P0[2]) / voxelSize);
		tetrahedronVoxelBox.maxz = ceil((tetrahedronBox.maxz - P0[2]) / voxelSize);

		/// Compute distance sign for every voxel in pTetrahedron.

		for (iz = tetrahedronVoxelBox.minz; iz <= tetrahedronVoxelBox.maxz; iz++)
			for (iy = tetrahedronVoxelBox.miny; iy <= tetrahedronVoxelBox.maxy; iy++)
				for (ix = tetrahedronVoxelBox.minx; ix <= tetrahedronVoxelBox.maxx; ix++)
				{
					// iVoxel <- index of the voxel with indices (ix, iy, iz)

					iVoxel = RVL3DARRAY_INDEX(TSDF, ix, iy, iz);

					// P_ <- coordinates of the voxel with indices (ix, iy, iz)

					P_[0] = (float)ix * voxelSize + P0[0];
					P_[1] = (float)iy * voxelSize + P0[1];
					P_[2] = (float)iz * voxelSize + P0[2];

					// Is P_ contained inside pTetrahedron?

					for (i = 0; i < 4; i++)
					{
						d = RVLDOTPRODUCT3(ZTe[i], P_) - dTe[i];

						if (d >= 0.0f)
							break;
					}

					// If P_ contained inside pTetrahedron, then TSDF.Element[iVoxel] has a negative value.

					if (i >= 4) // If P_ contained inside pTetrahedron
						TSDF.Element[iVoxel] = -TSDF.Element[iVoxel];
				}
	} // for every tetrahedron

	// Only for debugging purpose!

	// for (iz = triangleVoxelBox.minz; iz <= triangleVoxelBox.maxz; iz++)
	//	for (iy = triangleVoxelBox.miny; iy <= triangleVoxelBox.maxy; iy++)
	//		for (ix = triangleVoxelBox.minx; ix <= triangleVoxelBox.maxx; ix++)
	//		{
	//			iVoxel = RVL3DARRAY_INDEX(TSDF, ix, iy, iz);

	//			if (TSDF.Element[iVoxel] > 1.5f)
	//				continue;

	//			P_[0] = (float)ix * voxelSize + P0[0];
	//			P_[1] = (float)iy * voxelSize + P0[1];
	//			P_[2] = (float)iz * voxelSize + P0[2];

	//			triangles->InitTraversal();

	//			for (iTriangle = 0; iTriangle < nTriangles; iTriangle++)
	//			{
	//				triangles->GetNextCell(nTrianglePts, trianglePtsIds);

	//				for (i = 0; i < 3; i++)
	//					pointData->GetTupleValue(trianglePtsIds[i], P[i]);

	//				for (i = 0; i < 3; i++)
	//				{
	//					j = (i + 1) % 3;

	//					RVLDIF3VECTORS(P[j], P[i], X[i]);
	//					RVLNORM3(X[i], L[i]);
	//				}

	//				RVLCROSSPRODUCT3(X[0], X[2], Z);
	//				RVLNEGVECT3(Z, Z);
	//				RVLNORM3(Z, fTmp);
	//				d = RVLDOTPRODUCT3(Z, P[0]);

	//				z = (RVLDOTPRODUCT3(Z, P_) - d);

	//				if (z > 0.0f)
	//					break;
	//			}

	//			if (iTriangle >= nTriangles)
	//			{
	//				if (TSDF.Element[iVoxel] > 1e-6)
	//					int debug = 0;
	//			}
	//			else
	//			{
	//				if (TSDF.Element[iVoxel] < -1e-6)
	//					int debug = 0;
	//			}
	//		}

	// Compute the remaining TSDF values.

	// int dxyz[][3] = {
	//	{ -1, 0, 0 },
	//	{ 1, 0, 0 },
	//	{ 0, -1, 0 },
	//	{ 0, 1, 0 },
	//	{ 0, 0, -1 },
	//	{ 0, 0, 1 } };

	// int *RGBuff = new int[nVoxels];

	// int *pPut = RGBuff;
	// int *pFetch = RGBuff;

	// int ix_, iy_, iz_, iVoxel_;

	// for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
	//	if (TSDF.Element[iVoxel] < 1.5f)
	//	{
	//		RVL3DARRAY_INDICES(TSDF, iVoxel, ix, iy, iz);

	//		for (i = 0; i < 6; i++)
	//		{
	//			ix_ = ix + dxyz[i][0];
	//			iy_ = iy + dxyz[i][1];
	//			iz_ = iz + dxyz[i][2];

	//			if (ix_ >= 0 && ix_ < TSDF.a && iy_ >= 0 && iy_ < TSDF.b && iz_ >= 0 && iz_ < TSDF.c)
	//			{
	//				iVoxel_ = RVL3DARRAY_INDEX(TSDF, ix_, iy_, iz_);

	//				if (TSDF.Element[iVoxel_] > 1.5f)
	//					break;
	//			}
	//		}

	//		if (i < 6)
	//			*(pPut++) = iVoxel;
	//	}

	// while (pPut > pFetch)
	//{
	//	iVoxel = (*pFetch++);

	//	sign = (TSDF.Element[iVoxel] >= 0.0f ? 1.0f : -1.0);

	//	RVL3DARRAY_INDICES(TSDF, iVoxel, ix, iy, iz);

	//	for (i = 0; i < 6; i++)
	//	{
	//		ix_ = ix + dxyz[i][0];
	//		iy_ = iy + dxyz[i][1];
	//		iz_ = iz + dxyz[i][2];

	//		if (ix_ >= 0 && ix_ < TSDF.a && iy_ >= 0 && iy_ < TSDF.b && iz_ >= 0 && iz_ < TSDF.c)
	//		{
	//			iVoxel_ = RVL3DARRAY_INDEX(TSDF, ix_, iy_, iz_);

	//			if (TSDF.Element[iVoxel_] > 1.5f)
	//			{
	//				TSDF.Element[iVoxel_] = sign;

	//				*(pPut++) = iVoxel_;
	//			}
	//		}
	//	}
	//}

	// delete[] RGBuff;
}

void RVL::MESH::TSDF3(
	vtkSmartPointer<vtkPolyData> pPolyData,
	float resolution,
	float distanceValuePerVoxel,
	Array3D<float> &TSDF,
	float *P0,
	Box<float> &boundingBox,
	float &voxelSize)
{
	// Copy vertices from pPolyData to vertices.

	Array<Point> vertices;

	vertices.n = pPolyData->GetNumberOfPoints();

	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pPolyData->GetPoints()->GetData());

	if (pointData == NULL)
		return;

	vertices.Element = new Point[vertices.n];

	int iPt;

	for (iPt = 0; iPt < vertices.n; iPt++)
		pointData->GetTypedTuple(iPt, vertices.Element[iPt].P);

	// Copy triangles from pPolyData to triangles.

	Array<Triangle> triangles;

	triangles.n = pPolyData->GetPolys()->GetNumberOfCells();

	triangles.Element = new Triangle[triangles.n];

	vtkSmartPointer<vtkCellArray> Polys = pPolyData->GetPolys();

	Polys->InitTraversal();

	vtkSmartPointer<vtkIdList> ptList = vtkSmartPointer<vtkIdList>::New();

	int iTriangle = 0;

	int i;
	int nPts;
	int iPoly;
	Triangle *pTriangle;

	for (iPoly = 0; iPoly < triangles.n; iPoly++)
	{
		if (Polys->GetNextCell(ptList))
		{
			nPts = ptList->GetNumberOfIds();

			if (nPts == 3)
			{
				pTriangle = triangles.Element + iTriangle;

				iTriangle++;

				pTriangle->bSurface = true;

				for (i = 0; i < nPts; i++)
					pTriangle->iVertex[i] = ptList->GetId(i);
			}
		}
	}

	// Compute TDF of the union of all triangles in triangles.

	TriangleTDF(triangles, vertices, resolution, distanceValuePerVoxel, TSDF, P0, boundingBox, voxelSize);

	// Set negative sign of all TDF values.

	int nVoxels = TSDF.a * TSDF.b * TSDF.c;

	int iVoxel;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
		TSDF.Element[iVoxel] = -TSDF.Element[iVoxel];

	// Change the sign of all external TDF values by region growing from an external voxel.

	int *RGBuff = new int[nVoxels];

	int *pPut = RGBuff;
	int *pFetch = RGBuff;

	*(pPut++) = 0;

	TSDF.Element[0] = -TSDF.Element[0];

	int dijk[][3] = {
		{-1, 0, 0},
		{1, 0, 0},
		{0, -1, 0},
		{0, 1, 0},
		{0, 0, -1},
		{0, 0, 1}};

	int iVoxel_;
	int j, k, l;
	int i_, j_, k_;
	float f, f_, r;

	while (pPut > pFetch)
	{
		iVoxel = (*pFetch++);

		f = TSDF.Element[iVoxel];

		r = f / distanceValuePerVoxel;

		if (r >= 1.0f)
		{
			RVL3DARRAY_INDICES(TSDF, iVoxel, i, j, k);

			for (l = 0; l < 6; l++)
			{
				i_ = i + dijk[l][0];
				j_ = j + dijk[l][1];
				k_ = k + dijk[l][2];

				if (i_ >= 0 && i_ < TSDF.a && j_ >= 0 && j_ < TSDF.b && k_ >= 0 && k_ < TSDF.c)
				{
					iVoxel_ = RVL3DARRAY_INDEX(TSDF, i_, j_, k_);

					f_ = TSDF.Element[iVoxel_];

					if (f_ < 0.0f)
					{
						TSDF.Element[iVoxel_] = -TSDF.Element[iVoxel_];

						*(pPut++) = iVoxel_;
					}
				}
			}
		}
	}

	delete[] RGBuff;
}

void RVL::MESH::CreateVoxelGrid(
	Array<Point> vertices,
	float resolution,
	float distanceValuePerVoxel,
	Array3D<float> &f,
	Box<float> &boundingBox,
	Box<float> &box,
	float &voxelSize,
	float *P0)
{
	InitBoundingBox<float>(&boundingBox, vertices.Element[0].P);

	int iPt;

	for (iPt = 1; iPt < vertices.n; iPt++)
		UpdateBoundingBox<float>(&boundingBox, vertices.Element[iPt].P);

	float boundingBoxSizeX, boundingBoxSizeY, boundingBoxSizeZ;

	BoxSize<float>(&boundingBox, boundingBoxSizeX, boundingBoxSizeY, boundingBoxSizeZ);

	// float boundingBoxSize = BoxSize<float>(&boundingBox);

	voxelSize = pow(resolution * resolution * resolution * boundingBoxSizeX * boundingBoxSizeY * boundingBoxSizeZ, 1.0f / 3.0f);

	float fTriangleBorder = 1.0f / distanceValuePerVoxel;

	int triangleBorder = ceil(fTriangleBorder) + 1;

	fTriangleBorder *= voxelSize;

	int nx = (int)floor(0.5f * boundingBoxSizeX / voxelSize) + triangleBorder;
	int ny = (int)floor(0.5f * boundingBoxSizeY / voxelSize) + triangleBorder;
	int nz = (int)floor(0.5f * boundingBoxSizeZ / voxelSize) + triangleBorder;

	float a = voxelSize * (float)nx;
	float b = voxelSize * (float)ny;
	float c = voxelSize * (float)nz;

	float center[3];

	BoxCenter<float>(&boundingBox, center);

	box.minx = center[0] - a;
	box.miny = center[1] - b;
	box.minz = center[2] - c;
	box.maxx = center[0] + a;
	box.maxy = center[1] + b;
	box.maxz = center[2] + c;

	P0[0] = box.minx;
	P0[1] = box.miny;
	P0[2] = box.minz;

	f.a = 2 * nx;
	f.b = 2 * ny;
	f.c = 2 * nz;

	int nVoxels = f.a * f.b * f.c;

	f.Element = new float[nVoxels];
}

void RVL::MESH::TriangleTDF(
	Array<Triangle> triangles,
	Array<Point> vertices,
	float resolution,
	float distanceValuePerVoxel,
	Array3D<float> &TSDF,
	float *P0,
	Box<float> &boundingBox,
	float &voxelSize)
{
	// Create voxel grid.

	Box<float> box;

	MESH::CreateVoxelGrid(vertices, resolution, distanceValuePerVoxel, TSDF, boundingBox, box, voxelSize, P0);

	int nVoxels = TSDF.a * TSDF.b * TSDF.c;

	int iVoxel;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
		TSDF.Element[iVoxel] = 1.0f;

	//// Compute TSDF in the vicinity of the object surface.

	Array<int> iPositiveY;
	int iPositiveYMem[2];
	iPositiveY.Element = iPositiveYMem;

	int iCommonPt[3] = {1, 0, 2};

	int iTriangle;
	float P[4][3];
	int i, j;
	float X[3][3];
	// float Y[3][3];
	// float Z[3];
	float d;
	// float dy[3];
	float fTmp;
	// Box<float> triangleBox;
	// Box<int> triangleVoxelBox;
	int ix, iy, iz;
	float P_[3];
	// float x, y, xyDist2, z, zDist, dist2, dist_, dist2_;
	// float positiveY[2];
	// float V3Tmp[3];
	float *P__;
	// float L[3];
	MESH::Triangle *pTriangle;

	for (iTriangle = 0; iTriangle < triangles.n; iTriangle++)
	{
		pTriangle = triangles.Element + iTriangle;

		if (!pTriangle->bSurface)
			continue;

		TriangleTDF(pTriangle, vertices, voxelSize, distanceValuePerVoxel, box, TSDF);
	}
}

void RVL::MESH::TriangleTDF(
	Triangle *pTriangle,
	Array<Point> vertices,
	float voxelSize,
	float distanceValuePerVoxel,
	Box<float> box,
	Array3D<float> &TSDF)
{
	// P[i], i = 0,1,2 <- i-th vertex of pTriangle

	int i;
	float *P__;
	float P[3][3];

	for (i = 0; i < 3; i++)
	{
		P__ = vertices.Element[pTriangle->iVertex[i]].P;

		RVLCOPY3VECTOR(P__, P[i]);
	}

	// X[i] <- unit(P[i+1] - P[i]), L[i] <- length of the line (P[i+1], P[i]), i = 0,1,2

	int j;
	float X[3][3];
	float L[3];

	for (i = 0; i < 3; i++)
	{
		j = (i + 1) % 3;

		RVLDIF3VECTORS(P[j], P[i], X[i]);
		RVLNORM3(X[i], L[i]);
	}

	// Z <- -unit(X[0] x X[2])

	float Z[3];
	float fTmp;

	RVLCROSSPRODUCT3(X[0], X[2], Z);
	RVLNEGVECT3(Z, Z);
	RVLNORM3(Z, fTmp);

	// d <- Z' * P[0]

	float d = RVLDOTPRODUCT3(Z, P[0]);

	// Y[i] <- X[i] x Z, dy[i] <- Y[i]' * P[i], i = 0,1,2

	float Y[3][3];
	float dy[3];

	for (i = 0; i < 3; i++)
	{
		RVLCROSSPRODUCT3(X[i], Z, Y[i]);
		RVLNORM3(Y[i], fTmp);
		dy[i] = RVLDOTPRODUCT3(Y[i], P[i]);
	}

	// triangleBox <- bounding box of pTriangle expanded for fTriangleBorder

	Box<float> triangleBox;

	InitBoundingBox<float>(&triangleBox, P[0]);
	UpdateBoundingBox<float>(&triangleBox, P[1]);
	UpdateBoundingBox<float>(&triangleBox, P[2]);

	float fTriangleBorder = voxelSize / distanceValuePerVoxel;

	float kDistanceValue = 1.0f / fTriangleBorder;
	float kDistanceValue2 = kDistanceValue * kDistanceValue;

	ExpandBox<float>(&triangleBox, fTriangleBorder);

	// triangleVoxelBox <- box of voxels containing triangleBox

	Box<int> triangleVoxelBox;
	float P0[3];

	P0[0] = box.minx;
	P0[1] = box.miny;
	P0[2] = box.minz;

	triangleVoxelBox.minx = floor((triangleBox.minx - P0[0]) / voxelSize);
	triangleVoxelBox.maxx = ceil((triangleBox.maxx - P0[0]) / voxelSize);
	triangleVoxelBox.miny = floor((triangleBox.miny - P0[1]) / voxelSize);
	triangleVoxelBox.maxy = ceil((triangleBox.maxy - P0[1]) / voxelSize);
	triangleVoxelBox.minz = floor((triangleBox.minz - P0[2]) / voxelSize);
	triangleVoxelBox.maxz = ceil((triangleBox.maxz - P0[2]) / voxelSize);

	/// Compute distance value for every voxel in the vicinity of pTriangle.

	int nVoxels = TSDF.a * TSDF.b * TSDF.c;

	Array<int> iPositiveY;
	int iPositiveYMem[2];
	iPositiveY.Element = iPositiveYMem;
	int iCommonPt[3] = {1, 0, 2};
	int ix, iy, iz, iVoxel, iPt;
	float P_[3];
	float x, y, xyDist2, z, zDist, dist2, dist_, dist2_;
	float positiveY[2];
	float V3Tmp[3];

	for (iz = triangleVoxelBox.minz; iz <= triangleVoxelBox.maxz; iz++)
		for (iy = triangleVoxelBox.miny; iy <= triangleVoxelBox.maxy; iy++)
			for (ix = triangleVoxelBox.minx; ix <= triangleVoxelBox.maxx; ix++)
			{
				// if (ix == 3 && iy == 3 && iz == 104)
				//	int debug = 0;

				// iVoxel <- index of the voxel with indices (ix, iy, iz)

				iVoxel = RVL3DARRAY_INDEX(TSDF, ix, iy, iz);

				// if (iVoxel >= nVoxels)
				//	int debug = 0;

				// P_ <- coordinates of the voxel with indices (ix, iy, iz)

				P_[0] = (float)ix * voxelSize + P0[0];
				P_[1] = (float)iy * voxelSize + P0[1];
				P_[2] = (float)iz * voxelSize + P0[2];

				// z <- Z' * P_ - d

				z = (RVLDOTPRODUCT3(Z, P_) - d);

				// zDist <- kDistanceValue * |z|

				zDist = kDistanceValue * RVLABS(z);

				// If zDist < 1.0f, then compute the distance value of voxel (ix, iy, iz) and store it in TSDF.Element[iVoxel].

				if (zDist < 1.0f)
				{
					iPositiveY.n = 0;

					for (i = 0; i < 3; i++)
					{
						y = RVLDOTPRODUCT3(Y[i], P_) - dy[i];

						if (y > 0.0f)
						{
							iPositiveY.Element[iPositiveY.n] = i;
							positiveY[iPositiveY.n] = y;
							iPositiveY.n++;
						}
					}

					if (iPositiveY.n == 0)
						xyDist2 = 0.0f;
					else
					{
						RVLSCALE3VECTOR(Z, z, V3Tmp);
						RVLDIF3VECTORS(P_, V3Tmp, V3Tmp);

						if (iPositiveY.n == 1)
						{
							iPt = iPositiveY.Element[0];
							P__ = P[iPt];
							RVLDIF3VECTORS(V3Tmp, P__, V3Tmp);

							x = RVLDOTPRODUCT3(X[iPt], V3Tmp);

							y = positiveY[0];

							if (x < 0.0f)
								xyDist2 = kDistanceValue2 * (x * x + y * y);
							else if (x < L[iPt])
								xyDist2 = kDistanceValue2 * y * y;
							else
							{
								x -= L[iPt];
								xyDist2 = kDistanceValue2 * (x * x + y * y);
							}
						}
						else
						{
							iPt = iCommonPt[iPositiveY.Element[0] + iPositiveY.Element[1] - 1];
							P__ = P[iPt];
							RVLDIF3VECTORS(V3Tmp, P__, V3Tmp);
							xyDist2 = kDistanceValue2 * RVLDOTPRODUCT3(V3Tmp, V3Tmp);
						}
					}

					dist2 = xyDist2 + zDist * zDist;

					// dist2 = zDist * zDist;

					if (dist2 < 1.0f)
					{
						dist_ = TSDF.Element[iVoxel];

						dist2_ = dist_ * dist_;

						if (dist2 < dist2_)
							TSDF.Element[iVoxel] = sqrt(dist2);
					}
				} // if (zDist < 1.0f)
			}	  // for every voxel in triangleVoxelBox
				  /// END: Compute distance function value for every voxel in the vicinity of pTriangle.
}

void RVL::MESH::OrientedPCFromDepthImage(
	Array2D<short int> depthImage,
	Camera camera,
	Array2D<OrientedPoint> &PC,
	int winSize,
	float maxdz)
{
	// Constants.

	int halfWinSize = (winSize - 1) / 2;
	int w = depthImage.w;
	int h = depthImage.h;
	float fu = camera.fu;
	float fv = camera.fv;
	float uc = camera.uc;
	float vc = camera.vc;
	int nPix = w * h;

	// Create oriented point cloud.

	int iPix, u, v, i, j;
	short *d = depthImage.Element;
	Rect<int> ROI;
	// ROI.minx = halfWinSize;
	// ROI.maxx = w - halfWinSize - 1;
	// ROI.miny = halfWinSize;
	// ROI.maxy = h - halfWinSize - 1;
	ROI.minx = 0;
	ROI.maxx = w - 1;
	ROI.miny = 0;
	ROI.maxy = h - 1;
	PC.Element = new OrientedPoint[nPix];
	PC.w = w;
	PC.h = h;
	memset(PC.Element, 0, nPix * sizeof(OrientedPoint));
	OrientedPoint *pPt;
	float *P;
	float z;
	for (v = ROI.miny; v <= ROI.maxy; v++)
		for (u = ROI.minx; u <= ROI.maxx; u++)
		{
			iPix = u + v * w;
			z = 0.001f * d[iPix];
			if (z == 0.0f)
				continue;
			pPt = PC.Element + iPix;
			P = pPt->P;
			P[0] = z * ((float)u - uc) / fu;
			P[1] = z * ((float)v - vc) / fv;
			P[2] = z;
		}

	// Normals.

	int u_, v_, iPix_;
	OrientedPoint *pPt_;
	float *P_;
	Moments<float> M;
	float dz;
	float c[3];
	cv::Mat cvC(3, 3, CV_32FC1);
	float *C = (float *)(cvC.data);
	cv::Mat cvEigVC;
	float *eigVC;
	cv::Mat cvEigC;
	float *eigC;
	float *minEigVC;
	bool bEig;
	Rect<int> neig;
	for (v = ROI.miny; v <= ROI.maxy; v++)
		for (u = ROI.minx; u <= ROI.maxx; u++)
		{
			iPix = u + v * w;
			if (iPix == 26388)
				int debug = 0;
			pPt = PC.Element + iPix;
			P = pPt->P;
			if (P[2] == 0.0f)
				continue;
			InitMoments<float>(M);
			neig.minx = u - halfWinSize;
			neig.maxx = u + halfWinSize;
			neig.miny = v - halfWinSize;
			neig.maxy = v + halfWinSize;
			CropRect<int>(neig, ROI);
			// for (j = -halfWinSize; j <= halfWinSize; j++)
			//	for (i = -halfWinSize; i <= halfWinSize; i++)
			for (v_ = neig.miny; v_ <= neig.maxy; v_++)
				for (u_ = neig.minx; u_ <= neig.maxx; u_++)
				{
					// u_ = u + i;
					// v_ = v + j;
					iPix_ = u_ + v_ * w;
					pPt_ = PC.Element + iPix_;
					P_ = pPt_->P;
					dz = P_[2] - P[2];
					if (dz > maxdz)
						continue;
					if (dz < -maxdz)
						continue;
					UpdateMoments<float>(M, P_);
				}
			GetCovMatrix3<float>(&M, C, c);
			cv::eigen(cvC, cvEigC, cvEigVC);
			eigVC = (float *)(cvEigVC.data);
			eigC = (float *)(cvEigC.data);
			// if (eigC[1] >= 1e-6 && eigC[2] >= 0.0f)
			if (eigC[1] >= 1e-6)
			{
				minEigVC = eigVC + 6;
				if (RVLDOTPRODUCT3(minEigVC, P) <= 0.0f)
				{
					RVLCOPY3VECTOR(minEigVC, pPt->N);
				}
				else
				{
					RVLNEGVECT3(minEigVC, pPt->N);
				}
			}
		}
}

void RVL::MESH::OrientedPCFromRGBD(
	Array2D<short int> depthImage,
	Camera camera,
	Array2D<OrientedPoint> &PC,
	int winSize,
	float maxdz)
{
	// Constants.

	int halfWinSize = (winSize - 1) / 2;
	int w = depthImage.w;
	int h = depthImage.h;
	float fu = camera.fu;
	float fv = camera.fv;
	float uc = camera.uc;
	float vc = camera.vc;
	int nPix = w * h;

	// Create oriented point cloud.

	int iPix, u, v, i, j;
	short *d = depthImage.Element;
	Rect<int> ROI;
	// ROI.minx = halfWinSize;
	// ROI.maxx = w - halfWinSize - 1;
	// ROI.miny = halfWinSize;
	// ROI.maxy = h - halfWinSize - 1;
	ROI.minx = 0;
	ROI.maxx = w - 1;
	ROI.miny = 0;
	ROI.maxy = h - 1;
	PC.Element = new OrientedPoint[nPix];
	PC.w = w;
	PC.h = h;
	memset(PC.Element, 0, nPix * sizeof(OrientedPoint));
	OrientedPoint *pPt;
	float *P;
	float z;
	for (v = ROI.miny; v <= ROI.maxy; v++)
		for (u = ROI.minx; u <= ROI.maxx; u++)
		{
			iPix = u + v * w;
			z = 0.001f * d[iPix];
			if (z == 0.0f)
				continue;
			pPt = PC.Element + iPix;
			P = pPt->P;
			P[0] = z * ((float)u - uc) / fu;
			P[1] = z * ((float)v - vc) / fv;
			P[2] = z;
		}

	// Normals.

	int u_, v_, iPix_;
	OrientedPoint *pPt_;
	float *P_;
	Moments<float> M;
	float dz;
	float c[3];
	cv::Mat cvC(3, 3, CV_32FC1);
	float *C = (float *)(cvC.data);
	cv::Mat cvEigVC;
	float *eigVC;
	cv::Mat cvEigC;
	float *eigC;
	float *minEigVC;
	bool bEig;
	Rect<int> neig;
	for (v = ROI.miny; v <= ROI.maxy; v++)
		for (u = ROI.minx; u <= ROI.maxx; u++)
		{
			iPix = u + v * w;
			if (iPix == 26388)
				int debug = 0;
			pPt = PC.Element + iPix;
			P = pPt->P;
			if (P[2] == 0.0f)
				continue;
			InitMoments<float>(M);
			neig.minx = u - halfWinSize;
			neig.maxx = u + halfWinSize;
			neig.miny = v - halfWinSize;
			neig.maxy = v + halfWinSize;
			CropRect<int>(neig, ROI);
			// for (j = -halfWinSize; j <= halfWinSize; j++)
			//	for (i = -halfWinSize; i <= halfWinSize; i++)
			for (v_ = neig.miny; v_ <= neig.maxy; v_++)
				for (u_ = neig.minx; u_ <= neig.maxx; u_++)
				{
					// u_ = u + i;
					// v_ = v + j;
					iPix_ = u_ + v_ * w;
					pPt_ = PC.Element + iPix_;
					P_ = pPt_->P;
					dz = P_[2] - P[2];
					if (dz > maxdz)
						continue;
					if (dz < -maxdz)
						continue;
					UpdateMoments<float>(M, P_);
				}
			GetCovMatrix3<float>(&M, C, c);
			cv::eigen(cvC, cvEigC, cvEigVC);
			eigVC = (float *)(cvEigVC.data);
			eigC = (float *)(cvEigC.data);
			// if (eigC[1] >= 1e-6 && eigC[2] >= 0.0f)
			if (eigC[1] >= 1e-6)
			{
				minEigVC = eigVC + 6;
				if (RVLDOTPRODUCT3(minEigVC, P) <= 0.0f)
				{
					RVLCOPY3VECTOR(minEigVC, pPt->N);
				}
				else
				{
					RVLNEGVECT3(minEigVC, pPt->N);
				}
			}
		}
}

void RVL::MESH::PointTDF(
	vtkSmartPointer<vtkPolyData> pPolyData,
	float resolution,
	float distanceValuePerVoxel,
	float sigp,
	Array3D<float> &TDF,
	float *P0,
	Box<float> &boundingBox,
	float &voxelSize)
{
	// Copy vertices from pPolyData to vertices.

	Array<Point> vertices;

	vertices.n = pPolyData->GetNumberOfPoints();

	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pPolyData->GetPoints()->GetData());

	if (pointData == NULL)
		return;

	vertices.Element = new Point[vertices.n];

	int iPt;

	for (iPt = 0; iPt < vertices.n; iPt++)
		pointData->GetTypedTuple(iPt, vertices.Element[iPt].P);

	// Create voxel grid.

	Box<float> box;

	MESH::CreateVoxelGrid(vertices, resolution, distanceValuePerVoxel, TDF, boundingBox, box, voxelSize, P0);

	int nVoxels = TDF.a * TDF.b * TDF.c;

	int iVoxel;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
		TDF.Element[iVoxel] = 0.0f;

	Box<int> box_;

	box_.minx = 0;
	box_.maxx = TDF.a - 1;
	box_.miny = 0;
	box_.maxy = TDF.b - 1;
	box_.minz = 0;
	box_.maxz = TDF.c - 1;

	/// Compute TDF.

	float boundingBoxSize = BoxSize<float>(&boundingBox);

	float sigp_ = sigp * boundingBoxSize;

	float sigp2 = sigp_ * sigp_;

	float ptVoxelBoxSize = 3.0f * sigp_;

	float *P;
	Box<float> ptBox;
	Box<int> ptVoxelBox, ptVoxelBoxCropped;
	int ix, iy, iz;
	float P_[3], dP[3];
	float dist, w;

	for (iPt = 0; iPt < vertices.n; iPt++)
	{
		P = vertices.Element[iPt].P;

		// triangleBox <- bounding box of pTriangle expanded for fTriangleBorder

		InitBoundingBox<float>(&ptBox, P);

		ExpandBox<float>(&ptBox, ptVoxelBoxSize);

		// triangleVoxelBox <- box of voxels containing triangleBox

		ptVoxelBox.minx = floor((ptBox.minx - P0[0]) / voxelSize);
		ptVoxelBox.maxx = ceil((ptBox.maxx - P0[0]) / voxelSize);
		ptVoxelBox.miny = floor((ptBox.miny - P0[1]) / voxelSize);
		ptVoxelBox.maxy = ceil((ptBox.maxy - P0[1]) / voxelSize);
		ptVoxelBox.minz = floor((ptBox.minz - P0[2]) / voxelSize);
		ptVoxelBox.maxz = ceil((ptBox.maxz - P0[2]) / voxelSize);

		BoxIntersection<int>(&ptVoxelBox, &box_, &ptVoxelBoxCropped);

		for (iz = ptVoxelBoxCropped.minz; iz <= ptVoxelBoxCropped.maxz; iz++)
			for (iy = ptVoxelBoxCropped.miny; iy <= ptVoxelBoxCropped.maxy; iy++)
				for (ix = ptVoxelBoxCropped.minx; ix <= ptVoxelBoxCropped.maxx; ix++)
				{
					// if (ix == 3 && iy == 3 && iz == 104)
					//	int debug = 0;

					// iVoxel <- index of the voxel with indices (ix, iy, iz)

					iVoxel = RVL3DARRAY_INDEX(TDF, ix, iy, iz);

					// if (iVoxel >= nVoxels)
					//	int debug = 0;

					// P_ <- coordinates of the voxel with indices (ix, iy, iz)

					P_[0] = (float)ix * voxelSize + P0[0];
					P_[1] = (float)iy * voxelSize + P0[1];
					P_[2] = (float)iz * voxelSize + P0[2];

					// dP <- P_ - P;

					RVLDIF3VECTORS(P_, P, dP);

					// w <- exp(-||P_ - P||^2/sigp^2)

					dist = RVLDOTPRODUCT3(dP, dP);

					w = exp(-dist / sigp2);

					// TDF.Element[iVoxel] <- TDF.Element[iVoxel] + w;

					TDF.Element[iVoxel] += w;
				} // for every voxel in ptVoxelBox
	}			  // for every vertex

	///

	// Free memory.

	delete[] vertices.Element;
}

void RVL::MESH::OrientedPointDF(
	vtkSmartPointer<vtkPolyData> pPolyData,
	float resolution,
	float distanceValuePerVoxel,
	float sigp,
	Array3D<float> &DF,
	float *&W,
	float *P0,
	Box<float> &boundingBox,
	float &voxelSize)
{
	// Copy vertices and their normals from pPolyData to vertices.

	Array<Point> vertices;

	vertices.n = pPolyData->GetNumberOfPoints();

	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pPolyData->GetPoints()->GetData());

	if (pointData == NULL)
		return;

	vtkSmartPointer<vtkFloatArray> normalPointData = normalPointData->SafeDownCast(pPolyData->GetPointData()->GetArray("Normals"));

	if (normalPointData == NULL)
	{
		normalPointData = normalPointData->SafeDownCast(pPolyData->GetPointData()->GetNormals());

		if (normalPointData == NULL)
			return;
	}

	vertices.Element = new Point[vertices.n];

	int iPt;
	Point *pPt;
	float fTmp;

	for (iPt = 0; iPt < vertices.n; iPt++)
	{
		pPt = vertices.Element + iPt;

		pointData->GetTypedTuple(iPt, pPt->P);

		normalPointData->GetTypedTuple(iPt, pPt->N);
		RVLNORM3(pPt->N, fTmp);
	}

	// Create voxel grid.

	Box<float> box;

	MESH::CreateVoxelGrid(vertices, resolution, distanceValuePerVoxel, DF, boundingBox, box, voxelSize, P0);

	int nVoxels = DF.a * DF.b * DF.c;

	int iVoxel;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
		DF.Element[iVoxel] = 0.0f;

	Box<int> box_;

	box_.minx = 0;
	box_.maxx = DF.a - 1;
	box_.miny = 0;
	box_.maxy = DF.b - 1;
	box_.minz = 0;
	box_.maxz = DF.c - 1;

	/// Compute TDF.

	W = new float[nVoxels];

	memset(W, 0, nVoxels * sizeof(float));

	float boundingBoxSize = BoxSize<float>(&boundingBox);

	float sigp_ = sigp * boundingBoxSize;

	float sigp2 = sigp_ * sigp_;

	float ptVoxelBoxSize = 3.0f * sigp_;

	Box<float> ptBox;
	Box<int> ptVoxelBox, ptVoxelBoxCropped;
	int ix, iy, iz;
	float P_[3], dP[3];
	float ptDist, planeDist, w;

	for (iPt = 0; iPt < vertices.n; iPt++)
	{
		pPt = vertices.Element + iPt;

		// triangleBox <- bounding box of pTriangle expanded for fTriangleBorder

		InitBoundingBox<float>(&ptBox, pPt->P);

		ExpandBox<float>(&ptBox, ptVoxelBoxSize);

		// triangleVoxelBox <- box of voxels containing triangleBox

		ptVoxelBox.minx = floor((ptBox.minx - P0[0]) / voxelSize);
		ptVoxelBox.maxx = ceil((ptBox.maxx - P0[0]) / voxelSize);
		ptVoxelBox.miny = floor((ptBox.miny - P0[1]) / voxelSize);
		ptVoxelBox.maxy = ceil((ptBox.maxy - P0[1]) / voxelSize);
		ptVoxelBox.minz = floor((ptBox.minz - P0[2]) / voxelSize);
		ptVoxelBox.maxz = ceil((ptBox.maxz - P0[2]) / voxelSize);

		BoxIntersection<int>(&ptVoxelBox, &box_, &ptVoxelBoxCropped);

		for (iz = ptVoxelBoxCropped.minz; iz <= ptVoxelBoxCropped.maxz; iz++)
			for (iy = ptVoxelBoxCropped.miny; iy <= ptVoxelBoxCropped.maxy; iy++)
				for (ix = ptVoxelBoxCropped.minx; ix <= ptVoxelBoxCropped.maxx; ix++)
				{
					// if (ix == 3 && iy == 3 && iz == 104)
					//	int debug = 0;

					// iVoxel <- index of the voxel with indices (ix, iy, iz)

					iVoxel = RVL3DARRAY_INDEX(DF, ix, iy, iz);

					// if (iVoxel >= nVoxels)
					//	int debug = 0;

					// P_ <- coordinates of the voxel with indices (ix, iy, iz)

					P_[0] = (float)ix * voxelSize + P0[0];
					P_[1] = (float)iy * voxelSize + P0[1];
					P_[2] = (float)iz * voxelSize + P0[2];

					// dP <- P_ - P;

					RVLDIF3VECTORS(P_, pPt->P, dP);

					// w <- exp(-||P_ - P||^2/sigp^2)

					ptDist = RVLDOTPRODUCT3(dP, dP);

					w = exp(-ptDist / sigp2);

					W[iVoxel] += w;

					// planeDist <- pPt->N' * (P_ - pPt->P)

					planeDist = RVLDOTPRODUCT3(pPt->N, dP);

					// TDF.Element[iVoxel] <- TDF.Element[iVoxel] + w;

					DF.Element[iVoxel] += (w * planeDist);
				} // for every voxel in ptVoxelBox
	}			  // for every vertex

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
	{
		if (W[iVoxel] > 0.0f)
			DF.Element[iVoxel] /= W[iVoxel];
	}

	///

	// Free memory.

	delete[] vertices.Element;
}

#ifdef RVLVTK
vtkSmartPointer<vtkPolyData> RVL::MESH::CreateVisibleSurfaceMesh(
	vtkSmartPointer<vtkPolyData> pPolygonDataSrc,
	float voxelSize,
	int nFilter,
	float SDFIsoValue)
{
	Array3D<Voxel> volume;
	float P0[3];
	Box<float> boundingBox;
	Array<int> zeroDistanceVoxelArray;
	QLIST::Index *PtMem;
	Array3D<float> SDF;

	TSDF(pPolygonDataSrc, voxelSize, 1, volume, P0, boundingBox, zeroDistanceVoxelArray, PtMem);

	if (nFilter > 0)
	{
		Array3D<float> filter;
		filter.Element = NULL;

		CreateMeanFilter(filter, 3);

		FilterSDF(volume, filter, nFilter, SDF);

		delete[] filter.Element;
	}

	delete[] PtMem;
	delete[] zeroDistanceVoxelArray.Element;

	vtkSmartPointer<vtkPolyData> polyDataTgt = DisplayIsoSurface(SDF, P0, voxelSize, SDFIsoValue);

	delete[] SDF.Element;

	return polyDataTgt;
}

vtkSmartPointer<vtkPolyData> RVL::MESH::CreateVisibleSurfaceMesh2(
	vtkSmartPointer<vtkPolyData> pPolygonDataSrc,
	float resolution,
	float SDFIsoValue,
	int nFilter)
{
	Array3D<float> SDF;
	float P0[3];
	Box<float> boundingBox;
	float voxelSize;

	TSDF3(pPolygonDataSrc, resolution, 0.5f, SDF, P0, boundingBox, voxelSize);

	if (nFilter > 0)
	{
		Array3D<float> filter;
		filter.Element = NULL;

		CreateMeanFilter(filter, 3);

		FilterSDF(SDF, filter, nFilter, SDF);

		delete[] filter.Element;
	}

	vtkSmartPointer<vtkPolyData> polyDataTgt = DisplayIsoSurface(SDF, P0, voxelSize, SDFIsoValue);

	delete[] SDF.Element;

	return polyDataTgt;
}

vtkSmartPointer<vtkPolyData> RVL::MESH::CreateVisibleSurfaceMeshFromTetrahedrons(
	Tetrahedrons &tetrahedrons,
	float resolution,
	float SDFIsoValue,
	bool bRoom)
{
	Array3D<float> SDF;
	float P0[3];
	Box<float> boundingBox;
	float voxelSize;

	TSDF2(tetrahedrons, resolution, 0.5f, SDF, P0, boundingBox, voxelSize, bRoom);

	vtkSmartPointer<vtkPolyData> polyDataTgt = DisplayIsoSurface(SDF, P0, voxelSize, SDFIsoValue);

	delete[] SDF.Element;

	return polyDataTgt;
}

vtkSmartPointer<vtkPolyData> RVL::MESH::VTKPolyDataFromOrganizedOrientedPC(
	Array2D<OrientedPoint> PC,
	float maxTriangleEdgeLen)
{
	// Constants.

	float maxTriangleEdgeLen2 = maxTriangleEdgeLen * maxTriangleEdgeLen;

	// Triangles.

	int w = PC.w;
	int h = PC.h;
	int nPix = w * h;
	int u, v;
	int maxu = w - 2;
	int maxv = h - 2;
	OrientedPoint *pPt_[4];
	int iPix_[4];
	int iiPix[3];
	int iPix;
	float dP[3];
	float l2_[6];
	int i, j;
	Array<Vector3<int>> triangles;
	triangles.Element = new Vector3<int>[2 * (w - 1) * (h - 1)];
	Vector3<int> *pTriangle = triangles.Element;
	int nPixPts;
	bool bPixPt[4];
	for (v = 0; v <= maxv; v++)
	{
		for (u = 0; u <= maxu; u++)
		{
			iPix = u + v * w;
			nPixPts = 0;
			iPix_[0] = iPix;
			iPix_[1] = iPix + w;
			iPix_[3] = iPix + 1;
			iPix_[2] = iPix_[1] + 1;
			bPixPt[0] = bPixPt[1] = bPixPt[2] = bPixPt[3] = false;
			for (i = 0; i < 4; i++)
			{
				pPt_[i] = PC.Element + iPix_[i];
				if (bPixPt[i] = (RVLDOTPRODUCT3(pPt_[i]->N, pPt_[i]->N) > 0.5f))
					nPixPts++;
			}
			if (nPixPts == 3)
			{
				j = 0;
				for (i = 0; i < 4; i++)
					if (bPixPt[i])
					{
						pTriangle->Element[j] = iPix_[i];
						iiPix[j] = i;
						j++;
					}
				RVLDIF3VECTORS(pPt_[iiPix[1]]->P, pPt_[iiPix[0]]->P, dP);
				l2_[0] = RVLDOTPRODUCT3(dP, dP);
				RVLDIF3VECTORS(pPt_[iiPix[2]]->P, pPt_[iiPix[1]]->P, dP);
				l2_[1] = RVLDOTPRODUCT3(dP, dP);
				RVLDIF3VECTORS(pPt_[iiPix[0]]->P, pPt_[iiPix[2]]->P, dP);
				l2_[2] = RVLDOTPRODUCT3(dP, dP);
				for (i = 0; i < 3; i++)
					if (l2_[i] > maxTriangleEdgeLen2)
						break;
				if (i < 3)
					continue;
				pTriangle++;
			}
			else if (nPixPts == 4)
			{
				RVLDIF3VECTORS(pPt_[1]->P, pPt_[0]->P, dP);
				l2_[0] = RVLDOTPRODUCT3(dP, dP);
				RVLDIF3VECTORS(pPt_[2]->P, pPt_[1]->P, dP);
				l2_[1] = RVLDOTPRODUCT3(dP, dP);
				RVLDIF3VECTORS(pPt_[3]->P, pPt_[2]->P, dP);
				l2_[2] = RVLDOTPRODUCT3(dP, dP);
				RVLDIF3VECTORS(pPt_[0]->P, pPt_[3]->P, dP);
				l2_[3] = RVLDOTPRODUCT3(dP, dP);
				RVLDIF3VECTORS(pPt_[2]->P, pPt_[0]->P, dP);
				l2_[4] = RVLDOTPRODUCT3(dP, dP);
				RVLDIF3VECTORS(pPt_[3]->P, pPt_[1]->P, dP);
				l2_[5] = RVLDOTPRODUCT3(dP, dP);
				for (i = 0; i < 6; i++)
					if (l2_[i] > maxTriangleEdgeLen2)
						break;
				if (i < 6)
					continue;
				if (l2_[4] <= l2_[5])
				{
					RVLSET3VECTOR(pTriangle->Element, iPix_[0], iPix_[1], iPix_[2]);
					pTriangle++;
					RVLSET3VECTOR(pTriangle->Element, iPix_[0], iPix_[2], iPix_[3]);
				}
				else
				{
					RVLSET3VECTOR(pTriangle->Element, iPix_[0], iPix_[1], iPix_[3]);
					pTriangle++;
					RVLSET3VECTOR(pTriangle->Element, iPix_[1], iPix_[2], iPix_[3]);
				}
				pTriangle++;
			}
		}
	}
	triangles.n = pTriangle - triangles.Element;

	// Associate triangles with points.

	int diPixLT[8];
	diPixLT[0] = 1;
	diPixLT[1] = -w + 1;
	diPixLT[2] = -w;
	diPixLT[3] = -w - 1;
	diPixLT[4] = -1;
	diPixLT[5] = w - 1;
	diPixLT[6] = w;
	diPixLT[7] = w + 1;
	int *angleBinLTMem = new int[2 * w + 3];
	int *angleBinLT = angleBinLTMem + w + 1;
	for (i = 0; i < 8; i++)
		angleBinLT[diPixLT[i]] = i;
	int *pointTriangles = new int[8 * nPix];
	memset(pointTriangles, 0xff, 8 * nPix * sizeof(int));
	int iTriangle;
	int *pointTriangles_;
	int iPix1, iPix2, diPix;
	int iAngleBin1, iAngleBin2;
	for (iTriangle = 0; iTriangle < triangles.n; iTriangle++)
	{
		pTriangle = triangles.Element + iTriangle;
		for (i = 0; i < 3; i++)
		{
			iPix = pTriangle->Element[i];
			iPix1 = pTriangle->Element[(i + 1) % 3];
			diPix = iPix1 - iPix;
			iAngleBin1 = angleBinLT[diPix];
			iPix2 = pTriangle->Element[(i + 2) % 3];
			diPix = iPix2 - iPix;
			iAngleBin2 = (angleBinLT[diPix] + 7) % 8;
			pointTriangles_ = pointTriangles + 8 * iPix;
			pointTriangles_[iAngleBin1] = iTriangle;
			pointTriangles_[iAngleBin2] = iTriangle;
			// if (iAngleBin1 < 0 || iAngleBin1 >= 8 || iAngleBin2 < 0 || iAngleBin2 >= 8)
			//	int debug = 0;
		}
	}
	delete[] angleBinLTMem;

	// Detect and destroy butterflies.

	bool *bDeleteTriangle = new bool[triangles.n];
	memset(bDeleteTriangle, 0, triangles.n * sizeof(bool));
	Array<Pair<int, int>> wings;
	Pair<int, int> wingMem[4];
	wings.Element = wingMem;
	bool bWing;
	Pair<int, int> wing;
	int iStrongestWing;
	OrientedPoint *pPt, *pPt1;
	float l2;
	float minl2;
	int j_;
	for (iPix = 0; iPix < nPix; iPix++)
	{
		pointTriangles_ = pointTriangles + 8 * iPix;
		wings.n = 0;
		bWing = false;
		for (i = 0; i < 8; i++)
		{
			if (bWing)
			{
				if (pointTriangles_[i] < 0)
				{
					bWing = false;
					wings.Element[wings.n].b = i - 1;
					wings.n++;
				}
			}
			else
			{
				if (pointTriangles_[i] >= 0)
				{
					bWing = true;
					wings.Element[wings.n].a = i;
				}
			}
		}
		if (bWing)
		{
			wings.Element[wings.n].b = 7;
			wings.n++;
		}
		if (wings.n > 1)
		{
			pPt = PC.Element + iPix;
			if (wings.Element[0].a == 0 && wings.Element[wings.n - 1].b == 7)
			{
				wings.Element[0].a = wings.Element[wings.n - 1].a;
				wings.Element[0].b += 8;
				wings.n--;
			}
			if (wings.n > 1)
			{
				iStrongestWing = -1;
				for (i = 0; i < wings.n; i++)
				{
					wing = wings.Element[i];
					for (j = wing.a; j <= wing.b + 1; j++)
					{
						j_ = j % 8;
						iPix1 = iPix + diPixLT[j_];
						pPt1 = PC.Element + iPix1;
						RVLDIF3VECTORS(pPt1->P, pPt->P, dP);
						l2 = RVLDOTPRODUCT3(dP, dP);
						if (iStrongestWing < 0 || l2 < minl2)
						{
							minl2 = l2;
							iStrongestWing = i;
						}
					}
				}
				for (i = 0; i < wings.n; i++)
				{
					if (i == iStrongestWing)
						continue;
					wing = wings.Element[i];
					for (j = wing.a; j <= wing.b; j++)
						bDeleteTriangle[pointTriangles_[j]] = true;
				}
			}
		}
	}
	delete[] pointTriangles;
	Array<Vector3<int>> finalTriangles;
	finalTriangles.Element = new Vector3<int>[triangles.n];
	finalTriangles.n = 0;
	for (iTriangle = 0; iTriangle < triangles.n; iTriangle++)
	{
		pTriangle = triangles.Element + iTriangle;
		if (!bDeleteTriangle[iTriangle])
			finalTriangles.Element[finalTriangles.n++] = *pTriangle;
	}
	delete[] triangles.Element;
	delete[] bDeleteTriangle;

	/// VTKPolyData.

	// Points.

	vtkSmartPointer<vtkPoints> VTKPoints = vtkSmartPointer<vtkPoints>::New();
	pPt = PC.Element;
	for (iPix = 0; iPix < nPix; iPix++, pPt++)
		VTKPoints->InsertNextPoint(pPt->P[0], pPt->P[1], pPt->P[2]);

	// Triangles.

	vtkSmartPointer<vtkCellArray> VTKTriangles = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkTriangle> VTKTriangle = vtkSmartPointer<vtkTriangle>::New();
	for (iTriangle = 0; iTriangle < finalTriangles.n; iTriangle++)
	{
		pTriangle = finalTriangles.Element + iTriangle;
		for (i = 0; i < 3; i++)
			VTKTriangle->GetPointIds()->SetId(i, pTriangle->Element[i]);
		VTKTriangles->InsertNextCell(VTKTriangle);
	}
	delete[] finalTriangles.Element;

	// Normals.

	vtkSmartPointer<vtkFloatArray> VTKNormals = vtkSmartPointer<vtkFloatArray>::New();
	VTKNormals->SetNumberOfComponents(3);
	for (iPix = 0; iPix < nPix; iPix++)
		VTKNormals->InsertTypedTuple(iPix, PC.Element[iPix].N);

	// PolyData.

	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(VTKPoints);
	polyData->SetPolys(VTKTriangles);
	polyData->GetPointData()->SetNormals(VTKNormals);

	return polyData;
}

void RVL::MESH::MapRGBImageToVTKPolyData(
	vtkSmartPointer<vtkPolyData> pPolygonData,
	char *rgbImageData,
	int rgbImageWidth,
	int rgbImageHeight)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = vtkSmartPointer<vtkUnsignedCharArray>::New();
	rgbPointData->SetNumberOfComponents(3);
	rgbPointData->SetName("Colors");
	char *bgr = rgbImageData;
	int nPix = rgbImageWidth * rgbImageHeight;
	for (int iPt = 0; iPt < nPix; iPt++, bgr += 3)
		rgbPointData->InsertNextTuple3(bgr[2], bgr[1], bgr[0]);
	pPolygonData->GetPointData()->SetScalars(rgbPointData);
}

void RVL::MESH::MapRGBImageToVTKPolyData(
	vtkSmartPointer<vtkPolyData> pPolygonData,
	IplImage *rgbImagePNG)
{
	RVL::MESH::MapRGBImageToVTKPolyData(pPolygonData, rgbImagePNG->imageData, rgbImagePNG->width, rgbImagePNG->height);
}

vtkSmartPointer<vtkPolyData> RVL::MESH::RGBD2VTKPolyData(
	std::string RGBFileName,
	std::string depthFileName,
	Camera camera,
	float maxTriangleEdgeLen)
{
	Array2D<short int> depthImage;
	cv::Mat depthImagePNG = cv::imread(depthFileName, CV_LOAD_IMAGE_ANYDEPTH);
	depthImage.w = depthImagePNG.cols;
	depthImage.h = depthImagePNG.rows;
	depthImage.Element = (short int *)(depthImagePNG.data);
	IplImage *rgbImagePNG = cvLoadImage(RGBFileName.data());
	Array2D<OrientedPoint> PC;
	MESH::OrientedPCFromDepthImage(depthImage, camera, PC);
	// Visualizer visualizer;
	// visualizer.Create();
	// visualizer.DisplayOrganizedOrientedPC(PC, rgbImagePNG);
	// visualizer.Run();
	vtkSmartPointer<vtkPolyData> pPolygonData = MESH::VTKPolyDataFromOrganizedOrientedPC(PC, maxTriangleEdgeLen);
	delete[] PC.Element;
	RVL::MESH::MapRGBImageToVTKPolyData(pPolygonData, rgbImagePNG);

	return pPolygonData;
}

void RVL::MESH::RGBD2PLY(
	std::string RGBFileName,
	std::string depthFileName,
	Camera camera,
	float maxTriangleEdgeLen,
	std::string PLYFileName)
{
	vtkSmartPointer<vtkPolyData> pPolygonData = MESH::RGBD2VTKPolyData(RGBFileName, depthFileName, camera, maxTriangleEdgeLen);
	// vtkSmartPointer<vtkPLYWriter> pPLYWriter;
	// pPLYWriter->SetFileName((char*)(PLYFileName.data()));
	vtkSmartPointer<RVLVTKPLYWriter> pPLYWriter = vtkSmartPointer<RVLVTKPLYWriter>::New();
	pPLYWriter->WritePolyData(PLYFileName, pPolygonData);
	// int debug = 0;
}
#endif

void MESH::CreatePointArrayFromOrientedPointArray(
	Array<OrientedPoint> PtArray,
	Array<Point> &PtArray_,
	float maxZ)
{
	PtArray_.Element = new Point[PtArray.n];

	PtArray_.n = 0;

	int iPt;
	float *P_;
	OrientedPoint *pP;

	for (iPt = 0; iPt < PtArray.n; iPt++)
	{
		pP = PtArray.Element + iPt;

		if (pP->P[2] <= maxZ)
		{
			P_ = PtArray_.Element[PtArray_.n++].P;

			RVLCOPY3VECTOR(pP->P, P_);
		}
	}
}

void MESH::CreateOrientedPointArrayFromPointArray(
	Array<Point> PtArraySrc,
	Array<OrientedPoint> &PtArrayTgt)
{
	if (PtArrayTgt.Element == NULL)
		PtArrayTgt.Element = new OrientedPoint[PtArraySrc.n];
	PtArrayTgt.n = PtArraySrc.n;
	int iPt;
	OrientedPoint *pPtTgt;
	Point *pPtSrc;
	for (iPt = 0; iPt < PtArraySrc.n; iPt++)
	{
		pPtSrc = PtArraySrc.Element + iPt;
		pPtTgt = PtArrayTgt.Element + iPt;
		RVLCOPY3VECTOR(pPtSrc->P, pPtTgt->P);
		RVLCOPY3VECTOR(pPtSrc->N, pPtTgt->N);
	}
}

void MESH::CreateOrientedPointArrayFromPointArray(
	Array<Point> PtArraySrc,
	Array<int> PtIdx,
	Array<OrientedPoint> &PtArrayTgt)
{
	if (PtArrayTgt.Element == NULL)
		PtArrayTgt.Element = new OrientedPoint[PtIdx.n];
	PtArrayTgt.n = PtIdx.n;
	int iPt;
	OrientedPoint *pPtTgt = PtArrayTgt.Element;
	Point *pPtSrc;
	for (int i = 0; i < PtIdx.n; i++, pPtTgt++)
	{
		iPt = PtIdx.Element[i];
		pPtSrc = PtArraySrc.Element + iPt;
		RVLCOPY3VECTOR(pPtSrc->P, pPtTgt->P);
		RVLCOPY3VECTOR(pPtSrc->N, pPtTgt->N);
	}
}

void MESH::LoadTetrahedrons(
	char *fileName,
	RVL::MESH::Tetrahedrons &tetrahedrons)
{
	FILE *fp = fopen(fileName, "r");

	fscanf(fp, "%d %d\n", &(tetrahedrons.vertices.n), &(tetrahedrons.tetrahedrons.n));

	tetrahedrons.vertices.Element = new Point[tetrahedrons.vertices.n];

	Point *pPt = tetrahedrons.vertices.Element;

	int i;
	QList<MeshEdgePtr> *pEdgeList;

	for (i = 0; i < tetrahedrons.vertices.n; i++, pPt++)
	{
		fscanf(fp, "%f %f %f\n", pPt->P, pPt->P + 1, pPt->P + 2);

		pEdgeList = &(pPt->EdgeList);

		RVLQLIST_INIT(pEdgeList);
	}

	tetrahedrons.tetrahedrons.Element = new MESH::Tetrahedron[tetrahedrons.tetrahedrons.n];

	MESH::Tetrahedron *pTetrahedron;
	int j;
	int iTmp;

	for (i = 0; i < tetrahedrons.tetrahedrons.n; i++)
	{
		pTetrahedron = tetrahedrons.tetrahedrons.Element + i;

		fscanf(fp, "%d %d %d %d %d\n", pTetrahedron->iVertex, pTetrahedron->iVertex + 1, pTetrahedron->iVertex + 2, pTetrahedron->iVertex + 3, &iTmp);

		pTetrahedron->bValid = (iTmp > 0);

		for (j = 0; j < 4; j++)
			pTetrahedron->iTriangle[j] = -1;
	}

	fclose(fp);

	MESH::ComputeTetrahedronData(tetrahedrons);
}

void MESH::ComputeTetrahedronData(RVL::MESH::Tetrahedrons &tetrahedrons)
{
	// Edges.

	tetrahedrons.mem.Create(6 * tetrahedrons.tetrahedrons.n * (sizeof(PointEdge) + 2 * sizeof(MeshEdgePtr)));

	CRVLMem *pMem = &(tetrahedrons.mem);

	int iEdge = 0;

	int i, j;
	int iTetrahedron, iVertex, iVertex_, iVertex__;
	MESH::Tetrahedron *pTetrahedron;
	Point *pVertex, *pVertex_;
	QList<MeshEdgePtr> *pEdgeList;
	MeshEdge *pEdge;
	MeshEdgePtr *pEdgePtr;

	for (iTetrahedron = 0; iTetrahedron < tetrahedrons.tetrahedrons.n; iTetrahedron++)
	{
		pTetrahedron = tetrahedrons.tetrahedrons.Element + iTetrahedron;

		for (i = 0; i < 4; i++)
		{
			iVertex = pTetrahedron->iVertex[i];

			pVertex = tetrahedrons.vertices.Element + iVertex;

			pEdgeList = &(pVertex->EdgeList);

			for (j = 0; j < 4; j++)
			{
				if (j == i)
					continue;

				iVertex_ = pTetrahedron->iVertex[j];

				if (iVertex < iVertex_)
				{
					pEdgePtr = pEdgeList->pFirst;

					while (pEdgePtr)
					{
						RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iVertex, pEdgePtr, pEdge, iVertex__);

						if (iVertex__ == iVertex_)
							break;

						pEdgePtr = pEdgePtr->pNext;
					}

					if (pEdgePtr == NULL)
					{
						pVertex_ = tetrahedrons.vertices.Element + iVertex_;

						pEdge = ConnectNodes<Point, MeshEdge, MeshEdgePtr>(pVertex, pVertex_, iVertex, iVertex_, pMem);

						pEdge->idx = iEdge++;
					}
				}
			}
		}
	}

	int nEdges = iEdge;

	tetrahedrons.bEdgeValid = new bool[nEdges];

	memset(tetrahedrons.bEdgeValid, 0, nEdges * sizeof(bool));

	// Assign tetrahedrons to vertices.

	QList<QLIST::Index> *vertexTetrahedronList = new QList<QLIST::Index>[tetrahedrons.vertices.n];

	QList<QLIST::Index> *pTetrahedronList;

	for (iVertex = 0; iVertex < tetrahedrons.vertices.n; iVertex++)
	{
		pTetrahedronList = vertexTetrahedronList + iVertex;

		RVLQLIST_INIT(pTetrahedronList);
	}

	QLIST::Index *pVertexTetrahedronMem = new QLIST::Index[4 * tetrahedrons.tetrahedrons.n];

	QLIST::Index *pTetrahedronIdx = pVertexTetrahedronMem;

	for (iTetrahedron = 0; iTetrahedron < tetrahedrons.tetrahedrons.n; iTetrahedron++)
	{
		pTetrahedron = tetrahedrons.tetrahedrons.Element + iTetrahedron;

		for (i = 0; i < 4; i++)
		{
			iVertex = pTetrahedron->iVertex[i];

			pTetrahedronList = vertexTetrahedronList + iVertex;

			RVLQLIST_ADD_ENTRY(pTetrahedronList, pTetrahedronIdx);

			pTetrahedronIdx->Idx = iTetrahedron;

			pTetrahedronIdx++;
		}
	}

	// Triangles.

	tetrahedrons.triangles.Element = new MESH::Triangle[4 * tetrahedrons.tetrahedrons.n + 1];

	MESH::Triangle *pTriangle = tetrahedrons.triangles.Element;

	bool *bVertexBelongsToTetrahedron = new bool[tetrahedrons.vertices.n];

	memset(bVertexBelongsToTetrahedron, 0, tetrahedrons.vertices.n * sizeof(bool));

	// bool *bTriangleBelongsToTetrahedron = new bool[4 * tetrahedrons.tetrahedrons.n];

	// memset(bTriangleBelongsToTetrahedron, 0, 4 * tetrahedrons.tetrahedrons.n * sizeof(bool));

	int k;
	int iTetrahedron_;
	MESH::Tetrahedron *pTetrahedron_;
	int nCommonVertices;
	int iTetrahedronTriangle;
	int iTriangle;
	MESH::Triangle *pTriangle_;

	for (iTetrahedron = 0; iTetrahedron < tetrahedrons.tetrahedrons.n; iTetrahedron++)
	{
		// if (iTetrahedron == 8167)
		//	int debug = 0;

		pTetrahedron = tetrahedrons.tetrahedrons.Element + iTetrahedron;

		for (i = 0; i < 4; i++)
			bVertexBelongsToTetrahedron[pTetrahedron->iVertex[i]] = true;

		// for (i = 0; i < 4; i++)
		//	if (pTetrahedron->iTriangle[i] >= 0)
		//		bTriangleBelongsToTetrahedron[pTetrahedron->iTriangle[i]] = true;
		//	else
		//		break;

		for (i = 0; i < 4; i++)
			if (pTetrahedron->iTriangle[i] < 0)
				break;

		iTetrahedronTriangle = i;

		for (i = 0; i < 4; i++)
		{
			iVertex = pTetrahedron->iVertex[i];

			pTetrahedronList = vertexTetrahedronList + iVertex;

			pTetrahedronIdx = pTetrahedronList->pFirst;

			while (pTetrahedronIdx)
			{
				iTetrahedron_ = pTetrahedronIdx->Idx;

				if (iTetrahedron_ > iTetrahedron)
				{
					pTetrahedron_ = tetrahedrons.tetrahedrons.Element + iTetrahedron_;

					nCommonVertices = 0;

					for (j = 0; j < 4; j++)
					{
						iVertex_ = pTetrahedron_->iVertex[j];

						if (bVertexBelongsToTetrahedron[iVertex_])
						{
							pTriangle->iVertex[nCommonVertices++] = iVertex_;

							if (iVertex_ < iVertex)
								break;
						}
					}

					if (j >= 4)
					{
						if (nCommonVertices == 3)
						{
							pTriangle->iTetrahedron[0] = iTetrahedron;
							pTriangle->iTetrahedron[1] = iTetrahedron_;

							iTriangle = pTriangle - tetrahedrons.triangles.Element;

							if (iTetrahedronTriangle < 4)
								pTetrahedron->iTriangle[iTetrahedronTriangle++] = iTriangle;

							for (j = 0; j < 4; j++)
								if (pTetrahedron_->iTriangle[j] < 0)
									break;

							// if (j < 4 && iTriangle == 4)
							//	int debug = 0;

							if (j < 4)
								pTetrahedron_->iTriangle[j] = iTriangle;

							pTriangle++;
						}
					}
				}

				pTetrahedronIdx = pTetrahedronIdx->pNext;
			} // for every tetrahedron that shares vertex iVertex with pTetrahedron
		}	  // for every vertex of pTetrahedron

		for (i = 0; i < 4; i++)
		{
			bVertexBelongsToTetrahedron[pTetrahedron->iVertex[i]] = false; // Unmark the i-th vertex (the others remain marked).

			for (j = 0; j < 4; j++)
			{
				iTriangle = pTetrahedron->iTriangle[j];

				if (iTriangle < 0)
					break;

				pTriangle_ = tetrahedrons.triangles.Element + iTriangle;

				for (k = 0; k < 3; k++)
					if (!bVertexBelongsToTetrahedron[pTriangle_->iVertex[k]])
						break;

				if (k >= 3) // if pTriangle has all marked vertices
					break;
			}

			if (j >= 4 || iTriangle < 0) // if there is no triangle belonging to pTetrahedron which is defined by the marked vertices
			{
				k = 0;

				for (j = 0; j < 4; j++)
				{
					iVertex = pTetrahedron->iVertex[j];

					if (bVertexBelongsToTetrahedron[iVertex])
						pTriangle->iVertex[k++] = iVertex;
				}

				pTriangle->iTetrahedron[0] = iTetrahedron;
				pTriangle->iTetrahedron[1] = -1;

				iTriangle = pTriangle - tetrahedrons.triangles.Element;

				if (iTetrahedronTriangle < 4)
					pTetrahedron->iTriangle[iTetrahedronTriangle++] = iTriangle;

				pTriangle++;
			}

			bVertexBelongsToTetrahedron[pTetrahedron->iVertex[i]] = true;
		}

		for (i = 0; i < 4; i++)
			bVertexBelongsToTetrahedron[pTetrahedron->iVertex[i]] = false;

		// for (i = 0; i < 4; i++)
		//	bTriangleBelongsToTetrahedron[pTetrahedron->iTriangle[i]] = false;
	} // for every tetrahedron

	tetrahedrons.triangles.n = pTriangle - tetrahedrons.triangles.Element;

	// Identify surface triangles.

	// int debug = 0;

	int nInternalTetrahedrons, nExternalTetrahedrons;

	for (iTriangle = 0; iTriangle < tetrahedrons.triangles.n; iTriangle++)
	{
		pTriangle = tetrahedrons.triangles.Element + iTriangle;

		nInternalTetrahedrons = nExternalTetrahedrons = 0;

		for (i = 0; i < 2; i++)
		{
			iTetrahedron = pTriangle->iTetrahedron[i];

			if (iTetrahedron > 0)
			{
				if (tetrahedrons.tetrahedrons.Element[iTetrahedron].bValid)
					nInternalTetrahedrons++;
				else
					nExternalTetrahedrons++;
			}
		}

		pTriangle->bSurface = (nInternalTetrahedrons == 1 && nExternalTetrahedrons == 1);

		// if (pTriangle->bSurface)
		//	debug++;
	}

	// Free memory.

	delete[] vertexTetrahedronList;
	delete[] pVertexTetrahedronMem;
	delete[] bVertexBelongsToTetrahedron;
	// delete[] bTriangleBelongsToTetrahedron;
}

void MESH::ClearTetrahedronData(RVL::MESH::Tetrahedrons tetrahedrons)
{
	delete[] tetrahedrons.vertices.Element;
	delete[] tetrahedrons.bEdgeValid;
	delete[] tetrahedrons.triangles.Element;
	delete[] tetrahedrons.tetrahedrons.Element;
	tetrahedrons.mem.Clear();
}

void MESH::MouseRButtonDown(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor *>(caller);

	Visualizer *pVisualizer = (Visualizer *)clientdata;

	pVisualizer->pointPicker->Pick(interactor->GetEventPosition()[0], interactor->GetEventPosition()[1], 0,
								   interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	vtkIdType selectedPoint = pVisualizer->pointPicker->GetPointId();

	printf("Selected point: %d\n", selectedPoint);
}

void MESH::LabelParts(
	vtkSmartPointer<vtkPolyData> pDensePolygonData,
	vtkSmartPointer<vtkPolyData> pSparsePolygonData,
	char *labelFileName)
{
	// Copy vertices from pPolyData to vertices

	Array<Point> denseVertices, sparseVertices;
	Array<int> denseLabels, sparseLabels;

	denseVertices.n = pDensePolygonData->GetNumberOfPoints();
	sparseVertices.n = pSparsePolygonData->GetNumberOfPoints();

	denseLabels.n = denseVertices.n;
	sparseLabels.n = sparseVertices.n;

	vtkSmartPointer<vtkFloatArray> densePointData = densePointData->SafeDownCast(pDensePolygonData->GetPoints()->GetData());
	vtkSmartPointer<vtkFloatArray> sparsePointData = sparsePointData->SafeDownCast(pSparsePolygonData->GetPoints()->GetData());

	if (densePointData == NULL || sparsePointData == NULL)
		return;

	denseVertices.Element = new Point[denseVertices.n];
	sparseVertices.Element = new Point[sparseVertices.n];

	denseLabels.Element = new int[denseLabels.n];
	sparseLabels.Element = new int[sparseLabels.n];

	int iPt;

	// load sparse labels from file
	FILE *fp = fopen(labelFileName, "r");

	if (fp)
	{
		for (iPt = 0; iPt < sparseLabels.n; iPt++)
			fscanf(fp, "%d\n", &sparseLabels.Element[iPt]);

		fclose(fp);
	}

	fp = fopen("densePoints.txt", "w");

	for (iPt = 0; iPt < denseVertices.n; iPt++)
	{
		densePointData->GetTypedTuple(iPt, denseVertices.Element[iPt].P);
		fprintf(fp, "%f\t%f\t%f\n", denseVertices.Element[iPt].P[0], denseVertices.Element[iPt].P[1], denseVertices.Element[iPt].P[2]);
	}

	fclose(fp);

	for (iPt = 0; iPt < sparseVertices.n; iPt++)
		sparsePointData->GetTypedTuple(iPt, sparseVertices.Element[iPt].P);

	int idPt, isPt;
	float minDistance, distance;
	float V3Tmp[3];

	// find minimum distance for each point from denceVertices
	for (idPt = 0; idPt < denseVertices.n; idPt++)
	{
		minDistance = LONG_MAX;
		for (isPt = 0; isPt < sparseVertices.n; isPt++)
		{
			RVLDIF3VECTORS(denseVertices.Element[idPt].P, sparseVertices.Element[isPt].P, V3Tmp);
			distance = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

			if (distance < minDistance)
			{
				minDistance = distance;
				denseLabels.Element[idPt] = sparseLabels.Element[isPt];

				if (minDistance == 0.0)
					break;
			}
		}
	}

	// save dense labels to file
	char *denseLabelFileName = RVLCreateFileName(labelFileName, ".seg", -1, "_.seg");

	fp = fopen(denseLabelFileName, "w");

	if (fp)
	{
		for (iPt = 0; iPt < denseLabels.n; iPt++)
			fprintf(fp, "%d\n", denseLabels.Element[iPt]);

		fclose(fp);
	}

	RVL_DELETE_ARRAY(denseVertices.Element);
	RVL_DELETE_ARRAY(sparseVertices.Element);
	RVL_DELETE_ARRAY(denseLabels.Element);
	RVL_DELETE_ARRAY(sparseLabels.Element);
}

void MESH::SparsePointsCorrespondences(
	vtkSmartPointer<vtkPolyData> pDensePolygonData,
	vtkSmartPointer<vtkPolyData> pSparsePolygonData,
	char *labelFileName)
{
	// Copy vertices from pPolyData to vertices

	Array<Point> denseVertices, sparseVertices;
	Array<int> sparseCorrespondences;

	denseVertices.n = pDensePolygonData->GetNumberOfPoints();
	sparseVertices.n = pSparsePolygonData->GetNumberOfPoints();

	sparseCorrespondences.n = sparseVertices.n;

	vtkSmartPointer<vtkFloatArray> densePointData = densePointData->SafeDownCast(pDensePolygonData->GetPoints()->GetData());
	vtkSmartPointer<vtkFloatArray> sparsePointData = sparsePointData->SafeDownCast(pSparsePolygonData->GetPoints()->GetData());

	if (densePointData == NULL || sparsePointData == NULL)
		return;

	denseVertices.Element = new Point[denseVertices.n];
	sparseVertices.Element = new Point[sparseVertices.n];

	sparseCorrespondences.Element = new int[sparseCorrespondences.n];

	int iPt;

	FILE *fp = fopen("densePoints.txt", "w");

	for (iPt = 0; iPt < denseVertices.n; iPt++)
	{
		densePointData->GetTypedTuple(iPt, denseVertices.Element[iPt].P);
		fprintf(fp, "%f\t%f\t%f\n", denseVertices.Element[iPt].P[0], denseVertices.Element[iPt].P[1], denseVertices.Element[iPt].P[2]);
	}

	fclose(fp);

	for (iPt = 0; iPt < sparseVertices.n; iPt++)
		sparsePointData->GetTypedTuple(iPt, sparseVertices.Element[iPt].P);

	int idPt, isPt;
	float minDistance, distance;
	float V3Tmp[3];

	// find minimum distance for each point from sparceVertices
	for (isPt = 0; isPt < sparseVertices.n; isPt++)
	{
		minDistance = LONG_MAX;
		for (idPt = 0; idPt < denseVertices.n; idPt++)
		{
			RVLDIF3VECTORS(denseVertices.Element[idPt].P, sparseVertices.Element[isPt].P, V3Tmp);
			distance = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

			if (distance < minDistance)
			{
				minDistance = distance;
				sparseCorrespondences.Element[isPt] = idPt;

				if (minDistance == 0.0)
					break;
			}
		}
	}

	// save dense labels to file
	char *sparseCorrespondancesLabelFileName = RVLCreateFileName(labelFileName, ".seg", -1, ".cor");

	fp = fopen(sparseCorrespondancesLabelFileName, "w");

	if (fp)
	{
		for (iPt = 0; iPt < sparseCorrespondences.n; iPt++)
			fprintf(fp, "%d\n", sparseCorrespondences.Element[iPt]);

		fclose(fp);
	}

	RVL_DELETE_ARRAY(denseVertices.Element);
	RVL_DELETE_ARRAY(sparseVertices.Element);
	RVL_DELETE_ARRAY(sparseCorrespondences.Element);
}

void MESH::ComputeDistributionDouble(
	Moments<double> moments,
	MESH::Distribution &distribution)
{
	double C[9];
	double t[3];

	GetCovMatrix3<double>(&moments, C, t);

	RVLCOPY3VECTOR(t, distribution.t);

	Eigen::EigenSolver<Eigen::Matrix3d> eigenSolver;

	eigenSolver.compute(Eigen::Map<Eigen::Matrix3d>(C));

	Eigen::Matrix3d R = eigenSolver.pseudoEigenvectors();

	distribution.R[0] = R(0, 0);
	distribution.R[1] = R(1, 0);
	distribution.R[2] = R(2, 0);
	distribution.R[3] = R(0, 1);
	distribution.R[4] = R(1, 1);
	distribution.R[5] = R(2, 1);
	distribution.R[6] = R(0, 2);
	distribution.R[7] = R(1, 2);
	distribution.R[8] = R(2, 2);

	Eigen::Vector3cd var_ = eigenSolver.eigenvalues();

	distribution.var[0] = var_[0].real();
	distribution.var[1] = var_[1].real();
	distribution.var[2] = var_[2].real();
}

void MESH::ComputePlaneParameters(
	MESH::Distribution &distribution,
	float *N,
	float &d)
{
	float *var = distribution.var;

	int idx[3];
	int iTmp;

	RVLSORT3ASCEND(var, idx, iTmp);

	float *N_ = distribution.R + 3 * idx[0];

	RVLCOPY3VECTOR(N_, N);

	d = RVLDOTPRODUCT3(N, distribution.t);
}

void RVL::FilterSDF(
	Array3D<float> SDFSrc,
	Array3D<float> filter,
	int n,
	Array3D<float> &SDF)
{
	int nVoxels = SDFSrc.a * SDFSrc.b * SDFSrc.c;

	SDF.a = SDFSrc.a;
	SDF.b = SDFSrc.b;
	SDF.c = SDFSrc.c;

	if (SDF.Element == NULL)
		SDF.Element = new float[nVoxels];

	float *SDFBuff = new float[nVoxels];

	memcpy(SDFBuff, SDFSrc.Element, nVoxels * sizeof(float));
	memcpy(SDF.Element, SDFBuff, nVoxels * sizeof(float));

	int nf = (filter.a - 1) / 2;

	int iend = SDF.a - nf;
	int jend = SDF.b - nf;
	int kend = SDF.c - nf;

	int i, j, k, i_, j_, k_, l;
	int iFilter;
	float f;

	for (l = 0; l < n; l++)
	{
		for (i = nf; i < iend; i++)
			for (j = nf; j < jend; j++)
				for (k = nf; k < kend; k++)
				{
					f = 0.0f;

					iFilter = 0;

					for (i_ = -nf; i_ <= nf; i_++)
						for (j_ = -nf; j_ <= nf; j_++)
							for (k_ = -nf; k_ <= nf; k_++, iFilter++)
								f += filter.Element[iFilter] * SDFBuff[RVL3DARRAY_INDEX(SDF, i + i_, j + j_, k + k_)];

					SDF.Element[RVL3DARRAY_INDEX(SDF, i, j, k)] = f;
				}

		if (l < n - 1)
			memcpy(SDFBuff, SDF.Element, nVoxels * sizeof(float));
	}

	delete[] SDFBuff;
}

void RVL::FilterSDF(
	Array3D<Voxel> volume,
	Array3D<float> filter,
	int n,
	Array3D<float> &SDF)
{
	int nVoxels = volume.a * volume.b * volume.c;

	Array3D<float> SDFSrc;
	SDFSrc.Element = new float[nVoxels];
	SDFSrc.a = volume.a;
	SDFSrc.b = volume.b;
	SDFSrc.c = volume.c;

	int iVoxel;
	Voxel *pVoxel;
	float f;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
	{
		pVoxel = volume.Element + iVoxel;
		f = (float)(pVoxel->voxelDistance);
		SDFSrc.Element[iVoxel] = f;
	}

	FilterSDF(SDFSrc, filter, n, SDF);

	delete[] SDFSrc.Element;
}

void RVL::CreateMeanFilter(
	Array3D<float> &filter,
	int n)
{
	filter.a = filter.b = filter.c = n;

	int nf = filter.a * filter.b * filter.c;

	if (filter.Element == NULL)
		filter.Element = new float[nf];

	float w = 1.0f / (float)nf;

	int iFilter;

	for (iFilter = 0; iFilter < nf; iFilter++)
		filter.Element[iFilter] = w;
}

float RVL::PointSetToPlaneDistance(
	Array<Point> points,
	float *RMS,
	float *tMS,
	float *N,
	float d)
{
	float *PM = points.Element[0].P;

	float PS[3];

	RVLTRANSF3(PM, RMS, tMS, PS);

	float minDistance = RVLDOTPRODUCT3(N, PS) - d;

	float distance;
	int iPt;

	for (iPt = 1; iPt < points.n; iPt++)
	{
		PM = points.Element[iPt].P;

		RVLTRANSF3(PM, RMS, tMS, PS);

		distance = RVLDOTPRODUCT3(N, PS) - d;

		if (distance < minDistance)
			minDistance = distance;
	}

	return minDistance;
}
