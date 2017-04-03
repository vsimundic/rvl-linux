#pragma once

#define RVLMESH_POINT_FLAG_FOREGROUND		0x01
#define RVLMESH_POINT_FLAG_BACKGROUND		0x02
#define RVLMESH_POINT_FLAG_EDGE_CLASS		0x03

//#define RVLMESH_BOUNDARY_DEBUG

// Input:  mesh pMesh, 
//         vertex idx. iPt, 
//         connector pEdgePtr connecting an edge E to the vertex iPt,
//         array map defining a region R (map[i] = map[iPt] for all vertices i of the mesh pMesh belonging to the region R)
// Output: pEdge <- the next edge E' in EdgeList[iPt] following E, such that Opp(E', iPt) is in R, where Opp is defined in ARP3D.TR3
//         iNeighborPt <- Opp(E', iPt)
//         pEdgePtr <- connector connecting pEdge to iPt,
// Temporary variables: pPt, pEdgeList, side

#ifdef RVLMESH_BOUNDARY_DEBUG
#define RVLMESH_GET_NEXT_IN_REGION(pMesh, iPt, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge)\
{\
	pPt = pMesh->NodeArray.Element + iPt;\
	pEdgeList = &(pPt->EdgeList);\
	do\
	{\
		RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr)\
		RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side)\
		fprintf(fpDebug, "%d (%d) ", iNeighborPt, map[iNeighborPt]);\
	}while (map[iNeighborPt] != map[iPt]);\
}
#else
#define RVLMESH_GET_NEXT_IN_REGION(pMesh, iPt, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge)\
{\
	pPt = pMesh->NodeArray.Element + iPt;\
	pEdgeList = &(pPt->EdgeList);\
	do\
	{\
		RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr)\
		RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side)\
	}while (map[iNeighborPt] != map[iPt]);\
}
#endif

// Input: pEdge, side
// Output: iPt <- point on the side side of the edge pEdge; pEdgePtr <- connector of point iPt and pEdge

#define RVLMESH_GET_POINT(pEdge, side, iPt, pEdgePtr)\
{\
	iPt = pEdge->iVertex[side];\
	pEdgePtr = pEdge->pVertexEdgePtr[side];\
}

namespace RVL
{
	namespace MESH
	{
		struct Distribution
		{
			float t[3];		// centroid 
			float R[9];		// principal axes (each row is one axis)
			float var[3];	// point variances in directions of the principal axes
			int RGB[3];		// average color
		};
	}

	struct MeshEdgePtr;

	struct MeshEdge
	{
		int iVertex[2];
		MeshEdgePtr *pVertexEdgePtr[2];
		float cost;
		int idx;
	};

	struct MeshEdgePtr
	{
		MeshEdge *pEdge;
		MeshEdgePtr *pNext;
	};

	struct Point
	{
		int RGB[3];			// color
		float P[3];						// position
		float N[3];						// normal
		QList<MeshEdgePtr> EdgeList;	// edge list (list of edge connectors)
		bool bBoundary;					// true if the point is on the image boundary, on a depth discontinuity contur or on the boundary of a region of undefined depth,
										// i.e. if there is a boundary edge connected to this point.
		bool bValid;
		BYTE flags;
	};

	struct OrientedPoint
	{
		float P[3];
		float N[3];
	};

	namespace MESH
	{
		struct PointEdge
		{
			int iPt;				// vertex index
			MeshEdgePtr *pEdgePtr;	// connector connecting an edge E to the vertex iPt
			unsigned char side;		// side of the edge E to which the iPt is connected
		};
	}

	class Mesh : public Graph<Point, MeshEdge, MeshEdgePtr>
	{
		public:
			Mesh();
			virtual ~Mesh();
			//bool Load(
			//	char *FileName,
			//	PCLMeshBuilder *pMeshBuilder,
			//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
			//	pcl::PolygonMesh &PCLMesh,
			//	bool bSavePLY = false);
			void LoadPolyDataFromPLY(char *PLYFileName);
			void SavePolyDataToPLY(char *PLYFileName);
			bool CreateOrderedMeshFromPolyData();
			void ComputeDistribution(
				Array<int> &PtArray,
				MESH::Distribution &distribution);
			void ComputeDistributionDouble(
				Array<int> &PtArray,
				MESH::Distribution &distribution);
			bool FindBoundaryEdge(
				QList<QLIST::Index> *pInPtList,
				QLIST::Index *&pPtIdx,
				int *map,
				int &iPt,
				MeshEdgePtr *&pEdgePtr);
			void Boundary(
				QList<QLIST::Index2> *pInPtList,
				int *Map,
				QList<QLIST::Index> *pOutPtArray,
				QLIST::Index *pMem);
			void Boundary(
				QList<QLIST::Index2> *pInPtList,
				int *map,
				Array<Array<MeshEdgePtr *>> &BoundaryArray,
				MeshEdgePtr **&pBoundaryMem, 
				unsigned char *edgeMarkMap);
			void BoundingBox(Box<float> *pBox);

			// For a mesh point index iPt, the function returns true if the point is on the boundary of a region in the map map containing elements with value idx. 
			// pEdgePtr <- the connector connecting the first region boundar edge in CCW direction.

			inline bool IsBoundaryPoint(
				int iPt,
				int *map,
				int idx,
				MeshEdgePtr *&pEdgePtr)
			{
				Point *pPt = NodeArray.Element + iPt;

				bool bOut = false;		// A neighboring vertex belonging to another region is found.

				pEdgePtr = pPt->EdgeList.pFirst;

				int iPt_;
				MeshEdge *pEdge;

				// If the vertex is a boundary point and the first neighbor belongs to the same region, 
				// then the first edge is the first region boundary edge in CCW direction.
				// An example of this case is shown in ARP3D.TR3, Fig: Detection of region boundary (a).

				if (pPt->bBoundary)
				{
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

					if (map[iPt_] == idx)
						return true;
				}

				// If a neighbor belonging to another region is already found and the currently processed neighbor belongs to the query region, 
				// then the query edge is the first region boundary edge in CCW direction.
				// An example of this case is shown in ARP3D.TR3, Fig: Detection of region boundary (b).

				while (pEdgePtr)
				{
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

					if (map[iPt_] == idx)
					{
						if (bOut)
							return true;
					}
					else
						bOut = true;

					pEdgePtr = pEdgePtr->pNext;
				}

				// If no neighbors belonging to the query region are found after a neighbor belonging to another region is found,
				// then the only possibility that a region boundary edge is connected to the vertex iPt is that this is the first edge in the edge list.
				// An example of this case is shown in ARP3D.TR3, Fig: Detection of region boundary (c).

				if (bOut)
				{
					pEdgePtr = pPt->EdgeList.pFirst;

					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

					if (map[iPt_] == idx)
						return true;
				}

				// (all neighbors of vertex iPt belong to another region) or 
				// ((all neighbors belong to the query region) and (there is no mesh boundary edges in the edge list))
				// In either case, vertex iPt is not a region boundary point.

				return false;
			}

			// Input:  iPt - index of a boundary vertex,
			//         map - map,
			//         pEdgePtr - the connector connecting edge E to the vertex iPt, where E is the first region boundar edge in CCW direction
			// Output: pEdgePtr - the connector connecting an edge E' to the vertex iPt, where E' is the next first region boundar edge in CCW direction after E.

			inline bool GetNextBoundaryEdge(
				int iPt,
				int *map,
				MeshEdgePtr *&pEdgePtr)
			{
				int idx = map[iPt];

				bool bOut = false;

				MeshEdgePtr *pEdgePtr0 = pEdgePtr;

				pEdgePtr = pEdgePtr->pNext;

				int iPt_;
				MeshEdge *pEdge;

				while (pEdgePtr)
				{
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

					if (map[iPt_] == idx)
					{
						if (bOut)
							return (pEdgePtr != pEdgePtr0);
					}
					else
						bOut = true;

					pEdgePtr = pEdgePtr->pNext;
				}

				if (!bOut)
					return false;

				Point *pPt = NodeArray.Element + iPt;

				if (pPt->bBoundary)
					return false;

				pEdgePtr = pPt->EdgeList.pFirst;

				RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

				if (map[iPt_] == idx)
					return (pEdgePtr != pEdgePtr0);

				return false;
			}

		public:
			vtkSmartPointer<vtkPolyData> pPolygonData;
			float normalEstimationRadius;
			int nBoundaryPts;
			bool bOrganizedPC;
			int width;
			int height;

#ifdef RVLMESH_BOUNDARY_DEBUG		
			int debugState;
#endif
	};
}
