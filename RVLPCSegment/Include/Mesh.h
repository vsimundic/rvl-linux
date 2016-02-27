#pragma once

//#define RVLMESH_BOUNDARY_DEBUG

#ifdef RVLMESH_BOUNDARY_DEBUG
#define RVLMESH_GET_NEXT_BOUNDARY_EDGE(pMesh, iPt, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge)\
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
#define RVLMESH_GET_NEXT_BOUNDARY_EDGE(pMesh, iPt, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge)\
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

#define RVLMESH_GET_NEXT_BOUNDARY_POINT(iPt, pEdge, side, pEdgePtr)\
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
			float t[3];
			float R[9];
			float var[3];
			int RGB[3];
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
		unsigned char RGB[3];
		float P[3];
		float N[3];
		QList<MeshEdgePtr> EdgeList;
		bool bBoundary;
	};

	namespace MESH
	{
		struct PointEdge
		{
			int iPt;
			MeshEdgePtr *pEdgePtr;
			unsigned char side;
		};
	}

	class Mesh : public Graph<Point, MeshEdge, MeshEdgePtr>
	{
		public:
			Mesh();
			virtual ~Mesh();
			void LoadFromPLY(char *PLYFileName);
			void ComputeDistribution(
				Array<int> &PtArray,
				MESH::Distribution &distribution);
			bool FindBoundaryEdge(
				QList<QLIST::Index> *pInPtList,
				int *map,
				int &iPt,
				MeshEdgePtr *&pEdgePtr);
			bool Boundary(
				QList<QLIST::Index> *pInPtList,
				int *Map,
				QList<QLIST::Index> *pOutPtArray,
				QLIST::Index *pMem);
			bool IsBoundaryPoint(
				int iPt,
				int *map,
				int idx,
				MeshEdgePtr *&pEdgePtr)
			{
				Point *pPt = NodeArray.Element + iPt;

				bool bOut = false;

				pEdgePtr = pPt->EdgeList.pFirst;

				int iPt_;
				MeshEdge *pEdge;

				if (pPt->bBoundary)
				{
					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

					if (map[iPt_] == idx)
						return true;
				}

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

				if (bOut)
				{
					pEdgePtr = pPt->EdgeList.pFirst;

					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

					if (map[iPt_] == idx)
						return true;
				}

				return false;
			}

		public:
			vtkSmartPointer<vtkPolyData> pPolygonData;

#ifdef RVLMESH_BOUNDARY_DEBUG		
			int debugState;
#endif
	};
}
