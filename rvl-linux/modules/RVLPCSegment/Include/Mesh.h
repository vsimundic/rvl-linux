#pragma once

#define RVLMESH_POINT_FLAG_FOREGROUND 0x01
#define RVLMESH_POINT_FLAG_BACKGROUND 0x02
#define RVLMESH_POINT_FLAG_EDGE_CLASS 0x03

#define RVLMESH_FLAG_NORMALS 0x00000001
#define RVLMESH_FLAG_COLOR 0x00000002

#define RVLMESH_FACE_FLAG_REJECTED 0x01
#define RVLMESH_FACE_FLAG_VISIBLE 0x02

// #define RVLMESH_BOUNDARY_DEBUG

// Input:  mesh pMesh,
//         vertex idx. iPt,
//         connector pEdgePtr connecting an edge E to the vertex iPt,
//         array map defining a region R (map[i] = map[iPt] for all vertices i of the mesh pMesh belonging to the region R)
// Output: pEdge <- the next edge E' in EdgeList[iPt] following E, such that Opp(E', iPt) is in R, where Opp is defined in ARP3D.TR3
//         iNeighborPt <- Opp(E', iPt)
//         pEdgePtr <- connector connecting pEdge to iPt,
// Temporary variables: pPt, pEdgeList, side

#ifdef RVLMESH_BOUNDARY_DEBUG
#define RVLMESH_GET_NEXT_IN_REGION(pMesh, iPt, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge) \
	{                                                                                                   \
		pPt = pMesh->NodeArray.Element + iPt;                                                           \
		pEdgeList = &(pPt->EdgeList);                                                                   \
		do                                                                                              \
		{                                                                                               \
			RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr)                                             \
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side)                   \
			fprintf(fpDebug, "%d (%d) ", iNeighborPt, map[iNeighborPt]);                                \
		} while (map[iNeighborPt] != map[iPt]);                                                         \
	}
#else
#define RVLMESH_GET_NEXT_IN_REGION(pMesh, iPt, pEdgePtr, side, map, iNeighborPt, pPt, pEdgeList, pEdge) \
	{                                                                                                   \
		pPt = pMesh->NodeArray.Element + iPt;                                                           \
		pEdgeList = &(pPt->EdgeList);                                                                   \
		do                                                                                              \
		{                                                                                               \
			RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr)                                             \
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR2(iPt, pEdgePtr, pEdge, iNeighborPt, side)                   \
		} while (map[iNeighborPt] != map[iPt]);                                                         \
	}
#endif

// Input: pEdge, side
// Output: iPt <- point on the side side of the edge pEdge; pEdgePtr <- connector of point iPt and pEdge

#define RVLMESH_GET_POINT(pEdge, side, iPt, pEdgePtr) \
	{                                                 \
		iPt = pEdge->iVertex[side];                   \
		pEdgePtr = pEdge->pVertexEdgePtr[side];       \
	}

namespace RVL
{
	// Move to RVL3DTools.h

	template <typename Type>
	struct GaussianDistribution3D
	{
		Type P[3];
		Type C[9];
	};

	/////

	struct MeshEdgePtr;

	namespace MESH
	{
		struct Distribution
		{
			float t[3];	  // centroid
			float R[9];	  // principal axes (each row is one axis)
			float var[3]; // point variances in directions of the principal axes
			int RGB[3];	  // average color
		};

		struct Sample
		{
			float P[3];
			int iFeature;
			float SDF;
		};

		struct IntPlane
		{
			int N[3];
			int d;
		};

		struct Face
		{
			int idx;
			float N[3];
			float d;
			float Area;
			MeshEdgePtr *pFirstEdgePtr;
			uchar flags;
			IntPlane *piPlane;
			Face *pNext;
		};
	}

	struct MeshEdge
	{
		int iVertex[2];
		MeshEdgePtr *pVertexEdgePtr[2];
		MESH::Face *pFace[2];
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
		uint8_t RGB[3];				 // color
		float P[3];					 // position
		float N[3];					 // normal
		int label;					 // label
		QList<MeshEdgePtr> EdgeList; // edge list (list of edge connectors)
		bool bBoundary;				 // true if the point is on the image boundary, on a depth discontinuity contur or on the boundary of a region of undefined depth,
									 // i.e. if there is a boundary edge connected to this point.
		bool bValid;
		BYTE flags;
	};

	struct OrientedPoint
	{
		float P[3];
		float N[3];
	};

	struct Voxel
	{
		QList<QLIST::Index> PtList;
		int voxelDistance;
	};

	namespace MESH
	{
		struct PointEdge
		{
			int iPt;			   // vertex index
			MeshEdgePtr *pEdgePtr; // connector connecting an edge E to the vertex iPt
			unsigned char side;	   // side of the edge E to which the iPt is connected
		};

		struct Triangle
		{
			int iVertex[3];
			int iTetrahedron[2];
			bool bSurface;
		};

		struct Tetrahedron
		{
			int iVertex[4];
			bool bValid;
			int iTriangle[4];
		};

		struct Tetrahedrons
		{
			Array<Point> vertices;
			Array<Tetrahedron> tetrahedrons;
			Array<Triangle> triangles;
			Array<MeshEdge> edges;
			bool *bEdgeValid;
			CRVLMem mem;
		};

		template <typename T>
		void BoundingBox(
			vtkSmartPointer<vtkPolyData> pPolygonData,
			Box<T> *pBox)
		{
			int nPts = pPolygonData->GetNumberOfPoints();

			if (nPts == 0)
				return;

			vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pPolygonData->GetPoints()->GetData());

			if (pointData == NULL)
				return;

			float P[3];

			pointData->GetTypedTuple(0, P);

			T P_[3];

			RVLCOPY3VECTOR(P, P_);

			InitBoundingBox<T>(pBox, P_);

			int iPt;

			for (iPt = 1; iPt < nPts; iPt++)
			{
				pointData->GetTypedTuple(iPt, P);

				RVLCOPY3VECTOR(P, P_);

				UpdateBoundingBox<T>(pBox, P_);
			}
		}
		void BoundingBox(
			vtkSmartPointer<vtkPolyData> pPolygonData,
			Box<float> *pBox);
		void TSDF(
			vtkSmartPointer<vtkPolyData> pPolyData,
			float voxelSize,
			int border,
			Array3D<Voxel> &volume,
			float *P0,
			Box<float> &boundingBox,
			Array<int> &zeroDistanceVoxelArray,
			QLIST::Index *&PtMem);
		void TSDF2(
			Tetrahedrons &tetrahedrons,
			float resolution,
			float distanceValuePerVoxel,
			Array3D<float> &TSDF,
			float *P0,
			Box<float> &boundingBox,
			float &voxelSize,
			bool bRoom = false);
		void TSDF3(
			vtkSmartPointer<vtkPolyData> pPolyData,
			float resolution,
			float distanceValuePerVoxel,
			Array3D<float> &TSDF,
			float *P0,
			Box<float> &boundingBox,
			float &voxelSize);
		void OrientedPCFromRGBD(
			Array2D<short int> depthImage,
			Camera camera,
			Array2D<OrientedPoint> &PC,
			int winSize,
			float maxdz);
		void PointTDF(
			vtkSmartPointer<vtkPolyData> pPolyData,
			float resolution,
			float distanceValuePerVoxel,
			float sigp,
			Array3D<float> &TDF,
			float *P0,
			Box<float> &boundingBox,
			float &voxelSize);
		void OrientedPointDF(
			vtkSmartPointer<vtkPolyData> pPolyData,
			float resolution,
			float distanceValuePerVoxel,
			float sigp,
			Array3D<float> &DF,
			float *&W,
			float *P0,
			Box<float> &boundingBox,
			float &voxelSize);
		void CreateVoxelGrid(
			Array<Point> vertices,
			float resolution,
			float distanceValuePerVoxel,
			Array3D<float> &f,
			Box<float> &boundingBox,
			Box<float> &box,
			float &voxelSize,
			float *P0);
		void TriangleTDF(
			Array<Triangle> triangles,
			Array<Point> vertices,
			float resolution,
			float distanceValuePerVoxel,
			Array3D<float> &TSDF,
			float *P0,
			Box<float> &boundingBox,
			float &voxelSize);
		void TriangleTDF(
			Triangle *pTriangle,
			Array<Point> vertices,
			float voxelSize,
			float distanceValuePerVoxel,
			Box<float> box,
			Array3D<float> &TSDF);
		void OrientedPCFromDepthImage(
			Array2D<short int> depthImage,
			Camera camera,
			Array2D<OrientedPoint> &PC,
			int winSize = 11,
			float maxdz = 0.03f);
#ifdef RVLVTK
		vtkSmartPointer<vtkPolyData> CreateVisibleSurfaceMesh(
			vtkSmartPointer<vtkPolyData> pPolygonDataSrc,
			float voxelSize,
			int nFilter,
			float SDFIsoValue = 0.0f);
		vtkSmartPointer<vtkPolyData> CreateVisibleSurfaceMesh2(
			vtkSmartPointer<vtkPolyData> pPolygonDataSrc,
			float resolution,
			float SDFIsoValue = 0.0f,
			int nFilter = 0);
		vtkSmartPointer<vtkPolyData> CreateVisibleSurfaceMeshFromTetrahedrons(
			Tetrahedrons &tetrahedrons,
			float resolution,
			float SDFIsoValue = 0.0f,
			bool bRoom = false);
		vtkSmartPointer<vtkPolyData> VTKPolyDataFromOrganizedOrientedPC(
			Array2D<OrientedPoint> PC,
			float maxTriangleEdgeLen);
		void MapRGBImageToVTKPolyData(
			vtkSmartPointer<vtkPolyData> pPolygonData,
			char *rgbImageData,
			int rgbImageWidth,
			int rgbImageHeight);
		void MapRGBImageToVTKPolyData(
			vtkSmartPointer<vtkPolyData> pPolygonData,
			IplImage *rgbImagePNG);
		vtkSmartPointer<vtkPolyData> RGBD2VTKPolyData(
			std::string RGBFileName,
			std::string depthFileName,
			Camera camera,
			float maxTriangleEdgeLen);
		void RGBD2PLY(
			std::string RGBFileName,
			std::string depthFileName,
			Camera camera,
			float maxTriangleEdgeLen,
			std::string PLYFileName);
#endif
		void CreatePointArrayFromOrientedPointArray(
			Array<OrientedPoint> PtArray,
			Array<Point> &PtArray_,
			float maxZ = 1e6);
		void CreateOrientedPointArrayFromPointArray(
			Array<Point> PtArraySrc,
			Array<OrientedPoint> &PtArrayTgt);
		void CreateOrientedPointArrayFromPointArray(
			Array<Point> PtArraySrc,
			Array<int> PtIdx,
			Array<OrientedPoint> &PtArrayTgt);
		void LoadTetrahedrons(
			char *fileName,
			RVL::MESH::Tetrahedrons &tetrahedrons);
		void ComputeTetrahedronData(RVL::MESH::Tetrahedrons &tetrahedrons);
		void ClearTetrahedronData(RVL::MESH::Tetrahedrons tetrahedrons);
		void MouseRButtonDown(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata);
		void LabelParts(
			vtkSmartPointer<vtkPolyData> pDensePolygonData,
			vtkSmartPointer<vtkPolyData> pSparsePolygonData,
			char *labelFileName);
		void SparsePointsCorrespondences(
			vtkSmartPointer<vtkPolyData> pDensePolygonData,
			vtkSmartPointer<vtkPolyData> pSparsePolygonData,
			char *labelFileName);
		void ComputeDistributionDouble(
			Moments<double> moments,
			MESH::Distribution &distribution);
		void ComputePlaneParameters(
			MESH::Distribution &distribution,
			float *N,
			float &d);
	} // namespace MESH

	void FilterSDF(
		Array3D<Voxel> volume,
		Array3D<float> filter,
		int n,
		Array3D<float> &SDF);
	void FilterSDF(
		Array3D<float> SDFSrc,
		Array3D<float> filter,
		int n,
		Array3D<float> &SDF);
	void CreateMeanFilter(
		Array3D<float> &filter,
		int n = 3);
	float PointSetToPlaneDistance(
		Array<Point> points,
		float *RMS,
		float *tMS,
		float *N,
		float d);

	class Mesh : public Graph<Point, MeshEdge, MeshEdgePtr>
	{
	public:
		Mesh();
		virtual ~Mesh();
		// bool Load(
		//	char *FileName,
		//	PCLMeshBuilder *pMeshBuilder,
		//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
		//	pcl::PolygonMesh &PCLMesh,
		//	bool bSavePLY = false);
		void LoadFromPLY(
			char *PLYFileName,
			float maxMeshTriangleEdgeLen,
			bool bOrganizedPC = false,
			Camera *pCamera = NULL);
		void LoadPolyDataFromPLY(char *PLYFileName);
		void SavePolyDataToPLY(char *PLYFileName);
		void SaveNoisedPolyDataToPLY(char *PLYFileName);
		bool CreateOrderedMeshFromPolyData(
			int *pixMap = NULL,
			int pixMapSize = 0,
			float maxPolyEdgeLen = -1.0f);
		void ComputeMoments(Array<int> &PtArray,
							Moments<float> &moments);
		void ComputeMoments(Array<int> &PtArray,
							Moments<double> &moments);
		void ComputeDistribution(
			Array<int> &PtArray,
			GaussianDistribution3D<float> *pDistribution);
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
		bool ConvexHull(
			Array2D<float> points,
			CRVLMem *pMem,
			bool bClearMem = true,
			bool bDepthImage = true);
		bool ConvexHull2(
			Array2D<float> points,
			CRVLMem *pMem,
			bool bClearMem = true,
			bool bDepthImage = true);
		void VisibleSurface(
			Array2D<float> &N,
			float *&d);
		void VoxelGridFilter(
			float voxelSize,
			Array<OrientedPoint> &outputPts,
			Array3D<int> &grid,
			Box<double> &boundingBox,
			Array<int> &voxels,
			float scale = 1.0f,
			Array<int> *pSubset = NULL,
			bool bHollow = false);
		void VoxelGridFilter(
			float voxelSize,
			float normalProxThr,
			Array<OrientedPoint> &outputPts,
			Array3D<QList<QLIST::Index>> &grid,
			Box<double> &boundingBox,
			Array<int> &voxels,
			float scale = 1.0f,
			Array<int> *pSubset = NULL);
		void DistanceTransform(
			Array3D<int> grid,
			Box<double> boundingBox,
			Array<int> voxels);
		void CreateVTKPolyData(
			float *R = NULL,
			float *t = NULL,
			bool bVisibleFacesOnly = false);
		void Transform(
			float *R,
			float *t);
		int FurthestPoint(
			float *P,
			QList<QLIST::Index2> ptList);
		int FurthestPoint(
			float *P,
			Array<MeshEdgePtr *> ptArray,
			float *V = NULL);
		void GetDepth(Array2D<short> &depthImg);
		void LoadLabels(char *labelFileName);
		void SaveConvexHullVertices(FILE *fp);
		void GetBGR(cv::Mat &BGR);

		// For a mesh point index iPt, the function returns true if the point is on the boundary of a region in the map map containing elements with value idx.
		// pEdgePtr <- the connector connecting the first region boundar edge in CCW direction.

		inline bool IsBoundaryPoint(
			int iPt,
			int *map,
			int idx,
			MeshEdgePtr *&pEdgePtr)
		{
			Point *pPt = NodeArray.Element + iPt;

			bool bOut = false; // A neighboring vertex belonging to another region is found.

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
		Array<MESH::Face *> faces;
		MESH::Face *faceMem;
		vtkSmartPointer<vtkPolyData> pPolygonData;
		int *mapNodesToPolyData;
		DWORD flags;
		float normalEstimationRadius;
		int nBoundaryPts;
		bool bOrganizedPC;
		int width;
		int height;
		Array<int> iValidVertices;
		Array<int> iVisibleFaces;
		bool bLabels;
		char *name;
		int idPrimitive;
		int idCluster;

#ifdef RVLMESH_BOUNDARY_DEBUG
		int debugState;
#endif
	};
}
