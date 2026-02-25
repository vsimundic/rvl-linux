#pragma once

#define RVLSURFEL_GT_OBJECT_HISTOGRAM
#ifdef RVLVERSION_171125
#ifndef RVLSURFEL_GT_OBJECT_HISTOGRAM
#define RVLSURFEL_GT_OBJECT_HISTOGRAM
#endif
#endif
#define RVLSURFELGRAPH_DEBUG_RELATION_DESCRIPTOR

#define RVLSURFEL_DISPLAY_MODE_SURFELS 0
#define RVLSURFEL_DISPLAY_MODE_BOUNDARY 1
#define RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR 2
#define RVLSURFEL_DISPLAY_MODE_FOREGROUND_BACKGROUND 3
#define RVLSURFEL_DISPLAY_MODE_CONVEX_CONCAVE 4

#define RVLSURFEL_DISPLAY_VERTEX_NORMAL_HULL

#define RVLSURFEL_EDGE_FLAG_HARD 0x01
#define RVLSURFEL_EDGE_FLAG_CONVEX 0x02

#define RVLSURFEL_FLAG_RF 0x01
#define RVLSURFEL_FLAG_GND 0x02
#define RVLSURFEL_FLAG_VOI 0x04

#define RVLSURFELVERTEX_TYPE_CONVEX_CONCAVE 0x07
#define RVLSURFELVERTEX_TYPE_OCCLUSION 0x20
#define RVLSURFELVERTEX_TYPE_TANGENT 0x40
#define RVLSURFELVERTEX_TYPE_REDUNDANT 0x80

#define RVLSURFEL_VERSION_0 0

namespace RVL
{
	class SurfelGraph;

	struct SurfelAdjecencyDescriptors
	{
		double cupyDescriptor[4];
		double minDist;
		int commonBoundaryLength;
		double avgDist;
	};

	namespace SURFEL
	{
		struct DisplayCallbackData
		{
			Visualizer *pVisualizer;
			SurfelGraph *pSurfels;
			Mesh *pMesh;
			void *vpDetector;
			void *vpUserFunctionData;
			bool (*mouseRButtonDownUserFunction)(Mesh *pMesh, SurfelGraph *pSurfels, int iSelectedPt, int iSelectedSurfel, void *vpData);
			bool (*keyPressUserFunction)(Mesh *pMesh, SurfelGraph *pSurfels, std::string &key, void *vpData);
			bool bCallbackFunctionsDefined;
			int mode;
			unsigned char SelectionColor[3];
			unsigned char ForegroundColor[3];
			unsigned char BackgroundColor[3];
			unsigned char ConvexColor[3];
			unsigned char ConcaveColor[3];
			int iSelectedSurfel;
			int iSelectedSurfel2;
			int iSelection;
			float edgeFeatureDepth;
			vtkSmartPointer<vtkPolyData> edgeFeaturesPolyData;
			vtkSmartPointer<vtkActor> edgeFeatures;
			vtkSmartPointer<vtkActor> vertices;
			std::vector<vtkSmartPointer<vtkActor>> selectedActors;
			int *edgeFeatureIdxArray;
			float normalLen;
			bool bEdges;
			bool bVertices;
			bool bFirstKey;
			bool bPolygons;
		};

		struct EdgePtr;

		struct Edge
		{
			int iVertex[2];
			EdgePtr *pVertexEdgePtr[2];
			unsigned char flags;
			int idx;
			Edge *pNext;
		};

		struct EdgePtr
		{
			Edge *pEdge;
			EdgePtr *pNext;
		};

		struct NormalHullElement
		{
			float N[3];
			float Nh[3];
			float snq;
		};

		struct VertexEdge
		{
			int iVertex[2];
			GRAPH::EdgePtr2<VertexEdge> *pVertexEdgePtr[2];
			int idx;
			float N[3];
			int iSurfel[2];
			VertexEdge *pNext;
		};

		struct Vertex
		{
			float P[3];
			int idx;
			QList<GRAPH::EdgePtr2<VertexEdge>> EdgeList;
			Array<NormalHullElement> normalHull;
			Array<int> iSurfelArray;
			Vertex *pNext;
			bool bEdge;
			bool bForeground;
			BYTE type;
			float VTX[3];
			int iCluster;
		};

		struct PlaneDetectionRGData
		{
			float NRef[3];
			float csqThr;
			bool *bVisited;
		};

		struct SceneSamples
		{
			Array2D<float> imagePtArray;
			int w, h;
			Pair<int, int> *PtIdxArray;
			Array<float *> PtArray;
			float *PGnd;
			float *g;
			float *SDF;
			float *PMem;
			float Pc[3];
			uchar *status; // 0 - no point; 1 - foreground; 2 - background
			bool bCentered;
		};

		struct PolyEdgeGraphEdge;

		struct PolyEdge
		{
			int iSurfel;
			int iPolygon;
			int iEdge;
			int iVertex[2];
			float V[3];
			float len;
		};

		struct PolyEdgeGraphEdge
		{
			int idx;
			int iVertex[2];
			GRAPH::EdgePtr<PolyEdgeGraphEdge> *pVertexEdgePtr[2];
			float P[2][3];
			float dist2;
		};

		void DeleteSceneSamples(SURFEL::SceneSamples &sceneSamples);
		bool DefinePlaneInteractive(
			SurfelGraph *pSurfels,
			std::string keySym,
			int iSelectedSurfel);
		void UpdateNormalHull(
			Array<NormalHullElement> &NHull,
			float *N);
		float DistanceFromNormalHull(
			Array<SURFEL::NormalHullElement> &NHull,
			float *N);
	} // namespace SURFEL

	struct Surfel
	{
		QList<QLIST::Index2> PtList;
		Array<Array<MeshEdgePtr *>> BoundaryArray;
		Array<Array<MeshEdgePtr *>> PolygonBoundaryArray;
		float P[3];	 // centroid
		float N[3];	 // normal
		float R[9];	 // rotation matrix (orientation of the camera RF w.r.t. surfel RF)
		float d;	 // plane offset
		int RGB[3];	 // average color
		float P0[3]; // central point
		float V[3];
		int size;
		float r0;	  // distance
		float r1, r2; // radii of the approximating ellipse
		QList<SURFEL::EdgePtr> EdgeList;
		Surfel *pNext;
		float physicalSize;
		bool bEdge;
		BYTE flags;
		QList<QLIST::Index> children;
		Moments<double> *pMoments;
		float *representativePts;
		int ObjectID;										 // Filko
		std::shared_ptr<RVLColorDescriptor> colordescriptor; // Filko
#ifdef RVLSURFEL_IMAGE_ADJACENCY
		std::vector<Surfel *> imgAdjacency;								   // Filko
		std::vector<SurfelAdjecencyDescriptors *> imgAdjacencyDescriptors; // Filko
#endif
		// RVLColorDescriptor *colordescriptor; //Filko
#ifdef RVLSURFEL_GT_OBJECT_HISTOGRAM
		std::vector<int> GTObjHist; // Filko
#endif
		Array<Pair<int, int>> polygonVertexIntervals;
		Array<int> triangles;
	};

	void SampleMesh(
		Mesh *pMesh,
		float *R,
		float *t,
		Array<MESH::Sample> &sampleArray);
	void SampleMeshDistanceFunction(
		Mesh *pMesh,
		SurfelGraph *pSurfels,
		float voxelSize,
		int sampleVoxelDistance,
		Array3D<Voxel> &volume,
		float *P0,
		Array<MESH::Sample> &sampleArray,
		Box<float> &boundingBox);
#ifdef RVLVTK
	void DisplaySampledMesh(
		Visualizer *pVisualizer,
		Array3D<Voxel> volume,
		float *P0,
		float voxelSize);
#endif

	class SurfelGraph : public Graph<Surfel, MeshEdge, MeshEdgePtr>
	{
	public:
		SurfelGraph();
		virtual ~SurfelGraph();
		void CreateParamList(CRVLMem *pMem);
		void InitGetNeighborsBoundaryAndSize();
		void FreeGetNeighborsBoundaryAndSize();
		void DetectVertices(
			Mesh *pMesh);
		void UpdateNormalHull(
			Array<SURFEL::NormalHullElement> &NHull,
			float *N);
		float DistanceFromNormalHull(
			Array<SURFEL::NormalHullElement> &NHull,
			float *N);
		bool ComputeTangent(
			Array<int> iVertexArray,
			float *N,
			float o,
			bool bNormalTest,
			bool bVisibilityTest,
			float maxDistFromNHull,
			float maxedmax,
			float &d,
			bool &bd,
			int &iVertex);
		bool ComputeTangent(
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
			int &iVertex);
		bool ComputeTangent(
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
			int &iVertex);
		float Distance(
			Surfel *pSurfel,
			float *P,
			bool bUncertainty = false);
		void GetVertices(
			QList<QLIST::Index> surfelList,
			Array<int> *piVertexArray,
			int *&piVertexIdxMem);
		int GetNoVertices(QList<QLIST::Index> surfelList);
		bool BoundingBox(
			Array<int> iVertexArray,
			float *R,
			float *t,
			float scale,
			Box<float> &boundingBox);
		void Centroid(
			Array<int> iSurfelArray,
			float *centroid);
		void CenterVertices(
			Array<int> iVertexArray,
			float *P,
			float *centroid);
		void VertexCentroid(
			Array<int> iVertexArray,
			float *centroid);
		void DetectDominantPlane(
			Mesh *pMesh,
			Array<int> &dominantPlaneSurfelArray,
			float *N,
			float &d);
		void ComputeDistribution(
			Mesh *pMesh,
			Array<int> iSurfelArray,
			GaussianDistribution3D<float> *pDistribution,
			int *PtMem = NULL);
		bool FitPlane(
			Mesh *pMesh,
			Array<int> iSurfelArray,
			MESH::Distribution &PtDistribution,
			Array<int> &PtArray);
		void TransformVertices(
			Array<int> iVertexArray,
			float scale,
			float *R,
			float *t,
			float *PArray);
		void ProjectVerticesOntoGroundPlane(
			Array<int> iVertexArray,
			float *NGnd,
			float dGnd,
			float *PGnd);
		void SampleMeshDistanceFunction(
			Mesh *pMesh,
			float *R,
			float *t,
			int nSamplePts,
			float voxelSize,
			int sampleVoxelDistance,
			Array<MESH::Sample> &sampleArray);
		void SampleSurfelSet(
			Mesh *pMesh,
			Array<int> iSurfelArray,
			Array<int> iVertexArray,
			Camera camera,
			SURFEL::SceneSamples &sceneSamples,
			bool bExternalSamples = false,
			bool bCenter = true);
		bool ExternalSample(
			Mesh *pMesh,
			Camera camera,
			Rect<int> cropWin,
			float *m,
			int halfImageNeighborhood,
			bool *bBelongsToObject,
			float &dist,
			float *P);
		void GetPoints(
			Array<int> iSurfelArray,
			Array<int> &iPtArray);
		void GetVertices(Array2D<float> &vertices);
		void GetVertices(
			Array<int> iVertexArray,
			Array2D<float> &vertices);
		void GetVertices(
			int iSurfel,
			Array2D<float> &vertices);
		int SupportSize(Array<int> iSurfelArray);
		bool InVOI(
			Array<int> iVertexArray,
			float *RSG,
			float *tSG,
			float r);
		void SurfelsInVOI(
			float *RSG,
			float *tSG,
			float r);
		void CreatePolygonGraph(float distThr);
		void NodeColors(unsigned char *SelectionColor);
		void Display(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			int iSelectedSurfel = -1,
			unsigned char *SelectionColor = NULL,
			int *ColorScale = NULL,
			unsigned char *ColorOffset = NULL,
			bool bDisplayVertices = false);
		void DisplayHardEdges(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			int iSurfel,
			unsigned char *Color);
		void DisplayEdgeFeatures();
		void DisplayForegroundAndBackgroundEdges(
			Visualizer *pVisualizer,
			Mesh *pMesh);
		void DisplayVertexGraph(Visualizer *pVisualizer);
		void Init(Mesh *pMesh);
		void Clear();
		unsigned char *GetColor(int iSurfel);
		void PrintData(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			int iVertex,
			int iSurfel);
		void InitDisplay(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			void *vpDetector,
			bool bCallbackFunctions = true);
		void DisplaySurfelBoundary(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			int iSurfel,
			unsigned char *Color);
		void DisplayPolygons(
			Visualizer *pVisualizer,
			uchar *color,
			bool bDisplayEdgeGraph = false,
			bool bMultiColor = false);
		void DisplayVertices();
		void UpdateVertexDisplayLines();
		void PaintVertices(
			Array<int> *pVertexArray,
			unsigned char *color);
		void PaintSurfels(
			Mesh *pMesh,
			Visualizer *pVisualizer,
			Array<int> iSurfelArray,
			unsigned char *clusterColor,
			unsigned char *edgeColor = NULL,
			int *clusterMap = NULL);
		void DisplayRGB(cv::Mat RGB);
		void DisplaySphericalNormalHistogram(
			Array<int> surfels,
			Visualizer *pVisualizer);
		void SetDisplayPolygonsOn();
		void Save(
			int iSurfel,
			Mesh *pMesh,
			FILE *fpPoints,
			FILE *fpEdges);
		void SaveSurfel(
			FILE *fp,
			int iSurfel);
		void LoadSurfel(
			FILE *fp,
			int iSurfel);
		void Save(
			FILE *fp,
			char *meshFileName,
			void *vpDetector);
		void CalculateSurfelsColorHistograms(
			Mesh *pMesh,
			int colorspace,
			bool oneDimensional,
			const int *bindata,
			bool noBins,
			const int *chFilterThrreshold = NULL,
			bool useFilter = false); // Filko
#ifdef RVLSURFEL_IMAGE_ADJACENCY
		void ImageAdjacency(Mesh *pMesh);
		void ImageAdjacency(
			Mesh *pMesh,
			int iSurfel,
			int *surfelIdx,
			bool *bVisited);
		void DetermineImgAdjDescriptors(
			Surfel *pSurfel,
			Mesh *mesh);
		void SurfelAreaDistribution(
			Mesh *mesh,
			Surfel *pSurfel,
			int iBoundary,
			float *dN,
			float dOffset,
			float *a);
		void SurfelRelations(Mesh *pMesh);
		void GenerateSSF(
			std::string filename,
			int minSurfelSize,
			bool checkbackground);
		void SplitAndMergeError(
			Surfel *pCurrentSurfel,
			Surfel *pOtherSurfel,
			int nGTObjects,
			int &splitError,
			int &mergeError);
		void CalculateSurfelsColorHistograms(cv::Mat img, int colorspace, bool oneDimensional, const int *bindata, bool noBins);
		void DisplayConvexAndConcaveEdges(
			Visualizer *pVisualizer,
			Mesh *pMesh);
#endif

#ifdef RVLSURFEL_GT_OBJECT_HISTOGRAM
		void SetPrimaryGTObj(
			Surfel *pSurfel,
			cv::Mat labGTImg,
			int noObj);
		void AssignGroundTruthSegmentation(
			char *meshFileName,
			int minSurfelSize);
#endif
		cv::Mat GenColoredSurfelImg();
		cv::Mat GenColoredSurfelImgFromSSF(std::shared_ptr<SceneSegFile::SceneSegFile> ssf);
		void GetDepthImageROI(
			Array<int> iVertexArray,
			Camera camera,
			Rect<float> &ROI);
		void DetectOcclusionVertices(
			Mesh *pMesh,
			Camera camera);
		void HideOcclustionFaces(
			Mesh *pMesh,
			Array<int> iVertexArray);
		// Filko
		bool Coplanar(
			Moments<double> moments1,
			Array<int> iVertexArray1,
			Moments<double> moments2,
			QList<QLIST::Index> *pVertexList2,
			float tolerance);
		bool Coplanar(
			float *N,
			float d,
			Array<int> iVertexArray,
			float tolerance);
		bool Coplanar(
			float *N,
			float d,
			QList<QLIST::Index> *pVertexList,
			float tolerance);
		bool Below(int iSurfel, int iSurfel_);
		void EdgePointNormals(Mesh *pMesh);
		void RepresentativeSurfelSamples(
			Mesh *pMesh,
			int minSurfelSize,
			int minEdgeSize);
		void RepresentativeComplexSurfelSamples(
			Mesh *pMesh,
			int minSurfelSize,
			int minEdgeSize,
			SurfelGraph *pElements);

	public:
		CRVLParameterList ParamList;
		float sizeUnit;
		bool bGroundContactVertices;
		int nMeshVertices;
		int nMeshEdges;
		QLIST::Index2 *PtMem;
		int *surfelMap;
		int *edgeMap;
		MeshEdgePtr **surfelBndMem;
		Array<MeshEdgePtr *> *surfelBndMem2;
		// QLIST::Index2 *surfelBndMap;
		unsigned char *edgeMarkMap;
		CRVLMem *pMem;
		SURFEL::Edge **neighborEdge;
		Array<SURFEL::Edge *> EdgeArray;
		SURFEL::DisplayCallbackData DisplayData; // VIDOVIC
		QList<QLIST::Entry<Array<MeshEdgePtr *>>> BoundaryList;
		MeshEdgePtr **BndMem;
		int imageAdjacencyThr;
		int nImageAdjacencyRelations;
		QList<SURFEL::Vertex> vertexList;
		Array<SURFEL::Vertex *> vertexArray;
		Array<QList<QLIST::Index>> surfelVertexList;
		int nVertexSurfelRelations;
		float TIVertexToleranceAngle;
		QList<SURFEL::VertexEdge> vertexEdgeList;
		Array<SURFEL::VertexEdge *> vertexEdgeArray;
		int edgeDepth;
		bool *bVertexAssigned;
		int *iVertexMem;
		bool bContactEdgeVertices;
		float occlusionVertexMinZ;
		float occlusionVertexMaxZ;
		float occlusionVertexResolutionZ;
		int occlusionVertexWinSize;
		int occlusionVertexMeanShiftWinSize;
		int occlusionVertexMinClusterSize;
		int occlusionVertexMinDepthStep;
		Moments<double> *momentsMem;
		float *surfelRefPtMem;
		QLIST::Index *planarSurfaceSurfelMem;
		std::vector<Point2D> polygonVertices;
		Array<Vector3<float>> polygonVerticesS;
		Pair<int, int> *polygonDataMem;
		Array<SURFEL::PolyEdge> polyEdges;
		Graph<GRAPH::Node_<GRAPH::EdgePtr<SURFEL::PolyEdgeGraphEdge>>, SURFEL::PolyEdgeGraphEdge, GRAPH::EdgePtr<SURFEL::PolyEdgeGraphEdge>> polyEdgeGraph;
		std::vector<SURFEL::PolyEdgeGraphEdge> polygonEdgeGraphEdges;
		Graph<GRAPH::Node, GRAPH::Edge, GRAPH::EdgePtr<GRAPH::Edge>> surfelGraph2;
		int *triangleMem;

	private:
		unsigned char *nodeColor;
		QLIST::Index *surfelVertexMem;
		Array<Array<int>> vertexDisplayLineArray;
		int *vertexDisplayLineArrayMem;
		vtkSmartPointer<vtkPolyData> linesPolyData;
#ifdef RVLSURFELGRAPH_DEBUG_RELATION_DESCRIPTOR
		bool bDebug;
		FILE *fpDebug;
#endif
	};

	namespace SURFEL
	{
		void ComputeParameters(
			Surfel *pSurfel,
			MESH::Distribution &distribution,
			Point *pPt);
		void CreateFromPoint(
			Surfel *pSurfel,
			Point *pPt);
		void GetPoint(
			Surfel *pSurfel,
			Point *pPoint);
		void MouseRButtonDown(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata);
		void KeyPressCallback(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata);
		int PlaneDetectionRG(
			int iSurfel,
			int iSurfel_,
			Edge *pEdge,
			SurfelGraph *pSurfels,
			SURFEL::PlaneDetectionRGData *pData);
	};
}
