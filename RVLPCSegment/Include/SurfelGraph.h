#pragma once

#define RVLSURFEL_IMAGE_ADJACENCY //FILKO usporava debug :)

#define RVLSURFEL_DISPLAY_MODE_SURFELS					0
#define RVLSURFEL_DISPLAY_MODE_BOUNDARY					1
#define RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR			2
#define RVLSURFEL_DISPLAY_MODE_FOREGROUND_BACKGROUND	3
#define RVLSURFEL_DISPLAY_MODE_CONVEX_CONCAVE			4

#define RVLSURFEL_DISPLAY_VERTEX_NORMAL_HULL

#define RVLSURFEL_EDGE_FLAG_HARD				0x01
#define RVLSURFEL_EDGE_FLAG_CONVEX				0x02
#define RVLSURFEL_FLAG_RF						0x04

#define RVLSURFEL_VERSION_0		0

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
			bool(*mouseRButtonDownUserFunction)(Mesh *pMesh, SurfelGraph *pSurfels, int iSelectedPt, int iSelectedSurfel, void *vpData);
			bool(*keyPressUserFunction)(Mesh *pMesh, SurfelGraph *pSurfels, std::string &key, void *vpData);
			bool bCallbackFunctionsDefined;
			int  mode;
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
			float normalLen;
			bool bEdges;
			bool bVertices;
			bool bFirstKey;
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
		};

		struct Vertex
		{
			float P[3];
			Array<NormalHullElement> normalHull;
			Array<int> iSurfelArray;
			Vertex *pNext;
			bool bEdge;
			BYTE type;
			float VTX[3];
		};


	}

	struct Surfel
	{
		QList<QLIST::Index2> PtList;
		Array<Array<MeshEdgePtr *>> BoundaryArray;
		Array<Array<MeshEdgePtr *>> PolygonBoundaryArray;
		float P[3];		// centroid
		float N[3];		// normal
		float R[9];		// rotation matrix (orientation of the camera RF w.r.t. surfel RF)
		float d;		// plane offset
		int RGB[3];		// average color
		float P0[3];	// central point
		float V[3];
		int size;
		float r0;		// distance 
		float r1, r2;	// radii of the approximating ellipse
		QList<SURFEL::EdgePtr> EdgeList;
		Surfel *pNext;
		float physicalSize;
		bool bEdge;
		BYTE flags;
		int ObjectID;	//Filko
#ifdef 	RVLSURFEL_IMAGE_ADJACENCY
		std::vector<Surfel*> imgAdjacency;	//Filko
		std::vector<SurfelAdjecencyDescriptors*> imgAdjacencyDescriptors;	//Filko
		std::vector<int> GTObjHist; //Filko
		RVLColorDescriptor *colordescriptor; //Filko
#endif
	};

	class SurfelGraph : public Graph < Surfel, MeshEdge, MeshEdgePtr >
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
		float Distance(
			Surfel *pSurfel,
			float *P,
			bool bUncertainty = false);
		void GetVertices(
			QList<QLIST::Index> surfelList,
			Array<int> *piVertexArray,
			int *&piVertexIdxMem);
		void NodeColors(unsigned char *SelectionColor);
		void Display(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			int iSelectedSurfel = -1,
			unsigned char *SelectionColor = NULL,
			int *ColorScale = NULL,
			unsigned char *ColorOffset = NULL);
		void DisplayHardEdges(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			int iSurfel,
			unsigned char *Color);
		void DisplayEdgeFeatures();
		void DisplayForegroundAndBackgroundEdges(
			Visualizer *pVisualizer,
			Mesh *pMesh);
		void Init(Mesh *pMesh);
		void Clear();
		unsigned char * GetColor(int iSurfel);
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
			Mesh * pMesh, 
			int iSurfel,
			unsigned char *Color);
		void DisplayVertices();
		void UpdateVertexDisplayLines();
		void PaintVertices(
			Array<int> *pVertexArray,
			unsigned char *color);
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
		void SetPrimaryGTObj(
			Surfel *pSurfel, 
			cv::Mat labGTImg, 
			int noObj);
		void AssignGroundTruthSegmentation(
			char *meshFileName,
			int minSurfelSize);
		void CalculateSurfelsColorHistograms(cv::Mat img, int colorspace, bool oneDimensional, const int *bindata, bool noBins);
		void DisplayConvexAndConcaveEdges(
			Visualizer *pVisualizer,
			Mesh *pMesh);
#endif
		cv::Mat GenColoredSurfelImgFromSSF(std::shared_ptr<SceneSegFile::SceneSegFile> ssf);
		//Filko

	public:	
		CRVLParameterList ParamList;
		int nMeshVertices;
		int nMeshEdges;
		QLIST::Index2 *PtMem;
		int *surfelMap;
		int *edgeMap;
		MeshEdgePtr **surfelBndMem;
		Array<MeshEdgePtr *> *surfelBndMem2;
		//QLIST::Index2 *surfelBndMap;
		unsigned char *edgeMarkMap;
		CRVLMem *pMem;
		SURFEL::Edge **neighborEdge;
		Array<SURFEL::Edge *> EdgeArray;
		SURFEL::DisplayCallbackData DisplayData; //VIDOVIC
		QList<QLIST::Entry<Array<MeshEdgePtr *>>> BoundaryList;
		MeshEdgePtr **BndMem;
		int imageAdjacencyThr;
		int nImageAdjacencyRelations;
		QList<SURFEL::Vertex> vertexList;
		Array<SURFEL::Vertex *> vertexArray;
		Array<QList<QLIST::Index>> surfelVertexList;
		int nVertexSurfelRelations;
		float TIVertexToleranceAngle;
		int edgeDepth;
		bool *bVertexAssigned;
		int *iVertexMem;
	private:
		unsigned char *nodeColor;
		QLIST::Index *surfelVertexMem;
		Array<Array<int>> vertexDisplayLineArray;
		int *vertexDisplayLineArrayMem;
		vtkSmartPointer<vtkPolyData> linesPolyData;
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
		void MouseRButtonDown(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata);
		void KeyPressCallback(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata);
	};
}
