#pragma once

#define RVLSURFEL_DISPLAY_MODE_SURFELS			0
#define RVLSURFEL_DISPLAY_MODE_BOUNDARY			1
#define RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR	2

#define RVLSURFEL_EDGE_FLAG_HARD				0x01
#define RVLSURFEL_EDGE_FLAG_CONVEX				0x02

#define RVLSURFEL_VERSION_0		0

namespace RVL
{
	class SurfelGraph;

	struct SurfelAdjecencyDescriptors
	{
		double cupyDescriptor[4];
		double minDist;
		int commonBoundaryLength;
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
			int  mode;
			unsigned char SelectionColor[3];
			int iSelectedSurfel;
			int iSelectedSurfel2;
			int iSelection;
			float edgeFeatureDepth;
			vtkSmartPointer<vtkPolyData> edgeFeaturesPolyData;
			vtkSmartPointer<vtkActor> edgeFeatures;
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
	}

	struct Surfel
	{
		QList<QLIST::Index2> PtList;
		Array<Array<MeshEdgePtr *>> BoundaryArray;
		Array<Array<MeshEdgePtr *>> PolygonBoundaryArray;
		float P[3];		// centroid
		float N[3];		// normal
		float d;		// plane offset
		int RGB[3];		// average color
		float P0[3];	// central point
		float V[3];
		float r0;		// distance 
		QList<SURFEL::EdgePtr> EdgeList;
		Surfel *pNext;
		int size;
		float physicalSize;
		bool bEdge;
		int ObjectID;	//Filko
		std::vector<Surfel*> imgAdjacency;	//Filko
		std::vector<SurfelAdjecencyDescriptors*> imgAdjacencyDescriptors;	//Filko
		std::vector<int> GTObjHist; //Filko
	};

	class SurfelGraph : public Graph < Surfel, MeshEdge, MeshEdgePtr >
	{
	public:
		SurfelGraph();
		virtual ~SurfelGraph();
		void CreateParamList(CRVLMem *pMem);
		void InitGetNeighborsBoundaryAndSize();
		void FreeGetNeighborsBoundaryAndSize();
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
			void *vpDetector);
		void DisplaySurfelBoundary(
			Visualizer *pVisualizer, 
			Mesh * pMesh, 
			int iSurfel,
			unsigned char *Color);
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
		void ImageAdjacency(Mesh *pMesh);
		void ImageAdjacency(
			Mesh *pMesh, 
			int iSurfel, 
			int *surfelIdx,
			bool *bVisited);		

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
	private:
		unsigned char *nodeColor;
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
