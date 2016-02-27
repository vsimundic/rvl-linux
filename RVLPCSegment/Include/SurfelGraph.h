#pragma once

#define RVLSURFEL_DISPLAY_MODE_SURFELS			0
#define RVLSURFEL_DISPLAY_MODE_BOUNDARY			1
#define RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR	2

namespace RVL
{
	struct Surfel
	{
		QList<QLIST::Index> PtList;
		//QList<QLIST::Index> Boundary;
		float P[3];
		float N[3];
		float d;
		int RGB[3];
		float P0[3];
		float r0;
		QList<MeshEdgePtr> EdgeList;
		Surfel *pNext;
	};

	class SurfelGraph;

	namespace SURFEL
	{
		struct DisplayCallbackData
		{
			Visualizer *pVisualizer;
			SurfelGraph *pSurfels;
			Mesh *pMesh;
			void *vpDetector;
			int  mode;
			unsigned char SelectionColor[3];
			int iSelectedSurfel;
			int iSelectedSurfel2;
			int iSelection;
		};
	}

	class SurfelGraph : public Graph < Surfel, MeshEdge, MeshEdgePtr >
	{
	public:
		SurfelGraph();
		virtual ~SurfelGraph();
		void GetNeighbors(
			int iSurfel,
			Mesh *pMesh,
			CRVLMem *pMem);
		void InitGetNeighbors();
		void FreeGetNeighbors();
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
		void Init(int nPoints);
		void Clear();
		unsigned char * GetColor(int iSurfel);
		void PrintData(
			Visualizer *pVisualizer,
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

	public:		
		QLIST::Index *PtMem;
		int *surfelMap;
	private:
		bool *bConnected;
		unsigned char *nodeColor;
		SURFEL::DisplayCallbackData DisplayData;
		
	};

	namespace SURFEL
	{
		void ComputeParameters(
			Surfel *pSurfel,		
			MESH::Distribution &distribution,
			Point *pPt);
		void GetPoint(
			Surfel *pSurfel,
			Point *pPoint);
		void MouseRButtonDown(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata);
		void KeyPressCallback(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata);
	};
}
