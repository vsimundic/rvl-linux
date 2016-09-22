#pragma once
#include "Graph.h"

namespace RVL
{
	namespace SURFEL
	{
		struct AgEdge
		{
			int iVertex[2];
			GRAPH::EdgePtr2<AgEdge> *pVertexEdgePtr[2];
			int idx;
			SurfelAdjecencyDescriptors desc;
			float cost;
			AgEdge *pNext;
		};

		class ObjectGraph;

		struct ObjectDisplayData
		{
			Mesh *pMesh;
			SurfelGraph *pSurfels;
			ObjectGraph *pObjects;
			Visualizer *pVisualizer;
			unsigned char selectionColor[3];
			int iSelectedObject;
			bool bObjects;
		};

		bool objectKeyPressUserFunction(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			std::string &key,
			void *vpData);
		bool objectMouseRButtonDownUserFunction(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSelectedPt,
			int iSelectedSurfel,
			void *vpData);

		class ObjectGraph :
			public Graph < GRAPH::AggregateNode<AgEdge>, AgEdge, GRAPH::EdgePtr2<AgEdge> >
		{
		public:
			ObjectGraph();
			virtual ~ObjectGraph();
			void Create(SurfelGraph *pSurfels_);
			void WERSegmentation();
			void ComputeRelationCosts();
			void ComputeRelationCost(AgEdge *pEdge);
			void InitDisplay(
				Visualizer *pVisualizer,
				Mesh *pMesh,
				unsigned char *selectionColor);
			void Display();
			void PaintObject(
				int iObject,
				unsigned char *color);

		public:
			SurfelGraph *pSurfels;
			float WERSegmentationMinCostDiff;
			float WERSegmentationCostResolution;
			ObjectDisplayData displayData;
			int *objectMap;
		private:
			QLIST::Index *elementMem;
		};
	}
}

