#pragma once
#include "Graph.h"

namespace RVL
{
	namespace SURFEL
	{
		struct AgEdge
		{
			int iVertex[2];
			GRAPH::EdgePtr<AgEdge> *pVertexEdgePtr[2];
			int idx;
			SurfelAdjecencyDescriptors desc;
			float cost;
			AgEdge *pNext;
		};

		class ObjectGraph :
			public Graph < GRAPH::AggregateNode<AgEdge>, AgEdge, GRAPH::EdgePtr<AgEdge> >
		{
		public:
			ObjectGraph();
			virtual ~ObjectGraph();
			void Create(SurfelGraph *pSurfels);

		private:
			int *iElementMem;
		};
	}
}

