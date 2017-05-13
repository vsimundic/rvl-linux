#pragma once

namespace RVL
{
	class VertexGraph :
		public Graph < SURFEL::Vertex, SURFEL::VertexEdge, GRAPH::EdgePtr2<SURFEL::VertexEdge> >
	{
	public:
		VertexGraph();
		virtual ~VertexGraph();
		void Create(SurfelGraph *pSurfels);
		void Save(FILE *fp);
		bool Load(FILE *fp);
		bool BoundingBox(Box<float> *pBox);

	public:
		int idx;
	};
}

