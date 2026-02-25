#pragma once

namespace RVL
{
	namespace MESH
	{
		struct PolygonEdge
		{
			int iVertex[2];
			int iVertex_[2];
			GRAPH::EdgePtr<PolygonEdge> *pEdgePtr[2];
			int idx;
		};

		struct Polygon
		{
			float N[3];
			float d;
			QList<GRAPH::EdgePtr<PolygonEdge>> EdgeList;
			MESH::Polygon *pNext;
		};
	}

	class PolygonGraph : public Graph<MESH::Polygon, MESH::PolygonEdge, GRAPH::EdgePtr<MESH::PolygonEdge>>
	{
	public:
		PolygonGraph();
		virtual ~PolygonGraph();
		void UpdateConvex(
			MESH::Polygon *pPolygon,
			float *N,
			float d);

	public:
		Mesh *pMesh;
	};
}



