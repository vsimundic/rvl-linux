//#include "stdafx.h"
//#include <pcl/io/pcd_io.h>
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Graph.h"
//#include <Eigen\Eigenvalues>
//#include <pcl/common/common.h>
//#include <pcl/PolygonMesh.h>
//#include <pcl/surface/vtk_smoothing/vtk_utils.h>
//#include "PCLTools.h"
//#include "PCLMeshBuilder.h"
//#include "RGBDCamera.h"
#include "Mesh.h"
#include "PolygonGraph.h"

using namespace RVL;

PolygonGraph::PolygonGraph()
{
}


PolygonGraph::~PolygonGraph()
{
}

void PolygonGraph::UpdateConvex(
	MESH::Polygon *pPolygon,
	float *N,
	float d)
{
	//int iP;

	GRAPH::EdgePtr<MESH::PolygonEdge> *pEdgePtr = pPolygon->EdgeList.pFirst;

	while (pEdgePtr)
	{
		

		pEdgePtr = pEdgePtr->pNext;
	}

/////

	//for k_ = 1:nP_
	//	iP = F_(k_);
	//	P_ = P(:, iP);
	//	iPNext = F_(RVLNextCirc(k_, nP_));
	//	Pnext = P(:, iPNext);
	//	dP = Pnext - P_;
	//	l = Fn_(k_);
	//	if N'*P_ > d     
	//		if Premoved(iP) == 0
	//			Premoved(iP) = 1;
	//	end
	//		iPolygon = j; iVertex = k; RVLConvexTemplateRemoveVertexFromPolygon
	//		if N'*Pnext > d
	//			E(l, j) = 0;
	//	E(j, l) = 0;
	//	k = k - 1;
	//		else
	//			if E(j, l) == iP % Vertex is not updated.
	//				P0 = P_; V = dP; RVLConvexTemplateAddNewVertex % iPNew <-new vertex index
	//				E(j, l) = iPNew;
	//			else
	//				iPNew = E(j, l);
	//	end
	//		iPolygon = j; iVertex = k; iPNewVertex = iPNew; iFNewVertex = l; RVLConvexTemplateAddNewVertexToPolygon
	//		iNewVertices = [iNewVertices; iPNew];
	//	iNeighbors = [iNeighbors; j];
	//	E(i + 6, j) = iPNew;
	//	end
	//		elseif N'*Pnext > d
	//		if E(l, j) == iPNext % Vertex is not updated.
	//			P0 = P_; V = dP; RVLConvexTemplateAddNewVertex % iPNew <-new vertex index
	//			E(l, j) = iPNew;
	//		else
	//			iPNew = E(l, j);
	//	end
	//		iPolygon = j; iVertex = k + 1; iPNewVertex = iPNew; iFNewVertex = i + 6; RVLConvexTemplateAddNewVertexToPolygon
	//		E(j, i + 6) = iPNew;
	//	k = k + 1;
	//	end

	//	k = k + 1;
	//end

}
