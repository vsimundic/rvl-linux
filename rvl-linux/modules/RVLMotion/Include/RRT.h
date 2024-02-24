#pragma once

#define RVLMOTION_NODE_FLAG_FREE_PATH_TO_START	0x01
#define RVLMOTION_NODE_FLAG_PATH_TO_GOAL		0x02
#define RVLMOTION_EDGE_FLAG_FREE_PATH	0x01

#define RVLMOTION_ADD_NODE_TO_TREE(nodes, iNode, pParent, iSibling, pSibling)\
{\
	if (pParent->iFirstChild < 0)\
		pParent->iFirstChild = iNode;\
	else\
	{\
		iSibling = pParent->iFirstChild;\
		while (iSibling >= 0)\
		{\
			pSibling = nodes.data() + iSibling;\
			iSibling = pSibling->iSibling;\
		}\
		pSibling->iSibling = iNode;\
	}\
}

#define RVLMOTION_REMOVE_NODE_FROM_TREE(nodes, iNode, pNode, pParent, iSibling, pSibling)\
{\
	if (pParent->iFirstChild == iNode)\
		pParent->iFirstChild = pNode->iSibling;\
	else\
	{\
		iSibling = pParent->iFirstChild;\
		while (iSibling != iNode)\
		{\
			pSibling = nodes.data() + iSibling;\
			iSibling = pSibling->iSibling;\
		}\
		pSibling->iSibling = pNode->iSibling;\
	}\
}

namespace RVL
{
	// This should be moved to Util.cpp.

	class Solver
	{
	public:
		Solver();
		virtual ~Solver();
		void Create(
			int m,
			int n);
		void Clear();
		bool FeasibleSolution(
			float* A,
			float* b,
			int m,
			float* x0,
			float* x,
			float r = 0.0f);

	public:
		int maxm;
		int n;
		float r;
	private:
		float* ex;
		float* ev;		// Only for debugging purpose!!!
		Array<int> K;
		Array<int> J;
		Array<int> JPrev;
		bool* bK;
		bool* bJ;
		float* v;
		float* T;
		int* B;
		int* B_;
		float* c1;
		float* c2;
		bool* bB;
		float* e_J;
		bool *zero_e_J;
	};

	//

	namespace MOTION
	{
		struct Edge
		{
			int iVertex[2];
			GRAPH::EdgePtr<MOTION::Edge>* pVertexEdgePtr[2];
			int idx;
			float cost;
			uchar flags;
			MOTION::Edge* pNext;
		};

		struct Node
		{
			SE3Point pose;
			float cost;
			uchar flags;
			int iSE3Point;
			int iParent;
			int iSibling;
			int iFirstChild;
			GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>* pGNode;
		};
	}

	class RRT
	{
	public:
		RRT();
		virtual ~RRT();
		void Create();
		void Clear();
	};
}		// namespace RVL

