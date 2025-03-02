#pragma once

#define RVLMOTION_NODE_FLAG_FREE_PATH_TO_START 0x01
#define RVLMOTION_NODE_FLAG_PATH_TO_GOAL 0x02
#define RVLMOTION_EDGE_FLAG_FREE_PATH 0x01

#define RVLMOTION_ADD_NODE_TO_TREE(nodes, iNode, pParent, iSibling, pSibling) \
	{                                                                         \
		if (pParent->iFirstChild < 0)                                         \
			pParent->iFirstChild = iNode;                                     \
		else                                                                  \
		{                                                                     \
			iSibling = pParent->iFirstChild;                                  \
			while (iSibling >= 0)                                             \
			{                                                                 \
				pSibling = nodes.data() + iSibling;                           \
				iSibling = pSibling->iSibling;                                \
			}                                                                 \
			pSibling->iSibling = iNode;                                       \
		}                                                                     \
	}

#define RVLMOTION_REMOVE_NODE_FROM_TREE(nodes, iNode, pNode, pParent, iSibling, pSibling) \
	{                                                                                     \
		if (pParent->iFirstChild == iNode)                                                \
			pParent->iFirstChild = pNode->iSibling;                                       \
		else                                                                              \
		{                                                                                 \
			iSibling = pParent->iFirstChild;                                              \
			while (iSibling != iNode)                                                     \
			{                                                                             \
				pSibling = nodes.data() + iSibling;                                       \
				iSibling = pSibling->iSibling;                                            \
			}                                                                             \
			pSibling->iSibling = pNode->iSibling;                                         \
		}                                                                                 \
	}

/// The following macros should be moved to Util.h.

// c[S] = A[S,:] * b
#define RVLMULMXROWSVECT(A, n, S, b, c, a, fTmp, i_, j_, k_) \
	{                                                        \
		for (i_ = 0; i_ < S.n; i_++)                         \
		{                                                    \
			j_ = S.Element[i_];                              \
			a = A + j_ * n;                                  \
			RVLDOTPRODUCT(a, b, n, fTmp, k_);                \
			c[j_] = fTmp;                                    \
		}                                                    \
	}

// e[S] = A[S,:] * x - b[S]
#define RVLDISTTOPLANES(A, b, n, S, x, e, a, fTmp, i_, j_, k_) \
	{                                                          \
		for (i_ = 0; i_ < S.n; i_++)                           \
		{                                                      \
			j_ = S.Element[i_];                                \
			a = A + j_ * n;                                    \
			RVLDOTPRODUCT(a, x, n, fTmp, k_);                  \
			e[j_] = fTmp - b[j_];                              \
		}                                                      \
	}

// b = a[S]
#define RVLCOPYSELECTED(a, S, b, i_, j_) \
	{                                    \
		for (i_ = 0; i_ < S.n; i_++)     \
			b[i_] = a[S.Element[i_]];    \
	}

///

#define RVLSOLVER_FLAGS_SUCCESS 0x00
#define RVLSOLVER_FLAGS_NOFEASIBLESOLUTION 0x01
#define RVLSOLVER_FLAGS_UNBOUNDED 0x02
#define RVLSOLVER_FLAGS_OUTOFLIN 0x04

namespace RVL
{
	// This should be moved to Util.cpp or to its own source file.

	class Solver
	{
	public:
		Solver();
		virtual ~Solver();
		void Create(
			int m,
			int n);
		void Clear();
		void SetLinearConstraints(
			float *AIn,
			float *bIn);
		void SetInequalityConstraints(Array<int> SIn);
		void SetEqualityConstraints(Array<int> SEqIn);
		void SetLinearRegion(
			Array<int> linRegionXIn,
			float rLinRegionIn);
		void SetPose(Pose3D pose0In);
		void SetRadiusConstraint(
			int iRadiusIn,
			float rIn);
		void ClearRadiusConstraint();
		bool FeasibleSolution(
			float *A,
			float *b,
			int m,
			float *x0,
			float *x,
			float r = 0.0f);
		uchar FeasibleSolution2(
			float *A,
			float *b,
			Array<int> S,
			float *x0,
			float *x,
			float *AEq = NULL,
			int mEq = 0,
			float *cSC = NULL,
			float rSC = 1.0f,
			int ixSC = 0,
			float rSCmin = 0.0f,
			Array<int> *pL = NULL,
			float rL = 0.0f);
		uchar FeasibleSolution3(
			float *x0,
			float *x,
			float &exmax);
		bool Gradient(
			Array<int> J,
			float *AEq = NULL,
			int mEq = 0,
			int eqSign = 0);
		int SubSpace(
			float *A,
			int n,
			Array<int> J);
		bool ConstrainToLinearRegion(
			float *x,
			float *x0,
			float s);

	public:
		int maxm;
		int n;
		float r;
		float *v;

	private:
		float *ex;
		float *ev; // Only for debugging purpose!!!
		Array<int> K;
		Array<int> J;
		Array<int> JPrev;
		bool *bK;
		bool *bJ;
		float *T;
		int *B;
		int *B_;
		float *c1;
		float *c2;
		bool *bB;
		float *e_J;
		bool *zero_e_J;
		float *g;
		Array2D<int> JCombinations;
		int *JCombinationsMem;
		float *C;
		Array<int> seq;
		float *aSC;
		float *VnTmp;
		float *x_;
		float *dx;
		float *A;
		float *b;
		Array<int> S;
		Array<int> SEq;
		Array<int> linRegionX;
		float rLinRegion;
		float rL2;
		int iRadius;
		int iq;
		int it;
		Pose3D pose0;
		Array<int> JBuff;
	};

	//

	namespace MOTION
	{
		struct Edge
		{
			int iVertex[2];
			GRAPH::EdgePtr<MOTION::Edge> *pVertexEdgePtr[2];
			int idx;
			float cost;
			uchar flags;
			MOTION::Edge *pNext;
		};

		struct Node
		{
			SE3Point pose;
			float PRTCP[2];
			float cost;
			uchar flags;
			int iSE3Point;
			int iParent;
			int iSibling;
			int iFirstChild;
			GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>> *pGNode;
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
} // namespace RVL
