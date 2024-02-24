#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "SE3Grid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "VN.h"
#include "RRT.h"

using namespace RVL;

RRT::RRT()
{

}

RRT::~RRT()
{
	Clear();
}

void RRT::Create()
{

}

void RRT::Clear()
{

}

//**** Solver ***

Solver::Solver()
{
	ex = NULL;
	ev = NULL;
	K.Element = NULL;
	J.Element = NULL;
	JPrev.Element = NULL;
	bK = NULL;
	bJ = NULL;
	v = NULL;
	T = NULL;
	B = NULL;
	B_ = NULL;
	c1 = NULL;
	c2 = NULL;
	bB = NULL;
	e_J = NULL;
	zero_e_J = NULL;
}

Solver::~Solver()
{
	Clear();
}

void Solver::Create(
	int maxmIn,
	int nIn)
{
	maxm = maxmIn;
	n = nIn;
	RVL_DELETE_ARRAY(ex);
	ex = new float[maxm];
	RVL_DELETE_ARRAY(ev);
	ev = new float[maxm];
	RVL_DELETE_ARRAY(K.Element);
	K.Element = new int[maxm];
	RVL_DELETE_ARRAY(J.Element);
	J.Element = new int[maxm];
	RVL_DELETE_ARRAY(JPrev.Element);
	JPrev.Element = new int[maxm];
	RVL_DELETE_ARRAY(bK);
	bK = new bool[maxm];
	RVL_DELETE_ARRAY(bJ);
	bJ = new bool[maxm];
	RVL_DELETE_ARRAY(v);
	v = new float[n];
	RVL_DELETE_ARRAY(T);
	T = new float[2 * n * n];
	RVL_DELETE_ARRAY(B);
	B = new int[n];
	RVL_DELETE_ARRAY(B_);
	B_ = new int[n];
	RVL_DELETE_ARRAY(c1);
	c1 = new float[n];
	RVL_DELETE_ARRAY(c2);
	c2 = new float[n];
	RVL_DELETE_ARRAY(bB);
	bB = new bool[n];
	RVL_DELETE_ARRAY(e_J);
	e_J = new float[n];
	RVL_DELETE_ARRAY(zero_e_J);
	zero_e_J = new bool[n];
}

void Solver::Clear()
{
	RVL_DELETE_ARRAY(ex);
	RVL_DELETE_ARRAY(ev);
	RVL_DELETE_ARRAY(K.Element);
	RVL_DELETE_ARRAY(J.Element);
	RVL_DELETE_ARRAY(JPrev.Element);
	RVL_DELETE_ARRAY(bK);
	RVL_DELETE_ARRAY(bJ);
	RVL_DELETE_ARRAY(v);
	RVL_DELETE_ARRAY(T);
	RVL_DELETE_ARRAY(B);
	RVL_DELETE_ARRAY(B_);
	RVL_DELETE_ARRAY(c1);
	RVL_DELETE_ARRAY(c2);
	RVL_DELETE_ARRAY(bB);
	RVL_DELETE_ARRAY(e_J);
	RVL_DELETE_ARRAY(zero_e_J);
}

#define RVLSOLVER_DEBUG

bool Solver::FeasibleSolution(
	float* A,
	float* b,
	int m,
	float* x0,
	float* x,
	float r)
{
	// x <- x0

	memcpy(x, x0, n * sizeof(float));

	// K <- indices of all satisfied constraints.

	int i, j;
	float* a;
	RVLMULMXVECT(A, x, m, n, ex, i, j, a);
	RVLDIFVECTORS(ex, b, m, ex, i);
	K.n = 0;
	for (j = 0; j < m; j++)
		if (bK[j] = (ex[j] <= 0.0f))
			K.Element[K.n++] = j;
		else if (r > 0.0f)
		{
			if (ex[j] > r)
				return false;
		}
			 
	// If all constraints are satisfied, then the search is completed.

	if (K.n == m)
		return true;

	/// Main loop.

	J.n = 0;
	int k;
	int colsT;
	float pivotAbsVal;
	float* rowT, *rowT_;
	float fTmp;
	int j_;
	memset(bB, 0, n * sizeof(bool));
	memset(zero_e_J, 0, n * sizeof(bool));
	memset(bJ, 0, m * sizeof(bool));
	int nB_;
	int iB, iB_;
	int ie_J;
	int nZero_e_J;
	float w;
	float s, s_k;
	float s_j = 0.0f;
	int jOut;
	float e_;
	int it = 0;	// Only for debugging purpose!!!
	float de;
	bool bNoMove;
	while (K.n < m)
	{
		//if (it == 60)
		//	int debug = 0;

		// k <- the first constraint which is not satisfied.

		for(j = 0; j < m; j++)
			if (!bK[j])
			{
				k = j;
				break;
			}

		//

		if (J.n == 0)
		{
			// If x is not on any constraint, then v <- -A[k,:]

			a = A + n * k;
			for (i = 0; i < n; i++)
				v[i] = -a[i];
		}
		else
		{
			// Compute v.

			colsT = n + J.n;
			memset(T, 0, J.n * (n + J.n) * sizeof(float));
			for (j = 0; j < J.n; j++)
			{
				memcpy(T + colsT * j, A + n * J.Element[j], n * sizeof(float));
				T[n + j + colsT * j] = 1.0f;
			}
			rowT = T;
			for (j = 0; j < J.n; j++, rowT += colsT)
			{
				pivotAbsVal = 0.0f;
				for (i = 0; i < n; i++)
				{
					fTmp = RVLABS(rowT[i]);
					if (fTmp > pivotAbsVal)
					{
						pivotAbsVal = fTmp;
						B[j] = i;
					}
				}
				bB[B[j]] = true;
				fTmp = 1.0f / rowT[B[j]];
				RVLSCALEVECTOR(rowT, fTmp, rowT, colsT, i);
				if (J.n > 1)
				{
					rowT_ = T;
					for (j_ = 0; j_ < J.n; j_++, rowT_ += colsT)
					{
						if (j_ == j)
							continue;
						fTmp = rowT_[B[j]];
						for (i = 0; i < colsT; i++)
							rowT_[i] -= fTmp * rowT[i];
					}
				}
			}
			nB_ = 0;
			for (i = 0; i < n; i++)
				if (bB[i])
					bB[i] = false;
				else
					B_[nB_++] = i;
			a = A + n * k;
			for (iB_ = 0; iB_ < nB_; iB_++)
			{
				c1[iB_] = 0.0f;
				for (iB = 0; iB < J.n; iB++)
					c1[iB_] += a[B[iB]] * T[iB * colsT + B_[iB_]];
				c1[iB_] -= a[B_[iB_]];
			}
			nZero_e_J = 0;
			for (ie_J = 0; ie_J < J.n; ie_J++)
			{
				c2[ie_J] = 0.0f;
				for (iB = 0; iB < J.n; iB++)
					c2[ie_J] += (a[B[iB]] * T[iB * colsT + n + ie_J]);
				if (c2[ie_J] < 0.0f)
				{
					e_J[ie_J] = 0.0f;
					zero_e_J[ie_J] = true;
					nZero_e_J++;
				}
				else
					e_J[ie_J] = c2[ie_J];
			}
			if (nZero_e_J == n)
				return false;
			for (iB = 0; iB < J.n; iB++)
			{
				v[B[iB]] = 0.0f;
				for (iB_ = 0; iB_ < nB_; iB_++)
					v[B[iB]] -= T[iB * colsT + B_[iB_]] * c1[iB_];
				for (ie_J = 0; ie_J < J.n; ie_J++)
					v[B[iB]] -= T[iB * colsT + n + ie_J] * e_J[ie_J];
			}
			for (iB_ = 0; iB_ < nB_; iB_++)
				v[B_[iB_]] = c1[iB_];
			RVLDOTPRODUCT(v, v, n, fTmp, i);
			if (fTmp == 0.0f)
				return false;
			//fTmp = 1.0f / fTmp;
			//RVLSCALEVECTOR(v, fTmp, v, n, i);

			// Only for debugging purpose!!!

			//RVLMULMXVECT(A, v, m, n, ev, i, j, a);
			//float debug = -1.0f;
			//for (ie_J = 0; ie_J < J.n; ie_J++)
			//{
			//	j = J.Element[ie_J];
			//	fTmp = ev[j] + e_J[ie_J];
			//	fTmp = RVLABS(fTmp);
			//	if (fTmp > debug)
			//		debug = fTmp;
			//}

			// JPrev <- J

			JPrev.n = J.n;
			memcpy(JPrev.Element, J.Element, J.n * sizeof(int));

			// Remove from J all constraints which v is not on. 

			J.n = 0;
			for (ie_J = 0; ie_J < JPrev.n; ie_J++)
				if (bJ[JPrev.Element[ie_J]] = zero_e_J[ie_J])
				{
					J.Element[J.n++] = JPrev.Element[ie_J];
					zero_e_J[ie_J] = false;
				}
		}

		// Move to the target - constraint k.

		a = A + k * n;
		RVLDOTPRODUCT(a, v, n, w, i);
		RVLDOTPRODUCT(a, x, n, fTmp, i);
		s_k = (b[k] - fTmp) / w;
		jOut = -1;
		bNoMove = false;
		for (j = 0; j < m; j++)
		{
			if (j == k)
				continue;
			if (bJ[j])
				continue;
			a = A + j * n;
			RVLDOTPRODUCT(a, v, n, w, i);
			//
			if (w <= 0.0f)
				continue;
			RVLDOTPRODUCT(a, x, n, de, i);
			de = b[j] - de;
			if (de < 0.0f)
				continue;
			if (de < 1e-6)
			{
				s_j = 0.0f;
				jOut = j;
				bNoMove = true;
				break;
			}
			s = de / w;
			//if (w < 1e-7)
			//{
			//	if (w > -1e-7)
			//	{
			//		RVLDOTPRODUCT(a, x, n, fTmp, i);
			//		ex[j] = fTmp - b[j];
			//		if (RVLABS(ex[j]) < 1e-7)
			//		{
			//			bJ[j] = true;
			//			s = 0.0f;
			//		}
			//	}
			//	continue;
			//}
			//RVLDOTPRODUCT(a, x, n, fTmp, i);
			//ex[j] = fTmp - b[j];
			//s = -ex[j] / w;
			//if (s <= -1e-7)
			//	continue;
			//else if (s <= 1e-7)
			//{
			//	bJ[j] = true;
			//	s = 0.0f;
			//}
			if (jOut < 0 || s < s_j)
			{
				jOut = j;
				s_j = s;
			}
		}
		if (jOut >= 0 && s_j < s_k)
		{
#ifndef RVLSOLVER_DEBUG
			J.Element[J.n++] = jOut;
#endif
			bJ[jOut] = true;
			s = s_j;
		}
		else
		{
#ifndef RVLSOLVER_DEBUG
			J.Element[J.n++] = k;
#endif
			bJ[k] = true;
			K.Element[K.n++] = k;
			bK[k] = true;
			s = s_k;
		}
#ifdef RVLSOLVER_DEBUG
		J.n = 0;
		for (j = 0; j < m; j++)
			if (bJ[j])
				J.Element[J.n++] = j;
#endif
		if (bNoMove)
			for (i = 0; i < JPrev.n; i++)
			{
				j = JPrev.Element[i];
				if (!bJ[j])
				{
					J.Element[J.n++] = j;
					bJ[j] = true;
				}
			}
		else
		{
			for (i = 0; i < n; i++)
				x[i] += s * v[i];
			for (j = 0; j < m; j++)
			{
				//if (bK[j])
				//	continue;
				if (bJ[j])
					continue;
				a = A + j * n;
				RVLDOTPRODUCT(a, x, n, fTmp, i);
				e_ = fTmp - b[j];
				if (e_ <= 1e-6)
				{
					if (!bK[j])
					{
						K.Element[K.n++] = j;
						bK[j] = true;
					}
					if (e_ > -1e-6)
					{
						J.Element[J.n++] = j;
						bJ[j] = true;
					}
				}			 
			}
		}

		// Only for debugging purpose!!!

		//RVLMULMXVECT(A, x, m, n, ex, i, j, a);
		//RVLDIFVECTORS(ex, b, m, ex, i);
		//float maxe = 0.0;
		//for (j = 0; j < m; j++)
		//{
		//	if(bK[j])
		//		if (ex[j] > maxe)
		//			maxe = ex[j];
		//}

		it++;
	}

	///

	return true;
}

