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
	g = NULL;
	JCombinations.Element = NULL;
	JCombinationsMem = NULL;
	C = NULL;
	seq.Element = NULL;
	aSC = NULL;
	VnTmp = NULL;
	x_ = NULL;
	dx = NULL;
	S.n = 0;
	SEq.n = 0;
	linRegionX.n = 0;
	RVLUNITMX3(pose0.R);
	RVLNULL3VECTOR(pose0.t);
	iRadius = -1;
	JBuff.Element = NULL;
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
	memset(bK, 0, maxm * sizeof(bool));
	RVL_DELETE_ARRAY(bJ);
	bJ = new bool[maxm];
	memset(bJ, 0, maxm * sizeof(bool));
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
	RVL_DELETE_ARRAY(g);
	g = new float[maxm];
	RVL_DELETE_ARRAY(JCombinations.Element);
	JCombinations.Element = NULL;
	RVL_DELETE_ARRAY(JCombinationsMem);
	JCombinationsMem = new int[n];
	RVL_DELETE_ARRAY(C);
	C = new float[(n - 1) * n];
	RVL_DELETE_ARRAY(seq.Element);
	seq.Element = new int[n - 1];
	for (int i = 0; i < n - 1; i++)
		seq.Element[i] = i;
	RVL_DELETE_ARRAY(aSC);
	aSC = new float[n];
	RVL_DELETE_ARRAY(VnTmp);
	VnTmp = new float[n];
	RVL_DELETE_ARRAY(x_);
	x_ = new float[n];
	RVL_DELETE_ARRAY(dx);
	dx = new float[n];
	RVL_DELETE_ARRAY(JBuff.Element);
	JBuff.Element = new int[maxm];
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
	RVL_DELETE_ARRAY(g);
	RVL_DELETE_ARRAY(JCombinations.Element);
	RVL_DELETE_ARRAY(JCombinationsMem);
	RVL_DELETE_ARRAY(C);
	RVL_DELETE_ARRAY(seq.Element);
	RVL_DELETE_ARRAY(aSC);
	RVL_DELETE_ARRAY(VnTmp);
	RVL_DELETE_ARRAY(x_);
	RVL_DELETE_ARRAY(dx);
	RVL_DELETE_ARRAY(JBuff.Element);
	S.n = 0;
	SEq.n = 0;
	linRegionX.n = 0;
	iRadius = -1;
}

void Solver::SetLinearConstraints(
	float *AIn,
	float *bIn)
{
	A = AIn;
	b = bIn;
}

void Solver::SetInequalityConstraints(Array<int> SIn)
{
	S = SIn;
}

void Solver::SetEqualityConstraints(Array<int> SEqIn)
{
	SEq = SEqIn;
}

void Solver::SetLinearRegion(
	Array<int> linRegionXIn,
	float rLinRegionIn)
{
	linRegionX = linRegionXIn;
	rLinRegion = rLinRegionIn;
}

void Solver::SetPose(Pose3D pose0In)
{
	pose0 = pose0In;
}

void Solver::SetRadiusConstraint(
	int iRadiusIn,
	float rIn)
{
	iRadius = iRadiusIn;
	r = rIn;
}

void Solver::ClearRadiusConstraint()
{
	iRadius = -1;
}

#define RVLSOLVER_DEBUG

bool Solver::FeasibleSolution(
	float *A,
	float *b,
	int m,
	float *x0,
	float *x,
	float r)
{
	// x <- x0

	memcpy(x, x0, n * sizeof(float));

	// K <- indices of all satisfied constraints.

	int i, j;
	float *a;
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
	float fTmp;
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
	int it = 0; // Only for debugging purpose!!!
	float de;
	bool bNoMove;
	while (K.n < m)
	{
		// if (it == 60)
		//	int debug = 0;

		// k <- the first constraint which is not satisfied.

		for (j = 0; j < m; j++)
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

			nB_ = SubSpace(A, n, J);

			colsT = n + J.n;
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
			// fTmp = 1.0f / fTmp;
			// RVLSCALEVECTOR(v, fTmp, v, n, i);

			// Only for debugging purpose!!!

			// RVLMULMXVECT(A, v, m, n, ev, i, j, a);
			// float debug = -1.0f;
			// for (ie_J = 0; ie_J < J.n; ie_J++)
			//{
			//	j = J.Element[ie_J];
			//	fTmp = ev[j] + e_J[ie_J];
			//	fTmp = RVLABS(fTmp);
			//	if (fTmp > debug)
			//		debug = fTmp;
			// }

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
			// if (w < 1e-7)
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
			// }
			// RVLDOTPRODUCT(a, x, n, fTmp, i);
			// ex[j] = fTmp - b[j];
			// s = -ex[j] / w;
			// if (s <= -1e-7)
			//	continue;
			// else if (s <= 1e-7)
			//{
			//	bJ[j] = true;
			//	s = 0.0f;
			// }
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
				// if (bK[j])
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

		// RVLMULMXVECT(A, x, m, n, ex, i, j, a);
		// RVLDIFVECTORS(ex, b, m, ex, i);
		// float maxe = 0.0;
		// for (j = 0; j < m; j++)
		//{
		//	if(bK[j])
		//		if (ex[j] > maxe)
		//			maxe = ex[j];
		// }

		it++;
	}

	///

	return true;
}

uchar Solver::FeasibleSolution2(
	float *A,
	float *b,
	Array<int> S,
	float *x0,
	float *x,
	float *AEq,
	int mEq,
	float *cSC,
	float rSC,
	int ixSC,
	float rSCmin,
	Array<int> *pL,
	float rL)
{
	int m = S.n;

	// x <- x0

	memcpy(x, x0, n * sizeof(float));

	// xc - ptr. to the part of x subject to the spherical contstraint.

	float *xSC = x + ixSC;

	/// Main loop.

	int i, j, i_, j_;
	float *a;
	int k;
	int colsT;
	float fTmp;
	uchar flags = 0x00;
	rL2 = rL * rL;
	memset(bB, 0, n * sizeof(bool));
	memset(zero_e_J, 0, n * sizeof(bool));
	int r_;
	int nB_;
	int iB, iB_;
	int ie_J;
	int nZero_e_J;
	float w;
	float wk;
	float s;
	float s_K;
	int jOut = -1;
	int it = 0; // Only for debugging purpose!!!
	J.n = 0;
	K.n = 0;
	int nKPrev = -1;
	float ekPrev;
	bool bExpandJ = false;
	float *vSC = v + ixSC;
	memset(aSC, 0, n * sizeof(float));
	float *aSC_ = aSC + ixSC;
	float rSC2 = rSC * rSC;
	float eSC2;
	float lvSC2;
	float laSC2;
	float rSCmin2 = rSCmin * rSCmin;
	while (true)
	{
		RVLDISTTOPLANES(A, b, n, S, x, ex, a, fTmp, i_, j_, i);
		if (!bExpandJ)
		{
			for (j = 0; j < J.n; j++)
				bJ[J.Element[j]] = false;
			J.n = 0;
		}
		for (i_ = 0; i_ < m; i_++)
		{
			j = S.Element[i_];
			if (ex[j] <= 1e-6)
			{
				if (!bK[j])
				{
					bK[j] = true;
					K.Element[K.n++] = j;
				}
			}
			if (bK[j])
			{
				if (!bJ[j])
				{
					if (ex[j] >= -1e-6)
					{
						bJ[j] = true;
						J.Element[J.n++] = j;
					}
				}
			}
			else
				k = j;
		}

		if (cSC)
		{
			RVLDIF3VECTORS(xSC, cSC, aSC_);
		}

		if (K.n == m)
		{
			if (cSC)
			{
				eSC2 = RVLDOTPRODUCT3(aSC_, aSC_) - rSC2;
				if (eSC2 < 1e-6)
					break;
			}
			else
				break;
		}

		if (cSC == NULL)
		{
			bExpandJ = false;
			if (K.n > nKPrev)
				nKPrev = K.n;
			else if (ex[k] >= ekPrev)
			{
				bExpandJ = true;
				if (jOut >= 0)
				{
					if (!bJ[jOut])
					{
						bJ[jOut] = true;
						J.Element[J.n++] = jOut;
					}
				}
				else
				{
					flags = RVLSOLVER_FLAGS_NOFEASIBLESOLUTION;
					break;
				}
			}
			ekPrev = ex[k];
		}

		r_ = J.n + mEq;
		if (r_ == 0)
		{
			// If x is not on any constraint, then v <- -A[k,:]

			a = (cSC ? aSC : A + n * k);
			for (i = 0; i < n; i++)
				v[i] = -a[i];
		}
		else if (r_ < n)
		{
			// Compute v.

			float *rowC = C;
			int u;
			for (j = 0; j < J.n; j++, rowC += n)
			{
				u = J.Element[j];
				a = A + n * u;
				for (i = 0; i < n; i++)
					rowC[i] = a[i];
			}
			if (AEq)
				memcpy(rowC, AEq, mEq * n * sizeof(float));

			seq.n = r_;
			nB_ = SubSpace(C, n, seq);

			colsT = n + r_;
			a = (cSC ? aSC : A + n * k);
			for (iB_ = 0; iB_ < nB_; iB_++)
			{
				c1[iB_] = 0.0f;
				for (iB = 0; iB < r_; iB++)
					c1[iB_] += a[B[iB]] * T[iB * colsT + B_[iB_]];
				c1[iB_] -= a[B_[iB_]];
			}
			nZero_e_J = 0;
			for (ie_J = 0; ie_J < r_; ie_J++)
			{
				c2[ie_J] = 0.0f;
				for (iB = 0; iB < r_; iB++)
					c2[ie_J] += (a[B[iB]] * T[iB * colsT + n + ie_J]);
				if (c2[ie_J] < 0.0f || ie_J >= J.n)
				{
					e_J[ie_J] = 0.0f;
					nZero_e_J++;
				}
				else
					e_J[ie_J] = c2[ie_J];
			}
			if (nZero_e_J == n)
			{
				flags = RVLSOLVER_FLAGS_NOFEASIBLESOLUTION;
				break;
			}
			for (iB = 0; iB < r_; iB++)
			{
				v[B[iB]] = 0.0f;
				for (iB_ = 0; iB_ < nB_; iB_++)
					v[B[iB]] -= T[iB * colsT + B_[iB_]] * c1[iB_];
				for (ie_J = 0; ie_J < r_; ie_J++)
					v[B[iB]] -= T[iB * colsT + n + ie_J] * e_J[ie_J];
			}
			for (iB_ = 0; iB_ < nB_; iB_++)
				v[B_[iB_]] = c1[iB_];
			RVLDOTPRODUCT(v, v, n, fTmp, i);
			if (fTmp < 1e-6)
			{
				flags = RVLSOLVER_FLAGS_NOFEASIBLESOLUTION;
				break;
			}
		}
		else
		{
			flags = RVLSOLVER_FLAGS_NOFEASIBLESOLUTION;
			break;
		}

		// Move to the target - constraint k.

		a = (cSC ? aSC : A + n * k);
		RVLDOTPRODUCT(a, v, n, wk, i);
		if (wk >= 0)
			break;
		if (cSC)
		{
			lvSC2 = RVLDOTPRODUCT3(vSC, vSC);
			laSC2 = RVLDOTPRODUCT3(aSC_, aSC_);
			// fTmp = wk * wk - lvSC2 * (laSC2 - rSCmin2);
			fTmp = wk * wk - lvSC2 * (laSC2 - rSC2);
			fTmp = (fTmp > 0.0f ? sqrt(fTmp) : 0.0f);
			s = (-wk - fTmp) / lvSC2;
		}
		else
			s = -ex[k] / wk;
		s_K = s;
		jOut = -1;
		for (i_ = 0; i_ < m; i_++)
		{
			j = S.Element[i_];
			if (cSC == NULL)
				if (j == k)
					continue;
			if (bJ[j])
				continue;
			if (!bK[j])
				continue;
			a = A + j * n;
			RVLDOTPRODUCT(a, v, n, w, i);
			if (w <= 0.0f)
				continue;
			s = -ex[j] / w;
			if (s < s_K)
			{
				s_K = s;
				jOut = j;
			}
		}
		// if (s_K * wk == 0.0f)
		//	int debug = 0;
		if (pL)
		{
			if (!ConstrainToLinearRegion(x, x0, s_K))
			{
				flags = RVLSOLVER_FLAGS_OUTOFLIN;
				break;
			}
		}
		for (i = 0; i < n; i++)
			x[i] += s_K * v[i];

		it++;
	}

	///

	for (j = 0; j < J.n; j++)
		bJ[J.Element[j]] = false;
	for (j = 0; j < K.n; j++)
		bK[K.Element[j]] = false;

	return flags;
}

uchar Solver::FeasibleSolution3(
	float *x0,
	float *x,
	float &exmax)
{
	int m = S.n;

	// x <- x0

	memcpy(x, x0, n * sizeof(float));

	// For x representing a pose vector.

	float *q = x;
	float *t = x + 3;
	float *vt = v + 3;

	// General purpose variables.

	int i_;
	float *a;

	// Check if x satisfies the equality constraint.

	int eqSign = 0;
	float eEq;
	int mEq_ = 0;
	float *AEq = NULL;
	float bEq;
	if (SEq.n > 0)
	{
		AEq = A + n * SEq.Element[0];
		bEq = b[SEq.Element[0]];
		RVLDOTPRODUCT(AEq, x, n, eEq, i_);
		eEq -= bEq;
		if (eEq < -1e-6)
		{
			eqSign = -1;
			eEq = -eEq;
		}
		else if (eEq <= 1e-6)
		{
			eqSign = 0;
			mEq_ = 1;
		}
		else
			eqSign = 1;
	}

	// If there are equality constraints, move x to sattisfy them.

	// RVLDOTPRODUCT(AEq, x, n, s, i_);
	// s -= bEq;
	// RVLSCALEVECTOR(AEq, s, VnTmp, n, i_);
	// RVLDIFVECTORS(x, VnTmp, n, x, i_);
	//// Only for debugging purpose!!!
	//{
	//	float fTmp;
	//	RVLDOTPRODUCT(AEq, x, n, fTmp, i_);
	//	fTmp -= bEq;
	//	int debug = 0;
	//}
	//

	// Radius constraint constants.

	float uRadius[3];
	float eRadius0;
	float *at;
	float kRadius = 2.0f / r;
	if (iRadius >= 0)
	{
		RVLMULMX3X3TVECT(pose0.R, pose0.t, uRadius);
		RVLSCALE3VECTOR(uRadius, kRadius, uRadius);
		eRadius0 = RVLDOTPRODUCT3(pose0.t, pose0.t) / r - r;
		a = A + n * iRadius;
		RVLNULL3VECTOR(a);
	}

	//

	uchar flags = 0x00;
	rL2 = rLinRegion * rLinRegion;
	memset(bB, 0, n * sizeof(bool));
	memset(zero_e_J, 0, n * sizeof(bool));
	int u, i, j, j_;
	int umax;
	float sk = 0.0f;
	int k = -1;
	float EPrev = 0.0f;
	K.n = 0;
	J.n = 0;
	int *JCombination;
	int iCombination;
	bool bFirst = true;
	float fTmp;
	float s;
	float gEq;
	float g0;
	float ex0;
	float dg, de;
	float aQ;
	bool bIntersection;
	while (true)
	{
		// Only for debugging purpose !!!
		// float eEq;
		// RVLDOTPRODUCT(AEq, x, n, eEq, i);
		//
		RVLDISTTOPLANES(A, b, n, S, x, ex, a, fTmp, i_, j_, i);
		exmax = 0.0f;
		umax = -1;
		if (iRadius >= 0)
		{
			ex[iRadius] = RVLDOTPRODUCT3(t, t) / r + RVLDOTPRODUCT3(uRadius, t) + eRadius0;
			exmax = ex[iRadius];
			umax = iRadius;
			a = A + n * iRadius;
			at = a + 3;
			RVLSCALE3VECTOR(t, kRadius, at);
			RVLSUM3VECTORS(at, uRadius, at);
			if (bJ[iRadius])
			{
				JBuff.n = 0;
				for (j_ = 0; j_ < J.n; j_++)
				{
					u = J.Element[j_];
					de = ex[iRadius] - ex[u];
					if (bJ[u] = (de < 1e-6))
						JBuff.Element[JBuff.n++] = u;
				}
				for (j_ = 0; j_ < JBuff.n; j_++)
					J.Element[j_] = JBuff.Element[j_];
				J.n = JBuff.n;
			}
		}
		if (!bFirst)
		{
			if (SEq.n > 0)
			{
				if (eqSign != 0)
				{
					RVLDOTPRODUCT(AEq, x, n, eEq, i_);
					eEq -= bEq;
					if (eqSign < 0)
						eEq = -eEq;
					if (eEq < 1e-6)
					{
						eqSign = 0;
						mEq_ = 1;
					}
				}
			}
			if (J.n > 0)
			{
				if (ex[j] < EPrev)
				{
					for (j_ = 0; j_ < K.n; j_++)
						bK[K.Element[j_]] = false;
					for (j_ = 0; j_ < J.n; j_++)
					{
						u = J.Element[j_];
						K.Element[j_] = u;
						bK[u] = true;
					}
					K.n = J.n;
					if (k >= 0 && !bK[k])
					{
						K.Element[j_] = k;
						bK[k] = true;
						K.n++;
					}
				}
			}
		}
		for (i_ = 0; i_ < m; i_++)
		{
			u = S.Element[i_];
			if (umax < 0 || ex[u] > exmax)
			{
				exmax = ex[u];
				umax = u;
			}
		}
		if (SEq.n > 0)
		{
			if (eqSign != 0)
			{
				if (eEq > exmax)
				{
					exmax = eEq;
					umax = -1;
					mEq_ = 1;
				}
			}
		}
		EPrev = exmax;
		if (umax >= 0)
		{
			if (!bK[umax])
			{
				K.Element[K.n++] = umax;
				bK[umax] = true;
			}
		}
		if (K.n + mEq_ <= n)
		{
			for (j_ = 0; j_ < K.n; j_++)
			{
				u = K.Element[j_];
				if (!bJ[u])
				{
					J.Element[J.n++] = u;
					bJ[u] = true;
				}
			}
			if (J.n + mEq_ == 1)
			{
				if (J.n == 1)
				{
					j = J.Element[0];
					a = A + n * j;
					for (i_ = 0; i_ < n; i_++)
						v[i_] = -a[i_];
				}
				else
				{
					if (eqSign == 1)
						for (i_ = 0; i_ < n; i_++)
							v[i_] = -AEq[i_];
					else if (eqSign == -1)
						for (i_ = 0; i_ < n; i_++)
							v[i_] = AEq[i_];
				}
			}
			else if (!Gradient(J, AEq, mEq_, eqSign))
				break;
		}
		else
		{
			int nJ = n - mEq_;
			if (J.n > nJ)
				for (j_ = 0; j_ < J.n; j_++)
					bJ[J.Element[j_]] = false;
			J.n = nJ;
			Combinations(K.n, J.n, JCombinations, JCombinationsMem);
			for (iCombination = 0; iCombination < JCombinations.h; iCombination++)
			{
				JCombination = JCombinations.Element + JCombinations.w * iCombination;
				for (j_ = 0; j_ < J.n; j_++)
					bJ[J.Element[j_]] = false;
				for (j_ = 0; j_ < J.n; j_++)
				{
					J.Element[j_] = K.Element[JCombination[j_]];
					bJ[J.Element[j_]] = true;
				}
				if (Gradient(J, AEq, mEq_, eqSign))
				{
					j = J.Element[0];
					a = A + n * j;
					RVLDOTPRODUCT(a, v, n, g[j], i_);
					if (g[j] >= -1e-6)
						continue;
					for (j_ = 0; j_ < K.n; j_++)
					{
						u = K.Element[j_];
						if (!bJ[u])
						{
							a = A + n * u;
							RVLDOTPRODUCT(a, v, n, g[u], i_);
							if (g[u] > g[j])
								break;
						}
					}
					if (j_ >= K.n)
						break;
				}
			}
			if (iCombination >= JCombinations.h)
				break;
		}
		// Only for debugging purpose !!!
		// float evEq;
		// RVLDOTPRODUCT(AEq, v, n, evEq, i);
		//
		RVLMULMXROWSVECT(A, n, S, v, g, a, fTmp, i_, j_, i);
		if (iRadius > 0)
		{
			a = A + n * iRadius;
			RVLDOTPRODUCT(a, v, n, g[iRadius], i_);
			aQ = kRadius * RVLDOTPRODUCT3(vt, vt);
		}
		if (SEq.n > 0)
		{
			if (eqSign != 0)
			{
				RVLDOTPRODUCT(AEq, v, n, gEq, i_);
				if (eqSign < 0)
					gEq = -gEq;
			}
		}
		k = -2;
		if (J.n > 0)
		{
			j = J.Element[0];
			if (iRadius >= 0)
			{
				if (bJ[iRadius])
				{
					sk = -g[iRadius] / aQ;
					k = iRadius;
					j = iRadius;
				}
			}
			g0 = g[j];
			ex0 = ex[j];
		}
		else
		{
			g0 = gEq;
			ex0 = eEq;
		}
		for (i_ = 0; i_ < m; i_++)
		{
			u = S.Element[i_];
			if (!bJ[u] && !bK[u])
			{
				dg = g0 - g[u];
				if (dg < 0.0f)
				{
					de = ex0 - ex[u];
					if (j == iRadius)
					{
						fTmp = dg * dg - 2.0f * aQ * de;
						if (bIntersection = (fTmp > 0.0f))
							s = (-dg - sqrt(fTmp)) / aQ;
					}
					else
					{
						s = -de / dg;
						bIntersection = true;
					}
					if (k < 0 || (bIntersection && s < sk))
					{
						sk = s;
						k = u;
					}
				}
			}
		}
		// Only for debugging purpose!!!
		// fTmp = ex[k] + sk * g[k];
		// fTmp = ex[j] + 0.5f * sk * sk * aQ + g[j] * sk;

		//
		if (SEq.n > 0)
		{
			if (eqSign != 0)
			{
				if (mEq_ == 0)
				{
					if (gEq > g[j])
					{
						s = (ex[j] - eEq) / (gEq - g[j]);
						if (s < sk)
						{
							sk = s;
							k = -1;
							mEq_ = 1;
						}
					}
				}
				else
				{
					if (gEq < -1e-6)
					{
						s = -eEq / gEq;
						if (s < sk)
						{
							sk = s;
							k = -1;
						}
					}
				}
			}
		}
		if (iRadius)
			if (!bJ[iRadius])
			{
				u = iRadius;
				dg = g[u] - g0;
				de = ex[u] - ex0;
				fTmp = dg * dg - 2.0f * aQ * de;
				s = (-dg + sqrt(fTmp)) / aQ;
				if (k < 0 || s < sk)
				{
					sk = s;
					k = u;
				}
			}

		if (linRegionX.n > 0)
		{
			if (k < -1)
				sk = -1.0f;
			if (!ConstrainToLinearRegion(x, x0, sk))
			{
				flags |= RVLSOLVER_FLAGS_OUTOFLIN;
				break;
			}
		}
		else if (k < -1)
		{
			flags |= RVLSOLVER_FLAGS_UNBOUNDED;
			break;
		}

		for (i = 0; i < n; i++)
			x[i] += sk * v[i];
		if (k >= 0 && !bK[k])
		{
			K.Element[K.n++] = k;
			bK[k] = true;
		}
		bFirst = false;
	}

	for (j = 0; j < J.n; j++)
		bJ[J.Element[j]] = false;
	for (j = 0; j < K.n; j++)
		bK[K.Element[j]] = false;

	return flags;
}

bool Solver::Gradient(
	Array<int> J,
	float *AEq,
	int mEq,
	int eqSign)
{
	int r_ = J.n - 1;
	int u = J.Element[r_];
	float *a0 = A + n * u;
	int i, j;
	float *a;
	float *rowC = C;
	for (j = 0; j < r_; j++, rowC += n)
	{
		u = J.Element[j];
		a = A + n * u;
		RVLDIFVECTORS(a, a0, n, rowC, i);
	}
	if (mEq > 0)
	{
		switch (eqSign)
		{
		case 1:
			RVLDIFVECTORS(AEq, a0, n, rowC, i);
			break;
		case -1:
			for (i = 0; i < n; i++)
				rowC[i] = -AEq[i] - a0[i];
			break;
		case 0:
			memcpy(rowC, AEq, mEq * n * sizeof(float));
		}
	}
	r_ += mEq;
	seq.n = r_;
	int nB_ = SubSpace(C, n, seq);
	int colsT = n + r_;
	float lc2;
	int iB, iB_;
	u = J.Element[j];
	a = A + n * u;
	lc2 = 0.0f;
	for (iB_ = 0; iB_ < nB_; iB_++)
	{
		c1[iB_] = 0.0f;
		for (iB = 0; iB < r_; iB++)
			c1[iB_] -= a[B[iB]] * T[iB * colsT + B_[iB_]];
		c1[iB_] += a[B_[iB_]];
		lc2 += c1[iB_] * c1[iB_];
	}
	bool bNonZeroGradient = (lc2 >= 1e-6);
	if (bNonZeroGradient)
		for (iB_ = 0; iB_ < nB_; iB_++)
			v[B_[iB_]] = -c1[iB_];
	else
		for (iB_ = 0; iB_ < nB_; iB_++)
			v[B_[iB_]] = 1.0f;
	for (iB = 0; iB < r_; iB++)
	{
		v[B[iB]] = 0.0f;
		for (iB_ = 0; iB_ < nB_; iB_++)
			v[B[iB]] -= T[iB * colsT + B_[iB_]] * v[B_[iB_]];
	}

	return bNonZeroGradient;
}

// Given a matrix A(nxn) and an array J with r indices, function SubSpace forms matrix A_J(rxn) whose rows are rows of A with indices stored in J
// and then computes index array B and matrices T1(rx(n-r)) and T2(rxr) such that vector x, composed of vectors x_B and x_B_ in such a way that
// x_B contains elements of x with indices stored in B and x_B_ contains the remaining elements of x,
// represents the subspace of solutions to the equation A_J * x = b, if x_B = -T1 * x_B_ + T2 * b for any b and x_B_.
// The function decomposes A_J into matrices A_JB(rxr) and A_JB_(rx(n-r)) where A_JB is composed of r columns of A_J with indices stored in array B
// and A_JB_ is composed of the remaining columns of A_J.
// Then, it computes matrix T(rx(n+r)) whose first n columns represent matrix T1_ = inv(A_JB) * A_J and the remaining columns represent matrix T2 = inv(A_JB).
// Matrix T1 can be obtained by taking the columns of T1_ with indices B_.
// The outputs of this function are matrix T, array B and array B_, which is the complement of B.

int Solver::SubSpace(
	float *A,
	int n,
	Array<int> J)
{
	int colsT = n + J.n;
	memset(T, 0, J.n * (n + J.n) * sizeof(float));
	int j;
	for (j = 0; j < J.n; j++)
	{
		memcpy(T + colsT * j, A + n * J.Element[j], n * sizeof(float));
		T[n + j + colsT * j] = 1.0f;
	}
	int i;
	int j_;
	float *rowT = T;
	float *rowT_;
	float pivotAbsVal;
	float fTmp;
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
	int nB_ = 0;
	for (i = 0; i < n; i++)
		if (bB[i])
			bB[i] = false;
		else
			B_[nB_++] = i;

	return nB_;
}

bool Solver::ConstrainToLinearRegion(
	float *x,
	float *x0,
	float s)
{
	int i, i_;
	float fTmp;
	bool bOutOfLinearRegion;
	if (s >= 0.0f)
	{
		for (i = 0; i < n; i++)
			x_[i] = x[i] + s * v[i];
		float r2 = 0.0f;
		for (i_ = 0; i_ < linRegionX.n; i_++)
		{
			i = linRegionX.Element[i_];
			fTmp = x_[i] - x0[i];
			r2 += fTmp * fTmp;
		}
		bOutOfLinearRegion = (r2 > rL2);
	}
	else
		bOutOfLinearRegion = true;
	if (bOutOfLinearRegion)
	{
		float aL = 0.0f;
		float bL = 0.0f;
		float cL = -rL2;
		for (i_ = 0; i_ < linRegionX.n; i_++)
		{
			i = linRegionX.Element[i_];
			aL += v[i] * v[i];
			fTmp = x[i] - x0[i];
			bL += fTmp * v[i];
			cL += fTmp * fTmp;
		}
		float s_ = (-bL + sqrt(bL * bL - aL * cL)) / aL;
		for (i = 0; i < n; i++)
			x[i] += s_ * v[i];
		// Only for debugging purpose!!!
		// float r2 = 0.0f;
		// for (i_ = 0; i_ < pL->n; i_++)
		//{
		//	i = pL->Element[i_];
		//	fTmp = x[i] - x0[i];
		//	r2 += fTmp * fTmp;
		//}
		//
		return false;
	}
	return true;
}
