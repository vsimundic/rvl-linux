#include "RVLCore2.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include "vtkPLY.h"
#include "vtkStringArray.h"
#endif
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "AccuSphere.h"

// Class AccuSphere is tested only for bSymmetric = true!

using namespace RVL;

AccuSphere::AccuSphere()
{
	resolution = 0;
	maxnVotes = 0;
	for (int iAccu = 0; iAccu < 6; iAccu++)
		accu[iAccu] = NULL;
	voteMem = NULL;
	bSymmetric = false;
}

AccuSphere::~AccuSphere()
{
	Free();
}

void AccuSphere::Create(
	int resolutionIn,
	bool bSymmetricIn)
{
	resolution = resolutionIn;
	accuSize = resolution * resolution;
	accuRange = 0.5f * (float)resolution;
	bSymmetric = bSymmetricIn;

	for (int iAccu = 0; iAccu < 6; iAccu++)
	{
		RVL_DELETE_ARRAY(accu[iAccu]);
		accu[iAccu] = new AccuSphereBin[accuSize];
	}
}

void AccuSphere::Init(int maxnVotesIn)
{
	int iBin;
	AccuSphereBin* pBin;
	QList<AccuSphereVote>* pList;
	for (int iAccu = 0; iAccu < 6; iAccu++)
	{
		pBin = accu[iAccu];
		for (iBin = 0; iBin < accuSize; iBin++, pBin++)
		{
			pList = &(pBin->voteList);
			RVLQLIST_INIT(pList);
			pBin->w = 0.0f;
		}
	}
	if (maxnVotesIn > maxnVotes)
	{
		maxnVotes = maxnVotesIn;
		RVL_DELETE_ARRAY(voteMem);
		voteMem = new AccuSphereVote[maxnVotes];
	}
	pVote = voteMem;
}

void AccuSphere::Free()
{
	for (int iAccu = 0; iAccu < 6; iAccu++)
		RVL_DELETE_ARRAY(accu[iAccu]);
	RVL_DELETE_ARRAY(voteMem);
}

void AccuSphere::GetBinPosition(
	float* A,
	float* A_,
	int& iAxis0,
	int& iAccu,
	int& i,
	int& j,
	bool bNeighborhoodAnchor)
{
	float absA[3];
	absA[0] = RVLABS(A[0]); absA[1] = RVLABS(A[1]); absA[2] = RVLABS(A[2]);
	RVL3DVECTORMAXIDX(absA, iAxis0);
	int iAxis1 = (iAxis0 + 1) % 3;
	int iAxis2 = (iAxis1 + 1) % 3;
	if (bSymmetric)
	{
		if (A[iAxis0] < 0.0f)
		{
			RVLNEGVECT3(A, A_)
		}
		else
		{
			RVLCOPY3VECTOR(A, A_)
		}
		iAccu = iAxis0;
	}
	else
	{
		RVLCOPY3VECTOR(A, A_)
		iAccu = (A[iAxis0] > 0.0f ? iAxis0 : iAxis0 + 3);
	}
	float s = accuRange / absA[iAxis0];
	float V[3];
	RVLSCALE3VECTOR(A_, s, V);
	float p = V[iAxis1] + accuRange;
	if (bNeighborhoodAnchor)
		p -= 0.5f;
	i = (int)floor(p);
	if (i == resolution)
		i--;
	else if (i < 0)
		i = 0;
	p = V[iAxis2] + accuRange;
	if (bNeighborhoodAnchor)
		p -= 0.5f;
	j = (int)floor(p);
	if (j == resolution)
		j--;
	else if (j < 0)
		j = 0;
}

bool AccuSphere::CorrectBin(
	int& iAxis0,
	int& iAccu,
	int& i,
	int& j)
{
	bool bOut;
	bool bOutNeg;
	int j_ = j;
	if (bOut = ((bOutNeg = (i < 0)) || i >= resolution))
	{
		iAxis0 = (iAxis0 + 1) % 3;
		i = j;
		if (bSymmetric)
		{
			if (bOutNeg)
			{
				j_ = 0;
				i = 5 - i;
			}
			else
				j_ = resolution - 1;
			iAccu = iAxis0;
		}
		else
		{
			j_ = (iAccu < 3 ? resolution - 1 : 0);
			iAccu = (bOutNeg ? iAxis0 + 3 : iAxis0);
		}
	}
	if ((bOutNeg = (j < 0)) || j >= resolution)
	{
		if (bOut)
			return false;
		iAxis0 = (iAxis0 + 2) % 3;
		j_ = i;
		if (bSymmetric)
		{
			if (bOutNeg)
			{
				i = 0;
				j_ = 5 - j_;
			}
			else
				i = resolution - 1;
			iAccu = iAxis0;
		}
		else
		{
			i = (iAccu < 3 ? resolution - 1 : 0);
			iAccu = (bOutNeg ? iAxis0 + 3 : iAxis0);
		}
	}
	j = j_;
	return true;
}

void AccuSphere::Vote(
	float* A,
	float w,
	int idx)
{
	int i, j;
	int iAxis0, iAccu;
	float A_[3];
	GetBinPosition(A, A_, iAxis0, iAccu, i, j);
	//if (iAccu == 0 && i == 2 && j == 3)
	//	int debug = 0;
	int iBin = i + j * resolution;
	float lenA = sqrt(RVLDOTPRODUCT3(A_, A_));
	RVLSCALE3VECTOR2(A_, lenA, pVote->A);
	pVote->w = w;
	RVLSCALE3VECTOR(pVote->A, pVote->w, pVote->wA);
	pVote->idx = idx;
	AccuSphereBin* pBin = accu[iAccu] + iBin;
	QList<AccuSphereVote>* pList = &(pBin->voteList);
	RVLQLIST_ADD_ENTRY(pList, pVote);
	pBin->w += pVote->w;
	pVote++;
}

AccuSphereBin* AccuSphere::GetBin(
	int iAccu,
	int i,
	int j)
{
	return accu[iAccu] + i + j * resolution;
}

void AccuSphere::Neighbors(
	float* A,
	Array<AccuSphereVote*>& neighbors)
{
	neighbors.n = 0;
	AccuSphereBin* pBin;
	QList<AccuSphereVote>* pList;
	int i, j;
	int iAxis0, iAccu;
	float A_[3];
	GetBinPosition(A, A_, iAxis0, iAccu, i, j, true);
	int di, dj, i_, j_, i__, j__;
	int iBin;
	AccuSphereVote* pVote_;
	int iAxis0__;
	for (dj = 0; dj < 2; dj++)
	{
		j_ = j + dj;
		for (di = 0; di < 2; di++)
		{
			i_ = i + di;
			i__ = i_;
			j__ = j_;
			iAxis0__ = iAxis0;
			if (CorrectBin(iAxis0__, iAccu, i__, j__))
			{
				iBin = i__ + j__ * resolution;
				pBin = accu[iAccu] + iBin;
				pList = &(pBin->voteList);
				pVote_ = pList->pFirst;
				while (pVote_)
				{
					neighbors.Element[neighbors.n++] = pVote_;
					pVote_ = pVote_->pNext;
				}
				int debug = 0;
			}
		}
	}
}

int AccuSphere::GetNumOfVotes()
{
	return pVote - voteMem;
}

void AccuSphere::GetBinMean(
	AccuSphereBin* pBin,
	float* V)
{
	RVLNULL3VECTOR(V);
	AccuSphereVote* pVote = pBin->voteList.pFirst;
	while (pVote)
	{
		RVLSUM3VECTORS(V, pVote->wA, V);
		pVote = pVote->pNext;
	}
	float fTmp;
	RVLNORM3(V, fTmp);
}

void AccuSphere::GetLocalMaximumByMeanShift(
	float* VIn,
	float csMeanShiftWinAngle,
	int nIterations,
	float* VOut,
	AccuSphereVote** accuSphereVoteBuff)
{
	Array<AccuSphereVote*> neighbors;
	neighbors.Element = (accuSphereVoteBuff ? accuSphereVoteBuff : new AccuSphereVote*[GetNumOfVotes()]);

	int i;
	float fTmp;
	float newV[3];
	float cs;
	float wA[3];
	RVLCOPY3VECTOR(VIn, VOut);
	AccuSphereVote* pVote_;
	for (int it = 0; it < nIterations; it++)
	{
		Neighbors(VOut, neighbors);
		RVLNULL3VECTOR(newV);
		for (i = 0; i < neighbors.n; i++)
		{
			pVote_ = neighbors.Element[i];
			cs = RVLDOTPRODUCT3(pVote_->A, VOut);
			if (cs > 0.0f)
			{
				RVLCOPY3VECTOR(pVote_->wA, wA);
			}
			else
			{
				cs = -cs;
				RVLNEGVECT3(pVote_->wA, wA);
			}
			if (cs < csMeanShiftWinAngle)
				continue;
			RVLSUM3VECTORS(newV, wA, newV);
		}
		RVLNORM3(newV, fTmp);

		// Visualization of mean shift.

		//if (iiConvex == 1 && iProminentAccuSphereBin == 8)
		//{
		//	Visualizer visualizer;
		//	visualizer.Create();
		//	accuSphere.Visualize(&visualizer, newV, true);
		//	visualizer.Run();
		//	int debug = 0;
		//}

		//

		if (RVLDOTPRODUCT3(newV, VOut) > 0.9999f)
			break;
		RVLCOPY3VECTOR(newV, VOut);
	}

	if (accuSphereVoteBuff == NULL)
		delete[] neighbors.Element;
}

void AccuSphere::Visualize(
	Visualizer* pVisualizer,
	float* A,
	bool bNeighborhood,
	float neighborhoodAngleDeg)
{
	int nVotes = pVote - voteMem;
	uchar blue[] = { 0, 0, 255 };
	uchar green[] = { 0, 255, 0 };
	uchar magenta[] = { 255, 0, 255 };
	Array<Point> visPts;
	visPts.Element = new Point[nVotes];
	Point* pVisPt = visPts.Element;
	int i, j;
	AccuSphereVote* pVote_;
	int nAccu = (bSymmetric ? 3 : 6);
	for (int iAccu = 0; iAccu < nAccu; iAccu++)
	{
		for (i = 0; i < resolution; i++)
			for (j = 0; j < resolution; j++)
			{
				AccuSphereBin* pAccuBin = GetBin(iAccu, i, j);
				pVote_ = pAccuBin->voteList.pFirst;
				while (pVote_)
				{
					RVLCOPY3VECTOR(pVote_->A, pVisPt->P);
					pVisPt++;
					pVote_ = pVote_->pNext;
				}
			}
	}
	visPts.n = pVisPt - visPts.Element;
	pVisualizer->DisplayPointSet<float, Point>(visPts, blue, 2.0f);

	if (A)
	{
		float A_[3];
		RVLCOPY3VECTOR(A, A_);
		float fTmp;
		RVLNORM3(A_, fTmp);
		RVLCOPY3VECTOR(A_, visPts.Element[0].P);
		visPts.n = 1;
		pVisualizer->DisplayPointSet<float, Point>(visPts, green, 6.0f);

		if (bNeighborhood)
		{
			float fResolution = (float)(resolution);
			float csNeighborhoodAngle = (neighborhoodAngleDeg > 0.0f ? DEG2RAD * neighborhoodAngleDeg : 1.0f / sqrt(1.0f + 1.0f / (fResolution * fResolution)));
			Array<AccuSphereVote*> neighbors;
			neighbors.Element = new AccuSphereVote * [nVotes];
			Neighbors(A_, neighbors);
			float cs;
			pVisPt = visPts.Element;
			for (i = 0; i < neighbors.n; i++)
			{
				pVote_ = neighbors.Element[i];
				cs = RVLDOTPRODUCT3(pVote_->A, A_);
				if (RVLABS(cs) < csNeighborhoodAngle)
					continue;
				RVLCOPY3VECTOR(pVote_->A, pVisPt->P);
				pVisPt++;
			}
			visPts.n = pVisPt - visPts.Element;
			pVisualizer->DisplayPointSet<float, Point>(visPts, magenta, 3.0f);
			delete[] neighbors.Element;
		}
	}

	delete[] visPts.Element;
}