#pragma once

namespace RVL
{
	struct AccuSphereVote
	{
		float A[3];
		float w;
		float wA[3];
		int idx;
		AccuSphereVote* pNext;
	};

	struct AccuSphereBin
	{
		QList<AccuSphereVote> voteList;
		float w;
	};

	class AccuSphere
	{
	public:
		AccuSphere();
		virtual ~AccuSphere();
		void Create(
			int resolutionIn,
			bool bSymmetricIn = false);
		void Init(int maxnVotesIn);
		void Free();
		void Vote(
			float* A,
			float w,
			int idx);
		AccuSphereBin* GetBin(
			int iAccu,
			int i,
			int j);
		void Neighbors(
			float* A,
			Array<AccuSphereVote*>& neighbors);
		int GetNumOfVotes();
		void GetBinMean(
			AccuSphereBin* pBin,
			float* V);
		void GetLocalMaximumByMeanShift(
			float* VIn,
			float csMeanShiftWinAngle,
			int nIterations,
			float* VOut,
			AccuSphereVote** accuSphereVoteBuff = NULL);
		void Visualize(
			Visualizer* pVisualizer,
			float* A = NULL,
			bool bNeighborhood = false,
			float neighborhoodAngleDeg = 0.0f);
	private:
		void GetBinPosition(
			float* A,
			float* A_,
			int& iAxis0,
			int& iAccu,
			int& i,
			int& j,
			bool bNeighborhoodAnchor = false);
		bool CorrectBin(
			int& iAxis0,
			int& iAccu,
			int& i,
			int& j);

	public:
		int resolution;
	private:
		AccuSphereBin* accu[6];
		AccuSphereVote* voteMem;
		int accuSize;
		float accuRange;
		int maxnVotes;
		AccuSphereVote* pVote;
		bool bSymmetric;
	};
}

