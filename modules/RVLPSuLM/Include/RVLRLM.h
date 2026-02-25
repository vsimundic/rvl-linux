#pragma once

class CRVLRLM
{
public:
	DWORD m_Flags;
	DWORD m_DebugFlags;
	int m_iLocalModel;

public:
	CRVLRLM(void);
	virtual ~CRVLRLM(void);
	virtual int GetnModels();
	virtual void InitPathPlanningNodes(void *vpNodeArray);
	virtual int GetMostProbableDistance(int iPose1, 
										int iPose2,
										double *dX,
										double &dAlpha);
	virtual int GetPathPlanningNodeIndex(int iPose0, int iNeighbor);
	virtual int GetnReachableModels(int iPose0);
};