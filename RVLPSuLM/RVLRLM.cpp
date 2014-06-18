#include "RVLCore.h"
#include "RVLRLM.h"

CRVLRLM::CRVLRLM(void)
{
	m_Flags = 0x00000000;
	m_DebugFlags = 0x00000000;
}

CRVLRLM::~CRVLRLM(void)
{

}

int CRVLRLM::GetnModels()
{
	return 0;
}

void CRVLRLM::InitPathPlanningNodes(void *vpNodeArray)
{

}

int CRVLRLM::GetMostProbableDistance(	int iPose1, 
										int iPose2, 
										double *dX,
										double &dAlpha)
{
	return 0;
}

int CRVLRLM::GetPathPlanningNodeIndex(int iPose0, int iNeighbor)
{
	return -1;
}

int CRVLRLM::GetnReachableModels(int iPose0)
{
	return 0;
}