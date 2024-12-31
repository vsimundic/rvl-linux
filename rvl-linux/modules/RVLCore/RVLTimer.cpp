// RVLTimer.cpp: implementation of the CRVLTimer class.
//
//////////////////////////////////////////////////////////////////////

#include "RVLPlatform.h"

#ifndef RVLLINUX
#define RVLTIME_MEASUREMENT_WIN
#endif

#ifdef RVLTIME_MEASUREMENT_WIN
#include <Windows.h>
#endif

#include "RVLTimer.h"
#include <math.h>
#include <time.h>

#ifdef RVLTIME_MEASUREMENT_WIN
struct RVLTimerWinData
{
	LARGE_INTEGER CNTRStart;
	LARGE_INTEGER CNTREnd;
	LARGE_INTEGER frequency;
};
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLTimer::CRVLTimer()
{
#ifdef RVLTIME_MEASUREMENT_WIN
	RVLTimerWinData *pData = new RVLTimerWinData;

	vpData = pData;

	QueryPerformanceFrequency(&(pData->frequency));
#endif
}

CRVLTimer::~CRVLTimer()
{
#ifdef RVLTIME_MEASUREMENT_WIN
	RVLTimerWinData *pData = (RVLTimerWinData *)vpData;

	delete pData;
#endif
}

double CRVLTimer::GetTime()
{
#ifdef RVLTIME_MEASUREMENT_WIN
	RVLTimerWinData *pData = (RVLTimerWinData *)vpData;

	return (double)(pData->CNTREnd.QuadPart - pData->CNTRStart.QuadPart) * 1000.0 / (double)(pData->frequency.QuadPart);
#else
	return ((float)clock())/CLOCKS_PER_SEC;
#endif
}

void CRVLTimer::Start()
{
#ifdef RVLTIME_MEASUREMENT_WIN
	RVLTimerWinData *pData = (RVLTimerWinData *)vpData;

	QueryPerformanceCounter(&(pData->CNTRStart));
#endif
}

void CRVLTimer::Stop()
{
#ifdef RVLTIME_MEASUREMENT_WIN
	RVLTimerWinData *pData = (RVLTimerWinData *)vpData;

	QueryPerformanceCounter(&(pData->CNTREnd));
#endif
}
