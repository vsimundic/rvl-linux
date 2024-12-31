// RVLTimer.h: interface for the CRVLTimer class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLTIMER_H__38AF3E50_B50D_48FD_925C_63F1BE912F36__INCLUDED_)
#define AFX_RVLTIMER_H__38AF3E50_B50D_48FD_925C_63F1BE912F36__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVLTimer  
{
public:
	CRVLTimer();
	virtual ~CRVLTimer();
	void Start();
	void Stop();
	virtual double GetTime();

	void *vpData;
	double m_TimerFrequency;
};

#endif // !defined(AFX_RVLTIMER_H__38AF3E50_B50D_48FD_925C_63F1BE912F36__INCLUDED_)
