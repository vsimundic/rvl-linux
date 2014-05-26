// RVLHistogram.h: interface for the CRVLHistogram class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLHISTOGRAM_H__99525D61_A68D_4AF5_B232_AB00DB60009D__INCLUDED_)
#define AFX_RVLHISTOGRAM_H__99525D61_A68D_4AF5_B232_AB00DB60009D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVLHistogram  
{
public:
	int m_nBins;
	int m_miniBin, m_maxiBin;
	double m_BinSize;
	int *m_Histogram;
	CRVLMChain m_Data;
	CRVLMem *m_pMem;
	double m_mean;
	double m_min, m_max;
	char *m_Name;

public:
	void Save(FILE *fp,
			  char *FormatBinSize = NULL);
	void Init(double BinSize, 
			  CRVLMem *pMem,
			  char *Name = NULL,
			  int nData = 1);
	void Create(int iData = 0);
	BOOL Update(double data);
	CRVLHistogram();
	virtual ~CRVLHistogram();

};

#endif // !defined(AFX_RVLHISTOGRAM_H__99525D61_A68D_4AF5_B232_AB00DB60009D__INCLUDED_)
