// RVL2DCellArray.h: interface for the CRVL2DCellArray class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL2DCELLARRAY_H__E653CEB9_3B69_44EB_AFD4_062CB379ADA5__INCLUDED_)
#define AFX_RVL2DCELLARRAY_H__E653CEB9_3B69_44EB_AFD4_062CB379ADA5__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVL2DCellArray  
{
public:	
	int m_CellSize;
	int m_nCols, m_nRows;
	int m_Width, m_Height;
	CRVLQListArray m_CellArray;
	CRVLMem *m_pMem0;
	CRVLMem *m_pMem;

public:
	void RemoveAll();
	void Get(int minu, int maxu, int minv, int maxv, void **OutputArray, int &nOut);
	void Init();
	void Add(int u, int v, void *pObject);
	CRVL2DCellArray();
	virtual ~CRVL2DCellArray();

};

#endif // !defined(AFX_RVL2DCELLARRAY_H__E653CEB9_3B69_44EB_AFD4_062CB379ADA5__INCLUDED_)
