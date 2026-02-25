// RVLChain.h: Schnittstelle für die Klasse CRVLChain.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLCHAIN_H__2D97D4E7_9E29_4B4D_A728_28A2C67E95BC__INCLUDED_)
#define AFX_RVLCHAIN_H__2D97D4E7_9E29_4B4D_A728_28A2C67E95BC__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVLChain  
{
public:
	int m_nElements;
	int m_DataSize;
	BYTE *m_pFirst;
	BYTE *m_pLast;
	BYTE *m_pCurrent;
	BYTE *m_pNext;
private:
	int m_ElementSize;

public:
	void * GetLast();
	void Start();
	void * GetNext();
	void RemoveAll();
	void Add(void *pData);
	CRVLChain(int DataSize);
	CRVLChain();
	virtual ~CRVLChain();
};

#endif // !defined(AFX_RVLCHAIN_H__2D97D4E7_9E29_4B4D_A728_28A2C67E95BC__INCLUDED_)
