// RVLMPtrChain.h: interface for the CRVLMPtrChain class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLMPTRCHAIN_H__0517A2FB_E91C_41D3_8754_A32D5A2F1A96__INCLUDED_)
#define AFX_RVLMPTRCHAIN_H__0517A2FB_E91C_41D3_8754_A32D5A2F1A96__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVLMPtrChain  
{
public:
	int m_nElements;
	RVLPTRCHAIN_ELEMENT *m_pFirst;
	RVLPTRCHAIN_ELEMENT *m_pLast;
	RVLPTRCHAIN_ELEMENT *m_pCurrent;
	RVLPTRCHAIN_ELEMENT *m_pNext;
	CRVLMem *m_pMem;
	
public:
	void RemoveAt(RVLPTRCHAIN_ELEMENT *pCurrent);
	void Copy(CRVLMPtrChain *pChain);
	void InsertAt(void *pData, RVLPTRCHAIN_ELEMENT *pCurrent);
	void Start();
	void RemoveAll();
	void * GetNext();
	void Add(void *pData);
	CRVLMPtrChain(CRVLMem *pMem);
	CRVLMPtrChain();
	virtual ~CRVLMPtrChain();
};

#endif // !defined(AFX_RVLMPTRCHAIN_H__0517A2FB_E91C_41D3_8754_A32D5A2F1A96__INCLUDED_)
