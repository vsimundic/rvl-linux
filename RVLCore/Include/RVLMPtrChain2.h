// RVLMPtrChain2.h: interface for the CRVLMPtrChain2 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLMPTRCHAIN2_H__C91B63B1_4B00_4A49_8EE0_C4B83ED690E4__INCLUDED_)
#define AFX_RVLMPTRCHAIN2_H__C91B63B1_4B00_4A49_8EE0_C4B83ED690E4__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVLMPtrChain2  
{
public:
	RVLARRAY *m_pArray;
	RVLPTRCHAIN_ELEMENT *m_pCurrent;
	RVLPTRCHAIN_ELEMENT *m_pNext;
	CRVLMem *m_pMem;

public:	
	void ConvertToArray(RVLARRAY *pArray, 
		CRVLMem *pMem,
		int nElements = -1);
	void * GetLast();
	void Create(RVLARRAY *pArray, CRVLMem *pMem);
	void Start();
	void RemoveAll();
	void * GetNext();
	void Add(void *pData);
	CRVLMPtrChain2(RVLARRAY *pArray, CRVLMem *pMem);
	CRVLMPtrChain2();
	virtual ~CRVLMPtrChain2();

};

#endif // !defined(AFX_RVLMPTRCHAIN2_H__C91B63B1_4B00_4A49_8EE0_C4B83ED690E4__INCLUDED_)
