// RVLPtrChain.h: Schnittstelle für die Klasse CRVLPtrChain.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLPTRCHAIN_H__E3AA2526_6115_4484_BFE3_726DB1EE8F7A__INCLUDED_)
#define AFX_RVLPTRCHAIN_H__E3AA2526_6115_4484_BFE3_726DB1EE8F7A__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

struct RVLPTRCHAIN_ELEMENT
{
	void *pData;
	RVLPTRCHAIN_ELEMENT *pNext;
};


class CRVLPtrChain  
{
public:
	int m_nElements;
	RVLPTRCHAIN_ELEMENT *m_pFirst;
	RVLPTRCHAIN_ELEMENT *m_pLast;
	RVLPTRCHAIN_ELEMENT *m_pCurrent;
	RVLPTRCHAIN_ELEMENT *m_pNext;
	
public:
	void InsertAt(void *pData, RVLPTRCHAIN_ELEMENT *pCurrent);
	void Start();
	void RemoveAll();
	void * GetNext();
	void Add(void *pData);
	CRVLPtrChain();
	virtual ~CRVLPtrChain();
};

#endif // !defined(AFX_RVLPTRCHAIN_H__E3AA2526_6115_4484_BFE3_726DB1EE8F7A__INCLUDED_)
