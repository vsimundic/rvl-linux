// RVLRelation.h: interface for the CRVLRelation class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLRELATION_H__458B5D54_79D7_40D3_99FA_ADF16FBE0FA5__INCLUDED_)
#define AFX_RVLRELATION_H__458B5D54_79D7_40D3_99FA_ADF16FBE0FA5__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CRVLRelation  
{
public:
	//WORD m_Type;
	CRVLObject2 *m_pObject[2];

public:
	void ConnectObjects(CRVLObject2 *pObject1, 
						CRVLObject2 *pObject2,
						int RelListID,
						CRVLMem *pMem1, CRVLMem *pMem2);
	void ConnectObjects(CRVLObject2 *pObject1, 
						CRVLObject2 *pObject2,
						int RelListID);
	void ConnectObjects(CRVLObject2 *pObject1, 
						CRVLObject2 *pObject2,
						int iRelList1, 
						int iRelList2);
	void ConnectObjectsUniq(CRVLObject2 *pObject1, 
						    CRVLObject2 *pObject2,
							int iRelList1,
							int iRelList2);
	void ConnectObjects(CRVLObject2 *pObject1, 
						CRVLObject2 *pObject2,
					    int iRelList1, 
						int iRelList2,
					    CRVLMPtrChain2 *pRelList);
	CRVLRelation();
	virtual ~CRVLRelation();
};

inline CRVLObject2 *RVLGetObject(BYTE *vppRelation, int i)
{
	return (*((CRVLRelation **)vppRelation))->m_pObject[i];
};

#endif // !defined(AFX_RVLRELATION_H__458B5D54_79D7_40D3_99FA_ADF16FBE0FA5__INCLUDED_)
