// RVL2DObject.h: interface for the CRVL2DObject2 class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVL2DOBJECT2_H__17863528_5EC7_4186_B55D_C029BC2AFAAF__INCLUDED_)
#define AFX_RVL2DOBJECT2_H__17863528_5EC7_4186_B55D_C029BC2AFAAF__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVLObject2.h"

void RVLCreateC2D(CRVLClass *pClass);

class CRVL2DObject : public CRVLObject2  
{
public:
	virtual CRVLObject2 * Create2(CRVLClass *pClass);
	//virtual double MatchDescriptor(CRVLObject2 *pMObject);
	virtual BOOL Match(CRVLObject2 *pObject, 
					   int &MatchCost, void *vpMatchData = NULL);
	virtual int Match(CRVLMPtrChain *pSObjectArray,
					  CRVLCamera *pCamera, 
					  CRVL3DPose *pPoseLA, 
					  CRVL3DPose *pPoseA0, 
					  CRVLMPtrChain *pObjectBuff);
	void UpdateParams();
	CRVL2DObject();
	virtual ~CRVL2DObject();
};

#endif // !defined(AFX_RVL2DOBJECT2_H__17863528_5EC7_4186_B55D_C029BC2AFAAF__INCLUDED_)
