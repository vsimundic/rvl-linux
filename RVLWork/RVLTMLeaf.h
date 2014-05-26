#pragma once
#include "rvltm.h"

void RVLDisplayTMInstance(	IplImage *pImage,
							RVLTMR_NODE *pSNode,
							RVLTMR_MODEL_NODE *Model);

class CRVLTMLeaf :
	public CRVLTM
{
public:
	CRVLTMLeaf(void);
	virtual ~CRVLTMLeaf(void);
	void Create(void);
};
