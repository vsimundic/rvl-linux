//#include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "DisplayVector.h"
#include "Figure.h"

using namespace RVL;

Figure::Figure()
{
	name = NULL;
	pMem = NULL;

	QList<DisplayVector> *pVectorList = &vectorList;

	RVLQLIST_INIT(pVectorList);	
}


Figure::~Figure()
{
	RVL_DELETE_ARRAY(name);

	if (pMem)
		delete pMem;

	cvReleaseImage(&pImage);
}

void Figure::Create(int MemSize)
{
	pMem = new CRVLMem;

	pMem->Create(MemSize);
}