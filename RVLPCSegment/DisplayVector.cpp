//#include "stdafx.h"
#include "RVLCore2.h"
#include "DisplayVector.h"

using namespace RVL;

DisplayVector::DisplayVector()
{
	QList<QLIST::Entry<CvPoint>> *pPointList = &pointList;

	RVLQLIST_INIT(pPointList);
}


DisplayVector::~DisplayVector()
{
}
