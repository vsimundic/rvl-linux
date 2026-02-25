// RVLC2DContour.h: interface for the CRVLC2DContour class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RVLC2DCONTOUR_H__612CC349_0878_4BCB_AA8F_B76DBDEA832C__INCLUDED_)
#define AFX_RVLC2DCONTOUR_H__612CC349_0878_4BCB_AA8F_B76DBDEA832C__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "RVLC2D.h"

#define RVLC2DCONTOUR_FLAG_LINEAR			0x00000100

class CRVLC2DContour : public CRVLC2D 
{
public:
	int m_minnContourPoints;
	double m_LineSegmentThr;
	CRVLMPtrChain m_ContourSegmentList; // not currently used !!!!!
	CRVLMPtrChain m_BranchList;

public:			
	CRVLC2DContour();
	virtual ~CRVLC2DContour();
};

#endif // !defined(AFX_RVLC2DCONTOUR_H__612CC349_0878_4BCB_AA8F_B76DBDEA832C__INCLUDED_)
