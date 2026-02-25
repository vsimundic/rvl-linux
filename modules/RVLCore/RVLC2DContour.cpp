// RVLC2DContour.cpp: implementation of the CRVLC2DContour class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "RVLCore.h"
#include "RVLC2DContour.h"

//#ifdef _DEBUG
//#undef THIS_FILE
//static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
//#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVLC2DContour::CRVLC2DContour()
{
	m_minnContourPoints = 3;	
	m_LineSegmentThr = 1.5;		// pix
}

CRVLC2DContour::~CRVLC2DContour()
{

}




