// RVL3DObject3.cpp: implementation of the CRVL3DObject3 class.
//
//////////////////////////////////////////////////////////////////////


#include "RVLCore.h"
#include "RVLClass.h"
//#include "RVLC3D.h"
//#include "RVLC2DSIFT.h"
//#include "RVL2DObject3.h"
#include "RVL3DObject.h"
#include "RVLRelation.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL3DObject::CRVL3DObject()
{

}

CRVL3DObject::~CRVL3DObject()
{

}

void CRVL3DObject::UncertTransf(CRVL3DPose *pPose)
{

}
	
void CRVL3DObject::Transf(CRVL3DPose *pPose)
{

}

BOOL CRVL3DObject::Match(CRVL3DObject *pSObject, 
						  CRVL3DPose *pPose,
						  double &MatchQuality)
{
	return FALSE;
}

void CRVL3DObject::UpdateX(CRVL3DObject *pSObjectIn,
							CRVL3DPose *pPoseSM)
{

}

double * CRVL3DObject::GetX()
{
	return NULL;
}

double * CRVL3DObject::GetCX()
{
	return NULL;
}

void CRVL3DObject::UpdateParams()
{

}

//CRVL2DObject3 * CRVL3DObject::GetProjection()
//{
//	return NULL;
//}
