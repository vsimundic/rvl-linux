// RVL2DPose.cpp: implementation of the CRVL2DPose class.
//
//////////////////////////////////////////////////////////////////////


#include "RVLCore.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CRVL2DPose::CRVL2DPose()
{
	m_ParamFlags = 0x00000000;
}

CRVL2DPose::~CRVL2DPose()
{

}

void CRVL2DPose::UpdateiU(CRVLCamera *pCamera)
{
	if((m_ParamFlags & (RVL2DPOSE_PARAM_FLAGS_U | RVL2DPOSE_PARAM_FLAGS_IU)) ==
		RVL2DPOSE_PARAM_FLAGS_U)
	{
		RVLU2iU(m_U, pCamera->CenterXNrm, pCamera->CenterYNrm, m_iU);

		m_ParamFlags |= RVL2DPOSE_PARAM_FLAGS_IU;
	}
}
