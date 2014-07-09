//#include "highgui.h"
#include "RVLCore.h"
#include "RVLPCS.h"

CRVLPCSVS::CRVLPCSVS()
{
	m_pDelaunay = NULL;

	m_ConvexSegmentThr = 7;
}

CRVLPCSVS::~CRVLPCSVS()
{
	Clear();

	if(m_pDelaunay)
		delete m_pDelaunay;
}

DWORD CRVLPCSVS::Init(char *CfgFile2Name)
{
	CRVLVisionSystem::Init(CfgFile2Name);

	// create tools

	m_pDelaunay = new CRVLDelaunay;

	//// initialize scene

	//m_Scene.m_pMem0 = &m_Mem0;
	//m_Scene.m_pMem = &m_Mem;
	//m_Scene.m_pMem2 = &m_Mem2;
	//m_Scene.m_pRelList = &(m_AImage.m_RelList);	
	
	//m_Scene.Init();	

	// initialize PSD

	m_PSD.m_pStereoVision = &m_StereoVision;
	m_PSD.m_pAImage = &m_AImage;
	m_PSD.m_pMem = &m_Mem;
	m_PSD.m_pMem2 = &m_Mem2;
	m_PSD.CreateParamList(&m_Mem0);
	m_PSD.m_pTimer = m_pTimer;

	if(CfgFile2Name)
		m_PSD.m_ParamList.LoadParams(CfgFile2Name);

	m_PSD.Init();

	// initialize Delaunay triangulation

	m_pDelaunay->m_Width = m_CameraL.Width;
	m_pDelaunay->m_Height = m_CameraL.Height;
	m_pDelaunay->Init();
	m_PSD.m_pDelaunay = m_pDelaunay;

	return RVL_RES_OK;
}

void CRVLPCSVS::Clear()
{
	CRVLVisionSystem::Clear();

	//m_Scene.Clear();
}

void CRVLPCSVS::CreateParamList()
{
	CRVLVisionSystem::CreateParamList();

	RVLPARAM_DATA *pParamData;

	pParamData = m_ParamList.AddParam("Segmentation.Convex.Thr", RVLPARAM_TYPE_INT, &m_ConvexSegmentThr);

	pParamData = m_ParamList.AddParam("VS.PointCloud", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLSYS_FLAGS_PC);

	pParamData = m_ParamList.AddParam("VS.SegmentToConvexSets", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLSYS_FLAGS_SEGMENT_TO_CONVEX_SETS);
}