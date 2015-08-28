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

	if(m_Kinect.m_Flags & RVLKINECT_DEPTH_IMAGE_FORMAT_100UM)
		m_PSD.m_Flags |= RVLPSD_FLAG_100UM;

	if(m_Flags & RVLSYS_FLAGS_PC)
		m_PSD.m_nFOVExtensions = 1;

	m_PSD.Init();

	if (m_PSD.m_Flags & RVLPSD_SEGMENT_STRM)
		m_Flags |= RVLSYS_FLAGS_SEGMENT_MESH;

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

	pParamData = m_ParamList.AddParam("VS.GraphSegmentation", RVLPARAM_TYPE_FLAG, &m_Flags);
	m_ParamList.AddID(pParamData, "yes", RVLSYS_FLAGS_SEGMENT_GRAPH);
}

void CRVLPCSVS::Display(CRVLPCSGUI *pGUI)
{
	CRVLFigure *pFig = pGUI->m_pFig;

	// clear display

	pFig->Clear();

	// select bitmap to display

	if(m_Flags & RVLSYS_FLAGS_PC)
		m_PSD.DisplayPC(pGUI->m_pInputImage);
	else
	{
		switch (pGUI->m_DisplayBitmap){
		case 0:
			// display the depth image on the display image

			RVLDisplayDisparityMapColor(pGUI->m_pDepthImage, 0, FALSE, pGUI->m_pInputImage, pGUI->m_DepthMapFormat);

			break;
		case 1:
			// display RGB image on the display image

			cvCopy(pGUI->m_pRGBImage, pGUI->m_pInputImage);

			break;
		case 2:
			// display grayscale image on the display image

			cvCvtColor(pGUI->m_pGSImage, pGUI->m_pInputImage, CV_GRAY2RGB);

			break;
		case 3:
			// display segmentation image on the display image

			cvCopy(pGUI->m_pSegmentationImage, pGUI->m_pInputImage);
		}
	}

	RVLZoom(pGUI->m_pInputImage, pGUI->m_pZoomedInputImage, 2);

	// display the mesh or convex sets

	if (pGUI->m_bDisplayMesh)
		RVLDisplay2DRegions(pFig, &(m_AImage.m_C2DRegion.m_ObjectList), m_CameraL.Width, RVLColor(0, 255, 0));

	if (pGUI->m_bDisplayConvexSets)
	{
		RVLSegmentationEdgesFromLabels(&(m_AImage.m_C2DRegion));

		RVLDisplay2DRegions(pFig, &(m_AImage.m_C2DRegion.m_ObjectList), m_CameraL.Width, 
			RVLColor(255, 0, 255), 2, RVLMESH_LINK_FLAG_EDGE, RVLMESH_LINK_FLAG_EDGE);
	}

	if (pGUI->m_bDisplaySelectedObjects)
	{
		RVLResetFlags(&(m_AImage.m_C2DRegion.m_ObjectList), RVLMESH_LINK_FLAG_EDGE);

		RVLSegmentationEdgesFromLabels(&(m_AImage.m_C2DRegion), RVLOBJ2_FLAG_MARKED, RVLOBJ2_FLAG_MARKED);

		RVLDisplay2DRegions(pFig, &(m_AImage.m_C2DRegion.m_ObjectList), m_CameraL.Width, 
			RVLColor(255, 255, 0), 2, RVLMESH_LINK_FLAG_EDGE, RVLMESH_LINK_FLAG_EDGE);
	}			

	pGUI->DisplayVectors(pFig, 0, 0, (double)(pGUI->m_ZoomFactor));

	char str[200];

	if (!pGUI->m_bRecord)
	{
		// display some numerical data

		int iTextLine = 0;

		if (pGUI->m_bKinect)
		{
			if(m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE)
			{
				sprintf(str, "Sample %d", pGUI->m_iONISample);

				cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), 
					&(pFig->m_Font),  cvScalar(255, 0, 0));
			}
		}
		else
			cvPutText(pFig->m_pImage, m_ImageFileName, cvPoint(0, (++iTextLine) * pFig->m_FontSize), 
				&(pFig->m_Font),  cvScalar(255, 0, 0));

		sprintf(str, "Exec. Time = %4.0f ms", pGUI->m_ExecTime);

		cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), 
			&(pFig->m_Font),  cvScalar(255, 0, 0));

		sprintf(str, "TT = %d", ((m_Flags & RVLSYS_FLAGS_PC) || (m_PSD.m_Flags & RVLPSD_FLAG_MM) ? 
			m_PSD.m_MeshTol : m_PSD.m_uvdTol));

		cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));

		sprintf(str, "CT = %d", m_ConvexSegmentThr);

		cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));
	}

	// show the display image

	pGUI->ShowFigure(pFig);
}


