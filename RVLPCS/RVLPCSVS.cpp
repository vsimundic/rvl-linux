//#include "highgui.h"
//#define RVLPCL
#ifdef RVLPCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#endif
#include "RVLCore.h"
#include "RVLPCS.h"
#include "Include\RVLPCSVS.h"

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
	m_PSD.m_pLidarParams = &m_LidarParams;

	if(CfgFile2Name)
		m_PSD.m_ParamList.LoadParams(CfgFile2Name);

	if(m_Kinect.m_Flags & RVLKINECT_DEPTH_IMAGE_FORMAT_100UM)
		m_PSD.m_Flags |= RVLPSD_FLAG_100UM;

	if(m_Flags & RVLSYS_FLAGS_PC)
		m_PSD.m_nFOVExtensions = 1;

	m_PSD.Init();

	if (m_PSD.m_Flags & RVLPSD_SEGMENT_STRM)
		m_Flags |= RVLSYS_FLAGS_SEGMENT_MESH;

	m_StereoVision.m_DisparityMap.Format = (m_PSD.m_Flags & RVLPSD_FLAG_MM ?
		(m_PSD.m_Flags & RVLPSD_FLAG_100UM ? RVLKINECT_DEPTH_IMAGE_FORMAT_100UM : RVLKINECT_DEPTH_IMAGE_FORMAT_1MM) :
		RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY);

	// initialize Delaunay triangulation

	m_pDelaunay->m_Width = m_CameraL.Width;
	m_pDelaunay->m_Height = m_CameraL.Height;
	m_pDelaunay->Init();
	m_PSD.m_pDelaunay = m_pDelaunay;

#ifdef RVLOPENNI
	// initialize kinect

	bool bKinect = m_Kinect.Init();

	//VS.m_Kinect.RegisterDepthToColor(true);

	//VS.m_Kinect.GetParams();

	//int u, v, z_;
	//float x, y, z;
	//double x__, y__;

	//for(int i = 0; i < 1000; i++)
	//{
	//	u = RVLRandom(0, 639);
	//	v = RVLRandom(0, 479);
	//	z_ = RVLRandom(700, 10000);

	//	VS.m_Kinect.ConvertDepthToWorld(u, v, z_, &x, &y, &z);

	//	x__ = ((double)u - VS.m_Kinect.m_uc) / VS.m_Kinect.m_fu * (double)z_;
	//	y__ = ((double)v - VS.m_Kinect.m_vc) / VS.m_Kinect.m_fv * (double)z_;
	//}

	if (bKinect)
	{
		m_Flags |= RVLSYS_FLAGS_KINECT;
		m_Flags &= ~RVLSYS_FLAGS_PC;
	}	
#endif

	// point cloud

	if (m_Flags & RVLSYS_FLAGS_PC)
		m_PC = new double[3 * m_PSD.m_Width * m_PSD.m_Height];

	/////

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

			RVLDisplayDisparityMapColor(&(m_StereoVision.m_DisparityMap), 0, FALSE, pGUI->m_pInputImage, m_StereoVision.m_DisparityMap.Format);

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

bool CRVLPCSVS::InputRGBDImageFromFile(
	RVLDISPARITYMAP *pDepthImage,
	IplImage *pRGBImage,
	char *RGBExtension,
	char *DepthExtension)
{
	if (strcmp(RVLGETFILEEXTENSION(m_ImageFileName), "pcd") == 0)
	{
		m_Flags |= RVLSYS_FLAGS_PCLPCD;

		m_PSD.m_Flags |= RVLPSD_FLAG_MM;

		return InputRGBDImageFromPCDFile(pDepthImage, pRGBImage);
	}
	else if (CRVLVisionSystem::InputRGBDImageFromFile(pDepthImage, pRGBImage, RGBExtension, DepthExtension))
	{
		if (m_StereoVision.m_DisparityMap.Format == RVLKINECT_DEPTH_IMAGE_FORMAT_100UM)
			m_PSD.m_Flags |= RVLPSD_FLAG_100UM;

		return true;
	}
	else
		return false;
}


void CRVLPCSVS::Segment()
{
	// clear image features

	m_AImage.Clear();

	// compute a 3D point cloud from depth data

	if (m_Flags & RVLSYS_FLAGS_PC)
		m_PSD.GetOrgPC(m_PC, m_nPC);
	else if (!(m_Flags & RVLSYS_FLAGS_PCLPCD))
		m_PSD.GetPointsWithDisparity(&(m_StereoVision.m_DisparityMap));

	int w = m_StereoVision.m_DisparityMap.Width;
	int h = m_StereoVision.m_DisparityMap.Height;
	
	if (m_Flags & RVLSYS_FLAGS_SEGMENT_MESH)
	{
		// create a triangular mesh from the point cloud

		m_PSD.Segment(&(m_AImage.m_C2DRegion), &(m_AImage.m_C2DRegion2), &(m_AImage.m_C2DRegion3), &(m_Mem));

		// assign labels to segments

		if (m_PSD.m_Flags & RVLPSD_MESH_SEGMENT_PLANAR)
		{
			m_nObjects = m_AImage.m_C2DRegion3.m_ObjectList.m_nElements + 1;

			m_PSD.AssignLabels(&(m_AImage.m_C2DRegion), &(m_AImage.m_C2DRegion3));
		}

		// segment the triangular mesh to convex sets

		if (m_Flags & RVLSYS_FLAGS_SEGMENT_TO_CONVEX_SETS)
			m_nObjects = RVLSegmentToConvex(&(m_AImage.m_C2DRegion), NULL, &(m_AImage.m_C2DRegion2),
			m_ConvexSegmentThr, w, h, m_PSD.m_Point3DMap, &(m_Mem), NULL, NULL,
			(m_PSD.m_Flags & RVLPSD_FLAG_MM) != 0);
	}
}

bool CRVLPCSVS::InputRGBDImageFromPCDFile(
	RVLDISPARITYMAP *pDepthImage,
	IplImage *pRGBImage)
{
#ifdef RVLPCL
	pcl::PointCloud<pcl::PointXYZRGB> PC;

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(m_ImageFileName, PC) == -1)
		return false;

	int step;

	if (PC.width == pDepthImage->Width && PC.height == pDepthImage->Height)
		step = 1;
	else if (PC.width == 2 * pDepthImage->Width && PC.height == 2 * pDepthImage->Height)
		step = 2;
	else
		return false;

	short *pDepth = pDepthImage->Disparity;

	char *pRGB = pRGBImage->imageData;

	RVL3DPOINT2 **ppP3DMap = m_PSD.m_Point3DMap;

	RVL3DPOINT2 *pPoint3D = m_PSD.m_Point3DArray;

	float maxz = 0.001f * (float)(m_StereoVision.m_maxz);

	int iPt = 0;

	int u, v;
	pcl::PointXYZRGB Pt;
	double *X;
	int *iX;

	for (v = 0; v < pDepthImage->Height; v++)
	{
		for (u = 0; u < pDepthImage->Width; u++, ppP3DMap++, pDepth++, iPt += step)
		{
			Pt = PC.points[iPt];

			*(pRGB++) = Pt.r;
			*(pRGB++) = Pt.g;
			*(pRGB++) = Pt.b;

			if (isfinite(Pt.z))
			{
				if (Pt.z > maxz)
				{
					*ppP3DMap = NULL;

					continue;
				}
			}
			else
			{
				*pDepth = 0;

				*ppP3DMap = NULL;

				continue;
			}

			X = pPoint3D->XYZ;

			X[0] = 1000.0 * (double)(Pt.x);
			X[1] = 1000.0 * (double)(Pt.y);
			X[2] = 1000.0 * (double)(Pt.z);

			iX = pPoint3D->iX;

			iX[0] = DOUBLE2INT(X[0]);
			iX[1] = DOUBLE2INT(X[1]);
			iX[2] = DOUBLE2INT(X[2]);

			pPoint3D->x = X[0];
			pPoint3D->y = X[1];
			pPoint3D->z = X[2];

			pPoint3D->u = u;
			pPoint3D->v = v;
			pPoint3D->d = iX[2];
			pPoint3D->iPix = iPt;
			pPoint3D->segmentNumber = -1;
			pPoint3D->refSegmentNumber = -1;
			pPoint3D->iCell = -1;
			pPoint3D->regionList = NULL;

			*pDepth = iX[2];

			*ppP3DMap = pPoint3D;

			pPoint3D++;
		}

		iPt += ((step * pDepthImage->Width) * (step - 1));
	}

	m_PSD.m_n3DPoints = pPoint3D - m_PSD.m_Point3DArray;

	/////

	return true;
#else
	return false;
#endif
}


bool CRVLPCSVS::InputFromFile(
	RVLDISPARITYMAP *pDepthImage, 
	IplImage *pRGBImage)
{
	if (m_Flags & RVLSYS_FLAGS_PC)
		return RVLPCImport(m_ImageFileName, &m_PC, m_nPC, m_LidarParams.scale);
	else if (pDepthImage != NULL && pRGBImage != NULL)
		return InputRGBDImageFromFile(pDepthImage, pRGBImage, "-LW.bmp", "-D.txt");
	else
		return false;
}
