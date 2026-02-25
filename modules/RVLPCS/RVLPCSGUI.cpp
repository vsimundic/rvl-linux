#include "Platform.h"
//#ifdef RVLOPENNI
//#include "OpenNI.h"
//#endif
#include "RVLCore.h"
#include "RVLPCS.h"


CRVLPCSGUI::CRVLPCSGUI()
{
}


CRVLPCSGUI::~CRVLPCSGUI()
{
	Clear();
}

void CRVLPCSGUI::Init(
	void *vpVS)
{
	m_vpVS = vpVS;

	CRVLPCSVS *pVS = (CRVLPCSVS *)vpVS;

	m_pMem0 = &(pVS->m_Mem0);
	m_pMem = &(pVS->m_Mem);

	CRVLGUI::Init();	

	m_w = pVS->m_PSD.m_Width;
	m_h = pVS->m_PSD.m_Height;

#ifdef RVLVTK
	// create VTK renderer

	m_Renderer.Init(800, 600);
	m_Renderer.m_pWindow->Render();

	int *pointmap = new int[m_w * m_h];
#endif

	// create RGB image

	m_pRGBImage = cvCreateImage(cvSize(m_w, m_h), IPL_DEPTH_8U, 3);

	// create HSV image

	m_pHSVImage = cvCreateImage(cvSize(m_w, m_h), IPL_DEPTH_8U, 3);

	// create grayscale image

	m_pGSImage = cvCreateImage(cvSize(m_w, m_h), IPL_DEPTH_8U, 1);

	// create segmentation image

	m_pSegmentationImage = cvCreateImage(cvSize(m_w, m_h), IPL_DEPTH_8U, 3);

	// create a display image

	m_pFig = OpenFigure("RVLPCSGUI");

	m_pInputImage = cvCreateImage(cvSize(m_w, m_h), IPL_DEPTH_8U, 3);
	m_pZoomedInputImage = cvCreateImage(cvSize(2 * m_w, 2 * m_h), IPL_DEPTH_8U, 3);

	m_pFig->m_pImage = m_pInputImage;

	m_pFig->m_FontSize = 16;
	cvInitFont(&(m_pFig->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2);

	// initialize class object for VTK display

	m_Class.m_pMem = &(pVS->m_Mem);
	m_Class.m_pMem0 = &(pVS->m_Mem0);
	m_Class.m_pMem2 = &(pVS->m_Mem2);

	// initialize point map for VTK display

	m_PointMap = new int[m_w * m_h];

	// initialize display parameters

	m_bKinect = ((pVS->m_Flags & RVLSYS_FLAGS_KINECT) != 0);

	m_ImageWidth = m_w;

	m_bDisplayMesh = true;
	m_bDisplayConvexSets = true;
	m_bDisplaySelectedObjects = true;
	//m_bContinuous = bKinect;
	m_bContinuous = false;
	m_bRecord = false;
	m_bNextImage = true;

	m_iONISample = 0;
	m_ZoomFactor = 1;
	m_DisplayBitmap = (pVS->m_Flags & RVLSYS_FLAGS_SEGMENT_GRAPH ? 3 : 0);

	m_ONISpeed = 1;
}

bool CRVLPCSGUI::InteractiveVisualization()
{
	CRVLPCSVS *pVS = (CRVLPCSVS *)m_vpVS;

	RVLDISPARITYMAP *pDepthImage = &(pVS->m_StereoVision.m_DisparityMap);

#ifdef RVLOPENNI
	if (m_bKinect && m_bRecord)
		pDepthImage->Format = RVLKINECT_DEPTH_IMAGE_FORMAT_1MM;
#endif

	// load information about selected segments 

	if (pVS->m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE)
	{
		char *SelectedSegmentsFileName = RVLKinectCreateONISampleFileName(pVS->m_Kinect.m_ONIFileName, m_iONISample, "-SS.txt");

		RVLSegmentationLoadSelection(&(pVS->m_AImage.m_C2DRegion.m_ObjectList), pVS->m_nObjects, RVLOBJ2_FLAG_MARKED, SelectedSegmentsFileName);

		delete[] SelectedSegmentsFileName;
	}

	// main loop

	cvCvtColor(m_pRGBImage, m_pHSVImage, CV_BGR2RGB);

	m_pHSVImage->channelSeq[0] = 'R';
	m_pHSVImage->channelSeq[1] = 'G';
	m_pHSVImage->channelSeq[2] = 'B';

	CRVL3DMeshObject *objects = GenMeshObjects(&(pVS->m_AImage.m_C2DRegion.m_ObjectList), m_pHSVImage, pVS->m_nObjects, &m_Class);

	bool bVTKRendererActive = false;
	int VTKTexture = 0;
	char VTK3DModelFileName[] = "VTK3DModel_00000.obj";
	char VTKMessageConst[] = "3D model in OBJ-format saved in ";
	char *VTKMessage = new char[strlen(VTKMessageConst) + strlen(VTK3DModelFileName) + 1];

	int key;
	int iPrevONISample;	
	bool bRefresh;
	bool bNextImageSelected;
	char *VTKTextureFileName;

	do
	{
#ifdef NEVER
		// clear display

		pFig->Clear();

		// select bitmap to display

		if (pVS->m_Flags & RVLSYS_FLAGS_PC)
			pVS->m_PSD.DisplayPC(pInputImage);
		else
		{
			switch (DisplayBitmap){
			case 0:
				// display the depth image on the display image

				RVLDisplayDisparityMapColor(pDepthImage, 0, FALSE, pInputImage, DepthMapFormat);

				break;
			case 1:
				// display RGB image on the display image

				cvCopy(pRGBImage, pInputImage);

				break;
			case 2:
				// display grayscale image on the display image

				cvCvtColor(pGSImage, pInputImage, CV_GRAY2RGB);
			}
		}

		RVLZoom(pInputImage, pZoomedInputImage, 2);

		// display the mesh or convex sets

		if (bDisplayMesh)
			RVLDisplay2DRegions(pFig, &(pVS->m_AImage.m_C2DRegion.m_ObjectList), pVS->m_CameraL.Width, RVLColor(0, 255, 0));

		if (bDisplayConvexSets)
		{
			RVLSegmentationEdgesFromLabels(&(pVS->m_AImage.m_C2DRegion));

			RVLDisplay2DRegions(pFig, &(pVS->m_AImage.m_C2DRegion.m_ObjectList), pVS->m_CameraL.Width,
				RVLColor(255, 0, 255), 2, RVLMESH_LINK_FLAG_EDGE, RVLMESH_LINK_FLAG_EDGE);
		}

		if (bDisplaySelectedObjects)
		{
			RVLResetFlags(&(pVS->m_AImage.m_C2DRegion.m_ObjectList), RVLMESH_LINK_FLAG_EDGE);

			RVLSegmentationEdgesFromLabels(&(pVS->m_AImage.m_C2DRegion), RVLOBJ2_FLAG_MARKED, RVLOBJ2_FLAG_MARKED);

			RVLDisplay2DRegions(pFig, &(pVS->m_AImage.m_C2DRegion.m_ObjectList), pVS->m_CameraL.Width,
				RVLColor(255, 255, 0), 2, RVLMESH_LINK_FLAG_EDGE, RVLMESH_LINK_FLAG_EDGE);
		}

		GUI.DisplayVectors(pFig, 0, 0, (double)ZoomFactor);

		if (!bRecord)
		{
			// display some numerical data

			iTextLine = 0;

			if (bKinect)
			{
				if (pVS->m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE)
				{
					sprintf(str, "Sample %d", iONISample);

					cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font, cvScalar(255, 0, 0));
				}
			}
			else
				cvPutText(pFig->m_pImage, pVS->m_ImageFileName, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font, cvScalar(255, 0, 0));

			sprintf(str, "Exec. Time = %4.0f ms", 1000.0f * ((float)t) / CLOCKS_PER_SEC);

			cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font, cvScalar(255, 0, 0));

			sprintf(str, "Exec. Time = %4.0f ms", 1000.0f * ((float)t) / CLOCKS_PER_SEC);

			cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font, cvScalar(255, 0, 0));

			sprintf(str, "TT = %d", ((pVS->m_Flags & RVLSYS_FLAGS_PC) || (pVS->m_PSD.m_Flags & RVLPSD_FLAG_MM) ?
				pVS->m_PSD.m_MeshTol : pVS->m_PSD.m_uvdTol));

			cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font, cvScalar(255, 0, 0));

			sprintf(str, "CT = %d", pVS->m_ConvexSegmentThr);

			cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font, cvScalar(255, 0, 0));
		}

		// show the display image

		GUI.ShowFigure(pFig);
#endif
		pVS->Display(this);

		cvSetMouseCallback(m_pFig->m_ImageName, RVLPCSDisplayMouseCallback, this);

		//cvSaveImage("C:\\RVL\\ExpRez\\RVLDisplay.bmp", pDisplay);

		// wait until a key is pressed

		key = (m_bContinuous ? cvWaitKey(1) : cvWaitKey());

		// change the display according to the key pressed

		iPrevONISample = m_iONISample;

		m_bNextImage = true;
		bRefresh = false;
		bNextImageSelected = false;
		int VTKTexture_[] = { 0, 2, 1, 0 };

		switch (key){
		case 'm':
			m_bDisplayMesh = (!m_bDisplayMesh && !m_bRecord);

			bRefresh = true;

			break;
		case 's':
			m_bDisplayConvexSets = (!m_bDisplayConvexSets && !m_bRecord);

			bRefresh = true;

			break;
		case 'z':
			if (m_ZoomFactor == 1)
			{
				m_ZoomFactor = 2;
				m_pFig->m_pImage = m_pZoomedInputImage;
			}
			else
			{
				m_ZoomFactor = 1;
				m_pFig->m_pImage = m_pInputImage;
			}

			bRefresh = true;

			break;
		case 'c':
			m_bContinuous = !m_bContinuous;

			break;
		case 'b':
			m_DisplayBitmap = (m_DisplayBitmap + 1) % (pVS->m_Flags & RVLSYS_FLAGS_SEGMENT_GRAPH ? 4 : 3);

//#ifdef RVLOPENNI
//			pVS->m_Kinect.RegisterDepthToColor((m_DisplayBitmap != 0));
//#endif

			bRefresh = true;

			break;
		case 'r':
			if (pVS->m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE)
			{
				RVLSaveDepthImage(pDepthImage->Disparity, m_w, m_h, pVS->m_ImageFileName, pDepthImage->Format, pDepthImage->Format);

				int iSample = RVLGetFileNumber(pVS->m_ImageFileName, "00000-D.txt");

				RVLSetFileNumber(pVS->m_ImageFileName, "00000-D.txt", iSample + 1);

				char *RGBFileName = RVLCreateFileName(pVS->m_ImageFileName, "-D.txt", iSample + 1, "-LW.bmp");

				cvSaveImage(RGBFileName, m_pRGBImage);

				delete[] RGBFileName;

				bRefresh = true;
			}
			else
			{
				m_bRecord = (!m_bRecord && m_bKinect);

				m_bContinuous = false;

				m_DisplayBitmap = 0;

				m_bDisplayMesh = false;

				m_bDisplayConvexSets = false;
			}

			break;
#ifdef RVLVTK
		case 'v':
			RVLDisplaySegmentedMesh3D(&m_Renderer, &(pVS->m_AImage.m_C2DRegion.m_ObjectList), pVS->m_nObjects, m_w, m_h, pVS->m_PSD.m_nFOVExtensions, m_PointMap,
				pVS->m_PSD.m_Point3DMap, VTKTexture, m_pHSVImage);

			bRefresh = true;
			bVTKRendererActive = true;

			break;
#endif
		case 't':
#ifdef RVLVTK
			VTKTexture = (VTKTexture + 1) % 2;

			if (bVTKRendererActive)
				RVLDisplaySegmentedMesh3D(&m_Renderer, &(pVS->m_AImage.m_C2DRegion.m_ObjectList), pVS->m_nObjects, m_w, m_h, pVS->m_PSD.m_nFOVExtensions, m_PointMap,
				pVS->m_PSD.m_Point3DMap, VTKTexture, m_pHSVImage);
			else
#endif
				Message("VTK Texture mode changed.", 600, 100, cvScalar(0, 128, 255));

			bRefresh = true;

			break;

		case 'p':
			//if (bVTKRendererActive)
			//{
			//	RVLSetFileNumber(VTK3DModelFileName, "00000.ply", iVTK3DModel);

			//	Renderer.Save2PLY(VTK3DModelFileName);

			//	//iVTK3DModel++;

			//}

			FILE *dat, *mtldat;

			dat = fopen(VTK3DModelFileName, "w");

			VTKTextureFileName = RVLCreateFileName(VTK3DModelFileName, ".obj", 0, ".obj.mtl");

			mtldat = fopen(VTKTextureFileName, "w");

			cvSaveImage("Texture.bmp", m_pRGBImage);

			objects->SaveMeshObject2OBJ(dat, mtldat, VTKTextureFileName, pVS->m_PSD.m_Point3DMap, VTKTexture_[VTKTexture],
				"Texture.bmp");  //po dominantnom binu

			delete[] VTKTextureFileName;

			fclose(dat);

			fclose(mtldat);

			//primjer za teksture

			//pRootMeshObject->SaveMeshObject2OBJ(objf, mtlf, "test.obj.mtl", m_PSD.m_Point3DMap, 2, "sl-00000-LW.bmp");

			strcpy(VTKMessage, VTKMessageConst);
			strcat(VTKMessage, VTK3DModelFileName);

			Message(VTKMessage, 600, 100, cvScalar(0, 128, 255));

			bRefresh = true;

			break;
		case 0x00000008:	// Backspace
			m_iONISample -= m_ONISpeed;

			bNextImageSelected = true;

			break;
		case 0x00210000:	// PgUp
			if (m_bKinect && (pVS->m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE))
			{
				if (m_ONISpeed < 100)
					m_ONISpeed *= 10;
			}

			bRefresh = true;

			break;
		case 0x00220000:	// PgDn
			if (m_bKinect && (pVS->m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE))
			{
				if (m_ONISpeed > 1)
				{
					m_ONISpeed /= 10;

					if (m_ONISpeed < 1)
						m_ONISpeed = 1;
				}
			}

			bRefresh = true;

			break;
		case 0x00230000:	// End
#ifdef RVLOPENNI
			if (m_bKinect && (pVS->m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE))
			{
				m_iONISample = pVS->m_Kinect.GetNoONIFrames() - 1;

				bNextImageSelected = true;
			}
#endif

			break;
		case 0x00240000:	// Home
			if (m_bKinect && (pVS->m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE))
			{
				m_iONISample = 0;

				bNextImageSelected = true;
			}

			break;
		case 0x00250000:
			if (pVS->m_ConvexSegmentThr > 0)
				pVS->m_ConvexSegmentThr--;

			m_bNextImage = false;

			break;
		case 0x00260000:
			if ((pVS->m_Flags & RVLSYS_FLAGS_PC) || (pVS->m_PSD.m_Flags & RVLPSD_FLAG_MM))
				pVS->m_PSD.m_MeshTol++;
			else
				pVS->m_PSD.m_uvdTol++;

			m_bNextImage = false;

			break;
		case 0x00270000:
			pVS->m_ConvexSegmentThr++;

			m_bNextImage = false;

			break;
		case 0x00280000:
			if ((pVS->m_Flags & RVLSYS_FLAGS_PC) || (pVS->m_PSD.m_Flags & RVLPSD_FLAG_MM))
			{
				if (pVS->m_PSD.m_MeshTol > 1)
					pVS->m_PSD.m_MeshTol--;
			}
			else
			{
				if (pVS->m_PSD.m_uvdTol > 1)
					pVS->m_PSD.m_uvdTol--;
			}

			m_bNextImage = false;
		}
	} while (bRefresh && !m_bContinuous);

#ifdef RVLOPENNI
	if (m_bKinect && m_bRecord)
		pDepthImage->Format = RVLKINECT_DEPTH_IMAGE_FORMAT_1MM;
#endif

	if (m_bNextImage)
	{
		// save information about selected segments

		if (pVS->m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE)
		{
			char *SelectedSegmentsFileName = RVLKinectCreateONISampleFileName(pVS->m_Kinect.m_ONIFileName, iPrevONISample, "-SS.txt");

			RVLSegmentationSaveSelection(&(pVS->m_AImage.m_C2DRegion.m_ObjectList), pVS->m_nObjects, RVLOBJ2_FLAG_MARKED, SelectedSegmentsFileName);

			delete[] SelectedSegmentsFileName;

			
		}

		// get new sample name/ID

		if (m_bKinect)
		{
			if ((pVS->m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE) && !bNextImageSelected)
				m_iONISample += m_ONISpeed;
		}
	}

	delete[] VTKMessage;

	return (key != 27);
}

void RVLPCSDisplayMouseCallback(int event, int x, int y, int flags, void* vpData)
{
	CRVLPCSGUI *pGUI = (CRVLPCSGUI *)vpData;

	CRVLPCSVS *pVS = (CRVLPCSVS *)(pGUI->m_vpVS);

	bool bDraw = false;

	switch (event){
	case CV_EVENT_LBUTTONDOWN:
		int iPix = x / pGUI->m_ZoomFactor + y / pGUI->m_ZoomFactor * pGUI->m_ImageWidth;

		CRVL2DRegion2 *pSelectedTriangle = pVS->m_PSD.m_2DRegionMap[iPix];

		if (pSelectedTriangle)
		{
			CRVLMPtrChain *pTriangleList = &(pVS->m_AImage.m_C2DRegion.m_ObjectList);

			//RVLResetFlags<CRVL2DRegion2>(pTriangleList, RVLOBJ2_FLAG_MARKED);

			CRVL2DRegion2 *pTriangle;

			pTriangleList->Start();

			while (pTriangleList->m_pNext)
			{
				pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

				if (pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
					continue;

				if (pTriangle->m_Label == pSelectedTriangle->m_Label)
					pTriangle->m_Flags ^= RVLOBJ2_FLAG_MARKED;
			}

			bDraw = true;
		}
	}

	if (bDraw)
		pVS->Display(pGUI);
}

void CRVLPCSGUI::Clear()
{
	CloseFigure("RVLPCSdemo");

	cvReleaseImage(&m_pInputImage);
	cvReleaseImage(&m_pRGBImage);
	cvReleaseImage(&m_pGSImage);
	cvReleaseImage(&m_pSegmentationImage);
	cvReleaseImage(&m_pZoomedInputImage);
}