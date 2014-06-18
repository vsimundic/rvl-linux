// RVLPSuLMdemo.cpp : Defines the entry point for the console application.
//

#include "highgui.h"
#include <stdio.h>
#include <time.h>
#include "RVLCore.h"
#include "RVLPCS.h"
#include "RVLRLM.h"
#include "RVLPSuLMBuilder.h"
#include "RVLPSuLMVS.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#endif

struct RVLPSULMDEMO_MOUSE_CALLBACK_DATA
{
	int u, v;
	bool bSelection;
	int w;
	CRVLFigure *pFig;
	CRVLPSuLMVS *pVS;
	int ZoomFactor;
};

void MessageCanNotOpenFile(CRVLGUI *pGUI, char *FileName);
void MouseCallback(int event, int x, int y, int flags, void* pData);

int main(int argc, char* argv[])
{
	// create vision system

	CRVLPSuLMVS VS;

	// initialize vision system

	VS.CreateParamList();

	VS.Init("RVLPSuLMdemo.cfg");

	// create GUI

	CRVLGUI GUI;

	GUI.m_pMem0 = &(VS.m_Mem0);
	GUI.m_pMem = &(VS.m_Mem);

#ifdef RVLPSD_SEGMENT_STRM_DEBUG
	VS.m_PSD.m_DebugData.pGUI = &GUI;
#endif

	GUI.Init();

#ifdef RVLOPENNI
	// initialize kinect

	bool bKinect = VS.m_Kinect.Init();

	if(bKinect)
		VS.m_Flags &= ~RVLSYS_FLAGS_PC;
	else
		GUI.Message("Kinect is not available.", 400, 100, cvScalar(0, 128, 255));
#else
	bool bKinect = false;
#endif

	// get the pointer to the depth image

	RVLDISPARITYMAP *pDepthImage;
	int w;
	int h;
	double *PC;
	int nPC;

	if(VS.m_Flags & RVLSYS_FLAGS_PC)
	{
		w = VS.m_PSD.m_Width;
		h = VS.m_PSD.m_Height;

		PC = new double[3 * w * h];
	}
	else
	{
		pDepthImage = &(VS.m_StereoVision.m_DisparityMap);

		w = pDepthImage->Width;
		h = pDepthImage->Height;
	}

#ifdef RVLVTK
	// create VTK renderer

	CRVLVTKRenderer Renderer;

	Renderer.Init(800, 600);

	int *pointmap = new int[w * h];
#endif

	// create RGB image

	IplImage *pRGBImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	// create grayscale image

	IplImage *pGSImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);

	// create a display image

	CRVLFigure *pFig = GUI.OpenFigure("RVLPSuLMdemo");

	IplImage *pInputImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
	IplImage *pZoomedInputImage = cvCreateImage(cvSize(2 * w, 2 * h), IPL_DEPTH_8U, 3);

	pFig->m_pImage = pInputImage;

	pFig->m_FontSize = 16;
	cvInitFont(&(pFig->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2);

	pFig->m_PoseC0.Reset();

	RVLPSULMDEMO_MOUSE_CALLBACK_DATA MouseCallbackData;	

	MouseCallbackData.w = w;
	MouseCallbackData.pFig = pFig;
	MouseCallbackData.pVS = &VS;

	// allocate memory

	int *SizeArray = new int[2 * w * h];

	// main loop

	CRVL3DPose NullPose;

	RVLNULL3VECTOR(NullPose.m_X);
	RVLUNITMX3(NullPose.m_Rot);

	bool bDisplayMesh = false;
	bool bDisplayConvexSets = false;
	bool bDisplayHypothesis = true;
	bool bDisplayPSuLM = false;
	//bool bContinuous = bKinect;
	bool bContinuous = false;
	bool bRecord = false;
	int DisplayBitmap = 0;
	int ZoomFactor = 1;
	bool bVTKRendererActive = false;
	int iVTK3DModel = 0;

	char VTK3DModelFileName[] = "VTK3DModel_00000.ply";
	char VTKMessageConst[] = "3D model in PLY-format saved in ";
	char *VTKMessage = new char[strlen(VTKMessageConst) + strlen(VTK3DModelFileName) + 1];
	
	int key;
	//int iSample;
	int nObjects = 1;
	bool bRefresh;
	bool bNextImage;
	clock_t t;
	char str[200];
	int iTextLine;
	unsigned int DepthMapFormat;

	do
	{
		DepthMapFormat = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY;

#ifdef RVLOPENNI
		if(bKinect)
		{
			// acquire depth image from Kinect

			if(bRecord)
				DepthMapFormat = RVLKINECT_DEPTH_IMAGE_FORMAT_1MM;

			VS.m_Kinect.GetImages(pDepthImage->Disparity, pRGBImage, NULL, pGSImage, DepthMapFormat);
		}
		else
#endif

		if(bRecord)
		{
			if(VS.m_Flags & RVLSYS_FLAGS_PC)
			{
				if(!RVLPCImport(VS.m_ImageFileName, PC, nPC))
					MessageCanNotOpenFile(&GUI, VS.m_ImageFileName);
				else
				{
					int iSample = RVLGetFileNumber(VS.m_ImageFileName, "00000-PC.pcd");

					char *PCFileName = RVLCreateFileName(VS.m_ImageFileName, "-PC.pcd", iSample, "-PC.obj");

					RVLPCSaveToObj(PC, nPC, PCFileName);

					delete[] PCFileName;
				}
			}
			else
			{
				if(!RVLImportDisparityImage(VS.m_ImageFileName, pDepthImage, VS.m_Kinect.m_zToDepthLookupTable))
					MessageCanNotOpenFile(&GUI, VS.m_ImageFileName);
				else
				{
					RVLSaveDepthImage(pDepthImage->Disparity, w, h, VS.m_ImageFileName, RVLKINECT_DEPTH_IMAGE_FORMAT_1MM, 
						RVLKINECT_DEPTH_IMAGE_FORMAT_1MM);

					int iSample = RVLGetFileNumber(VS.m_ImageFileName, "00000-D.txt");
				
					RVLSetFileNumber(VS.m_ImageFileName, "00000-D.txt", iSample + 1);
				}
			}
		}
		else
		{
			t = clock();			

			//// clear image features

			//VS.m_AImage.Clear();

			//// compute a 3D point cloud from depth data

			//if(VS.m_Flags & RVLSYS_FLAGS_PC)
			//	VS.m_PSD.GetOrgPC(PC, nPC);
			//else
			//	VS.m_PSD.GetPointsWithDisparity(pDepthImage);

			//// create a triangular mesh from the point cloud

			//VS.m_PSD.Segment(&(VS.m_AImage.m_C2DRegion),&(VS.m_AImage.m_C2DRegion2),&(VS.m_AImage.m_C2DRegion3),&(VS.m_Mem));

			//// segment to convex sets
		
			//if(VS.m_Flags & RVLSYS_FLAGS_SEGMENT_TO_CONVEX_SETS)
			//	nObjects = RVLSegmentToConvex(&(VS.m_AImage.m_C2DRegion), NULL, &(VS.m_AImage.m_C2DRegion2),
			//		VS.m_ConvexSegmentThr, w, h, VS.m_PSD.m_Point3DMap, &(VS.m_Mem), NULL, NULL,
			//		(VS.m_PSD.m_Flags & RVLPSD_FLAG_MM) != 0);

			// debug

			//VS.m_PSuLMBuilder.m_pNearestModelPSuLM = VS.m_PSuLMBuilder.GetPSuLM(8);			

			/////

			VS.m_PSuLMBuilder.m_Flags |= RVLPSULMBUILDER_FLAG_KIDNAPPED;

			VS.Update();

			t = clock() - t;			

			// mark segment edges

			if(VS.m_PSD.m_Flags & RVLPSD_MESH_SEGMENT_PLANAR)
			{
				nObjects = VS.m_AImage.m_C2DRegion3.m_ObjectList.m_nElements + 1;

				VS.m_PSD.AssignLabels(&(VS.m_AImage.m_C2DRegion), &(VS.m_AImage.m_C2DRegion3));
			}

			RVLSegmentationEdgesFromLabels(&(VS.m_AImage.m_C2DRegion));
		}

		// display the results

		do
		{
			// clear display

			pFig->Clear();

			// select bitmap to display

			if(VS.m_Flags & RVLSYS_FLAGS_PC)
				VS.m_PSD.DisplayPC(pInputImage);
			else
			{
				switch(DisplayBitmap){
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

			if(bDisplayMesh)
				RVLDisplay2DRegions(pFig, &(VS.m_AImage.m_C2DRegion.m_ObjectList), VS.m_CameraL.Width, RVLColor(0, 255, 0));

			if(bDisplayConvexSets)
				RVLDisplay2DRegions(pFig, &(VS.m_AImage.m_C2DRegion.m_ObjectList), VS.m_CameraL.Width, 
					RVLColor(255, 0, 255), 1, RVLMESH_LINK_FLAG_EDGE, RVLMESH_LINK_FLAG_EDGE);

			if(bDisplayHypothesis)
			{
				VS.m_PSuLMBuilder.DisplayHypothesis(&GUI, pFig, VS.m_pPSuLM);

				VS.m_PSuLMBuilder.DisplayHypothesisData(pFig);
			}

			if(bDisplayPSuLM)
				VS.m_pPSuLM->Display(pFig, &NullPose, cvScalar(0, 255, 0), 
					RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_ELLIPSES | RVLPSULM_DISPLAY_LINES | RVLPSULM_DISPLAY_VECTORS);
					//RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_VECTORS);

			GUI.DisplayVectors(pFig, 0, 0, (double)ZoomFactor);

			if(!bRecord)
			{
				// display some numerical data

				iTextLine = 0;

				sprintf(str, "Exec. Time = %4.0f ms", 1000.0f * ((float)t)/CLOCKS_PER_SEC);

				cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));

				sprintf(str, "TT = %d", (VS.m_Flags & RVLSYS_FLAGS_PC ? VS.m_PSD.m_MeshTol : VS.m_PSD.m_uvdTol));

				cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));

				sprintf(str, "CT = %d", VS.m_ConvexSegmentThr);

				cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));
			}

			// show the display image

			MouseCallbackData.ZoomFactor = ZoomFactor;

			GUI.ShowFigure(pFig);	

			cvSetMouseCallback("RVLPSuLMdemo", MouseCallback, &MouseCallbackData);

			//cvSaveImage("C:\\RVL\\ExpRez\\RVLDisplay.bmp", pDisplay);

			// wait until a key is pressed

			key = (bContinuous ? cvWaitKey(1) : cvWaitKey());

			// change the display according to the key pressed

			bNextImage = true;
			bRefresh = false;

			switch(key){
			case 'm':
				bDisplayMesh = (!bDisplayMesh && !bRecord);

				bRefresh = true;

				break;
			case 's':
				bDisplayConvexSets = (!bDisplayConvexSets && !bRecord);

				bRefresh = true;

				break;
			case 'h':
				bDisplayHypothesis = (!bDisplayHypothesis && !bRecord);

				bDisplayPSuLM = (bDisplayPSuLM & !bDisplayHypothesis);

				bRefresh = true;

				break;
			case 'u':
				bDisplayPSuLM = (!bDisplayPSuLM && !bRecord);

				bDisplayHypothesis = (bDisplayHypothesis & !bDisplayPSuLM);

				bRefresh = true;

				break;
			case 'z':
				if(ZoomFactor == 1)
				{
					ZoomFactor = 2;
					pFig->m_pImage = pZoomedInputImage;
				}
				else
				{
					ZoomFactor = 1;
					pFig->m_pImage = pInputImage;
				}

				bRefresh = true;

				break;
			case 'c':
				bContinuous = !bContinuous;

				break;
#ifdef RVLOPENNI
			case 'b':
				if(bKinect)
					DisplayBitmap = (DisplayBitmap + 1) % 3;

				VS.m_Kinect.RegisterDepthToColor((DisplayBitmap != 0));

				break;
#endif
			case 'r':
				bRecord = (!bRecord && bKinect);

				bContinuous = false;

				DisplayBitmap = 0;

				bDisplayMesh = false;

				bDisplayConvexSets = false;

				break;
#ifdef RVLVTK
			case 'v':
				RVLDisplaySegmentedMesh3D(&Renderer, &(VS.m_AImage.m_C2DRegion.m_ObjectList), nObjects, w, h, pointmap, VS.m_PSD.m_Point3DMap);

				bRefresh = true;
				bVTKRendererActive = true;

				break;
			case 'p':
				if (bVTKRendererActive)
				{
					RVLSetFileNumber(VTK3DModelFileName, "00000.ply", iVTK3DModel);

					Renderer.Save2PLY(VTK3DModelFileName);

					iVTK3DModel++;

					strcpy(VTKMessage, VTKMessageConst);
					strcat(VTKMessage, VTK3DModelFileName);

					GUI.Message(VTKMessage, 600, 100, cvScalar(0, 128, 255));
				}

				bRefresh = true;

				break;
#endif
			case 0x00260000:
				if(VS.m_Flags & RVLSYS_FLAGS_PC)
					VS.m_PSD.m_MeshTol++;
				else
					VS.m_PSD.m_uvdTol++;

				bNextImage = false;

				break;
			case 0x00280000:
				if(VS.m_Flags & RVLSYS_FLAGS_PC)
				{
					if(VS.m_PSD.m_MeshTol > 1)
						VS.m_PSD.m_MeshTol--;
				}
				else
				{
					if(VS.m_PSD.m_uvdTol > 1)
						VS.m_PSD.m_uvdTol--;
				}

				bNextImage = false;

				break;
			case 0x00270000:
				VS.m_ConvexSegmentThr++;

				bNextImage = false;

				break;
			case 0x00250000:
				if(VS.m_ConvexSegmentThr > 0)
					VS.m_ConvexSegmentThr--;

				bNextImage = false;
			}
		}
		while(bRefresh && !bContinuous);

		if(!bKinect && bNextImage)
			RVLGetNextFileName(VS.m_ImageFileName, "00000-LW.bmp", 10000);

		if(!bRecord)
			VS.m_Mem.Clear();
	}
	while(key != 27);

	// free memory

	delete[] SizeArray;
	delete[] VTKMessage;

	GUI.CloseFigure("RVLPCSdemo");

	cvReleaseImage(&pInputImage);
	cvReleaseImage(&pRGBImage);
	cvReleaseImage(&pGSImage);
	cvReleaseImage(&pZoomedInputImage);

	return 0;
}

void MessageCanNotOpenFile(CRVLGUI *pGUI, char *FileName)
{
	char message[] = "Can not open file ";

	char *str = new char[strlen(FileName) + strlen(message) + 2];

	strcpy(str, message);

	strcat(str, FileName);

	str[strlen(FileName) + strlen(message)] = '!';

	pGUI->Message(str, 400, 100, cvScalar(0, 128, 255));

	delete[] str;
}

void MouseCallback(int event, int x, int y, int flags, void* vpData)
{
	RVLPSULMDEMO_MOUSE_CALLBACK_DATA *pData = (RVLPSULMDEMO_MOUSE_CALLBACK_DATA *)vpData;

	CRVL3DSurface2 *pSelectedSurf = NULL;
	CRVL3DLine2 *pSelectedLine = NULL;

	CRVLFigure *pFig = pData->pFig;

	CRVLPSuLMVS *pVS = pData->pVS;

	int w = pData->w;
	
	int iPix;

	switch( event )
	{
		case CV_EVENT_LBUTTONDOWN:
			//if(!pData->bSelection)
			{
				pData->u = x;

				pData->v = y;

				pData->bSelection = true;

				iPix = x / pData->ZoomFactor + y / pData->ZoomFactor * w;

				pVS->m_pPSuLM->Project(&(pFig->m_PoseC0), FALSE, iPix, &pSelectedSurf, &pSelectedLine);

				pVS->m_PSuLMBuilder.DisplayHypothesisData(pFig, 0, pSelectedSurf);
			}
	}
}