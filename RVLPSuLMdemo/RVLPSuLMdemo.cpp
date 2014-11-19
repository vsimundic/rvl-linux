// RVLPSuLMdemo.cpp : Defines the entry point for the console application.
//

//#include "highgui.h"
#include <stdio.h>
#include <time.h>
#include "RVLCore.h"
#include "RVLPCS.h"
#include "RVLRLM.h"
#include "RVLPSuLMBuilder.h"
#include "RVLPSuLMGroundTruth.h"
#include "RVLPSuLMVS.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#endif

#define RVLPSULMDEMO_DISPLAY_ONLY_REPRESENTATIVE_HYPOTHESES
//#define RVLPSULMDEMO_DISPLAY_ONLY_BEST_LOCAL_MODEL_HYPOTHESES

void MessageCanNotOpenFile(CRVLGUI *pGUI, char *FileName);

int main(int argc, char* argv[])
{
	CRVL3DPose NullPose;

	RVLNULL3VECTOR(NullPose.m_X);
	RVLUNITMX3(NullPose.m_Rot);

	//CRVL3DPose PoseLC;

	//RVLNULL3VECTOR(PoseLC.m_X);

	//RVLMXEL(PoseLC.m_Rot, 3, 0, 0) = 0.0;
	//RVLMXEL(PoseLC.m_Rot, 3, 1, 0) = 0.0;
	//RVLMXEL(PoseLC.m_Rot, 3, 2, 0) = 1.0;

	//RVLMXEL(PoseLC.m_Rot, 3, 0, 1) = -1.0;
	//RVLMXEL(PoseLC.m_Rot, 3, 1, 1) = 0.0;
	//RVLMXEL(PoseLC.m_Rot, 3, 2, 1) = 0.0;

	//RVLMXEL(PoseLC.m_Rot, 3, 0, 2) = 0.0;
	//RVLMXEL(PoseLC.m_Rot, 3, 1, 2) = -1.0;
	//RVLMXEL(PoseLC.m_Rot, 3, 2, 2) = 0.0;

	//CRVL3DPose PoseCL;

	//RVLINVTRANSF3D(PoseLC.m_Rot, PoseLC.m_X, PoseCL.m_Rot, PoseCL.m_X)

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

	/////

	int iMPSuLM = 0;

	if(VS.m_Flags & RVLSYS_FLAGS_EDIT_MAP)
	{
		while(iMPSuLM <= VS.m_PSuLMBuilder.m_maxPSuLMIndex && VS.m_PSuLMBuilder.m_PSuLMArray[iMPSuLM] == NULL)
			iMPSuLM++;

		if(VS.m_PSuLMBuilder.m_PSuLMArray[iMPSuLM] == NULL)
		{
			GUI.Message("Map is empty.", 400, 100, cvScalar(0, 128, 255));

			return 0;
		}
	}

	// initialize kinect

	bool bKinect;

	if(VS.m_Flags & RVLSYS_FLAGS_EDIT_MAP)
		bKinect = false;
	else
	{
#ifdef RVLOPENNI
		bool bKinect = VS.m_Kinect.Init();

		if(bKinect)
			VS.m_Flags &= ~RVLSYS_FLAGS_PC;
		else
			GUI.Message("Kinect is not available.", 400, 100, cvScalar(0, 128, 255));
#else
		bKinect = false;
#endif
	}

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

	VS.m_pRGBImage = pRGBImage;

	// create grayscale image

	IplImage *pGSImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);

	// create input image

	IplImage *pInputImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	// create zoomed image

	IplImage *pZoomedInputImage = cvCreateImage(cvSize(2 * w, 2 * h), IPL_DEPTH_8U, 3);

	// create auxiliary image

	//IplImage *pAuxImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	// create previous RGB image

	IplImage *pPrevRGBImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	// create HSV image

    IplImage *pHSVImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	// create mesh file

	if(VS.m_Flags & RVLSYS_FLAGS_CREATE_GLOBAL_MESH)
		VS.CreateMeshFile("Mesh.obj");

	// create main display image

	CRVLFigure *pFig = GUI.OpenFigure("Scene");

	pFig->m_Flags |= (RVLPSULM_DISPLAY_SCENE | RVLFIG_FLAG_DATA);

	pFig->m_FontSize = 16;
	cvInitFont(&(pFig->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2);

	//if(VS.m_Flags & RVLSYS_FLAGS_PC)
	//{
	//	RVLCOPYMX3X3(PoseCL.m_Rot, pFig->m_PoseC0.m_Rot)
	//	RVLCOPY3VECTOR(PoseCL.m_X, pFig->m_PoseC0.m_X)
	//}
	//else
		pFig->m_PoseC0.Reset();

	RVLPSULMDISPLAY_MOUSE_CALLBACK_DATA MouseCallbackData;	

	IplImage *pInputImage_ = pInputImage;

	MouseCallbackData.w = w;
	MouseCallbackData.pGUI = &GUI;
	MouseCallbackData.pFig = pFig;
	MouseCallbackData.pVS = &VS;
	MouseCallbackData.pImage = pInputImage_;
	MouseCallbackData.pImage2 = pPrevRGBImage;
	//MouseCallbackData.pPoseCM = (VS.m_Flags & RVLSYS_FLAGS_PC ? &PoseCL : &NullPose);

	// create auxiliary display image

	CRVLFigure *pFig2 = GUI.OpenFigure("Model");

	pFig2->m_Flags |= (RVLPSULM_DISPLAY_MODEL | RVLFIG_FLAG_DATA);

	pFig2->EmptyBitmap(cvSize(w, h), cvScalar(0, 0, 0));

	pFig2->m_FontSize = 16;
	cvInitFont(&(pFig2->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2);

	pFig2->m_PoseC0.Reset();

	MouseCallbackData.pFig2 = pFig2;

	RVLPSULMDISPLAY_MOUSE_CALLBACK_DATA MouseCallbackData2;	

	MouseCallbackData2.w = w;
	MouseCallbackData2.pGUI = &GUI;
	MouseCallbackData2.pFig = pFig2;
	MouseCallbackData2.pFig2 = pFig;
	MouseCallbackData2.pVS = &VS;
	MouseCallbackData2.ZoomFactor = 1;	
	MouseCallbackData2.pImage = pInputImage_;
	MouseCallbackData2.pImage2 = pPrevRGBImage;
	//MouseCallbackData2.pPoseCM = (VS.m_Flags & RVLSYS_FLAGS_PC ? &PoseCL : &NullPose);

	// allocate memory

	int *SizeArray = new int[2 * w * h];

	// main loop

	bool bDisplayMesh = false;
	bool bDisplayConvexSets = ((VS.m_Flags & RVLSYS_FLAGS_PC) != 0);
	bool bDisplayHypothesis = true;
	bool bDisplayPSuLM = false;
	//bool bContinuous = bKinect;
	bool bContinuous = false;
	bool bRecord = false;
	bool bFrames = false;
	//DWORD mDisplayPSuLMFlags = (RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_VECTORS | RVLPSULM_DISPLAY_SAMPLES);
	DWORD mDisplayPSuLMFlags = (RVLPSULM_DISPLAY_SURFACES | RVLPSULM_DISPLAY_ELLIPSES | RVLPSULM_DISPLAY_LINES | RVLPSULM_DISPLAY_VECTORS);
	int DisplayBitmap = 0;
	int ZoomFactor = 1;

	if(!bKinect)
		RVLGetFirstValidFileName(VS.m_ImageFileName, "00000-sl.bmp", 10000);

	bool bVTKRendererActive = false;
	int iVTK3DModel = 0;

	char VTK3DModelFileName[] = "VTK3DModel_00000.ply";
	char VTKMessageConst[] = "3D model in PLY-format saved in ";
	char *VTKMessage = new char[strlen(VTKMessageConst) + strlen(VTK3DModelFileName) + 1];

	char GlobalMeshFileName[] = "Mesh.obj";

	int VTKTexture = 0;
	
	int nObjects = 1;
	int textureFileNumber = 1;

	RVLPSULM_HYPOTHESIS *HypothesisMem = NULL;

	DWORD HypEvalMethod = (VS.m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD);

	int iHypothesis;
	int key;
	//int iSample;
	bool bRefresh;
	bool bNextImage;
	bool bBackwards;
	clock_t t;
	char str[200];
	int iTextLine;
	unsigned int DepthMapFormat;
	CRVLMPtrChain *pPSuLMList;
	CRVLMem *pMem;
	RVLQLIST *pMap;
	RVLPSULM_HYPOTHESIS *pHypothesis;
	CRVLPSuLM *pPSuLM;
	int iMPSuLM_;
#ifdef RVLPSULMDEMO_DISPLAY_ONLY_REPRESENTATIVE_HYPOTHESES
	int iHypothesis_;
#endif	
	int key_;
	int SampleStep;

	do
	{
		DepthMapFormat = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY;

		memcpy(pPrevRGBImage->imageData, pRGBImage->imageData, pRGBImage->imageSize);

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
				{
					MessageCanNotOpenFile(&GUI, VS.m_ImageFileName);

					cvWaitKey();

					return 0;
				}
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
				if(!RVLImportDisparityImage(VS.m_ImageFileName, pDepthImage, DepthMapFormat, VS.m_Kinect.m_zToDepthLookupTable))
				{
					MessageCanNotOpenFile(&GUI, VS.m_ImageFileName);
				
					cvWaitKey();

					return 0;
				}
				else
				{
					RVLSaveDepthImage(pDepthImage->Disparity, w, h, VS.m_ImageFileName, RVLKINECT_DEPTH_IMAGE_FORMAT_1MM, 
						RVLKINECT_DEPTH_IMAGE_FORMAT_1MM);

					int iSample = RVLGetFileNumber(VS.m_ImageFileName, "00000-D.txt");
				
					RVLSetFileNumber(VS.m_ImageFileName, "00000-D.txt", iSample + 1);
				}
			}
		}	// if(bRecord)

		if(!bRecord)
		{
			if(VS.m_Flags & RVLSYS_FLAGS_EDIT_MAP)
			{			
				pPSuLM = VS.m_PSuLMBuilder.m_PSuLMArray[iMPSuLM];				

				RVLCopyString(pPSuLM->m_FileName, &(VS.m_ImageFileName));

				if(HypEvalMethod == RVLPSULMBUILDER_FLAG_HYPOTHESIS_EVALUATION_METHOD_P)
					VS.m_PSuLMBuilder.InitHypothesisEvaluation4(pPSuLM);
			}
				
			if(!bKinect)
				pRGBImage = cvLoadImage(VS.m_ImageFileName);

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

			//VS.m_PSuLMBuilder.m_Flags |= RVLPSULMBUILDER_FLAG_KIDNAPPED;

			VS.Update(bKinect ? 0x00000000 : RVLPSULMBUILDER_CREATEMODEL_IMAGE_FROM_FILE);

			t = clock() - t;	

			//// mark segment edges

			//if(VS.m_PSD.m_Flags & RVLPSD_MESH_SEGMENT_PLANAR)
			if((VS.m_Flags & RVLSYS_FLAGS_CREATE_GLOBAL_MESH) == 0 || 
				(VS.m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MODE_LOCALIZATION) == 0)
			{
				nObjects = VS.m_AImage.m_C2DRegion3.m_ObjectList.m_nElements + 1;

				VS.m_PSD.AssignLabels(&(VS.m_AImage.m_C2DRegion), &(VS.m_AImage.m_C2DRegion3));
			}

			RVLSegmentationEdgesFromLabels(&(VS.m_AImage.m_C2DRegion));

			// store RGB image

			cvCvtColor(pRGBImage, pHSVImage, CV_BGR2RGB);

			pHSVImage->channelSeq[0] = 'R';

			pHSVImage->channelSeq[1] = 'G';

			pHSVImage->channelSeq[2] = 'B';
		}	// if(!bRecord)

		// display the results

		if(VS.m_Flags & RVLSYS_FLAGS_EDIT_MAP)
		{
			VS.m_pPSuLM->m_Index = pPSuLM->m_Index;

			RVLQLIST_PTR_ENTRY *pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pPSuLM->m_NeighbourList->pFirst);

			VS.m_PSuLMBuilder.m_nHypotheses = 0;

			while(pNeighborPtr)
			{
				pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);	

				VS.m_PSuLMBuilder.m_nHypotheses++;
			}

			VS.m_PSuLMBuilder.m_HypothesisList.m_nElements = VS.m_PSuLMBuilder.m_nHypotheses;

			if(VS.m_PSuLMBuilder.m_nHypotheses > 0)
			{
				if(HypothesisMem)
					delete[] HypothesisMem;

				HypothesisMem = new RVLPSULM_HYPOTHESIS[VS.m_PSuLMBuilder.m_nHypotheses];				

				if(VS.m_PSuLMBuilder.m_HypothesisArray)
					delete[] VS.m_PSuLMBuilder.m_HypothesisArray;

				VS.m_PSuLMBuilder.m_HypothesisArray = new RVLPSULM_HYPOTHESIS *[VS.m_PSuLMBuilder.m_nHypotheses];
			
				pHypothesis = HypothesisMem;

				pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pPSuLM->m_NeighbourList->pFirst);

				iHypothesis = 0;

				RVLPSULM_NEIGHBOUR *pNeighborRel;

				while(pNeighborPtr)
				{
					pNeighborRel = (RVLPSULM_NEIGHBOUR *)(pNeighborPtr->Ptr);

					pHypothesis->pMPSuLM = pNeighborRel->pPSuLM;

					double *RMS = pNeighborRel->pPoseRel->m_Rot;
					double *tMS = pNeighborRel->pPoseRel->m_X;
					double *RSM = pHypothesis->PoseSM.m_Rot;
					double *tSM = pHypothesis->PoseSM.m_X;

					RVLINVTRANSF3D(RMS, tMS, RSM, tSM)

					pHypothesis->PoseSM.UpdatePTRLL();

					pHypothesis->Index = iHypothesis;

					pHypothesis->cost = 0;

					pHypothesis->iRepresentative = 0xffffffff;

					pHypothesis->pMPSuLM->m_pHypothesis = pHypothesis;

					pHypothesis->Probability = 0.0;

					pHypothesis->pMPSuLM->m_PosteriorProbabilityLocal = pHypothesis->pMPSuLM->m_PosteriorProbabilityGlobal = 0.0;

					pHypothesis->pMPSuLM->m_PosteriorProbabilityLocal5DOF = 0.0;

					VS.m_PSuLMBuilder.m_HypothesisArray[iHypothesis] = pHypothesis;

					iHypothesis++;
				
					pHypothesis++;

					pNeighborPtr = (RVLQLIST_PTR_ENTRY *)(pNeighborPtr->pNext);	
				}

				iHypothesis = 0;
			}
		}	// if(VS.m_Flags & RVLSYS_FLAGS_EDIT_MAP)
		else
		{
#ifdef RVLPSULMDEMO_DISPLAY_ONLY_REPRESENTATIVE_HYPOTHESES
			iHypothesis = 0;

			if(VS.m_PSuLMBuilder.m_nHypotheses > 0)
			{
				while(iHypothesis < VS.m_PSuLMBuilder.m_nHypotheses)
				{
					pHypothesis = VS.m_PSuLMBuilder.m_HypothesisArray[iHypothesis];

#ifdef RVLPSULMDEMO_DISPLAY_ONLY_BEST_LOCAL_MODEL_HYPOTHESES
					if(pHypothesis == pHypothesis->pMPSuLM->m_pHypothesis)
						break;
#else
					if(pHypothesis->iRepresentative == 0xffffffff)
						break;
#endif

					iHypothesis++;
				}

				if(iHypothesis >= VS.m_PSuLMBuilder.m_nHypotheses)
					iHypothesis = 0;
			}
#else
			iHypothesis = 0;
#endif
			VS.m_pPSuLM->m_Index = 0xffffffff;
		}

		do
		{
			// clear display

			pFig->Clear();

			pFig2->Clear();

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
				VS.m_PSuLMBuilder.DisplayHypothesis(&GUI, pFig, pFig2, VS.m_pPSuLM, mDisplayPSuLMFlags, pInputImage_,
					pPrevRGBImage, iHypothesis);

				VS.m_PSuLMBuilder.DisplayHypothesisData(pFig, VS.m_pPSuLM, iHypothesis);
			}

			if(bDisplayPSuLM)
			{
				pFig->m_pImage = cvCloneImage(pInputImage_);

				//if(VS.m_Flags & RVLSYS_FLAGS_PC)
				//	VS.m_pPSuLM->Display(pFig, &PoseLC, cvScalar(0, 255, 0), mDisplayPSuLMFlags);
				//else
					VS.m_pPSuLM->Display(pFig, &NullPose, cvScalar(0, 255, 0), mDisplayPSuLMFlags);
			}

			GUI.DisplayVectors(pFig, 0, 0, (double)ZoomFactor);

			GUI.DisplayVectors(pFig2, 0, 0, 1.0);

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
			MouseCallbackData.mDisplayPSuLMFlags = mDisplayPSuLMFlags;
			MouseCallbackData.iHypothesis = (VS.m_PSuLMBuilder.m_nHypotheses > 0 ? iHypothesis : -1);
			MouseCallbackData.pImage = pInputImage_;

			GUI.ShowFigure(pFig);	

			cvSetMouseCallback("Scene", RVLPSuLMDisplayMouseCallback2, &MouseCallbackData);
							
			MouseCallbackData2.mDisplayPSuLMFlags = mDisplayPSuLMFlags;
			MouseCallbackData2.iHypothesis = (VS.m_PSuLMBuilder.m_nHypotheses > 0 ? iHypothesis : -1);

			GUI.ShowFigure(pFig2);	

			cvSetMouseCallback("Model", RVLPSuLMDisplayMouseCallback2, &MouseCallbackData2);

			//cvSaveImage("C:\\RVL\\ExpRez\\RVLDisplay.bmp", pDisplay);

			// wait until a key is pressed

			key = (bContinuous ? cvWaitKey(1) : cvWaitKey());

			// change the display according to the key pressed

			bNextImage = true;
			bRefresh = false;
			bBackwards = false;
			SampleStep = 1;

			switch(key){
			case '0':
				if(pHypothesis)
					VS.m_GroundTruth.Add(RVLGetFileNumber(VS.m_ImageFileName, "00000-LW.bmp"), pHypothesis->pMPSuLM->m_Index, &(pHypothesis->PoseSM));

				bRefresh = true;

				break;
			case 'a':
				mDisplayPSuLMFlags ^= RVLPSULM_DISPLAY_SAMPLES;
			
				bRefresh = true;				
	
				break;
			case 'b':
				//DisplayBitmap = (DisplayBitmap + 1) % 3;
				DisplayBitmap = (DisplayBitmap + 1) % 2;

#ifdef RVLOPENNI
				if(bKinect)
					VS.m_Kinect.RegisterDepthToColor((DisplayBitmap != 0));
#endif
				bRefresh = true;

				break;
			case 'c':
				bContinuous = !bContinuous;

				break;
			case 'e':
				mDisplayPSuLMFlags ^= (RVLPSULM_DISPLAY_ELLIPSES | RVLPSULM_DISPLAY_LINES);
			
				bRefresh = true;				
	
				break;
			case 'f':	// Map building on/off
				VS.m_PSuLMBuilder.m_Flags ^= RVLPSULMBUILDER_FLAG_MAPBUILDING;

				if(VS.m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING)
				{
					VS.m_PSuLMBuilder.m_Flags &= ~RVLPSULMBUILDER_FLAG_GLOBAL;

					//if(VS.m_PSuLMBuilder.m_nPlausibleHypotheses > 0)
					//	VS.m_PSuLMBuilder.m_pNearestModelPSuLM = VS.m_PSuLMBuilder.m_HypothesisArray[0]->pMPSuLM;

#ifdef RVLPSULMBUILDER_MAPBUILDING_SEQUENCE
					if(VS.m_PSuLMBuilder.m_nHypotheses > 0)
						if(VS.m_PSuLMBuilder.m_HypothesisArray[0]->pMPSuLM->m_PosteriorProbabilityLocal >= 0.999)
							VS.m_PSuLMBuilder.m_pNearestModelPSuLM = VS.m_PSuLMBuilder.m_HypothesisArray[0]->pMPSuLM;
#endif
				}

				bNextImage = false;

				break;
			case 'g':	// global localization on/off
				VS.m_PSuLMBuilder.m_Flags ^= RVLPSULMBUILDER_FLAG_GLOBAL;

				break;
			case 'h':
				bDisplayHypothesis = (!bDisplayHypothesis && !bRecord);

				bDisplayPSuLM = (bDisplayPSuLM & !bDisplayHypothesis);

				bRefresh = true;

				break;
			case 'i':	// loop start <- last MPSuLM
				if(VS.m_PSuLMBuilder.m_pNearestModelPSuLM)
				{
					VS.m_PSuLMBuilder.m_pLoopStartPSuLM = VS.m_PSuLMBuilder.m_pNearestModelPSuLM;

					bRefresh = true;
				}

				break;
			case 'l':	// loop closing 
				if((VS.m_PSuLMBuilder.m_Flags & RVLPSULMBUILDER_FLAG_MAPBUILDING) && VS.m_PSuLMBuilder.m_pLoopStartPSuLM)
				{
					VS.m_PSuLMBuilder.m_Flags |= RVLPSULMBUILDER_FLAG_MANUAL_LOOP_CLOSING;

					VS.m_PSuLMBuilder.m_Flags &= ~RVLPSULMBUILDER_FLAG_GLOBAL;
				}

				break;
			case 'm':
				bDisplayMesh = (!bDisplayMesh && !bRecord);

				bRefresh = true;

				break;
#ifdef RVLVTK
			case 'o':
				bNextImage = false;

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
			case 'r':
				bRecord = (!bRecord && bKinect);

				bContinuous = false;

				DisplayBitmap = 0;

				bDisplayMesh = false;

				bDisplayConvexSets = false;

				break;
			case 's':
				bDisplayConvexSets = (!bDisplayConvexSets && !bRecord);

				bRefresh = true;

				break;
			case 't':
#ifdef RVLVTK
				VTKTexture = (VTKTexture + 1) % 2;
 
				if(bVTKRendererActive)
					RVLDisplaySegmentedMesh3D(&Renderer, &(VS.m_AImage.m_C2DRegion.m_ObjectList), nObjects, w, h, pointmap,
						VS.m_PSD.m_Point3DMap, VTKTexture, pHSVImage);
				else
#endif
				GUI.Message("VTK Texture mode changed.", 600, 100, cvScalar(0, 128, 255));

				bRefresh = true;

				break;
			case 'u':
				bDisplayPSuLM = (!bDisplayPSuLM && !bRecord);

				bDisplayHypothesis = (bDisplayHypothesis & !bDisplayPSuLM);

				bRefresh = true;

				break;
#ifdef RVLVTK
			case 'v':
				RVLDisplaySegmentedMesh3D(&Renderer, &(VS.m_AImage.m_C2DRegion.m_ObjectList), nObjects, w, h, pointmap, VS.m_PSD.m_Point3DMap);

				bRefresh = true;
				bVTKRendererActive = true;

				break;
#endif
			case 'x':
				pPSuLMList = &(VS.m_PSuLMBuilder.m_PSuLMList);

				pMem = VS.m_PSuLMBuilder.m_pMem;

				pMap = VS.m_PSuLMBuilder.ConvertPtrChain2QLIST(pPSuLMList, pMem);

				VS.m_PSuLMBuilder.SaveXMLMap(VS.m_PSuLMBuilder.m_ModelMapPath, pMap);

				GUI.Message("Map saved.", 600, 100, cvScalar(0, 128, 255));

				bRefresh = true;

				break;
			case 'z':
				if(ZoomFactor == 1)
				{
					ZoomFactor = 2;
					pInputImage_ = pZoomedInputImage;
				}
				else
				{
					ZoomFactor = 1;
					pInputImage_ = pInputImage;
				}

				bRefresh = true;

				break;
			case '*':
				//key_ = GUI.Message("Run UpdateRelativePoseUncertainties()?", 600, 100, cvScalar(0, 128, 255));

				//if(key_ == 'y')
				//{
				//	VS.m_PSuLMBuilder.UpdateRelativePoseUncertainties();

				//	GUI.Message("UpdateRelativePoseUncertainties() completed.", 600, 100, cvScalar(0, 128, 255));
				//}

				if(VS.m_Flags & RVLSYS_FLAGS_EDIT_MAP)
				{
					key_ = GUI.Message("Run CreateLocal3DMesh()?", 600, 100, cvScalar(0, 128, 255));

					if(key_ == 'y')
					{
						VS.CreateLocal3DMesh(pPSuLM);

						GUI.Message("CreateLocal3DMesh() completed.", 600, 100, cvScalar(0, 128, 255));
					}
				}

				bRefresh = true;

				break;
			case 0x00000008:	// backspace
				bBackwards = true;

				break;
			case 0x00210000:	// PgUp
				bBackwards = true;

				SampleStep = 10;

				break;
			case 0x00220000:	// PgDn
				SampleStep = 10;

				break;
			case 0x00240000:	// Home
				if(VS.m_PSuLMBuilder.m_nHypotheses > 0)
				{
					iHypothesis = 0;

					bRefresh = true;
				}

				break;
			case 0x00260000:	// Up
				if(VS.m_PSuLMBuilder.m_nHypotheses > 0)
				{
#ifdef RVLPSULMDEMO_DISPLAY_ONLY_REPRESENTATIVE_HYPOTHESES
					iHypothesis_ = iHypothesis;

					iHypothesis--;

					while(iHypothesis >= 0)
					{
						pHypothesis = VS.m_PSuLMBuilder.m_HypothesisArray[iHypothesis];

#ifdef RVLPSULMDEMO_DISPLAY_ONLY_BEST_LOCAL_MODEL_HYPOTHESES
						if(pHypothesis == pHypothesis->pMPSuLM->m_pHypothesis)
							break;
#else
						if(pHypothesis->iRepresentative == 0xffffffff)
							break;
#endif

						iHypothesis--;						
					}

					if(iHypothesis < 0)
						iHypothesis = iHypothesis_;
#else
					if(iHypothesis > 0)
						iHypothesis--;
#endif

					bRefresh = true;
				}
			//	if(VS.m_Flags & RVLSYS_FLAGS_PC)
			//		VS.m_PSD.m_MeshTol++;
			//	else
			//		VS.m_PSD.m_uvdTol++;

			//	bNextImage = false;

				break;
			case 0x00280000:	// Down
				if(VS.m_PSuLMBuilder.m_nHypotheses > 0)
				{
#ifdef RVLPSULMDEMO_DISPLAY_ONLY_REPRESENTATIVE_HYPOTHESES
					iHypothesis_ = iHypothesis;

					iHypothesis++;

					while(iHypothesis < VS.m_PSuLMBuilder.m_nHypotheses)
					{
						pHypothesis = VS.m_PSuLMBuilder.m_HypothesisArray[iHypothesis];

#ifdef RVLPSULMDEMO_DISPLAY_ONLY_BEST_LOCAL_MODEL_HYPOTHESES
						if(pHypothesis == pHypothesis->pMPSuLM->m_pHypothesis)
							break;
#else
						if(pHypothesis->iRepresentative == 0xffffffff)
							break;
#endif

						iHypothesis++;						
					}

					if(iHypothesis >= VS.m_PSuLMBuilder.m_nHypotheses)
						iHypothesis = iHypothesis_;
#else
					if(iHypothesis < VS.m_PSuLMBuilder.m_nHypotheses - 1)
						iHypothesis++;
#endif

					bRefresh = true;
				}

			//	if(VS.m_Flags & RVLSYS_FLAGS_PC)
			//	{
			//		if(VS.m_PSD.m_MeshTol > 1)
			//			VS.m_PSD.m_MeshTol--;
			//	}
			//	else
			//	{
			//		if(VS.m_PSD.m_uvdTol > 1)
			//			VS.m_PSD.m_uvdTol--;
			//	}

			//	bNextImage = false;

				break;
			//case 0x00270000:
			//	VS.m_ConvexSegmentThr++;

			//	bNextImage = false;

			//	break;
			//case 0x00250000:
			//	if(VS.m_ConvexSegmentThr > 0)
			//		VS.m_ConvexSegmentThr--;

			//	bNextImage = false;
			case 0x002e0000:	// delete
				if(VS.m_Flags & RVLSYS_FLAGS_EDIT_MAP)
				{
					if(VS.m_PSuLMBuilder.m_nHypotheses > 0)
					{
						key_ = GUI.Message("Do you really want to delete this connection? (If yes, press 'y')", 600, 100, cvScalar(0, 128, 255));

						if(key_ == 'y')
						{
							pHypothesis = VS.m_PSuLMBuilder.m_HypothesisArray[iHypothesis];

							VS.m_PSuLMBuilder.DeleteConnection(pPSuLM, pHypothesis->pMPSuLM);
		
							int n = VS.m_PSuLMBuilder.m_nHypotheses - iHypothesis - 1;

							if(n > 0)
								memmove(VS.m_PSuLMBuilder.m_HypothesisArray + iHypothesis, VS.m_PSuLMBuilder.m_HypothesisArray + iHypothesis + 1, 
									n * sizeof(RVLPSULM_HYPOTHESIS *));

							VS.m_PSuLMBuilder.m_nHypotheses--;

							VS.m_PSuLMBuilder.m_HypothesisList.m_nElements--;

							if(iHypothesis > VS.m_PSuLMBuilder.m_nHypotheses - 1)
								iHypothesis = VS.m_PSuLMBuilder.m_nHypotheses - 1;
						}

						bRefresh = true;
					}
					else
					{
						key_ = GUI.Message("Do you really want to delete this PSuLM? (If yes, press 'y')", 600, 100, cvScalar(0, 0, 255));

						if(key_ == 'y')
						{
							VS.m_PSuLMBuilder.m_PSuLMList.Start();

							RVLPTRCHAIN_ELEMENT *pCurrent;
							CRVLPSuLM *pPSuLM_;

							while(VS.m_PSuLMBuilder.m_PSuLMList.m_pNext)
							{
								pCurrent = VS.m_PSuLMBuilder.m_PSuLMList.m_pCurrent;

								pPSuLM_ = (CRVLPSuLM *)(VS.m_PSuLMBuilder.m_PSuLMList.GetNext());

								if(pPSuLM_ == pPSuLM)
								{
									VS.m_PSuLMBuilder.m_PSuLMList.RemoveAt(pCurrent);

									VS.m_PSuLMBuilder.m_PSuLMArray[iMPSuLM] = NULL;

									if(VS.m_PSuLMBuilder.m_PSuLMList.m_nElements > 0)
									{
										while(iMPSuLM <= VS.m_PSuLMBuilder.m_maxPSuLMIndex && VS.m_PSuLMBuilder.m_PSuLMArray[iMPSuLM] == NULL)
											iMPSuLM++;

										if(iMPSuLM > VS.m_PSuLMBuilder.m_maxPSuLMIndex)
											iMPSuLM = 0;

										while(iMPSuLM <= VS.m_PSuLMBuilder.m_maxPSuLMIndex && VS.m_PSuLMBuilder.m_PSuLMArray[iMPSuLM] == NULL)
											iMPSuLM++;
									}
									else
										iMPSuLM = 0;

									break;
								}
							}
						}
					}
				}
			}
		}
		while(bRefresh && !bContinuous);

		// next image/model

		//if(!bKinect && bNextImage)
		if(bNextImage)
		{
			if(bKinect)
			{
				int iSample = RVLGetFileNumber(VS.m_ImageFileName, "00000-LW.bmp");

				RVLSetFileNumber(VS.m_ImageFileName, "00000-LW.bmp", iSample + 1);
			}
			else if(VS.m_Flags & RVLSYS_FLAGS_EDIT_MAP)
			{
				iMPSuLM_ = iMPSuLM;

				if(bBackwards)
				{
					iMPSuLM -= SampleStep;

					while(iMPSuLM >= 0 && VS.m_PSuLMBuilder.m_PSuLMArray[iMPSuLM] == NULL)
						iMPSuLM--;

					if(iMPSuLM < 0)
						iMPSuLM = iMPSuLM_;
				}
				else
				{
					iMPSuLM += SampleStep;

					while(iMPSuLM <= VS.m_PSuLMBuilder.m_maxPSuLMIndex && VS.m_PSuLMBuilder.m_PSuLMArray[iMPSuLM] == NULL)
						iMPSuLM++;

					if(iMPSuLM > VS.m_PSuLMBuilder.m_maxPSuLMIndex)
						iMPSuLM = iMPSuLM_;
				}
			}	
			else
				RVLGetNextFileName(VS.m_ImageFileName, "00000-LW.bmp", 10000);
		}

		if(!bRecord)
			VS.m_Mem.Clear();
	}
	while(key != 27);

	// free memory

	if(HypothesisMem)
		delete[] HypothesisMem;

	delete[] SizeArray;
	delete[] VTKMessage;

	GUI.CloseFigure("RVLPCSdemo");

	cvReleaseImage(&pInputImage);
	cvReleaseImage(&pRGBImage);
	cvReleaseImage(&pGSImage);
	cvReleaseImage(&pZoomedInputImage);
	//cvReleaseImage(&pAuxImage);
	cvReleaseImage(&pPrevRGBImage);

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

