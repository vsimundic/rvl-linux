// RVLPCSdemo.cpp : Defines the entry point for the console application.
//

//#include "highgui.h"
#include <stdio.h>
#include <time.h>
#include "RVLCore.h"
#include "RVLPCS.h"
#ifdef RVLVTK
//VTK headers
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);//(vtkRenderingFreeTypeOpenGL);
#include "RVLVTK.h"
//#include "VTKActorObj.h"
#endif

int main(int argc, char* argv[])
{
	// create vision system

	CRVLPCSVS VS;

	// initialize vision system

	VS.CreateParamList();

	VS.Init("RVLPCSdemo.cfg");

	// create GUI

	CRVLPCSGUI GUI;

#ifdef RVLPSD_SEGMENT_STRM_DEBUG
	VS.m_PSD.m_DebugData.pGUI = &GUI;
#endif

	GUI.Init(&VS);

	// If Kinect is not available, display a message.

	if (!(VS.m_Flags & RVLSYS_FLAGS_KINECT))
		GUI.Message("Kinect is not available.", 400, 100, cvScalar(0, 128, 255));

#ifdef RVLOPENNI
	if ((VS.m_Flags & RVLSYS_FLAGS_KINECT) && GUI.m_bRecord)
		VS.m_StereoVision.m_DisparityMap.Format = RVLKINECT_DEPTH_IMAGE_FORMAT_1MM;
#endif

	// get the pointer to the depth image

	RVLDISPARITYMAP *pDepthImage;

	pDepthImage = &(VS.m_StereoVision.m_DisparityMap);

#ifdef NEVER	// 150820

#ifdef RVLVTK
	// create VTK renderer

	CRVLVTKRenderer Renderer;

	Renderer.Init(800, 600);
	Renderer.m_pWindow->Render();

	int *pointmap = new int[w * h];
#endif

	// create RGB image

	IplImage *pRGBImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	// create grayscale image

	IplImage *pGSImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);

	// create segmentation image

	IplImage *pSegmentationImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

	// create a display image

	CRVLFigure *pFig = GUI.OpenFigure("RVLPCSdemo");

	IplImage *pInputImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
	IplImage *pZoomedInputImage = cvCreateImage(cvSize(2 * w, 2 * h), IPL_DEPTH_8U, 3);

	pFig->m_pImage = pInputImage;

	pFig->m_FontSize = 16;
	cvInitFont(&(pFig->m_Font), CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 2);

	// allocate memory

	// filko

    CRVL3DMeshObject *objects;

    IplImage *pHSVImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

    //Class

    CRVLClass pClass;

    pClass.m_pMem = &VS.m_Mem;

    pClass.m_pMem0 = &VS.m_Mem0;

    pClass.m_pMem2 = &VS.m_Mem2;

	// Initialize display

	VS.m_Display.m_pFig = pFig;
	VS.m_Display.m_pInputImage = pInputImage;
	VS.m_Display.m_pDepthImage = pDepthImage;	
	VS.m_Display.m_pRGBImage = pRGBImage;
	VS.m_Display.m_pGSImage = pGSImage;
	VS.m_Display.m_pSegmentationImage = pSegmentationImage;
	VS.m_Display.m_pZoomedInputImage = pZoomedInputImage;
	VS.m_Display.m_ImageWidth = w;
	VS.m_Display.m_bKinect = bKinect;

	// main loop

	bool bDisplayMesh = true;
	bool bDisplayConvexSets = true;
	bool bDisplaySelectedObjects = true;
	//bool bContinuous = bKinect;
	bool bContinuous = false;
	bool bRecord = false;
	int DisplayBitmap = (VS.m_Flags & RVLSYS_FLAGS_SEGMENT_GRAPH ? 3 : 0);
	int ZoomFactor = 1;
	bool bVTKRendererActive = false;
	int iVTK3DModel = 0;

	//char VTK3DModelFileName[] = "VTK3DModel_00000.ply";
	//char VTKMessageConst[] = "3D model in PLY-format saved in ";
	char VTK3DModelFileName[] = "VTK3DModel_00000.obj";
	char VTKMessageConst[] = "3D model in OBJ-format saved in ";
	char *VTKMessage = new char[strlen(VTKMessageConst) + strlen(VTK3DModelFileName) + 1];
	char *VTKTextureFileName;
	int VTKTexture = 0;
	
	int iONISample = 0;
	int ONISpeed = 1;

	int key;
	//int iSample;
	int nObjects = 1;
	bool bRefresh;	
	bool bNextImageSelected;
	unsigned int DepthMapFormat;
	int iPrevONISample;
#endif

	FILE *fpExecTime = fopen("ExecTime.txt", "a");

	fprintf(fpExecTime, "=======\n");

	clock_t t;
	bool bContinue;

	do
	{
#ifdef NEVER // 150820
		DepthMapFormat = (VS.m_PSD.m_Flags & RVLPSD_FLAG_MM ? 
			(VS.m_PSD.m_Flags & RVLPSD_FLAG_100UM ? RVLKINECT_DEPTH_IMAGE_FORMAT_100UM : RVLKINECT_DEPTH_IMAGE_FORMAT_1MM) : 
			RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY);
#endif

#ifdef RVLOPENNI
		if (VS.m_Flags & RVLSYS_FLAGS_KINECT)
		{
			// acquire depth image from Kinect

			if (GUI.m_bNextImage)
				VS.m_Kinect.GetImages(pDepthImage->Disparity, GUI.m_pRGBImage, NULL, GUI.m_pGSImage, pDepthImage->Format, GUI.m_iONISample);
		}
		else
#endif
		// import depth image

		if(!VS.InputFromFile(pDepthImage, GUI.m_pRGBImage))
		{
			GUI.MessageCannotOpenFile(VS.m_ImageFileName);

			return 0;
		}

		if (GUI.m_bRecord)
		{
			if (VS.m_Flags & RVLSYS_FLAGS_PC)
				VS.SavePC();
			else
				VS.SaveRGBDImageToFile(pDepthImage, GUI.m_pRGBImage, "-LW.bmp");
		}
		else
		{
			t = clock();				

#ifdef NEVER
			if (VS.m_Flags & RVLSYS_FLAGS_SEGMENT_GRAPH)
			{			
				int nPts = w * h;

				RVLSWER_NODE2 *Node = new RVLSWER_NODE2[2 * nPts];

				RVLARRAY_<RVLSWER_SEGMENT<RVLSWER_NODE2>> SegmentArray;

				SegmentArray.Element = new RVLSWER_SEGMENT<RVLSWER_NODE2>[nPts];

				int *PtMem = new int[nPts];

				//// RGB image segmentation

				//RVLRGBGraphSegmentation((unsigned char *)(pRGBImage->imageData), w, h, 5, 300, Node, SegmentArray, PtMem);

				RVLDisplayRGBSegmentation(GUI.m_pRGBImage, SegmentArray, GUI.m_pSegmentationImage);

				delete[] PtMem;
				delete[] SegmentArray.Element;
				delete[] Node;
			}
#endif

			VS.Segment();

			t = clock() - t;

			GUI.m_ExecTime = 1000.0f * ((float)t) / CLOCKS_PER_SEC;

			fprintf(fpExecTime, "%d\t%lf\n", GUI.m_iONISample, GUI.m_ExecTime);

			fflush(fpExecTime);

#ifdef NEVER // 150820

				// load information about selected segments 

				if (VS.m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE)
				{
					char *SelectedSegmentsFileName = RVLKinectCreateONISampleFileName(VS.m_Kinect.m_ONIFileName, GUI.m_iONISample, "-SS.txt");

					RVLSegmentationLoadSelection(&(VS.m_AImage.m_C2DRegion.m_ObjectList), VS.m_nObjects, RVLOBJ2_FLAG_MARKED, SelectedSegmentsFileName);

					delete[] SelectedSegmentsFileName;
				}

				// filko

				//cvCvtColor(pRGBImage, pHSVImage, CV_BGR2HSV);

				//pHSVImage->channelSeq[0] = 'H';

				//pHSVImage->channelSeq[1] = 'S';

				//pHSVImage->channelSeq[2] = 'V';

				cvCvtColor(m_pRGBImage, m_pHSVImage, CV_BGR2RGB);

				pHSVImage->channelSeq[0] = 'R';

				pHSVImage->channelSeq[1] = 'G';

				pHSVImage->channelSeq[2] = 'B';

				objects = GenMeshObjects(&(VS.m_AImage.m_C2DRegion.m_ObjectList), pHSVImage, nObjects, &pClass);
#endif
			//PruneTrianglesFromObjects(objects, nObjects);
		}	// if(!bRecord)

		// display the results

		bContinue = GUI.InteractiveVisualization();

#ifdef NEVER // 150820
		do
		{
			VS.m_Display.m_bDisplayMesh = bDisplayMesh;
			VS.m_Display.m_bDisplayConvexSets = bDisplayConvexSets;
			VS.m_Display.m_bDisplaySelectedObjects = bDisplaySelectedObjects;
			VS.m_Display.m_bRecord = bRecord;
			VS.m_Display.m_iONISample = iONISample;			
			VS.m_Display.m_ZoomFactor = ZoomFactor;
			VS.m_Display.m_DisplayBitmap = DisplayBitmap;
			VS.m_Display.m_DepthMapFormat = DepthMapFormat;
			VS.m_Display.m_pRGBImage = pRGBImage;
			VS.m_Display.m_pGSImage = pGSImage;
			VS.m_Display.m_pSegmentationImage = pSegmentationImage;
	
#ifdef NEVER
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
			{
				RVLSegmentationEdgesFromLabels(&(VS.m_AImage.m_C2DRegion));

				RVLDisplay2DRegions(pFig, &(VS.m_AImage.m_C2DRegion.m_ObjectList), VS.m_CameraL.Width, 
					RVLColor(255, 0, 255), 2, RVLMESH_LINK_FLAG_EDGE, RVLMESH_LINK_FLAG_EDGE);
			}

			if(bDisplaySelectedObjects)
			{
				RVLResetFlags(&(VS.m_AImage.m_C2DRegion.m_ObjectList), RVLMESH_LINK_FLAG_EDGE);

				RVLSegmentationEdgesFromLabels(&(VS.m_AImage.m_C2DRegion), RVLOBJ2_FLAG_MARKED, RVLOBJ2_FLAG_MARKED);

				RVLDisplay2DRegions(pFig, &(VS.m_AImage.m_C2DRegion.m_ObjectList), VS.m_CameraL.Width, 
					RVLColor(255, 255, 0), 2, RVLMESH_LINK_FLAG_EDGE, RVLMESH_LINK_FLAG_EDGE);
			}			

			GUI.DisplayVectors(pFig, 0, 0, (double)ZoomFactor);

			if(!bRecord)
			{
				// display some numerical data

				iTextLine = 0;

				if(bKinect)
				{
					if(VS.m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE)
					{
						sprintf(str, "Sample %d", iONISample);

						cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));
					}
				}
				else
					cvPutText(pFig->m_pImage, VS.m_ImageFileName, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));

				sprintf(str, "Exec. Time = %4.0f ms", 1000.0f * ((float)t)/CLOCKS_PER_SEC);

				cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));

				sprintf(str, "Exec. Time = %4.0f ms", 1000.0f * ((float)t)/CLOCKS_PER_SEC);

				cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));

				sprintf(str, "TT = %d", ((VS.m_Flags & RVLSYS_FLAGS_PC) || (VS.m_PSD.m_Flags & RVLPSD_FLAG_MM) ? 
					VS.m_PSD.m_MeshTol : VS.m_PSD.m_uvdTol));

				cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));

				sprintf(str, "CT = %d", VS.m_ConvexSegmentThr);

				cvPutText(pFig->m_pImage, str, cvPoint(0, (++iTextLine) * pFig->m_FontSize), &pFig->m_Font,  cvScalar(255, 0, 0));
			}

			// show the display image

			GUI.ShowFigure(pFig);	
#endif
			VS.Display();

			cvSetMouseCallback(pFig->m_ImageName, RVLPCSDisplayMouseCallback, &VS);

			//cvSaveImage("C:\\RVL\\ExpRez\\RVLDisplay.bmp", pDisplay);

			// wait until a key is pressed

			key = (bContinuous ? cvWaitKey(1) : cvWaitKey());

			// change the display according to the key pressed

			iPrevONISample = iONISample;

			bNextImage = true;
			bRefresh = false;
			bNextImageSelected = false;
			int VTKTexture_[] = {0, 2, 1, 0};

			switch(key){
			case 'm':
				bDisplayMesh = (!bDisplayMesh && !bRecord);

				bRefresh = true;

				break;
			case 's':
				bDisplayConvexSets = (!bDisplayConvexSets && !bRecord);

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
			case 'b':
				DisplayBitmap = (DisplayBitmap + 1) % (VS.m_Flags & RVLSYS_FLAGS_SEGMENT_GRAPH ? 4 : 3);

#ifdef RVLOPENNI
				VS.m_Kinect.RegisterDepthToColor((DisplayBitmap != 0));
#endif

				bRefresh = true;

				break;
			case 'r':
				if(VS.m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE)
				{
					RVLSaveDepthImage(pDepthImage->Disparity, w, h, VS.m_ImageFileName, DepthMapFormat, DepthMapFormat);

					int iSample = RVLGetFileNumber(VS.m_ImageFileName, "00000-D.txt");
				
					RVLSetFileNumber(VS.m_ImageFileName, "00000-D.txt", iSample + 1);

					char *RGBFileName = RVLCreateFileName(VS.m_ImageFileName, "-D.txt", iSample + 1, "-LW.bmp");

					cvSaveImage(RGBFileName, pRGBImage);

					delete[] RGBFileName;

					bRefresh = true;
				}
				else
				{
					bRecord = (!bRecord && bKinect);

					bContinuous = false;

					DisplayBitmap = 0;

					bDisplayMesh = false;

					bDisplayConvexSets = false;
				}

				break;
#ifdef RVLVTK
			case 'v':
				RVLDisplaySegmentedMesh3D(&Renderer, &(VS.m_AImage.m_C2DRegion.m_ObjectList), nObjects, w, h, VS.m_PSD.m_nFOVExtensions, pointmap,
					VS.m_PSD.m_Point3DMap, VTKTexture, pHSVImage);

				bRefresh = true;
				bVTKRendererActive = true;

				break;
#endif
			case 't':
#ifdef RVLVTK
				VTKTexture = (VTKTexture + 1) % 2;
 
				if(bVTKRendererActive)
					RVLDisplaySegmentedMesh3D(&Renderer, &(VS.m_AImage.m_C2DRegion.m_ObjectList), nObjects, w, h, VS.m_PSD.m_nFOVExtensions, pointmap,
						VS.m_PSD.m_Point3DMap, VTKTexture, pHSVImage);
				else
#endif
				GUI.Message("VTK Texture mode changed.", 600, 100, cvScalar(0, 128, 255));

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

				cvSaveImage("Texture.bmp", pRGBImage);				
				
				objects->SaveMeshObject2OBJ(dat, mtldat, VTKTextureFileName, VS.m_PSD.m_Point3DMap, VTKTexture_[VTKTexture],
					"Texture.bmp");  //po dominantnom binu

				delete[] VTKTextureFileName;
				
				fclose(dat);
				
				fclose(mtldat);
				
				//primjer za teksture
				
				//pRootMeshObject->SaveMeshObject2OBJ(objf, mtlf, "test.obj.mtl", m_PSD.m_Point3DMap, 2, "sl-00000-LW.bmp");

				strcpy(VTKMessage, VTKMessageConst);
				strcat(VTKMessage, VTK3DModelFileName);

				GUI.Message(VTKMessage, 600, 100, cvScalar(0, 128, 255));

				bRefresh = true;

				break;
			case 0x00000008:	// Backspace
				iONISample -= ONISpeed;

				bNextImageSelected = true;

				break;
			case 0x00210000:	// PgUp
				if(bKinect && (VS.m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE))
				{
					if(ONISpeed < 100)
						ONISpeed *= 10;
				}

				bRefresh = true;

				break;
			case 0x00220000:	// PgDn
				if(bKinect && (VS.m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE))
				{
					if(ONISpeed > 1)
					{
						ONISpeed /= 10;

						if(ONISpeed < 1)
							ONISpeed = 1;
					}
				}

				bRefresh = true;

				break;
			case 0x00230000:	// End
#ifdef RVLOPENNI
				if(bKinect && (VS.m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE))
				{
					iONISample = VS.m_Kinect.GetNoONIFrames() - 1;

					bNextImageSelected = true;
				}
#endif

				break;
			case 0x00240000:	// Home
				if(bKinect && (VS.m_Kinect.m_Flags & RVLKINECT_FLAG_ONI_FILE))
				{
					iONISample = 0;

					bNextImageSelected = true;
				}

				break;
			case 0x00250000:
				if(VS.m_ConvexSegmentThr > 0)
					VS.m_ConvexSegmentThr--;

				bNextImage = false;

				break;
			case 0x00260000:
				if((VS.m_Flags & RVLSYS_FLAGS_PC) || (VS.m_PSD.m_Flags & RVLPSD_FLAG_MM))
					VS.m_PSD.m_MeshTol++;
				else
					VS.m_PSD.m_uvdTol++;

				bNextImage = false;

				break;
			case 0x00270000:
				VS.m_ConvexSegmentThr++;

				bNextImage = false;

				break;
			case 0x00280000:
				if((VS.m_Flags & RVLSYS_FLAGS_PC) || (VS.m_PSD.m_Flags & RVLPSD_FLAG_MM))
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
			}
		}
		while(bRefresh && !bContinuous);
#endif

		if(GUI.m_bNextImage)
		{
			// get new sample name/ID

			if (!(VS.m_Flags & RVLSYS_FLAGS_KINECT))
				RVLGetNextFileName(VS.m_ImageFileName, "00000-LW.bmp", 10000);
		}

		if(!GUI.m_bRecord)
			VS.m_Mem.Clear();
	}
	while(bContinue);

	// free memory

	fclose(fpExecTime);

	return 0;
}

