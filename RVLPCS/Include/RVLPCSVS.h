#pragma once

#define RVLSYS_FLAGS_PSD						0x00000100
#define RVLSYS_FLAGS_DELAUNAY					0x00000200
#define RVLSYS_FLAGS_SEGMENT_MESH				0x00000800
#define RVLSYS_FLAGS_SEGMENT_TO_CONVEX_SETS		0x00001000
#define RVLSYS_FLAGS_SEGMENT_GRAPH				0x00002000

#define RVLSYS_MCMEMSIZE		2

//#define RVLSYS_PSULMBRLM_UPDATE_LOG_FILE

struct RVLPCS_DISPLAY
{
	CRVLGUI *m_pGUI;
	CRVLFigure *m_pFig;
	IplImage *m_pInputImage;
	int m_DisplayBitmap;
	RVLDISPARITYMAP *m_pDepthImage;
	DWORD m_DepthMapFormat;
	IplImage *m_pRGBImage;
	IplImage *m_pGSImage;
	IplImage *m_pSegmentationImage;
	IplImage *m_pZoomedInputImage;
	bool m_bDisplayMesh;
	bool m_bDisplayConvexSets;
	bool m_bDisplaySelectedObjects;
	bool m_bRecord;
	bool m_bKinect;
	int m_iONISample;
	double m_ExecTime;
	int m_ZoomFactor;
	int m_ImageWidth;
};

class CRVLPCSVS : public CRVLVisionSystem 
{
public:
	//CRVLScene m_Scene;
	CRVLPlanarSurfaceDetector m_PSD;	
	CRVLDelaunay *m_pDelaunay;	
	int m_ConvexSegmentThr;
	//RVLPCS_DISPLAY m_Display;
	int m_nObjects;

public:
	CRVLPCSVS();
	virtual ~CRVLPCSVS();
	DWORD Init(char *CfgFile2Name = NULL);
	void Clear();
	void CreateParamList();
	void Display(CRVLPCSGUI *pGUI);
	bool InputRGBDImageFromFile(
		RVLDISPARITYMAP *DepthImage,
		IplImage *pRGBImage,
		char *RGBExtension,
		char *pDepthExtension);
	void Segment();
	bool InputRGBDImageFromPCDFile(
		RVLDISPARITYMAP *pDepthImage,
		IplImage *pRGBImage);
	bool InputFromFile(
		RVLDISPARITYMAP *pDepthImage = NULL,
		IplImage *pRGBImage = NULL);
};
