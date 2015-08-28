//#pragma once
//#include "RVLDisplayVector.h"
//#include "RVLFigure.h"
//#include "RVLGUI.h"

void RVLPCSDisplayMouseCallback(int event, int x, int y, int flags, void* vpData);

class CRVLPCSGUI :
	public CRVLGUI
{
public:
	void *m_vpVS;
	CRVLFigure *m_pFig;
	IplImage *m_pInputImage;
	int m_DisplayBitmap;
	RVLDISPARITYMAP *m_pDepthImage;
	DWORD m_DepthMapFormat;
	IplImage *m_pRGBImage;
	IplImage *m_pHSVImage;
	IplImage *m_pGSImage;
	IplImage *m_pSegmentationImage;
	IplImage *m_pZoomedInputImage;
	bool m_bNextImage;
	bool m_bDisplayMesh;
	bool m_bDisplayConvexSets;
	bool m_bDisplaySelectedObjects;
	bool m_bRecord;
	bool m_bKinect;
	bool m_bContinuous;
	int m_iONISample;
	int m_ONISpeed;
	double m_ExecTime;
	int m_ZoomFactor;
	int m_ImageWidth;
	CRVLVTKRenderer m_Renderer;
	int m_w;
	int m_h;
	CRVLClass m_Class;
	int *m_PointMap;

public:
	CRVLPCSGUI();
	virtual ~CRVLPCSGUI();
	bool InteractiveVisualization();
	void Init(
		void *vpVS,
		bool bKinect = false);
	void Clear();
};

