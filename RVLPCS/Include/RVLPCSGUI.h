//#pragma once
//#include "RVLDisplayVector.h"
//#include "RVLFigure.h"
//#include "RVLGUI.h"

void RVLPCSDisplayMouseCallback(int event, int x, int y, int flags, void* vpData);

class CRVLPCSGUI :
	public CRVLGUI
{
public:
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
		void *vpVS);
	void Clear();
};

