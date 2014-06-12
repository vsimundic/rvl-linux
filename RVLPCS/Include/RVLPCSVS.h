#pragma once

#define RVLSYS_FLAGS_PSD						0x00000100
#define RVLSYS_FLAGS_DELAUNAY					0x00000200
#define RVLSYS_FLAGS_PC							0x00000400
#define RVLSYS_FLAGS_SEGMENT_TO_CONVEX_SETS		0x00000800

#define RVLSYS_MCMEMSIZE		2

//#define RVLSYS_PSULMBRLM_UPDATE_LOG_FILE

class CRVLPCSVS : public CRVLVisionSystem 
{
public:
	//CRVLScene m_Scene;
	CRVLPlanarSurfaceDetector m_PSD;	
	CRVLDelaunay *m_pDelaunay;	
	int m_ConvexSegmentThr;

public:
	CRVLPCSVS();
	virtual ~CRVLPCSVS();
	DWORD Init(char *CfgFile2Name = NULL);
	void Clear();
	void CreateParamList();
};
