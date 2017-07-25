#pragma once

#define RVLKINECT_FLAG_DEVICE	0x00000001
#define RVLKINECT_FLAG_ONI_FILE	0x00000002
#define RVLKINECT_FLAG_100UM	0x00000004

#define RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY	0
#define RVLKINECT_DEPTH_IMAGE_FORMAT_1MM		1
#define RVLKINECT_DEPTH_IMAGE_FORMAT_100UM		2

#define RVLKINECTZTODEPTH(z)	(2842.5 * (atan(z/123.6) - 1.1863))
#define RVLKINECTDEPTHTOZ(d)	(123.6*tan((double)d / 2842.5 + 1.1863))

char * RVLKinectCreateONISampleFileName(char *ONIFileName, 
										int iSample, 
										char *Extension);

class CRVLKinect
{
public:
	CRVLKinect(void);
	virtual ~CRVLKinect(void);
#ifdef RVLOPENNI
	bool Init(char *ONIFileName = NULL);
	bool GetImages(	short *pDepth,
					IplImage *pImageRGB = NULL,						
					IplImage *pImageDepth = NULL, 
					IplImage *pImageGS = NULL,
					unsigned int Format = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY,
					int frameIdx = 0,
					void *vpDepthFrame = NULL,
					void *vpColorFrame = NULL);
	void RegisterDepthToColor(bool bRegister);
	void GetParams();
	void ConvertDepthToWorld(int u, int v, int z, float *px, float *py, float *pz);
	void ConvertDepthToColor(int u, int v, int z, int *puRGB, int *pvRGB);
	int GetNoONIFrames(void);
	void SetPlaybeckSpeed(float speed);
#endif

public:
	unsigned int m_Flags;
	void *m_vpDevice;
	void *m_vpDepthStream;
	void *m_vpColorStream;
	void *m_vpPlaybackControl;
	void *m_vpCoordinateConverter;
	void *m_vpStream[2];
	short *m_zToDepthLookupTable;
	double m_fu;
	double m_fv;
	double m_uc;
	double m_vc;
	int m_scale;
	int m_RGBscale;
	char *m_ONIFileName;
};
