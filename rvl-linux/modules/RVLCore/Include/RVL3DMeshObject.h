#pragma once

#include "RVL3DObject.h"

#define RVL_MESH_HOG_1D					0
#define RVL_MESH_HOG_2D					1
#define RVL_MESH_HOG_3D					2

#define RVL_MESH_COLOR_RGB				1
#define RVL_MESH_COLOR_RGB_N_2D			2
#define RVL_MESH_COLOR_HSV_2D			3
#define RVL_MESH_COLOR_XYZ				4
#define RVL_MESH_COLOR_CIELAB_2D		5
#define RVL_MESH_COLOR_RGB_N_3D			6
#define RVL_MESH_COLOR_HSV_3D			7
#define RVL_MESH_COLOR_CIELAB_3D		8
#define RVL_MESH_COLOR_NORM_TRANS_3D	9	
#define RVL_MESH_COLOR_OPPONENT_2D		10
#define RVL_MESH_COLOR_OPPONENT_3D		11

//#define RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG	

class CRVL3DMeshObject;

CRVL3DMeshObject* GenMeshObjects(CRVLMPtrChain *pTriangleList, 
								 IplImage *pImg, 
								 int nObjects, 
								 CRVLClass *pClass);


class CRVL3DMeshObject : public CRVL3DObject
{
public:
	RVLQLIST *m_FaceList;
	int m_noFaces;
	int m_noVertices;
	int m_noUsedColorPts;
	int m_noUsedLbpPts;
	RVLQLIST *m_histRGB;
	float m_histRGB_base[3];
	int m_ColorSystem;
	int m_noHistDimensions;
	RVLQLIST *m_rgbHOG9;
	int m_TextureLabel;
	//Za sada beskorisno RVLQLIST *m_shapeD2;
	float *m_ADSD;
	int m_ShapeLabel;
	double *m_Ellipsoid;		
	void *rootMeshObject;
	void *parentMeshObject;
	RVLQLIST *m_ChildMeshObjects;
	int m_lbpP;	//broj tocaka u okolini za LBP deskriptor
	int m_lbpR;	//radius oko tocke za LBP deskriptor
	RVLQLIST *m_LBP;	//Osnovni LBP deskriptor
	RVLQLIST *m_LBP_RIU;	//Rotacijski invarijantni LBP deskriptor
	RVLQLIST *m_LBP_RIU_VAR;	//Rotacijski invarijantni 2D LBP deskriptor s varijancom
	RVLQLIST m_TextonList;		//Lista textona
	RVLQLIST *m_TextonHist;		//Histogram na bazi velicina i orjentacija textona

	CRVL3DMeshObject(void);
	virtual ~CRVL3DMeshObject(void);
	void Init(); //added by Karlo
	void InitParent();
	void InitChild();
	void RVLCalculateRGBHist(IplImage* rgbImg, float* histBase, bool soft = false);
	void RVLCalculateColorHist(IplImage* colorImg, float* histBase, bool soft = false);
	void RVLCalculateHSVHist(IplImage* hsvImg, float* histBase, bool soft = false);
	void RVLCalculateLabHist(IplImage* labImg, float* histBase, bool soft = false);
	void RVLCalculateXYZHist(IplImage* xyzImg, float* histBase, bool soft = false);
	void RVLCalculateRGCHist(IplImage* rgbImg, float* histBase, bool soft = false);
	void RVLCalculateRGBNTHist(IplImage* rgbImg, float* histBase, bool soft = false);
	void RVLCalculateRGBOppHist(IplImage* rgbImg, float* histBase, bool soft = false);
	void RVLCalculateRGBHOG(IplImage* rgbImg,
							float *r_x,
							float *r_y,
							float *g_x,
							float *g_y,
							float *b_x,
							float *b_y,
							float maxMagnitude = 150.0,
							int HOG_TYPE = RVL_MESH_HOG_2D);
	void RVLCalculateGrayHOG(IplImage* grayImg,
							float *x,
							float *y,
							float *angle,
							float *magnitude, 
							bool *map,
							float maxMagnitude = 150.0,
							int HOG_TYPE = RVL_MESH_HOG_2D);
	void RVLCalculateTextureLabel();
	void RVLCalculateEllipsoidDescriptor(RVL3DPOINT2 **Point3DMap);
	//Za sada beskorisno void RVLCalculateD2ShapeDistribution(RVL3DPOINT2 **Point3DMap);
	void RVLCalculateADSD(RVL3DPOINT2 **Point3DMap, float useRatio);
	void RVLCalculateShapeLabel();
	float RVLIntersectColorHistogram(CRVL3DMeshObject *model, int usedBins = 5, bool hard = false, float *helperArray = NULL);
	static float RVLIntersectHistograms(RVLQLIST *histS,
										RVLQLIST *histM,
										int noUsedPtsS,
										int noUsedPtsM,
										int maxNoBins,
										CRVLMem *pMem,
										int usedBins = -1,
										float *helperArray = NULL);
	
	void RVLIncreaseRGBHistBinSizeSYM(float ratio);
	static RVLQLIST* RVLIncreaseRGBHistBinSizeSYM(RVLQLIST* histRGB, float* oldHistBase, float ratio);
	void SaveMeshObject2Ply(FILE *fpPly,
					  RVLQLIST *pTriangleList,
					  RVL3DPOINT2 **Point3DMap,
					  DWORD Flags = 0x00000000,
					  int min3DPtsPerc = 0,
					  DWORD Mask = 0x00000000);
	void SaveMeshObject2Ply2(FILE *fpPly,
					   RVLQLIST *pTriangleList,
					   RVL3DPOINT2 **Point3DMap,
					   DWORD Flags = 0x00000000,
					   int min3DPtsPerc = 0,
					   DWORD Mask = 0x00000000);
	void SaveMeshObject2OFF(FILE *fpOFF,
					  RVLQLIST *pTriangleList,
					  RVL3DPOINT2 **Point3DMap,
					  DWORD Flags = 0x00000000,
					  int min3DPtsPerc = 0,
					  DWORD Mask = 0x00000000);
	void SaveMeshObject2OBJ(FILE *fpOBJ,
					  FILE *fpMTL,
					  char* mtlFileName,
					  RVL3DPOINT2 **Point3DMap,
					  int colorType = 0,
					  char* texFileName = NULL,
					  DWORD Flags = 0x00000000,
					  int min3DPtsPerc = 0,
					  DWORD Mask = 0x00000000);
	void AppendMeshObject2OBJ(	int &iPt, 
								int &br, 
								CRVL3DPose *pPose,
								FILE *fpOBJ,
								FILE *fpMTL,
								char* mtlFileName,
								RVL3DPOINT2 **Point3DMap,
								int colorType,
								char* texFileName,
								DWORD Flags = 0x00000000,
								int min3DPtsPerc = 0,
								DWORD Mask = 0x00000000);
	void Save(	FILE *fp,
				DWORD Flags = 0x00000000);
	void Load(	FILE *fp,
				DWORD Flags = 0x00000000);
	void CalculateLbp(int p,
					  int r,
					  IplImage* grayImg,
					  bool soft = false);
	void CalculateLbpRiu(int p,
						 int r,
						 IplImage* grayImg);
	void CalculateLbpRiuVar(int p,
							int r,
							float maxVar,
							IplImage* grayImg,
							bool soft = false);
	static void GetPointNeighbourhood(int *neighbourhood,
									  int x,
									  int y,
									  int p,
									  int r,
									  IplImage* grayImg);
	static int GetBiLinInterPixVal(float x,
								   float y,
								   IplImage* grayImg);
	void SaveRGBDSegmentMasks(char* rgbMaskPath,
							  char* depthMaskPath,
							  RVL3DPOINT2 **Point3DMap,
							  int width = 320,
							  int height = 240);
	float RVLCupecRGBProbMeasure(CRVL3DMeshObject *model, CRVL3DMeshObject *scene, int usedBins = 5);
	void CalculateTextonHistogram(float lambC1,
								  float lambC2 = 2.0,
								  float lambCap = 100.0,
								  int lambBins = 10,
								  int angBins = 9);
	float CalculateTextonHistogram2(float lambC,
								   float elongC,
								   float lambCap = 100.0,
								   float elongCap = 10.0,
								   int lambBins = 10,
								   int elongBins = 10);
};
