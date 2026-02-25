#pragma once

#define RVLTMR_MODEL_NODE_FLAG_CONTOUR	0x00000001
#define RVLTMR_MODEL_NODE_FLAG_5DOF		0x00000002
#define RVLTMR_MODEL_NODE_FLAG_SYMMETRY	0x00000004

#define RVLTMR_NODE_FLAG_ORIENTATION	0x00000001
#define RVLTMR_NODE_FLAG_CHILD0			0x00000002
#define RVLTMR_NODE_FLAG_CHILD1			0x00000004

#define RVLTMR_MODEL_N_PARAMS		13
#define RVLTMR_MODEL_PARAM_ID_ALPHA	0
#define RVLTMR_MODEL_PARAM_ID_BETA	1
#define RVLTMR_MODEL_PARAM_ID_THETA	2
#define RVLTMR_MODEL_PARAM_ID_AZ	3
#define RVLTMR_MODEL_PARAM_ID_EL	4
#define RVLTMR_MODEL_PARAM_ID_GA	5
#define RVLTMR_MODEL_PARAM_ID_DI	6
#define RVLTMR_MODEL_PARAM_ID_YPXN	7
#define RVLTMR_MODEL_PARAM_ID_YPZN	8
#define RVLTMR_MODEL_PARAM_ID_DZ	9
#define RVLTMR_MODEL_PARAM_ID_EX	10
#define RVLTMR_MODEL_PARAM_ID_XPYN	11
#define RVLTMR_MODEL_PARAM_ID_ZPYN	12

#define RVLTMR_PDFID_BILATERAL_GAUSSIAN	0

//#define RVLTMR_OLD_METHOD
//#define RVLTMR_DEBUG

struct RVLTMR_MODEL_PARAM
{
	DWORD PDFID;
	void *vpPDFData;
};

struct RVLTMR_MODEL_NODE
{
	//DWORD m_ID;
	DWORD m_Flags;
	CRVL3DPose m_PosePN;
	int m_ParentID;
	int m_iChild;
	int m_nChildren;
	double *m_maxChildLinkScore;
	int m_nParams;
	RVLTMR_MODEL_PARAM m_Param[RVLTMR_MODEL_N_PARAMS];
	void *m_vpParamPDFData;
	//RVLQLIST m_ChildList;
};

struct RVLTMR_NODE;

struct RVLTMR_CHILD_LINK
{
	RVLTMR_NODE *pNode;
	double score;
};

struct RVLTMR_NODE
{
	DWORD m_Flags;
	DWORD m_ID;
	CRVL3DPose m_PoseNC;
	RVLTMR_NODE *m_pParent;
	RVLTMR_CHILD_LINK *m_Child;
	double m_score;
};

double RVLTMR_PoseProbability(	double Alpha,
								double Beta,
								double Theta,
								double s,
								double az,
								double el,
								RVLPDF_BILATERAL_GAUSSIAN *PDF);
double RVLTMR_ParentPoseProbability(RVLTMR_NODE *pParent, 
									RVLTMR_NODE *pSNode,
									RVLTMR_MODEL_NODE *Model);
double RVLTMR_Probability(double *ParamVal,
						  RVLTMR_MODEL_PARAM **ParamPtrArray,
						  int nParams);

class CRVLTM
{
public:
	CRVLTM(void);
	virtual ~CRVLTM(void);
	void CreateInstance(RVLTMR_NODE *Instance);
	void DisplayInstance(	IplImage *pImage,
							RVLTMR_NODE *Instance,
							double fu0,
							double fv0,
							CvScalar colorContour,
							CvScalar colorTree,
							bool bTree = TRUE);
	bool Load();
	RVLTMR_NODE *Detect(CvPoint *Contour,
						int nContourPts,
						int &nNodes);

protected:
	void AllocateMem(void);

public:
	CRVLMem *m_pMem;
	RVLTMR_MODEL_NODE *m_Node;
	int m_nNodes;
	int m_nParents;
	CRVL2DCellArray *m_CellArray;
	double *m_ChildLinkScoreMem;
	int m_Width;
	int m_Height;
	DWORD *m_Contour;
	int m_nContourPts;
	CRVLParameterList m_ParamList;
	char *m_ModelFileName;

protected:
	unsigned char *m_pModelPDFData;	
	//RVLTMR_MODEL_PARAM *m_pParamDataMem;
	RVLTMR_NODE *m_NodeMem;	
	RVLTMR_CHILD_LINK *m_ChildPtrMem;
public:
	void CreateParamList(CRVLMem * pMem);
};
