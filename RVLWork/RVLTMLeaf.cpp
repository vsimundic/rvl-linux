#include "RVLCore.h"
#include "RVLTMLeaf.h"

CRVLTMLeaf::CRVLTMLeaf(void)
{
	m_nNodes = 11;
}

CRVLTMLeaf::~CRVLTMLeaf(void)
{
}

void CRVLTMLeaf::Create(void)
{
	int nLeafParams = 6;

	double scale = 20.0;

	double Leaf0[] = {
		0.0,		0.0,	0.0,	0.0,	0.0,	0.0,
		PI,			0.0,	0.0,	-scale * 1.34,	0.0,	0.0,
		0.0,		0.0,	0.0,	-scale * 0.67,	0.0,	0.0,
		0.0,		0.0,	0.0,	-scale * 1.09,	0.0,	0.0,
		0.0,		0.0,	0.0,	-scale * 1.70,	0.0,	0.0,
		atan2(0.71, -0.52),	0.0,	0.0,	-scale * sqrt(0.69 * 0.69 + 0.94 * 0.94),	0.0,	0.0,
		atan2(-0.71,-0.52),	0.0,	0.0,	-scale * sqrt(0.69 * 0.69 + 0.94 * 0.94),	0.0,	0.0,
		atan2(0.95, -0.06),	0.0,	0.0,	-scale * sqrt(0.09 * 0.09 + 1.34 * 1.34),	0.0,	0.0,
		atan2(-0.95,-0.06),	0.0,	0.0,	-scale * sqrt(0.09 * 0.09 + 1.34 * 1.34),	0.0,	0.0,
		atan2(0.91,  0.43),	0.0,	0.0,	-scale * sqrt(0.46 * 0.46 + 0.96 * 0.96),	0.0,	0.0,
		atan2(-0.91, 0.43),	0.0,	0.0,	-scale * sqrt(0.46 * 0.46 + 0.96 * 0.96),	0.0,	0.0
	};

	double LeafL[] = {
		0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
		-8.0 * DEG2RAD,	-1.0 * DEG2RAD,	-8.0 * DEG2RAD,	-scale * 0.1,	-8.0 * DEG2RAD, -8.0 * DEG2RAD,
		-8.0 * DEG2RAD,	-1.0 * DEG2RAD,	-8.0 * DEG2RAD,	-scale * 0.1,	-8.0 * DEG2RAD, -8.0 * DEG2RAD,
		-8.0 * DEG2RAD,	-1.0 * DEG2RAD,	-8.0 * DEG2RAD,	-scale * 0.1,	-8.0 * DEG2RAD, -8.0 * DEG2RAD,
		-8.0 * DEG2RAD,	-1.0 * DEG2RAD,	-8.0 * DEG2RAD,	-scale * 0.1,	-8.0 * DEG2RAD, -8.0 * DEG2RAD,
		-3.0 * DEG2RAD,	-20.0 * DEG2RAD,	-3.0 * DEG2RAD,		-scale * 0.1,	-8.0 * DEG2RAD, -8.0 * DEG2RAD,
		-3.0 * DEG2RAD,	-20.0 * DEG2RAD,	-3.0 * DEG2RAD,		-scale * 0.1,	-8.0 * DEG2RAD, -8.0 * DEG2RAD,
		-3.0 * DEG2RAD,	-20.0 * DEG2RAD,	-3.0 * DEG2RAD,		-scale * 0.1,	-8.0 * DEG2RAD, -8.0 * DEG2RAD,
		-3.0 * DEG2RAD,	-20.0 * DEG2RAD,	-3.0 * DEG2RAD,		-scale * 0.1,	-8.0 * DEG2RAD, -8.0 * DEG2RAD,
		-3.0 * DEG2RAD,	-20.0 * DEG2RAD,	-3.0 * DEG2RAD,		-scale * 0.1,	-8.0 * DEG2RAD, -8.0 * DEG2RAD,
		-3.0 * DEG2RAD,	-20.0 * DEG2RAD,	-3.0 * DEG2RAD,		-scale * 0.1	-8.0 * DEG2RAD, -8.0 * DEG2RAD
		};

	double LeafH[] = {
		0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
		8.0 * DEG2RAD,	8.0 * DEG2RAD,	8.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD,
		8.0 * DEG2RAD,	8.0 * DEG2RAD,	8.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD,
		8.0 * DEG2RAD,	8.0 * DEG2RAD,	8.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD,
		8.0 * DEG2RAD,	8.0 * DEG2RAD,	8.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD,
		3.0 * DEG2RAD,	20.0 * DEG2RAD,	3.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD,
		3.0 * DEG2RAD,	20.0 * DEG2RAD,	3.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD,
		3.0 * DEG2RAD,	20.0 * DEG2RAD,	3.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD,
		3.0 * DEG2RAD,	20.0 * DEG2RAD,	3.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD,
		3.0 * DEG2RAD,	20.0 * DEG2RAD,	3.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD,
		3.0 * DEG2RAD,	20.0 * DEG2RAD,	3.0 * DEG2RAD,	scale * 0.1,	8.0 * DEG2RAD, 8.0 * DEG2RAD
		};
		
	m_Node = new RVLTMR_MODEL_NODE[11];

	m_Node[0].m_ParentID = -1;
	m_Node[0].m_nChildren = 1;
	m_Node[0].m_Flags = RVLTMR_MODEL_NODE_FLAG_CONTOUR;

	m_Node[1].m_ParentID = 0;
	m_Node[1].m_nChildren = 3;
	m_Node[1].m_iChild = 0;
	m_Node[1].m_Flags = 0x00000000;

	m_Node[2].m_ParentID = 1;
	m_Node[2].m_nChildren = 3;
	m_Node[2].m_iChild = 2;
	m_Node[2].m_Flags = 0x00000000;

	m_Node[3].m_ParentID = 2;
	m_Node[3].m_nChildren = 3;
	m_Node[3].m_iChild = 2;
	m_Node[3].m_Flags = 0x00000000;

	m_Node[4].m_ParentID = 3;
	m_Node[4].m_nChildren = 0;
	m_Node[4].m_iChild = 2;
	m_Node[4].m_Flags = RVLTMR_MODEL_NODE_FLAG_CONTOUR;

	m_Node[5].m_ParentID = 1;
	m_Node[5].m_nChildren = 0;
	m_Node[5].m_iChild = 1;
	m_Node[5].m_Flags = RVLTMR_MODEL_NODE_FLAG_CONTOUR;

	m_Node[6].m_ParentID = 1;
	m_Node[6].m_nChildren = 0;
	m_Node[6].m_iChild = 0;
	m_Node[6].m_Flags = RVLTMR_MODEL_NODE_FLAG_CONTOUR;

	m_Node[7].m_ParentID = 2;
	m_Node[7].m_nChildren = 0;
	m_Node[7].m_iChild = 1;
	m_Node[7].m_Flags = RVLTMR_MODEL_NODE_FLAG_CONTOUR;

	m_Node[8].m_ParentID = 2;
	m_Node[8].m_nChildren = 0;
	m_Node[8].m_iChild = 0;
	m_Node[8].m_Flags = RVLTMR_MODEL_NODE_FLAG_CONTOUR;

	m_Node[9].m_ParentID = 3;
	m_Node[9].m_nChildren = 0;
	m_Node[9].m_iChild = 1;
	m_Node[9].m_Flags = RVLTMR_MODEL_NODE_FLAG_CONTOUR;

	m_Node[10].m_ParentID = 3;
	m_Node[10].m_nChildren = 0;
	m_Node[10].m_iChild = 0;
	m_Node[10].m_Flags = RVLTMR_MODEL_NODE_FLAG_CONTOUR;

	m_ChildLinkScoreMem = new double[10];

	double *pChildLinkScore = m_ChildLinkScoreMem;

	m_pModelPDFData = new unsigned char[6 * 11 * sizeof(RVLPDF_BILATERAL_GAUSSIAN)];

	//m_pParamDataMem = new RVLTMR_MODEL_PARAM[6 * 11];

	RVLPDF_BILATERAL_GAUSSIAN *ModelPDFData = (RVLPDF_BILATERAL_GAUSSIAN *)m_pModelPDFData;

	RVLTMR_MODEL_NODE *pMNode;
	int iNode;
	CRVL3DPose *pPose;
	double *t, *R;
	int iParamSet;
	RVLPDF_BILATERAL_GAUSSIAN *PDF;
	int iParam;

	for(iNode = 0; iNode <= 10; iNode++)
	{
		pMNode = m_Node + iNode;

		//pMNode->m_ID = iNode;

		pPose = &(pMNode->m_PosePN);

		iParamSet = nLeafParams * iNode;

		pPose->m_Alpha = Leaf0[iParamSet + 0];
		pPose->m_Beta  = Leaf0[iParamSet + 1];
		pPose->m_Theta = Leaf0[iParamSet + 2];
		pPose->UpdateRotA0();

		R = pPose->m_Rot;
		t = pPose->m_X;

		t[0] = Leaf0[iParamSet + 3];
		t[1] = t[2] = 0.0;	

		pMNode->m_maxChildLinkScore = pChildLinkScore;

		pChildLinkScore += pMNode->m_nChildren;

		PDF = ModelPDFData + 6 * iNode;

		pMNode->m_vpParamPDFData = (void *)PDF;

		for(iParam = 0; iParam < nLeafParams; iParam++)
		{
			PDF[iParam].mean = Leaf0[iParamSet + iParam];
			PDF[iParam].sigN = LeafL[iParamSet + iParam];
			PDF[iParam].sigP = LeafH[iParamSet + iParam];
		}

		pMNode->m_nParams = 6;
		//pMNode->m_Param = m_pParamDataMem + iNode * 6;

		for(int i = 0; i < 6; i++)
		{
			pMNode->m_Param[i].PDFID = RVLTMR_PDFID_BILATERAL_GAUSSIAN;
			pMNode->m_Param[i].vpPDFData = (void *)(PDF + i);
		}
	}

	RVLTMR_MODEL_NODE *pMParent;
	double cost;
	int iChild;

	for(iNode = 10; iNode >= 1; iNode--)
	{
		pMNode = m_Node + iNode;

		cost = 0.0;

		for(iChild = 0; iChild < pMNode->m_nChildren; iChild++)
			cost += pMNode->m_maxChildLinkScore[iChild];

		pMParent = m_Node + pMNode->m_ParentID;

		iParamSet = nLeafParams * iNode;

		//pMParent->m_maxChildLinkScore[pMNode->m_iChild] = cost + 
		//	RVLTMR_PoseProbability(Leaf0[iParamSet + 0] + PI, 0.0, 0.0, 200.0, Leaf0[iParamSet + 4] + PI, 0.0,
		//	Leaf0 + iParamSet, LeafL + iParamSet, LeafH + iParamSet);
		pMParent->m_maxChildLinkScore[pMNode->m_iChild] = cost 
			+ log(0.5 * (LeafH[iParamSet + 0] - LeafL[iParamSet + 0]))
			+ log(0.5 * (LeafH[iParamSet + 3] - LeafL[iParamSet + 3]))
			+ log(0.5 * (LeafH[iParamSet + 4] - LeafL[iParamSet + 4]))
			+ 1.5 * RVLLN2PI
			+ 3.0 * 4.5;
	}	

	m_nContourPts = 8;

	m_Contour = new DWORD[m_nContourPts];

	m_Contour[0] = 0;
	m_Contour[1] = 5;
	m_Contour[2] = 7;
	m_Contour[3] = 9;
	m_Contour[4] = 4;
	m_Contour[5] = 10;
	m_Contour[6] = 8;
	m_Contour[7] = 6;

	AllocateMem();
}

void RVLDisplayTMInstance(	IplImage *pImage,
							RVLTMR_NODE *pSNode,
							RVLTMR_MODEL_NODE *Model)
{
	RVLTMR_MODEL_NODE *pMNode = Model + pSNode->m_ID;

	int iChild;
	RVLTMR_NODE *pChild;

	for(iChild = 0; iChild < pMNode->m_nChildren; iChild++)
	{
		pChild = pSNode->m_Child[iChild].pNode;

		if(pChild)
		{
			int u1 = DOUBLE2INT(pSNode->m_PoseNC.m_X[0]);
			int v1 = DOUBLE2INT(pSNode->m_PoseNC.m_X[1]);
			int u2 = DOUBLE2INT(pChild->m_PoseNC.m_X[0]);
			int v2 = DOUBLE2INT(pChild->m_PoseNC.m_X[1]);

			cvLine(pImage, cvPoint(u1, v1), cvPoint(u2, v2), cvScalar(0, 0, 255));

			RVLDisplayTMInstance(pImage, pSNode->m_Child[iChild].pNode, Model);
		}
	}
}
