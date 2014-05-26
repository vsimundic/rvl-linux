#include "RVLCore.h"
#include "RVLTM.h"
#include <time.h>

CRVLTM::CRVLTM(void)
{
	m_Node = NULL;
	m_ChildLinkScoreMem = NULL;
	m_pModelPDFData = NULL;
	m_CellArray = NULL;
	//m_pParamDataMem = NULL;
	m_Contour = NULL;
	m_NodeMem = NULL;
	m_ChildPtrMem = NULL;
	m_ModelFileName = NULL;
}

CRVLTM::~CRVLTM(void)
{
	if(m_Node)
		delete[] m_Node;

	if(m_ChildLinkScoreMem)
		delete[] m_ChildLinkScoreMem;

	if(m_pModelPDFData)
		delete[] m_pModelPDFData;

	if(m_CellArray)
		delete[] m_CellArray;

	//if(m_pParamDataMem)
	//	delete[] m_pParamDataMem;

	if(m_Contour)
		delete[] m_Contour;

	if(m_NodeMem)
		delete[] m_NodeMem;

	if(m_ChildPtrMem)
		delete[] m_ChildPtrMem;

	if(m_ModelFileName)
		delete[] m_ModelFileName;
}

void CRVLTM::AllocateMem(void)
{
	m_nParents = 0;
	
	for(int i = 0; i < m_nNodes; i++)
		if(m_Node[i].m_nChildren > 0)
			m_nParents++;

	m_CellArray = new CRVL2DCellArray[m_nParents];

	int iParent = 0;
	
	for(int i = 0; i < m_nNodes; i++)
	{
		if(m_Node[i].m_nChildren > 0)
		{
			m_CellArray[iParent].m_pMem = m_pMem;
			m_CellArray[iParent].m_Width = m_Width;
			m_CellArray[iParent].m_Height = m_Height;
			m_CellArray[iParent].m_CellSize = 8;
			m_CellArray[iParent].Init();

			iParent++;
		}
	}
}

void CRVLTM::CreateInstance(RVLTMR_NODE *Instance)
{
	RVLTMR_NODE *pSNode = Instance;

	pSNode->m_ID = 0;
	double *R = pSNode->m_PoseNC.m_Rot;
	double *t = pSNode->m_PoseNC.m_X;

	RVLUNITMX3(R)
	RVLNULL3VECTOR(t)

	int iNode, iParam;
	double minParamVal, maxParamVal;
	double dVal;
	CRVL3DPose ParentPose;
	double *RPN, *tPN;
	double RNP[3 * 3], tNP[3];
	double *RPC, *tPC;
	RVLTMR_MODEL_NODE *pMNode;
	double cs, sn;
	RVLTMR_MODEL_PARAM *pParam;
	double *ParamVal;

	for(iNode = 1; iNode < m_nNodes; iNode++)
	{
		pSNode = Instance + iNode;

		pSNode->m_ID = iNode;

		pMNode = m_Node + pSNode->m_ID;

		ParamVal = new double[pMNode->m_nParams];

		for(iParam = 0; iParam < pMNode->m_nParams; iParam++)
		{
			pParam = pMNode->m_Param + iParam;

			RVLPDF_BILATERAL_GAUSSIAN *pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)(pParam->vpPDFData);

			double PDFParams[3];

			PDFParams[0] = 0.0;
			PDFParams[1] = pPDF->sigN;
			PDFParams[2] = pPDF->sigP;
			minParamVal = 3.0 * pPDF->sigN;
			maxParamVal = 3.0 * pPDF->sigP;
			dVal = 1e-3 * (maxParamVal - minParamVal);

			ParamVal[iParam] = pPDF->mean + RVLRandPDF(RVLBilateralGaussianPDF, PDFParams, minParamVal, maxParamVal, dVal);
		}

		ParentPose.m_Alpha = ParamVal[0];
		//ParentPose.m_Beta  = ParamVal[1];
		//ParentPose.m_Theta = ParamVal[2];
		ParentPose.m_Beta  = ParentPose.m_Theta = 0.0;
		ParentPose.UpdateRotA0();

		RPN = ParentPose.m_Rot;
		tPN = ParentPose.m_X;

		cs = cos(ParamVal[4]);
		sn = sin(ParamVal[4]);

		tPN[0] = ParamVal[3] * cs;
		tPN[1] = ParamVal[3] * sn;
		tPN[2] = 0.0;

		RVLINVTRANSF3D(RPN, tPN, RNP, tNP)

		pSNode->m_pParent = Instance + pMNode->m_ParentID;

		RPC = pSNode->m_pParent->m_PoseNC.m_Rot;
		tPC = pSNode->m_pParent->m_PoseNC.m_X;

		R = pSNode->m_PoseNC.m_Rot;
		t = pSNode->m_PoseNC.m_X;

		RVLCOMPTRANSF3D(RPC, tPC, RNP, tNP, R, t);

		delete[] ParamVal;
	}
}

void CRVLTM::DisplayInstance(	IplImage *pImage,
								RVLTMR_NODE *Instance,
								double fu0,
								double fv0,
								CvScalar colorContour,
								CvScalar colorTree,
								bool bTree)
{
	int u1, v1, u2, v2;
	int iNode;
	RVLTMR_NODE *pSNode;
	double *t;
	double *tPC;

	if(bTree)
	{
		for(iNode = 1; iNode < m_nNodes; iNode++)
		{
			pSNode = Instance + iNode;

			t = pSNode->m_PoseNC.m_X;

			tPC = pSNode->m_pParent->m_PoseNC.m_X;

			u1 = DOUBLE2INT(t[0] + fu0);
			v1 = DOUBLE2INT(t[1] + fv0);
			u2 = DOUBLE2INT(tPC[0] + fu0);
			v2 = DOUBLE2INT(tPC[1] + fv0);

			cvLine(pImage, cvPoint(u1, v1), cvPoint(u2, v2), colorTree); 
		}
	}

	pSNode = Instance + m_Contour[0];

	t = pSNode->m_PoseNC.m_X;

	u1 = DOUBLE2INT(t[0] + fu0);
	v1 = DOUBLE2INT(t[1] + fv0);
	
	for(int i = 1; i <= m_nContourPts; i++)
	{
		pSNode = Instance + m_Contour[i % m_nContourPts];

		t = pSNode->m_PoseNC.m_X;

		u2 = DOUBLE2INT(t[0] + fu0);
		v2 = DOUBLE2INT(t[1] + fv0);

		cvLine(pImage, cvPoint(u1, v1), cvPoint(u2, v2), colorContour, 2);

		u1 = u2;
		v1 = v2;
	}
}
bool CRVLTM::Load()
{
	FILE *fp = fopen(m_ModelFileName, "r");

	if(fp == NULL)
		return false;

	// first pass

	double scale;

	fscanf(fp, "scale=%lf\n\n", &scale);

	int nParamsTotal = 0;
	int ParamPDFDataMemSize = 0;

	m_nNodes = 0;

	int iNode;
	int nParams;
	int iTmp;
	int iParam;
	char line[200];
	char sTmp[200];

	while(!feof(fp))
	{
		fgets(line, 200, fp);

		if(strcmp(line, "contour:\n") == 0)
		{
			m_nContourPts = 0;

			do
			{
				if(fscanf(fp, "%d ", &iTmp))
					m_nContourPts++;
			}
			while(!feof(fp));

			break;
		}

		sscanf(line, "%d: Parent=%d child_idx=%d #children=%d #params=%d #DOF=%d %s\n", &iNode, &iTmp, &iTmp, &iTmp, &nParams, &iTmp, sTmp);

		nParamsTotal += nParams;

		for(iParam = 0; iParam < nParams; iParam++)
		{
			fgets(line, 200, fp);

			switch(line[3]){
			case 'G':
				ParamPDFDataMemSize += sizeof(RVLPDF_BILATERAL_GAUSSIAN);

				break;
			default:
				ParamPDFDataMemSize += sizeof(RVLPDF_BILATERAL_GAUSSIAN);
			}
		}

		fscanf(fp, "\n");

		m_nNodes++;
	}

	fclose(fp);

	m_Node = new RVLTMR_MODEL_NODE[m_nNodes];
	//m_pParamDataMem = new RVLTMR_MODEL_PARAM[nParamsTotal];
	//m_pParamDataMem = new RVLTMR_MODEL_PARAM[m_nNodes * RVLTMR_MODEL_N_PARAMS];
	m_pModelPDFData = new unsigned char[ParamPDFDataMemSize];
	m_ChildLinkScoreMem = new double[m_nNodes - 1];
	m_Contour = new DWORD[m_nContourPts];

	double *pChildLinkScore = m_ChildLinkScoreMem;

	// second pass

	fp = fopen(m_ModelFileName, "r");

	fscanf(fp, "scale=%lf\n\n", &scale);

	//RVLTMR_MODEL_PARAM *Param = m_pParamDataMem;

	unsigned char *pPDFData = m_pModelPDFData;

	int iNode_;
	RVLTMR_MODEL_NODE *pNode;
	double r;
	//double az, el;
	double *tPN;
	int sLen;
	RVLTMR_MODEL_PARAM *pParam;
	DWORD ParamID;
	double range;

	for(iNode_ = 0; iNode_ < m_nNodes; iNode_++)
	{
		fgets(line, 200, fp);

		sscanf(line, "%d: ", &iNode);

		pNode = m_Node + iNode;

		sscanf(line, "%d: Parent=%d child_idx=%d #children=%d #params=%d #DOF=%d %s\n", &iTmp, &(pNode->m_ParentID), 
			&(pNode->m_iChild), &(pNode->m_nChildren), &(pNode->m_nParams), &iTmp, sTmp);

		pNode->m_Flags = 0x00000000;

		if(sTmp[0] == 'C')
			pNode->m_Flags |= RVLTMR_MODEL_NODE_FLAG_CONTOUR;

		sLen = strlen(sTmp);

		if(sLen > 1)
			if(sTmp[1] == 'S')
				pNode->m_Flags |= RVLTMR_MODEL_NODE_FLAG_SYMMETRY;

		if(iTmp == 5)
			pNode->m_Flags |= RVLTMR_MODEL_NODE_FLAG_5DOF;

		pNode->m_vpParamPDFData = (void *)pPDFData;

		for(iParam = 0; iParam < pNode->m_nParams; iParam++)
		{
			fgets(line, 200, fp);

			RVLPDF_BILATERAL_GAUSSIAN *pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)pPDFData;

			sscanf(line + 5, "%lf %lf %lf %lf", &(pPDF->mean), &(pPDF->sigN), &(pPDF->sigP), &range);

			switch(line[0]){
			case 'A':
				switch(line[1]){
				case 'L':
					ParamID = RVLTMR_MODEL_PARAM_ID_ALPHA;

					//pParam->ID = RVLTMR_MODEL_PARAM_ID_ALPHA;
					
					//pNode->m_PosePN.m_Alpha = pPDF->mean * DEG2RAD;

					break;
				case 'Z':
					ParamID = RVLTMR_MODEL_PARAM_ID_AZ;
					//pParam->ID = RVLTMR_MODEL_PARAM_ID_AZ;
					//az = pPDF->mean * DEG2RAD;
				}

				break;
			case 'B':
				if(line[1] == 'E')
				{
					ParamID = RVLTMR_MODEL_PARAM_ID_BETA;

					//pParam->ID = RVLTMR_MODEL_PARAM_ID_BETA;
					
					//pNode->m_PosePN.m_Beta = pPDF->mean * DEG2RAD;
				}

				break;
			case 'D':
				switch(line[1]){
				case 'I':
					ParamID = RVLTMR_MODEL_PARAM_ID_DI;

					//pParam->ID = RVLTMR_MODEL_PARAM_ID_DI;

					r = pPDF->mean * scale;
				
					break;
				case 'Z':
					ParamID = RVLTMR_MODEL_PARAM_ID_DZ;
					//pParam->ID = RVLTMR_MODEL_PARAM_ID_DZ;
				}

				break;
			case 'E':
				switch(line[1]){
				case 'L':
					ParamID = RVLTMR_MODEL_PARAM_ID_EL;

					//pParam->ID = RVLTMR_MODEL_PARAM_ID_EL;

					//el = pPDF->mean * DEG2RAD;

					break;
				case 'X':
					ParamID = RVLTMR_MODEL_PARAM_ID_EX;
					//pParam->ID = RVLTMR_MODEL_PARAM_ID_EX;
				}

				break;
			case 'G':
				if(line[1] == 'A')
					ParamID = RVLTMR_MODEL_PARAM_ID_GA;
					//pParam->ID = RVLTMR_MODEL_PARAM_ID_GA;

				break;
			case 'T':
				if(line[1] == 'H')
				{
					ParamID = RVLTMR_MODEL_PARAM_ID_THETA;

					//pParam->ID = RVLTMR_MODEL_PARAM_ID_THETA;

					//pNode->m_PosePN.m_Theta = pPDF->mean * DEG2RAD;
				}

				break;
			case 'X':
				if(line[1] == 'Y')
					ParamID = RVLTMR_MODEL_PARAM_ID_XPYN;

				break;
			case 'Y':
				switch(line[1]){
				case 'X':
					ParamID = RVLTMR_MODEL_PARAM_ID_YPXN;

					//pParam->ID = RVLTMR_MODEL_PARAM_ID_YX;

					break;
				case 'Z':
					ParamID = RVLTMR_MODEL_PARAM_ID_YPZN;
					//pParam->ID = RVLTMR_MODEL_PARAM_ID_YZ;
				}

				break;
			case 'Z':
				if(line[1] == 'Y')
					ParamID = RVLTMR_MODEL_PARAM_ID_ZPYN;
			}

			pParam = pNode->m_Param + ParamID;

			pParam->vpPDFData = pPDF;

			pParam->PDFID = RVLTMR_PDFID_BILATERAL_GAUSSIAN;

			if(ParamID <= RVLTMR_MODEL_PARAM_ID_GA)
			{
				pPDF->mean *= DEG2RAD;
				pPDF->sigN *= DEG2RAD;
				pPDF->sigP *= DEG2RAD;
				range *= DEG2RAD;
			}
			else if(ParamID <= RVLTMR_MODEL_PARAM_ID_DI)
			{
				pPDF->mean *= scale;
				pPDF->sigN *= scale;
				pPDF->sigP *= scale;
				range *= scale;
			}

			pPDF->k = log(range) - log(0.5 * (pPDF->sigP - pPDF->sigN)) - 0.5 * RVLLN2PI;

			pPDFData += sizeof(RVLPDF_BILATERAL_GAUSSIAN);
		}

		//pNode->m_PosePN.UpdateRotA0();

		tPN = pNode->m_PosePN.m_X;

		tPN[0] = r;
		tPN[1] = tPN[2] = 0.0;

		pNode->m_maxChildLinkScore = pChildLinkScore;

		pChildLinkScore += pNode->m_nChildren;

		fscanf(fp, "\n");
	}

	fgets(line, 200, fp);

	for(int i = 0; i < m_nContourPts; i++)
		fscanf(fp, "%d ", m_Contour + i);

	fclose(fp);

	//// compute maxChildLinkScores

	//RVLTMR_MODEL_NODE *pParent;
	//double cost;
	//int iChild;

	//for(iNode = m_nNodes - 1; iNode >= 1; iNode--)
	//{
	//	pNode = m_Node + iNode;

	//	cost = 0.0;

	//	for(iChild = 0; iChild < pNode->m_nChildren; iChild++)
	//		cost += pNode->m_maxChildLinkScore[iChild];

	//	pParent = m_Node + pNode->m_ParentID;

	//	for(iParam = 0; iParam < pNode->m_nParams; iParam++)
	//	{
	//		pParam = pNode->m_Param + iParam;

	//		RVLPDF_BILATERAL_GAUSSIAN *pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)(pParam->vpPDFData);

	//		cost += (0.5  * RVLLN2PI + 4.5 + log(0.5 * (pPDF->sigP - pPDF->sigN)));
	//	}

	//	pParent->m_maxChildLinkScore[pNode->m_iChild] = cost;
	//}

	AllocateMem();

	return true;
}

RVLTMR_NODE *CRVLTM::Detect(CvPoint *Contour,
							int nContourPts,
							int &nSNodes)
{
	int ContourWin = 3;
	int ParentSearchWin = 7;
	double SampleOrientThr = 20.0 * DEG2RAD;
	int SampleDistThr = 5;

	RVLTMR_MODEL_NODE *Model = m_Node;

	int nPts = 0;

	double *Phi = new double[nContourPts];

	CvPoint *pPt, *pPt1, *pPt2;
	double fdu, fdv;
	double cs, sn;
	double fTmp;
	int iPt;

	for(iPt = 0; iPt < nContourPts; iPt++)
	{
		pPt1 = Contour + (iPt + nContourPts - ContourWin) % nContourPts;
		pPt2 = Contour + (iPt + ContourWin) % nContourPts;
		fdu = (double)(pPt2->x - pPt1->x);
		fdv = (double)(pPt2->y - pPt1->y);
		fTmp = sqrt(fdu * fdu + fdv * fdv);
		cs = -fdv / fTmp;
		sn = fdu / fTmp;

		Phi[iPt] = atan2(sn, cs);
	}

	bool *bSample = new bool[nContourPts];

	memset(bSample, 0, nContourPts * sizeof(bool));

	int iLastSample = -SampleDistThr;

	double ePhi;

	for(iPt = 0; iPt < nContourPts; iPt++)
	{
		if(iPt - iLastSample < SampleDistThr)
		{
			ePhi = Phi[iPt] - Phi[(iPt + 2) % nContourPts];

			if(ePhi > PI)
				ePhi -= (2.0 * PI);
			else if(ePhi < -PI)
				ePhi += (2.0 * PI);

			if(fabs(ePhi) < SampleOrientThr)
			{
				ePhi = Phi[iPt] - Phi[(iPt + nContourPts - 2) % nContourPts];

				if(ePhi > PI)
					ePhi -= (2.0 * PI);
				else if(ePhi < -PI)
					ePhi += (2.0 * PI);	

				if(fabs(ePhi) < SampleOrientThr)
					continue;
			}
		}

		iLastSample = iPt;

		bSample[iPt] = true;

		nPts++;
	}

	delete[] Phi;

	// Initialize TMR

	int *nContourChildren = new int[m_nNodes];

	memset(nContourChildren, 0, m_nNodes * sizeof(int));

	nSNodes = 0;

	int maxnChildren = 0;
	
	for(int i = 0; i < m_nNodes; i++)
	{
		if(Model[i].m_Flags & RVLTMR_MODEL_NODE_FLAG_CONTOUR)
		{
			if(Model[i].m_ParentID >= 0)
			{
				nContourChildren[Model[i].m_ParentID]++;
				nSNodes++;
			}

			nSNodes++;
		}

		if(Model[i].m_nChildren > maxnChildren)
			maxnChildren = Model[i].m_nChildren;
	}

	nSNodes *= nPts;

	if(m_NodeMem)
		delete[] m_NodeMem;

	m_NodeMem = new RVLTMR_NODE[nSNodes];

	RVLTMR_NODE **Node = new RVLTMR_NODE *[m_nNodes];

	int *nNodes = new int[m_nNodes];

	int *ChildLinkBlockStart = new int[m_nNodes];

	CRVL2DCellArray *CellArray = m_CellArray;

	for(int i = 0; i < m_nParents; i++)
		CellArray[i].RemoveAll();

	Node[0] = m_NodeMem;

	int nChildLinks = 0;
	int iParent = 0;
	
	for(int i = 0; i < m_nNodes; i++)
	{
		nNodes[i] = (Model[i].m_Flags & RVLTMR_MODEL_NODE_FLAG_CONTOUR ? 1 : nContourChildren[i]) * nPts;

		if(i < m_nNodes - 1)
			Node[i + 1] = Node[i] + nNodes[i];

		ChildLinkBlockStart[i] = nChildLinks;

		nChildLinks += (nNodes[i] * Model[i].m_nChildren);
	}

	delete[] nContourChildren;

	if(m_ChildPtrMem)
		delete[] m_ChildPtrMem;

	m_ChildPtrMem = new RVLTMR_CHILD_LINK[nChildLinks];

	delete[] nNodes;

#ifdef RVLTMR_DEBUG
	//RVLTMR_NODE *pDebugNode;
	//RVLTMR_NODE *pDebugParentNode;
	FILE *fpDebug = fopen("debug.txt", "w");
#endif
	clock_t t;

	t = clock();

	double RCN[3 * 3];
	double *XNC = RCN;
	double *YNC = RCN + 3;
	double *ZNC = RCN + 6;

	double RCP[3 * 3];
	double *XPC = RCP;
	double *YPC = RCP + 3;
	double *ZPC = RCP + 6;

	int iNode = 0;

	DWORD ID, IDParent;
	int NodeDataAddress;
	RVLTMR_NODE *pNode, *pParent;
	RVLTMR_MODEL_NODE *pMNode, *pMParent;
	double *tPN, *RNC, *tNC, *RPC, *tPC;
	int u, v;
	double V0[3], V1[3], U0[3];
	double gamma, r;
	RVLPDF_BILATERAL_GAUSSIAN *pPDF;
	double ParentLinkScore;
	RVLTMR_CHILD_LINK *pChildLink;

	for(iPt = 0; iPt < nContourPts; iPt++)
	{
		if(!bSample[iPt])
			continue;

		pPt = Contour + iPt;

		for(ID = 0; ID < (DWORD)(m_nNodes); ID++)
		{
			pMNode = Model + ID;

			if((pMNode->m_Flags & RVLTMR_MODEL_NODE_FLAG_CONTOUR) == 0)
				continue;

			pNode = Node[ID] + iNode;

			pNode->m_ID = ID;
			pNode->m_Flags = 0x00000000;
			pPt1 = Contour + (iPt + nContourPts - ContourWin) % nContourPts;
			pPt2 = Contour + (iPt + ContourWin) % nContourPts;
			fdu = (double)(pPt2->x - pPt1->x);
			fdv = (double)(pPt2->y - pPt1->y);
			fTmp = sqrt(fdu * fdu + fdv * fdv);
			cs = -fdv / fTmp;
			sn = fdu / fTmp;
			RNC = pNode->m_PoseNC.m_Rot;
			tNC = pNode->m_PoseNC.m_X;
			RVLROTZ(cs, sn, RNC)
			RVLCOPYCOLMX3X3(RNC, 1, YNC)
			tNC[0] = (double)(pPt->x);
			tNC[1] = (double)(pPt->y);
			tNC[2] = 0.0;
			pNode->m_PoseNC.m_Alpha = atan2(sn, cs);
			pNode->m_PoseNC.m_Beta = pNode->m_PoseNC.m_Theta = 0.0;			

#ifdef RVLTMR_DEBUG
			//if(ID == 4 && pPt->x == 181 && pPt->y == 75)
			//	pDebugNode = pNode;
			fprintf(fpDebug, "N%d(CID%d): (%3.0lf, %3.0lf) AL=%4.0lf\n", pNode - m_NodeMem, pNode->m_ID, tNC[0], tNC[1], 
				atan2(RNC[3], RNC[0]) * RAD2DEG, pNode->m_score);
#endif

			if(pMNode->m_ParentID >= 0)
			{
				IDParent = pMNode->m_ParentID;

				NodeDataAddress = iNode + nPts * pMNode->m_iChild;

				pParent = Node[IDParent] + NodeDataAddress;

#ifdef RVLTMR_DEBUG
				if(pParent - m_NodeMem == 1627)
					int debug = 0;
#endif

				pParent->m_ID = IDParent;

				if(pMNode->m_iChild == 0)
					pParent->m_Flags = RVLTMR_NODE_FLAG_CHILD0;
				else if(pMNode->m_iChild == 1)
					pParent->m_Flags = RVLTMR_NODE_FLAG_CHILD1;
				else
					pParent->m_Flags = RVLTMR_NODE_FLAG_ORIENTATION;

				pMParent = Model + IDParent;

				//RPN = pMNode->m_PosePN.m_Rot;
				tPN = pMNode->m_PosePN.m_X;					

				RPC = pParent->m_PoseNC.m_Rot;
				tPC = pParent->m_PoseNC.m_X;

				//RVLCOMPTRANSF3D(RNC, tNC, RPN, tPN, RPC, tPC)
				RVLMULMX3X3VECT(RNC, tPN, tPC)
				RVLSUM3VECTORS(tPC, tNC, tPC)

				u = DOUBLE2INT(tPC[0]);
				v = DOUBLE2INT(tPC[1]);

				CellArray[IDParent].Add(u, v, pParent);

				//pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)(pMNode->m_Param[RVLTMR_MODEL_PARAM_ID_DI].vpPDFData);

				//r = -pPDF->mean;

				//RVLDIF3VECTORS(tNC, tPC, V0)
				//RVLSCALE3VECTOR2(V0, r, V0)

				pParent->m_Child = m_ChildPtrMem + ChildLinkBlockStart[IDParent] + pMParent->m_nChildren * NodeDataAddress;

				for(int i = 0; i < pMParent->m_nChildren; i++)
				{
					//pParent->m_Child[i].score = pMParent->m_maxChildLinkScore[i];
					pParent->m_Child[i].score = 0.0;
					pParent->m_Child[i].pNode = NULL;
				}

				if(pMNode->m_iChild > 1)
				{
					pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)(pMNode->m_Param[RVLTMR_MODEL_PARAM_ID_DI].vpPDFData);

					r = -pPDF->mean;

					RVLDIF3VECTORS(tNC, tPC, V0)
					RVLSCALE3VECTOR2(V0, r, V0)

					RVLCOPY3VECTOR(V0, XPC)
					RVLCOPYCOLMX3X3(RNC, 1, U0)
					RVLCROSSPRODUCT3(V0, U0, ZPC)
					RVLNORM3(ZPC, fTmp)
					RVLCROSSPRODUCT3(ZPC, XPC, YPC)
					RVLCOPYMX3X3T(RCP, RPC)
				}
				else
				{
					pChildLink = pParent->m_Child + pMNode->m_iChild;

					pChildLink->pNode = pNode;
					pChildLink->score = RVLTMR_ParentPoseProbability(pParent, pNode, Model);

				//	RVLCROSSPRODUCT3(V0, YNC, ZNC)
				//	RVLNORM3(ZNC, fTmp)

				//	if(pMNode->m_iChild == 0)
				//	{
				//		RVLCROSSPRODUCT3(V0, ZNC, U0)
				//	}
				//	else
				//	{
				//		RVLCROSSPRODUCT3(ZNC, V0, U0)
				//	}

				//	pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)(pMParent->m_Param[RVLTMR_MODEL_PARAM_ID_GA].vpPDFData);

				//	gamma = pPDF->mean;

				//	cs = cos(gamma);
				//	sn = sin(gamma);

				//	RVLMXEL(RPC, 3, 0, 0) = cs * V0[0] + sn * U0[0];
				//	RVLMXEL(RPC, 3, 1, 0) = cs * V0[1] + sn * U0[1];
				//	RVLMXEL(RPC, 3, 2, 0) = cs * V0[2] + sn * U0[2];
				}

#ifdef RVLTMR_DEBUG
				fprintf(fpDebug, "P: %d(CID%d): (%3.0lf, %3.0lf) AL=%4.0lf\n", pParent - m_NodeMem, pParent->m_ID, 
					pParent->m_PoseNC.m_X[0], pParent->m_PoseNC.m_X[1], 
					atan2(pParent->m_PoseNC.m_Rot[3], pParent->m_PoseNC.m_Rot[0]) * RAD2DEG);
#endif
			}	// if(pMNode->m_ParentID >= 0)
			else
			{
				u = DOUBLE2INT(tNC[0]);
				v = DOUBLE2INT(tNC[1]);

				CellArray[0].Add(u, v, pNode);

				pNode->m_Child = m_ChildPtrMem + ChildLinkBlockStart[ID] + pMNode->m_nChildren * iNode;

				//pNode->m_Child[0].score = pMNode->m_maxChildLinkScore[0];
				pNode->m_Child[0].score = 0.0;
				pNode->m_Child[0].pNode = NULL;
			}
		}	// for(ID = 0; ID < (DWORD)(m_nNodes); ID++)

		iNode++;
	}	// for(iPt = 0; iPt < nContourPts; iPt++)

	delete[] bSample;

#ifdef RVLTMR_DEBUG
	fprintf(fpDebug, "\n");
#endif

	//// Display nodes

	//cvCopyImage(pInputImage, pDisplay);

	//for(iPt = 0; iPt < nSNodes; iPt++)
	//{
	//	pNode = m_NodeMem + iPt;

	//	if(pNode->m_ID == 2)
	//	{
	//		u = DOUBLE2INT(pNode->m_PoseNC.m_X[0]);
	//		v = DOUBLE2INT(pNode->m_PoseNC.m_X[1]);

	//		cvRectangle(pDisplay, cvPoint(u-1, v-1), cvPoint(u+1, v+1), cvScalar(0, 0, 0));
	//	}
	//}

	//cvShowImage("TMR", pDisplay);

	//cvSaveImage("debug.bmp", pDisplay);

	//cvWaitKey();

	// Complete the poses of the symmetry nodes

	void **ParentCandidateArray = new void *[maxnChildren * nPts];

	pNode = m_NodeMem + nSNodes - 1;

	double tPCref[3];
	int nParentCandidates;
	int iParentCandidate;
	double *R0C, *t0C, *R1C, *t1C;
	double Z0C[3], Z1C[3], Y0C[3], Y1C[3], XNC0[3], XNC1[3], DZ[3], DX[3];
	double dz, ex;	
	RVLTMR_MODEL_PARAM *ParamPtrArray[RVLTMR_MODEL_N_PARAMS];
	double ParamVal[RVLTMR_MODEL_N_PARAMS];
	double NodeScore;
	RVLTMR_NODE *pChild0, *pChild1;

	while(true)
	{
		pMNode = Model + pNode->m_ID;

		if((pMNode->m_Flags & RVLTMR_MODEL_NODE_FLAG_CONTOUR) == 0)
			break;

		pMParent = Model + pMNode->m_ParentID;

		if((pMParent->m_Flags & RVLTMR_MODEL_NODE_FLAG_SYMMETRY) == 0 || pMNode->m_iChild > 1)
		{
			pNode--;

			continue;
		}

#ifdef RVLTMR_DEBUG
		fprintf(fpDebug, "N%d(SID%d): (%3.0lf, %3.0lf) AL=%4.0lf\n", pNode - m_NodeMem, pNode->m_ID, 
			pNode->m_PoseNC.m_X[0], pNode->m_PoseNC.m_X[1], 
			atan2(pNode->m_PoseNC.m_Rot[3], pNode->m_PoseNC.m_Rot[0]) * RAD2DEG);

		//if(pNode - m_NodeMem == 2672)
		//	int debug = 0;
#endif

		RNC = pNode->m_PoseNC.m_Rot;
		tNC = pNode->m_PoseNC.m_X;							

		tPN = pMNode->m_PosePN.m_X;	

		RVLMULMX3X3VECT(RNC, tPN, tPCref)
		RVLSUM3VECTORS(tPCref, tNC, tPCref)

		u = DOUBLE2INT(tPCref[0]);
		v = DOUBLE2INT(tPCref[1]);

		CellArray[pMNode->m_ParentID].Get(u - ParentSearchWin, u + ParentSearchWin, v - ParentSearchWin, v + ParentSearchWin, 
			ParentCandidateArray, nParentCandidates);

		for(iParentCandidate = 0; iParentCandidate < nParentCandidates; iParentCandidate++)
		{
			pParent = (RVLTMR_NODE *)(ParentCandidateArray[iParentCandidate]);

#ifdef RVLTMR_DEBUG
			if(pNode - m_NodeMem == 3034 && pParent - m_NodeMem == 1244)
				int debug = 0;
#endif

			if((pParent->m_Flags & (RVLTMR_NODE_FLAG_CHILD0 | RVLTMR_NODE_FLAG_CHILD1)) == 0)
				continue;

			if(pMNode->m_iChild == 0)
			{
				if(pParent->m_Flags & RVLTMR_NODE_FLAG_CHILD0)
					continue;
				else
				{
					pChild0 = pNode;
					pChild1 = pParent->m_Child[1].pNode;
				}
			}
			else
			{
				if(pParent->m_Flags & RVLTMR_NODE_FLAG_CHILD1)
					continue;
				else
				{
					pChild0 = pParent->m_Child[0].pNode;
					pChild1 = pNode;
				}
			}

			ParentLinkScore = RVLTMR_ParentPoseProbability(pParent, pNode, Model);

			RPC = pParent->m_PoseNC.m_Rot;
			tPC = pParent->m_PoseNC.m_X;

			R0C = pChild0->m_PoseNC.m_Rot;
			RVLCOPYCOLMX3X3(R0C, 1, Y0C)
			t0C = pChild0->m_PoseNC.m_X;
			RVLDIF3VECTORS(t0C, tPC, V0)
			RVLNORM3(V0, fTmp)

			RVLCROSSPRODUCT3(V0, Y0C, Z0C)
			RVLNORM3(Z0C, fTmp)
			
			R1C = pChild1->m_PoseNC.m_Rot;
			RVLCOPYCOLMX3X3(R1C, 1, Y1C)
			t1C = pChild1->m_PoseNC.m_X;
			RVLDIF3VECTORS(t1C, tPC, V1)
			RVLNORM3(V1, fTmp)

			RVLDIF3VECTORS(V0, V1, YPC)
			fTmp = sqrt(RVLDOTPRODUCT3(YPC, YPC));
			if(fTmp > -1e-10 && fTmp < 1e-10)
				continue;
			RVLNORM3(YPC, fTmp)
			
			RVLCROSSPRODUCT3(V1, Y1C, Z1C)
			RVLNORM3(Z1C, fTmp)

			RVLCROSSPRODUCT3(YPC, Z0C, XNC0)
			RVLNORM3(XNC0, fTmp)

			RVLCROSSPRODUCT3(YPC, Z1C, XNC1)
			RVLNORM3(XNC1, fTmp)

			RVLDIF3VECTORS(Z0C, Z1C, DZ)

			dz = sqrt(RVLDOTPRODUCT3(DZ, DZ));

			RVLSUM3VECTORS(XNC0, XNC1, XPC)
			RVLNORM3(XPC, fTmp)

			RVLDIF3VECTORS(XNC0, XNC1, DX)

			ex = sqrt(RVLDOTPRODUCT3(DX, DX));

			RVLCROSSPRODUCT3(XPC, YPC, ZPC)			

			gamma = atan2(RVLDOTPRODUCT3(V0, YPC), RVLDOTPRODUCT3(V0, XPC));

			ParamVal[0] = gamma;
			ParamPtrArray[0] = pMParent->m_Param + RVLTMR_MODEL_PARAM_ID_GA;
			ParamVal[1] = dz;
			ParamPtrArray[1] = pMParent->m_Param + RVLTMR_MODEL_PARAM_ID_DZ;
			ParamVal[2] = ex;
			ParamPtrArray[2] = pMParent->m_Param + RVLTMR_MODEL_PARAM_ID_EX;

			NodeScore = RVLTMR_Probability(ParamVal, ParamPtrArray, 3);

			ParentLinkScore += NodeScore;

			pChildLink = pParent->m_Child + pMNode->m_iChild;

#ifdef RVLTMR_DEBUG
			fprintf(fpDebug, "P%d(%d): (%3.0lf, %3.0lf) AL=%4.0lf LS=%lf BLS=%lf\n", pParent - m_NodeMem, pParent->m_ID, 
				pParent->m_PoseNC.m_X[0], pParent->m_PoseNC.m_X[1], 
				atan2(pParent->m_PoseNC.m_Rot[3], pParent->m_PoseNC.m_Rot[0]) * RAD2DEG, ParentLinkScore, pChildLink->score);
#endif

			if(ParentLinkScore > pChildLink->score)
			{
				pChildLink->pNode = pNode;
				pChildLink->score = ParentLinkScore;

				RVLCOPYMX3X3T(RCP, RPC)

				pParent->m_Flags |= RVLTMR_NODE_FLAG_ORIENTATION;
			}
		}

		pNode--;
	}

	int iChild;
	RVLTMR_MODEL_NODE *pMChild;

	while(pNode >= m_NodeMem)
	{	
		pMNode = Model + pNode->m_ID;

		if((pMNode->m_Flags & RVLTMR_MODEL_NODE_FLAG_SYMMETRY) == 0)
		{
			pNode--;

			continue;
		}

		pChild0 = pNode->m_Child[0].pNode;
		pChild1 = pNode->m_Child[1].pNode;

		if(pChild0 == NULL)
		{
			pChild0 = pChild1;
			pChild1 = NULL;
		}

		if(pChild0 == NULL || pChild1 != NULL)
		{
			pNode--;

			continue;
		}

		RNC = pNode->m_PoseNC.m_Rot;
		tNC = pNode->m_PoseNC.m_X;

		R0C = pChild0->m_PoseNC.m_Rot;
		RVLCOPYCOLMX3X3(R0C, 1, Y0C)
		t0C = pChild0->m_PoseNC.m_X;
		RVLDIF3VECTORS(t0C, tNC, V0)
		RVLNORM3(V0, fTmp)

		RVLCROSSPRODUCT3(V0, Y0C, Z0C)
		RVLNORM3(Z0C, fTmp)

//		if(pChild1)
//		{
//			R1C = pChild1->m_PoseNC.m_Rot;
//			RVLCOPYCOLMX3X3(R1C, 1, Y1C)
//			t1C = pChild1->m_PoseNC.m_X;
//			RVLDIF3VECTORS(t1C, tNC, V1)
//			RVLNORM3(V1, fTmp)
//
//			RVLDIF3VECTORS(V0, V1, YNC)
//			RVLNORM3(YNC, fTmp)
//			
//			RVLCROSSPRODUCT3(V1, Y1C, Z1C)
//			RVLNORM3(Z1C, fTmp)
//
//			RVLCROSSPRODUCT3(YNC, Z0C, XNC0)
//			RVLNORM3(XNC0, fTmp)
//
//			RVLCROSSPRODUCT3(YNC, Z1C, XNC1)
//			RVLNORM3(XNC1, fTmp)
//
//			RVLDIF3VECTORS(Z0C, Z1C, DZ)
//
//			dz = sqrt(RVLDOTPRODUCT3(DZ, DZ));
//
//			RVLSUM3VECTORS(XNC0, XNC1, XNC)
//			RVLNORM3(XNC, fTmp)
//
//			RVLDIF3VECTORS(XNC0, XNC1, DX)
//
//			ex = sqrt(RVLDOTPRODUCT3(DX, DX));
//
//			RVLCROSSPRODUCT3(XNC, YNC, ZNC)
//
//			RVLCOPYMX3X3T(RCN, RNC)
//
//			gamma = atan2(RVLDOTPRODUCT3(V0, YNC), RVLDOTPRODUCT3(V0, XNC));
//
//			ParamVal[0] = gamma;
//			ParamPtrArray[0] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_GA;
//			ParamVal[1] = dz;
//			ParamPtrArray[1] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_DZ;
//			ParamVal[2] = ex;
//			ParamPtrArray[2] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_EX;
//
//			pNode->m_score = RVLTMR_Probability(ParamVal, ParamPtrArray, 3);
//
//#ifdef RVLTMR_DEBUG
//			fprintf(fpDebug, "N%d(SID%d): (%3.0lf, %3.0lf) AL=%4.0lf C0=%d C1=%d\n", pNode - m_NodeMem, pNode->m_ID, 
//				pNode->m_PoseNC.m_X[0], pNode->m_PoseNC.m_X[1], 
//				atan2(pNode->m_PoseNC.m_Rot[3], pNode->m_PoseNC.m_Rot[0]) * RAD2DEG,
//				pChild0 - m_NodeMem, pChild1 - m_NodeMem);
//#endif
//		}
//		else
//		{
			pMChild = Model + pChild0->m_ID;

			if(pMChild->m_iChild == 0)
			{
				RVLCROSSPRODUCT3(V0, Z0C, U0)
			}
			else
			{
				RVLCROSSPRODUCT3(Z0C, V0, U0)
			}

			pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)(pMNode->m_Param[RVLTMR_MODEL_PARAM_ID_GA].vpPDFData);

			gamma = pPDF->mean;

			cs = cos(gamma);
			sn = sin(gamma);

			XNC[0] = cs * V0[0] + sn * U0[0];
			XNC[1] = cs * V0[1] + sn * U0[1];
			XNC[2] = cs * V0[2] + sn * U0[2];

			RVLCROSSPRODUCT3(Z0C, XNC, U0)

			pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)(pMNode->m_Param[RVLTMR_MODEL_PARAM_ID_DZ].vpPDFData);

			dz = pPDF->mean;

			cs = sqrt(1.0 - dz * dz);

			YNC[0] = dz * Z0C[0] + cs * U0[0];
			YNC[1] = dz * Z0C[1] + cs * U0[1];
			YNC[2] = dz * Z0C[2] + cs * U0[2];

			RVLCROSSPRODUCT3(XNC, YNC, ZNC)

			RVLCOPYMX3X3T(RCN, RNC)

#ifdef RVLTMR_DEBUG
			fprintf(fpDebug, "N%d(SID%d): (%3.0lf, %3.0lf) AL=%4.0lf C=%d\n", pNode - m_NodeMem, pNode->m_ID, 
				pNode->m_PoseNC.m_X[0], pNode->m_PoseNC.m_X[1], 
				atan2(pNode->m_PoseNC.m_Rot[3], pNode->m_PoseNC.m_Rot[0]) * RAD2DEG,
				pChild0 - m_NodeMem);
#endif
		//}

		pNode->m_Flags |= RVLTMR_NODE_FLAG_ORIENTATION;

		pNode--;
	}

#ifdef RVLTMR_DEBUG
	fprintf(fpDebug, "\n");
#endif

	// Forward update

	pNode = m_NodeMem + nSNodes - 1;

	CRVL3DPose PosePN;

	while(pNode >= m_NodeMem)
	{		
		pMNode = Model + pNode->m_ID;

		pMParent = Model + pMNode->m_ParentID;
		
		if(pMNode->m_Flags & RVLTMR_MODEL_NODE_FLAG_CONTOUR)
			if(pMParent->m_Flags & RVLTMR_MODEL_NODE_FLAG_SYMMETRY)
				if(pMNode->m_iChild <= 1)
				{
					pNode--;

					continue;
				}

		pNode->m_score = 0.0;
		
		for(iChild = 0; iChild < pMNode->m_nChildren; iChild++)
			pNode->m_score += pNode->m_Child[iChild].score;

		if(pNode->m_ID > 0)
		{
			RNC = pNode->m_PoseNC.m_Rot;
			tNC = pNode->m_PoseNC.m_X;							

			tPN = pMNode->m_PosePN.m_X;	

			RVLMULMX3X3VECT(RNC, tPN, tPCref)
			RVLSUM3VECTORS(tPCref, tNC, tPCref)

			u = DOUBLE2INT(tPCref[0]);
			v = DOUBLE2INT(tPCref[1]);

#ifdef RVLTMR_DEBUG
			//if(pNode == pDebugNode)
			//	int debug = 0;

			if(pNode - m_NodeMem == 354)
				int debug = 0;

			fprintf(fpDebug, "N%d(ID%d): (%3.0lf, %3.0lf) AL=%4.0lf S=%lf\n", pNode - m_NodeMem, pNode->m_ID, tNC[0], tNC[1], 
				atan2(RNC[3], RNC[0]) * RAD2DEG, pNode->m_score);
#endif
			
			CellArray[pMNode->m_ParentID].Get(u - ParentSearchWin, u + ParentSearchWin, v - ParentSearchWin, v + ParentSearchWin, 
				ParentCandidateArray, nParentCandidates);

#ifdef RVLTMR_DEBUG
			double debugParentLinkScore = 0.0;
#endif

			for(iParentCandidate = 0; iParentCandidate < nParentCandidates; iParentCandidate++)
			{
				pParent = (RVLTMR_NODE *)(ParentCandidateArray[iParentCandidate]);

				ParentLinkScore = RVLTMR_ParentPoseProbability(pParent, pNode, Model);

				ParentLinkScore += pNode->m_score;

				pChildLink = pParent->m_Child + pMNode->m_iChild;

#ifdef RVLTMR_DEBUG
				//if(pNode == pDebugNode)
				//	if(ParentLinkScore > debugParentLinkScore)
				//	{
				//		pDebugParentNode = pParent;
				//		debugParentLinkScore = ParentLinkScore;
				//	}
				fprintf(fpDebug, "P: %d(%d): (%3.0lf, %3.0lf) AL=%4.0lf LS=%lf BLS=%lf\n", pParent - m_NodeMem, pParent->m_ID, 
					pParent->m_PoseNC.m_X[0], pParent->m_PoseNC.m_X[1], 
					atan2(pParent->m_PoseNC.m_Rot[3], pParent->m_PoseNC.m_Rot[0]) * RAD2DEG,
					ParentLinkScore, pChildLink->score);
#endif
				if(ParentLinkScore > pChildLink->score)
				{
					pChildLink->pNode = pNode;
					pChildLink->score = ParentLinkScore;
				}
			}

#ifdef RVLTMR_DEBUG
			//if(pNode == pDebugNode)
			//	pDebugNode = pDebugParentNode;
#endif
		}

		pNode--;
	}

	t = clock() - t;

	float ExecTime = ((float)t)/CLOCKS_PER_SEC;

#ifdef RVLTMR_DEBUG
	fclose(fpDebug);
#endif

	delete[] Contour;
	delete[] ParentCandidateArray;
	delete[] Node;
	delete[] ChildLinkBlockStart;

	return m_NodeMem;
}

void CRVLTM::CreateParamList(CRVLMem * pMem)
{
	m_ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	m_ParamList.Init();

	pParamData = m_ParamList.AddParam("TMR.ModelFileName", RVLPARAM_TYPE_STRING, &m_ModelFileName);
}

///////////////////////////////////// 
//
//     Global Functions
//
///////////////////////////////////// 

double RVLTMR_PoseProbability(	double Alpha,
								double Beta,
								double Theta,
								double s,
								double az,
								double el,
								RVLPDF_BILATERAL_GAUSSIAN *PDF)
{
	double eAlpha = RVLLogBilateralGaussianPDFAngle(Alpha, PDF[0].mean, PDF[0].sigN, PDF[0].sigP);
	//double eBeta = RVLLogBilateralGaussianPDFAngle(Beta,  Val0_[1], mindVal_[1], maxdVal_[1]);
	//double eTheta = RVLLogBilateralGaussianPDFAngle(Theta, Val0_[2], mindVal_[2], maxdVal_[2]);
	double es = RVLLogBilateralGaussianPDF(s, PDF[3].mean, PDF[3].sigN, PDF[3].sigP);
	double eaz = RVLLogBilateralGaussianPDFAngle(az, PDF[4].mean, PDF[4].sigN, PDF[4].sigP);
	//double eel = RVLLogBilateralGaussianPDFAngle(el, Val0_[5], mindVal_[5], maxdVal_[5];

	return eAlpha + es + eaz;
}

double RVLTMR_Probability(double *ParamVal,
						  RVLTMR_MODEL_PARAM **ParamPtrArray,
						  int nParams)
{
	RVLPDF_BILATERAL_GAUSSIAN *pPDF;
	RVLTMR_MODEL_PARAM *pParam;

	double score = 0.0;

	int iParam;
	double P;

	for(iParam = 0; iParam < nParams; iParam++)
	{
		pParam = ParamPtrArray[iParam];

		pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)(pParam->vpPDFData);

		P = pPDF->k + RVLLogBilateralGaussianPDF(ParamVal[iParam], pPDF->mean, pPDF->sigN, pPDF->sigP);

		score += P;
	}

	return score;
}

double RVLTMR_ParentPoseProbability(RVLTMR_NODE *pParent, 
									RVLTMR_NODE *pSNode,
									RVLTMR_MODEL_NODE *Model)
{
	RVLTMR_MODEL_NODE *pMNode = Model + pSNode->m_ID;

	RVLTMR_MODEL_NODE *pMParent = Model + pParent->m_ID;

	double *RNC = pSNode->m_PoseNC.m_Rot;
	double *tNC = pSNode->m_PoseNC.m_X;

	double *RPC = pParent->m_PoseNC.m_Rot;
	double *tPC = pParent->m_PoseNC.m_X;

	double r;

	int iParam = 0;

	double ParamVal[RVLTMR_MODEL_N_PARAMS];
	RVLTMR_MODEL_PARAM *ParamPtrArray[RVLTMR_MODEL_N_PARAMS];
	double fTmp;

	if(pMNode->m_Flags & RVLTMR_MODEL_NODE_FLAG_5DOF)
	{
		double dt[3];

		RVLDIF3VECTORS(tPC, tNC, dt)

		r = sqrt(RVLDOTPRODUCT3(dt, dt));

		ParamVal[iParam] = -r;
		ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_DI;
		iParam++;

		fTmp = RVLMULVECTORCOL3(dt, RNC, 1) / r;
		if(fTmp > 1.0)
			fTmp = 1.0;
		else if(fTmp < -1.0)
			fTmp = -1.0;
		ParamVal[iParam] = asin(fTmp);
		ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_AZ;
		iParam++;

		if((pMParent->m_Flags & RVLTMR_MODEL_NODE_FLAG_SYMMETRY) != 0 && pMNode->m_iChild <= 1)
		{
			//RVLPDF_BILATERAL_GAUSSIAN *pPDF = (RVLPDF_BILATERAL_GAUSSIAN *)(pMParent->m_Param[RVLTMR_MODEL_PARAM_ID_GA].vpPDFData);

			//double gamma = pPDF->mean;

			//double V0[3];

			//RVLDIF3VECTORS(tNC, tPC, V0)

			//RVLNORM3(V0, fTmp)

			//fTmp = RVLMULVECTORCOL3(V0, RPC, 0);
			//if(fTmp > 1.0)
			//	fTmp = 1.0;
			//else if(fTmp < -1.0)
			//	fTmp = -1.0;
			//ParamVal[iParam] = acos(fTmp);
			//ParamPtrArray[iParam] = pMParent->m_Param + RVLTMR_MODEL_PARAM_ID_GA;
			//iParam++;
		}
		else if(pParent->m_Flags & RVLTMR_NODE_FLAG_ORIENTATION)
		{
			ParamVal[iParam] = RVLMULCOLCOL3(RNC, RPC, 1, 0);
			ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_XPYN;
			iParam++;

			ParamVal[iParam] = RVLMULCOLCOL3(RNC, RPC, 1, 2);
			ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_ZPYN;
			iParam++;
		}
	}
	else
	{
		CRVL3DPose PosePN;
		double tmp3x1[3];

		double *tPN = PosePN.m_X;

		RVLDIF3VECTORS(tPC, tNC, tmp3x1)
		RVLMULMX3X3TVECT(RNC, tmp3x1, tPN)

		if(pMParent->m_Flags & RVLTMR_MODEL_NODE_FLAG_5DOF)
		{
			ParamVal[iParam] = RVLMULCOLCOL3(RNC, RPC, 0, 1);
			ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_YPXN;
			iParam++;

			ParamVal[iParam] = RVLMULCOLCOL3(RNC, RPC, 2, 1);
			ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_YPZN;
			iParam++;
		}
		else
		{
			double *RPN = PosePN.m_Rot;

			RVLMXMUL3X3T1(RNC, RPC, RPN)\

			ParamVal[iParam] = atan2(RPN[3], RPN[0]);
			ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_ALPHA;
			iParam++;

			ParamVal[iParam] = 0.0;
			ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_BETA;
			iParam++;

			ParamVal[iParam] = 0.0;
			ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_THETA;
			iParam++;
		}

		ParamVal[iParam] = -sqrt(RVLDOTPRODUCT3(tPN, tPN));
		ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_DI;
		iParam++;

		ParamVal[iParam] = atan2(-tPN[1], -tPN[0]);
		ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_AZ;
		iParam++;

		ParamVal[iParam] = atan(tPN[2] / sqrt(tPN[0] * tPN[0] + tPN[1] * tPN[1]));
		ParamPtrArray[iParam] = pMNode->m_Param + RVLTMR_MODEL_PARAM_ID_EL;
		iParam++;
	}

	return RVLTMR_Probability(ParamVal, ParamPtrArray, iParam);
}
