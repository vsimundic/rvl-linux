//#include "stdafx.h"
#include "RVLCore2.h"
#ifndef RVLLINUX
#include <Windows.h>
#endif
#include <algorithm>
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "MSTree.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "ReconstructionEval.h"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "ObjectDetector.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
#include "NeighborhoodTool.h"
#include "VN.h"
#include "VNClass.h"
#include "VNInstance.h"
#include "VNClassifier.h"

#define RVLVN_PART_SEGMENTATION_MATCH_LOG
//#define RVLVN_PART_SEGMENTATION_CTINET_LOG

using namespace RVL;
using namespace RECOG;

VNInstance::VNInstance()
{
	bConcave = NULL;
	d = NULL;
	bd = NULL;
	iPrimitiveClass = NULL;
	R = NULL;
	q = NULL;
	s = NULL;
	surfelCells.Element = NULL;
	//conditionalProbability.Element = NULL;
	//priorProbability.Element = NULL;
	componentSurfelCells = NULL;
	//surfelCellObjectMem = NULL;
	componentSurfelCellMem = NULL;
	PC.Element = NULL;
	iCTIElement = NULL;
	TREDMx = NULL;
	TREDCluster = NULL;
	nTREDClusters = NULL;
	bLabels = false;
}


VNInstance::~VNInstance()
{
	RVL_DELETE_ARRAY(bConcave);
	RVL_DELETE_ARRAY(d);
	RVL_DELETE_ARRAY(bd);
	RVL_DELETE_ARRAY(iPrimitiveClass);
	RVL_DELETE_ARRAY(R);
	RVL_DELETE_ARRAY(q);
	RVL_DELETE_ARRAY(s);
	RVL_DELETE_ARRAY(surfelCells.Element);
	//RVL_DELETE_ARRAY(conditionalProbability.Element);
	//RVL_DELETE_ARRAY(priorProbability.Element);
	RVL_DELETE_ARRAY(componentSurfelCells);
	//RVL_DELETE_ARRAY(surfelCellObjectMem);
	RVL_DELETE_ARRAY(componentSurfelCellMem);
	RVL_DELETE_ARRAY(PC.Element);
	RVL_DELETE_ARRAY(iCTIElement);
	RVL_DELETE_ARRAY(TREDMx);
	RVL_DELETE_ARRAY(TREDCluster);
	RVL_DELETE_ARRAY(nTREDClusters);
}

void VNInstance::UpdateCorrespondenceMatrix(
	Array<VNInstance *> models,
	int *Correspondences,
	int iModel1, 
	int iModel2,
	Array<int> *correspMatrix)
{
	VNInstance *pVNModelInstance = models.Element[iModel2];

	int *corresp = correspMatrix[iModel1].Element + nComponents*iModel2;
	int *corresp_ = correspMatrix[iModel2].Element + pVNModelInstance->nComponents*iModel1;

	memset(corresp_, 0xff, pVNModelInstance->nComponents * sizeof(int));

	for (int iCorr = 0; iCorr < nComponents; iCorr++)
	{
		corresp[iCorr] = Correspondences[iCorr];

		if (Correspondences[iCorr] >= 0)
			corresp_[Correspondences[iCorr]] = iCorr;
	}
}

void VNInstance::Match(
	RVL::QList<VNInstance> *pModelVNList,
	int *&Correspondences,
	void *vpClassifier,
	float *E,
	Array<int> *correspMatrix,
	int iModel1,
	int nModels,
	bool bReturnAllCorrespondences)
{
	VNInstance *pVNSceneInstance;
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	//when matching models from DB
	if (pClassifier->bGenerateModelVNInstances)
	{
		pVNSceneInstance = pModelVNList->pFirst;

		if (pVNSceneInstance == NULL)
			return;

		//to reach the last instance in list
		while (pVNSceneInstance->pNext)
			pVNSceneInstance = pVNSceneInstance->pNext;

	}
	//when matching scene (or MST)
	else
	{
		pVNSceneInstance = this;
	}
	
	if (!bReturnAllCorrespondences)
		Correspondences = new int[pVNSceneInstance->nComponents];

	FILE *fCorrespondencesArrayE = fopen("C:\\RVL\\ExpRez\\CorrespondencesError.txt", "w");
	fclose(fCorrespondencesArrayE);

	FILE *fCorrespondences;

	pClassifier->pTimer->Start();

	double StartTime = pClassifier->pTimer->GetTime();
#ifdef RVLVN_TIME_MESUREMENT
	LARGE_INTEGER CNTRSegmentationSTART, CNTRSegmentationEND;
	LARGE_INTEGER CNTRDescriptorGenerationSTART, CNTRDescriptorGenerationEND;

	LARGE_INTEGER frequency;
	QueryPerformanceFrequency((LARGE_INTEGER *)&frequency);

	QueryPerformanceCounter((LARGE_INTEGER *)&CNTRSegmentationSTART);
	#endif
#ifdef RVLVN_PART_SEGMENTATION_MATCH_LOG
	FILE *fpMatch = fopen((std::string(pClassifier->resultsFolder) + "\\VNInstanceToClassMatch.dat").data(), "w");
#endif

	VNInstance *pVNModelInstance = pModelVNList->pFirst;
	int iModel2 = 0;
	float cost = 0;
	int iSComp, matchedModelID;
	int *matchedCorrespondences = new int[pVNSceneInstance->nComponents];
	float minCost;
	bool bFirst = true;
	int *correspondencesOrder = new int[pVNSceneInstance->nComponents];
	int *matchedcCorrespondencesOrder = new int[pVNSceneInstance->nComponents];
	int *corresp, *corresp_;
	VNInstance *pVNModelInstance_;
	int *Correspondences_;

	//MATCH - compare "scene" instance to all "model" instances
	while (pVNModelInstance)
	{
		if (pClassifier->bMST && (pVNModelInstance == pVNSceneInstance))
			break;
		if (pVNModelInstance != pVNSceneInstance)
		{
			{
				float *E_ = E + (iModel1 * nModels + iModel2) * pVNSceneInstance->part.w;

				Correspondences_ = (bReturnAllCorrespondences ? Correspondences + nComponents * iModel2 : Correspondences);

				pVNSceneInstance->Match2Instances(pVNModelInstance, Correspondences_, vpClassifier, cost, correspondencesOrder, iModel2, E_);

				if (pClassifier->bMST)
					pVNSceneInstance->UpdateCorrespondenceMatrix(pClassifier->modelVNArray, Correspondences_, iModel1, iModel2, correspMatrix);

				if (bFirst == true)
				{
					minCost = cost;
					bFirst = false;
				}

				if (cost <= minCost)
				{
					minCost = cost;
					matchedModelID = iModel2;
					memcpy(matchedCorrespondences, Correspondences_, pVNSceneInstance->nComponents*sizeof(int));
					memcpy(matchedcCorrespondencesOrder, correspondencesOrder, pVNSceneInstance->nComponents*sizeof(int));
				}

#ifdef RVLVN_PART_SEGMENTATION_MATCH_LOG
				fprintf(fpMatch, "%d\t", iModel2);

				for (iSComp = 0; iSComp < nComponents; iSComp++)
					fprintf(fpMatch, "%d\t", Correspondences_[iSComp]);

				fprintf(fpMatch, "%f\n", cost);
#endif

				//fCorrespondences = fopen("C:\\RVL\\ExpRez\\Correspondences.txt", "a");
				//fprintf(fCorrespondences, "%d. model\n", iModel2);
				//fprintf(fCorrespondences, "cost = %f\n", cost);
				//for (int i = 0; i < pVNSceneInstance->nComponents; i++)
				//	fprintf(fCorrespondences, "%d\n", Correspondences[i]);
				//fprintf(fCorrespondences, "\n");
				//fclose(fCorrespondences);
			}

			iModel2++;
		}
			
		pVNModelInstance = pVNModelInstance->pNext;
	}

	pClassifier->pTimer->Stop();

	double ExecTime = pClassifier->pTimer->GetTime();

	//double ExecTime = pClassifier->pTimer->GetTime() - StartTime;

#ifdef RVLVN_TIME_MESUREMENT
	QueryPerformanceCounter((LARGE_INTEGER *)&CNTRSegmentationEND);
	double timeSegmentation = (CNTRSegmentationEND.QuadPart - CNTRSegmentationSTART.QuadPart) * 1000.0 / frequency.QuadPart;
	#endif
#ifdef RVLVN_PART_SEGMENTATION_MATCH_LOG
	fclose(fpMatch);
#endif

	//printf("%lf\n", timeSegmentation);

	//if (!pClassifier->bMST)
	//{
	//	//mIoU:
	//	float mIoU;
	//	int br = 0;

	//	pVNModelInstance = pClassifier->modelVNArray.Element[matchedModelID];

	//	pVNSceneInstance->mIoU(pVNModelInstance, matchedCorrespondences, mIoU, vpClassifier, matchedcCorrespondencesOrder, pMesh);

	//	fCorrespondences = fopen("C:\\RVL\\ExpRez\\Correspondences.txt", "a");
	//	fprintf(fCorrespondences, "\n");
	//	fprintf(fCorrespondences, "Matched model ID: %d\t", matchedModelID);
	//	fprintf(fCorrespondences, "Matched cost = %f\n", minCost);
	//	fprintf(fCorrespondences, "mIoU = %f\n", mIoU);
	//	fprintf(fCorrespondences, "Matched Correspondences\n");
	//	for (int i = 0; i < pVNSceneInstance->nComponents; i++)
	//		fprintf(fCorrespondences, "%d\n", matchedCorrespondences[i]);
	//	fprintf(fCorrespondences, "\n");
	//	fclose(fCorrespondences);

	//	FILE *fmIoU = fopen("C:\\RVL\\ExpRez\\mIoU.txt", "a");
	//	fprintf(fCorrespondences, "%f\n", mIoU);
	//	fclose(fmIoU);
	//}
	//delete[] matchedcCorrespondencesOrder;
	//delete[] correspondencesOrder;
}

void VNInstance::Match(
	Array<VNInstance *> models,
	void *vpClassifier,
	int *&Correspondences,
	int outMxSize,
	float *E,
	Array<int> *correspMatrix,
	int iModel1,
	bool bMatchAllComponents)
{
	Correspondences = new int[nComponents];

	//FILE *fCorrespondencesArrayE = fopen("C:\\RVL\\ExpRez\\CorrespondencesError.txt", "w");
	//fclose(fCorrespondencesArrayE);

	//FILE *fCorrespondences;

	VNInstance *pVNModelInstance;
	int iModel2;
	float cost;
	float minCost;
	bool bFirst = true;
	int *correspondencesOrder = new int[nComponents];
	float *E_, *E__;

	//MATCH - compare "scene" instance to all "model" instances
	for (iModel2 = 0; iModel2 < models.n; iModel2++)
	{
		if (iModel2 == iModel1)
			continue;

		pVNModelInstance = models.Element[iModel2];

		E_ = E + (iModel1 * outMxSize + iModel2) * part.w;

		Match2Instances(pVNModelInstance, Correspondences, vpClassifier, cost, correspondencesOrder, iModel2, E_, 0, bMatchAllComponents);

		E__ = E + (iModel2 * outMxSize + iModel1) * part.w;

		if (part.w > 1)
			memcpy(E__, E_, part.w * sizeof(float));
		else
			E__[0] = E_[0];

		UpdateCorrespondenceMatrix(models, Correspondences, iModel1, iModel2, correspMatrix);

		if (bFirst == true)
		{
			minCost = cost;
			bFirst = false;
		}
		else if (cost <= minCost)
			minCost = cost;

		//fCorrespondences = fopen("C:\\RVL\\ExpRez\\Correspondences.txt", "a");
		//fprintf(fCorrespondences, "%d. model\n", iModel2);
		//fprintf(fCorrespondences, "cost = %f\n", cost);
		//for (int i = 0; i < pVNSceneInstance->nComponents; i++)
		//	fprintf(fCorrespondences, "%d\n", Correspondences[i]);
		//fprintf(fCorrespondences, "\n");
		//fclose(fCorrespondences);
	}

	delete[] correspondencesOrder;
}

void VNInstance::mIoU(VNInstance *pVNModel,
	int *Correspondences,
	float &mIoU,
	void *vpClassifier,
	int *correspondencesOrder,
	Mesh *pMesh)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	VNInstance *pVNScene = this;

	int modelComponentID,sceneComponentID;
	int maxlabel = pVNModel->part.w-1;
	int nLabels = maxlabel + 1;

	float *ptComponentDist;

	AssignComponentsToPoints(pMesh, pClassifier->pSurfels, ptComponentDist);

	int *labeledPointsArr = new int[pMesh->NodeArray.n];
	memset(labeledPointsArr, 0xff, pMesh->NodeArray.n * sizeof(int));

	int pointID, iCell;
	SURFEL::Cell *pCell;
	Surfel *pSurfel;
	QLIST::Index *piSurfelIdx;
	QLIST::Index2 *piPtIdx;
	int k;
	int kmaxS = pVNScene->surfelCells.n;
	int *cellS = new int[kmaxS];
	memset(cellS, 0, kmaxS * sizeof(int));

	//Vidovic
	AssignLabelToAllMeshPoints(pMesh, -1);

	for (int i = 0; i < pVNScene->nComponents; i++)
	{
		if (correspondencesOrder[i] == -1)
			break;

		sceneComponentID = correspondencesOrder[i];

		modelComponentID = Correspondences[sceneComponentID];

		if (modelComponentID != -1) //which would indicate that this scene component was not associated with any of model components
		{
			for (iCell = 0; iCell < pVNScene->componentSurfelCells[sceneComponentID].n; iCell++)
			{
				k = pVNScene->componentSurfelCells[sceneComponentID].Element[iCell];
				if (cellS[k] != 0)
					continue;

				cellS[k]++;

				pCell = pVNScene->surfelCells.Element + pVNScene->componentSurfelCells[sceneComponentID].Element[iCell];

				piSurfelIdx = pCell->surfelList.pFirst;

				while (piSurfelIdx)
				{
					pSurfel = pClassifier->pSurfels->NodeArray.Element + piSurfelIdx->Idx;

					piPtIdx = pSurfel->PtList.pFirst;

					while (piPtIdx)
					{
						labeledPointsArr[piPtIdx->Idx] = pVNModel->label[modelComponentID];
						pMesh->NodeArray.Element[piPtIdx->Idx].label = pVNModel->label[modelComponentID];
						piPtIdx = piPtIdx->pNext;
					}

					piSurfelIdx = piSurfelIdx->pNext;
				}
			}
		}	
	}

	FILE *fpUnassigned = fopen((std::string(pClassifier->resultsFolder) + "\\pointsUnassigned.txt").data(), "w");

	SavePointsWithLabels(pMesh, fpUnassigned);

	//Vidovic
	AssignLabelsToUnassignedPoints(Correspondences, vpClassifier, pMesh, ptComponentDist, pVNModel);

	fclose(fpUnassigned);
	

	int U, I;
	float sumIoU = 0;
	float *IoU = new float[maxlabel+1];

	int pt;
	//std::vector<int> GTlabeledPointsArr;

	FILE *fSegFiles = fopen("D:\\ARP3D\\Shapenet\\ShapeNetCore.v1\\seg_sequence.txt", "r");
	FILE *fGTlabeledPoints; //Vidovic

	//Vidovic
	if (fSegFiles)
	{
		int fileIdx = pClassifier->sceneID;
		char tempString[100]; // tempString2[100];
		for (int sceneID = 0; sceneID <= fileIdx; sceneID++)
		{
			fscanf(fSegFiles, "%s\n", tempString);
		}

		std::string gtFileName = std::string("D:\\ARP3D\\Shapenet\\ShapeNetCore.v1\\04225987\\04225987\\") + tempString;
		strcpy(tempString, gtFileName.c_str());
		fGTlabeledPoints = fopen(tempString, "r");
	}
	else
		fGTlabeledPoints = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, "_.seg"), "r");

	//END Vidovic

	int *GTlabel = new int[pMesh->NodeArray.n];

	int GTLabel_;
	FILE *fpAssigned;

	if (fGTlabeledPoints)
	{
		for (int ipt = 0; ipt < pMesh->NodeArray.n; ipt++)
		{
			fscanf(fGTlabeledPoints, "%d\n", &GTLabel_);
			GTlabel[ipt] = GTLabel_;
		}

		fpAssigned = fopen((std::string(pClassifier->resultsFolder) + "\\pointsAssigned.txt").data(), "w");

		SavePointsWithLabels(pMesh, fpAssigned, NULL, GTlabel);

		fclose(fpAssigned);

		float mIoU_;

		for (int label = 0; label <= maxlabel; label++)
		{
			U = 0;
			I = 0;
			for (int i = 0; i < pMesh->NodeArray.n; i++)
			{
				//if (labeledPointsArr[i] == -1)
				//	int debug = 1;

				//if (labeledPointsArr[i] == label && GTlabel[i] == label)
				if (pMesh->NodeArray.Element[i].label == label && GTlabel[i] == label)
				{
					U++;
					I++;
				}
				if ((pMesh->NodeArray.Element[i].label == label && GTlabel[i] != label) || (pMesh->NodeArray.Element[i].label != label && GTlabel[i] == label))
				{
					U++;
				}

			}
			if (U == 0)
			{
				IoU[label] = -1;
				nLabels--;
			}
			else
			{
				IoU[label] = (float)I / U;
				sumIoU += IoU[label];
			}


		}

		mIoU_ = sumIoU / nLabels;

		fclose(fGTlabeledPoints);
	}

	// Computing mIoU for subsampled points.

	FILE *fCorr = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".cor"), "r");

	fGTlabeledPoints = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".seg"), "r");

	if (fCorr != NULL && fGTlabeledPoints != NULL)
	{
		int nCorr = 0;

		int corr_;

		while (!feof(fCorr))
		{
			fscanf(fCorr, "%d\n", &corr_);

			nCorr++;
		}

		fclose(fCorr);

		Array<int> corrs;

		corrs.Element = new int[nCorr];
		corrs.n = nCorr;

		fCorr = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".cor"), "r");

		int *pCorr = corrs.Element;

		while (!feof(fCorr))
			fscanf(fCorr, "%d\n", pCorr++);

		fclose(fCorr);

		for (int ipt = 0; ipt < nCorr; ipt++)
		{
			fscanf(fGTlabeledPoints, "%d\n", &GTLabel_);
			GTlabel[ipt] = GTLabel_;
		}

		fclose(fGTlabeledPoints);

		nLabels = maxlabel + 1;

		sumIoU = 0;

		Point *pPt;

		for (int label = 0; label <= maxlabel; label++)
		{
			U = 0;
			I = 0;
			for (int i = 0; i < nCorr; i++)
			{
				//if (labeledPointsArr[i] == -1)
				//	int debug = 1;

				pPt = pMesh->NodeArray.Element + corrs.Element[i];

				//if (labeledPointsArr[i] == label && GTlabel[i] == label)
				if (pPt->label == label && GTlabel[i] == label)
				{
					U++;
					I++;
				}
				if ((pPt->label == label && GTlabel[i] != label) || (pPt->label != label && GTlabel[i] == label))
				{
					U++;
				}

			}
			if (U == 0)
			{
				IoU[label] = -1;
				nLabels--;
			}
			else
			{
				IoU[label] = (float)I / U;
				sumIoU += IoU[label];
			}


		}

		mIoU = sumIoU / nLabels;

		fpAssigned = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".ptl"), "w");
		//fpAssigned = fopen((std::string(pClassifier->resultsFolder) + "\\pointsAssignedSS.txt").data(), "w");

		SavePointsWithLabels(pMesh, fpAssigned, &corrs, GTlabel);

		fclose(fpAssigned);

		delete[] corrs.Element;
	}
	else
	{
		if (fCorr)
			fclose(fCorr);

		if (fGTlabeledPoints)
			fclose(fGTlabeledPoints);
	}

	//Vidovic
	if (fSegFiles)
		fclose(fSegFiles);

	delete[] GTlabel;
	delete[] cellS;
	delete[] IoU;
	delete[] labeledPointsArr;
	delete[] ptComponentDist;
}

void VNInstance::Match(
	void *vpClassifier,
	int iModel,
	RECOG::VN_::CTINetOutput out)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	VNInstance *pModel = pClassifier->modelVNArray.Element[iModel];

	int nMCompsTotal = pClassifier->nMCompsTotalOU[0] + pClassifier->nMCompsTotalOU[1];

	int m = RVLMAX(pClassifier->classArray.Element[0].M.h, pClassifier->classArray.Element[1].M.h);

	float *dq = new float[m];

	int iFirstOutputRow = pClassifier->firstComponentAbsIdx.Element[iModel];

	int iSComp, iMComp, iMatch;
	float et, es, ea;
	float *qS, *qM;

	for (iSComp = 0; iSComp < nComponents; iSComp++)
	{
		qS = q + m * iSComp;

		for (iMComp = 0; iMComp < pModel->nComponents; iMComp++)
			if (bConcave[iSComp] == pModel->bConcave[iMComp])
			{
				qM = pModel->q + m * iMComp;

				iMatch = iFirstOutputRow + nMCompsTotal * iSComp + iMComp;

				out.fComp[iMatch] = pClassifier->MatchLatentVectors(qS, s[iSComp], qM, pModel->s[iMComp], m, et, es, ea, dq);

				out.fShape[iMatch] = es;
				out.fSize[iMatch] = ea;
			}
	}

	delete[] dq;
}

void VNInstance::Match2Instances(VNInstance *pVNModel,
	int *Correspondences,
	void *vpClassifier,
	float &cost,
	int *correspondencesOrder,
	int modelID,
	float *E_,
	int wE,
	bool bMatchAllComponents,
	bool bAssociation,
	bool *maskS,
	bool *maskM)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int m = RVLMAX(pClassifier->classArray.Element[0].M.h, pClassifier->classArray.Element[1].M.h);

	Array2D<float> distanceTable;

	if (bAssociation)
	{
		distanceTable.Element = new float[nComponents * pVNModel->nComponents];

		memset(Correspondences, 0xff, nComponents * sizeof(int));
		memset(correspondencesOrder, 0xff, nComponents * sizeof(int));
	}

	//float *dd = new float[pClassifier->sampledUnitSphere.h];
	float *dq = new float[m];
	
	int jS, jM, cS, cM;
	int j;
	float e, et, es, ea;
	//float *dS, *dM;
	float *qS, *qM;
	Array<int> CS, CM;
	
	bool bLabels_ = (bLabels && pClassifier->bInstanceMatchWithLabels);

	//E[modelID] = 0.0f;
	
	//for each part on the scene:
	for (int label = 0; label < pVNModel->part.w; label++)
	{
		if (!bLabels_ || label < part.w)
		{
			for (int ou = 0; ou <= 1; ou++)
			{
				jS = ou*part.w + (bLabels_ ? label : 0);

				CS = part.Element[jS];

				jM = ou*pVNModel->part.w + label;

				CM = pVNModel->part.Element[jM];

				for (int iS = 0; iS < CS.n; iS++)
				{
					cS = CS.Element[iS];

					//dS = d + pClassifier->sampledUnitSphere.h * cS;

					qS = q + m * cS;

					for (int iM = 0; iM < CM.n; iM++)
					{
						cM = CM.Element[iM];

						//dM = pVNModel->d + pClassifier->sampledUnitSphere.h * cM;

						qM = pVNModel->q + m * cM;

						//RVLDIFVECTORS(dS, dM, pClassifier->sampledUnitSphere.h, dd, j);
						//RVLDOTPRODUCT(dd, dd, pClassifier->sampledUnitSphere.h, e, j);

						e = pClassifier->MatchLatentVectors(qS, s[cS], qM, pVNModel->s[cM], m, et, es, ea, dq);

						//e *= (s[cS] * s[cS]);

						if (bAssociation)
							distanceTable.Element[pVNModel->nComponents*cS + cM] = e;
						else
							E_[wE * cS + cM] = e;
					}
				}
			}

			if (bAssociation && bLabels_)
			{
				GreedyInterpretation(pVNModel, label, distanceTable, Correspondences, vpClassifier, cost, correspondencesOrder, bMatchAllComponents, maskS, maskM);

				E_[label] = cost;
			}
		}
	
	}	// for every part 

	if (bAssociation && !bLabels_)
	{
		GreedyInterpretation(pVNModel, -1, distanceTable, Correspondences, vpClassifier, cost, correspondencesOrder, bMatchAllComponents, maskS, maskM);

		E_[0] = cost;
	}

	//delete[] dd;
	delete[] dq;

	if (bAssociation)
		delete[] distanceTable.Element;
}

void VNInstance::GreedyInterpretation(VNInstance *pVNModel, 
	int label, 
	Array2D<float> distanceTable, 
	int *Correspondences,
	void *vpClassifier,
	float &cost_,
	int *correspondencesOrder,
	bool bMatchAllComponents,
	bool *maskS,
	bool *maskM)
{
	
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	VNInstance *pVNScene = this;

	int jS, jM, cS, cM;
	int j, k, kmaxS, kmaxM;

	float e, cost;
	float *dS, *dM;
	Array<int> CS, CM;

	int *cellS, *cellM;
	Array<int> partCellsS, partCellsM;
	bool *STrue, *MTrue;

	int corrS, corrM;
	cost_ = 0;

	int sumS = 0, sumM = 0;
	int nTotalS = pVNScene->supportSize; 
	int nTotalM = pVNModel->supportSize;

	STrue = new bool[pVNScene->nComponents];
	memset(STrue, 0, pVNScene->nComponents * sizeof(bool));
	MTrue = new bool[pVNModel->nComponents];
	memset(MTrue, 0, pVNModel->nComponents * sizeof(bool));

	int labelS = (label >= 0 ? label : 0);

	int firstLabel, lastLabel;

	if (!bMatchAllComponents)
	{
		// partCellsS <- all cells of all scene components of part label

		kmaxS = pVNScene->surfelCells.n;

		cellS = new int[kmaxS];
		partCellsS.Element = new int[kmaxS];
		partCellsS.n = 0;
		memset(cellS, 0, kmaxS * sizeof(int));

		kmaxM = pVNModel->surfelCells.n;

		cellM = new int[kmaxM];
		partCellsM.Element = new int[kmaxM];
		partCellsM.n = 0;
		memset(cellM, 0, kmaxM * sizeof(int));		

		for (int ou = 0; ou <= 1; ou++)
		{
			jS = ou*pVNScene->part.w + labelS;

			CS = pVNScene->part.Element[jS]; //scene components for this part

			for (int iS = 0; iS < CS.n; iS++)
			{
				cS = CS.Element[iS];

				for (int j = 0; j < pVNScene->componentSurfelCells[cS].n; j++)
				{
					k = pVNScene->componentSurfelCells[cS].Element[j];

					if (cellS[k] == 0)
					{
						partCellsS.Element[partCellsS.n++] = k;

						cellS[k] = 1;
					}
				}
			}

			// partCellsM <- all cells of all model components of part label

			if (label >= 0)
				firstLabel = lastLabel = label;
			else
			{
				firstLabel = 0;
				lastLabel = pVNModel->part.w - 1;
			}

			for (int labelM = firstLabel; labelM <= lastLabel; labelM++)
			{
				jM = ou*pVNModel->part.w + labelM;

				CM = pVNModel->part.Element[jM]; //model components for this part

				for (int iM = 0; iM < CM.n; iM++)
				{
					cM = CM.Element[iM];

					for (int j = 0; j < pVNModel->componentSurfelCells[cM].n; j++)
					{
						k = pVNModel->componentSurfelCells[cM].Element[j];

						if (cellM[k] == 0)
						{
							partCellsM.Element[partCellsM.n++] = k;

							cellM[k] = 1;
						}
					}
				}
			}
		}

		memset(cellS, 0, kmaxS * sizeof(int));
		memset(cellM, 0, kmaxM * sizeof(int));
	}
		
	//greedy search:
	int nTrue = 0, n; // , label;
	bool allTrue = false;
	bool bFirst;	
	float ecorr;
	float *ecorrArr = new float[nComponents];

	int iOrder = 0;

	for (int ou = 0; ou <= 1; ou++)
	{

		jS = ou*pVNScene->part.w + labelS;

		CS = pVNScene->part.Element[jS]; //scene components of part label and of type ou
		n = CS.n;

		float mincost = 0.0f;
		while (true)
		{
			bFirst = true;

			for (int iS = 0; iS < CS.n; iS++)
			{				
				cS = CS.Element[iS];

				if (maskS)
					if (!maskS[cS])
						continue;

				if (STrue[cS])
					continue;

				if (!bMatchAllComponents)
				{
					// sumS <- the number of the supporting points of scene component cS, which don't belong to any component which is already in Correspondences

					sumS = 0;
					for (int j = 0; j < pVNScene->componentSurfelCells[cS].n; j++)
					{
						k = pVNScene->componentSurfelCells[cS].Element[j];
						if (cellS[k] == 0)
							sumS += pVNScene->surfelCells.Element[k].size;
					}
				}

				if (label >= 0)
					firstLabel = lastLabel = label;
				else
				{
					firstLabel = 0;
					lastLabel = pVNModel->part.w - 1;
				}

				for (int labelM = firstLabel; labelM <= lastLabel; labelM++)
				{
					jM = ou*pVNModel->part.w + labelM;

					CM = pVNModel->part.Element[jM]; //model components for this part

					for (int iM = 0; iM < CM.n; iM++)
					{					
						cM = CM.Element[iM];

						if (maskM)
							if (!maskM[cM])
								continue;

						//if (cS == 1 && cM == 1)
						//	int debug = 0;

						if (MTrue[cM])
							continue;

						if (!bMatchAllComponents)
						{
							// sumS <- the number of the supporting points of model component cM, which don't belong to any component which is already in Correspondences

							sumM = 0;
							for (int j = 0; j < pVNModel->componentSurfelCells[cM].n; j++)
							{
								k = pVNModel->componentSurfelCells[cM].Element[j];
								if (cellM[k] == 0)
									sumM += pVNModel->surfelCells.Element[k].size;
							}
						}

						e = distanceTable.Element[pVNModel->nComponents*cS + cM];
						
						if (bMatchAllComponents)
							cost = e;
						else
							cost = e - (pClassifier->lambda * ((float)(sumS) / nTotalS + (float)(sumM) / nTotalM));

						if (bFirst)
							mincost = cost;

						if ((cost < mincost || bFirst) && STrue[cS] == 0 && MTrue[cM] == 0)
						{
							mincost = cost;
							corrS = cS;
							corrM = cM;
							ecorr = e;
							bFirst = false;
						}
					}
				}
			}

			if ((!bMatchAllComponents && mincost > 0.0f) || bFirst)
				break;

			STrue[corrS] = 1;
			MTrue[corrM] = 1;
			Correspondences[corrS] = corrM;	
			ecorrArr[corrS] = ecorr;
			cost_ += ecorr;
				correspondencesOrder[iOrder] = corrS;
			iOrder++;

			if (!bMatchAllComponents)
			{
				// cellS[k] <- the number of scene components in Correspondences which contain the k-th surfel cell

				for (int j = 0; j < pVNScene->componentSurfelCells[corrS].n; j++)
				{
					k = pVNScene->componentSurfelCells[corrS].Element[j];
					cellS[k]++;
				}

				// cellM[k] <- the number of model components in Correspondences which contain the k-th surfel cell

				for (int j = 0; j < pVNModel->componentSurfelCells[corrM].n; j++)
				{
					k = pVNModel->componentSurfelCells[corrM].Element[j];
					cellM[k]++;
				}
			}
		}			
	}

	if (!bMatchAllComponents)
	{
		int NnaS = 0, NnaM = 0;

		// NnaS <- the number of scene points belonging to part label which are not assigned to any component in Correspondences

		for (int i = 0; i < partCellsS.n; i++)
		{
			k = partCellsS.Element[i];

			if (cellS[k] == 0)
				NnaS += pVNScene->surfelCells.Element[k].size;
		}

		// NnaM <- the number of model points belonging to part label which are not assigned to any component in Correspondences

		for (int i = 0; i < partCellsM.n; i++)
		{
			k = partCellsM.Element[i];

			if (cellM[k] == 0)
				NnaM += pVNModel->surfelCells.Element[k].size;
		}

		// Compute the final cost.

		cost_ += (0.5f * pClassifier->lambda*(((float)NnaS / nTotalS) + ((float)NnaM / nTotalM)));
	}
	
	//FILE *fCorrespondencesArrayE = fopen("C:\\RVL\\ExpRez\\CorrespondencesError.txt", "a");
	//
	//fprintf(fCorrespondencesArrayE, "Model: %d\n", modelID);

	//float N = ((float)(sumS) / nTotalS + (float)(sumM) / nTotalM);

	//for (int i = 0; i < pVNScene->nComponents; i++)
	//{
	//	fprintf(fCorrespondencesArrayE, "e = %f\tN = %f\tLmin=%f\n", ecorrArr[i], N, ecorrArr[i]/N);
	//}
	//	

	//fprintf(fCorrespondencesArrayE, "\n");
	//fclose(fCorrespondencesArrayE);

	delete[] ecorrArr;

	if (!bMatchAllComponents)
	{
		delete[] cellM;
		delete[] cellS;
		delete[] partCellsS.Element;
		delete[] partCellsM.Element;
	}
}

float VNInstance::MatchCost(int *X)
{
	

	return 0;
}

void VNInstance::SaveVNInstance(void *vpClassifier, FILE *fp)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	VNInstance *pVNModelInstance = this;

	fprintf(fp, "instance\n");

	int i;
	fprintf(fp, "%d\t", pVNModelInstance->nComponents);
	fprintf(fp, "%d\t", pVNModelInstance->surfelCells.n);
	for (i = 0; i < pVNModelInstance->nComponents; i++)
		fprintf(fp, "%d\t", pVNModelInstance->bConcave[i]);

	for (i = 0; i < pClassifier->sampledUnitSphere.h * pVNModelInstance->nComponents; i++)
		fprintf(fp, "%f\t", pVNModelInstance->d[i]);

	fprintf(fp, "%d\t", pVNModelInstance->nObjectCellRelations);

	fprintf(fp, "%d\t", pVNModelInstance->surfelCells.n);

	for (i = 0; i < pVNModelInstance->surfelCells.n; i++)
	{
		fprintf(fp, "%d\t", pVNModelInstance->surfelCells.Element[i].size);

	}

	for (i = 0; i < pVNModelInstance->nComponents; i++)
	{
		fprintf(fp, "%d\t", pVNModelInstance->label[i]);

	}


	fprintf(fp, "%d\t", pVNModelInstance->componentSurfelCells->n);


	for (i = 0; i < pVNModelInstance->nComponents; i++)
	{
		fprintf(fp, "%d\t", pVNModelInstance->componentSurfelCells[i].n);
		for (int j = 0; j < pVNModelInstance->componentSurfelCells[i].n; j++)
		{
			fprintf(fp, "%d\t", pVNModelInstance->componentSurfelCells[i].Element[j]);
		}

	}


	fprintf(fp, "%d\t", pVNModelInstance->supportSize);
	fprintf(fp, "%d\t", pVNModelInstance->part.w);
	int j;
	for (int ou = 0; ou <= 1; ou++)
	{
		for (int label = 0; label < pVNModelInstance->part.w; label++)
		{
			j = ou*pVNModelInstance->part.w + label;
			fprintf(fp, "%d\t", j);
			fprintf(fp, "%d\t", pVNModelInstance->part.Element[j].n);
			for (i = 0; i < pVNModelInstance->part.Element[j].n; i++)
			{
				fprintf(fp, "%d\t", pVNModelInstance->part.Element[j].Element[i]);
			}

			fprintf(fp, "%d\t", pVNModelInstance->part.Element[j].n);
		}
	}

	fprintf(fp, "\n");

}

void VNInstance::LoadVNInstance(void *vpClassifier, FILE *fp, CRVLMem *pMem0)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	VNInstance *pVNModel = this;

	int nComponents;

	fscanf(fp, "%d\t", &nComponents);
	pVNModel->nComponents = nComponents;
	fscanf(fp, "%d\t", &pVNModel->surfelCells.n);
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, bool, nComponents, pVNModel->bConcave);
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, float, nComponents*pClassifier->sampledUnitSphere.h, pVNModel->d);
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, SURFEL::Cell, pVNModel->surfelCells.n, pVNModel->surfelCells.Element);
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, Array<int>, nComponents, pVNModel->componentSurfelCells);
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, int, nComponents, pVNModel->label);

	int i;
	for (i = 0; i < nComponents; i++)
	{
		fscanf(fp, "%d\t", &pVNModel->bConcave[i]);
	}


	for (i = 0; i < nComponents * pClassifier->sampledUnitSphere.h; i++)
	{
		fscanf(fp, "%f\t", &pVNModel->d[i]);
	}

	fscanf(fp, "%d\t", &pVNModel->nObjectCellRelations);

	RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, int, pVNModel->nObjectCellRelations, pVNModel->componentSurfelCellMem);

	fscanf(fp, "%d\t", &pVNModel->surfelCells.n);

	for (i = 0; i < pVNModel->surfelCells.n; i++)
	{
		fscanf(fp, "%d\t", &pVNModel->surfelCells.Element[i].size);
	}



	int maxlabel = 0;
	for (i = 0; i < nComponents; i++)
	{
		fscanf(fp, "%d\t", &pVNModel->label[i]);
		if (pVNModel->label[i] > maxlabel)
			maxlabel = pVNModel->label[i];
	}


	fscanf(fp, "%d\t", &pVNModel->componentSurfelCells->n);


	int *pComponentSurfelRelation = pVNModel->componentSurfelCellMem;


	for (i = 0; i < pVNModel->nComponents; i++)
	{
		pVNModel->componentSurfelCells[i].Element = pComponentSurfelRelation;
		fscanf(fp, "%d\t", &pVNModel->componentSurfelCells[i].n);
		for (int j = 0; j < pVNModel->componentSurfelCells[i].n; j++)
		{
			fscanf(fp, "%d\t", &pVNModel->componentSurfelCells[i].Element[j]);
		}
		pComponentSurfelRelation += pVNModel->componentSurfelCells[i].n;

	}

	fscanf(fp, "%d\t", &pVNModel->supportSize);

	fscanf(fp, "%d\t", &pVNModel->part.w);


	int *partMem;
	int j;
	int nTypes = 2;
	pVNModel->part.h = nTypes; //convex and concave
	//pVNModel->part.w = maxlabel + 1;
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, Array<int>, pVNModel->part.h*pVNModel->part.w, pVNModel->part.Element);
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, int, nComponents, partMem);
	int *pPartLabel = partMem;



	for (int ou = 0; ou <= 1; ou++)
	{
		for (int label = 0; label < pVNModel->part.w; label++)
		{
			fscanf(fp, "%d\t", &j);
			pVNModel->part.Element[j].Element = pPartLabel;
			fscanf(fp, "%d\t", &pVNModel->part.Element[j].n);
			for (i = 0; i < pVNModel->part.Element[j].n; i++)
			{
				fscanf(fp, "%d\t", &pVNModel->part.Element[j].Element[i]);
			}
			fscanf(fp, "%d\t", &pVNModel->part.Element[j].n);
			pPartLabel += pVNModel->part.Element[j].n;

		}
	}

	bLabels = true;
}

void VNInstance::ProjectToLatentSubspace(void *vpClassifier)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int n = pClassifier->sampledUnitSphere.h;

	int m = RVLMAX(pClassifier->classArray.Element[0].M.h, pClassifier->classArray.Element[1].M.h);

	int ms = m - 3;

	RVL_DELETE_ARRAY(q);

	q = new float[nComponents * m];

	RVL_DELETE_ARRAY(s);

	s = new float[nComponents];

	int i, j, iComp;
	float *d_, *q_, *M, *a, *qs;
	float s2;

	for (iComp = 0; iComp < nComponents; iComp++)
	{
		d_ = d + iComp * n;

		M = pClassifier->classArray.Element[bConcave[iComp] ? 1 : 0].M.Element;

		q_ = q + iComp * m;

		RVLMULMXVECT(M, d_, m, n, q_, i, j, a);

		qs = q_ + 3;

		RVLDOTPRODUCT(qs, qs, ms, s2, i);

		s[iComp] = sqrt(s2);

		// Normalize the shape subvector.

		//q_[3] = s;

		//s = 1.0f / s;

		//RVLSCALEVECTOR(qs, s, qs, ms, i);		
	}
}

void VNInstance::MatchComponents(RVL::QList<VNInstance> *pModelVNList,
	int *&Correspondences,
	void *vpClassifier,
	Mesh *pMesh)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	VNInstance *pVNScene;

	//when matching models from DB
	if (pClassifier->bGenerateModelVNInstances)
	{
		pVNScene = pModelVNList->pFirst;

		if (pVNScene == NULL)
			return;

		//to reach the last instance in list
		while (pVNScene->pNext)
			pVNScene = pVNScene->pNext;

	}
	//when matching scene
	else
	{
		pVNScene = this;
	}

	Correspondences = new int[pVNScene->nComponents]; 
	int *correspondencesOrder = new int[pVNScene->nComponents]; // which component was first matched etc. 
	int *correspondencesModelID = new int[pVNScene->nComponents]; // to which model ID matched model component belongs to 

	memset(Correspondences, 0xff, nComponents * sizeof(int));
	memset(correspondencesOrder, 0xff, nComponents * sizeof(int));
	memset(correspondencesModelID, 0xff, nComponents * sizeof(int));


	std::vector<int> iModelComponents; //values = number of components in previous models
	std::vector<int> modelComponentsLabels;
	std::vector<int> componentModelID;

	//compute total number of model db components
	int br = 0;
	bool bFirst = true;
	int nMComponents = 0;
	VNInstance *pVNModelInstance = pModelVNList->pFirst;
	while (pVNModelInstance)
	{

		iModelComponents.push_back(nMComponents);

		for (int iComponent = 0; iComponent < pVNModelInstance->nComponents; iComponent++)
		{
			modelComponentsLabels.push_back(pVNModelInstance->label[iComponent]);
			componentModelID.push_back(br);
		}

		nMComponents += pVNModelInstance->nComponents;
		br++;

		pVNModelInstance = pVNModelInstance->pNext;
	}

	
	Array2D<float> distanceTable;
	distanceTable.Element = new float[pVNScene->nComponents * nMComponents];
	for (int iDTE = 0; iDTE < pVNScene->nComponents * nMComponents; iDTE++)
		distanceTable.Element[iDTE] = -1;
	
	int matchedModelID;
	float cost = 0;
	br = 0;
	float *dd = new float[pClassifier->sampledUnitSphere.h];

	//MATCH components 
	pVNModelInstance = pModelVNList->pFirst;

	while (pVNModelInstance)
	{
		if (pVNModelInstance != pVNScene) // possible only if matching training models with training models
		{

			int jS, jM, cS, cM;
			int j;
			float e;
			float *dS, *dM;
			Array<int> CS, CM;

			//for each part:
			for (int label = 0; label < pVNModelInstance->part.w; label++)
			{
				for (int ou = 0; ou <= 1; ou++)
				{
					jS = ou*pVNScene->part.w + (pVNScene->bLabels ? label : 0);

					CS = pVNScene->part.Element[jS];

					jM = ou*pVNModelInstance->part.w + label;

					CM = pVNModelInstance->part.Element[jM];

					for (int iS = 0; iS < CS.n; iS++)
					{
						cS = CS.Element[iS];

						dS = pVNScene->d + pClassifier->sampledUnitSphere.h * cS;

						for (int iM = 0; iM < CM.n; iM++)
						{
							cM = CM.Element[iM];

							dM = pVNModelInstance->d + pClassifier->sampledUnitSphere.h * cM;

							RVLDIFVECTORS(dS, dM, pClassifier->sampledUnitSphere.h, dd, j);
							RVLDOTPRODUCT(dd, dd, pClassifier->sampledUnitSphere.h, e, j);
							distanceTable.Element[nMComponents*cS + iModelComponents[br] + cM] = e; 
						}

					}
				}

				
			}// for each part 
		}
		br++;
		pVNModelInstance = pVNModelInstance->pNext;
	}

	FILE *fpDistanceTable = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".match"), "w");

	PrintMatrix<float>(fpDistanceTable, distanceTable.Element, nComponents, nMComponents);

	fclose(fpDistanceTable);

	//greedy:
	pVNScene->GreedyInterpretationComponents(distanceTable, modelComponentsLabels, Correspondences, vpClassifier, cost, correspondencesOrder, pMesh);


	FILE *fCorrespondences;
	fCorrespondences = fopen("C:\\RVL\\ExpRez\\Correspondences.txt", "a");
	fprintf(fCorrespondences, "cost = %f\n", cost);
	for (int i = 0; i < pVNScene->nComponents; i++)
	{
		fprintf(fCorrespondences, "%d\t", Correspondences[i]);
		if (Correspondences[i]!=-1)
			fprintf(fCorrespondences, "%d. model\n", componentModelID[Correspondences[i]]);
		else fprintf(fCorrespondences, "\n");
	}
	fprintf(fCorrespondences, "\n");
	fclose(fCorrespondences);

	//mIoU:
	float mIoU;
	pVNScene->mIoUComponents(Correspondences, modelComponentsLabels, mIoU, vpClassifier, correspondencesOrder, pMesh);

	FILE *fmIoU = fopen("C:\\RVL\\ExpRez\\mIoU.txt", "a");
	fprintf(fCorrespondences, "%f\n", mIoU);
	fclose(fmIoU);

	delete[] correspondencesOrder;
	delete[] correspondencesModelID;
	delete[] distanceTable.Element;
	delete[] dd;
}



void VNInstance::GreedyInterpretationComponents(
	Array2D<float> distanceTable,
	std::vector<int> modelComponentLabels,
	int *Correspondences,
	void *vpClassifier,
	float &cost_,
	int *correspondencesOrder,
	Mesh *pMesh)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	VNInstance *pVNScene = this;	

	float e_;

	for (int j = 0; j < nComponents; j++)
	{
		float mine = -1.0f;

		for (int i = 0; i < modelComponentLabels.size(); i++)
		{
			e_ = distanceTable.Element[j * modelComponentLabels.size() + i];

			if (e_ <= 0.0f)
				continue;

			if (mine < 0.0f || e_ < mine)
				mine = e_;
		}

		int debug = 0;
	}


	int jS, jM, cS, cM, j, k, kmaxS, corrS, corrM;
	float e, cost;
	float *dS, *dM;
	int *cellS; 
	bool *STrue;
	Array<int> CS, CM;

	int sumS = 0;
	int nTotalS = pVNScene->supportSize;

	kmaxS = pVNScene->surfelCells.n;

	cellS = new int[kmaxS];
	STrue = new bool[pVNScene->nComponents];
	memset(cellS, 0, kmaxS * sizeof(int));
	memset(STrue, 0, pVNScene->nComponents * sizeof(bool));

	cost_ = 0;

	//greedy search:
	int nTrue = 0, n, firstLabel, lastLabel, iOrder = 0;
	bool allTrue = false;
	bool bFirst;
	float ecorr;
	float *ecorrArr = new float[nComponents];

	//int labelS = (label >= 0 ? label : 0);
	int maxLabel = modelComponentLabels[0];
	for (int imaxLabel = 0; imaxLabel < modelComponentLabels.size(); imaxLabel++)
		if (modelComponentLabels[imaxLabel] > maxLabel)
			maxLabel = modelComponentLabels[imaxLabel];

	for (int ou = 0; ou <= 1; ou++)
	{

		jS = ou*pVNScene->part.w; //+ labelS;
		CS = pVNScene->part.Element[jS]; //scene components for this part
		n = CS.n;

		float mincost = 0.0f;

		while (true)
		{
			bFirst = true;

			for (int iS = 0; iS < CS.n; iS++)
			{
				cS = CS.Element[iS];
				if (STrue[cS])
					continue;
				sumS = 0;
				for (int j = 0; j < pVNScene->componentSurfelCells[cS].n; j++)
				{
					k = pVNScene->componentSurfelCells[cS].Element[j];
					if (cellS[k] == 0)
						sumS += pVNScene->surfelCells.Element[k].size;
				}


				for (cM = 0; cM < modelComponentLabels.size(); cM++)
				{
					e = distanceTable.Element[modelComponentLabels.size()*cS + cM];

					if (e != -1)
					{
						cost = e - (pClassifier->lambda * ((float)(sumS) / nTotalS));
						//cost = (e - pClassifier->lambda) * ((float)(sumS) / nTotalS);

						if (bFirst)
							mincost = cost;

						if ((cost < mincost || bFirst) && STrue[cS] == 0)
						{
							mincost = cost;
							corrS = cS;
							corrM = cM;
							ecorr = e;
							//ecorr = e * ((float)(sumS) / nTotalS);
							bFirst = false;
						}
					}
				}
			}

			if (mincost > 0.0f || bFirst)
				break;

			STrue[corrS] = 1;
			Correspondences[corrS] = corrM;
			ecorrArr[corrS] = ecorr;
			cost_ += ecorr;
			correspondencesOrder[iOrder] = corrS;
			iOrder++;

			for (int j = 0; j < pVNScene->componentSurfelCells[corrS].n; j++)
			{
				k = pVNScene->componentSurfelCells[corrS].Element[j];
				cellS[k]++;
			}
		}
	}

	int NnaS = 0;
	for (int i = 0; i < kmaxS; i++)
	{
		if (cellS[i] == 0)
			NnaS += pVNScene->surfelCells.Element[i].size;
	}

	cost_ += pClassifier->lambda*(((float)NnaS / nTotalS));

	delete[] cellS;
	delete[] STrue;
	delete[] ecorrArr;

}


//Vidovic
//void VNInstance::AssignLabelsToUnassignedPoints(
//	int *Correspondences,
//	void *vpClassifier,
//	Mesh *pMesh)
//{
//	//int *labeledPointsArr = new int[pMesh->NodeArray.n];
//	//memset(labeledPointsArr, 0xff, pMesh->NodeArray.n * sizeof(int));
//
//	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
//
//	SURFEL::Cell *pCell;
//	int iCell, iObject;
//	QLIST::Index *piObjectIdx;
//	QLIST::Index *piSurfelIdx;
//	SURFEL::Object *pObject;
//	int sceneComponentID;
//	int modelComponentID;
//	bool bCellLabelsAssigned;
//	Surfel *pSurfel;
//	QLIST::Index2 *piPtIdx;
//	int closestObject;
//	float *P = new float[3];
//
//	
//	for (int iCell = 0; iCell < surfelCells.n; iCell++)
//	{
//		pCell = surfelCells.Element + iCell;
//
//		bCellLabelsAssigned = false;
//
//		piObjectIdx = pCell->objectList.pFirst;
//
//		while (piObjectIdx)
//		{
//			pObject = pClassifier->pObjects->objectArray.Element + piObjectIdx->Idx;
//
//			//sceneComponentID = correspondencesOrder[piObjectIdx->Idx];
//			//modelComponentID = Correspondences[sceneComponentID];
//
//			modelComponentID = Correspondences[piObjectIdx->Idx];
//
//			//if there is object which has assigned labels to the points from this cell
//			if (modelComponentID != -1)
//			{
//				bCellLabelsAssigned = true;
//				break;
//			}
//
//			piObjectIdx = piObjectIdx->pNext;
//		}//for all objects in cell
//
//		if (!bCellLabelsAssigned)
//		{
//			piSurfelIdx = pCell->surfelList.pFirst;
//
//			while (piSurfelIdx)
//			{
//				pSurfel = pClassifier->pSurfels->NodeArray.Element + piSurfelIdx->Idx;
//
//				piPtIdx = pSurfel->PtList.pFirst;
//
//				while (piPtIdx)
//				{
//					P = pMesh->NodeArray.Element[piPtIdx->Idx].P;
//
//					//calculate closest component with assigned modelID
//					closestObject = FindClosestMatchedSceneComponent(Correspondences, vpClassifier, P);
//					
//					//Assign label to Pt
//					//labeledPointsArr[piPtIdx->Idx] = pVNModel->label[closestObject];
//					pMesh->NodeArray.Element[piPtIdx->Idx].label = label[closestObject];
//					
//					piPtIdx = piPtIdx->pNext;
//				}//for all Pts
//
//				piSurfelIdx = piSurfelIdx->pNext;
//
//			}//for all surfels
//
//		}//if (!bCellLabelsAssigned)
//
//	}//for all cells	
//	
//	delete[] P;
//	//delete[] labeledPointsArr;
//}

void VNInstance::AssignComponentsToPoints(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	float *&ptComponentDist)
{
	int nPtComponentRelations = pMesh->NodeArray.n * nComponents;

	ptComponentDist = new float[nPtComponentRelations];

	for (int i = 0; i < nPtComponentRelations; i++)
		ptComponentDist[i] = -1.0f;

	int iCell;
	SURFEL::Cell *pCell;
	Surfel *pSurfel;
	QLIST::Index *piSurfelIdx, *piObject;
	QLIST::Index2 *piPtIdx;

	for (iCell = 0; iCell < surfelCells.n; iCell++)
	{
		pCell = surfelCells.Element + iCell;

		piSurfelIdx = pCell->surfelList.pFirst;

		while (piSurfelIdx)
		{
			pSurfel = pSurfels->NodeArray.Element + piSurfelIdx->Idx;

			piPtIdx = pSurfel->PtList.pFirst;

			while (piPtIdx)
			{
				//if (piPtIdx->Idx == 44640)
				//	int debug = 0;

				piObject = pCell->objectList.pFirst;

				while (piObject)
				{
					ptComponentDist[piPtIdx->Idx * nComponents + piObject->Idx] = 0.0f;

					//if (piObject->Idx >= nComponents)
					//	int debug = 0;

					piObject = piObject->pNext;
				}

				piPtIdx = piPtIdx->pNext;
			}

			piSurfelIdx = piSurfelIdx->pNext;
		}
	}
}

void VNInstance::AssignLabelsToUnassignedPoints(
	int *Correspondences,
	void *vpClassifier,
	Mesh *pMesh,
	float *ptObjectDist,
	VNInstance *pVNModel,
	std::vector<int> *modelComponentLabels)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	Array<RECOG::PSGM_::Plane> *pConvexTemplate66 = &pClassifier->alignment.convexTemplate66;
	FILE *fpClusterDistance;

	if (pClassifier->bSaveClusterDistanceFile)
		fpClusterDistance = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".dist"), "w");


	int iPt, iCTIElement, iObject;
	int nObjects = pClassifier->pObjects->objectArray.n;
	//float *P = new float[3];
	float *P;
	float emax, e, emin;
	SURFEL::Object *pObject;

	bool bFirstObject = true;
	int closestObject = -1;
	int modelComponentID;

	for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
	{
		P = pMesh->NodeArray.Element[iPt].P;
		bFirstObject = true;

		for (iObject = 0; iObject < nObjects; iObject++)
		{
			if (ptObjectDist[nComponents * iPt + iObject] < 0.0f)
			{
				pObject = pClassifier->pObjects->objectArray.Element + iObject;

				emax = RVLDOTPRODUCT3(pConvexTemplate66->Element[0].N, P) - pObject->d[0];

				for (iCTIElement = 1; iCTIElement < pConvexTemplate66->n; iCTIElement++)
				{
					e = RVLDOTPRODUCT3(pConvexTemplate66->Element[iCTIElement].N, P) - pObject->d[iCTIElement];

					if (e > emax)
						emax = e;
				}

				if (emax < 0.0f)
					emax = -emax;
			}
			else
				emax = 0.0f;

			if (pClassifier->bSaveClusterDistanceFile)
				fprintf(fpClusterDistance, "%f\t", emax);

			//if point doesn't have label
			if (pMesh->NodeArray.Element[iPt].label == -1)
			{
				//find colsest matched object
				modelComponentID = Correspondences[iObject];

				//if current object has match
				if (modelComponentID != -1)
				{
					if (bFirstObject)
					{
						bFirstObject = false;
						closestObject = iObject;
						emin = emax;
					}
					else
					{
						if (emax < emin)
						{
							emin = emax;
							closestObject = iObject;
						}
					}
				}//if current object has match
			}

		}//for all scene objects

		//assign labels to unassigned points
		if (!bFirstObject)
		{
			if (pMesh->NodeArray.Element[iPt].label == -1)
				if (pVNModel != NULL)
					pMesh->NodeArray.Element[iPt].label = pVNModel->label[Correspondences[closestObject]];
				else
					pMesh->NodeArray.Element[iPt].label = modelComponentLabels->at(Correspondences[closestObject]);
		}

		if (pClassifier->bSaveClusterDistanceFile)
			fprintf(fpClusterDistance, "\n");		

	}//for all mesh points

	if (pClassifier->bSaveClusterDistanceFile)
		fclose(fpClusterDistance);
	//delete[] P;
}

void VNInstance::AssignLabelsToUnassignedPoints(
	RECOG::VN_::PartAssociation *association,
	void *vpClassifier,
	Mesh *pMesh,
	float *ptObjectDist)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	Array<RECOG::PSGM_::Plane> *pConvexTemplate66 = &pClassifier->alignment.convexTemplate66;
	FILE *fpClusterDistance;

	if (pClassifier->bSaveClusterDistanceFile)
		fpClusterDistance = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".dist"), "w");


	int iPt, iCTIElement, iObject;
	int nObjects = pClassifier->pObjects->objectArray.n;
	//float *P = new float[3];
	float *P;
	float emax, e, emin;
	SURFEL::Object *pObject;

	bool bFirstObject = true;
	int closestObject = -1;
	int modelComponentID;

	for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
	{
		P = pMesh->NodeArray.Element[iPt].P;
		bFirstObject = true;

		for (iObject = 0; iObject < nObjects; iObject++)
		{
			if (ptObjectDist[nComponents * iPt + iObject] < 0.0f)
			{
				pObject = pClassifier->pObjects->objectArray.Element + iObject;

				emax = RVLDOTPRODUCT3(pConvexTemplate66->Element[0].N, P) - pObject->d[0];

				for (iCTIElement = 1; iCTIElement < pConvexTemplate66->n; iCTIElement++)
				{
					e = RVLDOTPRODUCT3(pConvexTemplate66->Element[iCTIElement].N, P) - pObject->d[iCTIElement];

					if (e > emax)
						emax = e;
				}

				if (emax < 0.0f)
					emax = -emax;
			}
			else
				emax = 0.0f;

			if (pClassifier->bSaveClusterDistanceFile)
				fprintf(fpClusterDistance, "%f\t", emax);

			//if point doesn't have label
			if (pMesh->NodeArray.Element[iPt].label == -1)
			{
				//find colsest matched object
				//modelComponentID = association[iObject].iMetaModel;
				modelComponentID = association[iObject].label; //2019.11.10

				//if current object has match
				if (modelComponentID != -1)
				{
					if (bFirstObject)
					{
						bFirstObject = false;
						closestObject = iObject;
						emin = emax;
					}
					else
					{
						if (emax < emin)
						{
							emin = emax;
							closestObject = iObject;
						}
					}
				}//if current object has match
			}

		}//for all scene objects

		//assign labels to unassigned points
		if (!bFirstObject)
			if (pMesh->NodeArray.Element[iPt].label == -1)				
				pMesh->NodeArray.Element[iPt].label = association[closestObject].label;


		if (pClassifier->bSaveClusterDistanceFile)
			fprintf(fpClusterDistance, "\n");

	}//for all mesh points

	if (pClassifier->bSaveClusterDistanceFile)
		fclose(fpClusterDistance);
	//delete[] P;
}


int VNInstance::FindClosestMatchedSceneComponent(
	int *Correspondences,
	void *vpClassifier,
	float *P)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	Array<RECOG::PSGM_::Plane> *pConvexTemplate66 = &pClassifier->alignment.convexTemplate66;

	float emax, emin;
	float e;
	bool bFirstObject = true;
	int closestObject = -1;
	int modelComponentID;

	int iObject;
	SURFEL::Object *pObject;

	int iCTIElement;	

	for (iObject = 0; iObject < pClassifier->pObjects->objectArray.n; iObject++)
	{
		pObject = pClassifier->pObjects->objectArray.Element + iObject;

		//sceneComponentID = correspondencesOrder[iObject];
		//modelComponentID = Correspondences[sceneComponentID];

		modelComponentID = Correspondences[iObject];

		//if scene object is matched
		if (modelComponentID != -1)
		{
			emax = RVLDOTPRODUCT3(pConvexTemplate66->Element[0].N, P) - pObject->d[0];		

			for (iCTIElement = 1; iCTIElement < pConvexTemplate66->n; iCTIElement++)
			{
				e = RVLDOTPRODUCT3(pConvexTemplate66->Element[iCTIElement].N, P) - pObject->d[iCTIElement];

				if (e > emax)
					emax = e;
			}

			if (emax < 0.0f)
				emax = -emax;

			if (bFirstObject)
			{
				closestObject = iObject;
				emin = emax;

				bFirstObject = false;
			}
			else
			{			
				if (emax < emin)
				{
					emin = emax;
					closestObject = iObject;
				}
			}

		}// if (modelComponentID != -1)
	
	}// for all scene objects

	return closestObject;

}

void VNInstance::AssignLabelToAllMeshPoints(
	Mesh *pMesh,
	int label)
{
	for (int iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
		pMesh->NodeArray.Element[iPt].label = label;
}

void VNInstance::SavePointsWithLabels(
	Mesh *pMesh,
	FILE *fp,
	Array<int> *pSamples,
	int *GTLabel)
{
	Point *pPt;

	if (pSamples)
	{
		for (int iPt = 0; iPt < pSamples->n; iPt++)
		{
			pPt = pMesh->NodeArray.Element + pSamples->Element[iPt];

			if (GTLabel)
				fprintf(fp, "%f\t%f\t%f\t%d\t%d\n", pPt->P[0], pPt->P[1], pPt->P[2], pPt->label, GTLabel[iPt]);
			else
				fprintf(fp, "%f\t%f\t%f\t%d\n", pPt->P[0], pPt->P[1], pPt->P[2], pPt->label);
		}
	}
	else
		for (int iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
		{
			pPt = pMesh->NodeArray.Element + iPt;

			if (GTLabel)
				fprintf(fp, "%f\t%f\t%f\t%d\t%d\n", pPt->P[0], pPt->P[1], pPt->P[2], pPt->label, GTLabel[iPt]);
			else
				fprintf(fp, "%f\t%f\t%f\t%d\n", pPt->P[0], pPt->P[1], pPt->P[2], pPt->label);
		}
}

void VNInstance::mIoUComponents(
	int *Correspondences,
	std::vector<int> modelComponentLabels,
	float &mIoU,
	void *vpClassifier,
	int *correspondencesOrder,
	Mesh *pMesh)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	VNInstance *pVNScene = this;

	int modelComponentID, sceneComponentID;

	int maxLabel = modelComponentLabels[0];
	for (int imaxLabel = 0; imaxLabel < modelComponentLabels.size(); imaxLabel++)
	{
		if (modelComponentLabels[imaxLabel] > maxLabel)
			maxLabel = modelComponentLabels[imaxLabel];
	}

	float *ptComponentDist;

	AssignComponentsToPoints(pMesh, pClassifier->pSurfels, ptComponentDist);

	int *labeledPointsArr = new int[pMesh->NodeArray.n];
	memset(labeledPointsArr, 0xff, pMesh->NodeArray.n * sizeof(int));

	int pointID, iCell;
	SURFEL::Cell *pCell;
	Surfel *pSurfel;
	QLIST::Index *piSurfelIdx, *piObject;
	QLIST::Index2 *piPtIdx;
	int k;
	int kmaxS = pVNScene->surfelCells.n;
	int *cellS = new int[kmaxS];
	memset(cellS, 0, kmaxS * sizeof(int));

	//Vidovic
	AssignLabelToAllMeshPoints(pMesh, -1);

	for (int i = 0; i < pVNScene->nComponents; i++)
	{
		if (correspondencesOrder[i] == -1)
			break;

		sceneComponentID = correspondencesOrder[i];

		modelComponentID = Correspondences[sceneComponentID];

		if (modelComponentID != -1) //which would indicate that this scene component was not associated with any of model components
		{
			for (iCell = 0; iCell < pVNScene->componentSurfelCells[sceneComponentID].n; iCell++)
			{
				k = pVNScene->componentSurfelCells[sceneComponentID].Element[iCell];
				if (cellS[k] != 0)
					continue;

				cellS[k]++;

				pCell = pVNScene->surfelCells.Element + pVNScene->componentSurfelCells[sceneComponentID].Element[iCell];

				piSurfelIdx = pCell->surfelList.pFirst;

				while (piSurfelIdx)
				{
					pSurfel = pClassifier->pSurfels->NodeArray.Element + piSurfelIdx->Idx;

					piPtIdx = pSurfel->PtList.pFirst;

					while (piPtIdx)
					{
						if (piPtIdx->Idx == 44640)
							int debug = 0;

						labeledPointsArr[piPtIdx->Idx] = modelComponentLabels[modelComponentID];
						pMesh->NodeArray.Element[piPtIdx->Idx].label = modelComponentLabels[modelComponentID];
						piPtIdx = piPtIdx->pNext;
					}

					piSurfelIdx = piSurfelIdx->pNext;
				}
			}
		}
	}

	FILE *fpUnassigned = fopen((std::string(pClassifier->resultsFolder) + "\\pointsUnassigned.txt").data(), "w");

	SavePointsWithLabels(pMesh, fpUnassigned);

	//Vidovic
	AssignLabelsToUnassignedPoints(Correspondences, vpClassifier, pMesh, ptComponentDist, NULL, &modelComponentLabels);

	fclose(fpUnassigned);


	int U, I;
	float sumIoU = 0;
	float *IoU = new float[maxLabel + 1];

	int pt;
	//std::vector<int> GTlabeledPointsArr;

	FILE *fSegFiles = fopen("D:\\ARP3D\\Shapenet\\ShapeNetCore.v1\\seg_sequence.txt", "r");
	FILE *fGTlabeledPoints; //Vidovic

	//Vidovic
	if (fSegFiles)
	{
		int fileIdx = pClassifier->sceneID;
		char tempString[100]; // tempString2[100];
		for (int sceneID = 0; sceneID <= fileIdx; sceneID++)
		{
			fscanf(fSegFiles, "%s\n", tempString);
		}

		std::string gtFileName = std::string("D:\\ARP3D\\Shapenet\\ShapeNetCore.v1\\04225987\\04225987\\") + tempString;
		strcpy(tempString, gtFileName.c_str());
		fGTlabeledPoints = fopen(tempString, "r");
	}
	else
		fGTlabeledPoints = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, "_.seg"), "r");
	//END Vidovic

	int GTLabel_;

	int *GTlabel = new int[pMesh->NodeArray.n];

	for (int ipt = 0; ipt < pMesh->NodeArray.n; ipt++)
	{
		fscanf(fGTlabeledPoints, "%d\n", &GTLabel_);
		GTlabel[ipt] = GTLabel_;
	}

	FILE *fpAssigned = fopen((std::string(pClassifier->resultsFolder) + "\\pointsAssigned.txt").data(), "w");

	SavePointsWithLabels(pMesh, fpAssigned, NULL, GTlabel);

	fclose(fpAssigned);

	int nLabels = maxLabel + 1;

	float mIoU_;

	for (int label = 0; label <= maxLabel; label++)
	{
		U = 0;
		I = 0;
		for (int i = 0; i < pMesh->NodeArray.n; i++)
		{
			//if (labeledPointsArr[i] == -1)
			//	int debug = 1;


			//if (labeledPointsArr[i] == label && GTlabel[i] == label)
			if (pMesh->NodeArray.Element[i].label == label && GTlabel[i] == label)
			{
				U++;
				I++;
			}
			if ((pMesh->NodeArray.Element[i].label == label && GTlabel[i] != label) || (pMesh->NodeArray.Element[i].label != label && GTlabel[i] == label))
			{
				U++;
			}

		}
		if (U == 0)
		{
			IoU[label] = -1;
			nLabels--;
		}
		else
		{
			IoU[label] = (float)I / U;
			sumIoU += IoU[label];
		}


	}

	mIoU_ = sumIoU / nLabels;

	fclose(fGTlabeledPoints);

	// Computing mIoU for subsampled points.

	FILE *fCorr = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".cor"), "r");

	int nCorr = 0;

	int corr_;

	while (!feof(fCorr))
	{
		fscanf(fCorr, "%d\n", &corr_);

		nCorr++;
	}

	fclose(fCorr);

	Array<int> corrs;

	corrs.Element = new int[nCorr];
	corrs.n = nCorr;

	fCorr = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".cor"), "r");

	int *pCorr = corrs.Element;

	while (!feof(fCorr))
		fscanf(fCorr, "%d\n", pCorr++);

	fclose(fCorr);

	fGTlabeledPoints = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".seg"), "r");

	if (fGTlabeledPoints)
	{
		for (int ipt = 0; ipt < nCorr; ipt++)
		{
			fscanf(fGTlabeledPoints, "%d\n", &GTLabel_);
			GTlabel[ipt] = GTLabel_;
		}

		fclose(fGTlabeledPoints);

		nLabels = maxLabel + 1;

		sumIoU = 0;

		Point *pPt;

		for (int label = 0; label <= maxLabel; label++)
		{
			U = 0;
			I = 0;
			for (int i = 0; i < nCorr; i++)
			{
				//if (labeledPointsArr[i] == -1)
				//	int debug = 1;

				pPt = pMesh->NodeArray.Element + corrs.Element[i];

				//if (labeledPointsArr[i] == label && GTlabel[i] == label)
				if (pPt->label == label && GTlabel[i] == label)
				{
					U++;
					I++;
				}
				if ((pPt->label == label && GTlabel[i] != label) || (pPt->label != label && GTlabel[i] == label))
				{
					U++;
				}

			}
			if (U == 0)
			{
				IoU[label] = -1;
				nLabels--;
			}
			else
			{
				IoU[label] = (float)I / U;
				sumIoU += IoU[label];
			}


		}

		mIoU = sumIoU / nLabels;

		fpAssigned = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".ptl"), "w");
		//fpAssigned = fopen((std::string(pClassifier->resultsFolder) + "\\pointsAssignedSS.txt").data(), "w");

		SavePointsWithLabels(pMesh, fpAssigned, &corrs, GTlabel);

		fclose(fpAssigned);
	}

	//Vidovic
	if (fSegFiles)
		fclose(fSegFiles);

	delete[] GTlabel;
	delete[] cellS;
	delete[] IoU;
	delete[] labeledPointsArr;
	delete[] corrs.Element;
	delete[] ptComponentDist;

}

void VNInstance::GreedyProbabilisticAssociation(
	void *vpClassifier,
	int iClass,
	int inLabel,
	int *iMetaModel,
	bool *bSCompAssigned,
	bool *bCellAssigned,
	VN_::PartAssociation *association)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	float kUnassignedPointsLogProbability = pClassifier->dUnassignedPointsLogProbability / pClassifier->dUnassignedPoints;

	int nLabels = pClassifier->classes[iClass].nLabels;

	int firstLabel, lastLabel;

	if (inLabel >= 0)
		firstLabel = lastLabel = inLabel;
	else
	{
		firstLabel = 0;
		lastLabel = nLabels - 1;
	}

	int i, iSSegment, iComp, iCell, unassignedSupportSubsetSize, iSSegmentBestAssociation, label, iMM, nMMComponents;
	int iFirstMM, iLastMM;
	float costReduction, maxCostReduction, unassignedPointsLogProbability;
	VN_::PartAssociation bestAssociation;
	VN_::ModelTemplate2 metamodel;

	while (true)	// greedy search loop
	{
		iSSegmentBestAssociation = -1;
		maxCostReduction = 0.0f;

		for (iSSegment = 0; iSSegment < nComponents; iSSegment++)
		{
			if (bSCompAssigned[iSSegment])
				continue;

			unassignedSupportSubsetSize = 0;

			for (i = 0; i < componentSurfelCells[iSSegment].n; i++)
			{
				iCell = componentSurfelCells[iSSegment].Element[i];

				if (!bCellAssigned[iCell])
					unassignedSupportSubsetSize += surfelCells.Element[iCell].size;
			}			

			unassignedPointsLogProbability = kUnassignedPointsLogProbability * (float)unassignedSupportSubsetSize / (float)supportSize;

			for (label = firstLabel; label <= lastLabel; label++)
			{
				if (iMetaModel[label] >= 0)
					iFirstMM = iLastMM = iMetaModel[label];
				else
				{
					iFirstMM = 0;
					iLastMM = pClassifier->classes[iClass].metamodels[label].size() - 1;
				}

				for (iMM = iFirstMM; iMM <= iLastMM; iMM++)
				{
					metamodel = pClassifier->classes[iClass].metamodels[label][iMM];

					nMMComponents = metamodel.no + metamodel.nu;

					for (iComp = 0; iComp < nMMComponents; iComp++)
					{
						if (bConcave[iSSegment])
						{
							if (iComp < metamodel.no)
								continue;
						}
						else
						{
							if (iComp >= metamodel.no)
								continue;
						}

						if (metamodel.bAssigned[iComp])
							continue;

						costReduction = -metamodel.conditionalProbability.Element[metamodel.conditionalProbability.w * iSSegment + iComp] + unassignedPointsLogProbability;

						if (costReduction > maxCostReduction)
						{
							maxCostReduction = costReduction;

							iSSegmentBestAssociation = iSSegment;
							bestAssociation.label = label;
							bestAssociation.iMetaModel = iMM;
							bestAssociation.iComponent = iComp;
						}
					}
				}
			}
		}

		if (iSSegmentBestAssociation < 0)
			break;

		association[iSSegmentBestAssociation] = bestAssociation;

		bSCompAssigned[iSSegmentBestAssociation] = true;
		pClassifier->classes[iClass].metamodels[bestAssociation.label][bestAssociation.iMetaModel].bAssigned[bestAssociation.iComponent] = true;

		iMetaModel[bestAssociation.label] = bestAssociation.iMetaModel;

		LockCells(iSSegmentBestAssociation, bCellAssigned);
	}	// greedy search loop
}

void VNInstance::GreedyProbabilisticAssociation(
	void *vpClassifier,
	int iClass,
	bool *bLabelLocked,
	bool *bCellAssigned,
	Array<RECOG::VN_::PartAssociation> &association)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int nLabels = pClassifier->classes[iClass].nLabels;

	memset(bLabelLocked, 0, nLabels * sizeof(bool));

	memset(bCellAssigned, 0, surfelCells.n * sizeof(bool));

	association.n = 0;

	float costReduction;
	int label, iMM, iGauss;

	while (true)
	{
		costReduction = GreedyProbabilisticAssociation(vpClassifier, iClass, bLabelLocked, bCellAssigned, label, iMM, iGauss);

		if (costReduction > 0.0f)
		{
			bLabelLocked[label] = true;

			AddAssociation(vpClassifier, iClass, label, iMM, iGauss, association, bCellAssigned);
		}
		else
			break;
	}
}

float VNInstance::GreedyProbabilisticAssociation(
	void *vpClassifier,
	int iClass,
	bool *bLabelLocked,
	bool *bCellAssigned,
	int &bestAssociationLabel,
	int &bestAssociationMetaModel,
	int &bestAssociationGauss)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	float kUnassignedPointsLogProbability = pClassifier->dUnassignedPointsLogProbability / pClassifier->dUnassignedPoints;

	int nLabels = pClassifier->classes[iClass].nLabels;

	int maxnGaussPerComponent = pClassifier->classes[iClass].maxnGaussPerComponent;

	float maxCostReduction = 0.0f;

	int i, j, k, label, iMM, nMM, iMMGauss, iGauss, iMMComp, iSSegment, iCell, nMMComps;
	VN_::ModelTemplate2 metamodel;
	float costReduction, unassignedSupportSubsetSize, unassignedPointsLogProbability;
	Array<int> activeMetamodelGauss;

	for (label = 0; label < nLabels; label++)
	{
		if (bLabelLocked[label])
			continue;

		nMM = pClassifier->classes[iClass].metamodels[label].size();

		for (k = 0; k < pClassifier->probabilisticAssociationWorkData.activeMetaModels[label].n; k++)
		{
			iMM = pClassifier->probabilisticAssociationWorkData.activeMetaModels[label].Element[k];

			metamodel = pClassifier->classes[iClass].metamodels[label][iMM];

			nMMComps = metamodel.no + metamodel.nu;

			activeMetamodelGauss = pClassifier->activeMetamodelGauss[metamodel.idx];

			for (i = 0; i < activeMetamodelGauss.n; i++)
			{
				iMMGauss = activeMetamodelGauss.Element[i];

				pWorkData->newAssignedCells.n = 0;

				unassignedSupportSubsetSize = 0;

				for (iMMComp = 0; iMMComp < nMMComps; iMMComp++)
				{
					iGauss = iMMGauss + iMMComp;

					iSSegment = pClassifier->metamodelGaussAssociation[iGauss];

					for (j = 0; j < componentSurfelCells[iSSegment].n; j++)
					{
						iCell = componentSurfelCells[iSSegment].Element[j];

						if (!bCellAssigned[iCell])
						{
							unassignedSupportSubsetSize += surfelCells.Element[iCell].size;

							bCellAssigned[iCell] = true;

							pWorkData->newAssignedCells.Element[pWorkData->newAssignedCells.n++] = iCell;
						}
					}
				}

				for (j = 0; j < pWorkData->newAssignedCells.n; j++)
					bCellAssigned[pWorkData->newAssignedCells.Element[j]] = false;

				unassignedPointsLogProbability = kUnassignedPointsLogProbability * (float)unassignedSupportSubsetSize / (float)supportSize;

				costReduction = -pClassifier->metamodelGaussAssociationCost[iMMGauss] + unassignedPointsLogProbability;

				if (costReduction > maxCostReduction)
				{
					maxCostReduction = costReduction;

					bestAssociationLabel = label;
					bestAssociationMetaModel = iMM;
					bestAssociationGauss = iMMGauss;
				}
			}
		}
	}

	return maxCostReduction;
}

void VNInstance::ProbabilisticAssociation(
	void *vpClassifier,
	int iClass,
	VN_::PartAssociation *&association)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	InitAssociationProbability(vpClassifier, iClass);

	// Greedy search.

	int nLabels = pClassifier->classes[iClass].nLabels;

	int nMMCompsTotal = 0;

	int label, iMM, iComp, nMMComponents;
	VN_::ModelTemplate2 metamodel;

	for (label = 0; label < nLabels; label++)
		for (iMM = 0; iMM < pClassifier->classes[iClass].metamodels[label].size(); iMM++)
		{
			metamodel = pClassifier->classes[iClass].metamodels[label][iMM];

			nMMComponents = metamodel.no + metamodel.nu;

			nMMCompsTotal += nMMComponents;
		}

	bool *bMMCompAssigned = new bool[nMMCompsTotal];

	memset(bMMCompAssigned, 0, nMMCompsTotal * sizeof(bool));

	bool *pbMMCompAssigned = bMMCompAssigned;

	for (label = 0; label < nLabels; label++)
		for (iMM = 0; iMM < pClassifier->classes[iClass].metamodels[label].size(); iMM++)
		{
			pClassifier->classes[iClass].metamodels[label][iMM].bAssigned = pbMMCompAssigned;

			metamodel = pClassifier->classes[iClass].metamodels[label][iMM];

			nMMComponents = metamodel.no + metamodel.nu;

			pbMMCompAssigned += nMMComponents;
		}

	bool *bCellAssigned = new bool[surfelCells.n];

	memset(bCellAssigned, 0, surfelCells.n * sizeof(bool));

	int *iMetaModel = new int[nLabels];

	memset(iMetaModel, 0xff, nLabels * sizeof(int));

	int m = RVLMAX(pClassifier->classArray.Element[0].M.h, pClassifier->classArray.Element[1].M.h);

	bool *bSCompAssigned = new bool[nComponents];

	memset(bSCompAssigned, 0, nComponents * sizeof(bool));

	association = new VN_::PartAssociation[nComponents];

	memset(association, 0xff, nComponents * sizeof(VN_::PartAssociation));

	Array<VN_::PartAssociation> association_;

	association_.Element = new VN_::PartAssociation[nLabels * pClassifier->classes[iClass].maxnMetamodelComponents];

	bool *bLabelLocked = new bool[nLabels];

	VN_::VNInstanceWorkData workData;

	workData.newAssignedCells.Element = new int[surfelCells.n];

	pWorkData = &workData;

	float cost;

	if (pClassifier->bPartSegmentationMetamodelClusters)
	{
		GreedyProbabilisticAssociation(vpClassifier, iClass, bLabelLocked, bCellAssigned, association_);

		cost = AssociationProbability(vpClassifier, iClass, association_, bCellAssigned);
	}
	else
	{
		GreedyProbabilisticAssociation(vpClassifier, iClass, -1, iMetaModel, bSCompAssigned, bCellAssigned, association);

		cost = AssociationProbability(vpClassifier, iClass, association, bCellAssigned);
	}

	//// Local search.

	int metamodelChangeProbabilityInPercentage = (int)round(pClassifier->metamodelChangeProbability * 100.0f);

	int labelChangeProbabilityInPercentage = (int)round(pClassifier->labelChangeProbability * 100.0f);

	VN_::PartAssociation *bestAssociation = new VN_::PartAssociation[nComponents];

	memcpy(bestAssociation, association, nComponents * sizeof(VN_::PartAssociation));

	Array<VN_::PartAssociation> bestAssociation_;

	bestAssociation_.Element = new VN_::PartAssociation[nLabels * pClassifier->classes[iClass].maxnMetamodelComponents];

	memcpy(bestAssociation_.Element, association_.Element, association_.n * sizeof(VN_::PartAssociation));
	bestAssociation_.n = association_.n;

	VN_::PartAssociation *newAssociation = new VN_::PartAssociation[nComponents];

	Array<VN_::PartAssociation> newAssociation_;

	newAssociation_.Element = new VN_::PartAssociation[nLabels * pClassifier->classes[iClass].maxnMetamodelComponents];

	float minCost = cost;

	int i, k, iSample, iSSegment, iSSegment_, label1, label2, iGauss, iAssociation, iAssociation_;
	VN_::PartAssociation associationTmp;
	bool bSuccess;
	VN_::PartAssociation assoc;
	float costReduction;

	for (k = 0; k < pClassifier->nIterationsComponentAssociationLocalSearch; k++)
	{
		for (iSample = 0; iSample< pClassifier->nSamplesComponentAssociationLocalSearch; iSample++)
		{
			/// Perturbation.

			// newAssociation <- association

			if (pClassifier->bPartSegmentationMetamodelClusters)
			{
				if (rand() % 100 <= labelChangeProbabilityInPercentage)
				{
					// Randomly select two different labels.

					label1 = rand() % nLabels;

					do
					{
						label2 = rand() % nLabels;
					} while (label2 == label1);

					// Lock all cells belonging to all labels except label1 and label2.
					// Copy all associations related to all labels except label1 and label2 to newAssociation_;

					memset(bCellAssigned, 0, surfelCells.n * sizeof(bool));

					newAssociation_.n = 0;

					for (i = 0; i < association_.n; i++)
					{
						assoc = association_.Element[i];

						if (assoc.label != label1 && assoc.label != label2)
						{
							LockCells(assoc.iSceneSegment, bCellAssigned);

							newAssociation_.Element[newAssociation_.n++] = assoc;
						}
					}

					// Lock all labels except label1.

					for (label = 0; label < nLabels; label++)
						bLabelLocked[label] = (label != label1);

					// Greedy association.

					costReduction = GreedyProbabilisticAssociation(vpClassifier, iClass, bLabelLocked, bCellAssigned, label, iMM, iGauss);

					// If cost can be reduced by label1, then add it to association_.

					if (costReduction > 0.0f)
						AddAssociation(vpClassifier, iClass, label, iMM, iGauss, newAssociation_, bCellAssigned);

					// Lock label1 and unlock label2.

					bLabelLocked[label1] = true;
					bLabelLocked[label2] = false;

					// Greedy association.

					costReduction = GreedyProbabilisticAssociation(vpClassifier, iClass, bLabelLocked, bCellAssigned, label, iMM, iGauss);

					// If cost can be reduced by label2, then add it to association_.

					if (costReduction > 0.0f)
						AddAssociation(vpClassifier, iClass, label, iMM, iGauss, newAssociation_, bCellAssigned);
				}
				else
				{
					int no = 0;
					int nu = 0;

					for (iAssociation = 0; iAssociation < association_.n; iAssociation++)
						if (bConcave[association_.Element[iAssociation].iSceneSegment])
							nu++;
						else
							no++;
					
					if (no > 1 || nu > 1)
					{
						bSuccess = false;

						while (!bSuccess)	// Repeat until a metamodel is sucessfully changed.
						{
							// Randomly select two associations.

							iAssociation = rand() % association_.n;
							iAssociation_ = rand() % association_.n;

							if (iAssociation_ != iAssociation)
							{
								iSSegment = association_.Element[iAssociation].iSceneSegment;
								iSSegment_ = association_.Element[iAssociation_].iSceneSegment;

								if (bConcave[iSSegment_] == bConcave[iSSegment])
								{
									// Switch the scene segments of the two selected associations.

									association_.Element[iAssociation].iSceneSegment = iSSegment_;
									association_.Element[iAssociation_].iSceneSegment = iSSegment;

									bSuccess = true;
								}
							}
						}
					}
				}

				// Compute cost.

				cost = AssociationProbability(vpClassifier, iClass, newAssociation_, bCellAssigned);

				// bestAssociation_ <- the best association

				if (cost < minCost)
				{
					minCost = cost;

					memcpy(bestAssociation_.Element, newAssociation_.Element, newAssociation_.n * sizeof(VN_::PartAssociation));
					bestAssociation_.n = newAssociation_.n;
				}
			}	// if (pClassifier->bPartSegmentationMetamodelClusters)
			else
			{
				// newAssociation <- association

				memcpy(newAssociation, association, nComponents * sizeof(VN_::PartAssociation));

				bSuccess = false;

				while (!bSuccess)	// Repeat until a metamodel is sucessfully changed.
				{
					// Randomly select a scene segment.

					iSSegment = rand() % nComponents;

					if (rand() % 100 <= metamodelChangeProbabilityInPercentage)
					{
						// Change metamodel of all scene segments associated with the same label as the selected segment.

						label = association[iSSegment].label;

						if (label < 0)
							label = rand() % nLabels;

						if (pClassifier->classes[iClass].metamodels[label].size() > 0)
						{
							for (iSSegment_ = 0; iSSegment_ < nComponents; iSSegment_++)
								if (association[iSSegment_].label >= 0)
								{
									if (association[iSSegment_].label == label)
									{
										newAssociation[iSSegment_].label = newAssociation[iSSegment_].iMetaModel = newAssociation[iSSegment_].iComponent = -1;

										bSCompAssigned[iSSegment_] = false;
									}
									else
										bSCompAssigned[iSSegment_] = true;
								}
								else
									bSCompAssigned[iSSegment_] = false;

							iMM = rand() % pClassifier->classes[iClass].metamodels[label].size();

							if (iMM != association[iSSegment].iMetaModel)
							{
								iMetaModel[label] = iMM;

								metamodel = pClassifier->classes[iClass].metamodels[label][iMM];

								nMMComponents = metamodel.no + metamodel.nu;

								for (iComp = 0; iComp < nMMComponents; iComp++)
									metamodel.bAssigned[iComp] = false;

								memset(bCellAssigned, 0, surfelCells.n * sizeof(bool));

								GreedyProbabilisticAssociation(vpClassifier, iClass, label, iMetaModel, bSCompAssigned, bCellAssigned, newAssociation);

								//for (iSSegment_ = 0; iSSegment_ < nComponents; iSSegment_++)
								//	if (newAssociation[iSSegment_].label == label)
								//		if (newAssociation[iSSegment_].iMetaModel != iMM)
								//			int debug = 0;

								bSuccess = true;
							}
						}
					}
					else
					{
						// Switch the associations of the selected segment and another randomly selected scene segment.

						iSSegment_ = rand() % nComponents;

						if (iSSegment_ != iSSegment)
						{
							if (bConcave[iSSegment_] == bConcave[iSSegment])
							{
								if (newAssociation[iSSegment].label >= 0 || newAssociation[iSSegment_].label >= 0)
								{
									associationTmp = newAssociation[iSSegment];

									newAssociation[iSSegment] = newAssociation[iSSegment_];

									newAssociation[iSSegment_] = associationTmp;

									bSuccess = true;
								}
							}
						}
					}
				}	// Repeat until a metamodel is sucessfully changed.

				///

				// Compute cost.

				cost = AssociationProbability(vpClassifier, iClass, newAssociation, bCellAssigned);

				// bestAssociation <- the best association

				if (cost < minCost)
				{
					minCost = cost;

					memcpy(bestAssociation, newAssociation, nComponents * sizeof(VN_::PartAssociation));
				}
			}	// if (!pClassifier->bPartSegmentationMetamodelClusters)
		}	// for every sample

		// association <- bestAssociation

		if (pClassifier->bPartSegmentationMetamodelClusters)
		{
			memcpy(association_.Element, bestAssociation_.Element, bestAssociation_.n * sizeof(VN_::PartAssociation));
			association_.n = bestAssociation_.n;
		}
		else
			memcpy(association, bestAssociation, nComponents * sizeof(VN_::PartAssociation));
	}	// for every iteration

	//// END local search

	// Create association from association_ (This is not made properly!).

	if (pClassifier->bPartSegmentationMetamodelClusters)
	{
		printf("Association:\n\n");

		for (iAssociation = 0; iAssociation < association_.n; iAssociation++)
		{
			assoc = association_.Element[iAssociation];

			association[assoc.iSceneSegment] = assoc;

			printf("Segment %d label %d metamodel %d component %d\n", assoc.iSceneSegment, assoc.label, assoc.iMetaModel, assoc.iComponent);
		}
	}

	// Free memory.

	delete[] bCellAssigned;
	delete[] iMetaModel;
	delete[] bMMCompAssigned;
	delete[] bSCompAssigned;
	delete[] bestAssociation;
	delete[] newAssociation;
	delete[] association_.Element;
	delete[] newAssociation_.Element;
	delete[] bestAssociation_.Element;
	delete[] bLabelLocked;
	delete[] workData.newAssignedCells.Element;
}

float VNInstance::AssociationProbability(
	void *vpClassifier,
	int iClass,
	VN_::PartAssociation *association,
	bool *bAssigned)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	//float logPriord = log(pClassifier->priord);

	memset(bAssigned, 0, surfelCells.n * sizeof(bool));

	//float probabilityxANDmi;
	float probabilityDs_xANDmi = 0.0f;
	//int nDetectedComponents;
	int i, iCell;
	//int factoriel;
	//int exp1, exp2;
	//float beta, gama;
	VN_::ModelTemplate2 metamodel;

	//Calculate p(Ds|x AND mi_k)
	for (int iSSegment = 0; iSSegment < nComponents; iSSegment++)
	{
		if (association[iSSegment].label >= 0)
		{
			metamodel = pClassifier->classes[iClass].metamodels[association[iSSegment].label][association[iSSegment].iMetaModel];

			probabilityDs_xANDmi += metamodel.conditionalProbability.Element[metamodel.conditionalProbability.w * iSSegment + association[iSSegment].iComponent];

			//increase the number of detected components
			//nDetectedComponents++;
			//factoriel *= nDetectedComponents;

			for (i = 0; i < componentSurfelCells[iSSegment].n; i++)
			{
				iCell = componentSurfelCells[iSSegment].Element[i];

				bAssigned[iCell] = true;
			}
		}
		//else
		//{
			//probabilityDs_xANDmi -= logPriord;
			//probabilityDs_xANDmi *= priorProbability.Element[iSSegment];

		//}
	}

	int nUnassignedPts = 0;

	for (iCell = 0; iCell < surfelCells.n; iCell++)
		if (!bAssigned[iCell])
			nUnassignedPts += surfelCells.Element[iCell].size;

	float kUnassignedPointsLogProbability = pClassifier->dUnassignedPointsLogProbability / pClassifier->dUnassignedPoints;

	probabilityDs_xANDmi += (kUnassignedPointsLogProbability * (float)nUnassignedPts / (float)supportSize);

	//Calculate p(x AND mi_k)
	//exp1 = nComponents - pModelTemplate->iComponentCluster.n;
	//exp2 = pModelTemplate->iComponentCluster.n - nDetectedComponents;
	//beta = pClassifier->beta;
	//gama = pClassifier->gama;

	//probabilityxANDmi = (pow(beta, exp1) * (1 - beta) * pow(gama, exp2) * (1 - gama)) / factoriel;
	//probabilityxANDmi *= pModelTemplate->p;

	//Calculate p(x AND mi_k | Ds)

	return probabilityDs_xANDmi;
}

float VNInstance::AssociationProbability(
	void *vpClassifier,
	int iClass,
	Array<RECOG::VN_::PartAssociation> association,
	bool *bAssigned)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	Array<int> *sceneSegments = pClassifier->associationProbabilityWorkData.sceneSegments;

	int nLabels = pClassifier->classes[iClass].nLabels;

	int label;

	for (label = 0; label < nLabels; label++)
		sceneSegments[label].n = 0;

	memset(bAssigned, 0, surfelCells.n * sizeof(bool));

	memset(pClassifier->associationProbabilityWorkData.metamodel, 0xff, nLabels * sizeof(int));

	int i, iSSegment, iCell;
	VN_::ModelTemplate2 metamodel;

	for (i = 0; i < association.n; i++)
	{
		iSSegment = association.Element[i].iSceneSegment;

		label = association.Element[i].label;

		sceneSegments[label].Element[association.Element[i].iComponent] = iSSegment;

		sceneSegments[label].n++;

		pClassifier->associationProbabilityWorkData.metamodel[label] = association.Element[i].iMetaModel;
	}

	float cost = 0.0f;

	int j, iMM;

	for (label = 0; label < nLabels; label++)
	{
		iMM = pClassifier->associationProbabilityWorkData.metamodel[label];

		if (iMM < 0)
			continue;

		metamodel = pClassifier->classes[iClass].metamodels[label][iMM];

		if (sceneSegments[label].n != metamodel.no + metamodel.nu)
			continue;

		cost += AssociationProbability(vpClassifier, iClass, sceneSegments[label].Element, label, iMM);

		for (j = 0; j < sceneSegments[label].n; j++)
		{
			iSSegment = sceneSegments[label].Element[j];

			for (i = 0; i < componentSurfelCells[iSSegment].n; i++)
			{
				iCell = componentSurfelCells[iSSegment].Element[i];

				bAssigned[iCell] = true;
			}
		}
	}

	int nUnassignedPts = 0;

	for (iCell = 0; iCell < surfelCells.n; iCell++)
		if (!bAssigned[iCell])
			nUnassignedPts += surfelCells.Element[iCell].size;

	float kUnassignedPointsLogProbability = pClassifier->dUnassignedPointsLogProbability / pClassifier->dUnassignedPoints;

	cost += (kUnassignedPointsLogProbability * (float)nUnassignedPts / (float)supportSize);

	return cost;
}

float VNInstance::AssociationProbability(
	void *vpClassifier,
	int iClass,
	int *iSSegment__,
	int label,
	int iMetaModel)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	Array<VN_::PartAssociation> componentID = pClassifier->classes[iClass].componentID;

	VN_::ModelTemplate2 metamodel = pClassifier->classes[iClass].metamodels[label][iMetaModel];	

	Array<int> metaModelGauss = pClassifier->classes[iClass].metaModelGauss[metamodel.idx];

	Array<VN_::Gauss> gauss = pClassifier->classes[iClass].gauss;

	int nMMComps = metamodel.no + metamodel.nu;

	int *nActiveComps = pClassifier->associationProbabilityWorkData.nActiveComps;

	memset(nActiveComps, 0, metaModelGauss.n * sizeof(int));

	int iLastMComp = nMMComps - 1;

	int j, iMMComp, iMMCompAbs, iSSegment, iMMGauss, iMMComp_, iSSegment_, iMatch, iGauss;
	Array<int> *pActiveGauss;
	float pdf, e, g;
	VN_::Gauss *pGauss;

	for (iMMComp = iLastMComp; iMMComp >= 0; iMMComp--)
	{
		iMMCompAbs = metamodel.componentIdx[iMMComp];

		iSSegment = iSSegment__[iMMComp];

		pActiveGauss = pClassifier->activeGauss + iSSegment * componentID.n + iMMCompAbs;

		if (iMMComp > 0)
			for (j = 0; j < pActiveGauss->n; j++)
				nActiveComps[pActiveGauss->Element[j]]++;
		else
		{
			pdf = 0.0f;

			for (j = 0; j < pActiveGauss->n; j++)
			{
				iMMGauss = pActiveGauss->Element[j];

				if (nActiveComps[iMMGauss] == iLastMComp)
				{
					g = 0.0f;

					for (iMMComp_ = 0; iMMComp_ < nMMComps; iMMComp_++)
					{
						iGauss = iMMGauss + iMMComp_;

						iSSegment_ = pClassifier->metamodelGaussAssociation[iGauss];

						iMatch = gauss.n * iSSegment + iGauss;

						e = pClassifier->esComponent.Element[iMatch] + pClassifier->etComponent.Element[iMatch];

						g += e;
					}

					pGauss = gauss.Element + iMMGauss;

					pdf += (float)(pGauss->w) * exp(-0.5f * g);
				}
			}
		}
	}

	return -log(pdf);
}

void VNInstance::InitAssociationProbability(
	void *vpClassifier, 
	int iClass)
{
	// Parameters.

	int eThr = 42.98;

	//

	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int nLabels = pClassifier->classes[iClass].nLabels;

	Array<VN_::Gauss> gauss = pClassifier->classes[iClass].gauss;

	Array<int> *metaModelGauss = pClassifier->classes[iClass].metaModelGauss;

	//float sigma2_2 = (2 * pClassifier->sigmaP * pClassifier->sigmaP);
	//float sigma2_2 = 1.0f;

	//alocate memory for prior probability matrix
	//priorProbability.n = nComponents;

	//RVL_DELETE_ARRAY(priorProbability.Element);
	//priorProbability.Element = new float[priorProbability.n];

	float *dd = new float[pClassifier->sampledUnitSphere.h];

	VN_::ModelTemplate2 metamodel;

	Array<VN_::PartAssociation> componentID = pClassifier->classes[iClass].componentID;

	Array<Pair<int, int>> metamodelID = pClassifier->classes[iClass].metamodelID;

	int maxnGaussPerComponent = pClassifier->classes[iClass].maxnGaussPerComponent;

	Array2D<float> etMx, esMx;
	bool *bActive;

	if (pClassifier->bPartSegmentationMetamodelClusters)
	{
		int gaussMemSize = nComponents * gauss.n;

		RVL_DELETE_ARRAY(pClassifier->esComponent.Element);

		pClassifier->esComponent.Element = new float[gaussMemSize];

		RVL_DELETE_ARRAY(pClassifier->etComponent.Element);

		pClassifier->etComponent.Element = new float[gaussMemSize];

		pClassifier->esComponent.w = pClassifier->etComponent.w = gauss.n;
		pClassifier->esComponent.h = pClassifier->etComponent.h = nComponents;

		esMx = pClassifier->esComponent;
		etMx = pClassifier->etComponent;

		RVL_DELETE_ARRAY(pClassifier->activeGauss);

		pClassifier->activeGauss = new Array<int>[nComponents * componentID.n];

		RVL_DELETE_ARRAY(pClassifier->activeGaussMem);

		pClassifier->activeGaussMem = new int[gaussMemSize];

		RVL_DELETE_ARRAY(pClassifier->metamodelGaussAssociation);

		pClassifier->metamodelGaussAssociation = new int[gauss.n];

		RVL_DELETE_ARRAY(pClassifier->metamodelGaussAssociationCost);

		pClassifier->metamodelGaussAssociationCost = new float[gauss.n];

		bActive = new bool[gaussMemSize];

		memset(bActive, 0, gaussMemSize * sizeof(bool));

		RVL_DELETE_ARRAY(pClassifier->activeMetamodelGauss);

		pClassifier->activeMetamodelGauss = new Array<int>[metamodelID.n];

		pClassifier->activeMetamodelGaussMem = new int[gauss.n];
	}
	else
	{
		for (int iLabel = 0; iLabel < nLabels; iLabel++)
		{
			int nMM = pClassifier->classes[iClass].metamodels[iLabel].size();
			for (int iMM = 0; iMM < nMM; iMM++)
			{
				metamodel = pClassifier->classes[iClass].metamodels[iLabel][iMM];

				RVL_DELETE_ARRAY(pClassifier->classes[iClass].metamodels[iLabel][iMM].conditionalProbability.Element);

				int nMComps = metamodel.no + metamodel.nu;

				pClassifier->classes[iClass].metamodels[iLabel][iMM].conditionalProbability.Element = new float[nComponents * nMComps];
				pClassifier->classes[iClass].metamodels[iLabel][iMM].conditionalProbability.h = nComponents;
				pClassifier->classes[iClass].metamodels[iLabel][iMM].conditionalProbability.w = nMComps;
			}
		}
	}
	
	int m = RVLMAX(pClassifier->classArray.Element[0].M.h, pClassifier->classArray.Element[1].M.h);

	int *pActiveGaussIdxArray = pClassifier->activeGaussMem;

	int i, j, iLabel, iSSegment, iMM, iMMComp, iMMCompAbs, iFirstMMComp, iMMCompEnd, iMatch, iGauss;
	//int j;
	float e, g, et, es, ea;
	//float *dS, *dM;
	float *qS, *qM;
	VN_::Gauss *pGauss;
	float conditionalProbability;
	VN_::PartAssociation componentID_;
	Array<int> *pActiveGauss;

	for (iSSegment = 0; iSSegment < nComponents; iSSegment++)
	{
		//dS = d + iSSegment * pClassifier->sampledUnitSphere.h;

		qS = q + m * iSSegment;

		//priorProbability.Element[iSSegment] = 0.0f;

		if (pClassifier->bPartSegmentationMetamodelClusters)
		{
			// pClassifier->esComponent, pClassifier->etComponent <- costs of matching every model component with every scene segment 
			// pClassifier->activeGauss(i, j) <- array of gaussians activated by matching i-th scene segment with j-th model component 

			for (iMMCompAbs = 0; iMMCompAbs < componentID.n; iMMCompAbs++)
			{
				j = iSSegment * componentID.n + iMMCompAbs;

				pActiveGauss = pClassifier->activeGauss + j;

				pActiveGauss->Element = pActiveGaussIdxArray;

				pActiveGauss->n = 0;

				componentID_ = componentID.Element[iMMCompAbs];

				metamodel = pClassifier->classes[iClass].metamodels[componentID_.label][componentID_.iMetaModel];

				if (bConcave[iSSegment] == (componentID_.iComponent >= metamodel.no))
				{
					for (i = 0; i < metaModelGauss[metamodel.idx].n; i++)
					{
						iGauss = metaModelGauss[metamodel.idx].Element[i] + componentID_.iComponent;

						pGauss = gauss.Element + iGauss;

						qM = pGauss->x;

						e = pClassifier->MatchLatentVectors(qS, s[iSSegment], qM, pGauss->abs, m, et, es, ea, dd, 2);

						if (e <= eThr)
						{
							iMatch = gauss.n * iSSegment + iGauss;

							esMx.Element[iMatch] = es;
							etMx.Element[iMatch] = et;

							bActive[iMatch] = true;

							pActiveGauss->Element[pActiveGauss->n++] = iGauss;
						}
					}
				}

				pActiveGaussIdxArray += pActiveGauss->n;
			}
		}
		else
		{
			for (iLabel = 0; iLabel < nLabels; iLabel++)
			{
				int nMM = pClassifier->classes[iClass].metamodels[iLabel].size();

				for (iMM = 0; iMM < nMM; iMM++)
				{
					metamodel = pClassifier->classes[iClass].metamodels[iLabel][iMM];

					if (bConcave[iSSegment])
					{
						iFirstMMComp = metamodel.no;
						iMMCompEnd = metamodel.no + metamodel.nu;
					}
					else
					{
						iFirstMMComp = 0;
						iMMCompEnd = metamodel.no;
					}

					for (iMMComp = iFirstMMComp; iMMComp < iMMCompEnd; iMMComp++)
					{
						conditionalProbability = 0.0f;

						for (i = 0; i < metaModelGauss[metamodel.idx].n; i++)
						{
							iGauss = metaModelGauss[metamodel.idx].Element[i] + iMMComp;

							pGauss = gauss.Element + iGauss;

							//dM = pGauss->x;

							//RVLDIFVECTORS(dS, dM, pClassifier->sampledUnitSphere.h, dd, j);
							//RVLDOTPRODUCT(dd, dd, pClassifier->sampledUnitSphere.h, e, j);

							qM = pGauss->x;

							e = pClassifier->MatchLatentVectors(qS, s[iSSegment], qM, pGauss->abs, m, et, es, ea, dd, 2);

							g = (float)(pGauss->w) * exp(-0.5 * e);

							conditionalProbability += g;
						}

						//priorProbability.Element[iSSegment] += conditionalProbability;

						//if (iSSegment == 1 && iLabel == 1 && metamodel.no == 2 && metamodel.nu == 1 && iMMComp == 0)
						//	int debug = 0;

						if (conditionalProbability < pClassifier->priorProbabilityComponent)
							conditionalProbability = pClassifier->priorProbabilityComponent;

						//else
						//	int debug = 0;

						metamodel.conditionalProbability.Element[metamodel.conditionalProbability.w * iSSegment + iMMComp] = -log(conditionalProbability);
					}
				}
			}
		}
	}

	delete[] dd;

	if (pClassifier->bPartSegmentationMetamodelClusters)
	{
		// pClassifier->metamodelGaussAssociation(i : i + n - 1) <- best match scene segments for a metamodel gaussian,
		// where i : i + n - 1 is the sequence of gaussians representing a metamodel gaussian.
		// pClassifier->activeMetamodelGauss(i) <- array of active gaussians of the i-th metamodel
		// A metamodel gaussian is active if all its component gaussians are active.
		// pClassifier->metamodelGaussAssociationCost(iMMGauss) <- cost of matching the scene segment sequence contained in 
		// pClassifier->metamodelGaussAssociation(i : i + n - 1) to the sequence of gaussians i : i + n - 1
		// pClassifier->probabilisticAssociationWorkData.activeMetaModels(label) <- array of active metamodels of label
		// A metamodel is active if it has at least one active gaussian.

		for (iLabel = 0; iLabel < nLabels; iLabel++)
			pClassifier->probabilisticAssociationWorkData.activeMetaModels[iLabel].n = 0;

		int *iSSegment__ = new int[pClassifier->classes[iClass].maxnMetamodelComponents];

		int *pActiveMetamodelGaussIdxArray = pClassifier->activeMetamodelGaussMem;

		int nMComps, iSSegmentBestMatch, iMMAbs, iMMGauss;
		float mine;
		Array<int> activeMetamodelGauss;

		for (iMMAbs = 0; iMMAbs < metamodelID.n; iMMAbs++)
		{
			iLabel = metamodelID.Element[iMMAbs].a;

			iMM = metamodelID.Element[iMMAbs].b;

			metamodel = pClassifier->classes[iClass].metamodels[iLabel][iMM];

			pClassifier->activeMetamodelGauss[iMMAbs].Element = pActiveMetamodelGaussIdxArray;

			pClassifier->activeMetamodelGauss[iMMAbs].n = 0;

			nMComps = metamodel.no + metamodel.nu;

			for (i = 0; i < metaModelGauss[metamodel.idx].n; i++)
			{
				iMMGauss = metaModelGauss[metamodel.idx].Element[i];

				for (iMMComp = 0; iMMComp < nMComps; iMMComp++)
				{
					iGauss = iMMGauss + iMMComp;

					mine = 2.0f * eThr;

					for (iSSegment = 0; iSSegment < nComponents; iSSegment++)
					{
						iMatch = gauss.n * iSSegment + iGauss;

						if (bActive[iMatch])
						{
							e = esMx.Element[iMatch] + etMx.Element[iMatch];

							if (e < mine)
							{
								mine = e;

								iSSegmentBestMatch = iSSegment;
							}
						}
					}

					if (mine > eThr)
						break;

					pClassifier->metamodelGaussAssociation[iGauss] = iSSegmentBestMatch;
				}

				if (iMMComp >= nMComps)
					pClassifier->activeMetamodelGauss[iMMAbs].Element[pClassifier->activeMetamodelGauss[iMMAbs].n++] = iMMGauss;
			}

			activeMetamodelGauss = pClassifier->activeMetamodelGauss[iMMAbs];

			pActiveMetamodelGaussIdxArray += activeMetamodelGauss.n;

			for (i = 0; i < activeMetamodelGauss.n; i++)
			{
				iMMGauss = activeMetamodelGauss.Element[i];

				for (iMMComp = 0; iMMComp < nMComps; iMMComp++)
				{
					iGauss = iMMGauss + iMMComp;

					iSSegment__[iMMComp] = pClassifier->metamodelGaussAssociation[iGauss];
				}

				pClassifier->metamodelGaussAssociationCost[iMMGauss] = AssociationProbability(vpClassifier, iClass, iSSegment__, iLabel, iMM);
			}	// for every active metamodel Gaussian

			if (activeMetamodelGauss.n > 0)
				pClassifier->probabilisticAssociationWorkData.activeMetaModels[iLabel].Element[pClassifier->probabilisticAssociationWorkData.activeMetaModels[iLabel].n++] = iMM;
		}	// for every metamodel

		delete[] bActive;
		delete[] iSSegment__;
	}	// if (pClassifier->bPartSegmentationMetamodelClusters)
		 
	// Calculate prior probability	

	//for (iSSegment = 0; iSSegment < nComponents; iSSegment++)
	//{
	//	if (bConcave[iSSegment])
	//		priorProbability.Element[iSSegment] /= nConcaveComponentClusters;
	//	else
	//		priorProbability.Element[iSSegment] /= nConvexComponentClusters;

	//	if (priorProbability.Element[iSSegment] < pClassifier->priord)
	//		priorProbability.Element[iSSegment] = pClassifier->priord;
	//}
}

void VNInstance::NearestNeighborsAssociation(
	void *vpClassifier,
	int iClass,
	RECOG::VN_::PartAssociation *&association)
{
	// Parameters.

	int nNearestNeighbors = 20;
	float pThr = 0.2;

	//

	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int nLabels = pClassifier->classes[iClass].nLabels;

	int m = RVLMAX(pClassifier->classArray.Element[0].M.h, pClassifier->classArray.Element[1].M.h);

	// Match this to all models in pClassifier->modelVNArray.

	int *Correspondences = new int[nComponents * pClassifier->modelVNArray.n];

	float *E = new float[pClassifier->modelVNArray.n];

	Match(&(pClassifier->modelVNList), Correspondences, vpClassifier, E, NULL, 0, 0, true);

	// Identify the nNearestNeighbors nearest neighbors of this.

	bool *bNeighbor = new bool[pClassifier->modelVNArray.n];

	memset(bNeighbor, 0, pClassifier->modelVNArray.n * sizeof(bool));

	int *neighborID = new int[nNearestNeighbors];

	int iNeighbor, iModel, neighborID_;
	float minE;

	for (iNeighbor = 0; iNeighbor < nNearestNeighbors; iNeighbor++)
	{
		minE = -1.0f;

		for (iModel = 0; iModel < pClassifier->modelVNArray.n; iModel++)
		{
			if (bNeighbor[iModel])
				continue;

			if (minE < 0.0f || E[iModel] < minE)
			{
				minE = E[iModel];

				neighborID_ = iModel;
			}
		}

		neighborID[iNeighbor] = neighborID_;

		bNeighbor[neighborID_] = true;
	}

	// Compute association scores for all components of this and for all labels.

	float *p = new float[nComponents * nLabels];

	memset(p, 0, nComponents * nLabels * sizeof(float));

	float *eq = new float[m];

	int iSComp, iMComp, modelID;
	float et, es, ea, e;
	float *qS;
	VNInstance *pModel;

	for (iSComp = 0; iSComp < nComponents; iSComp++)
	{
		qS = q + iSComp * m;

		for (iNeighbor = 0; iNeighbor < nNearestNeighbors; iNeighbor++)
		{
			modelID = neighborID[iNeighbor];

			pModel = pClassifier->modelVNArray.Element[modelID];

			iMComp = Correspondences[nComponents * modelID + iSComp];

			if (iMComp >= 0)
			{
				e = pClassifier->MatchLatentVectors(qS, s[iSComp], pModel->q + iMComp * m, pModel->s[iMComp], m, et, es, ea, eq);

				p[nLabels * iSComp + pModel->label[iMComp]] += exp(-0.5f * e);
			}
		}
	}

	// Assign labels to the scene components.

	association = new VN_::PartAssociation[nComponents];

	int label;
	float maxp;

	for (iSComp = 0; iSComp < nComponents; iSComp++)
	{
		association[iSComp].label = -1;

		maxp = pThr;

		for (label = 0; label < nLabels; label++)
		{
			if (p[nLabels * iSComp + label] > maxp)
			{
				maxp = p[nLabels * iSComp + label];

				association[iSComp].label = label;
			}
		}
	}

	// Free memory.

	delete[] Correspondences;
	delete[] E;
	delete[] bNeighbor;
	delete[] neighborID;
	delete[] p;
	delete[] eq;
}

float VNInstance::Match(
	VNInstance *pModel,
	float *E,
	int strideS,
	int strideM,
	float *wCellIn)
{
	float *wCell;

	if (wCellIn)
		wCell = wCellIn;
	else
		wCell = new float[pModel->surfelCells.n];

	int iCell;

	for (iCell = 0; iCell < pModel->surfelCells.n; iCell++)
		wCell[iCell] = 0.0f;

	int i, iMComp, iSComp;
	float fComp, fComp_, fModel;
	Array<int> MComponentSurfelCells;

	for (iMComp = 0; iMComp < pModel->nComponents; iMComp++)
	{
		fComp = 0.0f;

		for (iSComp = 0; iSComp < nComponents; iSComp++)
		{
			if (bConcave[iSComp] == pModel->bConcave[iMComp])
			{
				fComp_ = E[strideS * iSComp + strideM * iMComp];

				if (fComp_ > fComp)
					fComp = fComp_;
			}
		}

		MComponentSurfelCells = pModel->componentSurfelCells[iMComp];

		for (i = 0; i < MComponentSurfelCells.n; i++)
		{
			iCell = MComponentSurfelCells.Element[i];

			if (fComp > wCell[iCell])
				wCell[iCell] = fComp;
		}
	}

	fModel = 0.0;

	for (iCell = 0; iCell < pModel->surfelCells.n; iCell++)
		fModel += (wCell[iCell] * (float)(pModel->surfelCells.Element[iCell].size));

	fModel /= (float)(pModel->supportSize);

	if (wCellIn == NULL)
		delete[] wCell;

	return fModel;
}

float VNInstance::Match2(
	VNInstance *pModel,
	float *fComp,
	int strideS,
	float *lnPDFSPrior,
	float *lnPDFMPrior)
{
	bool *bSMatched = new bool[nComponents];
	memset(bSMatched, 0, nComponents * sizeof(bool));

	bool *bMMatched = new bool[pModel->nComponents];
	memset(bMMatched, 0, pModel->nComponents * sizeof(bool));

	bool *bSCovered = new bool[surfelCells.n];
	memset(bSCovered, 0, surfelCells.n * sizeof(bool));

	bool *bMCovered = new bool[pModel->surfelCells.n];
	memset(bMCovered, 0, pModel->surfelCells.n * sizeof(bool));

	int nMatches = RVLMIN(nComponents, pModel->nComponents);

	float fModel = 0.0f;

	int i, iSComp, iMComp, iSCompBestMatch, iMCompBestMatch, iMatch, iCell, w, wS, wM, wSOrig, wMOrig;
	float fComp_, fBestMatch, orig, origS, origM, contribution;
	Array<int> Cells;

	for (iMatch = 0; iMatch < nMatches; iMatch++)
	{
#ifdef RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
		fBestMatch = -1.0f;
#else
		fBestMatch = 0.0f;
#endif

		for (iSComp = 0; iSComp < nComponents; iSComp++)
		{
			if (bSMatched[iSComp])
				continue;

			for (iMComp = 0; iMComp < pModel->nComponents; iMComp++)
			{
				if (bMMatched[iMComp])
					continue;

				if (bConcave[iSComp] != pModel->bConcave[iMComp])
					continue;

				fComp_ = fComp[strideS * iSComp + iMComp];

#ifdef RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
				if (fBestMatch < 0.0f || fComp_ < fBestMatch)
#else
				if (fComp_ > fBestMatch)
#endif
				{
					fBestMatch = fComp_;
					iSCompBestMatch = iSComp;
					iMCompBestMatch = iMComp;
				}
			}
		}

		bSMatched[iSCompBestMatch] = bMMatched[iMCompBestMatch] = true;

		wS = wSOrig = 0;

		Cells = componentSurfelCells[iSCompBestMatch];

		for (i = 0; i < Cells.n; i++)
		{
			iCell = Cells.Element[i];

			w = surfelCells.Element[iCell].size;

			wS += w;

			if (bSCovered[iCell])
				continue;

			wSOrig += w;

			bSCovered[iCell] = true;
		}

		origS = (float)wSOrig / (float)wS;

		wM = wMOrig = 0;

		Cells = pModel->componentSurfelCells[iMCompBestMatch];

		for (i = 0; i < Cells.n; i++)
		{
			iCell = Cells.Element[i];

			w = pModel->surfelCells.Element[iCell].size;

			wM += w;

			if (bMCovered[iCell])
				continue;

			wMOrig += w;

			bMCovered[iCell] = true;
		}

		origM = (float)wMOrig / (float)wM;

		orig = RVLMIN(origS, origM);

#ifdef RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
		contribution = fBestMatch + lnPDFSPrior[iSCompBestMatch] + lnPDFMPrior[iMCompBestMatch];

		if (contribution > 0.0f)
			contribution = 0.0f;
#else
		contribution = fBestMatch;
#endif
		fModel += (orig * contribution);
	}

	delete[] bSMatched;
	delete[] bMMatched;
	delete[] bSCovered;
	delete[] bMCovered;

	return fModel;
}

void VNInstance::CTINet(
	void *vpClassifier,
	int iClass,
	VN_::CTINetOutput &out,
	VN_::PartAssociation *&componentAssociation,
	VN_::PartAssociation *&cellAssociation,
	int *nMCompsTotalOU,
	float alpha,
	uchar flags,
	float *lnPDFPrior,
	int nModelsIn,
	int excludeModel)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int nLabels = pClassifier->classes[iClass].nLabels;

	int nModels = (nModelsIn > 0 ? nModelsIn : pClassifier->modelVNArray.n);

	// Parameters.

	float beta = 10.0f;
	float gamma = 5.0f;

	// Count all model components.
	// Determine the greatest number of model surfel cells.

	int nMCompsTotal = 0;

	int maxnSurfelCells = 0;

	int iModel;
	VNInstance *pModel;

	for (iModel = 0; iModel < nModels; iModel++)
	{
		pModel = pClassifier->modelVNArray.Element[iModel];

		nMCompsTotal += pModel->nComponents;

		if (pModel->surfelCells.n > maxnSurfelCells)
			maxnSurfelCells = pModel->surfelCells.n;
	}

	if (surfelCells.n > maxnSurfelCells)
		maxnSurfelCells = surfelCells.n;

	// Component layer: Match all components of this to all model components.

	int iSComp, iMComp, iMCompAbs;

	if (flags & RVLVN_CTINET_COMPONENT_LAYER)
	{
		if (out.fComp == NULL)
			out.fComp = new float[nComponents * nMCompsTotal];

		iMCompAbs = 0;

		int *Correspondences = NULL;
		int *correspondencesOrder = NULL;

		float kPDFPrior;

		if (flags & RVLVN_CTINET_COMPUTE_PDFPRIOR)
		{
			if (lnPDFPrior)
				memset(lnPDFPrior, 0, nComponents * sizeof(float));

			kPDFPrior = 0.5 / (alpha * alpha);
		}

		int ou, iMatch;
		float cost, PDFS, PDFSM;
		bool bSConcave;

		for (iModel = 0; iModel < nModels; iModel++)
		{
			pModel = pClassifier->modelVNArray.Element[iModel];

			//if (iModel != excludeModel)
			{
				if (flags & RVLVN_CTINET_LABELS)
					Match2Instances(pModel, Correspondences, vpClassifier, cost, correspondencesOrder, iModel, out.fComp + iMCompAbs, nMCompsTotal, false, false);
				else
					Match(vpClassifier, iModel, out);

				for (iSComp = 0; iSComp < nComponents; iSComp++)
				{
					PDFS = 0.0f;

					bSConcave = bConcave[iSComp];

					ou = (bSConcave ? 1 : 0);

					for (iMComp = 0; iMComp < pModel->nComponents; iMComp++)
						if (bSConcave == pModel->bConcave[iMComp])
						{
#ifndef RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
							iMatch = iSComp * nMCompsTotal + iMComp + iMCompAbs;
							out.fComp[iMatch] = exp(-0.5f * out.fComp[iMatch]);
							out.fShape[iMatch] = exp(-0.5f * out.fShape[iMatch]);
							out.fSize[iMatch] = exp(-0.5f * out.fSize[iMatch]);
#endif
							if (flags & RVLVN_CTINET_COMPUTE_PDFPRIOR)
							{
								PDFSM = exp(-kPDFPrior * out.fComp[iSComp * nMCompsTotal + iMComp + iMCompAbs]);

								PDFS += PDFSM;
							}
						}

					if (flags & RVLVN_CTINET_COMPUTE_PDFPRIOR)
						if (lnPDFPrior)
							lnPDFPrior[iSComp] += PDFS;
				}
			}

			iMCompAbs += pModel->nComponents;
		}

		if (flags & RVLVN_CTINET_COMPUTE_PDFPRIOR)
		{
			if (lnPDFPrior)
			{
				int nSComps[2] = { 0, 0 };

				for (iSComp = 0; iSComp < nComponents; iSComp++)
				{
					ou = (bSConcave ? 1 : 0);

					nSComps[ou]++;
				}

				for (iSComp = 0; iSComp < nComponents; iSComp++)
				{
					ou = (bSConcave ? 1 : 0);

					lnPDFPrior[iSComp] = log((float)nSComps[ou] * lnPDFPrior[iSComp] / (float)(nMCompsTotalOU[ou]));
				}
			}
		}

#ifdef	RVLVN_PART_SEGMENTATION_CTINET_LOG
		FILE *fpComps = fopen((std::string(pClassifier->resultsFolder) + "\\VNInstanceToClassCompLayer.dat").data(), "wb");

		fwrite(fComp, sizeof(float), nComponents * nMCompsTotal, fpComps);

		fclose(fpComps);
#endif
	}

	// Neighborhood layer: Match the component neighborhoods.

	if (flags & RVLVN_CTINET_NEIGHBORHOOD_LAYER)
	{
		if (out.fNeighborhood == NULL)
			out.fNeighborhood = new float[nComponents * nMCompsTotal];

		int i;

		for (i = 0; i < pClassifier->modelSet.n; i++)
		{
			iModel = pClassifier->modelSet.Element[i];

			if (iModel == excludeModel)
				continue;

			MatchComponentNeighborhood(vpClassifier, iModel, out);
		}
	}

	// Model layer: Compute the activation value for all models.

	if (flags & RVLVN_CTINET_MODEL_LAYER)
	{
		int iSCompAbs = 0;

		for (iModel = 0; iModel < excludeModel; iModel++)
		{
			pModel = pClassifier->modelVNArray.Element[iModel];

			iSCompAbs += pModel->nComponents;
		}

		if (out.fModel == NULL)
			out.fModel = new float[nModels];

		float *wCell = new float[maxnSurfelCells];

		iMCompAbs = 0;

		float fMS, fSM;
		float *tMS, *tSM;

		for (iModel = 0; iModel < nModels; iModel++)
		{
			pModel = pClassifier->modelVNArray.Element[iModel];

			if (iModel == excludeModel)
				out.fModel[iModel] = 0.0f;
			else
			{
				// Version 1

				//fMS = Match(pModel, fComp + iMCompAbs, nMCompsTotal, 1, wCell);
				//fSM = pModel->Match(this, fComp + iMCompAbs, 1, nMCompsTotal, wCell);

				//fModel[iModel] = fMS * fSM;

				// Version 2

#ifdef RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
				if (lnPDFPrior)
					fModel[iModel] = Match2(pModel, fComp + iMCompAbs, nMCompsTotal, lnPDFPrior + iSCompAbs, lnPDFPrior + iMCompAbs);
				else
					fModel[iModel] = Match2(pModel, fComp + iMCompAbs, nMCompsTotal);
#else
#ifdef RVLVN_PART_SEGMENTATION_CTINET_PCFIT
				//fSM = pModel->Fit(vpClassifier, PC, 1.0f, 0.02f, tSM);
				out.fModel[iModel] = Fit(vpClassifier, pModel->PC, pClassifier->componentAssociationAlphat, pClassifier->componentAssociationSig, tMS);
				delete[] tMS;
				//delete[] tSM;
#else
				fModel[iModel] = Match2(pModel, fComp + iMCompAbs, nMCompsTotal);
#endif
#endif
			}

			iMCompAbs += pModel->nComponents;

			printf("%d%%\r", 100 * iModel / pClassifier->modelVNArray.n);
		}

		printf("100%%\n");

		delete[] wCell;
	}

	if (flags & RVLVN_CTINET_GLOBAL_LOCAL_LAYER)
	{
		// Global + Local Layer: Combine model similarity and component similarity.

		float sumfModel = 0.0f;

		for (iModel = 0; iModel < nModels; iModel++)
		{
			out.fModel[iModel] = exp(beta * out.fModel[iModel]);

			sumfModel += out.fModel[iModel];
		}

		for (iModel = 0; iModel < nModels; iModel++)
			out.fModel[iModel] /= sumfModel;

		float *fLabel = new float[nComponents * nLabels];
		float *fLabelModel = new float[nModels * nComponents * nLabels];
		float *fCellLabel = new float[nLabels * surfelCells.n];
		componentAssociation = new VN_::PartAssociation[nComponents];
		cellAssociation = new VN_::PartAssociation[surfelCells.n];
		VN_::PartAssociation *componentAssociationModel = new VN_::PartAssociation[nComponents];
		VN_::PartAssociation *cellAssociationModel = new VN_::PartAssociation[surfelCells.n];

		int k, label, iCell, TP;
		float fComp_, fModel_;
		float *fLabel_;

		for (k = 0; k < 3; k++)
		{
			for (iSComp = 0; iSComp < nComponents; iSComp++)
			{
				for (label = 0; label < nLabels; label++)
					fLabel[iSComp * nLabels + label] = 0.0f;

				iMCompAbs = 0;

				for (iModel = 0; iModel < nModels; iModel++)
				{
					pModel = pClassifier->modelVNArray.Element[iModel];

					fLabel_ = fLabelModel + nLabels * (nComponents * iModel + iSComp);

					for (label = 0; label < nLabels; label++)
						fLabel_[label] = 0.0f;

					for (iMComp = 0; iMComp < pModel->nComponents; iMComp++)
						if (bConcave[iSComp] == pModel->bConcave[iMComp])
						{
							fComp_ = out.fComp[iSComp * nMCompsTotal + iMComp + iMCompAbs];
							label = pModel->label[iMComp];

							if (fComp_ > fLabel_[label])
								fLabel_[label] = fComp_;
						}

					fModel_ = out.fModel[iModel];

					for (label = 0; label < nLabels; label++)
						fLabel[iSComp * nLabels + label] += (fModel_ * fLabel_[label]);

					iMCompAbs += pModel->nComponents;
				}
			}

			AssignLabelsToComponentsAndSurfelCells(fLabel, nLabels, componentAssociation, cellAssociation, fCellLabel);

			sumfModel = 0.0f;

			for (iModel = 0; iModel < nModels; iModel++)
			{
				AssignLabelsToComponentsAndSurfelCells(fLabelModel + nLabels * nComponents * iModel, nLabels, componentAssociationModel, cellAssociationModel, fCellLabel);

				TP = 0;

				for (iCell = 0; iCell < surfelCells.n; iCell++)
					if (cellAssociationModel[iCell].label == cellAssociation[iCell].label)
						TP += surfelCells.Element[iCell].size;

				//fModel[iModel] = exp(-gamma * (1.0f - ((float)TP / (float)supportSize)));

				if (100 * TP / supportSize < 50)
					out.fModel[iModel] = 0;

				sumfModel += out.fModel[iModel];
			}

			for (iModel = 0; iModel < nModels; iModel++)
				out.fModel[iModel] /= sumfModel;
		}

#ifdef	RVLVN_PART_SEGMENTATION_CTINET_LOG
		FILE *fpModels = fopen((std::string(pClassifier->resultsFolder) + "\\VNInstanceToClassModelLayer.dat").data(), "wb");

		fwrite(fModel, sizeof(float), nModels, fpModels);

		fclose(fpModels);

		FILE *fpLabels = fopen((std::string(pClassifier->resultsFolder) + "\\VNInstanceToClassGlobalLocalLayer.dat").data(), "wb");

		fwrite(fLabel, sizeof(float), nComponents * nLabels, fpLabels);

		fclose(fpLabels);
#endif

		// Free memory.

		delete[] fLabel;
		delete[] fLabelModel;
		delete[] fCellLabel;
		delete[] componentAssociationModel;
		delete[] cellAssociationModel;
	}
}

void VNInstance::AssignLabelsToComponentsAndSurfelCells(
	float *fLabel,
	int nLabels,
	VN_::PartAssociation *componentAssociation,
	VN_::PartAssociation *cellAssociation,
	float *fCellLabel)
{
	// Assign labels to the scene components and surfel cells.

	memset(fCellLabel, 0, nLabels * surfelCells.n * sizeof(float));

	memset(componentAssociation, 0xff, nComponents * sizeof(VN_::PartAssociation));

	int i, iCell, iSComp, label;
	float maxp;

	for (iSComp = 0; iSComp < nComponents; iSComp++)
	{
		for (i = 0; i < componentSurfelCells[iSComp].n; i++)
		{
			iCell = componentSurfelCells[iSComp].Element[i];

			for (label = 0; label < nLabels; label++)
				if (fCellLabel[iCell * nLabels + label] < fLabel[iSComp * nLabels + label])
					fCellLabel[iCell * nLabels + label] = fLabel[iSComp * nLabels + label];
		}

		maxp = 0.0f;

		for (label = 0; label < nLabels; label++)
			if (fLabel[iSComp * nLabels + label] > maxp)
			{
				maxp = fLabel[iSComp * nLabels + label];

				componentAssociation[iSComp].label = label;
			}
	}

	memset(cellAssociation, 0xff, surfelCells.n * sizeof(VN_::PartAssociation));

	for (iCell = 0; iCell < surfelCells.n; iCell++)
	{
		maxp = 0.0f;

		for (label = 0; label < nLabels; label++)
			if (fCellLabel[iCell * nLabels + label] > maxp)
			{
				maxp = fCellLabel[iCell * nLabels + label];

				cellAssociation[iCell].label = label;
			}
	}
}

//void VNInstance::Match2(VNClass VNClass,
//	void *vpClassifier) //Probabilistic approach
//{
//	//parameters:
//	float pi = 3.14;
//
//
//	VNClassifier *pVNClassifier = (VNClassifier *)vpClassifier;
//	VNInstance *VNSceneInstance = this;
//	float *ds = new float[pVNClassifier->sampledUnitSphere.h];
//	float *dm = new float[pVNClassifier->sampledUnitSphere.h];
//
//	memcpy(ds, VNSceneInstance->d, sizeof(float)*pVNClassifier->sampledUnitSphere.h);
//
//	VN_::ModelTemplate2 metamodel;
//
//	for (int iLabel = 0; iLabel < VNClass.nLabels; iLabel++)
//	{
//		int nMM = VNClass.metamodels[iLabel].size();
//		for (int iMM = 0; iMM < nMM; iMM++)
//		{
//			metamodel = VNClass.metamodels[iLabel][iMM];
//
//			int nGaussians = metamodel.gaussianMixtureModel.w * VNClass.metamodels[iLabel][iMM].gaussianMixtureModel.h;
//
//			for (int iGMM = 0; iGMM < nGaussians; iGMM++)
//			{
//				memcpy(dm, VNClass.metamodels[iLabel][iMM].gaussianMixtureModel.Element[iGMM].x, sizeof(float)*pVNClassifier->sampledUnitSphere.h);
//				int f = VNClass.metamodels[iLabel][iMM].gaussianMixtureModel.Element[iGMM].w;
//
//			}
//		}
//
//	}
//	delete[] ds;
//	delete[] dm;
//
//}

/**
if label == -1 function visualizes part association
if label != -1 function visualizes part association evaluation

if assignLabelToUnassigned == -1 function does not assign labels to unassigned points
if assignLabelToUnassigned != -1 function assigns labels to unassigned points according to the closest matched scene component
**/

void VNInstance::VisualizePartAssociation(
	Mesh *pMesh,
	void *vpClassifier,
	int iClass,
	VN_::PartAssociation *componentAssociation,
	VN_::PartAssociation *cellAssociation,
	int label,
	int assignLabelToUnassigned)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	VNInstance *pVNScene = this;

	Visualizer *pVisualizer = pClassifier->visualizationData.pVisualizer;

	int nLabels = pClassifier->classes[iClass].nLabels;

	SURFEL::Cell *pCell;
	Surfel *pSurfel;
	QLIST::Index *piSurfelIdx;
	QLIST::Index2 *piPtIdx;
	int k, label_;
	int kmaxS = pVNScene->surfelCells.n;
	int *cellS = new int[kmaxS];
	memset(cellS, 0, kmaxS * sizeof(int));

	AssignLabelToAllMeshPoints(pMesh, -1);

	for (int iCell = 0; iCell < surfelCells.n; iCell++)
	{
		label_ = cellAssociation[iCell].label;

		if (label_ == -1)
			continue;

		pCell = surfelCells.Element + iCell;

		piSurfelIdx = pCell->surfelList.pFirst;

		while (piSurfelIdx)
		{
			pSurfel = pClassifier->pSurfels->NodeArray.Element + piSurfelIdx->Idx;

			piPtIdx = pSurfel->PtList.pFirst;

			while (piPtIdx)
			{
				pMesh->NodeArray.Element[piPtIdx->Idx].label = label_;
				piPtIdx = piPtIdx->pNext;
			}

			piSurfelIdx = piSurfelIdx->pNext;
		}
	}

	if (assignLabelToUnassigned != -1)
	{
		float *ptComponentDist;

		AssignComponentsToPoints(pMesh, pClassifier->pSurfels, ptComponentDist);

		AssignLabelsToUnassignedPoints(componentAssociation, vpClassifier, pMesh, ptComponentDist);
	}

	pVisualizer->renderer->RemoveAllViewProps();

	//Load GT labels and find max label ID
	FILE *fGTlabeledPoints;

	fGTlabeledPoints = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, "_.seg"), "r");

	if (fGTlabeledPoints)
	{

		int GTLabel_;
		pClassifier->maxGTPartLabel = -1;

		int *GTlabel = new int[pMesh->NodeArray.n];

		for (int ipt = 0; ipt < pMesh->NodeArray.n; ipt++)
		{
			fscanf(fGTlabeledPoints, "%d\n", &GTLabel_);
			GTlabel[ipt] = GTLabel_;

			if (GTLabel_ > pClassifier->maxGTPartLabel)
				pClassifier->maxGTPartLabel = GTLabel_;
		}

		fclose(fGTlabeledPoints);

		uchar UnassignedColor[] = { 255, 255, 255 };

		float *P_;
		int debugCnt = 0;

		//Visualize association
		if (label == -1)
		{
			uchar colors[5][3] = { { 0, 0, 0 }, { 0, 255, 0 }, { 255, 0, 0 }, { 0, 255, 255 }, { 0, 0, 255 } };

			//RandomColors(UnassignedColor, colors, pClassifier->maxGTPartLabel);			

			Array<Point> Pts;
			Pts.Element = new Point[pMesh->NodeArray.n];

			int iLabelStart;

			if (assignLabelToUnassigned == -1)
				iLabelStart = -1;
			else
				iLabelStart = 1;

			for (int iLabel = iLabelStart; iLabel <= pClassifier->maxGTPartLabel; iLabel++)
			{
				Pts.n = 0;

				for (int i = 0; i < pMesh->NodeArray.n; i++)
				{
					if (iLabel == pMesh->NodeArray.Element[i].label)
					{
						if (iLabel == -1)
							debugCnt++;

						P_ = Pts.Element[Pts.n++].P;

						RVLCOPY3VECTOR(pMesh->NodeArray.Element[i].P, P_);
					}
				}

				if (iLabel == -1)
					pVisualizer->DisplayPointSet<float, Point>(Pts, UnassignedColor, 2.0f);
				else
					pVisualizer->DisplayPointSet<float, Point>(Pts, colors[iLabel], 2.0f);
			}

			printf("Number of unassigned points: %d\n", debugCnt);

			delete[] Pts.Element;

			int *Intersection = new int[nLabels];
			memset(Intersection, 0, nLabels * sizeof(int));
			int *Union = new int[nLabels];
			memset(Union, 0, nLabels * sizeof(int));

			for (int i = 0; i < pMesh->NodeArray.n; i++)
			{
				label_ = pMesh->NodeArray.Element[i].label;

				if (label_ >= 0)
				{
					Union[label_]++;

					if (label_ == GTlabel[i])
						Intersection[label_]++;
					else if (GTlabel[i] >= 0)
						Union[GTlabel[i]]++;
				}
				else if (GTlabel[i] >= 0)
					Union[GTlabel[i]]++;
			}

			float mIoU = 0.0f;

			int nActiveLabels = 0;

			float IoU;

			for (label_ = 0; label_ <= pClassifier->maxGTPartLabel; label_++)
				if (Union[label_] > 0)
				{
					IoU = (float)Intersection[label_] / (float)Union[label_];
					mIoU += IoU;
					nActiveLabels++;
					printf("label %d: IoU=%f\n", label_, IoU);
				}

			mIoU /= (float)nActiveLabels;

			printf("mIoU=%f\n", mIoU);

			delete[] Intersection;
			delete[] Union;
		}
		//Visualize association evaluation
		else
		{
			Array<Point> TPPts, FPPts, FNPts, TNPts, UnassignedPts;
			unsigned char TPcolor[] = { 0, 255, 0 };			//green
			unsigned char FPcolor[] = { 255, 0, 0 };			//red
			unsigned char FNcolor[] = { 255, 0, 255 };			//purple
			unsigned char TNcolor[] = { 0, 0, 255 };			//blue

			unsigned char *color;

			TPPts.Element = new Point[pMesh->NodeArray.n];
			TPPts.n = 0;

			FPPts.Element = new Point[pMesh->NodeArray.n];
			FPPts.n = 0;

			FNPts.Element = new Point[pMesh->NodeArray.n];
			FNPts.n = 0;

			TNPts.Element = new Point[pMesh->NodeArray.n];
			TNPts.n = 0;

			UnassignedPts.Element = new Point[pMesh->NodeArray.n];
			UnassignedPts.n = 0;

			for (int i = 0; i < pMesh->NodeArray.n; i++)
			{
				if (GTlabel[i] == label)
					//TP
					if (pMesh->NodeArray.Element[i].label == label)
						P_ = TPPts.Element[TPPts.n++].P;
				//FN
					else
						P_ = FNPts.Element[FNPts.n++].P;
				else
					//Unassigned
					if (pMesh->NodeArray.Element[i].label == -1)
						P_ = UnassignedPts.Element[UnassignedPts.n++].P;
					else
						//FP
						if (pMesh->NodeArray.Element[i].label == label)
							P_ = FPPts.Element[FPPts.n++].P;
				//TN
						else
							P_ = TNPts.Element[TNPts.n++].P;

				RVLCOPY3VECTOR(pMesh->NodeArray.Element[i].P, P_);
			}

			pVisualizer->DisplayPointSet<float, Point>(TPPts, TPcolor, 2.0f);

			pVisualizer->DisplayPointSet<float, Point>(FPPts, FPcolor, 2.0f);

			pVisualizer->DisplayPointSet<float, Point>(FNPts, FNcolor, 2.0f);

			pVisualizer->DisplayPointSet<float, Point>(TNPts, TNcolor, 2.0f);

			pVisualizer->DisplayPointSet<float, Point>(UnassignedPts, UnassignedColor, 2.0f);

			printf("IoU=%f\n", (float)(TPPts.n) / (float)(TPPts.n + FPPts.n + FNPts.n));

			delete[] TPPts.Element;
			delete[] FPPts.Element;
			delete[] FNPts.Element;
			delete[] TNPts.Element;
		}

		pVisualizer->Run();

		delete[] cellS;
		delete[] GTlabel;
	}
	else
		printf("There is no GT file %s\n", RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, "_.seg"));
}

void VNInstance::VisualizeGTLabels(
	Mesh *pMesh,
	void *vpClassifier)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	Visualizer *pVisualizer = pClassifier->visualizationData.pVisualizer;

	pVisualizer->renderer->RemoveAllViewProps();

	//Load GT labels and find max label ID
	FILE *fGTlabeledPoints;

	fGTlabeledPoints = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, "_.seg"), "r");

	if (fGTlabeledPoints)
	{

		int GTLabel_;
		pClassifier->maxGTPartLabel = -1;

		int *GTlabel = new int[pMesh->NodeArray.n];

		for (int ipt = 0; ipt < pMesh->NodeArray.n; ipt++)
		{
			fscanf(fGTlabeledPoints, "%d\n", &GTLabel_);
			GTlabel[ipt] = GTLabel_;

			if (GTLabel_ > pClassifier->maxGTPartLabel)
				pClassifier->maxGTPartLabel = GTLabel_;
		}

		fclose(fGTlabeledPoints);

		Array<Point> Pts;
		Pts.Element = new Point[pMesh->NodeArray.n];
		Pts.n = 0;

		float *P_;

		uchar colors[5][3] = { { 0, 0, 0 }, { 0, 255, 0 }, { 255, 0, 0 }, { 0, 255, 255 }, { 0, 0, 255 } };

		for (int iLabel = 1; iLabel <= pClassifier->maxGTPartLabel; iLabel++)
		{
			Pts.n = 0;

			for (int i = 0; i < pMesh->NodeArray.n; i++)
			{
				if (GTlabel[i] == iLabel)
				{
					P_ = Pts.Element[Pts.n++].P;
					RVLCOPY3VECTOR(pMesh->NodeArray.Element[i].P, P_);
				}
			}

			pVisualizer->DisplayPointSet<float, Point>(Pts, colors[iLabel], 2.0f);
		}

		pVisualizer->Run();

		delete[] Pts.Element;
		delete[] GTlabel;
	}
	else
		printf("There is no GT file %s\n", RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, "_.seg"));
}

void VNInstance::VisualizeSparseGTLabels(
	Mesh *pMesh,
	void *vpClassifier)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	Visualizer *pVisualizer = pClassifier->visualizationData.pVisualizer;

	pVisualizer->renderer->RemoveAllViewProps();

	Array<Point> sparsePts;
	sparsePts.Element = new Point[pMesh->NodeArray.n];
	sparsePts.n = 0;

	Array<Point> Pts;
	Pts.Element = new Point[pMesh->NodeArray.n];
	Pts.n = 0;

	float P[3];
	int nPts;

	FILE *fSparseGTlabeledPoints, *fSparsePoints;

	fSparseGTlabeledPoints = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".seg"), "r");
	fSparsePoints = fopen(RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".pts"), "r");

	char *fileName = RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".pts");

	if (fSparsePoints)
	{
		while (!feof(fSparsePoints))
		{
			fscanf(fSparsePoints, "%f %f %f\n", &sparsePts.Element[sparsePts.n].P[0], &sparsePts.Element[sparsePts.n].P[1], &sparsePts.Element[sparsePts.n].P[2]);
			sparsePts.n++;
		}

		int *GTlabel = new int[sparsePts.n];

		if (fSparseGTlabeledPoints)
		{
			for (int i = 0; i < sparsePts.n; i++)
			{
				fscanf(fSparseGTlabeledPoints, "%d\n", &GTlabel[i]);

				if (GTlabel[i] > pClassifier->maxGTPartLabel)
					pClassifier->maxGTPartLabel = GTlabel[i];
			}


			Array<Point> Pts;
			Pts.Element = new Point[sparsePts.n];
			Pts.n = 0;

			float *P_;

			uchar colors[5][3] = { { 0, 0, 0 }, { 0, 255, 0 }, { 255, 0, 0 }, { 0, 255, 255 }, { 0, 0, 255 } };

			for (int iLabel = 1; iLabel <= pClassifier->maxGTPartLabel; iLabel++)
			{
				Pts.n = 0;

				for (int i = 0; i < sparsePts.n; i++)
				{
					if (GTlabel[i] == iLabel)
					{
						P_ = Pts.Element[Pts.n++].P;
						RVLCOPY3VECTOR(sparsePts.Element[i].P, P_);
					}
				}

				pVisualizer->DisplayPointSet<float, Point>(Pts, colors[iLabel], 2.0f);
			}

			pVisualizer->Run();

			delete[] Pts.Element;

		}
		else
			printf("There is no GT file %s\n", RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".seg"));

		delete[] GTlabel;
	}
	else
		printf("There is no PTS file %s\n", RVLCreateFileName(pClassifier->sceneFileName, ".ply", -1, ".pts"));

	





	delete[] sparsePts.Element;
	
}

void VNInstance::ProbabilisticAssociationInit(
	void *vpClassifier,
	int iClass,
	RECOG::VN_::ProbabilisticAssociationWorkData *pData)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int nLabels = pClassifier->classes[iClass].nLabels;

	pData->activeMetaModels = new Array<int>[nLabels];

	pData->activeMetaModelIdxMem = new int[pClassifier->classes[iClass].metamodelID.n];

	int *pActiveMetaModelIdx = pData->activeMetaModelIdxMem;

	int label;

	for (label = 0; label < nLabels; label++)
	{
		pData->activeMetaModels[label].Element = pActiveMetaModelIdx;

		pActiveMetaModelIdx += pClassifier->classes[iClass].metamodels[label].size();
	}

	pData->bInitialized = true;
}

void VNInstance::ProbabilisticAssociationFree(RECOG::VN_::ProbabilisticAssociationWorkData *pData)
{
	if (pData->bInitialized)
	{
		delete[] pData->activeMetaModels;
		delete[] pData->activeMetaModelIdxMem;
	}
}

void VNInstance::AssociationProbabilityComputationInit(
	void *vpClassifier,
	int iClass,
	RECOG::VN_::AssociationProbabilityWorkData *pData)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int nLabels = pClassifier->classes[iClass].nLabels;

	pData->nActiveComps = new int[pClassifier->classes[iClass].maxnGaussPerComponent];

	pData->sceneSegments = new Array<int>[nLabels];
	pData->sceneSegmentIdxMem = new int[pClassifier->classes[iClass].maxnMetamodelComponents * nLabels];

	int label;

	for (label = 0; label < nLabels; label++)
		pData->sceneSegments[label].Element = pData->sceneSegmentIdxMem + label * pClassifier->classes[iClass].maxnMetamodelComponents;

	pData->metamodel = new int[nLabels];

	pData->bInitialized = true;
}

void VNInstance::AssociationProbabilityComputationFree(RECOG::VN_::AssociationProbabilityWorkData *pData)
{
	if (pData->bInitialized)
	{
		delete[] pData->nActiveComps;
		delete[] pData->sceneSegments;
		delete[] pData->sceneSegmentIdxMem;
		delete[] pData->metamodel;
	}
}

void VNInstance::LockCells(
	int iSSegment,
	bool *bCellAssigned)
{
	int iCell;

	for (int i = 0; i < componentSurfelCells[iSSegment].n; i++)
	{
		iCell = componentSurfelCells[iSSegment].Element[i];

		bCellAssigned[iCell] = true;
	}
}

void VNInstance::AddAssociation(
	void *vpClassifier,
	int iClass,
	int label,
	int iMM,
	int iMMGauss,
	Array<RECOG::VN_::PartAssociation> &association,
	bool *bCellAssigned)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	VN_::ModelTemplate2 metamodel = pClassifier->classes[iClass].metamodels[label][iMM];

	int nMMComps = metamodel.no + metamodel.nu;

	int iMMComp, iGauss, iSSegment;

	for (iMMComp = 0; iMMComp < nMMComps; iMMComp++)
	{
		iGauss = iMMGauss + iMMComp;

		iSSegment = pClassifier->metamodelGaussAssociation[iGauss];

		association.Element[association.n].iSceneSegment = iSSegment;
		association.Element[association.n].label = label;
		association.Element[association.n].iMetaModel = iMM;
		association.Element[association.n].iComponent = iMMComp;

		association.n++;

		LockCells(iSSegment, bCellAssigned);
	}
}

float VNInstance::Fit(
	void *vpClassifier,
	Array2D<float> PC_,
	float alpha,
	float sigma,
	float *&t)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	// Parameters.

	int nIterations = 0;

	// Constants.

	float var = sigma * sigma;
	int nPts = PC_.h;
	float alpha_ = alpha / (float)nComponents;
	float wt = alpha_ * var * (float)nPts;
	int n = pClassifier->sampledUnitSphere.h;

	//

	float *A = pClassifier->sampledUnitSphere.Element;

	t = new float[3 * nComponents];

	memset(t, 0, 3 * nComponents * sizeof(float));

	Pair<int, int> *PtCompAssoc = new Pair<int, int>[nPts];

	float *e = new float[nPts];

	float *QMx = new float[9 * nComponents];

	float *rMx = new float[3 * nComponents];

	int *nCompPts = new int[nComponents];

	cv::Mat Q__(3, 3, CV_32FC1);
	float *pQ = (float *)(Q__.data);
	cv::Mat r__(3, 1, CV_32FC1);
	float *pr = (float *)(r__.data);
	cv::Mat t__(3, 1, CV_32FC1);
	float *pt = (float *)(t__.data);

	int i, k, iComp, iPt, j;
	float e_, e__, e2, w, fTmp;
	float *P, *N, *d_, *t_, *Q, *r;
	float dP[3];
	Pair<int, int> PtCompAssoc_;
	float Q_[9], r_[3];
	float wtComp, ebnd, mine, mine2;

	for (k = 0; k <= nIterations; k++)
	{
		// Point-component association.

		for (iPt = 0; iPt < nPts; iPt++)
		{
			P = PC_.Element + 3 * iPt;

			PtCompAssoc_.a = -1;

			mine2 = mine = 1e+8;

			for (iComp = 0; iComp < nComponents; iComp++)
			{
				d_ = d + n * iComp;

				t_ = t + 3 * iComp;

				if (bConcave[iComp])
				{
					ebnd = mine;

					for (i = 0; i < n; i++)
					{
						N = A + 3 * i;

						RVLDIF3VECTORS(P, t, dP);

						e__ = RVLDOTPRODUCT3(N, dP) - d_[i];

						if (e__ < ebnd)
						{
							ebnd = e__;

							if (e__ < -mine)
								break;

							j = i;
						}
					}
				}
				else
				{
					ebnd = -mine;

					for (i = 0; i < n; i++)
					{
						N = A + 3 * i;

						RVLDIF3VECTORS(P, t, dP);

						e__ = RVLDOTPRODUCT3(N, dP) - d_[i];

						if (e__ > ebnd)
						{
							ebnd = e__;

							if (e__ > mine)
								break;

							j = i;
						}
					}
				}

				e2 = ebnd * ebnd;

				if (e2 < mine2)
				{
					mine2 = e2;

					mine = RVLABS(ebnd);

					e_ = ebnd;

					PtCompAssoc_.a = iComp;
					PtCompAssoc_.b = j;
				}
			}

			PtCompAssoc[iPt] = PtCompAssoc_;
			e[iPt] = e_;
		}

		if (k == nIterations)
			break;

		// LS fitting.

		memset(QMx, 0, 9 * nComponents * sizeof(float));
		memset(rMx, 0, 3 * nComponents * sizeof(float));
		memset(nCompPts, 0, nComponents * sizeof(int));

		for (iPt = 0; iPt < nPts; iPt++)
		{
			iComp = PtCompAssoc[iPt].a;
			i = PtCompAssoc[iPt].b;

			N = A + 3 * i;

			w = exp(-0.5f * e[iPt] * e[iPt] / var);

			RVLVECTCOV3(N, Q_);

			RVLSCALEMX3X3UT(Q_, w, Q_);

			Q = QMx + 9 * iComp;

			RVLSUMMX3X3UT(Q, Q_, Q);

			fTmp = w * e[iPt];

			RVLSCALE3VECTOR(N, fTmp, r_);

			r = rMx + 3 * iComp;

			RVLSUM3VECTORS(r, r_, r);

			nCompPts[iComp]++;
		}

		for (iComp = 0; iComp < nComponents; iComp++)
		{
			if (nCompPts[iComp] == 0)
				continue;

			wtComp = wt / (s[iComp] * s[iComp]);

			Q = QMx + 9 * iComp;
			Q[0] += wtComp;
			Q[4] += wtComp;
			Q[8] += wtComp;
			RVLCOPYMX3X3(Q, pQ);

			r = rMx + 3 * iComp;
			RVLCOPY3VECTOR(r, pr);

			cv::solve(Q__, r__, t__);

			t_ = t + 3 * iComp;

			RVLCOPY3VECTOR(pt, t_);
		}
	}

	float scorePC = 0.0f;

	for (iPt = 0; iPt < nPts; iPt++)
		scorePC += exp(-0.5f * e[iPt] * e[iPt] / var);

	scorePC /= (float)nPts;

	float costt = 0.0f;

	for (iComp = 0; iComp < nComponents; iComp++)
	{
		t_ = t + 3 * iComp;

		costt += (alpha_ / (s[iComp] * s[iComp]) * RVLDOTPRODUCT3(t_, t_));
	}

	float score = scorePC - costt;

	delete[] PtCompAssoc;
	delete[] e;
	delete[] QMx;
	delete[] rMx;
	delete[] nCompPts;

	return score;
}

void VNInstance::PaintComponent(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iComp,
	uchar *color)
{
	if (iComp < 0 || iComp >= nComponents)
		return;

	int i;
	SURFEL::Cell *pCell;
	QLIST::Index *pSurfelIdx;
	Surfel *pSurfel;

	for (i = 0; i < componentSurfelCells[iComp].n; i++)
	{
		pCell = surfelCells.Element + componentSurfelCells[iComp].Element[i];

		pSurfelIdx = pCell->surfelList.pFirst;

		while (pSurfelIdx)
		{
			pSurfel = pSurfels->NodeArray.Element + pSurfelIdx->Idx;

			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);

			pSurfelIdx = pSurfelIdx->pNext;
		}
	}
}

void VNInstance::Display(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	uchar *color)
{
	int iComp;

	for (iComp = 0; iComp < nComponents; iComp++)
		PaintComponent(pVisualizer, pMesh, pSurfels, iComp, color + 3 * iComp);
}

float VNInstance::Fit(
	void *vpClassifier,
	VNInstance *pModel,
	float alphat,
	float alphas,
	float sigma,
	float *&qOut)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	Array2D<float> PC_ = pModel->PC;
	int *iCTIElement = pModel->iCTIElement;

	// Parameters.

	int nIterations = 10;

	// Constants.

	float var = sigma * sigma;
	int nPts = PC_.h;
	float alphat_ = alphat / (float)nComponents;
	float alphas_ = alphas / (float)nComponents;
	float wt = alphat_ * var * (float)nPts;
	float ws = alphas_ * var * (float)nPts;
	int n = pClassifier->sampledUnitSphere.h;
	int m = pClassifier->classArray.Element[0].M.h;
	int m2 = m * m;

	float *wq = new float[m];

	RVLSET3VECTOR(wq, wt, wt, wt);

	int i;

	for (i = 3; i < m; i++)
		wq[i] = ws;

	// Visualization.

	//Display(pClassifier->visualizationData.pVisualizer, vpClassifier, q, &PC_);

	//pClassifier->visualizationData.pVisualizer->Run();

	//pClassifier->visualizationData.pVisualizer->renderer->RemoveAllViewProps();

	//

	float *A = pClassifier->sampledUnitSphere.Element;

	//float *DS = new float[nPts * n];
	float *dS = new float[nPts];

	int iComp, iPt;
	float *P, *N;

	for (iPt = 0; iPt < nPts; iPt++)
	{
		P = PC_.Element + 3 * iPt;

		N = A + 3 * iCTIElement[iPt];

		dS[iPt] = RVLDOTPRODUCT3(N, P);

		//dS = DS + iPt * n;

		//for (i = 0; i < n; i++)
		//{
		//	N = A + 3 * i;

		//	dS[i] = RVLDOTPRODUCT3(N, P);
		//}
	}

	qOut = new float[nComponents * m];

	memcpy(qOut, q, nComponents * m * sizeof(float));

	float *DM = new float[nComponents * n];	

	Pair<int, int> *PtCompAssoc = new Pair<int, int>[nPts];

	float *e = new float[nPts];

	//float *QMx = new float[m2 * nComponents];

	//float *rMx = new float[m * nComponents];

	int *nCompPts = new int[nComponents];

	float *wQ = new float[nComponents * n];
	float *wr = new float[nComponents * n];

	bool *bActive = new bool[nComponents * n];

	cv::Mat Q__(m, m, CV_32FC1);
	float *Q = (float *)(Q__.data);
	cv::Mat r__(m, 1, CV_32FC1);
	float *r = (float *)(r__.data);
	cv::Mat q__;

	// Only for debugging purpose!!!

	//float *ENew = new float[nComponents];
	//float *E = new float[nComponents];
	//float *VTmp = new float[m];
	//float *B = new float[nComponents];
	//float debug, debug2, debug3, debug4;

	//

	int j, k, iClass, i_, j_;
	float e_, e__, e2, w, fTmp, wQ_, wr_;
	float *M, *q_, *dM, *t_, *a, *MM, *pMM, *pQ, *q0;
	float dP[3];
	Pair<int, int> PtCompAssoc_;
	float Q_[9], r_[3];
	float wtComp, ebnd, mine, mine2;
	ClassData *pClass;

	for (k = 0; k <= nIterations; k++)
	{
		// Compute CTIs.

		for (iComp = 0; iComp < nComponents; iComp++)
		{
			iClass = (bConcave[iComp] ? 1 : 0);

			pClass = pClassifier->classArray.Element + iClass;

			q_ = qOut + iComp * m;

			dM = DM + iComp * n;

			M = pClass->M.Element;

			RVLMULMXTVECT(M, q_, m, n, dM, i, j, a);
		}

		// Only for debugging purpose!!!	

		//if (k > 0)
		//{
		//	memset(ENew, 0, nComponents * sizeof(float));

		//	for (iPt = 0; iPt < nPts; iPt++)
		//	{
		//		iComp = PtCompAssoc[iPt].a;
		//		i = PtCompAssoc[iPt].b;

		//		//dS = DS + iPt * n;

		//		dM = DM + n * iComp;

		//		e_ = dS[iPt] - dM[i];
		//		//e_ = dS[i] - dM[i];

		//		w = exp(-0.5f * e[iPt] * e[iPt] / var);

		//		ENew[iComp] += (w * e_ * e_);
		//	}

		//	for (iComp = 0; iComp < nComponents; iComp++)
		//	{
		//		q0 = q + iComp * m;

		//		q_ = qOut + iComp * m;

		//		for (i_ = 0; i_ < m; i_++)
		//			ENew[iComp] += (wq[i_] * (q_[i_] - q0[i_]) * (q_[i_] - q0[i_]));
		//	}

		//	int debug = 0;
		//}

		//

		// Point-component association.

		for (iPt = 0; iPt < nPts; iPt++)
		{
			P = PC_.Element + 3 * iPt;

			i = iCTIElement[iPt];

			//dS = DS + iPt * n;

			PtCompAssoc_.a = -1;

			mine2 = 1e+8;
			//mine = 1e+8;

			for (iComp = 0; iComp < nComponents; iComp++)
			{
				//iClass = (bConcave[iComp] ? 1 : 0);

				//pClass = pClassifier->classArray.Element + iClass;

				dM = DM + n * iComp;

				//if (bConcave[iComp])
				{
					//ebnd = mine;

					//for (i = 0; i < n; i++)
					{
						//N = A + 3 * i;

						e__ = dS[iPt] - dM[i];
						//e__ = dS[i] - dM[i];

						//if (e__ < ebnd)
						//{
							//ebnd = e__;

						//	if (e__ < -mine)
						//		break;

							//j = i;
						//}
					}
				}
				//else
				//{
					//ebnd = -mine;

					//for (i = 0; i < n; i++)
					//{
					//	N = A + 3 * i;

					//	e__ = dS[iPt] - dM[i];
						//e__ = dS[i] - dM[i];

						//if (e__ > ebnd)
						//{
							//ebnd = e__;

						//	if (e__ > mine)
						//		break;

							//j = i;
						//}
					//}
				//}

				e2 = e__ * e__;
				//e2 = ebnd * ebnd;

				if (e2 < mine2)
				{
					mine2 = e2;

					//mine = RVLABS(ebnd);

					e_ = e__;
					//e_ = ebnd;

					PtCompAssoc_.a = iComp;
					PtCompAssoc_.b = i;
					//PtCompAssoc_.b = j;
				}
			}

			PtCompAssoc[iPt] = PtCompAssoc_;
			e[iPt] = e_;
		}

		if (k == nIterations)
			break;

		// Only for debugging purpose!!!
	
		//memset(E, 0, nComponents * sizeof(float));
		//memset(B, 0, nComponents * sizeof(float));

		//debug = debug2 = 0.0f;

		//for (iPt = 0; iPt < nPts; iPt++)
		//{
		//	iComp = PtCompAssoc[iPt].a;
		//	i = PtCompAssoc[iPt].b;

		//	w = exp(-0.5f * e[iPt] * e[iPt] / var);

		//	E[iComp] += (w * e[iPt] * e[iPt]);

		//	//dS = DS + iPt * n;

		//	B[iComp] += (w * dS[iPt] * dS[iPt]);
		//	//B[iComp] += (w * dS[i] * dS[i]);

		//	dM = DM + n * iComp;

		//	e_ = dS[i];

		//	iClass = (bConcave[iComp] ? 1 : 0);

		//	pClass = pClassifier->classArray.Element + iClass;

		//	M = pClass->M.Element + i;

		//	q_ = qOut + iComp * m;

		//	for (j = 0; j < m; j++)
		//		e_ -= (M[n * j] * q_[j]);

		//	e_ -= e[iPt];

		//	e_ = RVLABS(e_);

		//	if (iComp == 0)
		//	{
		//		debug += (w * dM[i] * dM[i]);
		//		debug2 += (w * dM[i] * dS[i]);
		//	}
		//}

		//float ETotal = 0.0f;

		//for (iComp = 0; iComp < nComponents; iComp++)
		//{
		//	q0 = q + iComp * m;

		//	q_ = qOut + iComp * m;

		//	for (i_ = 0; i_ < m; i_++)
		//		E[iComp] += (wq[i_] * (q_[i_] - q0[i_]) * (q_[i_] - q0[i_]));

		//	ETotal += E[iComp];
		//}

		//printf("E=%f\n", ETotal);

		Display(pClassifier->visualizationData.pVisualizer, vpClassifier, qOut, true, &PC_);
		//Display(pClassifier->visualizationData.pVisualizer, vpClassifier, qOut);

		//double red[] = {1.0, 0.0, 0.0};
		//double orange[] = {1.0, 0.5, 0.0};
 
		//pModel->Display(pClassifier->visualizationData.pVisualizer, vpClassifier, NULL, true, NULL, red, orange);

		pClassifier->visualizationData.pVisualizer->Run();

		pClassifier->visualizationData.pVisualizer->renderer->RemoveAllViewProps();

		//

		// LS fitting.

		memset(nCompPts, 0, nComponents * sizeof(int));
		memset(wQ, 0, nComponents * n * sizeof(float));
		memset(wr, 0, nComponents * n * sizeof(float));
		memset(bActive, 0, nComponents * n * sizeof(bool));

		for (iPt = 0; iPt < nPts; iPt++)
		{
			iComp = PtCompAssoc[iPt].a;
			i = PtCompAssoc[iPt].b;

			w = exp(-0.5f * e[iPt] * e[iPt] / var);

			j = iComp * n + i;

			bActive[j] = true;

			wQ[j] += w;

			//dS = DS + iPt * n;

			wr[j] += (w * dS[iPt]);
			//wr[j] += (w * dS[i]);

			//N = A + 3 * i;

			//RVLVECTCOV3(N, Q_);

			//RVLSCALEMX3X3UT(Q_, w, Q_);

			//Q = QMx + 9 * iComp;

			//RVLSUMMX3X3UT(Q, Q_, Q);

			//fTmp = w * e[iPt];

			//RVLSCALE3VECTOR(N, fTmp, r_);

			//r = rMx + 3 * iComp;

			//RVLSUM3VECTORS(r, r_, r);

			nCompPts[iComp]++;
		}

		for (iComp = 0; iComp < nComponents; iComp++)
		{
			if (nCompPts[iComp] == 0)
				continue;

			q0 = q + iComp * m;

			memset(Q, 0, m2 * sizeof(float));
			memset(r, 0, m * sizeof(float));

			for (i = 0; i < n; i++)
			{
				j = iComp * n + i;

				if (!bActive[j])
					continue;

				iClass = (bConcave[iComp] ? 1 : 0);

				pClass = pClassifier->classArray.Element + iClass;

				MM = pClass->Q + i * m2;

				pMM = MM;

				wQ_ = wQ[j];

				M = pClass->M.Element + i;

				wr_ = wr[j];

				for (i_ = 0; i_ < m; i_++)
				{
					j_ = i_ * m + i_;

					pQ = Q + j_;
					pMM = MM + j_;

					for (j_ = i_; j_ < m; j_++, pQ++, pMM++)
						*pQ += (wQ_ * (*pMM));

					r[i_] += (wr_ * M[n * i_]);
				}
			}

			for (i_ = 0; i_ < m; i_++)
				for (j_ = i_ + 1; j_ < m; j_++)
					Q[j_ * m + i_] = Q[i_ * m + j_];

			q_ = qOut + iComp * m;

			// Only for debugging purpose!!!

			//FILE *fpQ = fopen((std::string(pClassifier->resultsFolder) + "\\Q.txt").data(), "w");

			//PrintMatrix<float>(fpQ, Q, m, m);

			//fclose(fpQ);

			//RVLMULMXVECT(Q, q_, m, m, VTmp, i, j, a);
			//RVLDOTPRODUCT(VTmp, q_, m, debug, i);

			//RVLDOTPRODUCT(r, q_, m, debug2, i);

			//debug3 = debug - 2.0f * debug2 + B[iComp];

			//

			for (i_ = 0; i_ < m; i_++)
			{
				Q[m * i_ + i_] += wq[i_];

				r[i_] += (wq[i_] * q0[i_]);
			}

			q__ = cv::Mat(m, 1, CV_32FC1, q_);

			cv::solve(Q__, r__, q__);

			// Only for debugging purpose!!!

			//RVLMULMXVECT(Q, q_, m, m, VTmp, i, j, a);
			//RVLDOTPRODUCT(VTmp, q_, m, debug, i);

			//RVLDOTPRODUCT(r, q_, m, debug2, i);

			//debug4 = 0.0f;

			//for (i = 0; i < m; i++)
			//	debug4 += (wq[i] * q0[i] * q0[i]);

			//debug3 = debug - 2.0f * debug2 + B[iComp] + debug4;

			//debug3 = 0;

			//
		}
	}

	float scorePC = 0.0f;

	for (iPt = 0; iPt < nPts; iPt++)
		scorePC += exp(-0.5f * e[iPt] * e[iPt] / var);

	scorePC /= (float)nPts;

	float costq = 0.0f;

	int ms = m - 3;

	float costq_;
	float *qt, *qs;

	for (iComp = 0; iComp < nComponents; iComp++)
	{
		q_ = qOut + m * iComp;

		qt = q_;

		qs = q_ + 3;

		RVLDOTPRODUCT(qs, qs, ms, costq_, i);

		costq_ = alphas_ * costq_ + alphat_ * RVLDOTPRODUCT3(qt, qt);

		costq += costq_;
	}

	float score = scorePC - costq;

	delete[] wq;
	//delete[] DS;
	delete[] dS;
	delete[] DM;
	delete[] PtCompAssoc;
	delete[] e;
	//delete[] QMx;
	//delete[] rMx;
	delete[] nCompPts;
	delete[] wQ;
	delete[] wr;
	delete[] bActive;
	// Only for debugging purpose!!!
	//delete[] E;
	//delete[] ENew;
	//delete[] VTmp;
	//delete[] B;
	//

	return score;
}

void VNInstance::CreateTRED(
	void *vpClassifier,
	float *d1,
	bool bConcave1,
	float *d2,
	bool bConcave2,
	RECOG::VN_::TRED *pTRED,
	float *r,
	float *w)
{
	// Parameters.

	float wVvar = 0.025f * 0.025f;
	float varVNoise = 0.1f * 0.1f;

	//

	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int nCTI = pClassifier->sampledUnitSphere.h;

	float *A = pClassifier->sampledUnitSphere.Element;

	int halfnCTI = nCTI / 2;

	float *d[2];

	d[0] = d1;
	d[1] = d2;

	bool bConcave_[2];

	bConcave_[0] = bConcave1;
	bConcave_[1] = bConcave2;

	int i;
	int k[2];
	float o[2];

	for (i = 0; i < 2; i++)
	{
		if (bConcave_[i])
		{
			k[i] = halfnCTI;
			o[i] = -1.0f;
		}
		else
		{
			k[i] = 0;
			o[i] = 1.0f;
		}
	}

	float minr[2];
	minr[0] = minr[1] = 0.0f;

	int iCTI, iCTIminr;
	float L, e, wSum, fTmp, varV;
	float *V, VTmp[3];
	float *A_;

	for (i = 0; i < 2; i++)
	{
		for (iCTI = 0; iCTI < nCTI; iCTI++)
		{
			L = o[i] * (d[i][(iCTI + k[i]) % nCTI] + d[i][(iCTI + (halfnCTI - k[i])) % nCTI]);

			r[iCTI] = (o[i] * d[i][(iCTI + k[i]) % nCTI] + o[1 - i] * d[1 - i][(iCTI + (halfnCTI - k[1 - i])) % nCTI]) / L;

			if (iCTI == 0 || r[iCTI] < minr[i])
			{
				minr[i] = r[iCTI];

				iCTIminr = iCTI;
			}
		}

		V = (i == 0 ? pTRED->V12 : pTRED->V21);
	
		RVLNULL3VECTOR(V);

		wSum = 0.0f;

		for (iCTI = 0; iCTI < nCTI; iCTI++)
		{
			e = minr[i] - r[iCTI];

			e = RVLABS(e);

			w[iCTI] = exp(-0.5f * e * e / wVvar);

			A_ = A + 3 * iCTI;

			RVLSCALE3VECTOR(A_, w[iCTI], VTmp);

			RVLSUM3VECTORS(V, VTmp, V);

			wSum += w[iCTI];
		}

		RVLNORM3(V, fTmp);

		varV = 0.0f;

		for (iCTI = 0; iCTI < nCTI; iCTI++)
		{
			A_ = A + 3 * iCTI;

			fTmp = acos(RVLDOTPRODUCT3(V, A_));

			varV += (w[iCTI] * fTmp * fTmp);
		}

		varV = varV / wSum + varVNoise;

		if (i == 0)
			pTRED->varV12 = varV;
		else
			pTRED->varV21 = varV;
	}

	pTRED->r12 = minr[0];
	pTRED->r21 = minr[1];
}

//#define RVLVN_TRED_GET_TYPE(pTRED, TREDTypeTol, type)\
//{\
//	if (pTRED->r12 >= 1.0 - TREDTypeTol)\
//	{\
//		if (pTRED->r12 >= pTRED->r21 + 2.0f * TREDTypeTol)\
//			type = RVLVN_TRED_TYPE_CONTAINS;\
//	}\
//	else if (pTRED->r21 >= 1.0 - TREDTypeTol)\
//	{\
//		if (pTRED->r21 >= pTRED->r12 + 2.0f * TREDTypeTol)\
//			type = (RVLVN_TRED_TYPE_CONTAINS | );\
//	}\
//	else if (pTRED->r12 <= TREDTypeTol && pTRED->r21 <= TREDTypeTol && pTRED->r12 >= -TREDTypeTol && pTRED->r21 >= -TREDTypeTol)\
//		type = RVLVN_TRED_TYPE_TOUCH;\
//	else\
//		type = 0x00;\
//}

void VNInstance::ComputeTREDMx(void *vpClassifier)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int nCTI = pClassifier->sampledUnitSphere.h;

	// Compute TRED matrix.

	TREDMx = new VN_::TRED[nComponents * nComponents];

	float *r = new float[nCTI];
	float *w = new float[nCTI];

	int iComp, iComp_;

	for (iComp = 0; iComp < nComponents; iComp++)
		for (iComp_ = 0; iComp_ < nComponents; iComp_++)
		{
			if (iComp_ != iComp)
				CreateTRED(vpClassifier, d + nCTI * iComp, bConcave[iComp], d + nCTI * iComp_, bConcave[iComp_], TREDMx + iComp * nComponents + iComp_, r, w);
		}

	delete[] r;
	delete[] w;

	/// Create TRED clusters.

	// Parameters.

	//float TREDTypeTol = 0.2f;
	//float kTREDTypeCTol = 1.5;
	float TREDcsVTol = 0.5f;

	float TREDTypeTol = 0.333f;
	float kTREDTypeCTol = 1.0;

	//

	TREDCluster = new Pair<uchar, int>[nComponents * nComponents];

	nTREDClusters = new int[nComponents];

	memset(TREDCluster, 0, nComponents * nComponents * sizeof(Pair<uchar, int>));

	int iCluster, iComp__;
	VN_::TRED *pTRED_, *pTRED__;
	uchar type_;
	Pair<uchar, int> *cluster;
	float csV, csV12, csV21, maxcsV;

	for (iComp = 0; iComp < nComponents; iComp++)
	{
		iCluster = 0;

		cluster = TREDCluster + iComp * nComponents;

		for (iComp_ = 0; iComp_ < nComponents; iComp_++)
		{
			if (iComp_ == iComp)
				continue;

			pTRED_ = TREDMx + iComp * nComponents + iComp_;

			type_ = 0x00;

			if (pTRED_->r12 >= 1.0 - TREDTypeTol)
			{
				if (pTRED_->r12 >= pTRED_->r21 + kTREDTypeCTol * TREDTypeTol)
					type_ = RVLVN_TRED_TYPE_CONTAINS;
			}
			else if (pTRED_->r21 >= 1.0 - TREDTypeTol)
			{
				if (pTRED_->r21 >= pTRED_->r12 + kTREDTypeCTol * TREDTypeTol)
					type_ = (RVLVN_TRED_TYPE_CONTAINS | RVLVN_TRED_TYPE_12);
			}
			else if (bConcave[iComp_] == bConcave[iComp])
			{
				if (pTRED_->r12 <= TREDTypeTol && pTRED_->r21 <= TREDTypeTol && pTRED_->r12 >= -TREDTypeTol && pTRED_->r21 >= -TREDTypeTol)
					type_ = RVLVN_TRED_TYPE_TOUCH;
			}

			if (bConcave[iComp_] != bConcave[iComp])
			{
				if (type_ == 0x00)
				{
					if (pTRED_->r12 >= 1.0 - TREDTypeTol && pTRED_->r21 >= 1.0 - TREDTypeTol)
						type_ |= RVLVN_TRED_TYPE_OPPOSITE_OU;
				}
				else
					type_ |= RVLVN_TRED_TYPE_OPPOSITE_OU;
			}

			cluster[iComp_].a = type_;

			if (type_ != 0x00)
			{
				cluster[iComp_].b = -1;

				maxcsV = TREDcsVTol;

				for (iComp__ = 0; iComp__ < iComp_; iComp__++)
				{
					if (iComp__ == iComp)
						continue;

					if (cluster[iComp__].a == type_)
					{
						if (type_ == RVLVN_TRED_TYPE_TOUCH)
						{
							pTRED__ = TREDMx + iComp * nComponents + iComp__;

							csV12 = RVLDOTPRODUCT3(pTRED_->V12, pTRED__->V12);
							csV21 = RVLDOTPRODUCT3(pTRED_->V21, pTRED__->V21);

							csV = RVLMAX(csV12, csV21);

							if (csV > maxcsV)
							{
								maxcsV = csV;

								cluster[iComp_].b = cluster[iComp__].b;
							}
						}
						else
						{
							cluster[iComp_].b = cluster[iComp__].b;

							break;
						}
					}
				}

				if (cluster[iComp_].b < 0)
					cluster[iComp_].b = iCluster++;
			}
		}

		nTREDClusters[iComp] = iCluster;
	}

	///
}

void VNInstance::MatchComponentNeighborhood(
	void *vpClassifier,
	int iModel_,
	RECOG::VN_::CTINetOutput out)
{
	// Parameters.

	float thVthr = 2.0f * 2.0f;
	float unmatchPenal = 0.2f;

	//

	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int nCompsTotal = pClassifier->nMCompsTotalOU[0] + pClassifier->nMCompsTotalOU[1];

	VNInstance *pModel = pClassifier->modelVNArray.Element[iModel_];

	float *matchMx = new float[nComponents * pModel->nComponents];

	bool *bMatched = new bool[nComponents + pModel->nComponents];

	bool *bMatched_ = bMatched + nComponents;

	int ifCompFirstRow = pClassifier->firstComponentAbsIdx.Element[iModel_];

	int iComp, iComp_, iTRED, iTRED_, iCluster, iCluster_, iMatch, nMatchesI, nMatchesU, iBestMatchCluster, iBestMatchCluster_, ifCompRow, ifCompRow_;
	float csV12, csV21, thV, bestMatchScore, fNeighborhood_, fNeighbor;
	float *match;
	VN_::TRED *pTRED, *pTRED_, *CompTRED, *CompTRED_;
	Pair<uchar, int> *cluster, *cluster_;
	uchar type;

	for (iComp = 0; iComp < nComponents; iComp++)
	{
		ifCompRow = ifCompFirstRow + iComp * nCompsTotal;

		cluster = TREDCluster + iComp * nComponents;

		CompTRED = TREDMx + iComp * nComponents;

		for (iComp_ = 0; iComp_ < pModel->nComponents; iComp_++)
		{
			//if (ID == 1 && iModel_ == 17 && iComp == 0 && iComp_ == 1)
			//	int debug = 0;

			memset(matchMx, 0, nTREDClusters[iComp] * pModel->nComponents * sizeof(float));

			cluster_ = pModel->TREDCluster + iComp_ * pModel->nComponents;

			CompTRED_ = pModel->TREDMx + iComp_ * pModel->nComponents;

			for (iTRED = 0; iTRED < nComponents; iTRED++)
			{
				if (cluster[iTRED].a == 0x00)
					continue;

				ifCompRow_ = ifCompFirstRow + iTRED * nCompsTotal;

				type = cluster[iTRED].a;

				pTRED = CompTRED + iTRED;

				match = matchMx + cluster[iTRED].b * pModel->nComponents;

				for (iTRED_ = 0; iTRED_ < pModel->nComponents; iTRED_++)
				{
					if (cluster_[iTRED_].a != type)
						continue;

					if (type & RVLVN_TRED_TYPE_TOUCH)
					{
						pTRED_ = CompTRED_ + iTRED_;

						csV12 = RVLDOTPRODUCT3(pTRED->V12, pTRED_->V12);

						if (csV12 <= 0.0f)
							continue;

						csV21 = RVLDOTPRODUCT3(pTRED->V21, pTRED_->V21);

						if (csV21 <= 0.0f)
							continue;

						thV = acos(csV12);

						if ((thV * thV) / (pTRED->varV12 + pTRED_->varV12) > thVthr)
							continue;

						thV = acos(csV21);

						if ((thV * thV) / (pTRED->varV21 + pTRED_->varV21) > thVthr)
							continue;
					}

					iMatch = ifCompRow_ + iTRED_;

					fNeighbor = out.fShape[iMatch] * out.fSize[iMatch];

					if (fNeighbor > match[cluster_[iTRED_].b])
						match[cluster_[iTRED_].b] = fNeighbor;
				}
			}

			fNeighborhood_ = 0.0f;

			memset(bMatched, 0, (nComponents + pModel->nTREDClusters[iComp_]) * sizeof(bool));

			if (nTREDClusters[iComp] >= pModel->nTREDClusters[iComp_])
			{
				nMatchesI = pModel->nTREDClusters[iComp_];
				nMatchesU = nTREDClusters[iComp];
			}
			else
			{
				nMatchesI = nTREDClusters[iComp];
				nMatchesU = pModel->nTREDClusters[iComp_];
			}

			for (iMatch = 0; iMatch < nMatchesI; iMatch++)
			{
				bestMatchScore = 0.0f;
				iBestMatchCluster = -1;

				for (iCluster = 0; iCluster < nTREDClusters[iComp]; iCluster++)
				{
					if (bMatched[iCluster])
						continue;

					match = matchMx + iCluster * pModel->nComponents;

					for (iCluster_ = 0; iCluster_ < pModel->nTREDClusters[iComp_]; iCluster_++)
					{
						if (bMatched_[iCluster_])
							continue;

						if (match[iCluster_] > bestMatchScore)
						{
							bestMatchScore = match[iCluster_];
							iBestMatchCluster = iCluster;
							iBestMatchCluster_ = iCluster_;
						}
					}
				}

				if (iBestMatchCluster >= 0)
				{
					bMatched[iBestMatchCluster] = true;
					bMatched_[iBestMatchCluster_] = true;

					fNeighborhood_ += bestMatchScore;
					//fNeighborhood_ += 0.5f;
					//fNeighborhood_ += (0.4f + 0.2f * bestMatchScore);
				}
				else
					break;
			}

			//fNeighborhood_ -= (nMatchesU * pClassifier->componentNeighborhoodMatchUnmatchPenal);

			out.fNeighborhood[ifCompRow + iComp_] = fNeighborhood_;
		}	// for every component of pModel
	}	// for every component of this

	delete[] matchMx;
	delete[] bMatched;
}

void VNInstance::Display(
	Visualizer *pVisualizer,
	void *vpClassifier,
	float *qIn,
	bool bPC,
	Array2D<float> *pPCIn,
	double *colorOIn,
	double *colorUIn)
{
	float *Q = (qIn ? qIn : q);

	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;

	int n = pClassifier->sampledUnitSphere.h;
	int m = pClassifier->classArray.Element[0].M.h;

	float *N = pClassifier->sampledUnitSphere.Element;

	float *d_ = new float[n];

	float null3[3];

	RVLNULL3VECTOR(null3);

	float I[9];

	RVLUNITMX3(I);

	double blue[] = { 0.0, 0.0, 1.0 };
	double cyan[] = { 0.0, 1.0, 1.0 };

	double *colorO, *colorU;

	colorO = (colorOIn ? colorOIn : blue);
	colorU = (colorUIn ? colorUIn : cyan);

	int i, j, iComp, iClass;
	float *M, *q_, *a;
	PSGM PSGM_;
	ClassData *pClass;

	for (iComp = 0; iComp < nComponents; iComp++)
	{
		iClass = (bConcave[iComp] ? 1 : 0);

		pClass = pClassifier->classArray.Element + iClass;

		M = pClass->M.Element;

		q_ = Q + iComp * m;

		RVLMULMXTVECT(M, q_, m, n, d_, i, j, a);

		PSGM_.DisplayCTI(pVisualizer, N, n, d_, I, null3, NULL, (bConcave[iComp] ? colorU : colorO), true);
	}

	if (bPC)
	{
		Array2D<float> *pPC = (pPCIn ? pPCIn : &PC);

		uchar green[] = { 0, 255, 0};

		Array<Point> ptArray;

		ptArray.Element = new Point[pPC->h];
		ptArray.n = pPC->h;

		int iPt;
		float *P, *P_;

		for (iPt = 0; iPt < pPC->h; iPt++)
		{
			P = ptArray.Element[iPt].P;
			P_ = pPC->Element + 3 * iPt;

			RVLCOPY3VECTOR(P_, P);
		}

		pVisualizer->DisplayPointSet<float, Point>(ptArray, green, 4);

		delete[] ptArray.Element;
	}

	delete[] d_;
}

void VNInstance::SparsePCDistanceToComponents(
	void *vpClassifier,
	Mesh *pMesh,
	char *modelFileName,
	float *&ptCompDist,
	bool bSaveToFile)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	Array<RECOG::PSGM_::Plane> *pConvexTemplate66 = &pClassifier->alignment.convexTemplate66;

	ptCompDist = NULL;

	int nCTI = pClassifier->sampledUnitSphere.h;

	if (PC.Element == NULL)
		LoadPC(modelFileName);

	if (PC.Element)
	{
		char *sparseToDensePtCorrespondenceFileName = RVLCreateFileName(modelFileName, ".ply", -1, ".cor");

		FILE *fpSparseToDensePtCorrespondence = fopen(sparseToDensePtCorrespondenceFileName, "r");

		delete[] sparseToDensePtCorrespondenceFileName;

		if (fpSparseToDensePtCorrespondence)
		{
			int nPoints = PC.h;

			int *iDensePt = new int[nPoints];

			int iPt;

			for (iPt = 0; iPt < nPoints; iPt++)
				fscanf(fpSparseToDensePtCorrespondence, "%d\n", iDensePt + iPt);

			fclose(fpSparseToDensePtCorrespondence);

			float *densePtCompDist;

			AssignComponentsToPoints(pMesh, pClassifier->pSurfels, densePtCompDist);

			ptCompDist = new float[nPoints * nComponents];

			float *P;
			int iCTIElement, iComp, iDensePt_;
			float distance, distance_, o;
			float *d_;

			for (int iPoint = 0; iPoint < nPoints; iPoint++)
			{
				P = PC.Element + 3 * iPoint;

				iDensePt_ = iDensePt[iPoint];

				for (iComp = 0; iComp < nComponents; iComp++)
				{
					ptCompDist[iPoint * nComponents + iComp] = densePtCompDist[iDensePt_ * nComponents + iComp];

					if (ptCompDist[iPoint * nComponents + iComp] < 0.0f)
					{
						d_ = d + nCTI * iComp;

						o = (bConcave[iComp] ? -1.0f : 1.0f);

						distance = RVLDOTPRODUCT3(pConvexTemplate66->Element[0].N, P) - d_[0];

						for (iCTIElement = 1; iCTIElement < pConvexTemplate66->n; iCTIElement++)
						{
							distance_ = RVLDOTPRODUCT3(pConvexTemplate66->Element[iCTIElement].N, P) - d_[iCTIElement];

							if (o * distance_ > o * distance)
								distance = distance_;
						}

						ptCompDist[iPoint * nComponents + iComp] = RVLABS(distance);
					}
				}
			}

			delete[] iDensePt;
			delete[] densePtCompDist;

			if (bSaveToFile)
			{
				char *ptsCompCorrespFileName = RVLCreateFileName(modelFileName, "ply", -1, "pcc");

				FILE *fpPtsCompCorresp = fopen(ptsCompCorrespFileName, "w");

				delete[] ptsCompCorrespFileName;

				for (int iPoint = 0; iPoint < nPoints; iPoint++)
				{
					for (iComp = 0; iComp < nComponents; iComp++)
						fprintf(fpPtsCompCorresp, "%f\t", ptCompDist[iPoint * nComponents + iComp]);

					fprintf(fpPtsCompCorresp, "\n");
				}

				fclose(fpPtsCompCorresp);
			}
		}
	}
}

bool VNInstance::LoadPC(char *modelFileName)
{
	char *PCFileName = RVLCreateFileName(modelFileName, "ply", -1, "pts");

	FILE *fpPC = fopen(PCFileName, "r");

	if (fpPC == NULL)
		return false;

	PC.w = 3;
	PC.h = 0;

	float fTmp;

	while (!feof(fpPC))
	{
		if(fscanf(fpPC, "%f %f %f\n", &fTmp, &fTmp, &fTmp) == 3)
			PC.h++;
	}

	fclose(fpPC);

	RVL_DELETE_ARRAY(PC.Element);

	PC.Element = new float[PC.w * PC.h];

	fpPC = fopen(PCFileName, "r");

	delete[] PCFileName;

	int iPt;
	float *P;

	for (iPt = 0; iPt < PC.h; iPt++)
	{
		P = PC.Element + 3 * iPt;

		fscanf(fpPC, "%f %f %f\n", P, P + 1, P + 2);
	}

	fclose(fpPC);

	return true;
}

#ifdef NEVER
void VNInstance::InitAssociationProbability(
	RVL::QList<VNInstance> *pModelVNList,
	void *vpClassifier)
{
	VNClassifier *pClassifier = (VNClassifier *)vpClassifier;
	VNInstance *pVNScene = this;

	int nModelTemplates = pClassifier->modelTemplates.n;
	int nSSegments = pClassifier->pObjects->objectArray.n;
	int nComponentClusters = pClassifier->componentClusters.n;
	int iComponentCluster, iSSegment;

	float sigma2_2 = (2 * pClassifier->sigmaP * pClassifier->sigmaP);

	//alocate memory for conditional probability matrix
	conditionalProbability.h = nSSegments;
	conditionalProbability.w = nComponentClusters;

	RVL_DELETE_ARRAY(conditionalProbability.Element);
	conditionalProbability.Element = new float[conditionalProbability.w * conditionalProbability.h];

	//set all conditionalProbability elements to zero
	memset(conditionalProbability.Element, 0.0f, conditionalProbability.w * conditionalProbability.h * sizeof(float));

	//alocate memory for prior probability matrix
	priorProbability.n = pVNScene->nComponents;

	RVL_DELETE_ARRAY(priorProbability.Element);
	priorProbability.Element = new float[priorProbability.n];

	//set all priorProbability elements to zero
	memset(priorProbability.Element, 0.0f, priorProbability.n * sizeof(float));

	float *dd = new float[pClassifier->sampledUnitSphere.h];

	int j;
	float e, g;
	float *dS, *dM;

	VNInstance *pVNModelInstance = pModelVNList->pFirst;

	//for all model components (segments from DB)
	while (pVNModelInstance)
	{
		for (int iMComponent = 0; iMComponent < pVNModelInstance->nComponents; iMComponent++)
		{
			for (int iSSegment = 0; iSSegment < pVNScene->nComponents; iSSegment++)
			{
				if (pVNModelInstance->bConcave[iMComponent] != pVNScene->bConcave[iSSegment])
					continue;

				dS = pVNScene->d + iSSegment;
				dM = pVNModelInstance->d + iMComponent;
				iComponentCluster = pVNModelInstance->iPrimitiveClass[iMComponent];

				RVLDIFVECTORS(dS, dM, pClassifier->sampledUnitSphere.h, dd, j);
				RVLDOTPRODUCT(dd, dd, pClassifier->sampledUnitSphere.h, e, j);

				g = exp(-(e / sigma2_2));

				conditionalProbability.Element[iSSegment * conditionalProbability.w + iComponentCluster] += g;

				priorProbability.Element[iSSegment] += g;
			}
		}

		pVNModelInstance = pVNModelInstance->pNext;
	}

	int nConvexComponentClusters = 0;
	int nConcaveComponentClusters = 0;

	RECOG::VN_::ComponentCluster *pComponentCluster;

	//calculate conditional probability
	for (iComponentCluster = 0; iComponentCluster < pClassifier->componentClusters.n; iComponentCluster++)
	{
		pComponentCluster = pClassifier->componentClusters.Element + iComponentCluster;

		//calculate nuber of concave and convex components in component clusters
		if (pComponentCluster->bConcave)
			nConcaveComponentClusters += pComponentCluster->nComponents;
		else
			nConvexComponentClusters += pComponentCluster->nComponents;

		for (iSSegment = 0; iSSegment < pVNScene->nComponents; iSSegment++)
		{
			conditionalProbability.Element[iSSegment * conditionalProbability.w + iComponentCluster] /= pComponentCluster->nComponents;
			conditionalProbability.Element[iSSegment * conditionalProbability.w + iComponentCluster] += pClassifier->priord;
		}
	}

	//calculate prior probability
	for (iSSegment = 0; iSSegment < pVNScene->nComponents; iSSegment++)
	{
		if (pVNScene->bConcave[iSSegment])
			priorProbability.Element[iSSegment] /= nConcaveComponentClusters;
		else
			priorProbability.Element[iSSegment] /= nConvexComponentClusters;

		priorProbability.Element[iSSegment] += pClassifier->priord;
	}

	delete[] dd;
}
#endif