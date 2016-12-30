//#include "stdafx.h"

#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGM.h"
#include <Eigen\Eigenvalues>
#include <random> //VIDOVIC

using namespace RVL;

PSGM::PSGM()
{
	mode = RVLRECOGNITION_MODE_RECOGNITION;
	nDominantClusters = 1;
	kNoise = 1.2f;
	minInitialSurfelSize = 20;
	minVertexPerc = 50;
	kReferenceSurfelSize = 0.2f;
	kReferenceTangentSize = 0.3f;
	baseSeparationAngle = 22.5f;
	//edgeTangentAngle = 100.0f;	
	nModels = 35; //Vidovic
	nMSegments = 3; //Vidovic
	minClusterSize = 400;
	maxClusterSize = 66122;
	minSignificantClusterSize = 3200;
	minClusterBoundaryDiscontinuityPerc = 75;
	minClusterNormalDistributionStd = 0.1f;
	groundPlaneTolerance = 0.020f;

	convexTemplate.n = 66;
	convexTemplate.Element = new RECOG::PSGM_::Plane[convexTemplate.n];

	CreateTemplate();

	//VIDOVIC
	centroidID.n = 6;
	centroidID.Element = new QLIST::Index[centroidID.n];

	ConvexTemplateCentoidID();
	//END VIDOVIC

	clusters.Element = NULL;
	clusterMap = NULL;
	clusterMem = NULL;
	clusterSurfelMem = NULL;
	clusterVertexMem = NULL;
	//modelInstanceMem = NULL;
	sceneFileName = NULL;
	modelInstanceDB.Element = NULL; //VIDOVIC
	modelDataBase = NULL; //VIDOVIC
	modelsInDataBase = NULL; //VIDOVIC
	sceneMIMatch = NULL; //VIDOVIC
	modelInstanceDB.n = 0; //VIDOVIC
	nSModelInstances = 0; //VIDOVIC

	nSamples = 20; //VIDOVIC
	stdNoise = 2; //VIDOVIC

	bNormalValidityTest = true; //VIDOVIC

	matches.Element = NULL; //VIDOVIC
	iScene = 0;

	//Vidovic
	pECCVGT = new ECCVGTLoader;

	TP = 0;
	FP = 0;
	FN = 0;

	matchMatrix.Element = NULL;
	sortedMatches.Element = NULL;
	//End Vidovic
}


PSGM::~PSGM()
{
	RVL_DELETE_ARRAY(clusters.Element);
	RVL_DELETE_ARRAY(clusterMap);
	RVL_DELETE_ARRAY(clusterMem);
	RVL_DELETE_ARRAY(clusterSurfelMem);
	RVL_DELETE_ARRAY(clusterVertexMem);
	RVL_DELETE_ARRAY(convexTemplate.Element);	
	//RVL_DELETE_ARRAY(modelInstanceMem);
	RVL_DELETE_ARRAY(sceneFileName);
	RVL_DELETE_ARRAY(modelInstanceDB.Element); //VIDOVIC
	RVL_DELETE_ARRAY(modelDataBase); //VIDOVIC
	RVL_DELETE_ARRAY(modelsInDataBase); //VIDOVIC
	RVL_DELETE_ARRAY(sceneMIMatch); //VIDOVIC
	RVL_DELETE_ARRAY(matches.Element); //VIDOVIC
	RVL_DELETE_ARRAY(centroidID.Element); //VIDOVIC

	//Vidovic
	int iSSegment;

	if (matchMatrix.Element && sortedMatches.Element)
	{
		for (iSSegment = 0; iSSegment < nDominantClusters; iSSegment++)
		{
			RVL_DELETE_ARRAY(matchMatrix.Element[iSSegment].Element);
			RVL_DELETE_ARRAY(sortedMatches.Element[iSSegment].Element);
		}
	}

	RVL_DELETE_ARRAY(matchMatrix.Element);
	RVL_DELETE_ARRAY(sortedMatches.Element)

	pECCVGT->~ECCVGTLoader();
	//End Vidovic
}

void PSGM::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("Recognition.mode", RVLPARAM_TYPE_ID, &mode);
	ParamList.AddID(pParamData, "TRAINING", RVLRECOGNITION_MODE_TRAINING);
	ParamList.AddID(pParamData, "RECOGNITION", RVLRECOGNITION_MODE_RECOGNITION); //VIDOVIC
	ParamList.AddID(pParamData, "CREATE_CTIS", RVLRECOGNITION_MODE_PSGM_CREATE_CTIS);
	pParamData = ParamList.AddParam("PSGM.nDominantClusters", RVLPARAM_TYPE_INT, &nDominantClusters);
	pParamData = ParamList.AddParam("PSGM.kNoise", RVLPARAM_TYPE_FLOAT, &kNoise);
	pParamData = ParamList.AddParam("PSGM.minInitialSurfelSize", RVLPARAM_TYPE_INT, &minInitialSurfelSize);
	pParamData = ParamList.AddParam("PSGM.minVertexPerc", RVLPARAM_TYPE_INT, &minVertexPerc);
	pParamData = ParamList.AddParam("PSGM.kReferenceSurfelSize", RVLPARAM_TYPE_FLOAT, &kReferenceSurfelSize);
	pParamData = ParamList.AddParam("PSGM.kReferenceTangentSize", RVLPARAM_TYPE_FLOAT, &kReferenceTangentSize);
	pParamData = ParamList.AddParam("PSGM.baseSeparationAngle", RVLPARAM_TYPE_FLOAT, &baseSeparationAngle);
	//pParamData = ParamList.AddParam("PSGM.edgeTangentAngle", RVLPARAM_TYPE_FLOAT, &edgeTangentAngle);
	pParamData = ParamList.AddParam("ModelDataBase", RVLPARAM_TYPE_STRING, &modelDataBase); //VIDOVIC
	pParamData = ParamList.AddParam("ModelsInDataBase", RVLPARAM_TYPE_STRING, &modelsInDataBase); //VIDOVIC
	pParamData = ParamList.AddParam("PSGM.RANSAC.nSamples", RVLPARAM_TYPE_INT, &nSamples); //VIDOVIC
	pParamData = ParamList.AddParam("PSGM.RANSAC.stdNoise", RVLPARAM_TYPE_INT, &stdNoise); //VIDOVIC
	pParamData = ParamList.AddParam("PSGM.normalValidityTest", RVLPARAM_TYPE_BOOL, &bNormalValidityTest); //VIDOVIC
	pParamData = ParamList.AddParam("PSGM.SceneMIMatch", RVLPARAM_TYPE_STRING, &sceneMIMatch); //VIDOVIC
	pParamData = ParamList.AddParam("PSGM.nModels", RVLPARAM_TYPE_INT, &nModels); //Vidovic
	pParamData = ParamList.AddParam("PSGM.nMSegments", RVLPARAM_TYPE_INT, &nMSegments); //Vidovic
	pParamData = ParamList.AddParam("PSGM.minClusterSize", RVLPARAM_TYPE_INT, &minClusterSize);
	pParamData = ParamList.AddParam("PSGM.maxClusterSize", RVLPARAM_TYPE_INT, &maxClusterSize);
	pParamData = ParamList.AddParam("PSGM.minSignificantClusterSize", RVLPARAM_TYPE_INT, &minSignificantClusterSize);
	pParamData = ParamList.AddParam("PSGM.minClusterBoundaryDiscontinuityPerc", RVLPARAM_TYPE_INT, &minClusterBoundaryDiscontinuityPerc);
	pParamData = ParamList.AddParam("PSGM.minClusterNormalDistributionStd", RVLPARAM_TYPE_FLOAT, &minClusterNormalDistributionStd);
	pParamData = ParamList.AddParam("PSGM.groundPlaneTolerance", RVLPARAM_TYPE_FLOAT, &groundPlaneTolerance);	
}

void PSGM::Interpret(
	Mesh *pMeshIn)
{
	// Create ordered mesh.

	pMesh = pMeshIn;

	pMesh->CreateOrderedMeshFromPolyData();

	// Detect surfels.

	pSurfels->Init(pMesh);

	pSurfelDetector->Init(pMesh, pSurfels, pMem);

	printf("Segmentation to surfels...");

	pSurfelDetector->Segment(pMesh, pSurfels);

	printf("completed.\n");

	int nSurfels = pSurfels->NodeArray.n;

	printf("No. of surfels = %d\n", nSurfels);

	// Detect vertices.

	printf("Detect vertices.\n");

	pSurfels->DetectVertices(pMesh);

	// Cluster surfels into convex surfaces.

	printf("Detect convex clusters.\n");

	Clusters();

	// Fit model.

	printf("Fit convex template.\n");

	int nClusters = RVLMIN(clusters.n, nDominantClusters);

	int iCluster;
	RECOG::PSGM_::Cluster *pCluster;
	RECOG::PSGM_::ModelInstance *pModelInstance;

	for (iCluster = 0; iCluster < nClusters; iCluster++)
	{
		ReferenceFrames(iCluster);

		pCluster = clusters.Element[iCluster];

		pModelInstance = pCluster->modelInstanceList.pFirst;

		while (pModelInstance)
		{
			FitModel(pCluster, pModelInstance);

			pModelInstance = pModelInstance->pNext;

			nSModelInstances++; //VIDOVIC
		}
	}

	// Save model instances to a file.

	printf("Save model instances to a file.\n");

	char *PSGModelInstanceFileName = RVLCreateString(sceneFileName);

	sprintf(PSGModelInstanceFileName + strlen(PSGModelInstanceFileName) - 3, "cti");

	FILE *fp = fopen(PSGModelInstanceFileName, "w");

	for (iCluster = 0; iCluster < nClusters; iCluster++)
		SaveModelInstances(fp, -1, iCluster);

	fclose(fp);

	delete[] PSGModelInstanceFileName;

	//VIDOVIC
	//Match scene MI to model MI
	if (mode == RVLRECOGNITION_MODE_RECOGNITION)
		Match();
	//END VIDOVIC
}

void PSGM::Clusters()
{
	RVL_DELETE_ARRAY(clusterMap);

	clusterMap = new int[pSurfels->NodeArray.n];

	memset(clusterMap, 0xff, pSurfels->NodeArray.n * sizeof(int));

	RVL_DELETE_ARRAY(clusterMem);

	clusterMem = new RECOG::PSGM_::Cluster[pSurfels->NodeArray.n];

	clusters.n = 0;

	RVL_DELETE_ARRAY(clusterSurfelMem);

	clusterSurfelMem = new int[pSurfels->NodeArray.n];

	int *piSurfel = clusterSurfelMem;

	RVL_DELETE_ARRAY(clusterVertexMem);

	clusterVertexMem = new int[pSurfels->nVertexSurfelRelations];

	int *piVertex = clusterVertexMem;

	bool *bVertexVisited = new bool[pSurfels->vertexArray.n];
	bool *bVertexInCluster = new bool[pSurfels->vertexArray.n];

	bool *bSurfelVisited = new bool[pSurfels->NodeArray.n];

	QList<QLIST::Index> candidateList;
	QList<QLIST::Index> *pCandidateList = &candidateList;

	QLIST::Index *candidateMem = new QLIST::Index[pSurfels->NodeArray.n];

	Array<int> surfelBuff1, surfelBuff2;

	surfelBuff1.Element = new int[pSurfels->NodeArray.n];

	surfelBuff1.n = 0;

	int i;
	Surfel *pSurfel;

	for (i = 0; i < pSurfels->NodeArray.n; i++)
	{
		pSurfel = pSurfels->NodeArray.Element + i;

		if (!pSurfel->bEdge)
			surfelBuff1.Element[surfelBuff1.n++] = i;
	}

	surfelBuff2.Element = new int[surfelBuff1.n];

	Array<int> *pSurfelBuff = &surfelBuff1;
	Array<int> *pSurfelBuff_ = &surfelBuff2;

	int nValidClusters = 0;

	Array<int> *pTmp;

#ifdef RVLPSGM_NORMAL_HULL
	Array<RECOG::PSGM_::NormalHullElement> NHull;

	NHull.Element = new RECOG::PSGM_::NormalHullElement[pSurfels->NodeArray.n];
#else
	float meanN[3];
	float sumN[3];
	float wN;
#endif

	RECOG::PSGM_::Cluster *pCluster;
	int iCluster;
	int maxSurfelSize;
	int iLargestSurfel;
	int iFirstNewVertex;
	QLIST::Index *pCandidateIdx, *pBestCandidateIdx;
	QLIST::Index **ppCandidateIdx, **ppBestCandidateIdx;
	float dist, minDist;
	int nSurfelVertices;
	int nSurfelVerticesInCluster;
	int *piVertex_, *piVertex__;
	int iSurfel, iSurfel_;
	Surfel *pSurfel_;
	QList<QLIST::Index> *pSurfelVertexList;
	QLIST::Index *pVertexIdx;

	for (iCluster = 0; iCluster < pSurfels->NodeArray.n; iCluster++)
	{
		// pSurfel <- the largest surfel which is not assigned to a cluster.

		maxSurfelSize = minInitialSurfelSize - 1;

		iLargestSurfel = -1;

		pSurfelBuff_->n = 0;

		for (i = 0; i < pSurfelBuff->n; i++)
		{
			iSurfel = pSurfelBuff->Element[i];

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			if (!pSurfel->bEdge)
			{
				if (clusterMap[iSurfel] < 0)
				{
					pSurfelVertexList = pSurfels->surfelVertexList.Element + iSurfel;

					if (pSurfelVertexList->pFirst)
					{
						pSurfelBuff_->Element[pSurfelBuff_->n++] = iSurfel;

						if (pSurfel->size > maxSurfelSize)
						{
							maxSurfelSize = pSurfel->size;

							iLargestSurfel = iSurfel;
						}
					}
				}
			}
		}

		pTmp = pSurfelBuff;
		pSurfelBuff = pSurfelBuff_;
		pSurfelBuff_ = pTmp;

		if (iLargestSurfel < 0)
			break;

		//if (iLargestSurfel == 25)
		//	int debug = 0;

		//if (clusters.n == 15)
		//	int debug = 0;

		// Initialize a new cluster.

		pCluster = clusterMem + iCluster;

		pCluster->iSurfelArray.Element = piSurfel;
		pCluster->iVertexArray.Element = piVertex;

		pCluster->iSurfelArray.n = 0;
		pCluster->iVertexArray.n = 0;
		pCluster->size = 0;

		clusters.n++;

		clusterMap[iLargestSurfel] = iCluster;

		memset(bVertexVisited, 0, pSurfels->vertexArray.n * sizeof(bool));
		memset(bVertexInCluster, 0, pSurfels->vertexArray.n * sizeof(bool));
		memset(bSurfelVisited, 0, pSurfels->NodeArray.n * sizeof(bool));

		RVLQLIST_INIT(pCandidateList);

		QLIST::Index *pNewCandidate = candidateMem;

#ifdef RVLPSGM_NORMAL_HULL
		NHull.n = 0;
#else
		RVLNULL3VECTOR(sumN);
		wN = 0.0f;
#endif

		RVLQLIST_ADD_ENTRY(pCandidateList, pNewCandidate);

		pNewCandidate->Idx = iLargestSurfel;

		pNewCandidate++;

		bSurfelVisited[iLargestSurfel] = true;

		// Region growing.

		while (pCandidateList->pFirst)
		{
			// iSurfel <- the best candidate for expanding cluster.

			minDist = PI;

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = *ppCandidateIdx;

			while (pCandidateIdx)
			{
				iSurfel_ = pCandidateIdx->Idx;

				pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

#ifdef RVLPSGM_NORMAL_HULL
				dist = DistanceFromNormalHull(NHull, pSurfel_->N);
#else
				float e = RVLDOTPRODUCT3(meanN, pSurfel_->N);
				dist = (wN < 1e-10 ? 0.0f : acos(e));
#endif

				if (dist < minDist)
				{
					minDist = dist;

					iSurfel = iSurfel_;

					pBestCandidateIdx = pCandidateIdx;

					ppBestCandidateIdx = ppCandidateIdx;
				}

				ppCandidateIdx = &(pCandidateIdx->pNext);

				pCandidateIdx = *ppCandidateIdx;
			}

			// Remove iSurfel from candidateList.

			RVLQLIST_REMOVE_ENTRY(pCandidateList, pBestCandidateIdx, ppBestCandidateIdx);

			//if (iSurfel == 8)
			//	int debug = 0;

			// Add vertices of iSurfel, which are inside convex (or outside concave) surface into cluster.

			iFirstNewVertex = pCluster->iVertexArray.n;

			piVertex_ = piVertex;

			pSurfelVertexList = pSurfels->surfelVertexList.Element + iSurfel;

			nSurfelVertices = nSurfelVerticesInCluster = 0;

			pVertexIdx = pSurfelVertexList->pFirst;

			while (pVertexIdx)
			{
				if (bVertexVisited[pVertexIdx->Idx])
				{
					if (bVertexInCluster[pVertexIdx->Idx])
						nSurfelVerticesInCluster++;
				}
				else
				{
					if (Inside(pVertexIdx->Idx, pCluster, iSurfel))
					{
						*(piVertex++) = pVertexIdx->Idx;

						nSurfelVerticesInCluster++;
					}

				}

				nSurfelVertices++;

				pVertexIdx = pVertexIdx->pNext;
			}

			if (nSurfelVertices == 0)
				continue;

			if (100 * nSurfelVerticesInCluster / nSurfelVertices < minVertexPerc)
			{
				piVertex = piVertex_;

				continue;
			}

			pCluster->iVertexArray.n = piVertex - pCluster->iVertexArray.Element;

			pVertexIdx = pSurfelVertexList->pFirst;

			while (pVertexIdx)
			{
				bVertexVisited[pVertexIdx->Idx] = true;

				pVertexIdx = pVertexIdx->pNext;
			}

			for (piVertex__ = piVertex_; piVertex__ < piVertex; piVertex__++)
				bVertexInCluster[*piVertex__] = true;

			// Add iSurfel to cluster.

			//if (iLargestSurfel == 25 && iSurfel == 192)
			//	int debug = 0;

			clusterMap[iSurfel] = iCluster;

			*(piSurfel++) = iSurfel;

			pCluster->iSurfelArray.n++;

			pSurfel = pSurfels->NodeArray.Element + iSurfel;

			pCluster->size += pSurfel->size;

#ifdef RVLPSGM_NORMAL_HULL
			// Update normal hull.

			UpdateNormalHull(NHull, pSurfel->N);
#else
			// Update mean normal.

			UpdateMeanNormal(sumN, wN, pSurfel->N, (float)(pSurfel->size), meanN);
#endif

			// Remove candidates which are not consistent with new vertices added to the cluster.

			ppCandidateIdx = &(candidateList.pFirst);

			pCandidateIdx = candidateList.pFirst;

			while (pCandidateIdx)
			{
				pSurfel_ = pSurfels->NodeArray.Element + pCandidateIdx->Idx;

				//if (pCandidateIdx->Idx == 8)
				//	int debug = 0;

				if (BelowPlane(pCluster, pSurfel_, iFirstNewVertex))
					ppCandidateIdx = &(pCandidateIdx->pNext);
				else
					RVLQLIST_REMOVE_ENTRY(pCandidateList, pCandidateIdx, ppCandidateIdx)

					pCandidateIdx = pCandidateIdx->pNext;
			}

			// Add new candidates in candidateList.

			SURFEL::EdgePtr *pSurfelEdgePtr = pSurfel->EdgeList.pFirst;

			while (pSurfelEdgePtr)
			{
				iSurfel_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pSurfelEdgePtr);

				//if (iSurfel_ == 8)
				//	int debug = 0;

				if (clusterMap[iSurfel_] < 0)
				{
					if (!bSurfelVisited[iSurfel_])
					{
						//if (iSurfel_ == 8)
						//	int debug = 0;

						bSurfelVisited[iSurfel_] = true;

						pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

						if (pSurfel_->size > 1)
						{
							if (BelowPlane(pCluster, pSurfel_))
							{
								RVLQLIST_ADD_ENTRY(pCandidateList, pNewCandidate);

								pNewCandidate->Idx = iSurfel_;

								pNewCandidate++;
							}
						}
					}
				}

				pSurfelEdgePtr = pSurfelEdgePtr->pNext;
			}
		}	// region growing loop

		if(pCluster->bValid = (pCluster->size >= minClusterSize))
			nValidClusters++;
	}	// for each cluster

	delete[] candidateMem;
	delete[] bVertexVisited;
	delete[] bVertexInCluster;
	delete[] bSurfelVisited;
#ifdef RVLPSGM_NORMAL_HULL
	delete[] NHull.Element;
#endif

	// Sort clusters.

	Array<SortIndex<int>> sortedClusterArray;

	sortedClusterArray.Element = new SortIndex<int>[nValidClusters];

	SortIndex<int> *pSortIndex = sortedClusterArray.Element;

	for (i = 0; i < clusters.n; i++)
	{
		pCluster = clusterMem + i;

		if (pCluster->bValid)
		{
			pSortIndex->cost = pCluster->size;
			pSortIndex->idx = i;
			pSortIndex++;
		}
	}

	sortedClusterArray.n = nValidClusters;

	BubbleSort<SortIndex<int>>(sortedClusterArray, true);

	// Detect the ground plane and filter all clusters lying on the ground plane.

	Array<int> PtArray;

	PtArray.Element = new int[pMesh->NodeArray.n];

	bool bGnd = false;

	int *piPt;
	int iiSurfel;
	QLIST::Index2 *pPtIdx;
	MESH::Distribution PtDistribution;
	float *var;
	int idx[3];
	int iTmp;
	float *NGnd;
	float dGnd;
	float eGnd;

	for (i = 0; i < nValidClusters; i++)
	{
		iCluster = sortedClusterArray.Element[i].idx;

		pCluster = clusterMem + iCluster;

		if (bGnd)
		{
			//ComputeClusterNormalDistribution(pCluster);

			//if (pCluster->normalDistributionStd1 < minClusterNormalDistributionStd && pCluster->normalDistributionStd2 < minClusterNormalDistributionStd)
			{
				//if (RVLDOTPRODUCT3(NGnd, pCluster->N) >= 0.95)
				{
					for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
					{
						iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

						pSurfel = pSurfels->NodeArray.Element + iSurfel;

						eGnd = RVLDOTPRODUCT3(NGnd, pSurfel->P) - dGnd;

						if (RVLABS(eGnd) > groundPlaneTolerance)
							break;
					}

					if (iiSurfel >= pCluster->iSurfelArray.n)
						pCluster->bValid = false;
				}
			}
		}
		else
		{
			piPt = PtArray.Element;

			for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
			{
				iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

				pSurfel = pSurfels->NodeArray.Element + iSurfel;

				pPtIdx = pSurfel->PtList.pFirst;

				while (pPtIdx)
				{
					*(piPt++) = pPtIdx->Idx;

					pPtIdx = pPtIdx->pNext;
				}
			}

			PtArray.n = piPt - PtArray.Element;

			pMesh->ComputeDistribution(PtArray, PtDistribution);

			var = PtDistribution.var;

			RVLSORT3DESCEND(var, idx, iTmp);

			if (var[idx[0]] / var[idx[1]] <= 0.0005 && var[idx[0]] / var[idx[2]] <= 0.0005)
			{
				pCluster->bValid = false;

				NGnd = PtDistribution.R + 3 * idx[0];

				if (NGnd[2] > 0.0f)
				{
					RVLNEGVECT3(NGnd, NGnd);
				}

				dGnd = RVLDOTPRODUCT3(NGnd, PtDistribution.t);

				bGnd = true;
			}
		}
	}

	delete[] PtArray.Element;

	//// Filter and sort clusters.

	//for (i = 0; i < clusters.n; i++)
	//{
	//	pCluster = clusterMem + i;

	//	if (pCluster->bValid)
	//		ComputeClusterBoundaryDiscontinuityPerc(i);
	//}

	//for (i = 0; i < clusters.n; i++)
	//{
	//	pCluster = clusterMem + i;

	//	if (pCluster->bValid)
	//	{
	//		if (pCluster->size <= maxClusterSize)
	//		{
	//			ComputeClusterNormalDistribution(pCluster);

	//			if (pCluster->size < minSignificantClusterSize)
	//			{
	//				if (pCluster->boundaryDiscontinuityPerc < minClusterBoundaryDiscontinuityPerc)
	//					if (pCluster->normalDistributionStd1 < minClusterNormalDistributionStd || pCluster->normalDistributionStd2 < minClusterNormalDistributionStd)
	//						pCluster->bValid = false;
	//			}
	//		}
	//		else
	//			pCluster->bValid = false;
	//	}
	//}

	RVL_DELETE_ARRAY(clusters.Element);

	clusters.Element = new RECOG::PSGM_::Cluster *[nValidClusters];

	int *clusterIndexMap = new int[clusters.n];

	memset(clusterIndexMap, 0xff, clusters.n * sizeof(int));

	int iCluster_ = 0;

	for (i = 0; i < sortedClusterArray.n; i++)
	{
		iCluster = sortedClusterArray.Element[i].idx;

		pCluster = clusterMem + iCluster;

		if (pCluster->bValid)
		{
			clusterIndexMap[iCluster] = iCluster_;

			clusters.Element[iCluster_] = pCluster;

			iCluster_++;
		}
	}

	clusters.n = iCluster_;

	//int maxClusterSize_ = 0;
	//int size;

	//for (i = 0; i < clusters.n; i++)
	//{
	//	size = clusterMem[i].size;

	//	if (size > maxClusterSize_)
	//		maxClusterSize_ = size;
	//}

	//int maxnBins = 100000;

	//int k = (maxClusterSize_ < maxnBins ? 1 : maxClusterSize_ / maxnBins + 1);

	//int *key = new int[clusters.n];

	//for (i = 0; i < clusters.n; i++)
	//	key[i] = clusterMem[i].size / k;

	//RVL::QuickSort(key, surfelBuff1.Element, clusters.n);

	//RVL_DELETE_ARRAY(clusters.Element);

	//clusters.Element = new RECOG::PSGM_::Cluster *[clusters.n];

	//for (i = 0; i < clusters.n; i++)
	//{
	//	iCluster = surfelBuff1.Element[clusters.n - i - 1];
	//	clusters.Element[i] = clusterMem + iCluster;
	//	surfelBuff2.Element[iCluster] = i;
	//}

	//delete[] key;

	// Update cluster map.	

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		iCluster = clusterMap[iSurfel];

		if (iCluster >= 0)
			clusterMap[iSurfel] = clusterIndexMap[iCluster];
	}

	delete[] clusterIndexMap;
	delete[] sortedClusterArray.Element;
	delete[] surfelBuff1.Element;
	delete[] surfelBuff2.Element;	
}

void PSGM::CreateTemplate()
{
	float h = 0.25f * PI;
	float q = 0.5f * h;
	float sh = sin(h);
	float ch = cos(h);
	float sq = sin(q);
	float cq = cos(q);

	float *NT = new float[3 * 13];

	float *N;

	N = NT;
	RVLSET3VECTOR(N, 0.0f, 0.0f, 1.0f);
	N = NT + 3;
	RVLSET3VECTOR(N, 0.0f, -ch, ch);
	N = NT + 2 * 3;
	RVLSET3VECTOR(N, ch, 0.0f, ch);
	N = NT + 11 * 3;
	RVLSET3VECTOR(N, 0.0f, ch, ch);
	N = NT + 12 * 3;
	RVLSET3VECTOR(N, -ch, 0.0f, ch);

	int templ[] = {
		3, 0, 1,
		4, 0, 2,
		5, 1, 2,
		6, 0, 11,
		7, 0, 12,
		8, 2, 11,
		9, 1, 12,
		10, 11, 12 };

	int i;
	float *N_, *N__;
	float fTmp;

	for (i = 0; i < 8; i++)
	{
		N = NT + 3 * templ[3 * i];
		N_ = NT + 3 * templ[3 * i + 1];
		N__ = NT + 3 * templ[3 * i + 2];
		RVLSUM3VECTORS(N_, N__, N);
		RVLNORM3(N, fTmp);
	}

	float R[] = {
		0.0f, 0.0f, -1.0f,
		1.0f, 0.0f, 0.0f,
		0.0f, -1.0f, 0.0f };

	float R_[9];

	RVLMXMUL3X3(R, R, R_);

	int j;
	int i_;
	RECOG::PSGM_::Plane *pPlane;

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 11; j++)
		{
			pPlane = convexTemplate.Element + 11 * i + j;

			N = pPlane->N;

			N_ = NT + 3 * j;

			i_ = i % 3;

			if (i_ == 0)
			{
				RVLCOPY3VECTOR(N_, N);
			}
			else if (i_ == 1)
			{
				RVLMULMX3X3VECT(R, N_, N)
			}				
			else
			{
				RVLMULMX3X3VECT(R_, N_, N)
			}
				
			if (i >= 3)
			{ 
				RVLNEGVECT3(N, N);
			}
				
			pPlane->d = 1.0f;
		}
	}

	// Only for debugging purpose!

	FILE *fp = fopen("convex_template.txt", "w");

	for (i = 0; i < convexTemplate.n; i++)
		fprintf(fp, "%f\t%f\t%f\n", convexTemplate.Element[i].N[0], convexTemplate.Element[i].N[1], convexTemplate.Element[i].N[2]);

	fclose(fp);

	//

	delete[] NT;
}

void PSGM::FitModel(
	RECOG::PSGM_::Cluster *pCluster,
	RECOG::PSGM_::ModelInstance *pModelInstance)
{
	RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RECOG::PSGM_::ModelInstanceElement, convexTemplate.n, pModelInstance->modelInstance.Element);

	pModelInstance->modelInstance.n = convexTemplate.n;

	float *R = pModelInstance->R;

	int iModelInstanceElement;
	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;
	float d;
	SURFEL::Vertex *pVertex;
	int i;
	float *N, *P;
	float N_[3];
	float dist;
	float maxdDefinedNormal;

	for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
	{
		//if (iModelInstanceElement == 33)
		//	int debug = 0;

		pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

		pModelInstanceElement->valid = false;

		N = convexTemplate.Element[iModelInstanceElement].N;

		RVLMULMX3X3VECT(R, N, N_);

		pVertex = pSurfels->vertexArray.Element[pCluster->iVertexArray.Element[0]];

		pModelInstanceElement->d = RVLDOTPRODUCT3(N_, pVertex->P);

		for (i = 0; i < pCluster->iVertexArray.n; i++)
		{
			pVertex = pSurfels->vertexArray.Element[pCluster->iVertexArray.Element[i]];

			d = RVLDOTPRODUCT3(N_, pVertex->P);

			if (d > pModelInstanceElement->d)
				pModelInstanceElement->d = d;

			//VIDOVIC
			if (bNormalValidityTest)
			{
				//if (pVertex->normalHull.n >= 3)
				//{
				//	dist = DistanceFromNormalHull(pVertex->normalHull, N_);

				//	if (dist <= 0.0f)
				//	{
				//		if (pModelInstanceElement->valid)
				//		{
				//			if (d > maxdDefinedNormal)
				//				maxdDefinedNormal = d;
				//		}
				//		else
				//		{
				//			maxdDefinedNormal = d;
				//			pModelInstanceElement->valid = true;
				//		}
				//	}
				//}
				pModelInstanceElement->valid = true;

				P = pVertex->P;

				if (RVLDOTPRODUCT3(N_, P) >= 0.0f)
					pModelInstanceElement->valid = false;
			}
			else
				pModelInstanceElement->valid = true;
			//END VIDOVIC
		}	// for every vertex in the cluster

		//VIDOVIC
		//if (bNormalValidityTest)
		//	pModelInstanceElement->e = (pModelInstanceElement->valid ? pModelInstanceElement->d - maxdDefinedNormal : 0.0f);
		//else
			pModelInstanceElement->e = 0.0f;
		//END VIDOVIC
	}	// for every model instance descriptor element

	//calculate segment centroid VIDOVIC
	int minID, maxID;

	for (i = 0; i < 3; i++)
	{
		minID = centroidID.Element[i * 2].Idx;
		maxID = centroidID.Element[i * 2 + 1].Idx;
			
		pModelInstance->tc[i] = (pModelInstance->modelInstance.Element[maxID].d - pModelInstance->modelInstance.Element[minID].d) / 2; // PROVJERITI
	}

	//END calculate segment centroid VIDOVIC
	//spremiti u tc u MODEL Instance
}

bool PSGM::ReferenceFrames(int iCluster)
{
	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	// Identify the largest surfel.

	int maxSize = 0;

	int i;
	Surfel *pSurfel;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		pSurfel = pSurfels->NodeArray.Element + pCluster->iSurfelArray.Element[i];

		if (pSurfel->size > maxSize)
			maxSize = pSurfel->size;
	}

	if (maxSize == 0)
		return false;

	if (iCluster == 3)
		int debug = 0;

	int sizeThr = (int)((float)maxSize * kReferenceSurfelSize);

	// Sort surfels in the cluster.

	Array<SortIndex<int>> iSortedSurfelArray;
	
	iSortedSurfelArray.Element = new SortIndex<int>[pCluster->iSurfelArray.n];
	iSortedSurfelArray.n = 0;

	int iSurfel;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		iSurfel = pCluster->iSurfelArray.Element[i];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->size >= sizeThr)
		{
			iSortedSurfelArray.Element[iSortedSurfelArray.n].idx = iSurfel;
			iSortedSurfelArray.Element[iSortedSurfelArray.n].cost = pSurfel->size;
			iSortedSurfelArray.n++;
		}
	}
		
	BubbleSort<SortIndex<int>>(iSortedSurfelArray, true);

	/// Determine reference frames of model instances. 

	QList<RECOG::PSGM_::ModelInstance> *pModelInstanceList = &(pCluster->modelInstanceList);

	RVLQLIST_INIT(pModelInstanceList);

	float cs = COS45;

	Array<RECOG::PSGM_::Tangent> tangentArray;

	tangentArray.Element = new RECOG::PSGM_::Tangent[pSurfels->NodeArray.n];

	//Array<RECOG::PSGM_::NormalHullElement> normalHull;

	//normalHull.Element = new RECOG::PSGM_::NormalHullElement[pCluster->iSurfelArray.n];

	RECOG::PSGM_::TangentRegionGrowingData tangentRGData;

	tangentRGData.bParent = new bool[pSurfels->NodeArray.n];
	memset(tangentRGData.bParent, 0, pSurfels->NodeArray.n * sizeof(bool));
	tangentRGData.bBase = new bool[pSurfels->NodeArray.n];
	memset(tangentRGData.bBase, 0, pSurfels->NodeArray.n * sizeof(bool));
	tangentRGData.pRecognition = this;
	tangentRGData.cs = cs;
	tangentRGData.iCluster = iCluster;
	tangentRGData.pTangentArray = &tangentArray;
	//tangentRGData.pNormalHull = &normalHull;
	float baseSeparationAngleRad = baseSeparationAngle * DEG2RAD;
	tangentRGData.baseSeparationAngle = baseSeparationAngleRad;

	int *iSurfelBuff = new int[pCluster->iSurfelArray.n];

	float kReferenceTangentSize2 = kReferenceTangentSize *  kReferenceTangentSize;

	float csSeparationAngle = cos(baseSeparationAngleRad);

	Array<SortIndex<float>> iSortedTangentArray;
	
	iSortedTangentArray.Element = new SortIndex<float>[pSurfels->NodeArray.n];

	Array<QList<QLIST::Index>> iTangentAngleArray;

	iTangentAngleArray.n = (int)round(360.0f / baseSeparationAngle);
	iTangentAngleArray.Element = new QList<QLIST::Index>[iTangentAngleArray.n];
	QLIST::Index *iTangentAngleMem = new QLIST::Index[pSurfels->NodeArray.n];

	int *piSurfelFetch, *piSurfelPut, *piSurfel, *piSurfelBuffEnd;
	RECOG::PSGM_::ModelInstance *pModelInstance;
	int iTangent;
	float maxTangentLen;
	RECOG::PSGM_::Tangent *pTangent, *pTangent_;
	float *R, *Z, *X, *t, *P1, *P2, *X_;
	float Y[3], P[3];
	Eigen::Matrix3f M;
	Eigen::Vector3f B, t_;
	float p, q, d;
	float tangentLenThr;
	int iLargestTangent;
	float *X0;
	float Y0[3];
	int iAngle;
	QList<QLIST::Index> *pAngleBinList;	
	QLIST::Index *pTangentAngleEntry;
	int j;

	for (i = 0; i < iSortedSurfelArray.n; i++)
	{
		iSurfel = iSortedSurfelArray.Element[i].idx;

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (!tangentRGData.bBase[iSurfel])
		{
			Z = pSurfel->N;

			piSurfelPut = piSurfelFetch = iSurfelBuff;

			*(piSurfelPut++) = iSurfel;

			tangentRGData.bBase[iSurfel] = true;

			tangentRGData.bParent[iSurfel] = true;

			RVLCOPY3VECTOR(pSurfel->N, tangentRGData.planeA.N);
			tangentRGData.planeA.d = pSurfel->d;
			tangentArray.n = 0;
			//normalHull.n = 0;

			piSurfelBuffEnd = RegionGrowing<SurfelGraph, Surfel, SURFEL::Edge, SURFEL::EdgePtr, RECOG::PSGM_::TangentRegionGrowingData, RECOG::PSGM_::ValidTangent>
				(pSurfels, &tangentRGData, piSurfelFetch, piSurfelPut);

			for (piSurfel = iSurfelBuff; piSurfel < piSurfelBuffEnd; piSurfel++)
				tangentRGData.bParent[*piSurfel] = false;

			//if (piSurfelBuffEnd - iSurfelBuff > pCluster->iSurfelArray.n)
			//	int debug = 0;

			maxTangentLen = 0;
			
			for (iTangent = 0; iTangent < tangentArray.n; iTangent++)
			{
				pTangent = tangentArray.Element + iTangent;

				if (pTangent->len > maxTangentLen)
				{
					maxTangentLen = pTangent->len;

					iLargestTangent = iTangent;
				}					
			}

			if (maxTangentLen > 0.0f)
			{
				X0 = tangentArray.Element[iLargestTangent].V;

				RVLCROSSPRODUCT3(Z, X0, Y0);

				tangentLenThr = kReferenceTangentSize2 * maxTangentLen;

				for (iAngle = 0; iAngle < iTangentAngleArray.n; iAngle++)
				{
					pAngleBinList = iTangentAngleArray.Element + iAngle;

					RVLQLIST_INIT(pAngleBinList);
				}

				pTangentAngleEntry = iTangentAngleMem;

				iSortedTangentArray.n = 0;

				for (iTangent = 0; iTangent < tangentArray.n; iTangent++)
				{
					pTangent = tangentArray.Element + iTangent;

					if (pTangent->len >= tangentLenThr)
					{
						iSortedTangentArray.Element[iSortedTangentArray.n].idx = iTangent;
						iSortedTangentArray.Element[iSortedTangentArray.n].cost = pTangent->len;
						iSortedTangentArray.n++;

						X = pTangent->V;

						p = RVLDOTPRODUCT3(X0, X);
						q = RVLDOTPRODUCT3(Y0, X);

						iAngle = (int)round((atan2(q, p) + PI) / baseSeparationAngleRad) % iTangentAngleArray.n;

						pAngleBinList = iTangentAngleArray.Element + iAngle;

						RVLQLIST_ADD_ENTRY(pAngleBinList, pTangentAngleEntry);

						pTangentAngleEntry->Idx = iTangent;

						pTangentAngleEntry++;
					}
				}

				BubbleSort<SortIndex<float>>(iSortedTangentArray, true);

				for (iTangent = 0; iTangent < iSortedTangentArray.n; iTangent++)
				{
					pTangent = tangentArray.Element + iSortedTangentArray.Element[iTangent].idx;

					if (!pTangent->bMerged)
					{
						RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::ModelInstance, pModelInstance);

						RVLQLIST_ADD_ENTRY(pModelInstanceList, pModelInstance);

						R = pModelInstance->R;

						RVLCOPYTOCOL3(Z, 2, R);

						X = pTangent->V;

						RVLCOPYTOCOL3(X, 0, R);

						RVLCROSSPRODUCT3(Z, X, Y);

						RVLCOPYTOCOL3(Y, 1, R);

						// Computing origin of the reference frame.

						//P1 = pSurfels->vertexArray.Element[pTangent->iVertex[0]]->P;
						//P2 = pSurfels->vertexArray.Element[pTangent->iVertex[1]]->P;

						//RVLSUM3VECTORS(P1, P2, P);

						//RVLSCALE3VECTOR(P, 0.5f, P);

						//d = RVLDOTPRODUCT3(X, P);

						//M << pSurfel->N[0], pSurfel->N[1], pSurfel->N[2], pTangent->N[0], pTangent->N[1], pTangent->N[2], X[0], X[1], X[2];

						//B << pSurfel->d, pTangent->d, d;

						//t_ = M.colPivHouseholderQr().solve(B);

						t = pModelInstance->t;

						//RVLCOPY3VECTOR(t_, t);

						RVLNULL3VECTOR(t);

						p = RVLDOTPRODUCT3(X0, X);
						q = RVLDOTPRODUCT3(Y0, X);

						iAngle = (int)round((atan2(q, p) + PI) / baseSeparationAngleRad) % iTangentAngleArray.n;

						for (j = 0; j < 2; j++)
						{
							pAngleBinList = iTangentAngleArray.Element + iAngle;

							pTangentAngleEntry = pAngleBinList->pFirst;

							while (pTangentAngleEntry)
							{
								pTangent_ = tangentArray.Element + pTangentAngleEntry->Idx;

								X_ = pTangent_->V;

								if (RVLDOTPRODUCT3(X, X_) > csSeparationAngle)
									pTangent_->bMerged = true;

								pTangentAngleEntry = pTangentAngleEntry->pNext;
							}

							iAngle = (iAngle + iTangentAngleArray.n - 1) % iTangentAngleArray.n;
						}
					}	// if (pTangent->len >= kReferenceTangentSize2 * maxTangentLen)
				}	// for every tangent
			}	// if (maxTangentLen > 0.0f)
		}	// if (pSurfel->size >= kReferenceSurfelSize * maxSize)
	}	// for every surfel in the cluster

	delete[] tangentArray.Element;
	delete[] tangentRGData.bParent;
	delete[] tangentRGData.bBase;
	delete[] iSurfelBuff;
	//delete[] normalHull.Element;
	delete[] iSortedSurfelArray.Element;
	delete[] iSortedTangentArray.Element;
	delete[] iTangentAngleMem;

	return true;
}

int RVL::RECOG::PSGM_::ValidTangent(
	int iSurfel, 
	int iSurfel_, 
	SURFEL::Edge *pEdge, 
	SurfelGraph *pSurfels, 
	RECOG::PSGM_::TangentRegionGrowingData *pData)
{	
	PSGM *pRecognition = pData->pRecognition;

	//if (pRecognition->clusterMap[iSurfel] != pData->iCluster)
	//	return -1;

	if (pData->bParent[iSurfel])
		return -1;

	Surfel *pSurfel = pSurfels->NodeArray.Element + iSurfel;
	Surfel *pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

	float *N0 = pData->planeA.N;
	float d0 = pData->planeA.d;
	float *N = pSurfel->N;
	float *N_ = pSurfel_->N;
	float cs = RVLDOTPRODUCT3(N0, N);

	if (cs <= pData->cs)
	{
		RECOG::PSGM_::Tangent *pTangent = pData->pTangentArray->Element + pData->pTangentArray->n;

		pData->pTangentArray->n++;

		pTangent->bMerged = false;

		float *NT = pTangent->N;

		// N0'*NT = cs,     NT = (s*N + (1-s)*N_) / || s*N + (1-s)*N_ ||
		// N0'*(s*N + (1-s)*N_) = cs*sqrt(s*N + (1-s)*N_)'*(s*N + (1-s)*N_)
		// s*N0'*N + N0'*N_ - s*N0'*N_ = cs * sqrt(s^2*N'*N + 2*s*(1-s)*N'*N_ + (1-s)^2*N_'*N_)
		// N0'(N-N_)*s + N0'*N_ = cs * sqrt(s^2 + 2*s*(1-s)*N'*N_ + (1-s)^2)
		// N0'(N-N_)*s + N0'*N_ = cs * sqrt(2*(1 - N'*N_)*s^2 - 2*(1 - N'*N_)*s + 1)
		// a*s + b = cs * sqrt(c*s^2 - c*s + 1),     a = N0'(N-N_), b = N0'*N_, c = 2*(1 - N'*N_)
		// a^2*s^2 + 2*a*b*s + b^2 = cs^2 * (c*s^2 - c*s + 1)
		// p*s^2 + q*s + r = 0,     p = a^2-cs^2*c, q = 2*a*b+cs^2*c, r = b^2-cs^2
		// s = (-q +- sqrt(q^2 - 4*p*r))/(2*p),    0 <= s <= 1

		float VTmp[3];
		RVLDIF3VECTORS(N, N_, VTmp);
		float a = RVLDOTPRODUCT3(N0, VTmp);
		float b = RVLDOTPRODUCT3(N0, N_);
		float c = 2.0f * (1.0f - RVLDOTPRODUCT3(N, N_));
		float cs2 = pData->cs * pData->cs;
		float p = a*a - cs2 * c;
		float q = 2.0f * a * b + cs2 * c;
		float r = b * b - cs2;
		float f = -sqrt(q * q - 4.0f * p * r);
		float s = (-q + f) / (2 * p);
		
		if (s < 0.0f || s > 1.0f)
			s = (-q - f) / (2 * p);

		RVLSCALE3VECTOR(N, s, VTmp);
		float s_ = 1.0f - s;
		RVLSCALE3VECTOR(N_, s_, NT);
		RVLSUM3VECTORS(NT, VTmp, NT);
		float fTmp;
		RVLNORM3(NT, fTmp);

		//pRecognition->UpdateNormalHull(*(pData->pNormalHull), NT);

		QList<QLIST::Index> *pVertexList = pSurfels->surfelVertexList.Element + iSurfel;

		bool bMindT = false;

		int nTangentVertices = 0;
		
		pTangent->len = 0.0f;

		float *V = pTangent->V;

		RVLCROSSPRODUCT3(N0, NT, V);

		RVLNORM3(V, fTmp);

		Eigen::Matrix3f M;

		M << N0[0], N0[1], N0[2], NT[0], NT[1], NT[2], V[0], V[1], V[2];

		Eigen::Vector3f B, P_;
		SURFEL::Vertex *pVertex;
		int i;
		float mindT, dT, d_, len13, len23;
		float P1[3], P2[3], dP13[3], dP23[3], PProj[3], dP[3];
		float *P;

		QLIST::Index *pVertexIdx = pVertexList->pFirst;

		while (pVertexIdx)
		{
			pVertex = pSurfels->vertexArray.Element[pVertexIdx->Idx];

			for (i = 0; i < pVertex->iSurfelArray.n; i++)
			{
				if (pVertex->iSurfelArray.Element[i] == iSurfel_)
				{
					P = pVertex->P;

					dT = RVLDOTPRODUCT3(NT, P);

					d_ = RVLDOTPRODUCT3(V, P);

					B << d0, dT, d_;

					P_ = M.colPivHouseholderQr().solve(B);					
					
					if (nTangentVertices == 0)
					{
						nTangentVertices = 1;

						RVLCOPY3VECTOR(P_, P1);

						pTangent->iVertex[0] = pVertexIdx->Idx;
					}
					else if (nTangentVertices == 1)
					{
						nTangentVertices = 2;

						RVLCOPY3VECTOR(P_, P2);

						RVLDIF3VECTORS(P2, P1, dP);

						pTangent->len = RVLDOTPRODUCT3(dP, dP);

						pTangent->iVertex[1] = pVertexIdx->Idx;
					}
					else	// if (nTangentVertices == 2)
					{
						RVLCOPY3VECTOR(P_, PProj);

						RVLDIF3VECTORS(PProj, P1, dP13);

						len13 = RVLDOTPRODUCT3(dP13, dP13);

						RVLDIF3VECTORS(PProj, P2, dP23);

						len23 = RVLDOTPRODUCT3(dP23, dP23);

						if (len13 > pTangent->len || len23 > pTangent->len)
						{
							if (len13 > len23)
							{
								pTangent->iVertex[1] = pVertexIdx->Idx;

								pTangent->len = len13;
							}
							else
							{
								pTangent->iVertex[0] = pVertexIdx->Idx;

								pTangent->len = len23;
							}
						}
					}	// if (nTangentVertices == 2)

					if (bMindT)
					{
						if (dT < mindT)
							mindT = dT;
					}
					else
					{
						mindT = dT;

						bMindT = true;
					}
				}	// if (pVertex->iSurfelArray.Element[i] == iSurfel_) 			
			}	// for all surfels meeting in pVertex

			pVertexIdx = pVertexIdx->pNext;
		}	// for all vertices on the boundary of iSurfel

		pTangent->d = mindT;

		return -1;
	}	// if (RVLDOTPRODUCT3(N0, N) > pData->cs && RVLDOTPRODUCT3(N0, N_) <= pData->cs)
	else if (pRecognition->clusterMap[iSurfel] == pData->iCluster)
	{
		pData->bParent[iSurfel] = true;

		//if (cs < pData->baseSeparationAngle)
			pData->bBase[iSurfel] = true;

		return 1;
	}
	else
		return -1;
}

bool PSGM::Inside(
	int iVertex, 
	RECOG::PSGM_::Cluster *pCluster,
	int iSurfel)
{
	float maxe = kNoise * 2.0f / pSurfelDetector->kPlane;

	SURFEL::Vertex *pVertex = pSurfels->vertexArray.Element[iVertex];

	int i;
	int iSurfel_;
	float e;
	Surfel *pSurfel_;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		iSurfel_ = pCluster->iSurfelArray.Element[i];

		if (iSurfel_ == iSurfel)
			continue;

		pSurfel_ = pSurfels->NodeArray.Element + iSurfel_;

		e = RVLDOTPRODUCT3(pSurfel_->N, pVertex->P) - pSurfel_->d;

		if (e > maxe)
		//if (e < -maxe)
			return false;
	}

	return true;
}

bool PSGM::BelowPlane(
	RECOG::PSGM_::Cluster *pCluster,
	Surfel *pSurfel,
	int iFirstVertex)
{
	float maxe = kNoise * 2.0f / pSurfelDetector->kPlane;

	float e;
	int iVertex_;
	SURFEL::Vertex *pVertex;

	for (iVertex_ = 0; iVertex_ < pCluster->iVertexArray.n; iVertex_++)
	{
		pVertex = pSurfels->vertexArray.Element[pCluster->iVertexArray.Element[iVertex_]];

		e = RVLDOTPRODUCT3(pSurfel->N, pVertex->P) - pSurfel->d;

		if (e > maxe)
		//if (e < -maxe)
			return false;
	}

	return true;
}

float PSGM::DistanceFromNormalHull(
	Array<SURFEL::NormalHullElement> &NHull,
	float *N)
{
	if (NHull.n == 0)
		return 0.0f;
	if (NHull.n == 1)
	{
		float *N_ = NHull.Element[0].N;

		float e = RVLDOTPRODUCT3(N_, N);

		return (e < 0.0f ? 1.0f : sqrt(1.0f - e * e));
	}		

	float maxDist = 0.0f;

	int i;
	float dist;
	float *Nh_;

	for (i = 0; i < NHull.n; i++)
	{
		Nh_ = NHull.Element[i].Nh;

		dist = RVLDOTPRODUCT3(Nh_, N);

		if (dist > maxDist)
			maxDist = dist;
	}

	return maxDist;
}

void PSGM::UpdateMeanNormal(
	float *sumN,
	float &wN,
	float *N,
	float w,
	float *meanN)
{
	float VTmp[3];
	RVLSCALE3VECTOR(N, w, VTmp);
	RVLSUM3VECTORS(sumN, VTmp, sumN);
	wN += w;
	RVLSCALE3VECTOR2(sumN, wN, meanN);
	float fTmp = sqrt(RVLDOTPRODUCT3(meanN, meanN));
	if (fTmp > 1e-10)
	{
		RVLSCALE3VECTOR2(meanN, fTmp, meanN);
	}
	else
		RVLSET3VECTOR(meanN, 0.0f, 0.0f, 1.0f);
}

void PSGM::SetSceneFileName(char *sceneFileName_)
{
	RVLCopyString(sceneFileName_, &sceneFileName);
}

void PSGM::SaveModelInstances(
	FILE *fp,
	int iModel,
	int iCluster)
{
	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	int i;
	int iModelInstanceElement;
	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;	

	RECOG::PSGM_::ModelInstance *pModelInstance = pCluster->modelInstanceList.pFirst;

	while (pModelInstance)
	{
		fprintf(fp, "%d\t%d\t", iModel, iCluster);

		for (i = 0; i < 9; i++)
			fprintf(fp, "%f\t", pModelInstance->R[i]);

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t", pModelInstance->t[i]);

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%f\t", pModelInstanceElement->d);
		}

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%d\t", (int)(pModelInstanceElement->valid));
		}

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%f\t", pModelInstanceElement->e);
		}

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t", pModelInstance->tc[i]);

		fprintf(fp, "\n");

		pModelInstance = pModelInstance->pNext;
	}
}

//VIDOVIC
bool PSGM::ModelExistInDB(char *modelFileName, FileSequenceLoader dbLoader)
{
	char *dbFileName = new char[50];

	while (dbLoader.GetNextName(dbFileName))
		if (!strcmp(modelFileName, dbFileName))
			return 1;

	return 0;
}



void PSGM::SaveModelID(FileSequenceLoader dbLoader)
{
	FILE *fp = fopen(modelsInDataBase, "w");

	char modelName[50], modelPath[200];
	int modelID;

	while (dbLoader.GetNext(modelPath, modelName, &modelID))
		fprintf(fp, "%d\t%s\n", modelID, modelName);

	fprintf(fp, "\nend");

	fclose(fp);
}

void PSGM::Learn(
	char *modelSequenceFileName,
	Visualizer *visualizer)
{
	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	FileSequenceLoader modelsLoader;
	FileSequenceLoader dbLoader;

	char *modelFilePath = new char[200];
	char *modelFileName = new char[200];

	Mesh mesh;

	int iCluster, nClusters, currentModelID;

	//RVL_DELETE_ARRAY(modelDataBase);
	//RVL_DELETE_ARRAY(modelsInDataBase);

	if (!modelDataBase)
		modelDataBase = "modelDB.dat";

	if (!modelsInDataBase)
		modelsInDataBase = "DBModels.txt";

	modelsLoader.Init(modelSequenceFileName);
	dbLoader.Init(modelsInDataBase);

	FILE *fp = fopen(modelDataBase, "a");

	bool saveDBSequenceFile = false;

	printf("Model DB creation started...\n");

	while (modelsLoader.GetNext(modelFilePath, modelFileName))
	{
		if (ModelExistInDB(modelFileName, dbLoader))
			continue;

		printf("\nProcessing model %s!\n", modelFileName);

		saveDBSequenceFile = true;

		mesh.LoadPolyDataFromPLY(modelFilePath);

		SetSceneFileName(modelFilePath);

		Interpret(&mesh);

		nClusters = RVLMIN(clusters.n, nDominantClusters);

		currentModelID = dbLoader.GetLastModelID() + 1;

		for (iCluster = 0; iCluster < nClusters; iCluster++)
			SaveModelInstances(fp, currentModelID, iCluster);

		dbLoader.AddModel(currentModelID, modelFilePath, modelFileName);

		if (visualizer)
		{
			InitDisplay(visualizer, &mesh, SelectionColor);
			Display();
			visualizer->Run();

			visualizer->renderer->RemoveAllViewProps();
		}
	}

	printf("Model DB creation completed!\n");

	if (saveDBSequenceFile)
		SaveModelID(dbLoader);

	fclose(fp);
}


void PSGM::LoadModelDataBase()
{
	FILE *fp = fopen(modelDataBase, "r");

	char line[3000] = {0};

	int iModelInstance, iModelInstanceElement, i;

	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;
	RECOG::PSGM_::ModelInstance *pModelInstance;

	if (fp)
	{
		//count number of lines in model DB
		while (!feof(fp))
		{
			line[0] = '\0';

			fgets(line, 3000, fp);

			if (line[0] == '\0' || line[0] == '\n')
				continue;

			modelInstanceDB.n++;
		}

		rewind(fp);

		modelInstanceDB.Element = new RECOG::PSGM_::ModelInstance[modelInstanceDB.n];

		pModelInstance = modelInstanceDB.Element;

		for (iModelInstance = 0; iModelInstance < modelInstanceDB.n; iModelInstance++)
		{
			pModelInstance->modelInstance.Element = new RECOG::PSGM_::ModelInstanceElement[convexTemplate.n];

			pModelInstance->modelInstance.n = convexTemplate.n;

			fscanf(fp, "%d\t%d\t", &pModelInstance->iModel, &pModelInstance->iCluster);

			for (i = 0; i < 9; i++)
				fscanf(fp, "%f\t", &pModelInstance->R[i]);

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f\t", &pModelInstance->t[i]);

			for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
			{
				pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%f\t", &pModelInstanceElement->d);
			}

			for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
			{
				pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%d\t", &pModelInstanceElement->valid);
			}

			for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
			{
				pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

				fscanf(fp, "%f\t", &pModelInstanceElement->e);
			}

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f\t", &pModelInstance->tc[i]);

			if (iModelInstance == modelInstanceDB.n - 1)
				pModelInstance->pNext = NULL;
			else
			{
				pModelInstance->pNext = pModelInstance + 1;
				pModelInstance++;
			}
		}

		fclose(fp);
	}
}

void PSGM::Match()
{
	printf("Scene to model match started...");

	ClearMatchMatrix(); //Vidovic

	float csMinSampleAngleDiff = cos(PI / 4);
	float minE, minETotal, E, score, SMI_minETotal;

	float cos45 = cos(PI / 4);

	float sigma = 4;
	float sigma25 = 2.5*2.5;

	float tBestMatch[3], SMI_tBestMatch[3];
	int bestMatchMIMID, bestMatchModelID, bestSRF, iSRF;
	int SMI_bestMatchMIMID, SMI_bestMatchModelID, SMI_bestSRF, SMI_iSRF;

	QList<RECOG::PSGM_::MatchInstance> *pSMIMatches = &SMImatches;

	//if (iScene == 0)
	RVLQLIST_INIT(pSMIMatches);

	RECOG::PSGM_::MatchInstance *pSMIMatch;

	float R_[9], t_[3];

	Eigen::Matrix3f A;
	Eigen::Vector3f B, t;

	int i, idx;

	float dISv, dIMvt;

	FILE *fp;

	//if (!sceneMIMatch)
	//	sceneMIMatch = "sceneMatch.txt";

	//fp = fopen(sceneMIMatch, "w");

	//find sample candidates
	QList<QLIST::Index> iSampleCandidateList;
	QList<QLIST::Index> *pISampleCandidateList = &iSampleCandidateList;

	RVLQLIST_INIT(pISampleCandidateList);

	QLIST::Index *pNewISampleCandidate;
	RECOG::PSGM_::Plane *pPlane = convexTemplate.Element;

	for (int idx = 0; idx < convexTemplate.n; idx++)
	{
		if (RVLABS(pPlane->N[2]) <= csMinSampleAngleDiff)
		{
			RVLMEM_ALLOC_STRUCT(pMem, QLIST::Index, pNewISampleCandidate);

			RVLQLIST_ADD_ENTRY(pISampleCandidateList, pNewISampleCandidate);

			pNewISampleCandidate->Idx = idx;
		}		

		pPlane++;
	}

	int iMIS, iMIM;
	int iSCluster, iSClusterMI;

	int MatchID = 0;

	int nClusters = RVLMIN(clusters.n, nDominantClusters);

	RECOG::PSGM_::ModelInstance *pSModelInstance;
	RECOG::PSGM_::ModelInstance *pMModelInstance;

	RECOG::PSGM_::ModelInstance *pSBestModelInstance;
	RECOG::PSGM_::ModelInstance *pMBestModelInstance;

	RECOG::PSGM_::ModelInstance *SMI_pSBestModelInstance;
	RECOG::PSGM_::ModelInstance *SMI_pMBestModelInstance;

	bool breakPrint;
	bool cluserMatch;

	//Arrays allocation
	Array<QLIST::Index> iValidSampleCandidate;
	iValidSampleCandidate.Element = new QLIST::Index[convexTemplate.n];

	Array<QLIST::Index> iValid;
	iValid.Element = new QLIST::Index[convexTemplate.n];

	Array<QLIST::Index> iRansacCandidates;
	iRansacCandidates.Element = new QLIST::Index[26]; //max 26 planes which satisfy condition

	Array<QLIST::Index> iConsensus;
	iConsensus.Element = new QLIST::Index[convexTemplate.n];

	Array<QLIST::Index> iConsensusTemp;
	iConsensusTemp.Element = new QLIST::Index[convexTemplate.n];

	iMIS = 0;

	bool segmentTP, TP_;

	float distanceThresh = 50;
	
	char *fileName = strrchr(sceneFileName, '\\') + 1;
	
	if (strcmp(fileName, "frame_20111220T114628.408278.ply") == 0)
		distanceThresh = 45;
	else if (strcmp(fileName, "frame_20111220T115430.348560.ply") == 0)
		distanceThresh = 20;
	else if (strcmp(fileName, "frame_20111220T115445.303284.ply") == 0)
		distanceThresh = 15;
	else if (strcmp(fileName, "frame_20111221T142636.413299.ply") == 0)
		distanceThresh = 25;

	for (iSCluster = 0; iSCluster < nClusters; iSCluster++)
	{
		segmentTP = false;

		breakPrint = false; //for DEBUG!

		cluserMatch = false;

		printf("%d/%d", iSCluster + 1, nClusters);

		pSModelInstance = clusters.Element[iSCluster]->modelInstanceList.pFirst;

		iSRF = -1;

		while (pSModelInstance)
		{
			SMI_minETotal = 66.0;

			iSRF++;

			pMModelInstance = modelInstanceDB.Element;

			for (iMIM = 0; iMIM < modelInstanceDB.n; iMIM++)
			{
				//minE = 66.0;
				minE = 412.5; // for sigma = 2.5^2

				//CTDMatchRANSAC
				float pPrior = 2.5 * stdNoise;

				int nValidSampleCandidates = 0;

				//find valid sample candidates
				iValidSampleCandidate.n = 0;

				QLIST::Index *piValidSampleCandidate = iValidSampleCandidate.Element;

				QLIST::Index *pISampleCandidate = pISampleCandidateList->pFirst;

				while (pISampleCandidate)
				{
					if (pSModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true && pMModelInstance->modelInstance.Element[pISampleCandidate->Idx].valid == true)
					{
						piValidSampleCandidate->Idx = pISampleCandidate->Idx;

						piValidSampleCandidate->pNext = piValidSampleCandidate + 1;

						piValidSampleCandidate++;

						nValidSampleCandidates++;
					}

					pISampleCandidate = pISampleCandidate->pNext;
				}

				iValidSampleCandidate.n = nValidSampleCandidates;

				if (nValidSampleCandidates > 2)
				{
					int nValids = 0;

					//find iValid
					iValid.n = 0;

					QLIST::Index *piValid = iValid.Element;
					int iPlane;

					for (iPlane = 0; iPlane < convexTemplate.n; iPlane++)
					{
						if (pSModelInstance->modelInstance.Element[iPlane].valid == true && pMModelInstance->modelInstance.Element[iPlane].valid == true)
						{
							piValid->Idx = iPlane;

							piValid->pNext = piValid + 1;

							piValid++;

							nValids++;
						}
					}

					iValid.n = nValids;

					int iRansac;
					bool bValidSample;

					int iSample[2];
					int ID[2];
					float V[3];
					float N_[3] = { 0, 0, 1 };
					float N__[3] = { 0, -cos45, cos45 };
					float fTmp;
					int nValidSamplesSearch;
					int nRansacCandidates = 0;

					iConsensus.n = 0;

					QLIST::Index *piConsensus = iConsensus.Element;

					iRansacCandidates.n = 0;

					QLIST::Index *piRansacCandidates = iRansacCandidates.Element;

					//find RANSAC candidates
					RVLCROSSPRODUCT3(N_, N__, V);

					fTmp = sqrt(RVLDOTPRODUCT3(V, V));

					RVLSCALE3VECTOR2(V, fTmp, V);
					
					for (i = 0; i < iValidSampleCandidate.n; i++)
					{
						ID[0] = iValidSampleCandidate.Element[i].Idx;

						fTmp = RVLDOTPRODUCT3(V, convexTemplate.Element[ID[0]].N);

						if (RVLABS(fTmp) >= csMinSampleAngleDiff)
						{
							piRansacCandidates->Idx = ID[0];

							piRansacCandidates++;

							nRansacCandidates++;
						}
					}

					iRansacCandidates.n = nRansacCandidates;

					std::random_device rd;
					std::mt19937 eng(rd());
					std::uniform_int_distribution<> distribution(0, nRansacCandidates);

					nSamples = RVLMIN(13, nRansacCandidates);

					for (iRansac = 0; iRansac < nSamples; iRansac++)
					{
#ifdef NEVER
						bValidSample = false;

						nValidSamplesSearch = 0;

						while (!bValidSample)
						{
							nValidSamplesSearch++;

							iSample[0] = distribution(eng);
							iSample[1] = distribution(eng);

							ID[0] = iValidSampleCandidate.Element[iSample[0]].Idx;
							ID[1] = iValidSampleCandidate.Element[iSample[1]].Idx;

							RVLCROSSPRODUCT3(N_, convexTemplate.Element[ID[0]].N, V);

							fTmp = sqrt(RVLDOTPRODUCT3(V, V));

							//if (RVLABS(fTmp) < 1e-10)
							//return;

							RVLSCALE3VECTOR2(V, fTmp, V);

							if (RVLDOTPRODUCT3(V, convexTemplate.Element[ID[1]].N) >= csMinSampleAngleDiff)
								bValidSample = true;

							if (nValidSamplesSearch > 2000)
								break;
						}

						if (nValidSamplesSearch > 2000)
							break;
#endif

						ID[0] = 1; //second plane from convexTemplate

						iSample[1] = distribution(eng);

						ID[1] = iValidSampleCandidate.Element[iSample[1]].Idx;

						float dM[3];
						float dS[3];
						float N[9];

						dM[0] = pMModelInstance->modelInstance.Element[0].d;
						dM[1] = pMModelInstance->modelInstance.Element[ID[0]].d;
						dM[2] = pMModelInstance->modelInstance.Element[ID[1]].d;

						dS[0] = pSModelInstance->modelInstance.Element[0].d * 1000;
						dS[1] = pSModelInstance->modelInstance.Element[ID[0]].d * 1000;
						dS[2] = pSModelInstance->modelInstance.Element[ID[1]].d * 1000;

						RVLCOPYTOCOL3(convexTemplate.Element[0].N, 0, N);
						RVLCOPYTOCOL3(convexTemplate.Element[ID[0]].N, 1, N);
						RVLCOPYTOCOL3(convexTemplate.Element[ID[1]].N, 2, N);

						A << N[0], N[3], N[6], N[1], N[4], N[7], N[2], N[5], N[8]; // N'
						B << dS[0] - dM[0], dS[1] - dM[1], dS[2] - dM[2];
						t = A.colPivHouseholderQr().solve(B);

						iConsensusTemp.n = 0;

						QLIST::Index *piConsensusTemp = iConsensusTemp.Element;

						E = 0;

						for (i = 0; i < iValid.n; i++)
						{
							idx = iValid.Element[i].Idx;

							dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

							dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

							//fTmp = (dISv - dIMvt) / pPrior;
							fTmp = (dISv - dIMvt) / sigma;

							//if (fTmp*fTmp < 1)
							#ifdef RVLPSGM_MATCH_SATURATION
								if (fTmp*fTmp < sigma25)
								//if (fTmp*fTmp < 1)
								{
									E += fTmp*fTmp;

									piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
									piConsensusTemp->pNext = piConsensusTemp + 1;

									iConsensusTemp.n++;

									piConsensusTemp++;
								}
								else
									E += sigma25;
									//E += 1;
							#else
								E += fTmp*fTmp;

								//TREBA LI OVO?
								piConsensusTemp->Idx = idx;	//	!!! saved id in original MI array
								piConsensusTemp->pNext = piConsensusTemp + 1;

								iConsensusTemp.n++;

								piConsensusTemp++;
								
							#endif
						}

						if (E < minE)
						{
							minE = E;

							iConsensus.n = iConsensusTemp.n;

							piConsensusTemp = iConsensusTemp.Element;

							piConsensus = iConsensus.Element;

							for (i = 0; i < iConsensusTemp.n; i++)
							{
								piConsensus->Idx = piConsensusTemp->Idx;
								piConsensus->pNext = piConsensus + 1;

								piConsensus++;
								piConsensusTemp++;
							}
						}
					}

					if (iConsensus.n >= 3)
					{
						float dISc, dIMc, *nTc, *dISMc;

						nTc = new float[3 * iConsensus.n];
						dISMc = new float[iConsensus.n];

						piConsensus = iConsensus.Element;

						for (i = 0; i < iConsensus.n; i++)
						{
							idx = piConsensus->Idx;

							dISc = pSModelInstance->modelInstance.Element[idx].d * 1000;
							dIMc = pMModelInstance->modelInstance.Element[idx].d;

							dISMc[i] = dISc - dIMc;

							nTc[i] = convexTemplate.Element[idx].N[0];
							nTc[iConsensus.n + i] = convexTemplate.Element[idx].N[1];
							nTc[2 * iConsensus.n + i] = convexTemplate.Element[idx].N[2];

							piConsensus++;
						}

						int j, k;

						//nTc*nTc'
						for (i = 0; i < 3; i++)
							for (j = 0; j < 3; j++)
								if (i <= j)
								{
									A(i * 3 + j) = 0;

									for (k = 0; k < iConsensus.n; k++)
										A(i * 3 + j) += nTc[i * iConsensus.n + k] * nTc[j * iConsensus.n + k];
								}
								else
									A(i * 3 + j) = A(j * 3 + i);

						//nTc*(dISc-dIMc)
						for (i = 0; i < 3; i++)
						{
							B(i) = 0;
							for (j = 0; j < iConsensus.n; j++)
								B(i) += nTc[i * iConsensus.n + j] * dISMc[j];
						}

						t = A.colPivHouseholderQr().solve(B);

						delete[] nTc;
						delete[] dISMc;

						E = 0;

						for (i = 0; i < iValid.n; i++)
						{
							idx = iValid.Element[i].Idx;

							dISv = pSModelInstance->modelInstance.Element[idx].d * 1000;

							dIMvt = pMModelInstance->modelInstance.Element[idx].d + RVLDOTPRODUCT3(t, convexTemplate.Element[idx].N);

							//fTmp = (dISv - dIMvt) / pPrior;
							fTmp = (dISv - dIMvt) / sigma;

							#ifdef RVLPSGM_MATCH_SATURATION
								//if (fTmp*fTmp < 1)
								if (fTmp*fTmp < sigma25)
									E += fTmp*fTmp;
								else
									//E += 1;
									E += sigma25;
							#else
								E += fTmp*fTmp;
							#endif
						}

						//score = E + 66 - iValid.n;
						score = E + sigma25*(66 - iValid.n);

						//save all SMI matches						
						for (i = 0; i < 3; i++)
							SMI_tBestMatch[i] = t(i);

						MSTransformation(pMModelInstance, pSModelInstance, SMI_tBestMatch, R_, t_);

						RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pSMIMatch);

						RVLQLIST_ADD_ENTRY(pSMIMatches, pSMIMatch);

						FillMatch(pSMIMatch, MatchID, iScene, iSCluster, iSRF, iMIS, pMModelInstance->iModel, pMModelInstance->iCluster, iMIM, R_, t_, SMI_tBestMatch, E, score, 0.0, 0.0, NAN, NAN, iValid.n);

						MatchID++;							

						//check MatchMatrix update
						if (matchMatrix.Element[pSMIMatch->iCluster].Element[pSMIMatch->iModel*nMSegments + pSMIMatch->iMCluster] == NULL)
						{
							UpdateMatchMatrix(pSMIMatch, pSMIMatch->score);
						}
						else
						{
							if (pSMIMatch->score < matchMatrix.Element[pSMIMatch->iCluster].Element[pSMIMatch->iModel*nMSegments + pSMIMatch->iMCluster]->score)
							{
								UpdateMatchMatrix(pSMIMatch, pSMIMatch->score);
							}
						}

						//create SegmentGT
						TP_ = CompareMatchToGT(pSMIMatch, true, 0.0, distanceThresh);
						
						if (TP_ && !segmentTP)
						{
							segmentTP = true;
							segmentGT.Element[iScene*nDominantClusters + iSCluster].iScene = iScene;

							segmentGT.Element[iScene*nDominantClusters + iSCluster].iSSegment = iSCluster;
							segmentGT.Element[iScene*nDominantClusters + iSCluster].iModel = pSMIMatch->iModel;
							segmentGT.Element[iScene*nDominantClusters + iSCluster].iMSegment = pSMIMatch->iMCluster;
						}
					}
					else
					{
						t << 0, 0, 0;
						//E = 66;
						E = 412.5;
					}
				}
				else
				{
					if (!breakPrint)
					{
						//printf("BREAK - nValidSampleCandidates = %d; iCluster: %d; iMIM: %d \n", nValidSampleCandidates, iSCluster, iMIM);
						breakPrint = true;
					}
				}

				pMModelInstance = pMModelInstance->pNext;

			}	//for all model MI

			iMIS++;

			pSModelInstance = pSModelInstance->pNext;

		}	// for all MI in cluster

		//Scene Segment doesn't have GT instance
		if (!segmentTP)
		{
			segmentGT.Element[iScene*nDominantClusters + iSCluster].iScene = iScene;
			segmentGT.Element[iScene*nDominantClusters + iSCluster].iSSegment = iSCluster;
			segmentGT.Element[iScene*nDominantClusters + iSCluster].iModel = -1;
			segmentGT.Element[iScene*nDominantClusters + iSCluster].iMSegment = -1;
		}

		if (nDominantClusters < 10)
			printf("\b");
		else
			printf("\b\b");

		if (iSCluster < 9)
		{
			printf("\b\b");
		}
		else
			printf("\b\b\b");

	}	// for all dominant clusters

	printf("completed.\n");

	int nSMI = iMIS;

	RVL_DELETE_ARRAY(iConsensusTemp.Element);

	RVL_DELETE_ARRAY(iValid.Element);

	RVL_DELETE_ARRAY(iConsensus.Element);

	RVL_DELETE_ARRAY(iRansacCandidates.Element);

	RVL_DELETE_ARRAY(iValidSampleCandidate.Element);

#ifdef PSGM_CALCULATE_PROBABILITY

	//calculate p(Mi|d) probability
	//float sigma = 1; //POSTAVITI KAO VANJSKI PARAMETAR?

	//float tConst = 1 / (2 * sigma);
	float tConst = 0.5;

	int nModels = 35;
	int nMSegments = 3;

	float maxCost = 412.5;

	int maxMSegments = nModels * nMSegments;

	int iSMI, iModel, iMSegment, j;

	float probability;

	int nMSCTI;

	Array<float> SMIminE;
	SMIminE.Element = new float[nSMI];

	for (i = 0; i < nSMI; i++)	
		SMIminE.Element[i] = maxCost;

	Array<Array<RECOG::PSGM_::MatchInstance*>> DBMBestMatches;
	DBMBestMatches.Element = new Array<RECOG::PSGM_::MatchInstance*>[nSMI];

	for (i = 0; i < nSMI; i++)
		DBMBestMatches.Element[i].Element = new RECOG::PSGM_::MatchInstance*[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			DBMBestMatches.Element[i].Element[j] = NULL;

	RECOG::PSGM_::MatchInstance *pDBMBestMatches;

	Array<Array<float>> DBMminE;
	DBMminE.Element = new Array<float>[nSMI];
		
	for (i = 0; i < nSMI; i++)
		DBMminE.Element[i].Element = new float[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			DBMminE.Element[i].Element[j] = maxCost;

	Array<Array<float>> SMIProbability1;

	SMIProbability1.Element = new Array<float>[nSMI];

	for (i = 0; i < nSMI; i++)
		SMIProbability1.Element[i].Element = new float[maxMSegments];

	Array<Array<float>> SMIProbability2;

	SMIProbability2.Element = new Array<float>[nSMI];

	for (i = 0; i < nSMI; i++)
		SMIProbability2.Element[i].Element = new float[maxMSegments];

	Array<Array<int>> nCTI;

	nCTI.Element = new Array<int>[nSMI];

	for (i = 0; i < nSMI; i++)
		nCTI.Element[i].Element = new int[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			nCTI.Element[i].Element[j] = 0.0;

	Array<Array<float>> Msum_;
	Msum_.Element = new Array<float>[nSMI];

	for (i = 0; i < nSMI; i++)
		Msum_.Element[i].Element = new float[maxMSegments];

	for (i = 0; i < nSMI; i++)
		for (j = 0; j < maxMSegments; j++)
			Msum_.Element[i].Element[j] = 0.0;	

	Array<float> SMIsum;
	SMIsum.Element = new float[nSMI];

	Array<float> SMIsum2;
	SMIsum2.Element = new float[nSMI];

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 1 (DEBUG)!\n");

	//find min score for all SMI and for all DB model segments 
	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;
			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			if (pSMIMatch->E < SMIminE.Element[iSMI])
				SMIminE.Element[iSMI] = pSMIMatch->E;

			if (pSMIMatch->E < DBMminE.Element[iSMI].Element[iMSegment])
			{
				DBMminE.Element[iSMI].Element[iMSegment] = pSMIMatch->E;
				DBMBestMatches.Element[iSMI].Element[iMSegment] = pSMIMatch;
			}

			pSMIMatch = pSMIMatch->pNext;			
		}
		else
		{
			break;
		}		
	}

	for (i = 0; i < nSMI; i++)
		SMIsum.Element[i] = 0.0;

	printf("Probability calculation - STEP 2 (DEBUG)!\n");

	for (i = 0; i < nSMI; i++)
	{
		for (j = 0; j < maxMSegments; j++)
		{
			pSMIMatch = DBMBestMatches.Element[i].Element[j];

			if (pSMIMatch != NULL)
			{
				iSMI = pSMIMatch->iSMI;

				iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

				//SMIProbability1.Element[iSMI].Element[iMSegment] = exp(tConst*(SMIminE.Element[iSMI] * SMIminE.Element[iSMI] - pSMIMatch->E * pSMIMatch->E));
				SMIProbability1.Element[iSMI].Element[iMSegment] = exp(tConst*(SMIminE.Element[iSMI] - pSMIMatch->E));

				SMIsum.Element[iSMI] += SMIProbability1.Element[iSMI].Element[iMSegment];
			}
		}
	}

	printf("Probability calculation - STEP 3 (DEBUG)!\n");

	float tempSum = 0;

	for (i = 0; i < nSMI; i++)
	{
		for (j = 0; j < maxMSegments; j++)
		{
			pSMIMatch = DBMBestMatches.Element[i].Element[j];

			if (pSMIMatch != NULL)
			{
				iSMI = pSMIMatch->iSMI;

				iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

				probability = SMIProbability1.Element[iSMI].Element[iMSegment];

				pSMIMatch->probability1 = probability / SMIsum.Element[iSMI];

				tempSum += pSMIMatch->probability1;
			}
		}

		if (iSMI == 91)
			printf("\n\nTempSum P1: %f\n\n", tempSum);

		tempSum = 0;
	}

	//reset SMIminE
	//for (i = 0; i < nSMI; i++)
	//	SMIminE.Element[i] = 66.0;

	for (i = 0; i < nSMI; i++)
		SMIsum.Element[i] = 0.0;

	//pSMIMatch = SMImatches.pFirst;

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 4 (DEBUG)!\n");

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			nCTI.Element[iSMI].Element[iMSegment]++;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}


	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 5 (DEBUG)!\n");

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			probability = exp(tConst*(SMIminE.Element[iSMI] - pSMIMatch->E));

			nMSCTI = nCTI.Element[iSMI].Element[iMSegment];

			Msum_.Element[iSMI].Element[iMSegment] += probability / nMSCTI;

			SMIsum.Element[iSMI] += probability / nMSCTI;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}

	//calculate probability2
	printf("Probability calculation - STEP 6 (DEBUG)!\n");

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			if (SMIsum.Element[iSMI] > 0.0)
				pSMIMatch->probability2 = Msum_.Element[iSMI].Element[iMSegment] / SMIsum.Element[iSMI];
			else
				pSMIMatch->probability2 = 0.0;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}

	//find best matches for each scene segment
	QList<RECOG::PSGM_::MatchInstance> *pSSegmentMatches1 = &SSegmentMatches1;
	QList<RECOG::PSGM_::MatchInstance> *pSSegmentMatches2 = &SSegmentMatches2;

	RECOG::PSGM_::MatchInstance *pMatch;
	RECOG::PSGM_::MatchInstance *pNewMatch;

	bool newMatch = true;

	//if (iScene == 0)
	//{
	RVLQLIST_INIT(pSSegmentMatches1);
	RVLQLIST_INIT(pSSegmentMatches2);
	//}

	printf("Probability calculation - STEP 7 (DEBUG)!\n");

	//Probability1
	for (i = 0; i < nSMI; i++)
	{
		for (j = 0; j < maxMSegments; j++)
		{
			pSMIMatch = DBMBestMatches.Element[i].Element[j];

			if (pSMIMatch != NULL)
			{
				if (pSMIMatch->iScene == iScene)
				{
					iSMI = pSMIMatch->iSMI;

					iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

					pMatch = pSSegmentMatches1->pFirst;

					while (pMatch)
					{
						if (pMatch->iScene == pSMIMatch->iScene && pMatch->iCluster == pSMIMatch->iCluster && pMatch->iModel == pSMIMatch->iModel && pMatch->iMCluster == pSMIMatch->iMCluster)
						{
							//change pMatch in QList
							if (pSMIMatch->probability1 > pMatch->probability1)
							{
								pMatch->iCRF = pSMIMatch->iCRF;

								pMatch->iSMI = pSMIMatch->iSMI;

								pMatch->iMMI = pSMIMatch->iMMI;

								for (i = 0; i < 9; i++)
									pMatch->R[i] = pSMIMatch->R[i];

								for (i = 0; i < 3; i++)
								{
									pMatch->t[i] = pSMIMatch->t[i];
									pMatch->tMatch[i] = pSMIMatch->tMatch[i];
								}

								pMatch->E = pSMIMatch->E;

								pMatch->score = pSMIMatch->score;

								pMatch->probability1 = pSMIMatch->probability1;

								pMatch->probability2 = pSMIMatch->probability2;

								pMatch->angle = pSMIMatch->angle;

								pMatch->distance = pSMIMatch->distance;
							}

							newMatch = false;

							break;
						}

						pMatch = pMatch->pNext;
					}

					if (newMatch)
					{
						//Add new match to QList
						RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pNewMatch);

						RVLQLIST_ADD_ENTRY(pSSegmentMatches1, pNewMatch);

						pNewMatch->iScene = pSMIMatch->iScene;

						pNewMatch->iCluster = pSMIMatch->iCluster;

						pNewMatch->iCRF = pSMIMatch->iCRF;

						pNewMatch->iSMI = pSMIMatch->iSMI;

						pNewMatch->iModel = pSMIMatch->iModel;

						pNewMatch->iMCluster = pSMIMatch->iMCluster;

						pNewMatch->iMMI = pSMIMatch->iMMI;

						for (i = 0; i < 9; i++)
							pNewMatch->R[i] = pSMIMatch->R[i];

						for (i = 0; i < 3; i++)
						{
							pNewMatch->t[i] = pSMIMatch->t[i];
							pNewMatch->tMatch[i] = pSMIMatch->tMatch[i];
						}

						pNewMatch->E = pSMIMatch->E;

						pNewMatch->score = pSMIMatch->score;

						pNewMatch->probability1 = pSMIMatch->probability1;

						pNewMatch->probability2 = pSMIMatch->probability2;

						pNewMatch->angle = pSMIMatch->angle;

						pNewMatch->distance = pSMIMatch->distance;
					}

					newMatch = true;
				}
			}
		}
	}

	//Probability2
	newMatch = true;

	//if (iScene == 0)
		pSMIMatch = SMImatches.pFirst;
	//else
	//	pSMIMatch = pCurrentSceneMatch->pNext;

	printf("Probability calculation - STEP 8 (DEBUG)!\n");

	while (pSMIMatch)
	{
		if (pSMIMatch->iScene == iScene)
		{
			iSMI = pSMIMatch->iSMI;

			iMSegment = pSMIMatch->iModel * nMSegments + pSMIMatch->iMCluster;

			pMatch = pSSegmentMatches2->pFirst;

			while (pMatch)
			{
				if (pMatch->iScene == pSMIMatch->iScene && pMatch->iCluster == pSMIMatch->iCluster && pMatch->iModel == pSMIMatch->iModel && pMatch->iMCluster == pSMIMatch->iMCluster)
				{
					//change pMatch in QList
					if ((pSMIMatch->probability2 > pMatch->probability2) || (pSMIMatch->probability2 == pMatch->probability2 && pSMIMatch->E < pMatch->E))
					{
						pMatch->iCRF = pSMIMatch->iCRF;

						pMatch->iSMI = pSMIMatch->iSMI;

						pMatch->iMMI = pSMIMatch->iMMI;

						for (i = 0; i < 9; i++)
							pMatch->R[i] = pSMIMatch->R[i];

						for (i = 0; i < 3; i++)
						{
							pMatch->t[i] = pSMIMatch->t[i];
							pMatch->tMatch[i] = pSMIMatch->tMatch[i];
						}

						pMatch->E = pSMIMatch->E;

						pMatch->score = pSMIMatch->score;

						pMatch->probability1 = pSMIMatch->probability1;

						pMatch->probability2 = pSMIMatch->probability2;

						pMatch->angle = pSMIMatch->angle;

						pMatch->distance = pSMIMatch->distance;
					}

					newMatch = false;

					break;
				}

				pMatch = pMatch->pNext;
			}

			if (newMatch)
			{
				//Add new match to QList
				RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::MatchInstance, pNewMatch);

				RVLQLIST_ADD_ENTRY(pSSegmentMatches2, pNewMatch);

				pNewMatch->iScene = pSMIMatch->iScene;

				pNewMatch->iCluster = pSMIMatch->iCluster;

				pNewMatch->iCRF = pSMIMatch->iCRF;

				pNewMatch->iSMI = pSMIMatch->iSMI;

				pNewMatch->iModel = pSMIMatch->iModel;

				pNewMatch->iMCluster = pSMIMatch->iMCluster;

				pNewMatch->iMMI = pSMIMatch->iMMI;

				for (i = 0; i < 9; i++)
					pNewMatch->R[i] = pSMIMatch->R[i];

				for (i = 0; i < 3; i++)
				{
					pNewMatch->t[i] = pSMIMatch->t[i];
					pNewMatch->tMatch[i] = pSMIMatch->tMatch[i];
				}

				pNewMatch->E = pSMIMatch->E;

				pNewMatch->score = pSMIMatch->score;

				pNewMatch->probability1 = pSMIMatch->probability1;

				pNewMatch->probability2 = pSMIMatch->probability2;

				pNewMatch->angle = pSMIMatch->angle;

				pNewMatch->distance = pSMIMatch->distance;
			}

			newMatch = true;

			//pCurrentSceneMatch = pSMIMatch;

			pSMIMatch = pSMIMatch->pNext;
		}
		else
		{
			break;
		}
	}			

	RVL_DELETE_ARRAY(SMIminE.Element);

	RVL_DELETE_ARRAY(DBMBestMatches.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(DBMminE.Element[i].Element);

	RVL_DELETE_ARRAY(DBMminE.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(SMIProbability1.Element[i].Element);

	RVL_DELETE_ARRAY(SMIProbability1.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(SMIProbability2.Element[i].Element);

	RVL_DELETE_ARRAY(SMIProbability2.Element);

	//RVL_DELETE_ARRAY(Msum.Element);

	RVL_DELETE_ARRAY(SMIsum.Element);

	RVL_DELETE_ARRAY(SMIsum2.Element);

	for (i = 0; i < nSMI; i++)
		RVL_DELETE_ARRAY(Msum_.Element[i].Element);

	RVL_DELETE_ARRAY(Msum_.Element);

#endif

	iScene++;

	//fclose(fp);

	//reset GT matches flag
	pECCVGT->ResetMatchFlag();

	SortMatchMatrix();

#ifdef RVLPSGM_SAVE_MATCHES
	printf("Saving matches to txt file...");
	SaveMatches();
	printf("completed!\n\n");
#endif
}

void PSGM::ComputeClusterNormalDistribution(
	RECOG::PSGM_::Cluster *pCluster)
{
	float R[9];

	float *X = R;
	float *Y = R + 3;
	float *Z = R + 6;

	float *meanN = Z;

	RVLNULL3VECTOR(meanN);

	float wTotal = 0.0f;

	int iiSurfel, iSurfel;
	Surfel *pSurfel;
	float *N;
	float wN[3];
	float w;
	float fTmp;
	float NP[3];
	float eig[2];
	int i1, i2, i3;

	for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
	{
		iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		N = pSurfel->N;

		w = (float)(pSurfel->size);

		RVLSCALE3VECTOR(N, w, wN);

		RVLSUM3VECTORS(meanN, wN, meanN);

		wTotal += w;
	}

	RVLSCALE3VECTOR2(meanN, wTotal, meanN);

	// Define projection reference frame.

	RVLORTHOGONAL3(Z, X, i1, i2, i3, fTmp);

	RVLCROSSPRODUCT3(Z, X, Y);

	// Project surfel normals onto the xy-plane of the projection reference frame and compute covariance matrix.

	float C[4];

	C[0] = C[1] = C[3] = 0.0f;

	for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
	{
		iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		N = pSurfel->N;

		w = (float)(pSurfel->size);

		RVLMULMX3X3VECT(R, N, NP);

		C[0] += (w * NP[0] * NP[0]);
		C[1] += (w * NP[0] * NP[1]);
		C[3] += (w * NP[1] * NP[1]);
	}

	C[0] /= wTotal;
	C[1] /= wTotal;
	C[2] = C[1];
	C[3] /= wTotal;

	// Compute eigenvalues of C.

	Eig2<float>(C, eig);

	// Compute normalDistributionStds.

	pCluster->normalDistributionStd1 = sqrt(eig[0]);
	pCluster->normalDistributionStd2 = sqrt(eig[1]);
	RVLCOPY3VECTOR(meanN, pCluster->N);
}

void PSGM::ComputeClusterBoundaryDiscontinuityPerc(int iCluster)
{
	RECOG::PSGM_::Cluster *pCluster = clusterMem + iCluster;

	int nContinuity = 0;
	int nDiscontinuity = 0;

	int iCluster_, iiSurfel, iSurfel, iSurfel_, iPt, iPt_, iBoundary, iPointEdge;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_;
	Point *pPt;
	MeshEdge *pEdge;
	RECOG::PSGM_::Cluster *pCluster_;

	for (iiSurfel = 0; iiSurfel < pCluster->iSurfelArray.n; iiSurfel++)
	{
		iSurfel = pCluster->iSurfelArray.Element[iiSurfel];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				pPt = pMesh->NodeArray.Element + iPt;

				if (pPt->bBoundary)
					nDiscontinuity++;
				else
				{
					pEdgePtr_ = pMesh->NodeArray.Element[iPt].EdgeList.pFirst;

					while (pEdgePtr_)
					{
						RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr_, pEdge, iPt_);

						iSurfel_ = pSurfels->surfelMap[iPt_];

						if (iSurfel_ >= 0 && iSurfel_ < pSurfels->NodeArray.n)
						{
							iCluster_ = clusterMap[iSurfel_];

							if (iCluster_ != iCluster && iCluster_ >= 0)
							{
								if (iCluster_ >= clusters.n)
									printf("iCluster_=%d\n", iCluster_);

								pCluster_ = clusterMem + iCluster_;

								if (pCluster_->bValid)
									break;
							}
						}

						pEdgePtr_ = pEdgePtr_->pNext;
					}

					if (pEdgePtr_)
						nContinuity++;
				}
			}
		}
	}

	if (nDiscontinuity == 0)
		pCluster->boundaryDiscontinuityPerc = 0;
	else
		pCluster->boundaryDiscontinuityPerc = 100 * nDiscontinuity / (nContinuity + nDiscontinuity);
}

void PSGM::WriteClusterNormalDistribution(FILE *fp)
{
	int iCluster;
	RECOG::PSGM_::Cluster *pCluster;

	for (iCluster = 0; iCluster < clusters.n; iCluster++)
	{
		pCluster = clusters.Element[iCluster];

		// Write eigenvalues to file.

		fprintf(fp, "%d\t%d\t%d\t%f\t%f\n", 
			iCluster, 
			pCluster->boundaryDiscontinuityPerc, 
			pCluster->size, 
			pCluster->normalDistributionStd1, 
			pCluster->normalDistributionStd2);
	}
}

void PSGM::MSTransformation(
	RECOG::PSGM_::ModelInstance *pMModelInstance,
	RECOG::PSGM_::ModelInstance *pSModelInstance,
	float *tBestMatch,
	float *R,
	float *t)
{
	float R_[9], t_[3];

	//RVLSCALE3VECTOR(pMModelInstance->t, 0, pMModelInstance->t);

	RVLINVTRANSF3D(pMModelInstance->R, pMModelInstance->t, R_, t_);

	RVLSUM3VECTORS(t_, tBestMatch, t_);

	RVLCOMPTRANSF3D(pSModelInstance->R, pSModelInstance->t, R_, t_, R, t);
}

void PSGM::FillMatch(
	RECOG::PSGM_::MatchInstance *pMatch,
	int ID,
	int iScene,
	int iSSegment,
	int iSRF,
	int iCTIS,
	int iModel,
	int iMSegment,
	int iCTIM,
	float *R,
	float *t,
	float *tMatch,
	float E,
	float score,
	float probability1,
	float probability2,
	float angleGT,
	float distanceGT,
	int nValids)
{
	int i;

	pMatch->ID = ID;
	pMatch->iScene = iScene;
	pMatch->iCluster = iSSegment;
	pMatch->iCRF = iSRF;
	pMatch->iSMI = iCTIS;
	pMatch->iModel = iModel;
	pMatch->iMCluster = iMSegment;
	pMatch->iMMI = iCTIM;
	
	for (i = 0; i < 9; i++)
		pMatch->R[i] = R[i];

	for (i = 0; i < 3; i++)
	{
		pMatch->t[i] = t[i];
		pMatch->tMatch[i] = tMatch[i];
	}

	pMatch->E = E;
	pMatch->score = score;
	pMatch->probability1 = probability1;
	pMatch->probability2 = probability2;
	pMatch->angle = angleGT;
	pMatch->distance = distanceGT;
	pMatch->nValids = nValids;
}

/*
void PSGM::SetNumberOfScenes(int scenesNumber)
{
	matches.Element = new RECOG::PSGM_::MatchInstance[scenesNumber*nDominantClusters];
	matches.n = 0;

	pMatches = matches.Element;
}*/

void PSGM::CompareMatchesToGT(
	ECCVGTLoader *ECCVGT,
	float scoreThresh,
	float angleThresh,
	float distanceThresh,
	float &precision,
	float &recall)
{
	int iMatches, iGTS, iGTM, nGTModels;

	int nGTSecenes = ECCVGT->GT.n;

	int TP = 0, FP = 0, FN = 0;

	float R[9], RGT[9], tGT[3], t[3];
		
	float V[3], theta, distance;

	bool match = false;

	//reset matches pointer!!
	pMatches = matches.Element;

	RVL::GTInstance *pGT;

	FILE *fp = fopen("RM.txt", "w");
	FILE *fp2 = fopen("GT_R.txt", "w");
	FILE *fp3 = fopen("R.txt", "w");

	//Find TP and FP
	for (iMatches = 0; iMatches < matches.n; iMatches++)
	{
		if (pMatches->score < scoreThresh)
		{
			match = false;

			for (iGTS = 0; iGTS < nGTSecenes; iGTS++)
			{				
				pGT = ECCVGT->GT.Element[iGTS].Element;

				nGTModels = ECCVGT->GT.Element[iGTS].n;

				for (iGTM = 0; iGTM < nGTModels; iGTM++)
				{
					if (pMatches->iScene == pGT->iScene)
					{
						if (pMatches->iModel == pGT->iModel)
						{
							RVLSCALEMX3X3(pGT->R, 1000, RGT);

							RVLMXMUL3X3T2(pMatches->R, RGT, R);

							fprintf(fp, "\n");

							for (int i = 0; i < 9; i++)
								if (i % 3 == 2)
									fprintf(fp, "%f\n", pMatches->R[i]);								
								else
									fprintf(fp, "%f\t", pMatches->R[i]);

							fprintf(fp2, "\n");

							for (int i = 0; i < 9; i++)
								if (i % 3 == 2)
									fprintf(fp2, "%f\n", pGT->R[i]);
								else
									fprintf(fp2, "%f\t", pGT->R[i]);

							fprintf(fp3, "\n");

							for (int i = 0; i < 9; i++)
								if (i % 3 == 2)
									fprintf(fp3, "%f\n", R[i]);
								else
									fprintf(fp3, "%f\t", R[i]);

							RVLSCALE3VECTOR(pGT->t, 1000, tGT)

							RVLDIF3VECTORS(pMatches->t, tGT, t);

							GetAngleAxis(R, V, theta);

							GetDistance(t, distance);

							fprintf(fp3, "\n%f\t%f\n", theta, distance);

							//if ((theta < angleThresh || (theta > (PI - angleThresh) && theta < (PI + angleThresh))) && distance < distanceThresh)
							if (distance < distanceThresh)
							{
								if (!pGT->matched)
								{
									TP++;

									pGT->matched = true;
								}									

								match = true;								

								break;
							}
						}
					}

					pGT++;
				}

				if (match)
					break;			
			}

			if (!match)
				FP++;
		}

		pMatches++;
	}

	//find FN
	for (iGTS = 0; iGTS < nGTSecenes; iGTS++)
	{
		pGT = ECCVGT->GT.Element[iGTS].Element;

		nGTModels = ECCVGT->GT.Element[iGTS].n;

		for (iGTM = 0; iGTM < nGTModels; iGTM++)
		{
			if (!pGT->matched)
			{
				FN++;
				printf("GT Model %d not matched!\n", iGTM);
			}				

			pGT++;
		}		
	}

	precision = (float)TP / (float)(TP + FP);

	recall = (float)TP / (float)(TP + FN);

	printf("TP: %d\n", TP);
	printf("FP: %d\n", FP);
	printf("FN: %d\n", FN);

	fclose(fp);
	fclose(fp2);
	fclose(fp3);
}

//UNDER CONSTRUCTION
void PSGM::CompareSMIMatchesToGT(
	ECCVGTLoader *ECCVGT,
	float scoreThresh,
	float angleThresh,
	float distanceThresh,
	float &precision,
	float &recall)
{

	RVL::GTInstance *pGT;

	int nGTSecenes = ECCVGT->GT.n;

	int TP = 0, FP = 0, FN = 0;

	int iGTS, iGTM, nGTModels;

	float R[9], RGT[9], tGT[3], t[3];

	float V[3], theta, distance;

	bool match, FPmatch;

	QList<RECOG::PSGM_::FPMatch> FPMatchList;
	QList<RECOG::PSGM_::FPMatch> *pFPMatchList = &FPMatchList;

	RVLQLIST_INIT(pFPMatchList);

	RECOG::PSGM_::FPMatch *pNewFPMatch;
	RECOG::PSGM_::FPMatch *pFPMatch;

	RECOG::PSGM_::MatchInstance *pSMIMatch = SMImatches.pFirst;

	//find TP and FP
	while (pSMIMatch)
	{
		if (pSMIMatch->score < scoreThresh)
		{
			match = false;

			//TP
			for (iGTS = 0; iGTS < nGTSecenes; iGTS++)
			{
				pGT = ECCVGT->GT.Element[iGTS].Element;

				nGTModels = ECCVGT->GT.Element[iGTS].n;

				for (iGTM = 0; iGTM < nGTModels; iGTM++)
				{
					if (pSMIMatch->iScene == pGT->iScene)
					{
						if (pSMIMatch->iModel == pGT->iModel)
						{
							RVLSCALEMX3X3(pGT->R, 1000, RGT);

							RVLMXMUL3X3T2(pSMIMatch->R, RGT, R);

							RVLSCALE3VECTOR(pGT->t, 1000, tGT)

							RVLDIF3VECTORS(pSMIMatch->t, tGT, t);

							GetAngleAxis(R, V, theta);

							GetDistance(t, distance);

							//if ((theta < angleThresh || (theta >(PI - angleThresh) && theta < (PI + angleThresh))) && distance < distanceThresh)
							if (distance < distanceThresh)
							{
								if (!pGT->matched)
								{
									TP++;

									pGT->matched = true;
								}

								match = true;

								break;
							}
						}
					}

					pGT++;
				}

				if (match)
					break;
			}

			//FP
			if (!match)
			{
				FPmatch = false;

				pFPMatch = pFPMatchList->pFirst;

				while (pFPMatch)
				{
					if (pSMIMatch->iScene == pFPMatch->iScene)
					{
						if (pSMIMatch->iModel == pFPMatch->iModel)
						{
							RVLDIF3VECTORS(pSMIMatch->t, pFPMatch->t, t);

							GetDistance(t, distance);

							//printf("distance: %f\n", distance);

							if (distance < distanceThresh)
							{
								FPmatch = true;

								//Update FP match cluster
								RVLSCALE3VECTOR(pFPMatch->t, pFPMatch->n, pFPMatch->t);

								RVLSUM3VECTORS(pSMIMatch->t, pFPMatch->t, pFPMatch->t);

								pFPMatch->n++;

								RVLSCALE3VECTOR2(pFPMatch->t, pFPMatch->n, pFPMatch->t);

								break;
							}
						}
					}

					pFPMatch = pFPMatch->pNext;
				}

				if (!FPmatch)
				{
					//add new cluster to FPMatch list
					RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::FPMatch, pNewFPMatch);

					pNewFPMatch->iScene = pSMIMatch->iScene;

					pNewFPMatch->iModel = pSMIMatch->iModel;

					RVLCOPY3VECTOR(pSMIMatch->t, pNewFPMatch->t);

					pNewFPMatch->n = 1;

					RVLQLIST_ADD_ENTRY(pFPMatchList, pNewFPMatch);

					FP++;
				}

			}
		}

		pSMIMatch = pSMIMatch->pNext;		
	}

	//find FN
	for (iGTS = 0; iGTS < nGTSecenes; iGTS++)
	{
		pGT = ECCVGT->GT.Element[iGTS].Element;

		nGTModels = ECCVGT->GT.Element[iGTS].n;

		for (iGTM = 0; iGTM < nGTModels; iGTM++)
		{
			if (!pGT->matched)
			{
				FN++;
				printf("GT Model %d not matched on scene %d!\n", iGTM, iGTS);
			}

			pGT++;
		}
	}

	precision = (float)TP / (float)(TP + FP);

	recall = (float)TP / (float)(TP + FN);

	printf("TP: %d\n", TP);
	printf("FP: %d\n", FP);
	printf("FN: %d\n", FN);
}

void PSGM::CompareProbabilityMatchesToGT(
	ECCVGTLoader *ECCVGT,
	float probabilityThresh,
	int probabilityCalculation,
	bool poseCheck,
	float angleThresh,
	float distanceThresh,
	float &precision,
	float &recall)
{
	RVL::GTInstance *pGT;

	int nGTSecenes = ECCVGT->GT.n;

	int TP = 0, FP = 0, FN = 0;

	int iGTS, iGTM, nGTModels;

	float R[9], R_[9], RGT[9], tGT[3], t[3];

	float V[3], theta, distance;

	bool match;

	RECOG::PSGM_::MatchInstance *pMatch;

	if (probabilityCalculation == 1)
		pMatch = SSegmentMatches1.pFirst;
	else
		pMatch = SSegmentMatches2.pFirst;

	float probability;

	//find TP and FP
	while (pMatch)
	{
		if (probabilityCalculation == 1)
			probability = pMatch->probability1;
		else
			probability = pMatch->probability2;

		//if (probability >= probabilityThresh)
		if ((probability - probabilityThresh) >= -1.4901161138336505e-009) //because of float precision => 0.1 is represented by 0.100000001
		{
			match = false;

			//TP
			for (iGTS = 0; iGTS < nGTSecenes; iGTS++)
			{
				pGT = ECCVGT->GT.Element[iGTS].Element;

				nGTModels = ECCVGT->GT.Element[iGTS].n;

				if (pMatch->iScene == pGT->iScene)
				{
					for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
					{
						if (pMatch->iModel == pGT->iModel)
						{
							//if (pMatch->iCluster == 2 && pMatch->iModel == 32 && probability > 0.97)
							//	int debug = 0;

							if (pMatch->iCluster == 4 && pMatch->iModel == 22)
								int debug = 0;

							if (poseCheck)
							{

								RVLSCALEMX3X3(pGT->R, 1000, RGT);

								RVLMXMUL3X3T2(pMatch->R, RGT, R);

								RVLSCALE3VECTOR(pGT->t, 1000, tGT)

#ifdef RVLPSGM_MATCH_SEGMENT_CENTROID
									
								RVLDIFMX3X3(RGT, pMatch->R, R_);

								RVLMULMX3X3VECT(R_, modelInstanceDB.Element[pMatch->iMMI].tc, t);

								RVLSUM3VECTORS(t, tGT, t);

								RVLDIF3VECTORS(t, pMatch->t, t);
#else if
								RVLDIF3VECTORS(pMatch->t, tGT, t);
#endif

								GetAngleAxis(R, V, theta);

								GetDistance(t, distance);

								if (pMatch->iCluster == 2 && pMatch->iModel == 32 && pMatch->probability2 > 0.5)
									printf("Cluster: %d\nModel: %d\nProbability: %f\nDistance:%f\n\n", pMatch->iCluster, pMatch->iModel, pMatch->probability2, distance);

								if (pMatch->iCluster == 1 && pMatch->iModel == 26 && pMatch->probability2 > 0.5)
									printf("Cluster: %d\nModel: %d\nProbability: %f\nDistance:%f\n\n", pMatch->iCluster, pMatch->iModel, pMatch->probability2, distance);

								if (pMatch->iCluster == 4 && pMatch->iModel == 1 && pMatch->probability2 > 0.5)
									printf("Cluster: %d\nModel: %d\nProbability: %f\nDistance:%f\n\n", pMatch->iCluster, pMatch->iModel, pMatch->probability2, distance);

								if (pMatch->iCluster == 5 && pMatch->iModel == 14 && pMatch->probability2 > 0.5)
									printf("Cluster: %d\nModel: %d\nProbability: %f\nDistance:%f\n\n", pMatch->iCluster, pMatch->iModel, pMatch->probability2, distance);
								

								//if ((theta < angleThresh || (theta >(PI - angleThresh) && theta < (PI + angleThresh))) && distance < distanceThresh)
								if (distance < distanceThresh)
								{
									if (!pGT->matched)
									{
										TP++;

										pGT->matched = true;
									}

									match = true;

									break;
								}

							}
							else
							{
								if (!pGT->matched)
								{
									TP++;

									pGT->matched = true;
								}

								match = true;

								break;
							}
						}
					}
				}

				if (match)
					break;
			}

			//FP
			if (!match)
				FP++;
		}

		pMatch = pMatch->pNext;
	}

	//find FN
	for (iGTS = 0; iGTS < nGTSecenes; iGTS++)
	{
		pGT = ECCVGT->GT.Element[iGTS].Element;

		nGTModels = ECCVGT->GT.Element[iGTS].n;

		for (iGTM = 0; iGTM < nGTModels; iGTM++)
		{
			if (!pGT->matched)
			{
				FN++;
				printf("GT Model %d not matched on scene %d!\n", iGTM, iGTS);
			}

			pGT++;
		}
	}

	if (TP + FP > 0)
		precision = (float)TP / (float)(TP + FP);
	else
		precision = 0.0;

	recall = (float)TP / (float)(TP + FN);

	printf("TP: %d\n", TP);
	printf("FP: %d\n", FP);
	printf("FN: %d\n", FN);

}


//Compare single match to GT
bool PSGM::CompareMatchToGT(
	RECOG::PSGM_::MatchInstance *pMatch,
	//float score,
	//float scoreThresh,
	bool poseCheck,
	float angleThresh,
	float distanceThresh)
{
	RVL::GTInstance *pGT;

	int iGTS, iGTM, nGTModels;

	float R[9], R_[9], RGT[9], tGT[3], t[3];

	float V[3], theta, distance;

	iGTS = pMatch->iScene;

	//TP
	pGT = pECCVGT->GT.Element[iGTS].Element;

	nGTModels = pECCVGT->GT.Element[iGTS].n;

	for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
	{
		if (pMatch->iModel == pGT->iModel)
		{
			if (poseCheck)
			{
				RVLSCALEMX3X3(pGT->R, 1000, RGT);

				RVLMXMUL3X3T2(pMatch->R, RGT, R);

				RVLSCALE3VECTOR(pGT->t, 1000, tGT)

#ifdef RVLPSGM_MATCH_SEGMENT_CENTROID

				RVLDIFMX3X3(RGT, pMatch->R, R_);

				RVLMULMX3X3VECT(R_, modelInstanceDB.Element[pMatch->iMMI].tc, t);

				RVLSUM3VECTORS(t, tGT, t);

				RVLDIF3VECTORS(t, pMatch->t, t);
#else if
				RVLDIF3VECTORS(pMatch->t, tGT, t);
#endif

				GetAngleAxis(R, V, theta);

				GetDistance(t, distance);

				//if ((theta < angleThresh || (theta >(PI - angleThresh) && theta < (PI + angleThresh))) && distance < distanceThresh)
				if (distance < distanceThresh)
				{
					//if (!pGT->matched)
					//{
						pGT->matched = true;
						return true;								
					//}

					//return true; // Za sve TP matcheve vraæa true!! (Moguæe da za jedan GT model bude više TP-ova)
				}
			}
			else
			{
				//if (!pGT->matched)
				//{
					pGT->matched = true;
					return true;							
				//}

				//return true; // Za sve TP matcheve vraæa true!! (Moguæe da za jedan GT model bude više TP-ova)
			}
		}
	}

	//FP
	return false;
}

bool PSGM::CompareMatchToSegmentGT(
	RECOG::PSGM_::MatchInstance *pMatch)
{	
	int iScene = pMatch->iScene;
	int iSegmentGT = iScene * nDominantClusters + pMatch->iCluster;

	int nGTModels, iGTM;

	RVL::GTInstance *pGT;

	pGT = pECCVGT->GT.Element[iScene].Element;
	nGTModels = pECCVGT->GT.Element[iScene].n;

	//if (pMatch->iModel == segmentGT.Element[iSegmentGT].iModel && pMatch->iMCluster == segmentGT.Element[iSegmentGT].iMSegment)
	if (pMatch->iModel == segmentGT.Element[iSegmentGT].iModel)
	{
		//set GT matched flag
		for (iGTM = 0; iGTM < nGTModels; iGTM++, pGT++)
		{
			if (pMatch->iModel == pGT->iModel)
				pGT->matched = true;
		}

		return true;
	}
	else
		return false;

}

void PSGM::CountTPandFN(
	int &TP,
	int &FN,
	bool printMatchInfo)
{
	int iGTS, iGTM, nGTModels;
	
	TP = 0;
	FN = 0;

	RVL::GTInstance *pGT;

	int nGTSecenes = pECCVGT->GT.n;

	//find FN
	//for (iGTS = 0; iGTS < nGTSecenes; iGTS++)
	//{
		//pGT = ECCVGT->GT.Element[iGTS].Element;
		pGT = pECCVGT->GT.Element[iScene-1].Element; //iScene-1 because iScene is incremented in Match()

		nGTModels = pECCVGT->GT.Element[iScene-1].n; //iScene-1 because iScene is incremented in Match()

		for (iGTM = 0; iGTM < nGTModels; iGTM++)
		{
			if (!pGT->matched)
			{
				FN++;

				if (printMatchInfo)
					printf("GT Model %d NOT matched on scene %d!\n", iGTM, iScene - 1);
			}
			else
			{
				TP++;

				if(printMatchInfo)
					printf("GT Model %d matched on scene %d!\n", iGTM, iScene - 1);
			}

			pGT++;
		}
	//}
}

void PSGM::CalculatePR(int TP, int FP, int FN, float &precision, float &recall)
{
	if (TP + FP > 0)
		precision = (float)TP / (float)(TP + FP);
	else
		precision = 0.0;

	recall = (float)TP / (float)(TP + FN);

}

void PSGM::CreateMatchMatrix()
{
	int iSSegment, iMSegment;
	
	matchMatrix.Element = new Array<RECOG::PSGM_::MatchInstance*>[nDominantClusters];
	matchMatrix.n = nDominantClusters;

	sortedMatches.Element = new Array<SortIndex<float>>[nDominantClusters];
	sortedMatches.n = nDominantClusters;

	for (iSSegment = 0; iSSegment < nDominantClusters; iSSegment++)
	{
		matchMatrix.Element[iSSegment].Element = new RECOG::PSGM_::MatchInstance*[nModels*nMSegments];
		matchMatrix.Element[iSSegment].n = nModels*nMSegments;

		sortedMatches.Element[iSSegment].Element = new SortIndex<float>[nModels*nMSegments];
		sortedMatches.Element[iSSegment].n = nModels*nMSegments;


		for (iMSegment = 0; iMSegment < nModels*nMSegments; iMSegment++)
		{
			matchMatrix.Element[iSSegment].Element[iMSegment] = NULL;

			sortedMatches.Element[iSSegment].Element[iMSegment].idx = iMSegment;
			sortedMatches.Element[iSSegment].Element[iMSegment].cost = 413; //PROMIJENITI!!
		}
	}
}

void PSGM::ClearMatchMatrix()
{
	int iSSegment, iMSegment;

	for (iSSegment = 0; iSSegment < nDominantClusters; iSSegment++)
		for (iMSegment = 0; iMSegment < nModels*nMSegments; iMSegment++)
		{
			matchMatrix.Element[iSSegment].Element[iMSegment] = NULL;

			sortedMatches.Element[iSSegment].Element[iMSegment].idx = iMSegment;
			sortedMatches.Element[iSSegment].Element[iMSegment].cost = 413; //PROMIJENITI!!
		}

}

void PSGM::UpdateMatchMatrix(RECOG::PSGM_::MatchInstance *pMatch, float cost)
{
	int iModel, iMSegment, iSSegment;

	iModel = pMatch->iModel;
	iMSegment = pMatch->iMCluster;
	iSSegment = pMatch->iCluster;

	matchMatrix.Element[iSSegment].Element[iModel*nMSegments+iMSegment] = pMatch;

	sortedMatches.Element[iSSegment].Element[iModel*nMSegments + iMSegment].cost = cost;
}

void PSGM::SortMatchMatrix()
{
	int iSSegment;

	for (iSSegment = 0; iSSegment < nDominantClusters; iSSegment++)
	{
		BubbleSort<SortIndex<float>>(sortedMatches.Element[iSSegment]);
	}
}

void PSGM::EvaluateMatchesByScore(FILE *fp, FILE *fpLog, bool compareSegmentsWithoutGT)
{
	float precision, recall;
	float angleThresh, distanceThresh;
	float scoreThreshMin, scoreThreshMax, scoreThresh;

	int graphID = 0;
	int nBestMatches = 104;

	int *firstTP = new int[nDominantClusters];
	float *firstTPScore = new float[nDominantClusters];

	int iSSegment, iMSegment, iMatch, iBestMatches;

	distanceThresh = 100;

	scoreThreshMin = 180.0;
	scoreThreshMax = 350.0;

	//NEW PR calculation
	RECOG::PSGM_::MatchInstance *pMatch;
	RECOG::PSGM_::MatchInstance *pSegmentBestMatch;

	float scoreRatio;

	bool TPMatch;

	int TP_ = 0, FP_ = 0, FN_ = 0;

	for (iBestMatches = 0; iBestMatches <= nBestMatches; iBestMatches++)
	{
		for (scoreThresh = scoreThreshMin; scoreThresh <= scoreThreshMax; scoreThresh++)
		{
			for (iSSegment = 0; iSSegment < nDominantClusters; iSSegment++)
			{
				firstTP[iSSegment] = -1;

				iMatch = sortedMatches.Element[iSSegment].Element[0].idx;
				pSegmentBestMatch = matchMatrix.Element[iSSegment].Element[iMatch];

				for (iMSegment = 0; iMSegment <= iBestMatches; iMSegment++)
				{
					iMatch = sortedMatches.Element[iSSegment].Element[iMSegment].idx;

					pMatch = matchMatrix.Element[iSSegment].Element[iMatch];

					scoreRatio = pSegmentBestMatch->score / pMatch->score;

					if (pMatch && pMatch->score < scoreThresh)
					{
					#ifdef RVLPSGM_MATCH_USING_SEGMENT_GT
						TPMatch = CompareMatchToSegmentGT(pMatch);
					#else
						TPMatch = CompareMatchToGT(pMatch, true, 0.0, distanceThresh);
					#endif

						if (!TPMatch)
						{
							int iSegmentGT = pMatch->iScene * nDominantClusters + pMatch->iCluster;

							//eliminate FP from segments without GT
							if (!compareSegmentsWithoutGT && segmentGT.Element[iSegmentGT].iModel == -1)
								continue;
							else
								FP_++;
						}
						else
							if (firstTP[iSSegment] == -1)
							{
								firstTP[iSSegment] = iMSegment;
								firstTPScore[iSSegment] = pMatch->score;
							}
					}
				}
			}

			#ifdef RVLPSGM_EVALUATION_PRINT_INFO
				CountTPandFN(TP_, FN_, true);
			#else
				CountTPandFN(TP_, FN_, false);
			#endif

			CalculatePR(TP_, FP_, FN_, precision, recall);

			//TP += TP_;
			//FP += FP_;
			//FN += FN_;

			pECCVGT->ResetMatchFlag();
#ifdef RVLPSGM_EVALUATION_PRINT_INFO
			printf("---------------------------------------------------\n");
			printf("Scene: %d\n", iScene - 1);
			printf("TP: %d\n", TP_);
			printf("FP: %d\n", FP_);
			printf("FN: %d\n", FN_);
			printf("nBestMatches: %d\n", iBestMatches);
			printf("ScoreThresh: %f\n", scoreThresh);
			printf("Precision: %f\n", precision);
			printf("Recall: %f\n", recall);
			printf("...................................................\n");
			for (iSSegment = 0; iSSegment < nDominantClusters; iSSegment++)
				if (firstTP[iSSegment] != -1)
					printf("First TP for segment %d is: %d (score = %f)\n", iSSegment, firstTP[iSSegment], firstTPScore[iSSegment]);
			printf("---------------------------------------------------\n\n");
#endif

			//print to log file
				fprintf(fpLog, "---------------------------------------------------\n");
				fprintf(fpLog, "Scene: %d\n", iScene - 1);
				fprintf(fpLog, "TP: %d\n", TP_);
				fprintf(fpLog, "FP: %d\n", FP_);
				fprintf(fpLog, "FN: %d\n", FN_);
				fprintf(fpLog, "nBestMatches: %d\n", iBestMatches);
				fprintf(fpLog, "ScoreThresh: %f\n", scoreThresh);
				fprintf(fpLog, "Precision: %f\n", precision);
				fprintf(fpLog, "Recall: %f\n", recall);
				fprintf(fpLog, "...................................................\n");
				for (iSSegment = 0; iSSegment < nDominantClusters; iSSegment++)
					if (firstTP[iSSegment] != -1)
						fprintf(fpLog, "First TP for segment %d is: %d (score = %f)\n", iSSegment, firstTP[iSSegment], firstTPScore[iSSegment]);
				fprintf(fpLog, "---------------------------------------------------\n\n");

#ifdef RVLPSGM_MATCH_USING_SEGMENT_GT
			distanceThresh = -1.0;		
#endif
			fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\n", graphID, iScene - 1, TP_, FP_, FN_, iBestMatches, scoreThresh, distanceThresh, precision, recall);

			TP_ = 0; FP_ = 0; FN_ = 0;

			graphID++;
		}
	}

	delete[] firstTP;
	delete[] firstTPScore;
}

void PSGM::SaveMatches()
{

	char *matchFileName = RVLCreateString(sceneFileName);

	sprintf(matchFileName + strlen(matchFileName) - 3, "smf");
	

	//fp = fopen("F:\\Projekti\\ARP3D\\Auxiliary\\Models\\SMIMatches.txt", "w");
	FILE *fp = fopen(matchFileName, "w");

	RECOG::PSGM_::MatchInstance *pSMIMatch = SMImatches.pFirst;

	int i;

	while (pSMIMatch)
	{
		fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t", pSMIMatch->iScene, pSMIMatch->iCluster, pSMIMatch->iCRF, pSMIMatch->iSMI, pSMIMatch->iModel, pSMIMatch->iMCluster, pSMIMatch->iMMI);

		//fprintf(fp2, "%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\n", iSCluster, bestSRF, bestMatchMIMID, bestMatchModelID, minETotal, tBestMatch[0], tBestMatch[1], tBestMatch[2]);

		for (i = 0; i < 9; i++)
			fprintf(fp, "%f\t", pSMIMatch->R[i]);

		//for (i = 0; i < 3; i++)
		//	fprintf(fp, "%f\t", pSMIMatch->t[i]);

		for (i = 0; i < 3; i++)
			fprintf(fp, "%f\t", pSMIMatch->tMatch[i]);

		fprintf(fp, "%f\t", pSMIMatch->score);

		fprintf(fp, "%f\t", pSMIMatch->E);

		fprintf(fp, "%f\t", pSMIMatch->probability2);

		fprintf(fp, "%f\t", pSMIMatch->angle);

		fprintf(fp, "%f\t", pSMIMatch->distance);

		fprintf(fp, "%d\n", pSMIMatch->nValids);

		pSMIMatch = pSMIMatch->pNext;
	}

	fclose(fp);
}

void PSGM::SaveSegmentGT(FILE*fp)
{
	int iSSegment;

	for (iSSegment = 0; iSSegment < nDominantClusters; iSSegment++)
		fprintf(fp, "%d\t%d\t%d\t%d\n", segmentGT.Element[(iScene - 1)*nDominantClusters + iSSegment].iScene, segmentGT.Element[(iScene - 1)*nDominantClusters + iSSegment].iSSegment, segmentGT.Element[(iScene - 1)*nDominantClusters + iSSegment].iModel, segmentGT.Element[(iScene - 1)*nDominantClusters + iSSegment].iMSegment);
}

void PSGM::ConvexTemplateCentoidID()
{
	int iPlane, i;

	float minx, maxx, miny, maxy, minz, maxz;
	

	for (i = 0; i < 6; i++)
		centroidID.Element[i].Idx = 0;
	
	minx = convexTemplate.Element[0].N[0];
	maxx = convexTemplate.Element[0].N[0];

	miny = convexTemplate.Element[0].N[1];
	maxy = convexTemplate.Element[0].N[1];

	minz = convexTemplate.Element[0].N[2];
	maxz = convexTemplate.Element[0].N[2];		

	for (iPlane = 1; iPlane < convexTemplate.n; iPlane++)
	{
		//find minx
		if (convexTemplate.Element[iPlane].N[0] < minx)
		{
			minx = convexTemplate.Element[iPlane].N[0];
			centroidID.Element[0].Idx = iPlane;
		}

		//find maxx
		if (convexTemplate.Element[iPlane].N[0] > maxx)
		{
			maxx = convexTemplate.Element[iPlane].N[0];
			centroidID.Element[1].Idx = iPlane;
		}

		//find miny
		if (convexTemplate.Element[iPlane].N[1] < miny)
		{
			miny = convexTemplate.Element[iPlane].N[1];
			centroidID.Element[2].Idx = iPlane;
		}

		//find maxy
		if (convexTemplate.Element[iPlane].N[1] > maxy)
		{
			maxy = convexTemplate.Element[iPlane].N[1];
			centroidID.Element[3].Idx = iPlane;
		}

		//find minz
		if (convexTemplate.Element[iPlane].N[2] < minz)
		{
			minz = convexTemplate.Element[iPlane].N[2];
			centroidID.Element[4].Idx = iPlane;
		}

		//find maxz
		if (convexTemplate.Element[iPlane].N[2] > maxz)
		{
			maxz = convexTemplate.Element[iPlane].N[2];
			centroidID.Element[5].Idx = iPlane;
		}
	}
}
//END VIDOVIC

void PSGM::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	unsigned char *selectionColor)
{
	pVisualizer->normalLength = 10.0;

	pVisualizer->SetMesh(pMesh);

	displayData.pMesh = pMesh;
	displayData.pSurfels = pSurfels;
	displayData.pRecognition = this;
	displayData.pVisualizer = pVisualizer;	
	RVLCOPY3VECTOR(selectionColor, displayData.selectionColor);
	displayData.iSelectedCluster = -1;

	pSurfels->DisplayData.keyPressUserFunction = &RECOG::PSGM_::keyPressUserFunction;
	pSurfels->DisplayData.mouseRButtonDownUserFunction = &RECOG::PSGM_::mouseRButtonDownUserFunction;
	pSurfels->DisplayData.vpUserFunctionData = &displayData;

	pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);
}

void PSGM::Display()
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	DisplayClusters();

	pSurfels->DisplayEdgeFeatures();

	displayData.bClusters = true;

	//pSurfels->Display(pVisualizer, pMesh);

	//DisplayVertices();

	//if (!displayData.bVertices)
	//	displayData.vertices->VisibilityOff();

	//DisplayReferenceFrames();
}

void PSGM::DisplayClusters()
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	int iCluster;
	unsigned char color[3];

	for (iCluster = 0; iCluster < clusters.n; iCluster++)
	{
		RandomColor(color);

		PaintCluster(iCluster, color);
	}
}

void PSGM::DisplayModelInstance(Visualizer *pVisualizer)
{
	//Create polygonal mesh

	//// Setup four points
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(0.0, 0.0, 0.0);
	points->InsertNextPoint(1.0, 0.0, 0.0);
	points->InsertNextPoint(1.5, 0.5, 0.0);
	points->InsertNextPoint(1.0, 1.0, 0.0);
	points->InsertNextPoint(0.0, 1.0, 0.0);

	// Define some colors
	unsigned char red[3] = { 255, 0, 0 };
	unsigned char green[3] = { 0, 255, 0 };
	unsigned char blue[3] = { 0, 0, 255 };
	unsigned char white[3] = { 255, 255, 255 };
	unsigned char black[3] = { 0, 0, 0 };

	// Setup the colors array
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	// Add the three colors we have created to the array
	colors->InsertNextTupleValue(red);
	colors->InsertNextTupleValue(green);
	colors->InsertNextTupleValue(blue);
	colors->InsertNextTupleValue(white);
	colors->InsertNextTupleValue(black);

	// Create the polygon
	vtkSmartPointer<vtkPolygon> polygon =
		vtkSmartPointer<vtkPolygon>::New();
	polygon->GetPointIds()->SetNumberOfIds(5); //make a quad
	polygon->GetPointIds()->SetId(0, 0);
	polygon->GetPointIds()->SetId(1, 1);
	polygon->GetPointIds()->SetId(2, 2);
	polygon->GetPointIds()->SetId(3, 3);
	polygon->GetPointIds()->SetId(4, 4);
	//polygon->GetPointIds()->SetId(5, 5);

	// Add the polygon to a list of polygons
	vtkSmartPointer<vtkCellArray> polygons =
		vtkSmartPointer<vtkCellArray>::New();
	polygons->InsertNextCell(polygon);

	// Create a polydata object and add everything to it
	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	polydata->SetPolys(polygons);
	polydata->GetPointData()->SetScalars(colors);

	//Mapper
	pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
	//map->SetInputData(pMesh->pPolygonData);
	pVisualizer->map->SetInputData(polydata);
	//map->SetInputConnection(polyDataNormals->GetOutputPort());
	pVisualizer->map->InterpolateScalarsBeforeMappingOff();

	//Actor
	pVisualizer->actor = vtkSmartPointer<vtkActor>::New();
	pVisualizer->actor->SetMapper(pVisualizer->map);

	//Insert actor
	pVisualizer->renderer->AddActor(pVisualizer->actor);
}

void PSGM::PaintCluster(
	int iCluster,
	unsigned char *color)
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	int i;
	int iSurfel;
	Surfel *pSurfel;

	for (i = 0; i < pCluster->iSurfelArray.n; i++)
	{
		iSurfel = pCluster->iSurfelArray.Element[i];

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);
	}
}

void PSGM::PaintClusterVertices(
	int iCluster,
	unsigned char *color)
{
	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	pSurfels->PaintVertices(&(pCluster->iVertexArray), color);
}

void PSGM::DisplayReferenceFrames()
{
	Visualizer *pVisualizer = displayData.pVisualizer;

	double axesLength = 10.0;

	// Create the polydata where we will store all the geometric data
	referenceFramesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	// Create lines.
	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	// Create colors.
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	int nClusters = RVLMIN(clusters.n, nDominantClusters);

	RECOG::PSGM_::Cluster *pCluster;
	int iCluster;
	RECOG::PSGM_::ModelInstance *pModelInstance;

	for (iCluster = 0; iCluster < nClusters; iCluster++)
	{
		pCluster = clusters.Element[iCluster];

		pModelInstance = pCluster->modelInstanceList.pFirst;

		while (pModelInstance)
		{
			pVisualizer->AddReferenceFrame(pts, lines, colors, pModelInstance->R, pModelInstance->t, 10.0);

			//vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

			//axes->SetTotalLength(axesLength, axesLength, axesLength);

			//vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

			//vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
			//double T[16];
			//RVLCREATE3DTRANSF(pModelInstance->R, pModelInstance->t, T);

			//transform->SetMatrix(T);

			//axes->SetUserTransform(transform);

			//vtkMatrix4x4 *T_ = axes->GetMatrix();

			//pVisualizer->renderer->AddActor(axes);

			pModelInstance = pModelInstance->pNext;
		}
	}

	// Add the points to the polydata container
	referenceFramesPolyData->SetPoints(pts);

	// Add the lines to the polydata container
	referenceFramesPolyData->SetLines(lines);

	// Color the lines.
	referenceFramesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(referenceFramesPolyData);

	displayData.referenceFrames = vtkSmartPointer<vtkActor>::New();
	displayData.referenceFrames->SetMapper(mapper);

	pVisualizer->renderer->AddActor(displayData.referenceFrames);
}

bool RVL::RECOG::PSGM_::keyPressUserFunction(
	Mesh *pMesh, 
	SurfelGraph *pSurfels, 
	std::string &key, 
	void *vpData)
{
	RECOG::PSGM_::DisplayData *pData = (RECOG::PSGM_::DisplayData *)vpData;

	PSGM *pRecognition = pData->pRecognition;
	Visualizer *pVisualizer = pData->pVisualizer;

	if (key == "a")
	{
		pData->bClusters = !pData->bClusters;

		if (pData->bClusters)
			pRecognition->DisplayClusters();
		else
		{
			pSurfels->Display(pVisualizer, pMesh);

			if (pSurfels->DisplayData.bVertices)
			{
				if (pData->iSelectedCluster >= 0)
				{
					unsigned char color[3];

					RVLSET3VECTOR(color, 255, 0, 0);

					pRecognition->PaintClusterVertices(pData->iSelectedCluster, color);

					pSurfels->UpdateVertexDisplayLines();
				}
			}

			pData->iSelectedCluster = -1;
		}
			
		return true;
	}

	return false;
}

bool RVL::RECOG::PSGM_::mouseRButtonDownUserFunction(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	int iSelectedPt,
	int iSelectedSurfel,
	void *vpData)
{
	RECOG::PSGM_::DisplayData *pData = (RECOG::PSGM_::DisplayData *)vpData;

	if (!pData->bClusters)
		return false;

	PSGM *pRecognition = pData->pRecognition;
	Visualizer *pVisualizer = pData->pVisualizer;

	unsigned char color[3];

	if (pData->iSelectedCluster >= 0)
	{
		RandomColor(color);

		pRecognition->PaintCluster(pData->iSelectedCluster, color);

		if (pSurfels->DisplayData.bVertices)
		{
			RVLSET3VECTOR(color, 255, 0, 0);

			pRecognition->PaintClusterVertices(pData->iSelectedCluster, color);
		}
	}

	int iCluster = pRecognition->clusterMap[iSelectedSurfel];

	if (iCluster >= 0)
	{
		printf("Selected segment: %d\n", iCluster);

		pRecognition->PaintCluster(iCluster, pData->selectionColor);

		if (pSurfels->DisplayData.bVertices)
		{
			RVLSET3VECTOR(color, 255, 255, 0);

			pRecognition->PaintClusterVertices(iCluster, color);

			pSurfels->UpdateVertexDisplayLines();
		}

		pData->iSelectedCluster = iCluster;

		FILE *fp = fopen("C:\\RVL\\Debug\\cluster_vertices.txt", "w");

		RECOG::PSGM_::Cluster *pCluster = pRecognition->clusters.Element[iCluster];

		int i, iVertex;
		SURFEL::Vertex *pVertex;

		for (i = 0; i < pCluster->iVertexArray.n; i++)
		{
			iVertex = pCluster->iVertexArray.Element[i];

			pVertex = pSurfels->vertexArray.Element[iVertex];

			fprintf(fp, "%d\t%lf\t%lf\t%lf\n", iVertex, pVertex->P[0], pVertex->P[1], pVertex->P[2]);
		}

		fclose(fp);

		return true;
	}
	else
		return false;
}