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
#include "PSGM.h"
#include <Eigen\Eigenvalues>

using namespace RVL;

PSGM::PSGM()
{
	nDominantClusters = 1;
	kNoise = 1.2f;
	minInitialSurfelSize = 20;
	minVertexPerc = 50;
	kReferenceSurfelSize = 0.2f;
	kReferenceTangentSize = 0.3f;
	baseSeparationAngle = 22.5f;
	edgeTangentAngle = 100.0f;

	convexTemplate.n = 66;
	convexTemplate.Element = new RECOG::PSGM_::Plane[convexTemplate.n];

	CreateTemplate();

	clusters.Element = NULL;
	surfelVertexList.Element = NULL;
	surfelVertexMem = NULL;
	clusterMap = NULL;
	clusterMem = NULL;
	clusterSurfelMem = NULL;
	clusterVertexMem = NULL;
	vertexArray.Element = NULL;
	//modelInstanceMem = NULL;
	vertexDisplayLineArray.Element = NULL;
	vertexDisplayLineArrayMem = NULL;
	sceneFileName = NULL;
}


PSGM::~PSGM()
{
	RVL_DELETE_ARRAY(clusters.Element);
	RVL_DELETE_ARRAY(surfelVertexList.Element);
	RVL_DELETE_ARRAY(surfelVertexMem);
	RVL_DELETE_ARRAY(clusterMap);
	RVL_DELETE_ARRAY(clusterMem);
	RVL_DELETE_ARRAY(clusterSurfelMem);
	RVL_DELETE_ARRAY(clusterVertexMem);
	RVL_DELETE_ARRAY(vertexArray.Element);
	RVL_DELETE_ARRAY(convexTemplate.Element);	
	//RVL_DELETE_ARRAY(modelInstanceMem);
	RVL_DELETE_ARRAY(vertexDisplayLineArray.Element);
	RVL_DELETE_ARRAY(vertexDisplayLineArrayMem);
	RVL_DELETE_ARRAY(sceneFileName);
}

void PSGM::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("PSGM.nDominantClusters", RVLPARAM_TYPE_INT, &nDominantClusters);
	pParamData = ParamList.AddParam("PSGM.kNoise", RVLPARAM_TYPE_FLOAT, &kNoise);
	pParamData = ParamList.AddParam("PSGM.minInitialSurfelSize", RVLPARAM_TYPE_INT, &minInitialSurfelSize);
	pParamData = ParamList.AddParam("PSGM.minVertexPerc", RVLPARAM_TYPE_INT, &minVertexPerc);
	pParamData = ParamList.AddParam("PSGM.kReferenceSurfelSize", RVLPARAM_TYPE_FLOAT, &kReferenceSurfelSize);
	pParamData = ParamList.AddParam("PSGM.kReferenceTangentSize", RVLPARAM_TYPE_FLOAT, &kReferenceTangentSize);
	pParamData = ParamList.AddParam("PSGM.baseSeparationAngle", RVLPARAM_TYPE_FLOAT, &baseSeparationAngle);
	pParamData = ParamList.AddParam("PSGM.edgeTangentAngle", RVLPARAM_TYPE_FLOAT, &edgeTangentAngle);
}

void PSGM::Interpret(
	Mesh *pMesh)
{
	// Create ordered mesh.

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

	QList<RECOG::PSGM_::Vertex> *pVertexList = &vertexList;

	RVLQLIST_INIT(pVertexList);

	int nVertexSurfelRelations = 0;

	int nVertices = 0;

	RVL_DELETE_ARRAY(surfelVertexList.Element);

	surfelVertexList.Element = new QList<QLIST::Index>[pSurfels->NodeArray.n];
	surfelVertexList.n = pSurfels->NodeArray.n;

	float csEdgeTangentAngle = cos(edgeTangentAngle * DEG2RAD);
	float snEdgeTangentAngle = sqrt(1.0f - csEdgeTangentAngle * csEdgeTangentAngle);

	int iSurfel, iSurfel_, iPrevSurfel, iSurfel1, iSurfel2;
	int iBoundary;
	int iPointEdge;
	int iPt, iPt_;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_;
	MeshEdge *pEdge;
	QList<MeshEdgePtr> *pEdgeList;
	RECOG::PSGM_::Vertex *pVertex;
	Point *pPt;
	QList<QLIST::Index> *pSurfelVertexList;
	float *N, *N1, *N2;
	float N2_[3], VTmp[3];
	float fTmp;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		RVLQLIST_INIT(pSurfelVertexList);

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->size <= 1)
			continue;

		N = pSurfel->N;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				pEdgePtr_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr_);

				pPt = pMesh->NodeArray.Element + iPt;

				pEdgeList = &(pPt->EdgeList);

				iPrevSurfel = -1;

				//int debug = 0;

				while (true)
				{			
					if (pPt->bBoundary && pEdgePtr_->pNext == NULL && iPrevSurfel < pMesh->NodeArray.n)
						iSurfel_ = pMesh->NodeArray.n;
					else
					{
						RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr_);

						RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr_, pEdge, iPt_);

						iSurfel_ = pSurfels->surfelMap[iPt_];

						if (iSurfel_ == iSurfel)
							break;
					}

					//debug++;

					//if (debug >= 20)
					//	debug = 0;

					if (iSurfel_ >= 0 && iSurfel_ <= pMesh->NodeArray.n)
					{
						if (iPrevSurfel >= 0 && iPrevSurfel != iSurfel_)
						{
							if (iSurfel < iSurfel_ && iSurfel < iPrevSurfel)
							{
								RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::Vertex, pVertex);

								RVLCOPY3VECTOR(pPt->P, pVertex->P);

								RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, 3, pVertex->iSurfelArray.Element);

								if (iSurfel_ > iPrevSurfel)
								{
									iSurfel1 = iPrevSurfel;
									iSurfel2 = iSurfel_;
								}
								else
								{
									iSurfel1 = iSurfel_;
									iSurfel2 = iPrevSurfel;
								}

								pVertex->iSurfelArray.Element[0] = iSurfel;
								pVertex->iSurfelArray.Element[1] = iSurfel1;
								pVertex->iSurfelArray.Element[2] = iSurfel2;
								pVertex->iSurfelArray.n = 3;

								N1 = pSurfels->NodeArray.Element[iSurfel1].N;

								if (iSurfel2 == pMesh->NodeArray.n)
								{
									RVLSUM3VECTORS(N, N1, VTmp);

									fTmp = csEdgeTangentAngle / sqrt(RVLDOTPRODUCT3(VTmp, VTmp));

									RVLSCALE3VECTOR(VTmp, fTmp, VTmp);

									RVLCROSSPRODUCT3(N, N1, )

									N2 = N2_;
								}
								else
									N2 = pSurfels->NodeArray.Element[iSurfel2].N;

								RVLMEM_ALLOC_STRUCT_ARRAY(pMem, RECOG::PSGM_::NormalHullElement, 3, pVertex->normalHull.Element);
								pVertex->normalHull.n = 0;
								UpdateNormalHull(pVertex->normalHull, N);
								UpdateNormalHull(pVertex->normalHull, N1);
								UpdateNormalHull(pVertex->normalHull, N2);

								RVLQLIST_ADD_ENTRY(pVertexList, pVertex);

								nVertices++;

								nVertexSurfelRelations += 3;
							}
						}
					}	// if (iSurfel_ >= 0 && iSurfel_ <= pMesh->NodeArray.n)		

					iPrevSurfel = iSurfel_;
				}	// for each neighboring point of the point iPt
			}	// for each point-edge on the boundary contour
		}	// for each boundary contour
	}	// for each surfel

	RVL_DELETE_ARRAY(vertexArray.Element);

	vertexArray.Element = new RECOG::PSGM_::Vertex *[nVertices];
	vertexArray.n = nVertices;

	QLIST::CreatePtrArray<RECOG::PSGM_::Vertex>(&vertexList, &vertexArray);

	// Assign vertices to surfels.

	RVL_DELETE_ARRAY(surfelVertexMem);

	surfelVertexMem = new QLIST::Index[nVertexSurfelRelations];

	QLIST::Index *pVertexIdx = surfelVertexMem;

	int iVertex = 0;

	pVertex = vertexList.pFirst;

	while (pVertex)
	{
		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			pSurfelVertexList = surfelVertexList.Element + pVertex->iSurfelArray.Element[iSurfel];

			RVLQLIST_ADD_ENTRY(pSurfelVertexList, pVertexIdx);

			pVertexIdx->Idx = iVertex;

			pVertexIdx++;
		}

		iVertex++;

		pVertex = pVertex->pNext;
	}

	nVertices = iVertex;

	///// Cluster surfels into convex surfaces.

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

	clusterVertexMem = new int[nVertexSurfelRelations];

	int *piVertex = clusterVertexMem;

	bool *bVertexVisited = new bool[vertexArray.n];	
	bool *bVertexInCluster = new bool[vertexArray.n];

	bool *bSurfelVisited = new bool[pSurfels->NodeArray.n];

	QList<QLIST::Index> candidateList;
	QList<QLIST::Index> *pCandidateList = &candidateList;

	QLIST::Index *candidateMem = new QLIST::Index[pSurfels->NodeArray.n];

	Array<int> surfelBuff1, surfelBuff2;

	surfelBuff1.n = pSurfels->NodeArray.n;
	surfelBuff1.Element = new int[pSurfels->NodeArray.n];

	int i;

	for (i = 0; i < pSurfels->NodeArray.n; i++)
		surfelBuff1.Element[i] = i;
	
	surfelBuff2.Element = new int[pSurfels->NodeArray.n];

	Array<int> *pSurfelBuff = &surfelBuff1;
	Array<int> *pSurfelBuff_ = &surfelBuff2;
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
	Surfel *pSurfel_;
	int maxSurfelSize;
	int iLargestSurfel;
	int iFirstNewVertex;
	QLIST::Index *pCandidateIdx, *pBestCandidateIdx;
	QLIST::Index **ppCandidateIdx, **ppBestCandidateIdx;
	float dist, minDist;
	int nSurfelVertices;
	int nSurfelVerticesInCluster;
	int *piVertex_, *piVertex__;

	for (iCluster = 0; iCluster < pSurfels->NodeArray.n; iCluster++)
	{
		// pSurfel <- the largest surfel which is not assigned to a cluster.

		maxSurfelSize = minInitialSurfelSize - 1;

		iLargestSurfel = -1;

		pSurfelBuff_->n = 0;

		for (i = 0; i < pSurfelBuff->n; i++)
		{
			iSurfel = pSurfelBuff->Element[i];

			if (clusterMap[iSurfel] < 0)
			{
				pSurfelBuff_->Element[pSurfelBuff_->n++] = iSurfel;

				pSurfel = pSurfels->NodeArray.Element + iSurfel;

				if (pSurfel->size > maxSurfelSize)
				{
					maxSurfelSize = pSurfel->size;

					iLargestSurfel = iSurfel;
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

		// Initialize a new cluster.

		pCluster = clusterMem + iCluster;

		pCluster->iSurfelArray.Element = piSurfel;
		pCluster->iVertexArray.Element = piVertex;

		pCluster->iSurfelArray.n = 0;
		pCluster->iVertexArray.n = 0;
		pCluster->size = 0;

		clusters.n++;

		memset(bVertexVisited, 0, vertexArray.n * sizeof(bool));
		memset(bVertexInCluster, 0, vertexArray.n * sizeof(bool));
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

			pSurfelVertexList = surfelVertexList.Element + iSurfel;

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

						if (pSurfel_->size > 0)
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
	}	// for each cluster

	delete[] candidateMem;
	delete[] bVertexVisited;
	delete[] bVertexInCluster;
	delete[] bSurfelVisited;
#ifdef RVLPSGM_NORMAL_HULL
	delete[] NHull.Element;
#endif

	// Create sorted cluster array.

	int maxClusterSize = 0;
	int size;

	for (i = 0; i < clusters.n; i++)
	{
		size = clusterMem[i].size;

		if (size > maxClusterSize)
			maxClusterSize = size;
	}

	int maxnBins = 100000;

	int k = (maxClusterSize < maxnBins ? 1 : maxClusterSize / maxnBins + 1);	

	int *key = new int[clusters.n];

	for (i = 0; i < clusters.n; i++)
		key[i] = clusterMem[i].size / k;

	RVL::QuickSort(key, surfelBuff1.Element, clusters.n);

	RVL_DELETE_ARRAY(clusters.Element);

	clusters.Element = new RECOG::PSGM_::Cluster *[clusters.n];

	for (i = 0; i < clusters.n; i++)
	{
		iCluster = surfelBuff1.Element[clusters.n - i - 1];
		clusters.Element[i] = clusterMem + iCluster;
		surfelBuff2.Element[iCluster] = i;
	}		

	// Update cluster map.	

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		if (clusterMap[iSurfel] >= 0)
			clusterMap[iSurfel] = surfelBuff2.Element[clusterMap[iSurfel]];
	}	

	delete[] surfelBuff1.Element;
	delete[] surfelBuff2.Element;
	delete[] key;

	// Fit model.

	int nClusters = RVLMIN(clusters.n, nDominantClusters);

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
		}
	}

	// Save model instances to a file.

	char *PSGModelInstanceFileName = RVLCreateString(sceneFileName);

	sprintf(PSGModelInstanceFileName + strlen(PSGModelInstanceFileName) - 3, "txt");

	FILE *fp = fopen(PSGModelInstanceFileName, "w");

	for (iCluster = 0; iCluster < nClusters; iCluster++)
		SaveModelInstances(fp, iCluster);

	fclose(fp);

	delete[] PSGModelInstanceFileName;
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
	bool bDefined;
	float d;
	RECOG::PSGM_::Vertex *pVertex;
	int i;
	float *N;
	float N_[3];
	float dist;

	for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
	{
		//if (iModelInstanceElement == 32)
		//	int debug = 0;

		pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;
		pModelInstanceElement->defined = false;

		N = convexTemplate.Element[iModelInstanceElement].N;

		RVLMULMX3X3VECT(R, N, N_);

		for (i = 0; i < pCluster->iVertexArray.n; i++)
		{
			pVertex = vertexArray.Element[pCluster->iVertexArray.Element[i]];

			dist = DistanceFromNormalHull(pVertex->normalHull, N_);

			if (dist <= 0.0f)
			{
				d = RVLDOTPRODUCT3(N_, pVertex->P);

				if (pModelInstanceElement->defined)
				{
					if (d > pModelInstanceElement->d)
						pModelInstanceElement->d = d;
				}
				else
				{
					pModelInstanceElement->d = d;
					pModelInstanceElement->defined = true;
				}
			}
		}

		if (!pModelInstanceElement->defined)
		{
			bDefined = false;

			for (i = 0; i < pCluster->iVertexArray.n; i++)
			{
				pVertex = vertexArray.Element[pCluster->iVertexArray.Element[i]];				

				d = RVLDOTPRODUCT3(N_, pVertex->P);

				if (bDefined)
				{
					if (d > pModelInstanceElement->d)
						pModelInstanceElement->d = d;
				}
				else
				{
					pModelInstanceElement->d = d;
					bDefined = true;
				}
			}
		}
	}
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

	tangentArray.Element = new RECOG::PSGM_::Tangent[pCluster->iSurfelArray.n];

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
	
	iSortedTangentArray.Element = new SortIndex<float>[pCluster->iSurfelArray.n];

	Array<QList<QLIST::Index>> iTangentAngleArray;

	iTangentAngleArray.n = (int)round(360.0f / baseSeparationAngle);
	iTangentAngleArray.Element = new QList<QLIST::Index>[iTangentAngleArray.n];
	QLIST::Index *iTangentAngleMem = new QLIST::Index[pCluster->iSurfelArray.n];

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

			RVLCOPY3VECTOR(pSurfel->N, tangentRGData.planeA.N);
			tangentRGData.planeA.d = pSurfel->d;
			tangentArray.n = 0;
			//normalHull.n = 0;

			piSurfelBuffEnd = RegionGrowing<SurfelGraph, Surfel, SURFEL::Edge, SURFEL::EdgePtr, RECOG::PSGM_::TangentRegionGrowingData, RECOG::PSGM_::ValidTangent>
				(pSurfels, &tangentRGData, piSurfelFetch, piSurfelPut);

			for (piSurfel = iSurfelBuff; piSurfel < piSurfelBuffEnd; piSurfel++)
				tangentRGData.bParent[*piSurfel] = false;

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

						P1 = vertexArray.Element[pTangent->iVertex[0]]->P;
						P2 = vertexArray.Element[pTangent->iVertex[1]]->P;

						RVLSUM3VECTORS(P1, P2, P);

						RVLSCALE3VECTOR(P, 0.5f, P);

						d = RVLDOTPRODUCT3(X, P);

						M << pSurfel->N[0], pSurfel->N[1], pSurfel->N[2], pTangent->N[0], pTangent->N[1], pTangent->N[2], X[0], X[1], X[2];

						B << pSurfel->d, pTangent->d, d;

						t_ = M.colPivHouseholderQr().solve(B);

						t = pModelInstance->t;

						RVLCOPY3VECTOR(t_, t);

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

		QList<QLIST::Index> *pVertexList = pRecognition->surfelVertexList.Element + iSurfel;

		bool bMindT = false;

		int nTangentVertices = 0;
		
		pTangent->len = 0.0f;

		float *V = pTangent->V;

		RVLCROSSPRODUCT3(N0, NT, V);

		RVLNORM3(V, fTmp);

		Eigen::Matrix3f M;

		M << N0[0], N0[1], N0[2], NT[0], NT[1], NT[2], V[0], V[1], V[2];

		Eigen::Vector3f B, P_;
		RECOG::PSGM_::Vertex *pVertex;
		int i;
		float mindT, dT, d_, len13, len23;
		float P1[3], P2[3], dP13[3], dP23[3], PProj[3], dP[3];
		float *P;

		QLIST::Index *pVertexIdx = pVertexList->pFirst;

		while (pVertexIdx)
		{
			pVertex = pRecognition->vertexArray.Element[pVertexIdx->Idx];

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

		if (cs < pData->baseSeparationAngle)
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

	RECOG::PSGM_::Vertex *pVertex = vertexArray.Element[iVertex];

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
	RECOG::PSGM_::Vertex *pVertex;

	for (iVertex_ = 0; iVertex_ < pCluster->iVertexArray.n; iVertex_++)
	{
		pVertex = vertexArray.Element[pCluster->iVertexArray.Element[iVertex_]];

		e = RVLDOTPRODUCT3(pSurfel->N, pVertex->P) - pSurfel->d;

		if (e > maxe)
		//if (e < -maxe)
			return false;
	}

	return true;
}

void PSGM::UpdateNormalHull(
	Array<RECOG::PSGM_::NormalHullElement> &NHull,
	float *N)
{
	float *N_;
	float *Nh_;
	float fTmp;

	if (NHull.n == 0)
	{
		N_ = NHull.Element[0].N;

		RVLCOPY3VECTOR(N, N_);

		NHull.n = 1;

		return;
	}
	else if (NHull.n == 1)
	{
		N_ = NHull.Element[0].N;
		Nh_ = NHull.Element[0].Nh;

		RVLCROSSPRODUCT3(N, N_, Nh_);

		fTmp = sqrt(RVLDOTPRODUCT3(Nh_, Nh_));

		if (RVLABS(fTmp) < 1e-10)
			return;

		RVLSCALE3VECTOR2(Nh_, fTmp, Nh_);

		N_ = NHull.Element[1].N;
		float *Nh__ = NHull.Element[1].Nh;

		RVLCOPY3VECTOR(N, N_);

		RVLNEGVECT3(Nh_, Nh__);

		NHull.n = 2;

		return;
	}

	RECOG::PSGM_::NormalHullElement *pHullElement = NHull.Element + NHull.n - 1;

	N_ = pHullElement->N;
	Nh_ = pHullElement->Nh;

	bool bPrevIn = (RVLDOTPRODUCT3(Nh_, N) <= 0.0f);

	int iStart = -1;

	int iEnd;
	int i;	
	bool bIn;

	for (i = 0; i < NHull.n; i++)
	{
		pHullElement = NHull.Element + i;

		N_ = pHullElement->N;
		Nh_ = pHullElement->Nh;

		bIn = (RVLDOTPRODUCT3(Nh_, N) <= 0.0f);

		if (bIn)
		{
			if (!bPrevIn)
				iEnd = i;
		}
		else if (bPrevIn)
			iStart = i;

		bPrevIn = bIn;
	}

	if (iStart < 0)
		return;

	pHullElement = NHull.Element + iStart;

	N_ = pHullElement->N;
	Nh_ = pHullElement->Nh;

	float Nh[3];

	RVLCROSSPRODUCT3(N, N_, Nh);

	fTmp = sqrt(RVLDOTPRODUCT3(Nh, Nh));

	if (RVLABS(fTmp) < 1e-10)
		return;

	RVLSCALE3VECTOR2(Nh, fTmp, Nh_);

	pHullElement = NHull.Element + iEnd;

	N_ = pHullElement->N;

	RVLCROSSPRODUCT3(N_, N, Nh);

	fTmp = sqrt(RVLDOTPRODUCT3(Nh, Nh));

	if (RVLABS(fTmp) < 1e-10)
		return;

	if (iEnd == (iStart + 1) % NHull.n)	// Size of NHull should be increased.
	{
		if (iEnd > 0)
		{
			memmove(NHull.Element + iEnd + 1, NHull.Element + iEnd, (NHull.n - iEnd) * sizeof(RECOG::PSGM_::NormalHullElement));

			iEnd++;
		}

		NHull.n++;
	}
	else if (iEnd > (iStart + 2) % NHull.n)	// Size of NHull should be decreased.
	{
		if (iEnd > iStart)
		{
			memmove(NHull.Element + iStart + 2, NHull.Element + iEnd, (NHull.n - iEnd - 1) * sizeof(RECOG::PSGM_::NormalHullElement));

			NHull.n -= (iEnd - iStart - 2);
		}
		else 
		{
			if (iEnd > 0)
			{
				memmove(NHull.Element, NHull.Element + iEnd, (iStart - iEnd) * sizeof(RECOG::PSGM_::NormalHullElement));

				iStart -= iEnd;
			}

			NHull.n = iStart + 2;			
		}
	}

	pHullElement = NHull.Element + (iStart + 1) % NHull.n;

	N_ = pHullElement->N;
	Nh_ = pHullElement->Nh;

	RVLCOPY3VECTOR(N, N_);
	RVLSCALE3VECTOR2(Nh, fTmp, Nh_);
}

float PSGM::DistanceFromNormalHull(
	Array<RECOG::PSGM_::NormalHullElement> &NHull,
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
	int iCluster)
{
	RECOG::PSGM_::Cluster *pCluster = clusters.Element[iCluster];

	int iModelInstanceElement;
	RECOG::PSGM_::ModelInstanceElement *pModelInstanceElement;

	RECOG::PSGM_::ModelInstance *pModelInstance = pCluster->modelInstanceList.pFirst;

	while (pModelInstance)
	{
		fprintf(fp, "%d\t", iCluster);

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%f\t", pModelInstanceElement->d);
		}

		for (iModelInstanceElement = 0; iModelInstanceElement < convexTemplate.n; iModelInstanceElement++)
		{
			pModelInstanceElement = pModelInstance->modelInstance.Element + iModelInstanceElement;

			fprintf(fp, "%d\t", (int)(pModelInstanceElement->defined));
		}

		fprintf(fp, "\n");

		pModelInstance = pModelInstance->pNext;
	}
}

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

	displayData.bClusters = true;
	displayData.bVertices = false;

	//pSurfels->Display(pVisualizer, pMesh);

	DisplayVertices();

	if (!displayData.bVertices)
		displayData.vertices->VisibilityOff();

	DisplayReferenceFrames();
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

void PSGM::DisplayVertices()
{
	double lineLength = 10.0;

	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	// Create the polydata where we will store all the geometric data
	linesPolyData =	vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iLine = 0;

	double P0[3], P[3], V[3];
	Surfel *pSurfel;
	int iSurfel;

	RECOG::PSGM_::Vertex *pVertex = vertexList.pFirst;

	while (pVertex)
	{
		RVLCOPY3VECTOR(pVertex->P, P0);

		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			pSurfel = pSurfels->NodeArray.Element + pVertex->iSurfelArray.Element[iSurfel];

			pts->InsertNextPoint(P0);

			RVLSCALE3VECTOR(pSurfel->N, lineLength, V);

			RVLSUM3VECTORS(P0, V, P);

			pts->InsertNextPoint(P);

			iLine++;
		}

		pVertex = pVertex->pNext;
	}

	// Add the points to the polydata container
	linesPolyData->SetPoints(pts);

	// Create lines.

	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	unsigned char red[3] = { 255, 0, 0 };

	int nLines = iLine;

	vtkSmartPointer<vtkLine> *line = new vtkSmartPointer<vtkLine>[nLines];

	RVL_DELETE_ARRAY(vertexDisplayLineArray.Element);

	vertexDisplayLineArray.Element = new Array<int>[vertexArray.n];

	RVL_DELETE_ARRAY(vertexDisplayLineArrayMem);

	vertexDisplayLineArrayMem = new int[nLines];

	int *pVertexDisplayLineIdx = vertexDisplayLineArrayMem;

	iLine = 0;

	int iVertex;

	for (iVertex = 0; iVertex < vertexArray.n; iVertex++)
	{
		pVertex = vertexArray.Element[iVertex];

		vertexDisplayLineArray.Element[iVertex].n = pVertex->iSurfelArray.n;

		vertexDisplayLineArray.Element[iVertex].Element = pVertexDisplayLineIdx;

		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			line[iLine] = vtkSmartPointer<vtkLine>::New();

			line[iLine]->GetPointIds()->SetId(0, 2 * iLine);
			line[iLine]->GetPointIds()->SetId(1, 2 * iLine + 1);

			lines->InsertNextCell(line[iLine]);

			colors->InsertNextTupleValue(red);

			*(pVertexDisplayLineIdx++) = iLine;

			iLine++;
		}
	}

	// Add the lines to the polydata container
	linesPolyData->SetLines(lines);

	// Color the lines.
	// SetScalars() automatically associates the values in the data array passed as parameter
	// to the elements in the same indices of the cell data array on which it is called.
	// This means the first component (red) of the colors array
	// is matched with the first component of the cell array (line 0)
	// and the second component (green) of the colors array
	// is matched with the second component of the cell array (line 1)
	linesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(linesPolyData);

	//vtkSmartPointer<vtkActor> actor =
	//	vtkSmartPointer<vtkActor>::New();
	//actor->SetMapper(mapper);
	displayData.vertices = vtkSmartPointer<vtkActor>::New();
	displayData.vertices->SetMapper(mapper);

	pVisualizer->renderer->AddActor(displayData.vertices);

	delete[] line;
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

	int iVertex;
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(linesPolyData->GetCellData()->GetScalars());

	int i, j;

	for (i = 0; i < pCluster->iVertexArray.n; i++)
	{
		iVertex = pCluster->iVertexArray.Element[i];

		for (j = 0; j < vertexDisplayLineArray.Element[iVertex].n; j++)
			rgbPointData->SetTupleValue(vertexDisplayLineArray.Element[iVertex].Element[j], color);
	}	
}

void PSGM::UpdateVertexDisplayLines()
{
	linesPolyData->Modified();
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

			if (pData->bVertices)
			{
				if (pData->iSelectedCluster >= 0)
				{
					unsigned char color[3];

					RVLSET3VECTOR(color, 255, 0, 0);

					pRecognition->PaintClusterVertices(pData->iSelectedCluster, color);

					pRecognition->UpdateVertexDisplayLines();
				}
			}

			pData->iSelectedCluster = -1;
		}
			
		return true;
	}
	else if (key == "v")
	{
		pData->bVertices = !pData->bVertices;

		if (pData->bVertices)
			pData->vertices->VisibilityOn();
		else
			pData->vertices->VisibilityOff();

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

		if (pData->bVertices)
		{
			RVLSET3VECTOR(color, 255, 0, 0);

			pRecognition->PaintClusterVertices(pData->iSelectedCluster, color);
		}
	}

	int iCluster = pRecognition->clusterMap[iSelectedSurfel];

	if (iCluster >= 0)
	{
		pRecognition->PaintCluster(iCluster, pData->selectionColor);

		if (pData->bVertices)
		{
			RVLSET3VECTOR(color, 255, 255, 0);

			pRecognition->PaintClusterVertices(iCluster, color);

			pRecognition->UpdateVertexDisplayLines();
		}

		pData->iSelectedCluster = iCluster;

		return true;
	}
	else
		return false;
}