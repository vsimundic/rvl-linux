#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
// #include "ReconstructionEval.h"
#include "SurfelGraph.h"
// #include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "RVLBuffer.h"
#include "RVLPtrChain.h"
#include "RVLMPtrChain.h"
#include "Rect.h"
#include "RVLEDT.h"
#include "MSTree.h"
#include "BranchMatcher.h"

namespace RVL
{
	void RECOG::BM::MouseRButtonDown(vtkIdType closestPointId, double *closestPoint, void *callData)
	{
		RECOG::BM::DisplayCallbackData *pData = (RECOG::BM::DisplayCallbackData *)callData;

		printf("particle radius=%f\n", pData->particles.Element[closestPointId].r);
	}
}

using namespace RVL;

BranchMatcher::BranchMatcher()
{
	kappa = 1.0f;
	lambda = 2.0f;
	maxParticleDist = 0.2f; // m
	tau_dst_rel = 0.5f;
	tau_dst_abs = 0.005f; // m
	tau_ang_deg = 30.0f;  // deg
	tau_r_rel = 0.3f;
	tau_r_abs = 0.005f; // m
	maxDist = 1.0;		// m
	maxRotDeg = 45.0f;	// deg
	pVisualizationData = NULL;
}

BranchMatcher::~BranchMatcher()
{
	Clear();
}

void BranchMatcher::Create(char *cfgFileNameIn)
{
	cfgFileName = cfgFileNameIn;
	CreateParamList();
	paramList.LoadParams(cfgFileNameIn);

	// VTKPointPickerDemo();
}

void BranchMatcher::Clear()
{
	if (pVisualizationData)
	{
		if (pVisualizationData->bOwnVisualizer)
			delete pVisualizationData->pVisualizer;
	}
}

void BranchMatcher::CreateParamList()
{
	paramList.m_pMem = pMem0;
	RVLPARAM_DATA *pParamData;
	paramList.Init();
	pParamData = paramList.AddParam("BM.visualize", RVLPARAM_TYPE_BOOL, &bVisualize);
	pParamData = paramList.AddParam("BM.tau_dst_rel", RVLPARAM_TYPE_FLOAT, &tau_dst_rel);
	pParamData = paramList.AddParam("BM.tau_dst_abs", RVLPARAM_TYPE_FLOAT, &tau_dst_abs);
	pParamData = paramList.AddParam("BM.tau_ang_deg", RVLPARAM_TYPE_FLOAT, &tau_ang_deg);
	pParamData = paramList.AddParam("BM.tau_r_rel", RVLPARAM_TYPE_FLOAT, &tau_r_rel);
	pParamData = paramList.AddParam("BM.tau_r_abs", RVLPARAM_TYPE_FLOAT, &tau_r_abs);
	pParamData = paramList.AddParam("BM.lambda", RVLPARAM_TYPE_FLOAT, &lambda);
	pParamData = paramList.AddParam("BM.maxDist", RVLPARAM_TYPE_FLOAT, &maxDist);
	pParamData = paramList.AddParam("BM.maxRotDeg", RVLPARAM_TYPE_FLOAT, &maxRotDeg);
}

void BranchMatcher::Particles(
	Array2D<short int> depthImage,
	Array<RECOG::BM::Particle> &particles,
	bool bVisualize)
{
	// Parameters.

	int depthDiscontinuityThr = 20; // mm
	// float particleDistResolution = 0.001f;

	// Constants.

	int nPix = depthImage.w * depthImage.h;

	// Depth discontinuity map.

	bool *bDepthDiscontinuity = new bool[nPix];
	memset(bDepthDiscontinuity, 0, nPix * sizeof(bool));
	int neighborhood4[4];
	neighborhood4[0] = 1;
	neighborhood4[1] = -depthImage.w;
	neighborhood4[2] = -1;
	neighborhood4[3] = depthImage.w;
	int iPix, iPix_;
	int u, v;
	int iNeighbor;
	ushort d, d_;
	int dd;
	uchar *visPix;
	for (v = 1; v < depthImage.h - 1; v++)
		for (u = 1; u < depthImage.w - 1; u++)
		{
			iPix = u + v * depthImage.w;
			d = depthImage.Element[iPix];
			if (d == 0)
				continue;
			for (iNeighbor = 0; iNeighbor < 4; iNeighbor++)
			{
				iPix_ = iPix + neighborhood4[iNeighbor];
				d_ = depthImage.Element[iPix_];
				if (d_ == 0)
				{
					bDepthDiscontinuity[iPix] = true;
					break;
				}
				dd = d_ - d;
				if (dd > depthDiscontinuityThr)
				{
					bDepthDiscontinuity[iPix] = true;
					break;
				}
			}
		}

	// Depth discontinuity map visualization.

	// cv::Mat depthDiscontinuityDisplay;
	// depthDiscontinuityDisplay.create(depthImage.h, depthImage.w, CV_8UC1);
	// visPix = depthDiscontinuityDisplay.data;
	// for (iPix = 0; iPix < nPix; iPix++, visPix++)
	//	*visPix = (bDepthDiscontinuity[iPix] ? 255 : 0);
	// cv::imshow("depth discontinuity map", depthDiscontinuityDisplay);
	// cv::waitKey();

	// EDT.

	CRVLEDT EDT;
	EDT.m_Flags |= RVLEDT_FLAG_EDGE_CONTOURS;
	RVLEDT_PIX_ARRAY EDTImage;
	EDTImage.Width = depthImage.w;
	EDTImage.Height = depthImage.h;
	EDTImage.pPix = new RVLEDT_PIX[nPix];
	RVLEDT_BUCKET_ENTRY *EDTBucketMem = new RVLEDT_BUCKET_ENTRY[4 * nPix];
	WORD iBucket;
	for (iBucket = 0; iBucket < 4; iBucket++)
		EDT.m_BucketPtrArray[iBucket] = EDTBucketMem + iBucket * nPix;
	EDT.Border(&EDTImage);
	EDT.CreateRTSqrtLUT();
	CRVLBuffer EDTBuff;
	EDTBuff.DataBuff = new void *[2 * nPix];
	EDTBuff.m_bOwnData = FALSE;
	EDT.m_maxd2 = depthImage.w * depthImage.w + depthImage.h * depthImage.h;
	CRVLMem mem;
	mem.Create(nPix * sizeof(RVLPTRCHAIN_ELEMENT));
	CRVLMPtrChain boundary;
	boundary.m_pMem = &mem;
	RVLEDT_PIX *pEDTPix;
	for (v = 1; v < depthImage.h - 1; v++)
		for (u = 1; u < depthImage.w - 1; u++)
		{
			iPix = u + v * depthImage.w;
			if (depthImage.Element[iPix] > 0 && bDepthDiscontinuity[iPix])
			{
				EDTImage.pPix[iPix].d2 = 0;
				boundary.Add((void *)(EDTImage.pPix + iPix));
			}
			else if (depthImage.Element[iPix] == 0)
				EDTImage.pPix[iPix].d2 = 0;
		}
	EDT.Apply(&boundary, NULL, &EDTImage, &EDTBuff);
	delete[] EDTBucketMem;
	delete[] EDTBuff.DataBuff;

	// Visualize EDT.

	// cv::Mat EDTDisplayImageHSV(depthImage.h, depthImage.w, CV_8UC3);
	// visPix = EDTDisplayImageHSV.data;
	// pEDTPix = EDTImage.pPix;
	// int maxd2 = 140;
	// for (iPix = 0; iPix < nPix; iPix++, visPix += 3, pEDTPix++)
	//	if (pEDTPix->d2 == 0 && !bDepthDiscontinuity[iPix])
	//	{
	//		RVLNULL3VECTOR(visPix);
	//	}
	//	else
	//	{
	//		visPix[0] = (pEDTPix->d2 <= maxd2 ? pEDTPix->d2+1 : maxd2);
	//		visPix[1] = visPix[2] = 255;
	//	}
	// cv::Mat EDTDisplayImageBGR(depthImage.h, depthImage.w, CV_8UC3);
	// cv::cvtColor(EDTDisplayImageHSV, EDTDisplayImageBGR, CV_HSV2RGB);
	// cv::imshow("EDT", EDTDisplayImageBGR);
	// cv::waitKey();

	// Particles.

	int neighborhood8[8][2];
	neighborhood8[0][0] = 1;
	neighborhood8[0][1] = 0;
	neighborhood8[1][0] = 1;
	neighborhood8[1][1] = -1;
	neighborhood8[2][0] = 0;
	neighborhood8[2][1] = -1;
	neighborhood8[3][0] = -1;
	neighborhood8[3][1] = -1;
	neighborhood8[4][0] = -1;
	neighborhood8[4][1] = 0;
	neighborhood8[5][0] = -1;
	neighborhood8[5][1] = 1;
	neighborhood8[6][0] = 0;
	neighborhood8[6][1] = 1;
	neighborhood8[7][0] = 1;
	neighborhood8[7][1] = 1;
	int neighborhood8_[8];
	float fNeighborhood8[8][2];
	for (iNeighbor = 0; iNeighbor < 8; iNeighbor++)
	{
		neighborhood8_[iNeighbor] = neighborhood8[iNeighbor][0] + neighborhood8[iNeighbor][1] * depthImage.w;
		fNeighborhood8[iNeighbor][0] = (float)(neighborhood8[iNeighbor][0]);
		fNeighborhood8[iNeighbor][1] = (float)(neighborhood8[iNeighbor][1]);
	}
	int *particleMap = new int[nPix];
	memset(particleMap, 0xff, nPix * sizeof(int));
	bool *bNMS = new bool[nPix];
	memset(bNMS, 0, nPix * sizeof(bool));
	RVLEDT_PIX *pEDTPix_;
	int nCorePixels;
	float fnCorePixels;
	float du, dv;
	float cImg[2];
	float rImg;
	float z;
	float fTmp;
	float V3Tmp[3];
	Rect<int> neighborhood;
	Rect<int> imageRect;
	imageRect.minx = 0;
	imageRect.maxx = depthImage.w - 1;
	imageRect.miny = 0;
	imageRect.maxy = depthImage.h - 1;
	int rNeighborhood, rNeighborhood2;
	int u_, v_, du_, dv_;
	RECOG::BM::Particle *particleMem = new RECOG::BM::Particle[nPix];
	RECOG::BM::Particle *pParticle = particleMem;
	for (v = 1; v < depthImage.h - 1; v++)
		for (u = 1; u < depthImage.w - 1; u++)
		{
			iPix = u + v * depthImage.w;
			if (bNMS[iPix])
				continue;
			pEDTPix = EDTImage.pPix + iPix;
			if (pEDTPix->d2 > EDT.m_maxd2)
				continue;
			if (pEDTPix->d2 == 0 && !bDepthDiscontinuity[iPix])
				continue;
			for (iNeighbor = 0; iNeighbor < 8; iNeighbor++)
			{
				pEDTPix_ = EDTImage.pPix + iPix + neighborhood8_[iNeighbor];
				if (pEDTPix_->d2 > EDT.m_maxd2)
					continue;
				if (pEDTPix_->d2 > pEDTPix->d2)
					break;
			}
			if (iNeighbor < 8)
				continue;
			pParticle->bExists = true;
			pParticle->iPix = iPix;
			particleMap[iPix] = pParticle - particleMem;
			cImg[0] = cImg[1] = 0.0f;
			nCorePixels = 1;
			for (iNeighbor = 0; iNeighbor < 8; iNeighbor++)
			{
				pEDTPix_ = EDTImage.pPix + iPix + neighborhood8_[iNeighbor];
				if (pEDTPix_->d2 == pEDTPix->d2)
				{
					cImg[0] += fNeighborhood8[iNeighbor][0];
					cImg[1] += fNeighborhood8[iNeighbor][1];
					nCorePixels++;
				}
			}
			fnCorePixels = (float)nCorePixels;
			cImg[0] /= fnCorePixels;
			cImg[1] /= fnCorePixels;
			du = (float)(pEDTPix->dx) + cImg[0];
			dv = (float)(pEDTPix->dz) + cImg[1];
			rImg = sqrt(du * du + dv * dv) + 0.5f;
			z = 0.001f * (float)(depthImage.Element[iPix]);
			pParticle->r = rImg * z / (camera.fu - rImg);
			pParticle->c[0] = z * ((float)u - camera.uc) / camera.fu;
			pParticle->c[1] = z * ((float)v - camera.vc) / camera.fv;
			pParticle->c[2] = z;
			fTmp = pParticle->r / sqrt(RVLDOTPRODUCT3(pParticle->c, pParticle->c));
			RVLSCALE3VECTOR(pParticle->c, fTmp, V3Tmp);
			RVLSUM3VECTORS(pParticle->c, V3Tmp, pParticle->c);
			pParticle++;
			rNeighborhood = (int)ceil(rImg) + 1;
			rNeighborhood2 = rNeighborhood * rNeighborhood;
			neighborhood.minx = u - rNeighborhood;
			neighborhood.maxx = u + rNeighborhood;
			neighborhood.miny = v - rNeighborhood;
			neighborhood.maxy = v + rNeighborhood;
			CropRect<int>(neighborhood, imageRect);
			for (v_ = neighborhood.miny; v_ <= neighborhood.maxy; v_++)
				for (u_ = neighborhood.minx; u_ <= neighborhood.maxx; u_++)
				{
					du_ = u_ - u;
					dv_ = v_ - v;
					if (du_ * du_ + dv_ * dv_ <= rNeighborhood2)
					{
						iPix_ = u_ + v_ * depthImage.w;
						bNMS[iPix_] = true;
					}
				}
		}
	particles.n = pParticle - particleMem;
	particles.Element = new RECOG::BM::Particle[particles.n];
	memcpy(particles.Element, particleMem, particles.n * sizeof(RECOG::BM::Particle));
	delete[] particleMem;
	delete[] EDTImage.pPix;
	delete[] bDepthDiscontinuity;
	delete[] particleMap;
	delete[] bNMS;
	int iParticle;

	// Visualize particle map.

	if (bVisualize)
	{
		cv::Mat particleMapDisplay;
		particleMapDisplay.create(depthImage.h, depthImage.w, CV_8UC3);
		DisplayDisparityMap(depthImage, (unsigned char *)(particleMapDisplay.data), true, RVLRGB_DEPTH_FORMAT_1MM);
		for (iParticle = 0; iParticle < particles.n; iParticle++)
		{
			pParticle = particles.Element + iParticle;
			visPix = particleMapDisplay.data + 3 * pParticle->iPix;
			RVLSET3VECTOR(visPix, 0, 255, 0);
		}
		cv::imshow("Particle map", particleMapDisplay);
		cv::waitKey();
	}
}

#define RVLBM_MATCH_LOG

void BranchMatcher::Match(
	Array<RECOG::BM::Particle> P,
	Array<RECOG::BM::Particle> Q,
	int *c)
{
	// Parameters.

	float rResolution = 0.001f;	   // m
	float minDistBinSize = 0.005f; // m
	float distBinSizeLog = 1.2f;
	float kMaxDist = 1.5f;

	// Constants.

	float tau_dst_rel2 = tau_dst_rel * tau_dst_rel;
	float tau_dst_abs2 = tau_dst_abs * tau_dst_abs;
	float tau_ang = DEG2RAD * tau_ang_deg;
	float tau_ang2 = tau_ang * tau_ang;
	float tau_r_rel2 = tau_r_rel * tau_r_rel;
	float tau_r_abs2 = tau_r_abs * tau_r_abs;
	float kDistLT = (distBinSizeLog - 1.0f) / minDistBinSize;
	float logDistBinSizeLog = log(distBinSizeLog);
	int nDistBins = (int)ceil(log(kMaxDist * maxDist * kDistLT + 1.0f) / logDistBinSizeLog);
	float csMaxRot = cos(DEG2RAD * maxRotDeg);

	// Log file.

#ifdef RVLBM_MATCH_LOG
	FILE *fpLog = fopen((std::string(resultsFolder) + RVLFILEPATH_SEPARATOR + "bmlog.txt").data(), "w");
	fclose(fpLog);
#endif

	// Edges.

	Array<GRAPH::MSTreeEdge> edges;
	edges.Element = new GRAPH::MSTreeEdge[P.n * P.n];
	edges.n = 0;
	GRAPH::MSTreeEdge *pEdge;
	float V[3];
	int iParticle, iParticle_;
	RECOG::BM::Particle *pParticle, *pParticle_;
	float fTmp;
	for (iParticle = 0; iParticle < P.n; iParticle++)
	{
		pParticle = P.Element + iParticle;
		for (iParticle_ = 0; iParticle_ < iParticle; iParticle_++)
		{
			pParticle_ = P.Element + iParticle_;
			pEdge = edges.Element + edges.n;
			pEdge->iVertex[0] = iParticle;
			pEdge->iVertex[1] = iParticle_;
			RVLDIF3VECTORS(pParticle_->c, pParticle->c, V);
			pEdge->cost = sqrt(RVLDOTPRODUCT3(V, V));
			if (pEdge->cost > maxParticleDist)
				continue;
			fTmp = RVLMIN(pParticle_->r, pParticle->r);
			pEdge->cost /= fTmp;
			edges.n++;
		}
	}

	// Minimum spanning tree.

	MSTree MST;
	MST.Init(P.n);
	MST.Create(edges);
	GRAPH::Node *pMSTNode;
	GRAPH::EdgePtr<GRAPH::Edge> *pMSTEdgePtr;

	// Visualize MST.

	// RVLCOLORS
	// Array<Point> visPts;
	// visPts.Element = new Point[P.n];
	// visPts.n = P.n;
	// for (iParticle = 0; iParticle < P.n; iParticle++)
	//{
	//	pParticle = P.Element + iParticle;
	//	RVLCOPY3VECTOR(pParticle->c, visPts.Element[iParticle].P);
	// }
	// pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(visPts, darkGreen, 6);
	// Array<Pair<int, int>> visEdges;
	// visEdges.Element = new Pair<int, int>[P.n - 1];
	// visEdges.n = 0;

	// Pair<int, int>* pVisEdge;
	// for (iParticle = 0; iParticle < P.n; iParticle++)
	//{
	//	pMSTNode = MST.T.NodeArray.Element + iParticle;
	//	pMSTEdgePtr = pMSTNode->EdgeList.pFirst;
	//	while (pMSTEdgePtr)
	//	{
	//		iParticle_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pMSTEdgePtr);
	//		if (iParticle_ > iParticle)
	//		{
	//			pVisEdge = visEdges.Element + visEdges.n;
	//			pVisEdge->a = iParticle;
	//			pVisEdge->b = iParticle_;
	//			visEdges.n++;
	//		}
	//		pMSTEdgePtr = pMSTEdgePtr->pNext;
	//	}
	// }
	// pVisualizationData->pVisualizer->DisplayLines(visPts, visEdges, blue);
	// pVisualizationData->pVisualizer->Run();
	// delete[] visEdges.Element;
	// delete[] visPts.Element;

	// Determine the root - the particle with the greatest r.

	int iRoot = 0;
	float rRoot = 0.0f;
	pParticle = P.Element;
	for (iParticle = 0; iParticle < P.n; iParticle++, pParticle++)
		if (pParticle->r > rRoot)
		{
			rRoot = pParticle->r;
			iRoot = iParticle;
		}

	// Assign indices to particles in P by a region growing process, which starts with the root,
	// as an initial element of a growing set and growing this set by adding adjacent particles.
	// The set is grown by adding the particle with the greatest r of all particles adjacent to the elements of the set.

	// Assign successors and predecessors to the particles in P.

	int *frontierMem = new int[2 * P.n];
	int *frontierBuff1 = frontierMem;
	int *frontierBuff2 = frontierMem + P.n;
	Array<int> frontier;
	frontier.Element = frontierBuff1;
	frontier.n = 0;
	frontier.Element[frontier.n++] = iRoot;
	Array<int> newFrontier;
	newFrontier.Element = frontierBuff2;
	int i;
	float r;
	float maxrFrontier;
	int iParticleToExpand;
	pParticle = P.Element;
	for (iParticle = 0; iParticle < P.n; iParticle++, pParticle++)
		pParticle->successors.n = 0;
	int *successorMem = new int[P.n];
	int *pNewSuccessorSequence = successorMem;
	bool *bFrontier = new bool[P.n];
	memset(bFrontier, 0, P.n * sizeof(bool));
	int *particleIdx = new int[P.n];
	memset(particleIdx, 0xff, P.n * sizeof(int));
	int particleIdx_ = 0;
	Array<int> idxArrayTmp;
	while (frontier.n > 0)
	{
		maxrFrontier = 0.0f;
		for (i = 0; i < frontier.n; i++)
		{
			iParticle = frontier.Element[i];
			r = P.Element[iParticle].r;
			if (r > maxrFrontier)
			{
				maxrFrontier = r;
				iParticleToExpand = iParticle;
			}
		}
		particleIdx[iParticleToExpand] = particleIdx_++;
		bFrontier[iParticleToExpand] = true;
		newFrontier.n = 0;
		pParticle = P.Element + iParticleToExpand;
		pParticle->successors.Element = pNewSuccessorSequence;
		pMSTNode = MST.T.NodeArray.Element + iParticleToExpand;
		pMSTEdgePtr = pMSTNode->EdgeList.pFirst;
		while (pMSTEdgePtr)
		{
			iParticle_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pMSTEdgePtr);
			if (particleIdx[iParticle_] < 0)
			{
				newFrontier.Element[newFrontier.n++] = iParticle_;
				bFrontier[iParticle_] = true;
				pParticle->successors.Element[pParticle->successors.n++] = iParticle_;
				P.Element[iParticle_].iPredecessor = iParticleToExpand;
			}
			pMSTEdgePtr = pMSTEdgePtr->pNext;
		}
		pNewSuccessorSequence += pParticle->successors.n;
		for (i = 0; i < frontier.n; i++)
		{
			iParticle = frontier.Element[i];
			if (!bFrontier[iParticle])
				newFrontier.Element[newFrontier.n++] = iParticle;
		}
		bFrontier[iParticleToExpand] = false;
		for (i = 0; i < newFrontier.n; i++)
			bFrontier[newFrontier.Element[i]] = false;
		idxArrayTmp = frontier;
		frontier = newFrontier;
		newFrontier = idxArrayTmp;
	}
	delete[] frontierMem;
	delete[] bFrontier;

	// Order the particles in P according to the particle index.

	RECOG::BM::Particle *PUnsorted = new RECOG::BM::Particle[P.n];
	memcpy(PUnsorted, P.Element, P.n * sizeof(RECOG::BM::Particle));
	for (iParticle = 0; iParticle < P.n; iParticle++)
		if (particleIdx[iParticle] >= 0)
			P.Element[particleIdx[iParticle]] = PUnsorted[iParticle];
	delete[] PUnsorted;
	P.n = particleIdx_;
	int nSuccessorsTotal = pNewSuccessorSequence - successorMem;
	for (i = 0; i < nSuccessorsTotal; i++)
		successorMem[i] = particleIdx[successorMem[i]];
	pParticle = P.Element + 1;
	for (iParticle = 1; iParticle < P.n; iParticle++, pParticle++)
		pParticle->iPredecessor = particleIdx[pParticle->iPredecessor];
	delete[] particleIdx;

	/// Visualize tree and particles Q.

	Visualizer *pVisualizer;
	if (bVisualize)
	{
		pVisualizer = pVisualizationData->pVisualizer;
		Visualize(P, Q);
		pVisualizer->Run();
		pVisualizationData->assocActors.clear();

		// Incremental tree Visualization.

		//{
		//	pVisualizer->Clear();
		//	RVLCOLORS
		//	Point visPt[2];
		//	pParticle = P.Element;
		//	for (iParticle = 0; iParticle < P.n; iParticle++, pParticle++)
		//	{
		//		if (pParticle->iPredecessor < 0)
		//			continue;
		//		RVLCOPY3VECTOR(pParticle->c, visPt[0].P);
		//		RVLCOPY3VECTOR(P.Element[pParticle->iPredecessor].c, visPt[1].P);
		//		pVisualizer->DisplayLine(visPt, red);
		//		pVisualizer->Run();
		//	}
		//}
	}

	///

	// Vectors and distances between particles P.

	Vector3<float> *VP = new Vector3<float>[P.n * P.n];
	float *distP = new float[P.n * P.n];
	float *V_, *V__;
	float dist;
	int iPair;
	for (iParticle = 0; iParticle < P.n; iParticle++)
	{
		pParticle = P.Element + iParticle;
		for (iParticle_ = 0; iParticle_ < iParticle; iParticle_++)
		{
			pParticle_ = P.Element + iParticle_;
			iPair = iParticle * P.n + iParticle_;
			V_ = VP[iPair].Element;
			RVLDIF3VECTORS(pParticle_->c, pParticle->c, V_);
			dist = sqrt(RVLDOTPRODUCT3(V_, V_));
			distP[iPair] = dist;
			RVLSCALE3VECTOR2(V_, dist, V_);
			iPair = iParticle_ * P.n + iParticle;
			V__ = VP[iPair].Element;
			RVLNEGVECT3(V_, V__);
			distP[iPair] = dist;
		}
	}

	// Vectors between particles Q.
	// Distance lookup table.

	Vector3<float> *VQ = new Vector3<float>[Q.n * Q.n];
	float *distQ = new float[Q.n * Q.n];
	int nBinsTotal = Q.n * nDistBins;
	QList<QLIST::Index> *distLT = new QList<QLIST::Index>[nBinsTotal];
	QList<QLIST::Index> *pDistBin = distLT;
	int iBin;
	for (iBin = 0; iBin < nBinsTotal; iBin++, pDistBin++)
		RVLQLIST_INIT(pDistBin);
	QLIST::Index *distLTMem = new QLIST::Index[Q.n * Q.n];
	QLIST::Index *pDistLTEntry = distLTMem;
	for (iParticle = 0; iParticle < Q.n; iParticle++)
	{
		pParticle = Q.Element + iParticle;
		if (RVLABS(pParticle->c[1]) > 1e6 || RVLABS(pParticle->c[2]) > 1e6)
			int debug = 0;
		for (iParticle_ = 0; iParticle_ < Q.n; iParticle_++)
		{
			pParticle_ = Q.Element + iParticle_;
			iPair = iParticle * Q.n + iParticle_;
			V_ = VQ[iPair].Element;
			RVLDIF3VECTORS(pParticle_->c, pParticle->c, V_);
			dist = sqrt(RVLDOTPRODUCT3(V_, V_));
			distQ[iPair] = dist;
			RVLSCALE3VECTOR2(V_, dist, V_);
			iBin = (int)floor(log(kDistLT * dist + 1.0f) / logDistBinSizeLog);
			if (iBin >= nDistBins)
				continue;
			pDistBin = distLT + iParticle * nDistBins + iBin;
			RVLQLIST_ADD_ENTRY(pDistBin, pDistLTEntry);
			pDistLTEntry->Idx = iParticle_;
			pDistLTEntry++;
		}
	}

	// Particles of similar size.

	int *similarSizeMem = new int[P.n * Q.n];
	Array<int> *similarSizeParticles = new Array<int>[P.n];
	Array<int> *pSimilarSizeParticles = similarSizeParticles;
	float e_r;
	float rRange;
	for (iParticle = 0; iParticle < P.n; iParticle++, pSimilarSizeParticles++)
	{
		pParticle = P.Element + iParticle;
		pSimilarSizeParticles->Element = similarSizeMem + iParticle * Q.n;
		pSimilarSizeParticles->n = 0;
		rRange = tau_r_rel * pParticle->r;
		if (rRange < tau_r_abs)
			rRange = tau_r_abs;
		for (iParticle_ = 0; iParticle_ < Q.n; iParticle_++)
		{
			pParticle_ = Q.Element + iParticle_;
			e_r = pParticle_->r - pParticle->r;
			if (RVLABS(e_r) <= rRange)
				pSimilarSizeParticles->Element[pSimilarSizeParticles->n++] = iParticle_;
		}
	}

	///// Match Q with partcles_P.

	c = new int[P.n];

	std::vector<RECOG::BM::Assoc> A;
	A.reserve(100000);
	idxMem.clear();
	idxMem.reserve(100000);
	arrayPtrMem.clear();
	arrayPtrMem.reserve(100000);

	// Add the root association to A.

	RECOG::BM::Assoc a;
	a.p = a.q = a.iPredecessor = -1;
	a.C = 0.0f;
	a.bExpanded = false;
	A.push_back(a);

	//

	int iAssoc, iAssoc_, iAssoc__, iAssoc___;
	iAssoc = 0;
	Array<int> successors_;
	Array<int> assocRootSuccessors;
	assocRootSuccessors.n = 1;
	iRoot = 0;
	assocRootSuccessors.Element = &iRoot;
	int j, k;
	int pi, pj, pk, pl, qv, qw, qz;
	RECOG::BM::Assoc *pAssoc, *pAssoc_, *pAssoc__, *pAssoc___;
	float C_dst, C_ang;
	float C_dst2, C_ang2;
	float C_r, C_r2;
	float rj, rk, rhv;
	float distRange;
	Array<int> inRange;
	int *inRangeMem = new int[Q.n];
	float *VP_, *VQ_, *VP__, *VQ__;
	float angVP, angVQ;
	float e_ang;
	ArrayD<int> D;
	ArrayD<ArrayD<int>> DD;
	ArrayD<int> M;
	M.pStorage = &idxMem;
	float minC = 0.0f;
	int iBestSuccessor;
	int minpj = 0;
	int iFirstBin, iLastBin;
	int alpha;
	RECOG::BM::Assoc *pAssocAlpha;
	RECOG::BM::Assoc *pAssocNu;
	int minpk = 0;
	int nu;
	float L;
#ifdef RVLBM_MATCH_LOG
	int maxp = -1;
	float C;
	Array<int> optimalAssociations;
	optimalAssociations.Element = new int[P.n + 1];
#endif
	while (iAssoc >= 0)
	{
		if (iAssoc == 106)
			int debug = 0;

		//// Expand association iAssoc.

		// pAssoc == a
		// pAssoc_ == a'
		// pAssoc__ == a''
		// pAssoc___ == a'''

		pAssoc = A.data() + iAssoc;
		pi = pAssoc->p;

#ifdef RVLBM_MATCH_LOG
		fpLog = fopen((std::string(resultsFolder) + RVLFILEPATH_SEPARATOR + "bmlog.txt").data(), "a");
		OptimalAssociations(P, A, c, &optimalAssociations);
		C = 0.0f;
		for (int i = 0; i < optimalAssociations.n; i++)
		{
			alpha = optimalAssociations.Element[i];
			pAssocAlpha = A.data() + alpha;
			fprintf(fpLog, "a%d(%d,%d): C=%f\n", alpha, pAssocAlpha->p, pAssocAlpha->q, pAssocAlpha->C);
			C += pAssocAlpha->C;
		}
		fprintf(fpLog, "C=%f\n", C);
		fprintf(fpLog, "Expanding a%d(%d,%d)\n", iAssoc, pi, pAssoc->q);
		fclose(fpLog);
		fpLog = fopen((std::string(resultsFolder) + RVLFILEPATH_SEPARATOR + "bmlog.txt").data(), "a");
#endif

		// Visualization.

		if (bVisualize)
		{
			if (pi > maxp)
			{
				maxp = pi;
				printf("maxp=%d\n", maxp);
				Visualize(P, Q, A);
				pVisualizer->Run();
				pVisualizer->Clear(pVisualizationData->assocActors);
				pVisualizationData->assocActors.clear();
			}
		}

		//

		// if (pi == 10)
		//	int debug = 0;
		successors_ = (pAssoc->p >= 0 ? P.Element[pi].successors : assocRootSuccessors);

		// La <- 0

		L = 0.0f;

		// Initialize DD.

		RVLARRAYD_INIT(DD, arrayPtrMem);
		pAssoc->DDFirstIdx = DD.iFirst;

		//

		for (j = 0; j < successors_.n; j++)
		{
			pj = successors_.Element[j];

			/// Determine the set K(a, j) and process the associations which belong to this set.

			RVLARRAYD_INIT(D, idxMem);

			// If pAssoc->q != None, then pAssoc__ <- pAssoc, else pAssoc__ <- f(pAssoc).

			pAssoc__ = A.data() + iAssoc;
			while (pAssoc__->q < 0 && pAssoc__->iPredecessor >= 0)
				pAssoc__ = A.data() + pAssoc__->iPredecessor;
			qw = pAssoc__->q;

			//

			if (qw >= 0)
			{
				pk = pAssoc__->p;
				VP_ = VP[pk * P.n + pj].Element;

				// pAssoc___ <- f(pAssoc__)

				if (pAssoc__->iPredecessor >= 0)
				{
					pAssoc___ = A.data() + pAssoc__->iPredecessor;
					while (pAssoc___->q < 0 && pAssoc___->iPredecessor >= 0)
						pAssoc___ = A.data() + pAssoc___->iPredecessor;
					qz = pAssoc___->q;
				}
				else
					qz = -1;

				// angVP <- angle between vectors pk-pl and pj-pk

				if (qz >= 0)
				{
					pl = pAssoc___->p;
					if (qz >= 0)
					{
						VP__ = VP[pl * P.n + pk].Element;
						angVP = acos(RVLDOTPRODUCT3(VP_, VP__));
						VQ__ = VQ[qz * Q.n + qw].Element;
					}
				}
			}

			rj = P.Element[pj].r;

			if (qw >= 0)
			{
				// r <- max(rj, rk)

				rk = P.Element[pk].r;
				r = RVLMAX(rj, rk);

				//

				dist = distP[pk * P.n + pj];

				// Compute distRange.

				distRange = tau_dst_rel * r;
				if (distRange < tau_dst_abs)
					distRange = tau_dst_abs;

				// Compute rRange.

				rRange = tau_r_rel * rj;
				if (rRange < tau_r_abs)
					rRange = tau_r_abs;

				// inRange <- set of all particles in Q within the range [dist - distRange, dist + distRange] from qw
				// and with radius within the range [rj - rRange, rj + rRange]

				iFirstBin = (int)floor(log(kDistLT * (dist - distRange) + 1.0f) / logDistBinSizeLog);
				iLastBin = (int)floor(log(kDistLT * (dist + distRange) + 1.0f) / logDistBinSizeLog);
				if (iLastBin >= nDistBins)
					iLastBin = nDistBins - 1;
				inRange.n = 0;
				inRange.Element = inRangeMem;
				for (iBin = iFirstBin; iBin <= iLastBin; iBin++)
				{
					pDistBin = distLT + qw * nDistBins + iBin;
					pDistLTEntry = pDistBin->pFirst;
					while (pDistLTEntry)
					{
						qv = pDistLTEntry->Idx;
						e_r = rj - Q.Element[qv].r;
						if (RVLABS(e_r) <= rRange)
						{
							VQ_ = VQ[qw * Q.n + qv].Element;
							if (RVLDOTPRODUCT3(VP_, VQ_) > csMaxRot)
								inRange.Element[inRange.n++] = qv;
						}
						pDistLTEntry = pDistLTEntry->pNext;
					}
				}
			}
			else
			{
				// inRange <- set of all particles in Q with radius within the range [rj - rRange, rj + rRange]

				inRange = similarSizeParticles[pj];
			}

			for (i = 0; i < inRange.n; i++)
			{
				qv = inRange.Element[i];
				rhv = Q.Element[qv].r;
				e_r = rj - rhv;
				C_r = e_r;
				// C_r /= rj;
				C_r /= sqrt(rj);
				C_r2 = C_r * C_r;

				C_dst2 = C_ang2 = 0.0f;
				if (qw >= 0)
				{
					// C_dst = rj * (1.0f - distQ[qw * Q.n + qv] / distP[pk * P.n + pj]);
					C_dst = sqrt(rj) * (1.0f - distQ[qw * Q.n + qv] / distP[pk * P.n + pj]);
					// C_dst /= r;
					C_dst2 = C_dst * C_dst;

					if (qz >= 0)
					{
						// angVQ <- angle between vectors qw-qz and qv-qw

						VQ_ = VQ[qw * Q.n + qv].Element;
						fTmp = RVLDOTPRODUCT3(VQ_, VQ__);
						angVQ = acos(RVLLIMIT(fTmp, -1.0f, 1.0f));

						// e_ang <- angVP - angVQ

						e_ang = angVP - angVQ;

						// if |e_ang| > tau_ang, then association a' = (pj, qv) is not an element of K(a, j).

						e_ang = RVLABS(e_ang);
						if (e_ang > tau_ang)
							continue;

						// C_ang <- e_ang (Maybe this would change.)

						C_ang = e_ang;
						C_ang *= rj;

						// C_ang2 <- C_ang^2

						C_ang2 = C_ang * C_ang;
					}
				}

				// At this point, association a' = (pj, qv) is an element of K(a, j).

				// Add a' to D.

				RVLARRAYD_ADD(D, A.size());

				// Add a' to A.

				a.p = pj;
				a.q = qv;
				a.iPredecessor = iAssoc;
				a.iBranch = j;
				a.C = C_dst2 + C_ang2 + kappa * C_r2;
				a.L = 0.0f;
				a.bExpanded = false;
#ifdef RVLBM_MATCH_LOG
				fprintf(fpLog, "a%d(%d,%d): C=%f\n", A.size(), pj, qv, a.C);
#endif
				A.push_back(a);

				// minC <- min(minC, C)

				if (i == 0 || a.C < minC)
					minC = a.C;
			}

			///

			// a.C = lambda * rj * rj;
			a.C = lambda * rj;

			// minC <- min(minC, C)

			if (inRange.n == 0 || a.C < minC)
				minC = a.C;

			// La <- La + minC

			L += minC;

			// Add a' = (pj, None) to D.

			RVLARRAYD_ADD(D, A.size());

			// Add a' = (pj, None) to A.

			a.p = pj;
			a.q = -1;
			a.iPredecessor = iAssoc;
			a.iBranch = j;
			a.L = 0.0f;
			a.bExpanded = false;
#ifdef RVLBM_MATCH_LOG
			fprintf(fpLog, "a%d(%d,-1): C=%f\n", A.size(), pj, a.C);
#endif
			A.push_back(a);

			// Add D to a.DD.

			RVLARRAYD_ADD(DD, D);
		} // For every successor of pi.

		pAssoc = A.data() + iAssoc;
		pAssoc->L = L;

		// Add iBestSuccessor to a.M.
		// pAssoc->N <- association a' with the smallest index pj

		RVLARRAYD_INIT(M, idxMem);
		pAssoc->MFirstIdx = M.iFirst;
#ifdef RVLBM_MATCH_LOG
		fprintf(fpLog, "L=%f\n", pAssoc->L);
		fprintf(fpLog, "M=[ ");
#endif
		pAssoc->N = -1;
		for (j = 0; j < successors_.n; j++)
		{
			pj = successors_.Element[j];
			D = RVLARRAYD_ELEMENT(DD, j);
			iBestSuccessor = -1;
			for (i = 0; i < D.n; i++)
			{
				iAssoc_ = RVLARRAYD_ELEMENT(D, i);
				pAssoc_ = A.data() + iAssoc_;
				if (iBestSuccessor < 0 || pAssoc_->C < minC)
				{
					iBestSuccessor = iAssoc_;
					minC = pAssoc_->C;
				}
			}
			RVLARRAYD_ADD(M, iBestSuccessor)
#ifdef RVLBM_MATCH_LOG
			fprintf(fpLog, "a%d(%d,%d) ", iBestSuccessor, A[iBestSuccessor].p, A[iBestSuccessor].q);
#endif
			if (j == 0 || pj < minpj)
			{
				minpj = pj;
				pAssoc->N = iBestSuccessor;
			}
		}
#ifdef RVLBM_MATCH_LOG
		fprintf(fpLog, "]\n");
#endif

		pAssoc->bExpanded = true;

		//// END: Expand association iAssoc.

		//// Update the association tree.

#ifdef RVLBM_MATCH_LOG
		fprintf(fpLog, "Updating association tree\n");
#endif
		iAssoc_ = iAssoc;
		pAssoc_ = A.data() + iAssoc_;
		while (iAssoc_ > 0)
		{
			iAssoc__ = iAssoc_;
			pAssoc__ = pAssoc_;
			iAssoc_ = pAssoc_->iPredecessor;
			pAssoc_ = A.data() + iAssoc_;
#ifdef RVLBM_MATCH_LOG
			fprintf(fpLog, "a%d(%d,%d): ", iAssoc_, pAssoc_->p, pAssoc_->q);
#endif

			// D <- the set from a'.DD corresponding to the branch connnecting a'' with a'.

			DD.iFirst = pAssoc_->DDFirstIdx;
			D = RVLARRAYD_ELEMENT(DD, pAssoc__->iBranch);

			// a''' <- the association from D corresponding to the minimum value C + L

			for (i = 0; i < D.n; i++)
			{
				alpha = RVLARRAYD_ELEMENT(D, i);
				pAssocAlpha = A.data() + alpha;
				fTmp = pAssocAlpha->C + pAssocAlpha->L;
				if (i == 0 || fTmp < minC)
				{
					minC = fTmp;
					iAssoc___ = alpha;
				}
			}
			pAssoc___ = A.data() + iAssoc___;

			// Replace a'' with a''' in a'.M.

			M.iFirst = pAssoc_->MFirstIdx;
			RVLARRAYD_ELEMENT(M, pAssoc__->iBranch) = iAssoc___;
			// idxMem[pAssoc_->MFirstIdx + pAssoc__->iBranch] = iAssoc___;

			// pAssoc_->L <- sum of C + L of all associations from a'.M.

#ifdef RVLBM_MATCH_LOG
			fprintf(fpLog, "M=[ ");
#endif
			successors_ = (pAssoc_->p >= 0 ? P.Element[pAssoc_->p].successors : assocRootSuccessors);
			pAssoc_->L = 0.0f;
			for (i = 0; i < successors_.n; i++)
			{
				alpha = RVLARRAYD_ELEMENT(M, i);
				pAssocAlpha = A.data() + alpha;
				// if (pAssocAlpha->bExpanded)
				{
					pAssoc_->L += (pAssocAlpha->C + pAssocAlpha->L);
#ifdef RVLBM_MATCH_LOG
					fprintf(fpLog, "a%d(%d,%d) ", alpha, pAssocAlpha->p, pAssocAlpha->q);
#endif
				}
			}

			// a'' <- association from a'.M with the smallest first index of all associations in a'.M
			// a'.N <- a''.N

			pj = pAssoc_->p;
			successors_ = (pj >= 0 ? P.Element[pj].successors : assocRootSuccessors);
			pAssoc_->N = -1;
			for (k = 0; k < successors_.n; k++)
			{
				alpha = RVLARRAYD_ELEMENT(M, k);
				pAssocAlpha = A.data() + alpha;
				nu = (pAssocAlpha->bExpanded ? pAssocAlpha->N : alpha);
				if (nu < 0)
					continue;
				pAssocNu = A.data() + nu;
				pk = pAssocNu->p;
				if (pAssoc_->N < 0 || pk < minpk)
				{
					minpk = pk;
					pAssoc_->N = nu;
				}
			}
#ifdef RVLBM_MATCH_LOG
			fprintf(fpLog, "] L=%f\n", pAssoc_->L);
#endif
		}

		////

#ifdef RVLBM_MATCH_LOG
		fprintf(fpLog, "\n");
		fclose(fpLog);
#endif

		// if (idxMem[0] != 1)
		//	int debug = 0;

		iAssoc = A[0].N;
	}

	if (bVisualize)
	{
		Visualize(P, Q, A);
		pVisualizer->Run();
	}

	/////

	delete[] edges.Element;
	delete[] successorMem;
	delete[] VP;
	delete[] distP;
	delete[] VQ;
	delete[] distQ;
	delete[] inRangeMem;
	delete[] distLT;
	delete[] distLTMem;
	delete[] similarSizeMem;
	delete[] similarSizeParticles;
#ifdef RVLBM_MATCH_LOG
	delete[] optimalAssociations.Element;
#endif
}

void BranchMatcher::OptimalAssociations(
	Array<RECOG::BM::Particle> P,
	std::vector<RECOG::BM::Assoc> &A,
	int *c,
	Array<int> *pOptimalAssociations)
{
	memset(c, 0xff, P.n * sizeof(int));
	RECOG::BM::Assoc *pAssoc = A.data();
	int *assocBuff = (pOptimalAssociations ? pOptimalAssociations->Element : new int[P.n + 1]);
	int iFetch = 0;
	int iPush = 0;
	assocBuff[iPush++] = 0;
	int iAssoc;
	int pi;
	ArrayD<int> M;
	M.pStorage = &idxMem;
	int j;
	int iAssoc_;
	int nBranches;
	while (iFetch < iPush)
	{
		iAssoc = assocBuff[iFetch++];
		pAssoc = A.data() + iAssoc;
		pi = pAssoc->p;
		c[pi] = pAssoc->q;
		if (pAssoc->bExpanded)
		{
			M.iFirst = pAssoc->MFirstIdx;
			nBranches = (pi >= 0 ? P.Element[pi].successors.n : 1);
			for (j = 0; j < nBranches; j++)
			{
				iAssoc_ = RVLARRAYD_ELEMENT(M, j);
				assocBuff[iPush++] = iAssoc_;
			}
		}
	}
	if (pOptimalAssociations)
		pOptimalAssociations->n = iPush;
	else
		delete[] assocBuff;
}

#define RVLBM_TEST_NUM_PARTICLES 12

void BranchMatcher::TestTree(
	Array<RECOG::BM::Particle> &particles_P,
	Array<RECOG::BM::Particle> &particles_Q)
{
	// Parameters.

	float P[RVLBM_TEST_NUM_PARTICLES][2] = {{0.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, 2.0f}, {1.0f, 3.0f}, {-0.6f, 3.0f}, {1.4f, 4.0f}, {-1.0f, 4.0f}, {1.4f, 5.0f}, {-0.8, 5.0f}, {-2.0f, 4.2f}, {2.0f, 4.5f}, {1.4f, 6.0f}};
	int predecessor[RVLBM_TEST_NUM_PARTICLES] = {-1, 0, 1, 2, 2, 3, 4, 5, 6, 6, 5, 7};
	float rmin = 0.02f;
	float rmax = 0.2f;
	int noiseRatio = 3;
	float krNoise = 0.2f;
	float noiseDist = 0.5f;		// m
	float positionNoise = 0.1f; // m
	float rotZDeg = 30.0f;		// deg
	float transX = 1.0f;		// m

	// Pseudo random numbers.

	int iRndVal = 0;
	Array<int> rndVal;
	rndVal.n = 1000000;
	RandomIndices(rndVal);

	// P

	particles_P.n = RVLBM_TEST_NUM_PARTICLES;
	particles_P.Element = new RECOG::BM::Particle[particles_P.n];
	RECOG::BM::Particle *pParticleP = particles_P.Element;
	for (int i = 0; i < particles_P.n; i++, pParticleP++)
	{
		RVLSET3VECTOR(pParticleP->c, P[i][0], 0.0f, P[i][1]);
		pParticleP->r = rmax - (float)i * (rmax - rmin) / (float)(particles_P.n - 1);
	}

	// Relative camera pose.

	float rotZ = DEG2RAD * rotZDeg;
	float cs = cos(rotZ);
	float sn = sin(rotZ);
	Pose3D pose;
	RVLROTZ(cs, sn, pose.R);
	RVLSET3VECTOR(pose.t, transX, 0.0f, 0.0f);

	// Q

	float displacement[3];
	particles_Q.n = particles_P.n * (noiseRatio + 1);
	particles_Q.Element = new RECOG::BM::Particle[particles_Q.n];
	pParticleP = particles_P.Element;
	RECOG::BM::Particle *pParticleQ = particles_Q.Element;
	for (int i = 0; i < particles_P.n; i++, pParticleP++, pParticleQ++)
	{
		RVLTRANSF3(pParticleP->c, pose.R, pose.t, pParticleQ->c);
		for (int i = 0; i < 3; i++)
			displacement[i] = positionNoise * (2.0f * RealPseudoRand<float>(rndVal, iRndVal) - 1.0f);
		RVLSUM3VECTORS(pParticleQ->c, displacement, pParticleQ->c);
		pParticleQ->r = pParticleP->r * (1.0f + krNoise * (2.0f * RealPseudoRand<float>(rndVal, iRndVal) - 1.0f));
	}
	pParticleP = particles_P.Element;
	for (int pi = 0; pi < particles_P.n; pi++, pParticleP++)
	{
		for (int j = 0; j < noiseRatio; j++, pParticleQ++)
		{
			for (int i = 0; i < 3; i++)
				displacement[i] = noiseDist * (2.0f * RealPseudoRand<float>(rndVal, iRndVal) - 1.0f);
			RVLTRANSF3(pParticleP->c, pose.R, pose.t, pParticleQ->c);
			RVLSUM3VECTORS(pParticleQ->c, displacement, pParticleQ->c);
			pParticleQ->r = pParticleP->r * (1.0f + krNoise * (2.0f * RealPseudoRand<float>(rndVal, iRndVal) - 1.0f));
		}
	}

	// Visualization.

	// Visualize(particles_P, particles_Q, predecessor);
	// pVisualizationData->pVisualizer->Run();

	//

	delete[] rndVal.Element;
}

void BranchMatcher::InitVisualizer(Visualizer *pVisualizerIn)
{
	if (pVisualizationData == NULL)
		pVisualizationData = new RECOG::BM::DisplayCallbackData;
	if (pVisualizerIn)
	{
		pVisualizationData->pVisualizer = pVisualizerIn;
		pVisualizationData->bOwnVisualizer = false;
	}
	else
	{
		pVisualizationData->pVisualizer = new Visualizer;
		pVisualizationData->bOwnVisualizer = true;
	}
	pVisualizationData->paramList.m_pMem = pMem0;
	RVLPARAM_DATA *pParamData;
	pVisualizationData->paramList.Init();
	// pParamData = pVisualizationData->paramList.AddParam("DDD.visualizeSurfels", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bVisualizeSurfels));
	pVisualizationData->paramList.LoadParams((char *)(cfgFileName.data()));

	Visualizer *pVisualizer = pVisualizationData->pVisualizer;
	// pVisualizer->SetMouseRButtonDownCallback(RECOG::BM::MouseRButtonDown, pVisualizationData);

	pVisualizer->SetCellPicker();
	pVisualizer->SetMouseRButtonDownCallback(RECOG::BM::MouseRButtonDown, pVisualizationData);
}

void BranchMatcher::Visualize(
	Array<RECOG::BM::Particle> particles_P,
	Array<RECOG::BM::Particle> particles_Q)
{
	RVLCOLORS
	Visualizer *pVisualizer = pVisualizationData->pVisualizer;
	Array<Point> visPts;
	Array<Pair<int, int>> visLines;

	// Visualize P and tree.

	visPts.Element = new Point[particles_P.n];
	visPts.n = particles_P.n;
	visLines.Element = new Pair<int, int>[particles_P.n - 1];
	visLines.n = 0;
	RECOG::BM::Particle *pParticleP = particles_P.Element;
	for (int i = 0; i < particles_P.n; i++, pParticleP++)
		RVLCOPY3VECTOR(pParticleP->c, visPts.Element[i].P);
	Pair<int, int> *pVisLine = visLines.Element;
	int iPredecessor;
	for (int i = 0; i < particles_P.n; i++)
	{
		iPredecessor = particles_P.Element[i].iPredecessor;
		if (iPredecessor < 0)
			continue;
		pVisLine->a = i;
		pVisLine->b = iPredecessor;
		pVisLine++;
	}
	visLines.n = pVisLine - visLines.Element;
	vtkSmartPointer<vtkActor> actorP = pVisualizer->DisplayPointSet<float, Point>(visPts, blue, 6.0f);
	pVisualizer->DisplayLines(visPts, visLines, blue);
	delete[] visPts.Element;

	// Shift of the visualization of Q.

	Box<float> bboxP;
	pParticleP = particles_P.Element;
	InitBoundingBox<float>(&bboxP, pParticleP->c);
	pParticleP++;
	for (int i = 1; i < particles_P.n; i++, pParticleP++)
		UpdateBoundingBox<float>(&bboxP, pParticleP->c);
	float wP = bboxP.maxx - bboxP.minx;

	Box<float> bboxQ;
	RECOG::BM::Particle *pParticleQ = particles_Q.Element;
	InitBoundingBox<float>(&bboxQ, pParticleQ->c);
	pParticleQ++;
	for (int i = 1; i < particles_Q.n; i++, pParticleQ++)
		UpdateBoundingBox<float>(&bboxQ, pParticleQ->c);
	float wQ = bboxQ.maxx - bboxQ.minx;
	pVisualizationData->shiftQ[0] = bboxP.maxx + 0.5f * RVLMAX(wP, wQ) - bboxQ.minx;
	pVisualizationData->shiftQ[1] = pVisualizationData->shiftQ[2] = 0.0f;

	// Visualize Q.

	visPts.Element = new Point[particles_Q.n];
	visPts.n = particles_Q.n;
	pParticleQ = particles_Q.Element;
	for (int i = 0; i < particles_Q.n; i++, pParticleQ++)
		RVLSUM3VECTORS(pParticleQ->c, pVisualizationData->shiftQ, visPts.Element[i].P);
	vtkSmartPointer<vtkActor> actorQ = pVisualizer->DisplayPointSet<float, Point>(visPts, black, 6.0f);
	// pVisualizer->DisplayLines(visPts, visLines, black);
	delete[] visPts.Element;
	delete[] visLines.Element;

	// Associate particles with the visualizer point picker.

	pVisualizationData->particles = particles_Q;
	pVisualizer->AssociateActorWithPointPicker(actorQ);
}

void BranchMatcher::Visualize(
	Array<RECOG::BM::Particle> P,
	Array<RECOG::BM::Particle> Q,
	std::vector<RECOG::BM::Assoc> &A)
{
	RVLCOLORS
	Array<Point> visPts;
	visPts.Element = new Point[4 * P.n];
	Point *pVisPt = visPts.Element;
	Array<Pair<int, int>> visLines;
	visLines.Element = new Pair<int, int>[2 * P.n];
	Pair<int, int> *pVisLine = visLines.Element;
	Array<Pair<int, int>> visLines2;
	visLines2.Element = new Pair<int, int>[P.n];
	Pair<int, int> *pVisLine2 = visLines2.Element;
	RECOG::BM::Assoc *pAssoc = A.data();
	int *assocBuff = new int[P.n + 1];
	int iFetch = 0;
	int iPush = 0;
	assocBuff[iPush++] = 0;
	int iAssoc;
	int pi;
	ArrayD<int> M;
	M.pStorage = &idxMem;
	int j;
	int iAssoc_, iAssoc__;
	RECOG::BM::Assoc *pAssoc_, *pAssoc__;
	int pj, pk;
	int nBranches;
	RECOG::BM::Particle *pParticle;
	int iVisPt;
	while (iFetch < iPush)
	{
		iAssoc = assocBuff[iFetch++];
		pAssoc = A.data() + iAssoc;
		if (!pAssoc->bExpanded)
			continue;
		pi = pAssoc->p;
		M.iFirst = pAssoc->MFirstIdx;
		nBranches = (pi >= 0 ? P.Element[pi].successors.n : 1);
		for (j = 0; j < nBranches; j++)
		{
			iAssoc_ = RVLARRAYD_ELEMENT(M, j);
			assocBuff[iPush++] = iAssoc_;
		}
		if (pAssoc->q < 0)
			continue;
		iVisPt = pVisPt - visPts.Element;
		pParticle = P.Element + pAssoc->p;
		RVLCOPY3VECTOR(pParticle->c, pVisPt->P);
		pVisPt++;
		pParticle = Q.Element + pAssoc->q;
		RVLSUM3VECTORS(pParticle->c, pVisualizationData->shiftQ, pVisPt->P);
		pVisPt++;
		pVisLine2->a = iVisPt;
		pVisLine2->b = iVisPt + 1;
		pVisLine2++;
		pAssoc_ = pAssoc;
		do
		{
			iAssoc_ = pAssoc_->iPredecessor;
			if (iAssoc_ < 0)
				break;
			pAssoc_ = A.data() + iAssoc_;
		} while (pAssoc_->q < 0);
		if (iAssoc_ < 0 || pAssoc_->q < 0)
			continue;
		pParticle = P.Element + pAssoc_->p;
		RVLCOPY3VECTOR(pParticle->c, pVisPt->P);
		pVisPt++;
		pParticle = Q.Element + pAssoc_->q;
		RVLSUM3VECTORS(pParticle->c, pVisualizationData->shiftQ, pVisPt->P);
		pVisPt++;
		pVisLine->a = iVisPt;
		pVisLine->b = iVisPt + 2;
		pVisLine++;
		pVisLine->a = iVisPt + 1;
		pVisLine->b = iVisPt + 3;
		pVisLine++;
	}
	delete[] assocBuff;
	visPts.n = pVisPt - visPts.Element;
	visLines.n = pVisLine - visLines.Element;
	visLines2.n = pVisLine2 - visLines2.Element;
	if (visLines.n > 0)
	{
		pVisualizationData->assocActors.push_back(pVisualizationData->pVisualizer->DisplayPointSet<float, Point>(visPts, red, 8.0f));
		pVisualizationData->assocActors.push_back(pVisualizationData->pVisualizer->DisplayLines(visPts, visLines, red, 2.0f));
		pVisualizationData->assocActors.push_back(pVisualizationData->pVisualizer->DisplayLines(visPts, visLines2, green));
	}
	delete[] visPts.Element;
	delete[] visLines.Element;
	delete[] visLines2.Element;
}
