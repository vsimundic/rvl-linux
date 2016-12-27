//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkLine.h>
#include <vtkPolyLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
//#include "RFRecognition.h" //VIDOVIC
//#include <Eigen\Eigenvalues>

using namespace RVL;
using namespace SURFEL;

SurfelGraph::SurfelGraph()
{
	imageAdjacencyThr = 6;

	PtMem = NULL;
	surfelBndMem = NULL;
	surfelBndMem2 = NULL;
	BndMem = NULL;
	neighborEdge = NULL;
	surfelMap = NULL;
	edgeMap = NULL;
	nodeColor = NULL;
	NodeArray.Element = NULL;
	edgeMarkMap = NULL;
	EdgeArray.Element = NULL;
	surfelVertexList.Element = NULL;
	surfelVertexMem = NULL;
	vertexArray.Element = NULL;
	vertexDisplayLineArray.Element = NULL;
	vertexDisplayLineArrayMem = NULL;

	DisplayData.mouseRButtonDownUserFunction = NULL;
	DisplayData.keyPressUserFunction = NULL;
	DisplayData.edgeFeatureDepth = 0.01f;
	DisplayData.normalLen = 10.0f;
}


SurfelGraph::~SurfelGraph()
{
	Clear();
}

void SurfelGraph::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("SurfelGraph.visualization.edgeFeatureDepth", RVLPARAM_TYPE_FLOAT, &(DisplayData.edgeFeatureDepth));
	pParamData = ParamList.AddParam("SurfelGraph.visualization.normalLen", RVLPARAM_TYPE_FLOAT, &(DisplayData.normalLen));
}

void SurfelGraph::InitGetNeighborsBoundaryAndSize()
{
	neighborEdge = new SURFEL::Edge *[NodeArray.n];

	memset(neighborEdge, 0, NodeArray.n * sizeof(SURFEL::Edge *));

	memset(edgeMarkMap, 0, nMeshEdges * sizeof(unsigned char));
}

void SurfelGraph::FreeGetNeighborsBoundaryAndSize()
{
	RVL_DELETE_ARRAY(neighborEdge);
}

void SURFEL::ComputeParameters(
	Surfel *pSurfel,
	MESH::Distribution &distribution,
	Point *pPt)
{
	float *var = distribution.var;

	int idx[3];
	int iTmp;

	RVLSORT3DESCEND(var, idx, iTmp);

	float *N = pSurfel->N;
	float *N_ = distribution.R + 3 * idx[0];

	RVLCOPY3VECTOR(N_, N);

	if (RVLDOTPRODUCT3(pPt->N, N) > 0.0)
	{
		RVLCOPY3VECTOR(N, pSurfel->N)
	}
	else
	{
		RVLNEGVECT3(N, pSurfel->N)
	}

	float *P = pSurfel->P;
	float *P_ = distribution.t;

	RVLCOPY3VECTOR(P_, P);

	pSurfel->d = RVLDOTPRODUCT3(N, P);

	int *RGB = pSurfel->RGB;
	int *RGB_ = distribution.RGB;

	RVLCOPY3VECTOR(RGB_, RGB);

	float *P0 = pSurfel->P0;

	RVLCOPY3VECTOR(pPt->P, P0);

	pSurfel->r0 = pSurfel->d / RVLDOTPRODUCT3(N, P0);
}

void SURFEL::CreateFromPoint(
	Surfel *pSurfel, 
	Point *pPt)
{
	float *P = pSurfel->P;
	float *P_ = pPt->P;

	RVLCOPY3VECTOR(P_, P);

	float *P0 = pSurfel->P0;

	RVLCOPY3VECTOR(P_, P0);

	float *N = pSurfel->N;
	float *N_ = pPt->N;

	RVLCOPY3VECTOR(N_, N);

	pSurfel->d = RVLDOTPRODUCT3(N, P);

	RVLCOPY3VECTOR(pPt->RGB, pSurfel->RGB);

	pSurfel->r0 = pSurfel->d / RVLDOTPRODUCT3(N, P0);
}

// Create point pPoint from the surfel pSurfel such that its position is identical to the position of the surfel centroid,
// its normal i identical to the surfel normal and its color is identical to the surfel color

void SURFEL::GetPoint(
	Surfel *pSurfel,
	Point *pPoint)
{
	RVLCOPY3VECTOR(pSurfel->P, pPoint->P);
	RVLCOPY3VECTOR(pSurfel->N, pPoint->N);
	RVLCOPY3VECTOR(pSurfel->RGB, pPoint->RGB);
}

void SurfelGraph::Init(Mesh *pMesh)
{
	Clear();

	nMeshVertices = pMesh->NodeArray.n;
	nMeshEdges = pMesh->EdgeArray.n;

	PtMem = new QLIST::Index2[nMeshVertices];
	surfelBndMem = new MeshEdgePtr *[2 * nMeshEdges];
	surfelBndMem2 = new Array<MeshEdgePtr *>[nMeshEdges];
	BndMem = new MeshEdgePtr *[pMesh->nBoundaryPts];
	surfelMap = new int[nMeshVertices];
	edgeMap = new int[nMeshVertices];
	//surfelBndMap = new QLIST::Index2[nPoints];
	NodeArray.Element = new Surfel[2 * nMeshVertices];
	edgeMarkMap = new unsigned char[nMeshEdges];
}

#ifdef RVLSURFEL_IMAGE_ADJACENCY
void SurfelGraph::ImageAdjacency(Mesh *pMesh)
{
	bool *bVisited = new bool[NodeArray.n];

	memset(bVisited, 0, NodeArray.n * sizeof(bool));

	int *surfelIdx = new int[NodeArray.n];

	memset(surfelIdx, 0xff, NodeArray.n * sizeof(int));

	nImageAdjacencyRelations = 0;

	int iSurfel;
	Surfel *pSurfel;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->size <= 1)
			continue;

		ImageAdjacency(pMesh, iSurfel, surfelIdx, bVisited);
	}

	delete[] bVisited;
	delete[] surfelIdx;
}

void SurfelGraph::ImageAdjacency(
	Mesh *pMesh,
	int iSurfel,
	int *surfelIdx,
	bool *bVisited)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	//find largest boundary (most probable outer boundary)
	int boundary = 0;
	int boundarySize = 0;
	if (pSurfel->BoundaryArray.n > 1)
	{
		for (int b = 0; b < pSurfel->BoundaryArray.n; b++)
		{
			if (pSurfel->BoundaryArray.Element[b].n > boundarySize)
			{
				boundarySize = pSurfel->BoundaryArray.Element[b].n;
				boundary = b;
			}
		}
	}
	else
		boundarySize = pSurfel->BoundaryArray.Element[boundary].n;

	int i;
	int iOtherSurfel;

	for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
	{
		iOtherSurfel = pSurfel->imgAdjacency.at(i) - NodeArray.Element;

		surfelIdx[iOtherSurfel] = i;
	}

	//run through edges
	Array<MeshEdgePtr *> BoundaryArray = pSurfel->BoundaryArray.Element[boundary];
	MeshEdgePtr *pCurrEdge;
	Surfel *pOtherSurfel;
	SurfelAdjecencyDescriptors *desc;
	int iBoundary, iPointEdge;
	int iPt, iPt2, x, y;
	double tempDist;
	Point *pPt, *pPt2;
	float *P, *P2;
	float dP[3];

	for (iPointEdge = 0; iPointEdge < BoundaryArray.n; iPointEdge++)
	{
		pCurrEdge = BoundaryArray.Element[iPointEdge];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);

		pPt = pMesh->NodeArray.Element + iPt;

		P = pPt->P;

		y = floor(iPt / 640.0);
		x = floor(iPt - 640.0 * y);

		//Running through point neighbourhood
		for (int yy = y - imageAdjacencyThr; yy < y + imageAdjacencyThr; yy++)
		{
			if ((yy < 0) || (yy >= 480))
				continue;
			for (int xx = x - imageAdjacencyThr; xx < x + imageAdjacencyThr; xx++)
			{
				if ((xx < 0) || (xx >= 640))
					continue;
				iPt2 = yy * 640 + xx;
				pPt2 = pMesh->NodeArray.Element + iPt2;
				P2 = pPt2->P;
				iOtherSurfel = surfelMap[iPt2];

				if (iOtherSurfel < 0 || iOtherSurfel >= NodeArray.n)
					continue;

				pOtherSurfel = NodeArray.Element + iOtherSurfel;	//surfel owner of the pixel
				
				if ((pOtherSurfel->size < 640 * 480) && (pOtherSurfel->size > 1) && (iOtherSurfel != iSurfel))
				{
					if (surfelIdx[iOtherSurfel] < 0)
					{
						nImageAdjacencyRelations++;

						//calculate min dist
						//preallocate the adjacency descriptor for future use
						RVLMEM_ALLOC_STRUCT(pMem, SurfelAdjecencyDescriptors, desc);

						RVLDIF3VECTORS(P2, P, dP);

						desc->minDist = RVLDOTPRODUCT3(dP, dP);
						desc->cupyDescriptor[0] = 0.0;
						desc->cupyDescriptor[1] = 0.0;
						desc->cupyDescriptor[2] = 0.0;
						desc->cupyDescriptor[3] = 0.0;
						desc->commonBoundaryLength = 0;

						surfelIdx[iOtherSurfel] = pSurfel->imgAdjacency.size();

						pSurfel->imgAdjacency.push_back(pOtherSurfel);	//push surfel pointer on the list
						pSurfel->imgAdjacencyDescriptors.push_back(desc);	//push descriptor on the list

						pOtherSurfel->imgAdjacency.push_back(pSurfel);
						pOtherSurfel->imgAdjacencyDescriptors.push_back(desc);

						bVisited[iOtherSurfel] = true;

						//push to other surfel
					}
					else
					{
						desc = pSurfel->imgAdjacencyDescriptors.at(surfelIdx[iOtherSurfel]);	//get related descriptor

						RVLDIF3VECTORS(P2, P, dP);

						tempDist = RVLDOTPRODUCT3(dP, dP);
						if (tempDist < desc->minDist)	//update if the new one is smaller
							desc->minDist = tempDist;

						bVisited[iOtherSurfel] = true;
					}
				}
			}
		}	//Running through point neighbourhood

		for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
		{
			pOtherSurfel = pSurfel->imgAdjacency.at(i);

			iOtherSurfel = pOtherSurfel - NodeArray.Element;

			if (bVisited[iOtherSurfel])
			{
				bVisited[iOtherSurfel] = false;

				desc = pSurfel->imgAdjacencyDescriptors.at(i);

				desc->commonBoundaryLength++;
			}
		}
	}	// for every boundary point

	for (i = 0; i < pSurfel->imgAdjacency.size(); i++)
	{
		desc = pSurfel->imgAdjacencyDescriptors.at(i);

		desc->minDist = sqrt(desc->minDist);

		pOtherSurfel = pSurfel->imgAdjacency.at(i);

		iOtherSurfel = pOtherSurfel - NodeArray.Element;

		surfelIdx[iOtherSurfel] = -1;
	}
}

//Returns surfels Label ID with most object support
void SurfelGraph::SetPrimaryGTObj(Surfel *pSurfel, cv::Mat labGTImg, int noObj)
{
	int objIdx = -1;	//default value
	//Generate histogram of object (pixel) support
	int *objHist = new int[noObj];
	memset(objHist, 0, noObj * sizeof(int));
	RVL::QLIST::Index2 *pt;
	int x = 0, y = 0;
	pt = pSurfel->PtList.pFirst;
	for (int i = 0; i < pSurfel->size; i++)
	{
		y = floor(pt->Idx / 640.0);
		x = floor(pt->Idx - 640.0 * y);
		objHist[labGTImg.at<cv::Vec3b>(y, x)[0]]++;
		pt = pt->pNext;
	}
	//find max support and set surfel GTObjHist
	int max = 0;
	for (int i = 0; i < noObj; i++)
	{
		if (objHist[i] > max)
		{
			max = objHist[i];
			objIdx = i;
		}
		pSurfel->GTObjHist.push_back(objHist[i]);
	}
	delete[] objHist;
	pSurfel->ObjectID = objIdx;
}

void SurfelGraph::AssignGroundTruthSegmentation(
	char *meshFileName,
	int minSurfelSize)
{
	//Segmentation analysis
	//filenames
	std::string labelImgFileName(meshFileName);
	labelImgFileName.erase(labelImgFileName.find_last_of("."));
	//std::string depthImgFileName = labelImgFileName + "d.png";
	//std::string ssfFileName = labelImgFileName + ".ssf";
	labelImgFileName += "a.png";
	////TEST SSF LOAD
	//SceneSegFile::SceneSegFile* ssf = new SceneSegFile::SceneSegFile("test");
	//ssf->Load(ssfFileName);
	//
	//Load label image
	cv::Mat GTlabImg = cv::imread(labelImgFileName);
	//cv::Mat GTdepthImg = cv::imread(depthImgFileName, cv::ImreadModes::IMREAD_ANYDEPTH);
	////Preprocess GT label image (such as labeling background)
	//PreprocessGTLab(GTlabImg, GTdepthImg);
	//Get label min/max value
	double minLab, maxLab;
	cv::minMaxLoc(GTlabImg, &minLab, &maxLab);

	//Detect primary GT object for ALL surfels
	Surfel *pCurrSurfel = NodeArray.Element;
	std::cout << "Detecting primary GT object!" << std::endl;
	for (int i = 0; i < NodeArray.n; pCurrSurfel++, i++)
	{
		pCurrSurfel->ObjectID = -1;
		//if ((pCurrSurfel->size == 1) || (pCurrSurfel->size == 0) || pCurrSurfel->bEdge || pCurrSurfel->size < minSurfelSize)
		if ((pCurrSurfel->size <= 1) || pCurrSurfel->bEdge)
			continue;
		/*pCurrSurfel->ObjectID = DetPrimaryGTObj(pCurrSurfel, GTlabImg, 256);*/ //256 objects because background has label of 255
		SetPrimaryGTObj(pCurrSurfel, GTlabImg, maxLab + 1); //maxLab + 1 because the last GT object label has to be maxLab and not maxLab - 1
	}
}
#endif

void SurfelGraph::Clear()
{
	RVL_DELETE_ARRAY(PtMem);
	RVL_DELETE_ARRAY(surfelBndMem);
	RVL_DELETE_ARRAY(surfelBndMem2);
	RVL_DELETE_ARRAY(BndMem);
	RVL_DELETE_ARRAY(surfelMap);
	RVL_DELETE_ARRAY(edgeMap);	
	//RVL_DELETE_ARRAY(surfelBndMap);
	RVL_DELETE_ARRAY(nodeColor);
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(edgeMarkMap);
	RVL_DELETE_ARRAY(neighborEdge);
	RVL_DELETE_ARRAY(EdgeArray.Element);
	RVL_DELETE_ARRAY(vertexArray.Element);
	RVL_DELETE_ARRAY(surfelVertexList.Element);
	RVL_DELETE_ARRAY(surfelVertexMem);
	RVL_DELETE_ARRAY(vertexDisplayLineArray.Element);
	RVL_DELETE_ARRAY(vertexDisplayLineArrayMem);
}

void SurfelGraph::DetectVertices(
	Mesh *pMesh)
{
	QList<Vertex> *pVertexList = &vertexList;

	RVLQLIST_INIT(pVertexList);

	nVertexSurfelRelations = 0;

	int nVertices = 0;

	RVL_DELETE_ARRAY(surfelVertexList.Element);

	surfelVertexList.Element = new QList<QLIST::Index>[NodeArray.n];
	surfelVertexList.n = NodeArray.n;

	//float csEdgeTangentAngle = cos(edgeTangentAngle * DEG2RAD);
	//float snEdgeTangentAngle = sqrt(1.0f - csEdgeTangentAngle * csEdgeTangentAngle);

	bool *bVisited = new bool[NodeArray.n];

	memset(bVisited, 0, NodeArray.n * sizeof(bool));

	int iSurfel, iSurfel_;
	//int iSurfel1, iSurfel2;
	//int iPrevSurfel;
	int iBoundary;
	int iPointEdge;
	int iPt, iPt_, iPt__;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_, *pLastEdgePtr;
	//MeshEdge *pEdge;
	QList<MeshEdgePtr> *pEdgeList;
	Vertex *pVertex;
	Point *pPt;
	//Point *pPt_;
	QList<QLIST::Index> *pSurfelVertexList;
	float *N;
	//float *N1, *N2;
	//float N2_[3], VTmp[3];
	//float fTmp;
	int nPlanarFeatures, nEdgeFeatures, nFeatures, iFeature;
	int iEdgeFeature, iEdgeFeature_, iEdgeFeature__;
	bool bSmallestIndex;
	Surfel *pSurfel_, *pEdgeFeature;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		RVLQLIST_INIT(pSurfelVertexList);

		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		if (pSurfel->size <= 1)
			continue;

		N = pSurfel->N;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				// Determine if iPt is the point with the smallest index in its immediate neighborhood.

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

				pPt = pMesh->NodeArray.Element + iPt;

				if (pPt->bBoundary)
					iEdgeFeature = edgeMap[iPt];

				bVisited[iSurfel] = true;

				nPlanarFeatures = 1;

				pEdgeList = &(pPt->EdgeList);

				pEdgePtr_ = pEdgeList->pFirst;

				while (pEdgePtr_)
				{
					iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

					iSurfel_ = surfelMap[iPt_];

					if (iSurfel_ >= 0 && iSurfel_ < NodeArray.n)
					{
						if (!bVisited[iSurfel_])
						{
							if (iSurfel_ < iSurfel)
								break;

							nPlanarFeatures++;

							bVisited[iSurfel_] = true;
						}
					}

					if (pEdgePtr_->pNext == NULL)
						pLastEdgePtr = pEdgePtr_;

					pEdgePtr_ = pEdgePtr_->pNext;
				}

				bSmallestIndex = (pEdgePtr_ == NULL);

				// Reset bVisited.

				bVisited[iSurfel] = false;

				pEdgePtr_ = pEdgeList->pFirst;

				while (pEdgePtr_)
				{
					iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

					iSurfel_ = surfelMap[iPt_];

					if (iSurfel_ >= 0 && iSurfel_ < NodeArray.n)
						bVisited[iSurfel_] = false;

					pEdgePtr_ = pEdgePtr_->pNext;
				}

				nEdgeFeatures = 0;

				if (bSmallestIndex)	// If iPt is the point with the smallest index in its immediate neighborhood
				{
					if (pPt->bBoundary)
					{
						if (iEdgeFeature >= 0)
							nEdgeFeatures++;

						pEdgePtr_ = pEdgeList->pFirst;

						iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

						iEdgeFeature_ = edgeMap[iPt_];

						if (iEdgeFeature_ >= 0 && iEdgeFeature_ != iEdgeFeature)
							nEdgeFeatures++;

						iPt__ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pLastEdgePtr);

						iEdgeFeature__ = edgeMap[iPt__];

						if (iEdgeFeature__ >= 0 && iEdgeFeature__ != iEdgeFeature && iEdgeFeature__ != iEdgeFeature_)
							nEdgeFeatures++;

						if (nPlanarFeatures == 1 && nEdgeFeatures >= 2)
						{
							if ((iEdgeFeature_ >= 0 && iEdgeFeature > iEdgeFeature_) || (iEdgeFeature__ >= 0 && iEdgeFeature > iEdgeFeature__))
								bSmallestIndex = false;
						}
					}
				}	// if (pEdgePtr_ == NULL)

				nFeatures = nPlanarFeatures + nEdgeFeatures;

				if (bSmallestIndex && nFeatures >= 3)	// If iPt is the point with the smallest index in its immediate neighborhood 
					// and at least three features meet in iPt, then this point is a vertex.
				{
					//if (iPt == 221223)
					//	int debug = 0;

					// Create vertex.

					RVLMEM_ALLOC_STRUCT(pMem, Vertex, pVertex);

					pVertex->bEdge = pPt->bBoundary;

					RVLCOPY3VECTOR(pPt->P, pVertex->P);

					RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, nFeatures, pVertex->iSurfelArray.Element);

					RVLMEM_ALLOC_STRUCT_ARRAY(pMem, NormalHullElement, nFeatures, pVertex->normalHull.Element);

					nVertexSurfelRelations += nFeatures;

					pVertex->normalHull.n = 0;

					iFeature = 0;

					bVisited[iSurfel] = true;

					pVertex->iSurfelArray.Element[iFeature++] = iSurfel;

					UpdateNormalHull(pVertex->normalHull, pSurfel->N);

					pEdgePtr_ = pEdgeList->pFirst;

					while (pEdgePtr_)
					{
						iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

						iSurfel_ = surfelMap[iPt_];

						if (iSurfel_ >= 0 && iSurfel_ < NodeArray.n)
						{
							if (!bVisited[iSurfel_])
							{
								bVisited[iSurfel_] = true;

								pVertex->iSurfelArray.Element[iFeature++] = iSurfel_;

								pSurfel_ = NodeArray.Element + iSurfel_;

								UpdateNormalHull(pVertex->normalHull, pSurfel_->N);
							}
						}

						pEdgePtr_ = pEdgePtr_->pNext;
					}

					if (pPt->bBoundary)
					{
						if (iEdgeFeature >= 0)
						{
							pVertex->iSurfelArray.Element[iFeature++] = iEdgeFeature;

							pEdgeFeature = NodeArray.Element + iEdgeFeature;

							UpdateNormalHull(pVertex->normalHull, pEdgeFeature->N);
						}

						pEdgePtr_ = pEdgeList->pFirst;

						iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

						iEdgeFeature_ = edgeMap[iPt_];

						if (iEdgeFeature_ >= 0 && iEdgeFeature_ != iEdgeFeature)
						{
							pVertex->iSurfelArray.Element[iFeature++] = iEdgeFeature_;

							pEdgeFeature = NodeArray.Element + iEdgeFeature_;

							UpdateNormalHull(pVertex->normalHull, pEdgeFeature->N);
						}

						iPt__ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pLastEdgePtr);

						iEdgeFeature__ = edgeMap[iPt__];

						if (iEdgeFeature__ >= 0 && iEdgeFeature__ != iEdgeFeature && iEdgeFeature__ != iEdgeFeature_)
						{
							pVertex->iSurfelArray.Element[iFeature++] = iEdgeFeature__;

							pEdgeFeature = NodeArray.Element + iEdgeFeature__;

							UpdateNormalHull(pVertex->normalHull, pEdgeFeature->N);
						}
					}

					pVertex->iSurfelArray.n = nFeatures;

					//if (pVertex->normalHull.n < 3)
					//	int debug = 0;

					//if (iFeature != nFeatures)
					//	int debug = 0;

					// Reset bVisited.

					bVisited[iSurfel] = false;

					pEdgePtr_ = pEdgeList->pFirst;

					while (pEdgePtr_)
					{
						iPt_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr_);

						iSurfel_ = surfelMap[iPt_];

						if (iSurfel_ >= 0 && iSurfel_ < NodeArray.n)
							bVisited[iSurfel_] = false;

						pEdgePtr_ = pEdgePtr_->pNext;
					}

					// Add vertex to the vertex list.

					RVLQLIST_ADD_ENTRY(pVertexList, pVertex);

					nVertices++;
				}	// if (bSmallestIndex && nFeatures >= 3)

#ifdef NEVER
				// Old version.

				pEdgePtr_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr_);

				pPt = pMesh->NodeArray.Element + iPt;

				pEdgeList = &(pPt->EdgeList);

				iPrevSurfel = -1;

				while (true)	// for each neighboring point of the point iPt
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
								RVLMEM_ALLOC_STRUCT(pMem, Vertex, pVertex);

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

									//fTmp = csEdgeTangentAngle / sqrt(RVLDOTPRODUCT3(VTmp, VTmp));

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
#endif
			}	// for each point-edge on the boundary contour
		}	// for each boundary contour
	}	// for each surfel

	delete[] bVisited;

	RVL_DELETE_ARRAY(vertexArray.Element);

	vertexArray.Element = new Vertex *[nVertices];
	vertexArray.n = nVertices;

	QLIST::CreatePtrArray<Vertex>(&vertexList, &vertexArray);

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
}

void SurfelGraph::UpdateNormalHull(
	Array<NormalHullElement> &NHull,
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

	NormalHullElement *pHullElement = NHull.Element + NHull.n - 1;

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
			memmove(NHull.Element + iEnd + 1, NHull.Element + iEnd, (NHull.n - iEnd) * sizeof(NormalHullElement));

			iEnd++;
		}

		NHull.n++;
	}
	else if (iEnd > (iStart + 2) % NHull.n)	// Size of NHull should be decreased.
	{
		if (iEnd > iStart)
		{
			memmove(NHull.Element + iStart + 2, NHull.Element + iEnd, (NHull.n - iEnd - 1) * sizeof(NormalHullElement));

			NHull.n -= (iEnd - iStart - 2);
		}
		else
		{
			if (iEnd > 0)
			{
				memmove(NHull.Element, NHull.Element + iEnd, (iStart - iEnd) * sizeof(NormalHullElement));

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

void SurfelGraph::NodeColors(unsigned char *SelectionColor)
{
	int SelectionColor_[3];

	RVLCONVTOINT3(SelectionColor, SelectionColor_);

	RVL_DELETE_ARRAY(nodeColor);

	nodeColor = new unsigned char[3 * NodeArray.n];

	Surfel *pSurfel;
	int iNode;
	int Color[3], dColor[3];
	unsigned char *NodeColor_;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pSurfel = NodeArray.Element + iNode;

		do
		{
			Color[0] = rand() % 256;
			Color[1] = rand() % 256;
			Color[2] = rand() % 256;

			RVLDIF3VECTORS(Color, SelectionColor, dColor);
		} while (RVLDOTPRODUCT3(dColor, dColor) < 128 * 128);

		NodeColor_ = nodeColor + 3 * iNode;

		NodeColor_[0] = (unsigned char)Color[0];
		NodeColor_[1] = (unsigned char)Color[1];
		NodeColor_[2] = (unsigned char)Color[2];
	}
}

void SurfelGraph::DisplayHardEdges(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iSurfel,
	unsigned char *Color)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;

	int iPt, iPt_;
	MeshEdgePtr *pEdgePtr;
	MeshEdge *pEdge;
	int iSurfel_;
	//Surfel *pSurfel_;
	bool bEdge;
	Point *pPt;

	while (pPtIdx)
	{
		iPt = pPtIdx->Idx;

		pPt = pMesh->NodeArray.Element + iPt;

		bEdge = false;

		pEdgePtr = pPt->EdgeList.pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

			iSurfel_ = surfelMap[iPt_];

			if (iSurfel_ != iSurfel)
			{
				bEdge = true;

				break;
			}

			pEdgePtr = pEdgePtr->pNext;
		}

		if (bEdge)
			pVisualizer->PaintPoint(iPt, pMesh->pPolygonData, Color);
		//else
		//	int debug = 0;

		pPtIdx = pPtIdx->pNext;
	}
}

void SurfelGraph::Display(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iSelectedSurfel,
	unsigned char *SelectionColor,
	int *ColorScale,
	unsigned char *ColorOffset)
{
	//unsigned char HardEdgeColor[3];

	//HardEdgeColor[0] = 0;
	//HardEdgeColor[1] = 255;
	//HardEdgeColor[2] = 0;

	int iSurfel;
	Surfel *pSurfel;
	unsigned char color[3];
	unsigned char *color_;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		color_ = nodeColor + 3 * iSurfel;

		if (iSurfel == iSelectedSurfel)
			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, SelectionColor);
		else
		{
			RVLCOPY3VECTOR(color_, color);

			if (ColorScale)
				RVLSCALECOLOR2(color, ColorScale, color);

			if (ColorOffset)
			{
				RVLSUM3VECTORS(color, ColorOffset, color);
			}

			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);
		}
		
		//DisplayHardEdges(pVisualizer, pMesh, iSurfel, HardEdgeColor);
	}
}

//VTK Render window right mouse button press callback
void SURFEL::MouseRButtonDown(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor*>(caller);
	SURFEL::DisplayCallbackData *pData = (SURFEL::DisplayCallbackData *)clientdata;

	Mesh *pMesh = pData->pMesh;

	vtkSmartPointer<vtkPolyData> pd = pMesh->pPolygonData;

	vtkSmartPointer<vtkFloatArray> pointData;
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;
	vtkSmartPointer<vtkFloatArray> normalPointData;
	int noPts = 0;
	//FetchVTKPointData(pd, pointData, rgbPointData, normalPointData, noPts);

	pData->pVisualizer->pointPicker->Pick(interactor->GetEventPosition()[0], interactor->GetEventPosition()[1], 0, 
		interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	vtkIdType selectedPoint = pData->pVisualizer->pointPicker->GetPointId();

	if (selectedPoint >= 0)
	{
		int iSurfel = pData->pSurfels->surfelMap[selectedPoint];

		bool bSelection = false;

		if (pData->mouseRButtonDownUserFunction)
		{
			if (iSurfel >= 0)
				bSelection |= pData->mouseRButtonDownUserFunction(pMesh, pData->pSurfels, (int)selectedPoint, iSurfel, pData->vpUserFunctionData);
		}

		if (!bSelection)
		{
			if (pData->iSelectedSurfel >= 0 && (pData->iSelection == 1 && pData->iSelectedSurfel != iSurfel))
				pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[pData->iSelectedSurfel].PtList), pMesh->pPolygonData,
				pData->pSurfels->GetColor(pData->iSelectedSurfel));

			if (pData->iSelectedSurfel2 >= 0 && ((pData->iSelection == 1 || (pData->iSelection == 2 && pData->iSelectedSurfel2 != iSurfel))))
				pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[pData->iSelectedSurfel2].PtList), pMesh->pPolygonData,
				pData->pSurfels->GetColor(pData->iSelectedSurfel2));

			//pData->pSurfels->DisplaySurfelBoundary(pData->pVisualizer, pMesh, iSurfel, pData->SelectionColor);

			if (pData->iSelection == 1)
			{
				if (iSurfel >= 0)
					pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel].PtList), pMesh->pPolygonData, pData->SelectionColor);

				pData->iSelectedSurfel = iSurfel;

				pData->iSelectedSurfel2 = -1;
			}
			else// if (pData->iSelection == 2)
			{
				unsigned char SelectionColor2[3];

				RVLSCALECOLOR(pData->SelectionColor, 75, SelectionColor2);

				pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel].PtList), pMesh->pPolygonData, SelectionColor2);

				pData->iSelectedSurfel2 = iSurfel;

				pData->iSelection = 1;
			}

			bSelection = true;
		}

		if (bSelection)
		{
			pd->Modified();

			pData->pSurfels->PrintData(pData->pVisualizer, pMesh, selectedPoint, iSurfel);

			interactor->GetRenderWindow()->Render();
		}
	}
}

//VTK Render window key press callback
void SURFEL::KeyPressCallback(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor*>(caller);
	SURFEL::DisplayCallbackData *pData = (SURFEL::DisplayCallbackData *)clientdata;

	if (!pData->bFirstKey)
	{
		pData->bFirstKey = true;

		return;
	}

	pData->bFirstKey = false;

	Mesh *pMesh = pData->pMesh;

	vtkSmartPointer<vtkPolyData> pd = pMesh->pPolygonData;

	vtkSmartPointer<vtkFloatArray> pointData;
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;
	vtkSmartPointer<vtkFloatArray> normalPointData;
	int noPts = 0;

	std::string keySym = "";
	keySym = interactor->GetKeySym();

	PlanarSurfelDetector *pDetector = (PlanarSurfelDetector *)(pData->vpDetector);

	bool bUpdateDisplay = false;
	bool bDisplayBoundary = false;
	bool bDefineBoundary = false;
	bool bDisplaySoftEdges = false;

	if (keySym == "2")
	{
		pData->iSelection = 2;
	}
	else if(keySym == "b")
	{
		if (pData->mode != RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR)
		{
			int colorScale[3];

			RVLSET3VECTOR(colorScale, 0, 0, 75);

			unsigned char colorOffset[3];

			RVLSET3VECTOR(colorOffset, 0, 0, 64);

			pData->pSurfels->Display(pData->pVisualizer, pMesh, -1, NULL, colorScale, colorOffset);

			bDefineBoundary = true;

			bUpdateDisplay = true;
		}
	}
	else if (keySym == "c")
	{
		bDisplayBoundary = true;

		bUpdateDisplay = true;
	}
	else if (keySym == "g")
	{
		bDisplaySoftEdges = !bDisplaySoftEdges;

		if (bDisplaySoftEdges)
			pDetector->DisplaySoftEdges(pData->pVisualizer, pMesh, pData->pSurfels, pData->SelectionColor);
		else
			pData->pSurfels->Display(pData->pVisualizer, pMesh);

		bUpdateDisplay = true;
	}
	else if (keySym == "n")
	{
		if (pData->pVisualizer->bNormals)
		{
			pData->pVisualizer->bNormalsVisible = !pData->pVisualizer->bNormalsVisible;

			if (pData->pVisualizer->bNormalsVisible)
				pData->pVisualizer->normals->VisibilityOn();
			else
				pData->pVisualizer->normals->VisibilityOff();

			bUpdateDisplay = true;
		}
	}
	else if (keySym == "p")
	{
		if (pData->iSelectedSurfel >= 0)
		{
			pDetector->DefinePolygon(pMesh, pData->pSurfels, pData->iSelectedSurfel);

			pData->pSurfels->Display(pData->pVisualizer, pMesh, pData->iSelectedSurfel, pData->SelectionColor);

			bUpdateDisplay = true;
		}
	}
	else if (keySym == "s")
	{
		if (pData->mode == RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR)
		{
			pData->pSurfels->Display(pData->pVisualizer, pMesh, pData->iSelectedSurfel, pData->SelectionColor);

			bUpdateDisplay = true;
		}

		pData->mode = RVLSURFEL_DISPLAY_MODE_SURFELS;
	}
	else if (keySym == "v")
	{
		pData->bVertices = !pData->bVertices;

		if (pData->bVertices)
			pData->vertices->VisibilityOn();
		else
			pData->vertices->VisibilityOff();

		bUpdateDisplay = true;
	}
	else if (keySym == "F1")
	{
		std::cout << "Enter surfel index: ";

		std::string line;

		std::getline(std::cin, line);

		int iSelectedSurfel;

		sscanf(line.data(), "%d", &iSelectedSurfel);

		if (iSelectedSurfel >= 0 && iSelectedSurfel < pData->pSurfels->NodeArray.n)
		{
			if (pData->iSelectedSurfel >= 0 && pData->iSelectedSurfel < pData->pSurfels->NodeArray.n)
				pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[pData->iSelectedSurfel].PtList), pMesh->pPolygonData,
				pData->pSurfels->GetColor(pData->iSelectedSurfel));

			pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSelectedSurfel].PtList), pMesh->pPolygonData, pData->SelectionColor);

			pData->iSelectedSurfel = iSelectedSurfel;

			bUpdateDisplay = true;
		}
	}
#ifdef RVLMESH_BOUNDARY_DEBUG
	else if (keySym == "plus")
	{
		if (pData->mode == RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR)
		{
			if (pData->iSelectedSurfel >= 0 && pData->iSelectedSurfel2 >= 0)
			{
				pMesh->debugState++;

				bDefineBoundary = true;

				bUpdateDisplay = true;
			}
		}
		else
		{
			if (pData->iSelectedSurfel >= 0)
			{
				pMesh->debugState++;

				bDisplayBoundary = true;

				bUpdateDisplay = true;
			}
		}
	}
#endif

	if (pData->keyPressUserFunction)
		bUpdateDisplay |= pData->keyPressUserFunction(pMesh, pData->pSurfels, keySym, pData->vpUserFunctionData);

	if (bDefineBoundary)
	{	
		QList<QLIST::Index> G;

		int iSurfel = pData->iSelectedSurfel;
		int iSurfel_ = pData->iSelectedSurfel2;

		pDetector->DefineBoundaryTest(pMesh, pData->pSurfels, iSurfel, iSurfel_, G);

		unsigned char white[3];

		RVLSET3VECTOR(white, 255, 255, 255);

		unsigned char green[3];

		RVLSET3VECTOR(green, 0, 255, 0);

		unsigned char black[3];

		RVLSET3VECTOR(black, 0, 0, 0);

		pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel].PtList), pMesh->pPolygonData, white);

		//pData->pVisualizer->PaintPointSet(&G, pMesh->pPolygonData, green);

		pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel_].PtList), pMesh->pPolygonData, black);

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
		unsigned char red[3];

		RVLSET3VECTOR(red, 255, 0, 0);

		pData->pVisualizer->PaintPointSet(&(pDetector->debugPtArray), pMesh->pPolygonData, red);
#endif

//#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
//		pData->pVisualizer->PaintPointSet(&(pDetector->debugPtArray), pMesh->pPolygonData, green);
//#endif

		pData->mode = RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR;
	}
	
	if (bDisplayBoundary)
	{
		if (pData->iSelectedSurfel >= 0)
		{
			pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[pData->iSelectedSurfel].PtList), pMesh->pPolygonData,
				pData->pSurfels->GetColor(pData->iSelectedSurfel));

			FILE *fpPts = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugPoints.txt", "w");
			FILE *fpEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "w");

			pData->pSurfels->Save(pData->iSelectedSurfel, pMesh, fpPts, fpEdges);

			fclose(fpPts);
			fclose(fpEdges);

			pData->pSurfels->DisplaySurfelBoundary(pData->pVisualizer, pMesh, pData->iSelectedSurfel, pData->SelectionColor);
		}
	}

	if (bUpdateDisplay)
	{
		pd->Modified();

		pData->pSurfels->PrintData(pData->pVisualizer, pMesh, -1, pData->iSelectedSurfel);

		interactor->GetRenderWindow()->Render();
	}
}

void SurfelGraph::PrintData(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iVertex,
	int iSurfel)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	char str[2000], str2[500];

	if (iVertex >= 0)
	{
		Point *pPt = pMesh->NodeArray.Element + iVertex;

		sprintf(str, "Point %d\nP=(%f, %f, %f)\nN=(%f, %f, %f)\nRGB=(%d, %d, %d)",
			iVertex, pPt->P[0], pPt->P[1], pPt->P[2], pPt->N[0], pPt->N[1], pPt->N[2], pPt->RGB[0], pPt->RGB[1], pPt->RGB[2]);
	}
	else
		str[0] = 0;

	if (iSurfel >= 0)
	{
		sprintf(str2, "\nSurfel %d\nP=(%f, %f, %f)\nN=(%f, %f, %f)\nRGB=(%d, %d, %d)\nsize=%d",
			iSurfel,
			pSurfel->P[0], pSurfel->P[1], pSurfel->P[2],
			pSurfel->N[0], pSurfel->N[1], pSurfel->N[2],
			pSurfel->RGB[0], pSurfel->RGB[1], pSurfel->RGB[2],
			pSurfel->size);

		strcat(str, str2);
	}

	//// Print indices of the adjacent surfels

	//strcat(str, "\nNeighbors:\n");

	//VertexEdgePtr *pEdgePtr = pSurfel->EdgeList.pFirst;

	//Surfel *pSurfel_ = pSurfel;

	//Surfel *pSurfel__;
	//MeshEdge *pEdge;
	//int iSurfel__;
	//float eZ_, eZ__, eXY;
	//float N_[3], N__[3], Z[3];
	//float V3Tmp[3];
	//int RGB_[3], RGB__[3], dRGB[3], eRGB;
	//float fTmp;

	//RVLCONVTOINT3(pSurfel_->RGB, RGB_);

	//while (pEdgePtr)	// for each neighbor of iNode
	//{
	//	RVLSEGMENTATION_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iSurfel__);

	//	pSurfel__ = surfelArray.Element + iSurfel__;

	//	RVLCONVTOINT3(pSurfel__->RGB, RGB__);

	//	RVLDIF3VECTORS(RGB__, RGB_, dRGB);

	//	eRGB = RVLDOTPRODUCT3(dRGB, dRGB);

	//	RVLDIF3VECTORS(pSurfel__->P, pSurfel_->P, Z);

	//	RVLNORM3(Z, fTmp);

	//	eZ_ = RVLDOTPRODUCT3(Z, pSurfel_->N);

	//	eZ__ = RVLDOTPRODUCT3(Z, pSurfel__->N);

	//	RVLSCALE3VECTOR(Z, eZ_, V3Tmp);
	//	RVLDIF3VECTORS(pSurfel_->N, V3Tmp, N_);
	//	RVLNORM3(N_, fTmp);
	//	RVLSCALE3VECTOR(Z, eZ__, V3Tmp);
	//	RVLDIF3VECTORS(pSurfel__->N, V3Tmp, N__);
	//	RVLNORM3(N__, fTmp);
	//	RVLDIF3VECTORS(N__, N_, V3Tmp);

	//	eXY = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

	//	sprintf(str2, "%d: e=(%f, %f, %f, %f)\n", iSurfel__, eZ_, eZ__, sqrt(eXY), sqrt((float)eRGB));

	//	strcat(str, str2);

	//	pEdgePtr = pEdgePtr->pNext;
	//}	// for each neighbor of iNode

	// Put the text on the screen

	pVisualizer->text->SetText(2, str);
}


unsigned char * SurfelGraph::GetColor(int iSurfel)
{
	return nodeColor + 3 * iSurfel;
}


void SurfelGraph::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	void *vpDetector)
{
	pVisualizer->SetMesh(pMesh);

	DisplayData.pMesh = pMesh;
	DisplayData.pSurfels = this;
	DisplayData.pVisualizer = pVisualizer;
	DisplayData.vpDetector = vpDetector;
	RVLSET3VECTOR(DisplayData.SelectionColor, 0, 255, 0);
	DisplayData.mode = RVLSURFEL_DISPLAY_MODE_SURFELS;
	DisplayData.iSelectedSurfel = DisplayData.iSelectedSurfel2 = -1;
	DisplayData.iSelection = 1;
	DisplayData.bVertices = false;
	DisplayData.bFirstKey = true;

	pVisualizer->SetMouseRButtonDownCallback(SURFEL::MouseRButtonDown, &DisplayData);
	pVisualizer->SetKeyPressCallback(SURFEL::KeyPressCallback, &DisplayData);
}


void SurfelGraph::DisplaySurfelBoundary(
	Visualizer *pVisualizer, 
	Mesh * pMesh, 
	int iSurfel,
	unsigned char *Color)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	//QList<QLIST::Index2> *pSurfelPtList = &(pSurfel->PtList);

	//pSurfel->BoundaryArray.Element = new Array <MeshEdgePtr *>[nMeshVertices];

	//MeshEdgePtr **boundaryMem = new MeshEdgePtr *[pMesh->EdgeArray.n];

	//MeshEdgePtr **pBoundaryMem = boundaryMem;

	//pMesh->Boundary(pSurfelPtList, surfelMap, pSurfel->BoundaryArray, pBoundaryMem, edgeMarkMap);

	Array<int> boundaryPtArray;

	boundaryPtArray.Element = new int[nMeshVertices];
	boundaryPtArray.n = 0;

	int iBoundary, iPointEdge; 
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr;

	for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
	{
		pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

		for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
		{
			pEdgePtr = pBoundary->Element[iPointEdge];

			boundaryPtArray.Element[boundaryPtArray.n++] = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);
		}
	}

	pVisualizer->PaintPointSet(&boundaryPtArray, pMesh->pPolygonData, Color);

	//delete[] pSurfel->BoundaryArray.Element;
	//delete[] boundaryMem;
	delete[] boundaryPtArray.Element;

	//QList<QLIST::Index> Boundary;

	//QLIST::Index *BoundaryMem = new QLIST::Index[pMesh->NodeArray.n];

	//pMesh->Boundary(pSurfelPtList, surfelMap, &Boundary, BoundaryMem);

	//pVisualizer->PaintPointSet(&Boundary, pMesh->pPolygonData, Color);

	//delete[] BoundaryMem;
}

void SurfelGraph::DisplayEdgeFeatures()
{
	Visualizer *pVisualizer = DisplayData.pVisualizer;

	// Create the polydata where we will store all the geometric data
	DisplayData.edgeFeaturesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

	//// Create a cell array to store the lines in and add the lines to it
	vtkSmartPointer<vtkCellArray> polyLines = vtkSmartPointer<vtkCellArray>::New();

#ifdef NEVER
	// Create colors.
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();	

	colors->SetNumberOfComponents(3);

	unsigned char red[3] = { 255, 0, 0 };

	colors->InsertNextTupleValue(red);
#endif
	///

	// Determine the total number of edge features.

	int nEdgeFeatures = 0;

	int iFeature;

	for (iFeature = 0; iFeature < NodeArray.n; iFeature++)
		if (NodeArray.Element[iFeature].bEdge)
			nEdgeFeatures++;

	// Allocate polyline pointers.

	vtkSmartPointer<vtkPolyLine> *polyLine = new vtkSmartPointer<vtkPolyLine>[nEdgeFeatures];

	//

	int iEdgeFeature = 0;

	int i;
	Surfel *pFeature;
	float *N, *V, *P1;
	float P2[3], P3[3], P4[3], U[3], VTmp[3];
	double P[3];
	float fTmp;

	for (iFeature = 0; iFeature < NodeArray.n; iFeature++)
	{
		pFeature = NodeArray.Element + iFeature;

		if (!pFeature->bEdge)
			continue;

		// N <- edge feature normal

		N = pFeature->N;

		// V <- unit vector in edge direction.

		V = pFeature->V;

		// P1 <- the first endpoint of the edge feature

		P1 = pFeature->P;

		// P2 <- P1 + pFeature->physicalSize * V

		RVLSCALE3VECTOR(V, pFeature->physicalSize, VTmp);

		RVLSUM3VECTORS(P1, VTmp, P2);

		// U <- DisplayData.edgeFeatureDepth * unit(V x N)

		RVLCROSSPRODUCT3(V, N, U);

		RVLNORM3(U, fTmp);

		RVLSCALE3VECTOR(U, DisplayData.edgeFeatureDepth, U);

		// P3 <- P1 + U

		RVLSUM3VECTORS(P1, U, P3);

		// P4 <- P2 + U

		RVLSUM3VECTORS(P2, U, P4);

		// Add P1, P2, P3 and P4 to pts

		RVLCOPY3VECTOR(P1, P);

		pts->InsertNextPoint(P);

		RVLCOPY3VECTOR(P2, P);

		pts->InsertNextPoint(P);

		RVLCOPY3VECTOR(P4, P);

		pts->InsertNextPoint(P);

		RVLCOPY3VECTOR(P3, P);

		pts->InsertNextPoint(P);

		// Create rectangle P1-P2-P3-P4.

		polyLine[iEdgeFeature] = vtkSmartPointer<vtkPolyLine>::New();

		polyLine[iEdgeFeature]->GetPointIds()->SetNumberOfIds(5);

		for (i = 0; i < 4; i++)
			polyLine[iEdgeFeature]->GetPointIds()->SetId(i, 4 * iEdgeFeature + i);

		polyLine[iEdgeFeature]->GetPointIds()->SetId(4, 4 * iEdgeFeature);

		// Add polyline to polyLines.

		polyLines->InsertNextCell(polyLine[iEdgeFeature]);

		// Assign color to polyline.

		//colors->InsertNextTupleValue(red);

		iEdgeFeature++;
	}

	// Add the points to the polydata container
	DisplayData.edgeFeaturesPolyData->SetPoints(pts);

	// Add the lines to the polydata container
	DisplayData.edgeFeaturesPolyData->SetLines(polyLines);

	// Color the lines.
	//DisplayData.edgeFeaturesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(DisplayData.edgeFeaturesPolyData);

	DisplayData.edgeFeatures = vtkSmartPointer<vtkActor>::New();
	DisplayData.edgeFeatures->SetMapper(mapper);

	pVisualizer->renderer->AddActor(DisplayData.edgeFeatures);
}

void SurfelGraph::DisplayVertices()
{
	double lineLength = DisplayData.normalLen;

	Mesh *pMesh = DisplayData.pMesh;
	Visualizer *pVisualizer = DisplayData.pVisualizer;

	// Create the polydata where we will store all the geometric data
	linesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iLine = 0;

	double P0[3], P[3], V[3];
	Surfel *pSurfel;
	int iSurfel;

	Vertex *pVertex = vertexList.pFirst;

	while (pVertex)
	{
		RVLCOPY3VECTOR(pVertex->P, P0);

		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			pSurfel = NodeArray.Element + pVertex->iSurfelArray.Element[iSurfel];

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
	DisplayData.vertices = vtkSmartPointer<vtkActor>::New();
	DisplayData.vertices->SetMapper(mapper);

	pVisualizer->renderer->AddActor(DisplayData.vertices);

	delete[] line;
}

void SurfelGraph::UpdateVertexDisplayLines()
{
	linesPolyData->Modified();
}

void SurfelGraph::PaintVertices(
	Array<int> *pVertexArray,
	unsigned char *color)
{
	int iVertex;
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(linesPolyData->GetCellData()->GetScalars());

	int i, j;

	for (i = 0; i < pVertexArray->n; i++)
	{
		iVertex = pVertexArray->Element[i];

		for (j = 0; j < vertexDisplayLineArray.Element[iVertex].n; j++)
			rgbPointData->SetTupleValue(vertexDisplayLineArray.Element[iVertex].Element[j], color);
	}
}

void SurfelGraph::Save(
	int iSurfel,
	Mesh *pMesh,
	FILE *fpPoints,
	FILE *fpEdges)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	int iPt, iPt_;
	Point *pPt;
	MeshEdge *pEdge;
	MeshEdgePtr *pEdgePtr;

	QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;

	while (pPtIdx)
	{
		iPt = pPtIdx->Idx;

		pPt = pMesh->NodeArray.Element + iPt;

		fprintf(fpPoints, "%d\t%f\t%f\t%f\t%d\n", iPt, pPt->P[0], pPt->P[1], pPt->P[2], 1);

		pEdgePtr = pPt->EdgeList.pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

			if (surfelMap[iPt_] == iSurfel)
				if (iPt < iPt_)
					fprintf(fpEdges, "%d\t%d\t%d\t1\n", pEdge->idx, iPt, iPt_);

			pEdgePtr = pEdgePtr->pNext;
		}

		pPtIdx = pPtIdx->pNext;
	}
}

void SurfelGraph::SaveSurfel(
	FILE *fp,
	int iSurfel)
{
	fwrite(&iSurfel, sizeof(int), 1, fp);
	
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	fwrite(pSurfel->N, sizeof(float), 3, fp);
	fwrite(&(pSurfel->d), sizeof(float), 1, fp);
	fwrite(pSurfel->P, sizeof(float), 3, fp);
	fwrite(pSurfel->RGB, sizeof(int), 3, fp);
}

void SurfelGraph::LoadSurfel(
	FILE *fp,
	int iSurfel)
{
	fread(&iSurfel, sizeof(int), 1, fp);

	Surfel *pSurfel = NodeArray.Element + iSurfel;

	fread(pSurfel->N, sizeof(float), 3, fp);
	fread(&(pSurfel->d), sizeof(float), 1, fp);
	fread(pSurfel->P, sizeof(float), 3, fp);
	fread(pSurfel->RGB, sizeof(int), 3, fp);
}

void SurfelGraph::Save(
	FILE *fp,
	char *meshFileName,
	void *vpDetector)
{
	char header[] = "RVL::SurfelGraph 000";

	int headerLength = strlen(header);

	sprintf(header + headerLength - 3, "%03d", RVLSURFEL_VERSION_0);

	fwrite(header, sizeof(char), headerLength + 1, fp);

	fwrite(meshFileName, sizeof(char), strlen(meshFileName) + 1, fp);

	PlanarSurfelDetector *pDetector = (PlanarSurfelDetector *)vpDetector;

	pDetector->Save(fp);

	fwrite(&nMeshVertices, sizeof(int), 1, fp);
	fwrite(surfelMap, sizeof(int), nMeshVertices, fp);

	fwrite(&(NodeArray.n), sizeof(int), 1, fp);

	int iSurfel;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
		if (NodeArray.Element[iSurfel].size > 0)
			SaveSurfel(fp, iSurfel);
}

