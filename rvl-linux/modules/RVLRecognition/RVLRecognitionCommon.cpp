#include <vector>
#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkLine.h>
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "RVLRecognitionCommon.h"

using namespace RVL;
using namespace RECOG;

void RECOG::CreateDilatedDepthImage(
	Mesh *pMesh,
	cv::Mat &depth)
{
	// Generate scene depth
	double point[3];
	int u, v;

	depth.setTo(cv::Scalar(0));
	// memset(depth.data, 0, 640 * 480 * sizeof(ushort));

	// cv::Mat depth(480, 640, CV_16UC1, cv::Scalar::all(0));
	for (int i = 0; i < pMesh->pPolygonData->GetNumberOfPoints(); i++)
	{
		pMesh->pPolygonData->GetPoint(i, point);
		if ((point[0] == 0) && (point[1] == 0) && (point[2] == 0))
			continue;
		v = floor(float(i) / 640);
		u = i - v * 640;
		depth.at<uint16_t>(v, u) = (uint16_t)(point[2] * 1000); // in milimeters
	}
	// Postprocessing
	for (int y = 0; y < depth.rows; y++)
	{
		for (int x = 0; x < depth.cols; x++)
		{
			if (depth.at<uint16_t>(y, x) == 0)
				depth.at<uint16_t>(y, x) = 10000; // in milimeters
		}
	}
#ifndef RVLVERSION_171125
	cv::Mat elementE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(17, 17));
	cv::erode(depth, depth, elementE);
#endif
	// Set PSGM depth
	// depthImg = (unsigned short*)depth.data; //Vidovic 21.07.2017. depth.data is deleted after this function finish

	// Vidovic
	// unsigned short *depthTMP;
	// depthTMP = (unsigned short*)depth.data;
	// int a = sizeof(unsigned short);
	// int s = sizeof(depthImg);
	// memcpy(&depthImg, &depthTMP, depth.rows * depth.cols * sizeof(unsigned short));
	////END Vidovic
}

bool RECOG::InitZBuffer(
	Mesh *pMesh,
	int sceneSamplingResolution,
	Array2D<Point> &ZBuffer,
	Array<int> &ZBufferActivePtArray,
	int *&subImageMap)
{
	if (!pMesh->bOrganizedPC)
		return false;

	int w = (pMesh->width / sceneSamplingResolution + (pMesh->width % sceneSamplingResolution > 0 ? 1 : 0));
	int h = (pMesh->height / sceneSamplingResolution + (pMesh->height % sceneSamplingResolution > 0 ? 1 : 0));

	int n = w * h;

	if (ZBuffer.Element)
	{
		if (ZBuffer.w * ZBuffer.h < n)
		{
			delete[] ZBuffer.Element;

			ZBuffer.Element = new Point[n];

			RVL_DELETE_ARRAY(ZBufferActivePtArray.Element);

			ZBufferActivePtArray.Element = new int[n];

			RVL_DELETE_ARRAY(subImageMap);

			subImageMap = new int[n];
		}
	}
	else
	{
		ZBuffer.Element = new Point[n];

		RVL_DELETE_ARRAY(ZBufferActivePtArray.Element);

		ZBufferActivePtArray.Element = new int[n];

		RVL_DELETE_ARRAY(subImageMap);

		subImageMap = new int[n];
	}

	ZBuffer.w = w;
	ZBuffer.h = h;

	int iPix, u, v;

	for (iPix = 0; iPix < n; iPix++)
	{
		u = iPix % w;
		v = iPix / w;

		if (u == 0 || u == w - 1 || v == 0 || v == h - 1)
		{
			ZBuffer.Element[iPix].bValid = true;
			ZBuffer.Element[iPix].P[2] = 0.0f;
		}
		else
			ZBuffer.Element[iPix].bValid = false;

		subImageMap[iPix] = sceneSamplingResolution * (u + v * pMesh->width);
	}

	ZBufferActivePtArray.n = 0;

	return true;
}

float RECOG::EvaluateHypothesis2(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	bool *surfelMask,
	Array2D<Point> ZBuffer,
	Array<int> ZBufferActivePtArray,
	int *subImageMap,
	int *image3x3Neighborhood,
	float maxe,
	float transparencyDepthThr,
	int &nTransparentPts,
	int *SMCorrespondence,
	RECOG::SceneFittingError *errorRecord)
{
	float maxe2 = maxe * maxe;

	int *surfelMap = pSurfels->surfelMap;

	Point *PtArray = pMesh->NodeArray.Element;

	float score = 0.0f;

	nTransparentPts = 0;

	int i, j, iPix, iMPt, iSPt, iMPt_, iClosestPt;
	Point *pSPt, *pMPt;
	float dP[3];
	float e2, mine2;
	float csN;
	float ePlane;
	bool bTransparent;

	for (i = 0; i < ZBufferActivePtArray.n; i++)
	{
		iMPt = ZBufferActivePtArray.Element[i];

		pMPt = ZBuffer.Element + iMPt;

		mine2 = maxe2;

		iClosestPt = -1;

		for (j = 0; j < 9; j++)
		{
			iMPt_ = iMPt + image3x3Neighborhood[j];

			iSPt = subImageMap[iMPt_];

			pSPt = PtArray + iSPt;

			if (pSPt->N[0] != pSPt->N[0])
				continue;

			if (RVLDOTPRODUCT3(pSPt->N, pSPt->N) < 0.5f)
				continue;

			if (!surfelMask[surfelMap[iSPt]])
				continue;

			RVLDIF3VECTORS(pSPt->P, pMPt->P, dP);

			e2 = RVLDOTPRODUCT3(dP, dP);

			if (e2 < mine2)
			{
				mine2 = e2;

				iClosestPt = iSPt;
			}
		}

		if (iClosestPt >= 0)
		{
			pSPt = PtArray + iClosestPt;

			csN = RVLDOTPRODUCT3(pSPt->N, pMPt->N);

			if (csN < 0.0f)
				csN = 0.0f;

			score += ((1.0f - sqrt(mine2) / maxe) * csN);

			if (SMCorrespondence)
				SMCorrespondence[i] = iClosestPt;

			if (errorRecord)
			{
				errorRecord[i].eP = sqrt(mine2);
				errorRecord[i].csN = csN;
				errorRecord[i].dz = pSPt->P[2] - pMPt->P[2];
			}
		}
		else
		{
			bTransparent = false;

			iSPt = subImageMap[iMPt];

			pSPt = PtArray + iSPt;

			if (pSPt->P[2] > pMPt->P[2] + transparencyDepthThr)
			{
				// RVLDIF3VECTORS(pSPt->P, pMPt->P, dP);

				// ePlane = RVLDOTPRODUCT3(pMPt->N, dP);

				// if (ePlane < 0.02f)
				{
					bTransparent = true;

					nTransparentPts++;

					if (SMCorrespondence)
						SMCorrespondence[i] = -2;
				}
			}

			if (SMCorrespondence && !bTransparent)
				SMCorrespondence[i] = -1;

			if (errorRecord)
			{
				errorRecord[i].eP = maxe;
				errorRecord[i].csN = 0.0f;
				errorRecord[i].dz = pSPt->P[2] - pMPt->P[2];
			}
		}
	} // for every point in ZBufferActivePtArray

	return score;
}

float RECOG::EvaluateHypothesis3(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	bool *surfelMask,
	Array2D<Point> ZBuffer,
	Array<int> ZBufferActivePtArray,
	int *subImageMap,
	int *image3x3Neighborhood,
	float maxe,
	float transparencyDepthThr,
	int &nTransparentPts,
	int *SMCorrespondence,
	RECOG::SceneFittingError *errorRecord)
{
	int *surfelMap = pSurfels->surfelMap;
	Point *PtArray = pMesh->NodeArray.Element;
	float score = 0.0f;
	nTransparentPts = 0;
	int i, j, iPix, iMPt, iSPt, iMPt_, iClosestPt;
	Point *pSPt, *pMPt;
	float dP[3];
	float e, mine;
	float csN;
	float ePlane;
	bool bTransparent;
	int iSurfel;
	Surfel *pSurfel;
	// Visualizer visualizer;
	// visualizer.Create();
	// visualizer.SetMesh(pMesh);
	// Array<Point> visPts;
	// visPts.n = 3;
	// Point visPtsMem[3];
	// visPts.Element = visPtsMem;
	// iMPt = ZBufferActivePtArray.Element[1];
	// pMPt = ZBuffer.Element + iMPt;
	// visPts.Element[0] = *pMPt;
	// iSPt = subImageMap[iMPt];
	// pSPt = PtArray + iSPt;
	// visPts.Element[1] = *pSPt;
	// iSurfel = surfelMap[iSPt];
	// pSurfel = pSurfels->NodeArray.Element + iSurfel;
	// e = RVLDOTPRODUCT3(pSurfel->N, pMPt->P) - pSurfel->d;
	// float* PS = visPts.Element[2].P;
	// RVLSCALE3VECTOR(pSurfel->N, e, PS);
	// RVLDIF3VECTORS(pMPt->P, PS, PS);
	// uchar color[] = {255, 0, 0, 0, 255, 0, 0, 0, 255};
	// visualizer.DisplayPointSet<float, Point>(visPts, color, 4, true);
	// visualizer.Run();
	for (i = 0; i < ZBufferActivePtArray.n; i++)
	{
		iMPt = ZBufferActivePtArray.Element[i];
		pMPt = ZBuffer.Element + iMPt;
		mine = maxe;
		iClosestPt = -1;
		for (j = 0; j < 9; j++)
		{
			iMPt_ = iMPt + image3x3Neighborhood[j];
			iSPt = subImageMap[iMPt_];
			pSPt = PtArray + iSPt;
			iSurfel = surfelMap[iSPt];
			if (iSurfel < 0 || iSurfel > pSurfels->NodeArray.n)
				continue;
			pSurfel = pSurfels->NodeArray.Element + iSurfel;
			// if (RVLDOTPRODUCT3(pSurfel->N, pMPt->N) < 0.5f)
			csN = RVLDOTPRODUCT3(pSPt->N, pMPt->N);
			if (csN < 0.5f || csN != csN)
				continue;
			if (!surfelMask[iSurfel])
				continue;
			e = RVLDOTPRODUCT3(pSurfel->N, pMPt->P) - pSurfel->d;
			if (e < 0.0f)
				e = -e;
			if (e < mine)
			{
				mine = e;
				iClosestPt = iSPt;
			}
		}
		if (iClosestPt >= 0)
		{
			iSurfel = surfelMap[iClosestPt];
			pSurfel = pSurfels->NodeArray.Element + iSurfel;
			// csN = RVLDOTPRODUCT3(pSurfel->N, pMPt->N);
			pSPt = PtArray + iClosestPt;
			csN = RVLDOTPRODUCT3(pSPt->N, pMPt->N);
			score += ((1.0f - mine / maxe) * csN);
			if (SMCorrespondence)
				SMCorrespondence[i] = iClosestPt;
			if (errorRecord)
			{
				errorRecord[i].eP = mine;
				errorRecord[i].csN = csN;
				errorRecord[i].dz = pSPt->P[2] - pMPt->P[2];
			}
		}
		else
		{
			bTransparent = false;
			iSPt = subImageMap[iMPt];
			pSPt = PtArray + iSPt;
			if (pSPt->P[2] > pMPt->P[2] + transparencyDepthThr)
			{
				bTransparent = true;
				nTransparentPts++;
				if (SMCorrespondence)
					SMCorrespondence[i] = -2;
			}
			if (SMCorrespondence && !bTransparent)
				SMCorrespondence[i] = -1;
			if (errorRecord)
			{
				errorRecord[i].eP = maxe;
				errorRecord[i].csN = 0.0f;
				errorRecord[i].dz = pSPt->P[2] - pMPt->P[2];
			}
		}
	} // for every point in ZBufferActivePtArray

	return score;
}

void RECOG::DisplayHypothesisEvaluation(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	Array2D<Point> ZBuffer,
	Array<int> ZBufferActivePtArray,
	int *subImageMap,
	int *SMCorrespondence,
	int nTransparentPts,
	vtkSmartPointer<vtkActor> *actor)
{
	int nPts = ZBufferActivePtArray.n;

	Point *PC = new Point[2 * nPts];

	Array<Point> MatchedPC;

	MatchedPC.Element = PC;
	MatchedPC.n = 0;

	Array<Point> TransparentPC;

	TransparentPC.Element = PC + nPts - nTransparentPts;
	TransparentPC.n = 0;

	Array<Point> SPC;

	SPC.Element = PC + nPts;
	SPC.n = 0;

	Point *PtArray = pMesh->NodeArray.Element;

	int i, iMPt, iSPt;
	Point *pMPt, *pSPt;

	for (i = 0; i < nPts; i++)
	{
		iMPt = ZBufferActivePtArray.Element[i];

		pMPt = ZBuffer.Element + iMPt;

		if (SMCorrespondence[i] >= 0)
		{
			MatchedPC.Element[MatchedPC.n++] = *pMPt;

			pSPt = PtArray + SMCorrespondence[i];
		}
		else
		{
			iSPt = subImageMap[iMPt];

			pSPt = PtArray + iSPt;

			SPC.Element[SPC.n++] = *pSPt;

			if (SMCorrespondence[i] == -2)
				TransparentPC.Element[TransparentPC.n++] = *pMPt;
		}
	}

	unsigned char color[3];

	RVLSET3VECTOR(color, 0, 255, 0);

	actor[0] = pVisualizer->DisplayPointSet<float, Point>(MatchedPC, color, 4.0f);

	RVLSET3VECTOR(color, 0, 0, 255);

	actor[1] = pVisualizer->DisplayPointSet<float, Point>(SPC, color, 4.0f);

	RVLSET3VECTOR(color, 255, 0, 0);

	actor[2] = pVisualizer->DisplayPointSet<float, Point>(TransparentPC, color, 4.0f);

	delete[] PC;
}

void RECOG::DisplayHypothesisEvaluation2(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	Array2D<Point> ZBuffer,
	Array<int> ZBufferActivePtArray,
	int *subImageMap,
	int *SMCorrespondence,
	int nTransparentPts,
	vtkSmartPointer<vtkActor> *actor)
{
	int nPts = ZBufferActivePtArray.n;
	Point *PC = new Point[4 * nPts];
	Array<Point> MatchedPC;
	MatchedPC.Element = PC;
	MatchedPC.n = 0;
	Array<Point> OccludedPC;
	OccludedPC.Element = PC + nPts;
	OccludedPC.n = 0;
	Array<Point> TransparentPC;
	TransparentPC.Element = PC + 2 * nPts;
	TransparentPC.n = 0;
	Array<Point> SPC;
	SPC.Element = PC + 3 * nPts;
	SPC.n = 0;
	Array<Pair<int, int>> ptAssocs;
	ptAssocs.Element = new Pair<int, int>[nPts];
	ptAssocs.n = 0;
	Array<Point> ptAssocPts;
	ptAssocPts.Element = new Point[2 * nPts];
	Point *PtArray = pMesh->NodeArray.Element;
	int i, iMPt, iSPt;
	Point *pMPt, *pSPt;
	bool bAssoc;
	for (i = 0; i < nPts; i++)
	{
		iMPt = ZBufferActivePtArray.Element[i];
		pMPt = ZBuffer.Element + iMPt;
		if (SMCorrespondence[i] >= 0)
		{
			MatchedPC.Element[MatchedPC.n++] = *pMPt;
			iSPt = SMCorrespondence[i];
			pSPt = PtArray + iSPt;
			SPC.Element[SPC.n++] = *pSPt;
			bAssoc = true;
		}
		else
		{
			iSPt = subImageMap[iMPt];
			pSPt = PtArray + iSPt;
			if (SMCorrespondence[i] == -2)
			{
				TransparentPC.Element[TransparentPC.n++] = *pMPt;
				SPC.Element[SPC.n++] = *pSPt;
				bAssoc = true;
			}
			else
			{
				ptAssocs.Element[ptAssocs.n].a = nPts + OccludedPC.n;
				OccludedPC.Element[OccludedPC.n++] = *pMPt;
				bAssoc = false;
			}
		}
		if (bAssoc)
		{
			ptAssocs.Element[ptAssocs.n].a = 2 * ptAssocs.n;
			ptAssocs.Element[ptAssocs.n].b = 2 * ptAssocs.n + 1;
			ptAssocPts.Element[2 * ptAssocs.n] = *pMPt;
			ptAssocPts.Element[2 * ptAssocs.n + 1] = *pSPt;
			ptAssocs.n++;
		}
	}
	ptAssocPts.n = 2 * ptAssocs.n;
	unsigned char red[3];
	unsigned char green[3];
	unsigned char yellow[3];
	RVLSET3VECTOR(green, 0, 255, 0);
	RVLSET3VECTOR(yellow, 255, 255, 0);
	RVLSET3VECTOR(red, 255, 0, 0);
	actor[0] = (MatchedPC.n > 0 ? pVisualizer->DisplayPointSet<float, Point>(MatchedPC, green, 4.0f) : NULL);
	actor[1] = (OccludedPC.n > 0 ? pVisualizer->DisplayPointSet<float, Point>(OccludedPC, yellow, 4.0f) : NULL);
	actor[2] = (TransparentPC.n > 0 ? pVisualizer->DisplayPointSet<float, Point>(TransparentPC, red, 4.0f) : NULL);
	actor[3] = (ptAssocPts.n > 0 ? pVisualizer->DisplayLines(ptAssocPts, ptAssocs, red) : NULL);
	delete[] PC;
}

Grid::Grid()
{
	mem = NULL;
	cells.Element = NULL;
}

Grid::~Grid()
{
	Clear();
}

void Grid::Create(
	Array<Point> points,
	Camera *pCameraIn,
	int cellSizeIn)
{
	cellSize = cellSizeIn;
	fCellSize = (float)cellSize;
	pCamera = pCameraIn;
	mem = new QLIST::Index[points.n];
	QLIST::Index *pPtIdx = mem;
	cells.w = pCamera->w / cellSize;
	cells.h = pCamera->h / cellSize;
	int nCells = cells.w * cells.h;
	cells.Element = new QList<QLIST::Index>[nCells];
	idxRect.minx = 0;
	idxRect.maxx = cells.w - 1;
	idxRect.miny = 0;
	idxRect.maxy = cells.h - 1;
	int iCell;
	QList<QLIST::Index> *pCellPtList;
	for (iCell = 0; iCell < nCells; iCell++)
	{
		pCellPtList = cells.Element + iCell;
		RVLQLIST_INIT(pCellPtList);
	}
	int iPt;
	Point *pPt;
	for (iPt = 0; iPt < points.n; iPt++)
	{
		pPt = points.Element + iPt;
		if (pPt->bValid)
			break;
	}
	Box<float> bbox;
	InitBoundingBox<float>(&bbox, pPt->P);
	iPt = 0;
	int u, v, iu, iv;
	for (v = 0; v < pCamera->h; v++)
		for (u = 0; u < pCamera->w; u++, iPt++)
		{
			pPt = points.Element + iPt;
			if (!pPt->bValid)
				continue;
			UpdateBoundingBox<float>(&bbox, pPt->P);
			iu = u / cellSize;
			iv = v / cellSize;
			iCell = iu + iv * cells.w;
			pCellPtList = cells.Element + iCell;
			RVLQLIST_ADD_ENTRY(pCellPtList, pPtIdx);
			pPtIdx->Idx = iPt;
			pPtIdx++;
		}
	float bboxSize[3];
	BoxSize<float>(&bbox, bboxSize[0], bboxSize[1], bboxSize[2]);
	maxPtDist = RVLDOTPRODUCT3(bboxSize, bboxSize);
}

void Grid::Clear()
{
	RVL_DELETE_ARRAY(mem);
	RVL_DELETE_ARRAY(cells.Element);
}

bool Grid::GetNeighbors(
	float *P,
	Array<int> &points)
{
	float u_ = pCamera->fu * P[0] / P[2] + pCamera->uc;
	int u = (int)(u_ + 0.5f);
	if (u < 0)
		return false;
	if (u >= pCamera->w)
		return false;
	float v_ = pCamera->fv * P[1] / P[2] + pCamera->vc;
	int v = (int)(v_ + 0.5f);
	if (v < 0)
		return false;
	else if (v >= pCamera->h)
		return false;
	Rect<int> ROI;
	ROI.maxx = (int)round(u_ / fCellSize + 0.5f);
	ROI.minx = ROI.maxx - 1;
	ROI.maxy = (int)round(v_ / fCellSize + 0.5f);
	ROI.miny = ROI.maxy - 1;
	CropRect<int>(ROI, idxRect);
	points.n = 0;
	int ix, iy, iCell;
	QList<QLIST::Index> *pCellPtList;
	QLIST::Index *pPtIdx;
	for (iy = ROI.miny; iy <= ROI.maxy; iy++)
		for (ix = ROI.minx; ix <= ROI.maxx; ix++)
		{
			iCell = ix + iy * cells.w;
			pCellPtList = cells.Element + iCell;
			pPtIdx = pCellPtList->pFirst;
			while (pPtIdx)
			{
				points.Element[points.n++] = pPtIdx->Idx;
				pPtIdx = pPtIdx->pNext;
			}
		}
	return true;
}

void Grid::SubSample(
	Array<Point> srcPoints,
	float normalDiffThr,
	Array<int> &pointSubset)
{
	// Constants.

	float csNThr = cos(normalDiffThr * DEG2RAD);

	//

	pointSubset.Element = new int[srcPoints.n];
	pointSubset.n = 0;
	int nCells = cells.w * cells.h;

	Array<int> cellPointSubset;
	cellPointSubset.Element = pointSubset.Element;
	Array<int> cellPoints;
	cellPoints.Element = new int[srcPoints.n];
	float *dist = new float[srcPoints.n];

	QList<QLIST::Index> *pCellPtList = cells.Element;

	int i, j, iPt, iPt_, iCell;
	float fTmp, minDist;
	float c[3], PNrm[3];
	QLIST::Index *pPtIdx;
	Point *pPt, *pPt_;
	bool bPointAdded;
	for (iCell = 0; iCell < nCells; iCell++, pCellPtList++)
	{
		pPtIdx = pCellPtList->pFirst;
		if (pPtIdx == NULL)
			continue;
		cellPoints.n = 0;
		RVLNULL3VECTOR(c);
		while (pPtIdx)
		{
			cellPoints.Element[cellPoints.n++] = pPtIdx->Idx;
			pPt = srcPoints.Element + pPtIdx->Idx;
			RVLSUM3VECTORS(c, pPt->P, c);
			pPtIdx = pPtIdx->pNext;
		}
		fTmp = (float)(cellPoints.n);
		RVLSCALE3VECTOR2(c, fTmp, c);
		for (i = 0; i < cellPoints.n; i++)
		{
			iPt = cellPoints.Element[i];
			pPt = srcPoints.Element + iPt;
			RVLDIF3VECTORS(pPt->P, c, PNrm);
			dist[i] = RVLDOTPRODUCT3(PNrm, PNrm);
		}
		cellPointSubset.n = 0;
		do
		{
			bPointAdded = false;
			minDist = 2.0f * maxPtDist;
			for (i = 0; i < cellPoints.n; i++)
			{
				iPt = cellPoints.Element[i];
				pPt = srcPoints.Element + iPt;
				for (j = 0; j < cellPointSubset.n; j++)
				{
					iPt_ = cellPointSubset.Element[j];
					pPt_ = srcPoints.Element + iPt_;
					if (RVLDOTPRODUCT3(pPt->N, pPt_->N) >= csNThr)
						break;
				}
				if (j < cellPointSubset.n)
					continue;
				if (dist[i] < minDist)
				{
					minDist = dist[i];
					bPointAdded = true;
					cellPointSubset.Element[cellPointSubset.n] = iPt;
				}
			}
			if (bPointAdded)
				cellPointSubset.n++;
		} while (bPointAdded);
		cellPointSubset.Element += cellPointSubset.n;
	}
	pointSubset.n = cellPointSubset.Element - pointSubset.Element;

	delete[] cellPoints.Element;
	delete[] dist;
}

VoxelGrid::VoxelGrid()
{
	mem = NULL;
	cells.Element = NULL;
}

VoxelGrid::~VoxelGrid()
{
	Clear();
}

void VoxelGrid::Create(
	Array<Point> points,
	float cellSizeIn,
	int *ptIdx)
{
	cellSize = cellSizeIn;

	// Compute bounding box of pQMesh.

	float *P = points.Element[0].P;

	InitBoundingBox<float>(&BBox, P);

	int iPt;

	for (iPt = 1; iPt < points.n; iPt++)
	{
		P = points.Element[iPt].P;

		UpdateBoundingBox<float>(&BBox, P);
	}

	float BBoxSize[3];
	BBoxSize[0] = BBox.maxx - BBox.minx;
	BBoxSize[1] = BBox.maxy - BBox.miny;
	BBoxSize[2] = BBox.maxz - BBox.minz;
	maxPtDist = RVLDOTPRODUCT3(BBoxSize, BBoxSize);

	// Expand bounding box.

	ExpandBox<float>(&BBox, 1.1f * cellSize);

	// Create voxel grid.

	mem = new QLIST::Index[points.n];

	QLIST::Index *pPtIdx = mem;

	cells.a = (int)ceil((BBox.maxx - BBox.minx) / cellSize);
	cells.b = (int)ceil((BBox.maxy - BBox.miny) / cellSize);
	cells.c = (int)ceil((BBox.maxz - BBox.minz) / cellSize);
	int nVoxels = cells.a * cells.b * cells.c;
	cells.Element = new QList<QLIST::Index>[nVoxels];

	idxBox.minx = 0;
	idxBox.maxx = cells.a - 1;
	idxBox.miny = 0;
	idxBox.maxy = cells.b - 1;
	idxBox.minz = 0;
	idxBox.maxz = cells.c - 1;

	int iVoxel;
	QList<QLIST::Index> *pVoxelPtList;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
	{
		pVoxelPtList = cells.Element + iVoxel;

		RVLQLIST_INIT(pVoxelPtList);
	}

	RVLSET3VECTOR(P0, BBox.minx, BBox.miny, BBox.minz);

	float PNrm[3];
	int ix, iy, iz;

	for (iPt = 0; iPt < points.n; iPt++)
	{
		P = points.Element[iPt].P;

		RVLDIF3VECTORS(P, P0, PNrm);
		RVLSCALE3VECTOR2(PNrm, cellSize, PNrm);

		ix = (int)floor(PNrm[0]);
		iy = (int)floor(PNrm[1]);
		iz = (int)floor(PNrm[2]);

		iVoxel = RVL3DARRAY_ELEMENT_INDEX(cells, ix, iy, iz);

		pVoxelPtList = cells.Element + iVoxel;

		RVLQLIST_ADD_ENTRY(pVoxelPtList, pPtIdx);

		pPtIdx->Idx = (ptIdx ? ptIdx[iPt] : iPt);

		pPtIdx++;
	}
}

void VoxelGrid::Clear()
{
	RVL_DELETE_ARRAY(mem);
	RVL_DELETE_ARRAY(cells.Element);
}

void VoxelGrid::GetPointsWithinBlock(
	Box<float> boxIn,
	Array<int> &points)
{
	points.n = 0;

	Box<float> box;
	if (!BoxIntersection<float>(&BBox, &boxIn, &box))
		return;

	float PMin[3];
	RVLSET3VECTOR(PMin, box.minx, box.miny, box.minz);
	float PMax[3];
	RVLSET3VECTOR(PMax, box.maxx, box.maxy, box.maxz);

	float PMinNrm[3];
	RVLDIF3VECTORS(PMin, P0, PMinNrm);
	RVLSCALE3VECTOR2(PMinNrm, cellSize, PMinNrm);
	float PMaxNrm[3];
	RVLDIF3VECTORS(PMax, P0, PMaxNrm);
	RVLSCALE3VECTOR2(PMaxNrm, cellSize, PMaxNrm);

	int ixStart = (int)floor(PMinNrm[0]);
	int iyStart = (int)floor(PMinNrm[1]);
	int izStart = (int)floor(PMinNrm[2]);
	int ixEnd = (int)floor(PMaxNrm[0]);
	int iyEnd = (int)floor(PMaxNrm[1]);
	int izEnd = (int)floor(PMaxNrm[2]);

	int ix, iy, iz, iVoxel;
	QList<QLIST::Index> *pVoxelPtList;
	QLIST::Index *pPtIdx;

	for (iz = izStart; iz <= izEnd; iz++)
		for (iy = iyStart; iy <= iyEnd; iy++)
			for (ix = ixStart; ix <= ixEnd; ix++)
			{
				iVoxel = RVL3DARRAY_ELEMENT_INDEX(cells, ix, iy, iz);

				pVoxelPtList = cells.Element + iVoxel;

				pPtIdx = pVoxelPtList->pFirst;

				while (pPtIdx)
				{
					points.Element[points.n++] = pPtIdx->Idx;

					pPtIdx = pPtIdx->pNext;
				}
			}
}

void VoxelGrid::GetNeighbors(
	float *P,
	Array<int> &points)
{
	points.n = 0;

	float PNrm[3];
	RVLDIF3VECTORS(P, P0, PNrm);
	RVLSCALE3VECTOR2(PNrm, cellSize, PNrm);
	Box<int> ROI;
	ROI.maxx = (int)round(PNrm[0]);
	ROI.minx = ROI.maxx - 1;
	ROI.maxy = (int)round(PNrm[1]);
	ROI.miny = ROI.maxy - 1;
	ROI.maxz = (int)round(PNrm[2]);
	ROI.minz = ROI.maxz - 1;
	BoxIntersection<int>(&ROI, &idxBox, &ROI);

	int ix, iy, iz, iVoxel;
	QList<QLIST::Index> *pVoxelPtList;
	QLIST::Index *pPtIdx;

	for (iz = ROI.minz; iz <= ROI.maxz; iz++)
		for (iy = ROI.miny; iy <= ROI.maxy; iy++)
			for (ix = ROI.minx; ix <= ROI.maxx; ix++)
			{
				iVoxel = RVL3DARRAY_ELEMENT_INDEX(cells, ix, iy, iz);

				pVoxelPtList = cells.Element + iVoxel;

				pPtIdx = pVoxelPtList->pFirst;

				while (pPtIdx)
				{
					points.Element[points.n++] = pPtIdx->Idx;

					pPtIdx = pPtIdx->pNext;
				}
			}
}

void VoxelGrid::GetPointsWithinSphere(
	Array<Point> srcPoints,
	float *P,
	float r,
	Array<int> &tgtPoints,
	int *pointBuffMem)
{
	float r2 = r * r;

	Box<float> ROI;
	ROI.minx = P[0] - r;
	ROI.maxx = P[0] + r;
	ROI.miny = P[1] - r;
	ROI.maxy = P[1] + r;
	ROI.minz = P[2] - r;
	ROI.maxz = P[2] + r;

	Array<int> pointBuff;
	pointBuff.Element = pointBuffMem;

	GetPointsWithinBlock(ROI, pointBuff);

	tgtPoints.n = 0;

	int i, iPt;
	float dP[3];
	float *P_;

	for (i = 0; i < pointBuff.n; i++)
	{
		iPt = pointBuff.Element[i];
		P_ = srcPoints.Element[iPt].P;
		RVLDIF3VECTORS(P_, P, dP);
		if (RVLDOTPRODUCT3(dP, dP) <= r2)
			tgtPoints.Element[tgtPoints.n++] = iPt;
	}
}

void VoxelGrid::SubSample(
	Array<Point> srcPoints,
	float normalDiffThr,
	Array<int> &pointSubset)
{
	// Constants.

	float csNThr = cos(normalDiffThr * DEG2RAD);

	//

	pointSubset.Element = new int[srcPoints.n];
	pointSubset.n = 0;
	int nVoxels = cells.a * cells.b * cells.c;

	Array<int> cellPointSubset;
	cellPointSubset.Element = pointSubset.Element;
	Array<int> cellPoints;
	cellPoints.Element = new int[srcPoints.n];
	float *dist = new float[srcPoints.n];

	QList<QLIST::Index> *pVoxelPtList = cells.Element;

	int i, j, iPt, iPt_, iVoxel;
	float fTmp, minDist;
	float c[3], PNrm[3];
	QLIST::Index *pPtIdx;
	Point *pPt, *pPt_;
	bool bPointAdded;
	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++, pVoxelPtList++)
	{
		if (iVoxel == 603)
			//	int debug = 0;
			pPtIdx = pVoxelPtList->pFirst;
		if (pPtIdx == NULL)
			continue;
		cellPoints.n = 0;
		RVLNULL3VECTOR(c);
		while (pPtIdx)
		{
			cellPoints.Element[cellPoints.n++] = pPtIdx->Idx;
			pPt = srcPoints.Element + pPtIdx->Idx;
			RVLSUM3VECTORS(c, pPt->P, c);
			pPtIdx = pPtIdx->pNext;
		}
		fTmp = (float)(cellPoints.n);
		RVLSCALE3VECTOR2(c, fTmp, c);
		for (i = 0; i < cellPoints.n; i++)
		{
			iPt = cellPoints.Element[i];
			pPt = srcPoints.Element + iPt;
			RVLDIF3VECTORS(pPt->P, c, PNrm);
			dist[i] = RVLDOTPRODUCT3(PNrm, PNrm);
		}
		cellPointSubset.n = 0;
		do
		{
			bPointAdded = false;
			minDist = 2.0f * maxPtDist;
			for (i = 0; i < cellPoints.n; i++)
			{
				iPt = cellPoints.Element[i];
				pPt = srcPoints.Element + iPt;
				// if (pPt->P[0] * pPt->P[0] + pPt->P[2] * pPt->P[2] < 0.001f)
				//	int debug = 0;
				for (j = 0; j < cellPointSubset.n; j++)
				{
					iPt_ = cellPointSubset.Element[j];
					pPt_ = srcPoints.Element + iPt_;
					if (RVLDOTPRODUCT3(pPt->N, pPt_->N) >= csNThr)
						break;
				}
				if (j < cellPointSubset.n)
					continue;
				if (dist[i] < minDist)
				{
					minDist = dist[i];
					bPointAdded = true;
					cellPointSubset.Element[cellPointSubset.n] = iPt;
				}
			}
			if (bPointAdded)
				cellPointSubset.n++;
		} while (bPointAdded);
		cellPointSubset.Element += cellPointSubset.n;
	}
	pointSubset.n = cellPointSubset.Element - pointSubset.Element;

	delete[] cellPoints.Element;
	delete[] dist;
}

PointAssociationData::PointAssociationData()
{
	MNN = NULL;
	QNN = NULL;
	MNNDist = NULL;
	QNNDist = NULL;
	MPts.Element = NULL;
	PMQ = NULL;
	explainedQPts.Element = NULL;
	associatedMPts.Element = NULL;
}

PointAssociationData::~PointAssociationData()
{
	Clear();
}

void PointAssociationData::Create(
	int nMPts,
	int nQPts,
	bool bNormals)
{
	MNN = new int[nQPts];
	QNN = new int[nMPts];
	MNNDist = new float[nQPts];
	QNNDist = new float[nMPts];
	MPts.Element = new int[nMPts];
	PMQ = new float[(bNormals ? 6 : 3) * nMPts];
	explainedQPts.Element = new int[nQPts];
	associatedMPts.Element = new int[nMPts];
}

void PointAssociationData::Create(
	Mesh *pMMesh,
	Mesh *pQMesh,
	bool bNormals)
{
	Create(pMMesh->NodeArray.n, pQMesh->NodeArray.n, bNormals);
}

void PointAssociationData::Clear()
{
	RVL_DELETE_ARRAY(MNN);
	RVL_DELETE_ARRAY(QNN);
	RVL_DELETE_ARRAY(MNNDist);
	RVL_DELETE_ARRAY(QNNDist);
	RVL_DELETE_ARRAY(MPts.Element);
	RVL_DELETE_ARRAY(PMQ);
	RVL_DELETE_ARRAY(explainedQPts.Element);
	RVL_DELETE_ARRAY(associatedMPts.Element);
}
