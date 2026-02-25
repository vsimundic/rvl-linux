#include "RVLCore2.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#endif
#include "Util.h"
#include "Extractor.h"

using namespace RVL;

Extractor::Extractor()
{
	fuSrc = 570.34f;
	fvSrc = 570.34f;
	ucSrc = 320.0f;
	vcSrc = 240.0f;
	imgWidth = 640;
	imgHeight = 480;
	fTgt = 570.34;
	zn = 2.5f;
	zf = 6.0f;
	tgtImgWidth = 224;
	tgtImgHeight = 334;
	zInvalid = 6.0;
}

Extractor::~Extractor()
{

}

void Extractor::Create(char* cfgFileName)
{
	CreateParamList();

	paramList.LoadParams(cfgFileName);
}

void Extractor::CreateParamList()
{
	paramList.m_pMem = pMem0;

	RVLPARAM_DATA* pParamData;

	paramList.Init();

	pParamData = paramList.AddParam("Camera.fu", RVLPARAM_TYPE_FLOAT, &fuSrc);
	pParamData = paramList.AddParam("Camera.fv", RVLPARAM_TYPE_FLOAT, &fvSrc);
	pParamData = paramList.AddParam("Camera.uc", RVLPARAM_TYPE_FLOAT, &ucSrc);
	pParamData = paramList.AddParam("Camera.vc", RVLPARAM_TYPE_FLOAT, &vcSrc);
	pParamData = paramList.AddParam("Camera.imageWidth", RVLPARAM_TYPE_INT, &imgWidth);
	pParamData = paramList.AddParam("Camera.imageHeight", RVLPARAM_TYPE_INT, &imgHeight);
	pParamData = paramList.AddParam("Extractor.fTgt", RVLPARAM_TYPE_FLOAT, &fTgt);
	pParamData = paramList.AddParam("Extractor.zn", RVLPARAM_TYPE_FLOAT, &zn);
	pParamData = paramList.AddParam("Extractor.zf", RVLPARAM_TYPE_FLOAT, &zf);
	pParamData = paramList.AddParam("Extractor.tgtImgWidth", RVLPARAM_TYPE_INT, &tgtImgWidth);
	pParamData = paramList.AddParam("Extractor.tgtImgHeight", RVLPARAM_TYPE_INT, &tgtImgHeight);
	pParamData = paramList.AddParam("Extractor.zInvalid", RVLPARAM_TYPE_FLOAT, &zInvalid);
}

void Extractor::Apply(
	Array2D<float> srcImage,
	float* RBC,
	float* tBC,
	float* gC,
	float* boxSize,
	Array2D<float> &tgtImage,
	Array2D<float> &tgtPC,
	float *RCO,
	float *tCO)
{
	// ROI.

	float vertexBnrm[8][3] = { {1, 1, 1}, {-1, 1, 1}, {-1, -1, 1}, {1, -1, 1},  {1, 1, -1}, {-1, 1, -1}, {-1, -1, -1}, {1, -1, -1} };

	float vertexB[3], vertexC[3];

	Rect<int> ROI;
	int i;
	int vertexI[2];

	for (i = 0; i < 8; i++)
	{
		vertexB[0] = 0.5f * boxSize[0] * vertexBnrm[i][0];
		vertexB[1] = 0.5f * boxSize[1] * vertexBnrm[i][1];
		vertexB[2] = 0.5f * boxSize[2] * vertexBnrm[i][2];

		RVLTRANSF3(vertexB, RBC, tBC, vertexC);

		vertexI[0] = (int)round(fuSrc * vertexC[0] / vertexC[2] + ucSrc);
		vertexI[1] = (int)round(fvSrc * vertexC[1] / vertexC[2] + vcSrc);

		if (i == 0)
			InitRect<int>(&ROI, vertexI);
		else
			UpdateRect<int>(&ROI, vertexI);
	}

	Rect<int> imgWin;
	imgWin.minx = 0;
	imgWin.maxx = imgWidth - 1;
	imgWin.miny = 0;
	imgWin.maxy = imgHeight - 1;

	CropRect<int>(ROI, imgWin);

	// Object scale.

	float scale = RVLMAX(boxSize[0], boxSize[1]);
	scale = RVLMAX(scale, boxSize[2]) / 0.9f;

	// Object RF orientation.

	float* XOC = RCO;
	float* YOC = RCO + 3;
	float* ZOC = RCO + 6;

	float rSrc = sqrt(RVLDOTPRODUCT3(tBC, tBC));
	RVLSCALE3VECTOR2(tBC, rSrc, ZOC);
	RVLCROSSPRODUCT3(gC, ZOC, XOC);
	RVLNEGVECT3(XOC, XOC);
	float fTmp;
	RVLNORM3(XOC, fTmp);
	RVLCROSSPRODUCT3(ZOC, XOC, YOC);

	// Detect points within the box and transform them to the object RF.

	Box<float> box;
	box.minx = -0.5f * boxSize[0];
	box.maxx = 0.5f * boxSize[0];
	box.miny = -0.5f * boxSize[1];
	box.maxy = 0.5f * boxSize[1];
	box.minz = -0.5f * boxSize[2];
	box.maxz = 0.5f * boxSize[2];

	int nPix = imgWidth * imgHeight;

	if (tgtPC.Element == NULL)
		tgtPC.Element = new float[3 * nPix];
	tgtPC.w = 3;
	tgtPC.h = 0;

	bool *bInBox = new bool[nPix];
	memset(bInBox, 0, nPix * sizeof(bool));

	float* POs = new float[3 * nPix];

	Array<int> objPtsIdx;
	objPtsIdx.Element = new int[nPix];
	objPtsIdx.n = 0;

	Box<float> boxC;
	int u, v, iPix;
	float z;
	float PC[3], V3Tmp[3];
	float* PO, *PB;

	for (v = ROI.miny; v <= ROI.maxy; v++)
		for (u = ROI.minx; u <= ROI.maxx; u++)
		{
			iPix = u + v * imgWidth;

			z = srcImage.Element[iPix];

			if (z > 0.0)
			{
				PC[0] = z * ((float)u - ucSrc) / fuSrc;
				PC[1] = z * ((float)v - vcSrc) / fvSrc;
				PC[2] = z;

				PB = tgtPC.Element + 3 * tgtPC.h;

				RVLINVTRANSF3(PC, RBC, tBC, PB, V3Tmp);

				if (bInBox[iPix] = InBoundingBox<float>(&box, PB))
				{
					tgtPC.h++;

					PO = POs + 3 * iPix;

					RVLMULMX3X3VECT(RCO, PC, PO);

					if (objPtsIdx.n == 0)
						InitBoundingBox<float>(&boxC, PO);
					else
						UpdateBoundingBox<float>(&boxC, PO);

					objPtsIdx.Element[objPtsIdx.n++] = iPix;
				}
			}
			else
				bInBox[iPix] = false;
		}

	// Object RF position.

	int tgtImgSize = RVLMIN(tgtImgWidth, tgtImgHeight);
	float boxC2SizeX = RVLMAX(-boxC.minx, boxC.maxx);
	float boxC2SizeY = RVLMAX(-boxC.miny, boxC.maxy);
	float boxC2Size = RVLMAX(boxC2SizeX, boxC2SizeY);
	float rTgt = 2.0f / 0.95f * fTgt * boxC2Size / (float)tgtImgSize + rSrc - boxC.minz;
	float zn_ = 1.05f * zn * scale;
	rTgt = RVLMAX(rTgt, zn_);
	float rOC = rSrc - rTgt;
	tCO[0] = tCO[1] = 0.0;
	tCO[2] = -rOC;

	// Correct PO w.r.t. the object RF position and project object points to the target image.

	float* objPtITgts = new float[2 * nPix];

	float ucTgt = 0.5f * (float)(tgtImage.w);
	float vcTgt = 0.5f * (float)(tgtImage.h);

	int iObjPt;
	float* objPtITgt;

	for (iObjPt = 0; iObjPt < objPtsIdx.n; iObjPt++)
	{
		iPix = objPtsIdx.Element[iObjPt];
		PO = POs + 3 * iPix;
		PO[2] -= rOC;

		objPtITgt = objPtITgts + 2 * iPix;
		objPtITgt[0] = fTgt * PO[0] / PO[2] + ucTgt;
		objPtITgt[1] = fTgt * PO[1] / PO[2] + vcTgt;
	}

	// Create mesh.

	int maxu = ROI.maxx - 1;
	int maxv = ROI.maxy - 1;
	
	int squarePt[4][2] = { {0, 0}, {0, 1}, {1, 1}, {1, 0} };

	Array2D<int> triangle;
	triangle.w = 3;
	triangle.h = 0;
	triangle.Element = new int[6 * nPix];

	int missingPt, nMissingPts, iSquarePt, iTrianglePt;
	int squarePtIdx[4];	

	for(v = ROI.miny; v <= maxv; v++)
		for (u = ROI.minx; u <= maxu; u++)
		{
			missingPt = -1;
			nMissingPts = 0;
			for (iSquarePt = 0; iSquarePt < 4; iSquarePt++)
			{
				squarePtIdx[iSquarePt] = (u + squarePt[iSquarePt][0]) + (v + squarePt[iSquarePt][1]) * imgWidth;

				if (!bInBox[squarePtIdx[iSquarePt]])
				{
					missingPt = iSquarePt;
					nMissingPts++;
				}
			}

			if (nMissingPts > 1)
				continue;

			if (missingPt >= 0)
			{
				iTrianglePt = 0;

				for (iSquarePt = 0; iSquarePt < 4; iSquarePt++)
					if (iSquarePt != missingPt)
					{
						triangle.Element[3 * triangle.h + iTrianglePt] = squarePtIdx[iSquarePt];
						iTrianglePt++;
					}
				triangle.h++;
			}
			else
			{
				triangle.Element[3 * triangle.h + 0] = squarePtIdx[0];
				triangle.Element[3 * triangle.h + 1] = squarePtIdx[3];
				triangle.Element[3 * triangle.h + 2] = squarePtIdx[2];
				triangle.h++;

				triangle.Element[3 * triangle.h + 0] = squarePtIdx[0];
				triangle.Element[3 * triangle.h + 1] = squarePtIdx[2];
				triangle.Element[3 * triangle.h + 2] = squarePtIdx[1];
				triangle.h++;
			}

		}

	delete[] bInBox;

	// Project mesh to the output image.

	int nTgtPix = tgtImage.w * tgtImage.h;

	if (tgtImage.Element == NULL)
		tgtImage.Element = new float[nTgtPix];

	for (iPix = 0; iPix < nTgtPix; iPix++)
		tgtImage.Element[iPix] = zInvalid;

	int iTriangle, uTriangleROI, vTriangleROI;
	float* PT[3];
	float triangleVertexI[4][2];
	Rect<float> fTriangleROI;
	Rect<int> iTriangleROI;
	float NT[3], V3Tmp2[3], Ray[3], POTgt[3];
	float dT, fuTriangleROI, fvTriangleROI, e, sP;
	bool bInsideTriangle;

	for (iTriangle = 0; iTriangle < triangle.h; iTriangle++)
	{
		for (i = 0; i < 3; i++)
		{
			iPix = triangle.Element[3 * iTriangle + i];
			PT[i] = POs + 3 * iPix;
			objPtITgt = objPtITgts + 2 * iPix;
			triangleVertexI[i][0] = objPtITgt[0];
			triangleVertexI[i][1] = objPtITgt[1];

			if (i == 0)
				InitRect<float>(&fTriangleROI, triangleVertexI[0]);
			else
				UpdateRect<float>(&fTriangleROI, triangleVertexI[i]);
		}

		iTriangleROI.minx = (int)floor(fTriangleROI.minx);
		iTriangleROI.maxx = (int)ceil(fTriangleROI.maxx);
		iTriangleROI.miny = (int)floor(fTriangleROI.miny);
		iTriangleROI.maxy = (int)ceil(fTriangleROI.maxy);

		CropRect<int>(iTriangleROI, imgWin);

		triangleVertexI[3][0] = triangleVertexI[0][0];
		triangleVertexI[3][1] = triangleVertexI[0][1];

		RVLDIF3VECTORS(PT[1], PT[0], V3Tmp);
		RVLDIF3VECTORS(PT[2], PT[0], V3Tmp2);
		RVLCROSSPRODUCT3(V3Tmp, V3Tmp2, NT);
		fTmp = RVLDOTPRODUCT3(NT, NT);

		if (fTmp < 1e-10)
			continue;

		RVLNORM3(NT, fTmp);

		dT = RVLDOTPRODUCT3(NT, PT[0]);
		for (vTriangleROI = iTriangleROI.miny; vTriangleROI <= iTriangleROI.maxy; vTriangleROI++)
			for (uTriangleROI = iTriangleROI.minx; uTriangleROI <= iTriangleROI.maxx; uTriangleROI++)
			{
				fuTriangleROI = (float)uTriangleROI;
				fvTriangleROI = (float)vTriangleROI;

				bInsideTriangle = true;
				for (i = 0; i < 3; i++)
				{
					e = (fuTriangleROI - triangleVertexI[i][0]) * (triangleVertexI[i + 1][1] - triangleVertexI[i][1]) - (fvTriangleROI - triangleVertexI[i][1]) * (triangleVertexI[i + 1][0] - triangleVertexI[i][0]);
					if (e > 0)
					{
						bInsideTriangle = false;
						break;
					}
				}

				if (bInsideTriangle)
				{
					RVLSET3VECTOR(Ray, fuTriangleROI - ucTgt, fvTriangleROI - vcTgt, fTgt);
					RVLNORM3(Ray, fTmp);
					sP = dT / RVLDOTPRODUCT3(NT, Ray);
					RVLSCALE3VECTOR(Ray, sP, POTgt);
					tgtImage.Element[uTriangleROI + vTriangleROI * tgtImgWidth] = POTgt[2] / scale;
				}
			}
	}

	// Free memory.

	delete[] triangle.Element;
	delete[] POs;
	delete[] objPtsIdx.Element;
	delete[] objPtITgts;
}

void Extractor::Visualize(Array2D<float> tgtImage)
{
	float minValid = 1.1f * zf;
	float maxValid = 0.9f * zn;

	int iPix;
	float z;

	for (iPix = 0; iPix < tgtImage.h * tgtImage.w; iPix++)
	{
		z = tgtImage.Element[iPix];

		if (z == zInvalid)
			continue;

		if (z < minValid)
			minValid = z;

		if (z > maxValid)
			maxValid = z;
	}

	float zRange = maxValid - minValid;

	cv::Mat tgtObjDepthImgGS(tgtImage.h, tgtImage.w, CV_8UC1);

	uchar* outPix = tgtObjDepthImgGS.data;

	for (iPix = 0; iPix < tgtImage.h * tgtImage.w; iPix++, outPix++)
	{
		z = tgtImage.Element[iPix];

		if (z == zInvalid)
			*outPix = 0;
		else
			*outPix = (uchar)floor(191.0f / zRange * (z - minValid)) + 64;
	}
	cv::imshow("Target object depth image", tgtObjDepthImgGS);

	cv::waitKey();
}
