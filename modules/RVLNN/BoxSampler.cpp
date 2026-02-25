#include "RVLCore2.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#endif
#include "Util.h"
#include "RVL2DCellArray.h"
#include "BoxSampler.h"
#ifdef OPENCV_CONTRIB
#include "opencv2/xfeatures2d.hpp"
#endif


using namespace RVL;
using namespace RVLNN;

BoxSampler::BoxSampler()
{
	nSamples = 64;
	gridSize = 32;
	ZInit = NULL;
	PArray = NULL;
	iPixArray.Element = NULL;
	SURFHessianThrH = 100.0;
	SURFHessianThrL = 1.0;
	bRepeatFeatures = false;
	bVariableSize = false;
	nieghborhoodSize = 0.5f;
	boxSizeCoeff = 1.0f;
}


BoxSampler::~BoxSampler()
{
	RVL_DELETE_ARRAY(ZInit);
	RVL_DELETE_ARRAY(PArray);
	RVL_DELETE_ARRAY(iPixArray.Element);
}

void BoxSampler::Create(char *cfgFileName)
{
	CreateParamList();

	paramList.LoadParams(cfgFileName);

	if (gridSize <= 0)
		return;

	int nGridCells = gridSize * gridSize;

	ZInit = new float[nGridCells];

	int i;

	for (i = 0; i < nGridCells; i++)
		ZInit[i] = zOut;

	cells.m_pMem = pMem;
	cells.m_pMem0 = pMem0;
	cells.m_Width = imageWidth;
	cells.m_Height = imageHeight;

	cells.Init();
}

void BoxSampler::CreateParamList()
{
	paramList.m_pMem = pMem0;

	RVLPARAM_DATA *pParamData;

	paramList.Init();

	pParamData = paramList.AddParam("BoxSampler.nSamples", RVLPARAM_TYPE_INT, &nSamples);
	pParamData = paramList.AddParam("Camera.maxz", RVLPARAM_TYPE_FLOAT, &maxz);
	pParamData = paramList.AddParam("Camera.fu", RVLPARAM_TYPE_FLOAT, &fu);
	pParamData = paramList.AddParam("Camera.fv", RVLPARAM_TYPE_FLOAT, &fv);
	pParamData = paramList.AddParam("Camera.uc", RVLPARAM_TYPE_FLOAT, &uc);
	pParamData = paramList.AddParam("Camera.vc", RVLPARAM_TYPE_FLOAT, &vc);
	pParamData = paramList.AddParam("Camera.imageWidth", RVLPARAM_TYPE_INT, &imageWidth);
	pParamData = paramList.AddParam("Camera.imageHeight", RVLPARAM_TYPE_INT, &imageHeight);
	pParamData = paramList.AddParam("BoxSampler.boxSize", RVLPARAM_TYPE_FLOAT, &boxSize);
	pParamData = paramList.AddParam("BoxSampler.gridSize", RVLPARAM_TYPE_INT, &gridSize);
	pParamData = paramList.AddParam("BoxSampler.zOut", RVLPARAM_TYPE_FLOAT, &zOut);
	pParamData = paramList.AddParam("BoxSampler.Keypoints.CellArray.cellSize", RVLPARAM_TYPE_INT, &(cells.m_CellSize));
	pParamData = paramList.AddParam("BoxSampler.Keypoints.SURF.hessianThrH", RVLPARAM_TYPE_DOUBLE, &SURFHessianThrH);
	pParamData = paramList.AddParam("BoxSampler.Keypoints.SURF.hessianThrL", RVLPARAM_TYPE_DOUBLE, &SURFHessianThrL);
	pParamData = paramList.AddParam("BoxSampler.bRepeatFeatures", RVLPARAM_TYPE_BOOL, &bRepeatFeatures);
	pParamData = paramList.AddParam("BoxSampler.variableSize", RVLPARAM_TYPE_BOOL, &bVariableSize);
	pParamData = paramList.AddParam("BoxSampler.nieghborhoodSize", RVLPARAM_TYPE_FLOAT, &nieghborhoodSize);
	pParamData = paramList.AddParam("BoxSampler.distance", RVLPARAM_TYPE_FLOAT, &distance);
	pParamData = paramList.AddParam("BoxSampler.boxSizeCoeff", RVLPARAM_TYPE_FLOAT, &boxSizeCoeff);
}

void BoxSampler::Apply(
	Array2D<float> image,
	Array3D<float> &outputTensor,
	float *C,
	bool bKeypoints,
	bool bWholeImage,
	bool bImportedKeypoints,
	int nImportedKeypoints,
	Box<float>* pBoundingBox,
	float* boundingBoxR,
	float* boundingBoxt)
{
	int nPix = image.w * image.h;
	int keypointDataSize = (bVariableSize ? 4 : 3);

	// Identify all pixels in the range defined by maxz and compute 3D points.	
	
	RVL_DELETE_ARRAY(iPixArray.Element);

	iPixArray.Element = new int[nPix];
	iPixArray.n = 0;

	RVL_DELETE_ARRAY(PArray);

	PArray = new float[keypointDataSize * nPix];

	float *P;
	int u, v, iPix;
	float z;

	for (iPix = 0; iPix < nPix; iPix++)
		if (image.Element[iPix] > 0 && image.Element[iPix] <= maxz)
		{
			iPixArray.Element[iPixArray.n++] = iPix;

			u = iPix % image.w;
			v = iPix / image.w;
			z = image.Element[iPix];

			P = PArray + keypointDataSize * iPix;

			P[0] = z * ((float)u - uc) / fu;
			P[1] = z * ((float)v - vc) / fv;
			P[2] = z;
		}

	//// Sampling.

	if (bWholeImage)
	{
		float RCB[9];
		RVLUNITMX3(RCB);

		BoxFeature(uc + vc * image.w, image, outputTensor.Element, C, RCB, distance, false);

		outputTensor.a = outputTensor.b = gridSize;
		outputTensor.c = 1;

		RVLSET3VECTOR(C, 0.0f, 0.0f, distance);
	}
	else if (bImportedKeypoints)
	{
		int iKeypoint;
		float RCB[9];
		float r;

		for (iKeypoint = 0; iKeypoint < nImportedKeypoints; iKeypoint++)
		{
			int nGridCells = gridSize * gridSize;

			float* Z = outputTensor.Element + iKeypoint * nGridCells;

			float* C_ = C + keypointDataSize * iKeypoint;

			BoxFeature(iPix, image, Z, C_, RCB, r, true, true);
		}

		outputTensor.a = outputTensor.b = gridSize;
		outputTensor.c = nImportedKeypoints;
	}
	else if (bKeypoints)
		KeypointBasedSampling(image, outputTensor, C, pBoundingBox, boundingBoxR, boundingBoxt);
	else
		RandomSampling(image, outputTensor, C);
}

void BoxSampler::KeypointBasedSampling(
	Array2D<float> image,
	Array3D<float> &outputTensor,
	float* C,
	Box<float>* pBoundingBox,
	float* boundingBoxR,
	float* boundingBoxt)
{
#ifdef OPENCV_CONTRIB
	int nPix = image.w * image.h;
	int nGridCells = gridSize * gridSize;
	int keypointDataSize = (bVariableSize ? 4 : 3);

	bool* bProcessed = new bool[nPix];

	float r;
	float* Z, * C_;
	float RCB[9];

	// Keypoint detector.

	cv::Mat depthImageGS;
	CreateGrayScaleImage(image, depthImageGS);

	double hessianThr = SURFHessianThrH;

	cv::Ptr<cv::xfeatures2d::SURF> keypointDetector = cv::xfeatures2d::SURF::create();

	// Detect box features and store them in outputTensor.
	// The box feature centers are stored in matrix C.
	// If bVariableSize is true, then the box sizes are stored as the fourth column of matrix C.

	std::vector<cv::KeyPoint> keypoints;

	double hessianThrCorrectionFactor = 0.1;

	//char debugFileName[200];

	//for (int i = 0; i < 10; i++)
	//{
	//	cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(hessianThr);
	//	std::vector<cv::KeyPoint> keypoints;
	//	detector->detect(depthImageGS, keypoints);

	//	sprintf(debugFileName, "C:\\RVL\\Debug\\keypoints%d.txt", i);

	//	FILE* fpDebug = fopen(debugFileName, "w");

	//	for (int j = 0; j < keypoints.size(); j++)
	//		fprintf(fpDebug, "%d: %f\n", j, keypoints[j].response);

	//	fclose(fpDebug);

	//	hessianThr *= 0.5;
	//}

	float minKeypointSize = 0.25 * (float)gridSize;

	cells.RemoveAll();

	memset(bProcessed, 0, nPix * sizeof(bool));

	int iSample = 0;
	int iKeypoint = 0;

	int* keypointIdx = new int[nSamples];

	Array<void*> neighborArray;

	neighborArray.Element = new void* [nSamples];

	int nOldKeypoints = 0;

	bool bHessianThrLReached = false;

	cv::KeyPoint keypoint;
	Rect<int> patch;
	int i, iPix, nNewKeypoints, iNeighborSample, nSamplesMissing;
	int pix[2];
	float ax, ay, fTmp;
	float* PC, * PC_;
	float PvB[3], PvC[3], dP[3];
	float boxSize_, halfBoxSize, neighborhoodSize_;

	while (iSample < nSamples)
	{
		// If number of samples is less than nSamples, then detect keypoints.
		// Lower the feature detector threshold and repeat the feature detection until the total number of detected keypoints reaches nSamples or the threshold is below SURFHessianThrL.

		nSamplesMissing = nSamples - iSample;

		nOldKeypoints = keypoints.size();

		do
		{		
			keypointDetector->setHessianThreshold(hessianThr);

			keypointDetector->detect(depthImageGS, keypoints);

			//cv::Mat keypointDisplay;
			//cv::drawKeypoints(depthImageGS, keypoints, keypointDisplay);
			//cv::imshow("keypoints", keypointDisplay);
			//cv::waitKey();

			nNewKeypoints = keypoints.size() - nOldKeypoints;

			if (bHessianThrLReached = (hessianThr <= SURFHessianThrL))
				break;

			hessianThr *= hessianThrCorrectionFactor;
		} while (nNewKeypoints < nSamplesMissing);

		for (; iKeypoint < keypoints.size() && iSample < nSamples; iKeypoint++)
		{
			keypoint = keypoints[iKeypoint];

			pix[0] = (int)round(keypoint.pt.x);
			pix[1] = (int)round(keypoint.pt.y);

			iPix = pix[0] + pix[1] * image.w;

			if (bProcessed[iPix])
				continue;

			bProcessed[iPix] = true;

			if (image.Element[iPix] > maxz)
				continue;

			PC = PArray + keypointDataSize * iPix;

			if (pBoundingBox)
			{
				float CB[3];

				if (boundingBoxt)
				{
					RVLDIF3VECTORS(PC, boundingBoxt, CB);
				}

				if (boundingBoxR)
				{
					float V3Tmp[3];

					RVLMULMX3X3TVECT(boundingBoxR, CB, V3Tmp);
					RVLCOPY3VECTOR(V3Tmp, CB);
				}

				if (!InBoundingBox<float>(pBoundingBox, CB))
					continue;
			}

			BoxFeature(iPix, image, NULL, NULL, RCB, r);

			if (bVariableSize)
			{
				if (keypoint.size < minKeypointSize)
					continue;

				boxSize_ = boxSizeCoeff * keypoint.size / fu * r;

				PC[3] = boxSize_;
			}
			else
				boxSize_ = boxSize;

			halfBoxSize = 0.5f * boxSize_;

			PvB[2] = r - halfBoxSize;

			ax = halfBoxSize;
			ay = halfBoxSize;

			for (i = 0; i < 4; i++)
			{
				PvB[0] = ax;
				PvB[1] = ay;

				RVLMULMX3X3TVECT(RCB, PvB, PvC);

				pix[0] = (int)round(fu * PvC[0] / PvC[2] + uc);
				pix[1] = (int)round(fv * PvC[1] / PvC[2] + vc);

				if (i == 0)
					InitRect<int>(&patch, pix);
				else
					UpdateRect<int>(&patch, pix);

				fTmp = ax;
				ax = ay;
				ay = -fTmp;
			}

			cells.Get(patch.minx, patch.maxx, patch.miny, patch.maxy, neighborArray.Element, neighborArray.n);

			neighborhoodSize_ = nieghborhoodSize * boxSize_;

			for (i = 0; i < neighborArray.n; i++)
			{
				iNeighborSample = (int*)(neighborArray.Element[i]) - keypointIdx;

				PC_ = C + keypointDataSize * iNeighborSample;

				RVLDIF3VECTORS(PC_, PC, dP);

				if (RVLABS(dP[0]) > neighborhoodSize_)
					continue;

				if (RVLABS(dP[1]) > neighborhoodSize_)
					continue;

				if (RVLABS(dP[2]) > neighborhoodSize_)
					continue;

				break;
			}

			if (i < neighborArray.n)
				continue;

			Z = outputTensor.Element + iSample * nGridCells;

			C_ = C + keypointDataSize * iSample;

			if (!BoxFeature(iPix, image, Z, C_, RCB, r, false))
				continue;

			cells.Add(keypoint.pt.x, keypoint.pt.y, keypointIdx + iSample);

			keypointIdx[iSample] = iKeypoint;

			iSample++;
		}

		// If SURFHessianThrL is reached, then tile the detected feature to obtain exactly nSamples features. 

		if (bHessianThrLReached)
		{
			if (bRepeatFeatures)
			{
				nSamplesMissing = nSamples - iSample;

				while (nSamplesMissing > 0)
				{
					Z = outputTensor.Element + iSample * nGridCells;

					C_ = C + keypointDataSize * iSample;

					int nSamplesToCopy = RVLMIN(nSamplesMissing, iSample);

					memcpy(Z, outputTensor.Element, nSamplesToCopy * nGridCells * sizeof(float));
					memcpy(C_, C, nSamplesToCopy * keypointDataSize * sizeof(float));

					iSample += nSamplesToCopy;

					nSamplesMissing = nSamples - iSample;
				}
			}
			else
				break;
		}
	}

	delete[] bProcessed;
	delete[] keypointIdx;
	delete[] neighborArray.Element;

	outputTensor.a = outputTensor.b = gridSize;	
	outputTensor.c = iSample;

	//cv::Mat keypointDisplay;
	//cv::drawKeypoints(depthImageGS, keypoints, keypointDisplay);
	//cv::imshow("keypoints", keypointDisplay);
	//cv::waitKey();
#else
	printf("OpenCV Contrib required!\n");
#endif
}

void BoxSampler::RandomSampling(
	Array2D<float> image,
	Array3D<float> &outputTensor,
	float* C)
{
	int nPix = image.w * image.h;
	int nGridCells = gridSize * gridSize;

	/// Random sampling.	

	bool* bProcessed = new bool[nPix];

	float r;
	float* Z, * C_;
	float RCB[9];

	bool* bBorder = new bool[nPix];

	memset(bBorder, 0, nPix * sizeof(bool));

	int* iPixMem = new int[2 * nPix];

	Array<int> srcPixIdxArray;

	srcPixIdxArray.Element = iPixMem;

	Array<int> tgtPixIdxArray;

	tgtPixIdxArray.Element = iPixMem + nPix;

	int iSample = 0;

	int k, l, iPix;
	int nPixToSample, nSamples_;
	Array<int> tmpPixIdxArray;

	while (iSample < nSamples)
	{
		memset(bProcessed, 0, nPix * sizeof(bool));

		memcpy(srcPixIdxArray.Element, iPixArray.Element, iPixArray.n * sizeof(int));

		srcPixIdxArray.n = iPixArray.n;

		while (iSample < nSamples && srcPixIdxArray.n > 0)
		{
			nPixToSample = nSamples - iSample;

			nSamples_ = RVLMIN(nPixToSample, srcPixIdxArray.n);

			for (k = 0; k < nSamples_; k++)
			{
				l = rand() % srcPixIdxArray.n;

				iPix = srcPixIdxArray.Element[l];

				if (!bProcessed[iPix] && !bBorder[iPix])
				{
					bProcessed[iPix] = true;

					Z = outputTensor.Element + iSample * nGridCells;

					C_ = C + 3 * iSample;

					if (BoxFeature(iPix, image, Z, C_, RCB, r))
						iSample++;
					else
						bBorder[iPix] = true;
				}	// if (!bProcessed[iPix] && !bBorder[iPix])
			}	// for (k = 0; k < nPixToSample; k++)

			tgtPixIdxArray.n = 0;

			for (l = 0; l < srcPixIdxArray.n; l++)
			{
				iPix = srcPixIdxArray.Element[l];

				if (!bProcessed[iPix] && !bBorder[iPix])
					tgtPixIdxArray.Element[tgtPixIdxArray.n++] = iPix;
			}

			tmpPixIdxArray = srcPixIdxArray;
			srcPixIdxArray = tgtPixIdxArray;
			tgtPixIdxArray = tmpPixIdxArray;
		}	// while (iSample < nSamples && srcPixIdxArray.n > 0)		
	}	// while (iSample < nSamples)

	delete[] bProcessed;
	delete[] bBorder;
	delete[] iPixMem;

	outputTensor.a = outputTensor.b = gridSize;
	outputTensor.c = iSample;
}

// Function BoxFeature computes the box-feature in the form of array Z of size gridSize x gridSize.
// Inputs:
//     iPix - center pixel index
//     image   
//     RCB - The rotation matrix defining the orientation of the camera RF w.r.t. the box RF. 
//           If bComputeBox == true, then function BoxFeature computes RCB.
//           Otherwise, this is an input.
//     r - the distance of the box center from the camera.
//         If bComputeBox == true, then function BoxFeature computes r as the distance of iPix from the camera.
//         therwise, this is an input.
//     pBoundingBox - pointer to the bounding box
// Outputs: 
//     Z - box-feature z-array
//     RCB
//     r
//     C - vector of 3D coordinates of iPix (If bVariableSize is true, then the fourth coordinate is box size.)
// Parameters: 
//     PArray - 2D array of 3D points assigned to image pixels (If bVariableSize is true, then the fourth element is box size.)
//     zOut - value assigned to the points beyond the back side of the box;
//            value -(zOut - 1) is assigned to the points ahead of the front side of the box.
//     gridSize
//     boxSize - box size in meters
//     fu, fv, uc, vc - intrinsic camera parameters
//     maxz - maximum z coordinate

bool BoxSampler::BoxFeature(
	int iPix,
	Array2D<float> image,
	float *Z,
	float *C,
	float* RCB,
	float& r,
	bool bComputeBox,
	bool bImportCenter,
	Box<float>* pBoundingBox)
{
	float zfOut = -(zOut - 1.0f);

	int keypointDataSize = (bVariableSize ? 4 : 3);

	float *PcC = (bImportCenter ? C : PArray + keypointDataSize * iPix);

	float *XBC = RCB;
	float *YBC = RCB + 3;
	float *ZBC = RCB + 6;

	float fTmp;

	if (bComputeBox)
	{
		r = sqrt(RVLDOTPRODUCT3(PcC, PcC));

		XBC[1] = 0.0f;

		RVLSCALE3VECTOR2(PcC, r, ZBC);
		XBC[0] = ZBC[2];
		XBC[2] = -ZBC[0];
		RVLNORM3(XBC, fTmp);
		RVLCROSSPRODUCT3(ZBC, XBC, YBC);
	}

	if (Z == NULL)
		return true;

	int nGridCells = gridSize * gridSize;

	memcpy(Z, ZInit, nGridCells * sizeof(float));

	float boxSize_ = (bVariableSize ? PcC[3] : boxSize);

	float halfBoxSize = 0.5f * boxSize_;

	float zf = r - halfBoxSize;
	float zb = r + halfBoxSize;
	fTmp = zb / zf;
	int halfGrid = gridSize / 2;
	fTmp = floor(fTmp * (float)halfGrid + 0.5f);
	int w = 2 * (int)fTmp;
	float gridCellSize = boxSize_ / (float)gridSize;
	float b_ = (fTmp - 0.5f) * gridCellSize;
	float Pg0B[3], Pg0C[3], dXC[3], dYC[3], Pg0C_[3], Pg0B_[3];
	int iPix_, iZ;
	float *PiC;

	Pg0B[0] = Pg0B[1] = -b_;
	Pg0B[2] = zb;
	RVLMULMX3X3TVECT(RCB, Pg0B, Pg0C);
	RVLSCALE3VECTOR(XBC, gridCellSize, dXC);
	RVLSCALE3VECTOR(YBC, gridCellSize, dYC);

	RVLCOPY3VECTOR(Pg0C, Pg0C_);
	RVLCOPY3VECTOR(Pg0B, Pg0B_);

	int mini_ = 0;
	int maxi_ = gridSize;
	int minj_ = 0;
	int maxj_ = gridSize;

	if (pBoundingBox)
	{
		if (pBoundingBox->minx > -halfBoxSize)
			mini_ = (int)floor((pBoundingBox->minx + halfBoxSize) / gridCellSize);

		if (pBoundingBox->maxx < halfBoxSize)
			maxi_ = (int)floor((pBoundingBox->maxx + halfBoxSize) / gridCellSize);

		if (pBoundingBox->miny > -halfBoxSize)
			minj_ = (int)floor((pBoundingBox->miny + halfBoxSize) / gridCellSize);

		if (pBoundingBox->maxy < halfBoxSize)
			maxj_ = (int)floor((pBoundingBox->maxy + halfBoxSize) / gridCellSize);

		if (pBoundingBox->minz > -halfBoxSize)
			zf = r - pBoundingBox->minz;

		if (pBoundingBox->maxz < halfBoxSize)
			zb = r + pBoundingBox->maxz;
	}

	int i, j, i_, j_, u, v;
	float s, z, z_, ziB;
	float PgC[3], PgB[3], PbB[3];
	bool bAhead;

	for (i = 0; i < w; i++)
	{
		RVLCOPY3VECTOR(Pg0C_, PgC);
		RVLCOPY3VECTOR(Pg0B_, PgB);

		for (j = 0; j < w; j++)
		{
			u = (int)round(fu * PgC[0] / PgC[2] + uc);

			if (u < 0 || u >= image.w)
				return false;

			v = (int)round(fv * PgC[1] / PgC[2] + vc);

			if (v < 0 || v >= image.h)
				return false;

			iPix_ = u + v * image.w;

			if (image.Element[iPix_] <= maxz)
			{
				PiC = PArray + keypointDataSize * iPix_;

				ziB = RVLDOTPRODUCT3(ZBC, PiC);

				if (bAhead = (ziB < zf))
					ziB = zf;

				s = ziB / PgB[2];

				RVLSCALE3VECTOR(PgB, s, PbB);

				i_ = (int)floor((PbB[0] + halfBoxSize) / gridCellSize);

				if (i_ >= mini_ && i_ < maxi_)
				{
					j_ = (int)floor((PbB[1] + halfBoxSize) / gridCellSize);

					if (j_ >= minj_ && j_ < maxj_)
					{
						iZ = i_ + j_ * gridSize;

						if (bAhead)
							z = zfOut;
						else
						{
							z_ = PbB[2];

							z = (z_ <= zb ? (z_ - zf) / boxSize_ : zOut);
						}

						if (z < Z[iZ])
							Z[iZ] = z;
					}
				}
			}

			RVLSUM3VECTORS(PgC, dXC, PgC);
			PgB[0] += gridCellSize;
		}

		if (j < w)
			return false;

		RVLSUM3VECTORS(Pg0C_, dYC, Pg0C_);
		Pg0B_[1] += gridCellSize;
	}

	RVLCOPY3VECTOR(PcC, C);

	if (bVariableSize)
		C[3] = PcC[3];

	return true;
}

void BoxSampler::VisualizeBox(
	cv::Mat displayImage,
	float* C,
	int iBox,
	cv::Scalar color)
{	
	float PBoxCornerB[3], PBoxCornerC[3];
	int i;
	float RCB[9];

	float* XBC = RCB;
	float* YBC = RCB + 3;
	float* ZBC = RCB + 6;

	int keypointDataSize = (bVariableSize ? 4 : 3);

	float *P = C + keypointDataSize * iBox;

	float boxSize_ = (bVariableSize ? P[3] : boxSize);

	float halfBoxSize = 0.5f * boxSize_;

	float fTmp;

	float r = sqrt(RVLDOTPRODUCT3(P, P));

	XBC[1] = 0.0f;

	RVLSCALE3VECTOR2(P, r, ZBC);
	XBC[0] = ZBC[2];
	XBC[2] = -ZBC[0];
	RVLNORM3(XBC, fTmp);
	RVLCROSSPRODUCT3(ZBC, XBC, YBC);

	PBoxCornerB[0] = halfBoxSize;
	PBoxCornerB[1] = halfBoxSize;
	PBoxCornerB[2] = r - halfBoxSize;

	RVLMULMX3X3TVECT(RCB, PBoxCornerB, PBoxCornerC);

	cv::Point2i pt1, pt2;

	pt1.x = (int)round(fu * PBoxCornerC[0] / PBoxCornerC[2] + uc);
	pt1.y = (int)round(fv * PBoxCornerC[1] / PBoxCornerC[2] + vc);

	for (i = 0; i < 4; i++)
	{
		fTmp = PBoxCornerB[0];
		PBoxCornerB[0] = PBoxCornerB[1];
		PBoxCornerB[1] = -fTmp;

		RVLMULMX3X3TVECT(RCB, PBoxCornerB, PBoxCornerC);

		pt2.x = (int)round(fu * PBoxCornerC[0] / PBoxCornerC[2] + uc);
		pt2.y = (int)round(fv * PBoxCornerC[1] / PBoxCornerC[2] + vc);

		cv::line(displayImage, pt1, pt2, color);

		pt1 = pt2;
	}

}

void BoxSampler::Visualize(
	Array2D<float> image,
	Array3D<float> featureTensor,
	float *C,
	bool bVisualizeBoxes,
	Array3D<float> *pFeatureTensor_s)
{
	cv::Mat depthImageGS;
	CreateGrayScaleImage(image, depthImageGS);

	cv::Mat displayImage(image.h, image.w, CV_8UC3);

	cv::cvtColor(depthImageGS, displayImage, CV_GRAY2BGR);

	BOXSAMPLER_DISPLAY_DATA displayData;

	displayData.displayImage = displayImage;
	displayData.keypoints.Element = new cv::Point2i[nSamples];
	displayData.keypoints.n = nSamples;
	displayData.featureTensor = featureTensor;
	displayData.pFeatureTensor_s = pFeatureTensor_s;
	displayData.C = C;
	displayData.pSampler = this;

	cv::Scalar color(0, 255, 0);

	int keypointDataSize = (bVariableSize ? 4 : 3);

	int iSample;
	float *P;
	cv::Point2i *pPt;
	uchar* pix;

	for (iSample = 0; iSample < featureTensor.c; iSample++)
	{
		P = C + keypointDataSize * iSample;

		pPt = displayData.keypoints.Element + iSample;

		pPt->x = (int)round(fu * P[0] / P[2] + uc);
		pPt->y = (int)round(fv * P[1] / P[2] + vc);

		pix = displayImage.data + 3 * (pPt->x + pPt->y * image.w);

		RVLSET3VECTOR(pix, 0, 255, 0);

		if(bVisualizeBoxes)
			VisualizeBox(displayImage, C, iSample, color);
	}

	cv::imshow("keypoints", displayImage);

	cv::setMouseCallback("keypoints", RVLNN::BoxSamplerDisplayMouseCallback2, &displayData);

	cv::waitKey();

	delete[] displayData.keypoints.Element;
}

void RVLNN::DisplayBoxFeature(
	Array2D<float> boxFeature,
	char* windowName)
{
	cv::Mat feature(boxFeature.w, boxFeature.w, CV_32FC1, boxFeature.Element);

	double minI, maxI;

	cv::minMaxLoc(feature, &minI, &maxI);

	float range = (float)(maxI - minI);

	cv::Mat featureDisplay;

	feature.convertTo(featureDisplay, CV_8U, 255.0 / range, -255.0 * minI / range);

	int minFeatureDisplayImageSize = 240;

	if (boxFeature.w >= minFeatureDisplayImageSize)
		cv::imshow(windowName, featureDisplay);
	else
	{
		cv::Mat featureDisplayExtended(minFeatureDisplayImageSize, minFeatureDisplayImageSize, CV_8UC1);
		featureDisplayExtended.setTo(0);
		int offset = (minFeatureDisplayImageSize - boxFeature.w) / 2;
		featureDisplay.copyTo(featureDisplayExtended(cv::Rect(offset, offset, boxFeature.w, boxFeature.w)));
		cv::imshow(windowName, featureDisplayExtended);
	}
}

void RVLNN::BoxSamplerDisplayMouseCallback2(int event, int x, int y, int flags, void* vpData)
{
	BOXSAMPLER_DISPLAY_DATA *pData = (BOXSAMPLER_DISPLAY_DATA *)vpData;

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		// Identify the closest sample.

		int iSelectedSample = -1;

		int iSample, dist2, minDist2, dx, dy;
		cv::Point2i *pPt;

		for (iSample = 0; iSample < pData->keypoints.n; iSample++)
		{
			pPt = pData->keypoints.Element + iSample;

			dx = x - pPt->x;
			dy = y - pPt->y;
			dist2 = dx * dx + dy * dy;

			if (iSelectedSample < 0 || dist2 < minDist2)
			{
				minDist2 = dist2;

				iSelectedSample = iSample;
			}
		}

		// Display feature.

		printf("Sample %d\n", iSelectedSample);
		//printf("Size %f\n", pData->C[4 * iSelectedSample + 3]);

		Array2D<float> boxFeature;
		boxFeature.w = boxFeature.h = pData->pSampler->gridSize;
		boxFeature.Element = pData->featureTensor.Element + iSelectedSample * pData->pSampler->gridSize * pData->pSampler->gridSize;

		DisplayBoxFeature(boxFeature, "Box Features");

		if (pData->pFeatureTensor_s)
		{
			boxFeature.Element = pData->pFeatureTensor_s->Element + iSelectedSample * pData->pSampler->gridSize * pData->pSampler->gridSize;
			DisplayBoxFeature(boxFeature, "Source");
		}

		// Display selected box feature in red.

		cv::Mat displayImage = pData->displayImage.clone();

		cv::Point2i pt = pData->keypoints.Element[iSelectedSample];

		cv::circle(displayImage, pt, 2, cv::Scalar(0, 0, 255), -1);

		pData->pSampler->VisualizeBox(displayImage, pData->C, iSelectedSample, cv::Scalar(0, 0, 255));

		cv::imshow("keypoints", displayImage);
	}
}
