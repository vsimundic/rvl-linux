#include "RVLCore2.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#endif
#include "Util.h"
#include "Corruptor.h"

using namespace RVL;
//using namespace RVLNN;

Corruptor::Corruptor()
{
	holeSize = 2.0f;
	holeProbSig = 0.1f;
	holeProbMax = 0.5f;
	holeThr = 0.1f;
	zInvalid = 10.0f;
	noiseStd = 1.0f;
	uvNoiseStd = 2.0f;
	uvNoiseSmooth = 2.0f;
	maxz = 10.0f;
	stereoBaseLength = 0.08f;
}

Corruptor::~Corruptor()
{

}

void Corruptor::Create(char* cfgFileName)
{
	CreateParamList();

	paramList.LoadParams(cfgFileName);

	kappa = 1.0f / (0.00000285f * 0.7f * 1000.0f);
}

void Corruptor::CreateParamList()
{
	paramList.m_pMem = pMem0;

	RVLPARAM_DATA* pParamData;

	paramList.Init();

	pParamData = paramList.AddParam("Camera.minz", RVLPARAM_TYPE_FLOAT, &minz);
	pParamData = paramList.AddParam("Camera.fu", RVLPARAM_TYPE_FLOAT, &fu);
	pParamData = paramList.AddParam("Camera.fv", RVLPARAM_TYPE_FLOAT, &fv);
	pParamData = paramList.AddParam("Camera.uc", RVLPARAM_TYPE_FLOAT, &uc);
	pParamData = paramList.AddParam("Camera.vc", RVLPARAM_TYPE_FLOAT, &vc);
	pParamData = paramList.AddParam("Camera.maxz", RVLPARAM_TYPE_FLOAT, &maxz);
	pParamData = paramList.AddParam("Corruptor.holeSize", RVLPARAM_TYPE_FLOAT, &holeSize);
	pParamData = paramList.AddParam("Corruptor.holeProbSig", RVLPARAM_TYPE_FLOAT, &holeProbSig);
	pParamData = paramList.AddParam("Corruptor.holeProbMax", RVLPARAM_TYPE_FLOAT, &holeProbMax);
	pParamData = paramList.AddParam("Corruptor.holeThr", RVLPARAM_TYPE_FLOAT, &holeThr);
	pParamData = paramList.AddParam("Corruptor.zInvalid", RVLPARAM_TYPE_FLOAT, &zInvalid);
	pParamData = paramList.AddParam("Corruptor.noiseStd", RVLPARAM_TYPE_FLOAT, &noiseStd);
	pParamData = paramList.AddParam("Corruptor.uvNoiseStd", RVLPARAM_TYPE_FLOAT, &uvNoiseStd);
	pParamData = paramList.AddParam("Corruptor.uvNoiseSmooth", RVLPARAM_TYPE_FLOAT, &uvNoiseSmooth);
	pParamData = paramList.AddParam("Corruptor.stereoBaseLength", RVLPARAM_TYPE_FLOAT, &stereoBaseLength);
}

void Corruptor::Apply(
	Array2D<float> srcImage,
	Array2D<float> &HPM,
	Array2D<float> *pCorruptedImage,
	bool bComputeHPM)
{
	/// Create Hole Probability Map (HPM).

	HPM.w = srcImage.w;
	HPM.h = srcImage.h;

	int nPix = HPM.w * HPM.h;

	int u, v, u_, v_, iPix, iPix_;
	float z, d, fTmp;

	if (bComputeHPM)
	{
		if (HPM.Element == NULL)
			HPM.Element = new float[nPix];

		// Angle between the normal and projection ray.

		//float* N = new float[srcImage.w * srcImage.h];

		iPix = 0;

		int i, j, du, dv, iTmp, nNormals;
		float d_, dUVD;
		//float* N_;
		float N[3], NUVD[3], P[3], Q[3];
		float* N_, * N__, * a_, * a__;
		float a[4][3], NBuff[4][3];

		for (v = 0; v < srcImage.h; v++)
			for (u = 0; u < srcImage.w; u++, iPix++)
			{
				z = srcImage.Element[iPix];

				d = kappa * (1.0f - minz / z);

				du = 1;
				dv = 0;
				j = 0;

				for (i = 0; i < 4; i++)
				{
					u_ = u + du;

					if (u_ >= 0 && u_ < srcImage.w)
					{
						v_ = v + dv;

						if (v_ >= 0 && v_ < srcImage.h)
						{
							iPix_ = u_ + v_ * srcImage.w;

							d_ = kappa * (1.0f - minz / srcImage.Element[iPix_]);

							a[j][0] = (float)du;
							a[j][1] = (float)dv;
							a[j][2] = d_ - d;

							j++;
						}
					}

					iTmp = du;
					du = -dv;
					dv = iTmp;
				}

				nNormals = (j == 4 ? j : 1);

				for (i = 0; i < nNormals; i++)
				{
					a_ = a[i];
					a__ = a[(i + 1) % j];
					N_ = NBuff[i];

					RVLCROSSPRODUCT3(a__, a_, N_);
				}

				//N_ = N + 3 * iPix;

				RVLNULL3VECTOR(NUVD);

				for (i = 0; i < nNormals; i++)
				{
					N__ = NBuff[i];

					RVLSUM3VECTORS(NUVD, N__, NUVD);
				}

				RVLNORM3(NUVD, fTmp);

				Q[0] = (float)u - uc;
				Q[1] = (float)v - vc;
				Q[2] = d;

				dUVD = RVLDOTPRODUCT3(NUVD, Q);

				N[0] = NUVD[0] * fu;
				N[1] = NUVD[1] * fu;
				N[2] = NUVD[2] * kappa - dUVD;

				RVLNORM3(N, fTmp);

				P[0] = z * Q[0] / fu;
				P[1] = z * Q[1] / fu;
				P[2] = z;

				RVLNORM3(P, fTmp);

				HPM.Element[iPix] = -RVLDOTPRODUCT3(N, P);
			}

		// Shadows.

		if (RVLABS(stereoBaseLength) > 1e-10)
		{
			int uStart, uEnd, du;

			if (stereoBaseLength > 0.0f)
			{
				uStart = srcImage.w - 1;
				uEnd = 0;
				du = -1;
			}
			else
			{
				uStart = 0;
				uEnd = srcImage.w - 1;
				du = 1;
			}

			float b = fu * stereoBaseLength;

			float zShadowSrc, zRay, k, kmax;

			for (v = 0; v < srcImage.h; v++)
			{
				zShadowSrc = maxz;
				k = 1.0f;
				kmax = 0.0f;

				for (u = uStart; u != uEnd; u += du)
				{
					iPix = u + v * srcImage.w;

					z = srcImage.Element[iPix];

					zRay = zShadowSrc * (1.0f + 1.0f / (b / (zShadowSrc * k) - 1.0f));

					if (z < zRay || k >= kmax)
					{
						zShadowSrc = z;

						kmax = b / z;

						k = 1.0f;
					}
					else
					{
						srcImage.Element[iPix] = zInvalid;

						k += 1.0f;
					}
				}
			}
		}
	}

	///

	//delete[] N;

	/// Generate corrupted image.

	if (pCorruptedImage)
	{
		float noise;

		float holeProbSig2 = holeProbSig * holeProbSig;

		// uv-noise

		Array2D<float> uNoiseMap;
		uNoiseMap.w = srcImage.w;
		uNoiseMap.h = srcImage.h;
		uNoiseMap.Element = new float[srcImage.w * srcImage.h * 2];

		Array2D<float> vNoiseMap;
		vNoiseMap.w = srcImage.w;
		vNoiseMap.h = srcImage.h;
		vNoiseMap.Element = new float[srcImage.w * srcImage.h * 2];

		for (iPix = 0; iPix < nPix; iPix++)
		{
			noise = GaussRandBM<float>(uvNoiseStd);
			uNoiseMap.Element[iPix] = noise;
			noise = GaussRandBM<float>(uvNoiseStd);
			vNoiseMap.Element[iPix] = noise;
		}

		Array2D<float> uNoiseMapBlured;
		uNoiseMapBlured.w = srcImage.w;
		uNoiseMapBlured.h = srcImage.h;
		uNoiseMapBlured.Element = new float[srcImage.w * srcImage.h * 2];

		GaussianFilter2D(uNoiseMap, uvNoiseSmooth, uNoiseMapBlured);

		Array2D<float> vNoiseMapBlured;
		vNoiseMapBlured.w = srcImage.w;
		vNoiseMapBlured.h = srcImage.h;
		vNoiseMapBlured.Element = new float[srcImage.w * srcImage.h * 2];

		GaussianFilter2D(vNoiseMap, uvNoiseSmooth, vNoiseMapBlured);

		//FILE* fp = fopen("C:\\RVL\\ExpRez\\uvnoise.txt", "w");
		//
		//for (iPix = 0; iPix < nPix; iPix++)
		//	fprintf(fp, "%f\t%f\t%f\t%f\n", uNoiseMap.Element[iPix], vNoiseMap.Element[iPix], uNoiseMapBlured.Element[iPix], vNoiseMapBlured.Element[iPix]);
		//
		//fclose(fp);

		delete[] uNoiseMap.Element;
		delete[] vNoiseMap.Element;

		for (iPix = 0; iPix < nPix; iPix++)
		{
			u = iPix % srcImage.w;
			v = iPix / srcImage.w;

			noise = uNoiseMapBlured.Element[iPix];

			u_ = u + (int)round(noise);

			if (u_ < 0)
				u_ = 0;
			else if (u_ >= srcImage.w)
				u_ = srcImage.w - 1;

			noise = vNoiseMapBlured.Element[iPix];

			v_ = v + (int)round(noise);

			if (v_ < 0)
				v_ = 0;
			else if (v_ >= srcImage.h)
				v_ = srcImage.h - 1;

			iPix_ = u_ + v_ * srcImage.w;

			pCorruptedImage->Element[iPix] = srcImage.Element[iPix_];
		}

		delete[] uNoiseMapBlured.Element;
		delete[] vNoiseMapBlured.Element;

		// Holes and z-noise.

		Array2D<float> holeSeedMap;

		holeSeedMap.w = srcImage.w;
		holeSeedMap.h = srcImage.h;
		holeSeedMap.Element = new float[nPix];

		float holeProbability;
		float cs;

		for (iPix = 0; iPix < nPix; iPix++)
		{
			cs = HPM.Element[iPix];

			if (cs >= 0)
			{
				fTmp = (float)rand() / (float)RAND_MAX;

				holeProbability = holeProbMax * exp(-cs * cs / holeProbSig2);

				holeSeedMap.Element[iPix] = (fTmp < holeProbability ? (float)rand() / (float)RAND_MAX : 0.0f);
			}
			else
				holeSeedMap.Element[iPix] = 0.0f;
		}

		Array2D<float> bluredHoleSeedMap;

		bluredHoleSeedMap.w = srcImage.w;
		bluredHoleSeedMap.h = srcImage.h;
		bluredHoleSeedMap.Element = new float[nPix];

		GaussianFilter2D(holeSeedMap, holeSize, bluredHoleSeedMap);

		//Array2D<float> displayImg;
		//displayImg.w = srcImage.w;
		//displayImg.h = srcImage.h;

		//cv::Mat bluredHoleSeedMapDisplayImg;
		//displayImg.Element = bluredHoleSeedMap2;
		//CreateGrayScaleImage(displayImg, bluredHoleSeedMapDisplayImg);
		//cv::imshow("Blured Hole Seed Map", bluredHoleSeedMapDisplayImg);

		pCorruptedImage->w = srcImage.w;
		pCorruptedImage->h = srcImage.h;

		if(pCorruptedImage->Element == NULL)
			pCorruptedImage->Element = new float[nPix];

		for (iPix = 0; iPix < nPix; iPix++)
		{
			z = pCorruptedImage->Element[iPix];

			if (bluredHoleSeedMap.Element[iPix] > holeThr || z > maxz || HPM.Element[iPix] < 0.0f)
				pCorruptedImage->Element[iPix] = zInvalid;
			else
			{
				d = kappa * (1.0f - minz / z);

				noise = GaussRandBM<float>(noiseStd);

				pCorruptedImage->Element[iPix] = minz / (1.0f - (d + noise) / kappa);
			}
		}

		delete[] holeSeedMap.Element;
		delete[] bluredHoleSeedMap.Element;
	}
}

void Corruptor::Visualize(Array2D<float> HPM)
{
	cv::Mat HPMDisplayImg;
	CreateGrayScaleImage(HPM, HPMDisplayImg);

	cv::imshow("HPM", HPMDisplayImg);	

	cv::waitKey();
}

void RVL::GaussianFilter2D(
	Array2D<float> inImage,
	float sigma,
	Array2D<float> outImage)
{
	float filterA = 0.5f / (sigma * sigma);

	int filterWinSize = (int)ceil(2.0f * sigma);

	float* filter = new float[2 * filterWinSize + 1];

	float* filter_ = filter + filterWinSize;

	float fTmp = 0.0f;

	int i, iPix_;
	float fTmp2;

	for (i = -filterWinSize; i <= filterWinSize; i++)
	{
		fTmp2 = exp(-filterA * (float)(i * i));
		filter_[i] = fTmp2;
		fTmp += fTmp2;
	}

	for (i = -filterWinSize; i <= filterWinSize; i++)
		filter_[i] /= fTmp;

	int nPix = inImage.w * inImage.h;

	float* tmpImage = new float[nPix];

	int iPix = 0;

	int u, v, iStart, iEnd;

	for (v = 0; v < inImage.h; v++)
		for (u = 0; u < inImage.w; u++, iPix++)
		{
			iStart = -filterWinSize;

			if (u + iStart < 0)
				iStart = -u;

			iEnd = filterWinSize;

			if (u + iEnd >= inImage.w)
				iEnd = inImage.w - 1 - u;

			iPix_ = u + iStart + v * inImage.w;

			fTmp = 0.0f;

			for (i = iStart; i <= iEnd; i++, iPix_++)
				fTmp += (filter_[i] * inImage.Element[iPix_]);

			tmpImage[iPix] = fTmp;
		}

	int iCol = 0;

	for (u = 0; u < inImage.w; u++)
	{
		iPix = iCol;

		iCol++;

		for (v = 0; v < inImage.h; v++, iPix += inImage.w)
		{
			iStart = -filterWinSize;

			if (v + iStart < 0)
				iStart = -v;

			iEnd = filterWinSize;

			if (v + iEnd >= inImage.h)
				iEnd = inImage.h - 1 - v;

			iPix_ = u + (v + iStart) * inImage.w;

			fTmp = 0.0f;

			for (i = iStart; i <= iEnd; i++, iPix_ += inImage.w)
				fTmp += (filter_[i] * tmpImage[iPix_]);

			outImage.Element[iPix] = fTmp;
		}
	}

	delete[] filter;
	delete[] tmpImage;
}
