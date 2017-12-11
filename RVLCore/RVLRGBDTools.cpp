#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "RVLConst.h"
#include "RVLArray.h"
#include "RVLKinect.h"
#include "RVLRGBDTools.h"

using namespace RVL;

void RVL::DisplayDisparityMap(Array2D<short int> &depthImage,
	unsigned char *displayPixArray,
	bool bInverse,
	unsigned int format)
{
	int n = depthImage.w * depthImage.h;

	int maxDisparity = 0;
	int minDisparity = 2048;

	unsigned char *pPix = displayPixArray;
	short int *pDepth = depthImage.Element;

	int iPix;
	unsigned char I;
	int depth;

	for (iPix = 0; iPix < n; iPix++, pDepth++)
	{
		depth = (int)(*pDepth);

		if ((format == RVLRGB_DEPTH_FORMAT_DISPARITY && depth >= 0 && depth < 2047) ||
			((format == RVLRGB_DEPTH_FORMAT_1MM || format == RVLRGB_DEPTH_FORMAT_100UM)
			&& depth > 0))
		{
			if (depth > maxDisparity)
				maxDisparity = depth;

			if (depth < minDisparity)
				minDisparity = depth;
		}
	}

	int DisparityRange = maxDisparity - minDisparity;

	pDepth = depthImage.Element;

	if (bInverse)
	{
		for (iPix = 0; iPix < n; iPix++)
		{
			if (*pDepth >= 0)
			{
				I = 255 - (unsigned char)((int)(*pDepth) * 255 / maxDisparity);

				*(pPix++) = I;
				*(pPix++) = I;
				*(pPix++) = I;
			}
			else
			{
				*(pPix++) = 255;
				*(pPix++) = 255;
				*(pPix++) = 255;
			}

			pDepth++;
		}
	}
	else
	{
		for (iPix = 0; iPix < n; iPix++)
		{
			depth = (int)(*pDepth);

			if ((format == RVLRGB_DEPTH_FORMAT_DISPARITY && depth >= 0 && depth < 2047) ||
				((format == RVLRGB_DEPTH_FORMAT_1MM || format == RVLRGB_DEPTH_FORMAT_100UM)
				&& depth > 0))
			{
				I = 64 + (unsigned char)((depth - minDisparity) * (255 - 64) / DisparityRange);

				*(pPix++) = I;
				*(pPix++) = I;
				*(pPix++) = I;
			}
			else
			{
				*(pPix++) = 0;
				*(pPix++) = 0;
				*(pPix++) = 0;
			}

			pDepth++;
		}
	}
}

bool RVL::ImportDisparityImage(
	char *fileName,
	Array2D<short int> &depthImage,
	unsigned int &Format)
{
	FILE *fp;

	fp = fopen(fileName, "r");

	int Width, Height;
	int Size;

	if (fp)
	{
		char line[200];

		fgets(line, 200, fp);

		while (strstr(line, "width") == NULL)
			fgets(line, 200, fp);

		sscanf(line, "width %d", &Width);

		fscanf(fp, "height %d\n", &Height);

		fgets(line, 200, fp);

		if (strcmp(line, "1mm\n") == 0)
			Format = RVLKINECT_DEPTH_IMAGE_FORMAT_1MM;
		else if (strcmp(line, "100um\n") == 0)
			Format = RVLKINECT_DEPTH_IMAGE_FORMAT_100UM;
		else
		{
			Format = RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY;

			fclose(fp);

			return false;
		}

		Size = Width * Height;

		if (depthImage.w * depthImage.h != Size)
		{
			if (depthImage.Element)
				delete[] depthImage.Element;

			depthImage.Element = new short int[Size];
		}

		depthImage.w = Width;
		depthImage.h = Height;

		bool bOK = true;

		short int *pDisparity = depthImage.Element;

		int u, v, d;

		for (v = 0; v < Height && bOK; v++)
		{
			for (u = 0; u < Width; u++, pDisparity++)
			{
				if (!(bOK = (fscanf(fp, "%d ", &d) == 1)))
					break;

				*pDisparity = (short int)d;
			}
		}

		fclose(fp);

		return bOK;
	}
	else
		return false;
	//else
	//{
	//	char *FileName2 = RVLCreateString(FileName);

	//	sprintf(FileName2 + strlen(FileName2) - 3, "bmp");

	//	IplImage *pDisparityBmp = cvLoadImage(FileName2, false);

	//	if (pDisparityBmp)
	//	{
	//		Width = pDisparityBmp->width;
	//		Height = pDisparityBmp->height;

	//		Size = Width * Height;

	//		if (pDisparityImage->Width * pDisparityImage->Height != Size)
	//		{
	//			if (pDisparityImage->Disparity)
	//				delete[] pDisparityImage->Disparity;

	//			pDisparityImage->Disparity = new short int[Size];
	//		}

	//		pDisparityImage->Width = Width;
	//		pDisparityImage->Height = Height;

	//		IplImage *pDisparityMap = cvCreateImageHeader(cvSize(pDisparityBmp->width, pDisparityBmp->height), IPL_DEPTH_16S, 1);

	//		pDisparityMap->imageData = (char *)(pDisparityImage->Disparity);

	//		cvConvertScale(pDisparityBmp, pDisparityMap, 4);

	//		cvReleaseImage(&pDisparityBmp);

	//		cvReleaseImageHeader(&pDisparityMap);

	//		delete[] FileName2;

	//		return true;
	//	}
	//	else
	//		return false;
	//}
}

