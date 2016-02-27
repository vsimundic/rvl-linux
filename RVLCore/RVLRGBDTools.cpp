#include "RVLConst.h"
#include "RVLArray.h"
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
