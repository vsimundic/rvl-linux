#pragma once

#include "DisplayVector.h"

namespace RVL
{
	class Figure
	{
	public:
		Figure();
		virtual ~Figure();
		void Create(int MemSize);

	public:
		CRVLMem *pMem;
		DWORD flags;
		QList<DisplayVector> vectorList;
		IplImage *pImage;
		CvRect ROI;
		CvPoint ROIAnchorPt;
		char *name;
		DWORD ID;
		void *vpVisualizer;
		void *vpMouseCallbackData;
		int iSelectedPix;
		CvFont font;
		int fontSize;
	};
}

