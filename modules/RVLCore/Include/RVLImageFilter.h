#pragma once

class CRVLImageFilter
{
public:
	CRVLImageFilter(void);
	virtual ~CRVLImageFilter(void);
	static IplImage* RVLFilterNHS(IplImage *pImage);
	static IplImage* RVLFilterHEQGray(IplImage *pImage);
	static IplImage* RVLFilterHEQRGB(IplImage *pImage);
};
