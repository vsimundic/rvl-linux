#include "RVLCore.h"
#include "RVLImageFilter.h"

CRVLImageFilter::CRVLImageFilter(void)
{
}

CRVLImageFilter::~CRVLImageFilter(void)
{
}

IplImage* CRVLImageFilter::RVLFilterNHS(IplImage *pImage)
{
	IplImage *pImageR = cvCreateImage(cvSize(pImage->width, pImage->height), pImage->depth, pImage->nChannels);

	//Pronalazimo histogram vrijednosti po kanalu
	int hist[3*256], r, g, b;
	memset(&hist, 0, 3*256*sizeof(int));
	uchar *ptr, *ptr2;
	 
	 for (int j = 0; j < pImage->height; j++)
	 {
		 for (int i = 0; i < pImage->width; i++)
		 {
			 ptr = (uchar*)(pImage->imageData + j * pImage->widthStep);
			 if (pImage->channelSeq[0] == 'B')
			 {
			 	 b = ptr[i * 3];
			 	 g = ptr[i * 3 + 1];
				 r = ptr[i * 3 + 2];
			 }
			 else
			 {
				 r = ptr[i * 3];
				 g = ptr[i * 3 + 1];
				 b = ptr[i * 3 + 2];
			 }
			 hist[r]++;
			 hist[256 + g]++;
			 hist[512 + b]++;
		 }
	 }
	 //pronalazimo nisku, visoku i srednju vrijenost po kanalu
	 int hi[3], lo[3], md[3];
	 memset(&hi, 0, 3*sizeof(int));
	 memset(&lo, 0, 3*sizeof(int));
	 memset(&md, 0, 3*sizeof(int));
	 bool hiF[3], loF[3];
	 memset(&hiF, 0, 3*sizeof(bool));
	 memset(&loF, 0, 3*sizeof(bool));
	 int noPixel = pImage->width * pImage->height;
	 int cumLO[3], cumHI[3];
	 memset(&cumLO, 0, 3*sizeof(int));
	 memset(&cumHI, 0, 3*sizeof(int));
	 float highLevel = 0.995, lowLevel = 0.005;
	 float avg[3];
	 memset(&avg, 0, 3*sizeof(float));

	 for (int i = 0; i < 256; i++)
	 {
		 for (int j = 0; j < 3; j++)
		 {
			 if (!loF[j])
			 {
				 cumLO[j] += hist[j*256 + i];
				 if (cumLO[j] > lowLevel * noPixel)
				 {
					 lo[j] = i;
					 loF[j] = TRUE;
				 }
			 }
			 if (!hiF[j])
			 {
				 cumHI[j] += hist[j*256 + i];
				 if (cumHI[j] > highLevel * noPixel)
				 {
					 hi[j] = i;
					 hiF[j] = TRUE;
				 }
			 }
			 avg[j] += i * hist[j*256 + i];
		 }

	 }
	 md[0] = ceilf(avg[0]/(float)noPixel);
	 md[1] = ceilf(avg[1]/(float)noPixel);
	 md[2] = ceilf(avg[2]/(float)noPixel);
	 //Racunamo gammu za svaki kanal
	 float gamma[3];
	 memset(&gamma, 0, 3*sizeof(float));
	 gamma[0] = log10(0.5f) / log10((float)(md[0] - lo[0]) / (float)(hi[0] - lo[0]));
	 gamma[1] = log10(0.5f) / log10((float)(md[1] - lo[1]) / (float)(hi[1] - lo[1]));
	 gamma[2] = log10(0.5f) / log10((float)(md[2] - lo[2]) / (float)(hi[2] - lo[2]));
	 //Racunamo lookup tablicu za nove vrijednosti po R, G i B kanalima
	 int val = 0;
	 int lookup[3*256];
	 memset(&lookup, 0, 3*256*sizeof(int));
	 float newInt = 0.0;
	 for (int i = 0; i < 256; i++)
	 {
		 for (int j = 0; j < 3; j++)
		 {
			 val = i - lo[j];
			 //if (val < lo[j])
			 //	 lookup[j*256 + i] = lo[j];
			 //else if ((val + lo[j]) > hi[j])
			 //	 lookup[j*256 + i] = hi[j];
			 //else
			 newInt = ceil(255 * pow((float)val/(float)(hi[j] - lo[j]),gamma[j]));
			 if (newInt < 0.0)
				 lookup[j*256 + i] = 0;
			 else if (newInt > 255.0)
				 lookup[j*256 + i] = 255;
			 else
				 lookup[j*256 + i] = (int)newInt;

		 }
	 }
	 //Mjenjamo sliku pomocu lookup tablice
	 for (int j = 0; j < pImageR->height; j++)
	 {
		 for (int i = 0; i < pImageR->width; i++)
		 {
			 ptr = (uchar*)(pImageR->imageData + j * pImageR->widthStep);
			 ptr2 = (uchar*)(pImage->imageData + j * pImage->widthStep);
			 if (pImageR->channelSeq[0] == 'B')
			 {
			 	 b = ptr2[i * 3];
			 	 g = ptr2[i * 3 + 1];
				 r = ptr2[i * 3 + 2];
				 ptr[i * 3] = (uchar)lookup[512 + b];
				 ptr[i * 3 + 1] = (uchar)lookup[256 + g];
				 ptr[i * 3 + 2] = (uchar)lookup[r];
			 }
			 else
			 {
				 r = ptr2[i * 3];
				 g = ptr2[i * 3 + 1];
				 b = ptr2[i * 3 + 2];
				 ptr[i * 3] = (uchar)lookup[r];
				 ptr[i * 3 + 1] = (uchar)lookup[256 + g];
				 ptr[i * 3 + 2] = (uchar)lookup[512 + b];
			 }
		 }
	 }

	return pImageR;
}

IplImage* CRVLImageFilter::RVLFilterHEQGray(IplImage *pImage)
{
	IplImage *pImageR = cvCreateImage(cvSize(pImage->width, pImage->height), IPL_DEPTH_8U, 1 );
	cvEqualizeHist(pImage, pImageR);

	return pImageR;
}

IplImage* CRVLImageFilter::RVLFilterHEQRGB(IplImage *pImage)
{
	////Histogram equalization
	CvMat *r = cvCreateMat(pImage->height, pImage->width, CV_8UC1);
	CvMat *g = cvCreateMat(pImage->height, pImage->width, CV_8UC1);
	CvMat *b = cvCreateMat(pImage->height, pImage->width, CV_8UC1);
	CvMat *rE = cvCreateMat(pImage->height, pImage->width, CV_8UC1);
	CvMat *gE = cvCreateMat(pImage->height, pImage->width, CV_8UC1);
	CvMat *bE = cvCreateMat(pImage->height, pImage->width, CV_8UC1);
	cvSplit(pImage, r, g, b, NULL);
	cvEqualizeHist(r, rE);
	cvEqualizeHist(g, gE);
	cvEqualizeHist(b, bE);
	IplImage *pImageR_E = cvCreateImage(cvSize(pImage->width, pImage->height), pImage->depth, pImage->nChannels);
	cvMerge(rE, gE, bE, NULL, pImageR_E);
	cvReleaseMat(&r);
	cvReleaseMat(&g);
	cvReleaseMat(&b);

	return pImageR_E;
}