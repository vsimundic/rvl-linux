#pragma once

//Header file ==> Function Declarations 

namespace RVL
{
	class PanTilt
	{
	public:
		int depthScale;
		int RGBScale;
		int TiltOffset;		// deg

		PanTilt(); //default contructor
		virtual ~PanTilt(); //destructor
		void Clear();

		void RVLSaveDepthImage(short *DepthImage, int w, int h, char *DepthFileName);
		void RVLSetFileNumber(char *FileName, char *Extension, int n);
		char* RVLCreateFileName(char *SrcFileName, char *SrcExtension, int n, char *TgtExtension);

		bool Init(int depthScale_in, int RGBScale_in, int TiltOffset_in);
		int goTo_captureImg(int dPan_in, int dTilt_in, Array2D<short int> *depthImage_array, IplImage *&pRGBImage);

		int SequentialScan(int argc, char* argv[]);

		void *vpSP;
		char* inData;
		unsigned char *outData;
		int dataLength;
		int readResult;

		bool bDepth, bColor;
		int width, height;
		short *pDepth_;
		const openni::DepthPixel* pDepthRow;
		int rowSize;
		const openni::DepthPixel* pDepth__;
		const openni::RGB888Pixel* pRGBRow;
		char *pPixRGB;
		int rowSizeRGB;
		const openni::RGB888Pixel* pPix;
		openni::VideoMode DepthMode;
		int u, v;

		short *DepthImage;
		//IplImage *pRGBImage;

		int changedIndex;

		//*** Initialize OpenNI device
		openni::Device OpenNIDevice;
		openni::VideoStream DepthStream;
		openni::VideoStream ColorStream;
		openni::VideoStream *Stream[2];
		openni::VideoFrameRef DepthFrame;
		openni::VideoFrameRef ColorFrame;

		openni::Status rc;

	private:
		//member variables
	};
}