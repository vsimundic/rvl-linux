#pragma once

#define RVLVISUALSERVO_FLAG_KINECT		0x00000001

namespace RVL
{
	class VisualServo
	{
	public:
		VisualServo();
		virtual ~VisualServo();
		void Init(char *CfgFileName);
		void CreateParamList();

	public:
		CRVLMem *pMem0;
		unsigned int flags;
		CRVLKinect kinect;
		Array2D<short int> depthImage;
		IplImage *depthDisplayImage;
		IplImage *RGBImage;
		IplImage *GSImage;
		CRVLParameterList paramList;
		double sigmaS;
		double sigmaR;
	};
}
