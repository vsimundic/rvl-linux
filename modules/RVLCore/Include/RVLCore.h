// RVLCore.h

//#pragma once
//
//using namespace System;
//
//namespace RVLCore {
//
//	public ref class Class1
//	{
//		// TODO: Add your methods for this class here.
//	};
//}

#include "RVLPlatform.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
//#include "cv.h"
#ifdef RVLLINUX
#include "opencv2/opencv.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#else
#include "opencv2\opencv.hpp"
#include "opencv\cv.h"
#include "opencv\highgui.h"
//#include "opencv2\calib3d\calib3d_c.h"
#endif
#include "RVLConst.h"
#include "RVLMem.h"
#include "RVLArray.h"
//#ifdef NEVER
#include "RVLQListArray.h"
#include "RVLMChain.h"
#include "RVLParameterList.h"
#include "RVLBuffer.h"
#include "RVLChain.h"
#include "RVLWChain.h"
#include "RVLPtrChain.h"
#include "RVLMChain2.h"
#include "RVLMPtrChain2.h"
#include "RVLMPtrChain.h"
#include "RVL3DTools.h"
#include "RVLUtil.h"
#include "RVLKinect.h"
#include "RVLCamera.h"
#include "RVL2DPose.h"
#include "RVL3DPose.h"
#include "RVL2DCellArray.h"
#include "Rect.h"
#include "RVLAPix.h"
#include "RVLHistogram.h"
#include "RVLGUI.h"
#include "RVLTimer.h"
#include "RVLCoreObjectLib.h"
#include "RVLRelation.h"
#include "RVLAImage.h"
#include "RVLStereoVision.h"
#include "RVLPC.h"
#include "RVLVisionSystem.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#endif
#include "RVLVTKRenderer.h"
#include "RVLImageFilter.h"
#include "RVLColorDescriptor.h"


extern CvMat *RVLMatrix31;
extern CvMat *RVLMatrixHeader31;
extern CvMat *RVLMatrix33;
extern CvMat *RVLMatrixHeaderA33;
extern CvMat *RVLMatrixHeaderB33;
extern CvMat *RVLMatrixHeaderC33;
extern double RVLVector3[3];
extern double RVLMatrixA33[9];
extern double RVLMatrixB33[9];
extern double RVLMatrixC33[9];
extern int chi2_LookUpTable[1201];
extern unsigned char RVLColorMap[64 * 3];
extern double lnFactorial_LookUpTable[101];
extern int bitCount_LookUpTable[65536];
//#endif
