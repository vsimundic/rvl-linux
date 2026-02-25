//#include "stdafx.h"
#include "RVLPlatform.h"
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#ifdef RVLOPENNI
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#endif
#include "RVLCore2.h"
#include "RGBDCamera.h"


using namespace RVL;

RGBDCamera::RGBDCamera()
{
	//Kinect:
	depthFu0 = 584.70194597700402;
	depthUc0 = 318.55964537649561;
	depthFv0 = 585.70332900816618;
	depthVc0 = 256.14501544470505;


	//Orbec:
	//depthFu0 = 519.14016473372533;
	//depthUc0 = 316.36700343463434;
	//depthFv0 = 518.95999218830843;
	//depthVc0 = 250.72443412270945;


	depthFu = depthFu0;
	depthUc = depthUc0;
	depthFv = depthFv0;
	depthVc = depthVc0;

	w = 640;
	h = 480;
	
	zn = zMin = 0.7;
	kappa = 1 / (0.00000285 * zMin * 1000.0);		
	baseLength = 1.0;
	zf = 10.0;
	depthScale = 1.0;
}


RGBDCamera::~RGBDCamera()
{
}


void RGBDCamera::CreateParamList(CRVLMem *pMem)
{
	paramList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	paramList.Init();

	pParamData = paramList.AddParam("Camera.fu", RVLPARAM_TYPE_DOUBLE, &depthFu);
	pParamData = paramList.AddParam("Camera.fv", RVLPARAM_TYPE_DOUBLE, &depthFv);
	pParamData = paramList.AddParam("Camera.uc", RVLPARAM_TYPE_DOUBLE, &depthUc);
	pParamData = paramList.AddParam("Camera.vc", RVLPARAM_TYPE_DOUBLE, &depthVc);
	pParamData = paramList.AddParam("Camera.w", RVLPARAM_TYPE_INT, &w);
	pParamData = paramList.AddParam("Camera.h", RVLPARAM_TYPE_INT, &h);
	pParamData = paramList.AddParam("Camera.b", RVLPARAM_TYPE_DOUBLE, &baseLength);
	pParamData = paramList.AddParam("Camera.zMin", RVLPARAM_TYPE_DOUBLE, &zMin);
	pParamData = paramList.AddParam("Camera.zn", RVLPARAM_TYPE_DOUBLE, &zn);
	pParamData = paramList.AddParam("Camera.zf", RVLPARAM_TYPE_DOUBLE, &zf);
	pParamData = paramList.AddParam("Camera.baseLength", RVLPARAM_TYPE_DOUBLE, &baseLength);
	pParamData = paramList.AddParam("Camera.depthScale", RVLPARAM_TYPE_DOUBLE, &depthScale);
}

void RGBDCamera::GetPointCloud(
	Array2D<short int> *pDepthImage,
	IplImage *pRGBImage,
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
	bool bConvertToMetres,
	bool bUVDSpace)
{
#ifdef RVLOPENNI
	//pPC->header.seq = depth_image->getFrameID();
	//pPC->header.stamp = depth_image->getTimestamp();
	//pPC->header.frame_id = rgb_frame_id_;
	//pPC->height = pDepthImage->h;
	//pPC->width = pDepthImage->w;
	PC->is_dense = false;

	//pPC->points.resize(pPC->height * pPC->width);

	// Get inverse focal length for calculations below
	float fx = (float)depthFu;
	float fy = (float)depthFv;
	float fx_inv = 1.0f / fx;
	float fy_inv = 1.0f / fy;
	float cx = (float)depthUc;
	float cy = (float)depthVc;
	int w = pDepthImage->w;
	int h = pDepthImage->h;
	float fb = fx * baseLength;
	float dMax = fb / zMin;
	short int *depth = pDepthImage->Element;

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	// set xyz to Nan and rgb to 0 (black)  
	pcl::PointXYZRGBA pt;
	pt.x = pt.y = pt.z = bad_point;
	pt.b = pt.g = pt.r = 0;
	pt.a = 255; // point has no color info -> alpha = max => transparent 
	PC->points.assign(PC->points.size(), pt);

	// fill in XYZ values
	unsigned step = 1;
	unsigned skip = 0;

	int value_idx = 0;
	int point_idx = 0;

	float z;

	for (int v = 0; v < h; ++v, point_idx += skip)
	{
		for (int u = 0; u < w; ++u, ++value_idx, point_idx += step)
		{
			pcl::PointXYZRGBA &pt = PC->points[point_idx];
			/// @todo Different values for these cases
			// Check for invalid measurements

			OniDepthPixel pixel = depth[value_idx];
			if (pixel != 0
				//&&
				//pixel != depth_image->getNoSampleValue() &&
				//pixel != depth_image->getShadowValue()
				)
			{
				if (bUVDSpace)
				{
					z = (bConvertToMetres == true ? (float)pixel * 0.001f : (float)pixel) / depthScale;  // millimeters to meters
					pt.z = dMax - fb / z;
					pt.x = static_cast<float> (u)-cx;
					pt.y = static_cast<float> (v)-cy;
				}
				else
				{
					pt.z = (bConvertToMetres == true ? (float)pixel * 0.001f : (float)pixel) / depthScale;  // millimeters to meters
					pt.x = (static_cast<float> (u)-cx) * pt.z * fx_inv;
					pt.y = (static_cast<float> (v)-cy) * pt.z * fy_inv;
				}
			}
			else
			{
				pt.x = pt.y = pt.z = bad_point;
			}
		}
	}

	// fill in the RGB values

	point_idx = 0;
	RGBValue color;
	color.Alpha = 0xff;

	if (pRGBImage)
	{
		int wRGB = pRGBImage->width;
		int RGBSubsamplingRate;
		char *RGB = pRGBImage->imageData;

		if (w >= wRGB)
		{
			RGBSubsamplingRate = w / wRGB;
			for (unsigned yIdx = 0; yIdx < h; ++yIdx, point_idx += skip)
			{
				for (unsigned xIdx = 0; xIdx < w; ++xIdx, point_idx += step)
				{
					value_idx = 3 * (xIdx / RGBSubsamplingRate + (yIdx / RGBSubsamplingRate) * wRGB);

					pcl::PointXYZRGBA& pt = PC->points[point_idx];

					color.Blue = RGB[value_idx];
					color.Green = RGB[value_idx + 1];
					color.Red = RGB[value_idx + 2];

					pt.rgba = color.long_value;
				}
			}
		}
		else
		{
			RGBSubsamplingRate = wRGB / w;
			for (unsigned yIdx = 0; yIdx < h; ++yIdx, point_idx += skip)
			{
				for (unsigned xIdx = 0; xIdx < w; ++xIdx, point_idx += step)
				{
					value_idx = 3 * (xIdx * RGBSubsamplingRate + (yIdx * RGBSubsamplingRate) * wRGB);

					pcl::PointXYZRGBA& pt = PC->points[point_idx];

					color.Blue = RGB[value_idx];
					color.Green = RGB[value_idx + 1];
					color.Red = RGB[value_idx + 2];

					pt.rgba = color.long_value;
				}
			}
		}
	}
	else	// If pRGBImage == NULL, then assign white color to all points.
	{
		color.Red = color.Green = color.Blue = 0xff;

		for (unsigned yIdx = 0; yIdx < h; ++yIdx, point_idx += skip)
		{
			for (unsigned xIdx = 0; xIdx < w; ++xIdx, point_idx += step)
			{
				pcl::PointXYZRGBA &pt = PC->points[point_idx];

				pt.rgba = color.long_value;
			}
		}
	}

	PC->sensor_origin_.setZero();
	PC->sensor_orientation_.setIdentity();
#else
	printf("ERROR: OpenNI required!\n");
#endif
}