//#include "stdafx.h"
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#include "RVLCore2.h"
#include "RGBDCamera.h"


using namespace RVL;

RGBDCamera::RGBDCamera()
{
	depthFu0 = 584.70194597700402;
	depthUc0 = 318.55964537649561;
	depthFv0 = 585.70332900816618;
	depthVc0 = 256.14501544470505;
	depthFu = depthFu0;
	depthUc = depthUc0;
	depthFv = depthFv0;
	depthVc = depthVc0;
}


RGBDCamera::~RGBDCamera()
{
}


void RGBDCamera::GetPointCloud(
	Array2D<short int> *pDepthImage,
	IplImage *pRGBImage,
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC)
{
	//pPC->header.seq = depth_image->getFrameID();
	//pPC->header.stamp = depth_image->getTimestamp();
	//pPC->header.frame_id = rgb_frame_id_;
	//pPC->height = pDepthImage->h;
	//pPC->width = pDepthImage->w;
	PC->is_dense = false;

	//pPC->points.resize(pPC->height * pPC->width);

	// Get inverse focal length for calculations below
	float fx_inv = 1.0f / (float)depthFu;
	float fy_inv = 1.0f / (float)depthFv;
	float cx = (float)depthUc;
	float cy = (float)depthVc;
	int w = pDepthImage->w;
	int h = pDepthImage->h;
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
				pt.z = (float)pixel * 0.001f;  // millimeters to meters
				pt.x = (static_cast<float> (u)-cx) * pt.z * fx_inv;
				pt.y = (static_cast<float> (v)-cy) * pt.z * fy_inv;
			}
			else
			{
				pt.x = pt.y = pt.z = bad_point;
			}
		}
	}

	// fill in the RGB values

	char *RGB = pRGBImage->imageData;

	point_idx = 0;
	RGBValue color;
	color.Alpha = 0xff;

	for (unsigned yIdx = 0; yIdx < h; ++yIdx, point_idx += skip)
	{
		for (unsigned xIdx = 0; xIdx < w; ++xIdx, point_idx += step)
		{
			value_idx = 3 * (xIdx / 2 + (yIdx / 2) * (w / 2));

			pcl::PointXYZRGBA &pt = PC->points[point_idx];

			color.Blue = RGB[value_idx];
			color.Green = RGB[value_idx + 1];
			color.Red = RGB[value_idx + 2];

			pt.rgba = color.long_value;
		}
	}
	PC->sensor_origin_.setZero();
	PC->sensor_orientation_.setIdentity();
}