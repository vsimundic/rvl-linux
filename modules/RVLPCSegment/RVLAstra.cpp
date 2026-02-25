#include <stdio.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
//#include "atlstr.h"

#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>


//#include "stdafx.h"
//#include <pcl/io/pcd_io.h>
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Graph.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
//#include <pcl/common/common.h>
//#include <pcl/PolygonMesh.h>
//#include <pcl/surface/vtk_smoothing/vtk_utils.h>
//#include "PCLTools.h"
//#include "PCLMeshBuilder.h"
//#include "RGBDCamera.h"
#include "Mesh.h"
#ifdef RVL_ASTRA
#include "astraUtils.h"
#endif
#include "RVLAstra.h"
/////

unsigned char *img;
int16_t *imgD;

using namespace RVL;

Astra::Astra()
{
	Image = cv::Mat(480, 640, CV_8UC3);
	ImageF = cv::Mat(480, 640, CV_8UC3);
	depthImg = cv::Mat(480, 640, CV_8UC1);
	depthImgF = cv::Mat(480, 640, CV_8UC1);
	depth = cv::Mat(480, 640, CV_16UC1, cv::Scalar::all(0));
	depthF = cv::Mat(480, 640, CV_16UC1, cv::Scalar::all(0));

	fu = 519.14016473372533, fv = 518.95999218830843, uc = 316.36700343463434, vc = 250.72443412270945; //from config

	img = Image.data;
	imgD = new int16_t[640 * 480];
}

Astra::~Astra()
{

}
void Astra::InitAstra()
{
#ifdef RVL_ASTRA
	astra::initialize();

	reader = streamSet.create_reader();
	mode.set_height(480);
	mode.set_width(640);
	mode.set_fps(30);
	mode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
	reader.stream<astra::ColorStream>().set_mode(mode);
	reader.stream<astra::ColorStream>().start();

	modeD.set_height(480);
	modeD.set_width(640);
	modeD.set_fps(30);
	reader.stream<astra::DepthStream >().set_mode(modeD);
	reader.stream<astra::DepthStream>().enable_registration(true);
	reader.stream<astra::DepthStream>().start();


	reader.add_listener(listener);
	//img = Image.data;
#endif

};

void Astra::GetImageFromAstra()
{
#ifdef RVL_ASTRA
	astra_temp_update();
	cv::flip(Image, ImageF, 1);
	cv::imshow("in", ImageF);
	cv::waitKey();


	memcpy(depth.data, imgD, 640 * 480 * sizeof(int16_t));
	double minVal, maxVal;
	cv::flip(depth, depthF, 1);
	cv::minMaxLoc(depthF, &minVal, &maxVal);
	depthF.convertTo(depthImg, CV_8U, -255.0f / maxVal, 255.0f);
#endif
};


void Astra::TerminateAstra()
{
#ifdef RVL_ASTRA
	astra::terminate();
#endif
};
