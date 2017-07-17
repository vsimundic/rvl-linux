#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>
#include <sstream>
//#include "atlstr.h"
//#include "stdafx.h"

#include "RVLCore2.h"

//#include "PCLTools.h"
//#include "PCLMeshBuilder.h"

#include "GTTools.h"

using namespace RVL;

//Create and return full fileName
char *CreateFileName(int iImageCnt, char *pDefaultFileLocation, char *pFileExtension)
{
	char currentFileNo[6];
	sprintf(currentFileNo, "%05d", iImageCnt);

	char fullFileName[300];

	strcpy(fullFileName, pDefaultFileLocation);
	strcat(fullFileName, "image-");
	strcat(fullFileName, currentFileNo);
	strcat(fullFileName, pFileExtension);

	return  RVLCreateString(fullFileName);
}

bool ReadDepthImage(int iImageCnt, int *iDepth, char *pDefaultFileLocation)
{
	//Get depth image
	return ReadDataFile(iDepth, CreateFileName(iImageCnt, pDefaultFileLocation, ".txt"), true);

}

bool ReadGTMask(int iImageCnt, int *iGTMask, char *pDefaultFileLocation)
{
	//Get GT file
	return ReadDataFile(iGTMask, CreateFileName(iImageCnt, pDefaultFileLocation, "_GT.txt"), false);
}

//Read depth image or GT file
bool ReadDataFile(int *iData, char *pDataFileName, bool isDepth)
{
	FILE *fpData = fopen(pDataFileName, "r");

	int Width, Height;
	int u, v, d;

	char line[700];

	if (fpData)
	{
		fgets(line, 200, fpData);

		while (strstr(line, "width") == NULL)
			fgets(line, 200, fpData);

		sscanf(line, "width %d", &Width);

		fscanf(fpData, "height %d\n", &Height);

		if (isDepth)
		{
			fgets(line, 200, fpData);  //If 1mm or 100um is written to depth map
		}

		for (v = 0; v < Height; v++)
		{
			for (u = 0; u < Width; u++, iData++)
			{
				if (!fscanf(fpData, "%d ", &d))
					break;
				*iData = d;
			}
		}

		fclose(fpData);
		return true;
	}
	else
	{
		return false;
	}

}

void SaveDepthImage(cv::Mat *iDepth, int w, int h, char *pDepthFileName)
{
	FILE *fpDepth = fopen(pDepthFileName, "w");

	fprintf(fpDepth, "width %d\n", w);
	fprintf(fpDepth, "height %d\n", h);
	fprintf(fpDepth, "1mm\n");

	for (int r = 0; r<h; r++)
	{
		for (int c = 0; c<w; c++)
		{
			fprintf(fpDepth, "%d ", iDepth->at<ushort>(r, c));
		}
		fprintf(fpDepth, "\n");
	}
	fclose(fpDepth);
}

void SaveGTMask(int *iDepth, int w, int h, char *pGTFileName)
{

	FILE *fpGT = fopen(pGTFileName, "w");

	fprintf(fpGT, "width %d\n", w);
	fprintf(fpGT, "height %d\n", h);


	for (int r = 0; r<h; r++)
	{
		for (int c = 0; c<w; c++)
		{
			fprintf(fpGT, "%d ", iDepth[r*w + c]);
		}
		fprintf(fpGT, "\n");
	}



	fclose(fpGT);
}

void SaveFiles(int iImageCnt, IplImage *rgbImage, cv::Mat *depthMat, cv::Mat *depthMatTest, int w, int h, char *pDefaultFileLocation)
{
	//CString currentFileName;
	////char myString[200];

	//CString currentFileNo = "";
	//currentFileNo.Format(_T("%05d"), iImageCnt);


	////Save RGB image as BMP
	//currentFileName = defaultFileLocation + "image-" + currentFileNo + ".bmp";
	////strcpy(myString, CT2A(currentFileName));
	//cvSaveImage(CT2A(currentFileName), rgbImage);

	//Save RGB image as BMP
	char *pRGBFileName = CreateFileName(iImageCnt, pDefaultFileLocation, ".bmp");
	cvSaveImage(pRGBFileName, rgbImage);
	


	//Save Depth image as txt
	//currentFileName = defaultFileLocation + "image-" + currentFileNo + ".txt";
	//SaveDepthImage(depthMat, w, h, CT2A(currentFileName));
	char *pDepthFileName = CreateFileName(iImageCnt, pDefaultFileLocation, ".txt");
	SaveDepthImage(depthMat, w, h, pDepthFileName);

	//Save Test Depth image as txt
	//currentFileName = defaultFileLocation + "image-" + currentFileNo + "_T.txt";
	//SaveDepthImage(depthMatTest, w, h, CT2A(currentFileName));
	char *pDepthTestFileName = CreateFileName(iImageCnt, pDefaultFileLocation, "_T.txt");
	SaveDepthImage(depthMatTest, w, h, pDepthTestFileName);

	//Rescale rgb
	cv::Mat rgbMat = cv::cvarrToMat(rgbImage).clone();
	cv::Mat rgbMatX2;
	resize(rgbMat, rgbMatX2, cv::Size(rgbMat.cols * 2, rgbMat.rows * 2), 2, 2);
	//currentFileName = defaultFileLocation + "image-" + currentFileNo + "_R.bmp";
	//imwrite(cv::String(CT2CA(currentFileName)), rgbMatX2);
	//cvSaveImage(CT2A(currentFileName), &rgbMatX2);
	char *pRGBRescaledFileName = CreateFileName(iImageCnt, pDefaultFileLocation, "_R.bmp");
	imwrite(cv::String(pRGBRescaledFileName), rgbMatX2);
	


	//Save PCD file    
	RVLGT_INTRINSIC_PARAMS cam_params = GT_DEFAULT_CAM_PARAMS;
	pcl::PointCloud < pcl::RGB >          current_color_cloud;
	pcl::PointCloud < pcl::PointXYZ >     current_xyz_cloud;
	pcl::PointCloud < pcl::PointXYZRGBA > current_xyzrgb_cloud;



	load_cloud <pcl::RGB>(rgbMatX2, current_color_cloud, cam_params);

	load_cloud <pcl::PointXYZ>(*depthMat, current_xyz_cloud, cam_params);

	copyPointCloud(current_xyz_cloud, current_xyzrgb_cloud);
	for (size_t ii = 0; ii < current_color_cloud.size(); ++ii)
		current_xyzrgb_cloud.points[ii].rgba = current_color_cloud.points[ii].rgba;

	//currentFileName = defaultFileLocation + "image-" + currentFileNo + ".pcd";
	//std::string strCurrentFileName = CT2CA(currentFileName);
	//std::string strCurrentFileName = CT2CA(pRGBRescaledFileName);
	//pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZRGBA>(strCurrentFileName, current_xyzrgb_cloud);
	char *pPCDFileName = CreateFileName(iImageCnt, pDefaultFileLocation, ".pcd");
	pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZRGBA>(pPCDFileName, current_xyzrgb_cloud);
	//OR
	//pcl::io::savePCDFileASCII<pcl::PointXYZRGBA>(strCurrentFileName, current_xyzrgb_cloud);
	

}


void PCGT::DisplayLabelImage(cv::Mat labelImage, cv::Mat displayImage)
{
	double fMinLabel, fMaxLabel;

	cv::minMaxLoc(labelImage, &fMinLabel, &fMaxLabel);

	int nObjects = (int)round(fMaxLabel) + 1;

	unsigned char *color = new unsigned char[3 * nObjects];

	unsigned char *color_;
	
	int iObject;

	for (iObject = 0; iObject < nObjects; iObject++)
	{
		color_ = color + 3 * iObject;

		color_[0] = rand() % 255;
		color_[1] = rand() % 255;
		color_[2] = rand() % 255;
	}
	
	int nPix = labelImage.size().width * labelImage.size().height;
	
	unsigned char *pixTgt = displayImage.data;

	int u, v;

	for (v = 0; v < labelImage.size().height; v++)
	{
		for (u = 0; u < labelImage.size().width; u++, pixTgt += 3)
		{
			iObject = (int)labelImage.at<cv::Vec3b>(v, u)[0];

			color_ = color + 3 * iObject;

			RVLCOPY3VECTOR(color_, pixTgt);
		}
	}
}

void PCGT::SelectObjectMouseCallback(int event, int x, int y, int flags, void* param)
{
	cv::Mat GTImage = *(cv::Mat *)param;

	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		int iSelectedObject = (int)GTImage.at<cv::Vec3b>(y, x)[0];

		printf("Selected object %d\n", iSelectedObject);
	}
}

void PCGT::DisplayGroundTruthSegmentation(
	char *meshFileName,
	cv::Mat &GTLabImg,
	bool bRGB)
{
	char *GTFileName = RVLCreateFileName(meshFileName, ".ply", -1, "a.png");

	GTLabImg = cv::imread(GTFileName);

	cv::Mat displayImage(480, 640, CV_8UC3, cv::Scalar::all(0));

	PCGT::DisplayLabelImage(GTLabImg, displayImage);

	cv::imshow("GT Segmentation", displayImage);
	cv::setMouseCallback("GT Segmentation", PCGT::SelectObjectMouseCallback, &GTLabImg);

	if (bRGB)
	{
		char *RGBFileName;

		RGBFileName = RVLCreateFileName(meshFileName, ".ply", -1, ".png");

		cv::Mat RGBImage = cv::imread(RGBFileName);

		cv::imshow("RGB Image", RGBImage);
	}
}

template < class T >
void set_pixel(T &pcl_pixel, cv::Mat &src, int x, int y, RVLGT_INTRINSIC_PARAMS &cam_params)
{
	cerr << "set_pixel: Error - do not have proper specification for type: " << typeid(T).name() << endl;
	throw;
}


template <>
void set_pixel(pcl::RGB &pcl_color_pixel, cv::Mat &src, int x, int y, RVLGT_INTRINSIC_PARAMS &cam_params)
{
	uint32_t rgb;
	cv::Vec3b cur_rgb = src.at<cv::Vec3b>(y, x);// b,g,r
	rgb = (static_cast<int> (cur_rgb[2])) << 16 |
		(static_cast<int> (cur_rgb[1])) << 8 |
		(static_cast<int> (cur_rgb[0]));

	pcl_color_pixel.rgba = static_cast < uint32_t > (rgb);
}

template <>
void set_pixel(pcl::PointXYZ &xyz_pcl_pixel, cv::Mat &src, int x, int y, RVLGT_INTRINSIC_PARAMS &cam_params)
{
	xyz_pcl_pixel.z = src.at<unsigned short>(y * cam_params.width + x) * cam_params.scale_factor;
	xyz_pcl_pixel.x = xyz_pcl_pixel.z * (x - cam_params.cx) / cam_params.fx;
	xyz_pcl_pixel.y = xyz_pcl_pixel.z * (y - cam_params.cy) / cam_params.fy;
}

template < class T>
void load_cloud(cv::Mat &cur_mat, pcl::PointCloud<T> &pcl_cloud, RVLGT_INTRINSIC_PARAMS &cam_params)
{

	cv::Size s = cur_mat.size();
	int width = s.width;
	int height = s.height;
	int nchannels = cur_mat.channels();
	int step = cur_mat.step;

	pcl_cloud.width = width;
	pcl_cloud.height = height;
	pcl_cloud.is_dense = true;
	pcl_cloud.points.resize(width * height);

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			T   current_pixel;

			set_pixel <T>(current_pixel, cur_mat, x, y, cam_params);

			pcl_cloud(x, y) = current_pixel;

		}
	}
}