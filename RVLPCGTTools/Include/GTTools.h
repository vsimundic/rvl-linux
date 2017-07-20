
#define GT_IMWIDTH 640
#define GT_IMHEIGHT 480
#define GT_SEQUENCE_LENGTH 15
#define GT_MAXLABELVAL 640*480
	

struct RVLGT_INTRINSIC_PARAMS
{
	int width;
	int height;
	float fx;
	float fy;
	float cx;
	float cy;
	float scale_factor;
};


struct RVLGT_IMAGE_DETAILS
{
	int *pClickedPoints;
	cv::Mat *pRGBImage;
	int *pGTMask;
	int *pLabelMap;
	int imageNumber;
	int *segmentColor;
	char *pWindowTitle;
	int maxLabelNo;
	bool * pixelmode;
};

// COENE ROI data structure
struct RVLGT_IMAGE_DETAILS_ROI
{
	RVLGT_IMAGE_DETAILS* details;
	int startx;
	int starty;
	int width;
	int height;
};
/////

struct RVLGT_SEGMENTATION_PARAMS
{
	int iImageNo;
	int DiffThreshold;
	float PercThreshold;
	int MinNoOfPoints;
	int MaxDist;
	int minConnectedComponentSize;		//COENE
	char *pDefaultFileLocation;
};

struct RVLGT_PCLSUPERVOXEL_PARAMS
{
	bool disable_transform;
	float voxel_resolution;
	float seed_resolution;
	float color_importance;
	float spatial_importance;
	float normal_importance;
};


//Predefined data for freiburg dataset 3
const RVLGT_INTRINSIC_PARAMS GT_DEFAULT_CAM_PARAMS = {

	640,
	480,
	528.f,
	528.f,
	320.f,
	240.f,
	1.f
};

char *CreateFileName(int iImageCnt, char *pDefaultFileLocation, char *pFileExtension);
bool ReadDepthImage(int iImageCnt, int *iDepth, char *pDefaultFileLocation);
bool ReadGTMask(int iImageCnt, int *iGTMask, char *pDefaultFileLocation);
bool ReadDataFile(int *iData, char *pDataFileName, bool isDepth);
void SaveDepthImage(cv::Mat *iDepth, int w, int h, char *pDepthFileName);
void SaveGTMask(int *iDepth, int w, int h, char *pGTFileName);
void SaveFiles(int iImageCnt, IplImage *pRgbImage, cv::Mat *pDepthMat, cv::Mat *pDepthMatTest, int w, int h, char *pDefaultFileLocation);

namespace RVL
{
	namespace PCGT
	{
		void DisplayLabelImage(
			cv::Mat labelImage, 
			cv::Mat displayImage);
		void SelectObjectMouseCallback(int event, int x, int y, int flags, void* param);
		void DisplayGroundTruthSegmentation(
			char *meshFileName,
			cv::Mat &GTLabImg,
			bool bRGB = false);
	}
}
	
template < class T >
void set_pixel(T &pcl_pixel, cv::Mat &src, int x, int y, RVLGT_INTRINSIC_PARAMS &cam_params);


template <>
void set_pixel(pcl::RGB &pcl_color_pixel, cv::Mat &src, int x, int y, RVLGT_INTRINSIC_PARAMS &cam_params);

template <>
void set_pixel(pcl::PointXYZ &xyz_pcl_pixel, cv::Mat &src, int x, int y, RVLGT_INTRINSIC_PARAMS &cam_params);

template < class T>
void load_cloud(cv::Mat &cur_mat, pcl::PointCloud<T> &pcl_cloud, RVLGT_INTRINSIC_PARAMS &cam_params);

