#define RVLRGB_DEPTH_FORMAT_DISPARITY	0
#define RVLRGB_DEPTH_FORMAT_1MM			1
#define RVLRGB_DEPTH_FORMAT_100UM		2

namespace RVL
{
	void DisplayDisparityMap(Array2D<short int> &depthImage,
		unsigned char *displayPixArray,
		bool bInverse,
		unsigned int format);
	bool ImportDisparityImage(
		char *fileName,
		Array2D<short int> &depthImage,
		unsigned int &Format);
}
