#pragma once

namespace RVL
{
	typedef union
	{
		struct
		{
			unsigned char Blue;
			unsigned char Green;
			unsigned char Red;
			unsigned char Alpha;
		};
		float float_value;
		unsigned int long_value;
	} RGBValue;

	class RGBDCamera
	{
	public:
		RGBDCamera();
		virtual ~RGBDCamera();
		void CreateParamList(CRVLMem *pMem);
		void GetPointCloud(
			Array2D<short int> *pDepthImage,
			IplImage *pRGBImage,
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
			bool bConvertToMetres = true,
			bool bUVDSpace = false);

	public:
		int w;
		int h;
		double depthFu;
		double depthFv;
		double depthUc;
		double depthVc;
		double depthFu0;
		double depthFv0;
		double depthUc0;
		double depthVc0;
		double rgbFu;
		double rgbFv;
		double rgbUc;
		double rgbVc;
		double baseLength;
		double zMin;
		double zn;
		double zf;
		double kappa;
		double depthScale;
		CRVLParameterList paramList;
	};
}
