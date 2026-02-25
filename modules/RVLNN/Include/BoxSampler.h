#pragma once

namespace RVL
{
	class BoxSampler;

	namespace RVLNN
	{
		struct BOXSAMPLER_DISPLAY_DATA
		{
			BoxSampler* pSampler;
			cv::Mat displayImage;
			Array<cv::Point2i> keypoints;
			Array3D<float> featureTensor;
			Array3D<float> *pFeatureTensor_s;
			float* C;
		};

		void BoxSamplerDisplayMouseCallback2(int event, int x, int y, int flags, void* vpData);		
		void DisplayBoxFeature(
			Array2D<float> boxFeature,
			char *windowName);
	}

	class BoxSampler
	{
	public:
		BoxSampler();
		virtual ~BoxSampler();
		void Create(char *cfgFileName);
		void Apply(
			Array2D<float> image,
			Array3D<float> &outputTensor,
			float *C,
			bool bKeypoints = false,
			bool bWholeImage = false,
			bool bImportedKeypoints = false,
			int nImportedKeypoints = 0,
			Box<float> *pBoundingBox = NULL,
			float *boundingBoxR = NULL,
			float *boundingBoxt = NULL);
		void KeypointBasedSampling(
			Array2D<float> image,
			Array3D<float> &outputTensor,
			float* C,
			Box<float>* pBoundingBox = NULL,
			float* boundingBoxR = NULL,
			float* boundingBoxt = NULL);
		void RandomSampling(
			Array2D<float> image,
			Array3D<float> &outputTensor,
			float* C);
		bool BoxFeature(
			int iPix,
			Array2D<float> image,
			float *Z,
			float *C,
			float *RCB,
			float &r,
			bool bComputeBox = true,
			bool bImportCenter = false,
			Box<float> *pBoundingBox = NULL);
		void CreateParamList();
		void VisualizeBox(
			cv::Mat displayImage,
			float *C,
			int iBox,
			cv::Scalar color);
		void Visualize(
			Array2D<float> image,
			Array3D<float> featureTensor,
			float *C,
			bool bVisualizeBoxes = false,
			Array3D<float> *pFeatureTensor_s = NULL);

	public:
		CRVLMem *pMem0;
		CRVLMem* pMem;
		CRVLParameterList paramList;
		int nSamples;
		float maxz;
		float fu;
		float fv;
		float uc;
		float vc;
		int imageWidth;
		int imageHeight;
		float boxSize;
		int gridSize;
		float zOut;
		float *PArray;
		Array<int> iPixArray;
		CRVL2DCellArray cells;
		double SURFHessianThrH;
		double SURFHessianThrL;
		bool bRepeatFeatures;
		bool bVariableSize;
		float nieghborhoodSize;
		float distance;
		float boxSizeCoeff;

	private:
		float *ZInit;
	};
}

