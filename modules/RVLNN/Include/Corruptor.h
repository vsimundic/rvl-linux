#pragma once

namespace RVL
{
	void GaussianFilter2D(
		Array2D<float> inImage,
		float sigma,
		Array2D<float> outImage);

	class Corruptor
	{
	public:
		Corruptor();
		virtual ~Corruptor();
		void Create(char* cfgFileName);
		void CreateParamList();
		void Apply(
			Array2D<float> srcImage,
			Array2D<float> &HPM,
			Array2D<float>* pCorruptedImage = NULL,
			bool bComputeHPM = true);
		void Visualize(Array2D<float> HPM);

	public:
		CRVLMem* pMem0;
		CRVLParameterList paramList;
		float fu;
		float fv;
		float uc;
		float vc;
		float minz;
		float kappa;
		float holeSize;
		float holeProbSig;
		float holeProbMax;
		float holeThr;
		float zInvalid;
		float noiseStd;
		float uvNoiseStd;
		float uvNoiseSmooth;
		float maxz;
		float stereoBaseLength;
	};
}

