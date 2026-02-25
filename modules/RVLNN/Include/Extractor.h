#pragma once

namespace RVL
{
	class Extractor
	{
	public:
		Extractor();
		virtual ~Extractor();
		void Create(char* cfgFileName);
		void CreateParamList();
		void Apply(
			Array2D<float> srcImage,
			float *RBC,
			float *tBC,
			float* gC,
			float *boxSize,
			Array2D<float> &tgtImage,
			Array2D<float> &tgtPC,
			float *RCO,
			float *tCO);
		void Visualize(Array2D<float> tgtImage);

	public:
		CRVLMem* pMem0;
		CRVLParameterList paramList;
		float fuSrc;
		float fvSrc;
		float ucSrc;
		float vcSrc;
		int imgWidth;
		int imgHeight;
		float fTgt;
		float zn;
		float zf;
		int tgtImgWidth;
		int tgtImgHeight;
		float zInvalid;
	};
}
