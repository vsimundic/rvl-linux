#pragma once

namespace RVL
{
	namespace RECOG
	{
		namespace VN_
		{
			struct Gauss
			{
				float *x;
				float abs;
				float var;
				float w;
			};

			struct ModelTemplate2
			{
				int idx;
				int f;
				int no;
				int nu;
				float cost;
				int label;
				int *componentIdx;
				QList<QLIST::Entry<Pair<int, int>>> correspondences;
				//Array2D<VN_::Gauss> gaussianMixtureModel;
				//float *gaussMem;
				Array2D<float> conditionalProbability;
				bool *bAssigned;
			};

			struct PartAssociation
			{
				int iSceneSegment;
				int label;
				int iMetaModel;
				int iComponent;
			};
		}
	}

	class VNClass
	{
	public:
		VNClass();
		virtual ~VNClass();

	public:
		void Save(void *vpClassifier, FILE *fp);
		void Load(void *vpClassifier, FILE *fp);

		int nLabels;
		std::vector<RECOG::VN_::ModelTemplate2> *metamodels;
		Array<RECOG::VN_::Gauss> gauss;
		float *gaussMem;
		Array<RECOG::VN_::PartAssociation> componentID;
		Array<Pair<int, int>> metamodelID;
		int *componentIdxMem;
		int maxnGaussPerComponent;
		float woTotal;
		float wuTotal;
		int maxnMetamodelComponents;
		//Array<int> *componentGauss;
		//int *componentGaussMem;
		Array<int> *metaModelGauss;
		int *metaModelGaussMem;
	};
}

