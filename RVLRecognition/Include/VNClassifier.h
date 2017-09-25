#pragma once

namespace RVL
{
	class VNClassifier
	{
	public:
		VNClassifier();
		virtual ~VNClassifier();
		void Create(char *cfgFileName);
		void Init(Mesh *pMesh);
		void Clear();
		void CreateParamList();
		void Classify(
			Mesh *pMesh,
			float *&dS,
			bool *&bdS,
			Box<float> &SBoundingBox,
			int iModel = -1);

	public:
		CRVLMem *pMem0;
		CRVLMem *pMem;
		CRVLParameterList paramList;
		SurfelGraph *pSurfels;
		PlanarSurfelDetector *pSurfelDetector;
		PSGM convexClustering;
		PSGM concaveClustering;
		std::vector<VN *> models;
		float kMaxMatchCost;
		float clusteringTolerance;
		int maxnSClusters;
	};
}

