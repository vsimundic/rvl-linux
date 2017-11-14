#pragma once

namespace RVL
{
	class VNClassifier;

	namespace RECOG
	{
		namespace VN_
		{
			struct VisualizationData
			{
				float resolution;
				float SDFSurfaceValue;
			};

			void _3DNetDatabaseClasses(VNClassifier *pClassifier);
		}

		struct ClassData
		{
			int iMetaModel;
			int iRefInstance;
			int iFirstInstance;
			int nInstances;
		};
	}

	class VNClassifier
	{
	public:
		VNClassifier();
		virtual ~VNClassifier();
		void Create(char *cfgFileName);
		void Init(Mesh *pMesh);
		void Clear();
		void CreateParamList();
		void ComputeDescriptor(
			Mesh *pMesh,
			float *RIn,
			float *tIn,
			float *&dS,
			bool *&bdS,
			Box<float> &SBoundingBox,
			int iModel = -1);
		void Learn(
			char *modelSequenceFileName,
			int iClass = -1,
			Visualizer *pVisualizer = NULL); //Vidovic
		void Interpret(
			Mesh *pMesh,
			int iClass);
		void SaveDescriptor(
			FILE *fp,
			float *d,
			bool *bd,
			int iModel,
			int iMetaModel);
		void LoadDescriptor(
			FILE *fp,
			float *&d,
			bool *&bd,
			int &iModel,
			int &iMetaModel);

	public:
		CRVLMem *pMem0;
		CRVLMem *pMem;
		CRVLParameterList paramList;
		DWORD mode;
		SurfelGraph *pSurfels;
		SURFEL::ObjectGraph *pObjects;
		PlanarSurfelDetector *pSurfelDetector;
		PSGM alignment;
		PSGM convexClustering;
		PSGM concaveClustering;
		std::vector<VN *> models;
		float kMaxMatchCost;
		float clusteringTolerance;
		int maxnSCClusters;
		int maxnSUClusters;
		int maxnSTClusters;
		float voxelSize;
		int sampleVoxelDistance;
		float connectedComponentMaxDist;
		int connectedComponentMinSize;
		bool bLoadCTIDataBase;
		bool bVisualization;
		char *modelDataBase; //Vidovic
		char *modelsInDataBase; //Vidovic
		void *vpMeshBuilder;
		bool(*LoadMesh)(void *vpMeshBuilder,
			char *FileName,
			Mesh *pMesh,
			bool bSavePLY);
		RECOG::VN_::VisualizationData visualizationData;
		Array<RECOG::ClassData> classArray;
		RECOG::VN_::SceneObject sceneObject;
		float NGnd[3];
		float dGnd;
		bool bGnd;
		RECOG::VN_::Instance refModel;
	};
}

