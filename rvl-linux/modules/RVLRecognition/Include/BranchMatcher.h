#pragma once

namespace RVL
{
	// These structures and macros related to ArrayD should be moved to RVLArray.h

	template <typename Type>
	struct ArrayD
	{
		int iFirst;
		int n;
		std::vector<Type> *pStorage;
	};

#define RVLARRAYD_INIT(array_, storage) \
	{                                   \
		array_.pStorage = &storage;     \
		array_.iFirst = storage.size(); \
		array_.n = 0;                   \
	}
#define RVLARRAYD_ADD(array_, data)       \
	{                                     \
		array_.pStorage->push_back(data); \
		array_.n++;                       \
	}
#define RVLARRAYD_ELEMENT(array_, idx) array_.pStorage->at(array_.iFirst + (idx))

	namespace RECOG
	{
		namespace BM
		{
			struct Particle
			{
				bool bExists;
				float c[3];
				float r;
				int iPix;
				int iPredecessor;
				Array<int> successors;
			};

			struct Assoc
			{
				int p;
				int q;
				int iPredecessor;
				int iBranch;
				int DDFirstIdx;
				int MFirstIdx;
				float C;
				float L;
				int N;
				bool bExpanded;
			};

			struct DisplayCallbackData
			{
				Visualizer *pVisualizer;
				bool bOwnVisualizer;
				CRVLParameterList paramList;
				Mesh *pMesh;
				std::vector<vtkSmartPointer<vtkActor>> assocActors;
				float shiftQ[3];
				Array<RECOG::BM::Particle> particles;
			};

			void MouseRButtonDown(vtkIdType closestPointId, double *closestPoint, void *callData);
		}
	}

	class BranchMatcher
	{
	public:
		BranchMatcher();
		virtual ~BranchMatcher();
		void Create(char *cfgFileName);
		void Clear();
		void CreateParamList();
		void Particles(
			Array2D<short int> depthImage,
			Array<RECOG::BM::Particle> &particles,
			bool bVisualize = false);
		void Match(
			Array<RECOG::BM::Particle> particles_P,
			Array<RECOG::BM::Particle> particles_Q,
			int *c);
		void OptimalAssociations(
			Array<RECOG::BM::Particle> P,
			std::vector<RECOG::BM::Assoc> &A,
			int *c,
			Array<int> *pOptimalAssociations = NULL);
		void TestTree(
			Array<RECOG::BM::Particle> &particles_P,
			Array<RECOG::BM::Particle> &particles_Q);
		void InitVisualizer(Visualizer *pVisualizer = NULL);
		void Visualize(
			Array<RECOG::BM::Particle> particles_P,
			Array<RECOG::BM::Particle> particles_Q);
		void Visualize(
			Array<RECOG::BM::Particle> P,
			Array<RECOG::BM::Particle> Q,
			std::vector<RECOG::BM::Assoc> &A);

	public:
		CRVLMem *pMem0;
		CRVLParameterList paramList;
		Camera camera;
		RECOG::BM::DisplayCallbackData *pVisualizationData;
		std::string cfgFileName;
		float kappa;
		float lambda;
		float maxParticleDist;
		float tau_dst_rel;
		float tau_dst_abs;
		float tau_ang_deg;
		float tau_r_rel;
		float tau_r_abs;
		float maxDist;
		float maxRotDeg;
		bool bVisualize;
		char *resultsFolder;

	private:
		std::vector<int> idxMem;
		std::vector<ArrayD<int>> arrayPtrMem;
	};
}
