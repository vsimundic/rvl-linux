#pragma once

#define RVLVN_CLUSTER_TYPE_CONVEX	0
#define RVLVN_CLUSTER_TYPE_CONCAVE	1
#define RVLVN_CLUSTER_TYPE_XTORUS	2
#define RVLVN_CLUSTER_TYPE_ITORUS	3

#define RVLVN_METAMODEL_TORUS	0
#define RVLVN_METAMODEL_BOTTLE	1
#define RVLVN_METAMODEL_HAMMER	2
#define RVLVN_METAMODEL_BOWL	3
#define RVLVN_METAMODEL_MUG		4

// Move to Util.h.

#define RVLRND(n, iRnd, nRnd, iiRnd, x)	{x = iRnd[iiRnd] % n; iiRnd = (iiRnd + 1) % nRnd;}

namespace RVL
{
	void RandomIndices(Array<int> &A);
}

// Move to RVLQListArray.h.

namespace RVL
{
	namespace QLIST
	{
		template<typename T>
		void CopyToArray(QList<Entry<T>> *pList, Array<T> *pArray)
		{
			Entry<T> *pEntry = pList->pFirst;

			T *pData = pArray->Element;

			while (pEntry)
			{
				*(pData++) = pEntry->data;

				pEntry = pEntry->pNext;
			}

			pArray->n = pData - pArray->Element;
		}
	}
}

//

#define RVLVN_GET_NEXT_QUEUE_ENTRY(queue, pNode, pNextNode, iTopQueueBin, iBottomQueueBin, bCompleted, pQueueBin)\
{\
	if(pNode->pNext)\
		pNextNode = pNode->pNext;\
	else\
	{\
		while (true)\
		{\
			if (iTopQueueBin >= iBottomQueueBin)\
			{\
				bCompleted = true;\
				break;\
			}\
			else\
			{\
				iTopQueueBin++;\
				if (queue[iTopQueueBin])\
				{\
					pQueueBin = queue[iTopQueueBin];\
					if (pQueueBin->pFirst)\
					{\
						pNextNode = pQueueBin->pFirst;\
						break;\
					}\
				}\
			}\
		}\
	}\
}

#define RVLVN_ADD_QUEUE_ENTRY(pNode, NodeType, e, queue, minMatchCostDiff, nMatchCostLevels, iTopQueueBin, iBottomQueueBin, pMem, iBin, pQueueBin)\
{\
	iBin = (int)(e / minMatchCostDiff);\
	if (iBin < nMatchCostLevels)\
	{\
		pQueueBin = queue[iBin];\
		if (pQueueBin == NULL)\
		{\
			RVLMEM_ALLOC_STRUCT(pMem, QList<NodeType>, pQueueBin);\
			queue[iBin] = pQueueBin;\
			RVLQLIST_INIT(pQueueBin);\
		}\
		RVLQLIST_ADD_ENTRY(pQueueBin, pNode);\
		if (iBin < iTopQueueBin)\
			iTopQueueBin = iBin;\
		else if (iBin > iBottomQueueBin)\
			iBottomQueueBin = iBin;\
	}\
}

namespace RVL
{
	// Move to RVL3DTools.h.

	template <typename T>
	void ExpandBox(Box<T> *pBox, T extension)
	{
		pBox->minx -= extension;
		pBox->maxx += extension;
		pBox->miny -= extension;
		pBox->maxy += extension;
		pBox->minz -= extension;
		pBox->maxz += extension;
	}

	class VN;

	namespace RECOG
	{
		namespace VN_
		{
			struct Voxel
			{
				QList<QLIST::Index> PtList;
				int voxelDistance;
			};

			struct Sample
			{
				float P[3];
				int iFeature;
				float SDF;
			};

			struct Feature
			{
				float N[3];
				float d;
				int iAlpha;
				int iBeta;
			};

			struct Node
			{
				int operation;
				int iFeature;
				float fOperation;
				float output;
				bool bOutput;
				int iActiveFeature;
				Feature *pFeature;
			};

			struct Operation
			{
				int ID;
				int iNode;
				int operation;
				int operand[2];
				Operation *pNext;
			};

			struct Limit
			{
				int sourceClusterID;
				int iAlpha;
				int iBeta;
				int targetClusterID;
				Limit *pNext;
			};

			struct Correspondence
			{
				int iVertex;
				float d;
				float dCluster;
				bool bMerged;
				bool bPrevMerged;
				int iParent;
				int iSceneFeature;
			};

			struct SceneFeature
			{
				QList<QLIST::Index> iVertexList;
				float d;
			};

			struct ITNode
			{
				int iNode;
				int iCorrespondence;
				ITNode *pParent;
				int iLevel;
				//float *e;
				float e;
				bool bExpanded;
				ITNode *pNext;
			};

			struct ModelCluster
			{
				int ID;
				int iNode;
				BYTE type;
				float R[9];
				float t[3];
				float r;
				float rT;
				Array<float> alphaArray;
				Array<float> betaArray;
				ModelCluster *pNext;
				Pair<int, int> iFeatureInterval;
			};

			struct SceneCluster
			{
				BYTE type;
				void *vpCluster;
			};

			struct Cluster
			{
				int iParent;
				std::vector<int> iChild;
			};

			struct TorusRing
			{
				int idx;
				int iBeta;
				//float beta;
				Array<int> iVertexArray;
				Array<int> iEdgeArray;
				float *d;
				int *iKeyVertex;
				QList<QLIST::Ptr<TorusRing>> compList;
				bool bLast;
				TorusRing *pNext;
			};

			struct Torus
			{
				Array<TorusRing *> ringArray;
				Torus *pNext;
			};

			struct Queue
			{
				QList<ITNode> **queue;
				float maxMatchCost;
				float minMatchCostDiff;
				int nMatchCostLevels;
				int iTopQueueBin;
				int iBottomQueueBin;
				CRVLMem *pMem;
			};

			struct Edge
			{
				Pair < int, int > data;
				bool bPrimary;
				Edge *pNext;
			};

			struct ITPtr
			{
				ITNode *pLITNode;
				int iQueueBin;
			};

			struct GITNode
			{
				ITPtr *ITPtr_;
				GITNode *pParent;
				float e;
				GITNode *pNext;
			};

			struct Parameters
			{
				float kMaxMatchCost;
				float clusteringTolerance;
				int maxnSClusters;
			};

			struct FeatureNodeData
			{
				int iNode;
				float d;
			};

			struct Correspondence2
			{
				int cost;
				int *iFNode;
				Array<int> iFeatureArray;
			};

			struct Correspondence3
			{
				int cost;
				int iMCluster;
				int iSCluster;
				unsigned long long int *mFNodes;
				Correspondence3 *pParent;
				Correspondence3 *pNext;
			};

			struct Correspondence4
			{
				int cost;
				int iMCluster;
				int iSCluster;
				int iLevel;
				Correspondence4 *pParent;
				Correspondence4 *pNext;
			};

			struct EdgeTangent
			{
				float N[3];
				float d;
				EdgeTangent *pNext;
			};

			struct EdgeTangentSet
			{
				int miniBeta;
				int maxiBeta;
				Array<EdgeTangent *> tangentArray;
				float edgeLength;
			};

			struct TorusTreeNode
			{
				TorusRing *pRing;
				TorusTreeNode *pParent;
				TorusTreeNode *pNext;
			};

			void CreateTorus(
				VN *pVN,
				CRVLMem *pMem);
			void CreateBottle(
				VN *pVN,
				CRVLMem *pMem);
			void CreateHammer(
				VN *pVN,
				CRVLMem *pMem);
			void CreateBowl(
				VN *pVN,
				CRVLMem *pMem);
			void CreateMug(
				VN *pVN,
				CRVLMem *pMem);
		}	// namespace VN_
	}	// namespace RECOG

	void SampleMeshDistanceFunction(
		Mesh *pMesh,
		SurfelGraph *pSurfels,
		float voxelSize,
		int sampleVoxelDistance,
		Array3D<RECOG::VN_::Voxel> &volume,
		float *P0,
		Array<RECOG::VN_::Sample> &sampleArray,
		Box<float> &boundingBox);

	void DisplaySampledMesh(
		Visualizer *pVisualizer,
		Array3D<RECOG::VN_::Voxel> volume,
		float *P0,
		float voxelSize);

	class VN
	{
	public:
		VN();
		virtual ~VN();
		void CreateParamList(
			CRVLParameterList *pParamList, 
			RECOG::VN_::Parameters &params,
			CRVLMem *pMem);
		void CreateEmpty();
		void Create(CRVLMem *pMem);
		void AddModelCluster(
			int ID,
			BYTE type,
			float *R,
			float *t,
			float r,			
			Array<float> alphaArray,
			Array<float> betaArray,
			CRVLMem *pMem,
			float rT);
		void AddModelCluster(
			int ID,
			BYTE type,
			float *R,
			float *t,
			float r,
			int nAlphasPer2PI,
			int nBetasPerPI,
			Pair<int, int> iBetaInterval,
			CRVLMem *pMem,
			float rT = 0.0f,
			Pair<int, int> iAlphaInterval = {0, 0});
		void AddOperation(
			int ID,
			int operation,
			int operand1,
			int operand2,
			CRVLMem *pMem);
		void AddLimit(
			int sourceClusterID,
			int iAlpha,
			int iBeta,
			int targetClusterID,
			CRVLMem *pMem);
		void SetOutput(int outputID);
		void Create(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			CRVLMem *pMem,
			float voxelSize = 5.0f,
			int sampleVoxelDistance = 2,
			float eps = 2.0f,
			Visualizer *pVisualizer = NULL);
		float Evaluate(
			float *P,
			float *SDF,
			int &iActiveFeature,
			bool bComputeSDFs = true,
			float *d = NULL,
			bool *bd = NULL);
		float Evaluate(
			Array<float *>PArray,
			Array<Array<RECOG::VN_::SceneFeature>> correspondenceArray,
			int *solution,
			float *SDF,
			float *d,
			bool *bd = NULL,
			float maxe = 0.0f);
		float Evaluate(
			Mesh *pMesh,
			Array<int> iPtArray,
			float *SDF,
			float *d = NULL,
			bool *bd = NULL,
			float maxe = 0.0f);
		float Evaluate(
			float *PArray,
			int nP,
			float *SDF,
			float *d = NULL,
			bool *bd = NULL,
			float maxe = 0.0f);
		void ComputeFeatureSDFs(
			float *P,
			float *SDF,
			float *d = NULL);
		void Match(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			Box<float> boundingBox,
			RECOG::VN_::Parameters params,
			float *dS,
			bool *bdS);
		void MatchByBuildingInterpretationTree(
			SurfelGraph *pSurfels,
			Array<Array<RECOG::VN_::SceneFeature>> correspondenceArray,
			float maxMatchCost,
			float minMatchCostDiff,
			int maxnGITNodes,
			float *dS,
			bool *bdS,
			CRVLMem *pMem2);
		void GeneticAlg(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			Array<Array<RECOG::VN_::SceneFeature>> correspondenceArray,
			float *dS,
			bool *bdS);
		void Fit(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			float *dS,
			bool *bdS);
		void ModelClusters();
		void InitMatchCluster(RECOG::VN_::Queue &Q);
		bool MatchCluster(
			RECOG::VN_::Cluster *pCluster,
			Array<Array<RECOG::VN_::SceneFeature>> correspondenceArray,
			SurfelGraph *pSurfels,
			RECOG::VN_::ITNode *pITNodeIn,
			RECOG::VN_::Queue &Q,
			RECOG::VN_::ITNode *&pITNode);
		void Descriptor(
			RECOG::VN_::ITPtr *ITPtr_,
			Array<Array<RECOG::VN_::SceneFeature>> correspondenceArray,
			float *dS,
			bool *bdS);
		float Distance(
			SurfelGraph *pSurfels,
			Array<int> iVertexArray,
			float *dS,
			bool *bdS,
			int &iMaxErrVertex,
			float *SDF);
		float GetMeshSize(Box<float> boundingBox);
		void Match2(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			Box<float> boundingBox,
			RECOG::VN_::Parameters params,
			float *dS,
			bool *bdS);
		void Match3(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			Array<RECOG::PSGM_::Cluster *> SClusters,
			Box<float> boundingBox,
			RECOG::VN_::Parameters params,
			float *dS,
			bool *bdS);
		//void Match4(
		//	Mesh *pMesh,
		//	SurfelGraph *pSurfels,
		//	Array<RECOG::PSGM_::Cluster *> SCClusters,
		//	Array<RECOG::PSGM_::Cluster *> SUClusters,
		//	Box<float> boundingBox,
		//	RECOG::VN_::Parameters params,
		//	CRVLMem *pMem,
		//	float *dS,
		//	bool *bdS);
		void Match4(
			Mesh *pMesh,
			float *PArray,
			float *NArray,
			float *R,
			float *t,
			void *vpClassifier,
			Box<float> boundingBox,
			float *dS,
			bool *bdS);
		void ToroidalClusters(
			Mesh *pMesh,
			float *PArray,
			float *NArray,
			SurfelGraph *pSurfels,
			float *axis,
			Array<float> alphaArray,
			Array<float> betaArray,
			float maxErr,
			Array<RECOG::VN_::Torus *> &SClusters,
			CRVLMem *pMem);
		void DetectTorusRings(
			Mesh *pMesh,
			float *PArray,
			SurfelGraph *pSurfels,
			SURFEL::VertexEdge *pVEdge0,
			float *axis,
			int iBeta,
			RECOG::VN_::EdgeTangentSet *tangentSet,
			float maxError,
			QList<RECOG::VN_::TorusRing> *pRingList,
			CRVLMem *pMem,
			SURFEL::VertexEdge **VEdgeBuff,
			bool *bEdgeJoined,
			bool *bVertexJoined);
		RECOG::VN_::ModelCluster *GetModelCluster(int ID);
		RECOG::VN_::Operation *GetOperation(int ID);
		void Load(
			char *fileName,
			CRVLMem *pMem);
		void Display(
			Visualizer *pVisualizer,
			Box<float> box,
			float resolution,
			float *d = NULL,
			bool *bd = NULL,
			float SDFSurfaceValue = 0.0f);
		void PrintTorus(
			FILE *fp,
			SurfelGraph *pSurfels,
			RECOG::VN_::Torus *pTorus,
			int iTorus);
		void PrintTori(
			FILE *fp,
			SurfelGraph *pSurfels,
			Array<RECOG::VN_::Torus *> torusArray);

	public:
		Array<RECOG::VN_::Node> NodeArray;
		QList<RECOG::VN_::Edge> EdgeList;
		Array<RECOG::VN_::Feature> featureArray;
		QList<RECOG::VN_::ModelCluster> modelClusterList;
		QList<RECOG::VN_::Operation> operationList;
		QList<RECOG::VN_::Limit> limitList;
		int outputID;
		Array3D<RECOG::VN_::Voxel> volume;
		float voxelSize;
		float P0[3];
		SurfelGraph *pFeatures;
		int iy;
		Box<float> boundingBox;	
		std::vector<RECOG::VN_::Cluster> clusters;
	};
}

