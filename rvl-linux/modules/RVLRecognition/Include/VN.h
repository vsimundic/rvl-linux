#pragma once

#define RVLVN_GET_NEXT_QUEUE_ENTRY(queue, pNode, pNextNode, iTopQueueBin, iBottomQueueBin, bCompleted, pQueueBin) \
	{                                                                                                             \
		if (pNode->pNext)                                                                                         \
			pNextNode = pNode->pNext;                                                                             \
		else                                                                                                      \
		{                                                                                                         \
			while (true)                                                                                          \
			{                                                                                                     \
				if (iTopQueueBin >= iBottomQueueBin)                                                              \
				{                                                                                                 \
					bCompleted = true;                                                                            \
					break;                                                                                        \
				}                                                                                                 \
				else                                                                                              \
				{                                                                                                 \
					iTopQueueBin++;                                                                               \
					if (queue[iTopQueueBin])                                                                      \
					{                                                                                             \
						pQueueBin = queue[iTopQueueBin];                                                          \
						if (pQueueBin->pFirst)                                                                    \
						{                                                                                         \
							pNextNode = pQueueBin->pFirst;                                                        \
							break;                                                                                \
						}                                                                                         \
					}                                                                                             \
				}                                                                                                 \
			}                                                                                                     \
		}                                                                                                         \
	}

#define RVLVN_ADD_QUEUE_ENTRY(pNode, NodeType, e, queue, minMatchCostDiff, nMatchCostLevels, iTopQueueBin, iBottomQueueBin, pMem, iBin, pQueueBin) \
	{                                                                                                                                              \
		iBin = (int)(e / minMatchCostDiff);                                                                                                        \
		if (iBin < nMatchCostLevels)                                                                                                               \
		{                                                                                                                                          \
			pQueueBin = queue[iBin];                                                                                                               \
			if (pQueueBin == NULL)                                                                                                                 \
			{                                                                                                                                      \
				RVLMEM_ALLOC_STRUCT(pMem, QList<NodeType>, pQueueBin);                                                                             \
				queue[iBin] = pQueueBin;                                                                                                           \
				RVLQLIST_INIT(pQueueBin);                                                                                                          \
			}                                                                                                                                      \
			RVLQLIST_ADD_ENTRY(pQueueBin, pNode);                                                                                                  \
			if (iBin < iTopQueueBin)                                                                                                               \
				iTopQueueBin = iBin;                                                                                                               \
			else if (iBin > iBottomQueueBin)                                                                                                       \
				iBottomQueueBin = iBin;                                                                                                            \
		}                                                                                                                                          \
	}

#define RVLVN_HYPOTHESIS2_FLAG_LEVEL1 0x01

namespace RVL
{
	class VN;

	namespace RECOG
	{
		namespace VN_
		{
			struct Feature
			{
				float N[3];
				float d;
				int iAlpha;
				int iBeta;
				int iN;
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
				int iN;
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
				// float *e;
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
				Array2D<float> NArray;
				int *iN;
				ModelCluster *pNext;
				Pair<int, int> iFeatureInterval;
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
				// float beta;
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
				Pair<int, int> data;
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

			struct Correspondence5
			{
				int iSPoint;
				int iMCluster;
				int iBeta;
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

			struct SceneObject
			{
				float *vertexArray;
				float *NArray;
				Array<MESH::Sample> sampleArray;
				float R[9];
				float t[3];
			};

			struct Instance
			{
				float *d;
				bool *bd;
			};

			struct ParallelDescriptorComponent
			{
				float N[3];
				Array<int> idxArray;
			};

			struct FitParams
			{
				float alpha;
				float beta;
				float lambda0;
				float lambda1;
				int regularization;
				float maxe;
				float kSceneSupport;
				float kOutliers;
				float kHull;
				float kZMin;
				float cosSurfaceRayAngleThr;
				int maxnIterations;
				int nFitRotLSIterations;
				bool bInit;
				bool bGnd;
				bool bGnd2;
				FILE *fpScore;
			};

			struct FitData
			{
				Array2D<float> M;
				cv::Mat A;
				cv::Mat b;
				cv::Mat q;
				float *A_;
				float *b_;
				float *q_;
				Array<int> iV;
				float *a;
				float *dSV;
				float lambda0;
				float lambda1;
				float sqrtbeta;
				float Pc[3];
				float *P;
				// float *d;
				// uchar *bd;
			};

			struct SurfaceRayIntersection
			{
				float s;
				int iFeature;
			};

			struct SuperSegment
			{
				int ID;
				Array<int> segments;
				Array<int> iSurfelArray;
				Array<int> iVertexArray;
				Pose3D pose;
				float type;
				uchar flags;
				SuperSegment *pNext;
			};

			struct Hypothesis
			{
				VN *pVN;
				Array<int> iSegmentArray;
				Array<int> iSurfelArray;
				Array<int> iVertexArray;
				SuperSegment *pSuperSegment;
				int support;
				Array2D<float> modelVertices;
				int iClass;
				float *q;
				float *d;
				uchar *bd;
				float R[9];
				float P[3];
				float shapeCost;
				float convexityError;
				int nOutliers;
				float cost;
				float score;
				int estStage;			  // The hypothesis estimation is complete
				std::vector<int> objects; // list of objects
				int iMatch;
				int iCell;
				Hypothesis *pNext;
				Hypothesis **pPtrToThis;
			};

			struct Hypothesis2
			{
				int idx;
				SuperSegment *pSuperSegment;
				int iModel;
				float R[9];
				float P[3];
				float R_CHAL[9]; // Only for debuging purpose
				float P_CHAL[3]; // Only for debuging purpose
				float score;
				float CTIcost; // Only for debuging purpose
				int iMatch;
				int iMCTI; // Only for debugging purpose!
				// float tLMLS[3];		// Only for debugging purpose!
				int rank;
				int iCell;
				float EBB; // Only for debugging purpose!
				float gndPlaneDistance;
				float CHMatchingMetric;
				float matchedPtsPercentage;
				uchar flags;
				Hypothesis2 *pNext;
				Hypothesis2 **pPtrToThis;
				Array<int> segments;
			};

			struct PrimitiveLayerOutput
			{
				int mTotal;
				float *y;
				float *q;
				float *R;
				float *JR;
			};

			struct TorusDetectionDisplayData
			{
				Visualizer *pVisualizer;
				int nProps;
			};

			void CreateConvex(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateTorus(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateBanana(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateBottle(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateHammer(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateBowl(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateMug(
				VN *pVN,
				CRVLMem *pMem);
			void CreateMug2(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateMug3(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreatePipe(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateCar(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateApple(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateBowl2(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateTwoConvex(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateMug4(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			void CreateMug5(
				VN *pVN,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem);
			float SceneFittingScore(
				SURFEL::SceneSamples sceneSamples,
				Array<OrientedPoint> PtArray,
				float maxe,
				float maxz,
				float cThr,
				int &nOutliers,
				// float transparencyDepthThr,
				bool bVisualize = false);
			float SceneFittingScore2(
				SURFEL::SceneSamples sceneSamples,
				Array<OrientedPoint> PtArray,
				float maxe,
				float maxz,
				float cThr,
				int &nSamples,
				float &chamferDist,
				int &nOutliers,
				// float transparencyDepthThr,
				bool bVisualize = false);
			void Init(
				RECOG::VN_::FitData &fitData,
				Array2D<float> M,
				float beta,
				float lambda0,
				float lambda1,
				bool bRot,
				int nPts);
			void Delete(RECOG::VN_::FitData &fitData);
			void Create(
				VN *pMetaModel,
				int *valid,
				Array<RECOG::PSGM_::Plane> convexTemplate,
				CRVLMem *pMem,
				VN *pVN);
			bool TorusDetectionKeyPressCallback(
				// vtkObject* caller, unsigned long eid, void* clientdata, void *calldata)
				Mesh *pMesh,
				SurfelGraph *pSurfels,
				std::string &keySym,
				void *vpData);
			void TorusVisualization(
				Mesh *pMesh,
				SurfelGraph *pSurfels,
				PlanarSurfelDetector *pSurfelDetector,
				QList<RECOG::VN_::Torus> torusList,
				std::vector<RVL::Point> MidPointCH);
		} // namespace VN_
	}	  // namespace RECOG

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
		RECOG::VN_::ModelCluster *AddModelCluster(
			int ID,
			BYTE type,
			float *R,
			float *t,
			float r,
			Array<float> alphaArray,
			Array<float> betaArray,
			Array2D<float> NArray,
			CRVLMem *pMem,
			float rT,
			int *iN = NULL);
		RECOG::VN_::ModelCluster *AddModelCluster(
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
		RECOG::VN_::ModelCluster *AddModelCluster(
			int ID,
			BYTE type,
			float *R,
			float *t,
			float r,
			Array<RECOG::PSGM_::Plane> convexTemplate,
			Pair<float, float> betaInterval,
			Array2D<float> NArrayIn,
			CRVLMem *pMem);
		RECOG::VN_::ModelCluster *AddModelCluster(
			int ID,
			BYTE type,
			float *R,
			float *t,
			float r,
			Array<RECOG::PSGM_::Plane> convexTemplate,
			float *axis,
			Pair<float, float> betaInterval,
			Array2D<float> NArrayIn,
			CRVLMem *pMem);
		RECOG::VN_::ModelCluster *AddModelCluster(
			RECOG::VN_::ModelCluster *pMCluster,
			int *valid,
			float *R,
			float *t,
			CRVLMem *pMem);
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
			int iN,
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
		void UpdateClusterOrientations();
		void SetFeatureOffsets(float *d);
		void GetEdges(Array<RECOG::VN_::Edge> &edges);
		void Descriptor(float *d);
		void Descriptor(
			Array<Vector3<float>> points,
			Array<RECOG::VN_::Correspondence5> assoc,
			float *d);
		float Evaluate(
			float *P,
			float *SDF,
			int &iActiveFeature,
			bool bComputeSDFs = true,
			float *d = NULL,
			bool *bd = NULL);
		float Evaluate(
			Array<float *> PArray,
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
		float Evaluate(
			Array<MESH::Sample> sampleArray,
			float *SDF,
			float *d = NULL,
			bool *bd = NULL,
			float maxe = 0.0f);
		float LocalConstraints(
			float *P,
			float *SDF,
			int &iActiveFeature,
			bool bComputeSDFs = true,
			float *d = NULL);
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
			QList<QLIST::Index> surfelList,
			Array<int> iVertexArray,
			Camera camera,
			Array2D<float> M,
			float *R,
			float *d0,
			float *q,
			bool bVisualize = false);
		void FitLM(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			QList<QLIST::Index> surfelList,
			Array<int> iVertexArray,
			Camera camera,
			Array2D<float> M,
			float *R,
			float *d0,
			float *q,
			bool bVisualize = false);
		float FitRotLM(
			void *vpClassifier,
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			Array<int> iVertexArray,
			Camera camera,
			SURFEL::SceneSamples sceneSamples,
			RECOG::ClassData *pClass,
			float *d0,
			RECOG::VN_::FitParams params,
			float *qOpt,
			float *ROpt,
			bool bVisualize = false,
			bool bVisualizeBestInstance = false);
		float FitRotLM(
			SurfelGraph *pSurfels,
			Array<int> iVertexArray,
			SURFEL::SceneSamples sceneSamples,
			RECOG::ClassData *pClass,
			float *d0,
			float *qs0,
			float *RIn,
			RECOG::VN_::FitParams params,
			float *q,
			float *R,
			float &EHull,
			bool bVisualize = false);
		float FitRotLMCC(
			SurfelGraph *pSurfels,
			Array<int> iVertexArray,
			SURFEL::SceneSamples sceneSamples,
			RECOG::ClassData *pClass,
			float *d0,
			float *qs0,
			float *RIn,
			RECOG::VN_::FitParams params,
			float *q,
			float *R,
			float &EHull,
			bool bVisualize = false,
			Camera *pCamera = NULL);
		float FitConvexHull(
			SurfelGraph *pSurfels,
			Array<int> iVertexArray,
			float *Pc,
			float *PGnd,
			Array2D<float> M,
			float *R,
			float *q,
			float *qs0 = NULL);
		float FitConvexHullIteration(
			SurfelGraph *pSurfels,
			Array<int> iVertexArray,
			float *Pc,
			float *PGnd,
			float *d,
			Array2D<float> M,
			int nq,
			bool bRot,
			float alpha,
			float *R,
			float *A = NULL,
			float *b = NULL,
			float *J = NULL);
		void ModelClusters();
		int ClusterTypes(
			bool &bConcave,
			bool &bTorus);
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
		// void Match4(
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
			RECOG::VN_::SceneObject sceneObject,
			void *vpClassifier,
			Array<RECOG::SceneCluster> &SClusters_,
			Box<float> boundingBox,
			float *dS,
			bool *bdS,
			bool bValidateDescriptorComponents = false);
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
		void ToroidalClusters2(
			Mesh *pMesh,
			float *PArray,
			float *NArray,
			SurfelGraph *pSurfels,
			PlanarSurfelDetector *pSurfelDetector,
			float *axis,
			Array<float> alphaArray,
			Array<float> betaArray,
			float maxErr,
			QList<RECOG::VN_::Torus> *pTorusList,
			// Array<RECOG::VN_::Torus *> &SClusters,
			CRVLMem *pMem,
			bool *vertexCH,
			int &nToruses,
			int iSuperSegment);
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
			bool *bVertexJoined,
			bool *vertexCH = NULL);
		RECOG::VN_::ModelCluster *GetModelCluster(int ID);
		RECOG::VN_::Operation *GetOperation(int ID);
		void Project(
			float *d,
			float *R,
			float *t,
			Camera camera,
			Array2D<float> imgPtArray,
			Array<OrientedPoint> PtArray,
			float *Z = NULL,
			float *pMinZ = NULL);
		RECOG::VN_::SurfaceRayIntersection Project(
			float *d,
			float *r,
			RECOG::VN_::SurfaceRayIntersection *pSurfaceRayIntersectionEnd = NULL,
			bool *bd = NULL);
		Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>> *VolumeCylinderIntersection(
			float *d,
			float *P1,
			float *P2,
			float r);
		void Transform(
			float *dSrc,
			float *R,
			float *t,
			float *dTgt);
		void BoundingBox(
			float *d,
			Box<float> &box);
		void CreateFromConvexMesh(
			Mesh *pMesh,
			float *&d,
			CRVLMem *pMem);
		float SceneFittingScore(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			Camera camera,
			Array<int> iSurfelArray,
			Array<int> iVertexArray,
			float *d,
			float maxe,
			float maxz,
			float cThr,
			int &nSamples,
			float &chamferDist,
			int &nOutliers,
			bool bVisualize = false);
		void Load(
			char *fileName,
			CRVLMem *pMem);
		vtkSmartPointer<vtkActor> Display(
			Visualizer *pVisualizer,
			float kResolution = 0.01f,
			float *d = NULL,
			bool *bd = NULL,
			float SDFSurfaceValue = 0.0f,
			Box<float> *pBBoxIn = NULL);
		void Display(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			Array<int> iSurfelArray,
			Array<int> iVertexArray,
			float *d,
			float *R,
			uchar *color,
			Camera camera);
		void PrintTorus(
			FILE *fp,
			SurfelGraph *pSurfels,
			RECOG::VN_::Torus *pTorus,
			int iTorus);
		void PrintTori(
			FILE *fp,
			SurfelGraph *pSurfels,
			Array<RECOG::VN_::Torus *> torusArray);
		void SaveFeatures(FILE *fp);

	public:
		Array<RECOG::VN_::Node> NodeArray;
		QList<RECOG::VN_::Edge> EdgeList;
		Array<RECOG::VN_::Feature> featureArray;
		QList<RECOG::VN_::ModelCluster> modelClusterList;
		QList<RECOG::VN_::Operation> operationList;
		QList<RECOG::VN_::Limit> limitList;
		Array<RECOG::VN_::ParallelDescriptorComponent> parallelDescriptorComponentArray;
		int *parallelDescriptorComponentIdxMem;
		int outputID;
		Array3D<Voxel> volume;
		float voxelSize;
		float P0[3];
		SurfelGraph *pFeatures;
		int iy;
		Box<float> boundingBox;
		std::vector<RECOG::VN_::Cluster> clusters;
		FILE *fpDebug;
		VN *pNext;
		float score;
		Array<int> *localConstraints;

	private:
		Array<Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>> projectionIntervals;
		Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection> *projectionIntervalMem;
		Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>> projectionIntervalBuff;
		float *dc;
		int *localConstraintsMem;
	};
} // namespace RVL
