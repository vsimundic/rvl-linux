#pragma once

#define RVLVN_TRED_TYPE_CONTAINS	0x01
#define RVLVN_TRED_TYPE_TOUCH		0x02
#define RVLVN_TRED_TYPE_12			0x04
#define RVLVN_TRED_TYPE_OPPOSITE_OU	0x08
#define RVLVN_CTINET_COMPONENT_LAYER	0x01
#define RVLVN_CTINET_NEIGHBORHOOD_LAYER	0x02
#define RVLVN_CTINET_MODEL_LAYER		0x04
#define RVLVN_CTINET_GLOBAL_LOCAL_LAYER	0x08
#define RVLVN_CTINET_COMPUTE_PDFPRIOR	0x10
#define RVLVN_CTINET_LABELS				0x20

//#define RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
#define RVLVN_PART_SEGMENTATION_CTINET_PCFIT

namespace RVL
{
	namespace RECOG
	{
		namespace VN_
		{
			struct ComponentCluster
			{
				int label;
				int nComponents;
				bool bConcave;
			};

			struct ModelTemplate
			{
				int f;
				Array<int> iComponentCluster;
				float p;
				bool *bComponentCluster;
			};

			struct ModelTemplateInstance
			{
				Array<int> iComponent[2];
				int iModel2;
				ModelTemplateInstance *pNext;
			};

			struct CTINetOutput
			{
				float *fComp;
				float *fShape;
				float *fSize;
				float *fNeighborhood;
				float *fModel;
			};

			struct ProbabilisticAssociationWorkData
			{
				Array<int> *activeMetaModels;
				int *activeMetaModelIdxMem;
				bool bInitialized;
			};

			struct AssociationProbabilityWorkData
			{
				int *nActiveComps;
				Array<int> *sceneSegments;
				int *sceneSegmentIdxMem;
				int *metamodel;
				bool bInitialized;
			};

			struct VNInstanceWorkData
			{
				Array<int> newAssignedCells;
			};

			struct TRED
			{
				float r12;
				float r21;
				float V12[3];
				float V21[3];
				float varV12;
				float varV21;
			};

			struct CAGEdge
			{
				int iVertex[2];
				GRAPH::EdgePtr<CAGEdge> *pVertexEdgePtr[2];
				int idx;
				float cost;
				float fComp;
				float fNeighborhood;
				float fModel;
				GRAPH::Edge *pNext;
			};
		}
	}

	class VNInstance
	{
	public:
		VNInstance();
		virtual ~VNInstance();
		void UpdateCorrespondenceMatrix(
			Array<VNInstance *> models,
			int *Correspondences,
			int iModel1,
			int iModel2,
			Array<int> *correspMatrix);
		void Match(RVL::QList<VNInstance> *pModelVNList,
			int *&Correspondences,
			void *vpClassifier,
			float *E,
			Array<int> *correspMatrix = NULL,
			int iModel1 = 0,
			int nModels = 0,
			bool bReturnAllCorrespondences = false); //Matches scene with model instances in QLIST, (or last added model with previous models)
		void Match(
			Array<VNInstance *> models,
			void *vpClassifier,
			int *&Correspondences,
			int outMxSize,
			float *E = NULL,
			Array<int> *correspMatrix = NULL,
			int iModel1 = 0,
			bool bMatchAllComponents = false);
		void Match(
			void *vpClassifier,
			int iModel,
			RECOG::VN_::CTINetOutput out);
		void Match2Instances(VNInstance *pVNModel,
			int *Correspondences,
			void *vpClassifier,
			float &cost,
			int *correspondencesOrder,
			int modelID = -1,
			float *E_ = NULL,
			int wE = 0,
			bool bMatchAllComponents = false,
			bool bAssociation = true,
			bool *maskS = NULL,
			bool *maskM = NULL); //Matches 2 VN instances
		void GreedyInterpretation(VNInstance *pVNModel, 
			int label, 
			Array2D<float> distanceTable,  
			int *Correspondences,
			void *vpClassifier,
			float &cost_,
			int *correspondencesOrder,
			bool bMatchAllComponents = false,
			bool *maskS = NULL,
			bool *maskM = NULL);
		void GreedyInterpretationComponents(
			Array2D<float> distanceTable,
			std::vector<int> modelComponentLabels,
			int *Correspondences,
			void *vpClassifier,
			float &cost_,
			int *correspondencesOrder,
			Mesh *pMesh);
		float MatchCost(int *X);
		void SaveVNInstance(void *vpClassifier, FILE *fp);
		void LoadVNInstance(void *vpClassifier, FILE *fp, CRVLMem *pMem0 = NULL);
		void ProjectToLatentSubspace(void *vpClassifier);
		void mIoU(VNInstance *pVNModel,
			int *Correspondences,
			float &mIoU,
			void *vpClassifier,
			int *correspondencesOrder,
			Mesh *pMesh); 
		void mIoUComponents(
			int *Correspondences,
			std::vector<int> modelComponentLabels,
			float &mIoU,
			void *vpClassifier,
			int *correspondencesOrder,
			Mesh *pMesh);
		void MatchComponents(RVL::QList<VNInstance> *pModelVNList,
			int *&Correspondences,
			void *vpClassifier,
			Mesh *pMesh); //Matches each scene component with all components from DB
		void AssignLabelsToUnassignedPoints(
			int *Correspondences,
			void *vpClassifier,
			Mesh *pMesh,
			float *ptObjectDist,
			VNInstance *pVNModel = NULL,
			std::vector<int> *modelComponentLabels = NULL
			); //Vidovic
		// void VNInstance::AssignLabelsToUnassignedPoints(
		void AssignLabelsToUnassignedPoints(
			RECOG::VN_::PartAssociation *association,
			void *vpClassifier,
			Mesh *pMesh,
			float *ptObjectDist);//Vidovic
		void AssignComponentsToPoints(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			float *&ptComponentDist);
		int FindClosestMatchedSceneComponent(
			int *Correspondences,
			void *vpClassifier,
			float *P
			);//Vidovic
		void AssignLabelToAllMeshPoints(
			Mesh *pMesh,
			int label);//Vidovic
		void SavePointsWithLabels(
			Mesh *pMesh,
			FILE *fp,
			Array<int> *pSamples = NULL,
			int *GTLabel = NULL); //Vidovic
		void GreedyProbabilisticAssociation(
			void *vpClassifier,
			int iClass,
			int inLabel,
			int *iMetaModel,
			bool *bSCompAssigned,
			bool *bCellAssigned,
			RECOG::VN_::PartAssociation *association);
		void GreedyProbabilisticAssociation(
			void *vpClassifier,
			int iClass,
			bool *bLabelLocked,
			bool *bCellAssigned,
			Array<RECOG::VN_::PartAssociation> &association);
		float GreedyProbabilisticAssociation(
			void *vpClassifier,
			int iClass,
			bool *bLabelLocked,
			bool *bCellAssigned,
			int &bestAssociationLabel,
			int &bestAssociationMetaModel,
			int &bestAssociationGauss);
		void ProbabilisticAssociation(
			void *vpClassifier,
			int iClass,
			RECOG::VN_::PartAssociation *&association);
		float AssociationProbability(
			void *vpClassifier,
			int iClass,
			RECOG::VN_::PartAssociation *association,
			bool *bAssigned); //Vidovic
		float AssociationProbability(
			void *vpClassifier,
			int iClass,
			Array<RECOG::VN_::PartAssociation> association,
			bool *bAssigned); //Vidovic
		//void InitAssociationProbability(
		//	RVL::QList<VNInstance> *pModelVNList,
		//	void *vpClassifier); //Vidovic
		float AssociationProbability(
			void *vpClassifier,
			int iClass,
			int *iSSegment,
			int label,
			int iMetaModel);
		void InitAssociationProbability(
			void *vpClassifier,
			int iClass); //Vidovic
		//void Match2(VNClass VNClass,
		//	void *vpClassifier); //Probabilistic approach
		void NearestNeighborsAssociation(
			void *vpClassifier,
			int iClass,
			RECOG::VN_::PartAssociation *&association);
		float Match(
			VNInstance *pModel,
			float *E,
			int strideS,
			int strideM,
			float *wCell = NULL);
		float Match2(
			VNInstance *pModel,
			float *fComp,
			int strideS,
			float *lnPDFSPrior = NULL,
			float *lnPDFMPrior = NULL);
		void CTINet(
			void *vpClassifier,
			int iClass,
			RECOG::VN_::CTINetOutput &out,
			RECOG::VN_::PartAssociation *&componentAssociation,
			RECOG::VN_::PartAssociation *&cellAssociation,
			int *nMCompsTotalOU = NULL,
			float alpha = 1.0f,
			uchar flags = (RVLVN_CTINET_COMPONENT_LAYER | RVLVN_CTINET_MODEL_LAYER | RVLVN_CTINET_GLOBAL_LOCAL_LAYER | RVLVN_CTINET_LABELS),			
			float *lnPDFPrior = NULL,
			int nModels = 0,
			int excludeModel = -1);
		void AssignLabelsToComponentsAndSurfelCells(
			float *fLabel,
			int nLabels,
			RECOG::VN_::PartAssociation *componentAssociation,
			RECOG::VN_::PartAssociation *cellAssociation,
			float *fCellLabel);
		void VisualizePartAssociation(
			Mesh *pMesh,
			void *vpClassifier,
			int iClass,
			RECOG::VN_::PartAssociation *componentAssociation,
			RECOG::VN_::PartAssociation *cellAssociation,
			int label = -1,
			int assignLabelToUnassigned = -1); //Vidovic
		// void VNInstance::VisualizeGTLabels(
		void VisualizeGTLabels(
			Mesh *pMesh,
			void *vpClassifier); //Vidovic
		// void VNInstance::VisualizeSparseGTLabels(
		void VisualizeSparseGTLabels(
			Mesh *pMesh,
			void *vpClassifier); //Vidovic
		void ProbabilisticAssociationInit(
			void *vpClassifier,
			int iClass,
			RECOG::VN_::ProbabilisticAssociationWorkData *pData);
		void ProbabilisticAssociationFree(RECOG::VN_::ProbabilisticAssociationWorkData *pData);
		void AssociationProbabilityComputationInit(
			void *vpClassifier,
			int iClass,
			RECOG::VN_::AssociationProbabilityWorkData *pData);
		void AssociationProbabilityComputationFree(RECOG::VN_::AssociationProbabilityWorkData *pData);
		void LockCells(
			int iSSegment,
			bool *bCellAssigned);
		void AddAssociation(
			void *vpClassifier,
			int iClass,
			int label,
			int iMM,
			int iGauss,
			Array<RECOG::VN_::PartAssociation> &association,
			bool *bCellAssigned);
		float Fit(
			void *vpClassifier,
			Array2D<float> PC_,
			float alpha,
			float sigma,
			float *&t);
		void PaintComponent(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iComp,
			uchar *color);
		void Display(
			Visualizer *pVisualizer,
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			uchar *color);
		float Fit(
			void *vpClassifier,
			VNInstance *pModel,
			float alphat,
			float alphas,
			float sigma,
			float *&qOut);
		void CreateTRED(
			void *vpClassifier,
			float *d1,
			bool bConcave1,
			float *d2,
			bool bConcave2,
			RECOG::VN_::TRED *pTRED,
			float *r,
			float *w);
		void ComputeTREDMx(void *vpClassifier);
		void MatchComponentNeighborhood(
			void *vpClassifier,
			int iModel_,
			RECOG::VN_::CTINetOutput out);
		void Display(
			Visualizer *pVisualizer,
			void *vpClassifier, 
			float *qIn = NULL,
			bool bPC = false,
			Array2D<float> *pPC = NULL,
			double *colorO = NULL,
			double *colorU = NULL);
		void SparsePCDistanceToComponents(
			void *vpClassifier,
			Mesh *pMesh,
			char *modelFileName,
			float *&ptCompDist,
			bool bSaveToFile = false);
		bool LoadPC(char *modelFileName);

	public:
		int ID;
		int iClass;
		int nComponents;
		int nObjectCellRelations;
		bool *bConcave;
		float *d;
		bool *bd;
		float *R;
		float *q;
		float *s;
		int *iPrimitiveClass;
		Array<SURFEL::Cell> surfelCells;
		Array<int> *componentSurfelCells;
		//QLIST::Index *surfelCellObjectMem;
		int *componentSurfelCellMem;
		VNInstance *pNext;
		Array2D<Array<int>> part;
		int *label;
		int supportSize;
		int *partCells;
		bool bLabels;
		//Array2D<float> conditionalProbability; //Vidovic
		//Array<float> priorProbability; //Vidovic
		RECOG::VN_::VNInstanceWorkData *pWorkData;
		Array2D<float> PC;
		int *iCTIElement;
		RECOG::VN_::TRED *TREDMx;
		Pair<uchar, int> *TREDCluster;
		int *nTREDClusters;
	};
}
