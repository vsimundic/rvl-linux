#pragma once
#include "RVLVTK.h"

// #define RVLPSGM_NORMAL_HULL
#define RVLPSGM_MATCH_SATURATION // VIDOVIC
// #define RVLPSGM_MATCH_SEGMENT_CENTROID //VIDOVIC
// #define PSGM_CALCULATE_PROBABILITY //Vidovic
#define RVLPSGM_EVALUATION_PRINT_INFO  // Vidovic
#define RVLPSGM_MATCH_USING_SEGMENT_GT // Vidovic
// #define RVLPSGM_SAVE_MATCHES //Vidovic
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SQUARE_DISTANCE 1
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_MAX_ABS_DISTANCE 2
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_SATURATED_SQUARE_DISTANCE_INVISIBILITY_PENAL 3
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_MEAN_SATURATED_SQUARE_DISTANCE 4
#define RVLPSGM_MATCH_SIMILARITY_MEASURE_MEDIAN_ABS_DISTANCE 5
// #define RVLPSGM_RANSAC
#ifdef RVLVERSION_171125
#define RVLPSGM_ICP // 170601: ON
#endif
#define RVLPSGM_ICP_SIMILARITY_MEASURE_COSTNN 0
#define RVLPSGM_ICP_SIMILARITY_MEASURE_SATURATED_SCORE 1
#define RVLPSGM_GROUND_PLANE_DISTANCE_PENALIZATION

#define RVLRECOGNITION_MODE_PSGM_CREATE_CTIS 2

#define RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_CTI 0
#define RVLPSGM_HYPOTHESIS_VISUALIZATION_MODE_PLY 1

// #define RVLVN_METAMODEL_APPLE	0
// #define RVLVN_METAMODEL_DONUT	5
// #define RVLVN_METAMODEL_TETRAPAK 8
// #define RVLVN_METAMODEL_TOILETPAPER	9
#define RVLVN_METAMODEL_CONVEX 0
#define RVLVN_METAMODEL_BANANA 1
#define RVLVN_METAMODEL_BOTTLE 2
#define RVLVN_METAMODEL_BOWL 3
#define RVLVN_METAMODEL_TORUS 4
#define RVLVN_METAMODEL_HAMMER 5
#define RVLVN_METAMODEL_MUG 6
#define RVLVN_METAMODEL_PIPE 7
#define RVLVN_METAMODEL_MUG5 8
// #define RVLVN_METAMODEL_CAR		8
// #define RVLVN_METAMODEL_APPLE	8

#define RVLVN_CLUSTER_TYPE_CONVEX 0
#define RVLVN_CLUSTER_TYPE_CONCAVE 1
#define RVLVN_CLUSTER_TYPE_XTORUS 2
#define RVLVN_CLUSTER_TYPE_ITORUS 3
#define RVLVN_CLUSTER_TYPE_PLANE 4

#define RVLPSGM_FACECLUSTER_FLAG_SALIENT 0x01

#ifdef RVLLINUX
#include "Eigen/Dense"
#else
#include "Eigen\Dense"
#endif

vtkSmartPointer<vtkPolyData> GenerateCTIPrimitivePolydata_RW(
    float *planeNormals,
    float *planeDist,
    int nPlanes,
    bool centered = false,
    int *mask = NULL,
    float *t = NULL);

namespace RVL
{
    class PSGM;
    class CTISet;
    namespace RECOG
    {
        namespace PSGM_
        {
            struct Cluster
            {
                Array<int> iSurfelArray;
                Array<int> iVertexArray;
                int size;
                int orig;
                int boundaryDiscontinuityPerc;
                float N[3];
                float dPlane;
                float normalDistributionStd1;
                float normalDistributionStd2;
                bool bValid;
                float *d;
                uchar *bd;
                float *dU;
                uchar *bdU;
                std::shared_ptr<RVLColorDescriptor> colordescriptor; // Filko
                // QList<RECOG::PSGM_::ModelInstance> modelInstanceList; //Vidovic
            };

            struct TangentRegionGrowingData
            {
                RECOG::PSGM_::Plane planeA;
                float cs;
                int iCluster;
                PSGM *pRecognition;
                Array<RECOG::PSGM_::Tangent> *pTangentArray;
                bool *bParent;
                // Array<RECOG::PSGM_::NormalHullElement> *pNormalHull;
                float baseSeparationAngle;
                bool *bBase;
                int *clusterMap;
            };

            struct DisplayData
            {
                PSGM *pRecognition;
                Mesh *pMesh;
                SurfelGraph *pSurfels;
                Visualizer *pVisualizer;
                bool bClusters;
                unsigned char selectionColor[3];
                int iSelectedCluster;
                int iSegmentGTModification; // Vidovic
                vtkSmartPointer<vtkActor> referenceFrames;
                DWORD hypothesisVisualizationMode;
                int *clusterMap;
            };

            // Petra
            struct SegmentMatch
            {
                float Eseg;
                int iCTIs;
                int iCTIm;
                int iSS;
                int iSM;
                int iM;
                Eigen::VectorXf t;
            };

            struct InterclassAlignment
            {
                int idx;
                int iFirstModel;
                int iSecondModel;
                float cost;
                Eigen::Matrix4f T;
                Eigen::Matrix3f R;
                Eigen::Vector3f t;
            };

            // VIDOVIC
            struct MatchInstance
            {
                int ID;
                int iScene;
                int iSCTI;
                int iMCTI;
                int iMS;
                int iSS;
                float R[9]; // rotation after CTI match
                float t[3]; // translation after VTI match
                float tMatch[3];
                float E;
                float score;
                float probability1;
                float probability2;
                float angleGT;
                float distanceGT;
                int nValids;
                float eSeg;
                float gndDistance;       // Vidovic
                float transparencyRatio; // Vidovic
                int nTransparentPts;
                // Petra
                double cost_ICP;
                // float T_ICP[16]; //transformation between current and ICP pose
                float RICP_[9]; // transformation between current and ICP pose
                float tICP_[3]; // transformation between current and ICP pose
                float RICP[9];  // final pose after ICP - Vidovic
                float tICP[3];  // final pose after ICP - Vidovic
                double cost_NN;
                int iClass;
                int iModel;
                float s; // scale for classification
                float t_class[3];
                // end Petra
                bool bValid; // Vidovic
                ushort *pModelDepthImage;
                Eigen::Matrix4f T;
                MatchInstance *pNext;
            };

            struct FPMatch
            {
                int iScene;
                int iModel;
                float t[3];
                int n;
                FPMatch *pNext;
            };

            struct MGT
            {
                int iScene;
                int iSegment;
                int iModel;
                int matchID;
                int CTIrank;
                int ICPrank;
                float CTIscore;
                float ICPcost;
                float gndDistance;
                float transparencyRatio;
                // unsigned char color[3];
                MGT *pNext;
            };
            // END Vidovic

            struct SymmetryMatch
            {
                float d;
                float w;
                bool b;
                int iCTIElement;
            };

            struct Hypothesis
            {
                float R[9];
                float P[3];
                int iMatch;
                int iCell;
                float score;
                Hypothesis *pNext;
                Hypothesis **pPtrToThis;
            };

            template <typename T>
            struct CTIDescriptorMapping
            {
                int nPan;
                int nTilt;
                int nRoll;
                int nCTI;
                int *iD;
                Pair<int, int> *tree;
                T *R;
            };

            int ValidTangent(
                int iSurfel,
                int iSurfel_,
                SURFEL::Edge *pEdge,
                SurfelGraph *pSurfels,
                RECOG::PSGM_::TangentRegionGrowingData *pData);
            bool keyPressUserFunction(
                Mesh *pMesh,
                SurfelGraph *pSurfels,
                std::string &key,
                void *vpData);
            bool mouseRButtonDownUserFunction(
                Mesh *pMesh,
                SurfelGraph *pSurfels,
                int iSelectedPt,
                int iSelectedSurfel,
                void *vpData);
            void _3DNetVOI(
                float *RSG,
                float *tSG,
                float &r);
            bool LoadCTIDescriptorMapping(char *fileName, CTIDescriptorMapping<float> &CTIDescMap);
            void CreateCTIDescriptorMapping2(Array2D<float> A, CTIDescriptorMapping<int> &CTIDescMap);
            void CreateTemplate(
                Array2D<float> A,
                Array<RECOG::PSGM_::Plane> templ);
            template <typename T>
            void DeleteCTIDescriptorMapping(CTIDescriptorMapping<T> CTIDescMap)
            {
                RVL_DELETE_ARRAY(CTIDescMap.iD);
                RVL_DELETE_ARRAY(CTIDescMap.tree);
                RVL_DELETE_ARRAY(CTIDescMap.R);
            }
        } // namespace PSGM_

        struct VNGraspNormals
        {
            int n1, n2, n3, n4, n5;
        };

        struct ClassData
        {
            int iMetaModel;
            int iRefInstance;
            int iFirstInstance;
            int nInstances;
            Array2D<float> M;
            float *qMin;
            float *qMax;
            int nHull;
            float *A;
            float *b;
            int nq0;
            float *q0;
            float T[9];
            int *valid;
            Array<VNGraspNormals> VNGraspNormals_;
            Array<int> faces;
            bool bAxisAlignment;
            Array2D<int> iHypothesisBlocks;
            int iHypotheisisBlocksMem[6];
            bool bConvex;
            bool bConcave;
            float *Q;
        };

        struct SceneCluster
        {
            BYTE type;
            void *vpCluster;
        };

        struct ClusterVisualizationData
        {
            Mesh *pMesh;
            SurfelGraph *pSurfels;
            Visualizer *pVisualizer;
            uchar selectionColor[3];
            uchar *clusterColor;
            int iSelectedCluster;
            int *clusterMap;
            Array<RECOG::SceneCluster> clusters;
        };

        struct FaceCluster
        {
            float N[3];
            float d;
            float area;
            QList<QLIST::Index> faceList;
            Array<int> iPtArray;
            uchar flags;
        };

        struct convexHullFacesPairs
        {
            Pair<int, int> faces;
            float cost;
        };

        typedef void (*ICPfunction)(vtkSmartPointer<vtkPolyData>, vtkSmartPointer<vtkPolyData>, float *, int, float, int, double *, void *);

        bool ModelExistInDB(
            char *modelFileName,
            FileSequenceLoader dbLoader,
            int *pID = NULL); // Vidovic
        void SaveModelID(
            FileSequenceLoader dbLoader,
            char *modelsInDataBase); // Vidovic
        void _3DNetDatabaseClasses(RVL::PSGM *pClassifier);
        void GroupObjectsAccordingToCTIProximity(
            Mesh *pMesh,
            SURFEL::ObjectGraph *pObjects,
            PSGM *pPSGM,
            float distThr,
            uchar mask = 0x00,
            uchar flags = 0x00);
        void _3DNetGround(
            char *sceneFileName,
            float *NGnd,
            float &dGnd);
        void CreateObjectGraphFromSceneClusters(
            Array<RECOG::SceneCluster> SClusters,
            SURFEL::ObjectGraph *pObjects,
            SurfelGraph *pSurfels = NULL,
            Array<int> *pUnassignedSurfels = NULL,
            int minSurfelSize = 0,
            uchar clusteringSurfelFlagMask = 0x00,
            uchar clusteringSurfelFlags = 0x00);
        void DisplayClustersInteractive(
            Visualizer *pVisualizer,
            Array<RECOG::SceneCluster> clusters,
            Mesh *pMesh,
            SurfelGraph *pSurfels,
            PlanarSurfelDetector *pSurfelDetector,
            int *clusterMap,
            uchar *selectionColor,
            uchar *&clusterColor,
            ClusterVisualizationData *pVisualizationData);
        void DisplayClusters(
            Visualizer *pVisualizer,
            Array<RECOG::SceneCluster> SClusters,
            Mesh *pMesh,
            SurfelGraph *pSurfels,
            uchar *clusterColor,
            int *clusterMap);
        void DisplaySelectedCluster(
            Visualizer *pVisualizer,
            Mesh *pMesh,
            SurfelGraph *pSurfels,
            int iSelectedCluster,
            Array<RECOG::SceneCluster> clusters,
            int iPrevSelectedCluster,
            int *clusterMap,
            uchar *selectionColor,
            uchar *clusterColor);
        void DisplaySelectedClusters(
            Visualizer *pVisualizer,
            Mesh *pMesh,
            SurfelGraph *pSurfels,
            Array<int> selectedClusters,
            Array<RECOG::SceneCluster> clusters,
            Array<int> prevSelectedClusters,
            int *clusterMap,
            uchar *selectionColor,
            uchar *clusterColor);
        bool mouseRButtonDownUserFunctionClusterVisualization(
            Mesh *pMesh,
            SurfelGraph *pSurfels,
            int iSelectedPt,
            int iSelectedSurfel,
            void *vpData);
        void ResetClusterColor(
            Visualizer *pVisualizer,
            Mesh *pMesh,
            SurfelGraph *pSurfels,
            int iCluster,
            Array<SceneCluster> clusters,
            uchar *clusterColor,
            int *clusterMap);
        void ConvexMeshFaceClusters(
            Mesh *pMesh,
            Array<FaceCluster> &faceClusters,
            float normalAngleThr,
            bool bVisibleFacesOnly = false,
            int **pptIdxMem = NULL);
        void ReferenceFrames(
            Mesh *pConvexHull,
            Array<Pose3D> &referenceFrames,
            float kFaceClusterSizeThr = 0.0f,
            bool bVisibleFacesOnly = false,
            Array<OrientedPoint> *pZAxes = NULL);
        void ReferenceFrames(
            Array<FaceCluster> faceClusters,
            Array<Pose3D> &referenceFrames);
        vtkSmartPointer<vtkActor> DisplayZAxes(
            Visualizer *pVisualizer,
            Array<OrientedPoint> *pZAxes,
            float size);
        vtkSmartPointer<vtkPolyData> GetVisiblePart(
            vtkSmartPointer<vtkPolyData> PD,
            double *T_M_S,
            bool useTransfPoints = false);
        void ICP(
            RECOG::ICPfunction ICPFunction,
            int ICPvariant,
            vtkSmartPointer<vtkPolyData> model,
            float *RMSInit,
            float *tMSInit,
            vtkSmartPointer<vtkPolyData> scene,
            float *RMS,
            float *tMS,
            float scale = 1.0f);

        namespace PSGM_
        {
            void ConvexAndConcaveClusters(
                Mesh *pMesh,
                SurfelGraph *pSurfels,
                PlanarSurfelDetector *pSurfelDetector,
                PSGM *pConvexClustering,
                PSGM *pConcaveClustering,
                Array<RECOG::SceneCluster> &SClusters_,
                int &nCClusters,
                int &nUClusters,
                int minClusterSize = 0,
                int *clusterMap = NULL,
                int maxnSCClusters = -1,
                int maxnSUClusters = -1,
                bool bConcavity = true,
                int nAdditionalClusters = 0,
                bool bVisualizeConvexClusters = false,
                bool bVisualizeConcaveClusters = false,
                bool bVisualizeSurfels = false,
                bool bVisualizeAllClusters = false);
            void CreateCTIMesh(
                Array2D<float> A,
                float *d,
                Mesh *pMesh,
                int &vertexMemSize,
                CRVLMem *pMem,
                bool bFaces = false);
        }
    } // namespace RECOG
    // class CTISet
    //{
    // public:
    //	CTISet();
    //	virtual ~CTISet();

    //	void LoadSMCTI(char * filePath, Array<RECOG::PSGM_::Plane> *convexTemplate);

    //	Array<RECOG::PSGM_::ModelInstance> CTI;
    //	//std::vector<std::vector<int>> SegmentCTIs;
    //	Array<Array<int>> SegmentCTIs;
    //	int *segmentCTIIdxMem;
    //};

    template <typename T>
    struct NanoFlannPointCloud
    {
        struct Point
        {
            T x, y, z;
        };

        std::vector<Point> pts;

        // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return pts.size(); }

        // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
        inline T kdtree_distance(const T *p1, const size_t idx_p2, size_t /*size*/) const
        {
            const T d0 = p1[0] - pts[idx_p2].x;
            const T d1 = p1[1] - pts[idx_p2].y;
            const T d2 = p1[2] - pts[idx_p2].z;
            return d0 * d0 + d1 * d1 + d2 * d2;
        }

        // Returns the dim'th component of the idx'th point in the class:
        // Since this is inlined and the "dim" argument is typically an immediate value, the
        //  "if/else's" are actually solved at compile time.
        inline T kdtree_get_pt(const size_t idx, int dim) const
        {
            if (dim == 0)
                return pts[idx].x;
            else if (dim == 1)
                return pts[idx].y;
            else
                return pts[idx].z;
        }

        // Optional bounding-box computation: return false to default to a standard bbox computation loop.
        //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
        //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
    };

    class PSGM
    {
    public:
        PSGM();
        virtual ~PSGM();
        // void Create();
        void CreateParamList(CRVLMem *pMem);
        void Init(char *cfgFileName);
        void Init(Mesh *pMesh);
        void Interpret(
            Mesh *pMesh,
            int iScene = 0);

        // Petra
        void InterpreteCTIS(
            Mesh *pMesh);

        void MatchInPrimitiveSpace(
            Eigen::MatrixXf QM,
            Eigen::MatrixXf M,
            int iCTI);

        void CTIMatch(
            Eigen::MatrixXf dM,
            int iCTI);

        // void UpdateMatchMatrix(
        //	RECOG::PSGM_::SegmentMatch *SMatch,
        //	int iCTI
        //	);

        void VisualizeCTIMatch( // Damir
            float *nT,
            float *dM,
            float *dS,
            int *validS);

        Eigen::MatrixXf ConvexTemplatenT();

        void VisualizeCTIMatchidx( // for a given Scene and Model CTI index, calls visualization (prepares descriptors and visibility mask).
            int iSCTI,
            int iMCTI);

        void CalculatePose(int iMatch);

        typedef void (*ICPfunction)(vtkSmartPointer<vtkPolyData>, vtkSmartPointer<vtkPolyData>, float *, int, float, int, double *, void *);

        typedef void (*CUDAICPfunction)(unsigned short *, float *);

        // For a given scene segment adds desired ranked hypotheses to visualizer:
        void AddModelsToVisualizer(Visualizer *pVisualizer, bool align, ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr = NULL);

        // Runs ICP for a hypotheses chosen in "AddModelsToVisualizer" and visualizes it (not recomended, rather use AddOneModelToVisualizer):
        void AddOneModelToVisualizerICP(Visualizer *pVisualizer, int iMatch, bool align, ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr = NULL);

        // Recomended,
        // Visualizes chosen hypotheses 0-6 for each segment on the scene, activated when pressed "c":
        void AddOneModelToVisualizer(Visualizer *pVisualizer, int iMatch, int iRank, bool align, bool useTG = false, bool bICPPose = true, bool bCalculatePose = true, double *color = NULL);

        vtkSmartPointer<vtkActor> AddModelToVisualizer(
            Visualizer *pVisualizer,
            vtkSmartPointer<vtkPolyData> pModel,
            float *RMS,
            float *tMS,
            double *color,
            float pointSize = 3.0f,
            float scale = 1.0f);

        // Visualizes GT models on the scene, activated when pressed "g":
        void AddGTModelsToVisualizer(Visualizer *pVisualizer);

        void LoadModelMeshDB(char *modelSequenceFileName, std::map<int, vtkSmartPointer<vtkPolyData>> *vtkModelDB, bool bDecimate = false, float decimatePercent = 0.4);

        vtkSmartPointer<vtkPolyData> GetSceneModelPC(int iCluster);

        void CalculateICPCost(RVL::PSGM::ICPfunction ICPFunction, int ICPvariant, void *kdTreePtr = NULL);

        static vtkSmartPointer<vtkPolyData> GetVisiblePart(vtkSmartPointer<vtkPolyData> PD); // Models are reduced to only the visible part (using angle between normals) which improves ICP.

        static vtkSmartPointer<vtkPolyData> GetVisiblePart(vtkSmartPointer<vtkPolyData> PD, double *T_M_S, bool useTransfPoints = false); // Models are reduced to only the visible part (using angle between normals) which improves ICP.

        void CalculateNNCost(Visualizer *pVisualizer, RVL::PSGM::ICPfunction ICPFunction, int ICPvariant); // For each pair of scene segment and visible part of the matched model, calls NNCost.

        float NNCost(int iCluster, vtkSmartPointer<vtkPolyData> sourcePD, vtkSmartPointer<vtkPolyData> targetPD, int similarityMeasure = 0); // Calculates cost based on sum of distances between scene segment points and their nearest neighbours in visible part of the matched model.

        float ObjectAlignment(
            Array<int> iSCTIArray,
            RECOG::PSGM_::ModelInstance **SCTIArray,
            Array<int> iMCTIArray,
            RECOG::PSGM_::ModelInstance **MCTIArray,
            Eigen::MatrixXf A,
            float *R,
            float *t,
            bool bTSM = false,
            float *pENrm = NULL);

        // int ObjectAlignment(
        //	RECOG::PSGM_::ModelInstance *pSCTI,
        //	RECOG::PSGM_::ModelInstance *pMCTI,
        //	float *M,
        //	float alpha,
        //	float *t,
        //	float &s,
        //	float &E);

        float *CreateTranslationalSubspace(float &alpha);

        void CreateSubpace(
            float *d,
            float *MT,
            float alpha,
            float &beta,
            float *q);

        float OptimalTranslationAndScale(
            RECOG::PSGM_::ModelInstance *SCTI,
            RECOG::PSGM_::ModelInstance *MCTI,
            float *RLM0_LS0,
            float *MT,
            float alpha,
            float *t,
            float &s);

        void ObjectAlignment(); // Calculates transformation matrix to align object with a reference object (for classification)
        void CalculateAlignmentTransformation(Eigen::VectorXf t, float s, int iSCTI, int iMCTI, RECOG::CTISet &SCTISet, RECOG::CTISet &MCTISet);

        void TangentAlignment(
            int iMatch,
            float eThr,
            float *R,
            float *t,
            float &score,
            Array<RECOG::TangentVertexCorrespondence> &correspondences,
            Array<int> &iVertexArray,
            Array<int> &iSSegmentArray,
            bool *bVertexAlreadyStored);

        void VisualizeAlignedModels(int iRefModel, int iModel); // Visualizes models after alignment - for easier debugging

        void Classify(
            Mesh *pMesh,
            int iModelFirst,
            int iModelLast,
            int &iModel,
            float *R,
            float *t,
            bool bTSM = false); // For a given object on the scene, returns its object class

        void VisualizeObjectClass(int iModel, Mesh *pMesh); // Visualizes object on the scene and its class (most simmilar model from database)

        int FindReferentObject(int first, int last);

        void LoadTransformationMatrices();
        void LoadReferentModels();
        void FindReferentModelForEachClass();
        int ReferenceFrame(
            Mesh *pCovexHull,
            Array<int> iCTIArray,
            RECOG::PSGM_::ModelInstance **CTIArray);
        void CTI(Mesh *pConvexHull, RECOG::PSGM_::ModelInstance *pModelInstance);
        void Normalize(RECOG::PSGM_::ModelInstance *pModelInstance);

        // end Petra

        float groundPlaneDistance(int iModel, double *MSTransform); // Vidovic

        void RMSE(FILE *fp, bool allTPHypotheses = false); // calculate RMSE for TP hypothesis on the scene (only 0-th placed TP hypotheses-> allTPHypothesis = false; all TP hypotheses -> allTPHypothesis = true) - Vidovic

        float RMSE(int iModel, double *TGT, double *T); // calculate RMSE for single model - Vidovic

        void RMSE_Consensus(FILE *fp, bool onlyTPHypothesis = true); // calculate RMSE for consensus hypothesis - Vidovic

        void InitDisplay(
            Visualizer *pVisualizer,
            Mesh *pMesh,
            unsigned char *selectionColor,
            int *clusterMapExternal = NULL);
        void Display();
        void DisplayModelInstance(Visualizer *pVisualizer);
        void DisplayClusters();
        void DisplayCTIs(
            Visualizer *pVisualizer,
            RECOG::CTISet *pCTISet,
            Array<int> *pCTIArray = NULL);
        void DisplayCTI(
            Visualizer *pVisualizer,
            RECOG::PSGM_::ModelInstance *pCTI);
        void DisplayCTI(
            Visualizer *pVisualizer,
            float *N,
            int nT,
            float *d,
            float *R,
            float *t,
            int *mask = NULL,
            double *color = NULL,
            bool bWireframe = false);
        void PaintCluster(
            int iCluster,
            unsigned char *color);
        void ResetClusterColor(int iCluster);
        void PaintClusterVertices(
            int iCluster,
            unsigned char *color);
        void DisplayReferenceFrames();
        void SetSceneFileName(char *sceneFileName_);
        void Learn(
            char *modelSequenceFileName,
            Visualizer *visualizer = NULL); // Vidovic
        void LoadModelDataBase();           // Vidovic
        void Match();                       // Vidovic
        void Match(
            RECOG::PSGM_::ModelInstance *pSModelInstance,
            int startIdx,
            int endIdx); // Vidovic
        void MatchTGs();
        void ModelClusters(
            Pair<int, int> *&clusterInterval,
            int &maxnModelCTIs);
        void AddSegmentMatches(
            int iCluster,
            RECOG::PSGM_::MatchInstance **ppFirstMatch,
            Space3DGrid<RECOG::PSGM_::Hypothesis, float> &HSpace);
        bool IsFlat(
            Array<int> SurfelArray,
            float *N,
            float &d,
            Array<int> PtArray);
        void DetectGroundPlane(SURFEL::ObjectGraph *pObjects);
        void DetectPlanes();
        bool GravityReferenceFrames(
            QList<QLIST::Index> surfelList,
            RECOG::CTISet *pCTISet,
            CRVLMem *pMem_,
            bool bOppositeGnd = false);
        void Clusters();
        void CreateClusterFromObject(
            SURFEL::ObjectGraph *pObjects,
            int iObject);
        int CTIs(
            QList<QLIST::Index> surfelList,
            Array<int> iVertexArray,
            int iModel,
            int iCluster,
            RECOG::CTISet *pCTISet,
            CRVLMem *pMem,
            bool bOppositeGnd = false);
        void CTIs(
            int iModel,
            SURFEL::ObjectGraph *pObjects,
            RECOG::CTISet *pCTISet,
            CRVLMem *pMem);
        void CreateHullCTIs();
        void FitModel(
            Array<int> iVertexArray,
            RECOG::PSGM_::ModelInstance *pModelInstance,
            bool bMemAllocated = false,
            float *PGnd = NULL);
        float Symmetry(
            SURFEL::ObjectGraph *pObjects,
            int iObject1,
            int iObject2,
            RECOG::CTISet *pCTIs,
            Array<RECOG::PSGM_::SymmetryMatch> &symmetryMatch);
        void PrintMatchInfo(
            FILE *fp,
            FILE *fpLog,
            int TP_,
            int FP_,
            int FN_,
            float precision,
            float recall,
            int nSSegments,
            int *firstTP,
            int *firstTPiModel,
            float *firstTPScore,
            float scoreThresh,
            float minScore,
            float maxScore,
            float scoreStep,
            int nBestSegments,
            int iBestMatches,
            int graphID); // Vidovic
        void CalculateScore(
            int similarityMeasure = 3,
            int iFirstCTI = 0,
            int iEndCTI = -1);                                                     // Vidovic
        void UpdateScoreMatchMatrix(RECOG::PSGM_::ModelInstance *pSModelInstance); // Vidovic
        void SortScoreMatchMatrix(bool descending = false);                        // Vidovic
        void EvaluateMatchesByScore(
            FILE *fp,
            FILE *fpLog,
            FILE *fpPoseError,
            FILE *fpnotFirstInfo,
            FILE *fpnotFirstPoseErr,
            int nBestSegments = 0,
            bool evaluateICP = false); // Vidovic
        void WriteClusterNormalDistribution(FILE *fp);
        void MSTransformation(
            RECOG::PSGM_::ModelInstance *pMModelInstance,
            RECOG::PSGM_::ModelInstance *pSModelInstance,
            float *tBestMatch,
            float *R,
            float *t);            // Vidovic
        void SaveMatches();       // Vidovic
        void SaveMatchesMatrix(); // Vidovic
        bool PSGM::CompareMatchToGT(
            RECOG::PSGM_::MatchInstance *pMatch,
            bool poseCheck,
            float angleThresh,
            float distanceThresh); // Vidovic
        bool PSGM::CompareMatchToSegmentGT(
            RECOG::PSGM_::MatchInstance *pMatch); // Vidovic
        bool PSGM::CompareMatchToSegmentGT(
            int iScene,
            int iSSegment,
            int iMatchedModel,
            RECOG::PSGM_::MatchInstance *pMatch = NULL); // Vidovic
        void PSGM::CountTPandFN(
            int &TP,
            int &FN,
            bool printMatchInfo); // Vidovic
        void PSGM::CalculatePR(
            int TP,
            int FP,
            int FN,
            float &precision,
            float &recall);             // Vidovic
        void ConvexTemplateCentoidID(); // Vidovic
        void SaveSegmentGT(
            FILE *fp,
            int iScene); // Vidovic
        void LoadSegmentGT(
            FILE *fp,
            int iScene);                                              // Vidovic
        void LoadCompleteSegmentGT(FileSequenceLoader sceneSequence); // Vidovic
        void LoadCTI(char *fileName);                                 // Vidovic
        bool PoseCheck(
            RVL::GTInstance *pGT,
            RECOG::PSGM_::MatchInstance *pMatch,
            float distanceThresh,
            float angleThresh,
            FILE *fpLog = NULL,
            FILE *fpnotFirstPoseErr = NULL,
            bool evaluateICP = false); // Vidovic
        void FindGTInstance(
            RVL::GTInstance **pGT,
            int iScene,
            int iModel);
        // bool PSGM::CompareMatchToGT(RECOG::PSGM_::MatchInstance *pMatch, ECCVGTLoader *ECCVGT, bool poseCheck, float angleThresh, float distanceThresh); //VIDOVIC
        // void PSGM::CountTPandFN(ECCVGTLoader *ECCVGT, int &TP, int &FN, bool printMatchInfo); //VIDOVIC
        void CreateScoreMatchMatrixICP();
        void CreateScoreMatchMatrixICP_TMP(); // for multiple matches per model
        void FindMinMaxInScoreMatchMatrix(
            float &min,
            float &max,
            Array<Array<SortIndex<float>>> &scoreMatchMatrix_);
        void SaveCTIs(
            FILE *fp,
            RECOG::CTISet *pCTISet,
            int iModel = -1);
        void SaveCTI(
            FILE *fp,
            RECOG::PSGM_::ModelInstance *pCTI,
            int iModel);
        void RVLPSGInstanceMesh(Eigen::MatrixXf nI, float *dI);
        void BoundingBoxSize(
            RECOG::PSGM_::ModelInstance *pBoundingBox,
            float *size);
        void GetHypothesesCollisionConsensus(std::vector<int> *noCollisionHypotheses, Array<Array<SortIndex<float>>> *scoreMatchMatrix, float thr, bool bVerbose = false); // Filko
        bool CheckHypothesesCollision(int firstHyp, int secondHyp, float thr, float *collisionValue = NULL);                                                               // Filko //float *collisionValue added by Vidovic
        float CTIDistance(
            RECOG::PSGM_::ModelInstance *pCTI1,
            Array<int> iVertexArray1,
            RECOG::PSGM_::ModelInstance *pCTI2,
            Array<int> iVertexArray2);
        float GetObjectTransparencyRatio(vtkSmartPointer<vtkPolyData> object, unsigned short *depthImg, float depthThr, int width, int height, float c_fu, float c_fv, float c_uc, float c_vc); // Filko
        void FilterHypothesesUsingTransparency(float tranThr, float depthThr, bool verbose = false);                                                                                            // Filko
        vtkSmartPointer<vtkPolyData> GetPoseCorrectedVisibleModel(int iMatch, bool ICPPose = true, bool bCalculatePose = true);                                                                 // Filko
        void GetTransparencyAndCollisionConsensus(Visualizer *pVisualizer = NULL, bool bVerbose = false);                                                                                       // Vidovic
        void EvaluateConsensusMatches(float &precision, float &recall, bool verbose = false);                                                                                                   // Vidovic
        RECOG::PSGM_::MatchInstance *GetMatch(int matchID);                                                                                                                                     // Vidovic
        RECOG::PSGM_::ModelInstance *GetMCTI(int iMCTI);                                                                                                                                        // Vidovic
        RECOG::PSGM_::ModelInstance *GetMCTI(RECOG::PSGM_::MatchInstance *pMatch);                                                                                                              // Vidovic
        RECOG::PSGM_::ModelInstance *GetSCTI(int iSCTI);                                                                                                                                        // Vidovic
        RECOG::PSGM_::ModelInstance *GetSCTI(RECOG::PSGM_::MatchInstance *pMatch);                                                                                                              // Vidovic
        void ICP(
            RVL::PSGM::ICPfunction ICPFunction,
            int ICPvariant,
            Array<Array<SortIndex<float>>> sceneSegmentHypotheses); // Vidovic //for multiple matches per model
        void ICP_refined(
            RVL::PSGM::ICPfunction ICPFunction,
            int ICPvariant,
            Array<Array<SortIndex<float>>> sceneSegmentHypotheses); // Vidovic //for multiple matches per model
        void ICP_refined_cs_scene(
            RVL::PSGM::ICPfunction ICPFunction,
            int ICPvariant,
            Array<Array<SortIndex<float>>> sceneSegmentHypotheses); // Vidovic //for multiple matches per model
        void ICP(
            RVL::PSGM::CUDAICPfunction CUDAICPFunction,
            Array<Array<SortIndex<float>>> sceneSegmentHypotheses);
        void SubsampleModels(
            char *modelSequenceFileName,
            float samplingRate,
            float scale = 1.0f,
            Visualizer *pVisualizer = NULL);
        void SubsampleModels2(
            char *modelSequenceFileName,
            float voxelSize,
            float scale = 1.0f,
            Visualizer *pVisualizer = NULL);
        void PrintCTIMatches(bool bTAMatches = false);                                                                                   // Vidovic
        void PrintICPMatches();                                                                                                          // Vidovic
        void PrintTAICPMatches();                                                                                                        // Vidovic
        void VisualizeConsensusHypotheses(Visualizer *pVisualizer);                                                                      // Vidovic
        void VisualizeGTMatch(Visualizer *pVisualizer, Array<Array<SortIndex<float>>> *scoreMatchMatrix_ = NULL, bool bICPPose = false); // Vidovic
        int FindCTIMatchRank(int matchID, int iSegment);                                                                                 // Vidovic
        int FindICPMatchRank(int matchID, int iSegment);                                                                                 // Vidovic
        void createVersionTestFile();                                                                                                    // Vidovic
        void checkVersionTestFile(bool verbose = false);                                                                                 // Vidovic
        bool checkVersionTestFile(RECOG::PSGM_::MGT, bool verbose = false);                                                              // Vidovic
        void CreateDilatedDepthImage();
        std::vector<std::vector<int>> GetSegmentBBNeighbourhood(float dist, bool verbose = false);                                                                                                           // Filko
        std::map<int, std::vector<int>> GetSceneConsistancy(float nDist = 0.1, float d1 = 10.0, float d2 = 0.01, bool verbose = false);                                                                      // Filko
        std::map<int, std::vector<int>> GetSceneConsistancy(Array<Array<SortIndex<float>>> *scoreMatchMatrix, float nDist = 0.1, float d1 = 10.0, float d2 = 0.01, bool bICP = false, bool verbose = false); // Vidovic - added scoreMatchMatrix to function parameters
        int CheckHypothesesToSegmentEnvelopmentAndCollision(int hyp, int segment, float d1, float d2, RECOG::PSGM_::ModelInstance *pSCTI, bool bICP = false);                                                // Filko
        void CheckHypothesesToSegmentEnvelopmentAndCollision_DEBUG(int hyp, int segment, float d1, float d2);                                                                                                // Filko
        bool CheckHypothesesToSegmentEnvelopment(int iHypothesis, int iSegment, float thresh);                                                                                                               // Vidovic
        void FindBestGTHypothesis();                                                                                                                                                                         // Vidovic
        void CreateSegmentGT();                                                                                                                                                                              // Vidovic
        void AttachSegmentToModel(
            int iSegment,
            int iModel);                                                                                                              // Vidovic
        void PaintGTSegments();                                                                                                       // Vidovic
        void PrintSegmentGT();                                                                                                        // Vidovic
        void SaveSegmentGT();                                                                                                         // Vidovic
        bool LoadSegmentGT();                                                                                                         // Vidovic
        void DetermineThresholds();                                                                                                   // Vidovic
        float HypothesesToSegmentEnvelopment(int iHypothesis, int iSegment, RECOG::PSGM_::ModelInstance *pSCTI, bool ICPPose = true); // Vidovic
        float HypothesesToSegmentCollision(int iHypothesis, int iSegment, RECOG::PSGM_::ModelInstance *pSCTI, bool ICPPose = true);   // Vidovic
        int FindMGTHypothesis(int iSegment);                                                                                          // Vidovic
        void SaveModelInstances(
            FILE *fp,
            int iModel = -1);
        void GetVertices(
            int iMatch,
            Array<int> &iVertexArray,
            Array<int> &iSSegmentArray,
            bool *bVertexAlreadyStored);
        void SampleScene();
        float HypothesisEvaluation(
            int iHypothesis,
            Array<int> iSSegmentArray,
            bool bICP = false,
            bool bVisualize = false,
            float scale = 0.001);
        float HypothesisEvaluation2(
            int iHypothesis,
            int &nTransparentPts,
            bool bICP = false,
            float scale = 1.0f,
            bool bVisualize = false,
            RECOG::SceneFittingError *errorRecord = NULL);
        float HypothesisEvaluationIP2(
            Array<Point> modelPC,
            float *RMS,
            float *tMS,
            float scale,
            float maxe,
            int &nTransparentPts,
            int *SMCorrespondence = NULL,
            RECOG::SceneFittingError *errorRecord = NULL);
        void DisplayHypothesisEvaluationIP2(
            Visualizer *pVisualizer,
            int *SMCorrespondence,
            int nTransparentPts,
            vtkSmartPointer<vtkActor> *actor);
        float HypothesisEvaluationIP(
            Array<OrientedPoint> pointsM,
            float *RMS,
            float *tMS,
            float maxeP,
            int &nVisiblePts,
            int &nTransparentPts,
            Array<int> *pTP = NULL,
            Array<int> *pFP = NULL,
            Array<int> *pFN = NULL);
        void ICP(
            Array<OrientedPoint> pointsM,
            float *RMS0,
            float *tMS0,
            float maxeP,
            int nIterations,
            float *RMS,
            float *tMS);
        void HypothesisEvaluation(
            Array<Array<SortIndex<float>>> segmentHypothesisArray,
            bool bICP = false);
        void VisualizeHypotheses(
            Array<Array<SortIndex<float>>> segmentHypothesisArray,
            bool bICP = false);
        void Project(
            Array<Point> PtArray,
            float *R,
            float *t,
            float *Rs);
        float GroundDistance(
            Array<Point> PtArray,
            float *R,
            float *t,
            float s);
        void CreateModelPCs();
        void DeleteModelPCs();
        void InitZBuffer(Mesh *pMesh);
        void SceneBackward();
        void SetScene(int iSceneIn);
        void SaveZBuffer(char *fileName);
        void SaveHypothesisProjection(int iHypothesis);
        void SaveSubsampledScene();
        void createModelDepthImage(ushort *depthImage);                                                                                               // Vidovic
        void CreateSubsampledSceneDepthImage(ushort *depthImage);                                                                                     // Vidovic
        void PrepareCUDACTIMatchingMatrices(float **W, float **VA, int **V, float **Ds, float **Rs, float **ts, bool bSaveToFile = false);            // Vidovic
        void PrepareCUDACTIMatchingOfflineMatrices(float **Dm, float **Rm_inv, float **tm_inv, bool bSaveToFile = false);                             // Vidovic
        void PrepareCUDACTIMatchingMatrices_ColumnWise(float **W, float **VA, int **V, float **Ds, float **Rs, float **ts, bool bSaveToFile = false); // Vidovic
        void PrepareCUDACTIMatchingOfflineMatrices_ColumnWise(float **Dm, float **Rm_inv, float **tm_inv, bool bSaveToFile = false);                  // Vidovic
        void VisualizeDBModels(char *modelSequenceFileName, Visualizer *visualizer);                                                                  // for creating images - Vidovic
        void VisualizeModel(char *modelMeshFilePath, Visualizer *visualizer);                                                                         // for creating images - Vidovic
        void VisualizeCTI(int iCTI, bool DBmodelsCTI = false, Visualizer *pVisualizer = NULL);                                                        // for creating images - Vidovic
        void VisualizeWholeModelCTI(int iModel, Visualizer *pVisualizer = NULL);                                                                      // for creating images - Vidovic
        void TemplateMatrix(Array2D<float> &A);
        void SaveObjectPose(FILE *fp);
        float CompareBoundingBoxPoses(
            float *R1,
            float *t1,
            float *R2,
            float *t2,
            Array2D<double> PBox,
            int iModel = -1);
        void UpdateClustersColorHistograms();                                                 // Filko
        void TestCHMatching();                                                                // Filko
        int FindGTObjectIdx(int iCluster, float thr);                                         // Filko
        float CHMatching(Array<int> iSSegmentArray, int iModel, int metric, FILE *fp = NULL); // Vidovic
        void SaveCHModelDB();                                                                 // Vidovic
    private:
        void WholeMeshCluster();
        void CreateTemplate66();
        void CreateTemplateBox();
        bool ReferenceFrames(int iCluster);
        bool ReferenceFrames(
            RECOG::PSGM_::Cluster *pCluster,
            int iCluster = -1);
        bool ReferenceFrames(
            Array<int> iSurfelArray,
            int *clusterMap,
            int iCluster = -1);
        bool Inside(
            int iVertex,
            RECOG::PSGM_::Cluster *pCluster,
            int iSurfel = -1);
        bool BelowPlane(
            RECOG::PSGM_::Cluster *pCluster,
            Surfel *pSurfel,
            int iFirstVertex = 0);
        void UpdateMeanNormal(
            float *sumN,
            float &wN,
            float *N,
            float w,
            float *meanN);
        void ComputeClusterNormalDistribution(
            RECOG::PSGM_::Cluster *pCluster);
        void ComputeClusterBoundaryDiscontinuityPerc(int iCluster);
        RECOG::PSGM_::ModelInstance *AddReferenceFrame(
            // int iCluster, //Vidovic
            float *R = NULL,
            float *t = NULL);
        void PrintCTIMeshFaces(FILE *fp, Eigen::MatrixXi F, Eigen::MatrixXi Fn, int n, Eigen::MatrixXi nP);

    public:
        CRVLParameterList ParamList;
        DWORD mode;
        DWORD problem;
        DWORD dataSet;
        CRVLMem *pMem;
        CRVLMem *pMem0;
        void *vpMeshBuilder;
        bool (*LoadMesh)(void *vpMeshBuilder,
                         char *FileName,
                         Mesh *pMesh,
                         bool bSavePLY,
                         char *PLYFileName,
                         char *depthFileName);
        PlanarSurfelDetector *pSurfelDetector;
        void *vpObjectDetector;
        SurfelGraph *pSurfels;
        SURFEL::ObjectGraph *pObjects;
        Mesh *pMesh;
        RECOG::PSGM_::DisplayData displayData;
        Array<RECOG::PSGM_::Cluster *> clusters;
        int *clusterMap;
        char *modelsInDataBase;
        char *hollowModelsFileName;
        int nDominantClusters;
        int nBestMatchesPerCluster;
        float kNoise;
        Array<RECOG::PSGM_::Plane> convexTemplate;
        Array<RECOG::PSGM_::Plane> convexTemplate66;
        Array<RECOG::PSGM_::Plane> convexTemplateBox;
        int minInitialSurfelSize;
        int minVertexPerc;
        int planeDetectionMinInitialSurfelSize;
        int planeDetectionMinJoinedPlaneSurfelSize;
        float kReferenceSurfelSize;
        float kReferenceTangentSize;
        float baseSeparationAngle;
        float edgeTangentAngle;
        float gndCTIThr;
        float tangentAlignmentThr;
        float wTransparency1;
        float wTransparency2;
        float wGndDistance1;
        float wGndDistance2;
        float gndDistanceThresh;
        float wColor;
        float segmentGroupingDistanceThr;
        float transparencyDepthThr;
        float gndDetectionTolerance;
        float planeDetectionTolerance;
        float planeDetectionMinBoundingSphereRadius;
        int nModels;    // Vidovic
        int nMSegments; // Vidovic
        int minClusterSize;
        int maxClusterSize;
        int minSignificantClusterSize;
        int minClusterBoundaryDiscontinuityPerc;
        float minClusterNormalDistributionStd;
        float groundPlaneTolerance;
        int sceneSamplingResolution;
        bool bTangentRFDescriptors;
        bool bZeroRFDescriptor;
        bool bGTRFDescriptors;
        bool bGroundPlaneRFDescriptors;
        bool bMatchRANSAC; // Vidovic
        bool bGnd;
        bool bWholeMeshCluster;
        bool bDetectGroundPlane;
        bool bOverlappingClusters;
        bool bICP;
        bool bVisualizeHypothesisEvaluationLevel1;
        bool bVisualizeHypothesisEvaluationLevel2;
        bool bGenerateModelCTIs;
        bool bLearnMetamodels;
        bool bGenerateModelVNInstances;
        bool bLoadModelVNInstances;
        bool bAlignModels;
        bool bFindReferentModelForEachClass;
        bool bSegmentationOnly;
        bool bEdgeClusters;
        bool bObjectDetection;
        bool bLabelConstrainedClustering;
        bool bSurfelUncertainty;
        bool bModelsInMillimeters;
        bool bUseColor;
        Array<int> activeModels;
        Array<RECOG::PSGM_::ModelInstance> modelInstanceDB;    // Vidovic
        QList<RECOG::PSGM_::MatchInstance> CTImatches;         // Vidovic
        Array<RECOG::PSGM_::MatchInstance *> pCTImatchesArray; // Vidovic
        // RECOG::PSGM_::MatchInstance *pCurrentSceneMatch; //Vidovic
        // QList<RECOG::PSGM_::MatchInstance> SSegmentMatches1; //Vidovic - probability1
        // QList<RECOG::PSGM_::MatchInstance> SSegmentMatches2; //Vidovic - probability2
        Array<Array<SortIndex<float>>> scoreMatchMatrix;
        Array<Array<SortIndex<float>>> scoreMatchMatrixICP;
        Array<Array<SortIndex<float>>> sceneSegmentMatches;
        Array<SortIndex<float>> sceneSegmentMatchesArray;
        Array<Array<SortIndex<float>>> bestSceneSegmentMatches;
        Array<Array<SortIndex<float>>> bestSceneSegmentMatches2;
        Array<SortIndex<float>> bestSceneSegmentMatchesArray;
        Array<SortIndex<float>> bestSceneSegmentMatchesArray2;
        Array2D<float> hullCTIDescriptorArray;
        Array<QList<QLIST::Index>> sceneSegmentSampleArray;
        Array<Array<OrientedPoint>> sampledModels;
        QLIST::Index *sceneSegmentSampleMem;
        unsigned char *clusterColor;
        // bool *imageMask;
        bool *surfelMask;
        // Array2D<Array<int>> matchMatrix;
        // int *matchMatrixMem;
        DWORD scoreCalculation; // Vidovic - TO DO (Implement read from cfg file)
        ECCVGTLoader *pECCVGT;  // Vidovic
        Array<RVL::SegmentGTInstance> segmentGT;
        // Array <RVL::SegmentGTInstance> *pModelsSegmentGT;
        QList<SegmentGTInstance> modelsSegmentGTList;
        RECOG::CTISet CTISet;
        RECOG::CTISet MCTISet;
        RECOG::TGSet STGSet;
        RECOG::TGSet MTGSet;
        VertexGraph *pSVertexGraph;
        CRVLTimer *pTimer;
        FILE *fpTime;
        Eigen::MatrixXf nT;                 // Petra
        RECOG::PSGM_::SegmentMatch *SMatch; // Petra
        SortIndex<float> *sortedMatches;    // Petra
        // Eigen::VectorXf E;
        Eigen::MatrixXf t;
        std::map<int, vtkSmartPointer<vtkPolyData>> vtkModelDB;
        std::map<int, vtkSmartPointer<vtkPolyData>> vtkRMSEModelDB;   // Vidovic
        std::map<int, vtkSmartPointer<vtkPolyData>> segmentN_PD;      // neighbourhood
        std::map<int, void *> segmentN_KdTree;                        // neighbourhood
        std::map<int, std::shared_ptr<RVLColorDescriptor>> cdModelDB; // Filko
        // unsigned short * depthImg; //Current scene depth image // Filko //Vidovic commented
        cv::Mat depth;
        std::vector<int> transparentHypotheses;         // Vidovic
        std::vector<int> consensusHypotheses;           // Vidovic
        std::vector<int> noCollisionHypotheses;         // Vidovic
        std::vector<int> envelopmentColisionHypotheses; // Vidovic
        std::map<int, Array<Point>> modelPCs;

        bool createMatchGT;  // Vidovic
        int matchGTiS;       // Vidovic
        int matchGTiRank;    // Vidovic
        FILE *fpMatchGT;     // Vidovic
        char *sceneFileName; // Vidovic
        QList<RECOG::PSGM_::MGT> MGTList;
        bool createSegmentGT;          // Vidovic
        bool segmentGTLoaded;          // Vidovic
        bool visualizeTPHypotheses;    // Vidovic
        bool visualizeCTITPHypotheses; // Vidovic
        Array<RVL::ModelColor> modelColors;
        FILE *fpDetermineThresh, *fpDetermineThresh_, *fpHypothesesFiltering;
        // FILE *fpSegmentEnvelopment, *fpSegmentEnvelopmentCTI, *fpSegmentCollision, *fpSegmentCollisionCTI, *fpHypothesisCollision, *fpHypothesisTransparency, *fpHypothesisTransparencyCTI, *fpHypothesisGndDistance, *fpHypothesisGndDistanceCTI;
        FILE *fpPrunning;           // For results generation - Vidovic
        int nExperimentSegments;    // For results generation - Vidovic
        FILE *fpExperimentSegments; // For results generation - Vidovic
        float timeSegmentation, timeDescriptorGeneration, timeDescriptorMatchingAndE1, timeTangentAlignmentAndE2;

        float clusterType;

        // Petra & Ivan
        double *icpTMatrix;

        // For InstanceMesh:
        Eigen::MatrixXf P;     // points list
        Eigen::MatrixXi F;     // faces list (polygones)
        Eigen::MatrixXi Edges; // Edges
        float NGnd[3];
        float dGnd;
        int iGndObject;
        float symmetryMatchThr;

        // For alignment:
        Eigen::Matrix4f T0i;
        Array<Eigen::Matrix4f> TAlignmentWithReferentModel;
        int iCorrectClass;
        char *modelDataBase; // Vidovic
        Array2D<Point> ZBuffer;
        Array<int> ZBufferActivePtArray;
        int *subImageMap;
        Camera camera;
        char *falseHypothesesFileName;
        char *TPHypothesesCTIRankFileName; // Vidovic
        char *transformationMatricesClassification;
        char *referentModels;
        char *resultsFolder;
        Array<RECOG::ClassData> classArray;
        // float *transformationMatricesClassificationArray;
        Array<Array<RECOG::PSGM_::InterclassAlignment>> TArray;
        RECOG::PSGM_::InterclassAlignment *TArrayMem;
        RECOG::PSGM_::CTIDescriptorMapping<int> CTIDescMap;

        int nSmallSegments; // Vidovic - only for debug

        // for ICP CUDA
        Array<ushort *> modelsDepthImage;
        ushort *pSubsampledSceneDepthImage;

        int nBestHypothesesPerSSegment;

        // Pointer to KdTree of whole scene
        void *pKdTree;
        int debug1, debug2;
        BYTE clusteringSurfelFlags;
        BYTE clusteringSurfelFlagMask;
        bool *bHollow;
        int image3x3Neighborhood[9];

    private:
        RECOG::PSGM_::Cluster *clusterMem;
        int *clusterSurfelMem;
        int *clusterVertexMem;
        // RECOG::PSGM_::ModelInstanceElement *modelInstanceMem;
        vtkSmartPointer<vtkPolyData> referenceFramesPolyData;
        // char *sceneFileName; //moved to public - Vidovic
        // int nSamples; //RANSAC //Vidovic
        int stdNoise;             // RANSAC //Vidovic
        bool bNormalValidityTest; // Vidovic
        bool bBoundingPlanes;
        char *sceneMIMatch;                              // Vidovic
        int iScene;                                      // Vidovic
        Array<QLIST::Index> centroidID;                  // Vidovic
        QList<QLIST::Index> *pISampleCandidateList;      // Vidovic
        Array<QLIST::Index> iValidSampleCandidate;       // Vidovic
        Array<QLIST::Index> iValid;                      // Vidovic
        Array<QLIST::Index> iRansacCandidates;           // Vidovic
        Array<QLIST::Index> iConsensus;                  // Vidovic
        Array<QLIST::Index> iConsensusTemp;              // Vidovic
        Array<Array<float>> e;                           // Vidovic
        Array<Array<float>> tBestMatch;                  // Vidovic
        Array<float> score;                              // Vidovic
        QList<RECOG::PSGM_::MatchInstance> *pCTImatches; // Vidovic
        RECOG::PSGM_::MatchInstance *pCTIMatch;          // Vidovic
        RECOG::PSGM_::MatchInstance *pFirstSCTIMatch;    // Vidovic
        int matchID;                                     // Vidovic
        float *nTc;                                      // Vidovic
        float *dISMc;                                    // Vidovic
        int CTIIdx;                                      // Vidovic
        int nBestMatches;                                // n best matches for each scene segment
        RECOG::PSGM_::MatchInstance *CTIMatchMem;
        int nCorrectHyp;
        int *hypClass;
        int *hypClass2;
    };

}