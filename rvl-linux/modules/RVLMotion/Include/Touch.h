#pragma once

#define RVLSOLID_CONTACT_TYPE_NONE 0
#define RVLSOLID_CONTACT_TYPE_POINT_PLANE 1
#define RVLSOLID_CONTACT_TYPE_EDGE_EDGE 2
#define RVLSOLID_CONTACT_TYPE_PLANE_POINT 3

#define RVLMOTION_TOUCH_NUM_PARAMS 15
#define RVLMOTION_TOUCH_CONTACT_TYPE_NONE 0
#define RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE 1
#define RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE 2
#define RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT 3
#define RVLMOTION_TOUCH_NUM_REF_PTS 6
#define RVLMOTION_TOUCH_SIMULATION_RND 0
#define RVLMOTION_TOUCH_SIMULATION_OPEN 1
#define RVLMOTION_TOUCH_OPTIMIZATION_METHOD_BRUTEFORCE 0
#define RVLMOTION_TOUCH_OPTIMIZATION_METHOD_RNDSAMPLING 1

namespace RVL
{
    bool Line2DCircleIntersection(
        float a,
        float b,
        float c,
        float r,
        float *P1,
        float *P2);

	struct SolidVertex
	{
		float P[3];
		bool bConvex;
		int iEdge;
		int iSrcVertex;
	};

	struct SolidEdge
	{
		int iVertex;
		int iFace;
		int iNext;
		int iPrev;
		int iTwin;
		float len;
        float V[3];
		bool bConvex;
	};

	struct SolidFace
	{
		float N[3];
		float d;
		int iEdge;
		float area;
		bool bCovered;
		int iSrcPlane;
	};

	struct SolidParams
	{
		MOTION::Plane *plane;
		Vector3<float> *vertex;
	};

	struct SolidEdgeFaceIntersection
	{
		float P[2][3];
		int iEdgePart;
		int iEdge;
		int iFacePart;
		int iFace[2];
		bool bEdgeEndPoint[2];
	};

#define RVLSOLID_EDGE_END_VERTEX(pEdge, solid) (solid.edges.Element[pEdge->iNext].iVertex)
#define RVLSOLID_EDGE_END_VERTEX_(pEdge, pSolid) (pSolid->edges.Element[pEdge->iNext].iVertex)

	class Solid
	{
	public:
		Solid();
		virtual ~Solid();
		void Clear();
		void Add(Solid *pSolid);
		void Union();
		void Update();
		void Create(Array<Array<int>> faces_);
		void Move(
			Solid *pSolidSrc,
			Pose3D *pMove);
		void Copy(
			Solid *pSolidSrc,
			bool bCreate = true);
		void ComputeFaceParams();
		int GetNumVertices();
		void GetVertices(
			Array<Vector3<float>> &verticesOut,
			bool bAppend = false);
		void GetVertexEdgesAndFaces(
			int iVertex,
			Array<int> &vertexEdges,
			Array<int> &vertexFaces);
		void SetBoxParameters(
			float *size,
			Pose3D *pPose);
		Solid *CreateBox(
			float *size,
			Pose3D *pPose);
		bool InSolid(float *P);
		bool Intersect(
			float *P1,
			float *P2,
			float *s,
			int *iFace,
			float *V,
			float &l);
		bool Intersect(
			Solid *pSolid,
			Array<SolidEdgeFaceIntersection> *pIntersectionsWithOtherEdges,
			Array<SolidEdgeFaceIntersection> *pIntersectionsWithThisEdges = NULL);
		bool IntersectWithConvex(
			Solid *pSolid,
			Array<SolidEdgeFaceIntersection> *pIntersectionsWithOtherEdges,
			Array<SolidEdgeFaceIntersection> *pIntersectionsWithThisEdges = NULL);
		float FreeMove(
			float *V,
			Solid *pObstacle,
			uchar &contactType,
			int &iThisContactFeature,
			int &iOtherContactPart,
			int &iOtherContactFeature,
			bool bVisualize = false);
		std::vector<vtkSmartPointer<vtkActor>> Visualize(
			Visualizer *pVisualizer,
			uchar *color);
		void Log(FILE *fp);

	public:
		Array<SolidVertex> vertices;
		Array<SolidEdge> edges;
		Array<SolidFace> faces;
		std::vector<Solid *> solids;
		Box<float> bbox;
		Visualizer *pVisualizer;
        int maxnVertexEdges;
	};

	namespace MOTION
	{
		struct Vertex
		{
			float P[3];
			Array<Pair<int, int>> solidVertices;
		};

		struct PlanarSurface
		{
			MOTION::Plane plane;
			Array<int> VNFeatures;
			Array<Pair<int, int>> solidFaces;
		};

		struct TouchModel
		{
			Pose3D pose_C_E;
			Camera camera;
			float K[9];
			float kappa;
			float kappazn;
			float TCP_E[3];
			VN *pVNEnv;
			float *d;
			Pose3D pose_A_E;
			float PAxis_E[3];
			SolidParams *pEnvSolidParams;
		};

		struct TouchSample
		{
			float P[3];
			int iAxis;
			float direction;
		};

		struct Contact
		{
			int iTouch;
			uchar type;
			bool bMiss;
			int iToolFeature;
			float toolVertex[2][3];
			MOTION::Plane toolPlane[2];
			int nBoundaryPlanes;
			int iFirstBoundaryPlane;
			Pair<int, int> iEnvFeature;
			int iEnvVertex[2];
			int iEnvPlane[2];
		};

		class TouchEnvModel;

		struct TouchData
		{
			Pose3D pose;
			float V[3];
			float t;
			MOTION::Contact contact;
			int iFirstContact;
			int nContacts;
			int sessionIdx;
			int sceneIdx;
			bool bMiss;
			MOTION::TouchEnvModel *pEnvSolidParams;
		};

		struct Tool
		{
			// Array<Vector3<float>> vertices;
			// Array2D<float> vertices;
			// Array2D<int> faces;
			Solid solid;
			float TCP[3];
			VN *pVN;
			Array<RECOG::VN_::ModelCluster *> VNMClusters;
			float *d;
		};

		struct TouchPoint
		{
			int iPanel;
			int iFace;
			int iEdge;
			float u;
			float v;
		};

		struct TouchLMError
		{
			float ravg;
			float gmax;
			float Ex;
			float E;
		};

        struct LineArc
        {
            float P[2];
            float q;
            bool bArc;
            char rel;
        };

        struct DoorExperimentParams
        {
            int idx;
			int sessionIdx;
			int sceneIdx;
            float sx;
            float sy;
            float sz;
            float rx;
            float ry;
            float a;
            float b;
            float c;
            float sxgt;
            float sygt;
            float szgt;
			float rxgt;
			float rygt;
            bool bGT;
			Pose3D pose_A_0_gt;
            float qDeg;
			float qDeg_gt;
            Pose3D pose_E_0;
            Pose3D pose_A_C;
            Pose3D pose_C_E;
        };

		struct TouchSolution
		{
			float x[RVLMOTION_TOUCH_NUM_PARAMS];
			float cost;
			int *contacts;
		};

		struct TouchTarget
		{
			int iSolid;
			Array<int> vertices;
			Array<int> edges;
			Array<int> faces;
		};

        void InitCircleConvex(
            QList<QLIST::Entry2<LineArc>> *pCircleConvex,
            float r,
            QLIST::Entry2<LineArc> *&pNewLineArc);
        void UpdateCircleConvex(
            QList<QLIST::Entry2<LineArc>> *pCircleConvex,
            float *N,
            float d,
            float r,
            float eps,
            QLIST::Entry2<LineArc> *&pNewLineArc);
        void TestCircleConvex(Array<int> rndVal); // Only for debugging purpose!!!

		class TouchEnvModel
		{
		public:
			TouchEnvModel();
			virtual ~TouchEnvModel();
			void Create(
				Solid *pSolidIn,
				int nVertices,
				int nPlanes);
			void Clear();

		public:
			Solid *pSolid;
			int idx;
			SolidParams model0;
			SolidParams modelx;
			Array<int> verticesForUpdate;
			Array<int> planesForUpdate;
			bool *bVerticesForUpdate;
			bool *bPlanesForUpdate;
		};
	}

	class Touch
	{
	public:
		Touch();
		virtual ~Touch();
		void Create(char *cfgFileName);
		void CreateParamList();
		void Clear();
		void LM(
			float *x0,
			Array<MOTION::TouchData> touches,
			float *x,
			MOTION::TouchLMError *pErr = NULL);
		float SDF(
			MOTION::TouchData *pTouchData,
			VN *pVNEnv,
			float *d);
		Pair<float, float> Error(
			MOTION::TouchData *pTouchData,
			bool bToolPositioned = false,
			bool bPointTool = false,
			float *pOrthogonalDist = NULL,
			float *pLateralDist = NULL);
		void Sample(
			int nSamples,
			float rayDist,
			Array<MOTION::TouchSample> &samples);
		void CreateEnvironmentModelTemplete(
			MOTION::TouchModel *pModel,
			int nPanels,
			Array<RECOG::PSGM_::Plane> CT,
			Pair<float, float> betaInterval,
			Pose3D *pPose_Arot_A = NULL);
		void DeleteTouchModel(MOTION::TouchModel *pModel);
		void CreateSceneSolid(
			float sx,
			float sy,
			float sz,
			float rx,
			float ry,
			float a,
			float b,
			float c,
			float qDeg,
			bool bUpdate = false);
		void CreateScene(
			float sx,
			float sy,
			float sz,
			float rx,
			float ry,
			float a,
			float b,
			float c,
			float qDeg,
			bool bUpdate = false);
		void CreateSimpleTool(
			float a,
			float b,
			float c,
			float d,
			float h,
            Pose3D *pPose_tool_E = NULL);
		void Simulation(std::vector<MOTION::DoorExperimentParams> &simParams);
		void SimulateVisionWithError(
			float *x,
			bool bNewRndx = true);
		void Correct(
			MOTION::TouchModel *pModelSrc,
			float *x,
			MOTION::TouchModel *pModelTgt);
		void CorrectVertex(
			float *P0,
			float *a_,
			float b_,
			float *D,
			Pose3D *pPose_C_E_0,
			Pose3D *pPose_C_E_x,
			float *Px);
		void CorrectPlane(
			MOTION::Plane *pPlaneSrc,
			float *a,
			float b,
			float *M,
			float *V,
			MOTION::Plane *pPlaneTgt);
		void AddContact(
			MOTION::Contact contact,
			MOTION::TouchData *pTouch,
			std::vector<MOTION::Contact> &contacts);
		void ContactsEE(
			MOTION::TouchData *pTouch,
			std::vector<MOTION::Contact> &contacts,
			bool bTarget = false);
		void Contacts(
			MOTION::TouchData *pTouch_E,
			std::vector<MOTION::Contact> &contacts,
			bool bVisualization = false);
		bool Correction(
			float *xInit,
			Array<MOTION::TouchData> touches_E,
			std::vector<MOTION::Contact> &contacts,
			float *x,
			MOTION::Contact *pContactGT = NULL,
			float *x_gt = NULL,
			bool bUseGTContacts = false);
		void ClearSession(
			std::vector<MOTION::TouchData> &touches,
			std::vector<MOTION::Contact> &contacts);
		void Reconstruct(
			float *P_E,
			MOTION::TouchModel *pModel_x,
			float *P_E_x,
			float *P_C_x,
			float *invKxIn = NULL);
		void SetVerticesAndPlanes(
			Array<MOTION::PlanarSurface> surfaces_,
			Array<MOTION::Vertex> vertices_,
			Pose3D pose,
			MOTION::TouchModel *pModel);
		void UpdateVerticesAndPlanes(SolidParams *pModel);
		void CopyVerticesAndPlanesFromSolid();
		void UpdateDoorAxis(MOTION::TouchModel *pModel0);
		void UpdateDoorOrientation(MOTION::TouchModel *pModel);
		void UpdatEnvSolidParams(
			MOTION::TouchEnvModel *pEnvModel,
			Array<MOTION::TouchData> *pTouches,
			MOTION::TouchModel *pModel0,
			float *x,
			MOTION::TouchModel *pModelx);
		void UpdateEnvironmentModel(
			MOTION::TouchEnvModel *pEnvModel,
			MOTION::TouchModel *pModel0,
			float *x,
			MOTION::TouchModel *pModelx);
		void UpdateEnvironmentVNModel(
			MOTION::TouchModel *pModel0,
			float *x,
			MOTION::TouchModel *pModelx);
		bool Intersection(
			float *N1,
			float *N2,
			float *N3,
			float *N4);
		bool IntersectionWithUncert(
			MOTION::TouchData *pTouch,
			Array<Vector3<float>> *pEdgeVectors = NULL);
        bool Intersection(
            Solid *pSolid1,
            int iVertex,
            Solid *pSolid2,
            int iFace);
        bool Intersection(
            Array<Vector3<float>> vertexEdgeVectors,
            float *N);
        bool IntersectionEE(
            Solid *pSolid1,
            int iEdge1,
            Solid *pSolid2,
            int iEdge2);
		void RndTouchPoint(MOTION::TouchPoint &touchPt);
		void DoorRefFrame(
			Solid *pEnvSolid,
			Pose3D &pose_D_W,
			float *R_W_D);
        void OpenDoorContactPoint(
			Pose3D pose_Ek_W_gt,
			Pose3D pose_E_W,
			Pose3D &pose_Ek_E,
			Pose3D &pose_D_E);
		void RefPts();
		void PlanTouch(
			MOTION::TouchPoint touchPt,
			Pose3D &initPose,
			Array<Vector3<float>> &path,
			float *PTgt);
		void SimulateMove(
			Pose3D initPose,
			Array<Vector3<float>> path,
			Pose3D &finalPose,
			float *V,
			float &t,
			int &iLastSegment,
			MOTION::Contact *pContact);
		void SceneBBox(
			SolidParams *pSolidParams,
			Box<float> *pBBox);
		void TestCorrection(
			std::vector<MOTION::DoorExperimentParams> &expData,
			std::vector<MOTION::TouchData> &touches);
        void LoadExperimentFileFormat(
            std::string experimentFileHeader,
            std::vector<std::string> &expDataFormat);
        void LoadArray(
            std::stringstream &ss,
            int n,
            float *array_);
        void LoadExperimentDataFromFile(
            std::string experimentData,
            std::vector<std::string> expDataFormat,
            MOTION::DoorExperimentParams *pSample);
        void LoadTouch(
            std::string touchFileData,
            std::vector<std::string> touchFileFormat,
            MOTION::TouchData *pTouch);
		void InitVisualizer(
			Visualizer *pVisualizerIn,
			char *cfgFileName);
		void SetVisualizeOptimization(bool bVisualizeOptimization);
		void PrintContact(MOTION::Contact *pContact);
		void PrintX(float *x);
		
		// Simundic
		MOTION::TouchModel getModelX()
		{
			return model_x;
		}
		MOTION::TouchModel getModelGT()
		{
			return model_gt;
		}
        void TestCorrection3(MOTION::DoorExperimentParams *pExpData, std::vector<MOTION::Contact>& contacts, std::vector<MOTION::TouchData> &touches);
		void Update_pose_D_A();
		void Update_pose_D_0(Pose3D &pose_E_0);
		void InitSession(RVL::MOTION::DoorExperimentParams *pExpData, bool useGT = false);
		void InitScene(RVL::MOTION::DoorExperimentParams *pExpData);
		void TransformModelVertices(MOTION::TouchModel *pModel, Pose3D *pose);
		void loadTransfMatrixFromNPY(std::string fileName, Pose3D &pose);

	private:
		void Constants();
		void AuxParams(
			MOTION::TouchModel *pModel0,
			MOTION::TouchModel *pModelx,
			float gz,
			float hz,
			float *a,
			float &b,
			float *a_,
			float &b_,
			float *D,
			float *invD,
			float *M,
			float *V);
		void VisionVertex(
			float *PSrc,
			float *a,
			float b,
			float *invD,
			Pose3D pose_C_E,
			Pose3D pose_C_E_e,
			float *PTgt,
			float *P_E);
		void VisionPlane(
			MOTION::Plane *pPlaneSrc,
			float *a_,
			float b_,
			float *D,
			Pose3D *pPose_C_E,
			Pose3D *pPose_C_E_e,
			MOTION::Plane *pPlaneTgt);
		void TestPlaneMapping(MOTION::TouchModel *pModel_x);
		void TestVertexPlaneConsistency();

	public:
		CRVLMem *pMem0;
		CRVLParameterList paramList;
		char *resultsFolder;
		float kappa;
		float zn;
		Camera camera;
		float alpha;
		float beta;
		int nSamples;
		int maxnAttempts;
		int maxnContactCombinations;
		float contactAngleThr;
		float maxOrientErrDeg;
		float maxReconstructionError; // m
		bool bRefPtConstraints;
		float toolTiltDeg;
		int simulationSeed;
		int nSimulationTouches;
        DWORD simulation;
		bool bDoor;
		float contactIntersectionThr;
		Pose3D pose_tool_E;
		bool bVisualization;
		int sessionSize;
		int nPanels;
		DWORD optimizationMethod;
		int nBest;
		MOTION::TouchTarget target;
		int *targetMem;

		// Simundic
		Pose3D pose_A_E;
		float x_[RVLMOTION_TOUCH_NUM_PARAMS];
    	float xOpt[RVLMOTION_TOUCH_NUM_PARAMS];

		Solid envSolidGT;
		Box<float> bbox_;
		Solid envSolid_0;
		RVL::MOTION::TouchEnvModel *envSolidParams_;
		float t_DD_A_x[3];
		float t_DD_0_x[3];
		Solid envSolidxPrev;
		Mesh *pToolMesh = NULL;
		MOTION::TouchModel model_gt_E;
		Pose3D pose_G_Ek;
		std::vector<vtkSmartPointer<vtkActor>> gtActors, xActors, xActors2;
		Pose3D pose_D_Arot_x, pose_D_0_x;
	    Pose3D pose_D_E_x;
		vtkSmartPointer<vtkActor> actor_D_E;
	
		private:
		MOTION::DisplayCallbackData *pVisualizationData;
		Array<RECOG::VN_::ModelCluster *> VNMClusters;
		Array<int> rndVal;
		float stdgxyk;
		float stdgzk;
		float stdhxyk;
		float stdhzk;
		float stdphi;
		float stds;
		float stdc;
		float stdl[4];
		float stdgz;
		float stdhz;
		float stdphirad;
		float varx[RVLMOTION_TOUCH_NUM_PARAMS];
		float csContactAngleThr;
		int iRndVal;
		Pose3D pose_Arot_A;
		Pose3D pose_C_W;
		Pose3D pose_E_W;
		Pose3D pose_W_E;
		MOTION::TouchModel model_gt;
		MOTION::TouchModel model_e;
		MOTION::TouchModel model_x;
		float xInc[RVLMOTION_TOUCH_NUM_PARAMS];
		float *SDFBuff;
		Array<MOTION::PlanarSurface> surfaces;
		uchar *surfaceMem;
		Array<MOTION::Vertex> vertices;
		Pair<int, int> *vertexMem;
		MOTION::Tool tool;
		Solid toolMoved;
		Pose3D toolPose;
		Pose3D doorPose;
		Solid envSolid;
		Solid envSolidx;
		int *solidVNAssoc;
		Solid envSolid_E;
		Array<MOTION::TouchData> refPts;
		int doorRefSurfaceIdx;
		Array<MOTION::TouchSolution> bestSolutions;
		int *contactMem;
		std::vector<int> contactBoundaryPlanes;
		Array<MOTION::TouchEnvModel> scenes;
		int maxnContacts;
		float snMaxOrientErr;
		int *GTContacts;
		bool bDebug;
		std::vector<MOTION::Plane> contactToolBoundaryPlanes;
	};
}
