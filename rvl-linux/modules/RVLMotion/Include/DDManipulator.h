#pragma once

// The following code should be moved to RVL3DTools.h.

#define RVLRNDUNIT3VECTOR(X, fTmp)                                                                                                                                    \
	{                                                                                                                                                                 \
		RVLSET3VECTOR(X, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f) \
			RVLNORM3(X, fTmp)}

//

namespace RVL
{
	// The following code should be moved to RVL3DTools.h.

	template <typename T>
	void BoundingBoxOfBoxes(
		Box<T> *pBoxSrc1,
		Box<T> *pBoxSrc2,
		Box<T> *pBoxTgt)
	{
		pBoxTgt->minx = RVLMIN(pBoxSrc1->minx, pBoxSrc2->minx);
		pBoxTgt->maxx = RVLMAX(pBoxSrc1->maxx, pBoxSrc2->maxx);
		pBoxTgt->miny = RVLMIN(pBoxSrc1->miny, pBoxSrc2->miny);
		pBoxTgt->maxy = RVLMAX(pBoxSrc1->maxy, pBoxSrc2->maxy);
		pBoxTgt->minz = RVLMIN(pBoxSrc1->minz, pBoxSrc2->minz);
		pBoxTgt->maxz = RVLMAX(pBoxSrc1->maxz, pBoxSrc2->maxz);
	}

	//

	namespace MOTION
	{
		struct DisplayCallbackData
		{
			Visualizer *pVisualizer;
			bool bOwnVisualizer;
			CRVLParameterList paramList;
			bool bVNEnv;
			std::vector<vtkSmartPointer<vtkActor>> toolActors;
			bool bVisualize;
		};

		struct Sphere
		{
			Vector3<float> c;
			float r;
		};

		struct Plane
		{
			float N[3];
			float d;
		};

		struct ContactPose
		{
			Pose3D pose_G_DD;
			float PRTCP_DD[3];
		};

		class Robot
		{
		public:
			Robot();
			virtual ~Robot();
			void Create(char *cfgFileNameIn);
			void CreateParamList();
			void Clear();
			void FwdKinematics();
			void FwdKinematics(
				int i,
				Pose3D *pdPose);
			void FwdKinematicsRot(
				int i,
				float *R,
				float &cq,
				float &sq);
			bool InvKinematics(
				Pose3D toolPose,
				float *qOut = NULL);
			bool InvKinematics1E56(
				Pose3D toolPose,
				float *qOut = NULL);
			void InvKinematicsApprox23(float *qOut = NULL);
			bool InvKinematics(
				Pose3D pose_6_0,
				Array2D<float> &qOut);

		public:
			CRVLMem *pMem0;
			CRVLParameterList paramList;
			int n;
			float *q;
			float *d;
			float *a;
			float *al;
			uchar *jointType; // 0 - revolute; 1 - prismatic
			float maxr;
			float minr;
			float minz;
			Pose3D pose_TCP_6;
			float rotz_TCP_6;
			Pose3D link_pose[6];
			Pose3D pose_0_W;
			float minq[6], maxq[6];

		private:
			float *csal;
			float *snal;
			Pose3D pose_6_G;
			float d4_2;
			float k1;
			float k2;
			float maxa23_2;
			float R_E_1[9];
			float R_4_E[9];
			float a23_2;
		};
	}

	class DDManipulator
	{
	public:
		DDManipulator();
		virtual ~DDManipulator();
		void Create(char *cfgFileNameIn);
		void CreateParamList();
		void Clear();
		void SetEnvironmentState(float state);
		bool Free(
			Pose3D *pPose_G_S,
			float *SDF);
		bool Free(
			Pose3D *pPose_G_S_start,
			Pose3D *pPose_G_S_end);
		bool LocalConstraints(
			Pose3D *pPose_G_S,
			float *SDF,
			Array<Pair<int, int>> &localConstraints,
			Vector3<float> *c_S_rot,
			Vector3<float> *c_S);
		float Cost(
			Pose3D *pPose_G_S_start,
			Pose3D *pPose_G_S_end);
		bool FreePose(
			Pose3D *pPose_G_S_init,
			Array<Pair<int, int>> localConstraints,
			Vector3<float> *c_S_rot,
			Vector3<float> *c_S,
			Pose3D *pPose_G_S);
		bool FeasiblePose(
			Pose3D *pPose_G_0,
			float *SDF,
			float *q,
			bool bApproach = false);
		void Path(Pose3D *pPose_G_S_init);
		bool Path2(
			float *qInit,
			float endDoorState,
			int nStates,
			Array<Pose3D> &poses_G_0,
			Array2D<float> &robotJoints,
			Array<Array<Pose3D>> *pFeasiblePaths = NULL,
			Array<Array2D<float>> *pFeasiblePathsJoints = NULL);
		void CreateContactPoseGraph(std::string contactPoseGraphFileName);
		void TileFeasibleToolContactPoses(
			std::vector<MOTION::ContactPose> *pAllFeasibleTCPs,
			float *max_dd_size,
			Box<float> &TCPSpace);
		bool ApproachPath(
			Pose3D *pPose_G_S_contact,
			Array<Pose3D> &poses_G_0_via,
			float *SDF);
		void SetDoorModelParams(
			float sx,
			float sy,
			float sz,
			float rx,
			float ry,
			float opening_direction,
			float static_side_width,
			float moving_to_static_part_distance);
		void UpdateStaticParams();
		void SetDoorPose(Pose3D pose_A_S);
		void UpdateStaticPose();
		bool LoadFeasibleToolContactPoses(std::string contactPosesFileName);
		bool LoadContactPoseGraph(std::string contactPoseGraphFileName);
		void LoadToolModel(std::string toolModelDir);
		void InitVisualizer(Visualizer *pVisualizerIn);
		void Visualize(
			std::vector<MOTION::Node> *pNodes,
			std::vector<int> *pPath,
			Array<float> doorStates,
			bool bVisualizeStates = false,
			bool bVisualizeMotionPlanningTree = false,
			int iGoal = -1,
			Array2D<float> *pRobotJoints = NULL);
		void VisualizeTool(
			Pose3D pose_G_R,
			std::vector<vtkSmartPointer<vtkActor>> *pActors,
			bool bToolMesh = true,
			Array<int> *pSpheres = NULL);
		vtkSmartPointer<vtkActor> VisualizeRobot(float *q);
		vtkSmartPointer<vtkActor> VisualizeDoorPenel();
		void SetVisualizeVNEnvironmentModel();

	private:
		void UpdateDoorReferenceFrames();

	public:
		CRVLMem *pMem;
		CRVLMem *pMem0;
		CRVLParameterList paramList;
		std::string cfgFileName;
		char *resultsFolder;
		bool bLog;
		CRVLTimer *pTimer;

		// Door model parameters.

		float dd_contact_surface_params[2];
		float dd_panel_params[3];
		float dd_moving_to_static_part_distance;
		float dd_static_side_width;
		float dd_static_depth;
		float dd_sx;
		float dd_sy;
		float dd_sz;
		float dd_rx;
		float dd_ry;
		float dd_contact_surface_sampling_resolution;
		float dd_state_angle;
		float dd_opening_direction;
		bool bVNPanel; // If this variable is true, then the door panel is included in the VN model of the environment.

		// Door model constants.

		Pose3D pose_DD_A;
		Pose3D pose_A_F;

		// Door pose.

		Pose3D pose_F_S;
		Pose3D pose_Arot_A;
		Pose3D pose_DD_S;

		// Environment VN model.

		VN *pVNEnv;
		float *dVNEnv;

		// Robot.

		MOTION::Robot robot;

		// Tool model.

		bool bDefaultToolModel;
		Vector3<float> tool_contact_surface_params[3];
		Vector3<float> tool_finger_size;
		float tool_finger_distance;
		Vector3<float> tool_palm_size;
		float tool_wrist_len;
		float tool_wrist_r;
		float tool_len;
		MOTION::Sphere tool_bounding_sphere;
		Array<MOTION::Sphere> tool_sample_spheres;
		Mesh *pToolMesh;
		float PRTCP_G[3];
		Array<int> tool_contact_spheres;

		// Path planning.

		int maxnSE3Points;
		float kTemplateEndTol;
		float maxSurfaceContactAngle; // deg
		float visionTol;			  // m
		float minDistanceToAxis;	  // m
		bool bLock_T_G_DD;
		float posCostMaxDist;
		float wPos;

		// Feasible tool contact poses.

		Array<Pose3D> feasibleTCPs;

		// Local constraints.

		float rLocalConstraints;

	private:
		char *feasibleToolContactPosesFileName;
		char *contactPoseGraphFileName;
		char *toolModelDir;
		Array<MOTION::Node> nodes;
		Graph<GRAPH::Node_<GRAPH::EdgePtr<MOTION::Edge>>, MOTION::Edge, GRAPH::EdgePtr<MOTION::Edge>> graph;
		MOTION::DisplayCallbackData *pVisualizationData;
		Array<RECOG::VN_::ModelCluster *> VNMClusters;
		Box<float> dd_panel_box;
		Box<float> dd_static_box;
		Box<float> dd_storage_space_box;
		float half_tool_finger_distance;
		int *nodeBuffMem;
		int nodeBuffMemCapacity;
		Solver solver;
		float csMaxSurfaceContactAngle;
		MOTION::Plane freeSpacePlanes_S[4];
		Pose3D pose_DD_0;
		float default_tool_P1_G[3];
		float default_tool_P2_G[3];
		Array<int> rndIdx;
		Pose3D *pathPosesMem;
		Array<Pose3D> *pathMem;
		float *pathJointsMem;
		Array2D<float> *pathMemJoints;
	};
}
