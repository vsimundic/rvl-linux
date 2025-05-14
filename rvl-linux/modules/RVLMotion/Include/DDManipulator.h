#pragma once
#undef PI // FCL gives an error
#include "fcl/fcl.h"
constexpr double PI = 3.14159265358979;
// The following code should be moved to RVL3DTools.h.

#define RVLRNDUNIT3VECTOR(X, fTmp)                                                                                                                                    \
	{                                                                                                                                                                 \
		RVLSET3VECTOR(X, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f) \
		RVLNORM3(X, fTmp)                                                                                                                                             \
	}

// The following code should be moved to Util.h.

#define RVLNORMANGLE(x) x = (x > PI ? x -= (2.0 * PI) : (x <= -PI ? x += (2.0 * PI) : x))

//

#define RVLMOTION_JOINT_SPACE_CHEB_DIST(q, q_, dq, dist, dist_, i) \
	{                                                              \
		RVLDIFVECTORS(q, q_, 6, dq, i);                            \
		dist = 0.0;                                                \
		for (i = 0; i < 6; i++)                                    \
		{                                                          \
			RVLNORMANGLE(dq[i]);                                   \
			dist_ = RVLABS(dq[i]);                                 \
			if (dist_ > dist)                                      \
				dist = dist_;                                      \
		}                                                          \
	}

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
		struct Sphere
		{
			Vector3<float> c;
			float r;
		};

		struct Cylinder
		{
			float r;
			Vector3<float> P[2];
		};

		struct ContactPose
		{
			Pose3D pose_G_DD;
			float PRTCP_DD[3];
		};

		struct IKSolution
		{
			int i;
			float q[6];
		};

		struct NodeJS
		{
			int iContactNode;
			int iState;
			IKSolution IK;
			bool bFeasible;
			float cost;
			int *path;
			int iPrevNode;
			NodeJS *pNext;
		};

		struct NodeSpaceElement
		{
			QList<MOTION::NodeJS> nodesJS;
			bool bExplored;
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
			void FwdKinematics(
				float *qIn,
				Pose3D *pPose_G_0);
			bool InvKinematics(
				Pose3D toolPose,
				float *qOut = NULL);
			bool InvKinematics1E56(
				Pose3D toolPose,
				float *qOut = NULL);
			void InvKinematicsApprox23(float *qOut = NULL);
			bool SelfCollision(
				float *q,
				float *t_3_1);
			bool InvKinematics(
				Pose3D toolPose,
				Array<IKSolution> &solutions,
				bool bTCP = false);
			bool InvKinematicsPrev(
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
			float epsilon;
			MOTION::Cylinder *collisionCylinderMem;
			Array<Array<MOTION::Cylinder>> collisionCylinders;
			int maxCollisionLinkIdx;

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
			float epsilonRad;
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
		bool Free(float *q);
		bool Free(
			Pose3D *pPose_G_S_start,
			Pose3D *pPose_G_S_end);
		void Free(Array<MOTION::IKSolution> &IKSolutions);
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
			MOTION::NodeJS *nodesJS,
			Array<int> &IKSolutionIdxs,
			bool bApproach = false,
			Array<MOTION::IKSolution> *pAapproachPathJS = NULL,
			int *pnViaPts = NULL);
		void Path(Pose3D *pPose_G_S_init);
		bool Path2(
			float *qInit,
			float endDoorState,
			int nStates,
			Array<Pose3D> &poses_G_0,
			Array2D<float> &robotJoints,
			Array<Array<Pose3D>> *pFeasiblePaths = NULL,
			Array<Array2D<float>> *pFeasiblePathsJoints = NULL);
		bool Path3(
			float *qInit,
			float endDoorState,
			int nStates,
			Array<Pose3D> &poses_G_0,
			Array2D<float> &robotJoints,
			Array<Array<Pose3D>> *pFeasiblePaths = NULL,
			Array<Array2D<float>> *pFeasiblePathsJoints = NULL);
		void CreateContactPoseGraph(std::string contactPoseGraphFileName);
		void CreateContactPoseGraph2(std::string contactPoseGraphFileName);
		void TileFeasibleToolContactPoses(
			std::vector<MOTION::ContactPose> *pAllFeasibleTCPs,
			float *max_dd_size,
			Box<float> &TCPSpace);
		bool ApproachPath(
			Pose3D *pPose_G_S_contact,
			Array<Pose3D> &poses_G_0_via,
			float *SDF,
			Array<MOTION::IKSolution> *IKSolutions,
			Array<Pair<int, int>> &paths);
		void SetDoorModelParams(
			float sx,
			float sy,
			float sz,
			float rx,
			float ry,
			float opening_direction,
			float static_side_width,
			float moving_to_static_part_distance);
		void UpdateFurnitureParams();
		void SetDoorPose(Pose3D pose_A_S);
		void UpdateFurniturePose();
		void UpdateStaticOrientation();
		void AdaptContactPoseGraph();
		void FreeSpacePlanes();
		void Neighbors(
			int iNodeC,
			MOTION::NodeSpaceElement *nodeSpace,
			Array<MOTION::NodeJS *> &neighbors);
		bool LoadFeasibleToolContactPoses(std::string contactPosesFileName);
		bool LoadContactPoseGraph(std::string contactPoseGraphFileName);
		void LoadToolModel(std::string toolModelDir);
		void LoadExample(std::string example);
		void LoadExampleIndexed(std::string example);
		void InitVisualizer(Visualizer *pVisualizerIn);
		void Visualize(
			std::vector<MOTION::Node> *pNodes,
			std::vector<int> *pPath,
			Array<float> doorStates,
			bool bVisualizeToolBoundingSphere = false,
			bool bVisualizeStates = false,
			bool bVisualizeMotionPlanningTree = false,
			int iGoal = -1,
			Array2D<float> *pRobotJoints = NULL);
		void VisualizeCurrentState(float *q, Pose3D toolPose);
		void VisualizeTool(
			Pose3D pose_G_R,
			std::vector<vtkSmartPointer<vtkActor>> *pActors,
			bool bToolMesh = true,
			Array<int> *pSpheres = NULL);
		void VisualizeRobot(
			float *q,
			std::vector<vtkSmartPointer<vtkActor>> *pActors);
		vtkSmartPointer<vtkActor> VisualizeDoorPenel();
		void SetVisualizeVNEnvironmentModel();

		// FCL
		void RVLPose2FCLPose(Pose3D poseRVL, fcl::Transform3<double>& poseFCL);
		void LoadToolModelFCL();
		void LoadCabinetStaticFCL(std::string cabinetStaticFilename_, Pose3D pose_A_S);
		void LoadCabinetPanelFCL(std::string cabinetPanelFilename_);
		void CreateRobotCylindersFCL();
		void CreateGndFCL();
		void GetVerticesFromPolyData(
			vtkSmartPointer<vtkPolyData> &vtkPolyData, 
			std::vector<fcl::Vector3<double>> &vertices);
		void GetTrianglesFromPolyData(
			vtkSmartPointer<vtkPolyData> &vtkPolyData, 
			std::vector<fcl::Triangle> &triangles);
		bool FreeFCL(Pose3D *pPose_G_S);
		bool FreeFCL(Pose3D *pPose_G_S_start, Pose3D *pPose_G_S_end);
		bool CylinderIntersectionFCL(double r, double h, Pose3D pose_C_S);
		bool CylinderIntersectionFCL(std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> fclCylinderMesh, Pose3D pose_C_S);
		void VisualizeCabinetStaticMesh(Pose3D pose_A_S, vtkSmartPointer<vtkActor> &actor);
		void VisualizeCabinetWholeMesh(Pose3D pose_A_S, vtkSmartPointer<vtkActor> &actor);
		void VisualizeToolMesh(Pose3D pose_G_S, vtkSmartPointer<vtkActor> &actor);
		void VisualizeDoorPanelMesh(vtkSmartPointer<vtkActor> &actor);
		void setPose_DD_0();
		bool ApproachPathPoses(
			Pose3D *pPose_G_S_contact,
			Array<Pose3D> &poses_G_0_via,
			float *SDF);
	private:
		void SetDoorReferenceFrames();
		void UpdateVNClusterOrientations();

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
		VN *pVNPanel;
		float *dVNPanel;

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


		// debug vars
		bool bCountPoses;
		int numExploredPoses_debug;
		std::vector<int> vecnumExploredPoses_debug;
		std::vector<int> vecFeasiblePoses_debug;
		std::vector<int> vecSelectedNodes_debug;
		std::vector<int> vecNumColchecks_Freeq, vecNumColchecks_FreeSDF;
		int numColchecks_Freeq, numColchecks_FreeSDF;

		// Approach path collision check
		bool bApproachPathCollisionCheck;
		bool bVisualizeApproachCollision;
		int last_approach_error_code;

		enum ApproachPathError {
			APPROACH_SUCCESS = 0,
			APPROACH_NO_IK_FOR_CONTACT = 1,
			APPROACH_INVALID_SPHERE_POSE = 2,
			APPROACH_COLLISION_CONTACT_VIA = 3,
			APPROACH_NO_IK_FOR_VIA1 = 4,
			APPROACH_NO_IK_FOR_VIA0 = 5,
			APPROACH_NO_VALID_PATH = 6,
			APPROACH_CYLINDER_COLLISION = 7,
			APPROACH_CONTACT_SPHERE_COLLISION = 8
		};

		// FCL
		bool use_fcl = false;
		std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> fclToolMesh;
		std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> fclCabinetStaticMesh, fclCabinetPanelMesh, fclGndMesh;
		std::shared_ptr<fcl::CollisionObject<double>> collisionCabinetObj, collisionGndObj;
		fcl::CollisionRequest<double> requestFCL;
	    fcl::Transform3d TArot_S;
		Pose3D pose_A_S_FCL, poseArot_A_FCL, poseArot_S_FCL;
		float csFCL;
		float snFCL;
		std::vector<std::vector<std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>>> fclRobotCylinderMeshes;
		// fcl::CollisionObjectd* collisionCabinetObj;
		std::string cabinetStaticFilename, cabinetPanelFilename;
		Mesh *pCabinetMeshStatic, *pCabinetMeshPanel;
		Mesh *pCabinetWholeMesh;
		// Pose3D pose_A_S;
		char *cabinetStaticDirPath;
		fcl::Vector3d contact_pos;
	
		// Path planning.

		int maxnSE3Points;
		float kTemplateEndTol;
		float maxSurfaceContactAngle; // deg
		float visionTol;			  // m
		float minDistanceToAxis;	  // m
		bool bLock_T_G_DD;
		float posCostMaxDist;
		float wPos;
		int maxnIKSolutions;
		float JSDistThr;
		int maxnNodesJSPerStep;

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
		RVL::MOTION::DisplayCallbackData *pVisualizationData;
		Array<RECOG::VN_::ModelCluster *> VNMClusters;
		RECOG::VN_::ModelCluster *pPanelVNMCluster;
		Box<float> dd_panel_box;
		Box<float> dd_static_box;
		Box<float> dd_storage_space_box;
		float half_tool_finger_distance;
		int *nodeBuffMem;
		int nodeBuffMemCapacity;
		Solver solver;
		float csMaxSurfaceContactAngle;
		RVL::MOTION::Plane freeSpacePlanes_S[4];
		Pose3D pose_DD_0;
		float default_tool_P1_G[3];
		float default_tool_P2_G[3];
		Array<int> rndIdx;
		Pose3D *pathPosesMem;
		Array<Pose3D> *pathMem;
		float *pathJointsMem;
		Array2D<float> *pathMemJoints;
		Array<MOTION::IKSolution> approachIKSolutions[3];
		MOTION::IKSolution *approachIKSolutionsMem;
		Pair<int, int> *approachPathMem;
		Array<int> selectedNodes;
		bool *bSelected;
		int *contactNode;
		int maxnContactPoseGraphNeighbors;
		bool bPath3;
	};
}
