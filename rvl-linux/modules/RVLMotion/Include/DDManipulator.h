#pragma once

#define RVLMOTION_TCP(pPose_G, half_tool_finger_distance, TCP)\
{\
	TCP[0] = pPose_G->t[0] - half_tool_finger_distance * pPose_G->R[0];\
	TCP[1] = pPose_G->t[1] - half_tool_finger_distance * pPose_G->R[3];\
	TCP[2] = pPose_G->t[2] - half_tool_finger_distance * pPose_G->R[6];\
 }

// The following code should be moved to RVL3DTools.h.

#define RVLRNDUNIT3VECTOR(X, fTmp) \
{\
	RVLSET3VECTOR(X, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f, 2.0f * (float)rand() / (float)RAND_MAX - 1.0f)\
	RVLNORM3(X, fTmp)\
}

// 

namespace RVL
{
	// The following code should be moved to RVL3DTools.h.

	template <typename T>
	void BoundingBoxOfBoxes(
		Box<T>* pBoxSrc1,
		Box<T>* pBoxSrc2,
		Box<T>* pBoxTgt)
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
			Visualizer* pVisualizer;
			bool bOwnVisualizer;
			bool bVNEnv;
			std::vector<vtkSmartPointer<vtkActor>> toolActors;
		};

		struct Sphere
		{
			Vector3<float> c;
			float r;
		};
	}

	class DDManipulator
	{
	public:
		DDManipulator();
		virtual ~DDManipulator();
		void Create(char* cfgFileNameIn);
		void CreateParamList();
		void Clear();
		void SetEnvironmentState(float state);
		bool Free(
			Pose3D *pPose_G_S,
			float *SDF);
		bool Free(
			Pose3D* pPose_G_S_start,
			Pose3D* pPose_G_S_end);
		bool LocalConstraints(
			Pose3D* pPose_G_S,
			float* SDF,
			Array<Pair<int, int>>& localConstraints,
			Vector3<float>* c_S_rot,
			Vector3<float>* c_S);
		float Cost(
			Pose3D* pPose_G_S_start,
			Pose3D* pPose_G_S_end);
		bool FreePose(
			Pose3D* pPose_G_S_init,
			Array<Pair<int, int>> localConstraints,
			Vector3<float>* c_S_rot,
			Vector3<float>* c_S,
			Pose3D* pPose_G_S);
		void Path(Pose3D * pPose_G_S_init);
		void LoadFeasibleToolContactPoses(std::string contactPointsFileName);
		void InitVisualizer(Visualizer* pVisualizerIn);
		void Visualize(
			std::vector<MOTION::Node> *pNodes,
			std::vector<int> *pPath,
			int iGoal = -1);
		void VisualizeTool(
			Pose3D pose_G_S,
			std::vector<vtkSmartPointer<vtkActor>> *pActors);
		// void DDManipulator::VisualizeTool(
		// 	Pose3D pose_G_S,
		// 	bool useDefaultGripper,
		// 	std::string gripperModelFileName,
		// 	std::vector<vtkSmartPointer<vtkActor>> *pActors);

		void SetVisualizeVNEnvironmentModel();

	public:
		CRVLMem* pMem;
		CRVLMem* pMem0;
		CRVLParameterList paramList;

		// Door model parameters.

		float dd_contact_surface_params[2];
		float dd_panel_params[3];
		float dd_moving_to_static_part_distance;
		float dd_static_side_width;
		float dd_static_depth;
		float dd_axis_distance;
		float dd_contact_surface_sampling_resolution;
		float dd_state_angle;

		// Door model constants.

		Pose3D pose_DD_A;
		Pose3D pose_A_W;

		// Door pose.

		Pose3D pose_W_S;
		Pose3D pose_Arot_A;

		// Environment VN model.

		VN* pVNEnv;
		float* dVNEnv;

		// Tool model.

		Vector3<float> tool_contact_surface_params[2];
		Vector3<float> tool_finger_size;
		float tool_finger_distance;
		Vector3<float> tool_palm_size;
		Array<MOTION::Sphere> tool_sample_spheres;
		bool useDefaultGripper;
		char *gripperModelFileName;
		// Path planning.

		int maxnSE3Points;

		// Feasible tool contact poses.

		Array<Pose3D> feasibleTCPs;

		// Local constraints.

		float rLocalConstraints;

	private:
		MOTION::DisplayCallbackData* pVisualizationData;
		Array<RECOG::VN_::ModelCluster *> VNMClusters;
		Box<float> dd_panel_box;
		Box<float> dd_static_box;
		Box<float> dd_storage_space_box;
		float half_tool_finger_distance;
		int* nodeBuffMem;
		int nodeBuffMemCapacity;
		Solver solver;
	};
}

