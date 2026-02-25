#pragma once

#define RVLRECONSTRUCTION_MODEL_TYPE_BOX		1
#define RVLRECONSTRUCTION_MODEL_TYPE_CYLINDER	2

namespace RVL
{
	namespace RECONSTRUCT
	{
		struct Model
		{
			int type;
			float param1;
			float param2;
			float param3;
		};

		struct GTData
		{
			int iModel;
			Pose3D pose;
		};
	}

	class ReconstructionEval
	{
	public:
		ReconstructionEval();
		virtual ~ReconstructionEval();
		void CreateGTGrid(int iScene);
		void LoadGT(char *GTFileName);
		void DisplayGT(
			Visualizer *pVisualizer,
			int iScene);
		void DisplayGrid(
			Visualizer *pVisualizer,
			Array3D<uchar> grid_,
			uchar *color);
		void DisplayGridComparison(
			Visualizer *pVisualizer,
			Array3D<uchar> grid,
			Array3D<uchar> gridGT);

	public:
		std::map<int, std::vector<RECONSTRUCT::GTData>> scenes;
		std::map<int, RECONSTRUCT::Model> models;
		Pose3D cameraPose;
		bool bGT;
		float resolution;
		Box<int> sceneBBox;
		Array3D<uchar> grid;
	};
}

