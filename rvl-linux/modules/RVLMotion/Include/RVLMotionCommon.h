#pragma once
namespace RVL
{
	namespace MOTION
	{
		struct Plane
		{
			float N[3];
			float d;
		};

		struct VisualizationNode
		{
			float P[3];
			int iState;
			int iNodeJS;
		};

		struct DisplayCallbackData
		{
			Visualizer *pVisualizer;
			bool bOwnVisualizer;
			CRVLParameterList paramList;
			bool bVNEnv;
			std::vector<vtkSmartPointer<vtkActor>> robotActors;
			std::vector<vtkSmartPointer<vtkActor>> envActors;
			std::vector<vtkSmartPointer<vtkActor>> errorActors;
			std::vector<vtkSmartPointer<vtkActor>> envActors2;
			bool bVisualize;
			bool bOptimization;
			bool bContacts;
			Box<float> VNBBox;
			std::vector<MOTION::VisualizationNode> visNodes;
			std::vector<Pair<int, int>> visEdges;
		};

		void LoadDoorExample(
			std::string example,
			float &sy,
			float &sz,
			float *t,
			float &al,
			float &q);

		void InitVisualizer(
			Visualizer *pVisualizerIn,
			MOTION::DisplayCallbackData *&pVisualizationData,
			CRVLMem *pMem);
	}
}
