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

		struct DisplayCallbackData
		{
			Visualizer *pVisualizer;
			bool bOwnVisualizer;
			CRVLParameterList paramList;
			bool bVNEnv;
			std::vector<vtkSmartPointer<vtkActor>> robotActors;
			bool bVisualize;
			Box<float> VNBBox;
		};

		void InitVisualizer(
			Visualizer *pVisualizerIn,
			MOTION::DisplayCallbackData *&pVisualizationData,
			CRVLMem *pMem);
	}
}
