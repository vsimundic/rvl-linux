#pragma once

namespace RVL
{
	class PSGM;

	namespace RECOG
	{
		namespace PSGM_
		{
			struct Vertex
			{
				float P[3];
				Array<int> iSurfelArray;
				Vertex *pNext;
			};

			struct DisplayData
			{
				PSGM *pRecognition;
				Mesh *pMesh;
				SurfelGraph *pSurfels;
				Visualizer *pVisualizer;
			};
		}
	}

	class PSGM
	{
	public:
		PSGM();
		virtual ~PSGM();
		void Interpret(
			Mesh *pMesh);
		void InitDisplay(
			Visualizer *pVisualizer,
			Mesh *pMesh);
		void Display();
		void DisplayModelInstance(Visualizer *pVisualizer);
		void DisplayVertices();

	public:
		CRVLMem *pMem;
		PlanarSurfelDetector *pSurfelDetector;
		SurfelGraph *pSurfels;
		QList<RECOG::PSGM_::Vertex> vertexList;
		int nVertices;
		Array<QList<QLIST::Index>> surfelVertexList;
		QLIST::Index *surfelVertexMem;
		RECOG::PSGM_::DisplayData displayData;
	};
}

