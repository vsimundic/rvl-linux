#pragma once
#include "Graph.h"
#include <memory>
#include "SceneSegFile.hpp"
#include "SVMClassifier.h"

//#define RVLPCSEGMENT_OBJECT_GRAPH_LOG

namespace RVL
{
	namespace SURFEL
	{
		struct AgEdge
		{
			int iVertex[2];
			GRAPH::EdgePtr2<AgEdge> *pVertexEdgePtr[2];
			int idx;
			SurfelAdjecencyDescriptors desc;
			float cost;
			AgEdge *pNext;
		};

		class ObjectGraph;

		struct ObjectDisplayData
		{
			Mesh *pMesh;
			SurfelGraph *pSurfels;
			ObjectGraph *pObjects;
			Visualizer *pVisualizer;
			unsigned char selectionColor[3];
			int iSelectedObject;
			bool bObjects;
		};

		struct ObjectEdgeData
		{
			float PContinuous;
			float PConvex;
			float PClean;
			float P;
		};

		bool objectKeyPressUserFunction(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			std::string &key,
			void *vpData);
		bool objectMouseRButtonDownUserFunction(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			int iSelectedPt,
			int iSelectedSurfel,
			void *vpData);

		//Filko
		//Definition of iterator type
		typedef std::map<int, bool>::iterator ObjectsSurfelConvexity_iterator_type;

		struct ObjectGraphObjectData
		{
			std::vector<std::vector<int>> CHVertexIndices;
			std::vector<std::map<int, bool>> ObjectsSurfelConvexity;
		};
		//

		class ObjectGraph :
			public Graph < GRAPH::AggregateNode<AgEdge>, AgEdge, GRAPH::EdgePtr2<AgEdge> >
		{
		public:
			ObjectGraph();
			virtual ~ObjectGraph();
			void CreateParamList(CRVLMem *pMem);
			void Create(SurfelGraph *pSurfels_);
			void CreateFromSSF(std::string ssfFileName);	//Filko
			void CalculateOverAndUnderSegmentation(int *E, int &N, bool useGTNoPix = true,  bool useBackground = true);	//Filko
			void DetermineObjectConvexityData(float convexThr = 0.005, float ratioThr = 0.9);	//Filko
			void WERSegmentation();
			void ComputeRelationCosts();
			void ComputeRelationCost(
				AgEdge *pEdge,
				ObjectEdgeData &data);
			void CreateSortedObjectArray();
			void SortElements(
				GRAPH::AggregateNode<AgEdge> *pAgNode,
				Array<SortIndex<int>> *pSortedElementIdxArray);
			void InitDisplay(
				Visualizer *pVisualizer,
				Mesh *pMesh,
				unsigned char *selectionColor);
			void Display();
			void PaintObject(
				int iObject,
				unsigned char *color);
			void WriteSurfelDataToFile(FILE *fp);
			void WriteObjectDataToFile(FILE *fp);
			void InitSVMClassifier(char *svmParamsFileName);	//Nyarko
			void Debug();

		public:
			CRVLParameterList ParamList;
			SurfelGraph *pSurfels;
			float WERSegmentationMinCostDiff;
			float WERSegmentationCostResolution;
			ObjectDisplayData displayData;
			int *objectMap;
			std::shared_ptr<SceneSegFile::SceneSegFile> ssf;	//Filko
			std::map<int, int> objID2idxMap; //Filko
			SVMClassifier *pSVMClassifier;  //Nyarko
			Array<int> objectArray;
			float kCoverage;
			float alpha;
			ObjectGraphObjectData additionalObjectData;	//Filko
			//Array<int> *sortedElementIdxArray;
		private:
			QLIST::Index *elementMem;
			//int *sortedElementIdxMem;
		};
	}
}

