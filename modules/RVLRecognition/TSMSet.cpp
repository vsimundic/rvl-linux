//#include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "TSM.h"
#include "TSMSet.h"

using namespace RVL;
using namespace RECOG;

TSMSet::TSMSet()
{
	vertexArray.Element = NULL;
	mVertexFlags = NULL;
	convexTemplate.Element = NULL;
	convexTemplate.w = 3;
}


TSMSet::~TSMSet()
{
	RVL_DELETE_ARRAY(vertexArray.Element);
	RVL_DELETE_ARRAY(mVertexFlags);
	RVL_DELETE_ARRAY(convexTemplate.Element);
}

void TSMSet::Init(SurfelGraph *pSurfels)
{
	vertexArray.Element = new int[pSurfels->vertexArray.n];
	mVertexFlags = new BYTE[pSurfels->vertexArray.n];
	memset(mVertexFlags, 0, pSurfels->vertexArray.n * sizeof(BYTE));
}

