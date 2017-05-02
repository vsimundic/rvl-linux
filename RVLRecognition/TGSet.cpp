//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "TG.h"
#include "TGSet.h"

using namespace RVL;
using namespace RECOG;

TGSet::TGSet()
{
	//vertexArray.Element = NULL;
	//mVertexFlags = NULL;
	//convexTemplate.Element = NULL;
	//convexTemplate.w = 3;
}


TGSet::~TGSet()
{
	//RVL_DELETE_ARRAY(vertexArray.Element);
	//RVL_DELETE_ARRAY(mVertexFlags);
	//RVL_DELETE_ARRAY(convexTemplate.Element);

}

void TGSet::Init(SurfelGraph *pSurfels)
{
	//vertexArray.Element = new int[pSurfels->vertexArray.n];
	//mVertexFlags = new BYTE[pSurfels->vertexArray.n];
	//memset(mVertexFlags, 0, pSurfels->vertexArray.n * sizeof(BYTE));
}
