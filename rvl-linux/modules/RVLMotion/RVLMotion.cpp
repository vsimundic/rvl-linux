#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "SE3Grid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "RVLMotionCommon.h"

using namespace RVL;
using namespace MOTION;

void MOTION::InitVisualizer(
    Visualizer *pVisualizerIn,
    MOTION::DisplayCallbackData *&pVisualizationData,
    CRVLMem *pMem)
{
    if (pVisualizationData == NULL)
        pVisualizationData = new MOTION::DisplayCallbackData;
    if (pVisualizerIn)
    {
        pVisualizationData->pVisualizer = pVisualizerIn;
        pVisualizationData->bOwnVisualizer = false;
    }
    else
    {
        pVisualizationData->pVisualizer = new Visualizer;
        pVisualizationData->bOwnVisualizer = true;
    }
    pVisualizationData->bVisualize = false;
}