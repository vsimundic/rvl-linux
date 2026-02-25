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

void MOTION::LoadDoorExample(
    std::string example,
    float &sy,
    float &sz,
    float *t,
    float &al,
    float &q)
{
    std::stringstream ss(example);
    std::string value;
    std::getline(ss, value, ',');
    sy = std::stof(value);
    std::getline(ss, value, ',');
    sz = std::stof(value);
    for (int i = 0; i < 3; i++)
    {
        std::getline(ss, value, ',');
        t[i] = std::stof(value);
    }
    std::getline(ss, value, ',');
    float rotz_A_S_deg = std::stof(value);
    al = DEG2RAD * rotz_A_S_deg;
    std::getline(ss, value, ',');
    float dd_state_angle_deg = std::stof(value);
    q = DEG2RAD * dd_state_angle_deg;
}

void MOTION::InitVisualizer(
    Visualizer *pVisualizerIn,
    MOTION::DisplayCallbackData *&pVisualizationData,
    CRVLMem *pMem)
{
    if (pVisualizationData == NULL)
        pVisualizationData = new MOTION::DisplayCallbackData;
    assert(pVisualizationData->selectedPoints.size() == 0);

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
    pVisualizationData->bVNEnv = false;
    pVisualizationData->bOptimization = false;
}
