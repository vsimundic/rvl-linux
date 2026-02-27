// #include "RVLCore2.h"
// #include "RVLVTK.h"
// #include "Util.h"
// #include "Space3DGrid.h"
// #include "SE3Grid.h"
// #include "Graph.h"
// #include "Mesh.h"
// #include "Visualizer.h"
// #include "SceneSegFile.hpp"
// #include "SurfelGraph.h"
// #include "ObjectGraph.h"
// #include "PlanarSurfelDetector.h"
// #include "RVLRecognition.h"
// #include "RVLRecognitionCommon.h"
// #include "PSGMCommon.h"
// #include "CTISet.h"
// #include "VertexGraph.h"
// #include "TG.h"
// #include "TGSet.h"
// #include "PSGM.h"
// #include "VN.h"
// #include "RVLMotionCommon.h"
// #include "Touch.h"

// #include <pybind11/pybind11.h>
// #include <pybind11/numpy.h>

#include "RVLPYTouch.h"


////////////////////////////////////////////////////////////////////
//
//     PYTouch
//
////////////////////////////////////////////////////////////////////

// class PYTouch
// {
// public:
//     PYTouch();
//     ~PYTouch();
//     void create(std::string cfgFileName);
//     void clear();
//     void create_scene(
//         float sx, float sy, float sz,
//         float rx, float ry, float a, float b, float c, float qDeg);
//     void create_simple_tool(
//         float a, float b, float c, float d, float h, float *t = NULL);

//     void correct_real_experiment(
//     py::array T_Ek_E,
//     py::array V,
//     py::array T_A_E,
//     py::array T_E_0);

//     Touch touch;
// 	int memSize;
// 	int mem0Size;
//     Visualizer visualizer;
//     Array<MOTION::TouchData> touches;
//     std::vector<MOTION::Contact> contacts;
// };

PYTouch::PYTouch()
{
	memSize = 1000000000;
	mem0Size = 1000000000;
}
PYTouch::~PYTouch()
{
    clear();
}

void PYTouch::clear()
{
    delete touch.pMem0;
    // Add when necessary
}

void PYTouch::create(std::string cfgFileName)
{
    touch.pMem0 = new CRVLMem;
    touch.pMem0->Create(mem0Size);

    char* cfgFileNameCStr = (char *)(cfgFileName.data());
    touch.Create(cfgFileNameCStr);

    visualizer.Create();
    touch.InitVisualizer(&visualizer, cfgFileNameCStr);
    touch.bDoor = true;
}

void PYTouch::create_scene(
    float sx, float sy, float sz,
    float rx, float ry, float a, float b, float c, float qDeg)
{
    touch.CreateScene(sx, sy, sz, rx, ry, a, b, c, qDeg);
}

void PYTouch::create_simple_tool(
    float a, float b, float c, float d, float h, float *t)
{
    touch.CreateSimpleTool(a, b, c, d, h, t);
}

void PYTouch::correct_real_experiment(
    py::array T_Ek_E,
    py::array V,
    py::array T_A_E,
    py::array T_E_0,
    py::array T_0_S)
{
    // Convert numpy arrays to RVL types
    Pose3D pose_Ek_E, pose_A_E, pose_E_0;
    double *T_Ek_E_ = (double *)T_Ek_E.request().ptr;
    RVLHTRANSFMXDECOMP(T_Ek_E_, pose_Ek_E.R, pose_Ek_E.t);
    double *V_ = (double *)V.request().ptr;
    float V_f[3] = { (float)V_[0], (float)V_[1], (float)V_[2] };

    double *T_A_E_ = (double *)T_A_E.request().ptr;
    RVLHTRANSFMXDECOMP(T_A_E_, pose_A_E.R, pose_A_E.t);
    double *T_E_0_ = (double *)T_E_0.request().ptr;
    RVLHTRANSFMXDECOMP(T_E_0_, pose_E_0.R, pose_E_0.t);

    double *T_0_S_ = (double *)T_0_S.request().ptr;
    Pose3D pose_0_S;
    RVLHTRANSFMXDECOMP(T_0_S_, pose_0_S.R, pose_0_S.t);

    touch.RealExpCorrect(
        touches,
        contacts,
        pose_Ek_E,
        V_f,
        pose_A_E,
        pose_E_0,
        pose_0_S
    );

}

void PYTouch::set_capturing_flange_pose(py::array T_E_0)
{
    double *T_E_0_ = (double *)T_E_0.request().ptr;
    RVLHTRANSFMXDECOMP(T_E_0_, pose_E_0.R, pose_E_0.t);
    // pose_E_0 = pose_E_0;
}

////////////////////////////////////////////////////////////////////
//
//     RVL PYTouch Wrapper
//
////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(RVLPYTouch, m)
{
    m.doc() = "RVL PYTouch wrapper";

    py::class_<PYTouch>(m, "PYTouch")
        .def(py::init<>())
        .def("create", &PYTouch::create)
        .def("clear", &PYTouch::clear)
        .def("create_scene", &PYTouch::create_scene)
        .def("create_simple_tool", &PYTouch::create_simple_tool)
        .def("correct_real_experiment", &PYTouch::correct_real_experiment)
        .def("set_capturing_flange_pose", &PYTouch::set_capturing_flange_pose);
}