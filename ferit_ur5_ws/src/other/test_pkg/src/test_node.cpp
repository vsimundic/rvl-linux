#include "ros/ros.h"

// RVL includes
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "SE3Grid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "RRT.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "PSGM.h"
#include "VN.h"
#include "DDManipulator.h"
// #include "cnpy.h"
// #include "vtkNew.h"


int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    CRVLMem mem0;	// permanent memory
    mem0.Create(1000000000);
    CRVLMem mem;	// cycle memory
    mem.Create(1000000000);


    RVL::MOTION::Robot UR5;
    UR5.pMem0 = &mem0;
    cout << "UR5 initialized" << endl;
    UR5.Create("/home/RVLuser/ferit_ur5_ws/RVL_cfg_file.cfg");
    cout << "UR5 created" << endl;
    RVL::Pose3D pose_6_0;
    pose_6_0.R[0] = -1;
    pose_6_0.R[5] = 1;
    pose_6_0.R[7] = 1;
    pose_6_0.t[0] = 0.39267777;
    pose_6_0.t[1] = 0.19130912;
    pose_6_0.t[2] = 0.41946068;
    // float Q[4] = {0.70709, -0.707124, 0.0, 0.0};
    // RVLQUATERNIONTOROT(Q, pose_6_0.R);
    RVL::Array2D<float> invKinSolutions;
    invKinSolutions.Element = NULL;
    UR5.InvKinematics(pose_6_0, invKinSolutions);
    float *q;

    for (int i = 0; i < invKinSolutions.h; i++) {
        q = invKinSolutions.Element + 6 * i;
        for (int j = 0; j < 6; j++) {
            std::cout << q[j] << ", ";
        }
        std::cout << std::endl;
    }
    
    float pi_ = 3.14159265359;
    q[0] += pi_;
    q[5] += pi_;

    for (int i = 0; i < 6; ++i) {
        if (q[i] > pi_) 
        {
            q[i] -= 2.0f * pi_;
        } 
        else if (q[i] < -pi_) 
        {
            q[i] += 2.0f * pi_;
        }
    }

    // ros::spin(); // Keep the node running
    return 0;
}