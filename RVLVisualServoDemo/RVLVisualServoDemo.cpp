// RVLVisualServoDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#define RVLPCL
#ifdef RVLPCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#include <pcl/io/boost.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/common/time.h>
#include <pcl/io/ply_io.h>
#include "PCLMeshBuilder.h"
#endif
#include "RVLCore2.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "VisualServo.h"
#include "RGBDCamera.h"
//#include "RVLGUI.h"

using namespace RVL;

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	// Create VisualServo object.

	VisualServo VS;

	VS.pMem0 = &mem0;

	VS.Init("RVLVisualServoDemo.cfg");

	//// Create GUI.

	//CRVLGUI GUI;

	//GUI.Init(&VS);

	// If Kinect is not available, display a message.

	if (!(VS.flags & RVLVISUALSERVO_FLAG_KINECT))
		return 1;
	//	GUI.Message("Kinect is not available.", 400, 100, cvScalar(0, 128, 255));

	RGBDCamera camera;

#ifdef RVLPCL
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(VS.depthImage.w, VS.depthImage.h));

	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	PCLMeshBuilder meshBuilder;

	pcl::PolygonMesh PCLMesh;
#endif

	bool bContinuous = true;

	bool bContinue = true;

	int key;
#ifdef RVLOPENNI
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef RGBFrame;
#endif

	do
	{
#ifdef RVLOPENNI
		if (VS.flags & RVLVISUALSERVO_FLAG_KINECT)
		{
			// acquire depth image from Kinect

			VS.kinect.GetImages(VS.depthImage.Element, VS.RGBImage, NULL, VS.GSImage, RVLKINECT_DEPTH_IMAGE_FORMAT_1MM, 0, &depthFrame, &RGBFrame);
		}
		//else

		DisplayDisparityMap(VS.depthImage, (unsigned char *)(VS.depthDisplayImage->imageData), false, RVLRGB_DEPTH_FORMAT_1MM);

		cvShowImage("depth image", VS.depthDisplayImage);

		//cvShowImage("RGB image", VS.RGBImage);

		key = (bContinuous ? cvWaitKey(1) : cvWaitKey());

		switch (key)
		{
			case 'g':
#ifdef RVLPCL
				camera.GetPointCloud(&(VS.depthImage), VS.RGBImage, PC);

				//viewer.showCloud(PC);

				//pcl::io::savePCDFile("PC.pcd", *PC);

				meshBuilder.CreateMesh(PC, PCLMesh);

				viewer->addPolygonMesh(PCLMesh, "mesh", 0);
				//viewer->addPointCloud<pcl::PointXYZRGBA>(FPC, rgb, "points");
				//viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(FPC, N, 10, 0.01, "normals");
				//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points");
				//viewer->addCoordinateSystem(1.0);
				viewer->initCameraParameters();
				viewer->setRepresentationToWireframeForAllActors();
				while (!viewer->wasStopped())
				{
					viewer->spinOnce(100, true);
					boost::this_thread::sleep(boost::posix_time::microseconds(100000));
				}

				pcl::io::savePLYFile("mesh.ply", PCLMesh);
#endif

				break;
			case 0x0000001B:	// Esc
				bContinue = false;
				
				break;
		}
#endif
			// import depth image
	} while (bContinue);

	return 0;
}

//#ifdef RVLPCL
//class OpenNIProcessor
//{
//public:
//	//OpenNIProcessor() : viewer("PCL OpenNI Viewer") {}
//	OpenNIProcessor(){}
//
//	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
//	{
//		//if (!viewer.wasStopped())
//		//	viewer.showCloud(cloud);
//
//		static unsigned count = 0;
//		static double last = pcl::getTime();
//		if (++count == 30)
//		{
//			double now = pcl::getTime();
//			std::cout << "distance of center pixel :" << cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count) / double(now - last) << " Hz" << std::endl;
//			count = 0;
//			last = now;
//		}
//	}
//
//	void run()
//	{
//		pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
//
//		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
//			boost::bind(&OpenNIProcessor::cloud_cb_, this, _1);
//
//		boost::signals2::connection c = interface->registerCallback(f);
//
//		interface->start();
//
//		//while (!viewer.wasStopped())
//		while (true)
//		{
//			boost::this_thread::sleep(boost::posix_time::seconds(1));
//		}
//
//		interface->stop();
//	}
//
//	//pcl::visualization::CloudViewer viewer;
//};
//#endif
//
//int main(int argc, char ** argv)
//{
//#ifdef RVLPCL
//	OpenNIProcessor process;
//	process.run();
//#endif
//
//	return 0;
//}


