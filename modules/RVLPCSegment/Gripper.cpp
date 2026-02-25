//#include "stdafx.h"
#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkLine.h>
#include <vtkAxesActor.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricEllipsoid.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkRenderer.h>
#include "Util.h"
#include "Graph.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
//#include <pcl/common/common.h>
//#include <pcl/PolygonMesh.h>
//#include "PCLTools.h"
//#include "PCLMeshBuilder.h"
//#include "RGBDCamera.h"
#include "Mesh.h"
#include "Visualizer.h"

#include "Gripper.h"
using namespace RVL;


Gripper::Gripper()
{
	pose = vtkSmartPointer<vtkTransform>::New();

	pose->PostMultiply();

	actor = NULL;
}


Gripper::~Gripper()
{
}

void Gripper::CreateBox(
	double a,
	double b,
	double c,
	double red,
	double green,
	double blue)
{
	// Create a cube.
	vtkSmartPointer<vtkCubeSource> cubeSource =
		vtkSmartPointer<vtkCubeSource>::New();
	cubeSource->SetXLength(a);
	cubeSource->SetYLength(b);
	cubeSource->SetZLength(c);


	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(cubeSource->GetOutputPort());
	
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	
	actor->GetProperty()->SetInterpolationToFlat();
	actor->GetProperty()->SetColor(red, green, blue);
	
}

void Gripper::AddPart(Gripper * pPart)
{
	parts.push_back(pPart);
}


void Gripper::Translate(double x, double y, double z)
{
	pose->Translate(x, y, z);

	vtkMatrix4x4 *debugM = pose->GetMatrix();

	//double *t_ = pose->GetPosition();

	//int debug = 0;
}


void Gripper::RotX(double q)
{
	pose->RotateX(q);

	//double *q_ = pose->GetOrientationWXYZ();
}

void Gripper::RotY(double q)
{
	pose->RotateY(q);
}

void Gripper::RotZ(double q)
{
	pose->RotateZ(q);
}


void Gripper::AddToRenderer(
	vtkSmartPointer<vtkRenderer> renderer,
	vtkSmartPointer<vtkTransform> TIn)
{
	vtkSmartPointer<vtkTransform> T;

	if (TIn)
	{
		T = TIn;

		//actor->SetUserTransform(T);

		//vtkMatrix4x4 *debugM = actor->GetUserTransform()->GetMatrix();

		//double *t = actor->GetPosition();

		//vtkMatrix4x4 *debugM = T->GetMatrix();

		if (actor)
		{
			double *t_ = T->GetPosition();

			actor->SetPosition(t_);

			double *q_ = T->GetOrientationWXYZ();

			actor->RotateWXYZ(q_[0], q_[1], q_[2], q_[3]);
			
		}

		//int debug = 0;
	}
	else
		T = vtkSmartPointer<vtkTransform>::New();

	//vtkMatrix4x4 *debugM = T->GetMatrix();
	renderer->AddActor(actor);

	vtkSmartPointer<vtkTransform> T_ = vtkSmartPointer<vtkTransform>::New();

	for (Gripper *part : parts)
	{
		T_->Identity();

		T_->Concatenate(T);

		//debugM = T_->GetMatrix();

		//debugM = part->pose->GetMatrix();

		T_->Concatenate(part->pose);

		//debugM = T_->GetMatrix();
	
		part->AddToRenderer(renderer, T_);
	}
}

void Gripper::CreateGripper(
	float width,
	vtkSmartPointer<vtkTransform> TIn,
	float *translate,
	vtkSmartPointer<vtkRenderer> renderer,
	bool scale)
{
	float s;
	if (scale)
		s = 10;
	else s = 1;

	Gripper gripper, b1, b2, b3, b4; //b1=rotation axis box, b2=perpendicular box, b3,b4=fingers

	double b1x, b1y, b1z, b2x, b2y, b2z, b3x, b3y, b3z, b4x, b4y, b4z;



	b1x = 0.05/s;
	b1y = 0.05/s;
	b1z = 0.1/s;

	b2x = 0.05/s; 
	b2y = 0.1/s;
	b2z = 0.05/s;
	
	b3x = 0.01/s; 
	b3y = 0.01/s;
	b3z = 0.05/s; 

	b4x = 0.01/s;
	b4y = 0.01/s;
	b4z = 0.05/s;

	double red, green, blue;
	red = 0.0;
	green = 0.8;
	blue = 0.0;

	b1.CreateBox(b1x, b1y, b1z, red, green, blue);
	gripper.AddPart(&b1);
	b1.Translate(0.0, 0.0, (b4z / 2) + b2z + (b1z / 2));

	b2.CreateBox(b2x, b2y, b2z, red, green, blue);
	gripper.AddPart(&b2);
	b2.Translate(0.0, 0.0, (b4z / 2) + (b2z/2));
	
	red = 0.9;
	green = 0.0;
	blue = 0.0;

	b3.CreateBox(b3x, b3y, b3z, red, green, blue);
	gripper.AddPart(&b3);
	b3.Translate(0.0, - (b4y / 2) - width/2, 0.0);

	b4.CreateBox(b4x, b4y, b4z, red, green, blue);
	gripper.AddPart(&b4);
	b4.Translate(0.0, (b4y / 2) + width/2, 0.0);

	//gripper.Translate(translate[0], translate[1], translate[2]);

	//float thetax, thetay, thetaz;

	//thetax = atan2(rotate[7], rotate[8]);
	//thetay = atan2(-rotate[6], sqrt(rotate[7] * rotate[7] + rotate[8] * rotate[8]));
	//thetaz = atan2(rotate[3], rotate[0]);
	//gripper.RotX(thetax * 180 / 3.14);
	//gripper.RotY(thetay * 180 / 3.14);
	//gripper.RotZ(thetaz * 180 / 3.14);

	//vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	//vtkSmartPointer<vtkRenderWindow> renderWindow =	vtkSmartPointer<vtkRenderWindow>::New();
	//renderWindow->AddRenderer(renderer);
	//vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	//renderWindowInteractor->SetRenderWindow(renderWindow);
	//vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	//renderWindowInteractor->SetInteractorStyle(style);

	// Add the actor to the scene
	//renderer->AddActor(actor);
	gripper.AddToRenderer(renderer, TIn);
	renderer->SetBackground(.1, .3, .2); // Background color dark green

	//// Render and interact
	//renderWindow->SetWindowName("gripper");
	//renderWindow->Render();
	//renderWindowInteractor->Start();
}

