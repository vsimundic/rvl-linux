#pragma once

namespace RVL
{
	class Gripper
	{
	public:
		Gripper();
		~Gripper();

		void CreateBox(
			double a,
			double b,
			double c,
			double red,
			double green,
			double blue);
		void AddPart(Gripper * pPart);
		void Translate(
			double x,
			double y,
			double z);
		void RotX(double q);
		void RotY(double q);
		void RotZ(double q);
		void AddToRenderer(
			vtkSmartPointer<vtkRenderer> renderer,
			vtkSmartPointer<vtkTransform> T = NULL);
		void CreateGripper(
			float width,
			vtkSmartPointer<vtkTransform> TIn,
			float *translate,
			vtkSmartPointer<vtkRenderer> renderer,
			bool scale=false);

	public:
		std::vector<Gripper *> parts;
		Gripper *parent;

	private:
		vtkSmartPointer<vtkActor> actor;
		vtkSmartPointer<vtkTransform> pose;
	};
}

