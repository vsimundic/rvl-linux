//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);
#include "vtkActor.h"
#include "vtkPointData.h"
#include "vtkCellData.h"
#include "vtkFloatArray.h"
#include "vtkSmartPointer.h"
#include "vtkPolyData.h"
#include "vtkTexture.h"

class VTKActorObj
{
public:
	vtkSmartPointer<vtkActor> actor;
	vtkSmartPointer<vtkPolyData> polydata;
	vtkSmartPointer<vtkTexture> texture;

	VTKActorObj()
	{
		actor = vtkSmartPointer<vtkActor>::New();
		polydata = vtkSmartPointer<vtkPolyData>::New();
		texture = vtkSmartPointer<vtkTexture>::New();
	}

	VTKActorObj(vtkSmartPointer<vtkActor> actor, vtkSmartPointer<vtkPolyData> polydata, vtkSmartPointer<vtkTexture> texture = NULL)
	{
		this->actor = actor;
		this->polydata = polydata;
		this->texture = texture;
	}

	~VTKActorObj()
	{
		actor = NULL;
		polydata = NULL;
		texture = NULL;
	}
};