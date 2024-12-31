#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);
#ifdef RVLVTK
//#include "vtkActor.h"
//#include "vtkPointData.h"
//#include "vtkCellData.h"
//#include "vtkFloatArray.h"
//#include "vtkSmartPointer.h"
//#include "vtkPolyData.h"
//#include "vtkTexture.h"

class VTKActorObj
{
public:
	vtkSmartPointer<vtkProp> prop;
	vtkSmartPointer<vtkPolyData> polydata;
	vtkSmartPointer<vtkTexture> texture;

	VTKActorObj()
	{
		prop = vtkSmartPointer<vtkActor>::New(); //vtkSmartPointer<vtkProp>::New();	does not pass
		polydata = vtkSmartPointer<vtkPolyData>::New();
		texture = vtkSmartPointer<vtkTexture>::New();
	}

	VTKActorObj(vtkSmartPointer<vtkProp> prop, vtkSmartPointer<vtkPolyData> polydata = NULL, vtkSmartPointer<vtkTexture> texture = NULL)
	{
		this->prop = prop;
		this->polydata = polydata;
		this->texture = texture;
	}

	~VTKActorObj()
	{
		prop = NULL;
		polydata = NULL;
		texture = NULL;
	}
};
#endif