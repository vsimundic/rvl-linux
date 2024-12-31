#include "Platform.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#endif
#include "RVLVTKRenderer.h"

CRVLVTKRenderer::CRVLVTKRenderer(void)
{
#ifdef RVLVTK
	m_pRenderer = NULL;
	m_pWindow = NULL;
	m_pInteractor = NULL;
	m_pInteractorStyle = NULL;
#endif
}

CRVLVTKRenderer::~CRVLVTKRenderer(void)
{
//#ifdef RVLVTK
//	if(m_pRenderer)
//		delete m_pRenderer;
//
//	if(m_pWindow)
//		delete m_pWindow;
//
//	if(m_pInteractor)
//		delete m_pInteractor;
//
//	if(m_pInteractorStyle)
//		delete m_pInteractorStyle;
//#endif
}

#ifdef RVLVTK
void CRVLVTKRenderer::Init(int Width, int Height)
{
	m_pRenderer = vtkRenderer::New(); 
	m_pWindow = vtkRenderWindow::New();
	m_pInteractor = vtkRenderWindowInteractor::New();
	m_pWindow->AddRenderer(m_pRenderer);
	m_pWindow->SetSize(Width, Height);
	m_pInteractor->SetRenderWindow(m_pWindow);
	m_pInteractorStyle = vtkInteractorStyleTrackballCamera::New();
	m_pInteractor->SetInteractorStyle(m_pInteractorStyle);
}

void CRVLVTKRenderer::Save2PLY(char *filename)
{
	vtkActorCollection* actColl;
	actColl = m_pRenderer->GetActors();
	vtkActor* act;
	actColl->InitTraversal();
	act = actColl->GetNextActor();
	
	//Saving
	vtkSmartPointer<vtkPLYWriter> plyExp = vtkSmartPointer<vtkPLYWriter>::New();
	plyExp->SetFileName(filename);
	plyExp->SetInputData(act->GetMapper()->GetInput());
	//plyExp->SetInputConnection(act->GetMapper()->GetInput()->);
	plyExp->SetArrayName("RGB");
	plyExp->Write();
}
#endif