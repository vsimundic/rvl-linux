#pragma once

class CRVLVTKRenderer
{
public:
	CRVLVTKRenderer(void);
	virtual ~CRVLVTKRenderer(void);
#ifdef RVLVTK
	void Init(int Width, int Height);
	void Save2PLY(char *filename);

public:
	vtkRenderer *m_pRenderer;
	vtkRenderWindow *m_pWindow;
	vtkRenderWindowInteractor *m_pInteractor;
private:
	vtkInteractorStyleTrackballCamera *m_pInteractorStyle;
#endif
};
