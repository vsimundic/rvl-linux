// RVLLocalizationDemo.cpp : Defines the entry point for the console application.
//

#include "OpenNI.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include <vtkTriangle.h>		// Remove after completion of MarchingCubes.
#include <vtkLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
//#include "RFRecognition.h"
//#include "RVLMeshNoiser.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include <pcl/common/common.h>
//#include <pcl/registration/registration.h>
#include <pcl/PolygonMesh.h>
//#include <pcl/io/openni2_grabber.h>
//#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"
#include "MarchingCubes.h"
#include "NeighborhoodTool.h"
#include "VN.h"
#include "VNInstance.h"
#include "VNClassifier.h"
#include "PanTilt.h"			// Mateja
#include "ProjectionCube.h"		// Mateja
#include "View.h"				// Mateja
#include "VNLocalizer.h"




#define RVLLOCALIZATION_DEMO_FLAG_SAVE_PLY				0x00000001
//#define RVLLOCALIZATION_DEMO_LOG_FILE
#define RVLLOCALIZATION_DEMO_LOG_RESULTS

using namespace RVL;

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pSceneSequenceFileName,
	char **pActiveVisionPLYFilePath,
	char **pResultsFolder,
	DWORD &flags,
	Camera &camera,
	bool &bActiveVision,
	bool &bActiveVisionSimulation,
	int &noGeneratedViews,
	int &noIterations)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SceneSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceFileName);
	pParamData = pParamList->AddParam("ActiveVisionPLYFilePath", RVLPARAM_TYPE_STRING, pActiveVisionPLYFilePath);
	pParamData = pParamList->AddParam("ResultsFolder", RVLPARAM_TYPE_STRING, pResultsFolder);
	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLLOCALIZATION_DEMO_FLAG_SAVE_PLY);
	pParamData = pParamList->AddParam("Camera.fu", RVLPARAM_TYPE_FLOAT, &(camera.fu));
	pParamData = pParamList->AddParam("Camera.fv", RVLPARAM_TYPE_FLOAT, &(camera.fv));
	pParamData = pParamList->AddParam("Camera.uc", RVLPARAM_TYPE_FLOAT, &(camera.uc));
	pParamData = pParamList->AddParam("Camera.vc", RVLPARAM_TYPE_FLOAT, &(camera.vc));
	pParamData = pParamList->AddParam("Camera.w", RVLPARAM_TYPE_INT, &(camera.w));
	pParamData = pParamList->AddParam("Camera.h", RVLPARAM_TYPE_INT, &(camera.h));
	
	pParamData = pParamList->AddParam("activeVision", RVLPARAM_TYPE_BOOL, &bActiveVision); // Hrzica
	pParamData = pParamList->AddParam("activeVisionSimulation", RVLPARAM_TYPE_BOOL, &bActiveVisionSimulation); // Hrzica
	pParamData = pParamList->AddParam("numberOfViews", RVLPARAM_TYPE_INT, &noGeneratedViews);
	pParamData = pParamList->AddParam("numberOfIterations", RVLPARAM_TYPE_INT, &noIterations);

}

float maxAbsCoordinate(Point* p)
{
	float max = 0.0;
	for (int it = 0; it < 3; it++)
	{
		if (abs(p->N[it]) > max)
		{
			max = abs(p->N[it]);
		}
	}
	return max;
}


void VisualizeCorrespondences(Visualizer *pVisualizer, Array<Point> points_vw1, Array<Point> points_vw2, std::vector<Pair<int, int>> correspondences)//Array<Pair<int, int>> correspondences)
{

	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> linesPolyData =
		vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iLine = 0;

	double P_vw1[3], P_vw2[3];
	Point *pPt_vw1, *pPt_vw2;
	float *pP_vw1, *pP_vw2;

	int noCrsp = correspondences.size();
	//int noCrsp = correspondences.n;
	//Pair<int, int> *current_correspondence;
	int ptID_vw1, ptID_vw2;

	//correspondences.n = 0;

	for (int i = 0; i < noCrsp; i++)
	{
		ptID_vw1 = correspondences[i].a;
		ptID_vw2 = correspondences[i].b;
		//current_correspondence = correspondences.Element + i;
		//ptID_vw1 = current_correspondence->a;
		//ptID_vw2 = current_correspondence->b;

		pPt_vw1 = points_vw1.Element + ptID_vw1;
		RVLCOPY3VECTOR(pPt_vw1->P, P_vw1);
		pts->InsertNextPoint(P_vw1);

		pPt_vw2 = points_vw2.Element + ptID_vw2;
		RVLCOPY3VECTOR(pPt_vw2->P, P_vw2);
		pts->InsertNextPoint(P_vw2);

		iLine++;

		float dP[3];
		RVLDIF3VECTORS(pPt_vw2->P, pPt_vw1->P, dP);
		float e = sqrt(RVLDOTPRODUCT3(dP, dP));

		int debug = 0;
	}


	// Add the points to the polydata container
	linesPolyData->SetPoints(pts);

	// Create lines.
	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	unsigned char red[3] = { 255, 0, 0 };

	int nLines = iLine;

	vtkSmartPointer<vtkLine> *line = new vtkSmartPointer<vtkLine>[nLines];

	for (iLine = 0; iLine < nLines; iLine++)
	{
		line[iLine] = vtkSmartPointer<vtkLine>::New();

		line[iLine]->GetPointIds()->SetId(0, 2 * iLine);
		line[iLine]->GetPointIds()->SetId(1, 2 * iLine + 1);

		lines->InsertNextCell(line[iLine]);

		colors->InsertNextTupleValue(red);
	}

	// Add the lines to the polydata container
	linesPolyData->SetLines(lines);

	// Color the lines.
	// SetScalars() automatically associates the values in the data array passed as parameter
	// to the elements in the same indices of the cell data array on which it is called.
	// This means the first component (red) of the colors array
	// is matched with the first component of the cell array (line 0)
	// and the second component (green) of the colors array
	// is matched with the second component of the cell array (line 1)
	linesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(linesPolyData);

	//vtkSmartPointer<vtkActor> actor =
	//	vtkSmartPointer<vtkActor>::New();
	//actor->SetMapper(mapper);
	pVisualizer->normals = vtkSmartPointer<vtkActor>::New();
	pVisualizer->normals->SetMapper(mapper);
	pVisualizer->normals->GetProperty()->SetLineWidth(3);

	pVisualizer->renderer->AddActor(pVisualizer->normals);

	//visualizer.Run();

	delete[] line;
}


float MultiViewICP(View* iView, Point *p_i, View* jView, Point *p_j, float d_ik, float* g_ij_, float* h_ij_, float* A_ij_, float* B_ij_, float* D_ij_)
{
	
	float t_[3];
	float nR[3];
	float d, E_ij_;


	//T_ij = Rj*p_j + tj - ti;
	float T_ij[3];
	float Rj_pj[3];
	RVLMULMX3X3VECT(jView->cameraView.R, p_j->P, Rj_pj);	// Rj_pj = Rj * pj
	RVLSUM3VECTORS(Rj_pj, jView->cameraView.t, t_);			// t_ = Rj_pj + tj
	RVLDIF3VECTORS(t_, iView->cameraView.t, T_ij);			// T_ij = t_ - ti

	//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
	float Ri_ni[3];
	float e_ijk;
	RVLMULMX3X3VECT(iView->cameraView.R, p_i->N, Ri_ni);	// Ri_ni = (Ri*n_ik)
	e_ijk = RVLDOTPRODUCT3(Ri_ni, T_ij);					// e_ijk = Rn' * T_ij
	e_ijk = e_ijk - d_ik;
	
	// cost calcualtion
	E_ij_ = e_ijk * e_ijk;

	// a - matrix
	float a_ijk_up[3];
	RVLCROSSPRODUCT3(Ri_ni, T_ij, a_ijk_up);				// a_ijk_up = cross((Ri*n_ik), T_ij);

	float a_ijk_dwn[3];
	RVLNEGVECT3(Ri_ni, a_ijk_dwn);							// a_ijk_dwn = -(Ri*n_ik) = -Rn;

	
	cv::Mat a_up;
	a_up.create(3, 1, CV_32FC1);
	std::memcpy(a_up.data, a_ijk_up, 3 * 1 * sizeof(float));

	cv::Mat a_dwn;
	a_dwn.create(3, 1, CV_32FC1);
	std::memcpy(a_dwn.data, a_ijk_dwn, 3 * 1 * sizeof(float));

	cv::Mat a;												//a_ijk = vertcat(a_ijk_up, a_ijk_dwn);
	a.create(6, 1, CV_32FC1);
	a_up.copyTo(a(cv::Rect(0, 0, 1, 3)));
	a_dwn.copyTo(a(cv::Rect(0, 3, 1, 3)));

	// b - matrix
	float b_ijk_up[3];
	RVLCROSSPRODUCT3(Rj_pj, Ri_ni, b_ijk_up);				// b_ijk_up = cross((Rj*p_j), (Ri*n_ik));

	cv::Mat b_up;
	b_up.create(3, 1, CV_32FC1);
	std::memcpy(b_up.data, b_ijk_up, 3 * 1 * sizeof(float));

	cv::Mat b_dwn;
	b_dwn.create(3, 1, CV_32FC1);
	std::memcpy(b_dwn.data, Ri_ni, 3 * 1 * sizeof(float));

	cv::Mat b;												//b_ijk = vertcat(b_ijk_up, b_ijk_dwn);
	b.create(6, 1, CV_32FC1);
	b_up.copyTo(b(cv::Rect(0, 0, 1, 3)));
	b_dwn.copyTo(b(cv::Rect(0, 3, 1, 3)));

	// g - matrix
	cv::Mat g;
	g.create(6, 1, CV_32FC1);
	g = a * e_ijk;
	float *g_ = (float *)(g.data);

	// h - matrix
	cv::Mat h;
	h.create(6, 1, CV_32FC1);
	h = b * e_ijk;
	float *h_ = (float *)(h.data);

	// A - matrix
	cv::Mat A;
	A.create(6, 6, CV_32FC1);
	A = a * a.t();				// A = a*a';
	float *A_ = (float *)(A.data);

	// B - matrix
	cv::Mat B;
	B.create(6, 6, CV_32FC1);
	B = b * b.t();
	float *B_ = (float *)(B.data);

	// D - matrix
	cv::Mat D;
	D.create(6, 6, CV_32FC1);
	D = a * b.t();
	float *D_ = (float *)(D.data);


	float ni_Ri[3];
	RVLMULMX3X3VECT(iView->cameraView.R, p_i->N, ni_Ri);		// ni_Ri = ni'*Ri'
	d = RVLDOTPRODUCT3(ni_Ri, T_ij);

	//x_nl = d - d_ik;

	std::memcpy(g_ij_, g_, 6 * 1 * sizeof(float));
	std::memcpy(h_ij_, h_, 6 * 1 * sizeof(float));
	std::memcpy(A_ij_, A_, 6 * 6 * sizeof(float));
	std::memcpy(B_ij_, B_, 6 * 6 * sizeof(float));
	std::memcpy(D_ij_, D_, 6 * 6 * sizeof(float));


	return E_ij_;

}


float MultiPointCloudReg(std::vector<View*> views, float* X, Visualizer *pVisualizer)
{
	int noViews = views.size();
	//float CosAngle = cos(20.00 * PI / 180.0);
	float CosAngle = views[0]->projectionCube.CosAngle;
	float distance_threshold = views[0]->projectionCube.distance_threshold;
	float x_nl_sum = 0;

	int cubeHalfSize = views[0]->projectionCube.cubeHalfSize;
	int noCubeSideRows = 2 * cubeHalfSize + 1;
	int noCubeSideColumns = 2 * cubeHalfSize + 1;


	// Write correspondences to file
	FILE* pCubeCrspPts = NULL;
	////char *viewFileName3 = RVLCreateFileName(filePath, ".ply", -1, "cubePts.txt");
	pCubeCrspPts = fopen("cubeCrspPts.txt", "w");

	FILE* pCrspPts = NULL;
	pCrspPts = fopen("crspPts_list.txt", "w");

	cv::Mat Q;
	int noRows = 6 * noViews;
	int Q_noColumns = 6 * noViews;
	Q.create(noRows, Q_noColumns, CV_32FC1);
	Q = Q.zeros(noRows, Q_noColumns, CV_32FC1);

	cv::Mat Q_;
	Q_.create(6, 6, CV_32FC1);

	cv::Mat r;
	r.create(noRows, 1, CV_32FC1);
	r = r.zeros(noRows, 1, CV_32FC1);

	cv::Mat u;
	u.create(noRows, 6, CV_32FC1);
	u = u.zeros(noRows, 6, CV_32FC1);

	cv::Mat u_;
	u_.create(6, 6, CV_32FC1);

	//initialization
	cv::Mat g_ij_sum;
	g_ij_sum.create(6, 1, CV_32FC1);
	float *g_ij_sum_ = (float *)(g_ij_sum.data);

	cv::Mat h_ij_sum;
	h_ij_sum.create(6, 1, CV_32FC1);
	float *h_ij_sum_ = (float *)(h_ij_sum.data);

	cv::Mat A_ij_sum;
	A_ij_sum.create(6, 6, CV_32FC1);
	float *A_ij_sum_ = (float *)(A_ij_sum.data);

	cv::Mat B_ij_sum;
	B_ij_sum.create(6, 6, CV_32FC1);
	float *B_ij_sum_ = (float *)(B_ij_sum.data);

	cv::Mat D_ij_sum;
	D_ij_sum.create(6, 6, CV_32FC1);
	float *D_ij_sum_ = (float *)(D_ij_sum.data);

	cv::Mat g_ij;
	g_ij.create(6, 1, CV_32FC1);
	float *g_ij_ = (float *)(g_ij.data);

	cv::Mat h_ij;
	h_ij.create(6, 1, CV_32FC1);
	float *h_ij_ = (float *)(h_ij.data);

	cv::Mat A_ij;
	A_ij.create(6, 6, CV_32FC1);
	float *A_ij_ = (float *)(A_ij.data);

	cv::Mat B_ij;
	B_ij.create(6, 6, CV_32FC1);
	float *B_ij_ = (float *)(B_ij.data);

	cv::Mat D_ij;
	D_ij.create(6, 6, CV_32FC1);
	float *D_ij_ = (float *)(D_ij.data);

	float E_ij;
	float E_ij_sum = 0;

	//Array<Pair<int, int>> crspPts_list;
	//crspPts_list.Element = new Pair < int, int >;

	//crspPts_list.n = 0;

	//Pair<int, int> *crspPts;

	std::vector<Pair<int, int>> correspondences;

	int nCorrespTotal = 0;

	int goTo_column = 0;
	int goTo_row = 0;

	for (int iView = 0; iView < noViews; iView++)
	{
		//views[iView]->correspodance_list.reserve(noViews);

		for (int jView = 0; jView < noViews; jView++)
		{

			//crspPts_list.n = 0;
			views[iView]->correspodance_list[jView].clear();
			//correspondences.clear();

			//Initialize variables by setting them to zero
			g_ij_sum = g_ij_sum.zeros(6, 1, CV_32FC1);
			h_ij_sum = h_ij_sum.zeros(6, 1, CV_32FC1);
			A_ij_sum = A_ij_sum.zeros(6, 6, CV_32FC1);
			B_ij_sum = B_ij_sum.zeros(6, 6, CV_32FC1);
			D_ij_sum = D_ij_sum.zeros(6, 6, CV_32FC1);

			if (iView == jView)
			{
				goTo_column = goTo_column + 6;
				
				Pair<int, int> crspPts_temp;
				crspPts_temp.a = -1;
				crspPts_temp.b = -1;

				views[iView]->correspodance_list[jView].push_back(crspPts_temp);
				//correspondences.push_back(crspPts_temp);
				//views[iView]->correspodance_list.push_back(correspondences);
				continue;
			}


			for (int cubeSide = 0; cubeSide < 6; cubeSide++)				//for every projectionCube side
			{
				for (int cubeSideRow = 0; cubeSideRow < noCubeSideRows; cubeSideRow++)
				{
					for (int cubeSideColumn = 0; cubeSideColumn < noCubeSideColumns; cubeSideColumn++)
						//for (int cubeSideColumn = cubeSide; cubeSideColumn < (noCubeSideColumns * 6); cubeSideColumn += 6)
					{
						int ptIndex_iView = views[iView]->projectionCube.cubeMatrix[cubeSide][cubeSideColumn + cubeSideRow * noCubeSideColumns];
						int ptIndex_jView = views[jView]->projectionCube.cubeMatrix[cubeSide][cubeSideColumn + cubeSideRow * noCubeSideColumns];

						//Ako su obje tockice unutar thresholda, uzeti njihovu srednju vrijednost
						//Ako su tockice izvan thresholda, uzeti onu koja je blize kameri

						if (ptIndex_iView == -1 || ptIndex_jView == -1)
							continue;

						Point *p_i = &(views[iView]->projectionCube.transformedPoints.Element[ptIndex_iView]);
						Point *p_j = &(views[jView]->projectionCube.transformedPoints.Element[ptIndex_jView]);

						float CosAngleBetweenPoints = RVLDOTPRODUCT3(p_i->N, p_j->N);

						if (CosAngleBetweenPoints >= CosAngle)
						{
							float pt_distance[3];
							float d_ik, e_ik;

							RVLDIF3VECTORS(p_j->P, p_i->P, pt_distance);
							//e_ik = sqrt(RVLDOTPRODUCT3(pt_distance, pt_distance));
							e_ik = RVLDOTPRODUCT3(p_i->N, pt_distance);
							d_ik = RVLDOTPRODUCT3(p_i->N, p_i->P);

							//d_ik = abs(d_ik);

							if (abs(e_ik) < distance_threshold)
							{
								fprintf(pCubeCrspPts, "%d %d %d %d\n", iView, ptIndex_iView, jView, ptIndex_jView);

								// Save correspodence
								Pair<int, int> crspPts_temp;
								crspPts_temp.a = ptIndex_iView;
								crspPts_temp.b = ptIndex_jView;

								views[iView]->correspodance_list[jView].push_back(crspPts_temp);
								//correspondences.push_back(crspPts_temp);

								if (iView < jView)
								{
									//crspPts.a = ptIndex_iView;
									//crspPts.b = ptIndex_jView;
									
									//crspPts_list.Element->a = ptIndex_iView;
									//crspPts_list.Element->b = ptIndex_jView;

									//fprintf(pCrspPts, "%d %d %d %d\n", iView, crspPts_list.Element->a, jView, crspPts_list.Element->b);
									//crspPts_list.n++;									

									//Pair<int, int> crspPts_temp;
									//
									//crspPts_temp.a = ptIndex_iView;
									//crspPts_temp.b = ptIndex_jView;

									//crspPts_temp->iView = iView;
									//crspPts_temp->jView = jView;
									//crspPts_temp->ptID_vw1 = ptIndex_iView;
									//crspPts_temp->ptID_vw2 = ptIndex_jView;

									//crspPts crspPts_temp;
									//crspPts_temp.iView = iView;
									//crspPts_temp.ptID_iView = ptIndex_iView;
									//crspPts_temp.jView = jView;
									//crspPts_temp.ptID_jView = ptIndex_jView;



									//views[iView]->crspPts_list.push_back(crspPts_temp);

									//views[iView]->correspondences[jView].push_back(crspPts_temp);
								}

								p_i = &(views[iView]->mesh.NodeArray.Element[ptIndex_iView]);
								p_j = &(views[jView]->mesh.NodeArray.Element[ptIndex_jView]);
								d_ik = RVLDOTPRODUCT3(p_i->N, p_i->P);

								
								g_ij = g_ij.zeros(6, 1, CV_32FC1);
								h_ij = h_ij.zeros(6, 1, CV_32FC1);
								A_ij = A_ij.zeros(6, 6, CV_32FC1);
								B_ij = B_ij.zeros(6, 6, CV_32FC1);
								D_ij = D_ij.zeros(6, 6, CV_32FC1);

								//MultiplePtCloud_calc
								E_ij = MultiViewICP(views[iView], p_i, views[jView], p_j, d_ik, g_ij_, h_ij_, A_ij_, B_ij_, D_ij_);
								g_ij_sum = g_ij_sum + g_ij;
								h_ij_sum = h_ij_sum + h_ij;
								A_ij_sum = A_ij_sum + A_ij;
								B_ij_sum = B_ij_sum + B_ij;
								D_ij_sum = D_ij_sum + D_ij;								

								E_ij_sum = E_ij_sum + E_ij;
								//x_nl_sum = x_nl_sum + x_nl;

							} //end if (abs(d_ik) < distance_threshold)
						} //end if (CosAngleBetweenPoints >= CosAngle)
					} //end for (int cubeSideColumn)
				} // end for (int cubeSideRow)
			} // end for (int cubeSide)


			//views[iView]->correspodance_list.push_back(correspondences);
			printf("correspondences view%d-view%d: %d\n", iView, jView, views[iView]->correspodance_list[jView].size());
			//fclose(pCrspPts);

			nCorrespTotal += views[iView]->correspodance_list[jView].size();

			// Q - matrix
			Q_ = 2 * D_ij_sum;
			Q_.copyTo(Q(cv::Rect(goTo_column, goTo_row, 6, 6)));

			// u - matrix (blocks of Q-diagonal)
			u(cv::Rect(0, iView * 6, 6, 6)) = u(cv::Rect(0, iView * 6, 6, 6)) + A_ij_sum;
			u(cv::Rect(0, jView * 6, 6, 6)) = u(cv::Rect(0, jView * 6, 6, 6)) + B_ij_sum;

			// r - matrix
			r(cv::Rect(0, iView * 6, 1, 6)) = r(cv::Rect(0, iView * 6, 1, 6)) + g_ij_sum;
			r(cv::Rect(0, jView * 6, 1, 6)) = r(cv::Rect(0, jView * 6, 1, 6)) + h_ij_sum;

			goTo_column = goTo_column + 6;
			if (jView == noViews -1)
				goTo_column = 0;

		} // end for (int jView)

		goTo_row = goTo_row + 6;

	} // for (int iView)

	fclose(pCubeCrspPts);


	// Q - matrix diagonal elements
	int du = 0;
	for (int iView = 0; iView < noViews; iView++)
	{
		u(cv::Rect(0, iView * 6, 6, 6)).copyTo(Q(cv::Rect(du, du, 6, 6)));
		du = du + 6;
	}


	// Q matrix with regard to the referent view (1st view)
	cv::Mat Q_wrt_rv;
	Q_wrt_rv.create(noRows - 6, Q_noColumns - 6, CV_32FC1);
	Q(cv::Rect(6, 6, Q_noColumns - 6, noRows - 6)).copyTo(Q_wrt_rv);

	cv::Mat r_wrt_rv;
	r_wrt_rv.create(noRows - 6, 1, CV_32FC1);
	r(cv::Rect(0, 6, 1, noRows - 6)).copyTo(r_wrt_rv);

	cv::Mat x_wrt_rv;
	x_wrt_rv.create(noRows-6, 1, CV_32FC1);
	cv::solve(Q_wrt_rv, -r_wrt_rv, x_wrt_rv);
	float *x_wrt_rv_ = (float *)(x_wrt_rv.data);
	//RVLSCALE3VECTOR(x_wrt_rv_, 0.1, x_wrt_rv_);
	//float *x_wrt_rv_t_ = x_wrt_rv_ + 3;
	//RVLSCALE3VECTOR(x_wrt_rv_t_, 0.1, x_wrt_rv_t_);

	// solve Q^(-1)*(-r), formulated as Qx = -r wrt 1st view
	cv::Mat x;
	x.create(noRows, 1, CV_32FC1);
	x = x.zeros(noRows, 1, CV_32FC1);
	x_wrt_rv.copyTo(x(cv::Rect(0, 6, 1, noRows - 6)));
	float *x_ = (float *)(x.data);
	

	std::memcpy(X, x_, noViews*6 * 1 * sizeof(float));

	printf("Cost = %.4f\n", E_ij_sum);

	printf("Avg. cost = %f\n", sqrt(E_ij_sum / (float)nCorrespTotal));

	cv::Mat expectedCostChange;
	expectedCostChange = 2 * r_wrt_rv.t() * x_wrt_rv + x_wrt_rv.t() * Q_wrt_rv * x_wrt_rv;	// expectedCostChange = 2 * r_'*x_views_+x_views_'*Q_*x_views_;
	float* expectedCostChange_ = (float *)(expectedCostChange.data);

	printf("Expected Cost Change = %f\n", *expectedCostChange_);
	
	return E_ij_sum;
}

void MultiPointCloudCorrection(std::vector<View*> views, float* X)
{
	int noViews = views.size();

	float R0[9];
	float R_[9];
	float t0[3];
	float t_[3];

	cv::Mat x;
	x.create(noViews * 6, 1, CV_32FC1);
	std::memcpy(x.data, X, noViews * 6 * 1 * sizeof(float));
	
	cv::Mat fi;
	fi.create(3, 1, CV_32FC1);
	float *fi_ = (float *)(fi.data);

	float theta;

	cv::Mat s;
	s.create(3, 1, CV_32FC1);
	float *s_ = (float *)(s.data);

	float u[3];
	float dR[9];

	int goTo_row_fi = 0;
	int goTo_row_s = 3;

	for (int iView = 0; iView < noViews; iView++)
	{	
		x(cv::Rect(0, goTo_row_fi, 1, 3)).copyTo(fi);
		x(cv::Rect(0, goTo_row_s, 1, 3)).copyTo(s);
		
		theta = sqrt(RVLDOTPRODUCT3(fi_, fi_));

		if (theta >= 1e-6)
		{
			RVLSCALE3VECTOR2(fi_, theta, u);

			AngleAxisToRot<float>(u, theta, dR);

			RVLMXMUL3X3(dR, views[iView]->cameraView.R, R_);		// R_ = dR * R_

			printf("theta=%f\n", theta);
		}
		else
			RVLCOPYMX3X3(views[iView]->cameraView.R, R_);

		RVLSUM3VECTORS(views[iView]->cameraView.t, s_, t_);		// t_ = t_ + s			

		GetAngleAxis(R_, u, theta);

		AngleAxisToRot<float>(u, theta, R_);

		RVLCOPYMX3X3(R_, views[iView]->cameraView.R);
		RVLCOPY3VECTOR(t_, views[iView]->cameraView.t)

		goTo_row_fi = goTo_row_fi + 6;
		goTo_row_s = goTo_row_s + 6;
	}
}

float MultiPointCloudToPlaneError(std::vector<View*> views)
{
	int noViews = views.size();

	int cubeHalfSize = views[0]->projectionCube.cubeHalfSize;
	int noCubeSideRows = 2 * cubeHalfSize + 1;
	int noCubeSideColumns = 2 * cubeHalfSize + 1;

	float CosAngle = views[0]->projectionCube.CosAngle;
	float distance_threshold = views[0]->projectionCube.distance_threshold;

	float t_[3];
	float nR[3];
	float d;

	float T_ij[3];
	float Rj_pj[3];

	float Ri_ni[3];
	float e_ijk;

	float E_ij_;
	float E_ij_sum = 0;

	for (int iView = 0; iView < noViews; iView++)
	{
		for (int jView = 0; jView < noViews; jView++)
		{
			
			if (iView == jView)
				continue;
			
			int noCrsps = views[iView]->correspodance_list[jView].size();

			for (int iCrsp = 0; iCrsp < noCrsps; iCrsp++)
			{
				Pair<int, int> crspPts;
				crspPts = views[iView]->correspodance_list[jView].at(iCrsp);

				int ptIndex_iView = crspPts.a;
				int ptIndex_jView = crspPts.b;
				
				Point *p_i = &(views[iView]->mesh.NodeArray.Element[ptIndex_iView]);
				Point *p_j = &(views[jView]->mesh.NodeArray.Element[ptIndex_jView]);

				float CosAngleBetweenPoints = RVLDOTPRODUCT3(p_i->N, p_j->N);

				//if (CosAngleBetweenPoints >= CosAngle)
				{
					float pt_distance[3];
					float e_ik, d_ik;

					RVLDIF3VECTORS(p_j->P, p_i->P, pt_distance);
					e_ik = RVLDOTPRODUCT3(p_i->N, pt_distance);
					d_ik = RVLDOTPRODUCT3(p_i->N, p_i->P);

					//d_ik = abs(d_ik);

					//if (abs(e_ik) < distance_threshold)
					{
						p_i = &(views[iView]->mesh.NodeArray.Element[ptIndex_iView]);
						p_j = &(views[jView]->mesh.NodeArray.Element[ptIndex_jView]);

						//T_ij = Rj*p_j + tj - ti;
						RVLMULMX3X3VECT(views[jView]->cameraView.R, p_j->P, Rj_pj);		// Rj_pj = Rj * pj
						RVLSUM3VECTORS(Rj_pj, views[jView]->cameraView.t, t_);			// t_ = Rj_pj + tj
						RVLDIF3VECTORS(t_, views[iView]->cameraView.t, T_ij);			// T_ij = t_ - ti

						//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
						RVLMULMX3X3VECT(views[iView]->cameraView.R, p_i->N, Ri_ni);	// Ri_ni = (Ri*n_ik)
						e_ijk = RVLDOTPRODUCT3(Ri_ni, T_ij);					// e_ijk = Rn' * T_ij
						e_ijk = e_ijk - d_ik;

						// cost calcualtion
						E_ij_ = e_ijk * e_ijk;

						E_ij_sum = E_ij_sum + E_ij_;
					}
				}

			}
		}	
	}


	
	//for (int iView = 0; iView < noViews; iView++)
	//{
	//	for (int jView = 0; jView < noViews; jView++)
	//	{
	//		if (iView == jView)
	//			continue;

	//		for (int cubeSide = 0; cubeSide < 6; cubeSide++)				//for every projectionCube side
	//		{
	//			for (int cubeSideRow = 0; cubeSideRow < noCubeSideRows; cubeSideRow++)
	//			{
	//				for (int cubeSideColumn = 0; cubeSideColumn < noCubeSideColumns; cubeSideColumn++)
	//				{
	//					int ptIndex_iView = views[iView]->projectionCube.cubeMatrix[cubeSide][cubeSideColumn + cubeSideRow * noCubeSideColumns];
	//					int ptIndex_jView = views[jView]->projectionCube.cubeMatrix[cubeSide][cubeSideColumn + cubeSideRow * noCubeSideColumns];

	//					if (ptIndex_iView == -1 || ptIndex_jView == -1)
	//						continue;

	//					Point *p_i = &(views[iView]->projectionCube.transformedPoints.Element[ptIndex_iView]);
	//					Point *p_j = &(views[jView]->projectionCube.transformedPoints.Element[ptIndex_jView]);

	//					float CosAngleBetweenPoints = RVLDOTPRODUCT3(p_i->N, p_j->N);

	//					//if (CosAngleBetweenPoints >= CosAngle)
	//					{
	//						float pt_distance[3];
	//						float e_ik, d_ik;

	//						RVLDIF3VECTORS(p_j->P, p_i->P, pt_distance);
	//						e_ik = RVLDOTPRODUCT3(p_i->N, pt_distance);
	//						d_ik = RVLDOTPRODUCT3(p_i->N, p_i->P);

	//						//d_ik = abs(d_ik);

	//						//if (abs(e_ik) < distance_threshold)
	//						{
	//							p_i = &(views[iView]->mesh.NodeArray.Element[ptIndex_iView]);
	//							p_j = &(views[jView]->mesh.NodeArray.Element[ptIndex_jView]);

	//							//T_ij = Rj*p_j + tj - ti;
	//							RVLMULMX3X3VECT(views[jView]->cameraView.R, p_j->P, Rj_pj);		// Rj_pj = Rj * pj
	//							RVLSUM3VECTORS(Rj_pj, views[jView]->cameraView.t, t_);			// t_ = Rj_pj + tj
	//							RVLDIF3VECTORS(t_, views[iView]->cameraView.t, T_ij);			// T_ij = t_ - ti

	//							//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
	//							RVLMULMX3X3VECT(views[iView]->cameraView.R, p_i->N, Ri_ni);	// Ri_ni = (Ri*n_ik)
	//							e_ijk = RVLDOTPRODUCT3(Ri_ni, T_ij);					// e_ijk = Rn' * T_ij
	//							e_ijk = e_ijk - d_ik;

	//							// cost calcualtion
	//							E_ij_ = e_ijk * e_ijk;

	//							E_ij_sum = E_ij_sum + E_ij_;
	//						}
	//					}
	//				}
	//			}
	//		}
	//	}
	//}

	return E_ij_sum;

}



///////////////////
///    MAIN	   ////
///////////////////

//#define RVLLOCALIZATION_DEMO_LOG_FILE

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000000);

	CRVLMem mem;	// cycle memory

	mem.Create(1000000000);

	// Read parameters from a configuration file.

	char cfgSelectionFileName[] = "RVLLocalizationDemo.cfg";

	char *cfgFileName = ReadConfigurationFile(cfgSelectionFileName);

	if (cfgFileName == NULL)
		return 1;

	char *sceneSequenceFileName = NULL;
	char *ActiveVisionPLYFilePath = NULL;
	char *resultsFolder = NULL;
	char *cameraFileName = NULL;
	Camera camera;

	DWORD flags = 0x00000000;

	// Hrzica
	bool bActiveVision = false;
	bool bActiveVisionSimulation = false;
	int noGeneratedViews = 2;	// default value
	int noIterations = 50;		// default value

	CRVLParameterList ParamList;

	CreateParamList(&ParamList,
		&mem0,
		&sceneSequenceFileName,
		&ActiveVisionPLYFilePath,
		&resultsFolder,
		flags,
		camera,
		bActiveVision,
		bActiveVisionSimulation,
		noGeneratedViews,
		noIterations);

	ParamList.LoadParams(cfgFileName);

	// Create mesh builder.

	PCLMeshBuilder meshBuilder;

	meshBuilder.CreateParamList(&mem0);

	meshBuilder.Create(cfgFileName, &mem0);

	meshBuilder.ParamList.LoadParams(cfgFileName);

	meshBuilder.camera.depthFu = camera.fu;
	meshBuilder.camera.depthFv = camera.fv;
	meshBuilder.camera.depthUc = camera.uc;
	meshBuilder.camera.depthVc = camera.vc;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(meshBuilder.width, meshBuilder.height));

	//Mateja
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(camera.w, camera.h));
	//

	meshBuilder.PC = PC;

	Mesh mesh;


////	// PROBA
	////CV_CAP_OPENNI_POINT_CLOUD_MAP;
	////RVLSerial::RVLSerial("COM3");
	//cv::VideoCapture capture(CV_CAP_OPENNI);
	//for (;;)
	//{
	//	
	//	cv::Mat ptCloud;
	//	capture.grab();

	//	capture.retrieve(ptCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP);

	//	capture >> ptCloud;

	//	if (cv::waitKey(30) >= 0)
	//		break;
	//}
//

	// Create file sequence handleer.

	FileSequenceLoader sceneSequence;
	char filePath[200];

	sceneSequence.Init(sceneSequenceFileName);

	// Generate scene PLY files.

	if (flags & RVLLOCALIZATION_DEMO_FLAG_SAVE_PLY)
	{
		while (sceneSequence.GetNextPath(filePath))
			LoadMesh(&meshBuilder, filePath, &mesh, true);


		RVL_DELETE_ARRAY(sceneSequenceFileName);
		RVL_DELETE_ARRAY(resultsFolder);

		return 0;
	}

	Mesh roomMesh;

	// Read PLY for Active Vision simulation

	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();

	if (bActiveVisionSimulation == true)
	{		
		reader->SetFileName(ActiveVisionPLYFilePath);
		reader->Update();
		roomMesh.pPolygonData = reader->GetOutput();

		CameraView cameraView;
		
		//cameraView.LoadCameraFile(ActiveVisionPLYFilePath);				// Get R, t

		//RVL_DELETE_ARRAY(ActiveVisionPLYFilePath);
	}


	// Initialize surfel detection.

	SurfelGraph surfels;

	surfels.pMem = &mem;

	surfels.CreateParamList(&mem0);

	surfels.ParamList.LoadParams(cfgFileName);

	PlanarSurfelDetector surfelDetector;

	surfelDetector.CreateParamList(&mem0);

	surfelDetector.ParamList.LoadParams(cfgFileName);

	// Initialize localization.	

	VNLocalizer localizer;

	localizer.pMem0 = &mem0;
	localizer.pMem = &mem;
	localizer.pSurfels = &surfels;
	localizer.pSurfelDetector = &surfelDetector;
	
	localizer.camera.fu = meshBuilder.camera.depthFu;
	localizer.camera.fv = meshBuilder.camera.depthFv;
	localizer.camera.uc = meshBuilder.camera.depthUc;
	localizer.camera.vc = meshBuilder.camera.depthVc;
	localizer.camera.w = meshBuilder.width;
	localizer.camera.h = meshBuilder.height;

	//Mateja
	//localizer.camera = camera;
	//

	localizer.Create(cfgFileName);

	// Initialize visualization.

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	Visualizer visualizer;

	visualizer.Create();

	localizer.pVisualizer = &visualizer;

//#ifdef NEVER
	/// main loop

	
	// Mateja
	//std::vector<CameraView> cameraViews;
	
	// Load all camera views. (from SUNCG dataset)
	//if (VNLocalizer::cameraFileName == "outputcamerasfile.txt")
	//{
	//	LOCAL::LoadViews(cameraFileName, &cameraViews);
	//}
	//



	// Initialize views
	std::vector<View*> views;

	
	// Default parameters for projection cube
	//int cubeHalfSizeIn = 400;
	int cubeHalfSizeIn = 19;
	//float CosAngle_in = 0.0;
	float CosAngle_in = cos(20.00 * PI / 180.0);
	float distance_threshold_in = 0.02;

	int cubeSize = 2 * cubeHalfSizeIn + 1;	

	// Visualization.
	bool bVisualizer = true;									//change to "true" visualize scene

	localizer.cube.CreateProjectionCube(cubeHalfSizeIn, CosAngle_in, distance_threshold_in);		// Initialize projection cube 
	
	if (bActiveVision)
	{
		PanTilt panTilt;
		int depthScale = 2;
		int RGBScale = 1;
		int TiltOffset = -10; //deg

		float RCamPanTilt[9] = {0, 0, 1, -1, 0, 0, 0, -1, 0};


		if (bActiveVisionSimulation == false)
		{
			panTilt.Init(depthScale, RGBScale, TiltOffset);				//To do: Dodati u cfg file da kaze koji mu je COM port?
		}

		int dPan = 0;			// deg
		//int dTilt = 35;		// deg
		int dTilt = 0;			// deg
		int dRoll = 0;			// deg

		float PanTiltRoll[3], RPanTiltCube[9];
		PanTiltRoll[2] = dRoll;

#ifdef RVLLOCALIZATION_DEMO_LOG_RESULTS
		FILE *fpT = fopen((std::string(resultsFolder) + "\\R_groundTruth.txt").data(), "w");

		fclose(fpT);

		fpT = fopen((std::string(resultsFolder) + "\\R_real.txt").data(), "w");

		fclose(fpT);

		fpT = fopen((std::string(resultsFolder) + "\\R_corrected.txt").data(), "w");

		fclose(fpT);

		fpT = fopen((std::string(resultsFolder) + "\\t_groundTruth.txt").data(), "w");

		fclose(fpT);

		fpT = fopen((std::string(resultsFolder) + "\\t_real.txt").data(), "w");

		fclose(fpT);

		fpT = fopen((std::string(resultsFolder) + "\\t_corrected.txt").data(), "w");

		fclose(fpT);
#endif

		int axis[3] = { 2, 1, 0 };

		// Create pojection cube with samller resolution than default
		//cubeHalfSizeIn = 19;

		float pan, tilt;
		
		while (true)
		{

			// View registration.
			View *view = NULL;
			view = new View;

			view->overlapingViewList = localizer.overlappingViews_list;

			PanTiltRoll[0] = dPan;
			PanTiltRoll[1] = dTilt;

			Rotation<float>(PanTiltRoll, axis, RPanTiltCube);

			RVLMXMUL3X3(RPanTiltCube, RCamPanTilt, view->cameraView.R);

			RVLNULL3VECTOR(view->cameraView.t);

#ifdef RVLLOCALIZATION_DEMO_LOG_RESULTS
			FILE *fpT = fopen((std::string(resultsFolder) + "\\R_real.txt").data(), "a");

			PrintMatrix<float>(fpT, view->cameraView.R, 3, 3);

			fclose(fpT);

			fpT = fopen((std::string(resultsFolder) + "\\t_real.txt").data(), "a");

			PrintMatrix<float>(fpT, view->cameraView.t, 3, 1);

			fclose(fpT);
#endif


			Array2D<short int> depthImage_array;
			IplImage *pRGBImage = NULL;
			cv::Mat renderedDepthImg(camera.h, camera.w, CV_16UC1, cv::Scalar::all(0));

			if (bActiveVisionSimulation == true)
			{
				// Get depth data from a PLY scene
				//vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
				//mesh.pPolygonData->DeepCopy(pd);

				float RCamCubeSim[9], tCamCubeSim[3];
				float PanTiltRollSim[3];	

				// Perturbation of pan-tilt angles
				if (localizer.Views.size() > 0)
				{
					pan = (float)dPan + (float)(rand() % 2000 - 1000) * 0.001f * 5.0f;
					tilt = (float)dTilt + (float)(rand() % 2000 - 1000) * 0.001f * 5.0f;
				}
				else
				{
					pan = (float)dPan;
					tilt = (float)dTilt;
				}

				PanTiltRollSim[0] = pan;
				PanTiltRollSim[1] = tilt;
				PanTiltRollSim[2] = 0.0f;

				float RPanTiltCubeSim[9];

				Rotation<float>(PanTiltRollSim, axis, RPanTiltCubeSim);


				// Perturbation of angle-axis
				float fAxis[3];
				if (localizer.Views.size() > 0)
				{
					fAxis[0] = (float)(rand() % 2000 - 1000) * 0.001f;
					fAxis[1] = (float)(rand() % 2000 - 1000) * 0.001f;
					fAxis[2] = (float)(rand() % 2000 - 1000) * 0.001f;

					float fTmp;

					RVLNORM3(fAxis, fTmp);

					float RPerturb[9];

					AngleAxisToRot<float>(fAxis, 4.0f * DEG2RAD, RPerturb);

					RVLMXMUL3X3(RPerturb, RPanTiltCube, RPanTiltCubeSim);
				}

				RVLMXMUL3X3(RPanTiltCubeSim, RCamPanTilt, RCamCubeSim);

				//tCamCubeSim[0] = (float)(rand() % 2000 - 1000) * 0.001f * 3.0f;	// cm
				//tCamCubeSim[1] = (float)(rand() % 2000 - 1000) * 0.001f * 3.0f;	// cm
				//tCamCubeSim[2] = (float)(rand() % 2000 - 1000) * 0.001f * 3.0f;	// cm

				RVLNULL3VECTOR(tCamCubeSim);

#ifdef RVLLOCALIZATION_DEMO_LOG_RESULTS
				FILE *fpT = fopen((std::string(resultsFolder) + "\\R_groundTruth.txt").data(), "a");

				PrintMatrix<float>(fpT, RCamCubeSim, 3, 3);

				fclose(fpT);

				fpT = fopen((std::string(resultsFolder) + "\\t_groundTruth.txt").data(), "a");

				PrintMatrix<float>(fpT, tCamCubeSim, 3, 1);

				fclose(fpT);
#endif
							
				//renderedDepthImg = GenerateVTKPolyDataDepthImage_Kinect(roomMesh.pPolygonData, 10.0f, 70.0f, 1000.0f, camera.w, camera.h, camera.fv, camera.uc, camera.vc, RCamCubeSim, tCamCubeSim); // input ply in cm, returnes depth in mm
				renderedDepthImg = GenerateVTKPolyDataDepthImage_Kinect(roomMesh.pPolygonData, 1000.0f, 0.7f, 10.0f, camera.w, camera.h, camera.fv, camera.uc, camera.vc, RCamCubeSim, tCamCubeSim);	// input ply in m, returens  depth in mm

				// Image acquisition.
				// Copy renderedDepthImg to depthImage_array
				depthImage_array.Element = (short int *)(renderedDepthImg.data);
				depthImage_array.w = camera.w;
				depthImage_array.h = camera.h;	
			}
			else
			{
				//Image acquisition.
				panTilt.goTo_captureImg(dPan, dTilt, &depthImage_array, pRGBImage);
			}
			
			meshBuilder.CreateMesh(depthImage_array, pRGBImage, &(view->mesh), true, flags & RVLPCLMESHBUILDER_FLAG_UVD_SPACE);
			
#ifdef RVLLOCALIZATION_DEMO_LOG_FILE
			// Write R, t to file
	
			FILE *pCameraFile = fopen((std::string(resultsFolder) + "\\sl-" + std::to_string(localizer.Views.size()) + "_camera.txt").data(), "w");
				
			fprintf(pCameraFile, "%f %f %f %f %f %f %f %f %f ", 
				view->cameraView.R[0], view->cameraView.R[1], view->cameraView.R[2],
				view->cameraView.R[3], view->cameraView.R[4], view->cameraView.R[5], 
				view->cameraView.R[6], view->cameraView.R[7], view->cameraView.R[8]);
			fprintf(pCameraFile, "%f %f %f\n", view->cameraView.t[0], view->cameraView.t[1], view->cameraView.t[2]);

			fclose(pCameraFile);

			FILE *pPtsFile = fopen((std::string(resultsFolder) + "\\sl-" + std::to_string(localizer.Views.size()) + "_pts.txt").data(), "w");

			int iPt;
			float *P, *N;
			Point *pPt;

			for (iPt = 0; iPt < view->mesh.NodeArray.n; iPt++)
			{
				pPt = view->mesh.NodeArray.Element + iPt;

				P = pPt->P;
				N = pPt->N;

				fprintf(pPtsFile, "%d %.5f %.5f %.5f %.5f %.5f %.5f\n", iPt, P[0], P[1], P[2], N[0], N[1], N[2]);
			}

			fclose(pPtsFile);
#endif

			// Save acquired image as ply file
			//char *PLYFileName = RVLCreateString("generated_pc_from_view.ply");
			//PCLSavePLY(PLYFileName, meshBuilder.PCLMesh);

			// Image pre-processing.
			//view->Load(&mesh);
			view->InformationContnet();

			// Initialize unexplored regions in current scene
			view->projectionCube.CreateProjectionCube(cubeHalfSizeIn, CosAngle_in, distance_threshold_in);		//Initalizes projection cube by creating RCube

			localizer.Views.push_back(view);

			localizer.UnexploredRegions(&localizer.cube, view->cameraView.R, view->cameraView.t);
	
#ifdef RVLLOCALIZATION_DEMO_LOG_FILE
			//Save unexplored regions to file.
			std::string resultsFolderName = std::string(resultsFolder);
			FILE *fpUnexplored = fopen((resultsFolderName + "\\unexplored.txt").data(), "w");			

			for (int iCubeSide = 0; iCubeSide < 6; iCubeSide++)
				for (int i = 0; i < cubeSize; i++)
				{
					for (int j = 0; j < cubeSize; j++)
						fprintf(fpUnexplored, "%d\t", localizer.cube.cubeMatrix[iCubeSide][i * cubeSize + j]);

					fprintf(fpUnexplored, "\n");
				}
			fclose(fpUnexplored);
#endif

			// Create list with the next view direction candidates
			Array<Pair<int, int>> viewDirectionCandidates;
			viewDirectionCandidates.Element = new Pair<int, int>[6 * cubeSize * cubeSize];
			viewDirectionCandidates.n = 0;

			for (int iCubeSide = 0; iCubeSide < 6; iCubeSide++)
			{
				for (int cellIndex = 0; cellIndex < cubeSize * cubeSize; cellIndex++)
				{
					if (localizer.cube.cubeMatrix[iCubeSide][cellIndex] == 1)
					{
						viewDirectionCandidates.Element[viewDirectionCandidates.n].a = iCubeSide;
						viewDirectionCandidates.Element[viewDirectionCandidates.n].b = cellIndex;
						viewDirectionCandidates.n++;
					}
				}
			}
							
			// Compute parameters of unexplored regions in current scene for every potential candidate	

			float bestPanTilt[2];
			
			//Compute next best view
			localizer.CalculateScore(viewDirectionCandidates, &localizer.cube, cubeSize, bestPanTilt);

			dPan = bestPanTilt[0];
			dTilt = bestPanTilt[1];

			//dPan = 0.0f;
			//dTilt = 30.0f;

			// Project current image onto cube
			// localizer.GetCenteralPoints(view);
			// view->projectionCube.ProjectCentralPoints(view->centralPoints_array, view->cameraView.R, view->cameraView.t);

			

			//Move camera

			//...Ako je dosta slika, prebaci bActiveVision = false ??

			if (localizer.Views.size() == noGeneratedViews)
			{
				/*localizer.ImagesSampling();
				localizer.DataAssociation();*/
				localizer.MultiViewRegistration(noIterations);

#ifdef RVLLOCALIZATION_DEMO_LOG_RESULTS

				for (int iView = 0; iView < localizer.Views.size(); iView++)
				{

					FILE *fpT = fopen((std::string(resultsFolder) + "\\R_corrected.txt").data(), "a");

					PrintMatrix<float>(fpT, localizer.Views[iView]->cameraView.R, 3, 3);

					fclose(fpT);

					fpT = fopen((std::string(resultsFolder) + "\\t_corrected.txt").data(), "a");

					PrintMatrix<float>(fpT, localizer.Views[iView]->cameraView.t, 3, 1);

					fclose(fpT);
				}
#endif

				break;
			}
				


			delete[] viewDirectionCandidates.Element;
		}

#ifdef RVLLOCALIZATION_DEMO_LOG_FILE
		FILE *pCrspFile = fopen((std::string(resultsFolder) + "\\sl-crsp.txt").data(), "w");

		int kView, lView, l, iCrsp;
		View *view;
		Pair<int, int> crsp;

		for (kView = 0; kView < localizer.Views.size(); kView++)
		{
			view = localizer.Views[kView];

			for (l = 0; l < view->overlapingViewList.size(); l++)
			{
				lView = view->overlapingViewList[l];

				for (iCrsp = 0; iCrsp < view->correspodance_list[l].size(); iCrsp++)
				{
					crsp = view->correspodance_list[l][iCrsp];

					//std::vector<std::vector<Pair<int, int>>> correspodance_list;

					fprintf(pCrspFile, "%d %d %d %d\n", kView, crsp.a, lView, crsp.b);
				}
			}
		}

		fclose(pCrspFile);
#endif
	}
	else	// if bActiveVision = false:
	{

		while (sceneSequence.GetNextPath(filePath))
		{
			// View registration.
			View *view = NULL;
			view = new View;

			printf("Scene: %s:\n", filePath);

			// Load mesh.
			LoadMesh(&meshBuilder, filePath, &(view->mesh), true);

			view->cameraView.LoadCameraFile(filePath);

			view->projectionCube.CreateProjectionCube(cubeHalfSizeIn, CosAngle_in, distance_threshold_in);	//Initalizes projection cube by creating RCube	
			view->projectionCube.ProjectOnCube(&(view->mesh), view->cameraView.R, view->cameraView.t);
			
			//view->projectionCube.ProjectOnCube(&(view->mesh), view->cameraView.R, view->cameraView.t, filePath);

			
			//views.push_back(view);
			localizer.Views.push_back(view);


			// Reset memory.
			mem.Clear();


			// Localization.
			//Mateja:
			//localizer.UpdateMap(&mesh, &(cameraViews[iView++]), filePath); //v1 - SUNCG

			//localizer.UpdateMap(&(view->mesh), view->cameraView, filePath);

			//Cupec
			//localizer.UpdateMap(&mesh);


			//delete view;
		}

		View *mergedView = NULL;
		mergedView = new View;

		//if (bVisualizer)
		//	visualizer.Run();

		// ICP 
		int noIterations = 10;
		int noViews = views.size();
		float X[24]; // CHANGE!!

		std::vector<Pair<int, int>> correspondences;

		for (int iView = 0; iView < noViews; iView++)
			for (int jView = 0; jView < noViews; jView++)
				views[iView]->correspodance_list.push_back(correspondences);

		float E, E_new;
		float expectedCostChange;
		float costChange;

		for (int iteration = 0; iteration < noIterations; iteration++)
		{
			printf("Iteration: %d\n", iteration + 1);

			for (int iView = 0; iView < noViews; iView++)
			{
				views[iView]->projectionCube.ProjectOnCube(&(views[iView]->mesh), views[iView]->cameraView.R, views[iView]->cameraView.t);
				//views[iView]->projectionCube.ProjectOnCube(&(views[iView]->mesh), views[iView]->cameraView.R, views[iView]->cameraView.t, filePath);

				if (bVisualizer)
				{
					unsigned char color[3];
					//RVLSET3VECTOR(color, 0, 255, 0);
					RandomColor(color);
					visualizer.DisplayPointSet<float, Point>(views[iView]->projectionCube.transformedPoints, color, 4.0f);
				}
			}

			E = MultiPointCloudReg(views, X, &visualizer);
			MultiPointCloudCorrection(views, X);

			// Visualize correspondences
			for (int iView = 0; iView < noViews; iView++)
			{
				for (int jView = iView + 1; jView < noViews; jView++)
				{
					VisualizeCorrespondences(&visualizer, views[iView]->projectionCube.transformedPoints, views[jView]->projectionCube.transformedPoints, views[iView]->correspodance_list[jView]);
				}
			}

			E_new = MultiPointCloudToPlaneError(views);
			costChange = E_new - E;
			printf("Cost change = %.4f\n\n", costChange);


			//for (int iView = 0; iView < noViews; iView++)
			//{
			//	for (int jView = 0; jView < noViews; jView++)
			//	{
			//		views[iView]->correspodance_list[jView].clear();
			//	}
			//}
			// obrisati korespodencije za novo racunanje!



			//unsigned char color[3];
			//RandomColor(color);
			//visualizer.DisplayPointSet<float, Point>(views[0]->projectionCube.transformedPoints, color, 4.0f);

			if (bVisualizer)
			{
				visualizer.Run();

				visualizer.renderer->RemoveAllViewProps();
			}
		}
	}

	// Localizer -> CreateMesh


	///
	
	// Free memory.

	RVL_DELETE_ARRAY(sceneSequenceFileName);
	RVL_DELETE_ARRAY(resultsFolder);

	//#endif

	return 0;
}







