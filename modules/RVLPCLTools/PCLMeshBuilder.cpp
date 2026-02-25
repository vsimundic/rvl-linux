//#include "stdafx.h"
//#include "RVLCore2.h"

#include "PCLPointCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/normal_3d_omp.h>
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include <vtkSmoothPolyDataFilter.h>
#include "Graph.h"
#include "Mesh.h"
#include "RGBDCamera.h"
#include "PCLTools.h"
#include "PCLMeshBuilder.h"


using namespace RVL;

PCLMeshBuilder::PCLMeshBuilder()
{
	sigmaS = 5.0f;
	sigmaR = 0.05f;	
	normalEstR = 0.010f;
	organizedFastMeshMaxEdgeLen = -1.0f;
	voxelSize = 0.01f;
	nSDFFilter = 5;
	SDFIsoValue = 0.0f;
	flags = 0x00000000;

	vpBilateralFilter = new pcl::FastBilateralFilter<pcl::PointXYZRGBA>;
	vpNormalEstimator = new pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal>;
	vpOFM = new pcl::OrganizedFastMesh<pcl::PointXYZRGBA>;
}


PCLMeshBuilder::~PCLMeshBuilder()
{
	delete ((pcl::FastBilateralFilter<pcl::PointXYZRGBA> *)vpBilateralFilter);
	delete ((pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> *)vpNormalEstimator);
	delete ((pcl::OrganizedFastMesh<pcl::PointXYZRGBA> *)vpOFM);
}

void PCLMeshBuilder::Create(
	char *cfgFileName,
	CRVLMem *pMem)
{
	CreateParamList(pMem);

	ParamList.LoadParams(cfgFileName);

	camera.CreateParamList(pMem);

	camera.paramList.LoadParams(cfgFileName);

	camera.depthFu0 = camera.depthFu;
	camera.depthFv0 = camera.depthFv;
	camera.depthUc0 = camera.depthUc;
	camera.depthVc0 = camera.depthVc;
	width = camera.w;
	height = camera.h;

	camera.kappa = camera.depthFu * camera.baseLength / camera.zMin;
}

void PCLMeshBuilder::CreateMesh(
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
	pcl::PolygonMesh &mesh,
	bool bUVDspace)
{
	if (flags & RVLPCLMESHBUILDER_FLAG_ORGANIZED_PC)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC_;

		int i;

		// Bilateral filtering

		if (flags & RVLPCLMESHBUILDER_FLAG_BILATERAL_FILTER)
		{
			pcl::FastBilateralFilter<pcl::PointXYZRGBA> *pBilateralFilter = (pcl::FastBilateralFilter<pcl::PointXYZRGBA> *)vpBilateralFilter;

			pBilateralFilter->setSigmaS((float)sigmaS);
			pBilateralFilter->setSigmaR((float)sigmaR);

			pBilateralFilter->setInputCloud(PC);

			pBilateralFilter->applyFilter(FPC);

			PC_ = { boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>(FPC) };
		}
		else
			PC_ = PC;

		// Compute normals

		pcl::search::OrganizedNeighbor<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGBA>());

		pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> *pNormalEstimator = (pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> *)vpNormalEstimator;

		pNormalEstimator->setRadiusSearch((float)normalEstR);
		pNormalEstimator->setInputCloud(PC_);
		pNormalEstimator->setSearchMethod(tree);
		pNormalEstimator->compute(N);

		// HRZICA 
		if (bUVDspace)	// NEW
		{		
			for (int iPt = 0; iPt < PC_->points.size(); iPt++)
									   
			{
				float pointPCL[3];
				pointPCL[0] = PC_->points[iPt].x;
				pointPCL[1] = PC_->points[iPt].y;
				pointPCL[2] = PC_->points[iPt].z;								

				if (std::isfinite(N.points[iPt].normal_x))
				{
					float normalPCL[3];
					normalPCL[0] = N.points[iPt].normal_x;
					normalPCL[1] = N.points[iPt].normal_y;
					normalPCL[2] = N.points[iPt].normal_z;

					convertPCLnormalFromUVDtoXYZspace(normalPCL, pointPCL);

					N.points[iPt].normal_x = normalPCL[0];
					N.points[iPt].normal_y = normalPCL[1];
					N.points[iPt].normal_z = normalPCL[2];

				}
				else
				{
					N.points[iPt].normal_x = 0.0f;
					N.points[iPt].normal_y = 0.0f;
					N.points[iPt].normal_z = 0.0f;
					N.points[iPt].curvature = 0.0f;					
				}

				convertPCLpointFromUVDtoXYZspace(pointPCL);

				PC_->points[iPt].x = pointPCL[0];
				PC_->points[iPt].y = pointPCL[1];
				PC_->points[iPt].z = pointPCL[2];
			}
		}
		else // OLD
		{
			for (i = 0; i < N.points.size(); i++)
			{
				if (!std::isfinite(N.points[i].normal_x))
				{
					N.points[i].normal_x = 0.0f;
					N.points[i].normal_y = 0.0f;
					N.points[i].normal_z = 0.0f;
					N.points[i].curvature = 0.0f;
				}
			}
		}
		// Create OrganizedFastMesh

		pcl::OrganizedFastMesh<pcl::PointXYZRGBA> *pOFM = (pcl::OrganizedFastMesh<pcl::PointXYZRGBA> *)vpOFM;

		pOFM->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGBA>::TRIANGLE_ADAPTIVE_CUT);

		pOFM->setInputCloud(PC_);

		if (organizedFastMeshMaxEdgeLen > 0.0f)
			pOFM->setMaxEdgeLength(organizedFastMeshMaxEdgeLen);

		pOFM->reconstruct(mesh);	// Conditions for adding a triangle are defined in function isShadowed in organized_fast_mesh.h.
		// This function is applied to endpoints of every triangle edge.
		// The meaning of most of the parameters of OrganizedFastMesh method can be understood from the code of this function.

		// Add normals to mesh

		//pcl::toPCLPointCloud2(*N, N2);
		pcl::toPCLPointCloud2(N, N2);
		pcl::concatenateFields(N2, mesh.cloud, aux);
		mesh.cloud = aux;
	}
	else
	{
		pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> *pNormalEstimator = (pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> *)vpNormalEstimator;

		pNormalEstimator->setRadiusSearch((float)normalEstR);
		pNormalEstimator->setInputCloud(PC);

		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());

		pNormalEstimator->setSearchMethod(tree);
		pNormalEstimator->compute(N);

		pcl::PCLPointCloud2 PCN2;
		pcl::toPCLPointCloud2(N, N2);
		pcl::PCLPointCloud2 PC2;
		pcl::toPCLPointCloud2(*PC, PC2);
		pcl::concatenateFields(PC2, N2, PCN2);

		pcl::PointCloud<pcl::PointNormal>::Ptr PCN(new pcl::PointCloud<pcl::PointNormal>());

		pcl::fromPCLPointCloud2(PCN2, *PCN);

		//print_info("Using parameters: depth %d, solverDivide %d, isoDivide %d\n", depth, solver_divide, iso_divide);

		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setDepth(8);
		poisson.setSolverDivide(8);
		poisson.setIsoDivide(8);
		poisson.setPointWeight(4.0f);
		poisson.setInputCloud(PCN);

		poisson.reconstruct(mesh);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr meshPC(new pcl::PointCloud<pcl::PointXYZRGBA>());

		pcl::fromPCLPointCloud2(mesh.cloud, *meshPC);

		pNormalEstimator->setInputCloud(meshPC);
		pNormalEstimator->compute(N);
		pcl::toPCLPointCloud2(N, N2);

		pcl::concatenateFields(N2, mesh.cloud, aux);
		mesh.cloud = aux;
	}

	///
}

bool PCLMeshBuilder::CreateMesh(
	vtkSmartPointer<vtkPolyData> pPolygonData,
	pcl::PolygonMesh &mesh)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("Colors"));
	if (rgbPointData == NULL)
	{
		rgbPointData = rgbPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("RGB"));

		if (rgbPointData)
			rgbPointData->SetName("Colors");
		else
			return false;
	}

	pcl::VTKUtils::vtk2mesh(pPolygonData, mesh);

	vtkSmartPointer<vtkFloatArray> normalPointData = normalPointData->SafeDownCast(pPolygonData->GetPointData()->GetArray("Normals"));
	if (normalPointData == NULL)
	{
		normalPointData = normalPointData->SafeDownCast(pPolygonData->GetPointData()->GetNormals());

		if (normalPointData == NULL)
			return false;
	}

	int nPts = pPolygonData->GetNumberOfPoints();

	nPts = normalPointData->GetNumberOfTuples();

	N.resize(nPts);

	float N_[3];
	
	for (int i = 0; i < nPts; i++)
	{
		normalPointData->GetTypedTuple(i, N_);

		N.points[i].normal_x = N_[0];
		N.points[i].normal_y = N_[1];
		N.points[i].normal_z = N_[2];
	}

	pcl::toPCLPointCloud2(N, N2);
	pcl::concatenateFields(N2, mesh.cloud, aux);
	mesh.cloud = aux;

	return true;
}

void PCLMeshBuilder::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("MeshBuilder.sigmaS", RVLPARAM_TYPE_DOUBLE, &sigmaS);
	pParamData = ParamList.AddParam("MeshBuilder.sigmaR", RVLPARAM_TYPE_DOUBLE, &sigmaR);
	pParamData = ParamList.AddParam("MeshBuilder.normalEstR", RVLPARAM_TYPE_DOUBLE, &normalEstR);
	pParamData = ParamList.AddParam("MeshBuilder.organizedFastMesh.maxEdgeLen", RVLPARAM_TYPE_FLOAT, &organizedFastMeshMaxEdgeLen);
	pParamData = ParamList.AddParam("MeshBuilder.bilateralFilter", RVLPARAM_TYPE_FLAG, &flags);
	ParamList.AddID(pParamData, "yes", RVLPCLMESHBUILDER_FLAG_BILATERAL_FILTER);
	pParamData = ParamList.AddParam("MeshBuilder.organizedPC", RVLPARAM_TYPE_FLAG, &flags);
	ParamList.AddID(pParamData, "yes", RVLPCLMESHBUILDER_FLAG_ORGANIZED_PC);
	pParamData = ParamList.AddParam("MeshBuilder.width", RVLPARAM_TYPE_INT, &width);
	pParamData = ParamList.AddParam("MeshBuilder.height", RVLPARAM_TYPE_INT, &height);
	pParamData = ParamList.AddParam("MeshBuilder.voxelSize", RVLPARAM_TYPE_FLOAT, &voxelSize);
	pParamData = ParamList.AddParam("MeshBuilder.nSDFFilter", RVLPARAM_TYPE_INT, &nSDFFilter);
	pParamData = ParamList.AddParam("MeshBuilder.SDFIsoValue", RVLPARAM_TYPE_FLOAT, &SDFIsoValue);
	pParamData = ParamList.AddParam("MeshBuilder.createOrganizedPC", RVLPARAM_TYPE_FLAG, &flags);
	ParamList.AddID(pParamData, "yes", RVLPCLMESHBUILDER_FLAG_CREATE_ORGANIZED_PC);
	pParamData = ParamList.AddParam("MeshBuilder.UVDSpace", RVLPARAM_TYPE_FLAG, &flags);
	ParamList.AddID(pParamData, "yes", RVLPCLMESHBUILDER_FLAG_UVD_SPACE);
	pParamData = ParamList.AddParam("MeshBuilder.smoothingFilterNumIterations", RVLPARAM_TYPE_INT, &smoothingFilterNumIterations);
}

bool PCLMeshBuilder::Load(
	char *RGBFileName,
	char *depthFileName,
	Mesh *pMesh,
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC_,
	pcl::PolygonMesh &PCLMesh_,
	bool bSavePLY,
	char *PLYFileNameIn)
{
	char *fileExtension = (strlen(RGBFileName) > 0 ? RVLGETFILEEXTENSION(RGBFileName) : RVLGETFILEEXTENSION(depthFileName));

	int *pixMap = NULL;
	int pixMapSize = 0;

	if (strcmp(fileExtension, "ply") == 0)
	{
		if (flags & RVLPCLMESHBUILDER_FLAG_VISIBLE_SURFACE)
		{
			vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
			reader->SetFileName(RGBFileName);
			reader->Update();
			vtkSmartPointer<vtkPolyData> pPolygonData = reader->GetOutput();

			printf("Creating visible surface mesh...");

			pMesh->pPolygonData = MESH::CreateVisibleSurfaceMesh(pPolygonData, voxelSize, nSDFFilter, SDFIsoValue);

			printf("completed.\n");

			pMesh->bOrganizedPC = false;

			char *PLYFileName = RVLCreateFileName(RGBFileName, ".ply", -1, "_vs.ply");

			printf("Saving mesh to %s...", PLYFileName);

			pMesh->SavePolyDataToPLY(PLYFileName);

			printf("completed.\n");

			delete[] PLYFileName;

			return false;
		}
		else
			pMesh->LoadPolyDataFromPLY(RGBFileName);

		/// Hack for the purpose of camera calibration. Remove after completing this research.

		//pcl::PointCloud<pcl::PointXYZRGBA> PC_;

		//N.clear();

		//PolygonDataToPCLPointCloudWithNormals(pMesh->pPolygonData, PC_, N);

		//PC_.width = N.width = width;
		//PC_.height = N.height = height;

		//CreateMesh(PC_);

		//char* PLYFileName = new char[strlen(RGBFileName) + 6];

		//strcpy(PLYFileName, RGBFileName);
		//	
		//sprintf(RVLGETFILEEXTENSION(PLYFileName), "mesh.ply");

		//PCLSavePLY(PLYFileName, PCLMesh);		

		//pMesh->LoadPolyDataFromPLY(PLYFileName);

		//delete[] PLYFileName;		

		///

		if (flags & RVLPCLMESHBUILDER_FLAG_CREATE_ORGANIZED_PC)
		{
			if (!CreatePixelMapFromPolyData(pMesh->pPolygonData, pixMap, pMesh->mapNodesToPolyData))
				printf("ERROR: Ambiguous point-to-image mapping!\n");

			pixMapSize = camera.w * camera.h;

			flags |= RVLPCLMESHBUILDER_FLAG_ORGANIZED_PC;
		}
	}
	else
	{
		if (strcmp(fileExtension, "pcd") == 0)
		{
			if (flags & RVLPCLMESHBUILDER_FLAG_CREATE_ORGANIZED_PC)
			{
				//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCSrc(new pcl::PointCloud<pcl::PointXYZRGBA>(width, height));

				//PCLLoadPCD(FileName, PCSrc);

				//CreateOrganizedPC(PCSrc, PC_);

				PCLLoadPCD(RGBFileName, PC_);

				RepareOrganizedPC(PC_);
			}
			else
				PCLLoadPCD(RGBFileName, PC_);
		}
		else if (strcmp(fileExtension, "bmp") == 0 || strcmp(fileExtension, "png") == 0)
		{
			IplImage* RGBImage = cvLoadImage(RGBFileName);

			Array2D<short int> depthImage;

			depthImage.Element = NULL;
			depthImage.w = depthImage.h = 0;

			char* depthFileName_ = RVLCreateString(depthFileName);

			sprintf(RVLGETFILEEXTENSION(depthFileName_), "txt");

			unsigned int format;

			if (ImportDisparityImage(depthFileName_, depthImage, format))
			{
				printf("Creating point cloud from RGB-D image.\n");

				camera.GetPointCloud(&depthImage, RGBImage, PC_, true, flags & RVLPCLMESHBUILDER_FLAG_UVD_SPACE);

				RVL_DELETE_ARRAY(depthImage.Element);
			}
			else
			{
				sprintf(RVLGETFILEEXTENSION(depthFileName_), "png");

				cv::Mat depthImagePNG = cv::imread(depthFileName_, CV_LOAD_IMAGE_ANYDEPTH);

				depthImage.w = depthImagePNG.cols;
				depthImage.h = depthImagePNG.rows;

				depthImage.Element = (short int*)(depthImagePNG.data);

				camera.GetPointCloud(&depthImage, RGBImage, PC_, true, flags & RVLPCLMESHBUILDER_FLAG_UVD_SPACE);
			}

			delete[] depthFileName_;

			cvReleaseImage(&RGBImage);
		}
#ifdef RVLHDF5
		else if (strcmp(fileExtension, "h5") == 0)
		{
			double* depth;
			hsize_t dims[3];

			ReadFromHDF5<double>(depthFileName, std::string("tensor"), 3, H5::PredType::NATIVE_DOUBLE, depth, dims);

			int nImages = dims[0];
			int h = dims[1];
			int w = dims[2];

			int nPix = w * h;

			Array2D<short int> depthImage;

			depthImage.Element = new short int[nPix];
			depthImage.w = w;
			depthImage.h = h;

			double zRange = camera.zf - camera.zn;

			int iPix;

			for (iPix = 0; iPix < nPix; iPix++)
				depthImage.Element[iPix] = (short int)round(1000.0 * (camera.zn + zRange * depth[iPix]));

			camera.GetPointCloud(&depthImage, NULL, PC_, true, flags & RVLPCLMESHBUILDER_FLAG_UVD_SPACE);

			RVL_DELETE_ARRAY(depthImage.Element);
		}
#endif
		else
		{
			printf("ERROR: Unknown file format!\n");

			return false;
		}

		printf("Creating organized PCL mesh from point cloud...");

		//FILE *fpDebug = fopen("C:\\RVL\\Debug\\P.txt", "w");

		//for (int i = 0; i < PC_->points.size(); i++)
		//	fprintf(fpDebug, "%f\t%f\t%f\n", PC_->points[i].x, PC_->points[i].y, PC_->points[i].z);

		//fclose(fpDebug);

		CreateMesh(PC_, PCLMesh_, flags & RVLPCLMESHBUILDER_FLAG_UVD_SPACE);

		printf("completed.\n");

		if (bSavePLY)
		{
			char* PLYFileName;

			if (PLYFileNameIn)
				PLYFileName = PLYFileNameIn;
			else
			{
				PLYFileName = RVLCreateString(RGBFileName);

				sprintf(RVLGETFILEEXTENSION(PLYFileName), "ply");
			}

			printf("Saving mesh to %s...", PLYFileName);

			PCLSavePLY(PLYFileName, PCLMesh_);

			printf("completed.\n");

			if (PLYFileNameIn == NULL)
				delete[] PLYFileName;

			return true;
		}

		vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
		PCLMeshToPolygonData(PCLMesh_, pd);
		pMesh->pPolygonData = vtkSmartPointer<vtkPolyData>::New();
		pMesh->pPolygonData->DeepCopy(pd);
	}

	if (flags & RVLPCLMESHBUILDER_FLAG_ORGANIZED_PC)
	{
		pMesh->bOrganizedPC = true;
		pMesh->width = width;
		pMesh->height = height;
	}
	else
		pMesh->bOrganizedPC = false;

	//commented 28.12.2017 - Vidovic (time measurement TPAMI18)
	//printf("Creating ordered mesh from PCL mesh...");

	if (smoothingFilterNumIterations > 0)
	{
		vtkSmartPointer<vtkSmoothPolyDataFilter> smoothingFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
		smoothingFilter->SetInputData(pMesh->pPolygonData);
		smoothingFilter->SetNumberOfIterations(smoothingFilterNumIterations);
		smoothingFilter->SetRelaxationFactor(1.0);
		smoothingFilter->Update();
		pMesh->pPolygonData = smoothingFilter->GetOutput();
	}

	pMesh->CreateOrderedMeshFromPolyData(pixMap, pixMapSize, organizedFastMeshMaxEdgeLen);

	if (pixMap)
		delete[] pixMap;

	//commented 28.12.2017 - Vidovic (time measurement TPAMI18)
	//printf("completed.\n");

	return true;
}

void PCLMeshBuilder::CreateOrganizedPC(
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCSrc,
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCTgt)
{
	int nPix = PCTgt->width * PCTgt->height;

	PCTgt->is_dense = false;

	float invalid = std::numeric_limits<float>::quiet_NaN();

	int iPixTgt;

	for (iPixTgt = 0; iPixTgt < nPix; iPixTgt++)
		PCTgt->points[iPixTgt].x = PCTgt->points[iPixTgt].y = PCTgt->points[iPixTgt].z = invalid;
		 
	int uTgt, vTgt, iPixSrc;

	for (iPixSrc = 0; iPixSrc < nPix; iPixSrc++)
		if (std::isfinite(PCSrc->points[iPixSrc].x))
		{
			uTgt = (int)round(camera.depthFu * PCSrc->points[iPixSrc].x / PCSrc->points[iPixSrc].z + camera.depthUc);
			vTgt = (int)round(camera.depthFv * PCSrc->points[iPixSrc].y / PCSrc->points[iPixSrc].z + camera.depthVc);
			iPixTgt = uTgt + vTgt * camera.w;
			PCTgt->points[iPixTgt] = PCSrc->points[iPixSrc];
		}
}

void PCLMeshBuilder::CreateMesh(
	Array2D<short int> depthImage_array,
	IplImage *pRGBImage,
	Mesh *pMesh,
	bool bConvertToMetres,
	bool bUVDSpace)
{
	//Compute normals
	printf("Creating point cloud from RGB-D image.\n");
	camera.GetPointCloud(&depthImage_array, pRGBImage, PC, bConvertToMetres, bUVDSpace);
	//RVL_DELETE_ARRAY(depthImage_aray.Element);

	printf("Creating organized PCL mesh from point cloud...");
	CreateMesh(PC, PCLMesh, bUVDSpace);

	printf("completed.\n");

	//char *PLYFileName = RVLCreateString("test_pc.ply");
	//PCLSavePLY(PLYFileName, PCLMesh);

	vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
	PCLMeshToPolygonData(PCLMesh, pd);
	pMesh->pPolygonData = vtkSmartPointer<vtkPolyData>::New();
	pMesh->pPolygonData->DeepCopy(pd);

	pMesh->bOrganizedPC = true;
	pMesh->width = depthImage_array.w;
	pMesh->height = depthImage_array.h;

	printf("Creating ordered mesh from PCL mesh...");

	pMesh->CreateOrderedMeshFromPolyData();

	printf("completed.\n");
}

void PCLMeshBuilder::CreateMesh(pcl::PointCloud<pcl::PointXYZRGBA> &PC_)
{
	pcl::OrganizedFastMesh<pcl::PointXYZRGBA>* pOFM = (pcl::OrganizedFastMesh<pcl::PointXYZRGBA>*)vpOFM;

	pOFM->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGBA>::TRIANGLE_ADAPTIVE_CUT);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC__ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>(PC_);

	pOFM->setInputCloud(PC__);
	pOFM->setMaxEdgeLength(5.0f);

	pOFM->reconstruct(PCLMesh);	// Conditions for adding a triangle are defined in function isShadowed in organized_fast_mesh.h.
	// This function is applied to endpoints of every triangle edge.
	// The meaning of most of the parameters of OrganizedFastMesh method can be understood from the code of this function.

	// Add normals to mesh

	pcl::toPCLPointCloud2(N, N2);
	pcl::concatenateFields(N2, PCLMesh.cloud, aux);
	PCLMesh.cloud = aux;

}

void PCLMeshBuilder::RepareOrganizedPC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC)
{
	int nPix = PC->width * PC->height;

	int iPix, u, v;

	for (iPix = 0; iPix < nPix; iPix++)
	{
		if (std::isfinite(PC->points[iPix].z))
		{
			u = iPix % camera.w;
			v = iPix / camera.w;

			PC->points[iPix].x = PC->points[iPix].z * ((float)u - camera.depthUc) / camera.depthFu;
			PC->points[iPix].y = PC->points[iPix].z * ((float)v - camera.depthVc) / camera.depthFv;
		}
	}
}

bool PCLMeshBuilder::CreatePixelMapFromPolyData(
	vtkSmartPointer<vtkPolyData> pPolygonData,
	int *&pixMap,
	int *&mapPixelsToPolyData)
{
	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pPolygonData->GetPoints()->GetData());
	if (pointData == NULL)
		return false;

	int noPts = pPolygonData->GetNumberOfPoints();

	if (pixMap == NULL)
		pixMap = new int[noPts];

	if (mapPixelsToPolyData == NULL)
	{
		mapPixelsToPolyData = new int[camera.w * camera.h];

		memset(mapPixelsToPolyData, 0xff, camera.w * camera.h * sizeof(int));
	}

	int iPt, u, v, iPix;
	float P[3];

	for (iPt = 0; iPt < noPts; iPt++)
	{
		pointData->GetTypedTuple(iPt, P);

		u = (int)round(camera.depthFu * P[0] / P[2] + camera.depthUc);
		v = (int)round(camera.depthFv * P[1] / P[2] + camera.depthVc);

		iPix = u + v * camera.w;

		pixMap[iPt] = iPix;

		if (mapPixelsToPolyData[iPix] >= 0)
			return false;

		mapPixelsToPolyData[iPix] = iPt;
	}

	return true;
}

void RVL::CreateMesh(
	void *vpMeshBuilder,
	Array2D<short int> *pDepthImage,
	IplImage *pRGBImage,
	Mesh *pMesh,
	bool bSavePLY,
	char *FileName)
{
	PCLMeshBuilder *pMeshBuilder = (PCLMeshBuilder *)vpMeshBuilder;

	pMeshBuilder->camera.GetPointCloud(pDepthImage, pRGBImage, pMeshBuilder->PC);

	pMeshBuilder->CreateMesh(pMeshBuilder->PC, pMeshBuilder->PCLMesh);

	if (bSavePLY)
	{
		char *PLYFileName = RVLCreateString(FileName);

		sprintf(RVLGETFILEEXTENSION(PLYFileName), "ply");

		printf("Saving mesh to %s...", PLYFileName);

		PCLSavePLY(PLYFileName, pMeshBuilder->PCLMesh);

		printf("completed.\n");

		delete[] PLYFileName;
	}

	vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
	PCLMeshToPolygonData(pMeshBuilder->PCLMesh, pd);
	pMesh->pPolygonData = vtkSmartPointer<vtkPolyData>::New();
	pMesh->pPolygonData->DeepCopy(pd);

	pMesh->bOrganizedPC = true;
	pMesh->width = pDepthImage->w;
	pMesh->height = pDepthImage->h;

	printf("Creating ordered mesh from PCL mesh...");

	pMesh->CreateOrderedMeshFromPolyData();

	printf("completed.\n");
}

bool RVL::LoadMesh(
	void *vpMeshBuilder,
	char *FileName,
	Mesh *pMesh,
	bool bSavePLY,
	char *PLYFileName,
	char *depthFileNameIn)
{
	PCLMeshBuilder *pMeshBuilder = (PCLMeshBuilder *)vpMeshBuilder;

	char *depthFileName = (depthFileNameIn ? depthFileNameIn : FileName);

	return pMeshBuilder->Load(FileName, depthFileName, pMesh, pMeshBuilder->PC, pMeshBuilder->PCLMesh, bSavePLY, PLYFileName);
}

// HRZICA
void PCLMeshBuilder::convertMeshfromUVDtoXYZspace(Mesh *pMesh)
{

	float pTemp[3];
	float nTemp[3];

	float fx = (float)camera.depthFu;					// focal lenght
	
	for (int iPoint = 0; iPoint < pMesh->NodeArray.n; iPoint++)
	{
		Point *q = pMesh->NodeArray.Element + iPoint;
		
		//if nije valid tockica -> skipaj
		if (!q->bValid)
			continue;

		// transform point coordinates from UVD to XYZ		
		float z = (float)camera.zMin / (1 - q->P[2] / (float)camera.kappa) *0.001;
		float z_fx = z / fx;

		pTemp[0] = z_fx * q->P[0];
		pTemp[1] = z_fx * q->P[1];
		pTemp[2] = z;

		// transform normals from UVD to XYZ
		float delta = RVLDOTPRODUCT3(q->N, q->P);		// delta = eta' * q

		float ux = q->N[0] * fx;						// eta_u * fx
		float vx = q->N[1] * fx;						// eta_v * fx
		float dx = q->N[2] * (float)camera.kappa - delta;				// eta_d * d0 - delta

		float weight = 1 / sqrt(ux * ux + vx * vx + dx * dx);

		nTemp[0] = weight * ux;
		nTemp[1] = weight * vx;
		nTemp[2] = weight * dx;
	
		// change mesh point from UVD to XYZ 
		RVLCOPY3VECTOR(pTemp, q->P);
		RVLCOPY3VECTOR(nTemp, q->N);		
	}
}


void PCLMeshBuilder::convertPolygonDatafromUVDtoXYZspace(Mesh *pMesh)
{
	int noPts = pMesh->pPolygonData->GetNumberOfPoints();
	vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(pMesh->pPolygonData->GetPoints()->GetData());

	vtkSmartPointer<vtkFloatArray> normalPointData = normalPointData->SafeDownCast(pMesh->pPolygonData->GetPointData()->GetArray("Normals"));

	if (normalPointData == NULL)
	{
		normalPointData = normalPointData->SafeDownCast(pMesh->pPolygonData->GetPointData()->GetNormals());
	}

	float pTemp[3];
	float nTemp[3];

	float fx = (float)camera.depthFu;					// focal lenght

	for (int iPt = 0; iPt < noPts; iPt++)
	{
		Point q;
		
		pointData->GetTypedTuple(iPt, q.P);
		normalPointData->GetTypedTuple(iPt, q.N);

		//if (!q.bValid)
		//	continue;

		// transform point coordinates from UVD to XYZ
		float z = (float)camera.zMin / (1 - q.P[2] / (float)camera.kappa) *0.001;		// convert from millimeters to meters
		float z_fx = z / fx;

		pTemp[0] = z_fx * q.P[0];
		pTemp[1] = z_fx * q.P[1];
		pTemp[2] = z;

		// transform normals from UVD to XYZ
		float delta = RVLDOTPRODUCT3(q.N, q.P);		// delta = eta' * q

		float ux = q.N[0] * fx;						// eta_u * fx
		float vx = q.N[1] * fx;						// eta_v * fx
		float dx = q.N[2] * (float)camera.kappa - delta;			// eta_d * d0 - delta

		float weight = 1 / sqrt(ux * ux + vx * vx + dx * dx);

		nTemp[0] = weight * ux;
		nTemp[1] = weight * vx;
		nTemp[2] = weight * dx;

		// change mesh point from UVD to XYZ 
		RVLCOPY3VECTOR(pTemp, q.P);
		RVLCOPY3VECTOR(nTemp, q.N);

		pointData->SetTypedTuple(iPt, q.P);
		normalPointData->SetTypedTuple(iPt, q.N);
	}
}

void PCLMeshBuilder::convertPCLnormalFromUVDtoXYZspace(float* normalUVDtoXYZ, float* pointUVDtoXYZ)
{
	float nTemp[3];

	float fx = (float)camera.depthFu;								// focal lenght

	float delta = RVLDOTPRODUCT3(normalUVDtoXYZ, pointUVDtoXYZ);	// delta = eta' * q

	float ux = normalUVDtoXYZ[0] * fx;								// eta_u * fx
	float vx = normalUVDtoXYZ[1] * fx;								// eta_v * fx
	float dx = normalUVDtoXYZ[2] * (float)camera.kappa - delta;		// eta_d * d0 - delta

	float weight = 1 / sqrt(ux * ux + vx * vx + dx * dx);

	nTemp[0] = weight * ux;
	nTemp[1] = weight * vx;
	nTemp[2] = weight * dx;

	// change mesh point from UVD to XYZ 
	RVLCOPY3VECTOR(nTemp, normalUVDtoXYZ);
}

void PCLMeshBuilder::convertPCLpointFromUVDtoXYZspace(float* pointUVDtoXYZ)
{		
	float pTemp[3];

	float z = camera.zMin / (1.0 - pointUVDtoXYZ[2] / (float)camera.kappa);
	float z_fx = z / (float)camera.depthFu;

	pTemp[0] = z_fx * pointUVDtoXYZ[0];
	pTemp[1] = z_fx * pointUVDtoXYZ[1];
	pTemp[2] = z;

	// change mesh point from UVD to XYZ 
	RVLCOPY3VECTOR(pTemp, pointUVDtoXYZ);
}


//END HRZICA
