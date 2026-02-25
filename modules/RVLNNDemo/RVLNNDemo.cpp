#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <vtkOBJReader.h>
#include "RVLCore2.h"
#ifndef RVLLINUX
#include <windows.h>
#endif
#ifdef RVLVTK
#include "RVLVTK.h"
#endif
#include "Util.h"
#include "RVL2DCellArray.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "BoxSampler.h"
#include "Corruptor.h"
#include "Extractor.h"

using namespace RVL;

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pSceneSequenceFileName,
	char ** pTargetDirectory,
	bool &bScenesInOwnDirectories,
	float &zn,
	float &zf,
	bool & bVisualize,
	bool &bVisualizeBoxes,
	bool &bBoxSampler,
	bool &bBoxFeatureFromWholeImage,
	bool &bBoxFeatureCorruptedInput,
	char **pDepthFileName,
	char **pSourceDepthFileName,
	bool &bCorruptor,
	bool &bGenerateCorruptedImage,
	bool &bCreateHPM,
	bool &bSaveHPM,
	bool &bBoundingBox,
	bool &bExtractor,
	int &targetObjIdx)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;
	
	pParamList->Init();

	pParamData = pParamList->AddParam("SceneSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceFileName);
	pParamData = pParamList->AddParam("TargetDirectory", RVLPARAM_TYPE_STRING, pTargetDirectory);
	pParamData = pParamList->AddParam("ScenesInOwnDirectories", RVLPARAM_TYPE_BOOL, &bScenesInOwnDirectories);
	pParamData = pParamList->AddParam("clippingPlane.zn", RVLPARAM_TYPE_FLOAT, &zn);
	pParamData = pParamList->AddParam("clippingPlane.zf", RVLPARAM_TYPE_FLOAT, &zf);
	pParamData = pParamList->AddParam("visualize", RVLPARAM_TYPE_BOOL, &bVisualize);
	pParamData = pParamList->AddParam("visualizeBoxes", RVLPARAM_TYPE_BOOL, &bVisualizeBoxes);
	pParamData = pParamList->AddParam("BoxSampler", RVLPARAM_TYPE_BOOL, &bBoxSampler);
	pParamData = pParamList->AddParam("BoxSampler.wholeImage", RVLPARAM_TYPE_BOOL, &bBoxFeatureFromWholeImage);
	pParamData = pParamList->AddParam("BoxSampler.corruptedInput", RVLPARAM_TYPE_BOOL, &bBoxFeatureCorruptedInput);
	pParamData = pParamList->AddParam("BoxSampler.depthFileName", RVLPARAM_TYPE_STRING, pDepthFileName);
	pParamData = pParamList->AddParam("BoxSampler.sourceDepthFileName", RVLPARAM_TYPE_STRING, pSourceDepthFileName);
	pParamData = pParamList->AddParam("BoxSampler.corruptedInput", RVLPARAM_TYPE_BOOL, &bBoxFeatureCorruptedInput);	
	pParamData = pParamList->AddParam("BoxSampler.boundingBox", RVLPARAM_TYPE_BOOL, &bBoundingBox);
	pParamData = pParamList->AddParam("Corruptor", RVLPARAM_TYPE_BOOL, &bCorruptor);
	pParamData = pParamList->AddParam("Corruptor.generateCorruptedImage", RVLPARAM_TYPE_BOOL, &bGenerateCorruptedImage);
	pParamData = pParamList->AddParam("Corruptor.createHPM", RVLPARAM_TYPE_BOOL, &bCreateHPM);
	pParamData = pParamList->AddParam("Corruptor.saveHPM", RVLPARAM_TYPE_BOOL, &bSaveHPM);
	pParamData = pParamList->AddParam("Extractor", RVLPARAM_TYPE_BOOL, &bExtractor);
	pParamData = pParamList->AddParam("Extractor.targetObjIdx", RVLPARAM_TYPE_INT, &targetObjIdx);	
}

int main(int argc, char ** argv)
{
	CRVLMem mem0;

	mem0.Create(10000000);

	CRVLMem mem;

	mem.Create(10000000);

	char cfgSelectionFileName[] = "RVLNNDemo.cfg";

	char* cfgFileName = ReadConfigurationFile(cfgSelectionFileName);

	if (cfgFileName == NULL)
		return 1;

	char *sceneSequenceFileName = NULL;
	char* targetDirectory = NULL;
	char *depthFileName = NULL;
	char *sourceDepthFileName = NULL;

	bool bScenesInOwnDirectories = true;
	bool bVisualize = false;
	bool bVisualizeBoxes = false;
	bool bBoxSampler = false;
	bool bBoxFeatureFromWholeImage = false;
	bool bBoxFeatureCorruptedInput = false;
	bool bCorruptor = false;
	bool bGenerateCorruptedImage = false;
	bool bCreateHPM = true;
	bool bSaveHPM = true;
	bool bBoundingBox = false;
	bool bExtractor = false;
	
	CRVLParameterList ParamList;
	float zn, zf;
	int targetObjIdx;

	CreateParamList(&ParamList,	&mem0,
		&sceneSequenceFileName,
		&targetDirectory,
		bScenesInOwnDirectories,
		zn,
		zf,
		bVisualize,
		bVisualizeBoxes,
		bBoxSampler,
		bBoxFeatureFromWholeImage,
		bBoxFeatureCorruptedInput,
		&depthFileName,
		&sourceDepthFileName,
		bCorruptor,
		bGenerateCorruptedImage,
		bCreateHPM,
		bSaveHPM,
		bBoundingBox,
		bExtractor,
		targetObjIdx);

	ParamList.LoadParams(cfgFileName);

	float zRange = zf - zn;

	if (sceneSequenceFileName == NULL)
	{
		printf("No scene specified!\n");

		return 1;
	}

	// Create BoxSampler.

	BoxSampler sampler;

	sampler.pMem0 = &mem0;
	sampler.pMem = &mem;

	sampler.Create(cfgFileName);

	int nGridCells = sampler.gridSize * sampler.gridSize;

	// Create Corruptor.

	Corruptor corruptor;

	corruptor.pMem0 = &mem0;

	corruptor.Create(cfgFileName);	

	// Create Extractor.

	Extractor extractor;

	extractor.pMem0 = &mem0;

	extractor.Create(cfgFileName);

	delete[] cfgFileName;

	// Main Loop.

	FileSequenceLoader sceneSequence;

	sceneSequence.Init(sceneSequenceFileName);

	RVL_DELETE_ARRAY(sceneSequenceFileName);

	char filePath[200];

//#ifdef RVLHDF5
//	const H5std_string	FILE_NAME("h5tutr_dset.h5");
//	const H5std_string	DATASET_NAME("dset");
//#endif

	while (sceneSequence.GetNextPath(filePath))
	{
		printf("Scene: %s:\n", filePath);

		std::string filePath_ = std::string(filePath);

		std::string outputFilePath;

		if (targetDirectory)
		{
			std::string sceneName = filePath_.substr(filePath_.rfind("\\"));
			outputFilePath = targetDirectory + sceneName;
#ifndef RVLLINUX
			CreateDirectory((char *)outputFilePath.data(), NULL);
#endif
		}
		else
			outputFilePath = (bScenesInOwnDirectories ? filePath_ : filePath_.substr(0, filePath_.rfind("\\")));

		mem.Clear();

#ifdef RVLHDF5
		// Read depth data from file.

		std::string fileName = (bScenesInOwnDirectories ? filePath_ + "\\" + std::string(depthFileName) + ".h5" : filePath_);

		FILE* fp = fopen((char*)fileName.data(), "r");

		if (fp == NULL)
			fileName = (bScenesInOwnDirectories ? filePath_ + "\\" + std::string(depthFileName) + ".png" : filePath_);
		else
			fclose(fp);

		double* depth;
		hsize_t dims[3];
		int nImages;
		int w, h, nPix;

		std::string fileExtension = fileName.substr(fileName.rfind(".") + 1);

		if (fileExtension == "h5")
		{
			ReadFromHDF5<double>(fileName, std::string("tensor"), 3, H5::PredType::NATIVE_DOUBLE, depth, dims);

			nImages = dims[0];
			h = dims[1];
			w = dims[2];

			nPix = w * h;
		}
		else
		{
			cv::Mat depthImageUINT16 = cv::imread(fileName, cv::IMREAD_UNCHANGED);

			nImages = 1;
			w = depthImageUINT16.cols;
			h = depthImageUINT16.rows;

			nPix = w * h;

			ushort* depthUINT16 = (ushort*)(depthImageUINT16.data);

			depth = new double[nPix];

			for (int iPix = 0; iPix < nPix; iPix++)
				depth[iPix] = (depthUINT16[iPix] > 0 ? ((double)depthUINT16[iPix] / 8000 - zn) / zRange : 1.0);

			dims[0] = nImages;
			dims[1] = h;
			dims[2] = w;

			WriteToHDF5<double>(std::string(outputFilePath) + "\\" + std::string(depthFileName) + ".h5", std::string("tensor"), 3, dims, H5::PredType::NATIVE_DOUBLE, depth);
		}

		// Read corrupted depth data from file.

		double* depth_s = NULL;

		if (bBoxFeatureCorruptedInput)
		{
			fileName = (bScenesInOwnDirectories ? std::string(filePath) + "\\" + std::string(sourceDepthFileName) + ".h5" : std::string(filePath));
		
			ReadFromHDF5<double>(fileName, std::string("tensor"), 3, H5::PredType::NATIVE_DOUBLE, depth_s, dims);
		}

		//

		Array2D<float> depthImage;

		depthImage.Element = new float[nPix];
		depthImage.w = w;
		depthImage.h = h;

		Array2D<float> depthImage_s;
		depthImage_s.Element = NULL;

		if (bBoxFeatureCorruptedInput)
		{
			depthImage_s.Element = new float[nPix];
			depthImage_s.w = w;
			depthImage_s.h = h;
		}	

		int keypointDataSize = (bBoxSampler ? (sampler.bVariableSize ? 4 : 3) : 3);
	
		Array3D<float>* featureTensorArray = NULL;
		Array3D<float>* featureTensorArray_s = NULL;
		float** keypointArrayArray = NULL;
		Array3D<float> featureTensor;
		Array3D<float> featureTensor_s;
		featureTensor.Element = NULL;
		featureTensor_s.Element = NULL;
		float* keypoints = NULL;
		int* featureBlockEndIdx = NULL;
		int totalNoFeatures = 0;
		double* bbox_data = NULL;
		double *view = NULL;

		Box<float> *pBbox = NULL;
		Box<float> bbox;

		if (bBoxSampler)
		{
			featureTensorArray = new Array3D<float>[nImages];
			keypointArrayArray = new float* [nImages];

			featureTensor.Element = new float[nGridCells * sampler.nSamples];

			keypoints = new float[keypointDataSize * sampler.nSamples];

			featureBlockEndIdx = new int[nImages];			

			if (bBoxFeatureCorruptedInput)
			{
				featureTensorArray_s = new Array3D<float>[nImages];
				featureTensor_s.Element = new float[nGridCells * sampler.nSamples];				
			}

			if (bBoundingBox)
			{
				fileName = (bScenesInOwnDirectories ? std::string(filePath) + "\\bbox.h5" : std::string(filePath));

				ReadFromHDF5<double>(fileName, std::string("tensor"), 2, H5::PredType::NATIVE_DOUBLE, bbox_data, dims);

				bbox.minx = bbox_data[0] * 1.05;
				bbox.maxx = bbox_data[1] * 1.05;
				bbox.miny = bbox_data[2] * 1.05;
				bbox.maxy = bbox_data[3] * 1.05;
				bbox.minz = bbox_data[4] * 1.05;
				bbox.maxz = bbox_data[5] * 1.05;

				pBbox = &bbox;

				fileName = (bScenesInOwnDirectories ? std::string(filePath) + "\\view3.h5" : std::string(filePath));

				ReadFromHDF5<double>(fileName, std::string("tensor"), 2, H5::PredType::NATIVE_DOUBLE, view, dims);
			}
		}

		float* HPMMem = NULL;
		Array2D<float> *pCorruptedImage = NULL;
		Array2D<float> corruptedImage;
		corruptedImage.Element = NULL;

		bool bCreateHPM_ = true;

		if (bCorruptor)
		{		
			// Read HPM from file.

			if (bCreateHPM)
				HPMMem = new float[nPix * nImages];
			else
			{
				fileName = std::string(filePath) + "\\hpm.h5";

				FILE* fp = fopen(fileName.data(), "r");

				if (fp)
				{
					ReadFromHDF5<float>(fileName, std::string("HPM"), 3, H5::PredType::NATIVE_FLOAT, HPMMem, dims);

					fclose(fp);

					bCreateHPM_ = false;
				}
				else
					HPMMem = new float[nPix * nImages];
			}

			// Allocate corrupted image.

			if (bGenerateCorruptedImage)
			{
				corruptedImage.Element = new float[nPix];
				pCorruptedImage = &corruptedImage;
			}
		}

		float RBC[9], tBC[3], bboxSize[3], gC[3];

		if (bExtractor)
		{
			// Read bounding box data from file.

			fileName = filePath_ + "\\" + "bbox" + std::to_string(targetObjIdx) + ".txt";

			fp = fopen((char*)fileName.data(), "r");

			for (int i = 0; i < 3; i++)
				fscanf(fp, "%f %f %f %f %f %f\n", RBC + 3 * i, RBC + 3 * i + 1, RBC + 3 * i + 2, tBC + i, bboxSize + i, gC + i);

			fclose(fp);
		}

		float boundingBoxR[9];
		float boundingBoxt[3];
		float* pBoundingBoxR;
		float* pBoundingBoxt;
		Array2D<float> HPM;
		int iPix, iImage;
		double* d, *d_s;

		for (iImage = 0; iImage < nImages; iImage++)
		{
			d = depth + iImage * nPix;

			for (iPix = 0; iPix < nPix; iPix++)
				depthImage.Element[iPix] = zn + zRange * (float)d[iPix];

			if (bBoxFeatureCorruptedInput)
			{
				d_s = depth_s + iImage * nPix;

				for (iPix = 0; iPix < nPix; iPix++)
					depthImage_s.Element[iPix] = zn + zRange * (float)d_s[iPix];
			}

			if (bBoxSampler)
			{
				if (bBoundingBox)
				{
					double* r = view + 7 * iImage;
					RVLQUATERNIONTOROT(r, boundingBoxR);
					pBoundingBoxR = boundingBoxR;
					double* t = view + 7 * iImage + 4;
					RVLCOPY3VECTOR(t, boundingBoxt);
					pBoundingBoxt = boundingBoxt;
				}
				else
					pBoundingBoxR = pBoundingBoxt = NULL;

				sampler.Apply(depthImage, featureTensor, keypoints, true, bBoxFeatureFromWholeImage, false, 0, pBbox, pBoundingBoxR, pBoundingBoxt);

				featureTensorArray[iImage].a = featureTensor.a;
				featureTensorArray[iImage].b = featureTensor.b;
				featureTensorArray[iImage].c = featureTensor.c;
				featureTensorArray[iImage].Element = new float[nGridCells * featureTensor.c];
				memcpy(featureTensorArray[iImage].Element, featureTensor.Element, nGridCells * featureTensor.c * sizeof(float));

				keypointArrayArray[iImage] = new float[keypointDataSize * featureTensor.c];
				memcpy(keypointArrayArray[iImage], keypoints, keypointDataSize* featureTensor.c * sizeof(float));

				if (bBoxFeatureCorruptedInput)
				{
					sampler.Apply(depthImage_s, featureTensor_s, keypoints, true, false, true, featureTensorArray[iImage].c);

					featureTensorArray_s[iImage].a = featureTensor_s.a;
					featureTensorArray_s[iImage].b = featureTensor_s.b;
					featureTensorArray_s[iImage].c = featureTensor_s.c;
					featureTensorArray_s[iImage].Element = new float[nGridCells * featureTensor_s.c];
					memcpy(featureTensorArray_s[iImage].Element, featureTensor_s.Element, nGridCells * featureTensor_s.c * sizeof(float));
				}					

				if (bVisualize)
					if(iImage == 0)
						sampler.Visualize(depthImage, featureTensor, keypoints, false, (bBoxFeatureCorruptedInput ? &featureTensor_s : NULL));

				totalNoFeatures += featureTensor.c;

				featureBlockEndIdx[iImage] = totalNoFeatures;
			}

			if (bCorruptor)
			{
				HPM.Element = HPMMem + nPix * iImage;

				corruptor.Apply(depthImage, HPM, pCorruptedImage, bCreateHPM_);

				// Save HPM to file.

				if (bSaveHPM)
				{
					hsize_t HPMDim[3];
					HPMDim[0] = nImages;
					HPMDim[1] = h;
					HPMDim[2] = w;

					WriteToHDF5<float>(std::string(filePath) + "\\hpm.h5", std::string("HPM"), 3, HPMDim, H5::PredType::NATIVE_FLOAT, HPMMem);
				}

				//

				if (bVisualize)
				{
					Array2D<float> displayImg;
					displayImg.w = depthImage.w;
					displayImg.h = depthImage.h;
					displayImg.Element = pCorruptedImage->Element;
					cv::Mat corruptedDisplayImg;
					CreateGrayScaleImage(displayImg, corruptedDisplayImg);
					cv::imshow("Corrupted depth image", corruptedDisplayImg);

					cv::Mat depthImageGS;
					CreateGrayScaleImage(depthImage, depthImageGS);
					cv::imshow("depth", depthImageGS);

					corruptor.Visualize(HPM);
				}
			}

			if (bExtractor)
			{
				Array2D<float> tgtImage;
				tgtImage.w = extractor.tgtImgWidth;
				tgtImage.h = extractor.tgtImgHeight;
				tgtImage.Element = new float[tgtImage.w * tgtImage.h];

				extractor.Apply(depthImage, RBC, tBC, gC, bboxSize, tgtImage);

				if (bVisualize)
				{
					cv::Mat displayImg;
					CreateGrayScaleImage(depthImage, displayImg);
					cv::imshow("Source depth image", displayImg);

					extractor.Visualize(tgtImage);
				}
			}
		}

		if (bBoxSampler)
		{
			delete[] featureTensor.Element;

			featureTensor.Element = new float[nGridCells * totalNoFeatures];

			delete[] keypoints;

			keypoints = new float[keypointDataSize * totalNoFeatures];

			float* featuresPtr = featureTensor.Element;
			float* keypointsPtr = keypoints;
			int nFeatures;

			for (iImage = 0; iImage < nImages; iImage++)
			{
				nFeatures = featureTensorArray[iImage].c;
				memcpy(featuresPtr, featureTensorArray[iImage].Element, nGridCells * nFeatures * sizeof(float));
				featuresPtr += (nGridCells * nFeatures);
				delete[] featureTensorArray[iImage].Element;
				memcpy(keypointsPtr, keypointArrayArray[iImage], keypointDataSize* nFeatures * sizeof(float));
				keypointsPtr += (keypointDataSize * nFeatures);
				delete[] keypointArrayArray[iImage];
			}

			hsize_t featureTensorDim[3];
			featureTensorDim[0] = totalNoFeatures;
			featureTensorDim[1] = featureTensor.b;
			featureTensorDim[2] = featureTensor.a;

			WriteToHDF5<float>(std::string(outputFilePath) + "\\feature.h5", std::string("featureTensor"), 3, featureTensorDim, H5::PredType::NATIVE_FLOAT, featureTensor.Element);

			hsize_t keypointsDim[2];
			keypointsDim[0] = totalNoFeatures;
			keypointsDim[1] = keypointDataSize;

			WriteToHDF5<float>(std::string(outputFilePath) + "\\keypoints.h5", std::string("keypoints"), 2, keypointsDim, H5::PredType::NATIVE_FLOAT, keypoints);

			hsize_t idxDim[1];
			idxDim[0] = nImages;

			WriteToHDF5<int>(std::string(outputFilePath) + "\\idx.h5", std::string("idx"), 1, idxDim, H5::PredType::NATIVE_INT, featureBlockEndIdx);

			if (bBoxFeatureCorruptedInput)
			{
				delete[] featureTensor_s.Element;

				featureTensor_s.Element = new float[nGridCells * totalNoFeatures];

				featuresPtr = featureTensor_s.Element;

				for (iImage = 0; iImage < nImages; iImage++)
				{
					nFeatures = featureTensorArray_s[iImage].c;
					memcpy(featuresPtr, featureTensorArray_s[iImage].Element, nGridCells * nFeatures * sizeof(float));
					featuresPtr += (nGridCells * nFeatures);
					delete[] featureTensorArray_s[iImage].Element;
				}

				featureTensorDim[3];
				featureTensorDim[0] = totalNoFeatures;
				featureTensorDim[1] = featureTensor_s.b;
				featureTensorDim[2] = featureTensor_s.a;

				WriteToHDF5<float>(std::string(outputFilePath) + "\\feature_s.h5", std::string("featureTensor"), 3, featureTensorDim, H5::PredType::NATIVE_FLOAT, featureTensor_s.Element);
			}
		}

		delete[] depth;
		delete[] depth_s;
		delete[] depthImage.Element;
		RVL_DELETE_ARRAY(depthImage_s.Element);
		RVL_DELETE_ARRAY(bbox_data);
		RVL_DELETE_ARRAY(view);
		RVL_DELETE_ARRAY(featureTensor.Element);
		RVL_DELETE_ARRAY(featureTensor_s.Element);
		RVL_DELETE_ARRAY(keypoints);
		RVL_DELETE_ARRAY(featureTensorArray);
		RVL_DELETE_ARRAY(featureTensorArray_s);
		RVL_DELETE_ARRAY(keypointArrayArray);
		RVL_DELETE_ARRAY(featureBlockEndIdx);
		RVL_DELETE_ARRAY(HPMMem);
		RVL_DELETE_ARRAY(corruptedImage.Element);
#endif
	}

	RVL_DELETE_ARRAY(depthFileName);
	RVL_DELETE_ARRAY(sourceDepthFileName);
	RVL_DELETE_ARRAY(targetDirectory);

	return 0;
}

