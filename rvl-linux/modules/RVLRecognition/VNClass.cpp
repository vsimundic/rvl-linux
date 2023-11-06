//#include "stdafx.h"
#include "RVLCore2.h"
#ifndef RVLLINUX
#include <Windows.h>
#endif
#include <algorithm>
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "MSTree.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "ReconstructionEval.h"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "ObjectDetector.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
#include "NeighborhoodTool.h"
#include "VN.h"
#include "VNClass.h"
#include "VNInstance.h"
#include "VNClassifier.h"

using namespace RVL;
using namespace RECOG;

VNClass::VNClass()
{
	metamodels = NULL;
	nLabels = 0;
	componentID.Element = NULL;
	componentIdxMem = NULL;
	metamodelID.Element = NULL;
	//componentGauss = NULL;
	//componentGaussMem = NULL;
	metaModelGauss = NULL;
	metaModelGaussMem = NULL;
	gauss.Element = NULL;
	gaussMem = NULL;
}


VNClass::~VNClass()
{
	int iLabel, iMM;

	for (iLabel = 0; iLabel < nLabels; iLabel++)
	{
		for (iMM = 0; iMM < metamodels[iLabel].size(); iMM++)
			RVL_DELETE_ARRAY(metamodels[iLabel][iMM].conditionalProbability.Element);
	}

	RVL_DELETE_ARRAY(metamodels);
	RVL_DELETE_ARRAY(componentID.Element);
	RVL_DELETE_ARRAY(componentIdxMem);
	RVL_DELETE_ARRAY(metamodelID.Element);
	//RVL_DELETE_ARRAY(componentGauss);
	//RVL_DELETE_ARRAY(componentGaussMem);
	RVL_DELETE_ARRAY(metaModelGauss);
	RVL_DELETE_ARRAY(metaModelGaussMem);
	RVL_DELETE_ARRAY(gauss.Element);
	RVL_DELETE_ARRAY(gaussMem);
}

void VNClass::Save(void *vpClassifier, FILE *fp)
{
	VNClassifier *pVNClassifier = (VNClassifier *)vpClassifier;
	//fp = fopen((std::string(pVNClassifier->resultsFolder) + "\\VNClasses.dat").data(), "wb");

	int m = RVLMAX(pVNClassifier->classArray.Element[0].M.h, pVNClassifier->classArray.Element[1].M.h);

	fwrite(&m, sizeof(int), 1, fp);

	int iMMAbs = 0;

	VNClass *pVNClass = this;
	int nMM;
	VN_::ModelTemplate2 metamodel;

	fwrite(&nLabels, sizeof(int), 1, fp);

	fwrite(&metamodelID.n, sizeof(int), 1, fp);

	fwrite(&gauss.n, sizeof(int), 1, fp);

	for (int iLabel = 0; iLabel < nLabels; iLabel++)
	{
		nMM = metamodels[iLabel].size();
		fwrite(&nMM, sizeof(int), 1, fp);

		for (int iMM = 0; iMM < nMM; iMM++, iMMAbs++)
		{
			metamodel = metamodels[iLabel][iMM];

			fwrite(&metamodel.f, sizeof(int), 1, fp);
			fwrite(&metamodel.no, sizeof(int), 1, fp);
			fwrite(&metamodel.nu, sizeof(int), 1, fp);
			fwrite(&metamodel.label, sizeof(int), 1, fp);
			fwrite(&(metaModelGauss[iMMAbs].n), sizeof(int), 1, fp);
			fwrite(metaModelGauss[iMMAbs].Element, sizeof(int), metaModelGauss[iMMAbs].n, fp);
		}
	}

	int iGauss;

	for (iGauss = 0; iGauss < gauss.n; iGauss++)
	{
		fwrite(&(gauss.Element[iGauss].abs), sizeof(float), 1, fp);
		fwrite(&(gauss.Element[iGauss].w), sizeof(float), 1, fp);
	}

	fwrite(gaussMem, sizeof(float), gauss.n * m, fp);

	fclose(fp);
}


void VNClass::Load(void *vpClassifier, FILE *fp)
{
	VNClassifier *pVNClassifier = (VNClassifier *)vpClassifier;

	if (fp == NULL)
		return;

	int m;

	fread(&m, sizeof(int), 1, fp);

	int nMM;
	VN_::ModelTemplate2 metamodel;

	fread(&nLabels, sizeof(int), 1, fp);

	RVL_DELETE_ARRAY(metamodels);

	metamodels = new std::vector<VN_::ModelTemplate2>[nLabels];

	fread(&metamodelID.n, sizeof(int), 1, fp);

	RVL_DELETE_ARRAY(metamodelID.Element);

	metamodelID.Element = new Pair<int, int>[metamodelID.n];

	RVL_DELETE_ARRAY(metaModelGauss);

	metaModelGauss = new Array<int>[metamodelID.n];

	fread(&gauss.n, sizeof(int), 1, fp);

	metaModelGaussMem = new int[gauss.n];

	int *pMetaModelGaussIdx = metaModelGaussMem;

	componentID.n = 0;

	maxnGaussPerComponent = 0;

	int iMMAbs = 0;

	int iLabel, iMM;

	for (iLabel = 0; iLabel < nLabels; iLabel++)
	{
		fread(&nMM, sizeof(int), 1, fp);

		metamodelID.n += nMM;

		for (iMM = 0; iMM < nMM; iMM++, iMMAbs++)
		{
			metamodel.idx = iMMAbs;
			fread(&metamodel.f, sizeof(int), 1, fp);
			fread(&metamodel.no, sizeof(int), 1, fp);
			fread(&metamodel.nu, sizeof(int), 1, fp);
			fread(&metamodel.label, sizeof(int), 1, fp);
			fread(&(metaModelGauss[iMMAbs].n), sizeof(int), 1, fp);

			metaModelGauss[iMMAbs].Element = pMetaModelGaussIdx;

			fread(metaModelGauss[iMMAbs].Element, sizeof(int), metaModelGauss[iMMAbs].n, fp);

			pMetaModelGaussIdx += metaModelGauss[iMMAbs].n;

			metamodelID.Element[iMMAbs].a = iLabel;
			metamodelID.Element[iMMAbs].b = iMM;

			componentID.n += (metamodel.no + metamodel.nu);

			metamodel.conditionalProbability.Element = NULL;

			metamodels[iLabel].push_back(metamodel);
		}
	}

	RVL_DELETE_ARRAY(gauss.Element);

	gauss.Element = new VN_::Gauss[gauss.n];

	RVL_DELETE_ARRAY(gaussMem);

	gaussMem = new float[gauss.n * m];

	int iGauss;

	for (iGauss = 0; iGauss < gauss.n; iGauss++)
	{
		fread(&(gauss.Element[iGauss].abs), sizeof(float), 1, fp);
		fread(&(gauss.Element[iGauss].w), sizeof(float), 1, fp);
		gauss.Element[iGauss].x = gaussMem + iGauss * m;
	}

	fread(gaussMem, sizeof(float), gauss.n * m, fp);

	fclose(fp);

	RVL_DELETE_ARRAY(componentID.Element);

	componentID.Element = new VN_::PartAssociation[componentID.n];

	componentIdxMem = new int[componentID.n];

	int iCompAbs = 0;

	maxnMetamodelComponents = 0;

	int iComp, nComps;
	int *componentIdx;

	for (iLabel = 0; iLabel < nLabels; iLabel++)
	{
		nMM = metamodels[iLabel].size();

		for (iMM = 0; iMM < nMM; iMM++)
		{
			metamodel = metamodels[iLabel][iMM];

			nComps = metamodel.no + metamodel.nu;

			componentIdx = componentIdxMem + iCompAbs;

			for (iComp = 0; iComp < nComps; iComp++)
			{
				componentID.Element[iCompAbs].label = iLabel;
				componentID.Element[iCompAbs].iMetaModel = iMM;
				componentID.Element[iCompAbs].iComponent = iComp;

				componentIdx[iComp] = iCompAbs;

				iCompAbs++;
			}

			metamodels[iLabel][iMM].componentIdx = componentIdx;

			if (nComps > maxnMetamodelComponents)
				maxnMetamodelComponents = nComps;
		}
	}
}

