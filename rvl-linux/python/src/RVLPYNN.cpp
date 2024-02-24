#include "RVLCore2.h"
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
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

using namespace RVL;

void create_param_list(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pSceneSequenceFileName,
	bool &bScenesInOwnDirectories,
	float &zn,
	float &zf,
	bool & bVisualize,
	bool &bVisualizeBoxes,
	bool &bBoxSampler,
	bool &bCorruptor,
	bool &bGenerateCorruptedImage)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;
	
	pParamList->Init();

	pParamData = pParamList->AddParam("SceneSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceFileName);
	pParamData = pParamList->AddParam("ScenesInOwnDirectories", RVLPARAM_TYPE_BOOL, &bScenesInOwnDirectories);
	pParamData = pParamList->AddParam("clippingPlane.zn", RVLPARAM_TYPE_FLOAT, &zn);
	pParamData = pParamList->AddParam("clippingPlane.zf", RVLPARAM_TYPE_FLOAT, &zf);
	pParamData = pParamList->AddParam("visualize", RVLPARAM_TYPE_BOOL, &bVisualize);
	pParamData = pParamList->AddParam("visualizeBoxes", RVLPARAM_TYPE_BOOL, &bVisualizeBoxes);
	pParamData = pParamList->AddParam("BoxSampler", RVLPARAM_TYPE_BOOL, &bBoxSampler);
	pParamData = pParamList->AddParam("Corruptor", RVLPARAM_TYPE_BOOL, &bCorruptor);
	pParamData = pParamList->AddParam("Corruptor.generateCorruptedImage", RVLPARAM_TYPE_BOOL, &bGenerateCorruptedImage);
}

////////////////////////////////////////////////////////////////////
//
//     Corruptor
//
////////////////////////////////////////////////////////////////////

class RVLPYNNCorruptor
{
public:
	RVLPYNNCorruptor()
	{
		mem0.Create(10000000);

		corruptor.pMem0 = &mem0;
	}
	~RVLPYNNCorruptor() {};
	void create(std::string cfgFileName);
	py::tuple apply(py::array srcImage);
	float get_hole_size();

private:
	CRVLMem mem0;
	Corruptor corruptor;
};

void RVLPYNNCorruptor::create(std::string cfgFileName)
{
	corruptor.Create((char *)cfgFileName.data());
}

py::tuple RVLPYNNCorruptor::apply(py::array srcImage)
{	
	const ssize_t *imgSize = srcImage.shape();

	int nImages = (int)imgSize[0];
	int h = (int)imgSize[1];
	int w = (int)imgSize[2];
	int nPix = h * w;

	Array2D<float> srcImage_;
	srcImage_.h = h;
	srcImage_.w = w;
	float *srcImageMem = (float *)srcImage.request().ptr;	

	auto HPM = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		3,
		{nImages, h, w},
		{sizeof(float) * nPix, sizeof(float) * w, sizeof(float)}
	));	

	Array2D<float> HPM_;
	HPM_.h = h;
	HPM_.w = w;
	float *HPMMem = (float *)HPM.request().ptr;

	auto corruptedImage = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		3,
		{nImages, h, w},
		{sizeof(float) * nPix, sizeof(float) * w, sizeof(float)}
	));

	Array2D<float> corruptedImage_;
	corruptedImage_.h = h;
	corruptedImage_.w = w;
	float *corruptedImageMem = (float *)corruptedImage.request().ptr;

	int imageStart = 0;

	for (int iImage = 0; iImage < nImages; iImage++, imageStart += nPix)
	{
		srcImage_.Element = srcImageMem + imageStart;
		corruptedImage_.Element = corruptedImageMem + imageStart;
		HPM_.Element = HPMMem + imageStart;
		corruptor.Apply(srcImage_, HPM_, &corruptedImage_);
	}

	py::tuple result_tuple = py::make_tuple(HPM, corruptedImage);

	return result_tuple;	
}

float RVLPYNNCorruptor::get_hole_size()
{
	return corruptor.holeSize;
}

////////////////////////////////////////////////////////////////////
//
//     BoxSampler
//
////////////////////////////////////////////////////////////////////

class RVLPYNNBoxSampler
{
public:
	RVLPYNNBoxSampler()
	{
		mem0.Create(10000000);

		boxSampler.pMem0 = &mem0;
	}
	~RVLPYNNBoxSampler() {};
	void create(std::string cfgFileName);
	py::tuple apply(
		py::array srcImage,
		py::array C_in,
		bool keypoints,
		bool whole_image,
		bool imported_keypoints);
	void set_box_size(float box_size);
	float get_box_size();

private:
	CRVLMem mem0;
	BoxSampler boxSampler;	
};

void RVLPYNNBoxSampler::create(std::string cfgFileName)
{
	boxSampler.Create((char *)cfgFileName.data());
}

py::tuple RVLPYNNBoxSampler::apply(
	py::array srcImage,
	py::array C_in,
	bool keypoints,
	bool whole_image,
	bool imported_keypoints)
{
	const ssize_t *imgSize = srcImage.shape();

	int nImages = (int)imgSize[0];
	int h = (int)imgSize[1];
	int w = (int)imgSize[2];
	int nPix = h * w;

	Array2D<float> srcImage_;
	srcImage_.h = h;
	srcImage_.w = w;
	srcImage_.Element = (float *)srcImage.request().ptr;

	const ssize_t *C_in_size = C_in.shape();

	int num_imported_keypoints = (int)C_in_size[0];

	float *C_in_;

	if (whole_image | imported_keypoints)
		C_in_ = (float *)C_in.request().ptr;

	int nGridCells = boxSampler.gridSize * boxSampler.gridSize;

	auto output_tensor = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		3,
		{nImages * boxSampler.nSamples, boxSampler.gridSize, boxSampler.gridSize},
		{sizeof(float) * nGridCells, sizeof(float) * boxSampler.gridSize, sizeof(float)}
	));	

	Array3D<float> output_tensor_;
	output_tensor_.Element = (float *)output_tensor.request().ptr;

	auto num_features = py::array(py::buffer_info(
		nullptr,
		sizeof(int),
		py::format_descriptor<int>::value,
		1,
		{nImages},
		{sizeof(int)}
	));

	int *num_features_ = (int *)num_features.request().ptr;

	auto C = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{nImages * boxSampler.nSamples, 3},
		{3 * sizeof(float), sizeof(float)}
	));

	float *C_ = (float *)C.request().ptr;

	if (whole_image | imported_keypoints)
		memcpy(C_, C_in_, nImages * boxSampler.nSamples * 3 * sizeof(float));

	int outputTensorSize = boxSampler.nSamples * nGridCells;
	int keypointArraySize = boxSampler.nSamples * 3;

	for (int iImage = 0; iImage < nImages; iImage++, srcImage_.Element += nPix, output_tensor_.Element += outputTensorSize, C_ += keypointArraySize)
	{
		boxSampler.Apply(srcImage_, output_tensor_, C_, keypoints, whole_image, imported_keypoints, num_imported_keypoints);
		num_features_[iImage] = output_tensor_.c;
	}

	py::tuple result_tuple = py::make_tuple(output_tensor, num_features_, C);

	return result_tuple;	
}

void RVLPYNNBoxSampler::set_box_size(float box_size)
{
	boxSampler.boxSize = box_size;
}

float RVLPYNNBoxSampler::get_box_size()
{
	return boxSampler.boxSize;
}


////////////////////////////////////////////////////////////////////
//
//     Extractor
//
////////////////////////////////////////////////////////////////////

class RVLPYNNExtractor
{
public:
	RVLPYNNExtractor()
	{
		mem0.Create(10000000);

		extractor.pMem0 = &mem0;
	}
	~RVLPYNNExtractor() {};
	void create(std::string cfgFileName);
	py::tuple apply(
		py::array srcImage,
		py::array RBC,
		py::array tBC,
		py::array gC,
		py::array boxSize);
	float get_tgt_focal_length();
	py::tuple get_tgt_image_size();
	py::tuple get_tgt_clipping_planes();

private:
	CRVLMem mem0;
	Extractor extractor;		
};

void RVLPYNNExtractor::create(std::string cfgFileName)
{
	extractor.Create((char *)cfgFileName.data());
}

py::tuple RVLPYNNExtractor::apply(
	py::array srcImage,
	py::array RBC,
	py::array tBC,
	py::array gC,
	py::array boxSize)
{	
	const ssize_t *imgSize = srcImage.shape();

	int h = (int)imgSize[0];
	int w = (int)imgSize[1];
	int nPix = h * w;

	Array2D<float> srcImage_;
	srcImage_.h = h;
	srcImage_.w = w;
	srcImage_.Element = (float *)srcImage.request().ptr;

	float *RBC_ = (float *)RBC.request().ptr;
	float *tBC_ = (float *)tBC.request().ptr;
	float *gC_ = (float *)gC.request().ptr;
	float *boxSize_ = (float *)boxSize.request().ptr;

	auto tgtImage = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{extractor.tgtImgHeight, extractor.tgtImgWidth},
		{sizeof(float) * extractor.tgtImgWidth, sizeof(float)}
	));	

	Array2D<float> tgtImage_;
	tgtImage_.h = extractor.tgtImgHeight;
	tgtImage_.w = extractor.tgtImgWidth;
	float *tgtImageMem = (float *)tgtImage.request().ptr;

	tgtImage_.Element = tgtImageMem;

	Array2D<float> tgtPC_;
	tgtPC_.Element = NULL;

	auto RCO = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{3, 3},
		{sizeof(float) * 3, sizeof(float)}
	));

	float *RCO_ = (float *)RCO.request().ptr;

	auto tCO = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		1,
		{3},
		{sizeof(float)}
	));

	float *tCO_ = (float *)tCO.request().ptr;

	extractor.Apply(srcImage_, RBC_, tBC_, gC_, boxSize_, tgtImage_, tgtPC_, RCO_, tCO_);

	auto tgtPC = py::array(py::buffer_info(
		nullptr,
		sizeof(float),
		py::format_descriptor<float>::value,
		2,
		{tgtPC_.h, 3},
		{sizeof(float) * 3, sizeof(float)}
	));
	
	float *tgtPCMem = (float *)tgtPC.request().ptr;
	memcpy(tgtPCMem, tgtPC_.Element, 3 * tgtPC_.h * sizeof(float));
	delete[] tgtPC_.Element;

	py::tuple result_tuple = py::make_tuple(tgtImage, tgtPC, RCO, tCO);

	return result_tuple;	
}

float RVLPYNNExtractor::get_tgt_focal_length()
{
	return extractor.fTgt;
}

py::tuple RVLPYNNExtractor::get_tgt_image_size()
{
	py::tuple result_tuple = py::make_tuple(extractor.tgtImgHeight, extractor.tgtImgWidth);

	return result_tuple;	
}

py::tuple RVLPYNNExtractor::get_tgt_clipping_planes()
{
	py::tuple result_tuple = py::make_tuple(extractor.zn, extractor.zf);

	return result_tuple;
}


////////////////////////////////////////////////////////////////////
//
//     RVLNN Wrapper
//
////////////////////////////////////////////////////////////////////

PYBIND11_MODULE(RVLPYNN, m) 
{
	m.doc() = "RVLNN wrapper";
	
	py::class_<RVLPYNNCorruptor>(m, "Corruptor")
		.def(py::init<>())
		.def("create", &RVLPYNNCorruptor::create)
		.def("apply", &RVLPYNNCorruptor::apply)
		.def("get_hole_size", &RVLPYNNCorruptor::get_hole_size);

	py::class_<RVLPYNNBoxSampler>(m, "BoxSampler")
		.def(py::init<>())
		.def("create", &RVLPYNNBoxSampler::create)
		.def("apply", &RVLPYNNBoxSampler::apply)
		.def("set_box_size", &RVLPYNNBoxSampler::set_box_size)
		.def("get_box_size", &RVLPYNNBoxSampler::get_box_size);

	py::class_<RVLPYNNExtractor>(m, "Extractor")
		.def(py::init<>())
		.def("create", &RVLPYNNExtractor::create)
		.def("apply", &RVLPYNNExtractor::apply)
		.def("get_tgt_focal_length", &RVLPYNNExtractor::get_tgt_focal_length)
		.def("get_tgt_image_size", &RVLPYNNExtractor::get_tgt_image_size)
		.def("get_tgt_clipping_planes", &RVLPYNNExtractor::get_tgt_clipping_planes);
}

