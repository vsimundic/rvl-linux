
class Astra
{

public:
	Astra();
	virtual ~Astra();
	void InitAstra();
	void GetImageFromAstra();
	void TerminateAstra();

#ifdef RVL_ASTRA
	astra::StreamSet streamSet;
	astra::StreamReader reader;
	astra::ImageStreamMode mode;
	astra::ImageStreamMode modeD;
	ColorFrameListener listener;
#endif
	float fu, fv, uc, vc;

	cv::Mat Image;
	cv::Mat ImageF;
	cv::Mat depthImg;
	cv::Mat depthImgF;
	cv::Mat depth;
	cv::Mat depthF;

};