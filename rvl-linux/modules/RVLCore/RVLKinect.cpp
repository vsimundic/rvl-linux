#include "RVLPlatform.h"

//#include "cv.h"
#ifdef RVLLINUX
#include "opencv2/opencv.hpp"
#else
#include "opencv2\opencv.hpp"
#endif
//#include "opencv\cv.h"
#ifdef RVLOPENNI
#include "OpenNI.h"
#endif
#include "RVLKinect.h"

CRVLKinect::CRVLKinect(void)
{
	m_Flags = 0x00000000;

	m_scale = 2;
	m_RGBscale = 1;

#ifdef RVLOPENNI
	m_vpDevice = new openni::Device;
	m_vpDepthStream = new openni::VideoStream;
	m_vpColorStream = new openni::VideoStream;
	m_vpStream[0] = m_vpDepthStream;
	m_vpStream[1] = m_vpColorStream;
	m_vpCoordinateConverter = new openni::CoordinateConverter;
#endif

	int maxz = 0x7fff;

	m_zToDepthLookupTable = new short[maxz + 1];

	m_zToDepthLookupTable[0] = 2047;

	for(int iz = 1; iz <= maxz; iz++)
		m_zToDepthLookupTable[iz] = (short)RVLKINECTZTODEPTH((double)iz);
}

CRVLKinect::~CRVLKinect(void)
{
#ifdef RVLOPENNI
	openni::VideoStream *pDepthStream = (openni::VideoStream *)m_vpDepthStream;
	pDepthStream->destroy();

	openni::VideoStream *pColorStream = (openni::VideoStream *)m_vpColorStream;
	pColorStream->destroy();

	openni::Device *pDevice = (openni::Device *)m_vpDevice;

	openni::CoordinateConverter *pCoordinateConverter = (openni::CoordinateConverter *)m_vpCoordinateConverter;
	
	//openni::OpenNI::shutdown();

	delete pDepthStream;
	delete pColorStream;
	delete pDevice;
	delete pCoordinateConverter;
#endif
	delete[] m_zToDepthLookupTable;
}

#ifdef RVLOPENNI
// This function is made by modifying an OpenNI sample.
// Therefore it is subject to Apache License (http://www.apache.org/licenses/LICENSE-2.0)
// This licence should be read before distribution of the file containing this function
// and all conditions required by this licence should be met.

bool CRVLKinect::Init(char *ONIFileName)
{
	openni::Device *pDevice = (openni::Device *)m_vpDevice;
	openni::VideoStream *pDepthStream = (openni::VideoStream *)m_vpDepthStream;
	openni::VideoStream *pColorStream = (openni::VideoStream *)m_vpColorStream;

	openni::Status rc = openni::STATUS_OK;

	const char* deviceURI;
	
	if(ONIFileName)
	{
		deviceURI = ONIFileName;
		m_Flags |= RVLKINECT_FLAG_ONI_FILE;
	}
	else if(m_Flags & RVLKINECT_FLAG_ONI_FILE)	
		deviceURI = m_ONIFileName;
	else
		deviceURI = openni::ANY_DEVICE;

	rc = openni::OpenNI::initialize();

	//printf("Kinect initialization:\n%s\n", openni::OpenNI::getExtendedError());

	rc = pDevice->open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		//printf("Kinct open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return FALSE;
	}

	rc = pDevice->setDepthColorSyncEnabled(true);

	//if(rc == openni::STATUS_OK)
	//	printf("Kinect: depth - color synchronization on\n");
	//else
	//	printf("Kinect: depth - color synchronization off\n");

	rc = pDepthStream->create(*pDevice, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		openni::VideoMode dmod = openni::VideoMode();
		dmod.setFps(30);
		dmod.setResolution(640, 480);
		dmod.setPixelFormat(m_Flags & RVLKINECT_FLAG_100UM ? openni::PixelFormat::PIXEL_FORMAT_DEPTH_100_UM :
			openni::PixelFormat::PIXEL_FORMAT_DEPTH_1_MM);
		pDepthStream->setVideoMode(dmod);
		rc = pDepthStream->start();
		if (rc != openni::STATUS_OK)
		{
			//printf("Kinect: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			pDepthStream->destroy();
		}

		// only for debug purpose!!!

		openni::VideoMode DepthMode = pDepthStream->getVideoMode();

		int debug = 0;

		/////
	}
	else
	{
		//printf("Kinect: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}	

	rc = pColorStream->create(*pDevice, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		openni::VideoMode cmod = openni::VideoMode();
		cmod.setFps(30);
		cmod.setResolution(320, 240);
		//cmod.setResolution(640, 480);
		cmod.setPixelFormat(openni::PixelFormat::PIXEL_FORMAT_RGB888);
		//cmod.setPixelFormat(openni::PixelFormat::PIXEL_FORMAT_YUV422);
		pColorStream->setVideoMode(cmod);
		rc = pColorStream->start();
		if (rc != openni::STATUS_OK)
		{
			//printf("Kinect: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			pColorStream->destroy();
		}
	}
	else
	{
		//printf("Kinect: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	}	

	if (pDepthStream->isValid() && pColorStream->isValid())
		m_Flags |= RVLKINECT_FLAG_DEVICE;
	else
	{
		//printf("Kinect: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return FALSE;
	}

	//if(pDevice->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	//	rc = pDevice->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

	m_vpPlaybackControl = pDevice->getPlaybackControl();

	return TRUE;
}

// This function is made using the code of an OpenNI sample.
// Therefore it is subject to Apache License (http://www.apache.org/licenses/LICENSE-2.0)
// This licence should be read before distribution of the file containing this function
// and all conditions required by this licence should be met.

bool CRVLKinect::GetImages(	short *pDepth,
							IplImage *pImageRGB,						
							IplImage *pImageDepth, 
							IplImage *pImageGS,
							unsigned int Format,
							int frameIdx,
							void *vpDepthFrame,
							void *vpColorFrame)
{
	openni::Device *pDevice = (openni::Device *)m_vpDevice;
	openni::VideoStream *pDepthStream = (openni::VideoStream *)m_vpDepthStream;
	openni::VideoStream *pColorStream = (openni::VideoStream *)m_vpColorStream;
	openni::VideoStream **pStreams = (openni::VideoStream **)m_vpStream;
	openni::PlaybackControl *pPlaybackControl = (openni::PlaybackControl *)m_vpPlaybackControl;

	openni::VideoFrameRef DepthFrame;
	openni::VideoFrameRef ColorFrame;

	openni::VideoFrameRef *pDepthFrame = (vpDepthFrame ? (openni::VideoFrameRef *)vpDepthFrame : &DepthFrame);
	openni::VideoFrameRef *pColorFrame = (vpColorFrame ? (openni::VideoFrameRef *)vpColorFrame : &ColorFrame);

	bool bDepth = false;
	bool bColor = (pImageRGB == NULL);

	int changedIndex = -1;
	int width, height;
	short *pDepth_;
	const openni::DepthPixel* pDepthRow;
	int rowSize;
	unsigned char *pDisplayDepth;
	short d;
	const openni::DepthPixel* pDepth__;
	const openni::RGB888Pixel* pRGBRow;
	char *pPixRGB;
	int rowSizeRGB;
	const openni::RGB888Pixel* pPix;
	openni::VideoMode DepthMode;

	int u, v;

	while(!bDepth || !bColor)
	{
		if(m_Flags & RVLKINECT_FLAG_ONI_FILE)
			changedIndex++;
		else
		{
			openni::Status rc = openni::OpenNI::waitForAnyStream(pStreams, 2, &changedIndex);

			if(rc != openni::STATUS_OK)
				return false;
		}

		switch(changedIndex){
		case 0:
			if(m_Flags & RVLKINECT_FLAG_ONI_FILE)
				pPlaybackControl->seek(*pDepthStream, frameIdx);

			pDepthStream->readFrame(pDepthFrame);

			if(m_Flags & RVLKINECT_FLAG_ONI_FILE)
			{
				//pPlaybackControl->seek(*pColorStream, DepthFrame.getFrameIndex() - 3);

				pColorStream->readFrame(pColorFrame);

				//pPlaybackControl->seek(*pColorStream, ColorFrame.getFrameIndex() - 1);

				//pColorStream->readFrame(&ColorFrame);
			}

			if(!pDepthFrame->isValid())
				continue;

			bDepth = true;

			width = pDepthFrame->getWidth() / m_scale;
			height = pDepthFrame->getHeight() / m_scale;

			DepthMode = pDepthFrame->getVideoMode();

			//if(DepthMode.getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM )
			//	int debug = 0;

			pDepth_ = pDepth;

			pDepthRow = (const openni::DepthPixel*)pDepthFrame->getData();

			rowSize = pDepthFrame->getStrideInBytes() / sizeof(openni::DepthPixel);
			
			if(pImageDepth)
				pDisplayDepth = (unsigned char *)(pImageDepth->imageData);

			////

			//cv::Mat imgM1(480, 640, CV_32FC1);

		 //  for (int j = 0; j < 480; j++) //0-480, 200-300

		 //  {

		 //         for (int i = 0; i < 640; i++) //0-640, 300-600

		 //         {

		 //               imgM1.at<float>(j, i) = pDepthRow[j*640 + i];

		 //         }

		 //  }

		 //  cv::Mat imgM2(480, 640, CV_32FC1);

		 //  cv::bilateralFilter(imgM1, imgM2, 0, 10, 5);

		 //  uint16_t* filteredDepth = new uint16_t[640*480];

		 //  for (int j = 0; j < 480; j++) //0-480, 200-300

		 //  {

		 //         for (int i = 0; i < 640; i++) //0-640, 300-600

		 //         {

		 //               filteredDepth[j*640 + i] = (uint16_t)imgM2.at<float>(j, i);

		 //         }

		 //  }

			//////

			//pDepthRow = filteredDepth;

			for (v = 0; v < height; v++)
			{
				pDepth__ = pDepthRow + m_scale * (width - 1);

				for (u = 0; u < width; u++, pDepth__-= m_scale, pDepth_++)
				{
					*pDepth_ = (Format == RVLKINECT_DEPTH_IMAGE_FORMAT_DISPARITY ? m_zToDepthLookupTable[*pDepth__] : *pDepth__);

					if(pImageDepth)
					{
						d = (*pDepth_ - 600) / 2;

						if(d < 0)
							*pDisplayDepth = 0;
						else if(d < 256)
							*pDisplayDepth = (unsigned char)d;
						else
							*pDisplayDepth = 0;

						pDisplayDepth++;
					}
				}

				pDepthRow += (m_scale * rowSize);
			}
			
			break;
		case 1:
			if((m_Flags & RVLKINECT_FLAG_ONI_FILE) == 0)
			{
				pColorStream->readFrame(pColorFrame);

				if(!pColorFrame->isValid())
					continue;	
			}

			bColor = true;

			width = pColorFrame->getWidth() / m_RGBscale;
			height = pColorFrame->getHeight() / m_RGBscale;

			if(pImageRGB)
			{		
				pColorStream->readFrame(pColorFrame);

				pRGBRow = (const openni::RGB888Pixel*)pColorFrame->getData();

				pPixRGB = pImageRGB->imageData;

				rowSizeRGB = pColorFrame->getStrideInBytes() / sizeof(openni::RGB888Pixel);

				for (v = 0; v < height; v++)
				{
					pPix = pRGBRow + m_RGBscale * (width - 1);

					for (u = 0; u < width; u++, pPix -= m_RGBscale)
					{
						*(pPixRGB++) = pPix->b;
						*(pPixRGB++) = pPix->g;
						*(pPixRGB++) = pPix->r;
					}

					pRGBRow += (m_RGBscale * rowSizeRGB);
				}	

				if(pImageGS)
					cvCvtColor(pImageRGB, pImageGS, CV_BGR2GRAY);
			}	// switch(changedIndex)
			
			break;
		default:
			return false;
		}
	}	// while(!bDepth || !bColor)

	//imgM1.release();
	//imgM2.release();
	//delete[] filteredDepth;

	return true;
}

void  CRVLKinect::RegisterDepthToColor(bool bRegister)
{
	openni::Device *pDevice = (openni::Device *)m_vpDevice;

	pDevice->setImageRegistrationMode(bRegister ? openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR : openni::IMAGE_REGISTRATION_OFF);

	openni::ImageRegistrationMode mode = pDevice->getImageRegistrationMode();
}

void CRVLKinect::GetParams()
{
	openni::CoordinateConverter *pCoordinateConverter = (openni::CoordinateConverter *)(m_vpCoordinateConverter);

	openni::VideoStream *pDepthStream = (openni::VideoStream *)m_vpDepthStream;

	//int width = DepthFrame.getWidth();
	//int height = DepthFrame.getHeight();
	int width = 640;
	int height = 480;

	int u1 = width / 4;
	int u2 = 3 * width / 4;
	int v1 = height / 4;
	int v2 = 3 * height / 4;
	openni::DepthPixel z1_ = 1000;
	openni::DepthPixel z2_ = 1000;

	float x1, y1, z1, x2, y2, z2;
	
	pCoordinateConverter->convertDepthToWorld(*pDepthStream, u1, v1, z1_, &x1, &y1, &z1);
	pCoordinateConverter->convertDepthToWorld(*pDepthStream, u2, v2, z2_, &x2, &y2, &z2);

	m_fu = (double)(u2 - u1) / (double)(x2 / z2 - x1 / z1);
	m_fv = (double)(v2 - v1) / (double)(y2 / z2 - y1 / z1);
	m_uc = (double)u1 - m_fu * (double)(x1 / z1);
	m_vc = (double)v1 - m_fv * (double)(y1 / z1);
}

void CRVLKinect::ConvertDepthToWorld(int u, int v, int z, float *px, float *py, float *pz)
{
	openni::CoordinateConverter *pCoordinateConverter = (openni::CoordinateConverter *)(m_vpCoordinateConverter);

	openni::VideoStream *pDepthStream = (openni::VideoStream *)m_vpDepthStream;

	pCoordinateConverter->convertDepthToWorld(*pDepthStream, u, v, z, px, py, pz);
}

void CRVLKinect::ConvertDepthToColor(int u, int v, int z, int *puRGB, int *pvRGB)
{
	openni::CoordinateConverter *pCoordinateConverter = (openni::CoordinateConverter *)(m_vpCoordinateConverter);

	openni::VideoStream *pDepthStream = (openni::VideoStream *)m_vpDepthStream;
	openni::VideoStream *pColorStream = (openni::VideoStream *)m_vpColorStream;

	int uRGB, vRGB;

	openni::Status rc = pCoordinateConverter->convertDepthToColor(*pDepthStream, *pColorStream, m_scale * u, m_scale * v, z, &uRGB, &vRGB);

	*puRGB = uRGB / m_scale;
	*pvRGB = vRGB / m_scale;
}

int CRVLKinect::GetNoONIFrames(void)
{
	openni::VideoStream *pDepthStream = (openni::VideoStream *)m_vpDepthStream;
	openni::PlaybackControl *pPlaybackControl = (openni::PlaybackControl *)(m_vpPlaybackControl);

	return pPlaybackControl->getNumberOfFrames(*pDepthStream);
}

void CRVLKinect::SetPlaybeckSpeed(float speed)
{
	openni::PlaybackControl *pPlaybackControl = (openni::PlaybackControl *)(m_vpPlaybackControl);

	pPlaybackControl->setSpeed(speed);
}

#endif	//RVLOPENNI

char * RVLKinectCreateONISampleFileName(char *ONIFileName, 
										int iSample, 
										char *Extension)
{
	int ONIFileNameLen = strlen(ONIFileName);
	int ExtensionLen = strlen(Extension);

	char *SampleFileName = new char[ONIFileNameLen - 4 + ExtensionLen + 7];

	strcpy(SampleFileName, ONIFileName);

	sprintf(SampleFileName + ONIFileNameLen - 4, "-%05d", iSample);

	strcpy(SampleFileName + ONIFileNameLen - 4 + 6, Extension);
	
	return SampleFileName;
}