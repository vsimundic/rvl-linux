#include "openni2_data_capture.h"

#include <cmath>

using namespace std;

namespace openni2_data_capture
{

	DataCapture::DataCapture(int width, int height)
	{
		m_width = width;
		m_height = height;

		m_depth_data_f = new float[m_width * m_height];
		m_gray_buf = new uint8_t[m_width * m_height];
	}

	DataCapture::~DataCapture()
	{
		delete[] m_depth_data_f;
		delete[] m_gray_buf;
	}

	bool DataCapture::initialize()
	{
		m_device = new openni::Device();
		m_DepthStream = new openni::VideoStream();
		m_ColorStream = new openni::VideoStream();
		m_depthFrame = new openni::VideoFrameRef();
		m_colorFrame = new openni::VideoFrameRef();
		m_DepthData = new uint16_t[m_width * m_height * 3];
		m_ColorData = new uint8_t[m_width * m_height * 3];

		openni::Status rc = openni::STATUS_OK;
		//Initializing
		openni::OpenNI::initialize();
		printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
		//Fetching ANY device
		const char* deviceURI = openni::ANY_DEVICE;
		rc = m_device->open(deviceURI);
		if (rc != openni::STATUS_OK)
		{
			printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			return false;
		}
		//Initializing depth and color stream
		m_DepthStream->create(*m_device, openni::SENSOR_DEPTH);
		if (rc == openni::STATUS_OK)
		{
			//setting depth video mode
			openni::VideoMode dmod = openni::VideoMode();
			dmod.setFps(30);
			dmod.setResolution(m_width, m_height);
			dmod.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
			m_DepthStream->setVideoMode(dmod);
			//starting stream
			rc = m_DepthStream->start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
				m_DepthStream->destroy();
				return false;
			}
		}
		else
		{
			printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			return false;
		}

		rc = m_ColorStream->create(*m_device, openni::SENSOR_COLOR);
		if (rc == openni::STATUS_OK)
		{
			//setting color video mode
			openni::VideoMode cmod = openni::VideoMode();
			cmod.setFps(30);
			cmod.setResolution(m_width / 2, m_height / 2);  //Color is half depth
			//cmod.setResolution(m_width, m_height);
			cmod.setPixelFormat(openni::PixelFormat::PIXEL_FORMAT_RGB888);
			m_ColorStream->setVideoMode(cmod);

			//starting stream
			rc = m_ColorStream->start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
				m_ColorStream->destroy();
				return false;
			}
		}
		else
		{
			printf("Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
			return false;
		}

		if (!m_DepthStream->isValid() || !m_ColorStream->isValid())
		{
			printf("OpenNI: No valid streams. Exiting\n");
			openni::OpenNI::shutdown();
			return false;
		}
		//Setting registration
		if (m_device->isImageRegistrationModeSupported(openni::ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
		{
			rc = m_device->setImageRegistrationMode(openni::ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't activate depth to color registration:\n%s\n", openni::OpenNI::getExtendedError());
				openni::OpenNI::shutdown();
				return false;
			}
		}
		//Force sync between dapth and color streams
		rc = m_device->setDepthColorSyncEnabled(true);
		if (rc != openni::STATUS_OK)
		{
			printf("Couldn't sync depth ansd color streams:\n%s\n", openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			return false;
		}

		//Mirror disabled
		m_DepthStream->setMirroringEnabled(false);
		m_ColorStream->setMirroringEnabled(false);

		m_streams = new openni::VideoStream*[2];
		m_streams[0] = m_DepthStream;
		m_streams[1] = m_ColorStream;

		//All phases passed
		return true;
	}

	bool DataCapture::startDataCapture()
	{
		// start data capture
		printf("Starting data capture\n");
		//IT ALREADY STARETD IN INITIALIZE!!!!
		return true;
	}

	bool DataCapture::stopDataCapture()
	{
		printf("Stopping streams and shutting down openni!");
		m_DepthStream->stop();
		m_ColorStream->stop();
		openni::OpenNI::shutdown();
		return true;
	}

	bool DataCapture::captureOne()
	{
		//NEW CODE
		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed\n");
			return false;
		}

		switch (changedIndex)
		{
		case 0:
			m_DepthStream->readFrame(m_depthFrame); break;
		case 1:
			m_ColorStream->readFrame(m_colorFrame); break;
		default:
			printf("Error in wait\n");
		}

		if (m_depthFrame->isValid())
		{
			//Prepare depth data
			const openni::DepthPixel* pDepth = (const openni::DepthPixel*)m_depthFrame->getData();
			memcpy(m_DepthData, pDepth, m_depthFrame->getDataSize());
		}

		if (m_colorFrame->isValid())
		{
			m_pImage = (const openni::RGB888Pixel*)m_colorFrame->getData();
			memcpy(m_ColorData, m_pImage, m_colorFrame->getDataSize());
		}

		//depth_image->setDepthImage(depth_data);
		m_currentFrameIdx++;
		return true;
	}

	bool DataCapture::openonifile(std::string onifilename)
	{
		m_device = new openni::Device();
		m_DepthStream = new openni::VideoStream();
		m_ColorStream = new openni::VideoStream();
		m_depthFrame = new openni::VideoFrameRef();
		m_colorFrame = new openni::VideoFrameRef();
		m_DepthData = new uint16_t[m_width * m_height * 3];
		m_ColorData = new uint8_t[m_width * m_height * 3];

		openni::Status rc = openni::STATUS_OK;
		//Initializing
		openni::OpenNI::initialize();
		printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
		//Fetching ANY device
		const char* deviceURI = onifilename.c_str();
		rc = m_device->open(deviceURI);
		if (rc != openni::STATUS_OK)
		{
			printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
			openni::OpenNI::shutdown();
			return false;
		}

		//getting playback control
		m_devicePC = m_device->getPlaybackControl();

		//Initializing depth and color stream
		m_DepthStream->create(*m_device, openni::SENSOR_DEPTH);
		if (rc == openni::STATUS_OK)
		{
			//starting stream
			rc = m_DepthStream->start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
				m_DepthStream->destroy();
				return false;
			}
		}
		else
		{
			printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			return false;
		}

		rc = m_ColorStream->create(*m_device, openni::SENSOR_COLOR);
		if (rc == openni::STATUS_OK)
		{
			//starting stream
			rc = m_ColorStream->start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
				m_ColorStream->destroy();
				return false;
			}
		}
		else
		{
			printf("Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
			return false;
		}

		//Checking validity
		if (!m_DepthStream->isValid() || !m_ColorStream->isValid())
		{
			printf("OpenNI: No valid streams. Exiting\n");
			openni::OpenNI::shutdown();
			return false;
		}

		//Other options such as registrarion should be set when recording stream

		//Mirror disabled
		m_DepthStream->setMirroringEnabled(false);
		m_ColorStream->setMirroringEnabled(false);

		m_streams = new openni::VideoStream*[2];
		m_streams[0] = m_DepthStream;
		m_streams[1] = m_ColorStream;

		m_noFrames = m_devicePC->getNumberOfFrames(*m_DepthStream);

		//All phases passed
		return true;
	}

	bool DataCapture::seekFrame(int idx)
	{
		m_devicePC->seek(*m_DepthStream, idx);

		m_DepthStream->readFrame(m_depthFrame);
		m_ColorStream->readFrame(m_colorFrame);

		if (m_depthFrame->isValid())
		{
			//Prepare depth data
			const openni::DepthPixel* pDepth = (const openni::DepthPixel*)m_depthFrame->getData();
			memcpy(m_DepthData, pDepth, m_depthFrame->getDataSize());
		}

		if (m_colorFrame->isValid())
		{
			m_pImage = (const openni::RGB888Pixel*)m_colorFrame->getData();
			memcpy(m_ColorData, m_pImage, m_colorFrame->getDataSize());
		}

		//depth_image->setDepthImage(depth_data);
		m_currentFrameIdx == idx; //WHAT IS THIS??!

		return true;
	}

}
