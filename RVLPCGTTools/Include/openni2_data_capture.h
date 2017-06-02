//#include <fovis/fovis.hpp>
#include <string>
#include "OpenNI.h"


namespace openni2_data_capture
{

	class DataCapture
	{
	public:
		int m_noFrames;
		int m_currentFrameIdx;

		DataCapture(int width = 640, int height = 480);
		~DataCapture();

		bool initialize();

		bool startDataCapture();

		bool stopDataCapture();

		bool captureOne();

		bool openonifile(std::string onifilename);

		bool seekFrame(int idx);


		void UpdateGrayImage()
		{
			// convert to grayscale.
			int num_rgb_pixels = m_width * m_height;
			for (int i = 0; i < num_rgb_pixels; i++)
				m_gray_buf[i] = std::floor(0.2125 * m_ColorData[i * 4] + 0.7154 * m_ColorData[i * 4 + 1] + 0.0721 * m_ColorData[i * 4 + 2]);	//RGBA
		}

		const uint8_t* getGrayImage()
		{
			return m_gray_buf;
		}

		const uint8_t* getRGBData()
		{
			return m_ColorData;
		}

		const uint16_t* getDepthData()
		{
			return m_DepthData;
		}

		void UpdateFloatDepthData()
		{
			// convert to meters, and set unknown depth values to NAN
			int num_depth_pixels = m_width * m_height;
			for (int i = 0; i<num_depth_pixels; i++)
			{
				UINT16 d = m_DepthData[i];
				if (d != 0)
					m_depth_data_f[i] = d * 1e-3;
				else
					m_depth_data_f[i] = NAN;
			}
		}

		const float* getFloatDepthData()
		{
			return m_depth_data_f;
		}

		int getWidth()
		{
			return m_width;
		}

		int getHeight()
		{
			return m_height;
		}

		void transformDepthToWorld(int u, int v, int d, float &x, float &y, float &z)
		{
			openni::CoordinateConverter::convertDepthToWorld(*m_DepthStream, u, v, d, &x, &y, &z);
		}

		void transformDepthToWorld(int u, int v, int d, double &x_x, double &y_y, double &z_z)
		{
			float x, y, z;
			openni::CoordinateConverter::convertDepthToWorld(*m_DepthStream, u, v, d, &x, &y, &z);
			x_x = x;
			y_y = y;
			z_z = z;
		}

		void getXYZ(int u, int v, float &x, float &y, float &z)
		{
			openni::CoordinateConverter::convertDepthToWorld(*m_DepthStream, u, v, m_DepthData[v * m_width + u], &x, &y, &z);
		}

		void getXYZ(int u, int v, double &x_x, double &y_y, double &z_z)
		{
			float x, y, z;
			openni::CoordinateConverter::convertDepthToWorld(*m_DepthStream, u, v, m_DepthData[v * m_width + u], &x, &y, &z);
			x_x = x;
			y_y = y;
			z_z = z;
		}

	private:
		openni::Device* m_device;
		openni::PlaybackControl* m_devicePC;
		openni::VideoStream* m_DepthStream;
		openni::VideoStream* m_ColorStream;
		openni::VideoFrameRef*	m_depthFrame;
		openni::VideoFrameRef*	m_colorFrame;
		openni::VideoStream**	m_streams;
		const openni::RGB888Pixel* m_pImage;
		uint16_t* m_DepthData;
		uint8_t* m_ColorData;

		int m_width;
		int m_height;

		float* m_depth_data_f;
		uint8_t* m_gray_buf;
	};

}
