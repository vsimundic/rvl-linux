#pragma once

namespace RVL
{
	class PCLPointCloud
	{
	public:
		PCLPointCloud();
		virtual ~PCLPointCloud();
		void Create(int w, int h);

	public:
		void *vpPC;
	};
}

