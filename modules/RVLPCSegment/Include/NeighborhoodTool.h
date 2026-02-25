#pragma once

namespace RVL
{
	class NeighborhoodTool
	{
	public:
		NeighborhoodTool();
		virtual ~NeighborhoodTool();
		virtual void Create(void *vpPointArray);
		virtual void RadiusSearch(
			float *P,
			float radius,
			std::vector<int> &pointIdxRadiusSearch,
			std::vector<float> &pointRadiusSquaredDistance);
	};
}

