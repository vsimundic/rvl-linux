#pragma once

namespace RVL
{
	class Voter1D
	{
	public:
		Voter1D();
		void Vote(Array<float> data, float cellSizeIn);
		float GetCellValue(int cellIdx);
		int GetCellVotes(int cellIdx);
		float GetMax();

	private:
		Array<int> votes;
		float cellSize;
		float minValue;
		float maxValue;
	};

	class Voter3D
	{
	public:
		Voter3D();
		bool Vote(Array<Vector3<float>> data, float* cellSizeIn);
		Vector3<float>	GetCellValue(int* cellIdx);
		Vector3<float>	GetCellValue(int cellIdx);
		int GetCellVotes(int* cellIdx);
		int GetCellVotes(int cellIdx);
		Vector3<float> GetMax();

	public:
		float minValue[3];
		float maxValue[3];

	private:
		int nDimensions;
		Array<int> votes;
		float cellSize[3];
		int nCells[3];
	};
}