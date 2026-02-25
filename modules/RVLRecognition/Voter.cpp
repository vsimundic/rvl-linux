#include "RVLCore2.h"
//#include "Util.h"
#include "Voter.h"

using namespace RVL;

Voter1D::Voter1D()
{
	votes.Element = NULL;
	votes.n = 0;
	cellSize = 0;
	minValue = 0;
	maxValue = 0;
}

void Voter1D::Vote(Array<float> data, float cellSizeIn)
{
	cellSize = cellSizeIn;

	minValue = data.Element[0];
	maxValue = data.Element[0];

	for (int iData = 1; iData < data.n; iData++)
	{
		if (data.Element[iData] < minValue)
			minValue = data.Element[iData];
		if (data.Element[iData] > maxValue)
			maxValue = data.Element[iData];
	}

	int nCells = (int)ceil((maxValue - minValue) / cellSize);

	votes.Element = new int[nCells];
	votes.n = nCells;

	for (int iVote = 0; iVote < votes.n; iVote++)
	{
		votes.Element[iVote] = 0;
	}

	for (int iData = 0; iData < data.n; iData++)
	{
		float diff = data.Element[iData] - minValue;
		int iCell = (int)floor(diff / cellSize);

		votes.Element[iCell]++;
	}
}

float Voter1D::GetCellValue(int cellIdx)
{
	return minValue + (0.5 + cellIdx) * cellSize;
}

int Voter1D::GetCellVotes(int cellIdx)
{
	return votes.Element[cellIdx];
}

float Voter1D::GetMax()
{
	int iMax = 0;
	for (int iVote = 1; iVote < votes.n; iVote++)
	{
		int v = GetCellVotes(iVote);
		int maxVote = GetCellVotes(iMax);
		if (v > maxVote)
			iMax = iVote;
	}
	return GetCellValue(iMax);
}

Voter3D::Voter3D()
{
	nDimensions = 3;
	votes.Element = NULL;
	votes.n = 0;

	for (int iDimension = 0; iDimension < nDimensions; iDimension++)
	{
		cellSize[iDimension] = 0;
		nCells[iDimension] = 0;
		minValue[iDimension] = 0;
		maxValue[iDimension] = 0;
	}
}

bool Voter3D::Vote(Array<Vector3<float>> data, float* cellSizeIn)
{
	for (int i = 0; i < nDimensions; i++)
	{
		cellSize[i] = cellSizeIn[i];
	}

	minValue[0] = FLT_MAX;
	minValue[1] = FLT_MAX;
	minValue[2] = FLT_MAX;
	maxValue[0] = FLT_MIN;
	maxValue[1] = FLT_MIN;
	maxValue[2] = FLT_MIN;


	for (int iData = 0; iData < data.n; iData++)
	{
		for (int iDimension = 0; iDimension < nDimensions; iDimension++)
		{
			if (data.Element[iData].Element[iDimension] < minValue[iDimension])
			{
				minValue[iDimension] = data.Element[iData].Element[iDimension];
			}
			if (data.Element[iData].Element[iDimension] > maxValue[iDimension])
			{
				maxValue[iDimension] = data.Element[iData].Element[iDimension];
			}
		}
	}

	int totalCells = 1;

	for (int iDimension = 0; iDimension < nDimensions; iDimension++)
	{
		nCells[iDimension] = (int)ceil((maxValue[iDimension] - minValue[iDimension]) / cellSize[iDimension]);
		totalCells *= nCells[iDimension];
	}

	if (totalCells == 0)
		return false;

	votes.Element = new int[totalCells];
	votes.n = totalCells;

	for (int iVote = 0; iVote < votes.n; iVote++)
	{
		votes.Element[iVote] = 0;
	}

	float diff;
	int iCell[3];

	for (int iData = 0; iData < data.n; iData++)
	{
		for (int iDimension = 0; iDimension < nDimensions; iDimension++)
		{
			diff = data.Element[iData].Element[iDimension] - minValue[iDimension];
			iCell[iDimension] = (int)floor(diff / cellSize[iDimension]);
		}
		//x * y_size * z_size + y * z_size + z
		//z_size * (x * y_size + y) + z
		int idx = nCells[2] * (iCell[0] * nCells[1] + iCell[1]) + iCell[2];
		votes.Element[idx]++;
	}

	return true;
}

Vector3<float> Voter3D::GetCellValue(int* cellIdx)
{
	Vector3<float> cellValue;

	for (int iDimension = 0; iDimension < nDimensions; iDimension++)
	{
		cellValue.Element[iDimension] = minValue[iDimension] + (0.5 + cellIdx[iDimension]) * cellSize[iDimension];
	}

	return cellValue;
}


Vector3<float> Voter3D::GetCellValue(int cellIdx)
{
	Vector3<float> cellValue;

	// z = idx % z_size
	// y = (idx / z_size) % y_size
	// x = idx / (y_size * z_size)
	//-----------------------------
	// z = idx % z_size
	// y = idx / z_size
	// x = y / y_size
	// y = y % y_size

	int idz = cellIdx % nCells[2];
	int idy = cellIdx / nCells[2];
	int idx = idy / nCells[1];
	idy = idy % nCells[1];

	int idArray[3] = { idx, idy, idz };

	for (int iDimension = 0; iDimension < nDimensions; iDimension++)
	{
		cellValue.Element[iDimension] = minValue[iDimension] + (0.5 + idArray[iDimension]) * cellSize[iDimension];
	}

	return cellValue;
}

int Voter3D::GetCellVotes(int* cellIdx)
{
	//x * y_size * z_size + y * z_size + z
	//z_size * (x * y_size + y) + z
	int idx = nCells[2] * (cellIdx[0] * nCells[1] + cellIdx[1]) + cellIdx[2];
	return votes.Element[idx];
}

int Voter3D::GetCellVotes(int cellIdx)
{
	return votes.Element[cellIdx];
}

Vector3<float> Voter3D::GetMax()
{
	int iMax = 0;
	int vote, maxVote;
	for (int iVote = 1; iVote < votes.n; iVote++)
	{
		vote = GetCellVotes(iVote);
		maxVote = GetCellVotes(iMax);
		if (vote > maxVote)
			iMax = iVote;
	}

	return GetCellValue(iMax);
}