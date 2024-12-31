#include "RVLPlatform.h"
#include "RVLColorDescriptor.h"
#include "RVLVTK.h"
//#include "opencv2\opencv.hpp"
#include <math.h>

namespace RVL
{
	int* RVLColorDescriptor::BinSize2NoBin(const int* binSize, int cs)
	{
		int *noBins = new int[3];	//number of bins per dimension
		if (cs == RVLColorDescriptor::ColorSpaceList::RGB || cs == RVLColorDescriptor::ColorSpaceList::BGR)
		{
			if (binSize[0] > 0)
				noBins[0] = ceilf(255.0 / binSize[0]);	//R
			else
				noBins[0] = 0;
			if (binSize[1] > 0)
				noBins[1] = ceilf(255.0 / binSize[1]);	//G
			else
				noBins[1] = 0;
			if (binSize[2] > 0)
				noBins[2] = ceilf(255.0 / binSize[2]);	//B
			else
				noBins[2] = 0;
		}
		else if (cs == RVLColorDescriptor::ColorSpaceList::HSV)
		{
			if (binSize[0] > 0)
				noBins[0] = ceilf(180.0 / binSize[0]);	//H
			else
				noBins[0] = 0;
			if (binSize[1] > 0)
				noBins[1] = ceilf(255.0 / binSize[1]);	//S
			else
				noBins[1] = 0;
			if (binSize[2] > 0)
				noBins[2] = ceilf(255.0 / binSize[2]);	//V
			else
				noBins[2] = 0;
		}
		else if (cs == RVLColorDescriptor::ColorSpaceList::Lab)
		{
			if (binSize[0] > 0)
				noBins[0] = ceilf(255.0 / binSize[0]);	//L
			else
				noBins[0] = 0;
			if (binSize[1] > 0)
				noBins[1] = ceilf(255.0 / binSize[1]);	//a
			else
				noBins[1] = 0;
			if (binSize[2] > 0)
				noBins[2] = ceilf(255.0 / binSize[2]);	//b
			else
				noBins[2] = 0;
		}

		return noBins;
	}

	int* RVLColorDescriptor::NoBin2BinSize(const int* noBins, int cs)
	{
		int *binSize = new int[3];	//number of bins per dimension
		if (cs == RVLColorDescriptor::ColorSpaceList::RGB || cs == RVLColorDescriptor::ColorSpaceList::BGR)
		{
			if (noBins[0] > 0)
				binSize[0] = ceilf(255.0 / noBins[0]);	//R
			else
				binSize[0] = 0;
			if (noBins[1] > 0)
				binSize[1] = ceilf(255.0 / noBins[1]);	//G
			else
				binSize[1] = 0;
			if (noBins[2] > 0)
				binSize[2] = ceilf(255.0 / noBins[2]);	//B
			else
				binSize[2] = 0;
		}
		else if (cs == RVLColorDescriptor::ColorSpaceList::HSV)
		{
			if (noBins[0] > 0)
				binSize[0] = ceilf(179.0 / noBins[0]);	//H
			else
				binSize[0] = 0;
			if (noBins[1] > 0)
				binSize[1] = ceilf(256.0 / noBins[1]);	//S //RETURN TO 255 or FIX problem with 1 bin in channel 20.12.2019.
			else
				binSize[1] = 0;
			if (noBins[2] > 0)
				binSize[2] = ceilf(255.0 / noBins[2]);	//V
			else
				binSize[2] = 0;
		}
		else if (cs == RVLColorDescriptor::ColorSpaceList::Lab)
		{
			if (noBins[0] > 0)
				binSize[0] = ceilf(255.0 / noBins[0]);	//L
			else
				binSize[0] = 0;
			if (noBins[1] > 0)
				binSize[1] = ceilf(255.0 / noBins[1]);	//a
			else
				binSize[1] = 0;
			if (noBins[2] > 0)
				binSize[2] = ceilf(255.0 / noBins[2]);	//b
			else
				binSize[2] = 0;
		}

		return binSize;
	}

	void RVLColorDescriptor::InsertPointIntoHistogram(int *color, bool useFilter)
	{
		int binAdr;
		if (useFilter)	//Check if filter is used and if one of the values is below threshold return without inserting
		{
			if (color[0] < this->chFilterThr[0])
				return;
			if (color[1] < this->chFilterThr[1])
				return;
			if (color[2] < this->chFilterThr[2])
				return;
		}
		if (this->oneDimensional)	//1D histogram
		{
			//needs to add up to 3 points to histogram
			int binOffset = 0;
			for (int k = 0; k < 3; k++)
			{
				if (this->chBinSize[k] <= 0)
					continue;
				binAdr = (int)(floor(color[k] / this->chBinSize[k])) + binOffset;
				//working with map
				if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
					this->colorhistogram.at(binAdr)++;	//increment existing bin
				else
					this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
				//update offset
				binOffset += this->chNoBins[k];
			}
		}
		else
		{
			if (this->noChannelsUsed == 3) //3D histograms
			{
				binAdr = (int)(floor(color[0] / this->chBinSize[0]) * this->chNoBins[1] * this->chNoBins[2] + floor(color[1] / this->chBinSize[1]) * this->chNoBins[2] + floor(color[2] / this->chBinSize[2]));
				//working with map
				if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
					this->colorhistogram.at(binAdr)++;	//increment existing bin
				else
					this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
			}
			else if (this->noChannelsUsed == 2) //2D histograms
			{
				if (this->chBinSize[0] == 0) //first channel is not used
					binAdr = (int)(floor(color[1] / this->chBinSize[1]) * this->chNoBins[2] + floor(color[2] / this->chBinSize[2]));
				else if (this->chBinSize[1] == 0) //second channel is not used
					binAdr = (int)(floor(color[0] / this->chBinSize[0]) * this->chNoBins[2] + floor(color[2] / this->chBinSize[2]));
				else //third channel is not used
					binAdr = (int)(floor(color[0] / this->chBinSize[0]) * this->chNoBins[1] + floor(color[1] / this->chBinSize[1]));
				//working with map
				if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
					this->colorhistogram.at(binAdr)++;	//increment existing bin
				else
					this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
			}
			else if (this->noChannelsUsed == 1)
			{
				if (this->chBinSize[0] > 0) //first channel is used
					binAdr = (int)(floor(color[0] / this->chBinSize[0]));
				else if (this->chBinSize[1] > 0) //second channel is used
					binAdr = (int)(floor(color[1] / this->chBinSize[1]));
				else //third channel is used
					binAdr = (int)(floor(color[2] / this->chBinSize[2]));
				//working with map
				if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
					this->colorhistogram.at(binAdr)++;	//increment existing bin
				else
					this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
			}
		}
		this->chPts++;
	}

	void RVLColorDescriptor::InsertArrayIntoHistogram(int *color, int arraysize, bool useFilter)
	{
		int binAdr;
		int noPtsInserted = 0;
		//running through all points in the array
		for (int i = 0; i < arraysize; i++)
		{
			if (useFilter)	//Check if filter is used and ignore this point if one of the values is below threshold
			{
				if (color[i * 3] < this->chFilterThr[0])
					continue;
				if (color[1 + i * 3] < this->chFilterThr[1])
					continue;
				if (color[2 + i * 3] < this->chFilterThr[2])
					continue;
			}

			if (this->oneDimensional)	//1D histogram
			{
				//needs to add up to 3 points to histogram
				int binOffset = 0;
				for (int k = 0; k < 3; k++)
				{
					if (this->chBinSize[k] <= 0)
						continue;
					binAdr = (int)(floor(color[k + i * 3] / this->chBinSize[k])) + binOffset;
					//working with map
					if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
						this->colorhistogram.at(binAdr)++;	//increment existing bin
					else
						this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
					//update offset
					binOffset += this->chNoBins[k];
				}
			}
			else
			{
				if (this->noChannelsUsed == 3) //3D histograms
				{
					binAdr = (int)(floor(color[i * 3] / this->chBinSize[0]) * this->chNoBins[1] * this->chNoBins[2] + floor(color[1 + i * 3] / this->chBinSize[1]) * this->chNoBins[2] + floor(color[2 + i * 3] / this->chBinSize[2]));
					//working with map
					if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
						this->colorhistogram.at(binAdr)++;	//increment existing bin
					else
						this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
				}
				else if (this->noChannelsUsed == 2) //2D histograms
				{
					if (this->chBinSize[0] == 0) //first channel is not used
						binAdr = (int)(floor(color[1 + i * 3] / this->chBinSize[1]) * this->chNoBins[2] + floor(color[2 + i * 3] / this->chBinSize[2]));
					else if (this->chBinSize[1] == 0) //second channel is not used
						binAdr = (int)(floor(color[i * 3] / this->chBinSize[0]) * this->chNoBins[2] + floor(color[2 + i * 3] / this->chBinSize[2]));
					else //third channel is not used
						binAdr = (int)(floor(color[i * 3] / this->chBinSize[0]) * this->chNoBins[1] + floor(color[1 + i * 3] / this->chBinSize[1]));
					//working with map
					if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
						this->colorhistogram.at(binAdr)++;	//increment existing bin
					else
						this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
				}
				else if (this->noChannelsUsed == 1)
				{
					if (this->chBinSize[0] > 0) //first channel is used
						binAdr = (int)(floor(color[i * 3] / this->chBinSize[0]));
					else if (this->chBinSize[1] > 0) //second channel is used
						binAdr = (int)(floor(color[1 + i * 3] / this->chBinSize[1]));
					else //third channel is used
						binAdr = (int)(floor(color[2 + i * 3] / this->chBinSize[2]));
					//working with map
					if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
						this->colorhistogram.at(binAdr)++;	//increment existing bin
					else
						this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
				}
			}
			noPtsInserted++;
		}
		this->chPts += noPtsInserted;	//arraysize;
	}

	void RVLColorDescriptor::InsertPointIntoHistogram(uint8_t *color, bool useFilter)
	{
		//convert to int
		int *colorNew = new int[3];
		colorNew[0] = color[0];
		colorNew[1] = color[1];
		colorNew[2] = color[2];
		//run INT version of the function
		this->InsertPointIntoHistogram(colorNew, useFilter);
		delete colorNew;
	}

	void RVLColorDescriptor::InsertArrayIntoHistogram(uint8_t *color, int arraysize, bool useFilter)
	{
		int binAdr;
		int noPtsInserted = 0;
		//running through all points in the array
		for (int i = 0; i < arraysize; i++)
		{
			if (useFilter)	//Check if filter is used and ignore this point if one of the values is below threshold
			{
				if (color[i * 3] < this->chFilterThr[0])
					continue;
				if (color[1 + i * 3] < this->chFilterThr[1])
					continue;
				if (color[2 + i * 3] < this->chFilterThr[2])
					continue;
			}

			if (this->oneDimensional)	//1D histogram
			{
				//needs to add up to 3 points to histogram
				int binOffset = 0;
				for (int k = 0; k < 3; k++)
				{
					if (this->chBinSize[k] <= 0)
						continue;
					binAdr = (int)(floor(color[k + i * 3] / this->chBinSize[k])) + binOffset;
					//working with map
					if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
						this->colorhistogram.at(binAdr)++;	//increment existing bin
					else
						this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
					//update offset
					binOffset += this->chNoBins[k];
				}
			}
			else
			{
				if (this->noChannelsUsed == 3) //3D histograms
				{
					binAdr = (int)(floor(color[i * 3] / this->chBinSize[0]) * this->chNoBins[1] * this->chNoBins[2] + floor(color[1 + i * 3] / this->chBinSize[1]) * this->chNoBins[2] + floor(color[2 + i * 3] / this->chBinSize[2]));
					//working with map
					if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
						this->colorhistogram.at(binAdr)++;	//increment existing bin
					else
						this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
				}
				else if (this->noChannelsUsed == 2) //2D histograms
				{
					if (this->chBinSize[0] == 0) //first channel is not used
						binAdr = (int)(floor(color[1 + i * 3] / this->chBinSize[1]) * this->chNoBins[2] + floor(color[2 + i * 3] / this->chBinSize[2]));
					else if (this->chBinSize[1] == 0) //second channel is not used
						binAdr = (int)(floor(color[i * 3] / this->chBinSize[0]) * this->chNoBins[2] + floor(color[2 + i * 3] / this->chBinSize[2]));
					else //third channel is not used
						binAdr = (int)(floor(color[i * 3] / this->chBinSize[0]) * this->chNoBins[1] + floor(color[1 + i * 3] / this->chBinSize[1]));
					//working with map
					if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
						this->colorhistogram.at(binAdr)++;	//increment existing bin
					else
						this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
				}
				else if (this->noChannelsUsed == 1)
				{
					if (this->chBinSize[0] > 0) //first channel is used
						binAdr = (int)(floor(color[i * 3] / this->chBinSize[0]));
					else if (this->chBinSize[1] > 0) //second channel is used
						binAdr = (int)(floor(color[1 + i * 3] / this->chBinSize[1]));
					else //third channel is used
						binAdr = (int)(floor(color[2 + i * 3] / this->chBinSize[2]));
					//working with map
					if (this->colorhistogram.find(binAdr) != this->colorhistogram.end())
						this->colorhistogram.at(binAdr)++;	//increment existing bin
					else
						this->colorhistogram.insert(std::pair<int, unsigned int>(binAdr, 1)); //add new bin
				}
			}
			noPtsInserted++;
		}
		this->chPts += noPtsInserted;	//arraysize;
	}

	double RVLColorDescriptor::L1Distance(RVLColorDescriptor& other)
	{
		double d = 0.0;
		//Find max number of bins
		int maxNoBins;
		double chPtsF;
		// if 1D histogram
		if (this->oneDimensional)
		{
			maxNoBins = 0;
			//check which dimensions are used
			for (int i = 0; i < 3; i++)
			{
				if (this->chBinSize[i] > 0)
					maxNoBins += this->chNoBins[i];
			}
			chPtsF = 2 * this->chPts;
		}
		else // MD histogram
		{
			maxNoBins = 1;
			//check which dimension are used
			for (int i = 0; i < 3; i++)
			{
				if (this->chBinSize[i] > 0)
					maxNoBins *= this->chNoBins[i];
			}
			chPtsF = this->chPts;
		}
		//create color histogram arrays
		double *thisCH = new double[maxNoBins];
		memset(thisCH, 0, maxNoBins * sizeof(double));
		double *otherCH = new double[maxNoBins];
		memset(otherCH, 0, maxNoBins * sizeof(double));
		//fill arrays
		//iterating through current color histogram
		for (colorhistogram_iterator_type bin = this->colorhistogram.begin(); bin != this->colorhistogram.end(); bin++)
		{
			// iterator->first = key
			// iterator->second = value
			thisCH[bin->first] = (double)bin->second / chPtsF;
		}
		//iterating through other color histogram
		for (colorhistogram_iterator_type bin = other.colorhistogram.begin(); bin != other.colorhistogram.end(); bin++)
		{
			// iterator->first = key
			// iterator->second = value
			otherCH[bin->first] = (double)bin->second / chPtsF;
		}

		//calculating metric
		for (int i = 0; i < maxNoBins; i++)
		{
			d += abs(thisCH[i] - otherCH[i]);
		}

		//dereference
		delete[] thisCH;
		delete[] otherCH;
		//
		return d;
	}

	double RVLColorDescriptor::L2Distance(RVLColorDescriptor& other)
	{
		double d = 0.0;
		//Find max number of bins
		int maxNoBins;
		double chPtsF;
		// if 1D histogram
		if (this->oneDimensional)
		{
			maxNoBins = 0;
			//check which dimensions are used
			for (int i = 0; i < 3; i++)
			{
				if (this->chBinSize[i] > 0)
					maxNoBins += this->chNoBins[i];
			}
			chPtsF = 2 * this->chPts;
		}
		else // MD histogram
		{
			maxNoBins = 1;
			//check which dimension are used
			for (int i = 0; i < 3; i++)
			{
				if (this->chBinSize[i] > 0)
					maxNoBins *= this->chNoBins[i];
			}
			chPtsF = this->chPts;
		}
		//create color histogram arrays
		double *thisCH = new double[maxNoBins];
		memset(thisCH, 0, maxNoBins * sizeof(double));
		double *otherCH = new double[maxNoBins];
		memset(otherCH, 0, maxNoBins * sizeof(double));
		//fill arrays
		//iterating through current color histogram
		for (colorhistogram_iterator_type bin = this->colorhistogram.begin(); bin != this->colorhistogram.end(); bin++)
		{
			// iterator->first = key
			// iterator->second = value
			thisCH[bin->first] = (double)bin->second / chPtsF;
		}
		//iterating through other color histogram
		for (colorhistogram_iterator_type bin = other.colorhistogram.begin(); bin != other.colorhistogram.end(); bin++)
		{
			// iterator->first = key
			// iterator->second = value
			otherCH[bin->first] = (double)bin->second / chPtsF;
		}

		//calculating metric
		double temp;
		for (int i = 0; i < maxNoBins; i++)
		{
			temp = thisCH[i] - otherCH[i];
			d += temp * temp;
		}
		d = sqrt(d);

		//dereference
		delete[] thisCH;
		delete[] otherCH;
		//
		return d;
	}

	double RVLColorDescriptor::Intersection(RVLColorDescriptor& other)
	{
		double d = 0.0;
		//Find max number of bins
		int maxNoBins;
		double chPtsF;
		double chPtsF_o;
		// if 1D histogram
		if (this->oneDimensional)
		{
			maxNoBins = 0;
			//check which dimensions are used
			for (int i = 0; i < 3; i++)
			{
				if (this->chBinSize[i] > 0)
					maxNoBins += this->chNoBins[i];
			}
			chPtsF = 2 * this->chPts;
		}
		else // MD histogram
		{
			maxNoBins = 1;
			//check which dimension are used
			for (int i = 0; i < 3; i++)
			{
				if (this->chBinSize[i] > 0)
					maxNoBins *= this->chNoBins[i];
			}
			chPtsF = this->chPts;
		}
		//create color histogram arrays
		double *thisCH = new double[maxNoBins];
		memset(thisCH, 0, maxNoBins * sizeof(double));
		double *otherCH = new double[maxNoBins];
		memset(otherCH, 0, maxNoBins * sizeof(double));
		//fill arrays
		//iterating through current color histogram
		for (colorhistogram_iterator_type bin = this->colorhistogram.begin(); bin != this->colorhistogram.end(); bin++)
		{
			// iterator->first = key
			// iterator->second = value
			thisCH[bin->first] = (double)bin->second / chPtsF;
		}
		//iterating through other color histogram
		chPtsF_o = other.chPts;
		for (colorhistogram_iterator_type bin = other.colorhistogram.begin(); bin != other.colorhistogram.end(); bin++)
		{
			// iterator->first = key
			// iterator->second = value
			otherCH[bin->first] = (double)bin->second / chPtsF_o;
		}

		//calculating metric
		for (int i = 0; i < maxNoBins; i++)
		{
			if (thisCH[i] < otherCH[i])
				d += thisCH[i];
			else
				d += otherCH[i];
		}

		//dereference
		delete[] thisCH;
		delete[] otherCH;
		//
		return d;
	}

	double RVLColorDescriptor::BhattacharyyaDistance(RVLColorDescriptor& other)
	{
		double d = 0.0;
		//Find max number of bins
		int maxNoBins;
		double chPtsF;
		double chPtsF_o;
		// if 1D histogram
		if (this->oneDimensional)
		{
			maxNoBins = 0;
			//check which dimensions are used
			for (int i = 0; i < 3; i++)
			{
				if (this->chBinSize[i] > 0)
					maxNoBins += this->chNoBins[i];
			}
			chPtsF = 2 * this->chPts;
		}
		else // MD histogram
		{
			maxNoBins = 1;
			//check which dimension are used
			for (int i = 0; i < 3; i++)
			{
				if (this->chBinSize[i] > 0)
					maxNoBins *= this->chNoBins[i];
			}
			chPtsF = this->chPts;
		}
		//create color histogram arrays
		double *thisCH = new double[maxNoBins];
		memset(thisCH, 0, maxNoBins * sizeof(double));
		double *otherCH = new double[maxNoBins];
		memset(otherCH, 0, maxNoBins * sizeof(double));
		//fill arrays
		//iterating through current color histogram
		for (colorhistogram_iterator_type bin = this->colorhistogram.begin(); bin != this->colorhistogram.end(); bin++)
		{
			// iterator->first = key
			// iterator->second = value
			thisCH[bin->first] = (double)bin->second / chPtsF;
		}
		//iterating through other color histogram
		chPtsF_o = other.chPts;
		for (colorhistogram_iterator_type bin = other.colorhistogram.begin(); bin != other.colorhistogram.end(); bin++)
		{
			// iterator->first = key
			// iterator->second = value
			otherCH[bin->first] = (double)bin->second / chPtsF_o;
		}

		//calculating metric
		//double temp;
		for (int i = 0; i < maxNoBins; i++)
		{
			//temp = thisCH[i] - otherCH[i];
			d += sqrt(thisCH[i] * otherCH[i]);
		}
		d = (-1) * log(d);

		//dereference
		delete[] thisCH;
		delete[] otherCH;
		//
		return d;
	}

	double RVLColorDescriptor::CalculateCHMetric(RVLColorDescriptor& other, int metric)
	{
		//check if both histograms have same parameters
		if ((this->colorspace != other.colorspace) || (this->chBinSize[0] != other.chBinSize[0]) || (this->chBinSize[1] != other.chBinSize[1]) || (this->chBinSize[2] != other.chBinSize[2]) || (this->chNoBins[0] != other.chNoBins[0]) || (this->chNoBins[1] != other.chNoBins[1]) || (this->chNoBins[2] != other.chNoBins[2]) || (this->noChannelsUsed != other.noChannelsUsed) || (this->oneDimensional != other.oneDimensional))
			return -1.0;

		if (metric == RVLColorDescriptor::MetricsList::L1)
			return this->L1Distance(other);
		else if (metric == RVLColorDescriptor::MetricsList::L2)
			return this->L2Distance(other);
		else if (metric == RVLColorDescriptor::MetricsList::Intersection)
			return this->Intersection(other);
		else if (metric == RVLColorDescriptor::MetricsList::Bhattacharyya)
			return this->BhattacharyyaDistance(other);
		//else
		return -1.0;
	}

	void RVLColorDescriptor::DisplayColorHistogram(bool usescale)
	{
		// Initialize VTK.
		vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
		vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow>::New();
		vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
		window->AddRenderer(renderer);
		window->SetSize(800, 600);
		interactor->SetRenderWindow(window);
		vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
		interactor->SetInteractorStyle(style);
		renderer->SetBackground(0.5294, 0.8078, 0.9803);

		//Points, colors and scales
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
		colors->SetNumberOfComponents(3);
		colors->SetName("RGB");
		vtkSmartPointer<vtkFloatArray> scales = vtkSmartPointer<vtkFloatArray>::New();
		scales->SetNumberOfComponents(3);
		scales->SetName("scales");

		cv::Mat colorPts(1, this->colorhistogram.size(), CV_8UC3, cv::Scalar::all(0));
		float xyz[3];
		int binOffset = 0;
		float phi;
		float tempf;
		int i = 0;
		float scale[3] = { 0.0, 0.0, 0.0 };
		//iterating through color histogram 
		for (colorhistogram_iterator_type bin = this->colorhistogram.begin(); bin != this->colorhistogram.end(); bin++)
		{
			// iterator->first = key
			// iterator->second = value
			if (this->oneDimensional)	//1D histogram
			{
				//needs to add up to 3 points to histogram
				for (int k = 0; k < 3; k++)
				{
					xyz[0] = (float)bin->first - binOffset;
					//update offset
					binOffset += this->chNoBins[k];
				}
			}
			else
			{
				if (this->noChannelsUsed == 3) //3D histograms
				{
					//binAdr = (int)(floor(color[0] / this->chBinSize[0]) * this->chNoBins[1] * this->chNoBins[2] + floor(color[1] / this->chBinSize[1]) * this->chNoBins[2] + floor(color[2] / this->chBinSize[2]));
					//bin coordinates
					xyz[0] = floor((float)bin->first / (this->chNoBins[1] * this->chNoBins[2]));
					xyz[1] = floor(((float)bin->first - xyz[0] * (this->chNoBins[1] * this->chNoBins[2])) / (this->chNoBins[2]));
					xyz[2] = floor((float)bin->first - xyz[0] * (this->chNoBins[1] * this->chNoBins[2]) - (xyz[1] * this->chNoBins[2]));
				}
				else if (this->noChannelsUsed == 2) //2D histograms
				{
					if (this->chBinSize[0] == 0) //first channel is not used
					{
						xyz[0] = 0.0;
						xyz[1] = floor((float)bin->first / (this->chNoBins[2]));
						xyz[2] = floor((float)bin->first - xyz[1] * (this->chNoBins[2]));
					}
					else if (this->chBinSize[1] == 0) //second channel is not used
					{
						xyz[0] = floor((float)bin->first / (this->chNoBins[2]));
						xyz[1] = 0.0;
						xyz[2] = floor((float)bin->first - xyz[0] * (this->chNoBins[2]));
					}
					else //third channel is not used
					{
						xyz[0] = floor((float)bin->first / (this->chNoBins[1]));
						xyz[1] = floor((float)bin->first - xyz[0] * (this->chNoBins[1]));
						xyz[2] = 0.0;
					}
				}
			}
			//color space coordinates
			if (this->colorspace == RVLColorDescriptor::ColorSpaceList::HSV)
			{
				xyz[0] = xyz[0] * this->chBinSize[0] + this->chBinSize[0] / 2;	//OpenCV H channel is in range 0 -> 180
				xyz[1] = xyz[1] * this->chBinSize[1] + this->chBinSize[1] / 2;
				xyz[2] = xyz[2] * this->chBinSize[2] + this->chBinSize[2] / 2;
				colorPts.at<cv::Vec3b>(i)[0] = xyz[0];
				colorPts.at<cv::Vec3b>(i)[1] = xyz[1];
				if ((xyz[2] == 0) && (this->chNoBins[2] == 0))
					colorPts.at<cv::Vec3b>(i)[2] = 255;
				else
					colorPts.at<cv::Vec3b>(i)[2] = xyz[2];
				phi = xyz[0] * 2.0; //OpenCV H channel is in range 0 -> 180
				xyz[0] = cosf(phi * (CV_PI / 180.0)) * xyz[1];
				xyz[1] = sinf(phi * (CV_PI / 180.0)) * xyz[1];

			}
			else
			{
				xyz[0] = xyz[0] * this->chBinSize[0] + this->chBinSize[0] / 2;
				xyz[1] = xyz[1] * this->chBinSize[1] + this->chBinSize[1] / 2;
				xyz[2] = xyz[2] * this->chBinSize[2] + this->chBinSize[2] / 2;
				if ((this->colorspace == RVLColorDescriptor::ColorSpaceList::Lab) && (xyz[0] == 0) && (this->chNoBins[0] == 0))
					colorPts.at<cv::Vec3b>(i)[0] = 127;
				else
					colorPts.at<cv::Vec3b>(i)[0] = xyz[0];
				colorPts.at<cv::Vec3b>(i)[1] = xyz[1];
				colorPts.at<cv::Vec3b>(i)[2] = xyz[2];
			}
			//Reverse Y and Z coordinates for better visualization
			tempf = xyz[1];
			xyz[1] = xyz[2];
			xyz[2] = tempf;
			//Insert point
			points->InsertNextPoint(xyz);
			//Insert scale
			scale[0] = ((float)bin->second / this->chPts);
			scales->InsertNextTypedTuple(scale);
			//std::cout << "scale " << i << ": " << (float)bin->second / this->chPts << std::endl;
			i++;
		}

		//Changing color to RGB (if needed)
		if (this->colorspace == RVLColorDescriptor::ColorSpaceList::HSV)
			cv::cvtColor(colorPts, colorPts, CV_HSV2RGB);
		else if (this->colorspace == RVLColorDescriptor::ColorSpaceList::Lab)
			cv::cvtColor(colorPts, colorPts, CV_Lab2RGB);
		else if (this->colorspace == RVLColorDescriptor::ColorSpaceList::BGR)
			cv::cvtColor(colorPts, colorPts, CV_BGR2RGB);
		//filling color array
		for (i = 0; i < this->colorhistogram.size(); i++)
		{
			colors->InsertNextTypedTuple(&colorPts.data[i * 3]);
		}

		//Create polydata
		vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
		pd->SetPoints(points);
		pd->GetPointData()->SetScalars(colors);
		pd->GetPointData()->AddArray(scales);
		pd->GetPointData()->SetActiveScalars("RGB");
		pd->GetPointData()->SetActiveVectors("scales");
		//pd->GetPointData()->SetScalars(scales);

		//Creating 3D glyph and source
		//find max radius
		double radius = 10000;
		for (int i = 0; i < 3; i++)
		{
			if ((this->chBinSize[i] < radius) && (this->chBinSize[i] != 0))
				radius = this->chBinSize[i];
		}
		vtkSmartPointer<vtkSphereSource> spheresource = vtkSmartPointer<vtkSphereSource>::New();
		spheresource->SetRadius(radius / 2.0);
		spheresource->SetPhiResolution(10);
		spheresource->SetThetaResolution(10);
		vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
		glyph3D->SetColorModeToColorByScalar();
		if (usescale)
			glyph3D->SetScaleModeToScaleByVector();
		else
			glyph3D->ScalingOff();
		glyph3D->ClampingOff();
		glyph3D->SetSourceConnection(spheresource->GetOutputPort());
		glyph3D->SetInputData(pd);

		//Creating mapper and actor
		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(glyph3D->GetOutputPort());
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);
		//Add actor
		renderer->AddActor(actor);

		//Creating outline
		if (this->colorspace == RVLColorDescriptor::ColorSpaceList::HSV)
		{
			vtkSmartPointer<vtkCylinderSource> cylindersource = vtkSmartPointer<vtkCylinderSource>::New();
			cylindersource->SetResolution(50);
			if (this->chBinSize[2] > 0)
				cylindersource->SetHeight(this->chBinSize[2] * this->chNoBins[2]);
			else
				cylindersource->SetHeight(radius);
			cylindersource->SetRadius(radius * 2.0 * this->chNoBins[1]);
			if (this->chBinSize[2] > 0)
				cylindersource->SetCenter(0, this->chBinSize[2] * this->chNoBins[2] / 2.0, 0);
			//else
			//	cylindersource->SetCenter(0, radius / 2.0, 0);
			vtkSmartPointer<vtkPolyDataMapper> csMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			csMapper->SetInputConnection(cylindersource->GetOutputPort());
			vtkSmartPointer<vtkActor> csActor = vtkSmartPointer<vtkActor>::New();
			csActor->SetMapper(csMapper);
			csActor->GetProperty()->SetRepresentationToWireframe();
			//Add actor
			renderer->AddActor(csActor);
		}
		else
		{
			vtkSmartPointer<vtkCubeSource> cubesource = vtkSmartPointer<vtkCubeSource>::New();
			cubesource->SetXLength(this->chBinSize[0] * this->chNoBins[0]);
			cubesource->SetYLength(this->chBinSize[1] * this->chNoBins[1]);
			cubesource->SetZLength(this->chBinSize[2] * this->chNoBins[2]);
			cubesource->SetCenter(127, 127, 127);
			vtkSmartPointer<vtkPolyDataMapper> csMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			csMapper->SetInputConnection(cubesource->GetOutputPort());
			vtkSmartPointer<vtkActor> csActor = vtkSmartPointer<vtkActor>::New();
			csActor->SetMapper(csMapper);
			csActor->GetProperty()->SetRepresentationToWireframe();
			//Add actor
			renderer->AddActor(csActor);
		}

		//start interactor
		interactor->Start();
	}

	void RVLColorDescriptor::TestCHVisualization(int colorspace)
	{
		uint8_t *testRGB;
		int i = 0;
		int arraysize = 0;
		if ((colorspace == RVLColorDescriptor::ColorSpaceList::RGB) || (colorspace == RVLColorDescriptor::ColorSpaceList::Lab))
		{
			testRGB = new uint8_t[256 * 256 * 256 * 3];
			arraysize = 256 * 256 * 256;
			for (int r = 0; r < 256; r++)
			{
				for (int g = 0; g < 256; g++)
				{
					for (int b = 0; b < 256; b++)
					{
						testRGB[i * 3] = r;
						testRGB[i * 3 + 1] = g;
						testRGB[i * 3 + 2] = b;
						i++;
					}
				}
			}
		}
		else if (colorspace == RVLColorDescriptor::ColorSpaceList::HSV)
		{
			testRGB = new uint8_t[180 * 256 * 256 * 3];
			arraysize = 180 * 256 * 256;
			for (int h = 0; h < 180; h++)
			{
				for (int s = 0; s < 256; s++)
				{
					for (int v = 0; v < 256; v++)
					{
						testRGB[i * 3] = h;
						testRGB[i * 3 + 1] = s;
						testRGB[i * 3 + 2] = v;
						i++;
					}
				}
			}
		}

		int noBins[3] = { 8, 8, 8 };
		RVLColorDescriptor testCH(colorspace, false, noBins, true);
		testCH.InsertArrayIntoHistogram(testRGB, arraysize);
		testCH.DisplayColorHistogram(false);
	}

	void RVLColorDescriptor::SaveCH2File(FILE *fp)
	{
		if (fp)
		{
			//iterating through color histogram 
			for (colorhistogram_iterator_type bin = this->colorhistogram.begin(); bin != this->colorhistogram.end(); bin++)
			{
				fprintf(fp, "%d\t", bin->second);
			}

			fprintf(fp, "\n");
		}
	}
}
