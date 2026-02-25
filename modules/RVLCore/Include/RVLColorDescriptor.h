// RVLColorDescriptor.h: interface for the RVLColorDescriptor class.
//
// Generic class used for color matching
//////////////////////////////////////////////////////////////////////
#include <map>
#include <string>
#include <stdint.h>

namespace RVL
{
	class RVLColorDescriptor
	{
	public:
		//additional type definitions
		typedef std::map<int, unsigned int>::iterator colorhistogram_iterator_type;
		class ColorSpaceList
		{
		public:
			enum
			{
				RGB = 0,
				HSV,
				Lab,
				BGR,
			};
		};
		class MetricsList
		{
		public:
			enum
			{
				L1 = 0,
				L2,
				Intersection,
				Bhattacharyya,
			};
		};
		//

		//public parts
	public:
		//empty constructor
		RVLColorDescriptor()
		{
			this->colorspace = -1;
			this->noChannelsUsed = -1;
			this->oneDimensional = false;
			this->chBinSize[0] = -1;
			this->chBinSize[1] = -1;
			this->chBinSize[2] = -1;
			this->chNoBins[0] = -1;
			this->chNoBins[1] = -1;
			this->chNoBins[2] = -1;
			this->chFilterThr[0] = 0;
			this->chFilterThr[1] = 0;
			this->chFilterThr[2] = 0;
			this->chPts = 0;
		}
		//copy constructor
		RVLColorDescriptor(const RVLColorDescriptor &other)
		{
			this->colorhistogram = other.colorhistogram;
			this->chPts = other.chPts;
			this->colorspace = other.colorspace;
			this->noChannelsUsed = other.noChannelsUsed;
			this->oneDimensional = other.oneDimensional;
			this->chBinSize[0] = other.chBinSize[0];
			this->chBinSize[1] = other.chBinSize[1];
			this->chBinSize[2] = other.chBinSize[2];
			this->chNoBins[0] = other.chNoBins[0];
			this->chNoBins[1] = other.chNoBins[1];
			this->chNoBins[2] = other.chNoBins[2];
			this->chFilterThr[0] = other.chFilterThr[0];
			this->chFilterThr[1] = other.chFilterThr[1];
			this->chFilterThr[2] = other.chFilterThr[2];
		}
		//constructor, bsize must be 3-element array and it determines which color channel is used (and number of used channels) as well as bin width per channel
		RVLColorDescriptor(int cs, bool oneDim, const int *bindata, bool noBins, const int * chFilterThrreshold = NULL)	//RGB, 3D histogram, bin size 1
		{
			this->colorspace = cs;
			this->oneDimensional = oneDim;
			if (noBins)
			{
				this->chNoBins[0] = bindata[0];
				this->chNoBins[1] = bindata[1];
				this->chNoBins[2] = bindata[2];
				int *nob = NoBin2BinSize(bindata, cs);
				this->chBinSize[0] = nob[0];
				this->chBinSize[1] = nob[1];
				this->chBinSize[2] = nob[2];
				delete[] nob;
			}
			else
			{
				this->chBinSize[0] = bindata[0];
				this->chBinSize[1] = bindata[1];
				this->chBinSize[2] = bindata[2];
				int *nob = BinSize2NoBin(bindata, cs);
				this->chNoBins[0] = nob[0];
				this->chNoBins[1] = nob[1];
				this->chNoBins[2] = nob[2];
				delete[] nob;
			}
			this->noChannelsUsed = 0;
			if (bindata[0] > 0)
				this->noChannelsUsed++;
			if (bindata[1] > 0)
				this->noChannelsUsed++;
			if (bindata[2] > 0)
				this->noChannelsUsed++;
			if (chFilterThrreshold)
			{
				this->chFilterThr[0] = chFilterThrreshold[0];
				this->chFilterThr[1] = chFilterThrreshold[1];
				this->chFilterThr[2] = chFilterThrreshold[2];
			}
			else
			{
				this->chFilterThr[0] = 0;
				this->chFilterThr[1] = 0;
				this->chFilterThr[2] = 0;
			}

			this->chPts = 0;
		}
		//copy operator
		RVLColorDescriptor& operator= (const RVLColorDescriptor& other)
		{
			this->colorhistogram = other.colorhistogram;
			this->chPts = other.chPts;
			this->colorspace = other.colorspace;
			this->noChannelsUsed = other.noChannelsUsed;
			this->oneDimensional = other.oneDimensional;
			this->chBinSize[0] = other.chBinSize[0];
			this->chBinSize[1] = other.chBinSize[1];
			this->chBinSize[2] = other.chBinSize[2];
			this->chNoBins[0] = other.chNoBins[0];
			this->chNoBins[1] = other.chNoBins[1];
			this->chNoBins[2] = other.chNoBins[2];
			this->chFilterThr[0] = other.chFilterThr[0];
			this->chFilterThr[1] = other.chFilterThr[1];
			this->chFilterThr[2] = other.chFilterThr[2];
			return *this;
		}
		//add operator
		RVLColorDescriptor operator+ (RVLColorDescriptor& other)
		{
			RVLColorDescriptor newDesc(*this);
			//iterating through current color histogram
			for (colorhistogram_iterator_type bin = other.colorhistogram.begin(); bin != other.colorhistogram.end(); bin++)
			{
				// iterator->first = key
				// iterator->second = value
				//working with map
				if (newDesc.colorhistogram.find(bin->first) != newDesc.colorhistogram.end())
					newDesc.colorhistogram.at(bin->first) += bin->second;	//increment existing bin
				else
					newDesc.colorhistogram.insert(std::pair<int, unsigned int>(bin->first, bin->second)); //add new bin
			}
			newDesc.chPts += other.chPts;
			return newDesc;
		}
		//add operator
		RVLColorDescriptor& operator+= (RVLColorDescriptor& other)
		{
			//iterating through current color histogram
			for (colorhistogram_iterator_type bin = other.colorhistogram.begin(); bin != other.colorhistogram.end(); bin++)
			{
				// iterator->first = key
				// iterator->second = value
				//working with map
				if (this->colorhistogram.find(bin->first) != this->colorhistogram.end())
					this->colorhistogram.at(bin->first) += bin->second;	//increment existing bin
				else
					this->colorhistogram.insert(std::pair<int, unsigned int>(bin->first, bin->second)); //add new bin
			}
			this->chPts += other.chPts;
			return *this;
		}
		~RVLColorDescriptor(){};
		//Get methods
		int GetColorSpace(){ return this->colorspace; }
		int GetNoChannelsUsed(){ return this->noChannelsUsed; }
		bool GetOneDimensional(){ return this->oneDimensional; }
		int* GetBinSize(){ return this->chBinSize; }
		int* GetNoBins(){ return this->chNoBins; }
		int GetCHPts(){ return this->chPts; }
		//
		//Insert color point (int array with noChannels size) into color histogram. Colorspace, noChannels, noDimensions and binSize is taken from object parameters
		void InsertPointIntoHistogram(int *color, bool useFilter = false);
		//Insert array of color points (int array with arraysize * noChannels size) into color histogram. Colorspace, noChannels, noDimensions and binSize is taken from object parameters
		void InsertArrayIntoHistogram(int *color, int arraysize, bool useFilter = false);
		//Insert color point (uint8_t array with noChannels size) into color histogram. Colorspace, noChannels, noDimensions and binSize is taken from object parameters
		void InsertPointIntoHistogram(uint8_t *color, bool useFilter = false);
		//Insert array of color points (char array with arraysize * noChannels size) into color histogram. Colorspace, noChannels, noDimensions and binSize is taken from object parameters
		void InsertArrayIntoHistogram(uint8_t *color, int arraysize, bool useFilter = false);
		//Calculate color histogram distance to another
		double CalculateCHMetric(RVLColorDescriptor& other, int metric);
		//Helper function that generates an array with number of bins per dimension from an array with bin sizes per dimension, cs is color space
		static int* BinSize2NoBin(const int* binSize, int cs);
		//Helper function that generates an array with bin sizes per dimension from an array with number of bins per dimension, cs is color space
		static int* NoBin2BinSize(const int* noBins, int cs);
		//Visualize color histogram (not fully debuged!)
		void DisplayColorHistogram(bool usescale = true);
		//Test color histogram visualization
		static void TestCHVisualization(int colorspace);
		//Save color histogram to file
		void SaveCH2File(FILE *fp); //Vidovic

		//private parts
	private:
		std::map<int, unsigned int> colorhistogram;
		unsigned int chPts; //color histogram points
		int colorspace;
		int noChannelsUsed;	//max number of channels is 3
		bool oneDimensional;
		int chBinSize[3];	//Bin widths per dimension. Can support different bin sizes per dimension (max number of dimensions is 3)
		int chNoBins[3];	//Number of bins per dimension
		int chFilterThr[3];	//Filter threshold values per dimension
		//Calculate L1 distance between current and other RVLColorDescriptor 
		double L1Distance(RVLColorDescriptor& other);
		//Calculate L2 distance between current and other RVLColorDescriptor 
		double L2Distance(RVLColorDescriptor& other);
		//Calculate normalized interesection distance between current and other RVLColorDescriptor 
		double Intersection(RVLColorDescriptor& other);
		//Calculate Bhattacharyya distance between current and other RVLColorDescriptor 
		double BhattacharyyaDistance(RVLColorDescriptor& other);
	};
}