#pragma once

struct FREIBURG_DATASET_SAMPLE
{
	int iSubset;
	int tRGBs;
	int tRGBus;
	int tDs;
	int tDus;
	double t[3];
	double R[9];
};

class CRVLFreiburgDataSet
{
public:
	CRVLFreiburgDataSet();
	virtual ~CRVLFreiburgDataSet();
	bool Load(char *DataSetFileName);
	char * GetDepthFileName(char *RGBFileName);
	char * GetNextRGBFileName(
		char *RGBFileName,
		bool bBackwards = false);
	char * GetValidRGBFileName(char *RGBFileName);
	void TransformDepthMap(
		RVLDISPARITYMAP *pDepthMap,
		short *zToDepthLookupTable);
	bool GetSample(
		char *RGBFileName,
		FREIBURG_DATASET_SAMPLE *pSample);

private:
	char * CreateImageFileName(
		FREIBURG_DATASET_SAMPLE *pSample,
		bool bDepth = false);
	void GetTime(
		char *RGBFileName,
		int &ts,
		int &tus);
	int GetTimeDifference(
		int ts1,
		int tus1,
		int ts2,
		int tus2);

public:
	int m_maxet;
	int m_SampleTime;
	int m_SampleStep;
	bool m_bSampleTime;

private:
	std::vector<char *> m_SubsetArray;
	std::vector<FREIBURG_DATASET_SAMPLE> m_SampleArray;
	char *m_DepthDirectory;
	char *m_RGBDirectory;
};

