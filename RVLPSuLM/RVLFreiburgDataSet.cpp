#include "RVLCore.h"
#include "RVLFreiburgDataSet.h"


CRVLFreiburgDataSet::CRVLFreiburgDataSet()
{
	//m_maxet = 60000;
	m_maxet = 200000;
	m_SampleTime = 500000;
	m_SampleStep = 1;
	m_bSampleTime = false;
	m_DepthDirectory = RVLCreateString("\\depth\\");
	m_RGBDirectory = RVLCreateString("\\rgb\\");
}


CRVLFreiburgDataSet::~CRVLFreiburgDataSet()
{
	for (std::vector<char *>::iterator Subset = m_SubsetArray.begin(); Subset != m_SubsetArray.end(); ++Subset)
		delete[] *Subset;

	delete[] m_DepthDirectory;
	delete[] m_RGBDirectory;
}


bool CRVLFreiburgDataSet::Load(char *DataSetFileName)
{
	FILE *fpDataSet = fopen(DataSetFileName, "r");

	if (fpDataSet == NULL)
		return false;

	char RGBFileName_[] = "\\rgb.txt";
	char DFileName_[] = "\\depth.txt";
	char GTFileName_[] = "\\groundtruth.txt";

	int iSubset = 0;

	CRVL3DPose GT;

	double *t = GT.m_X;
	double *R = GT.m_Rot;
	double q[4];
	GT.m_q = q;

	CRVL3DPose GTPrev;

	double *tPrev = GTPrev.m_X;
	double *RPrev = GTPrev.m_Rot;
	double qPrev[4];
	GTPrev.m_q = qPrev;

	CRVL3DPose dGT;

	double *dR = dGT.m_Rot;
	double *dt = dGT.m_X;

	char sLine[500];
	int SetDirectoryNameLen;
	char *SubsetFileName;
	FILE *fpDepth, *fpRGB, *fpGT;
	char *RGBFileName, *DFileName, *GTFileName;
	FREIBURG_DATASET_SAMPLE Sample;
	int tDs, tDus, tDsPrev, tDusPrev, tGTs, tGTus, tGTsPrev, tGTusPrev, dtGT;
	int et, etPrev;
	bool btDReached;
	double s, dq;
	double V[3];

	while (!feof(fpDataSet))
	{
		// Get next subset

		fgets(sLine, 500, fpDataSet);		

		SetDirectoryNameLen = strlen(sLine);

		SubsetFileName = new char[SetDirectoryNameLen + 1];

		strcpy(SubsetFileName, sLine);

		m_SubsetArray.push_back(SubsetFileName);

		// Open RGB sample list file.

		RGBFileName = new char[SetDirectoryNameLen + strlen(RGBFileName_) + 1];

		strcpy(RGBFileName, SubsetFileName);

		strcat(RGBFileName, RGBFileName_);

		fpRGB = fopen(RGBFileName, "r");

		if (fpRGB == NULL)
			return false;

		// Open depth sample list file.

		DFileName = new char[SetDirectoryNameLen + strlen(DFileName_) + 1];

		strcpy(DFileName, SubsetFileName);

		strcat(DFileName, DFileName_);

		fpDepth = fopen(DFileName, "r");

		if (fpDepth == NULL)
			return false;

		// Open ground truth file.

		GTFileName = new char[SetDirectoryNameLen + strlen(GTFileName_)];

		strcpy(GTFileName, SubsetFileName);

		strcat(GTFileName, GTFileName_);

		fpGT = fopen(GTFileName, "r");

		if (fpGT == NULL)
			return false;

		// Get samples.

		tDs = tDsPrev = -1;
		tDus = tDusPrev = 0;
		tGTs = tGTsPrev = -1;
		tGTus = tGTusPrev = 0;

		while (!feof(fpRGB))
		{
			// Get next RGB sample.

			fgets(sLine, 500, fpRGB);

			if (sLine[0] == '#')
				continue;

			sscanf(sLine, "%d.%d", &(Sample.tRGBs), &(Sample.tRGBus));

			//if (Sample.tRGBs == 1311878278 && Sample.tRGBus == 47881)
			//	int debug = 0;

			Sample.iSubset = iSubset;

			// Get depth sample closest to the RGB sample.

			while (tDs < Sample.tRGBs || (tDs == Sample.tRGBs && tDus < Sample.tRGBus))
			{
				if (feof(fpDepth))
					break;

				fgets(sLine, 500, fpDepth);

				if (sLine[0] != '#')
				{
					tDsPrev = tDs;
					tDusPrev = tDus;

					sscanf(sLine, "%d.%d", &tDs, &tDus);
				}
			}

			if (tDs >= 0)
			{
				et = GetTimeDifference(tDs, tDus, Sample.tRGBs, Sample.tRGBus);

				if (tDs > Sample.tRGBs || (tDs == Sample.tRGBs && tDus >= Sample.tRGBus))
					etPrev = (tDsPrev > 0 ? GetTimeDifference(tDsPrev, tDusPrev, Sample.tRGBs, Sample.tRGBus) : 2000000);
				else
					etPrev = 2000000;
			}
			else
				break;

			if (et <= etPrev)
			{
				Sample.tDs = tDs;
				Sample.tDus = tDus;
			}
			else
			{
				Sample.tDs = tDsPrev;
				Sample.tDus = tDusPrev;
				et = etPrev;
			}

			if (et > m_maxet)
				continue;

			// Get GT corresponding to the depth sample.

			while (!(btDReached = (tGTs > Sample.tDs || (tGTs == Sample.tDs && tGTus >= Sample.tDus))))
			{
				if (feof(fpGT))
					break;

				fgets(sLine, 500, fpGT);

				if (sLine[0] != '#')
				{
					tGTsPrev = tGTs;
					tGTusPrev = tGTus;
					RVLCOPY3VECTOR(t, tPrev);
					RVLCOPY3VECTOR(q, qPrev);
					qPrev[3] = q[3];

					sscanf(sLine, "%d.%d %lf %lf %lf %lf %lf %lf %lf", &tGTs, &tGTus, t, t + 1, t + 2, q, q + 1, q + 2, q + 3);

					tGTus *= 100;
				}
			}

			if (!btDReached)
				break;

			if (tGTsPrev < 0)
				continue;

			et = GetTimeDifference(tGTs, tGTus, Sample.tDs, Sample.tDus);

			if (et > m_maxet)
				continue;

			dtGT = GetTimeDifference(tGTs, tGTus, tGTsPrev, tGTusPrev);

			s = (double)et / (double)dtGT;

			RVLDIF3VECTORS(t, tPrev, dt);
			RVLSCALE3VECTOR(dt, s, dt);
			RVLDIF3VECTORS(t, dt, Sample.t);

			GT.UpdateRotFromQuat();
			GTPrev.UpdateRotFromQuat();
			RVLMXMUL3X3T1(R, RPrev, dR);
			dGT.GetAngleAxis(V, dq);
			dGT.UpdateRotFromAngleAxis(V, s * dq);
			RVLMXMUL3X3(R, dR, Sample.R);

			//// Only for debugging purpose !!!
			//GT.UpdatePTRLA();
			//GTPrev.UpdatePTRLA();
			//CRVL3DPose PoseTmp;
			//RVLCOPYMX3X3(Sample.R, PoseTmp.m_Rot);
			//PoseTmp.UpdatePTRLA();
			///////

			// Store sample in m_SampleArray.

			m_SampleArray.push_back(Sample);
		}

		iSubset++;

		fclose(fpRGB);
		fclose(fpDepth);

		delete[] DFileName;
	}

	fclose(fpDataSet);

	return true;
}

int CRVLFreiburgDataSet::GetTimeDifference(
	int ts1,
	int tus1,
	int ts2,
	int tus2)
{
	int et = ts1 - ts2;

	if (et > 2)
		return 2000000;
	else if (et < -2)
		return 2000000;
	else
	{
		et = 1000000 * et + (tus1 - tus2);

		return (et >= 0 ? et : -et);
	}
}

char * CRVLFreiburgDataSet::CreateImageFileName(
	FREIBURG_DATASET_SAMPLE *pSample,
	bool bDepth)
{
	int iSubset = pSample->iSubset;

	char *ImageTypeDirectory = (bDepth ? m_DepthDirectory : m_RGBDirectory);

	int PathLen = strlen(m_SubsetArray[iSubset]) + strlen(ImageTypeDirectory);

	char *FileName = new char[PathLen + 22];

	strcpy(FileName, m_SubsetArray[iSubset]);
	strcat(FileName, ImageTypeDirectory);
	if (bDepth)
		sprintf(FileName + PathLen, "%d.%06d.png", pSample->tDs, pSample->tDus);
	else
		sprintf(FileName + PathLen, "%d.%06d.png", pSample->tRGBs, pSample->tRGBus);

	return FileName;
}

void CRVLFreiburgDataSet::GetTime(
	char *RGBFileName,
	int &ts,
	int &tus)
{
	sscanf(strrchr(RGBFileName, 92) + 1, "%d.%d.png", &ts, &tus);
}

bool CRVLFreiburgDataSet::GetSample(
	char *RGBFileName,
	FREIBURG_DATASET_SAMPLE *pSample_
	)
{
	int tRGBs, tRGBus;

	GetTime(RGBFileName, tRGBs, tRGBus);

	for (std::vector<FREIBURG_DATASET_SAMPLE>::iterator pSample = m_SampleArray.begin(); pSample != m_SampleArray.end(); ++pSample)
	{
		if (pSample->tRGBs == tRGBs && pSample->tRGBus == tRGBus)
		{
			*pSample_ = *pSample;

			return true;
		}
	}

	return false;
}

char * CRVLFreiburgDataSet::GetDepthFileName(char *RGBFileName)
{
	FREIBURG_DATASET_SAMPLE	Sample;

	if(GetSample(RGBFileName, &Sample))
		return CreateImageFileName(&Sample, true);
	else
		return NULL;
}

char * CRVLFreiburgDataSet::GetNextRGBFileName(
	char *RGBFileName,
	bool bBackwards)
{
	int tRGBs, tRGBus;

	GetTime(RGBFileName, tRGBs, tRGBus);

	int direction = (bBackwards ? -1 : 1);

	if (m_bSampleTime)
	{
		tRGBus += m_SampleTime;
		tRGBs += (tRGBus / 1000000);
		tRGBus = tRGBus % 1000000;
	}

	int iSample = 0;

	FREIBURG_DATASET_SAMPLE	Sample;

	for (std::vector<FREIBURG_DATASET_SAMPLE>::iterator pSample = m_SampleArray.begin(); pSample != m_SampleArray.end(); ++pSample, iSample++)
	{
		if (pSample->tRGBs > tRGBs || (pSample->tRGBs == tRGBs && pSample->tRGBus >= tRGBus))
		{
			if (!m_bSampleTime)
			{
				iSample += (direction * m_SampleStep);

				if (iSample < 0)
					Sample = m_SampleArray[0];
				else if (iSample < (int)(m_SampleArray.size()))
					Sample = m_SampleArray[iSample];
				else
					return NULL;
			}
			else
				Sample = *pSample;

			return CreateImageFileName(&Sample);
		}
	}

	return NULL;
}

char * CRVLFreiburgDataSet::GetValidRGBFileName(char *RGBFileName)
{
	int tRGBs, tRGBus;

	GetTime(RGBFileName, tRGBs, tRGBus);

	for (std::vector<FREIBURG_DATASET_SAMPLE>::iterator pSample = m_SampleArray.begin(); pSample != m_SampleArray.end(); ++pSample)
	{
		if (pSample->tRGBs > tRGBs || (pSample->tRGBs == tRGBs && pSample->tRGBus >= tRGBus))
		{
			FREIBURG_DATASET_SAMPLE	Sample = *pSample;

			return CreateImageFileName(&Sample);
		}
	}

	return NULL;
}

void CRVLFreiburgDataSet::TransformDepthMap(
	RVLDISPARITYMAP *pDepthMap,
	short *zToDepthLookupTable)
{
	unsigned short int *pDisparityArrayEnd = (unsigned short int *)(pDepthMap->Disparity) + pDepthMap->Width * pDepthMap->Height;

	unsigned short int *pDisparity;

	for (pDisparity = (unsigned short int *)(pDepthMap->Disparity); pDisparity < pDisparityArrayEnd; pDisparity++)
		*pDisparity = zToDepthLookupTable[(*pDisparity) / 5];
}