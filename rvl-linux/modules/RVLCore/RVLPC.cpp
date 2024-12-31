#include "RVLCore2.h"

bool RVLPCImport(char *FileName, double **pX, int &n, double scale)
{
	FILE *fp = fopen(FileName, "r");

	if(fp == NULL)
		return false;

	char line[200];

	fgets(line, 200, fp);

	if (line[0] == '#')
	{
		bool bData = false;

		int nLines = 0;

		n = 0;

		do
		{
			fgets(line, 200, fp);

			if (strncmp("POINTS", line, 6) == 0)
				sscanf(line, "POINTS %d\n", &n);

			nLines++;

			bData = (strncmp("DATA ascii", line, 10) == 0);
		} while (!bData && nLines < 20);

		if (n == 0 || !bData)
			return false;
	}
	else if (line[0] >= '0' && line[0] <= '9')
		sscanf(line, "%d\n", &n);
	else
		return false;

	double *X = *pX;

	if(X)
		delete[] X;

	X = new double[3 * n];

	*pX = X;

	double *X_ = X;

	int i;
	double X__[3];

	for(i = 0; i < n; i++, X_ += 3)
	{
		fscanf(fp, "%lf %lf %lf\n", X__, X__ + 1, X__ + 2);

		X_[0] = -scale * X__[1];
		X_[1] = -scale * X__[2];
		X_[2] = scale * X__[0];
	}

	fclose(fp);

	return true;
}

bool RVLPCSaveToObj(double *X, int n, char *FileName)
{
	FILE *fp = fopen(FileName, "w");

	if(fp == NULL)
		return false;

	double *X_ = X;

	int i;

	for(i = 0; i < n; i++, X_ += 3)
		fprintf(fp, "v %lf %lf %lf\n", X_[0], X_[1], X_[2]);

	fclose(fp);

	return true;
}

