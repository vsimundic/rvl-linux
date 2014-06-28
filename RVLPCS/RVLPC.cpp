#include "RVLCore.h"
#include "RVLPC.h"

bool RVLPCImport(char *FileName, double *X, int &n)
{
	FILE *fp = fopen(FileName, "r");

	if(fp == NULL)
		return false;

	fscanf(fp, "%d\n", &n);

	double *X_ = X;

	int i;
	double X__[3];

	for(i = 0; i < n; i++, X_ += 3)
	{
		fscanf(fp, "%lf %lf %lf\n", X__, X__ + 1, X__ + 2);

		X_[0] = -1000.0 * X__[1];
		X_[1] = -1000.0 * X__[2];
		X_[2] = 1000.0 * X__[0];
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
