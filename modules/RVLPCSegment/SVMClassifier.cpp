/****************************************************************************
*
* SVM binary classifier
*
* Outputs probability (0-1) if ok, else negative value (error code)
*
* Error codes:
*	-1	wrong file name
*	-2	error in string conversion lines of file
*	-3	cannot read file line
*	-4	memory allocation problem
*	-5	wrong size of query point
*****************************************************************************/

#include "SVMClassifier.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>


SVMClassifier::SVMClassifier(char *fileName)
{
	flagError = 0;

	std::ifstream fileInput(fileName);

	if (!fileInput) {
		flagError = -1;
	}
	else
	{
		std::string line;

		int lineCnt = 0;

		while (lineCnt < 6 && std::getline(fileInput, line))
		{
			lineCnt++;
			try
			{
				if (lineCnt==1)
					noInputVars = std::stoi(line);
				else if (lineCnt == 2)
					noSupportVectors = std::stoi(line);
				else if (lineCnt == 3)
					bias = std::stod(line);
				else if (lineCnt == 4)
					gaussianScale = std::stod(line);
				else if (lineCnt == 5)
					aParamSigmoid = std::stod(line);
				else if (lineCnt == 6)
					cParamSigmoid = std::stod(line);
			}
			catch (...)
			{
				flagError = -2;
				break;
			}
		}

		if (lineCnt < 6)
			flagError = -3;


		/*
		//1. line = number of input variables
		if(std::getline(fileInput, line))
		{
			try
			{
				noInputVars = std::stoi(line);
			}
			catch (...)
			{
				flagError = -2;
			}
		}
		else
		{
			flagError = -3;
		}

		//2. line = number of input variables
		if (std::getline(fileInput, line))
		{
			if (flagError == 0)
			{
				try
				{
					noSupportVectors = std::stoi(line);
				}
				catch (...)
				{
					flagError = -2;
				}
			}
		}
		else
		{
			flagError = -3;
		}

		//3. line = bias
		if (std::getline(fileInput, line))
		{
			if (flagError == 0)
			{
				try
				{
					bias = std::stod(line);
				}
				catch (...)
				{
					flagError = -2;
				}
			}
		}
		else
		{
			flagError = -3;
		}

		//4. line = scale
		if (std::getline(fileInput, line))
		{
			if (flagError == 0)
			{
				try
				{
					gaussianScale = std::stod(line);
				}
				catch (...)
				{
					flagError = -2;
				}
			}
		}
		else
		{
			flagError = -3;
		}

		//5. line = slope of sigmoid
		if (std::getline(fileInput, line))
		{
			if (flagError == 0)
			{
				try
				{
					aParamSigmoid = std::stod(line);
				}
				catch (...)
				{
					flagError = -2;
				}
			}
		}
		else
		{
			flagError = -3;
		}

		//6. line = intercept of sigmoid
		if (std::getline(fileInput, line))
		{
			if (flagError == 0)
			{
				try
				{
					cParamSigmoid = std::stod(line);
				}
				catch (...)
				{
					flagError = -2;
				}
			}
		}
		else
		{
			flagError = -3;
		}
		*/

		//allocate space for arrays
		valSV = new double[noSupportVectors*noInputVars];
		classSV = new double[noSupportVectors];
		alfaCoeff = new double[noSupportVectors];
		meanVal = new double[noInputVars];
		stdVal = new double[noInputVars];
		queryNorm = new double[noInputVars];

		if (valSV == NULL || classSV == NULL || alfaCoeff == NULL || meanVal == NULL || stdVal == NULL || queryNorm == NULL)
		{
			flagError = -4;

			if (valSV != NULL)
				delete[] valSV;

			if (classSV != NULL)
				delete[] valSV;

			if (alfaCoeff != NULL)
				delete[] valSV;

			if (meanVal != NULL)
				delete[] valSV;

			if (stdVal != NULL)
				delete[] valSV;

			if (queryNorm != NULL)
				delete[] queryNorm;
		}


		//read standardization parameters - mean values
		std::string temp;
		unsigned int i = 0, start = 0, end;
		
		if (std::getline(fileInput, line))
		{
			if (flagError == 0)
			{
				i = 0;
				start = 0;

				do {

					end = line.find_first_of(',', start);
					temp = line.substr(start, (end - start));

					meanVal[i] = atof(temp.c_str());

					i++;
					start = end + 1;

				} while (start && i < noInputVars);
			}
		}
		else
		{
			flagError = -3;
		}
		
		//read standardization parameters - sigma values 
		if (std::getline(fileInput, line) && flagError == 0)
		{
			i = 0;
			start = 0;

			do {

				end = line.find_first_of(',', start);
				temp = line.substr(start, (end - start));

				stdVal[i] = atof(temp.c_str());

				i++;
				start = end + 1;

			} while (start && i < noInputVars);
		}
		else
		{
			flagError = -3;
		}
		

		//read support vectors
		unsigned int k = 0;

		while (std::getline(fileInput, line) && flagError == 0)
		{

			i = 0;
			start = 0;

			do {

				end = line.find_first_of(',', start);
				temp = line.substr(start, (end - start));

				if (i < noInputVars)
				{
					valSV[noInputVars*k + i] = atof(temp.c_str());
				}
				else if (i == noInputVars)
				{
					classSV[k] = atof(temp.c_str());
				}
				else
				{
					alfaCoeff[k] = atof(temp.c_str());
				}

				i++;
				start = end + 1;

			} while (start);

			k++;
		}

		if (flagError != -1)
			fileInput.close();

		//do some precalculations
		for (i = 0; i < noSupportVectors; i++)
			alfaCoeff[i] *= classSV[i];

		gaussianScale *= gaussianScale;

	}
	
}


SVMClassifier::~SVMClassifier()
{

}


double SVMClassifier::makeClassification(double *query, unsigned int sizeQuery)
{

	double resultVal = 0.0;
	double temp = 0.0;
	unsigned int i = 0;
	unsigned int j = 0;
	
	if (sizeQuery != noInputVars)
		flagError = -5;

	if (flagError == 0)
	{
		//standardize input
		for (j = 0; j < noInputVars; j++)
			queryNorm[j] = (query[j] - meanVal[j]) / stdVal[j];
		
		//for each support vector
		for (i = 0; i < noSupportVectors; i++)
		{
			temp = 0.0;
			for (j = 0; j < noInputVars; j++)
			{
				temp += (valSV[j + i*noInputVars] - queryNorm[j])*(valSV[j + i*noInputVars] - queryNorm[j]);
			}
			
			resultVal += alfaCoeff[i] * exp(-temp / gaussianScale);
		}

		resultVal += bias;
		
		//make probability
		resultVal = 1 / (1 + exp(aParamSigmoid*resultVal + cParamSigmoid));
		
		return resultVal;

	}
	else
		return flagError;

}

int SVMClassifier::getInputSize()
{
	return noInputVars;
}