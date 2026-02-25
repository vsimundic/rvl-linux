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


#ifndef SVMCLASSIFIER_H
#define SVMCLASSIFIER_H


class SVMClassifier
{

private:

	unsigned int noInputVars;			//number of input variables
	unsigned int noSupportVectors;		//number of support vectors

	double gaussianScale;				//scale factor of gaussian kernel
	double aParamSigmoid;				//slope of sigmoid
	double cParamSigmoid;				//intercept of sigmoid
	double bias;						//bias term of SVM classifier
	double *alfaCoeff = 0;			//alfa coefficients of SVM classifier
	double *classSV = 0;				//class of each support vector
	double *valSV = 0;				//values of each support vector
	double *meanVal = 0;				//standardization; mean values
	double *stdVal = 0;				//standardization; sigma values
	double *queryNorm = 0;			//temp

	int flagError;						//0 if there is no error

	//result of classification (0-1)
	int result;


public:

	SVMClassifier(char *filename);

	~SVMClassifier();

	double makeClassification(double *query, unsigned int sizeQuery);

	int getInputSize();

};

#endif