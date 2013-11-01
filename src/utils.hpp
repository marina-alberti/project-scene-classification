#include <string.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <cstring>
#include <dirent.h>
#include <cstdlib>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/opencv.hpp>
#include "opencv2/ml/ml.hpp"
#include <cmath>

#define DEBUG 0
#define PI 3.14159265

using namespace std;

vector<double> computeMean(cv::Mat &);
vector<double> computeStd(cv::Mat &, vector<double> );
cv::Mat doNormalization(cv::Mat &, vector<double> , vector<double>);
double computeGMMProbability(cv::Mat feats, cv::Mat means, vector<cv::Mat> covs, cv::Mat weights );
vector<double> computeMin(cv::Mat FeatMat);
vector<double> computeMax(cv::Mat FeatMat);
cv::Mat doNormalizationMinMax(cv::Mat & FeatMat, vector<double> maxvector, vector<double> minvector);
void evaluatePerformance(cv::Mat);
