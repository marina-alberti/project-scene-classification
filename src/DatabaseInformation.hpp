#ifndef DATABASEINFORMATION_H
#define DATABASEINFORMATION_H

#include <string.h>
#include <boost/property_tree/ptree.hpp>
#include "Object.hpp"
#include "ApiConvertKTHDB.hpp"
#include "ApiFeatureExtraction.hpp"
#include "SceneInformation.hpp"
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


class DatabaseInformation{

private:
  string dirname;
  int numberOfScenes;

  vector<string> filesList;

  vector<SceneInformation> sceneList;
  vector<vector<FeatureInformation> > featureMatrixSingleObject;
  vector<vector<FeatureInformation> > featureMatrixPairObject;
  std::vector<cv::EM> learnedModelSingleObject;   //  it will contain 3 models : 1 per object
  std::vector<cv::EM> learnedModelPairObject;     //  it will contain 3 X 5 models (if using separate features)

  vector<vector<double> > meanNormalization;
  vector<vector<double> > stdNormalization;
 
public:

  DatabaseInformation( vector<string> );
  int getNumberOfScenes() { return numberOfScenes; }

/*
 parse all xml files in the data folder
 add to sceneList push_back
*/
  void loadAnnotationsInIDS();   

/* 
For each scene the features are extracted in the 
vector<FeatureInformation> featureList;  
*/
  void callApiFeatureExtraction();

/*
  This function collects all the features stored into featureList of 
  each scene into a 2d matrix of this class.
  For each SceneInformation :
  goes through vector<FeatureInformation> featureListSingleObject
  stored it in element of featureMatrix
*/

  void setFeatureMatrix(); 

  void printFeatureMatrix();

  void computeGMM_SingleObject_SingleFeat(int);  // ok check with matlab plots

  void computeGMM_SingleObject_AllFeat(int );    // 

  void computeGMM_PairObject_SingleFeat(int);    // ok check with matlab plots

  std::vector<cv::EM> getLearnedModelSingleObject () { return learnedModelSingleObject; }

  std::vector<cv::EM> getLearnedModelPairObject() {return learnedModelPairObject; }

  vector<vector<double> > getmeanNormalization() {return meanNormalization; }

  vector<vector<double> > getstdNormalization() {return stdNormalization; }


};

#endif