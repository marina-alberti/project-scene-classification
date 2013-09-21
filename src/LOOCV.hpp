#ifndef LOOCV_H
#define LOOCV_H

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
#include "DatabaseInformation.hpp"
#include "TestScene.hpp"

#define N_CLUSTERS_OBJECT 3
#define N_CLUSTERS_PAIR 3
#define INDEX_TEST 30


class LOOCV{ 

private:

  int indexLoop;
  string dirname;
  int numberOfFiles;
  vector<string> allFileNames;
  vector<string> trainingFilesList;
  string testFilesList;

  std::vector<cv::EM> learnedModelSingleObject;   //  it will contain 3 models : 1 per object
  std::vector<cv::EM> learnedModelPairObject;     //  it will contain 3 X 5 models (if using separate features)

  vector<vector<double> > meanNormalization;
  vector<vector<double> > stdNormalization;

  vector<double> probSceneListLoocv;

  cv::Mat cMatrixObjectClassification;

  bool cMatrixSet;

public:

  LOOCV(string);
  void storeFiles();
  void compute();
  void createTrainingSet(int);
  void createTestSet(int);
  void doTraining();
  void doTest();
 // DatabaseInformation storeDatabaseTraining( vector<string> );

};

#endif
