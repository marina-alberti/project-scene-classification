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

#define N_CLUSTERS_OBJECT 2
#define N_CLUSTERS_PAIR 2  // 5
#define INDEX_TEST 30
#define BOOLREMOVE 1

class LOOCV{ 

private:

  int indexLoop;
  string dirname;
  int numberOfFiles;

  vector<double> thresholds;
  // names of all files in the folder chosen for cross-validation experiments
  vector<string> allFileNames;

  // names of all files used for training
  vector<string> trainingFilesList;

  // name of one file used for testing
  string testFilesList;                 

  //  it will contain N_objects models : 1 per object category
  std::vector<cv::EM> learnedModelSingleObject;   
  //  it will contain N_combinations models : combintations of object category pairs
  std::vector<vector<cv::EM> > learnedModelPairObject;     

  // <N_objectClasses x N_featuresSingleObject (1-D features)>
  vector<vector<double> > meanNormalization;
  vector<vector<double> > stdNormalization;

  // the similarity scores for all the test scenes in the cross-validation experiment
  vector<double> probSceneListLoocv;

  // the confusion matrix for object class prediction = object classification
  cv::Mat cMatrixObjectClassification;

  bool cMatrixSet;
  
  vector<int> countObjectFrequencies; 
  vector<int> countObjectFrequencies1;
  vector<vector< int> > countObjectPairFrequencies;  
  
  vector<vector<vector<double> > > meanNormalizationPair;
  vector<vector<vector<double> > > stdNormalizationPair;
  vector<vector<vector<double> > > minFeatPair;
  vector<vector<vector<double> > > maxFeatPair;

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
