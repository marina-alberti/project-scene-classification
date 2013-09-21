#ifndef TESTSCENE_H
#define TESTSCENE_H

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
#include "opencv2/gpu/gpu.hpp"
#include <cmath>
#include <algorithm>

#define N_OBJECTS 3

typedef map<string, float> objectParametersKTH;

class TestScene{

private:
  
  int numberOfObjects;
  vector<Object> objectList;
  vector<double> predictedClasses;
  vector<Object> orderedObjectList;
  string fileNameXML;

/* Each element is one object - 3 separate feat Information vectors*/
  vector<vector<FeatureInformation> > featureListSingleObject;
  vector<FeatureInformation> featureListPairObject;
  pcl::PointXYZ deskCentroid;
  float deskLength;
  float deskWidth;
  objectParametersKTH mapKTHparameters;
  std::vector<cv::EM> learnedModelSingleObject;   //  it will contain 3 models : 1 per object
  std::vector<cv::EM> learnedModelPairObject;     //  it will contain 15models (if using separate features)
  vector<vector<double> > meanNormalization;
  vector<vector<double> > stdNormalization;

  // functions
  void parseObject(boost::property_tree::ptree & );
  vector<pcl::PointXYZ> convertObjectParameters();
  void setmapKTHparameters(float, float , float , float , float , float , float , float , float );
  void setDeskCentroid();

  double totalSceneLogP;

public:

  /*  load the given file index data */
  TestScene(string, vector<cv::EM> , vector<vector<double> > , vector<vector<double> > , vector<cv::EM> );

  /* loadAnnotations In IDS */
  void loadAnnotation();

  void parseFileXML();
    
  /* extract feats only for one scene only single objects */
  void extractFeatures();

  /* Compute the probabilities for each object matched with all the learned models */
  void predictObjectClasses();

 
  /* Set and store the object classes */ 
  void setObjectClasses();

  /* Extract feats object pairs */
   void extractFeaturesPairObjects();

  /* Compute the probabilities / likelihoods for the object pairs */ 
  double computeProbObjectPairs();

};

#endif
