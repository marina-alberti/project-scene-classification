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
  // (0) monitor, (1) keyboard, (2) mouse in this order
  vector<Object> orderedObjectList;  
  string fileNameXML;

/* Each element is one object -> 3 separate FeatureInformation vectors*/
  vector<vector<FeatureInformation> > featureListSingleObject;
  vector<vector<FeatureInformation> > featureListPairObject;
  pcl::PointXYZ deskCentroid;
  float deskLength;
  float deskWidth;
  objectParametersKTH mapKTHparameters;
  std::vector<cv::EM> learnedModelSingleObject;   //  it will contain 3 models : 1 per object
  std::vector<cv::EM> learnedModelPairObject;     //  it will contain 6 models (if using all features)
  vector<vector<double> > meanNormalization;
  vector<vector<double> > stdNormalization;
  vector <int> objectPairID;
  int removeID;

  // functions
  void parseObject(boost::property_tree::ptree & , bool);
  vector<pcl::PointXYZ> convertObjectParameters();
  void setmapKTHparameters(float, float , float , float , float , float , float , float , float );
  void setDeskCentroid();

  double totalSceneLogP;

  cv::Mat cMatrix;

public:

  /*  load the given file index data */
  TestScene(string, vector<cv::EM> , vector<vector<double> > , vector<vector<double> > , vector<cv::EM> );

  /* loadAnnotations In IDS */
  void loadAnnotation(bool);

  void parseFileXML(bool);
    
  /* extract feats only for one scene only single objects */
  void extractFeatures();

  /* Compute the probabilities for each object matched with all the learned models */
  void predictObjectClasses();

  void evaluateObjectClassificationPerformance(); 

  /* Set and store the object classes */ 
  void setObjectClasses();

  /* Extract feats object pairs */
   void extractFeaturesPairObjects();

  /* Extract feats object pairs, handles missing objects and multiple objects. */
   void extractFeaturesPairObjects_HandleMissing();

  /* Compute the probabilities / likelihoods for the object pairs */ 
  double computeProbObjectPairs();

  /* Compute the probabilities / likelihoods for the object pairs - models from all feats at a time */ 
  double computeProbObjectPairs_AllFeats();  

  double computeProbObjectPairs_AllFeats_old(); 

  cv::Mat getConfusionMatrix() {return cMatrix; }

};

#endif
