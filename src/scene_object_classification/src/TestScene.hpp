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
#include "AllFeatPairObject.hpp"
#include "utils.hpp"
#include "DatabaseInformation.hpp"

#define N_OBJECTS 7
#define NORMALIZEPAIR 1
#define NSCENES 41
#define PI 3.14159265
#define REMOVEID 6

typedef map<string, float> objectParametersKTH;

class TestScene{

private:
  
  vector<double> thresholds;
  int numberOfObjects;
  vector<Object> objectList;
  vector<double> predictedClasses;
  
  string fileNameXML;

  /* Each element is one object -> N_objects separate FeatureInformation vectors */
  vector<vector<FeatureInformation> > featureListSingleObject;
  // vector<vector<FeatureInformation> > featureListPairObject;
  vector<AllFeatPairObject>  featureListPair;

  pcl::PointXYZ deskCentroid;
  float deskLength;
  float deskWidth;
  objectParametersKTH mapKTHparameters;

  //  it will contain 3 models : 1 per object  
  std::vector<cv::EM> learnedModelSingleObject;   
  std::vector<vector<cv::EM> > learnedModelPairObject;    

  vector<vector<double> > meanNormalization;
  vector<vector<double> > stdNormalization;

  vector<int> countObjectFrequencies;
  vector<int> countObjectFrequencies1;
  vector<vector< int> > countObjectPairFrequencies;  

  //  vector <int> objectPairID;
  int removeID;

  // functions
  void parseObject(boost::property_tree::ptree & , bool);
  vector<pcl::PointXYZ> convertObjectParameters();
  void setmapKTHparameters(float, float , float , float , float , float , float , float , float );
  void setDeskCentroid();

  double totalSceneLogP;

  cv::Mat cMatrix;
  
  vector<vector<vector<double> > > meanNormalizationPair;
  vector<vector<vector<double> > > stdNormalizationPair;
  vector<vector<vector<double> > > minFeatPair;
  vector<vector<vector<double> > > maxFeatPair;

  DatabaseInformation trainedParameters;

public:

  /*  load the given file index data */
  TestScene(DatabaseInformation & ) ;

  /* loadAnnotations In IDS */
  void loadAnnotation(string, bool);

  //  void parseFileXML(bool);
    
  void loadAnnotationServiceFormat(vector<string>, vector<string>, vector<vector<pcl::PointXYZ> >);

  /* extract feats only for one scene only "single object features" */
  void extractFeatures();

  /* Compute the probabilities for each object matched with all the learned models */
  vector<vector<double> > predictObjectClasses();

  void evaluateObjectClassificationPerformance(); 

  /* Set and store the object classes */ 
  void setObjectClasses();

  /* Extract feats object pairs */
   void extractFeaturesPairObjects();

  /* Extract feats object pairs, handles missing objects and multiple objects. */
  // void extractFeaturesPairObjects_HandleMissing();

  /* Compute the probabilities / likelihoods for the object pairs */ 
  double computeProbObjectPairs();

  /* Compute the probabilities / likelihoods for the object pairs - models from all feats at a time */ 
  double computeProbObjectPairs_AllFeats();  

  double computeProbObjectPairs_AllFeats_old(); 

  cv::Mat getConfusionMatrix() {return cMatrix; }

  double computeSimilarityScore();


};

#endif
