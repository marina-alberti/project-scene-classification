#ifndef ALLFEATSINGLEOBJECT_H
#define ALLFEATSINGLEOBJECT_H


#include <string>
#include <vector>
#include "FeatureInformation.hpp"

using namespace std;

class AllFeatSingleObject{

private:
  int featureNumber;
  vector<FeatureInformation> features;
  // i will check this when checking / computing features 
  //  and iterating over a vector of "AllFeatSingleObject" coming from different objects
  int objectID;

public:

  AllFeatSingleObject();
  // adding 1 feature at a time
  void addFeature(FeatureInformation);  
  vector<FeatureInformation> getAllFeatures();
  int getFeatureSize() { return features.size(); }  

  // TO DO : modify considering the dimention features ("FeatureInformation" instances)  which are not 1-dimentional
  int getTotalFeatureDim();

  void setObjectID(int inputID) { objectID = inputID; }
  int getObjectID() { return objectID; }
};

#endif
