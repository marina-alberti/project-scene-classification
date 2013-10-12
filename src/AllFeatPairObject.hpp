#ifndef ALLFEATPAIROBJECT_H
#define ALLFEATPAIROBJECT_H


#include <string>
#include <vector>
#include "FeatureInformation.hpp"

using namespace std;

class AllFeatPairObject{

private:
  int featureNumber;
  vector<FeatureInformation> features;

  // i will check this when checking / computing features 
  //  and iterating over a vector of "AllFeatSingleObject" coming from different objects
  int objectID1;
  int objectID2;

public:

  AllFeatPairObject();
  void addFeature(FeatureInformation);
  vector<FeatureInformation> getAllFeatures();
  int getFeatureSize() { return features.size(); }  

  // TO DO : modify considering the dimention features ("FeatureInformation" instances)  which are not 1-dimentional
  int getTotalFeatureDim();

  void setObjectID1(int inputID) { objectID1 = inputID; }
  int getObjectID1() { return objectID1; }

  void setObjectID2(int inputID) { objectID2 = inputID; }
  int getObjectID2() { return objectID2; }

};

#endif
