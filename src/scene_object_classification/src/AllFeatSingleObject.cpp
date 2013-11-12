#include "AllFeatSingleObject.hpp"


AllFeatSingleObject::AllFeatSingleObject () {
  featureNumber = 0;
}

void AllFeatSingleObject::addFeature(FeatureInformation inputFeature) {
  features.push_back(inputFeature);
  featureNumber++;
}

vector<FeatureInformation> AllFeatSingleObject::getAllFeatures() {
  return features;
}

/*
void FeatureInformation setFeatureSize(int inputSize) {
  featureValues.resize(inputSize);
}
*/

