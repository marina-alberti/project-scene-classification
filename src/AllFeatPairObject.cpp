#include "AllFeatPairObject.hpp"


AllFeatPairObject::AllFeatPairObject () {
  featureNumber = 0;
}

void AllFeatPairObject::addFeature(FeatureInformation inputFeature) {
  features.push_back(inputFeature);
  featureNumber++;
}

vector<FeatureInformation> AllFeatPairObject::getAllFeatures() {
  return features;
}

/*
void FeatureInformation setFeatureSize(int inputSize) {
  featureValues.resize(inputSize);
}
*/

