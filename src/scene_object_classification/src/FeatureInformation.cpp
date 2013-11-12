#include "FeatureInformation.hpp"


FeatureInformation::FeatureInformation () {
  featureSize = 0;

}

void FeatureInformation::addValue(float inputValue) {

  featureValues.push_back(inputValue);
  featureSize++;
}

vector<float> FeatureInformation::getAllValues () {
  return featureValues;
}

/*
void FeatureInformation setFeatureSize(int inputSize) {
  featureValues.resize(inputSize);
}
*/

