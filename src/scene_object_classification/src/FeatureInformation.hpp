#ifndef FEATUREINFORMATION_H
#define FEATUREINFORMATION_H


#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


using namespace std;

class FeatureInformation{

private:
  int featureSize;
  vector<float> featureValues;

public:

  FeatureInformation();
  void addValue(float);
  vector<float> getAllValues();
  int getFeatureSize() { return featureSize; }
 /* void setFeatureSize(int); */


};


#endif
