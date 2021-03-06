#ifndef OBJECTPAIRFEATURES_H
#define OBJECTPAIRFEATURES_H

#include "Object.hpp"
#include "ObjectFeatures.hpp"
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "FeatureInformation.hpp"

using namespace std;

class ObjectPairFeatures{

private:

  Object referenceObject;  
  Object targetObject;

  FeatureInformation EuclideanDistance;
  FeatureInformation EuclideanDistance2d;

/* 
  see kasper2011: angle between "direction of landmark" and 
  relative position of other obj, where :
   0 = means obj in front of landmark object (it has a orientation ref frame)
   180 means behind
*/
  FeatureInformation orientation2d;

  FeatureInformation orientation3d;
  FeatureInformation sizeDifference;   
  FeatureInformation verticalHeightDifference;  
  FeatureInformation minimumDistanceBoudaries; 

  vector<FeatureInformation> allFeatures;

  void computeEuclideanDistance();
  void computeEuclideanDistance2d();
  void computeOrientation3d();
  void computeOrientation2d();   // to check
  void computeMinimumDistanceBoudaries();
  void computeSizeDifference();
  void computeVerticalHeightDifference();
  

public:

  ObjectPairFeatures( Object &, Object & );
 
/*
  float getEuclideanDistance();
  float getEuclideanDistance2d();
  float getOrientation3d();
  float getOrientation2d();
  float getMinimumDistanceBoudaries();
*/

// returns all the features of the single object as a vector of float
// to do: change the data structure !!
  void extractFeatures();
  vector<FeatureInformation> getAllFeatures() { return allFeatures; }

};


#endif
