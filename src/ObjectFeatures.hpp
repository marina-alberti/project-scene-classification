#ifndef OBJECTFEATURES_H
#define OBJECTFEATURES_H

#include "Object.hpp"
#include "SceneInformation.hpp"
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <algorithm>
#include <cmath>
#include "FeatureInformation.hpp"

using namespace std;

class ObjectFeatures{

private:

  Object internalObject;
  pcl::PointXYZ deskCentroid;

  FeatureInformation pose;
  FeatureInformation angle2dCentroid;
  FeatureInformation angle2d;
  FeatureInformation angle3d;
  FeatureInformation volumeSize;
  FeatureInformation sizeProjectedX;
  FeatureInformation sizeProjectedY;
  FeatureInformation sizeProjectedZ;
  
  vector<FeatureInformation> allFeatures;

  void computePose();
  void computeAngle2dCentroid();  
  void computeAngle2d();   

/* to do: check vector4f - check : which angle do i want to compute?  
   See kasper2011, they dont compute the 3d orientations but onty 2d */ 
  void computeAngle3d();  
                        
  void computeVolumeSize();       
  void computeSizeProjectedX();   
  void computeSizeProjectedY();   
  void computeSizeProjectedZ();   

public:

  ObjectFeatures(Object & , pcl::PointXYZ );   

/* 
  pcl::PointXYZ getPose();
  float getAngle2d();
  float getAngle3d();
  float getVolumeSize();
  float getSizeProjectedX();
  float getSizeProjectedY();
  float getSizeProjectedZ();
*/

  void showObjectFeatures();
  void extractFeatures();
  vector<FeatureInformation> getAllFeatures() { return allFeatures; }
};


#endif
