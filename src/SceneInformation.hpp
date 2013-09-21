#ifndef SCENEINFORMATION_H
#define SCENEINFORMATION_H

#include <string.h>
#include <boost/property_tree/ptree.hpp>
#include "Object.hpp"
//#include "FeatureInformation.hpp"
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "FeatureInformation.hpp"

class SceneInformation{

private:
  string sceneType;
  int numberOfObjects;
  vector<Object> objectList;
 //vector<Object> landmarkObjectList;

  vector<FeatureInformation> featureListSingleObject;
  vector<FeatureInformation> featureListPairObject;

  pcl::PointXYZ deskCentroid;
  float deskLength;
  float deskWidth;

public:
  SceneInformation();

  void setDeskLength(float);
  float getDeskLength();
  void setDeskWidth(float);
  float getDeskWidth();
  void setType(string);
  string getType();
  void setDeskCentroid();
  pcl::PointXYZ getDeskCentroid();

  void addObject(Object&);
  vector<Object> getObjectList();
  void showSceneInformation();

  /* 
  This function sorts the objects in vector<Object> objectList in a way that 
  the 3 KTHDB annotated objects (Monitor keyboard mouse)
  always are in the same order. 
  Scope: Since SR/QSR are dependent on object type, the use of same order 
  will allow to use Iterators on the objectList vector,
  to compute features between pairs of objects.
  */
  void orderObjectList(); 
  void addFeatureSingleObject(FeatureInformation);
  void addFeaturePairObject(FeatureInformation);
  vector<FeatureInformation> getFeatureListSingleObject() { return featureListSingleObject; }
  vector<FeatureInformation> getFeatureListPairObject() { return featureListPairObject; }

};

#endif
