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
#include "AllFeatSingleObject.hpp"
#include "AllFeatPairObject.hpp"


class SceneInformation{

private:
  string sceneType;
  int numberOfObjects;

  vector<Object> objectList;   // will contain also the object IDs
 //vector<Object> landmarkObjectList;

 // vector<FeatureInformation> featureListSingleObject;
 // vector<FeatureInformation> featureListPairObject;

  // new classes TO DO:
  vector<AllFeatSingleObject> featureListSingle;
  vector<AllFeatPairObject>  featureListPair;

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

  // new: to inetgrate into the ApiFeatureExtraction 
  void addAllFeatSingleObject(vector<FeatureInformation>, int);
  void addAllFeatPairObject(vector<FeatureInformation>, int, int);

  vector<AllFeatSingleObject> getAllFeatSingleObject() { return featureListSingle; }
  vector<AllFeatPairObject> getAllFeatPairObject() { return featureListPair; }

 // vector<FeatureInformation> getFeatureListSingleObject() { return featureListSingleObject; }
 // vector<FeatureInformation> getFeatureListPairObject() { return featureListPairObject; }

};

#endif
