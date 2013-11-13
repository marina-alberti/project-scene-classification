#ifndef APICONVERTKTHDB_H
#define APICONVERTKTHDB_H

#include "Object.hpp"
#include "SceneInformation.hpp"
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Input: the filename of annotations
// 	  an object of class SceneInformation
// Output: SceneInformation object contains the internal data structure for all objects in the scene

typedef map<string, float> objectParametersKTH;

class ApiConvertKTHDB {

private:

  string fileNameXML;

  objectParametersKTH mapKTHparameters;

  void parseObject(boost::property_tree::ptree &parent, SceneInformation &);
  
  /* given the object parameters contained in KTH annotation 
  (x, y, z, roll, pitch, yaw, length, width, height) 
  computes the IDS parameters: 8 coordinates of the (same) 3D bounding box
   + the centroid of same bounding box. */
  vector<pcl::PointXYZ> convertObjectParameters();

  void setmapKTHparameters(float, float, float, float, float, float, float, float, float);

public:

  ApiConvertKTHDB(string);
  void parseFileXML(SceneInformation &);
  

};

#endif
