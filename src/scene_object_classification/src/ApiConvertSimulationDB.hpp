#ifndef APICONVERTSIMULATIONDB_H
#define APICONVERTSIMULATIONDB_H

#include "Object.hpp"
#include "SceneInformation.hpp"
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class ApiConvertSimulationDB {

private:

  string fileNameJSON;

public:

  ApiConvertSimulationDB(string);
  void parseFileJSON(vector<SceneInformation> &);
  void parseSceneJSON(boost::property_tree::ptree::value_type &, SceneInformation &);
  

};

#endif
